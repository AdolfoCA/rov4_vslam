// Upstream src/Optimizer.cc, split into parts only so that each compiles in < 4 GB of
// RAM (the whole file needed more). The parts concatenated are the original file; see
// UPSTREAM.md. sortByVal() is repeated as a file-local helper in every part.

/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "Optimizer.h"

#include <complex>

#include <Eigen/StdVector>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <opencv2/core/eigen.hpp>

#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "G2oTypes.h"
#include "Converter.h"
#include "System.h"
#include <sophus/geometry.hpp>

#include <mutex>

#include "OptimizableTypes.h"

namespace ORB_SLAM3
{

static bool sortByVal(const pair<MapPoint *, int> &a, const pair<MapPoint *, int> &b)
{
	return (a.second < b.second);
}

void Optimizer::OptimizeEssentialGraph(KeyFrame *pCurKF,
                                       vector<KeyFrame *> &vpFixedKFs,
                                       vector<KeyFrame *> &vpFixedCorrectedKFs,
                                       vector<KeyFrame *> &vpNonFixedKFs,
                                       vector<MapPoint *> &vpNonCorrectedMPs,
                                       bool &bFinished,
                                       bool *bReSet)
{
	Verbose::PrintMess("Opt_Essential: There are " + to_string(vpFixedKFs.size()) + " KFs fixed in the merged map",
	                   Verbose::VERBOSITY_DEBUG);
	Verbose::PrintMess(
		"Opt_Essential: There are " + to_string(vpFixedCorrectedKFs.size()) + " KFs fixed in the old map",
		Verbose::VERBOSITY_DEBUG);
	Verbose::PrintMess(
		"Opt_Essential: There are " + to_string(vpNonFixedKFs.size()) + " KFs non-fixed in the merged map",
		Verbose::VERBOSITY_DEBUG);
	Verbose::PrintMess(
		"Opt_Essential: There are " + to_string(vpNonCorrectedMPs.size()) + " MPs non-corrected in the merged map",
		Verbose::VERBOSITY_DEBUG);

	g2o::SparseOptimizer optimizer;
	optimizer.setVerbose(false);
	g2o::BlockSolver_7_3::LinearSolverType *linearSolver =
		new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
	g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

	solver->setUserLambdaInit(1e-16);
	optimizer.setAlgorithm(solver);

	Map *pMap = pCurKF->GetMap();
	const unsigned int nMaxKFid = pMap->GetMaxKFid();

	vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
	vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(nMaxKFid + 1);
	vector<g2o::VertexSim3Expmap *> vpVertices(nMaxKFid + 1);

	const int minFeat = 100;

	for (KeyFrame *pKFi: vpFixedKFs) {
		if (pKFi->isBad()) {
			continue;
		}

		g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

		const int nIDi = pKFi->mnId;

		Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKFi->GetRotation());
		Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKFi->GetTranslation());
		g2o::Sim3 Siw(Rcw, tcw, 1.0);
		vScw[nIDi] = Siw;
		vCorrectedSwc[nIDi] = Siw.inverse(); // This KFs mustn't be corrected
		VSim3->setEstimate(Siw);

		VSim3->setFixed(true);

		VSim3->setId(nIDi);
		VSim3->setMarginalized(false);
		VSim3->_fix_scale = true; //TODO

		optimizer.addVertex(VSim3);

		vpVertices[nIDi] = VSim3;
	}
	Verbose::PrintMess("Opt_Essential: vpFixedKFs loaded", Verbose::VERBOSITY_DEBUG);

	set<unsigned long> sIdKF;
	for (KeyFrame *pKFi: vpFixedCorrectedKFs) {
		if (pKFi->isBad()) {
			continue;
		}

		g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

		const int nIDi = pKFi->mnId;

		Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKFi->GetRotation());
		Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKFi->GetTranslation());
		g2o::Sim3 Siw(Rcw, tcw, 1.0);
		//vScw[nIDi] = Siw;
		vCorrectedSwc[nIDi] = Siw.inverse(); // This KFs mustn't be corrected
		VSim3->setEstimate(Siw);

		cv::Mat Tcw_bef = pKFi->mTcwBefMerge;
		Eigen::Matrix<double, 3, 3> Rcw_bef = Converter::toMatrix3d(Tcw_bef.rowRange(0, 3).colRange(0, 3));
		Eigen::Matrix<double, 3, 1> tcw_bef = Converter::toVector3d(Tcw_bef.rowRange(0, 3).col(3));
		vScw[nIDi] = g2o::Sim3(Rcw_bef, tcw_bef, 1.0);

		VSim3->setFixed(true);

		VSim3->setId(nIDi);
		VSim3->setMarginalized(false);

		optimizer.addVertex(VSim3);

		vpVertices[nIDi] = VSim3;

		sIdKF.insert(nIDi);
	}
	Verbose::PrintMess("Opt_Essential: vpFixedCorrectedKFs loaded", Verbose::VERBOSITY_DEBUG);

	for (KeyFrame *pKFi: vpNonFixedKFs) {
		if (pKFi->isBad()) {
			continue;
		}

		const int nIDi = pKFi->mnId;

		if (sIdKF.count(nIDi)) { // It has already added in the corrected merge KFs
			continue;
		}

		g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

		//cv::Mat Tcw = pKFi->mTcwBefMerge;
		//Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(Tcw.rowRange(0,3).colRange(0,3));
		//Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(Tcw.rowRange(0,3).col(3));
		Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKFi->GetRotation());
		Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKFi->GetTranslation());
		g2o::Sim3 Siw(Rcw, tcw, 1.0);
		vScw[nIDi] = Siw;
		VSim3->setEstimate(Siw);

		VSim3->setFixed(false);

		VSim3->setId(nIDi);
		VSim3->setMarginalized(false);

		optimizer.addVertex(VSim3);

		vpVertices[nIDi] = VSim3;

		sIdKF.insert(nIDi);
	}
	Verbose::PrintMess("Opt_Essential: vpNonFixedKFs loaded", Verbose::VERBOSITY_DEBUG);

	vector<KeyFrame *> vpKFs;
	vpKFs.reserve(vpFixedKFs.size() + vpFixedCorrectedKFs.size() + vpNonFixedKFs.size());
	vpKFs.insert(vpKFs.end(), vpFixedKFs.begin(), vpFixedKFs.end());
	vpKFs.insert(vpKFs.end(), vpFixedCorrectedKFs.begin(), vpFixedCorrectedKFs.end());
	vpKFs.insert(vpKFs.end(), vpNonFixedKFs.begin(), vpNonFixedKFs.end());
	set<KeyFrame *> spKFs(vpKFs.begin(), vpKFs.end());

	Verbose::PrintMess("Opt_Essential: List of KF loaded", Verbose::VERBOSITY_DEBUG);

	const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

	for (KeyFrame *pKFi: vpKFs) {
		int num_connections = 0;
		const int nIDi = pKFi->mnId;

		g2o::Sim3 Swi = vScw[nIDi].inverse();
		/*if(pKFi->mnMergeCorrectedForKF == pCurKF->mnId)
        {
             Swi = vScw[nIDi].inverse();
        }
        else
        {
            cv::Mat Twi = pKFi->mTwcBefMerge;
            Swi = g2o::Sim3(Converter::toMatrix3d(Twi.rowRange(0, 3).colRange(0, 3)),
                            Converter::toVector3d(Twi.rowRange(0, 3).col(3)),1.0);
        }*/

		KeyFrame *pParentKFi = pKFi->GetParent();

		// Spanning tree edge
		if (pParentKFi && spKFs.find(pParentKFi) != spKFs.end()) {
			int nIDj = pParentKFi->mnId;

			g2o::Sim3 Sjw = vScw[nIDj];

			/*if(pParentKFi->mnMergeCorrectedForKF == pCurKF->mnId)
            {
                 Sjw =  vScw[nIDj];
            }
            else
            {
                cv::Mat Tjw = pParentKFi->mTcwBefMerge;
                Sjw = g2o::Sim3(Converter::toMatrix3d(Tjw.rowRange(0, 3).colRange(0, 3)),
                                Converter::toVector3d(Tjw.rowRange(0, 3).col(3)),1.0);
            }*/

			g2o::Sim3 Sji = Sjw * Swi;

			g2o::EdgeSim3 *e = new g2o::EdgeSim3();
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
			e->setMeasurement(Sji);

			e->information() = matLambda;
			optimizer.addEdge(e);
			num_connections++;
		}

		// Loop edges
		const set<KeyFrame *> sLoopEdges = pKFi->GetLoopEdges();
		for (set<KeyFrame *>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++) {
			KeyFrame *pLKF = *sit;
			if (spKFs.find(pLKF) != spKFs.end() && pLKF->mnId < pKFi->mnId) {
				g2o::Sim3 Slw = vScw[pLKF->mnId];

				/*if(pLKF->mnMergeCorrectedForKF == pCurKF->mnId)
                {
                     Slw = vScw[pLKF->mnId];
                }
                else
                {
                    cv::Mat Tlw = pLKF->mTcwBefMerge;
                    Slw = g2o::Sim3(Converter::toMatrix3d(Tlw.rowRange(0, 3).colRange(0, 3)),
                                    Converter::toVector3d(Tlw.rowRange(0, 3).col(3)),1.0);
                }*/

				g2o::Sim3 Sli = Slw * Swi;
				g2o::EdgeSim3 *el = new g2o::EdgeSim3();
				el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pLKF->mnId)));
				el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
				el->setMeasurement(Sli);
				el->information() = matLambda;
				optimizer.addEdge(el);
				num_connections++;
			}
		}

		// Covisibility graph edges
		const vector<KeyFrame *> vpConnectedKFs = pKFi->GetCovisiblesByWeight(minFeat);
		for (vector<KeyFrame *>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++) {
			KeyFrame *pKFn = *vit;
			if (pKFn && pKFn != pParentKFi && !pKFi->hasChild(pKFn) && !sLoopEdges.count(pKFn)
				&& spKFs.find(pKFn) != spKFs.end()) {
				if (!pKFn->isBad() && pKFn->mnId < pKFi->mnId) {

					g2o::Sim3 Snw = vScw[pKFn->mnId];
					/*if(pKFn->mnMergeCorrectedForKF == pCurKF->mnId)
                    {
                        Snw = vScw[pKFn->mnId];
                    }
                    else
                    {
                        cv::Mat Tnw = pKFn->mTcwBefMerge;
                        Snw = g2o::Sim3(Converter::toMatrix3d(Tnw.rowRange(0, 3).colRange(0, 3)),
                                        Converter::toVector3d(Tnw.rowRange(0, 3).col(3)),1.0);
                    }*/

					g2o::Sim3 Sni = Snw * Swi;

					g2o::EdgeSim3 *en = new g2o::EdgeSim3();
					en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFn->mnId)));
					en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
					en->setMeasurement(Sni);
					en->information() = matLambda;
					optimizer.addEdge(en);
					num_connections++;
				}
			}
		}

		if (num_connections == 0) {
			Verbose::PrintMess("Opt_Essential: KF " + to_string(pKFi->mnId) + " has 0 connections",
			                   Verbose::VERBOSITY_DEBUG);
		}
	}

	// Optimize!
	optimizer.initializeOptimization();
	optimizer.optimize(20);

	Verbose::PrintMess("Opt_Essential: Finish the optimization", Verbose::VERBOSITY_DEBUG);

	unique_lock<shared_timed_mutex> lock(pMap->mMutexMapUpdate, std::defer_lock);
	while (!lock.try_lock_for(std::chrono::milliseconds(300))) {
		if (*bReSet) {
			bFinished = false;
			return;
		}
	}

	Verbose::PrintMess("Opt_Essential: Apply the new pose to the KFs", Verbose::VERBOSITY_DEBUG);
	// SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
	for (KeyFrame *pKFi: vpNonFixedKFs) {
		if (pKFi->isBad()) {
			continue;
		}

		const int nIDi = pKFi->mnId;

		g2o::VertexSim3Expmap *VSim3 = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(nIDi));
		g2o::Sim3 CorrectedSiw = VSim3->estimate();
		vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
		Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
		Eigen::Vector3d eigt = CorrectedSiw.translation();
		double s = CorrectedSiw.scale();

		eigt *= (1. / s); //[R t/s;0 1]

		cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);

		/*{
            cv::Mat Tco_cn = pKFi->GetPose() * Tiw.inv();
            cv::Vec3d trasl = Tco_cn.rowRange(0,3).col(3);
            double dist = cv::norm(trasl);
            if(dist > 1.0)
            {
                cout << "--Distance: " << dist << " meters" << endl;
                cout << "--To much distance correction in EssentGraph: It has connected " << pKFi->GetVectorCovisibleKeyFrames().size() << " KFs" << endl;
            }
            string strNameFile = pKFi->mNameFile;
            cv::Mat imLeft = cv::imread(strNameFile, cv::IMREAD_UNCHANGED);
            cv::cvtColor(imLeft, imLeft, cv::COLOR_GRAY2BGR);
            vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();
            for(int j=0; j<vpMapPointsKFi.size(); ++j)
            {
                if(!vpMapPointsKFi[j] || vpMapPointsKFi[j]->isBad())
                {
                    continue;
                }
                string strNumOBs = to_string(vpMapPointsKFi[j]->Observations());
                cv::circle(imLeft, pKFi->mvKeys[j].pt, 2, cv::Scalar(0, 255, 0));
                cv::putText(imLeft, strNumOBs, pKFi->mvKeys[j].pt, cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0));
            }
            string namefile = "./test_OptEssent/Essent_" + to_string(pCurKF->mnId) + "_KF" + to_string(pKFi->mnId) +"_D" + to_string(dist) +".png";
            cv::imwrite(namefile, imLeft);
        }*/

		pKFi->mTcwBefMerge = pKFi->GetPose();
		pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
		pKFi->SetPose(Tiw);
	}

	Verbose::PrintMess("Opt_Essential: Apply the new pose to the MPs", Verbose::VERBOSITY_DEBUG);
	// Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
	for (MapPoint *pMPi: vpNonCorrectedMPs) {
		if (pMPi->isBad()) {
			continue;
		}

		Verbose::PrintMess("Opt_Essential: MP id " + to_string(pMPi->mnId), Verbose::VERBOSITY_DEBUG);
		/*int nIDr;
        if(pMPi->mnCorrectedByKF==pCurKF->mnId)
        {
            nIDr = pMPi->mnCorrectedReference;
        }
        else
        {
        }*/
		KeyFrame *pRefKF = pMPi->GetReferenceKeyFrame();
		g2o::Sim3 Srw;
		g2o::Sim3 correctedSwr;
		while (pRefKF->isBad()) {
			if (!pRefKF) {
				Verbose::PrintMess("MP " + to_string(pMPi->mnId) + " without a valid reference KF",
				                   Verbose::VERBOSITY_DEBUG);
				break;
			}

			pMPi->EraseObservation(pRefKF);
			pRefKF = pMPi->GetReferenceKeyFrame();
		}
		/*if(pRefKF->mnMergeCorrectedForKF == pCurKF->mnId)
        {
            int nIDr = pRefKF->mnId;
            Srw = vScw[nIDr];
            correctedSwr = vCorrectedSwc[nIDr];
        }
        else
        {*/
		cv::Mat TNonCorrectedwr = pRefKF->mTwcBefMerge;
		Eigen::Matrix<double, 3, 3>
			RNonCorrectedwr = Converter::toMatrix3d(TNonCorrectedwr.rowRange(0, 3).colRange(0, 3));
		Eigen::Matrix<double, 3, 1> tNonCorrectedwr = Converter::toVector3d(TNonCorrectedwr.rowRange(0, 3).col(3));
		Srw = g2o::Sim3(RNonCorrectedwr, tNonCorrectedwr, 1.0).inverse();

		cv::Mat Twr = pRefKF->GetPoseInverse();
		Eigen::Matrix<double, 3, 3> Rwr = Converter::toMatrix3d(Twr.rowRange(0, 3).colRange(0, 3));
		Eigen::Matrix<double, 3, 1> twr = Converter::toVector3d(Twr.rowRange(0, 3).col(3));
		correctedSwr = g2o::Sim3(Rwr, twr, 1.0);
		//}
		//cout << "Opt_Essential: Loaded the KF reference position" << endl;

		cv::Mat P3Dw = pMPi->GetWorldPos();
		Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
		Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

		//cout << "Opt_Essential: Calculated the new MP position" << endl;
		cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
		//cout << "Opt_Essential: Converted the position to the OpenCV format" << endl;
		pMPi->SetWorldPos(cvCorrectedP3Dw);
		//cout << "Opt_Essential: Loaded the corrected position in the MP object" << endl;

		pMPi->UpdateNormalAndDepth();
	}

	Verbose::PrintMess("Opt_Essential: End of the optimization", Verbose::VERBOSITY_DEBUG);
}

void Optimizer::GlobalVAPoseGraphOptimization(KeyFrame* pCurKF, vector<KeyFrame*> &vpFixedKFs,
                                                     vector<KeyFrame*> &vpFixedCorrectedKFs,
                                                      vector<KeyFrame *> &vpNonFixedKFs,
                                                      vector<MapPoint *> &vpNonCorrectedMPs,
                                                     Atlas* pAtlas)
{
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType *linearSolver =
            new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    Map *pMap = pCurKF->GetMap();
    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(nMaxKFid + 1);
    vector<g2o::VertexSim3Expmap *> vpVertices(nMaxKFid + 1);

    const int minFeat = 100;
    set<unsigned long> sIdKF;
    set<unsigned long> sIdMP;

    for (KeyFrame *pKFi: vpFixedKFs) {
        if (pKFi->isBad()) {
            continue;
        }

        g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKFi->mnId;

        Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKFi->GetRotation());
        Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKFi->GetTranslation());
        g2o::Sim3 Siw(Rcw, tcw, 1.0);
        vScw[nIDi] = Siw;
        vCorrectedSwc[nIDi] = Siw.inverse(); // This KFs mustn't be corrected
        VSim3->setEstimate(Siw);

        VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = true; //TODO

        optimizer.addVertex(VSim3);

        vpVertices[nIDi] = VSim3;
        sIdKF.insert(nIDi);
    }


    for (KeyFrame *pKFi: vpFixedCorrectedKFs) {
        if (pKFi->isBad()) {
            continue;
        }

        g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKFi->mnId;

        Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKFi->GetRotation());
        Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKFi->GetTranslation());
        g2o::Sim3 Siw(Rcw, tcw, 1.0);
        //vScw[nIDi] = Siw;
        vCorrectedSwc[nIDi] = Siw.inverse(); // This KFs mustn't be corrected
        VSim3->setEstimate(Siw);

        cv::Mat Tcw_bef = pKFi->mTcwBefMerge;
        Eigen::Matrix<double, 3, 3> Rcw_bef = Converter::toMatrix3d(Tcw_bef.rowRange(0, 3).colRange(0, 3));
        Eigen::Matrix<double, 3, 1> tcw_bef = Converter::toVector3d(Tcw_bef.rowRange(0, 3).col(3));
        vScw[nIDi] = g2o::Sim3(Rcw_bef, tcw_bef, 1.0);

        VSim3->setFixed(true);
        VSim3->_fix_scale = true;
        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);

        optimizer.addVertex(VSim3);

        vpVertices[nIDi] = VSim3;

        sIdKF.insert(nIDi);
    }

    for (KeyFrame *pKFi: vpNonFixedKFs) {
        if (pKFi->isBad()) {
            continue;
        }

        const int nIDi = pKFi->mnId;

        if (sIdKF.count(nIDi)) { // It has already added in the corrected merge KFs
            continue;
        }

        g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

        //cv::Mat Tcw = pKFi->mTcwBefMerge;
        //Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(Tcw.rowRange(0,3).colRange(0,3));
        //Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(Tcw.rowRange(0,3).col(3));
        Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKFi->GetRotation());
        Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKFi->GetTranslation());
        g2o::Sim3 Siw(Rcw, tcw, 1.0);
        vScw[nIDi] = Siw;
        VSim3->setEstimate(Siw);

        VSim3->setFixed(false);
        VSim3->_fix_scale = true;
        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);

        optimizer.addVertex(VSim3);

        vpVertices[nIDi] = VSim3;

        sIdKF.insert(nIDi);
    }
    vector<KeyFrame*> KFs_in_othermaps;
    for(auto m:pAtlas->GetAllMaps()){
        for (KeyFrame *pKFi: m->GetAllKeyFrames()) {
            if (pKFi->isBad()) {
                ROS_DEBUG_STREAM("KF: "<<pKFi->mnId<<"is bad");
                continue;
            }

            const int nIDi = pKFi->mnId;

            if (sIdKF.count(nIDi)) { // It has already added in the corrected merge KFs
                continue;
            }

            g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

            //cv::Mat Tcw = pKFi->mTcwBefMerge;
            //Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(Tcw.rowRange(0,3).colRange(0,3));
            //Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(Tcw.rowRange(0,3).col(3));
            Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKFi->GetRotation());
            Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKFi->GetTranslation());
            g2o::Sim3 Siw(Rcw, tcw, 1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);

            VSim3->setFixed(false);
            VSim3->_fix_scale = true;
            VSim3->setId(nIDi);
            VSim3->setMarginalized(false);

            optimizer.addVertex(VSim3);

            vpVertices[nIDi] = VSim3;

            sIdKF.insert(nIDi);
            KFs_in_othermaps.push_back(pKFi);
        }
    }


    vector<KeyFrame *> vpKFs;
    vpKFs.reserve(vpFixedKFs.size() + vpFixedCorrectedKFs.size() + vpNonFixedKFs.size()+ KFs_in_othermaps.size());
    vpKFs.insert(vpKFs.end(), vpFixedKFs.begin(), vpFixedKFs.end());
    vpKFs.insert(vpKFs.end(), vpFixedCorrectedKFs.begin(), vpFixedCorrectedKFs.end());
    vpKFs.insert(vpKFs.end(), vpNonFixedKFs.begin(), vpNonFixedKFs.end());
    vpKFs.insert(vpKFs.end(), KFs_in_othermaps.begin(), KFs_in_othermaps.end());
    set<KeyFrame *> spKFs(vpKFs.begin(), vpKFs.end());


    const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

    for (KeyFrame *pKFi: vpKFs) {
        int num_connections = 0;
        const int nIDi = pKFi->mnId;

        g2o::Sim3 Swi = vScw[nIDi].inverse();

        KeyFrame *pParentKFi = pKFi->GetParent();

        // Spanning tree edge
        if (pParentKFi && spKFs.find(pParentKFi) != spKFs.end()) {
            int nIDj = pParentKFi->mnId;

            g2o::Sim3 Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3 *e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;
            optimizer.addEdge(e);
            num_connections++;
        }

        // Loop edges
        const set<KeyFrame *> sLoopEdges = pKFi->GetLoopEdges();
        for (set<KeyFrame *>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++) {
            KeyFrame *pLKF = *sit;
            if (spKFs.find(pLKF) != spKFs.end() && pLKF->mnId < pKFi->mnId) {
                g2o::Sim3 Slw = vScw[pLKF->mnId];


                g2o::Sim3 Sli = Slw * Swi;
                g2o::EdgeSim3 *el = new g2o::EdgeSim3();
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pLKF->mnId)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
                num_connections++;
            }
        }

        // Covisibility graph edges
        const vector<KeyFrame *> vpConnectedKFs = pKFi->GetCovisiblesByWeight(minFeat);
        for (vector<KeyFrame *>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++) {
            KeyFrame *pKFn = *vit;
            if (pKFn && pKFn != pParentKFi && !pKFi->hasChild(pKFn) && !sLoopEdges.count(pKFn)
                && spKFs.find(pKFn) != spKFs.end()) {
                if (!pKFn->isBad() && pKFn->mnId < pKFi->mnId) {

                    g2o::Sim3 Snw = vScw[pKFn->mnId];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3 *en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFn->mnId)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                    num_connections++;
                }
            }
        }

        if (pKFi->mpDvlPreintegrationLossRefKF){
            KeyFrame *pKFn = pKFi->mpLossRefKF;
            if (pKFn && pKFn != pParentKFi && !pKFi->hasChild(pKFn) && !sLoopEdges.count(pKFn)
                && spKFs.find(pKFn) != spKFs.end()) {
                if (!pKFn->isBad() && pKFn->mnId < pKFi->mnId) {

                    g2o::Sim3 Snw = vScw[pKFn->mnId];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3 *en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFn->mnId)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                    num_connections++;
                }
            }
            ROS_INFO_STREAM("add multi-map constrain for global pose graph, from ID: "<<pKFi->mnId<<"to ID: "<<pKFn->mnId);
        }

        if (num_connections == 0) {
            Verbose::PrintMess("Opt_Essential: KF " + to_string(pKFi->mnId) + " has 0 connections",
                               Verbose::VERBOSITY_DEBUG);
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    Verbose::PrintMess("Opt_Essential: Finish the optimization", Verbose::VERBOSITY_DEBUG);

    // unique_lock<timed_mutex> lock(pMap->mMutexMapUpdate, std::defer_lock);

    Verbose::PrintMess("Opt_Essential: Apply the new pose to the KFs", Verbose::VERBOSITY_DEBUG);
    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for (KeyFrame *pKFi: vpKFs) {
        if (pKFi->isBad()) {
            continue;
        }

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap *VSim3 = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw = VSim3->estimate();
        vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();

        eigt *= (1. / s); //[R t/s;0 1]

        cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);


        pKFi->mTcwBefMerge = pKFi->GetPose();
        pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
        // ROS_INFO_STREAM("set KF: "<<pKFi->mnId<<" pose before merging");
        pKFi->SetPose(Tiw);

        if(pKFi->GetMapPointMatches().size()==0){
            continue;
        }
        for(auto pMPi:pKFi->GetMapPointMatches()){
            if(!pMPi){
                continue;
            }
            if (!sIdMP.count(pMPi->mnId)){
                if (pMPi->isBad()) {
                    continue;
                }
                sIdMP.insert(pMPi->mnId);

                KeyFrame *pRefKF = pKFi;
                g2o::Sim3 Srw;
                g2o::Sim3 correctedSwr;
                while (pRefKF->isBad()) {
                    if (!pRefKF) {
                        break;
                    }

                    pMPi->EraseObservation(pRefKF);
                    pRefKF = pMPi->GetReferenceKeyFrame();
                }
                // if (!sIdKF.count(pRefKF->mnId)){
                //     ROS_INFO_STREAM("skip update map point["<<pMPi->mnId<<"] refer to KF["<<pRefKF->mnId<<"]");
                //     continue;
                // }

                cv::Mat TNonCorrectedwr = pRefKF->mTwcBefMerge;
                Eigen::Matrix<double, 3, 3>
                        RNonCorrectedwr = Converter::toMatrix3d(TNonCorrectedwr.rowRange(0, 3).colRange(0, 3));
                Eigen::Matrix<double, 3, 1> tNonCorrectedwr = Converter::toVector3d(TNonCorrectedwr.rowRange(0, 3).col(3));
                Srw = g2o::Sim3(RNonCorrectedwr, tNonCorrectedwr, 1.0).inverse();

                cv::Mat Twr = pRefKF->GetPoseInverse();
                Eigen::Matrix<double, 3, 3> Rwr = Converter::toMatrix3d(Twr.rowRange(0, 3).colRange(0, 3));
                Eigen::Matrix<double, 3, 1> twr = Converter::toVector3d(Twr.rowRange(0, 3).col(3));
                correctedSwr = g2o::Sim3(Rwr, twr, 1.0);
                //}
                //cout << "Opt_Essential: Loaded the KF reference position" << endl;

                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

                //cout << "Opt_Essential: Calculated the new MP position" << endl;
                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                //cout << "Opt_Essential: Converted the position to the OpenCV format" << endl;
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                //cout << "Opt_Essential: Loaded the corrected position in the MP object" << endl;

                pMPi->UpdateNormalAndDepth();
            }

        }
    }
    // for(auto m:pAtlas->GetAllMaps()){
    //     for (MapPoint *pMPi: m->GetAllMapPoints()) {
    //         if (pMPi->isBad()) {
    //             continue;
    //         }
    //
    //         KeyFrame *pRefKF = pMPi->GetReferenceKeyFrame();
    //         g2o::Sim3 Srw;
    //         g2o::Sim3 correctedSwr;
    //         while (pRefKF->isBad()) {
    //             if (!pRefKF) {
    //                 break;
    //             }
    //
    //             pMPi->EraseObservation(pRefKF);
    //             pRefKF = pMPi->GetReferenceKeyFrame();
    //         }
    //         // if (!sIdKF.count(pRefKF->mnId)){
    //         //     ROS_INFO_STREAM("skip update map point["<<pMPi->mnId<<"] refer to KF["<<pRefKF->mnId<<"]");
    //         //     continue;
    //         // }
    //
    //         cv::Mat TNonCorrectedwr = pRefKF->mTwcBefMerge;
    //         Eigen::Matrix<double, 3, 3>
    //                 RNonCorrectedwr = Converter::toMatrix3d(TNonCorrectedwr.rowRange(0, 3).colRange(0, 3));
    //         Eigen::Matrix<double, 3, 1> tNonCorrectedwr = Converter::toVector3d(TNonCorrectedwr.rowRange(0, 3).col(3));
    //         Srw = g2o::Sim3(RNonCorrectedwr, tNonCorrectedwr, 1.0).inverse();
    //
    //         cv::Mat Twr = pRefKF->GetPoseInverse();
    //         Eigen::Matrix<double, 3, 3> Rwr = Converter::toMatrix3d(Twr.rowRange(0, 3).colRange(0, 3));
    //         Eigen::Matrix<double, 3, 1> twr = Converter::toVector3d(Twr.rowRange(0, 3).col(3));
    //         correctedSwr = g2o::Sim3(Rwr, twr, 1.0);
    //         //}
    //         //cout << "Opt_Essential: Loaded the KF reference position" << endl;
    //
    //         cv::Mat P3Dw = pMPi->GetWorldPos();
    //         Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
    //         Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));
    //
    //         //cout << "Opt_Essential: Calculated the new MP position" << endl;
    //         cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
    //         //cout << "Opt_Essential: Converted the position to the OpenCV format" << endl;
    //         pMPi->SetWorldPos(cvCorrectedP3Dw);
    //         //cout << "Opt_Essential: Loaded the corrected position in the MP object" << endl;
    //
    //         pMPi->UpdateNormalAndDepth();
    //     }
    // }

}

void Optimizer::OptimizeEssentialGraph(KeyFrame *pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3)
{
	// Setup optimizer
	Map *pMap = pCurKF->GetMap();
	g2o::SparseOptimizer optimizer;
	optimizer.setVerbose(false);
	g2o::BlockSolver_7_3::LinearSolverType *linearSolver =
		new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
	g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

	solver->setUserLambdaInit(1e-16);
	optimizer.setAlgorithm(solver);

	const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
	const vector<MapPoint *> vpMPs = pMap->GetAllMapPoints();

	const unsigned int nMaxKFid = pMap->GetMaxKFid();

	vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
	vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(nMaxKFid + 1);
	vector<g2o::VertexSim3Expmap *> vpVertices(nMaxKFid + 1);

	const int minFeat = 100; // TODO Check. originally 100

	// Set KeyFrame vertices
	for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
		KeyFrame *pKF = vpKFs[i];
		if (pKF->isBad()) {
			continue;
		}
		g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

		const int nIDi = pKF->mnId;

		Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
		Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKF->GetTranslation());
		g2o::Sim3 Siw(Rcw, tcw, 1.0);
		vScw[nIDi] = Siw;
		VSim3->setEstimate(Siw);

		if (pKF->mnBALocalForKF == pCurKF->mnId || pKF->mnBAFixedForKF == pCurKF->mnId) {
			cout << "fixed fk: " << pKF->mnId << endl;
			VSim3->setFixed(true);
		}
		else {
			VSim3->setFixed(false);
		}

		VSim3->setId(nIDi);
		VSim3->setMarginalized(false);
		// TODO Check
		// VSim3->_fix_scale = bFixScale;

		optimizer.addVertex(VSim3);

		vpVertices[nIDi] = VSim3;
	}

	set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

	const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

	int count_edges[3] = {0, 0, 0};
	// Set normal edges
	for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
		KeyFrame *pKF = vpKFs[i];

		const int nIDi = pKF->mnId;

		g2o::Sim3 Swi;

		LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

		if (iti != NonCorrectedSim3.end()) {
			Swi = (iti->second).inverse();
		}
		else {
			Swi = vScw[nIDi].inverse();
		}

		KeyFrame *pParentKF = pKF->GetParent();

		// Spanning tree edge
		if (pParentKF) {
			int nIDj = pParentKF->mnId;

			g2o::Sim3 Sjw;
			LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

			if (itj != NonCorrectedSim3.end()) {
				Sjw = itj->second;
			}
			else {
				Sjw = vScw[nIDj];
			}

			g2o::Sim3 Sji = Sjw * Swi;

			g2o::EdgeSim3 *e = new g2o::EdgeSim3();
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
			e->setMeasurement(Sji);

			e->information() = matLambda;
			optimizer.addEdge(e);
			count_edges[0]++;
		}

		// Loop edges
		const set<KeyFrame *> sLoopEdges = pKF->GetLoopEdges();
		for (set<KeyFrame *>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++) {
			KeyFrame *pLKF = *sit;
			if (pLKF->mnId < pKF->mnId) {
				g2o::Sim3 Slw;
				LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

				if (itl != NonCorrectedSim3.end()) {
					Slw = itl->second;
				}
				else {
					Slw = vScw[pLKF->mnId];
				}

				g2o::Sim3 Sli = Slw * Swi;
				g2o::EdgeSim3 *el = new g2o::EdgeSim3();
				el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pLKF->mnId)));
				el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
				el->setMeasurement(Sli);
				el->information() = matLambda;
				optimizer.addEdge(el);
				count_edges[1]++;
			}
		}

		// Covisibility graph edges
		const vector<KeyFrame *> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
		for (vector<KeyFrame *>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++) {
			KeyFrame *pKFn = *vit;
			if (pKFn && pKFn != pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn)) {
				if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
					// just one edge between frames
					if (sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId), max(pKF->mnId, pKFn->mnId)))) {
						continue;
					}

					g2o::Sim3 Snw;

					LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

					if (itn != NonCorrectedSim3.end()) {
						Snw = itn->second;
					}
					else {
						Snw = vScw[pKFn->mnId];
					}

					g2o::Sim3 Sni = Snw * Swi;

					g2o::EdgeSim3 *en = new g2o::EdgeSim3();
					en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFn->mnId)));
					en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
					en->setMeasurement(Sni);
					en->information() = matLambda;
					optimizer.addEdge(en);
					count_edges[2]++;
				}
			}
		}
	}

	Verbose::PrintMess("edges pose graph: " + to_string(count_edges[0]) + ", " + to_string(count_edges[1]) + ", "
		                   + to_string(count_edges[2]), Verbose::VERBOSITY_DEBUG);
	// Optimize!
	optimizer.initializeOptimization();
	optimizer.setVerbose(false);
	optimizer.optimize(20);

	unique_lock<shared_timed_mutex> lock(pMap->mMutexMapUpdate);

	// SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];

		const int nIDi = pKFi->mnId;

		g2o::VertexSim3Expmap *VSim3 = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(nIDi));
		g2o::Sim3 CorrectedSiw = VSim3->estimate();
		vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
		Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
		Eigen::Vector3d eigt = CorrectedSiw.translation();
		double s = CorrectedSiw.scale();

		eigt *= (1. / s); //[R t/s;0 1]

		cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);

		pKFi->SetPose(Tiw);
	}

	// Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
	for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
		MapPoint *pMP = vpMPs[i];

		if (pMP->isBad()) {
			continue;
		}

		int nIDr;
		if (pMP->mnCorrectedByKF == pCurKF->mnId) {
			nIDr = pMP->mnCorrectedReference;
		}
		else {
			KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
			nIDr = pRefKF->mnId;
		}

		g2o::Sim3 Srw = vScw[nIDr];
		g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

		cv::Mat P3Dw = pMP->GetWorldPos();
		Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
		Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

		cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
		pMP->SetWorldPos(cvCorrectedP3Dw);

		pMP->UpdateNormalAndDepth();
	}

	// TODO Check this changeindex
	pMap->IncreaseChangeIndex();
}

int Optimizer::OptimizeSim3(KeyFrame *pKF1,
                            KeyFrame *pKF2,
                            vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12,
                            const float th2,
                            const bool bFixScale)
{
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);

	// Calibration
	const cv::Mat &K1 = pKF1->mK;
	const cv::Mat &K2 = pKF2->mK;

	// Camera poses
	const cv::Mat R1w = pKF1->GetRotation();
	const cv::Mat t1w = pKF1->GetTranslation();
	const cv::Mat R2w = pKF2->GetRotation();
	const cv::Mat t2w = pKF2->GetTranslation();

	// Set Sim3 vertex
	g2o::VertexSim3Expmap *vSim3 = new g2o::VertexSim3Expmap();
	vSim3->_fix_scale = bFixScale;
	vSim3->setEstimate(g2oS12);
	vSim3->setId(0);
	vSim3->setFixed(false);
	vSim3->_principle_point1[0] = K1.at<float>(0, 2);
	vSim3->_principle_point1[1] = K1.at<float>(1, 2);
	vSim3->_focal_length1[0] = K1.at<float>(0, 0);
	vSim3->_focal_length1[1] = K1.at<float>(1, 1);
	vSim3->_principle_point2[0] = K2.at<float>(0, 2);
	vSim3->_principle_point2[1] = K2.at<float>(1, 2);
	vSim3->_focal_length2[0] = K2.at<float>(0, 0);
	vSim3->_focal_length2[1] = K2.at<float>(1, 1);
	optimizer.addVertex(vSim3);

	// Set MapPoint vertices
	const int N = vpMatches1.size();
	const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
	vector<g2o::EdgeSim3ProjectXYZ *> vpEdges12;
	vector<g2o::EdgeInverseSim3ProjectXYZ *> vpEdges21;
	vector<size_t> vnIndexEdge;

	vnIndexEdge.reserve(2 * N);
	vpEdges12.reserve(2 * N);
	vpEdges21.reserve(2 * N);

	const float deltaHuber = sqrt(th2);

	int nCorrespondences = 0;

	for (int i = 0; i < N; i++) {
		if (!vpMatches1[i]) {
			continue;
		}

		MapPoint *pMP1 = vpMapPoints1[i];
		MapPoint *pMP2 = vpMatches1[i];

		const int id1 = 2 * i + 1;
		const int id2 = 2 * (i + 1);

		const int i2 = get<0>(pMP2->GetIndexInKeyFrame(pKF2));

		if (pMP1 && pMP2) {
			if (!pMP1->isBad() && !pMP2->isBad() && i2 >= 0) {
				g2o::VertexSBAPointXYZ *vPoint1 = new g2o::VertexSBAPointXYZ();
				cv::Mat P3D1w = pMP1->GetWorldPos();
				cv::Mat P3D1c = R1w * P3D1w + t1w;
				vPoint1->setEstimate(Converter::toVector3d(P3D1c));
				vPoint1->setId(id1);
				vPoint1->setFixed(true);
				optimizer.addVertex(vPoint1);

				g2o::VertexSBAPointXYZ *vPoint2 = new g2o::VertexSBAPointXYZ();
				cv::Mat P3D2w = pMP2->GetWorldPos();
				cv::Mat P3D2c = R2w * P3D2w + t2w;
				vPoint2->setEstimate(Converter::toVector3d(P3D2c));
				vPoint2->setId(id2);
				vPoint2->setFixed(true);
				optimizer.addVertex(vPoint2);
			}
			else {
				continue;
			}
		}
		else {
			continue;
		}

		nCorrespondences++;

		// Set edge x1 = S12*X2
		Eigen::Matrix<double, 2, 1> obs1;
		const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
		obs1 << kpUn1.pt.x, kpUn1.pt.y;

		g2o::EdgeSim3ProjectXYZ *e12 = new g2o::EdgeSim3ProjectXYZ();
		e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id2)));
		e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
		e12->setMeasurement(obs1);
		const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
		e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

		g2o::RobustKernelHuber *rk1 = new g2o::RobustKernelHuber;
		e12->setRobustKernel(rk1);
		rk1->setDelta(deltaHuber);
		optimizer.addEdge(e12);

		// Set edge x2 = S21*X1
		Eigen::Matrix<double, 2, 1> obs2;
		const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
		obs2 << kpUn2.pt.x, kpUn2.pt.y;

		g2o::EdgeInverseSim3ProjectXYZ *e21 = new g2o::EdgeInverseSim3ProjectXYZ();

		e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id1)));
		e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
		e21->setMeasurement(obs2);
		float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
		e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

		g2o::RobustKernelHuber *rk2 = new g2o::RobustKernelHuber;
		e21->setRobustKernel(rk2);
		rk2->setDelta(deltaHuber);
		optimizer.addEdge(e21);

		vpEdges12.push_back(e12);
		vpEdges21.push_back(e21);
		vnIndexEdge.push_back(i);
	}

	// Optimize!
	optimizer.initializeOptimization();
	optimizer.optimize(5);

	// Check inliers
	int nBad = 0;
	for (size_t i = 0; i < vpEdges12.size(); i++) {
		g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
		g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
		if (!e12 || !e21) {
			continue;
		}

		if (e12->chi2() > th2 || e21->chi2() > th2) {
			size_t idx = vnIndexEdge[i];
			vpMatches1[idx] = static_cast<MapPoint *>(NULL);
			optimizer.removeEdge(e12);
			optimizer.removeEdge(e21);
			vpEdges12[i] = static_cast<g2o::EdgeSim3ProjectXYZ *>(NULL);
			vpEdges21[i] = static_cast<g2o::EdgeInverseSim3ProjectXYZ *>(NULL);
			nBad++;
		}
	}

	int nMoreIterations;
	if (nBad > 0) {
		nMoreIterations = 10;
	}
	else {
		nMoreIterations = 5;
	}

	if (nCorrespondences - nBad < 10) {
		return 0;
	}

	// Optimize again only with inliers

	optimizer.initializeOptimization();
	optimizer.optimize(nMoreIterations);

	int nIn = 0;
	for (size_t i = 0; i < vpEdges12.size(); i++) {
		g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
		g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
		if (!e12 || !e21) {
			continue;
		}

		if (e12->chi2() > th2 || e21->chi2() > th2) {
			size_t idx = vnIndexEdge[i];
			vpMatches1[idx] = static_cast<MapPoint *>(NULL);
		}
		else {
			nIn++;
		}
	}

	// Recover optimized Sim3
	g2o::VertexSim3Expmap *vSim3_recov = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
	g2oS12 = vSim3_recov->estimate();

	return nIn;
}

int Optimizer::OptimizeSim3(KeyFrame *pKF1,
                            KeyFrame *pKF2,
                            vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12,
                            const float th2,
                            const bool bFixScale,
                            Eigen::Matrix<double, 7, 7> &mAcumHessian,
                            const bool bAllPoints)
{
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);

	// Camera poses
	const cv::Mat R1w = pKF1->GetRotation();
	const cv::Mat t1w = pKF1->GetTranslation();
	const cv::Mat R2w = pKF2->GetRotation();
	const cv::Mat t2w = pKF2->GetTranslation();

	// Set Sim3 vertex
	ORB_SLAM3::VertexSim3Expmap *vSim3 = new ORB_SLAM3::VertexSim3Expmap();
	vSim3->_fix_scale = bFixScale;
	vSim3->setEstimate(g2oS12);
	vSim3->setId(0);
	vSim3->setFixed(false);
	vSim3->pCamera1 = pKF1->mpCamera;
	vSim3->pCamera2 = pKF2->mpCamera;
	optimizer.addVertex(vSim3);

	// Set MapPoint vertices
	const int N = vpMatches1.size();
	const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
	vector<ORB_SLAM3::EdgeSim3ProjectXYZ *> vpEdges12;
	vector<ORB_SLAM3::EdgeInverseSim3ProjectXYZ *> vpEdges21;
	vector<size_t> vnIndexEdge;
	vector<bool> vbIsInKF2;

	vnIndexEdge.reserve(2 * N);
	vpEdges12.reserve(2 * N);
	vpEdges21.reserve(2 * N);
	vbIsInKF2.reserve(2 * N);

	const float deltaHuber = sqrt(th2);

	int nCorrespondences = 0;
	int nBadMPs = 0;
	int nInKF2 = 0;
	int nOutKF2 = 0;
	int nMatchWithoutMP = 0;

	vector<int> vIdsOnlyInKF2;

	for (int i = 0; i < N; i++) {
		if (!vpMatches1[i]) {
			continue;
		}

		MapPoint *pMP1 = vpMapPoints1[i];
		MapPoint *pMP2 = vpMatches1[i];

		const int id1 = 2 * i + 1;
		const int id2 = 2 * (i + 1);

		const int i2 = get<0>(pMP2->GetIndexInKeyFrame(pKF2));
		/*if(i2 < 0)
            cout << "Sim3-OPT: Error, there is a matched which is not find it" << endl;*/

		cv::Mat P3D1c;
		cv::Mat P3D2c;

		if (pMP1 && pMP2) {
			//if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
			if (!pMP1->isBad() && !pMP2->isBad()) {
				g2o::VertexSBAPointXYZ *vPoint1 = new g2o::VertexSBAPointXYZ();
				cv::Mat P3D1w = pMP1->GetWorldPos();
				P3D1c = R1w * P3D1w + t1w;
				vPoint1->setEstimate(Converter::toVector3d(P3D1c));
				vPoint1->setId(id1);
				vPoint1->setFixed(true);
				optimizer.addVertex(vPoint1);

				g2o::VertexSBAPointXYZ *vPoint2 = new g2o::VertexSBAPointXYZ();
				cv::Mat P3D2w = pMP2->GetWorldPos();
				P3D2c = R2w * P3D2w + t2w;
				vPoint2->setEstimate(Converter::toVector3d(P3D2c));
				vPoint2->setId(id2);
				vPoint2->setFixed(true);
				optimizer.addVertex(vPoint2);
			}
			else {
				nBadMPs++;
				continue;
			}
		}
		else {
			nMatchWithoutMP++;

			//TODO The 3D position in KF1 doesn't exist
			if (!pMP2->isBad()) {
				g2o::VertexSBAPointXYZ *vPoint2 = new g2o::VertexSBAPointXYZ();
				cv::Mat P3D2w = pMP2->GetWorldPos();
				P3D2c = R2w * P3D2w + t2w;
				vPoint2->setEstimate(Converter::toVector3d(P3D2c));
				vPoint2->setId(id2);
				vPoint2->setFixed(true);
				optimizer.addVertex(vPoint2);

				vIdsOnlyInKF2.push_back(id2);
			}
			continue;
		}

		if (i2 < 0 && !bAllPoints) {
			Verbose::PrintMess("    Remove point -> i2: " + to_string(i2) + "; bAllPoints: " + to_string(bAllPoints),
			                   Verbose::VERBOSITY_DEBUG);
			continue;
		}

		if (P3D2c.at<float>(2) < 0) {
			Verbose::PrintMess("Sim3: Z coordinate is negative", Verbose::VERBOSITY_DEBUG);
			continue;
		}

		nCorrespondences++;

		// Set edge x1 = S12*X2
		Eigen::Matrix<double, 2, 1> obs1;
		const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
		obs1 << kpUn1.pt.x, kpUn1.pt.y;

		ORB_SLAM3::EdgeSim3ProjectXYZ *e12 = new ORB_SLAM3::EdgeSim3ProjectXYZ();

		e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id2)));
		e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
		e12->setMeasurement(obs1);
		const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
		e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

		g2o::RobustKernelHuber *rk1 = new g2o::RobustKernelHuber;
		e12->setRobustKernel(rk1);
		rk1->setDelta(deltaHuber);
		optimizer.addEdge(e12);

		// Set edge x2 = S21*X1
		Eigen::Matrix<double, 2, 1> obs2;
		cv::KeyPoint kpUn2;
		bool inKF2;
		if (i2 >= 0) {
			kpUn2 = pKF2->mvKeysUn[i2];
			obs2 << kpUn2.pt.x, kpUn2.pt.y;
			inKF2 = true;

			nInKF2++;
		}
		else {
			float invz = 1 / P3D2c.at<float>(2);
			float x = P3D2c.at<float>(0) * invz;
			float y = P3D2c.at<float>(1) * invz;

			obs2 << x, y;
			kpUn2 = cv::KeyPoint(cv::Point2f(x, y), pMP2->mnTrackScaleLevel);

			inKF2 = false;
			nOutKF2++;
		}

		ORB_SLAM3::EdgeInverseSim3ProjectXYZ *e21 = new ORB_SLAM3::EdgeInverseSim3ProjectXYZ();

		e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id1)));
		e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
		e21->setMeasurement(obs2);
		float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
		e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

		g2o::RobustKernelHuber *rk2 = new g2o::RobustKernelHuber;
		e21->setRobustKernel(rk2);
		rk2->setDelta(deltaHuber);
		optimizer.addEdge(e21);

		vpEdges12.push_back(e12);
		vpEdges21.push_back(e21);
		vnIndexEdge.push_back(i);

		vbIsInKF2.push_back(inKF2);
	}

	Verbose::PrintMess(
		"Sim3: There are " + to_string(nCorrespondences) + " matches, " + to_string(nInKF2) + " are in the KF and "
			+ to_string(nOutKF2) + " are in the connected KFs. There are " + to_string(nMatchWithoutMP)
			+ " matches which have not an associate MP", Verbose::VERBOSITY_DEBUG);

	// Optimize!
	optimizer.initializeOptimization();
	optimizer.optimize(5);

	// Check inliers
	int nBad = 0;
	int nBadOutKF2 = 0;
	for (size_t i = 0; i < vpEdges12.size(); i++) {
		ORB_SLAM3::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
		ORB_SLAM3::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
		if (!e12 || !e21) {
			continue;
		}

		if (e12->chi2() > th2 || e21->chi2() > th2) {
			size_t idx = vnIndexEdge[i];
			vpMatches1[idx] = static_cast<MapPoint *>(NULL);
			optimizer.removeEdge(e12);
			optimizer.removeEdge(e21);
			vpEdges12[i] = static_cast<ORB_SLAM3::EdgeSim3ProjectXYZ *>(NULL);
			vpEdges21[i] = static_cast<ORB_SLAM3::EdgeInverseSim3ProjectXYZ *>(NULL);
			nBad++;

			if (!vbIsInKF2[i]) {
				nBadOutKF2++;
			}
			continue;
		}

		//Check if remove the robust adjustment improve the result
		e12->setRobustKernel(0);
		e21->setRobustKernel(0);
	}

	Verbose::PrintMess(
		"Sim3: First Opt -> Correspondences: " + to_string(nCorrespondences) + "; nBad: " + to_string(nBad)
			+ "; nBadOutKF2: " + to_string(nBadOutKF2), Verbose::VERBOSITY_DEBUG);

	int nMoreIterations;
	if (nBad > 0) {
		nMoreIterations = 10;
	}
	else {
		nMoreIterations = 5;
	}

	if (nCorrespondences - nBad < 10) {
		return 0;
	}

	// Optimize again only with inliers

	optimizer.initializeOptimization();
	optimizer.optimize(nMoreIterations);

	int nIn = 0;
	mAcumHessian = Eigen::MatrixXd::Zero(7, 7);
	for (size_t i = 0; i < vpEdges12.size(); i++) {
		ORB_SLAM3::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
		ORB_SLAM3::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
		if (!e12 || !e21) {
			continue;
		}

		e12->computeError();
		e21->computeError();

		if (e12->chi2() > th2 || e21->chi2() > th2) {
			size_t idx = vnIndexEdge[i];
			vpMatches1[idx] = static_cast<MapPoint *>(NULL);
		}
		else {
			nIn++;
			//mAcumHessian += e12->GetHessian();
		}
	}

	// Recover optimized Sim3
	//Verbose::PrintMess("Sim3: Initial seed " + g2oS12, Verbose::VERBOSITY_DEBUG);
	g2o::VertexSim3Expmap *vSim3_recov = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
	g2oS12 = vSim3_recov->estimate();
	//Verbose::PrintMess("Sim3: Optimized solution " + g2oS12, Verbose::VERBOSITY_DEBUG);

	return nIn;
}

int Optimizer::OptimizeSim3(KeyFrame *pKF1,
                            KeyFrame *pKF2,
                            vector<MapPoint *> &vpMatches1,
                            vector<KeyFrame *> &vpMatches1KF,
                            g2o::Sim3 &g2oS12,
                            const float th2,
                            const bool bFixScale,
                            Eigen::Matrix<double, 7, 7> &mAcumHessian,
                            const bool bAllPoints)
{
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);

	// Calibration
	const cv::Mat &K1 = pKF1->mK;
	const cv::Mat &K2 = pKF2->mK;

	// Camera poses
	const cv::Mat R1w = pKF1->GetRotation();
	const cv::Mat t1w = pKF1->GetTranslation();
	Verbose::PrintMess("Extracted rotation and traslation from the first KF ", Verbose::VERBOSITY_DEBUG);
	const cv::Mat R2w = pKF2->GetRotation();
	const cv::Mat t2w = pKF2->GetTranslation();
	Verbose::PrintMess("Extracted rotation and traslation from the second KF ", Verbose::VERBOSITY_DEBUG);

	// Set Sim3 vertex
	g2o::VertexSim3Expmap *vSim3 = new g2o::VertexSim3Expmap();
	vSim3->_fix_scale = bFixScale;
	vSim3->setEstimate(g2oS12);
	vSim3->setId(0);
	vSim3->setFixed(false);
	vSim3->_principle_point1[0] = K1.at<float>(0, 2);
	vSim3->_principle_point1[1] = K1.at<float>(1, 2);
	vSim3->_focal_length1[0] = K1.at<float>(0, 0);
	vSim3->_focal_length1[1] = K1.at<float>(1, 1);
	vSim3->_principle_point2[0] = K2.at<float>(0, 2);
	vSim3->_principle_point2[1] = K2.at<float>(1, 2);
	vSim3->_focal_length2[0] = K2.at<float>(0, 0);
	vSim3->_focal_length2[1] = K2.at<float>(1, 1);
	optimizer.addVertex(vSim3);

	// Set MapPoint vertices
	const int N = vpMatches1.size();
	const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
	vector<ORB_SLAM3::EdgeSim3ProjectXYZ *> vpEdges12;
	vector<ORB_SLAM3::EdgeInverseSim3ProjectXYZ *> vpEdges21;
	vector<size_t> vnIndexEdge;

	vnIndexEdge.reserve(2 * N);
	vpEdges12.reserve(2 * N);
	vpEdges21.reserve(2 * N);

	const float deltaHuber = sqrt(th2);

	int nCorrespondences = 0;

	KeyFrame *pKFm = pKF2;
	for (int i = 0; i < N; i++) {
		if (!vpMatches1[i]) {
			continue;
		}

		MapPoint *pMP1 = vpMapPoints1[i];
		MapPoint *pMP2 = vpMatches1[i];

		const int id1 = 2 * i + 1;
		const int id2 = 2 * (i + 1);

		pKFm = vpMatches1KF[i];
		const int i2 = get<0>(pMP2->GetIndexInKeyFrame(pKFm));
		if (i2 < 0) {
			Verbose::PrintMess("Sim3-OPT: Error, there is a matched which is not find it", Verbose::VERBOSITY_DEBUG);
		}

		cv::Mat P3D2c;

		if (pMP1 && pMP2) {
			//if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
			if (!pMP1->isBad() && !pMP2->isBad()) {
				g2o::VertexSBAPointXYZ *vPoint1 = new g2o::VertexSBAPointXYZ();
				cv::Mat P3D1w = pMP1->GetWorldPos();
				cv::Mat P3D1c = R1w * P3D1w + t1w;
				vPoint1->setEstimate(Converter::toVector3d(P3D1c));
				vPoint1->setId(id1);
				vPoint1->setFixed(true);
				optimizer.addVertex(vPoint1);

				g2o::VertexSBAPointXYZ *vPoint2 = new g2o::VertexSBAPointXYZ();
				cv::Mat P3D2w = pMP2->GetWorldPos();
				P3D2c = R2w * P3D2w + t2w;
				vPoint2->setEstimate(Converter::toVector3d(P3D2c));
				vPoint2->setId(id2);
				vPoint2->setFixed(true);
				optimizer.addVertex(vPoint2);
			}
			else {
				continue;
			}
		}
		else {
			continue;
		}

		if (i2 < 0 && !bAllPoints) {
			Verbose::PrintMess("    Remove point -> i2: " + to_string(i2) + "; bAllPoints: " + to_string(bAllPoints),
			                   Verbose::VERBOSITY_DEBUG);
			continue;
		}

		nCorrespondences++;

		// Set edge x1 = S12*X2
		Eigen::Matrix<double, 2, 1> obs1;
		const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
		obs1 << kpUn1.pt.x, kpUn1.pt.y;

		ORB_SLAM3::EdgeSim3ProjectXYZ *e12 = new ORB_SLAM3::EdgeSim3ProjectXYZ();
		e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id2)));
		e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
		e12->setMeasurement(obs1);
		const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
		e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

		g2o::RobustKernelHuber *rk1 = new g2o::RobustKernelHuber;
		e12->setRobustKernel(rk1);
		rk1->setDelta(deltaHuber);
		optimizer.addEdge(e12);

		// Set edge x2 = S21*X1
		Eigen::Matrix<double, 2, 1> obs2;
		cv::KeyPoint kpUn2;
		if (i2 >= 0 && pKFm == pKF2) {
			kpUn2 = pKFm->mvKeysUn[i2];
			obs2 << kpUn2.pt.x, kpUn2.pt.y;
		}
		else {
			float invz = 1 / P3D2c.at<float>(2);
			float x = P3D2c.at<float>(0) * invz;
			float y = P3D2c.at<float>(1) * invz;

			// Project in image and check it is not outside
			float u = pKF2->fx * x + pKFm->cx;
			float v = pKF2->fy * y + pKFm->cy;
			obs2 << u, v;
			kpUn2 = cv::KeyPoint(cv::Point2f(u, v), pMP2->mnTrackScaleLevel);
		}

		ORB_SLAM3::EdgeInverseSim3ProjectXYZ *e21 = new ORB_SLAM3::EdgeInverseSim3ProjectXYZ();

		e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id1)));
		e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
		e21->setMeasurement(obs2);
		float invSigmaSquare2 = pKFm->mvInvLevelSigma2[kpUn2.octave];
		e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

		g2o::RobustKernelHuber *rk2 = new g2o::RobustKernelHuber;
		e21->setRobustKernel(rk2);
		rk2->setDelta(deltaHuber);
		optimizer.addEdge(e21);

		vpEdges12.push_back(e12);
		vpEdges21.push_back(e21);
		vnIndexEdge.push_back(i);
	}

	// Optimize!
	optimizer.initializeOptimization();
	optimizer.optimize(5);

	// Check inliers
	int nBad = 0;
	for (size_t i = 0; i < vpEdges12.size(); i++) {
		ORB_SLAM3::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
		ORB_SLAM3::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
		if (!e12 || !e21) {
			continue;
		}

		if (e12->chi2() > th2 || e21->chi2() > th2) {
			size_t idx = vnIndexEdge[i];
			vpMatches1[idx] = static_cast<MapPoint *>(NULL);
			optimizer.removeEdge(e12);
			optimizer.removeEdge(e21);
			vpEdges12[i] = static_cast<ORB_SLAM3::EdgeSim3ProjectXYZ *>(NULL);
			vpEdges21[i] = static_cast<ORB_SLAM3::EdgeInverseSim3ProjectXYZ *>(NULL);
			nBad++;
			continue;
		}

		//Check if remove the robust adjustment improve the result
		e12->setRobustKernel(0);
		e21->setRobustKernel(0);
	}

	//cout << "Sim3 -> Correspondences: " << nCorrespondences << "; nBad: " << nBad << endl;

	int nMoreIterations;
	if (nBad > 0) {
		nMoreIterations = 10;
	}
	else {
		nMoreIterations = 5;
	}

	if (nCorrespondences - nBad < 10) {
		return 0;
	}

	// Optimize again only with inliers

	optimizer.initializeOptimization();
	optimizer.optimize(nMoreIterations);

	int nIn = 0;
	mAcumHessian = Eigen::MatrixXd::Zero(7, 7);
	for (size_t i = 0; i < vpEdges12.size(); i++) {
		ORB_SLAM3::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
		ORB_SLAM3::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
		if (!e12 || !e21) {
			continue;
		}

		e12->computeError();
		e21->computeError();

		if (e12->chi2() > th2 || e21->chi2() > th2) {
			size_t idx = vnIndexEdge[i];
			vpMatches1[idx] = static_cast<MapPoint *>(NULL);
		}
		else {
			nIn++;
			//mAcumHessian += e12->GetHessian();
		}
	}

	// Recover optimized Sim3
	ORB_SLAM3::VertexSim3Expmap *vSim3_recov = static_cast<ORB_SLAM3::VertexSim3Expmap *>(optimizer.vertex(0));
	g2oS12 = vSim3_recov->estimate();

	return nIn;
}

void Optimizer::LocalInertialBA(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, bool bLarge, bool bRecInit)
{
	std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
	Map *pCurrentMap = pKF->GetMap();

	int maxOpt = 10;
	int opt_it = 10;
	if (bLarge) {
		maxOpt = 25;
		opt_it = 4;
	}
	const int Nd = std::min((int)pCurrentMap->KeyFramesInMap() - 2, maxOpt);
	const unsigned long maxKFid = pKF->mnId;

	vector<KeyFrame *> vpOptimizableKFs;
	const vector<KeyFrame *> vpNeighsKFs = pKF->GetVectorCovisibleKeyFrames();
	list<KeyFrame *> lpOptVisKFs;

	vpOptimizableKFs.reserve(Nd);
	vpOptimizableKFs.push_back(pKF);
	pKF->mnBALocalForKF = pKF->mnId;
	for (int i = 1; i < Nd; i++) {
		if (vpOptimizableKFs.back()->mPrevKF) {
			vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
			vpOptimizableKFs.back()->mnBALocalForKF = pKF->mnId;
		}
		else {
			break;
		}
	}

	int N = vpOptimizableKFs.size();

	// Optimizable points seen by temporal optimizable keyframes
	list<MapPoint *> lLocalMapPoints;
	for (int i = 0; i < N; i++) {
		vector<MapPoint *> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
		for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
			MapPoint *pMP = *vit;
			if (pMP) {
				if (!pMP->isBad()) {
					if (pMP->mnBALocalForKF != pKF->mnId) {
						lLocalMapPoints.push_back(pMP);
						pMP->mnBALocalForKF = pKF->mnId;
					}
				}
			}
		}
	}

	// Fixed Keyframe: First frame previous KF to optimization window)
	list<KeyFrame *> lFixedKeyFrames;
	if (vpOptimizableKFs.back()->mPrevKF) {
		lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
		vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
	}
	else {
		vpOptimizableKFs.back()->mnBALocalForKF = 0;
		vpOptimizableKFs.back()->mnBAFixedForKF = pKF->mnId;
		lFixedKeyFrames.push_back(vpOptimizableKFs.back());
		vpOptimizableKFs.pop_back();
	}

	// Optimizable visual KFs
	const int maxCovKF = 0;
	for (int i = 0, iend = vpNeighsKFs.size(); i < iend; i++) {
		if (lpOptVisKFs.size() >= maxCovKF) {
			break;
		}

		KeyFrame *pKFi = vpNeighsKFs[i];
		if (pKFi->mnBALocalForKF == pKF->mnId || pKFi->mnBAFixedForKF == pKF->mnId) {
			continue;
		}
		pKFi->mnBALocalForKF = pKF->mnId;
		if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
			lpOptVisKFs.push_back(pKFi);

			vector<MapPoint *> vpMPs = pKFi->GetMapPointMatches();
			for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
				MapPoint *pMP = *vit;
				if (pMP) {
					if (!pMP->isBad()) {
						if (pMP->mnBALocalForKF != pKF->mnId) {
							lLocalMapPoints.push_back(pMP);
							pMP->mnBALocalForKF = pKF->mnId;
						}
					}
				}
			}
		}
	}

	// Fixed KFs which are not covisible optimizable
	const int maxFixKF = 200;

	for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
		map<KeyFrame *, tuple<int, int>> observations = (*lit)->GetObservations();
		for (map<KeyFrame *, tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end();
		     mit != mend; mit++) {
			KeyFrame *pKFi = mit->first;

			if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
				pKFi->mnBAFixedForKF = pKF->mnId;
				if (!pKFi->isBad()) {
					lFixedKeyFrames.push_back(pKFi);
					break;
				}
			}
		}
		if (lFixedKeyFrames.size() >= maxFixKF) {
			break;
		}
	}

	bool bNonFixed = (lFixedKeyFrames.size() == 0);

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	if (bLarge) {
		g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
		solver->setUserLambdaInit(1e-2); // to avoid iterating for finding optimal lambda
		optimizer.setAlgorithm(solver);
	}
	else {
		g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
		solver->setUserLambdaInit(1e0);
		optimizer.setAlgorithm(solver);
	}

	// Set Local temporal KeyFrame vertices
	N = vpOptimizableKFs.size();
	for (int i = 0; i < N; i++) {
		KeyFrame *pKFi = vpOptimizableKFs[i];

		VertexPose *VP = new VertexPose(pKFi);
		VP->setId(pKFi->mnId);
		VP->setFixed(false);
		optimizer.addVertex(VP);

		if (pKFi->bImu) {
			VertexVelocity *VV = new VertexVelocity(pKFi);
			VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
			VV->setFixed(false);
			optimizer.addVertex(VV);
			VertexGyroBias *VG = new VertexGyroBias(pKFi);
			VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
			VG->setFixed(false);
			optimizer.addVertex(VG);
			VertexAccBias *VA = new VertexAccBias(pKFi);
			VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
			VA->setFixed(false);
			optimizer.addVertex(VA);
		}
	}

	// Set Local visual KeyFrame vertices
	for (list<KeyFrame *>::iterator it = lpOptVisKFs.begin(), itEnd = lpOptVisKFs.end(); it != itEnd; it++) {
		KeyFrame *pKFi = *it;
		VertexPose *VP = new VertexPose(pKFi);
		VP->setId(pKFi->mnId);
		VP->setFixed(false);
		optimizer.addVertex(VP);
	}

	// Set Fixed KeyFrame vertices
	for (list<KeyFrame *>::iterator lit = lFixedKeyFrames.begin(), lend = lFixedKeyFrames.end(); lit != lend; lit++) {
		KeyFrame *pKFi = *lit;
		VertexPose *VP = new VertexPose(pKFi);
		VP->setId(pKFi->mnId);
		VP->setFixed(true);
		optimizer.addVertex(VP);

		if (pKFi->bImu) // This should be done only for keyframe just before temporal window
		{
			VertexVelocity *VV = new VertexVelocity(pKFi);
			VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
			VV->setFixed(true);
			optimizer.addVertex(VV);
			VertexGyroBias *VG = new VertexGyroBias(pKFi);
			VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
			VG->setFixed(true);
			optimizer.addVertex(VG);
			VertexAccBias *VA = new VertexAccBias(pKFi);
			VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
			VA->setFixed(true);
			optimizer.addVertex(VA);
		}
	}

	// Create intertial constraints
	vector<EdgeInertial *> vei(N, (EdgeInertial *)NULL);
	vector<EdgeGyroRW *> vegr(N, (EdgeGyroRW *)NULL);//bias
	vector<EdgeAccRW *> vear(N, (EdgeAccRW *)NULL);//bias

	for (int i = 0; i < N; i++) {
		KeyFrame *pKFi = vpOptimizableKFs[i];

		if (!pKFi->mPrevKF) {
			cout << "NOT INERTIAL LINK TO PREVIOUS FRAME!!!!" << endl;
			continue;
		}
		if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {
			pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
			g2o::HyperGraph::Vertex *VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
			g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);
			g2o::HyperGraph::Vertex *VG1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
			g2o::HyperGraph::Vertex *VA1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
			g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(pKFi->mnId);
			g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);
			g2o::HyperGraph::Vertex *VG2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
			g2o::HyperGraph::Vertex *VA2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);

			if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
				cerr << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1 << ", " << VP2 << ", " << VV2
				     << ", " << VG2 << ", " << VA2 << endl;
				continue;
			}

			vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

			vei[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
			vei[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
			vei[i]->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG1));
			vei[i]->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA1));
			vei[i]->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
			vei[i]->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));

			if (i == N - 1 || bRecInit) {
				// All inertial residuals are included without robust cost function, but not that one linking the
				// last optimizable keyframe inside of the local window and the first fixed keyframe out. The
				// information matrix for this measurement is also downweighted. This is done to avoid accumulating
				// error due to fixing variables.
				g2o::RobustKernelHuber *rki = new g2o::RobustKernelHuber;
				vei[i]->setRobustKernel(rki);
				if (i == N - 1) {
					vei[i]->setInformation(vei[i]->information() * 1e-2);
				}
				rki->setDelta(sqrt(16.92));
			}
			optimizer.addEdge(vei[i]);

			vegr[i] = new EdgeGyroRW();
			vegr[i]->setVertex(0, VG1);
			vegr[i]->setVertex(1, VG2);
			cv::Mat cvInfoG = pKFi->mpImuPreintegrated->C.rowRange(9, 12).colRange(9, 12).inv(cv::DECOMP_SVD);
			Eigen::Matrix3d InfoG;

			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
					InfoG(r, c) = cvInfoG.at<float>(r, c);
			vegr[i]->setInformation(InfoG);
			optimizer.addEdge(vegr[i]);

			// cout << "b";
			vear[i] = new EdgeAccRW();
			vear[i]->setVertex(0, VA1);
			vear[i]->setVertex(1, VA2);
			cv::Mat cvInfoA = pKFi->mpImuPreintegrated->C.rowRange(12, 15).colRange(12, 15).inv(cv::DECOMP_SVD);
			Eigen::Matrix3d InfoA;
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
					InfoA(r, c) = cvInfoA.at<float>(r, c);
			vear[i]->setInformation(InfoA);

			optimizer.addEdge(vear[i]);
		}
		else {
			cout << "ERROR building inertial edge" << endl;
		}
	}

	// Set MapPoint vertices
	const int nExpectedSize = (N + lFixedKeyFrames.size()) * lLocalMapPoints.size();

	// Mono
	vector<EdgeMono *> vpEdgesMono;
	vpEdgesMono.reserve(nExpectedSize);

	vector<KeyFrame *> vpEdgeKFMono;
	vpEdgeKFMono.reserve(nExpectedSize);

	vector<MapPoint *> vpMapPointEdgeMono;
	vpMapPointEdgeMono.reserve(nExpectedSize);

	// Stereo
	vector<EdgeStereo *> vpEdgesStereo;
	vpEdgesStereo.reserve(nExpectedSize);

	vector<KeyFrame *> vpEdgeKFStereo;
	vpEdgeKFStereo.reserve(nExpectedSize);

	vector<MapPoint *> vpMapPointEdgeStereo;
	vpMapPointEdgeStereo.reserve(nExpectedSize);

	const float thHuberMono = sqrt(5.991);
	const float chi2Mono2 = 5.991;
	const float thHuberStereo = sqrt(7.815);
	const float chi2Stereo2 = 7.815;

	const unsigned long iniMPid = maxKFid * 5;

	map<int, int> mVisEdges;
	for (int i = 0; i < N; i++) {
		KeyFrame *pKFi = vpOptimizableKFs[i];
		mVisEdges[pKFi->mnId] = 0;
	}
	for (list<KeyFrame *>::iterator lit = lFixedKeyFrames.begin(), lend = lFixedKeyFrames.end(); lit != lend; lit++) {
		mVisEdges[(*lit)->mnId] = 0;
	}

	for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
		MapPoint *pMP = *lit;
		g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));

		unsigned long id = pMP->mnId + iniMPid + 1;
		vPoint->setId(id);
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);
		const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

		// Create visual constraints
		for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(), mend = observations.end();
		     mit != mend; mit++) {
			KeyFrame *pKFi = mit->first;

			if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
				continue;
			}

			if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
				const int leftIndex = get<0>(mit->second);

				cv::KeyPoint kpUn;

				// Monocular left observation
				if (leftIndex != -1 && pKFi->mvuRight[leftIndex] < 0) {
					mVisEdges[pKFi->mnId]++;

					kpUn = pKFi->mvKeysUn[leftIndex];
					Eigen::Matrix<double, 2, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y;

					EdgeMono *e = new EdgeMono(0);

					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
					e->setMeasurement(obs);

					// Add here uncerteinty
					const float unc2 = pKFi->mpCamera->uncertainty2(obs);

					const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
					e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberMono);

					optimizer.addEdge(e);
					vpEdgesMono.push_back(e);
					vpEdgeKFMono.push_back(pKFi);
					vpMapPointEdgeMono.push_back(pMP);
				}
					// Stereo-observation
				else if (leftIndex != -1) // Stereo observation
				{
					kpUn = pKFi->mvKeysUn[leftIndex];
					mVisEdges[pKFi->mnId]++;

					const float kp_ur = pKFi->mvuRight[leftIndex];
					Eigen::Matrix<double, 3, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

					EdgeStereo *e = new EdgeStereo(0);

					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
					e->setMeasurement(obs);

					// Add here uncerteinty
					const float unc2 = pKFi->mpCamera->uncertainty2(obs.head(2));

					const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
					e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberStereo);

					optimizer.addEdge(e);
					vpEdgesStereo.push_back(e);
					vpEdgeKFStereo.push_back(pKFi);
					vpMapPointEdgeStereo.push_back(pMP);
				}

				// Monocular right observation
				if (pKFi->mpCamera2) {
					int rightIndex = get<1>(mit->second);

					if (rightIndex != -1) {
						rightIndex -= pKFi->NLeft;
						mVisEdges[pKFi->mnId]++;

						Eigen::Matrix<double, 2, 1> obs;
						cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
						obs << kp.pt.x, kp.pt.y;

						EdgeMono *e = new EdgeMono(1);

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
						e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
						e->setMeasurement(obs);

						// Add here uncerteinty
						const float unc2 = pKFi->mpCamera->uncertainty2(obs);

						const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
						e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

						g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(thHuberMono);

						optimizer.addEdge(e);
						vpEdgesMono.push_back(e);
						vpEdgeKFMono.push_back(pKFi);
						vpMapPointEdgeMono.push_back(pMP);
					}
				}
			}
		}
	}

	//cout << "Total map points: " << lLocalMapPoints.size() << endl;
	for (map<int, int>::iterator mit = mVisEdges.begin(), mend = mVisEdges.end(); mit != mend; mit++) {
		assert(mit->second >= 3);
	}

	optimizer.initializeOptimization();
	optimizer.computeActiveErrors();
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	float err = optimizer.activeRobustChi2();
	optimizer.optimize(opt_it); // Originally to 2
	float err_end = optimizer.activeRobustChi2();
	if (pbStopFlag) {
		optimizer.setForceStopFlag(pbStopFlag);
	}

	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

	vector<pair<KeyFrame *, MapPoint *>> vToErase;
	vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

	// Check inlier observations
	// Mono
	for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
		EdgeMono *e = vpEdgesMono[i];
		MapPoint *pMP = vpMapPointEdgeMono[i];
		bool bClose = pMP->mTrackDepth < 10.f;

		if (pMP->isBad()) {
			continue;
		}

		if ((e->chi2() > chi2Mono2 && !bClose) || (e->chi2() > 1.5f * chi2Mono2 && bClose) || !e->isDepthPositive()) {
			KeyFrame *pKFi = vpEdgeKFMono[i];
			vToErase.push_back(make_pair(pKFi, pMP));
		}
	}

	// Stereo
	for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
		EdgeStereo *e = vpEdgesStereo[i];
		MapPoint *pMP = vpMapPointEdgeStereo[i];

		if (pMP->isBad()) {
			continue;
		}

		if (e->chi2() > chi2Stereo2) {
			KeyFrame *pKFi = vpEdgeKFStereo[i];
			vToErase.push_back(make_pair(pKFi, pMP));
		}
	}

	// Get Map Mutex and erase outliers
	unique_lock<shared_timed_mutex> lock(pMap->mMutexMapUpdate);

	// TODO: Some convergence problems have been detected here
	//cout << "err0 = " << err << endl;
	//cout << "err_end = " << err_end << endl;
	if ((2 * err < err_end || isnan(err) || isnan(err_end)) && !bLarge) //bGN)
	{
		cout << "FAIL LOCAL-INERTIAL BA!!!!" << endl;
		return;
	}

	if (!vToErase.empty()) {
		for (size_t i = 0; i < vToErase.size(); i++) {
			KeyFrame *pKFi = vToErase[i].first;
			MapPoint *pMPi = vToErase[i].second;
			pKFi->EraseMapPointMatch(pMPi);
			pMPi->EraseObservation(pKFi);
		}
	}

	// Display main statistcis of optimization
	Verbose::PrintMess("LIBA KFs: " + to_string(N), Verbose::VERBOSITY_DEBUG);
	Verbose::PrintMess("LIBA bNonFixed?: " + to_string(bNonFixed), Verbose::VERBOSITY_DEBUG);
	Verbose::PrintMess("LIBA KFs visual outliers: " + to_string(vToErase.size()), Verbose::VERBOSITY_DEBUG);

	for (list<KeyFrame *>::iterator lit = lFixedKeyFrames.begin(), lend = lFixedKeyFrames.end(); lit != lend; lit++)
		(*lit)->mnBAFixedForKF = 0;

	// Recover optimized data
	// Local temporal Keyframes
	N = vpOptimizableKFs.size();
	for (int i = 0; i < N; i++) {
		KeyFrame *pKFi = vpOptimizableKFs[i];

		VertexPose *VP = static_cast<VertexPose *>(optimizer.vertex(pKFi->mnId));
		cv::Mat Tcw = Converter::toCvSE3(VP->estimate().Rcw[0], VP->estimate().tcw[0]);
		pKFi->SetPose(Tcw);
		pKFi->mnBALocalForKF = 0;

		if (pKFi->bImu) {
			VertexVelocity *VV = static_cast<VertexVelocity *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
			pKFi->SetVelocity(Converter::toCvMat(VV->estimate()));
			VertexGyroBias *VG = static_cast<VertexGyroBias *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
			VertexAccBias *VA = static_cast<VertexAccBias *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
			Vector6d b;
			b << VG->estimate(), VA->estimate();
			pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
		}
	}

	// Local visual KeyFrame
	for (list<KeyFrame *>::iterator it = lpOptVisKFs.begin(), itEnd = lpOptVisKFs.end(); it != itEnd; it++) {
		KeyFrame *pKFi = *it;
		VertexPose *VP = static_cast<VertexPose *>(optimizer.vertex(pKFi->mnId));
		cv::Mat Tcw = Converter::toCvSE3(VP->estimate().Rcw[0], VP->estimate().tcw[0]);
		pKFi->SetPose(Tcw);
		pKFi->mnBALocalForKF = 0;
	}

	//Points
	for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
		MapPoint *pMP = *lit;
		g2o::VertexSBAPointXYZ
			*vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + iniMPid + 1));
		pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMP->UpdateNormalAndDepth();
	}

	pMap->IncreaseChangeIndex();

	std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

	/*double t_const = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t0).count();
    double t_opt = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    double t_rec = std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
    /*std::cout << " Construction time: " << t_const << std::endl;
    std::cout << " Optimization time: " << t_opt << std::endl;
    std::cout << " Recovery time: " << t_rec << std::endl;
    std::cout << " Total time: " << t_const+t_opt+t_rec << std::endl;
    std::cout << " Optimization iterations: " << opt_it << std::endl;*/
}

Eigen::MatrixXd Optimizer::Marginalize(const Eigen::MatrixXd &H, const int &start, const int &end)
{
	// Goal
	// a  | ab | ac       a*  | 0 | ac*
	// ba | b  | bc  -->  0   | 0 | 0
	// ca | cb | c        ca* | 0 | c*

	// Size of block before block to marginalize
	const int a = start;
	// Size of block to marginalize
	const int b = end - start + 1;
	// Size of block after block to marginalize
	const int c = H.cols() - (end + 1);

	// Reorder as follows:
	// a  | ab | ac       a  | ac | ab
	// ba | b  | bc  -->  ca | c  | cb
	// ca | cb | c        ba | bc | b

	Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(), H.cols());
	if (a > 0) {
		Hn.block(0, 0, a, a) = H.block(0, 0, a, a);
		Hn.block(0, a + c, a, b) = H.block(0, a, a, b);
		Hn.block(a + c, 0, b, a) = H.block(a, 0, b, a);
	}
	if (a > 0 && c > 0) {
		Hn.block(0, a, a, c) = H.block(0, a + b, a, c);
		Hn.block(a, 0, c, a) = H.block(a + b, 0, c, a);
	}
	if (c > 0) {
		Hn.block(a, a, c, c) = H.block(a + b, a + b, c, c);
		Hn.block(a, a + c, c, b) = H.block(a + b, a, c, b);
		Hn.block(a + c, a, b, c) = H.block(a, a + b, b, c);
	}
	Hn.block(a + c, a + c, b, b) = H.block(a, a, b, b);

	// Perform marginalization (Schur complement)
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hn.block(a + c, a + c, b, b), Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv = svd.singularValues();
	for (int i = 0; i < b; ++i) {
		if (singularValues_inv(i) > 1e-6) {
			singularValues_inv(i) = 1.0 / singularValues_inv(i);
		}
		else {
			singularValues_inv(i) = 0;
		}
	}
	Eigen::MatrixXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose();
	Hn.block(0, 0, a + c, a + c) =
		Hn.block(0, 0, a + c, a + c) - Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);
	Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
	Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
	Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

	// Inverse reorder
	// a*  | ac* | 0       a*  | 0 | ac*
	// ca* | c*  | 0  -->  0   | 0 | 0
	// 0   | 0   | 0       ca* | 0 | c*
	Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(), H.cols());
	if (a > 0) {
		res.block(0, 0, a, a) = Hn.block(0, 0, a, a);
		res.block(0, a, a, b) = Hn.block(0, a + c, a, b);
		res.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
	}
	if (a > 0 && c > 0) {
		res.block(0, a + b, a, c) = Hn.block(0, a, a, c);
		res.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
	}
	if (c > 0) {
		res.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
		res.block(a + b, a, c, b) = Hn.block(a, a + c, c, b);
		res.block(a, a + b, b, c) = Hn.block(a + c, a, b, c);
	}

	res.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

	return res;
}

Eigen::MatrixXd Optimizer::Condition(const Eigen::MatrixXd &H, const int &start, const int &end)
{
	// Size of block before block to condition
	const int a = start;
	// Size of block to condition
	const int b = end + 1 - start;

	// Set to zero elements related to block b(start:end,start:end)
	// a  | ab | ac       a  | 0 | ac
	// ba | b  | bc  -->  0  | 0 | 0
	// ca | cb | c        ca | 0 | c

	Eigen::MatrixXd Hn = H;

	Hn.block(a, 0, b, H.cols()) = Eigen::MatrixXd::Zero(b, H.cols());
	Hn.block(0, a, H.rows(), b) = Eigen::MatrixXd::Zero(H.rows(), b);

	return Hn;
}

Eigen::MatrixXd Optimizer::Sparsify(const Eigen::MatrixXd &H,
                                    const int &start1,
                                    const int &end1,
                                    const int &start2,
                                    const int &end2)
{
	// Goal: remove link between a and b
	// p(a,b,c) ~ p(a,b,c)*p(a|c)/p(a|b,c) => H' = H + H1 - H2
	// H1: marginalize b and condition c
	// H2: condition b and c
	Eigen::MatrixXd Hac = Marginalize(H, start2, end2);
	Eigen::MatrixXd Hbc = Marginalize(H, start1, end1);
	Eigen::MatrixXd Hc = Marginalize(Hac, start1, end1);

	return Hac + Hbc - Hc;
}

void Optimizer::InertialOptimization(Map *pMap,
                                     Eigen::Matrix3d &Rwg,
                                     double &scale,
                                     Eigen::Vector3d &bg,
                                     Eigen::Vector3d &ba,
                                     bool bMono,
                                     Eigen::MatrixXd &covInertial,
                                     bool bFixedVel,
                                     bool bGauss,
                                     float priorG,
                                     float priorA)
{
	Verbose::PrintMess("inertial optimization", Verbose::VERBOSITY_NORMAL);
	int its = 200; // Check number of iterations
	long unsigned int maxKFid = pMap->GetMaxKFid();
	const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

	if (priorG != 0.f) {
		solver->setUserLambdaInit(1e3);
	}

	optimizer.setAlgorithm(solver);

	// Set KeyFrame vertices (fixed poses and optimizable velocities)
	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}
		VertexPose *VP = new VertexPose(pKFi);
		VP->setId(pKFi->mnId);
		VP->setFixed(true);
		optimizer.addVertex(VP);

		VertexVelocity *VV = new VertexVelocity(pKFi);
		VV->setId(maxKFid + (pKFi->mnId) + 1);
		if (bFixedVel) {
			VV->setFixed(true);
		}
		else {
			VV->setFixed(false);
		}

		optimizer.addVertex(VV);
	}

	// Biases
	VertexGyroBias *VG = new VertexGyroBias(vpKFs.front());
	VG->setId(maxKFid * 2 + 2);
	if (bFixedVel) {
		VG->setFixed(true);
	}
	else {
		VG->setFixed(false);
	}
	optimizer.addVertex(VG);
	VertexAccBias *VA = new VertexAccBias(vpKFs.front());
	VA->setId(maxKFid * 2 + 3);
	if (bFixedVel) {
		VA->setFixed(true);
	}
	else {
		VA->setFixed(false);
	}

	optimizer.addVertex(VA);
	// prior acc bias
	EdgePriorAcc *epa = new EdgePriorAcc(cv::Mat::zeros(3, 1, CV_32F));
	epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
	double infoPriorA = priorA;
	epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
	optimizer.addEdge(epa);
	EdgePriorGyro *epg = new EdgePriorGyro(cv::Mat::zeros(3, 1, CV_32F));
	epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
	double infoPriorG = priorG;
	epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
	optimizer.addEdge(epg);

	// Gravity and scale
	VertexGDir *VGDir = new VertexGDir(Rwg);
	VGDir->setId(maxKFid * 2 + 4);
	VGDir->setFixed(false);
	optimizer.addVertex(VGDir);
	VertexScale *VS = new VertexScale(scale);
	VS->setId(maxKFid * 2 + 5);
	VS->setFixed(!bMono); // Fixed for stereo case
	optimizer.addVertex(VS);

	// Graph edges
	// IMU links with gravity and scale
	vector<EdgeInertialGS *> vpei;
	vpei.reserve(vpKFs.size());
	vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
	vppUsedKF.reserve(vpKFs.size());
	std::cout << "build optimization graph" << std::endl;

	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];

		if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
			if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) {
				continue;
			}
			if (!pKFi->mpImuPreintegrated) {
				std::cout << "Not preintegrated measurement" << std::endl;
			}

			pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
			g2o::HyperGraph::Vertex *VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
			g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
			g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(pKFi->mnId);
			g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
			g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid * 2 + 2);
			g2o::HyperGraph::Vertex *VA = optimizer.vertex(maxKFid * 2 + 3);
			g2o::HyperGraph::Vertex *VGDir = optimizer.vertex(maxKFid * 2 + 4);
			g2o::HyperGraph::Vertex *VS = optimizer.vertex(maxKFid * 2 + 5);
			if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
				cout << "Error" << VP1 << ", " << VV1 << ", " << VG << ", " << VA << ", " << VP2 << ", " << VV2 << ", "
				     << VGDir << ", " << VS << endl;

				continue;
			}
			EdgeInertialGS *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
			ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
			ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
			ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
			ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
			ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
			ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
			ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VGDir));
			ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VS));

			vpei.push_back(ei);

			vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
			optimizer.addEdge(ei);
		}
	}

	// Compute error for different scales
	std::set<g2o::HyperGraph::Edge *> setEdges = optimizer.edges();

	std::cout << "start optimization" << std::endl;
	optimizer.setVerbose(false);
	optimizer.initializeOptimization();
	optimizer.optimize(its);

	std::cout << "end optimization" << std::endl;

	scale = VS->estimate();

	// Recover optimized data
	// Biases
	VG = static_cast<VertexGyroBias *>(optimizer.vertex(maxKFid * 2 + 2));
	VA = static_cast<VertexAccBias *>(optimizer.vertex(maxKFid * 2 + 3));
	Vector6d vb;
	vb << VG->estimate(), VA->estimate();
	bg << VG->estimate();
	ba << VA->estimate();
	scale = VS->estimate();

	IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
	Rwg = VGDir->estimate().Rwg;

	cv::Mat cvbg = Converter::toCvMat(bg);

	//Keyframes velocities and biases
	std::cout << "update Keyframes velocities and biases" << std::endl;

	const int N = vpKFs.size();
	for (size_t i = 0; i < N; i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}

		VertexVelocity *VV = static_cast<VertexVelocity *>(optimizer.vertex(maxKFid + (pKFi->mnId) + 1));
		Eigen::Vector3d Vw = VV->estimate(); // Velocity is scaled after
		pKFi->SetVelocity(Converter::toCvMat(Vw));

		if (cv::norm(pKFi->GetGyroBias() - cvbg) > 0.01) {
			pKFi->SetNewBias(b);
			if (pKFi->mpImuPreintegrated) {
				pKFi->mpImuPreintegrated->Reintegrate();
			}
		}
		else {
			pKFi->SetNewBias(b);
		}
	}
}

void Optimizer::InertialOptimization(Map *pMap, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG, float priorA)
{
	int its = 200; // Check number of iterations
	long unsigned int maxKFid = pMap->GetMaxKFid();
	const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	solver->setUserLambdaInit(1e3);

	optimizer.setAlgorithm(solver);

	// Set KeyFrame vertices (fixed poses and optimizable velocities)
	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}
		VertexPose *VP = new VertexPose(pKFi);
		VP->setId(pKFi->mnId);
		VP->setFixed(true);
		optimizer.addVertex(VP);

		VertexVelocity *VV = new VertexVelocity(pKFi);
		VV->setId(maxKFid + (pKFi->mnId) + 1);
		VV->setFixed(false);

		optimizer.addVertex(VV);
	}

	// Biases
	VertexGyroBias *VG = new VertexGyroBias(vpKFs.front());
	VG->setId(maxKFid * 2 + 2);
	VG->setFixed(false);
	optimizer.addVertex(VG);

	VertexAccBias *VA = new VertexAccBias(vpKFs.front());
	VA->setId(maxKFid * 2 + 3);
	VA->setFixed(false);

	optimizer.addVertex(VA);
	// prior acc bias
	EdgePriorAcc *epa = new EdgePriorAcc(cv::Mat::zeros(3, 1, CV_32F));
	epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
	double infoPriorA = priorA;
	epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
	optimizer.addEdge(epa);
	EdgePriorGyro *epg = new EdgePriorGyro(cv::Mat::zeros(3, 1, CV_32F));
	epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
	double infoPriorG = priorG;
	epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
	optimizer.addEdge(epg);

	// Gravity and scale
	VertexGDir *VGDir = new VertexGDir(Eigen::Matrix3d::Identity());
	VGDir->setId(maxKFid * 2 + 4);
	VGDir->setFixed(true);
	optimizer.addVertex(VGDir);
	VertexScale *VS = new VertexScale(1.0);
	VS->setId(maxKFid * 2 + 5);
	VS->setFixed(true); // Fixed since scale is obtained from already well initialized map
	optimizer.addVertex(VS);

	// Graph edges
	// IMU links with gravity and scale
	vector<EdgeInertialGS *> vpei;
	vpei.reserve(vpKFs.size());
	vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
	vppUsedKF.reserve(vpKFs.size());

	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];

		if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
			if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) {
				continue;
			}

			pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
			g2o::HyperGraph::Vertex *VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
			g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
			g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(pKFi->mnId);
			g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
			g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid * 2 + 2);
			g2o::HyperGraph::Vertex *VA = optimizer.vertex(maxKFid * 2 + 3);
			g2o::HyperGraph::Vertex *VGDir = optimizer.vertex(maxKFid * 2 + 4);
			g2o::HyperGraph::Vertex *VS = optimizer.vertex(maxKFid * 2 + 5);
			if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
				cout << "Error" << VP1 << ", " << VV1 << ", " << VG << ", " << VA << ", " << VP2 << ", " << VV2 << ", "
				     << VGDir << ", " << VS << endl;

				continue;
			}
			EdgeInertialGS *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
			ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
			ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
			ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
			ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
			ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
			ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
			ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VGDir));
			ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VS));

			vpei.push_back(ei);

			vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
			optimizer.addEdge(ei);
		}
	}

	// Compute error for different scales
	optimizer.setVerbose(false);
	optimizer.initializeOptimization();
	optimizer.optimize(its);

	// Recover optimized data
	// Biases
	VG = static_cast<VertexGyroBias *>(optimizer.vertex(maxKFid * 2 + 2));
	VA = static_cast<VertexAccBias *>(optimizer.vertex(maxKFid * 2 + 3));
	Vector6d vb;
	vb << VG->estimate(), VA->estimate();
	bg << VG->estimate();
	ba << VA->estimate();

	IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);

	cv::Mat cvbg = Converter::toCvMat(bg);

	//Keyframes velocities and biases
	const int N = vpKFs.size();
	for (size_t i = 0; i < N; i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}

		VertexVelocity *VV = static_cast<VertexVelocity *>(optimizer.vertex(maxKFid + (pKFi->mnId) + 1));
		Eigen::Vector3d Vw = VV->estimate();
		pKFi->SetVelocity(Converter::toCvMat(Vw));

		if (cv::norm(pKFi->GetGyroBias() - cvbg) > 0.01) {
			pKFi->SetNewBias(b);
			if (pKFi->mpImuPreintegrated) {
				pKFi->mpImuPreintegrated->Reintegrate();
			}
		}
		else {
			pKFi->SetNewBias(b);
		}
	}
}

void Optimizer::InertialOptimization(vector<KeyFrame *> vpKFs,
                                     Eigen::Vector3d &bg,
                                     Eigen::Vector3d &ba,
                                     float priorG,
                                     float priorA)
{
	int its = 200; // Check number of iterations
	long unsigned int maxKFid = vpKFs[0]->GetMap()->GetMaxKFid();

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	solver->setUserLambdaInit(1e3);

	optimizer.setAlgorithm(solver);

	// Set KeyFrame vertices (fixed poses and optimizable velocities)
	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];
		//if(pKFi->mnId>maxKFid)
		//    continue;
		VertexPose *VP = new VertexPose(pKFi);
		VP->setId(pKFi->mnId);
		VP->setFixed(true);
		optimizer.addVertex(VP);

		VertexVelocity *VV = new VertexVelocity(pKFi);
		VV->setId(maxKFid + (pKFi->mnId) + 1);
		VV->setFixed(false);

		optimizer.addVertex(VV);
	}

	// Biases
	VertexGyroBias *VG = new VertexGyroBias(vpKFs.front());
	VG->setId(maxKFid * 2 + 2);
	VG->setFixed(false);
	optimizer.addVertex(VG);

	VertexAccBias *VA = new VertexAccBias(vpKFs.front());
	VA->setId(maxKFid * 2 + 3);
	VA->setFixed(false);

	optimizer.addVertex(VA);
	// prior acc bias
	EdgePriorAcc *epa = new EdgePriorAcc(cv::Mat::zeros(3, 1, CV_32F));
	epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
	double infoPriorA = priorA;
	epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
	optimizer.addEdge(epa);
	EdgePriorGyro *epg = new EdgePriorGyro(cv::Mat::zeros(3, 1, CV_32F));
	epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
	double infoPriorG = priorG;
	epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
	optimizer.addEdge(epg);

	// Gravity and scale
	VertexGDir *VGDir = new VertexGDir(Eigen::Matrix3d::Identity());
	VGDir->setId(maxKFid * 2 + 4);
	VGDir->setFixed(true);
	optimizer.addVertex(VGDir);
	VertexScale *VS = new VertexScale(1.0);
	VS->setId(maxKFid * 2 + 5);
	VS->setFixed(true); // Fixed since scale is obtained from already well initialized map
	optimizer.addVertex(VS);

	// Graph edges
	// IMU links with gravity and scale
	vector<EdgeInertialGS *> vpei;
	vpei.reserve(vpKFs.size());
	vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
	vppUsedKF.reserve(vpKFs.size());

	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];

		if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
			if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) {
				continue;
			}

			pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
			g2o::HyperGraph::Vertex *VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
			g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
			g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(pKFi->mnId);
			g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
			g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid * 2 + 2);
			g2o::HyperGraph::Vertex *VA = optimizer.vertex(maxKFid * 2 + 3);
			g2o::HyperGraph::Vertex *VGDir = optimizer.vertex(maxKFid * 2 + 4);
			g2o::HyperGraph::Vertex *VS = optimizer.vertex(maxKFid * 2 + 5);
			if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
				cout << "Error" << VP1 << ", " << VV1 << ", " << VG << ", " << VA << ", " << VP2 << ", " << VV2 << ", "
				     << VGDir << ", " << VS << endl;

				continue;
			}
			EdgeInertialGS *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
			ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
			ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
			ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
			ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
			ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
			ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
			ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VGDir));
			ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VS));

			vpei.push_back(ei);

			vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
			optimizer.addEdge(ei);
		}
	}

	// Compute error for different scales
	optimizer.setVerbose(false);
	optimizer.initializeOptimization();
	optimizer.optimize(its);

	// Recover optimized data
	// Biases
	VG = static_cast<VertexGyroBias *>(optimizer.vertex(maxKFid * 2 + 2));
	VA = static_cast<VertexAccBias *>(optimizer.vertex(maxKFid * 2 + 3));
	Vector6d vb;
	vb << VG->estimate(), VA->estimate();
	bg << VG->estimate();
	ba << VA->estimate();

	IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);

	cv::Mat cvbg = Converter::toCvMat(bg);

	//Keyframes velocities and biases
	const int N = vpKFs.size();
	for (size_t i = 0; i < N; i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}

		VertexVelocity *VV = static_cast<VertexVelocity *>(optimizer.vertex(maxKFid + (pKFi->mnId) + 1));
		Eigen::Vector3d Vw = VV->estimate();
		pKFi->SetVelocity(Converter::toCvMat(Vw));

		if (cv::norm(pKFi->GetGyroBias() - cvbg) > 0.01) {
			pKFi->SetNewBias(b);
			if (pKFi->mpImuPreintegrated) {
				pKFi->mpImuPreintegrated->Reintegrate();
			}
		}
		else {
			pKFi->SetNewBias(b);
		}
	}
}

} // namespace ORB_SLAM3
