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

void Optimizer::InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale)
{
	int its = 10;
	long unsigned int maxKFid = pMap->GetMaxKFid();
	const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
	optimizer.setAlgorithm(solver);

	// Set KeyFrame vertices (all variables are fixed)
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
		VV->setId(maxKFid + 1 + (pKFi->mnId));
		VV->setFixed(true);
		optimizer.addVertex(VV);

		// Vertex of fixed biases
		VertexGyroBias *VG = new VertexGyroBias(vpKFs.front());
		VG->setId(2 * (maxKFid + 1) + (pKFi->mnId));
		VG->setFixed(true);
		optimizer.addVertex(VG);
		VertexAccBias *VA = new VertexAccBias(vpKFs.front());
		VA->setId(3 * (maxKFid + 1) + (pKFi->mnId));
		VA->setFixed(true);
		optimizer.addVertex(VA);
	}

	// Gravity and scale
	VertexGDir *VGDir = new VertexGDir(Rwg);
	VGDir->setId(4 * (maxKFid + 1));
	VGDir->setFixed(false);
	optimizer.addVertex(VGDir);
	VertexScale *VS = new VertexScale(scale);
	VS->setId(4 * (maxKFid + 1) + 1);
	VS->setFixed(false);
	optimizer.addVertex(VS);

	// Graph edges
	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];

		if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
			if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) {
				continue;
			}

			g2o::HyperGraph::Vertex *VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
			g2o::HyperGraph::Vertex *VV1 = optimizer.vertex((maxKFid + 1) + pKFi->mPrevKF->mnId);
			g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(pKFi->mnId);
			g2o::HyperGraph::Vertex *VV2 = optimizer.vertex((maxKFid + 1) + pKFi->mnId);
			g2o::HyperGraph::Vertex *VG = optimizer.vertex(2 * (maxKFid + 1) + pKFi->mPrevKF->mnId);
			g2o::HyperGraph::Vertex *VA = optimizer.vertex(3 * (maxKFid + 1) + pKFi->mPrevKF->mnId);
			g2o::HyperGraph::Vertex *VGDir = optimizer.vertex(4 * (maxKFid + 1));
			g2o::HyperGraph::Vertex *VS = optimizer.vertex(4 * (maxKFid + 1) + 1);
			if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
				Verbose::PrintMess(
					"Error" + to_string(VP1->id()) + ", " + to_string(VV1->id()) + ", " + to_string(VG->id()) + ", "
						+ to_string(VA->id()) + ", " + to_string(VP2->id()) + ", " + to_string(VV2->id()) + ", "
						+ to_string(VGDir->id()) + ", " + to_string(VS->id()), Verbose::VERBOSITY_NORMAL);

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

			optimizer.addEdge(ei);
		}
	}

	// Compute error for different scales
	optimizer.setVerbose(false);
	optimizer.initializeOptimization();
	optimizer.optimize(its);

	// Recover optimized data
	scale = VS->estimate();
	Rwg = VGDir->estimate().Rwg;
}

void Optimizer::MergeBundleAdjustmentVisual(KeyFrame *pCurrentKF,
                                            vector<KeyFrame *> vpWeldingKFs,
                                            vector<KeyFrame *> vpFixedKFs,
                                            bool *pbStopFlag)
{
	vector<MapPoint *> vpMPs;

	g2o::SparseOptimizer optimizer;
	g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

	g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);

	if (pbStopFlag) {
		optimizer.setForceStopFlag(pbStopFlag);
	}

	long unsigned int maxKFid = 0;
	set<KeyFrame *> spKeyFrameBA;

	// Set not fixed KeyFrame vertices
	for (KeyFrame *pKFi: vpWeldingKFs) {
		if (pKFi->isBad()) {
			continue;
		}

		pKFi->mnBALocalForKF = pCurrentKF->mnId;

		g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
		vSE3->setId(pKFi->mnId);
		vSE3->setFixed(false);
		optimizer.addVertex(vSE3);
		if (pKFi->mnId > maxKFid) {
			maxKFid = pKFi->mnId;
		}

		set<MapPoint *> spViewMPs = pKFi->GetMapPoints();
		for (MapPoint *pMPi: spViewMPs) {
			if (pMPi) {
				if (!pMPi->isBad()) {
					if (pMPi->mnBALocalForKF != pCurrentKF->mnId) {
						vpMPs.push_back(pMPi);
						pMPi->mnBALocalForKF = pCurrentKF->mnId;
					}
				}
			}
		}

		spKeyFrameBA.insert(pKFi);
	}

	// Set fixed KeyFrame vertices
	for (KeyFrame *pKFi: vpFixedKFs) {
		if (pKFi->isBad()) {
			continue;
		}

		pKFi->mnBALocalForKF = pCurrentKF->mnId;

		g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
		vSE3->setId(pKFi->mnId);
		vSE3->setFixed(true);
		optimizer.addVertex(vSE3);
		if (pKFi->mnId > maxKFid) {
			maxKFid = pKFi->mnId;
		}

		set<MapPoint *> spViewMPs = pKFi->GetMapPoints();
		for (MapPoint *pMPi: spViewMPs) {
			if (pMPi) {
				if (!pMPi->isBad()) {
					if (pMPi->mnBALocalForKF != pCurrentKF->mnId) {
						vpMPs.push_back(pMPi);
						pMPi->mnBALocalForKF = pCurrentKF->mnId;
					}
				}
			}
		}

		spKeyFrameBA.insert(pKFi);
	}

	const int nExpectedSize = (vpWeldingKFs.size() + vpFixedKFs.size()) * vpMPs.size();

	vector<g2o::EdgeSE3ProjectXYZ *> vpEdgesMono;
	vpEdgesMono.reserve(nExpectedSize);

	vector<KeyFrame *> vpEdgeKFMono;
	vpEdgeKFMono.reserve(nExpectedSize);

	vector<MapPoint *> vpMapPointEdgeMono;
	vpMapPointEdgeMono.reserve(nExpectedSize);

	vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
	vpEdgesStereo.reserve(nExpectedSize);

	vector<KeyFrame *> vpEdgeKFStereo;
	vpEdgeKFStereo.reserve(nExpectedSize);

	vector<MapPoint *> vpMapPointEdgeStereo;
	vpMapPointEdgeStereo.reserve(nExpectedSize);

	const float thHuber2D = sqrt(5.99);
	const float thHuber3D = sqrt(7.815);

	// Set MapPoint vertices
	for (unsigned int i = 0; i < vpMPs.size(); ++i) {
		MapPoint *pMPi = vpMPs[i];
		if (pMPi->isBad()) {
			continue;
		}

		g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Converter::toVector3d(pMPi->GetWorldPos()));
		const int id = pMPi->mnId + maxKFid + 1;
		vPoint->setId(id);
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);

		const map<KeyFrame *, tuple<int, int>> observations = pMPi->GetObservations();
		int nEdges = 0;
		//SET EDGES
		for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(); mit != observations.end();
		     mit++) {
			//cout << "--KF view init" << endl;

			KeyFrame *pKF = mit->first;
			if (spKeyFrameBA.find(pKF) == spKeyFrameBA.end() || pKF->isBad() || pKF->mnId > maxKFid
				|| pKF->mnBALocalForKF != pCurrentKF->mnId || !pKF->GetMapPoint(get<0>(mit->second))) {
				continue;
			}

			//cout << "-- KF view exists" << endl;
			nEdges++;

			const cv::KeyPoint &kpUn = pKF->mvKeysUn[get<0>(mit->second)];
			//cout << "-- KeyPoint loads" << endl;

			if (pKF->mvuRight[get<0>(mit->second)] < 0) //Monocular
			{
				Eigen::Matrix<double, 2, 1> obs;
				obs << kpUn.pt.x, kpUn.pt.y;

				g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
				e->setMeasurement(obs);
				const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
				e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);
				//cout << "-- Sigma loads" << endl;

				g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
				e->setRobustKernel(rk);
				rk->setDelta(thHuber2D);

				e->fx = pKF->fx;
				e->fy = pKF->fy;
				e->cx = pKF->cx;
				e->cy = pKF->cy;
				//cout << "-- Calibration loads" << endl;

				optimizer.addEdge(e);
				//cout << "-- Edge added" << endl;

				vpEdgesMono.push_back(e);
				vpEdgeKFMono.push_back(pKF);
				vpMapPointEdgeMono.push_back(pMPi);
				//cout << "-- Added to vector" << endl;
			}
			else // RGBD or Stereo
			{
				Eigen::Matrix<double, 3, 1> obs;
				const float kp_ur = pKF->mvuRight[get<0>(mit->second)];
				obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

				g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
				e->setMeasurement(obs);
				const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
				Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
				e->setInformation(Info);

				g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
				e->setRobustKernel(rk);
				rk->setDelta(thHuber3D);

				e->fx = pKF->fx;
				e->fy = pKF->fy;
				e->cx = pKF->cx;
				e->cy = pKF->cy;
				e->bf = pKF->mbf;

				optimizer.addEdge(e);

				vpEdgesStereo.push_back(e);
				vpEdgeKFStereo.push_back(pKF);
				vpMapPointEdgeStereo.push_back(pMPi);
			}
			//cout << "-- End to load point" << endl;
		}
	}

	//cout << "End to load MPs" << endl;

	if (pbStopFlag) {
		if (*pbStopFlag) {
			return;
		}
	}

	optimizer.initializeOptimization();
	optimizer.optimize(5);

	//cout << "End the first optimization" << endl;

	bool bDoMore = true;

	if (pbStopFlag) {
		if (*pbStopFlag) {
			bDoMore = false;
		}
	}

	if (bDoMore) {

		// Check inlier observations
		for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
			g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
			MapPoint *pMP = vpMapPointEdgeMono[i];

			if (pMP->isBad()) {
				continue;
			}

			if (e->chi2() > 5.991 || !e->isDepthPositive()) {
				e->setLevel(1);
			}

			e->setRobustKernel(0);
		}

		for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
			g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
			MapPoint *pMP = vpMapPointEdgeStereo[i];

			if (pMP->isBad()) {
				continue;
			}

			if (e->chi2() > 7.815 || !e->isDepthPositive()) {
				e->setLevel(1);
			}

			e->setRobustKernel(0);
		}

		// Optimize again without the outliers

		optimizer.initializeOptimization(0);
		optimizer.optimize(10);

		//cout << "End the second optimization (without outliers)" << endl;
	}

	vector<pair<KeyFrame *, MapPoint *>> vToErase;
	vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

	// Check inlier observations
	for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
		g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
		MapPoint *pMP = vpMapPointEdgeMono[i];

		if (pMP->isBad()) {
			continue;
		}

		if (e->chi2() > 5.991 || !e->isDepthPositive()) {
			KeyFrame *pKFi = vpEdgeKFMono[i];
			vToErase.push_back(make_pair(pKFi, pMP));
		}
	}

	for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
		g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
		MapPoint *pMP = vpMapPointEdgeStereo[i];

		if (pMP->isBad()) {
			continue;
		}

		if (e->chi2() > 7.815 || !e->isDepthPositive()) {
			KeyFrame *pKFi = vpEdgeKFStereo[i];
			vToErase.push_back(make_pair(pKFi, pMP));
		}
	}

	// Get Map Mutex
	unique_lock<shared_timed_mutex> lock(pCurrentKF->GetMap()->mMutexMapUpdate);

	if (!vToErase.empty()) {
		for (size_t i = 0; i < vToErase.size(); i++) {
			KeyFrame *pKFi = vToErase[i].first;
			MapPoint *pMPi = vToErase[i].second;
			pKFi->EraseMapPointMatch(pMPi);
			pMPi->EraseObservation(pKFi);
		}
	}
	//cout << "End to erase observations" << endl;

	// Recover optimized data

	//Keyframes
	for (KeyFrame *pKFi: vpWeldingKFs) {
		if (pKFi->isBad()) {
			continue;
		}

		g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKFi->mnId));
		g2o::SE3Quat SE3quat = vSE3->estimate();
		pKFi->SetPose(Converter::toCvMat(SE3quat));
	}
	//cout << "End to update the KeyFrames" << endl;

	//Points
	for (MapPoint *pMPi: vpMPs) {
		if (pMPi->isBad()) {
			continue;
		}

		g2o::VertexSBAPointXYZ
			*vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMPi->mnId + maxKFid + 1));
		pMPi->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMPi->UpdateNormalAndDepth();
	}
}

void Optimizer::LocalBundleAdjustment(KeyFrame *pMainKF,
                                      vector<KeyFrame *> vpAdjustKF,
                                      vector<KeyFrame *> vpFixedKF,
                                      bool *pbStopFlag)
{
	bool bShowImages = false;

	vector<MapPoint *> vpMPs;

	g2o::SparseOptimizer optimizer;
	g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

	g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);

	optimizer.setVerbose(false);

	if (pbStopFlag) {
		optimizer.setForceStopFlag(pbStopFlag);
	}

	long unsigned int maxKFid = 0;
	set<KeyFrame *> spKeyFrameBA;

	Map *pCurrentMap = pMainKF->GetMap();

	//set<MapPoint*> sNumObsMP;

	// Set fixed KeyFrame vertices
	for (KeyFrame *pKFi: vpFixedKF) {
		if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap) {
			Verbose::PrintMess("ERROR LBA: KF is bad or is not in the current map", Verbose::VERBOSITY_NORMAL);
			continue;
		}

		pKFi->mnBALocalForMerge = pMainKF->mnId;

		g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
		vSE3->setId(pKFi->mnId);
		vSE3->setFixed(true);
		optimizer.addVertex(vSE3);
		if (pKFi->mnId > maxKFid) {
			maxKFid = pKFi->mnId;
		}

		set<MapPoint *> spViewMPs = pKFi->GetMapPoints();
		for (MapPoint *pMPi: spViewMPs) {
			if (pMPi) {
				if (!pMPi->isBad() && pMPi->GetMap() == pCurrentMap) {

					if (pMPi->mnBALocalForMerge != pMainKF->mnId) {
						vpMPs.push_back(pMPi);
						pMPi->mnBALocalForMerge = pMainKF->mnId;
					}
				}
			}
			/*if(sNumObsMP.find(pMPi) == sNumObsMP.end())
                    {
                        sNumObsMP.insert(pMPi);
                    }
                    else
                    {
                        if(pMPi->mnBALocalForMerge!=pMainKF->mnId)
                        {
                            vpMPs.push_back(pMPi);
                            pMPi->mnBALocalForMerge=pMainKF->mnId;
                        }
                    }*/
		}

		spKeyFrameBA.insert(pKFi);
	}

	//cout << "End to load Fixed KFs" << endl;

	// Set non fixed Keyframe vertices
	set<KeyFrame *> spAdjustKF(vpAdjustKF.begin(), vpAdjustKF.end());
	for (KeyFrame *pKFi: vpAdjustKF) {
		if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap) {
			continue;
		}

		pKFi->mnBALocalForKF = pMainKF->mnId;

		g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
		vSE3->setId(pKFi->mnId);
		optimizer.addVertex(vSE3);
		if (pKFi->mnId > maxKFid) {
			maxKFid = pKFi->mnId;
		}

		set<MapPoint *> spViewMPs = pKFi->GetMapPoints();
		for (MapPoint *pMPi: spViewMPs) {
			if (pMPi) {
				if (!pMPi->isBad() && pMPi->GetMap() == pCurrentMap) {
					/*if(sNumObsMP.find(pMPi) == sNumObsMP.end())
                    {
                        sNumObsMP.insert(pMPi);
                    }*/
					if (pMPi->mnBALocalForMerge != pMainKF->mnId) {
						vpMPs.push_back(pMPi);
						pMPi->mnBALocalForMerge = pMainKF->mnId;
					}
				}
			}
		}

		spKeyFrameBA.insert(pKFi);
	}

	//Verbose::PrintMess("LBA: There are " + to_string(vpMPs.size()) + " MPs to optimize", Verbose::VERBOSITY_NORMAL);

	//cout << "End to load KFs for position adjust" << endl;

	const int nExpectedSize = (vpAdjustKF.size() + vpFixedKF.size()) * vpMPs.size();

	vector<ORB_SLAM3::EdgeSE3ProjectXYZ *> vpEdgesMono;
	vpEdgesMono.reserve(nExpectedSize);

	vector<KeyFrame *> vpEdgeKFMono;
	vpEdgeKFMono.reserve(nExpectedSize);

	vector<MapPoint *> vpMapPointEdgeMono;
	vpMapPointEdgeMono.reserve(nExpectedSize);

	vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
	vpEdgesStereo.reserve(nExpectedSize);

	vector<KeyFrame *> vpEdgeKFStereo;
	vpEdgeKFStereo.reserve(nExpectedSize);

	vector<MapPoint *> vpMapPointEdgeStereo;
	vpMapPointEdgeStereo.reserve(nExpectedSize);

	const float thHuber2D = sqrt(5.99);
	const float thHuber3D = sqrt(7.815);

	// Set MapPoint vertices
	map<KeyFrame *, int> mpObsKFs;
	map<KeyFrame *, int> mpObsFinalKFs;
	map<MapPoint *, int> mpObsMPs;
	for (unsigned int i = 0; i < vpMPs.size(); ++i) {
		MapPoint *pMPi = vpMPs[i];
		if (pMPi->isBad()) {
			continue;
		}

		g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Converter::toVector3d(pMPi->GetWorldPos()));
		const int id = pMPi->mnId + maxKFid + 1;
		vPoint->setId(id);
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);

		const map<KeyFrame *, tuple<int, int>> observations = pMPi->GetObservations();
		int nEdges = 0;
		//SET EDGES
		for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(); mit != observations.end();
		     mit++) {
			//cout << "--KF view init" << endl;

			KeyFrame *pKF = mit->first;
			if (pKF->isBad() || pKF->mnId > maxKFid || pKF->mnBALocalForMerge != pMainKF->mnId
				|| !pKF->GetMapPoint(get<0>(mit->second))) {
				continue;
			}

			//cout << "-- KF view exists" << endl;
			nEdges++;

			const cv::KeyPoint &kpUn = pKF->mvKeysUn[get<0>(mit->second)];
			//cout << "-- KeyPoint loads" << endl;

			if (pKF->mvuRight[get<0>(mit->second)] < 0) //Monocular
			{
				mpObsMPs[pMPi]++;
				Eigen::Matrix<double, 2, 1> obs;
				obs << kpUn.pt.x, kpUn.pt.y;

				ORB_SLAM3::EdgeSE3ProjectXYZ *e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
				e->setMeasurement(obs);
				const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
				e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);
				//cout << "-- Sigma loads" << endl;

				g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
				e->setRobustKernel(rk);
				rk->setDelta(thHuber2D);

				e->pCamera = pKF->mpCamera;
				//cout << "-- Calibration loads" << endl;

				optimizer.addEdge(e);
				//cout << "-- Edge added" << endl;

				vpEdgesMono.push_back(e);
				vpEdgeKFMono.push_back(pKF);
				vpMapPointEdgeMono.push_back(pMPi);
				//cout << "-- Added to vector" << endl;

				mpObsKFs[pKF]++;
			}
			else // RGBD or Stereo
			{
				mpObsMPs[pMPi] += 2;
				Eigen::Matrix<double, 3, 1> obs;
				const float kp_ur = pKF->mvuRight[get<0>(mit->second)];
				obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

				g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
				e->setMeasurement(obs);
				const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
				Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
				e->setInformation(Info);

				g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
				e->setRobustKernel(rk);
				rk->setDelta(thHuber3D);

				e->fx = pKF->fx;
				e->fy = pKF->fy;
				e->cx = pKF->cx;
				e->cy = pKF->cy;
				e->bf = pKF->mbf;

				optimizer.addEdge(e);

				vpEdgesStereo.push_back(e);
				vpEdgeKFStereo.push_back(pKF);
				vpMapPointEdgeStereo.push_back(pMPi);

				mpObsKFs[pKF]++;
			}
			//cout << "-- End to load point" << endl;
		}
	}
	//Verbose::PrintMess("LBA: number total of edged -> " + to_string(vpEdgeKFMono.size() + vpEdgeKFStereo.size()), Verbose::VERBOSITY_NORMAL);

	map<int, int> mStatsObs;
	for (map<MapPoint *, int>::iterator it = mpObsMPs.begin(); it != mpObsMPs.end(); ++it) {
		MapPoint *pMPi = it->first;
		int numObs = it->second;

		mStatsObs[numObs]++;
		/*if(numObs < 5)
        {
            cout << "LBA: MP " << pMPi->mnId << " has " << numObs << " observations" << endl;
        }*/
	}

	/*for(map<int, int>::iterator it = mStatsObs.begin(); it != mStatsObs.end(); ++it)
    {
        cout << "LBA: There are " << it->second << " MPs with " << it->first << " observations" << endl;
    }*/

	//cout << "End to load MPs" << endl;

	if (pbStopFlag) {
		if (*pbStopFlag) {
			return;
		}
	}

	optimizer.save("/home/da/project/ros/orb_dvl2_ws/src/dvl2/orb3_result/merge_BA.g2o");
	optimizer.initializeOptimization();
	optimizer.optimize(5);

	//cout << "End the first optimization" << endl;

	bool bDoMore = true;

	if (pbStopFlag) {
		if (*pbStopFlag) {
			bDoMore = false;
		}
	}

	map<unsigned long int, int> mWrongObsKF;
	if (bDoMore) {

		// Check inlier observations
		int badMonoMP = 0, badStereoMP = 0;
		for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
			ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
			MapPoint *pMP = vpMapPointEdgeMono[i];

			if (pMP->isBad()) {
				continue;
			}

			if (e->chi2() > 5.991 || !e->isDepthPositive()) {
				e->setLevel(1);
				badMonoMP++;
			}

			e->setRobustKernel(0);
		}

		for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
			g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
			MapPoint *pMP = vpMapPointEdgeStereo[i];

			if (pMP->isBad()) {
				continue;
			}

			if (e->chi2() > 7.815 || !e->isDepthPositive()) {
				e->setLevel(1);
				badStereoMP++;
			}

			e->setRobustKernel(0);
		}
		Verbose::PrintMess(
			"LBA: First optimization, there are " + to_string(badMonoMP) + " monocular and " + to_string(badStereoMP)
				+ " sterero bad edges", Verbose::VERBOSITY_DEBUG);

		// Optimize again without the outliers

		optimizer.initializeOptimization(0);
		optimizer.optimize(10);

		//cout << "End the second optimization (without outliers)" << endl;
	}

	vector<pair<KeyFrame *, MapPoint *>> vToErase;
	vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());
	set<MapPoint *> spErasedMPs;
	set<KeyFrame *> spErasedKFs;

	// Check inlier observations
	int badMonoMP = 0, badStereoMP = 0;
	for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
		ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
		MapPoint *pMP = vpMapPointEdgeMono[i];

		if (pMP->isBad()) {
			continue;
		}

		if (e->chi2() > 5.991 || !e->isDepthPositive()) {
			KeyFrame *pKFi = vpEdgeKFMono[i];
			vToErase.push_back(make_pair(pKFi, pMP));
			mWrongObsKF[pKFi->mnId]++;
			badMonoMP++;

			spErasedMPs.insert(pMP);
			spErasedKFs.insert(pKFi);
		}
	}

	for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
		g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
		MapPoint *pMP = vpMapPointEdgeStereo[i];

		if (pMP->isBad()) {
			continue;
		}

		if (e->chi2() > 7.815 || !e->isDepthPositive()) {
			KeyFrame *pKFi = vpEdgeKFStereo[i];
			vToErase.push_back(make_pair(pKFi, pMP));
			mWrongObsKF[pKFi->mnId]++;
			badStereoMP++;

			spErasedMPs.insert(pMP);
			spErasedKFs.insert(pKFi);
		}
	}
	Verbose::PrintMess(
		"LBA: Second optimization, there are " + to_string(badMonoMP) + " monocular and " + to_string(badStereoMP)
			+ " sterero bad edges", Verbose::VERBOSITY_DEBUG);

	// Get Map Mutex
	unique_lock<shared_timed_mutex> lock(pMainKF->GetMap()->mMutexMapUpdate, std::defer_lock);
	if (!lock.try_lock()) {
		return;
	}

	if (!vToErase.empty()) {
		map<KeyFrame *, int> mpMPs_in_KF;
		for (KeyFrame *pKFi: spErasedKFs) {
			int num_MPs = pKFi->GetMapPoints().size();
			mpMPs_in_KF[pKFi] = num_MPs;
		}

		Verbose::PrintMess(
			"LBA: There are " + to_string(vToErase.size()) + " observations whose will be deleted from the map",
			Verbose::VERBOSITY_DEBUG);
		for (size_t i = 0; i < vToErase.size(); i++) {
			KeyFrame *pKFi = vToErase[i].first;
			MapPoint *pMPi = vToErase[i].second;
			pKFi->EraseMapPointMatch(pMPi);
			pMPi->EraseObservation(pKFi);
		}

		Verbose::PrintMess("LBA: " + to_string(spErasedMPs.size()) + " MPs had deleted observations",
		                   Verbose::VERBOSITY_DEBUG);
		Verbose::PrintMess("LBA: Current map is " + to_string(pMainKF->GetMap()->GetId()), Verbose::VERBOSITY_DEBUG);
		int numErasedMP = 0;
		for (MapPoint *pMPi: spErasedMPs) {
			if (pMPi->isBad()) {
				Verbose::PrintMess(
					"LBA: MP " + to_string(pMPi->mnId) + " has lost almost all the observations, its origin map is "
						+ to_string(pMPi->mnOriginMapId), Verbose::VERBOSITY_DEBUG);
				numErasedMP++;
			}
		}
		Verbose::PrintMess("LBA: " + to_string(numErasedMP) + " MPs had deleted from the map",
		                   Verbose::VERBOSITY_DEBUG);

		for (KeyFrame *pKFi: spErasedKFs) {
			int num_MPs = pKFi->GetMapPoints().size();
			int num_init_MPs = mpMPs_in_KF[pKFi];
			Verbose::PrintMess(
				"LBA: Initially KF " + to_string(pKFi->mnId) + " had " + to_string(num_init_MPs) + ", at the end has "
					+ to_string(num_MPs), Verbose::VERBOSITY_DEBUG);
		}
	}
	for (unsigned int i = 0; i < vpMPs.size(); ++i) {
		MapPoint *pMPi = vpMPs[i];
		if (pMPi->isBad()) {
			continue;
		}

		const map<KeyFrame *, tuple<int, int>> observations = pMPi->GetObservations();
		for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(); mit != observations.end();
		     mit++) {
			//cout << "--KF view init" << endl;

			KeyFrame *pKF = mit->first;
			if (pKF->isBad() || pKF->mnId > maxKFid || pKF->mnBALocalForKF != pMainKF->mnId
				|| !pKF->GetMapPoint(get<0>(mit->second))) {
				continue;
			}

			const cv::KeyPoint &kpUn = pKF->mvKeysUn[get<0>(mit->second)];
			//cout << "-- KeyPoint loads" << endl;

			if (pKF->mvuRight[get<0>(mit->second)] < 0) //Monocular
			{
				mpObsFinalKFs[pKF]++;
			}
			else // RGBD or Stereo
			{

				mpObsFinalKFs[pKF]++;
			}
			//cout << "-- End to load point" << endl;
		}
	}

	//cout << "End to erase observations" << endl;

	// Recover optimized data

	//Keyframes
	for (KeyFrame *pKFi: vpAdjustKF) {
		if (pKFi->isBad()) {
			continue;
		}

		g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKFi->mnId));
		g2o::SE3Quat SE3quat = vSE3->estimate();
		cv::Mat Tiw = Converter::toCvMat(SE3quat);
		cv::Mat Tco_cn = pKFi->GetPose() * Tiw.inv();
		cv::Vec3d trasl = Tco_cn.rowRange(0, 3).col(3);
		double dist = cv::norm(trasl);

		int numMonoBadPoints = 0, numMonoOptPoints = 0;
		int numStereoBadPoints = 0, numStereoOptPoints = 0;
		vector<MapPoint *> vpMonoMPsOpt, vpStereoMPsOpt;
		vector<MapPoint *> vpMonoMPsBad, vpStereoMPsBad;

		for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
			ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
			MapPoint *pMP = vpMapPointEdgeMono[i];
			KeyFrame *pKFedge = vpEdgeKFMono[i];

			if (pKFi != pKFedge) {
				continue;
			}

			if (pMP->isBad()) {
				continue;
			}

			if (e->chi2() > 5.991 || !e->isDepthPositive()) {
				numMonoBadPoints++;
				vpMonoMPsBad.push_back(pMP);
			}
			else {
				numMonoOptPoints++;
				vpMonoMPsOpt.push_back(pMP);
			}
		}

		for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
			g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
			MapPoint *pMP = vpMapPointEdgeStereo[i];
			KeyFrame *pKFedge = vpEdgeKFMono[i];

			if (pKFi != pKFedge) {
				continue;
			}

			if (pMP->isBad()) {
				continue;
			}

			if (e->chi2() > 7.815 || !e->isDepthPositive()) {
				numStereoBadPoints++;
				vpStereoMPsBad.push_back(pMP);
			}
			else {
				numStereoOptPoints++;
				vpStereoMPsOpt.push_back(pMP);
			}
		}

		if (numMonoOptPoints + numStereoOptPoints < 50) {
			Verbose::PrintMess("LBA ERROR: KF " + to_string(pKFi->mnId) + " has only " + to_string(numMonoOptPoints)
				                   + " monocular and " + to_string(numStereoOptPoints) + " stereo points",
			                   Verbose::VERBOSITY_DEBUG);
		}
		if (dist > 1.0) {
			if (bShowImages) {
				string strNameFile = pKFi->mNameFile;
				cv::Mat imLeft = cv::imread(strNameFile, cv::IMREAD_UNCHANGED);

				cv::cvtColor(imLeft, imLeft, cv::COLOR_GRAY2BGR);

				int numPointsMono = 0, numPointsStereo = 0;
				int numPointsMonoBad = 0, numPointsStereoBad = 0;
				for (int i = 0; i < vpMonoMPsOpt.size(); ++i) {
					if (!vpMonoMPsOpt[i] || vpMonoMPsOpt[i]->isBad()) {
						continue;
					}
					int index = get<0>(vpMonoMPsOpt[i]->GetIndexInKeyFrame(pKFi));
					if (index < 0) {
						//cout << "LBA ERROR: KF has a monocular observation which is not recognized by the MP" << endl;
						//cout << "LBA: KF " << pKFi->mnId << " and MP " << vpMonoMPsOpt[i]->mnId << " with index " << endl;
						continue;
					}

					//string strNumOBs = to_string(vpMapPointsKF[i]->Observations());
					cv::circle(imLeft, pKFi->mvKeys[index].pt, 2, cv::Scalar(255, 0, 0));
					//cv::putText(imLeft, strNumOBs, pKF->mvKeys[i].pt, cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0));
					numPointsMono++;
				}

				for (int i = 0; i < vpStereoMPsOpt.size(); ++i) {
					if (!vpStereoMPsOpt[i] || vpStereoMPsOpt[i]->isBad()) {
						continue;
					}
					int index = get<0>(vpStereoMPsOpt[i]->GetIndexInKeyFrame(pKFi));
					if (index < 0) {
						//cout << "LBA: KF has a stereo observation which is not recognized by the MP" << endl;
						//cout << "LBA: KF " << pKFi->mnId << " and MP " << vpStereoMPsOpt[i]->mnId << endl;
						continue;
					}

					//string strNumOBs = to_string(vpMapPointsKF[i]->Observations());
					cv::circle(imLeft, pKFi->mvKeys[index].pt, 2, cv::Scalar(0, 255, 0));
					//cv::putText(imLeft, strNumOBs, pKF->mvKeys[i].pt, cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0));
					numPointsStereo++;
				}

				for (int i = 0; i < vpMonoMPsBad.size(); ++i) {
					if (!vpMonoMPsBad[i] || vpMonoMPsBad[i]->isBad()) {
						continue;
					}
					int index = get<0>(vpMonoMPsBad[i]->GetIndexInKeyFrame(pKFi));
					if (index < 0) {
						//cout << "LBA ERROR: KF has a monocular observation which is not recognized by the MP" << endl;
						//cout << "LBA: KF " << pKFi->mnId << " and MP " << vpMonoMPsOpt[i]->mnId << " with index " << endl;
						continue;
					}

					//string strNumOBs = to_string(vpMapPointsKF[i]->Observations());
					cv::circle(imLeft, pKFi->mvKeys[index].pt, 2, cv::Scalar(0, 0, 255));
					//cv::putText(imLeft, strNumOBs, pKF->mvKeys[i].pt, cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0));
					numPointsMonoBad++;
				}
				for (int i = 0; i < vpStereoMPsBad.size(); ++i) {
					if (!vpStereoMPsBad[i] || vpStereoMPsBad[i]->isBad()) {
						continue;
					}
					int index = get<0>(vpStereoMPsBad[i]->GetIndexInKeyFrame(pKFi));
					if (index < 0) {
						//cout << "LBA: KF has a stereo observation which is not recognized by the MP" << endl;
						//cout << "LBA: KF " << pKFi->mnId << " and MP " << vpStereoMPsOpt[i]->mnId << endl;
						continue;
					}

					//string strNumOBs = to_string(vpMapPointsKF[i]->Observations());
					cv::circle(imLeft, pKFi->mvKeys[index].pt, 2, cv::Scalar(0, 0, 255));
					//cv::putText(imLeft, strNumOBs, pKF->mvKeys[i].pt, cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0));
					numPointsStereoBad++;
				}

				string namefile =
					"./test_LBA/LBA_KF" + to_string(pKFi->mnId) + "_" + to_string(numPointsMono + numPointsStereo)
						+ "_D" + to_string(dist) + ".png";
				cv::imwrite(namefile, imLeft);

				Verbose::PrintMess("--LBA in KF " + to_string(pKFi->mnId), Verbose::VERBOSITY_DEBUG);
				Verbose::PrintMess("--Distance: " + to_string(dist) + " meters", Verbose::VERBOSITY_DEBUG);
				Verbose::PrintMess("--Number of observations: " + to_string(numMonoOptPoints) + " in mono and "
					                   + to_string(numStereoOptPoints) + " in stereo", Verbose::VERBOSITY_DEBUG);
				Verbose::PrintMess(
					"--Number of discarded observations: " + to_string(numMonoBadPoints) + " in mono and "
						+ to_string(numStereoBadPoints) + " in stereo", Verbose::VERBOSITY_DEBUG);
				Verbose::PrintMess(
					"--To much distance correction in LBA: It has " + to_string(mpObsKFs[pKFi]) + " observated MPs",
					Verbose::VERBOSITY_DEBUG);
				Verbose::PrintMess("--To much distance correction in LBA: It has " + to_string(mpObsFinalKFs[pKFi])
					                   + " deleted observations", Verbose::VERBOSITY_DEBUG);
				Verbose::PrintMess("--------", Verbose::VERBOSITY_DEBUG);
			}
		}
		pKFi->SetPose(Tiw);
	}
	//cout << "End to update the KeyFrames" << endl;

	//Points
	for (MapPoint *pMPi: vpMPs) {
		if (pMPi->isBad()) {
			continue;
		}

		g2o::VertexSBAPointXYZ
			*vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMPi->mnId + maxKFid + 1));
		pMPi->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMPi->UpdateNormalAndDepth();
	}
	//cout << "End to update MapPoint" << endl;
	lock.unlock();
}

void Optimizer::MergeInertialBA(KeyFrame *pCurrKF,
                                KeyFrame *pMergeKF,
                                bool *pbStopFlag,
                                Map *pMap,
                                LoopClosing::KeyFrameAndPose &corrPoses)
{
	const int Nd = 6;
	const unsigned long maxKFid = pCurrKF->mnId;

	vector<KeyFrame *> vpOptimizableKFs;
	vpOptimizableKFs.reserve(2 * Nd);

	// For cov KFS, inertial parameters are not optimized
	const int maxCovKF = 30;
	vector<KeyFrame *> vpOptimizableCovKFs;
	vpOptimizableCovKFs.reserve(maxCovKF);

	// Add sliding window for current KF
	vpOptimizableKFs.push_back(pCurrKF);
	pCurrKF->mnBALocalForKF = pCurrKF->mnId;
	for (int i = 1; i < Nd; i++) {
		if (vpOptimizableKFs.back()->mPrevKF) {
			vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
			vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
		}
		else {
			break;
		}
	}

	list<KeyFrame *> lFixedKeyFrames;
	if (vpOptimizableKFs.back()->mPrevKF) {
		vpOptimizableCovKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
		vpOptimizableKFs.back()->mPrevKF->mnBALocalForKF = pCurrKF->mnId;
	}
	else {
		vpOptimizableCovKFs.push_back(vpOptimizableKFs.back());
		vpOptimizableKFs.pop_back();
	}

	KeyFrame *pKF0 = vpOptimizableCovKFs.back();
	cv::Mat Twc0 = pKF0->GetPoseInverse();

	// Add temporal neighbours to merge KF (previous and next KFs)
	vpOptimizableKFs.push_back(pMergeKF);
	pMergeKF->mnBALocalForKF = pCurrKF->mnId;

	// Previous KFs
	for (int i = 1; i < (Nd / 2); i++) {
		if (vpOptimizableKFs.back()->mPrevKF) {
			vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
			vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
		}
		else {
			break;
		}
	}

	// We fix just once the old map
	if (vpOptimizableKFs.back()->mPrevKF) {
		lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
		vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pCurrKF->mnId;
	}
	else {
		vpOptimizableKFs.back()->mnBALocalForKF = 0;
		vpOptimizableKFs.back()->mnBAFixedForKF = pCurrKF->mnId;
		lFixedKeyFrames.push_back(vpOptimizableKFs.back());
		vpOptimizableKFs.pop_back();
	}

	// Next KFs
	if (pMergeKF->mNextKF) {
		vpOptimizableKFs.push_back(pMergeKF->mNextKF);
		vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
	}

	while (vpOptimizableKFs.size() < (2 * Nd)) {
		if (vpOptimizableKFs.back()->mNextKF) {
			vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mNextKF);
			vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
		}
		else {
			break;
		}
	}

	int N = vpOptimizableKFs.size();

	// Optimizable points seen by optimizable keyframes
	list<MapPoint *> lLocalMapPoints;
	map<MapPoint *, int> mLocalObs;
	for (int i = 0; i < N; i++) {
		vector<MapPoint *> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
		for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
			// Using mnBALocalForKF we avoid redundance here, one MP can not be added several times to lLocalMapPoints
			MapPoint *pMP = *vit;
			if (pMP) {
				if (!pMP->isBad()) {
					if (pMP->mnBALocalForKF != pCurrKF->mnId) {
						mLocalObs[pMP] = 1;
						lLocalMapPoints.push_back(pMP);
						pMP->mnBALocalForKF = pCurrKF->mnId;
					}
					else {
						mLocalObs[pMP]++;
					}
				}
			}
		}
	}

	std::vector<std::pair<MapPoint *, int>> pairs;
	pairs.reserve(mLocalObs.size());
	for (auto itr = mLocalObs.begin(); itr != mLocalObs.end(); ++itr)
		pairs.push_back(*itr);
	sort(pairs.begin(), pairs.end(), sortByVal);

	// Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
	int i = 0;
	for (vector<pair<MapPoint *, int>>::iterator lit = pairs.begin(), lend = pairs.end(); lit != lend; lit++, i++) {
		map<KeyFrame *, tuple<int, int>> observations = lit->first->GetObservations();
		if (i >= maxCovKF) {
			break;
		}
		for (map<KeyFrame *, tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end();
		     mit != mend; mit++) {
			KeyFrame *pKFi = mit->first;

			if (pKFi->mnBALocalForKF != pCurrKF->mnId
				&& pKFi->mnBAFixedForKF != pCurrKF->mnId) // If optimizable or already included...
			{
				pKFi->mnBALocalForKF = pCurrKF->mnId;
				if (!pKFi->isBad()) {
					vpOptimizableCovKFs.push_back(pKFi);
					break;
				}
			}
		}
	}

	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

	solver->setUserLambdaInit(1e3); // TODO uncomment

	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(false);

	// Set Local KeyFrame vertices
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

	// Set Local cov keyframes vertices
	int Ncov = vpOptimizableCovKFs.size();
	for (int i = 0; i < Ncov; i++) {
		KeyFrame *pKFi = vpOptimizableCovKFs[i];

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

	// Set Fixed KeyFrame vertices
	for (list<KeyFrame *>::iterator lit = lFixedKeyFrames.begin(), lend = lFixedKeyFrames.end(); lit != lend; lit++) {
		KeyFrame *pKFi = *lit;
		VertexPose *VP = new VertexPose(pKFi);
		VP->setId(pKFi->mnId);
		VP->setFixed(true);
		optimizer.addVertex(VP);

		if (pKFi->bImu) {
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
	vector<EdgeGyroRW *> vegr(N, (EdgeGyroRW *)NULL);
	vector<EdgeAccRW *> vear(N, (EdgeAccRW *)NULL);
	for (int i = 0; i < N; i++) {
		//cout << "inserting inertial edge " << i << endl;
		KeyFrame *pKFi = vpOptimizableKFs[i];

		if (!pKFi->mPrevKF) {
			Verbose::PrintMess("NOT INERTIAL LINK TO PREVIOUS FRAME!!!!", Verbose::VERBOSITY_NORMAL);
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

			// TODO Uncomment
			g2o::RobustKernelHuber *rki = new g2o::RobustKernelHuber;
			vei[i]->setRobustKernel(rki);
			rki->setDelta(sqrt(16.92));
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
			Verbose::PrintMess("ERROR building inertial edge", Verbose::VERBOSITY_NORMAL);
		}
	}

	Verbose::PrintMess("end inserting inertial edges", Verbose::VERBOSITY_NORMAL);

	// Set MapPoint vertices
	const int nExpectedSize = (N + Ncov + lFixedKeyFrames.size()) * lLocalMapPoints.size();

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

	const unsigned long iniMPid = maxKFid * 5; // TODO: should be  maxKFid*4;

	Verbose::PrintMess("start inserting MPs", Verbose::VERBOSITY_NORMAL);
	for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
		MapPoint *pMP = *lit;
		if (!pMP) {
			continue;
		}

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

			if (!pKFi) {
				continue;
			}

			if ((pKFi->mnBALocalForKF != pCurrKF->mnId) && (pKFi->mnBAFixedForKF != pCurrKF->mnId)) {
				continue;
			}

			if (pKFi->mnId > maxKFid) {
				Verbose::PrintMess("ID greater than current KF is", Verbose::VERBOSITY_NORMAL);
				continue;
			}

			if (optimizer.vertex(id) == NULL || optimizer.vertex(pKFi->mnId) == NULL) {
				continue;
			}

			if (!pKFi->isBad()) {
				const cv::KeyPoint &kpUn = pKFi->mvKeysUn[get<0>(mit->second)];

				if (pKFi->mvuRight[get<0>(mit->second)] < 0) // Monocular observation
				{
					Eigen::Matrix<double, 2, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y;

					EdgeMono *e = new EdgeMono();
					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
					e->setMeasurement(obs);
					const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
					e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberMono);
					optimizer.addEdge(e);
					vpEdgesMono.push_back(e);
					vpEdgeKFMono.push_back(pKFi);
					vpMapPointEdgeMono.push_back(pMP);
				}
				else // stereo observation
				{
					const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
					Eigen::Matrix<double, 3, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

					EdgeStereo *e = new EdgeStereo();

					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
					e->setMeasurement(obs);
					const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
					e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberStereo);

					optimizer.addEdge(e);
					vpEdgesStereo.push_back(e);
					vpEdgeKFStereo.push_back(pKFi);
					vpMapPointEdgeStereo.push_back(pMP);
				}
			}
		}
	}

	if (pbStopFlag) {
		if (*pbStopFlag) {
			return;
		}
	}
	optimizer.initializeOptimization();
	optimizer.optimize(3);
	if (pbStopFlag) {
		if (!*pbStopFlag) {
			optimizer.optimize(5);
		}
	}

	optimizer.setForceStopFlag(pbStopFlag);

	vector<pair<KeyFrame *, MapPoint *>> vToErase;
	vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

	// Check inlier observations
	// Mono
	for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
		EdgeMono *e = vpEdgesMono[i];
		MapPoint *pMP = vpMapPointEdgeMono[i];

		if (pMP->isBad()) {
			continue;
		}

		if (e->chi2() > chi2Mono2) {
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
	// unique_lock<timed_mutex> lock(pMap->mMutexMapUpdate);
	if (!vToErase.empty()) {
		for (size_t i = 0; i < vToErase.size(); i++) {
			KeyFrame *pKFi = vToErase[i].first;
			MapPoint *pMPi = vToErase[i].second;
			pKFi->EraseMapPointMatch(pMPi);
			pMPi->EraseObservation(pKFi);
		}
	}

	// Recover optimized data
	//Keyframes
	for (int i = 0; i < N; i++) {
		KeyFrame *pKFi = vpOptimizableKFs[i];

		VertexPose *VP = static_cast<VertexPose *>(optimizer.vertex(pKFi->mnId));
		cv::Mat Tcw = Converter::toCvSE3(VP->estimate().Rcw[0], VP->estimate().tcw[0]);
		pKFi->SetPose(Tcw);

		cv::Mat Tiw = pKFi->GetPose();
		cv::Mat Riw = Tiw.rowRange(0, 3).colRange(0, 3);
		cv::Mat tiw = Tiw.rowRange(0, 3).col(3);
		g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw), Converter::toVector3d(tiw), 1.0);
		corrPoses[pKFi] = g2oSiw;

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

	for (int i = 0; i < Ncov; i++) {
		KeyFrame *pKFi = vpOptimizableCovKFs[i];

		VertexPose *VP = static_cast<VertexPose *>(optimizer.vertex(pKFi->mnId));
		cv::Mat Tcw = Converter::toCvSE3(VP->estimate().Rcw[0], VP->estimate().tcw[0]);
		pKFi->SetPose(Tcw);

		cv::Mat Tiw = pKFi->GetPose();
		cv::Mat Riw = Tiw.rowRange(0, 3).colRange(0, 3);
		cv::Mat tiw = Tiw.rowRange(0, 3).col(3);
		g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw), Converter::toVector3d(tiw), 1.0);
		corrPoses[pKFi] = g2oSiw;

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

	//Points
	for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
		MapPoint *pMP = *lit;
		g2o::VertexSBAPointXYZ
			*vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + iniMPid + 1));
		pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMP->UpdateNormalAndDepth();
	}

	pMap->IncreaseChangeIndex();
}

int Optimizer::PoseInertialOptimizationLastKeyFrame(Frame *pFrame, bool bRecInit)
{
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
	optimizer.setVerbose(false);
	optimizer.setAlgorithm(solver);

	int nInitialMonoCorrespondences = 0;
	int nInitialStereoCorrespondences = 0;
	int nInitialCorrespondences = 0;

	// Set Frame vertex
	VertexPose *VP = new VertexPose(pFrame);
	VP->setId(0);
	VP->setFixed(false);
	optimizer.addVertex(VP);
	VertexVelocity *VV = new VertexVelocity(pFrame);
	VV->setId(1);
	VV->setFixed(false);
	optimizer.addVertex(VV);
	VertexGyroBias *VG = new VertexGyroBias(pFrame);
	VG->setId(2);
	VG->setFixed(false);
	optimizer.addVertex(VG);
	VertexAccBias *VA = new VertexAccBias(pFrame);
	VA->setId(3);
	VA->setFixed(false);
	optimizer.addVertex(VA);

	// Set MapPoint vertices
	const int N = pFrame->N;
	const int Nleft = pFrame->Nleft;
	const bool bRight = (Nleft != -1);

	vector<EdgeMonoOnlyPose *> vpEdgesMono;
	vector<EdgeStereoOnlyPose *> vpEdgesStereo;
	vector<size_t> vnIndexEdgeMono;
	vector<size_t> vnIndexEdgeStereo;
	vpEdgesMono.reserve(N);
	vpEdgesStereo.reserve(N);
	vnIndexEdgeMono.reserve(N);
	vnIndexEdgeStereo.reserve(N);

	const float thHuberMono = sqrt(5.991);
	const float thHuberStereo = sqrt(7.815);

	{
		unique_lock<mutex> lock(MapPoint::mGlobalMutex);

		for (int i = 0; i < N; i++) {
			MapPoint *pMP = pFrame->mvpMapPoints[i];
			if (pMP) {
				cv::KeyPoint kpUn;

				// Left monocular observation
				if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
					if (i < Nleft) { // pair left-right
						kpUn = pFrame->mvKeys[i];
					}
					else {
						kpUn = pFrame->mvKeysUn[i];
					}

					nInitialMonoCorrespondences++;
					pFrame->mvbOutlier[i] = false;

					Eigen::Matrix<double, 2, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y;

					EdgeMonoOnlyPose *e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

					e->setVertex(0, VP);
					e->setMeasurement(obs);

					// Add here uncerteinty
					const float unc2 = pFrame->mpCamera->uncertainty2(obs);

					const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
					e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberMono);

					optimizer.addEdge(e);

					vpEdgesMono.push_back(e);
					vnIndexEdgeMono.push_back(i);
				}
					// Stereo observation
				else if (!bRight) {
					nInitialStereoCorrespondences++;
					pFrame->mvbOutlier[i] = false;

					kpUn = pFrame->mvKeysUn[i];
					const float kp_ur = pFrame->mvuRight[i];
					Eigen::Matrix<double, 3, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

					EdgeStereoOnlyPose *e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

					e->setVertex(0, VP);
					e->setMeasurement(obs);

					// Add here uncerteinty
					const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

					const float &invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
					e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberStereo);

					optimizer.addEdge(e);

					vpEdgesStereo.push_back(e);
					vnIndexEdgeStereo.push_back(i);
				}

				// Right monocular observation
				if (bRight && i >= Nleft) {
					nInitialMonoCorrespondences++;
					pFrame->mvbOutlier[i] = false;

					kpUn = pFrame->mvKeysRight[i - Nleft];
					Eigen::Matrix<double, 2, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y;

					EdgeMonoOnlyPose *e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

					e->setVertex(0, VP);
					e->setMeasurement(obs);

					// Add here uncerteinty
					const float unc2 = pFrame->mpCamera->uncertainty2(obs);

					const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
					e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberMono);

					optimizer.addEdge(e);

					vpEdgesMono.push_back(e);
					vnIndexEdgeMono.push_back(i);
				}
			}
		}
	}
	nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;

	KeyFrame *pKF = pFrame->mpLastKeyFrame;
	VertexPose *VPk = new VertexPose(pKF);
	VPk->setId(4);
	VPk->setFixed(true);
	optimizer.addVertex(VPk);
	VertexVelocity *VVk = new VertexVelocity(pKF);
	VVk->setId(5);
	VVk->setFixed(true);
	optimizer.addVertex(VVk);
	VertexGyroBias *VGk = new VertexGyroBias(pKF);
	VGk->setId(6);
	VGk->setFixed(true);
	optimizer.addVertex(VGk);
	VertexAccBias *VAk = new VertexAccBias(pKF);
	VAk->setId(7);
	VAk->setFixed(true);
	optimizer.addVertex(VAk);

	EdgeInertial *ei = new EdgeInertial(pFrame->mpImuPreintegrated);

	ei->setVertex(0, VPk);
	ei->setVertex(1, VVk);
	ei->setVertex(2, VGk);
	ei->setVertex(3, VAk);
	ei->setVertex(4, VP);
	ei->setVertex(5, VV);
	optimizer.addEdge(ei);

	EdgeGyroRW *egr = new EdgeGyroRW();
	egr->setVertex(0, VGk);
	egr->setVertex(1, VG);
	cv::Mat cvInfoG = pFrame->mpImuPreintegrated->C.rowRange(9, 12).colRange(9, 12).inv(cv::DECOMP_SVD);
	Eigen::Matrix3d InfoG;
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++)
			InfoG(r, c) = cvInfoG.at<float>(r, c);
	egr->setInformation(InfoG);
	optimizer.addEdge(egr);

	EdgeAccRW *ear = new EdgeAccRW();
	ear->setVertex(0, VAk);
	ear->setVertex(1, VA);
	cv::Mat cvInfoA = pFrame->mpImuPreintegrated->C.rowRange(12, 15).colRange(12, 15).inv(cv::DECOMP_SVD);
	Eigen::Matrix3d InfoA;
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++)
			InfoA(r, c) = cvInfoA.at<float>(r, c);
	ear->setInformation(InfoA);
	optimizer.addEdge(ear);

	// We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
	// At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
	float chi2Mono[4] = {12, 7.5, 5.991, 5.991};
	float chi2Stereo[4] = {15.6, 9.8, 7.815, 7.815};

	int its[4] = {10, 10, 10, 10};

	int nBad = 0;
	int nBadMono = 0;
	int nBadStereo = 0;
	int nInliersMono = 0;
	int nInliersStereo = 0;
	int nInliers = 0;
	bool bOut = false;
	for (size_t it = 0; it < 4; it++) {
		optimizer.initializeOptimization(0);
		optimizer.optimize(its[it]);

		nBad = 0;
		nBadMono = 0;
		nBadStereo = 0;
		nInliers = 0;
		nInliersMono = 0;
		nInliersStereo = 0;
		float chi2close = 1.5 * chi2Mono[it];

		// For monocular observations
		for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
			EdgeMonoOnlyPose *e = vpEdgesMono[i];

			const size_t idx = vnIndexEdgeMono[i];

			if (pFrame->mvbOutlier[idx]) {
				e->computeError();
			}

			const float chi2 = e->chi2();
			bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

			if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) || !e->isDepthPositive()) {
				pFrame->mvbOutlier[idx] = true;
				e->setLevel(1);
				nBadMono++;
			}
			else {
				pFrame->mvbOutlier[idx] = false;
				e->setLevel(0);
				nInliersMono++;
			}

			if (it == 2) {
				e->setRobustKernel(0);
			}
		}

		// For stereo observations
		for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
			EdgeStereoOnlyPose *e = vpEdgesStereo[i];

			const size_t idx = vnIndexEdgeStereo[i];

			if (pFrame->mvbOutlier[idx]) {
				e->computeError();
			}

			const float chi2 = e->chi2();

			if (chi2 > chi2Stereo[it]) {
				pFrame->mvbOutlier[idx] = true;
				e->setLevel(1); // not included in next optimization
				nBadStereo++;
			}
			else {
				pFrame->mvbOutlier[idx] = false;
				e->setLevel(0);
				nInliersStereo++;
			}

			if (it == 2) {
				e->setRobustKernel(0);
			}
		}

		nInliers = nInliersMono + nInliersStereo;
		nBad = nBadMono + nBadStereo;

		if (optimizer.edges().size() < 10) {
			cout << "PIOLKF: NOT ENOUGH EDGES" << endl;
			break;
		}
	}

	// If not too much tracks, recover not too bad points
	if ((nInliers < 30) && !bRecInit) {
		nBad = 0;
		const float chi2MonoOut = 18.f;
		const float chi2StereoOut = 24.f;
		EdgeMonoOnlyPose *e1;
		EdgeStereoOnlyPose *e2;
		for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
			const size_t idx = vnIndexEdgeMono[i];
			e1 = vpEdgesMono[i];
			e1->computeError();
			if (e1->chi2() < chi2MonoOut) {
				pFrame->mvbOutlier[idx] = false;
			}
			else {
				nBad++;
			}
		}
		for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
			const size_t idx = vnIndexEdgeStereo[i];
			e2 = vpEdgesStereo[i];
			e2->computeError();
			if (e2->chi2() < chi2StereoOut) {
				pFrame->mvbOutlier[idx] = false;
			}
			else {
				nBad++;
			}
		}
	}

	// Recover optimized pose, velocity and biases
	pFrame->SetImuPoseVelocity(Converter::toCvMat(VP->estimate().Rwb),
	                           Converter::toCvMat(VP->estimate().twb),
	                           Converter::toCvMat(VV->estimate()));
	Vector6d b;
	b << VG->estimate(), VA->estimate();
	pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

	// Recover Hessian, marginalize keyFframe states and generate new prior for frame
	Eigen::Matrix<double, 15, 15> H;
	H.setZero();

	H.block<9, 9>(0, 0) += ei->GetHessian2();
	H.block<3, 3>(9, 9) += egr->GetHessian2();
	H.block<3, 3>(12, 12) += ear->GetHessian2();

	int tot_in = 0, tot_out = 0;
	for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
		EdgeMonoOnlyPose *e = vpEdgesMono[i];

		const size_t idx = vnIndexEdgeMono[i];

		if (!pFrame->mvbOutlier[idx]) {
			H.block<6, 6>(0, 0) += e->GetHessian();
			tot_in++;
		}
		else {
			tot_out++;
		}
	}

	for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
		EdgeStereoOnlyPose *e = vpEdgesStereo[i];

		const size_t idx = vnIndexEdgeStereo[i];

		if (!pFrame->mvbOutlier[idx]) {
			H.block<6, 6>(0, 0) += e->GetHessian();
			tot_in++;
		}
		else {
			tot_out++;
		}
	}

	pFrame->mpcpi = new ConstraintPoseImu(VP->estimate().Rwb,
	                                      VP->estimate().twb,
	                                      VV->estimate(),
	                                      VG->estimate(),
	                                      VA->estimate(),
	                                      H);

	return nInitialCorrespondences - nBad;
}

int Optimizer::PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit)
{
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(false);

	int nInitialMonoCorrespondences = 0;
	int nInitialStereoCorrespondences = 0;
	int nInitialCorrespondences = 0;

	// Set Current Frame vertex
	VertexPose *VP = new VertexPose(pFrame);
	VP->setId(0);
	VP->setFixed(false);
	optimizer.addVertex(VP);
	VertexVelocity *VV = new VertexVelocity(pFrame);
	VV->setId(1);
	VV->setFixed(false);
	optimizer.addVertex(VV);
	VertexGyroBias *VG = new VertexGyroBias(pFrame);
	VG->setId(2);
	VG->setFixed(false);
	optimizer.addVertex(VG);
	VertexAccBias *VA = new VertexAccBias(pFrame);
	VA->setId(3);
	VA->setFixed(false);
	optimizer.addVertex(VA);

	// Set MapPoint vertices
	const int N = pFrame->N;
	const int Nleft = pFrame->Nleft;
	const bool bRight = (Nleft != -1);

	vector<EdgeMonoOnlyPose *> vpEdgesMono;
	vector<EdgeStereoOnlyPose *> vpEdgesStereo;
	vector<size_t> vnIndexEdgeMono;
	vector<size_t> vnIndexEdgeStereo;
	vpEdgesMono.reserve(N);
	vpEdgesStereo.reserve(N);
	vnIndexEdgeMono.reserve(N);
	vnIndexEdgeStereo.reserve(N);

	const float thHuberMono = sqrt(5.991);
	const float thHuberStereo = sqrt(7.815);

	{
		unique_lock<mutex> lock(MapPoint::mGlobalMutex);

		for (int i = 0; i < N; i++) {
			MapPoint *pMP = pFrame->mvpMapPoints[i];
			if (pMP) {
				cv::KeyPoint kpUn;
				// Left monocular observation
				if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
					if (i < Nleft) { // pair left-right
						kpUn = pFrame->mvKeys[i];
					}
					else {
						kpUn = pFrame->mvKeysUn[i];
					}

					nInitialMonoCorrespondences++;
					pFrame->mvbOutlier[i] = false;

					Eigen::Matrix<double, 2, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y;

					EdgeMonoOnlyPose *e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

					e->setVertex(0, VP);
					e->setMeasurement(obs);

					// Add here uncerteinty
					const float unc2 = pFrame->mpCamera->uncertainty2(obs);

					const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
					e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberMono);

					optimizer.addEdge(e);

					vpEdgesMono.push_back(e);
					vnIndexEdgeMono.push_back(i);
				}
					// Stereo observation
				else if (!bRight) {
					nInitialStereoCorrespondences++;
					pFrame->mvbOutlier[i] = false;

					kpUn = pFrame->mvKeysUn[i];
					const float kp_ur = pFrame->mvuRight[i];
					Eigen::Matrix<double, 3, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

					EdgeStereoOnlyPose *e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

					e->setVertex(0, VP);
					e->setMeasurement(obs);

					// Add here uncerteinty
					const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

					const float &invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
					e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberStereo);

					optimizer.addEdge(e);

					vpEdgesStereo.push_back(e);
					vnIndexEdgeStereo.push_back(i);
				}

				// Right monocular observation
				if (bRight && i >= Nleft) {
					nInitialMonoCorrespondences++;
					pFrame->mvbOutlier[i] = false;

					kpUn = pFrame->mvKeysRight[i - Nleft];
					Eigen::Matrix<double, 2, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y;

					EdgeMonoOnlyPose *e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

					e->setVertex(0, VP);
					e->setMeasurement(obs);

					// Add here uncerteinty
					const float unc2 = pFrame->mpCamera->uncertainty2(obs);

					const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
					e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberMono);

					optimizer.addEdge(e);

					vpEdgesMono.push_back(e);
					vnIndexEdgeMono.push_back(i);
				}
			}
		}
	}

	nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;

	// Set Previous Frame Vertex
	Frame *pFp = pFrame->mpPrevFrame;

	VertexPose *VPk = new VertexPose(pFp);
	VPk->setId(4);
	VPk->setFixed(false);
	optimizer.addVertex(VPk);
	VertexVelocity *VVk = new VertexVelocity(pFp);
	VVk->setId(5);
	VVk->setFixed(false);
	optimizer.addVertex(VVk);
	VertexGyroBias *VGk = new VertexGyroBias(pFp);
	VGk->setId(6);
	VGk->setFixed(false);
	optimizer.addVertex(VGk);
	VertexAccBias *VAk = new VertexAccBias(pFp);
	VAk->setId(7);
	VAk->setFixed(false);
	optimizer.addVertex(VAk);

	EdgeInertial *ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);

	ei->setVertex(0, VPk);
	ei->setVertex(1, VVk);
	ei->setVertex(2, VGk);
	ei->setVertex(3, VAk);
	ei->setVertex(4, VP);
	ei->setVertex(5, VV);
	optimizer.addEdge(ei);

	EdgeGyroRW *egr = new EdgeGyroRW();
	egr->setVertex(0, VGk);
	egr->setVertex(1, VG);
	cv::Mat cvInfoG = pFrame->mpImuPreintegratedFrame->C.rowRange(9, 12).colRange(9, 12).inv(cv::DECOMP_SVD);
	Eigen::Matrix3d InfoG;
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++)
			InfoG(r, c) = cvInfoG.at<float>(r, c);
	egr->setInformation(InfoG);
	optimizer.addEdge(egr);

	EdgeAccRW *ear = new EdgeAccRW();
	ear->setVertex(0, VAk);
	ear->setVertex(1, VA);
	cv::Mat cvInfoA = pFrame->mpImuPreintegratedFrame->C.rowRange(12, 15).colRange(12, 15).inv(cv::DECOMP_SVD);
	Eigen::Matrix3d InfoA;
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++)
			InfoA(r, c) = cvInfoA.at<float>(r, c);
	ear->setInformation(InfoA);
	optimizer.addEdge(ear);

	if (!pFp->mpcpi) {
		Verbose::PrintMess("pFp->mpcpi does not exist!!!\nPrevious Frame " + to_string(pFp->mnId),
		                   Verbose::VERBOSITY_NORMAL);
	}

	EdgePriorPoseImu *ep = new EdgePriorPoseImu(pFp->mpcpi);

	ep->setVertex(0, VPk);
	ep->setVertex(1, VVk);
	ep->setVertex(2, VGk);
	ep->setVertex(3, VAk);
	g2o::RobustKernelHuber *rkp = new g2o::RobustKernelHuber;
	ep->setRobustKernel(rkp);
	rkp->setDelta(5);
	optimizer.addEdge(ep);

	// We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
	// At the next optimization, outliers are not included, but at the end they can be classified as inliers again.

	const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
	const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
	const int its[4] = {10, 10, 10, 10};

	int nBad = 0;
	int nBadMono = 0;
	int nBadStereo = 0;
	int nInliersMono = 0;
	int nInliersStereo = 0;
	int nInliers = 0;
	for (size_t it = 0; it < 4; it++) {
		optimizer.initializeOptimization(0);
		optimizer.optimize(its[it]);

		nBad = 0;
		nBadMono = 0;
		nBadStereo = 0;
		nInliers = 0;
		nInliersMono = 0;
		nInliersStereo = 0;
		float chi2close = 1.5 * chi2Mono[it];

		for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
			EdgeMonoOnlyPose *e = vpEdgesMono[i];

			const size_t idx = vnIndexEdgeMono[i];
			bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

			if (pFrame->mvbOutlier[idx]) {
				e->computeError();
			}

			const float chi2 = e->chi2();

			if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) || !e->isDepthPositive()) {
				pFrame->mvbOutlier[idx] = true;
				e->setLevel(1);
				nBadMono++;
			}
			else {
				pFrame->mvbOutlier[idx] = false;
				e->setLevel(0);
				nInliersMono++;
			}

			if (it == 2) {
				e->setRobustKernel(0);
			}
		}

		for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
			EdgeStereoOnlyPose *e = vpEdgesStereo[i];

			const size_t idx = vnIndexEdgeStereo[i];

			if (pFrame->mvbOutlier[idx]) {
				e->computeError();
			}

			const float chi2 = e->chi2();

			if (chi2 > chi2Stereo[it]) {
				pFrame->mvbOutlier[idx] = true;
				e->setLevel(1);
				nBadStereo++;
			}
			else {
				pFrame->mvbOutlier[idx] = false;
				e->setLevel(0);
				nInliersStereo++;
			}

			if (it == 2) {
				e->setRobustKernel(0);
			}
		}

		nInliers = nInliersMono + nInliersStereo;
		nBad = nBadMono + nBadStereo;

		if (optimizer.edges().size() < 10) {
			cout << "PIOLF: NOT ENOUGH EDGES" << endl;
			break;
		}
	}

	if ((nInliers < 30) && !bRecInit) {
		nBad = 0;
		const float chi2MonoOut = 18.f;
		const float chi2StereoOut = 24.f;
		EdgeMonoOnlyPose *e1;
		EdgeStereoOnlyPose *e2;
		for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
			const size_t idx = vnIndexEdgeMono[i];
			e1 = vpEdgesMono[i];
			e1->computeError();
			if (e1->chi2() < chi2MonoOut) {
				pFrame->mvbOutlier[idx] = false;
			}
			else {
				nBad++;
			}
		}
		for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
			const size_t idx = vnIndexEdgeStereo[i];
			e2 = vpEdgesStereo[i];
			e2->computeError();
			if (e2->chi2() < chi2StereoOut) {
				pFrame->mvbOutlier[idx] = false;
			}
			else {
				nBad++;
			}
		}
	}

	nInliers = nInliersMono + nInliersStereo;

	// Recover optimized pose, velocity and biases
	pFrame->SetImuPoseVelocity(Converter::toCvMat(VP->estimate().Rwb),
	                           Converter::toCvMat(VP->estimate().twb),
	                           Converter::toCvMat(VV->estimate()));
	Vector6d b;
	b << VG->estimate(), VA->estimate();
	pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

	// Recover Hessian, marginalize previous frame states and generate new prior for frame
	Eigen::Matrix<double, 30, 30> H;
	H.setZero();

	H.block<24, 24>(0, 0) += ei->GetHessian();

	Eigen::Matrix<double, 6, 6> Hgr = egr->GetHessian();
	H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
	H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
	H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
	H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);

	Eigen::Matrix<double, 6, 6> Har = ear->GetHessian();
	H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
	H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
	H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
	H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);

	H.block<15, 15>(0, 0) += ep->GetHessian();

	int tot_in = 0, tot_out = 0;
	for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
		EdgeMonoOnlyPose *e = vpEdgesMono[i];

		const size_t idx = vnIndexEdgeMono[i];

		if (!pFrame->mvbOutlier[idx]) {
			H.block<6, 6>(15, 15) += e->GetHessian();
			tot_in++;
		}
		else {
			tot_out++;
		}
	}

	for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
		EdgeStereoOnlyPose *e = vpEdgesStereo[i];

		const size_t idx = vnIndexEdgeStereo[i];

		if (!pFrame->mvbOutlier[idx]) {
			H.block<6, 6>(15, 15) += e->GetHessian();
			tot_in++;
		}
		else {
			tot_out++;
		}
	}

	H = Marginalize(H, 0, 14);

	pFrame->mpcpi = new ConstraintPoseImu(VP->estimate().Rwb,
	                                      VP->estimate().twb,
	                                      VV->estimate(),
	                                      VG->estimate(),
	                                      VA->estimate(),
	                                      H.block<15, 15>(15, 15));
	delete pFp->mpcpi;
	pFp->mpcpi = NULL;

	return nInitialCorrespondences - nBad;
}

int Optimizer::PoseDvlGyrosOPtimizationLastFrame(Frame *pFrame, double lamda_DVL, bool bRecInit)
{
	g2o::SparseOptimizer optimizer;
//	g2o::BlockSolverX::LinearSolverType *linearSolver;

//	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
//
//	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

	g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

//	g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	solver->setUserLambdaInit(100);
	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(false);

	int nInitialMonoCorrespondences = 0;
	int nInitialStereoCorrespondences = 0;
	int nInitialCorrespondences = 0;

	// Set Current Frame vertex
	VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pFrame);
	VP->setId(0);
	VP->setFixed(false);
	optimizer.addVertex(VP);

	VertexGyroBias *VG = new VertexGyroBias(pFrame);
	VG->setId(2);
	VG->setFixed(true);
	optimizer.addVertex(VG);

	g2o::VertexSE3Expmap *vT_d_c = new g2o::VertexSE3Expmap();
	vT_d_c->setEstimate(Converter::toSE3Quat(pFrame->GetExtrinsicParamters().mT_dvl_c));
	vT_d_c->setId(3);
	vT_d_c->setFixed(true);
	optimizer.addVertex(vT_d_c);

	g2o::VertexSE3Expmap *vT_g_d = new g2o::VertexSE3Expmap();
	vT_g_d->setEstimate(Converter::toSE3Quat(pFrame->GetExtrinsicParamters().mT_gyro_dvl));
	vT_g_d->setId(4);
	vT_g_d->setFixed(true);
	optimizer.addVertex(vT_g_d);

	// Set MapPoint vertices
	const int N = pFrame->N;
	const int Nleft = pFrame->Nleft;
	const bool bRight = (Nleft != -1);

	vector<EdgeMonoOnlyPose_DvlGyros *> vpEdgesMono;
	vector<EdgeStereoOnlyPose_DvlGyros *> vpEdgesStereo;
	vector<size_t> vnIndexEdgeMono;
	vector<size_t> vnIndexEdgeStereo;
	vpEdgesMono.reserve(N);
	vpEdgesStereo.reserve(N);
	vnIndexEdgeMono.reserve(N);
	vnIndexEdgeStereo.reserve(N);

	const float thHuberMono = sqrt(5.991);
	const float thHuberStereo = sqrt(7.815);

	// set visual constains
	{
		unique_lock<mutex> lock(MapPoint::mGlobalMutex);

		for (int i = 0; i < N; i++) {
			MapPoint *pMP = pFrame->mvpMapPoints[i];
			if (pMP) {

				// Left monocular observation
//				if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
				if (!pFrame->mpCamera2) {

					if (pFrame->mvuRight[i] < 0) {

						nInitialMonoCorrespondences++;
						pFrame->mvbOutlier[i] = false;

						Eigen::Matrix<double, 2, 1> obs;
						const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
						obs << kpUn.pt.x, kpUn.pt.y;

						EdgeMonoOnlyPose_DvlGyros *e = new EdgeMonoOnlyPose_DvlGyros(pMP->GetWorldPos(), 0);

						e->setVertex(0, VP);
						e->setMeasurement(obs);

						// Add here uncerteinty
						const float unc2 = pFrame->mpCamera->uncertainty2(obs);

						// octave (pyramid layer) from which the keypoint has been extracted
						const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
						e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

						g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(thHuberMono);

						optimizer.addEdge(e);

						vpEdgesMono.push_back(e);
						vnIndexEdgeMono.push_back(i);
					}
					else // Stereo observation
					{
						nInitialStereoCorrespondences++;
						pFrame->mvbOutlier[i] = false;
						const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
						const float kp_ur = pFrame->mvuRight[i];
						Eigen::Matrix<double, 3, 1> obs;
						obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

						EdgeStereoOnlyPose_DvlGyros *e = new EdgeStereoOnlyPose_DvlGyros(pMP->GetWorldPos());

						e->setVertex(0, VP);
						e->setMeasurement(obs);

						// Add here uncerteinty
						const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

						const float &invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
						e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

						g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(thHuberStereo);

						optimizer.addEdge(e);

						vpEdgesStereo.push_back(e);
						vnIndexEdgeStereo.push_back(i);
					}

				}
			}
		}
	}

	nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;


	//todo_tightly
	//	use prior information to contrain pose of previous frame
	//	but do not understand the propogation of uncertainty(Hessian Matrix)
	//  just set it as fixed for now
	Frame *pFp = pFrame->mpPrevFrame;
	VertexPoseDvlIMU *VP2 = new VertexPoseDvlIMU(pFp);
	VP2->setId(1);
	VP2->setFixed(true);
	optimizer.addVertex(VP2);

	// set DVL_Gyros constrain
	//todo_tightly
	//	maybe add velocity to optimization
	EdgeDvlGyroTrack *ei = new EdgeDvlGyroTrack(pFrame->mpDvlPreintegrationFrame);

	ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP));
	ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
	ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
	ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vT_d_c));
	ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vT_g_d));
	ei->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * lamda_DVL*(vpEdgesStereo.size()+vpEdgesStereo.size()));
	ei->setId(pFrame->mnId);
	optimizer.addEdge(ei);


	if (!pFp->mpcpi) {
		Verbose::PrintMess("pFp->mpcpi does not exist!!!\nPrevious Frame " + to_string(pFp->mnId),
		                   Verbose::VERBOSITY_NORMAL);
	}
//	EdgePriorPoseImu *ep = new EdgePriorPoseImu(pFp->mpcpi);
//	ep->setVertex(0, VPk);
//	ep->setVertex(1, VVk);
//	ep->setVertex(2, VGk);
//	ep->setVertex(3, VAk);
//	g2o::RobustKernelHuber *rkp = new g2o::RobustKernelHuber;
//	ep->setRobustKernel(rkp);
//	rkp->setDelta(5);
//	optimizer.addEdge(ep);

	// We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
	// At the next optimization, outliers are not included, but at the end they can be classified as inliers again.

	const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
	const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
	const int its[4] = {10, 10, 10, 10};

	int nBad = 0;
	int nBadMono = 0;
	int nBadStereo = 0;
	int nInliersMono = 0;
	int nInliersStereo = 0;
	int nInliers = 0;
	for (size_t it = 0; it < 4; it++) {
//		cout<<"optimization iteration: "<<it<<endl;
		optimizer.initializeOptimization(0);
		optimizer.optimize(its[it]);

		nBad = 0;
		nBadMono = 0;
		nBadStereo = 0;
		nInliers = 0;
		nInliersMono = 0;
		nInliersStereo = 0;
		float chi2close = 1.5 * chi2Mono[it];

		for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
			EdgeMonoOnlyPose_DvlGyros *e = vpEdgesMono[i];

			const size_t idx = vnIndexEdgeMono[i];
			bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

			if (pFrame->mvbOutlier[idx]) {
				e->computeError();
			}

			const float chi2 = e->chi2();

			if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close)) {
				pFrame->mvbOutlier[idx] = true;
				e->setLevel(1);
				nBadMono++;
//				cout<<"outlier edge id:"<<e->id()<<" chi2: "<<chi2<<endl;
			}
			else {
				pFrame->mvbOutlier[idx] = false;
				e->setLevel(0);
				nInliersMono++;
			}

			if (it == 2) {
				e->setRobustKernel(0);
			}
		}

		for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
			EdgeStereoOnlyPose_DvlGyros *e = vpEdgesStereo[i];

			const size_t idx = vnIndexEdgeStereo[i];

			if (pFrame->mvbOutlier[idx]) {
				e->computeError();
			}

			const float chi2 = e->chi2();

			if (chi2 > chi2Stereo[it]) {
				pFrame->mvbOutlier[idx] = true;
				e->setLevel(1);
				nBadStereo++;
//				cout<<"outlier edge id:"<<e->id()<<" chi2: "<<chi2<<endl;
			}
			else {
				pFrame->mvbOutlier[idx] = false;
				e->setLevel(0);
				nInliersStereo++;
			}

			if (it == 2) {
				e->setRobustKernel(0);
			}
		}

		nInliers = nInliersMono + nInliersStereo;
		nBad = nBadMono + nBadStereo;

//		ROS_INFO_STREAM("track local map inlier: "<<nInliers<<" outlier: "<<nBad);
//		cout<<"inlier map points: "<<nInliers<<endl;
//		cout<<"outlier map points: "<<nBad<<endl;

		if (optimizer.edges().size() < 10) {
			cout << "PIOLF: NOT ENOUGH EDGES" << endl;
			break;
		}
	}

	if ((nInliers < 30) && !bRecInit) {
		nBad = 0;
		const float chi2MonoOut = 18.f;
		const float chi2StereoOut = 24.f;
		EdgeMonoOnlyPose_DvlGyros *e1;
		EdgeStereoOnlyPose_DvlGyros *e2;
		for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
			const size_t idx = vnIndexEdgeMono[i];
			e1 = vpEdgesMono[i];
			e1->computeError();
			if (e1->chi2() < chi2MonoOut) {
				pFrame->mvbOutlier[idx] = false;
			}
			else {
				nBad++;
			}
		}
		for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
			const size_t idx = vnIndexEdgeStereo[i];
			e2 = vpEdgesStereo[i];
			e2->computeError();
			if (e2->chi2() < chi2StereoOut) {
				pFrame->mvbOutlier[idx] = false;
			}
			else {
				nBad++;
			}
		}
	}

	nInliers = nInliersMono + nInliersStereo;

	if (nInliers > 30) {
		// Recover optimized pose, velocity and biases
//	pFrame->SetImuPoseVelocity(Converter::toCvMat(VP->estimate().Rwb),
//							   Converter::toCvMat(VP->estimate().twb),
//							   Converter::toCvMat(VV->estimate()));
		Eigen::Matrix3d R_w_g = VP->estimate().Rwc * VP->estimate().R_c_gyro[0];
		Eigen::Vector3d t_w_d = VP->estimate().twc + VP->estimate().Rwc * VP->estimate().t_c_dvl[0];
		Eigen::Isometry3d T_c_w = Eigen::Isometry3d::Identity();
		T_c_w.pretranslate(VP->estimate().tcw[0]);
		T_c_w.rotate(VP->estimate().Rcw[0]);
		cv::Mat R_w_g_cv;
		cv::eigen2cv(R_w_g, R_w_g_cv);
		R_w_g_cv.convertTo(R_w_g_cv, CV_32F);
		cv::Mat T_c_w_cv;
		cv::eigen2cv(T_c_w.matrix(), T_c_w_cv);
		T_c_w_cv.convertTo(T_c_w_cv, CV_32F);
		pFrame->SetPose(T_c_w_cv);
//		pFrame->SetDvlPoseVelocity(Converter::toCvMat(R_w_g),
//								   Converter::toCvMat(t_w_d),
//								   pFrame->GetDvlVelocity());

		Vector6d b;
		b << VG->estimate(), 0, 0, 0;
		pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);
	}




	return nInitialCorrespondences - nBad;
}

} // namespace ORB_SLAM3
