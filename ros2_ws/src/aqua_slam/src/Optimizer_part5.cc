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

int Optimizer::PoseDvlGyrosOPtimizationLastKeyFrame(Frame *pFrame, double lamda_DVL, bool bRecInit)
{
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

//	g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
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


	KeyFrame *pFK = pFrame->mpLastKeyFrame;
	VertexPoseDvlIMU *VP2 = new VertexPoseDvlIMU(pFK);
	VP2->setId(1);
	VP2->setFixed(true);
	optimizer.addVertex(VP2);

	// set DVL_Gyros constrain
	//todo_tightly
	//	maybe add velocity to optimization
	EdgeDvlGyroTrack *ei = new EdgeDvlGyroTrack(pFrame->mpDvlPreintegrationKeyFrame);

	ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP));
	ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
	ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
	ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vT_d_c));
	ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vT_g_d));
	ei->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * lamda_DVL*(vpEdgesStereo.size()+vpEdgesMono.size()));
	ei->setId(pFrame->mnId);
	optimizer.addEdge(ei);


//	if (!pFK->mpcpi) {
//		Verbose::PrintMess("pFp->mpcpi does not exist!!!\nPrevious Frame " + to_string(pFK->mnId),
//						   Verbose::VERBOSITY_NORMAL);
//	}
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
//	pFrame->SetDvlPoseVelocity(Converter::toCvMat(R_w_g),
//							   Converter::toCvMat(t_w_d),
//							   pFrame->GetDvlVelocity());
	Vector6d b;
	b << VG->estimate(), 0, 0, 0;
	pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

	//todo_tightly
	// do not understand how to get Hessian matrix
	// Recover Hessian, marginalize previous frame states and generate new prior for frame
//	Eigen::Matrix<double, 30, 30> H;
//	H.setZero();
//
//	H.block<24, 24>(0, 0) += ei->GetHessian();
//
//	Eigen::Matrix<double, 6, 6> Hgr = egr->GetHessian();
//	H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
//	H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
//	H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
//	H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);
//
//	Eigen::Matrix<double, 6, 6> Har = ear->GetHessian();
//	H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
//	H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
//	H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
//	H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);
//
//	H.block<15, 15>(0, 0) += ep->GetHessian();
//
//	int tot_in = 0, tot_out = 0;
//	for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
//		EdgeMonoOnlyPose *e = vpEdgesMono[i];
//
//		const size_t idx = vnIndexEdgeMono[i];
//
//		if (!pFrame->mvbOutlier[idx]) {
//			H.block<6, 6>(15, 15) += e->GetHessian();
//			tot_in++;
//		}
//		else {
//			tot_out++;
//		}
//	}
//
//	for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
//		EdgeStereoOnlyPose *e = vpEdgesStereo[i];
//
//		const size_t idx = vnIndexEdgeStereo[i];
//
//		if (!pFrame->mvbOutlier[idx]) {
//			H.block<6, 6>(15, 15) += e->GetHessian();
//			tot_in++;
//		}
//		else {
//			tot_out++;
//		}
//	}
//
//	H = Marginalize(H, 0, 14);
//
//	pFrame->mpcpi = new ConstraintPoseImu(VP->estimate().Rwb,
//										  VP->estimate().twb,
//										  VV->estimate(),
//										  VG->estimate(),
//										  VA->estimate(),
//										  H.block<15, 15>(15, 15));
//	delete pFp->mpcpi;
//	pFp->mpcpi = NULL;

	return nInitialCorrespondences - nBad;
}

void Optimizer::OptimizeEssentialGraph4DoF(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                           const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                           const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                           const map<KeyFrame *, set<KeyFrame *>> &LoopConnections)
{
	typedef g2o::BlockSolver<g2o::BlockSolverTraits<4, 4>> BlockSolver_4_4;

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	optimizer.setVerbose(false);
	g2o::BlockSolverX::LinearSolverType *linearSolver =
		new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

	optimizer.setAlgorithm(solver);

	const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
	const vector<MapPoint *> vpMPs = pMap->GetAllMapPoints();

	const unsigned int nMaxKFid = pMap->GetMaxKFid();

	vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
	vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(nMaxKFid + 1);

	vector<VertexPose4DoF *> vpVertices(nMaxKFid + 1);

	const int minFeat = 100;
	// Set KeyFrame vertices
	for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
		KeyFrame *pKF = vpKFs[i];
		if (pKF->isBad()) {
			continue;
		}

		VertexPose4DoF *V4DoF;

		const int nIDi = pKF->mnId;

		LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

		if (it != CorrectedSim3.end()) {
			vScw[nIDi] = it->second;
			const g2o::Sim3 Swc = it->second.inverse();
			Eigen::Matrix3d Rwc = Swc.rotation().toRotationMatrix();
			Eigen::Vector3d twc = Swc.translation();
			V4DoF = new VertexPose4DoF(Rwc, twc, pKF);
		}
		else {
			Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
			Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKF->GetTranslation());
			g2o::Sim3 Siw(Rcw, tcw, 1.0);
			vScw[nIDi] = Siw;
			V4DoF = new VertexPose4DoF(pKF);
		}

		if (pKF == pLoopKF) {
			V4DoF->setFixed(true);
		}

		V4DoF->setId(nIDi);
		V4DoF->setMarginalized(false);

		optimizer.addVertex(V4DoF);
		vpVertices[nIDi] = V4DoF;
	}
	cout << "PoseGraph4DoF: KFs loaded" << endl;

	set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

	// Edge used in posegraph has still 6Dof, even if updates of camera poses are just in 4DoF
	Eigen::Matrix<double, 6, 6> matLambda = Eigen::Matrix<double, 6, 6>::Identity();
	matLambda(0, 0) = 1e3;
	matLambda(1, 1) = 1e3;
	matLambda(0, 0) = 1e3;

	// Set Loop edges
	Edge4DoF *e_loop;
	for (map<KeyFrame *, set<KeyFrame *>>::const_iterator mit = LoopConnections.begin(), mend = LoopConnections.end();
	     mit != mend; mit++) {
		KeyFrame *pKF = mit->first;
		const long unsigned int nIDi = pKF->mnId;
		const set<KeyFrame *> &spConnections = mit->second;
		const g2o::Sim3 Siw = vScw[nIDi];
		const g2o::Sim3 Swi = Siw.inverse();

		for (set<KeyFrame *>::const_iterator sit = spConnections.begin(), send = spConnections.end(); sit != send;
		     sit++) {
			const long unsigned int nIDj = (*sit)->mnId;
			if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) && pKF->GetWeight(*sit) < minFeat) {
				continue;
			}

			const g2o::Sim3 Sjw = vScw[nIDj];
			const g2o::Sim3 Sij = Siw * Sjw.inverse();
			Eigen::Matrix4d Tij;
			Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
			Tij.block<3, 1>(0, 3) = Sij.translation();
			Tij(3, 3) = 1.;

			Edge4DoF *e = new Edge4DoF(Tij);
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));

			e->information() = matLambda;
			e_loop = e;
			optimizer.addEdge(e);

			sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
		}
	}
	cout << "PoseGraph4DoF: Loop edges loaded" << endl;

	// 1. Set normal edges
	for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
		KeyFrame *pKF = vpKFs[i];

		const int nIDi = pKF->mnId;

		g2o::Sim3 Siw;

		// Use noncorrected poses for posegraph edges
		LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

		if (iti != NonCorrectedSim3.end()) {
			Siw = iti->second;
		}
		else {
			Siw = vScw[nIDi];
		}

		// 1.1.0 Spanning tree edge
		KeyFrame *pParentKF = static_cast<KeyFrame *>(NULL);
		if (pParentKF) {
			int nIDj = pParentKF->mnId;

			g2o::Sim3 Swj;

			LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

			if (itj != NonCorrectedSim3.end()) {
				Swj = (itj->second).inverse();
			}
			else {
				Swj = vScw[nIDj].inverse();
			}

			g2o::Sim3 Sij = Siw * Swj;
			Eigen::Matrix4d Tij;
			Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
			Tij.block<3, 1>(0, 3) = Sij.translation();
			Tij(3, 3) = 1.;

			Edge4DoF *e = new Edge4DoF(Tij);
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
			e->information() = matLambda;
			optimizer.addEdge(e);
		}

		// 1.1.1 Inertial edges
		KeyFrame *prevKF = pKF->mPrevKF;
		if (prevKF) {
			int nIDj = prevKF->mnId;

			g2o::Sim3 Swj;

			LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(prevKF);

			if (itj != NonCorrectedSim3.end()) {
				Swj = (itj->second).inverse();
			}
			else {
				Swj = vScw[nIDj].inverse();
			}

			g2o::Sim3 Sij = Siw * Swj;
			Eigen::Matrix4d Tij;
			Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
			Tij.block<3, 1>(0, 3) = Sij.translation();
			Tij(3, 3) = 1.;

			Edge4DoF *e = new Edge4DoF(Tij);
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
			e->information() = matLambda;
			optimizer.addEdge(e);
		}

		// 1.2 Loop edges
		const set<KeyFrame *> sLoopEdges = pKF->GetLoopEdges();
		for (set<KeyFrame *>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++) {
			KeyFrame *pLKF = *sit;
			if (pLKF->mnId < pKF->mnId) {
				g2o::Sim3 Swl;

				LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

				if (itl != NonCorrectedSim3.end()) {
					Swl = itl->second.inverse();
				}
				else {
					Swl = vScw[pLKF->mnId].inverse();
				}

				g2o::Sim3 Sil = Siw * Swl;
				Eigen::Matrix4d Til;
				Til.block<3, 3>(0, 0) = Sil.rotation().toRotationMatrix();
				Til.block<3, 1>(0, 3) = Sil.translation();
				Til(3, 3) = 1.;

				Edge4DoF *e = new Edge4DoF(Til);
				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pLKF->mnId)));
				e->information() = matLambda;
				optimizer.addEdge(e);
			}
		}

		// 1.3 Covisibility graph edges
		const vector<KeyFrame *> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
		for (vector<KeyFrame *>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++) {
			KeyFrame *pKFn = *vit;
			if (pKFn && pKFn != pParentKF && pKFn != prevKF && pKFn != pKF->mNextKF && !pKF->hasChild(pKFn)
				&& !sLoopEdges.count(pKFn)) {
				if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
					if (sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId), max(pKF->mnId, pKFn->mnId)))) {
						continue;
					}

					g2o::Sim3 Swn;

					LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

					if (itn != NonCorrectedSim3.end()) {
						Swn = itn->second.inverse();
					}
					else {
						Swn = vScw[pKFn->mnId].inverse();
					}

					g2o::Sim3 Sin = Siw * Swn;
					Eigen::Matrix4d Tin;
					Tin.block<3, 3>(0, 0) = Sin.rotation().toRotationMatrix();
					Tin.block<3, 1>(0, 3) = Sin.translation();
					Tin(3, 3) = 1.;
					Edge4DoF *e = new Edge4DoF(Tin);
					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFn->mnId)));
					e->information() = matLambda;
					optimizer.addEdge(e);
				}
			}
		}
	}
	cout << "PoseGraph4DoF: Covisibility edges loaded" << endl;

	optimizer.initializeOptimization();
	optimizer.computeActiveErrors();
	optimizer.optimize(20);

	unique_lock<shared_timed_mutex> lock(pMap->mMutexMapUpdate);

	// SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];

		const int nIDi = pKFi->mnId;

		VertexPose4DoF *Vi = static_cast<VertexPose4DoF *>(optimizer.vertex(nIDi));
		Eigen::Matrix3d Ri = Vi->estimate().Rcw[0];
		Eigen::Vector3d ti = Vi->estimate().tcw[0];

		g2o::Sim3 CorrectedSiw = g2o::Sim3(Ri, ti, 1.);
		vCorrectedSwc[nIDi] = CorrectedSiw.inverse();

		cv::Mat Tiw = Converter::toCvSE3(Ri, ti);
		pKFi->SetPose(Tiw);
	}

	// Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
	for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
		MapPoint *pMP = vpMPs[i];

		if (pMP->isBad()) {
			continue;
		}

		int nIDr;

		KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
		nIDr = pRefKF->mnId;

		g2o::Sim3 Srw = vScw[nIDr];
		g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

		cv::Mat P3Dw = pMP->GetWorldPos();
		Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
		Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

		cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
		pMP->SetWorldPos(cvCorrectedP3Dw);

		pMP->UpdateNormalAndDepth();
	}
	pMap->IncreaseChangeIndex();
}

void Optimizer::DvlGyroInitOptimization(Map *pMap,
                                        Eigen::Vector3d &bg,
                                        bool bMono,
                                        float priorG)
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
		VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
		VP->setId(pKFi->mnId);
		VP->setFixed(true);
		optimizer.addVertex(VP);
	}

	// Biases
	//todo_tightly
	//	set fixed for debuging
	VertexGyroBias *VG = new VertexGyroBias(vpKFs.front());
	VG->setId(maxKFid + 1);
	VG->setFixed(true);
	optimizer.addVertex(VG);

	// prior acc bias
//		EdgePriorGyro *epg = new EdgePriorGyro(cv::Mat::zeros(3, 1, CV_32F));
//		epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
//		double infoPriorG = priorG;
//		epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
//		optimizer.addEdge(epg);

	// extrinsic parameter
	//todo_tightly
	//	set fixed for debuging
	g2o::VertexSE3Expmap *vT_d_c = new g2o::VertexSE3Expmap();
	vT_d_c->setEstimate(Converter::toSE3Quat(vpKFs[0]->mImuCalib.mT_dvl_c));
	vT_d_c->setId(maxKFid + 2);
	vT_d_c->setFixed(false);
	optimizer.addVertex(vT_d_c);

	g2o::VertexSE3Expmap *vT_g_d = new g2o::VertexSE3Expmap();
	vT_g_d->setEstimate(Converter::toSE3Quat(vpKFs[0]->mImuCalib.mT_gyro_dvl));
	vT_g_d->setId(maxKFid + 3);
	vT_g_d->setFixed(true);
	optimizer.addVertex(vT_g_d);


	// Graph edges
	vector<EdgeDvlGyroInit *> vpei;
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
			if (!pKFi->mpDvlPreintegrationKeyFrame) {
				std::cout << "Not preintegrated measurement" << std::endl;
			}

			pKFi->mpDvlPreintegrationKeyFrame->SetNewBias(pKFi->mPrevKF->GetImuBias());
			VertexPoseDvlIMU *VP1 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mPrevKF->mnId));
//				g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
			VertexPoseDvlIMU *VP2 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
//				g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
			g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid + 1);
			g2o::HyperGraph::Vertex *VT_d_c = optimizer.vertex(maxKFid + 2);
			g2o::HyperGraph::Vertex *VT_g_d = optimizer.vertex(maxKFid + 3);
//				g2o::HyperGraph::Vertex *VA = optimizer.vertex(maxKFid * 2 + 3);
//				g2o::HyperGraph::Vertex *VGDir = optimizer.vertex(maxKFid * 2 + 4);
//				g2o::HyperGraph::Vertex *VS = optimizer.vertex(maxKFid * 2 + 5);
//				cout<<"VP1: Rcw[0]"<<VP1->estimate().Rcw[0]<<endl;
//				cout<<"VP1: Rwc"<<VP1->estimate().Rwc<<endl;
//				cout<<"VP1: tcw[0]"<<VP1->estimate().tcw[0]<<endl;
//				cout<<"VP1: twc"<<VP1->estimate().twc<<endl;
//
//				cout<<"VP2: Rcw[0]"<<VP2->estimate().Rcw[0]<<endl;
//				cout<<"VP2: Rwc"<<VP2->estimate().Rwc<<endl;
//				cout<<"VP2: tcw[0]"<<VP2->estimate().tcw[0]<<endl;
//				cout<<"VP2: twc"<<VP2->estimate().twc<<endl;

			if (!VP1 || !VG || !VP2) {
				cout << "Error" << VP1 << ", " << VG << ", " << VP2 << endl;

				continue;
			}
//				EdgeInertialGS *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
			EdgeDvlGyroInit *ei = new EdgeDvlGyroInit(pKFi->mpDvlPreintegrationKeyFrame);
//				ei->setVertex(0, VP1);

//			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
//			ei->setRobustKernel(rk);
//			rk->setDelta(sqrt(7.815));
			ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
			ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
			ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
			ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
			ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
			ei->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100000);
			ei->setId(pKFi->mnId);


			vpei.push_back(ei);

			vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
			optimizer.addEdge(ei);
		}
	}

	// Compute error for different scales
	std::set<g2o::HyperGraph::Edge *> setEdges = optimizer.edges();
	double total_error = 0;

	for (vector<EdgeDvlGyroInit *>::iterator it = vpei.begin(); it != vpei.end(); it++) {
		VertexPoseDvlIMU *VP1 = dynamic_cast<VertexPoseDvlIMU *>((*it)->vertex(0));
		VertexPoseDvlIMU *VP2 = dynamic_cast<VertexPoseDvlIMU *>((*it)->vertex(1));
		VertexGyroBias *VG = dynamic_cast<VertexGyroBias *>((*it)->vertex(2));
		g2o::VertexSE3Expmap *VT = dynamic_cast<g2o::VertexSE3Expmap *>((*it)->vertex(3));
		const IMU::Bias b(0, 0, 0, VG->estimate()[0], VG->estimate()[1], VG->estimate()[2]);

		Eigen::Isometry3d T_dvl_c = VT->estimate();
		const Eigen::Matrix3d R_dvl_c = T_dvl_c.rotation();
		const Eigen::Matrix3d R_c_dvl = T_dvl_c.inverse().rotation();
		const Eigen::Vector3d t_dvl_c = T_dvl_c.translation();
		const Eigen::Vector3d t_c_dvl = T_dvl_c.inverse().translation();

		const Eigen::Matrix3d dR = Converter::toMatrix3d((*it)->mpInt->GetDeltaRotation(b));
		const Eigen::Vector3d dP = Converter::toVector3d((*it)->mpInt->GetDVLPosition(b));

		Eigen::Isometry3d T_di_dj_mea = Eigen::Isometry3d::Identity();
		Eigen::Isometry3d T_di_dj_est = Eigen::Isometry3d::Identity();
		T_di_dj_mea.rotate(dR);
		T_di_dj_mea.pretranslate(dP);

		Eigen::Matrix3d R_est = R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl;
		Eigen::Vector3d t_est = (t_dvl_c - R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * t_dvl_c) +
			(R_dvl_c * (VP1->estimate().Rcw[0] * VP2->estimate().twc - VP1->estimate().Rcw[0] * VP1->estimate().twc));
		T_di_dj_est.rotate(R_est);
		T_di_dj_est.pretranslate(t_est);

		(*it)->computeError();
		Eigen::Matrix<double, 6, 1> error = (*it)->error();
		cout << "edge id: " << (*it)->id() << "\n"
		     //			<<"VP1:\n"
		     //			<<"tcw: "<<VP1->estimate().tcw[0].transpose()<<"twc: "<<VP1->estimate().twc.transpose()<<"\n"
		     //			<<"VP2:\n"
		     //			<<"tcw: "<<VP2->estimate().tcw[0].transpose()<<"twc: "<<VP2->estimate().twc.transpose()<<"\n"
		     //			<<"T_dvl_c:\n"
		     //			<<T_dvl_c.matrix()<<"\n"
		     //			<<"t_est1:\n"
		     //			<<(t_dvl_c - R_dvl_c * VP1->estimate().Rcw[0] * VP2->estimate().Rwc * R_c_dvl * t_dvl_c).transpose()<<"\n"
		     //			<<"t_est2:\n"
		     //			<<(R_dvl_c* (VP1->estimate().Rcw[0]*VP2->estimate().twc - VP1->estimate().Rcw[0]*VP1->estimate().twc)).transpose()<<"\n"
		     //			<<"dP: \n"
		     //			<<(*it)->mpInt->dP.t()<<"\n"
		     //			<<"dR: \n"
		     //			<<(*it)->mpInt->dR<<"\n"
		     //			<<"T_di_dj_est: \n"
		     //			<<T_di_dj_est.matrix()<<"\n"
		     //			<<"T_di_dj_mea: \n"
		     //			<<T_di_dj_mea.matrix()<<"\n"

		     //			<<"have dvl: "<<(*it)->mpInt->bDVL<<"\n"
		     << " error: " << error.transpose() << endl;
		total_error += error.transpose() * error;

	}
	cout << "total error: " << total_error << endl;

	std::cout << "start optimization" << std::endl;
	optimizer.setVerbose(true);
	optimizer.initializeOptimization();
	optimizer.optimize(its);

	std::cout << "end optimization" << std::endl;


	// Recover optimized data
	// Biases
	VG = static_cast<VertexGyroBias *>(optimizer.vertex(maxKFid + 1));
	Vector6d vb;
	vb << VG->estimate(), 0, 0, 0;
	bg << VG->estimate();

	vT_d_c = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(maxKFid + 2));

	Eigen::Isometry3d T_dvl_c = vT_d_c->estimate();
	Eigen::Isometry3d T_gyros_dvl = vT_g_d->estimate();

	IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);

	Eigen::Matrix3d R_gt;
	R_gt << 0, 0, 1,
		-1, 0, 0,
		0, -1, 0;
	cout << "init optimization result: \n"
	     << "bias_gyro:\n" << bg << "\n"
	     << "R_dvl_c:\n" << T_dvl_c.rotation() << "\n"
	     << "R_dvl_c(eular yaw-pitch-roll):" << T_dvl_c.rotation().eulerAngles(2, 1, 0).transpose() << "\n"
	     << "t_dvl_c:" << T_dvl_c.translation().transpose() << "\n"
	     << "R_dvl_c distance with R_gt(LogSO3()): " << LogSO3(T_dvl_c.rotation().inverse() * R_gt).transpose() << "\n"
	     << "R_gyros_dvl:\n" << T_gyros_dvl.rotation() << "\n"
	     << "R_gyros_dvl(eular yaw-pitch-roll):" << T_gyros_dvl.rotation().eulerAngles(2, 1, 0).transpose() << "\n"
	     << endl;


	cv::Mat cvbg = Converter::toCvMat(bg);

	//Keyframes velocities and biases
	std::cout << "update Keyframes biases and extrinsic paramters" << std::endl;

	cv::Mat T_dvl_c_cv, T_gyros_dvl_cv, T_gyros_c_cv;
	cv::eigen2cv(T_dvl_c.matrix(), T_dvl_c_cv);
	T_dvl_c_cv.convertTo(T_dvl_c_cv, CV_32FC1);
	cv::eigen2cv(T_gyros_dvl.matrix(), T_gyros_dvl_cv);
	T_gyros_dvl_cv.convertTo(T_gyros_dvl_cv, CV_32FC1);
	Eigen::Isometry3d T_gyros_c = T_gyros_dvl * T_dvl_c;
	cv::eigen2cv(T_gyros_c.matrix(), T_gyros_c_cv);
	T_gyros_c_cv.convertTo(T_gyros_c_cv, CV_32FC1);

	// IMU::Calib extrinsic_para(T_gyros_c_cv, T_dvl_c_cv);

	const int N = vpKFs.size();
	for (size_t i = 0; i < N; i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}
		pKFi->mImuCalib.SetExtrinsic(T_gyros_c_cv, T_dvl_c_cv);

		if (cv::norm(pKFi->GetGyroBias() - cvbg) > 0.01) {
			pKFi->SetNewBias(b);
			if (pKFi->mpDvlPreintegrationKeyFrame) {
				pKFi->mpDvlPreintegrationKeyFrame->ReintegrateWithVelocity();
			}
		}
		else {
			pKFi->SetNewBias(b);
		}
	}

}
void Optimizer::DvlGyroInitOptimization2(Map *pMap,
                                         Eigen::Vector3d &bg,
                                         bool bMono,
                                         float priorG)
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
	vector<VertexGyroBias *> vpgb;
	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}
		VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
		VP->setId(pKFi->mnId);
		VP->setFixed(true);
		optimizer.addVertex(VP);

		// Biases
		VertexGyroBias *VG = new VertexGyroBias(pKFi);
		VG->setId(maxKFid + 1 + pKFi->mnId);
		VG->setFixed(true);
		optimizer.addVertex(VG);
		vpgb.push_back(VG);
	}



	// prior acc bias
	//		EdgePriorGyro *epg = new EdgePriorGyro(cv::Mat::zeros(3, 1, CV_32F));
	//		epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
	//		double infoPriorG = priorG;
	//		epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
	//		optimizer.addEdge(epg);

	// extrinsic parameter
	g2o::VertexSE3Expmap *vT_d_c = new g2o::VertexSE3Expmap();
	vT_d_c->setEstimate(Converter::toSE3Quat(vpKFs[0]->mImuCalib.mT_dvl_c));
	vT_d_c->setId(maxKFid + 1 + maxKFid + 1);
	vT_d_c->setFixed(false);
	optimizer.addVertex(vT_d_c);

	g2o::VertexSE3Expmap *vT_g_d = new g2o::VertexSE3Expmap();
	vT_g_d->setEstimate(Converter::toSE3Quat(vpKFs[0]->mImuCalib.mT_gyro_dvl));
	vT_g_d->setId(maxKFid + 1 + maxKFid + 2);
	vT_g_d->setFixed(true);
	optimizer.addVertex(vT_g_d);


	// Graph edges
	vector<EdgeDvlGyroInit *> vpei;
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
			if (!pKFi->mpDvlPreintegrationKeyFrame) {
				std::cout << "Not preintegrated measurement" << std::endl;
			}

			pKFi->mpDvlPreintegrationKeyFrame->SetNewBias(pKFi->mPrevKF->GetImuBias());
			VertexPoseDvlIMU *VP1 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mPrevKF->mnId));
			//				g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
			VertexPoseDvlIMU *VP2 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
			//				g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
			g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid + 1 + pKFi->mPrevKF->mnId);
			g2o::HyperGraph::Vertex *VT_d_c = optimizer.vertex(maxKFid + 1 + maxKFid + 1);
			g2o::HyperGraph::Vertex *VT_g_d = optimizer.vertex(maxKFid + 1 + maxKFid + 2);
			//				g2o::HyperGraph::Vertex *VA = optimizer.vertex(maxKFid * 2 + 3);
			//				g2o::HyperGraph::Vertex *VGDir = optimizer.vertex(maxKFid * 2 + 4);
			//				g2o::HyperGraph::Vertex *VS = optimizer.vertex(maxKFid * 2 + 5);
			//				cout<<"VP1: Rcw[0]"<<VP1->estimate().Rcw[0]<<endl;
			//				cout<<"VP1: Rwc"<<VP1->estimate().Rwc<<endl;
			//				cout<<"VP1: tcw[0]"<<VP1->estimate().tcw[0]<<endl;
			//				cout<<"VP1: twc"<<VP1->estimate().twc<<endl;
			//
			//				cout<<"VP2: Rcw[0]"<<VP2->estimate().Rcw[0]<<endl;
			//				cout<<"VP2: Rwc"<<VP2->estimate().Rwc<<endl;
			//				cout<<"VP2: tcw[0]"<<VP2->estimate().tcw[0]<<endl;
			//				cout<<"VP2: twc"<<VP2->estimate().twc<<endl;

			if (!VP1 || !VG || !VP2) {
				cout << "Error" << VP1 << ", " << VG << ", " << VP2 << endl;

				continue;
			}
			//				EdgeInertialGS *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
			EdgeDvlGyroInit *ei = new EdgeDvlGyroInit(pKFi->mpDvlPreintegrationKeyFrame);
			//				ei->setVertex(0, VP1);

			//			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
			//			ei->setRobustKernel(rk);
			//			rk->setDelta(sqrt(7.815));
			ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
			ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
			ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
			ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
			ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
			ei->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100000);
			ei->setId(pKFi->mnId);


			vpei.push_back(ei);

			vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
			optimizer.addEdge(ei);
		}
	}

	// Compute error for different scales
	std::set<g2o::HyperGraph::Edge *> setEdges = optimizer.edges();
	double total_error = 0;

	const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
	const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
	optimizer.setVerbose(true);

	int mono_outlier;
	int stereo_outlier;
	float total_chi2;
	for (int i = 0; i < 4; i++) {
		if (i > 0) {
			vT_g_d->setFixed(true);
		}
//		if (i>1){
//			for(auto v_gb:vpgb){
//				v_gb->setFixed(false);
//			}
//		}

		optimizer.initializeOptimization(0);
		optimizer.optimize(10);
//		for (auto ei:vpei) {
//			ei->computeError();
//			cout << "calibration optimization iteration : " << i << "edge of " << ei->id() - 1 << " and " << ei->id()
//				 << "\nchi2:" << ei->chi2() << endl;
//		}

	}

//	std::cout << "start optimization" << std::endl;
//	optimizer.setVerbose(true);
//	optimizer.initializeOptimization();
//	optimizer.optimize(its);
//
//	std::cout << "end optimization" << std::endl;


	// Recover optimized data
	// Biases
	for (auto pkf: vpKFs) {
		int kf_id = pkf->mnId;
		int bias_vertex_id = kf_id + maxKFid + 1;
		VertexGyroBias *v_gb = dynamic_cast<VertexGyroBias *>(optimizer.vertex(bias_vertex_id));
		bg << v_gb->estimate();
		IMU::Bias b(0, 0, 0, bg[0], bg[1], bg[2]);

//		cout << "kf id: " << pkf->mnId << " gyros bias: " << bg.transpose() << endl;


		cv::Mat cvbg;
		cv::eigen2cv(bg, cvbg);
		cvbg.convertTo(cvbg, CV_32F);
		if (cv::norm(pkf->GetGyroBias() - cvbg) > 0.01) {
			pkf->SetNewBias(b);
			if (pkf->mpDvlPreintegrationKeyFrame) {
				pkf->mpDvlPreintegrationKeyFrame->ReintegrateWithVelocity();
			}
		}
		else {
			pkf->SetNewBias(b);
		}
		pkf->SetNewBias(b);
	}


	Eigen::Isometry3d T_dvl_c = vT_d_c->estimate();
	Eigen::Isometry3d T_gyros_dvl = vT_g_d->estimate();

	Eigen::Matrix3d R_gt;
	R_gt << 0, 0, 1,
		-1, 0, 0,
		0, -1, 0;
	cout << "init optimization result: \n"
	     << "R_dvl_c:\n" << T_dvl_c.rotation() << "\n"
	     << "R_dvl_c(eular yaw-pitch-roll):" << T_dvl_c.rotation().eulerAngles(2, 1, 0).transpose() << "\n"
	     << "t_dvl_c:" << T_dvl_c.translation().transpose() << "\n"
	     << "R_dvl_c distance with R_gt(LogSO3()): " << LogSO3(T_dvl_c.rotation().inverse() * R_gt).transpose() << "\n"
	     << "R_gyros_dvl:\n" << T_gyros_dvl.rotation() << "\n"
	     << "R_gyros_dvl(eular yaw-pitch-roll):" << T_gyros_dvl.rotation().eulerAngles(2, 1, 0).transpose() << "\n"
	     << endl;


	cv::Mat cvbg = Converter::toCvMat(bg);

	//Keyframes velocities and biases
	std::cout << "update Keyframes biases and extrinsic paramters" << std::endl;

	cv::Mat T_dvl_c_cv, T_gyros_dvl_cv, T_gyros_c_cv;
	cv::eigen2cv(T_dvl_c.matrix(), T_dvl_c_cv);
	T_dvl_c_cv.convertTo(T_dvl_c_cv, CV_32FC1);
	cv::eigen2cv(T_gyros_dvl.matrix(), T_gyros_dvl_cv);
	T_gyros_dvl_cv.convertTo(T_gyros_dvl_cv, CV_32FC1);
	Eigen::Isometry3d T_gyros_c = T_gyros_dvl * T_dvl_c;
	cv::eigen2cv(T_gyros_c.matrix(), T_gyros_c_cv);
	T_gyros_c_cv.convertTo(T_gyros_c_cv, CV_32FC1);

	// IMU::Calib extrinsic_para(T_gyros_c_cv, T_dvl_c_cv);

	const int N = vpKFs.size();
	for (size_t i = 0; i < N; i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}
        pKFi->mImuCalib.SetExtrinsic(T_gyros_c_cv, T_dvl_c_cv);
	}

}
void Optimizer::DvlGyroInitOptimization3(Map *pMap,
                                         Eigen::Vector3d &bg,
                                         bool bMono,
                                         float priorG)
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
		solver->setUserLambdaInit(100);
	}

	optimizer.setAlgorithm(solver);

	// Set KeyFrame vertices (fixed poses and optimizable velocities)
	vector<VertexGyroBias *> vpgb;
	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}
		VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
		VP->setId(pKFi->mnId);
		VP->setFixed(true);
		optimizer.addVertex(VP);

		// Biases
		VertexGyroBias *VG = new VertexGyroBias(pKFi);
		VG->setId(maxKFid + 1 + pKFi->mnId);
		VG->setFixed(true);
		optimizer.addVertex(VG);
		vpgb.push_back(VG);
	}



	// prior acc bias
	//		EdgePriorGyro *epg = new EdgePriorGyro(cv::Mat::zeros(3, 1, CV_32F));
	//		epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
	//		double infoPriorG = priorG;
	//		epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
	//		optimizer.addEdge(epg);

	// extrinsic parameter
	g2o::VertexSE3Expmap *vT_d_c = new g2o::VertexSE3Expmap();
	vT_d_c->setEstimate(Converter::toSE3Quat(vpKFs[0]->mImuCalib.mT_dvl_c));
	vT_d_c->setId(maxKFid + 1 + maxKFid + 1);
	vT_d_c->setFixed(false);
	optimizer.addVertex(vT_d_c);

	g2o::VertexSE3Expmap *vT_g_d = new g2o::VertexSE3Expmap();
	vT_g_d->setEstimate(Converter::toSE3Quat(vpKFs[0]->mImuCalib.mT_gyro_dvl));
	vT_g_d->setId(maxKFid + 1 + maxKFid + 2);
	vT_g_d->setFixed(true);
	optimizer.addVertex(vT_g_d);


	// Graph edges
	vector<EdgeDvlGyroInit *> vpei;
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
			if (!pKFi->mpDvlPreintegrationKeyFrame) {
				std::cout << "Not preintegrated measurement" << std::endl;
			}

			pKFi->mpDvlPreintegrationKeyFrame->SetNewBias(pKFi->mPrevKF->GetImuBias());
			VertexPoseDvlIMU *VP1 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mPrevKF->mnId));
			//				g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
			VertexPoseDvlIMU *VP2 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
			//				g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
			g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid + 1 + pKFi->mPrevKF->mnId);
			g2o::HyperGraph::Vertex *VT_d_c = optimizer.vertex(maxKFid + 1 + maxKFid + 1);
			g2o::HyperGraph::Vertex *VT_g_d = optimizer.vertex(maxKFid + 1 + maxKFid + 2);
			//				g2o::HyperGraph::Vertex *VA = optimizer.vertex(maxKFid * 2 + 3);
			//				g2o::HyperGraph::Vertex *VGDir = optimizer.vertex(maxKFid * 2 + 4);
			//				g2o::HyperGraph::Vertex *VS = optimizer.vertex(maxKFid * 2 + 5);
			//				cout<<"VP1: Rcw[0]"<<VP1->estimate().Rcw[0]<<endl;
			//				cout<<"VP1: Rwc"<<VP1->estimate().Rwc<<endl;
			//				cout<<"VP1: tcw[0]"<<VP1->estimate().tcw[0]<<endl;
			//				cout<<"VP1: twc"<<VP1->estimate().twc<<endl;
			//
			//				cout<<"VP2: Rcw[0]"<<VP2->estimate().Rcw[0]<<endl;
			//				cout<<"VP2: Rwc"<<VP2->estimate().Rwc<<endl;
			//				cout<<"VP2: tcw[0]"<<VP2->estimate().tcw[0]<<endl;
			//				cout<<"VP2: twc"<<VP2->estimate().twc<<endl;

			if (!VP1 || !VG || !VP2) {
				cout << "Error" << VP1 << ", " << VG << ", " << VP2 << endl;

				continue;
			}
			//				EdgeInertialGS *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
			EdgeDvlGyroInit *ei = new EdgeDvlGyroInit(pKFi->mpDvlPreintegrationKeyFrame);
			//				ei->setVertex(0, VP1);

			//			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
			//			ei->setRobustKernel(rk);
			//			rk->setDelta(sqrt(7.815));
			ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
			ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
			ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
			ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
			ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
			ei->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100000);
			ei->setId(pKFi->mnId);


			vpei.push_back(ei);

			vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
			optimizer.addEdge(ei);
		}
	}


	optimizer.setVerbose(true);
	std::set<g2o::HyperGraph::Edge *> setEdges = optimizer.edges();
	double total_error = 0;

	const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
	const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};

	int mono_outlier;
	int stereo_outlier;
	float total_chi2;
	for (int i = 0; i < 4; i++) {
		if (i > 1) {
			vT_g_d->setFixed(false);
		}

		optimizer.initializeOptimization(0);
		optimizer.optimize(10);
//		for (auto ei:vpei) {
//			ei->computeError();
//			cout << "calibration optimization iteration : " << i << "edge of " << ei->id() - 1 << " and " << ei->id()
//				 << "\nchi2:" << ei->chi2() << endl;
//		}

	}

	//	std::cout << "start optimization" << std::endl;
	//	optimizer.setVerbose(true);
	//	optimizer.initializeOptimization();
	//	optimizer.optimize(its);
	//
	//	std::cout << "end optimization" << std::endl;


	// Recover optimized data
	// Biases
	for (auto pkf: vpKFs) {
		int kf_id = pkf->mnId;
		int bias_vertex_id = kf_id + maxKFid + 1;
		VertexGyroBias *v_gb = dynamic_cast<VertexGyroBias *>(optimizer.vertex(bias_vertex_id));
		bg << v_gb->estimate();
		IMU::Bias b(0, 0, 0, bg[0], bg[1], bg[2]);

//		cout << "kf id: " << pkf->mnId << " gyros bias: " << bg.transpose() << endl;


		cv::Mat cvbg;
		cv::eigen2cv(bg, cvbg);
		cvbg.convertTo(cvbg, CV_32F);
//		if (cv::norm(pkf->GetGyroBias() - cvbg) > 0.01) {
//			pkf->SetNewBias(b);
//			if (pkf->mpDvlPreintegrationKeyFrame) {
//				pkf->mpDvlPreintegrationKeyFrame->ReintegrateWithVelocity();
//			}
//		}
//		else {
//			pkf->SetNewBias(b);
//		}
		pkf->SetNewBias(b);
	}


	Eigen::Isometry3d T_dvl_c = vT_d_c->estimate();
	Eigen::Quaterniond q_dvl_c(T_dvl_c.rotation());
	Eigen::Isometry3d T_gyros_dvl = vT_g_d->estimate();
	Eigen::Isometry3d T_gyros_c = T_gyros_dvl * T_dvl_c;
	Eigen::Quaterniond q_gyros_c(T_gyros_c.rotation());

	Eigen::Isometry3d T_dvl_c_gt = Eigen::Isometry3d::Identity();
	Eigen::Matrix3d R_dvl_c_gt;
	R_dvl_c_gt << 0, 1, 0, -1, 0, 0, 0, 0, 1;
	Eigen::Vector3d t_dvl_c_gt(-0.88, -0.348, 1.056);
	T_dvl_c_gt.rotate(R_dvl_c_gt);
	T_dvl_c_gt.pretranslate(t_dvl_c_gt);

	Eigen::Isometry3d T_gyros_c_gt = Eigen::Isometry3d::Identity();
	Eigen::Matrix3d R_gyros_c_gt;
	R_gyros_c_gt << 0, 0, 1, -1, 0, 0, 0, -1, 0;
	Eigen::Vector3d t_gyros_c_gt(0, 0, 0);
	T_gyros_c_gt.rotate(R_gyros_c_gt);
	T_gyros_c_gt.pretranslate(t_gyros_c_gt);

	Eigen::Vector3d err_R_dvl_c = LogSO3(T_dvl_c.rotation().inverse() * T_dvl_c_gt.rotation());
	Eigen::Vector3d err_R_gyros_c = LogSO3(T_gyros_c.rotation().inverse() * T_gyros_c_gt.rotation());
	Eigen::Vector3d err_t_dvl_c = T_dvl_c.translation() - T_dvl_c_gt.translation();
	Eigen::Vector3d err_t_gyros_c = T_gyros_c.translation() - T_gyros_c_gt.translation();

	stringstream ss;
	ss << "result_" << ros::Time::now().toNSec() << ".txt";
	ofstream f("/home/da/project/ros/orb_dvl2_ws/src/dvl2/calibration_results/" + ss.str());

	if (f.is_open()) {
		f << fixed;
		f
			<< "#T_dvl_camera: tranlation(x y z) quaternion(x y z w) tranlation_error(x y z) rotation_error(LogSO3 x y z)\n";
		f << setprecision(9) << T_dvl_c.translation().x() << " " << T_dvl_c.translation().y() << " "
		  << T_dvl_c.translation().z() << " " << q_dvl_c.x() << " " << q_dvl_c.y() << " " << q_dvl_c.z() << " "
		  << q_dvl_c.w() << "\n" << err_t_dvl_c.x() << " " << err_t_dvl_c.y() << " " << err_t_dvl_c.z() << "\n"
		  << err_R_dvl_c.x() << " " << err_R_dvl_c.y() << " " << err_R_dvl_c.z() << " " << endl;
		f << "#T_gyroscope_camera: quaternion(x y z w) rotation_error(LogSO3 x y z)\n";
		f << setprecision(9) << q_gyros_c.x() << " " << q_gyros_c.y() << " " << q_gyros_c.z() << " " << q_gyros_c.w()
		  << "\n" << err_R_gyros_c.x() << " " << err_R_gyros_c.y() << " " << err_R_gyros_c.z() << " " << endl;
	}


	cout << "init optimization result: \n"
	     << "bias_gyro:\n" << bg << "\n"
	     << "R_dvl_c:\n" << T_dvl_c.rotation() << "\n"
	     << "t_dvl_c:" << T_dvl_c.translation().transpose() << "\n"
	     << "R_gyros_c:\n" << T_gyros_c.rotation() << "\n"
	     << endl;


	cv::Mat cvbg = Converter::toCvMat(bg);

	//Keyframes velocities and biases
	std::cout << "update Keyframes biases and extrinsic paramters" << std::endl;

	cv::Mat T_dvl_c_cv, T_gyros_dvl_cv, T_gyros_c_cv;
	cv::eigen2cv(T_dvl_c.matrix(), T_dvl_c_cv);
	T_dvl_c_cv.convertTo(T_dvl_c_cv, CV_32FC1);
	cv::eigen2cv(T_gyros_dvl.matrix(), T_gyros_dvl_cv);
	T_gyros_dvl_cv.convertTo(T_gyros_dvl_cv, CV_32FC1);
	T_gyros_c = T_gyros_dvl * T_dvl_c;
	cv::eigen2cv(T_gyros_c.matrix(), T_gyros_c_cv);
	T_gyros_c_cv.convertTo(T_gyros_c_cv, CV_32FC1);

	// IMU::Calib extrinsic_para(T_gyros_c_cv, T_dvl_c_cv);

	const int N = vpKFs.size();
	for (size_t i = 0; i < N; i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}
        pKFi->mImuCalib.SetExtrinsic(T_gyros_c_cv, T_dvl_c_cv);
	}

}

void Optimizer::DvlGyroInitOptimization5(Map *pMap, Eigen::Vector3d &bg, bool bMono, float priorG)
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
		solver->setUserLambdaInit(100);
	}

	optimizer.setAlgorithm(solver);

	// Set KeyFrame vertices (fixed poses and optimizable velocities)
	vector<VertexGyroBias *> vpgb;
	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}
		VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
		VP->setId(pKFi->mnId);
		VP->setFixed(true);
		optimizer.addVertex(VP);

		// Biases
		VertexGyroBias *VG = new VertexGyroBias(pKFi);
		VG->setId(maxKFid + 1 + pKFi->mnId);
		VG->setFixed(true);
		optimizer.addVertex(VG);
		vpgb.push_back(VG);
	}



	// prior acc bias
	//		EdgePriorGyro *epg = new EdgePriorGyro(cv::Mat::zeros(3, 1, CV_32F));
	//		epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
	//		double infoPriorG = priorG;
	//		epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
	//		optimizer.addEdge(epg);

	// extrinsic parameter
	g2o::VertexSE3Expmap *vT_d_c = new g2o::VertexSE3Expmap();
	vT_d_c->setEstimate(Converter::toSE3Quat(vpKFs[0]->mImuCalib.mT_dvl_c));
	vT_d_c->setId(maxKFid + 1 + maxKFid + 1);
	vT_d_c->setFixed(true);
	optimizer.addVertex(vT_d_c);

	g2o::VertexSE3Expmap *vT_g_d = new g2o::VertexSE3Expmap();
	vT_g_d->setEstimate(Converter::toSE3Quat(vpKFs[0]->mImuCalib.mT_gyro_dvl));
	vT_g_d->setId(maxKFid + 1 + maxKFid + 2);
	vT_g_d->setFixed(true);
	optimizer.addVertex(vT_g_d);

	VertexDVLBeamOritenstion *v_beam_ori = new VertexDVLBeamOritenstion();
	Eigen::Matrix<double, 8, 1> alpha_beta;
//	alpha_beta << 67.5 / 180.0 * M_PI,
//		67.5 / 180.0 * M_PI,
//		67.5 / 180.0 * M_PI,
//		67.5 / 180.0 * M_PI,
//		45 / 180.0 * M_PI,
//		45 / 180.0 * M_PI,
//		45 / 180.0 * M_PI,
//		45 / 180.0 * M_PI;
	alpha_beta << 1 / 180.0 * M_PI,
		1 / 180.0 * M_PI,
		1 / 180.0 * M_PI,
		1 / 180.0 * M_PI,
		45 / 180.0 * M_PI,
		45 / 180.0 * M_PI,
		45 / 180.0 * M_PI,
		45 / 180.0 * M_PI;
	v_beam_ori->setFixed(false);
	v_beam_ori->setId(maxKFid + 1 + maxKFid + 3);
	v_beam_ori->setEstimate(alpha_beta);
	optimizer.addVertex(v_beam_ori);

	// Graph edges
	vector<EdgeDvlGyroInit2 *> vpei;
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
			if (!pKFi->mpDvlPreintegrationKeyFrame) {
				std::cout << "Not preintegrated measurement" << std::endl;
			}

			pKFi->mpDvlPreintegrationKeyFrame->SetNewBias(pKFi->mPrevKF->GetImuBias());
			VertexPoseDvlIMU *VP1 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mPrevKF->mnId));
			//				g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
			VertexPoseDvlIMU *VP2 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
			//				g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
			g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid + 1 + pKFi->mPrevKF->mnId);
			g2o::HyperGraph::Vertex *VT_d_c = optimizer.vertex(maxKFid + 1 + maxKFid + 1);
			g2o::HyperGraph::Vertex *VT_g_d = optimizer.vertex(maxKFid + 1 + maxKFid + 2);
			g2o::HyperGraph::Vertex *Valpha_beta = optimizer.vertex(maxKFid + 1 + maxKFid + 3);

			if (!VP1 || !VG || !VP2) {
				cout << "Error" << VP1 << ", " << VG << ", " << VP2 << endl;

				continue;
			}
			//				EdgeInertialGS *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
			EdgeDvlGyroInit2 *ei = new EdgeDvlGyroInit2(pKFi->mpDvlPreintegrationKeyFrame);
			//				ei->setVertex(0, VP1);

			//			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
			//			ei->setRobustKernel(rk);
			//			rk->setDelta(sqrt(7.815));
			ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
			ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
			ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
			ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
			ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
			ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(Valpha_beta));
			ei->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100000);
			ei->setId(pKFi->mnId);


			vpei.push_back(ei);

			vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
			optimizer.addEdge(ei);
		}
	}


	optimizer.setVerbose(true);
	std::set<g2o::HyperGraph::Edge *> setEdges = optimizer.edges();
	double total_error = 0;

	const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
	const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};

	int mono_outlier;
	int stereo_outlier;
	float total_chi2;
	for (int i = 0; i < 4; i++) {
//		if (i > 1) {
//			vT_g_d->setFixed(false);
//		}

		optimizer.initializeOptimization(0);
		optimizer.optimize(10);
//		for (auto ei:vpei) {
//			ei->computeError();
//			cout << "calibration optimization iteration : " << i << "edge of " << ei->id() - 1 << " and " << ei->id()
//				 << "\nchi2:" << ei->chi2() << endl;
//		}

	}

	//	std::cout << "start optimization" << std::endl;
	//	optimizer.setVerbose(true);
	//	optimizer.initializeOptimization();
	//	optimizer.optimize(its);
	//
	//	std::cout << "end optimization" << std::endl;


	// Recover optimized data
	// Biases
	for (auto pkf: vpKFs) {
		int kf_id = pkf->mnId;
		int bias_vertex_id = kf_id + maxKFid + 1;
		VertexGyroBias *v_gb = dynamic_cast<VertexGyroBias *>(optimizer.vertex(bias_vertex_id));
		bg << v_gb->estimate();
		IMU::Bias b(0, 0, 0, bg[0], bg[1], bg[2]);

//		cout << "kf id: " << pkf->mnId << " gyros bias: " << bg.transpose() << endl;


		cv::Mat cvbg;
		cv::eigen2cv(bg, cvbg);
		cvbg.convertTo(cvbg, CV_32F);
//		if (cv::norm(pkf->GetGyroBias() - cvbg) > 0.01) {
//			pkf->SetNewBias(b);
//			if (pkf->mpDvlPreintegrationKeyFrame) {
//				pkf->mpDvlPreintegrationKeyFrame->ReintegrateWithVelocity();
//			}
//		}
//		else {
//			pkf->SetNewBias(b);
//		}
		pkf->SetNewBias(b);
	}


	Eigen::Isometry3d T_dvl_c = vT_d_c->estimate();
	Eigen::Quaterniond q_dvl_c(T_dvl_c.rotation());
	Eigen::Isometry3d T_gyros_dvl = vT_g_d->estimate();
	Eigen::Isometry3d T_gyros_c = T_gyros_dvl * T_dvl_c;
	Eigen::Quaterniond q_gyros_c(T_gyros_c.rotation());

	Eigen::Isometry3d T_dvl_c_gt = Eigen::Isometry3d::Identity();
	Eigen::Matrix3d R_dvl_c_gt;
	R_dvl_c_gt << 0, 1, 0, -1, 0, 0, 0, 0, 1;
	Eigen::Vector3d t_dvl_c_gt(-0.88, -0.348, 1.056);
	T_dvl_c_gt.rotate(R_dvl_c_gt);
	T_dvl_c_gt.pretranslate(t_dvl_c_gt);

	Eigen::Isometry3d T_gyros_c_gt = Eigen::Isometry3d::Identity();
	Eigen::Matrix3d R_gyros_c_gt;
	R_gyros_c_gt << 0, 0, 1, -1, 0, 0, 0, -1, 0;
	Eigen::Vector3d t_gyros_c_gt(0, 0, 0);
	T_gyros_c_gt.rotate(R_gyros_c_gt);
	T_gyros_c_gt.pretranslate(t_gyros_c_gt);

	Eigen::Vector3d err_R_dvl_c = LogSO3(T_dvl_c.rotation().inverse() * T_dvl_c_gt.rotation());
	Eigen::Vector3d err_R_gyros_c = LogSO3(T_gyros_c.rotation().inverse() * T_gyros_c_gt.rotation());
	Eigen::Vector3d err_t_dvl_c = T_dvl_c.translation() - T_dvl_c_gt.translation();
	Eigen::Vector3d err_t_gyros_c = T_gyros_c.translation() - T_gyros_c_gt.translation();

	stringstream ss;
	ss << "result_" << ros::Time::now().toNSec() << ".txt";
	ofstream f("/home/da/project/ros/orb_dvl2_ws/src/dvl2/calibration_results/" + ss.str());

	if (f.is_open()) {
		f << fixed;
		f
			<< "#T_dvl_camera: tranlation(x y z) quaternion(x y z w) tranlation_error(x y z) rotation_error(LogSO3 x y z)\n";
		f << setprecision(9) << T_dvl_c.translation().x() << " " << T_dvl_c.translation().y() << " "
		  << T_dvl_c.translation().z() << " " << q_dvl_c.x() << " " << q_dvl_c.y() << " " << q_dvl_c.z() << " "
		  << q_dvl_c.w() << "\n" << err_t_dvl_c.x() << " " << err_t_dvl_c.y() << " " << err_t_dvl_c.z() << "\n"
		  << err_R_dvl_c.x() << " " << err_R_dvl_c.y() << " " << err_R_dvl_c.z() << " " << endl;
		f << "#T_gyroscope_camera: quaternion(x y z w) rotation_error(LogSO3 x y z)\n";
		f << setprecision(9) << q_gyros_c.x() << " " << q_gyros_c.y() << " " << q_gyros_c.z() << " " << q_gyros_c.w()
		  << "\n" << err_R_gyros_c.x() << " " << err_R_gyros_c.y() << " " << err_R_gyros_c.z() << " " << endl;
	}

	alpha_beta = v_beam_ori->estimate();

	cout << "init optimization result: \n"
	     << "bias_gyro:\n" << bg << "\n"
	     << "R_dvl_c:\n" << T_dvl_c.rotation() << "\n"
	     << "t_dvl_c:" << T_dvl_c.translation().transpose() << "\n"
	     << "R_gyros_c:\n" << T_gyros_c.rotation() << "\n"
	     << "alpha_beta:\n" << alpha_beta.transpose() << "\n"
	     << endl;


	cv::Mat cvbg = Converter::toCvMat(bg);

	//Keyframes velocities and biases
	std::cout << "update Keyframes biases and extrinsic paramters" << std::endl;

	cv::Mat T_dvl_c_cv, T_gyros_dvl_cv, T_gyros_c_cv;
	cv::eigen2cv(T_dvl_c.matrix(), T_dvl_c_cv);
	T_dvl_c_cv.convertTo(T_dvl_c_cv, CV_32FC1);
	cv::eigen2cv(T_gyros_dvl.matrix(), T_gyros_dvl_cv);
	T_gyros_dvl_cv.convertTo(T_gyros_dvl_cv, CV_32FC1);
	T_gyros_c = T_gyros_dvl * T_dvl_c;
	cv::eigen2cv(T_gyros_c.matrix(), T_gyros_c_cv);
	T_gyros_c_cv.convertTo(T_gyros_c_cv, CV_32FC1);

	// IMU::Calib extrinsic_para(T_gyros_c_cv, T_dvl_c_cv);

	const int N = vpKFs.size();
	for (size_t i = 0; i < N; i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}
        pKFi->mImuCalib.SetExtrinsic(T_gyros_c_cv, T_dvl_c_cv);
	}

}

void Optimizer::DvlGyroInitOptimization6(Map *pMap, Eigen::Vector3d &bg, bool bMono, float priorG)
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
		solver->setUserLambdaInit(100);
	}

	optimizer.setAlgorithm(solver);

	// Set KeyFrame vertices (fixed poses and optimizable velocities)
	vector<VertexGyroBias *> vpgb;
	vector<VertexVelocity *> vpv;
	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}
		VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
		VP->setId(pKFi->mnId);
		VP->setFixed(true);
		optimizer.addVertex(VP);

		// Biases
		VertexGyroBias *VG = new VertexGyroBias(pKFi);
		VG->setId(maxKFid + 1 + pKFi->mnId);
		VG->setFixed(true);
		optimizer.addVertex(VG);
		vpgb.push_back(VG);

		VertexVelocity *VV = new VertexVelocity();
		Eigen::Vector3d v(0.1, 0.1, 0.1);
		VV->setEstimate(v);
		VV->setId(maxKFid + 1 + maxKFid + 1 + pKFi->mnId);
		VV->setFixed(false);
		optimizer.addVertex(VV);
		vpv.push_back(VV);

	}



	// prior acc bias
	//		EdgePriorGyro *epg = new EdgePriorGyro(cv::Mat::zeros(3, 1, CV_32F));
	//		epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
	//		double infoPriorG = priorG;
	//		epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
	//		optimizer.addEdge(epg);

	// extrinsic parameter
	g2o::VertexSE3Expmap *vT_d_c = new g2o::VertexSE3Expmap();
	vT_d_c->setEstimate(Converter::toSE3Quat(vpKFs[0]->mImuCalib.mT_dvl_c));
	vT_d_c->setId(maxKFid + 1 + maxKFid + 1 + maxKFid + 1);
	vT_d_c->setFixed(true);
	optimizer.addVertex(vT_d_c);

	g2o::VertexSE3Expmap *vT_g_d = new g2o::VertexSE3Expmap();
	vT_g_d->setEstimate(Converter::toSE3Quat(vpKFs[0]->mImuCalib.mT_gyro_dvl));
	vT_g_d->setId(maxKFid + 1 + maxKFid + 1 + maxKFid + 2);
	vT_g_d->setFixed(true);
	optimizer.addVertex(vT_g_d);

	VertexDVLBeamOritenstion *v_beam_ori = new VertexDVLBeamOritenstion();
//	Eigen::Matrix<double,8,1> alpha_beta;
//	alpha_beta << 67.5 / 180.0 * M_PI,
//		67.5 / 180.0 * M_PI,
//		67.5 / 180.0 * M_PI,
//		67.5 / 180.0 * M_PI,
//		45 / 180.0 * M_PI,
//		45 / 180.0 * M_PI,
//		45 / 180.0 * M_PI,
//		45 / 180.0 * M_PI;
//	alpha_beta << 1 / 180.0 * M_PI,
//		1 / 180.0 * M_PI,
//		1 / 180.0 * M_PI,
//		1 / 180.0 * M_PI,
//		45 / 180.0 * M_PI,
//		45 / 180.0 * M_PI,
//		45 / 180.0 * M_PI,
//		45 / 180.0 * M_PI;
//	v_beam_ori->setFixed(false);
//	v_beam_ori->setId(maxKFid + 1 + maxKFid + 1 + maxKFid + 3);
//	v_beam_ori->setEstimate(alpha_beta);
//	optimizer.addVertex(v_beam_ori);

	// Graph edges
	vector<EdgeDvlGyroInit3 *> vpei;
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
			if (!pKFi->mpDvlPreintegrationKeyFrame) {
				std::cout << "Not preintegrated measurement" << std::endl;
			}

			pKFi->mpDvlPreintegrationKeyFrame->SetNewBias(pKFi->mPrevKF->GetImuBias());
			VertexPoseDvlIMU *VP1 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mPrevKF->mnId));
			//				g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
			VertexPoseDvlIMU *VP2 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
			//				g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
			g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid + 1 + pKFi->mPrevKF->mnId);
			g2o::HyperGraph::Vertex *VV = optimizer.vertex(maxKFid + 1 + maxKFid + 1 + pKFi->mnId);
			g2o::HyperGraph::Vertex *VT_d_c = optimizer.vertex(maxKFid + 1 + maxKFid + 1 + maxKFid + 1);
			g2o::HyperGraph::Vertex *VT_g_d = optimizer.vertex(maxKFid + 1 + maxKFid + 1 + maxKFid + 2);
//			g2o::HyperGraph::Vertex *Valpha_beta = optimizer.vertex(maxKFid + 1 + maxKFid + 1 + maxKFid + 3);

			if (!VP1 || !VG || !VP2) {
				cout << "Error" << VP1 << ", " << VG << ", " << VP2 << endl;

				continue;
			}
			//				EdgeInertialGS *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
			EdgeDvlGyroInit3 *ei = new EdgeDvlGyroInit3(pKFi->mpDvlPreintegrationKeyFrame);
			//				ei->setVertex(0, VP1);

			//			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
			//			ei->setRobustKernel(rk);
			//			rk->setDelta(sqrt(7.815));
			ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
			ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
			ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
			ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV));
			ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
			ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
			ei->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100000);
			ei->setId(pKFi->mnId);


			vpei.push_back(ei);

			vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
			optimizer.addEdge(ei);
		}
	}


	optimizer.setVerbose(true);
	std::set<g2o::HyperGraph::Edge *> setEdges = optimizer.edges();
	double total_error = 0;

	const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
	const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};

	int mono_outlier;
	int stereo_outlier;
	float total_chi2;
	for (int i = 0; i < 4; i++) {
//		if (i > 1) {
//			vT_g_d->setFixed(false);
//		}

		optimizer.initializeOptimization();
		optimizer.optimize(10);
//		for (auto ei:vpei) {
//			ei->computeError();
//			cout << "calibration optimization iteration : " << i << "edge of " << ei->id() - 1 << " and " << ei->id()
//				 << "\nchi2:" << ei->chi2() << endl;
//		}

	}

	//	std::cout << "start optimization" << std::endl;
	//	optimizer.setVerbose(true);
	//	optimizer.initializeOptimization();
	//	optimizer.optimize(its);
	//
	//	std::cout << "end optimization" << std::endl;


	// Recover optimized data
	// Biases
	for (auto pkf: vpKFs) {
		int kf_id = pkf->mnId;
		int bias_vertex_id = kf_id + maxKFid + 1;
		int velocity_vertex_id = kf_id + maxKFid + 1 + maxKFid + 1;
		VertexGyroBias *v_gb = dynamic_cast<VertexGyroBias *>(optimizer.vertex(bias_vertex_id));
		bg << v_gb->estimate();
		IMU::Bias b(0, 0, 0, bg[0], bg[1], bg[2]);

		VertexVelocity *v_v = dynamic_cast<VertexVelocity *>(optimizer.vertex(velocity_vertex_id));
		Eigen::Vector3d v_dk = v_v->estimate();
		pkf->mpDvlPreintegrationKeyFrame->v_dk_visual = v_dk;

//		cout << "kf id: " << pkf->mnId << " gyros bias: " << bg.transpose() << endl;
		ROS_INFO_STREAM("beam calibration: KeyFrame id:<<" << pkf->mnId << " dvl velocity: "
		                                                   << pkf->mpDvlPreintegrationKeyFrame->v_dk_dvl
		                                                   << " visual velocity: "
		                                                   << pkf->mpDvlPreintegrationKeyFrame->v_dk_visual.transpose());
	}

	DvlBeamOptimization(pMap);
	DvlBeamOptimization_dvl(pMap);




}

void Optimizer::DvlGyroInitOptimization4(Map *pMap,
                                         Eigen::Vector3d &bg,
                                         bool bMono,
                                         float priorG)
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
	vector<VertexGyroBias *> vpgb;
	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}
		VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
		VP->setId(pKFi->mnId);
		VP->setFixed(true);
		optimizer.addVertex(VP);

		// Biases
		VertexGyroBias *VG = new VertexGyroBias(pKFi);
		VG->setId(maxKFid + 1 + pKFi->mnId);
		VG->setFixed(true);
		optimizer.addVertex(VG);
		vpgb.push_back(VG);
	}



	// prior acc bias
	//		EdgePriorGyro *epg = new EdgePriorGyro(cv::Mat::zeros(3, 1, CV_32F));
	//		epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
	//		double infoPriorG = priorG;
	//		epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
	//		optimizer.addEdge(epg);

	// extrinsic parameter
	g2o::VertexSE3Expmap *vT_d_c = new g2o::VertexSE3Expmap();
	vT_d_c->setEstimate(Converter::toSE3Quat(vpKFs[0]->mImuCalib.mT_dvl_c));
	vT_d_c->setId(maxKFid + 1 + maxKFid + 1);
	vT_d_c->setFixed(false);
	optimizer.addVertex(vT_d_c);

	g2o::VertexSE3Expmap *vT_g_d = new g2o::VertexSE3Expmap();
	vT_g_d->setEstimate(Converter::toSE3Quat(vpKFs[0]->mImuCalib.mT_gyro_dvl));
	vT_g_d->setId(maxKFid + 1 + maxKFid + 2);
	vT_g_d->setFixed(true);
	optimizer.addVertex(vT_g_d);


	// Graph edges
	vector<EdgeDvlGyroInit *> vpei;
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
			if (!pKFi->mpDvlPreintegrationKeyFrame) {
				std::cout << "Not preintegrated measurement" << std::endl;
			}

			pKFi->mpDvlPreintegrationKeyFrame->SetNewBias(pKFi->mPrevKF->GetImuBias());
			VertexPoseDvlIMU *VP1 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mPrevKF->mnId));
			//				g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
			VertexPoseDvlIMU *VP2 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
			//				g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
			g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid + 1 + pKFi->mPrevKF->mnId);
			g2o::HyperGraph::Vertex *VT_d_c = optimizer.vertex(maxKFid + 1 + maxKFid + 1);
			g2o::HyperGraph::Vertex *VT_g_d = optimizer.vertex(maxKFid + 1 + maxKFid + 2);
			//				g2o::HyperGraph::Vertex *VA = optimizer.vertex(maxKFid * 2 + 3);
			//				g2o::HyperGraph::Vertex *VGDir = optimizer.vertex(maxKFid * 2 + 4);
			//				g2o::HyperGraph::Vertex *VS = optimizer.vertex(maxKFid * 2 + 5);
			//				cout<<"VP1: Rcw[0]"<<VP1->estimate().Rcw[0]<<endl;
			//				cout<<"VP1: Rwc"<<VP1->estimate().Rwc<<endl;
			//				cout<<"VP1: tcw[0]"<<VP1->estimate().tcw[0]<<endl;
			//				cout<<"VP1: twc"<<VP1->estimate().twc<<endl;
			//
			//				cout<<"VP2: Rcw[0]"<<VP2->estimate().Rcw[0]<<endl;
			//				cout<<"VP2: Rwc"<<VP2->estimate().Rwc<<endl;
			//				cout<<"VP2: tcw[0]"<<VP2->estimate().tcw[0]<<endl;
			//				cout<<"VP2: twc"<<VP2->estimate().twc<<endl;

			if (!VP1 || !VG || !VP2) {
				cout << "Error" << VP1 << ", " << VG << ", " << VP2 << endl;

				continue;
			}
			//				EdgeInertialGS *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
			EdgeDvlGyroInit *ei = new EdgeDvlGyroInit(pKFi->mpDvlPreintegrationKeyFrame);
			//				ei->setVertex(0, VP1);

			//			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
			//			ei->setRobustKernel(rk);
			//			rk->setDelta(sqrt(7.815));
			ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
			ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
			ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
			ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
			ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
			ei->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100000);
			ei->setId(pKFi->mnId);


			vpei.push_back(ei);

			vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
			optimizer.addEdge(ei);
		}
	}


	optimizer.setVerbose(true);
	std::set<g2o::HyperGraph::Edge *> setEdges = optimizer.edges();
	double total_error = 0;

	const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
	const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};

	int mono_outlier;
	int stereo_outlier;
	float total_chi2;
	for (int i = 0; i < 4; i++) {
		if (i > 0) {
			vT_g_d->setFixed(false);
		}
		if (i > 2) {
			for (auto v_gb: vpgb) {
				v_gb->setFixed(false);
			}
		}

		optimizer.initializeOptimization(0);
		optimizer.optimize(10);
		//		for (auto ei:vpei) {
		//			ei->computeError();
		//			cout << "calibration optimization iteration : " << i << "edge of " << ei->id() - 1 << " and " << ei->id()
		//				 << "\nchi2:" << ei->chi2() << endl;
		//		}

	}

	//	std::cout << "start optimization" << std::endl;
	//	optimizer.setVerbose(true);
	//	optimizer.initializeOptimization();
	//	optimizer.optimize(its);
	//
	//	std::cout << "end optimization" << std::endl;


	// Recover optimized data
	// Biases
	for (auto pkf: vpKFs) {
		int kf_id = pkf->mnId;
		int bias_vertex_id = kf_id + maxKFid + 1;
		VertexGyroBias *v_gb = dynamic_cast<VertexGyroBias *>(optimizer.vertex(bias_vertex_id));
		bg << v_gb->estimate();
		IMU::Bias b(0, 0, 0, bg[0], bg[1], bg[2]);

		cout << "kf id: " << pkf->mnId << " gyros bias: " << bg.transpose() << endl;


		cv::Mat cvbg;
		cv::eigen2cv(bg, cvbg);
		cvbg.convertTo(cvbg, CV_32F);
		//		if (cv::norm(pkf->GetGyroBias() - cvbg) > 0.01) {
		//			pkf->SetNewBias(b);
		//			if (pkf->mpDvlPreintegrationKeyFrame) {
		//				pkf->mpDvlPreintegrationKeyFrame->ReintegrateWithVelocity();
		//			}
		//		}
		//		else {
		//			pkf->SetNewBias(b);
		//		}
		pkf->SetNewBias(b);
	}


	Eigen::Isometry3d T_dvl_c = vT_d_c->estimate();
	Eigen::Isometry3d T_gyros_dvl = vT_g_d->estimate();

	Eigen::Matrix3d R_gt;
	R_gt << 0, 0, 1,
		-1, 0, 0,
		0, -1, 0;
	cout << "init optimization result: \n"
	     << "R_dvl_c:\n" << T_dvl_c.rotation() << "\n"
	     << "R_dvl_c(eular yaw-pitch-roll):" << T_dvl_c.rotation().eulerAngles(2, 1, 0).transpose() << "\n"
	     << "t_dvl_c:" << T_dvl_c.translation().transpose() << "\n"
	     << "R_dvl_c distance with R_gt(LogSO3()): " << LogSO3(T_dvl_c.rotation().inverse() * R_gt).transpose() << "\n"
	     << "R_gyros_dvl:\n" << T_gyros_dvl.rotation() << "\n"
	     << "R_gyros_dvl(eular yaw-pitch-roll):" << T_gyros_dvl.rotation().eulerAngles(2, 1, 0).transpose() << "\n"
	     << endl;


	cv::Mat cvbg = Converter::toCvMat(bg);

	//Keyframes velocities and biases
	std::cout << "update Keyframes biases and extrinsic paramters" << std::endl;

	cv::Mat T_dvl_c_cv, T_gyros_dvl_cv, T_gyros_c_cv;
	cv::eigen2cv(T_dvl_c.matrix(), T_dvl_c_cv);
	T_dvl_c_cv.convertTo(T_dvl_c_cv, CV_32FC1);
	cv::eigen2cv(T_gyros_dvl.matrix(), T_gyros_dvl_cv);
	T_gyros_dvl_cv.convertTo(T_gyros_dvl_cv, CV_32FC1);
	Eigen::Isometry3d T_gyros_c = T_gyros_dvl * T_dvl_c;
	cv::eigen2cv(T_gyros_c.matrix(), T_gyros_c_cv);
	T_gyros_c_cv.convertTo(T_gyros_c_cv, CV_32FC1);

	// IMU::Calib extrinsic_para(T_gyros_c_cv, T_dvl_c_cv);

	const int N = vpKFs.size();
	for (size_t i = 0; i < N; i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}
        pKFi->mImuCalib.SetExtrinsic(T_gyros_c_cv, T_dvl_c_cv);
	}

}

double Optimizer::DvlIMUInitOptimization(Map *pMap, double priori_g, double priori_a)
{
	// Verbose::PrintMess("inertial optimization", Verbose::VERBOSITY_NORMAL);
	int its = 200; // Check number of iterations
	long unsigned int maxKFid = pMap->GetMaxKFid();
	const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);


	optimizer.setAlgorithm(solver);
    // solver->setUserLambdaInit(1e3);

	// Set KeyFrame vertices (fixed poses and optimizable velocities)
	vector<VertexGyroBias *> vpgb;
    vector<VertexAccBias *> vpab;
	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}
		VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
		VP->setId(pKFi->mnId);
		VP->setFixed(true);
		optimizer.addVertex(VP);



		//velocity
		VertexVelocity *VV = new VertexVelocity(pKFi);
        VV->setId((maxKFid + 1)*3 + pKFi->mnId);
        VV->setFixed(false);
        optimizer.addVertex(VV);
	}

    // Biases
    VertexGyroBias *VG = new VertexGyroBias(vpKFs.front());
    VG->setId(maxKFid + 1 + 1);
    VG->setFixed(false);
    optimizer.addVertex(VG);

    VertexAccBias *VA = new VertexAccBias(vpKFs.front());
    VA->setId((maxKFid + 1)*2 + 1);
    VA->setFixed(true);
    optimizer.addVertex(VA);





	// extrinsic parameter
	g2o::VertexSE3Expmap *vT_d_c = new g2o::VertexSE3Expmap();
	vT_d_c->setEstimate(Converter::toSE3Quat(vpKFs[0]->mImuCalib.mT_dvl_c));
	vT_d_c->setId((maxKFid + 1)*4);
	vT_d_c->setFixed(true);
	optimizer.addVertex(vT_d_c);

	g2o::VertexSE3Expmap *vT_g_d = new g2o::VertexSE3Expmap();
	vT_g_d->setEstimate(Converter::toSE3Quat(vpKFs[0]->mImuCalib.mT_gyro_dvl));
	vT_g_d->setId((maxKFid + 1)*4+1);
	vT_g_d->setFixed(true);
	optimizer.addVertex(vT_g_d);

    VertexGDir *VGDir = new VertexGDir(pMap->getRGravity());
    VGDir->setId((maxKFid + 1)*4+2);
    VGDir->setFixed(false);
    optimizer.addVertex(VGDir);


	// Graph edges
    EdgePriorGyro* eg_pri_bias = new EdgePriorGyro();
    eg_pri_bias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
    eg_pri_bias->setInformation(Eigen::Matrix3d::Identity()*priori_g);
    optimizer.addEdge(eg_pri_bias);
    EdgePriorAcc* e_pri_bias = new EdgePriorAcc();
    e_pri_bias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
    e_pri_bias->setInformation(Eigen::Matrix3d::Identity()*priori_a);
    optimizer.addEdge(e_pri_bias);

	vector<EdgeDvlIMUInitWithoutBias *> vpei;
	vpei.reserve(vpKFs.size());
	vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
	vppUsedKF.reserve(vpKFs.size());
	// std::cout << "build optimization graph" << std::endl;
    vector<EdgeDvlIMU*> dvlimu_edges;
	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId<=1)
			continue;

		if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
			if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) {
				continue;
			}
			if (!pKFi->mpDvlPreintegrationKeyFrame) {
				std::cout << "Not preintegrated measurement" << std::endl;
			}

			// pKFi->mpDvlPreintegrationKeyFrame->SetNewBias(pKFi->mPrevKF->GetImuBias());
			VertexPoseDvlIMU *VP1 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mPrevKF->mnId));
			//				g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
			VertexPoseDvlIMU *VP2 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
			//				g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
            g2o::HyperGraph::Vertex *VV1 = optimizer.vertex((maxKFid + 1)*3 + pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex *VV2 = optimizer.vertex((maxKFid + 1)*3 + pKFi->mnId);
            g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid + 1 + 1);
            g2o::HyperGraph::Vertex *VA = optimizer.vertex((maxKFid + 1) * 2 + 1);


			g2o::HyperGraph::Vertex *VT_d_c = optimizer.vertex((maxKFid + 1)*4);
			g2o::HyperGraph::Vertex *VT_g_d = optimizer.vertex((maxKFid + 1)*4+1);
            g2o::HyperGraph::Vertex *VR_w_b0 = optimizer.vertex((maxKFid + 1)*4+2);

			if (!VP1 || !VP2 || !VV1 || !VV2 || !VG || !VA  || !VT_d_c || !VT_g_d || !VR_w_b0) {
                ROS_ERROR_STREAM("DVL IMU initialzation Error, KF1 ID:"<< pKFi->mPrevKF->mnId << "KF2 ID:" << pKFi->mnId << "VP1: " << VP1 <<", VP2: " << VP2 << ", VV1: " << VV1
								 << ", VV2: " << VV2 << ", VG: " << VG << ", VA: " << VA
								 << ", VT_d_c: " << VT_d_c << ", VT_g_d: " << VT_g_d
								 << ", VR_w_b0: " << VR_w_b0);
				continue;
                // assert(-1);
			}
            // // prior acc bias
            // EdgePriorAcc *epa = new EdgePriorAcc(cv::Mat::zeros(3, 1, CV_32F));
            // epa->setLevel(0);
            // epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
            // epa->setInformation(bias_info * Eigen::Matrix3d::Identity());
            // optimizer.addEdge(epa);
            // // prior gyro bias
            // EdgePriorGyro *epg = new EdgePriorGyro(cv::Mat::zeros(3, 1, CV_32F));
            // epg->setLevel(0);
            // epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
            // epg->setInformation(bias_info * Eigen::Matrix3d::Identity());
            // optimizer.addEdge(epg);




			EdgeDvlIMU *ei2 = new EdgeDvlIMU(pKFi->mpDvlPreintegrationKeyFrame);
			ei2->setLevel(0);
			ei2->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
			ei2->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
			ei2->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
			ei2->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
			ei2->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
			ei2->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
			ei2->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
			ei2->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
			ei2->setVertex(8, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VR_w_b0));
            Eigen::Matrix<double, 9, 9> info = Eigen::Matrix<double, 9, 9>::Identity()* 1e6;
            cv::Mat cvInfo = pKFi->mpDvlPreintegrationKeyFrame->C.rowRange(0,9).colRange(0,9).inv(cv::DECOMP_SVD);
            cv::cv2eigen(cvInfo, info);
            // info.block(0,0,3,3) = Eigen::Matrix3d::Identity() * 1e6;
            // info.block(3,3,3,3) = Eigen::Matrix3d::Identity() * 1e4;
            // info.block(6,6,3,3) = Eigen::Matrix3d::Identity() * 1e4;
            // info.block(0,0,3,3) = Eigen::Matrix3d::Identity() * 1e6;
            // info(0,0) = info(0,0)*lamda_DVL * 5e3; // 10_24
            // info_DI(1,1) = 1e9; // before 10_24
            // info.block(3,3,3,3) = Eigen::Matrix3d::Identity() * 1e6;
            // info.block(6,6,3,3) = Eigen::Matrix3d::Identity() * 1e6;
			ei2->setInformation(info);
			// ei2->setId(pKFi->mnId);
			optimizer.addEdge(ei2);
            dvlimu_edges.push_back(ei2);

		}
	}


	optimizer.setVerbose(false);
    optimizer.initializeOptimization(0);
    optimizer.optimize(20);
    VG->setFixed(false);
    VA->setFixed(false);
    optimizer.initializeOptimization(0);
    optimizer.optimize(20);

    auto bias_g = VG->estimate();
    auto bias_a = VA->estimate();
    ROS_INFO_STREAM("bias_g: "<< bias_g.transpose());
    ROS_INFO_STREAM("bias_a: "<< bias_a.transpose());

    double total_dvl = 0;
    double avg_dvl = 0;
    for(auto e:dvlimu_edges){
        total_dvl += e->error().norm();
    }
    avg_dvl = total_dvl/dvlimu_edges.size();
    ROS_INFO_STREAM("avg_dvl: "<< avg_dvl);
    ROS_INFO_STREAM("total_dvl:"<< total_dvl);
    // VGDir->setFixed(true);
    // e_bias->setLevel(0);
    // e_bias_without->setLevel(1);
    // optimizer.initializeOptimization(0);
    // optimizer.optimize(2);

    // update gravity direction
    // Eigen::Matrix3d R_b0_w = NormalizeRotation(VGDir->estimate().Rwg);
    // Eigen::Matrix3d R_b0_w = VGDir->estimate().Rwg;
    // ROS_INFO_STREAM("gravity calibration result: "<< R_b0_w);
    // Sophus::SO3<double> R_b0_w_SO3(R_b0_w);
    // Eigen::Vector3d R_b0_w_so3 = R_b0_w_SO3.log();
    // ROS_INFO_STREAM("gravity calibration result: \n"<< VGDir->estimate().Rwg);
    pMap->setRGravity(VGDir->estimate().Rwg);
    // if(vpKFs.size()<200){
    // pMap->SetImuInitialized();
    if(avg_dvl>2000){
        return avg_dvl;
    }

	// Recover optimized data
	// Biases
	for (auto pkf: vpKFs) {
		int kf_id = pkf->mnId;
		int gyros_bias_vertex_id = 1 + maxKFid + 1;
        int acc_bias_vertex_id = 1 + (maxKFid + 1)*2;
		VertexGyroBias *v_gb = dynamic_cast<VertexGyroBias *>(optimizer.vertex(gyros_bias_vertex_id));
        VertexAccBias *v_ab = dynamic_cast<VertexAccBias *>(optimizer.vertex(acc_bias_vertex_id));
        // bg << v_gb->estimate();
		IMU::Bias b(v_ab->estimate().x(), v_ab->estimate().y(), v_ab->estimate().z(),
                    v_gb->estimate().x(), v_gb->estimate().y(), v_gb->estimate().z());
		cv::Mat cvbg;
		cv::eigen2cv(v_gb->estimate(), cvbg);
		cvbg.convertTo(cvbg, CV_32F);
		pkf->SetNewBias(b);
	}
    return avg_dvl;
	// pkf->SetNewBias(b)

}


void Optimizer::DvlIMURefineOptimization(Atlas* pAtlas)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);


    long unsigned int maxKFid = 0;
    for(auto m:pAtlas->GetAllMaps()){
        if(maxKFid<m->GetMaxKFid())
            maxKFid = m->GetMaxKFid();
    }

    // extrinsic parameter
    g2o::VertexSE3Expmap *vT_d_c = new g2o::VertexSE3Expmap();
    vT_d_c->setEstimate(Converter::toSE3Quat(pAtlas->GetAllMaps().front()->GetOriginKF()->mImuCalib.mT_dvl_c));
    vT_d_c->setId((maxKFid + 1)*4);
    vT_d_c->setFixed(true);
    optimizer.addVertex(vT_d_c);

    g2o::VertexSE3Expmap *vT_g_d = new g2o::VertexSE3Expmap();
    vT_g_d->setEstimate(Converter::toSE3Quat(pAtlas->GetAllMaps().front()->GetOriginKF()->mImuCalib.mT_gyro_dvl));
    vT_g_d->setId((maxKFid + 1)*4+1);
    vT_g_d->setFixed(true);
    optimizer.addVertex(vT_g_d);

    VertexGDir *VGDir = new VertexGDir(pAtlas->getRGravity());
    VGDir->setId((maxKFid + 1)*4+2);
    VGDir->setFixed(false);
    optimizer.addVertex(VGDir);

    stringstream ss;
    ss<<"Gravity refine optimization: \n";
    for(auto pMap:pAtlas->GetAllMaps()){
        ss << "map id: " << pMap->GetId() << endl;
        const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

        // Set KeyFrame vertices (fixed poses and optimizable velocities)
        vector<VertexGyroBias *> vpgb;
        vector<VertexAccBias *> vpab;
        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKFi = vpKFs[i];
            if (pKFi->mnId > maxKFid) {
                continue;
            }
            VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
            VP->setId(pKFi->mnId);
            VP->setFixed(true);
            optimizer.addVertex(VP);

            // Biases
            VertexGyroBias *VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid + 1 + pKFi->mnId);
            VG->setFixed(true);
            optimizer.addVertex(VG);
            vpgb.push_back(VG);

            VertexAccBias *VA = new VertexAccBias(pKFi);
            VA->setId((maxKFid + 1)*2 + pKFi->mnId);
            VA->setFixed(true);
            optimizer.addVertex(VA);
            vpab.push_back(VA);

            //velocity
            VertexVelocity *VV = new VertexVelocity(pKFi);
            VV->setId((maxKFid + 1)*3 + pKFi->mnId);
            VV->setFixed(true);
            optimizer.addVertex(VV);
        }







        // Graph edges
        vector<EdgeDvlIMUInitWithoutBias *> vpei;
        vpei.reserve(vpKFs.size());
        vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
        vppUsedKF.reserve(vpKFs.size());

        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKFi = vpKFs[i];

            if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
                if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) {
                    continue;
                }
                if (!pKFi->mpDvlPreintegrationKeyFrame) {
                    std::cout << "Not preintegrated measurement" << std::endl;
                }
                ss << "Add gravity edge from KF[" << pKFi->mnId << "] to KF{" << pKFi->mPrevKF->mnId<<"]"<< endl;
                // pKFi->mpDvlPreintegrationKeyFrame->SetNewBias(pKFi->mPrevKF->GetImuBias());
                VertexPoseDvlIMU *VP1 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mPrevKF->mnId));
                //				g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
                VertexPoseDvlIMU *VP2 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
                //				g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
                g2o::HyperGraph::Vertex *VV1 = optimizer.vertex((maxKFid + 1)*3 + pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex *VV2 = optimizer.vertex((maxKFid + 1)*3 + pKFi->mnId);
                g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid + 1 + pKFi->mnId);
                g2o::HyperGraph::Vertex *VA = optimizer.vertex((maxKFid + 1)*2 + pKFi->mnId);
                g2o::HyperGraph::Vertex *VT_d_c = optimizer.vertex((maxKFid + 1)*4);
                g2o::HyperGraph::Vertex *VT_g_d = optimizer.vertex((maxKFid + 1)*4+1);
                g2o::HyperGraph::Vertex *VR_w_b0 = optimizer.vertex((maxKFid + 1)*4+2);
                //				g2o::HyperGraph::Vertex *VA = optimizer.vertex(maxKFid * 2 + 3);
                //				g2o::HyperGraph::Vertex *VGDir = optimizer.vertex(maxKFid * 2 + 4);
                //				g2o::HyperGraph::Vertex *VS = optimizer.vertex(maxKFid * 2 + 5);
                //				cout<<"VP1: Rcw[0]"<<VP1->estimate().Rcw[0]<<endl;
                //				cout<<"VP1: Rwc"<<VP1->estimate().Rwc<<endl;
                //				cout<<"VP1: tcw[0]"<<VP1->estimate().tcw[0]<<endl;
                //				cout<<"VP1: twc"<<VP1->estimate().twc<<endl;
                //
                //				cout<<"VP2: Rcw[0]"<<VP2->estimate().Rcw[0]<<endl;
                //				cout<<"VP2: Rwc"<<VP2->estimate().Rwc<<endl;
                //				cout<<"VP2: tcw[0]"<<VP2->estimate().tcw[0]<<endl;
                //				cout<<"VP2: twc"<<VP2->estimate().twc<<endl;

                if (!VP1 || !VG || !VP2) {
                    cout << "Error" << VP1 << ", " << VG << ", " << VP2 << endl;

                    continue;
                }
                // prior acc bias
                EdgePriorAcc *epa = new EdgePriorAcc(cv::Mat::zeros(3, 1, CV_32F));
                epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
                epa->setInformation(100 * Eigen::Matrix3d::Identity());
                optimizer.addEdge(epa);
                // prior gyro bias
                EdgePriorGyro *epg = new EdgePriorGyro(cv::Mat::zeros(3, 1, CV_32F));
                epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
                epg->setInformation(100 * Eigen::Matrix3d::Identity());
                optimizer.addEdge(epg);
                //				EdgeInertialGS *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
                EdgeDvlIMUInitWithoutBias *ei = new EdgeDvlIMUInitWithoutBias(pKFi->mpDvlPreintegrationKeyFrame);
                ei->setLevel(0);
                //				ei->setVertex(0, VP1);

                //			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                //			ei->setRobustKernel(rk);
                //			rk->setDelta(sqrt(7.815));
                ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
                ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
                ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
                ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
                ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
                ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
                ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
                ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
                ei->setVertex(8, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VR_w_b0));
                ei->setInformation(Eigen::Matrix<double, 3, 3>::Identity()*100);
                ei->setId(pKFi->mnId);
                optimizer.addEdge(ei);
            }
        }


    }


    // optimizer.setVerbose(true);
    optimizer.initializeOptimization(0);
    optimizer.optimize(5);
    ROS_DEBUG_STREAM(ss.str());
    // for(VertexGyroBias* v:vpgb){
    //     v->setFixed(false);
    // }
	//
    // vT_d_c->setFixed(false);
    // optimizer.initializeOptimization();
    // optimizer.optimize(5);

    // update gravity direction
    // Eigen::Matrix3d R_b0_w = NormalizeRotation(VGDir->estimate().Rwg);
    Eigen::Matrix3d R_b0_w = VGDir->estimate().Rwg;
    ROS_INFO_STREAM("gravity refine");
    // Sophus::SO3<double> R_b0_w_SO3(R_b0_w);
    // Eigen::Vector3d R_b0_w_so3 = R_b0_w_SO3.log();
    // ROS_INFO_STREAM("gravity calibration result: "<< R_b0_w_so3.transpose());
    pAtlas->setRGravity(R_b0_w);
    for(auto m:pAtlas->GetAllMaps()){
        m->setRGravity(R_b0_w);
        m->SetImuInitialized();
    }

}


void Optimizer::DvlBeamOptimization(Map *pMap)
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


	optimizer.setAlgorithm(solver);



	VertexDVLBeamOritenstion *v_beam_ori = new VertexDVLBeamOritenstion();
	Eigen::Matrix<double,8,1> alpha_beta;
	alpha_beta << 1.5 / 180.0 * M_PI,
		1.0 / 180.0 * M_PI,
		1.5 / 180.0 * M_PI,
		1.0 / 180.0 * M_PI,
		1.5 / 180.0 * M_PI,
		1.0 / 180.0 * M_PI,
		1.5 / 180.0 * M_PI,
		1.0 / 180.0 * M_PI;
//	alpha_beta << 67.5 / 180.0 * M_PI,
//		45 / 180.0 * M_PI,
//		67.5 / 180.0 * M_PI,
//		45 / 180.0 * M_PI,
//		67.5 / 180.0 * M_PI,
//		45 / 180.0 * M_PI,
//		67.5 / 180.0 * M_PI,
//		45 / 180.0 * M_PI;
	v_beam_ori->setFixed(false);
	v_beam_ori->setId(0);
	v_beam_ori->setEstimate(alpha_beta);
	optimizer.addVertex(v_beam_ori);

	// Graph edges
	vector<EdgeDvlGyroInit3 *> vpei;
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
			if (!pKFi->mpDvlPreintegrationKeyFrame) {
				std::cout << "Not preintegrated measurement" << std::endl;
			}


			EdgeDVLBeamCalibration1* ei = new EdgeDVLBeamCalibration1();
			ei->setVertex(0,v_beam_ori);
			ei->setInformation(Eigen::Matrix<double, 4, 4>::Identity() * 180.0 / M_PI);
			ei->setMeasurement(pKFi->mpDvlPreintegrationKeyFrame);


			vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
			optimizer.addEdge(ei);
		}
	}


	optimizer.setVerbose(true);
	optimizer.initializeOptimization();
	optimizer.optimize(10);

	Eigen::Matrix<double, 8, 1> r_opt = v_beam_ori->estimate();

	ROS_INFO_STREAM(
		"DVL Calibration(visual data):\nbeam1_theta=" << r_opt(0) / M_PI * 180.0 << " beam1_phi=" << r_opt(1) / M_PI * 180.0
		                                 << "\nbeam2_theta=" << r_opt(2) / M_PI * 180.0 << " beam2_phi="
		                                 << r_opt(3) / M_PI * 180.0
		                                 << "\nbeam3_theta=" << r_opt(4) / M_PI * 180.0 << " beam3_phi="
		                                 << r_opt(5) / M_PI * 180.0
		                                 << "\nbeam4_theta=" << r_opt(6) / M_PI * 180.0 << " beam4_phi="
		                                 << r_opt(7) / M_PI * 180.0);



}

void Optimizer::DvlBeamOptimization_dvl(Map *pMap)
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


	optimizer.setAlgorithm(solver);



	VertexDVLBeamOritenstion *v_beam_ori = new VertexDVLBeamOritenstion();
	Eigen::Matrix<double,8,1> alpha_beta;
	alpha_beta << 1.5 / 180.0 * M_PI,
		1.0 / 180.0 * M_PI,
		1.5 / 180.0 * M_PI,
		1.0 / 180.0 * M_PI,
		1.5 / 180.0 * M_PI,
		1.0 / 180.0 * M_PI,
		1.5 / 180.0 * M_PI,
		1.0 / 180.0 * M_PI;
//	alpha_beta << 67.5 / 180.0 * M_PI,
//		45 / 180.0 * M_PI,
//		67.5 / 180.0 * M_PI,
//		45 / 180.0 * M_PI,
//		67.5 / 180.0 * M_PI,
//		45 / 180.0 * M_PI,
//		67.5 / 180.0 * M_PI,
//		45 / 180.0 * M_PI;
	v_beam_ori->setFixed(false);
	v_beam_ori->setId(0);
	v_beam_ori->setEstimate(alpha_beta);
	optimizer.addVertex(v_beam_ori);

	// Graph edges
	vector<EdgeDvlGyroInit3 *> vpei;
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
			if (!pKFi->mpDvlPreintegrationKeyFrame) {
				std::cout << "Not preintegrated measurement" << std::endl;
			}


			EdgeDVLBeamCalibration2* ei = new EdgeDVLBeamCalibration2();
			ei->setVertex(0,v_beam_ori);
			ei->setInformation(Eigen::Matrix<double, 4, 4>::Identity() * 180.0 / M_PI);
			ei->setMeasurement(pKFi->mpDvlPreintegrationKeyFrame);


			vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
			optimizer.addEdge(ei);
		}
	}


	optimizer.setVerbose(true);
	optimizer.initializeOptimization();
	optimizer.optimize(10);

	Eigen::Matrix<double, 8, 1> r_opt = v_beam_ori->estimate();

	ROS_INFO_STREAM(
		"DVL Calibration(DVL data):\nbeam1_theta=" << r_opt(0) / M_PI * 180.0 << " beam1_phi=" << r_opt(1) / M_PI * 180.0
		                                 << "\nbeam2_theta=" << r_opt(2) / M_PI * 180.0 << " beam2_phi="
		                                 << r_opt(3) / M_PI * 180.0
		                                 << "\nbeam3_theta=" << r_opt(4) / M_PI * 180.0 << " beam3_phi="
		                                 << r_opt(5) / M_PI * 180.0
		                                 << "\nbeam4_theta=" << r_opt(6) / M_PI * 180.0 << " beam4_phi="
		                                 << r_opt(7) / M_PI * 180.0);



}

} // namespace ORB_SLAM3
