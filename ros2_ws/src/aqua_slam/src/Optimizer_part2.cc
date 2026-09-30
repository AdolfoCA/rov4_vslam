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

// ??? what is vpNonEnoughOptKFs
void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, vector<KeyFrame *> &vpNonEnoughOptKFs)
{
	// Local KeyFrames: First Breath Search from Current Keyframe
	list<KeyFrame *> lLocalKeyFrames;

	lLocalKeyFrames.push_back(pKF);

	// ??? what is mnBALocalForKF
	pKF->mnBALocalForKF = pKF->mnId;
	Map *pCurrentMap = pKF->GetMap();

	// get all key frames share covisibility
	const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
	// add all key frames share covisibility to lLocalKeyFrames
	for (int i = 0, iend = vNeighKFs.size(); i < iend; i++) {
		KeyFrame *pKFi = vNeighKFs[i];
		// ??? what is mnBALocalForKF
		pKFi->mnBALocalForKF = pKF->mnId;
		if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
			lLocalKeyFrames.push_back(pKFi);
		}
	}
	// add some of key frames in vpNonEnoughOptKFs to lLocalKeyFrames
	for (KeyFrame *pKFi: vpNonEnoughOptKFs) {
		// pKFi is not bad, pKFi is in the same map with pKF, and pKFi is not in lLocalKeyFrames
		if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap && pKFi->mnBALocalForKF != pKF->mnId) {
			pKFi->mnBALocalForKF = pKF->mnId;
			lLocalKeyFrames.push_back(pKFi);
		}
	}

	// Local MapPoints seen in Local KeyFrames
	list<MapPoint *> lLocalMapPoints;
	set<MapPoint *> sNumObsMP;
	// ??? what is num_fixedKF
	int num_fixedKF;
	// add all map points associated to keypoints in the key frames in lLocalKeyFrames to lLocalMapPoints
	for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
		KeyFrame *pKFi = *lit;
		if (pKFi->mnId == pCurrentMap->GetInitKFid()) {
			num_fixedKF = 1;
		}
		// get all MapPoints associated to keypoints in the key frame pKFi
		vector<MapPoint *> vpMPs = pKFi->GetMapPointMatches();
		for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
			MapPoint *pMP = *vit;
			if (pMP) {
				if (!pMP->isBad() && pMP->GetMap() == pCurrentMap) {

					if (pMP->mnBALocalForKF != pKF->mnId) {
						lLocalMapPoints.push_back(pMP);
						pMP->mnBALocalForKF = pKF->mnId;
					}
				}
			}
		}
	}

	// Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
	list<KeyFrame *> lFixedCameras;
	for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
		map<KeyFrame *, tuple<int, int>> observations = (*lit)->GetObservations();
		for (map<KeyFrame *, tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end();
		     mit != mend; mit++) {
			KeyFrame *pKFi = mit->first;

			if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
				pKFi->mnBAFixedForKF = pKF->mnId;
				if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
					lFixedCameras.push_back(pKFi);
				}
			}
		}
	}
	num_fixedKF = lFixedCameras.size() + num_fixedKF;

	// if fixedKF is too less, add fixed key franme
	if (num_fixedKF < 2) {
		//Verbose::PrintMess("LM-LBA: New Fixed KFs had been set", Verbose::VERBOSITY_NORMAL);
		//TODO We set 2 KFs to fixed to avoid a degree of freedom in scale
		list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin();
		int lowerId = pKF->mnId;
		KeyFrame *pLowerKf;
		int secondLowerId = pKF->mnId;
		KeyFrame *pSecondLowerKF;

		for (; lit != lLocalKeyFrames.end(); lit++) {
			KeyFrame *pKFi = *lit;
			if (pKFi == pKF || pKFi->mnId == pCurrentMap->GetInitKFid()) {
				continue;
			}

			if (pKFi->mnId < lowerId) {
				lowerId = pKFi->mnId;
				pLowerKf = pKFi;
			}
			else if (pKFi->mnId < secondLowerId) {
				secondLowerId = pKFi->mnId;
				pSecondLowerKF = pKFi;
			}
		}
		lFixedCameras.push_back(pLowerKf);
		lLocalKeyFrames.remove(pLowerKf);
		num_fixedKF++;
		if (num_fixedKF < 2) {
			lFixedCameras.push_back(pSecondLowerKF);
			lLocalKeyFrames.remove(pSecondLowerKF);
			num_fixedKF++;
		}
	}

	if (num_fixedKF == 0) {
		Verbose::PrintMess("LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted", Verbose::VERBOSITY_NORMAL);
		//return;
	}
	//Verbose::PrintMess("LM-LBA: There are " + to_string(lLocalKeyFrames.size()) + " KFs and " + to_string(lLocalMapPoints.size()) + " MPs to optimize. " + to_string(num_fixedKF) + " KFs are fixed", Verbose::VERBOSITY_DEBUG);

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

	g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	if (pCurrentMap->IsInertial()) {
		solver->setUserLambdaInit(100.0);
	} // TODO uncomment
	//cout << "LM-LBA: lambda init: " << solver->userLambdaInit() << endl;

	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(false);

	if (pbStopFlag) {
		optimizer.setForceStopFlag(pbStopFlag);
	}

	unsigned long maxKFid = 0;

	// Set Local KeyFrame vertices
	for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
		KeyFrame *pKFi = *lit;
		g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
		vSE3->setId(pKFi->mnId);
		// fixed the first key frame of map
		vSE3->setFixed(pKFi->mnId == pCurrentMap->GetInitKFid());
		optimizer.addVertex(vSE3);
		if (pKFi->mnId > maxKFid) {
			maxKFid = pKFi->mnId;
		}
	}

	// Set Fixed KeyFrame vertices
	for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++) {
		KeyFrame *pKFi = *lit;
		g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
		vSE3->setId(pKFi->mnId);
		vSE3->setFixed(true);
		optimizer.addVertex(vSE3);
		if (pKFi->mnId > maxKFid) {
			maxKFid = pKFi->mnId;
		}
	}

	Verbose::PrintMess(
		"LM-LBA: opt/fixed KFs: " + to_string(lLocalKeyFrames.size()) + "/" + to_string(lFixedCameras.size()),
		Verbose::VERBOSITY_DEBUG);
	Verbose::PrintMess("LM-LBA: local MPs: " + to_string(lLocalMapPoints.size()), Verbose::VERBOSITY_DEBUG);

	// Set MapPoint vertices
	const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

	vector<ORB_SLAM3::EdgeSE3ProjectXYZ *> vpEdgesMono;
	vpEdgesMono.reserve(nExpectedSize);

	vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody *> vpEdgesBody;
	vpEdgesBody.reserve(nExpectedSize);

	vector<KeyFrame *> vpEdgeKFMono;
	vpEdgeKFMono.reserve(nExpectedSize);

	vector<KeyFrame *> vpEdgeKFBody;
	vpEdgeKFBody.reserve(nExpectedSize);

	vector<MapPoint *> vpMapPointEdgeMono;
	vpMapPointEdgeMono.reserve(nExpectedSize);

	vector<MapPoint *> vpMapPointEdgeBody;
	vpMapPointEdgeBody.reserve(nExpectedSize);

	vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
	vpEdgesStereo.reserve(nExpectedSize);

	vector<KeyFrame *> vpEdgeKFStereo;
	vpEdgeKFStereo.reserve(nExpectedSize);

	vector<MapPoint *> vpMapPointEdgeStereo;
	vpMapPointEdgeStereo.reserve(nExpectedSize);

	const float thHuberMono = sqrt(5.991);
	const float thHuberStereo = sqrt(7.815);

	int nPoints = 0;

	int nKFs = lLocalKeyFrames.size() + lFixedCameras.size(), nEdges = 0;

	// add map points and edges
	for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
		MapPoint *pMP = *lit;
		g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
		int id = pMP->mnId + maxKFid + 1;
		vPoint->setId(id);
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);
		nPoints++;

		const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

		//Set edges
		for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(), mend = observations.end();
		     mit != mend; mit++) {
			KeyFrame *pKFi = mit->first;

			if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
				const int cam0Index = get<0>(mit->second);

				// Monocular observation of Camera 0
				if (cam0Index != -1 && pKFi->mvuRight[cam0Index] < 0) {
					const cv::KeyPoint &kpUn = pKFi->mvKeysUn[cam0Index];
					Eigen::Matrix<double, 2, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y;

					ORB_SLAM3::EdgeSE3ProjectXYZ *e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
					e->setMeasurement(obs);
					const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
					e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberMono);

					e->pCamera = pKFi->mpCamera;

					optimizer.addEdge(e);
					vpEdgesMono.push_back(e);
					vpEdgeKFMono.push_back(pKFi);
					vpMapPointEdgeMono.push_back(pMP);

					nEdges++;
				}
				else if (cam0Index != -1
					&& pKFi->mvuRight[cam0Index] >= 0) // Stereo observation (with rectified images)
				{
					const cv::KeyPoint &kpUn = pKFi->mvKeysUn[cam0Index];
					Eigen::Matrix<double, 3, 1> obs;
					const float kp_ur = pKFi->mvuRight[cam0Index];
					obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

					g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
					e->setMeasurement(obs);
					const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
					Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
					e->setInformation(Info);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberStereo);

					e->fx = pKFi->fx;
					e->fy = pKFi->fy;
					e->cx = pKFi->cx;
					e->cy = pKFi->cy;
					e->bf = pKFi->mbf;

					optimizer.addEdge(e);
					vpEdgesStereo.push_back(e);
					vpEdgeKFStereo.push_back(pKFi);
					vpMapPointEdgeStereo.push_back(pMP);

					nEdges++;
				}

				// Monocular observation of Camera 0
				if (pKFi->mpCamera2) {
					int rightIndex = get<1>(mit->second);

					if (rightIndex != -1) {
						rightIndex -= pKFi->NLeft;

						Eigen::Matrix<double, 2, 1> obs;
						cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
						obs << kp.pt.x, kp.pt.y;

						ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
						e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
						e->setMeasurement(obs);
						const float &invSigma2 = pKFi->mvInvLevelSigma2[kp.octave];
						e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

						g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(thHuberMono);

						e->mTrl = Converter::toSE3Quat(pKFi->mTrl);

						e->pCamera = pKFi->mpCamera2;

						optimizer.addEdge(e);
						vpEdgesBody.push_back(e);
						vpEdgeKFBody.push_back(pKFi);
						vpMapPointEdgeBody.push_back(pMP);

						nEdges++;
					}
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

	//std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	int numPerform_it = optimizer.optimize(5);
	//std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

	//std::cout << "LBA time = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
	//std::cout << "Keyframes: " << nKFs << " --- MapPoints: " << nPoints << " --- Edges: " << nEdges << endl;

	bool bDoMore = true;

	if (pbStopFlag) {
		if (*pbStopFlag) {
			bDoMore = false;
		}
	}

	if (bDoMore) {

		// Check inlier observations
		int nMonoBadObs = 0;
		for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
			ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
			MapPoint *pMP = vpMapPointEdgeMono[i];

			if (pMP->isBad()) {
				continue;
			}

			if (e->chi2() > 5.991 || !e->isDepthPositive()) {
				//e->setLevel(1);
				nMonoBadObs++;
			}

			//e->setRobustKernel(0);
		}

		int nBodyBadObs = 0;
		for (size_t i = 0, iend = vpEdgesBody.size(); i < iend; i++) {
			ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = vpEdgesBody[i];
			MapPoint *pMP = vpMapPointEdgeBody[i];

			if (pMP->isBad()) {
				continue;
			}

			if (e->chi2() > 5.991 || !e->isDepthPositive()) {
				//e->setLevel(1);
				nBodyBadObs++;
			}

			//e->setRobustKernel(0);
		}

		int nStereoBadObs = 0;
		for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
			g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
			MapPoint *pMP = vpMapPointEdgeStereo[i];

			if (pMP->isBad()) {
				continue;
			}

			if (e->chi2() > 7.815 || !e->isDepthPositive()) {
				//e->setLevel(1);
				nStereoBadObs++;
			}

			//e->setRobustKernel(0);
		}
		Verbose::PrintMess(
			"LM-LBA: First optimization has " + to_string(nMonoBadObs) + " monocular and " + to_string(nStereoBadObs)
				+ " stereo bad observations", Verbose::VERBOSITY_DEBUG);

		// Optimize again without the outliers
		//Verbose::PrintMess("LM-LBA: second optimization", Verbose::VERBOSITY_DEBUG);
		//optimizer.initializeOptimization(0);
		//numPerform_it = optimizer.optimize(10);
		numPerform_it += optimizer.optimize(5);
	}

	vector<pair<KeyFrame *, MapPoint *>> vToErase;
	vToErase.reserve(vpEdgesMono.size() + vpEdgesBody.size() + vpEdgesStereo.size());

	// Check inlier observations
	for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
		ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
		MapPoint *pMP = vpMapPointEdgeMono[i];

		if (pMP->isBad()) {
			continue;
		}

		if (e->chi2() > 5.991 || !e->isDepthPositive()) {
			KeyFrame *pKFi = vpEdgeKFMono[i];
			vToErase.push_back(make_pair(pKFi, pMP));
		}
	}

	for (size_t i = 0, iend = vpEdgesBody.size(); i < iend; i++) {
		ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = vpEdgesBody[i];
		MapPoint *pMP = vpMapPointEdgeBody[i];

		if (pMP->isBad()) {
			continue;
		}

		if (e->chi2() > 5.991 || !e->isDepthPositive()) {
			KeyFrame *pKFi = vpEdgeKFBody[i];
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

	Verbose::PrintMess("LM-LBA: outlier observations: " + to_string(vToErase.size()), Verbose::VERBOSITY_DEBUG);
	Verbose::PrintMess(
		"LM-LBA: total of observations: " + to_string(vpMapPointEdgeMono.size() + vpMapPointEdgeStereo.size()),
		Verbose::VERBOSITY_DEBUG);
	bool bRedrawError = false;
	bool bWriteStats = false;

	// Get Map Mutex
	unique_lock<shared_timed_mutex> lock(pCurrentMap->mMutexMapUpdate);

	if (!vToErase.empty()) {

		//cout << "LM-LBA: There are " << vToErase.size() << " observations whose will be deleted from the map" << endl;
		for (size_t i = 0; i < vToErase.size(); i++) {
			KeyFrame *pKFi = vToErase[i].first;
			MapPoint *pMPi = vToErase[i].second;
			pKFi->EraseMapPointMatch(pMPi);
			pMPi->EraseObservation(pKFi);
		}
	}

	// Recover optimized data
	//Keyframes
	vpNonEnoughOptKFs.clear();
	for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
		KeyFrame *pKFi = *lit;
		g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKFi->mnId));
		g2o::SE3Quat SE3quat = vSE3->estimate();
		cv::Mat Tiw = Converter::toCvMat(SE3quat);
		cv::Mat Tco_cn = pKFi->GetPose() * Tiw.inv();
		cv::Vec3d trasl = Tco_cn.rowRange(0, 3).col(3);
		double dist = cv::norm(trasl);
		pKFi->SetPose(Converter::toCvMat(SE3quat));

		pKFi->mnNumberOfOpt += numPerform_it;
		//cout << "LM-LBA: KF " << pKFi->mnId << " had performed " <<  pKFi->mnNumberOfOpt << " iterations" << endl;
		if (pKFi->mnNumberOfOpt < 10) {
			vpNonEnoughOptKFs.push_back(pKFi);
		}
	}

	//Points
	for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
		MapPoint *pMP = *lit;
		g2o::VertexSBAPointXYZ
			*vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));
		pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMP->UpdateNormalAndDepth();
	}

	pCurrentMap->IncreaseChangeIndex();
}

void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, int &num_fixedKF)
{
	//cout << "LBA" << endl;
	// Local KeyFrames: First Breath Search from Current Keyframe
	list<KeyFrame *> lLocalKeyFrames;

	lLocalKeyFrames.push_back(pKF);
	pKF->mnBALocalForKF = pKF->mnId;
	Map *pCurrentMap = pKF->GetMap();

	const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
	for (int i = 0, iend = vNeighKFs.size(); i < iend; i++) {
		KeyFrame *pKFi = vNeighKFs[i];
		pKFi->mnBALocalForKF = pKF->mnId;
		if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
			lLocalKeyFrames.push_back(pKFi);
		}
	}

	// Local MapPoints seen in Local KeyFrames
	num_fixedKF = 0;
	list<MapPoint *> lLocalMapPoints;
	set<MapPoint *> sNumObsMP;
	for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
		KeyFrame *pKFi = *lit;
		if (pKFi->mnId == pMap->GetInitKFid()) {
			num_fixedKF = 1;
		}
		vector<MapPoint *> vpMPs = pKFi->GetMapPointMatches();
		for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
			MapPoint *pMP = *vit;
			if (pMP) {
				if (!pMP->isBad() && pMP->GetMap() == pCurrentMap) {

					if (pMP->mnBALocalForKF != pKF->mnId) {
						lLocalMapPoints.push_back(pMP);
						pMP->mnBALocalForKF = pKF->mnId;
					}
				}
			}
		}
	}

	// Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
	list<KeyFrame *> lFixedCameras;
	for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
		map<KeyFrame *, tuple<int, int>> observations = (*lit)->GetObservations();
		for (map<KeyFrame *, tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end();
		     mit != mend; mit++) {
			KeyFrame *pKFi = mit->first;

			if (pKFi && pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
				pKFi->mnBAFixedForKF = pKF->mnId;
				if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
					lFixedCameras.push_back(pKFi);
				}
			}
		}
	}
	num_fixedKF = lFixedCameras.size() + num_fixedKF;
	if (num_fixedKF < 2) {
		//Verbose::PrintMess("LM-LBA: New Fixed KFs had been set", Verbose::VERBOSITY_NORMAL);
		//TODO We set 2 KFs to fixed to avoid a degree of freedom in scale
		list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin();
		int lowerId = pKF->mnId;
		KeyFrame *pLowerKf;
		int secondLowerId = pKF->mnId;
		KeyFrame *pSecondLowerKF;

		for (; lit != lLocalKeyFrames.end(); lit++) {
			KeyFrame *pKFi = *lit;
			if(!pKFi){
				continue;
			}
			if (pKFi == pKF || pKFi->mnId == pMap->GetInitKFid()) {
				continue;
			}

			if (pKFi->mnId < lowerId) {
				lowerId = pKFi->mnId;
				pLowerKf = pKFi;
			}
			else if (pKFi->mnId < secondLowerId) {
				secondLowerId = pKFi->mnId;
				pSecondLowerKF = pKFi;
			}
		}
		lFixedCameras.push_back(pLowerKf);
		lLocalKeyFrames.remove(pLowerKf);
		num_fixedKF++;
		if (num_fixedKF < 2) {
			lFixedCameras.push_back(pSecondLowerKF);
			lLocalKeyFrames.remove(pSecondLowerKF);
			num_fixedKF++;
		}
	}

	if (num_fixedKF == 0) {
		Verbose::PrintMess("LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted", Verbose::VERBOSITY_NORMAL);
		//return;
	}
	//Verbose::PrintMess("LM-LBA: There are " + to_string(lLocalKeyFrames.size()) + " KFs and " + to_string(lLocalMapPoints.size()) + " MPs to optimize. " + to_string(num_fixedKF) + " KFs are fixed", Verbose::VERBOSITY_DEBUG);

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

	g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	if (pMap->IsInertial()) {
		solver->setUserLambdaInit(100.0);
	} // TODO uncomment
	//cout << "LM-LBA: lambda init: " << solver->userLambdaInit() << endl;

	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(false);

	if (pbStopFlag) {
		optimizer.setForceStopFlag(pbStopFlag);
	}

	unsigned long maxKFid = 0;

	// Set Local KeyFrame vertices
	for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
		KeyFrame *pKFi = *lit;
		g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
		vSE3->setId(pKFi->mnId);
		vSE3->setFixed(pKFi->mnId == pMap->GetInitKFid());
		optimizer.addVertex(vSE3);
		if (pKFi->mnId > maxKFid) {
			maxKFid = pKFi->mnId;
		}
	}
	//Verbose::PrintMess("LM-LBA: KFs to optimize added", Verbose::VERBOSITY_DEBUG);

	// Set Fixed KeyFrame vertices
	for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++) {
		KeyFrame *pKFi = *lit;
		if(!pKFi){
			continue;
		}
		g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
		vSE3->setId(pKFi->mnId);
		vSE3->setFixed(true);
		optimizer.addVertex(vSE3);
		if (pKFi->mnId > maxKFid) {
			maxKFid = pKFi->mnId;
		}
	}

	// Set MapPoint vertices
	const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

	vector<ORB_SLAM3::EdgeSE3ProjectXYZ *> vpEdgesMono;
	vpEdgesMono.reserve(nExpectedSize);

	vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody *> vpEdgesBody;
	vpEdgesBody.reserve(nExpectedSize);

	vector<KeyFrame *> vpEdgeKFMono;
	vpEdgeKFMono.reserve(nExpectedSize);

	vector<KeyFrame *> vpEdgeKFBody;
	vpEdgeKFBody.reserve(nExpectedSize);

	vector<MapPoint *> vpMapPointEdgeMono;
	vpMapPointEdgeMono.reserve(nExpectedSize);

	vector<MapPoint *> vpMapPointEdgeBody;
	vpMapPointEdgeBody.reserve(nExpectedSize);

	vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
	vpEdgesStereo.reserve(nExpectedSize);

	vector<KeyFrame *> vpEdgeKFStereo;
	vpEdgeKFStereo.reserve(nExpectedSize);

	vector<MapPoint *> vpMapPointEdgeStereo;
	vpMapPointEdgeStereo.reserve(nExpectedSize);

	const float thHuberMono = sqrt(5.991);
	const float thHuberStereo = sqrt(7.815);

	int nPoints = 0;

	int nKFs = lLocalKeyFrames.size() + lFixedCameras.size(), nEdges = 0;

	for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
		MapPoint *pMP = *lit;
		g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
		int id = pMP->mnId + maxKFid + 1;
		vPoint->setId(id);
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);
		nPoints++;

		const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

		//Set edges
		for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(), mend = observations.end();
		     mit != mend; mit++) {
			KeyFrame *pKFi = mit->first;

			if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
				const int leftIndex = get<0>(mit->second);

				// Monocular observation
				if (leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)] < 0) {
					const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
					Eigen::Matrix<double, 2, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y;

					ORB_SLAM3::EdgeSE3ProjectXYZ *e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
					e->setMeasurement(obs);
					const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
					e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberMono);

					e->pCamera = pKFi->mpCamera;

					optimizer.addEdge(e);
					vpEdgesMono.push_back(e);
					vpEdgeKFMono.push_back(pKFi);
					vpMapPointEdgeMono.push_back(pMP);

					nEdges++;
				}
				else if (leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)] >= 0) // Stereo observation
				{
					const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
					Eigen::Matrix<double, 3, 1> obs;
					const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
					obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

					g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
					e->setMeasurement(obs);
					const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
					Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
					e->setInformation(Info);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberStereo);

					e->fx = pKFi->fx;
					e->fy = pKFi->fy;
					e->cx = pKFi->cx;
					e->cy = pKFi->cy;
					e->bf = pKFi->mbf;

					optimizer.addEdge(e);
					vpEdgesStereo.push_back(e);
					vpEdgeKFStereo.push_back(pKFi);
					vpMapPointEdgeStereo.push_back(pMP);

					nEdges++;
				}

				if (pKFi->mpCamera2) {
					int rightIndex = get<1>(mit->second);

					if (rightIndex != -1) {
						rightIndex -= pKFi->NLeft;

						Eigen::Matrix<double, 2, 1> obs;
						cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
						obs << kp.pt.x, kp.pt.y;

						ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
						e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
						e->setMeasurement(obs);
						const float &invSigma2 = pKFi->mvInvLevelSigma2[kp.octave];
						e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

						g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(thHuberMono);

						e->mTrl = Converter::toSE3Quat(pKFi->mTrl);

						e->pCamera = pKFi->mpCamera2;

						optimizer.addEdge(e);
						vpEdgesBody.push_back(e);
						vpEdgeKFBody.push_back(pKFi);
						vpMapPointEdgeBody.push_back(pMP);

						nEdges++;
					}
				}
			}
		}
	}

	//Verbose::PrintMess("LM-LBA: total observations: " + to_string(vpMapPointEdgeMono.size()+vpMapPointEdgeStereo.size()), Verbose::VERBOSITY_DEBUG);

	if (pbStopFlag) {
		if (*pbStopFlag) {
			return;
		}
	}

	optimizer.initializeOptimization();

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	optimizer.optimize(5);
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

	//std::cout << "LBA time = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
	//std::cout << "Keyframes: " << nKFs << " --- MapPoints: " << nPoints << " --- Edges: " << nEdges << endl;

	bool bDoMore = true;

	if (pbStopFlag) {
		if (*pbStopFlag) {
			bDoMore = false;
		}
	}

	if (bDoMore) {

		// Check inlier observations
		int nMonoBadObs = 0;
		for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
			ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
			MapPoint *pMP = vpMapPointEdgeMono[i];

			if (pMP->isBad()) {
				continue;
			}

			if (e->chi2() > 5.991 || !e->isDepthPositive()) {
				// e->setLevel(1); // MODIFICATION
				nMonoBadObs++;
			}

			//e->setRobustKernel(0);
		}

		int nBodyBadObs = 0;
		for (size_t i = 0, iend = vpEdgesBody.size(); i < iend; i++) {
			ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = vpEdgesBody[i];
			MapPoint *pMP = vpMapPointEdgeBody[i];

			if (pMP->isBad()) {
				continue;
			}

			if (e->chi2() > 5.991 || !e->isDepthPositive()) {
				//e->setLevel(1);
				nBodyBadObs++;
			}

			//e->setRobustKernel(0);
		}

		int nStereoBadObs = 0;
		for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
			g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
			MapPoint *pMP = vpMapPointEdgeStereo[i];

			if (pMP->isBad()) {
				continue;
			}

			if (e->chi2() > 7.815 || !e->isDepthPositive()) {
				//TODO e->setLevel(1);
				nStereoBadObs++;
			}

			//TODO e->setRobustKernel(0);
		}
		//Verbose::PrintMess("LM-LBA: First optimization has " + to_string(nMonoBadObs) + " monocular and " + to_string(nStereoBadObs) + " stereo bad observations", Verbose::VERBOSITY_DEBUG);

		// Optimize again without the outliers
		//Verbose::PrintMess("LM-LBA: second optimization", Verbose::VERBOSITY_DEBUG);
		optimizer.initializeOptimization(0);
		optimizer.optimize(10);
	}

	vector<pair<KeyFrame *, MapPoint *>> vToErase;
	vToErase.reserve(vpEdgesMono.size() + vpEdgesBody.size() + vpEdgesStereo.size());

	// Check inlier observations
	for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
		ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
		MapPoint *pMP = vpMapPointEdgeMono[i];

		if (pMP->isBad()) {
			continue;
		}

		if (e->chi2() > 5.991 || !e->isDepthPositive()) {
			KeyFrame *pKFi = vpEdgeKFMono[i];
			vToErase.push_back(make_pair(pKFi, pMP));
		}
	}

	for (size_t i = 0, iend = vpEdgesBody.size(); i < iend; i++) {
		ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = vpEdgesBody[i];
		MapPoint *pMP = vpMapPointEdgeBody[i];

		if (pMP->isBad()) {
			continue;
		}

		if (e->chi2() > 5.991 || !e->isDepthPositive()) {
			KeyFrame *pKFi = vpEdgeKFBody[i];
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

	//Verbose::PrintMess("LM-LBA: outlier observations: " + to_string(vToErase.size()), Verbose::VERBOSITY_DEBUG);
	bool bRedrawError = false;
	if (vToErase.size() >= (vpMapPointEdgeMono.size() + vpMapPointEdgeStereo.size()) * 0.5) {
		Verbose::PrintMess("LM-LBA: ERROR IN THE OPTIMIZATION, MOST OF THE POINTS HAS BECOME OUTLIERS",
		                   Verbose::VERBOSITY_NORMAL);

		return;
		bRedrawError = true;
		string folder_name = "test_LBA";
		string name = "_PreLM_LBA";
		name = "_PreLM_LBA_Fixed";
	}

	// Get Map Mutex
	unique_lock<shared_timed_mutex> lock(pMap->mMutexMapUpdate);

	if (!vToErase.empty()) {
		map<KeyFrame *, int> mspInitialConnectedKFs;
		map<KeyFrame *, int> mspInitialObservationKFs;
		if (bRedrawError) {
			for (KeyFrame *pKFi: lLocalKeyFrames) {

				mspInitialConnectedKFs[pKFi] = pKFi->GetConnectedKeyFrames().size();
				mspInitialObservationKFs[pKFi] = pKFi->GetNumberMPs();
			}
		}

		//cout << "LM-LBA: There are " << vToErase.size() << " observations whose will be deleted from the map" << endl;
		for (size_t i = 0; i < vToErase.size(); i++) {
			KeyFrame *pKFi = vToErase[i].first;
			MapPoint *pMPi = vToErase[i].second;
			pKFi->EraseMapPointMatch(pMPi);
			pMPi->EraseObservation(pKFi);
		}

		map<KeyFrame *, int> mspFinalConnectedKFs;
		map<KeyFrame *, int> mspFinalObservationKFs;
		if (bRedrawError) {
			ofstream f_lba;
			f_lba.open("test_LBA/LBA_failure_KF" + to_string(pKF->mnId) + ".txt");
			f_lba << "# KF id, Initial Num CovKFs, Final Num CovKFs, Initial Num MPs, Fimal Num MPs" << endl;
			f_lba << fixed;

			for (KeyFrame *pKFi: lLocalKeyFrames) {
				pKFi->UpdateConnections();
				int finalNumberCovKFs = pKFi->GetConnectedKeyFrames().size();
				int finalNumberMPs = pKFi->GetNumberMPs();
				f_lba << pKFi->mnId << ", " << mspInitialConnectedKFs[pKFi] << ", " << finalNumberCovKFs << ", "
				      << mspInitialObservationKFs[pKFi] << ", " << finalNumberMPs << endl;

				mspFinalConnectedKFs[pKFi] = finalNumberCovKFs;
				mspFinalObservationKFs[pKFi] = finalNumberMPs;
			}

			f_lba.close();
		}
	}

	// Recover optimized data
	//Keyframes
	bool bShowStats = false;
	for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
		KeyFrame *pKFi = *lit;
		g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKFi->mnId));
		g2o::SE3Quat SE3quat = vSE3->estimate();
		cv::Mat Tiw = Converter::toCvMat(SE3quat);
		cv::Mat Tco_cn = pKFi->GetPose() * Tiw.inv();
		cv::Vec3d trasl = Tco_cn.rowRange(0, 3).col(3);
		double dist = cv::norm(trasl);
		pKFi->SetPose(Converter::toCvMat(SE3quat));

		if (dist > 1.0) {
			bShowStats = true;
			Verbose::PrintMess("LM-LBA: Too much distance in KF " + to_string(pKFi->mnId) + ", " + to_string(dist)
				                   + " meters. Current KF " + to_string(pKF->mnId), Verbose::VERBOSITY_DEBUG);
			Verbose::PrintMess("LM-LBA: Number of connections between the KFs " + to_string(pKF->GetWeight((pKFi))),
			                   Verbose::VERBOSITY_DEBUG);

			int numMonoMP = 0, numBadMonoMP = 0;
			int numStereoMP = 0, numBadStereoMP = 0;
			for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
				if (vpEdgeKFMono[i] != pKFi) {
					continue;
				}
				ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
				MapPoint *pMP = vpMapPointEdgeMono[i];

				if (pMP->isBad()) {
					continue;
				}

				if (e->chi2() > 5.991 || !e->isDepthPositive()) {
					numBadMonoMP++;
				}
				else {
					numMonoMP++;
				}
			}

			for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
				if (vpEdgeKFStereo[i] != pKFi) {
					continue;
				}
				g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
				MapPoint *pMP = vpMapPointEdgeStereo[i];

				if (pMP->isBad()) {
					continue;
				}

				if (e->chi2() > 7.815 || !e->isDepthPositive()) {
					numBadStereoMP++;
				}
				else {
					numStereoMP++;
				}
			}
			Verbose::PrintMess(
				"LM-LBA: Good observations in mono " + to_string(numMonoMP) + " and stereo " + to_string(numStereoMP),
				Verbose::VERBOSITY_DEBUG);
			Verbose::PrintMess("LM-LBA: Bad observations in mono " + to_string(numBadMonoMP) + " and stereo "
				                   + to_string(numBadStereoMP), Verbose::VERBOSITY_DEBUG);
		}
	}

	//Points
	for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
		MapPoint *pMP = *lit;
		g2o::VertexSBAPointXYZ
			*vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));
		pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMP->UpdateNormalAndDepth();
	}

	if (bRedrawError) {
		string folder_name = "test_LBA";
		string name = "_PostLM_LBA";
		//pMap->printReprojectionError(lLocalKeyFrames, pKF, name, folder_name);
		name = "_PostLM_LBA_Fixed";
		//pMap->printReprojectionError(lFixedCameras, pKF, name, folder_name);
	}

	// TODO Check this changeindex
	pMap->IncreaseChangeIndex();
}

void Optimizer::LocalDVLBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, int &num_fixedKF)
{
	Map *pCurrentMap = pKF->GetMap();
	int Nd = std::min(10, (int)pCurrentMap->KeyFramesInMap() - 2);// number of keyframes in current map
	const unsigned long maxKFid = pKF->mnId;

	vector<KeyFrame *> OptDVLKFs;
	const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
	list<KeyFrame *> OptVisualKFS;

	OptDVLKFs.reserve(Nd);
	OptDVLKFs.push_back(pKF);
	pKF->mnBALocalForKF = pKF->mnId;

	for (int i = 1; i < Nd; i++) {
		if (OptDVLKFs.back()->mPrevKF) {
			OptDVLKFs.push_back(OptDVLKFs.back()->mPrevKF);
			OptDVLKFs.back()->mnBALocalForKF = pKF->mnId;
		}
		else {
			break;
		}
	}
	int N = OptDVLKFs.size();

	//cout << "LBA" << endl;
	// Local KeyFrames: First Breath Search from Current Keyframe
//		list<KeyFrame *> lLocalKeyFrames;
//
//		lLocalKeyFrames.push_back(pKF);
//		pKF->mnBALocalForKF = pKF->mnId;
//
//
//
//		for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
//		{
//			KeyFrame *pKFi = vNeighKFs[i];
//			pKFi->mnBALocalForKF = pKF->mnId;
//			if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
//				lLocalKeyFrames.push_back(pKFi);
//		}

	// Local MapPoints seen in Local KeyFrames
	list<MapPoint *> lLocalMapPoints;
	for (int i = 0; i < N; i++) {
		vector<MapPoint *> vpMPs = OptDVLKFs[i]->GetMapPointMatches();
		for (vector<MapPoint *>::iterator it = vpMPs.begin(); it != vpMPs.end(); it++) {
			MapPoint *pMP = *it;
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

	// Fixed Keyframe
	list<KeyFrame *> lFixedKFs;
	if (OptDVLKFs.back()->mPrevKF) {
		lFixedKFs.push_back(OptDVLKFs.back()->mPrevKF);
		OptDVLKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
	}
	else {
		OptDVLKFs.back()->mnBALocalForKF = 0;
		OptDVLKFs.back()->mnBAFixedForKF = pKF->mnId;
		lFixedKFs.push_back(OptDVLKFs.back());
		OptDVLKFs.pop_back();
	}

	const int maxFixedKF = 200;
	for (list<MapPoint *>::iterator it = lLocalMapPoints.begin(); it != lLocalMapPoints.end(); it++) {
		map<KeyFrame *, tuple<int, int>> observations = (*it)->GetObservations();
		for (map<KeyFrame *, tuple<int, int>>::iterator it_ob = observations.begin(); it_ob != observations.end();
		     it_ob++) {
			KeyFrame *pKFi = it_ob->first;
			if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
				pKFi->mnBAFixedForKF = pKF->mnId;
				if (!pKFi->isBad()) {
					lFixedKFs.push_back(pKFi);
					break;
				}
			}
		}
		if (lFixedKFs.size() >= maxFixedKF) {
			break;
		}
	}



	//Verbose::PrintMess("LM-LBA: There are " + to_string(lLocalKeyFrames.size()) + " KFs and " + to_string(lLocalMapPoints.size()) + " MPs to optimize. " + to_string(num_fixedKF) + " KFs are fixed", Verbose::VERBOSITY_DEBUG);

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

	g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	if (pMap->IsInertial()) {
		solver->setUserLambdaInit(100.0);
	} // TODO uncomment
	//cout << "LM-LBA: lambda init: " << solver->userLambdaInit() << endl;

	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(false);

	if (pbStopFlag) {
		optimizer.setForceStopFlag(pbStopFlag);
	}

//		unsigned long maxKFid = 0;

	// Set Local KeyFrame vertices
	for (int i = 0; i < N; i++) {
		KeyFrame *pKFi = OptDVLKFs[i];
		pKFi->IntegrateDVL(pKFi->mPrevKF);

		g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
//			cout<<"id:"<<pKFi->mnId<<"initial pose: \n"<<pKFi->GetPose()<<endl;
		vSE3->setId(pKFi->mnId);
		vSE3->setFixed(false);
		optimizer.addVertex(vSE3);
	}
	//Verbose::PrintMess("LM-LBA: KFs to optimize added", Verbose::VERBOSITY_DEBUG);

	// Set Fixed KeyFrame vertices
	for (list<KeyFrame *>::iterator it = lFixedKFs.begin(), lend = lFixedKFs.end(); it != lend; it++) {
		KeyFrame *pKFi = *it;
		if (pKFi->mPrevKF) {
			pKFi->IntegrateDVL(pKFi->mPrevKF);
		}
		g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
		vSE3->setId(pKFi->mnId);
		vSE3->setFixed(true);
		optimizer.addVertex(vSE3);
	}

	// Set MapPoint vertices
	const int nExpectedSize = (OptDVLKFs.size() + lFixedKFs.size()) * lLocalMapPoints.size();

	vector<ORB_SLAM3::EdgeSE3ProjectXYZ *> vpEdgesMono;
	vpEdgesMono.reserve(nExpectedSize);

	vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody *> vpEdgesBody;
	vpEdgesBody.reserve(nExpectedSize);

	vector<KeyFrame *> vpEdgeKFMono;
	vpEdgeKFMono.reserve(nExpectedSize);

	vector<KeyFrame *> vpEdgeKFBody;
	vpEdgeKFBody.reserve(nExpectedSize);

	vector<MapPoint *> vpMapPointEdgeMono;
	vpMapPointEdgeMono.reserve(nExpectedSize);

	vector<MapPoint *> vpMapPointEdgeBody;
	vpMapPointEdgeBody.reserve(nExpectedSize);

	vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
	vpEdgesStereo.reserve(nExpectedSize);

	vector<KeyFrame *> vpEdgeKFStereo;
	vpEdgeKFStereo.reserve(nExpectedSize);

	vector<MapPoint *> vpMapPointEdgeStereo;
	vpMapPointEdgeStereo.reserve(nExpectedSize);

	const float thHuberMono = sqrt(5.991);
	const float thHuberStereo = sqrt(7.815);

	int nPoints = 0;

	int nKFs = OptDVLKFs.size() + lFixedKFs.size(), nEdges = 0;

	for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
		MapPoint *pMP = *lit;
		g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
		int id = pMP->mnId + maxKFid + 1;
		vPoint->setId(id);
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);
		nPoints++;

		const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

		//Set edges
		for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(), mend = observations.end();
		     mit != mend; mit++) {
			KeyFrame *pKFi = mit->first;

			if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
				continue;
			}

			if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
				const int leftIndex = get<0>(mit->second);

				// Monocular observation
				if (leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)] < 0) {
					const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
					Eigen::Matrix<double, 2, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y;

					ORB_SLAM3::EdgeSE3ProjectXYZ *e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
					e->setMeasurement(obs);
					const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
					e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberMono);

					e->pCamera = pKFi->mpCamera;

					optimizer.addEdge(e);
					vpEdgesMono.push_back(e);
					vpEdgeKFMono.push_back(pKFi);
					vpMapPointEdgeMono.push_back(pMP);

					nEdges++;
				}
				else if (leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)] >= 0) // Stereo observation
				{
					const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
					Eigen::Matrix<double, 3, 1> obs;
					const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
					obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

					g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
					e->setMeasurement(obs);
					const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
					Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
					e->setInformation(Info);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberStereo);

					e->fx = pKFi->fx;
					e->fy = pKFi->fy;
					e->cx = pKFi->cx;
					e->cy = pKFi->cy;
					e->bf = pKFi->mbf;

					optimizer.addEdge(e);
					vpEdgesStereo.push_back(e);
					vpEdgeKFStereo.push_back(pKFi);
					vpMapPointEdgeStereo.push_back(pMP);

					nEdges++;
				}

				if (pKFi->mpCamera2) {
					int rightIndex = get<1>(mit->second);

					if (rightIndex != -1) {
						rightIndex -= pKFi->NLeft;

						Eigen::Matrix<double, 2, 1> obs;
						cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
						obs << kp.pt.x, kp.pt.y;

						ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
						e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
						e->setMeasurement(obs);
						const float &invSigma2 = pKFi->mvInvLevelSigma2[kp.octave];
						e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

						g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(thHuberMono);

						e->mTrl = Converter::toSE3Quat(pKFi->mTrl);

						e->pCamera = pKFi->mpCamera2;

						optimizer.addEdge(e);
						vpEdgesBody.push_back(e);
						vpEdgeKFBody.push_back(pKFi);
						vpMapPointEdgeBody.push_back(pMP);

						nEdges++;
					}
				}
			}
		}
	}

	for (vector<KeyFrame *>::iterator it = OptDVLKFs.begin(); it != OptDVLKFs.end(); it++) {
		if ((*it)->mPrevKF) {
			KeyFrame *p_cur = *it;
			KeyFrame *p_pre = (*it)->mPrevKF;
//				p_cur->IntegrateDVL(p_pre);
			EdgeSE3DVLBA *e = new EdgeSE3DVLBA(p_cur->mT_ei_ej, p_cur->mT_e_c);
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(p_pre->mnId)));
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(p_cur->mnId)));
			e->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 50000000);
			optimizer.addEdge(e);
//				cout<<"add edge"<<endl;
			nEdges++;
		}
		else {
			cout << "no previous keyframe for DVL" << endl;
		}

	}
	//Verbose::PrintMess("LM-LBA: total observations: " + to_string(vpMapPointEdgeMono.size()+vpMapPointEdgeStereo.size()), Verbose::VERBOSITY_DEBUG);

	if (pbStopFlag) {
		if (*pbStopFlag) {
			return;
		}
	}

	optimizer.initializeOptimization();

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	optimizer.optimize(5);
//		cout<<"first optimization"<<endl;
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

	//std::cout << "LBA time = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
	//std::cout << "Keyframes: " << nKFs << " --- MapPoints: " << nPoints << " --- Edges: " << nEdges << endl;

	bool bDoMore = true;

	if (pbStopFlag) {
		if (*pbStopFlag) {
			bDoMore = false;
		}
	}

	if (bDoMore) {

		// Check inlier observations
		int nMonoBadObs = 0;
		for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
			ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
			MapPoint *pMP = vpMapPointEdgeMono[i];

			if (pMP->isBad()) {
				continue;
			}

			if (e->chi2() > 5.991 || !e->isDepthPositive()) {
				// e->setLevel(1); // MODIFICATION
				nMonoBadObs++;
			}

			//e->setRobustKernel(0);
		}

		int nBodyBadObs = 0;
		for (size_t i = 0, iend = vpEdgesBody.size(); i < iend; i++) {
			ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = vpEdgesBody[i];
			MapPoint *pMP = vpMapPointEdgeBody[i];

			if (pMP->isBad()) {
				continue;
			}

			if (e->chi2() > 5.991 || !e->isDepthPositive()) {
				//e->setLevel(1);
				nBodyBadObs++;
			}

			//e->setRobustKernel(0);
		}

		int nStereoBadObs = 0;
		for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
			g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
			MapPoint *pMP = vpMapPointEdgeStereo[i];

			if (pMP->isBad()) {
				continue;
			}

			if (e->chi2() > 7.815 || !e->isDepthPositive()) {
				//TODO e->setLevel(1);
				nStereoBadObs++;
			}

			//TODO e->setRobustKernel(0);
		}
		//Verbose::PrintMess("LM-LBA: First optimization has " + to_string(nMonoBadObs) + " monocular and " + to_string(nStereoBadObs) + " stereo bad observations", Verbose::VERBOSITY_DEBUG);

		// Optimize again without the outliers
		//Verbose::PrintMess("LM-LBA: second optimization", Verbose::VERBOSITY_DEBUG);
		optimizer.initializeOptimization(0);
		optimizer.optimize(10);
//			cout<<"second optimization"<<endl;
	}

	vector<pair<KeyFrame *, MapPoint *>> vToErase;
	vToErase.reserve(vpEdgesMono.size() + vpEdgesBody.size() + vpEdgesStereo.size());

	// Check inlier observations
	for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
		ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
		MapPoint *pMP = vpMapPointEdgeMono[i];

		if (pMP->isBad()) {
			continue;
		}

		if (e->chi2() > 5.991 || !e->isDepthPositive()) {
			KeyFrame *pKFi = vpEdgeKFMono[i];
			vToErase.push_back(make_pair(pKFi, pMP));
		}
	}

	for (size_t i = 0, iend = vpEdgesBody.size(); i < iend; i++) {
		ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = vpEdgesBody[i];
		MapPoint *pMP = vpMapPointEdgeBody[i];

		if (pMP->isBad()) {
			continue;
		}

		if (e->chi2() > 5.991 || !e->isDepthPositive()) {
			KeyFrame *pKFi = vpEdgeKFBody[i];
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

	//Verbose::PrintMess("LM-LBA: outlier observations: " + to_string(vToErase.size()), Verbose::VERBOSITY_DEBUG);
	bool bRedrawError = false;
	if (vToErase.size() >= (vpMapPointEdgeMono.size() + vpMapPointEdgeStereo.size()) * 0.5) {
		Verbose::PrintMess("LM-LBA: ERROR IN THE OPTIMIZATION, MOST OF THE POINTS HAS BECOME OUTLIERS",
		                   Verbose::VERBOSITY_NORMAL);

		return;
		bRedrawError = true;
		string folder_name = "test_LBA";
		string name = "_PreLM_LBA";
		name = "_PreLM_LBA_Fixed";
	}

	// Get Map Mutex
	unique_lock<shared_timed_mutex> lock(pMap->mMutexMapUpdate);

	if (!vToErase.empty()) {
		map<KeyFrame *, int> mspInitialConnectedKFs;
		map<KeyFrame *, int> mspInitialObservationKFs;
		if (bRedrawError) {
			for (KeyFrame *pKFi: OptDVLKFs) {

				mspInitialConnectedKFs[pKFi] = pKFi->GetConnectedKeyFrames().size();
				mspInitialObservationKFs[pKFi] = pKFi->GetNumberMPs();
			}
		}

		//cout << "LM-LBA: There are " << vToErase.size() << " observations whose will be deleted from the map" << endl;
		for (size_t i = 0; i < vToErase.size(); i++) {
			KeyFrame *pKFi = vToErase[i].first;
			MapPoint *pMPi = vToErase[i].second;
			pKFi->EraseMapPointMatch(pMPi);
			pMPi->EraseObservation(pKFi);
		}

		map<KeyFrame *, int> mspFinalConnectedKFs;
		map<KeyFrame *, int> mspFinalObservationKFs;
		if (bRedrawError) {
			ofstream f_lba;
			f_lba.open("test_LBA/LBA_failure_KF" + to_string(pKF->mnId) + ".txt");
			f_lba << "# KF id, Initial Num CovKFs, Final Num CovKFs, Initial Num MPs, Fimal Num MPs" << endl;
			f_lba << fixed;

			for (KeyFrame *pKFi: OptDVLKFs) {
				pKFi->UpdateConnections();
				int finalNumberCovKFs = pKFi->GetConnectedKeyFrames().size();
				int finalNumberMPs = pKFi->GetNumberMPs();
				f_lba << pKFi->mnId << ", " << mspInitialConnectedKFs[pKFi] << ", " << finalNumberCovKFs << ", "
				      << mspInitialObservationKFs[pKFi] << ", " << finalNumberMPs << endl;

				mspFinalConnectedKFs[pKFi] = finalNumberCovKFs;
				mspFinalObservationKFs[pKFi] = finalNumberMPs;
			}

			f_lba.close();
		}
	}

	// Recover optimized data
	//Keyframes
	bool bShowStats = false;
	for (vector<KeyFrame *>::iterator lit = OptDVLKFs.begin(), lend = OptDVLKFs.end(); lit != lend; lit++) {
		KeyFrame *pKFi = *lit;
		g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKFi->mnId));
		g2o::SE3Quat SE3quat = vSE3->estimate();
		cv::Mat Tiw = Converter::toCvMat(SE3quat);
		cv::Mat Tco_cn = pKFi->GetPose() * Tiw.inv();
		cv::Vec3d trasl = Tco_cn.rowRange(0, 3).col(3);
		double dist = cv::norm(trasl);
		pKFi->SetPose(Converter::toCvMat(SE3quat));

		if (dist > 1.0) {
			bShowStats = true;
			Verbose::PrintMess("LM-LBA: Too much distance in KF " + to_string(pKFi->mnId) + ", " + to_string(dist)
				                   + " meters. Current KF " + to_string(pKF->mnId), Verbose::VERBOSITY_DEBUG);
			Verbose::PrintMess("LM-LBA: Number of connections between the KFs " + to_string(pKF->GetWeight((pKFi))),
			                   Verbose::VERBOSITY_DEBUG);

			int numMonoMP = 0, numBadMonoMP = 0;
			int numStereoMP = 0, numBadStereoMP = 0;
			for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
				if (vpEdgeKFMono[i] != pKFi) {
					continue;
				}
				ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
				MapPoint *pMP = vpMapPointEdgeMono[i];

				if (pMP->isBad()) {
					continue;
				}

				if (e->chi2() > 5.991 || !e->isDepthPositive()) {
					numBadMonoMP++;
				}
				else {
					numMonoMP++;
				}
			}

			for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
				if (vpEdgeKFStereo[i] != pKFi) {
					continue;
				}
				g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
				MapPoint *pMP = vpMapPointEdgeStereo[i];

				if (pMP->isBad()) {
					continue;
				}

				if (e->chi2() > 7.815 || !e->isDepthPositive()) {
					numBadStereoMP++;
				}
				else {
					numStereoMP++;
				}
			}
			Verbose::PrintMess(
				"LM-LBA: Good observations in mono " + to_string(numMonoMP) + " and stereo " + to_string(numStereoMP),
				Verbose::VERBOSITY_DEBUG);
			Verbose::PrintMess("LM-LBA: Bad observations in mono " + to_string(numBadMonoMP) + " and stereo "
				                   + to_string(numBadStereoMP), Verbose::VERBOSITY_DEBUG);
		}
	}

	//Points
	for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
		MapPoint *pMP = *lit;
		g2o::VertexSBAPointXYZ
			*vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));
		pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMP->UpdateNormalAndDepth();
	}

	if (bRedrawError) {
		string folder_name = "test_LBA";
		string name = "_PostLM_LBA";
		//pMap->printReprojectionError(lLocalKeyFrames, pKF, name, folder_name);
		name = "_PostLM_LBA_Fixed";
		//pMap->printReprojectionError(lFixedCameras, pKF, name, folder_name);
	}

	// TODO Check this changeindex
	pMap->IncreaseChangeIndex();
}

void Optimizer::LocalDVLRefinement(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, int &num_fixedKF)
{
	Map *pCurrentMap = pKF->GetMap();
//	int Nd=std::min(20,(int)pCurrentMap->KeyFramesInMap()-2) ;// number of keyframes in current map
	int Nd = (int)pCurrentMap->KeyFramesInMap() - 2;// number of keyframes in current map
	const unsigned long maxKFid = pKF->mnId;

	vector<KeyFrame *> OptDVLKFs;
	const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
	list<KeyFrame *> OptVisualKFS;

	OptDVLKFs.reserve(Nd);
	OptDVLKFs.push_back(pKF);
	pKF->mnBALocalForKF = pKF->mnId;

	for (int i = 1; i < Nd; i++) {
		if (OptDVLKFs.back()->mPrevKF) {
			OptDVLKFs.push_back(OptDVLKFs.back()->mPrevKF);
			OptDVLKFs.back()->mnBALocalForKF = pKF->mnId;
		}
		else {
			break;
		}
	}
	int N = OptDVLKFs.size();

	//cout << "LBA" << endl;
	// Local KeyFrames: First Breath Search from Current Keyframe
//		list<KeyFrame *> lLocalKeyFrames;
//
//		lLocalKeyFrames.push_back(pKF);
//		pKF->mnBALocalForKF = pKF->mnId;
//
//
//
//		for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
//		{
//			KeyFrame *pKFi = vNeighKFs[i];
//			pKFi->mnBALocalForKF = pKF->mnId;
//			if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
//				lLocalKeyFrames.push_back(pKFi);
//		}

	// Local MapPoints seen in Local KeyFrames
	list<MapPoint *> lLocalMapPoints;
	for (int i = 0; i < N; i++) {
		vector<MapPoint *> vpMPs = OptDVLKFs[i]->GetMapPointMatches();
		for (vector<MapPoint *>::iterator it = vpMPs.begin(); it != vpMPs.end(); it++) {
			MapPoint *pMP = *it;
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

	// Fixed Keyframe
	list<KeyFrame *> lFixedKFs;
	if (OptDVLKFs.back()->mPrevKF) {
		lFixedKFs.push_back(OptDVLKFs.back()->mPrevKF);
		OptDVLKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
	}
	else {
		OptDVLKFs.back()->mnBALocalForKF = 0;
		OptDVLKFs.back()->mnBAFixedForKF = pKF->mnId;
		lFixedKFs.push_back(OptDVLKFs.back());
		OptDVLKFs.pop_back();
	}

	const int maxFixedKF = 2000;
	for (list<MapPoint *>::iterator it = lLocalMapPoints.begin(); it != lLocalMapPoints.end(); it++) {
		map<KeyFrame *, tuple<int, int>> observations = (*it)->GetObservations();
		for (map<KeyFrame *, tuple<int, int>>::iterator it_ob = observations.begin(); it_ob != observations.end();
		     it_ob++) {
			KeyFrame *pKFi = it_ob->first;
			if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
				pKFi->mnBAFixedForKF = pKF->mnId;
				if (!pKFi->isBad()) {
					lFixedKFs.push_back(pKFi);
					break;
				}
			}
		}
		if (lFixedKFs.size() >= maxFixedKF) {
			break;
		}
	}



	//Verbose::PrintMess("LM-LBA: There are " + to_string(lLocalKeyFrames.size()) + " KFs and " + to_string(lLocalMapPoints.size()) + " MPs to optimize. " + to_string(num_fixedKF) + " KFs are fixed", Verbose::VERBOSITY_DEBUG);

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

	g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	if (pMap->IsInertial()) {
		solver->setUserLambdaInit(100.0);
	} // TODO uncomment
	//cout << "LM-LBA: lambda init: " << solver->userLambdaInit() << endl;

	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(false);

	if (pbStopFlag) {
		optimizer.setForceStopFlag(pbStopFlag);
	}


	Eigen::Isometry3d initial_T_e_c = pKF->mT_e_c;
	fstream file;
	file.open("data/Edge_T.txt", ios::out | ios::app);
	if (!file) {
		cout << "fail to open data/Edge_T.txt" << endl;
	}
//	file<<"initial T_e_c: "<<endl;
//
//	file<<"t: "<<initial_T_e_c.translation()<<endl;
//	file<<"r: "<<initial_T_e_c.rotation().eulerAngles(0,1,2)<<endl;

//	cv::Mat initial_T_e_c_mat;
//	cv::eigen2cv(initial_T_e_c.matrix(),initial_T_e_c_mat);
	g2o::VertexSE3Expmap *vSE3_e_c = new g2o::VertexSE3Expmap();
	vSE3_e_c->setEstimate(g2o::SE3Quat(initial_T_e_c.rotation(), initial_T_e_c.translation()));
	vSE3_e_c->setId(0);
	vSE3_e_c->setFixed(false);
	optimizer.addVertex(vSE3_e_c);
//	cout<<initial_T_e_c_mat<<endl;

	for (vector<KeyFrame *>::iterator it = OptDVLKFs.begin(); it != OptDVLKFs.end(); it++) {
		if ((*it)->mPrevKF) {
			KeyFrame *p_cur = *it;
			KeyFrame *p_pre = (*it)->mPrevKF;
			p_cur->IntegrateDVL(p_pre);
			Eigen::Isometry3d T_ci_c0 = Eigen::Isometry3d::Identity();
			Eigen::Isometry3d T_cj_c0 = Eigen::Isometry3d::Identity();
			cv::cv2eigen(p_pre->GetPose(), T_ci_c0.matrix());
			cv::cv2eigen(p_cur->GetPose(), T_cj_c0.matrix());
			EdgeDVLRefine *e = new EdgeDVLRefine(p_cur->mT_ei_ej, T_ci_c0, T_cj_c0);
//			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(p_pre->mnId)));
//			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(p_cur->mnId)));
			e->setVertex(0, vSE3_e_c);
			e->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
			optimizer.addEdge(e);
		}
		else {
			cout << "no previous keyframe for DVL" << endl;
		}

	}
	//Verbose::PrintMess("LM-LBA: total observations: " + to_string(vpMapPointEdgeMono.size()+vpMapPointEdgeStereo.size()), Verbose::VERBOSITY_DEBUG);

	if (pbStopFlag) {
		if (*pbStopFlag) {
			return;
		}
	}

	optimizer.initializeOptimization();

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	optimizer.optimize(50);
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

	Eigen::Isometry3d T_e_c(vSE3_e_c->estimate());
	cout << "DVL refinement finished!" << endl;
	file << pKF->mnId << " " << T_e_c.translation().transpose() << " "
	     << T_e_c.rotation().eulerAngles(0, 1, 2).transpose() << endl;
//	file<<"r matrix: "<<T_e_c.rotation()<<endl;

	file.close();

}

void Optimizer::OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *>> &LoopConnections, const bool &bFixScale)
{
	// Setup optimizer
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

	vector<Eigen::Vector3d> vZvectors(nMaxKFid + 1); // For debugging
	Eigen::Vector3d z_vec;
	z_vec << 0.0, 0.0, 1.0;

	const int minFeat = 100; // MODIFICATION originally was set to 100

	// Set KeyFrame vertices
	for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
		KeyFrame *pKF = vpKFs[i];
		if (pKF->isBad()) {
			continue;
		}
		g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

		const int nIDi = pKF->mnId;

		LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

		if (it != CorrectedSim3.end()) {
			vScw[nIDi] = it->second;
			VSim3->setEstimate(it->second);
		}
		else {
			Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
			Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKF->GetTranslation());
			g2o::Sim3 Siw(Rcw, tcw, 1.0);
			vScw[nIDi] = Siw;
			VSim3->setEstimate(Siw);
		}

		if (pKF->mnId == pMap->GetInitKFid()) {
			VSim3->setFixed(true);
		}

		VSim3->setId(nIDi);
		VSim3->setMarginalized(false);
		VSim3->_fix_scale = bFixScale;

		optimizer.addVertex(VSim3);
		vZvectors[nIDi] = vScw[nIDi].rotation().toRotationMatrix() * z_vec; // For debugging

		vpVertices[nIDi] = VSim3;
	}

	set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

	const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

	// Set Loop edges
	int count_loop = 0;
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
			const g2o::Sim3 Sji = Sjw * Swi;

			g2o::EdgeSim3 *e = new g2o::EdgeSim3();
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
			e->setMeasurement(Sji);

			e->information() = matLambda;

			optimizer.addEdge(e);
			count_loop++;
			sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
		}
	}

	int count_spa_tree = 0;
	int count_cov = 0;
	int count_imu = 0;
	int count_kf = 0;
	// Set normal edges
	for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
		count_kf = 0;
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
            auto v1 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj));
            auto v2 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi));
            if(v1&&v2){
                e->setVertex(1, v1);
                e->setVertex(0, v2);
                e->setMeasurement(Sji);
                count_kf++;
                count_spa_tree++;
                e->information() = matLambda;
                optimizer.addEdge(e);
            }
            else
                continue;

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
                auto v1 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pLKF->mnId));
                auto v2 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi));
                if(v1&&v2){
                    el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pLKF->mnId)));
                    el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                    el->setMeasurement(Sli);
                    el->information() = matLambda;
                    optimizer.addEdge(el);
                    count_kf++;
                    count_loop++;
                }
			}
		}

		// Covisibility graph edges
		const vector<KeyFrame *> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
		for (vector<KeyFrame *>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++) {
			KeyFrame *pKFn = *vit;
			if (pKFn && pKFn != pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn)) {
				if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
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
                    auto v1 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFn->mnId));
                    auto v2 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi));
                    if(v1&&v2){
                        en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFn->mnId)));
                        en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                        en->setMeasurement(Sni);
                        en->information() = matLambda;
                        optimizer.addEdge(en);
                        count_kf++;
                        count_cov++;
                    }
				}
			}
		}

		// Inertial edges if inertial
		// if (pKF->bImu && pKF->mPrevKF) {
		// 	g2o::Sim3 Spw;
		// 	LoopClosing::KeyFrameAndPose::const_iterator itp = NonCorrectedSim3.find(pKF->mPrevKF);
		// 	if (itp != NonCorrectedSim3.end()) {
		// 		Spw = itp->second;
		// 	}
		// 	else {
		// 		Spw = vScw[pKF->mPrevKF->mnId];
		// 	}
        //
		// 	g2o::Sim3 Spi = Spw * Swi;
		// 	g2o::EdgeSim3 *ep = new g2o::EdgeSim3();
		// 	ep->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mPrevKF->mnId)));
		// 	ep->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
		// 	ep->setMeasurement(Spi);
		// 	ep->information() = matLambda;
		// 	optimizer.addEdge(ep);
		// 	count_kf++;
		// 	count_imu++;
		// }
		/*if(count_kf<3)
            cout << "EG: kf with " << count_kf << " edges!!    ID: " << pKF->mnId << endl;*/
	}

	//cout << "EG: Number of KFs: " << vpKFs.size() << endl;
	//cout << "EG: spaning tree edges: " << count_spa_tree << endl;
	//cout << "EG: Loop edges: " << count_loop << endl;
	//cout << "EG: covisible edges: " << count_cov << endl;
	//cout << "EG: imu edges: " << count_imu << endl;
	// Optimize!
	optimizer.initializeOptimization();
	optimizer.computeActiveErrors();
	float err0 = optimizer.activeRobustChi2();
	optimizer.optimize(20);
	optimizer.computeActiveErrors();
	float errEnd = optimizer.activeRobustChi2();
	//cout << "err_0/err_end: " << err0 << "/" << errEnd << endl;
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
		// cout << "angle KF " << nIDi << ": " << (180.0/3.1415)*acos(vZvectors[nIDi].dot(eigR*z_vec)) << endl;
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

void Optimizer::OptimizeEssentialGraph6DoF(KeyFrame *pCurKF,
                                           vector<KeyFrame *> &vpFixedKFs,
                                           vector<KeyFrame *> &vpFixedCorrectedKFs,
                                           vector<KeyFrame *> &vpNonFixedKFs,
                                           vector<MapPoint *> &vpNonCorrectedMPs,
                                           double scale)
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
	g2o::BlockSolver_6_3::LinearSolverType *linearSolver =
		new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

	solver->setUserLambdaInit(1e-16);
	optimizer.setAlgorithm(solver);

	Map *pMap = pCurKF->GetMap();
	const unsigned int nMaxKFid = pMap->GetMaxKFid();

	vector<g2o::SE3Quat, Eigen::aligned_allocator<g2o::SE3Quat>> vScw(nMaxKFid + 1);
	vector<g2o::SE3Quat, Eigen::aligned_allocator<g2o::SE3Quat>> vScw_bef(nMaxKFid + 1);
	vector<g2o::SE3Quat, Eigen::aligned_allocator<g2o::SE3Quat>> vCorrectedSwc(nMaxKFid + 1);
	vector<g2o::VertexSE3Expmap *> vpVertices(nMaxKFid + 1);
	vector<bool> vbFromOtherMap(nMaxKFid + 1);

	const int minFeat = 100;

	for (KeyFrame *pKFi: vpFixedKFs) {
		if (pKFi->isBad()) {
			continue;
		}

		g2o::VertexSE3Expmap *VSE3 = new g2o::VertexSE3Expmap();

		const int nIDi = pKFi->mnId;

		Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKFi->GetRotation());
		Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKFi->GetTranslation());
		g2o::SE3Quat Siw(Rcw, tcw);
		vScw[nIDi] = Siw;
		vCorrectedSwc[nIDi] = Siw.inverse(); // This KFs mustn't be corrected
		VSE3->setEstimate(Siw);

		VSE3->setFixed(true);

		VSE3->setId(nIDi);
		VSE3->setMarginalized(false);
		//VSim3->_fix_scale = true; //TODO
		vbFromOtherMap[nIDi] = false;

		optimizer.addVertex(VSE3);

		vpVertices[nIDi] = VSE3;
	}
	cout << "Opt_Essential: vpFixedKFs loaded" << endl;

	set<unsigned long> sIdKF;
	for (KeyFrame *pKFi: vpFixedCorrectedKFs) {
		if (pKFi->isBad()) {
			continue;
		}

		g2o::VertexSE3Expmap *VSE3 = new g2o::VertexSE3Expmap();

		const int nIDi = pKFi->mnId;

		Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKFi->GetRotation());
		Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKFi->GetTranslation());
		g2o::SE3Quat Siw(Rcw, tcw);
		vScw[nIDi] = Siw;
		vCorrectedSwc[nIDi] = Siw.inverse(); // This KFs mustn't be corrected
		VSE3->setEstimate(Siw);

		cv::Mat Tcw_bef = pKFi->mTcwBefMerge;
		Eigen::Matrix<double, 3, 3> Rcw_bef = Converter::toMatrix3d(Tcw_bef.rowRange(0, 3).colRange(0, 3));
		Eigen::Matrix<double, 3, 1> tcw_bef = Converter::toVector3d(Tcw_bef.rowRange(0, 3).col(3)) / scale;
		vScw_bef[nIDi] = g2o::SE3Quat(Rcw_bef, tcw_bef);

		VSE3->setFixed(true);

		VSE3->setId(nIDi);
		VSE3->setMarginalized(false);
		//VSim3->_fix_scale = true;
		vbFromOtherMap[nIDi] = true;

		optimizer.addVertex(VSE3);

		vpVertices[nIDi] = VSE3;

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

		g2o::VertexSE3Expmap *VSE3 = new g2o::VertexSE3Expmap();

		//cv::Mat Tcw = pKFi->mTcwBefMerge;
		//Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(Tcw.rowRange(0,3).colRange(0,3));
		//Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(Tcw.rowRange(0,3).col(3));
		Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKFi->GetRotation());
		Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKFi->GetTranslation()) / scale;
		g2o::SE3Quat Siw(Rcw, tcw);
		vScw_bef[nIDi] = Siw;
		VSE3->setEstimate(Siw);

		VSE3->setFixed(false);

		VSE3->setId(nIDi);
		VSE3->setMarginalized(false);
		//VSim3->_fix_scale = true;
		vbFromOtherMap[nIDi] = true;

		optimizer.addVertex(VSE3);

		vpVertices[nIDi] = VSE3;

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

	const Eigen::Matrix<double, 6, 6> matLambda = Eigen::Matrix<double, 6, 6>::Identity();

	for (KeyFrame *pKFi: vpKFs) {
		int num_connections = 0;
		const int nIDi = pKFi->mnId;

		g2o::SE3Quat Swi = vScw[nIDi].inverse();
		g2o::SE3Quat Swi_bef;
		if (vbFromOtherMap[nIDi]) {
			Swi_bef = vScw_bef[nIDi].inverse();
		}
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

			g2o::SE3Quat Sjw = vScw[nIDj];
			g2o::SE3Quat Sjw_bef;
			if (vbFromOtherMap[nIDj]) {
				Sjw_bef = vScw_bef[nIDj];
			}

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

			g2o::SE3Quat Sji;

			if (vbFromOtherMap[nIDi] && vbFromOtherMap[nIDj]) {
				Sji = Sjw_bef * Swi_bef;
			}
			else {
				Sji = Sjw * Swi;
			}

			g2o::EdgeSE3 *e = new g2o::EdgeSE3();
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
				g2o::SE3Quat Slw = vScw[pLKF->mnId];
				g2o::SE3Quat Slw_bef;
				if (vbFromOtherMap[pLKF->mnId]) {
					Slw_bef = vScw_bef[pLKF->mnId];
				}

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

				g2o::SE3Quat Sli;

				if (vbFromOtherMap[nIDi] && vbFromOtherMap[pLKF->mnId]) {
					Sli = Slw_bef * Swi_bef;
				}
				else {
					Sli = Slw * Swi;
				}

				g2o::EdgeSE3 *el = new g2o::EdgeSE3();
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
					g2o::SE3Quat Snw = vScw[pKFn->mnId];

					g2o::SE3Quat Snw_bef;
					if (vbFromOtherMap[pKFn->mnId]) {
						Snw_bef = vScw_bef[pKFn->mnId];
					}
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

					g2o::SE3Quat Sni;

					if (vbFromOtherMap[nIDi] && vbFromOtherMap[pKFn->mnId]) {
						Sni = Snw_bef * Swi_bef;
					}
					else {
						Sni = Snw * Swi;
					}

					g2o::EdgeSE3 *en = new g2o::EdgeSE3();
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

	unique_lock<shared_timed_mutex> lock(pMap->mMutexMapUpdate);

	Verbose::PrintMess("Opt_Essential: Apply the new pose to the KFs", Verbose::VERBOSITY_DEBUG);
	// SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
	for (KeyFrame *pKFi: vpNonFixedKFs) {
		if (pKFi->isBad()) {
			continue;
		}

		const int nIDi = pKFi->mnId;

		g2o::VertexSE3Expmap *VSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(nIDi));
		g2o::SE3Quat CorrectedSiw = VSE3->estimate();
		vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
		Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
		Eigen::Vector3d eigt = CorrectedSiw.translation();
		//double s = CorrectedSiw.scale();

		//eigt *=(1./s); //[R t/s;0 1]

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
	cout << "Opt_Essential: number of points -> " << vpNonCorrectedMPs.size() << endl;
	// Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
	for (MapPoint *pMPi: vpNonCorrectedMPs) {
		if (pMPi->isBad()) {
			continue;
		}

		//Verbose::PrintMess("Opt_Essential: MP id " + to_string(pMPi->mnId), Verbose::VERBOSITY_DEBUG);
		/*int nIDr;
        if(pMPi->mnCorrectedByKF==pCurKF->mnId)
        {
            nIDr = pMPi->mnCorrectedReference;
        }
        else
        {

        }*/
		KeyFrame *pRefKF = pMPi->GetReferenceKeyFrame();
		g2o::SE3Quat Srw;
		g2o::SE3Quat correctedSwr;
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
		//cv::Mat TNonCorrectedwr = pRefKF->mTwcBefMerge;
		//Eigen::Matrix<double,3,3> RNonCorrectedwr = Converter::toMatrix3d(TNonCorrectedwr.rowRange(0,3).colRange(0,3));
		//Eigen::Matrix<double,3,1> tNonCorrectedwr = Converter::toVector3d(TNonCorrectedwr.rowRange(0,3).col(3));
		Srw = vScw_bef[pRefKF->mnId]; //g2o::SE3Quat(RNonCorrectedwr,tNonCorrectedwr).inverse();

		cv::Mat Twr = pRefKF->GetPoseInverse();
		Eigen::Matrix<double, 3, 3> Rwr = Converter::toMatrix3d(Twr.rowRange(0, 3).colRange(0, 3));
		Eigen::Matrix<double, 3, 1> twr = Converter::toVector3d(Twr.rowRange(0, 3).col(3));
		correctedSwr = g2o::SE3Quat(Rwr, twr);
		//}
		//cout << "Opt_Essential: Loaded the KF reference position" << endl;

		cv::Mat P3Dw = pMPi->GetWorldPos() / scale;
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

} // namespace ORB_SLAM3
