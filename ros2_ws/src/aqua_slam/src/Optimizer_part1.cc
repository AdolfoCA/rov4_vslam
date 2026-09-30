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

void Optimizer::GlobalBundleAdjustemnt(Map *pMap,
                                       int nIterations,
                                       bool *pbStopFlag,
                                       const unsigned long nLoopKF,
                                       const bool bRobust)
{
	vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
	vector<MapPoint *> vpMP = pMap->GetAllMapPoints();
	BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
}

// vpKFs: the keyframe(transformation) to optimiza
// vpMP: observed map points
void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                 int nIterations, bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
	// whether a map point has connection to a keyframe
	vector<bool> vbNotIncludedMP;
	vbNotIncludedMP.resize(vpMP.size());

	// Map : a set of map points and key frames
	Map *pMap = vpKFs[0]->GetMap();

	// set up g2o solver
	g2o::SparseOptimizer optimizer;

	// ??? what does 3 means
	// 6: 6 variables to optimize(6-degree-pose), 3: dimension of error(camera tranlation)
	g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

	g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(false);
	// set up g2o solver finished

	// ????
	if (pbStopFlag) {
		optimizer.setForceStopFlag(pbStopFlag);
	}

	// the max id of Keyframe
	long unsigned int maxKFid = 0;

	// the number of all edges
	const int nExpectedSize = (vpKFs.size()) * vpMP.size();

	// edges between map points(3) and camera pose(6)
	// for monocular
	vector<ORB_SLAM3::EdgeSE3ProjectXYZ *> vpEdgesMono;
	vpEdgesMono.reserve(nExpectedSize);

	// edges between map points(3) and camera pose(6)
	// for stereo, and only for those
	vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody *> vpEdgesBody;
	vpEdgesBody.reserve(nExpectedSize);

	// points to keyframes
	// ?? why allocate nExpectedSize(num_map_points*num_keyframe)
	vector<KeyFrame *> vpEdgeKFMono;
	vpEdgeKFMono.reserve(nExpectedSize);
	// ?? why allocate nExpectedSize(num_map_points*num_keyframe)
	vector<KeyFrame *> vpEdgeKFBody;
	vpEdgeKFBody.reserve(nExpectedSize);

	// points to mappoints
	// ?? why allocate nExpectedSize(num_map_points*num_keyframe)
	vector<MapPoint *> vpMapPointEdgeMono;
	vpMapPointEdgeMono.reserve(nExpectedSize);
	// ?? why allocate nExpectedSize(num_map_points*num_keyframe)
	vector<MapPoint *> vpMapPointEdgeBody;
	vpMapPointEdgeBody.reserve(nExpectedSize);

	// stereo edge
	vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
	vpEdgesStereo.reserve(nExpectedSize);
	// stereo keyframe
	vector<KeyFrame *> vpEdgeKFStereo;
	vpEdgeKFStereo.reserve(nExpectedSize);
	// stereo map points
	vector<MapPoint *> vpMapPointEdgeStereo;
	vpMapPointEdgeStereo.reserve(nExpectedSize);

	// Set KeyFrame vertices

	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKF = vpKFs[i];
		if (pKF->isBad()) {
			continue;
		}
		// add camera pose as vertex to the graph
		g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
		vSE3->setId(pKF->mnId);
		// set the first frame as fixed, because we do not optimize the first frame.
		vSE3->setFixed(pKF->mnId == pMap->GetInitKFid());
		optimizer.addVertex(vSE3);
		if (pKF->mnId > maxKFid) {
			maxKFid = pKF->mnId;
		}
		//cout << "KF id: " << pKF->mnId << endl;
	}

	// RobustKernelHuber parameter
	const float thHuber2D = sqrt(5.99);
	const float thHuber3D = sqrt(7.815);

	// Set MapPoint vertices
	//cout << "start inserting MPs" << endl;

	for (size_t i = 0; i < vpMP.size(); i++) {
		MapPoint *pMP = vpMP[i];
		// ignore bad points
		if (pMP->isBad()) {
			continue;
		}

		// add map points to graph,
		g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
		const int id = pMP->mnId + maxKFid + 1;
		vPoint->setId(id);
		// do not optimize map points
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);

		// when using monocular, first int: map point index
		// when using stereo, first int: left index, second int: right index
		const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

		int nEdges = 0;
		//SET EDGES
		for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(); mit != observations.end();
		     mit++) {
			KeyFrame *pKF = mit->first;
			// ignore bad frame, ignore out bound Key frame
			if (pKF->isBad() || pKF->mnId > maxKFid) {
				continue;
			}
			// ignore map points which are not in the graph
			if (optimizer.vertex(id) == NULL || optimizer.vertex(pKF->mnId) == NULL) {
				continue;
			}
			nEdges++;

			// number of features in left(stereo)
			const int leftIndex = get<0>(mit->second);

			// using monocular
			if (leftIndex != -1 && pKF->mvuRight[get<0>(mit->second)] < 0) {
				//undistorted feature points
				const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];
				//set observation
				Eigen::Matrix<double, 2, 1> obs;
				obs << kpUn.pt.x, kpUn.pt.y;

				ORB_SLAM3::EdgeSE3ProjectXYZ *e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
				e->setMeasurement(obs);

				// set information according to octave(pyramid layer)/distance
				const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
				e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

				if (bRobust) {
					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuber2D);
				}

				e->pCamera = pKF->mpCamera;

				optimizer.addEdge(e);

				vpEdgesMono.push_back(e);
				vpEdgeKFMono.push_back(pKF);
				vpMapPointEdgeMono.push_back(pMP);
			}
				// using stereo,
				// for the features are both in the left image and the right image
			else if (leftIndex != -1 && pKF->mvuRight[leftIndex] >= 0) //Stereo observation
			{
				//undistorted feature points
				const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];

				Eigen::Matrix<double, 3, 1> obs;
				// get<0>(mit->second): leftindex
				// kp_ur: the position of x in thr right image
				const float kp_ur = pKF->mvuRight[get<0>(mit->second)];
				obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

				g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
				e->setMeasurement(obs);
				// set information according to octave(pyramid layer)/distance
				const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
				Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
				e->setInformation(Info);

				if (bRobust) {
					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuber3D);
				}

				e->fx = pKF->fx;
				e->fy = pKF->fy;
				e->cx = pKF->cx;
				e->cy = pKF->cy;
				e->bf = pKF->mbf;

				optimizer.addEdge(e);

				vpEdgesStereo.push_back(e);
				vpEdgeKFStereo.push_back(pKF);
				vpMapPointEdgeStereo.push_back(pMP);
			}

			// using stereo,
			// for the features are only in the right image
			if (pKF->mpCamera2) {
				int rightIndex = get<1>(mit->second);

				if (rightIndex != -1 && rightIndex < pKF->mvKeysRight.size()) {
					rightIndex -= pKF->NLeft;

					Eigen::Matrix<double, 2, 1> obs;
					cv::KeyPoint kp = pKF->mvKeysRight[rightIndex];
					obs << kp.pt.x, kp.pt.y;

					ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
					e->setMeasurement(obs);
					const float &invSigma2 = pKF->mvInvLevelSigma2[kp.octave];
					e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuber2D);

					e->mTrl = Converter::toSE3Quat(pKF->mTrl);

					e->pCamera = pKF->mpCamera2;

					optimizer.addEdge(e);
					vpEdgesBody.push_back(e);
					vpEdgeKFBody.push_back(pKF);
					vpMapPointEdgeBody.push_back(pMP);
				}
			}
		}

		if (nEdges == 0) {
			optimizer.removeVertex(vPoint);
			vbNotIncludedMP[i] = true;
		}
		else {
			vbNotIncludedMP[i] = false;
		}
	}

	//cout << "end inserting MPs" << endl;
	// Optimize!
	optimizer.setVerbose(false);
	optimizer.initializeOptimization();
	optimizer.optimize(nIterations);
	Verbose::PrintMess("BA: End of the optimization", Verbose::VERBOSITY_NORMAL);

	// Recover optimized data

	//Keyframes
	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKF = vpKFs[i];
		if (pKF->isBad()) {
			continue;
		}
		g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));

		g2o::SE3Quat SE3quat = vSE3->estimate();

		// ???? what is nLoopKF
		// if current frame is the first keyframe of this map
		if (nLoopKF == pMap->GetOriginKF()->mnId) {
			pKF->SetPose(Converter::toCvMat(SE3quat));
		}
		else {
			/*if(!vSE3->fixed())
            {
                //cout << "KF " << pKF->mnId << ": " << endl;
                pKF->mHessianPose = cv::Mat(6, 6, CV_64F);
                pKF->mbHasHessian = true;
                for(int r=0; r<6; ++r)
                {
                    for(int c=0; c<6; ++c)
                    {
                        //cout  << vSE3->hessian(r, c) << ", ";
                        pKF->mHessianPose.at<double>(r, c) = vSE3->hessian(r, c);
                    }
                    //cout << endl;
                }
            }*/

			//????? in else, where to uodate the pose of keyframe

			// optimizaed transformation matrix from world to camera
			pKF->mTcwGBA.create(4, 4, CV_32F);
			Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
			pKF->mnBAGlobalForKF = nLoopKF;

			// the inverse of previous transformation matrix from world to camera
			cv::Mat mTwc = pKF->GetPoseInverse();
			// transformation matrix from previous camera to optimized camera
			cv::Mat mTcGBA_c = pKF->mTcwGBA * mTwc;
			cv::Vec3d vector_dist = mTcGBA_c.rowRange(0, 3).col(3);
			double dist = cv::norm(vector_dist);
			if (dist > 1) {
				int numMonoBadPoints = 0, numMonoOptPoints = 0;
				int numStereoBadPoints = 0, numStereoOptPoints = 0;
				vector<MapPoint *> vpMonoMPsOpt, vpStereoMPsOpt;

				//using mono
				// iterate all edges, find edges include current keyframe
				// and find all map points connected to current keyframe, and add to vpMonoMPsOpt
				for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
					ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
					MapPoint *pMP = vpMapPointEdgeMono[i];
					KeyFrame *pKFedge = vpEdgeKFMono[i];

					if (pKF != pKFedge) {
						continue;
					}

					if (pMP->isBad()) {
						continue;
					}

					if (e->chi2() > 5.991 || !e->isDepthPositive()) {
						numMonoBadPoints++;
					}
					else {
						numMonoOptPoints++;
						vpMonoMPsOpt.push_back(pMP);
					}
				}

				// using stereo
				// iterate all edges, find edges include current keyframe
				// and find all map points connected to current keyframe, and add to vpMonoMPsOpt
				for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
					g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
					MapPoint *pMP = vpMapPointEdgeStereo[i];
					KeyFrame *pKFedge = vpEdgeKFMono[i];

					if (pKF != pKFedge) {
						continue;
					}

					if (pMP->isBad()) {
						continue;
					}

					if (e->chi2() > 7.815 || !e->isDepthPositive()) {
						numStereoBadPoints++;
					}
					else {
						numStereoOptPoints++;
						vpStereoMPsOpt.push_back(pMP);
					}
				}
				Verbose::PrintMess("GBA: KF " + to_string(pKF->mnId) + " had been moved " + to_string(dist) + " meters",
				                   Verbose::VERBOSITY_DEBUG);
				Verbose::PrintMess("--Number of observations: " + to_string(numMonoOptPoints) + " in mono and "
					                   + to_string(numStereoOptPoints) + " in stereo", Verbose::VERBOSITY_DEBUG);
				Verbose::PrintMess(
					"--Number of discarded observations: " + to_string(numMonoBadPoints) + " in mono and "
						+ to_string(numStereoBadPoints) + " in stereo", Verbose::VERBOSITY_DEBUG);
			}
		}
	}
	Verbose::PrintMess("BA: KFs updated", Verbose::VERBOSITY_DEBUG);

	//Points
	for (size_t i = 0; i < vpMP.size(); i++) {
		// ???
		if (vbNotIncludedMP[i]) {
			continue;
		}

		MapPoint *pMP = vpMP[i];

		if (pMP->isBad()) {
			continue;
		}
		g2o::VertexSBAPointXYZ
			*vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));

		// if current frame is the first keyframe of this map
		if (nLoopKF == pMap->GetOriginKF()->mnId) {
			pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
			pMP->UpdateNormalAndDepth();
		}
		else {
			pMP->mPosGBA.create(3, 1, CV_32F);
			// ????
			Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
			pMP->mnBAGlobalForKF = nLoopKF;
		}
	}
}

void Optimizer::FullInertialBA(Map *pMap,
                               int its,
                               const bool bFixLocal,
                               const long unsigned int nLoopId,
                               bool *pbStopFlag,
                               bool bInit,
                               float priorG,
                               float priorA,
                               Eigen::VectorXd *vSingVal,
                               bool *bHess)
{
	long unsigned int maxKFid = pMap->GetMaxKFid();
	const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
	const vector<MapPoint *> vpMPs = pMap->GetAllMapPoints();

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	solver->setUserLambdaInit(1e-5);
	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(false);

	if (pbStopFlag) {
		optimizer.setForceStopFlag(pbStopFlag);
	}

	int nNonFixed = 0;

	// Set KeyFrame vertices
	KeyFrame *pIncKF;
	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}
		VertexPose *VP = new VertexPose(pKFi);
		VP->setId(pKFi->mnId);
		pIncKF = pKFi;
		bool bFixed = false;
		if (bFixLocal) {
			bFixed = (pKFi->mnBALocalForKF >= (maxKFid - 1)) || (pKFi->mnBAFixedForKF >= (maxKFid - 1));
			if (!bFixed) {
				nNonFixed++;
			}
			VP->setFixed(bFixed);
		}
		optimizer.addVertex(VP);

		if (pKFi->bImu) {
			VertexVelocity *VV = new VertexVelocity(pKFi);
			VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
			VV->setFixed(bFixed);
			optimizer.addVertex(VV);
			if (!bInit) {
				VertexGyroBias *VG = new VertexGyroBias(pKFi);
				VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
				VG->setFixed(bFixed);
				optimizer.addVertex(VG);
				VertexAccBias *VA = new VertexAccBias(pKFi);
				VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
				VA->setFixed(bFixed);
				optimizer.addVertex(VA);
			}
		}
	}

	if (bInit) {
		VertexGyroBias *VG = new VertexGyroBias(pIncKF);
		VG->setId(4 * maxKFid + 2);
		VG->setFixed(false);
		optimizer.addVertex(VG);
		VertexAccBias *VA = new VertexAccBias(pIncKF);
		VA->setId(4 * maxKFid + 3);
		VA->setFixed(false);
		optimizer.addVertex(VA);
	}

	if (bFixLocal) {
		if (nNonFixed < 3) {
			return;
		}
	}

	// IMU links
	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];

		if (!pKFi->mPrevKF) {
			Verbose::PrintMess("NOT INERTIAL LINK TO PREVIOUS FRAME!", Verbose::VERBOSITY_NORMAL);
			continue;
		}

		if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
			if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) {
				continue;
			}
			if (pKFi->bImu && pKFi->mPrevKF->bImu) {
				pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
				g2o::HyperGraph::Vertex *VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
				g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);

				g2o::HyperGraph::Vertex *VG1;
				g2o::HyperGraph::Vertex *VA1;
				g2o::HyperGraph::Vertex *VG2;
				g2o::HyperGraph::Vertex *VA2;
				if (!bInit) {
					VG1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
					VA1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
					VG2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
					VA2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);
				}
				else {
					VG1 = optimizer.vertex(4 * maxKFid + 2);
					VA1 = optimizer.vertex(4 * maxKFid + 3);
				}

				g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(pKFi->mnId);
				g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);

				if (!bInit) {
					if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
						cout << "Error" << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1 << ", " << VP2 << ", "
						     << VV2 << ", " << VG2 << ", " << VA2 << endl;

						continue;
					}
				}
				else {
					if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2) {
						cout << "Error" << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1 << ", " << VP2 << ", "
						     << VV2 << endl;

						continue;
					}
				}

				EdgeInertial *ei = new EdgeInertial(pKFi->mpImuPreintegrated);
				ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
				ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
				ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG1));
				ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA1));
				ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
				ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));

				g2o::RobustKernelHuber *rki = new g2o::RobustKernelHuber;
				ei->setRobustKernel(rki);
				rki->setDelta(sqrt(16.92));

				optimizer.addEdge(ei);

				if (!bInit) {
					EdgeGyroRW *egr = new EdgeGyroRW();
					egr->setVertex(0, VG1);
					egr->setVertex(1, VG2);
					cv::Mat cvInfoG = pKFi->mpImuPreintegrated->C.rowRange(9, 12).colRange(9, 12).inv(cv::DECOMP_SVD);
					Eigen::Matrix3d InfoG;
					for (int r = 0; r < 3; r++)
						for (int c = 0; c < 3; c++)
							InfoG(r, c) = cvInfoG.at<float>(r, c);
					egr->setInformation(InfoG);
					egr->computeError();
					optimizer.addEdge(egr);

					EdgeAccRW *ear = new EdgeAccRW();
					ear->setVertex(0, VA1);
					ear->setVertex(1, VA2);
					cv::Mat cvInfoA = pKFi->mpImuPreintegrated->C.rowRange(12, 15).colRange(12, 15).inv(cv::DECOMP_SVD);
					Eigen::Matrix3d InfoA;
					for (int r = 0; r < 3; r++)
						for (int c = 0; c < 3; c++)
							InfoA(r, c) = cvInfoA.at<float>(r, c);
					ear->setInformation(InfoA);
					ear->computeError();
					optimizer.addEdge(ear);
				}
			}
			else {
				cout << pKFi->mnId << " or " << pKFi->mPrevKF->mnId << " no imu" << endl;
			}
		}
	}

	if (bInit) {
		g2o::HyperGraph::Vertex *VG = optimizer.vertex(4 * maxKFid + 2);
		g2o::HyperGraph::Vertex *VA = optimizer.vertex(4 * maxKFid + 3);

		// Add prior to comon biases
		EdgePriorAcc *epa = new EdgePriorAcc(cv::Mat::zeros(3, 1, CV_32F));
		epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
		double infoPriorA = priorA; //
		epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
		optimizer.addEdge(epa);

		EdgePriorGyro *epg = new EdgePriorGyro(cv::Mat::zeros(3, 1, CV_32F));
		epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
		double infoPriorG = priorG; //
		epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
		optimizer.addEdge(epg);
	}

	const float thHuberMono = sqrt(5.991);
	const float thHuberStereo = sqrt(7.815);

	const unsigned long iniMPid = maxKFid * 5;

	vector<bool> vbNotIncludedMP(vpMPs.size(), false);

	for (size_t i = 0; i < vpMPs.size(); i++) {
		MapPoint *pMP = vpMPs[i];
		g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
		unsigned long id = pMP->mnId + iniMPid + 1;
		vPoint->setId(id);
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);

		const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

		bool bAllFixed = true;

		//Set edges
		for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(), mend = observations.end();
		     mit != mend; mit++) {
			KeyFrame *pKFi = mit->first;

			if (pKFi->mnId > maxKFid) {
				continue;
			}

			if (!pKFi->isBad()) {
				const int leftIndex = get<0>(mit->second);
				cv::KeyPoint kpUn;

				if (leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)] < 0) // Monocular observation
				{
					kpUn = pKFi->mvKeysUn[leftIndex];
					Eigen::Matrix<double, 2, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y;

					EdgeMono *e = new EdgeMono(0);

					g2o::OptimizableGraph::Vertex
						*VP = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId));
					if (bAllFixed) {
						if (!VP->fixed()) {
							bAllFixed = false;
						}
					}

					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
					e->setVertex(1, VP);
					e->setMeasurement(obs);
					const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

					e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberMono);

					optimizer.addEdge(e);
				}
				else if (leftIndex != -1 && pKFi->mvuRight[leftIndex] >= 0) // stereo observation
				{
					kpUn = pKFi->mvKeysUn[leftIndex];
					const float kp_ur = pKFi->mvuRight[leftIndex];
					Eigen::Matrix<double, 3, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

					EdgeStereo *e = new EdgeStereo(0);

					g2o::OptimizableGraph::Vertex
						*VP = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId));
					if (bAllFixed) {
						if (!VP->fixed()) {
							bAllFixed = false;
						}
					}

					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
					e->setVertex(1, VP);
					e->setMeasurement(obs);
					const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

					e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuberStereo);

					optimizer.addEdge(e);
				}

				if (pKFi->mpCamera2) { // Monocular right observation
					int rightIndex = get<1>(mit->second);

					if (rightIndex != -1 && rightIndex < pKFi->mvKeysRight.size()) {
						rightIndex -= pKFi->NLeft;

						Eigen::Matrix<double, 2, 1> obs;
						kpUn = pKFi->mvKeysRight[rightIndex];
						obs << kpUn.pt.x, kpUn.pt.y;

						EdgeMono *e = new EdgeMono(1);

						g2o::OptimizableGraph::Vertex
							*VP = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId));
						if (bAllFixed) {
							if (!VP->fixed()) {
								bAllFixed = false;
							}
						}

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
						e->setVertex(1, VP);
						e->setMeasurement(obs);
						const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
						e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

						g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(thHuberMono);

						optimizer.addEdge(e);
					}
				}
			}
		}

		if (bAllFixed) {
			optimizer.removeVertex(vPoint);
			vbNotIncludedMP[i] = true;
		}
	}

	if (pbStopFlag) {
		if (*pbStopFlag) {
			return;
		}
	}

	optimizer.initializeOptimization();
	optimizer.optimize(its);

	// Recover optimized data
	//Keyframes
	for (size_t i = 0; i < vpKFs.size(); i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}
		VertexPose *VP = static_cast<VertexPose *>(optimizer.vertex(pKFi->mnId));
		if (nLoopId == 0) {
			cv::Mat Tcw = Converter::toCvSE3(VP->estimate().Rcw[0], VP->estimate().tcw[0]);
			pKFi->SetPose(Tcw);
		}
		else {
			pKFi->mTcwGBA = cv::Mat::eye(4, 4, CV_32F);
			Converter::toCvMat(VP->estimate().Rcw[0]).copyTo(pKFi->mTcwGBA.rowRange(0, 3).colRange(0, 3));
			Converter::toCvMat(VP->estimate().tcw[0]).copyTo(pKFi->mTcwGBA.rowRange(0, 3).col(3));
			pKFi->mnBAGlobalForKF = nLoopId;
		}
		if (pKFi->bImu) {
			VertexVelocity *VV = static_cast<VertexVelocity *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
			if (nLoopId == 0) {
				pKFi->SetVelocity(Converter::toCvMat(VV->estimate()));
			}
			else {
				pKFi->mVwbGBA = Converter::toCvMat(VV->estimate());
			}

			VertexGyroBias *VG;
			VertexAccBias *VA;
			if (!bInit) {
				VG = static_cast<VertexGyroBias *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
				VA = static_cast<VertexAccBias *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
			}
			else {
				VG = static_cast<VertexGyroBias *>(optimizer.vertex(4 * maxKFid + 2));
				VA = static_cast<VertexAccBias *>(optimizer.vertex(4 * maxKFid + 3));
			}

			Vector6d vb;
			vb << VG->estimate(), VA->estimate();
			IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
			if (nLoopId == 0) {
				pKFi->SetNewBias(b);
			}
			else {
				pKFi->mBiasGBA = b;
			}
		}
	}

	//Points
	for (size_t i = 0; i < vpMPs.size(); i++) {
		if (vbNotIncludedMP[i]) {
			continue;
		}

		MapPoint *pMP = vpMPs[i];
		g2o::VertexSBAPointXYZ
			*vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + iniMPid + 1));

		if (nLoopId == 0) {
			pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
			pMP->UpdateNormalAndDepth();
		}
		else {
			pMP->mPosGBA.create(3, 1, CV_32F);
			Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
			pMP->mnBAGlobalForKF = nLoopId;
		}
	}

	pMap->IncreaseChangeIndex();
}

int Optimizer::PoseOptimization(Frame *pFrame)
{
	// setup solver
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

	g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);

	// number of edges
	int nInitialCorrespondences = 0;

	// Set Frame vertex
	g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
	vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
	vSE3->setId(0);
	vSE3->setFixed(false);
	optimizer.addVertex(vSE3);

	// Set MapPoint vertices

	// Number of KeyPoints.
	const int N = pFrame->N;

	// setup edges pointer
	vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *> vpEdgesMono;
	vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody *> vpEdgesMono_FHR;
	vector<size_t> vnIndexEdgeMono, vnIndexEdgeRight;
	vpEdgesMono.reserve(N);
	vpEdgesMono_FHR.reserve(N);
	vnIndexEdgeMono.reserve(N);
	vnIndexEdgeRight.reserve(N);

	vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> vpEdgesStereo;
	vector<size_t> vnIndexEdgeStereo;
	vpEdgesStereo.reserve(N);
	vnIndexEdgeStereo.reserve(N);

	// robust kernel
	const float deltaMono = sqrt(5.991);
	const float deltaStereo = sqrt(7.815);

	// add edges
	{
		unique_lock<mutex> lock(MapPoint::mGlobalMutex);

		// iterate all key points
		// N: Number of KeyPoints in the frame
		for (int i = 0; i < N; i++) {
			MapPoint *pMP = pFrame->mvpMapPoints[i];
			// if the key points has its correspending map points created
			if (pMP) {
				//Conventional SLAM
				if (!pFrame->mpCamera2) {
					// Monocular observation
					// add edge
					if (pFrame->mvuRight[i] < 0) {
						nInitialCorrespondences++;
						pFrame->mvbOutlier[i] = false;

						Eigen::Matrix<double, 2, 1> obs;
						//undistorted feature points
						const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
						obs << kpUn.pt.x, kpUn.pt.y;

						ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
						e->setMeasurement(obs);
						const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
						e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

						g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(deltaMono);

						e->pCamera = pFrame->mpCamera;
						// xw: position of the map point under world coordinate
						cv::Mat Xw = pMP->GetWorldPos();
						e->Xw[0] = Xw.at<float>(0);
						e->Xw[1] = Xw.at<float>(1);
						e->Xw[2] = Xw.at<float>(2);

						optimizer.addEdge(e);

						vpEdgesMono.push_back(e);
						vnIndexEdgeMono.push_back(i);
					}
						// Stereo observation
						// add edge, for those key ponits in both left and right
					else {
						nInitialCorrespondences++;
						pFrame->mvbOutlier[i] = false;

						//SET EDGE
						Eigen::Matrix<double, 3, 1> obs;

						//undistorted feature points
						const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
						// kp_ur: the position of x in thr right image
						const float &kp_ur = pFrame->mvuRight[i];
						obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

						g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
						e->setMeasurement(obs);
						const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
						Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
						e->setInformation(Info);

						g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(deltaStereo);

						e->fx = pFrame->fx;
						e->fy = pFrame->fy;
						e->cx = pFrame->cx;
						e->cy = pFrame->cy;
						e->bf = pFrame->mbf;
						cv::Mat Xw = pMP->GetWorldPos();
						e->Xw[0] = Xw.at<float>(0);
						e->Xw[1] = Xw.at<float>(1);
						e->Xw[2] = Xw.at<float>(2);

						optimizer.addEdge(e);

						vpEdgesStereo.push_back(e);
						vnIndexEdgeStereo.push_back(i);
					}
				}
					// SLAM with respect a rigid body
					// using stereo, only in one image
					// add edge, for those key ponits only in left or only in right
				else {
					nInitialCorrespondences++;

					cv::KeyPoint kpUn;

					if (i < pFrame->Nleft) { //Left camera observation
						kpUn = pFrame->mvKeys[i];

						pFrame->mvbOutlier[i] = false;

						Eigen::Matrix<double, 2, 1> obs;
						obs << kpUn.pt.x, kpUn.pt.y;

						ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
						e->setMeasurement(obs);
						const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
						e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

						g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(deltaMono);

						e->pCamera = pFrame->mpCamera;
						cv::Mat Xw = pMP->GetWorldPos();
						e->Xw[0] = Xw.at<float>(0);
						e->Xw[1] = Xw.at<float>(1);
						e->Xw[2] = Xw.at<float>(2);

						optimizer.addEdge(e);

						vpEdgesMono.push_back(e);
						vnIndexEdgeMono.push_back(i);
					}
					else { //Right camera observation
						//continue;
						kpUn = pFrame->mvKeysRight[i - pFrame->Nleft];

						Eigen::Matrix<double, 2, 1> obs;
						obs << kpUn.pt.x, kpUn.pt.y;

						pFrame->mvbOutlier[i] = false;

						ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody
							*e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
						e->setMeasurement(obs);
						const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
						e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

						g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(deltaMono);

						e->pCamera = pFrame->mpCamera2;
						cv::Mat Xw = pMP->GetWorldPos();
						e->Xw[0] = Xw.at<float>(0);
						e->Xw[1] = Xw.at<float>(1);
						e->Xw[2] = Xw.at<float>(2);

						e->mTrl = Converter::toSE3Quat(pFrame->mTrl);

						optimizer.addEdge(e);

						vpEdgesMono_FHR.push_back(e);
						vnIndexEdgeRight.push_back(i);
					}
				}
			}
		}
	}

	//cout << "PO: vnIndexEdgeMono.size() = " << vnIndexEdgeMono.size() << "   vnIndexEdgeRight.size() = " << vnIndexEdgeRight.size() << endl;
	if (nInitialCorrespondences < 3) {
		return 0;
	}

	// We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
	// At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
	const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
	const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
	const int its[4] = {10, 10, 10, 10};

	int nBad = 0;
	for (size_t it = 0; it < 4; it++) {
		// set estimate of camera vertex as the pose of current frame
		vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
		optimizer.initializeOptimization(0);
		optimizer.optimize(its[it]);

		nBad = 0;

		// 3 for loop for check outlier

		// using mono, or using stereo but feature only in left image
		for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
			ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *e = vpEdgesMono[i];

			const size_t idx = vnIndexEdgeMono[i];

			if (pFrame->mvbOutlier[idx]) {
				e->computeError();
			}

			// ??? what is the meaning of chi2
			const float chi2 = e->chi2();

			if (chi2 > chi2Mono[it]) {
				pFrame->mvbOutlier[idx] = true;
				// ???? what does level work?
				e->setLevel(1);
				nBad++;
			}
			else {
				pFrame->mvbOutlier[idx] = false;
				e->setLevel(0);
			}

			if (it == 2) {
				e->setRobustKernel(0);
			}
		}
		// using stereo, but feature only in right
		for (size_t i = 0, iend = vpEdgesMono_FHR.size(); i < iend; i++) {
			ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody *e = vpEdgesMono_FHR[i];

			const size_t idx = vnIndexEdgeRight[i];

			if (pFrame->mvbOutlier[idx]) {
				e->computeError();
			}

			const float chi2 = e->chi2();

			if (chi2 > chi2Mono[it]) {
				pFrame->mvbOutlier[idx] = true;
				e->setLevel(1);
				nBad++;
			}
			else {
				pFrame->mvbOutlier[idx] = false;
				e->setLevel(0);
			}

			if (it == 2) {
				e->setRobustKernel(0);
			}
		}
		// using stereo, feature in both left and right image
		for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
			g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = vpEdgesStereo[i];

			const size_t idx = vnIndexEdgeStereo[i];

			if (pFrame->mvbOutlier[idx]) {
				e->computeError();
			}

			const float chi2 = e->chi2();

			if (chi2 > chi2Stereo[it]) {
				pFrame->mvbOutlier[idx] = true;
				e->setLevel(1);
				nBad++;
			}
			else {
				e->setLevel(0);
				pFrame->mvbOutlier[idx] = false;
			}

			if (it == 2) {
				e->setRobustKernel(0);
			}
		}

		if (optimizer.edges().size() < 10) {
			break;
		}
	}

	// Recover optimized pose and return number of inliers
	g2o::VertexSE3Expmap *vSE3_recov = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
	g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
	cv::Mat pose = Converter::toCvMat(SE3quat_recov);
	pFrame->SetPose(pose);

	//cout << "[PoseOptimization]: initial correspondences-> " << nInitialCorrespondences << " --- outliers-> " << nBad << endl;

	return nInitialCorrespondences - nBad;
}

void Optimizer::PoseOnlyOptimizationDVLIMU(set<KeyFrame*, KFComparator> &loss_kfs, Atlas* pAtlas, int& optimized_kf_id)
{
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    optimizer.setAlgorithm(solver);


    // Set KeyFrame vertices (fixed poses and optimizable velocities)
    long maxKFid = (*loss_kfs.rbegin())->mnId;


	stringstream ss;
	ss << "DVL_IMU_PoseOnly optimization: " << endl;
    // iterate ls_kf from last to first, and add vertex to optimizer, and set the last 10 of them as fixed
    for (auto rit = loss_kfs.rbegin(); rit != loss_kfs.rend(); ++rit) {
        KeyFrame* pKFi = *rit;
        if (pKFi->isBad()){
            ROS_ERROR_STREAM("bad KF");
            assert(-1);
        }
        if(pKFi->GetPose().empty()){
            ROS_ERROR_STREAM("empty pose");
            continue;
        }
        VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
        VP->setId(pKFi->mnId);
        if (pKFi->mnId > (maxKFid-4)){
            VP->setFixed(false);
            ss << "optimize: " << pKFi->mnId << endl;
        }
        else {
            VP->setFixed(true);
            ss << "fixed: " << pKFi->mnId << endl;
        }
        // if (pKFi->mnId == (*loss_kfs.rbegin())->mnId) {
        //     VP->setFixed(false);
		// 	ss << "optimizable: " << pKFi->mnId << endl;
        // }
        // else if (pKFi->mnId == (*loss_kfs.begin())->mnId) {
        //     ss << "fixed: " << pKFi->mnId << endl;
        //     VP->setFixed(true);
        // }
		// else if (pKFi->mnId > (optimized_kf_id-4)) {
		// 	VP->setFixed(false);
		// 	ss << "optimizable: " << pKFi->mnId << endl;
		// }
        // else{
        //     VP->setFixed(true);
        //     ss << "fixed: " << pKFi->mnId << endl;
        // }
        optimizer.addVertex(VP);
    }
    // ROS_INFO_STREAM(ss.str());
    optimized_kf_id = maxKFid;


    // set<KeyFrame*, KFComparator>::reverse_iterator rit;
    // for (rit = loss_kfs.rbegin(); rit != loss_kfs.rend(); ++rit) {
    //     KeyFrame* pKFi = *rit;
    //     if (pKFi->isBad()){
    //         ROS_ERROR_STREAM("bad KF");
    //         assert(-1);
    //     }
    //     VertexPoseDvlGro *VP = new VertexPoseDvlGro(pKFi);
    //     VP->setId(pKFi->mnId);
    //     VP->setFixed(true);
    //     if (pKFi->mnId == (*loss_kfs.rbegin())->mnId) {
    //         VP->setFixed(false);
    //     }
    //     optimizer.addVertex(VP);
    // }
    // for(auto pKFi:loss_kfs) {
    //     if (pKFi->isBad()){
    //         ROS_ERROR_STREAM("bad KF");
    //         assert(-1);
    //     }
    //     VertexPoseDvlGro *VP = new VertexPoseDvlGro(pKFi);
    //     VP->setId(pKFi->mnId);
    //     VP->setFixed(true);
    //     if (pKFi->mnId == (*loss_kfs.rbegin())->mnId) {
    //         VP->setFixed(false);
    //     }
    //     optimizer.addVertex(VP);
    //     // ROS_INFO_STREAM("opt KF: "<<pKFi->mnId);
    // }

    // Biases
    vector<VertexGyroBias*> vpgb;
    vector<VertexAccBias*> vpab;
    //todo_tightly
    //	set fixed for debuging
    for(auto pKFi:loss_kfs) {
        VertexGyroBias *VG = new VertexGyroBias(pKFi);
        VG->setId(maxKFid + 1 + pKFi->mnId);
        VG->setFixed(true);
        optimizer.addVertex(VG);
        vpgb.push_back(VG);

        VertexAccBias *VA = new VertexAccBias(pKFi);
        VA->setId((maxKFid + 1)*2 + pKFi->mnId);
        VA->setFixed(true);
        if (pKFi->GetMap() != (*loss_kfs.begin())->GetMap() && pKFi->mnId > (maxKFid-4)) {
            vpab.push_back(VA);
            VA->setFixed(true);
            ROS_DEBUG_STREAM("optimizable bias: " << pKFi->mnId);
        }
        optimizer.addVertex(VA);

        VertexVelocity *VV = new VertexVelocity(pKFi);
        VV->setId((maxKFid + 1)*3 + pKFi->mnId);
        VV->setFixed(true);
        optimizer.addVertex(VV);
    }

    // extrinsic parameter
    g2o::VertexSE3Expmap *vT_d_c = new g2o::VertexSE3Expmap();
    vT_d_c->setEstimate(Converter::toSE3Quat((*loss_kfs.begin())->mImuCalib.mT_dvl_c));
    vT_d_c->setId((maxKFid + 1)*4);
    vT_d_c->setFixed(true);
    optimizer.addVertex(vT_d_c);

    g2o::VertexSE3Expmap *vT_g_d = new g2o::VertexSE3Expmap();
    vT_g_d->setEstimate(Converter::toSE3Quat((*loss_kfs.begin())->mImuCalib.mT_gyro_dvl));
    vT_g_d->setId((maxKFid + 1)*4+1);
    vT_g_d->setFixed(true);
    optimizer.addVertex(vT_g_d);


    Eigen::Matrix3d R_b0_w = pAtlas->getRGravity();
    VertexGDir *VGDir = new VertexGDir(R_b0_w);
    VGDir->setId((maxKFid + 1)*4+2);
    VGDir->setFixed(true);
    optimizer.addVertex(VGDir);

    // add map points
    set<MapPoint *, MapPointComp> LocalMapPoints;
    for(auto pKFi:loss_kfs) {
        vector<MapPoint *> vpMPs = pKFi->GetMapPointMatches();
        for (vector<MapPoint *>::iterator it = vpMPs.begin(); it != vpMPs.end(); it++) {
            MapPoint *pMP = *it;
            if (pMP) {
                //				cout<<"find local map point"<<endl;
                if (!pMP->isBad()) {
                    if(LocalMapPoints.count(pMP)==0){
                        LocalMapPoints.insert(pMP);
                    }
                }
            }
        }
    }
    int N_map_points = LocalMapPoints.size();

    vector<EdgeMonoBA_DvlGyros *> mono_edges;
    vector<EdgeStereoBA_DvlGyros *> stereo_edges;


    // Graph edges
    vector<EdgeDvlGyroBA *> dvl_edges;
    // dvl_edges.reserve(loss_kfs.size());
    vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
    //	vppUsedKF.reserve(OptKFs.size() + FixedKFs.size());
    //	std::cout << "build optimization graph" << std::endl;
    EdgeDvlIMUInit *bias_edge = NULL;
    EdgeDvlIMU *fixed_bias_edge = NULL;
    std::set<EdgeDvlIMU*> dvlimu_edge;
    std::set<EdgeDvlIMUGravityRefine*> dvlimuG_edge;
    for(auto pKFi:loss_kfs) {
        if (pKFi->mPrevKF&&loss_kfs.count(pKFi->mPrevKF)) {
            if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) {
                continue;
            }
            ROS_DEBUG_STREAM("add dvl-imu edge: " << pKFi->mPrevKF->mnId << " -> " << pKFi->mnId);
            VertexPoseDvlIMU *VP1 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mPrevKF->mnId));
            //				g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
            VertexPoseDvlIMU *VP2 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
            //				g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
            g2o::HyperGraph::Vertex *VV1 = optimizer.vertex((maxKFid + 1)*3 + pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex *VV2 = optimizer.vertex((maxKFid + 1)*3 + pKFi->mnId);
            g2o::HyperGraph::Vertex *VG1 = optimizer.vertex(maxKFid + 1 + pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex *VA1 = optimizer.vertex((maxKFid + 1) * 2 + pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex *VG2 = optimizer.vertex(maxKFid + 1 + pKFi->mnId);
            g2o::HyperGraph::Vertex *VA2 = optimizer.vertex((maxKFid + 1) * 2 + pKFi->mnId);
            g2o::HyperGraph::Vertex *VT_d_c = optimizer.vertex((maxKFid + 1)*4);
            g2o::HyperGraph::Vertex *VT_g_d = optimizer.vertex((maxKFid + 1)*4+1);
            g2o::HyperGraph::Vertex *VR_b0_w = optimizer.vertex((maxKFid + 1) * 4 + 2);

            if (!VP1 || !VG2 || !VP2) {
                cout << "Error" << VP1 << ", " << VG2 << ", " << VP2 << endl;

                continue;
            }

            EdgeAccRW* e_bias = new EdgeAccRW();
            e_bias->setLevel(1);
            e_bias->setVertex(0,VA1);
            e_bias->setVertex(1,VA2);
        	Eigen::Matrix3d info_acc_bias = Eigen::Matrix3d::Identity();
        	cv::Mat cvInfoA = pKFi->mpDvlPreintegrationKeyFrame->C.rowRange(12,15).colRange(12,15).inv(cv::DECOMP_SVD);
        	cv::cv2eigen(cvInfoA,info_acc_bias);
            e_bias->setInformation(info_acc_bias);
            optimizer.addEdge(e_bias);


        	EdgeGyroRW* eg_bias = new EdgeGyroRW();
        	eg_bias->setLevel(1);
        	eg_bias->setVertex(0,VG1);
        	eg_bias->setVertex(1,VG2);
        	Eigen::Matrix3d info_gyro_bias = Eigen::Matrix3d::Identity()*1e4;
        	cv::Mat cvInfoG = pKFi->mpDvlPreintegrationKeyFrame->C.rowRange(9,12).colRange(9,12).inv(cv::DECOMP_SVD);
        	cv::cv2eigen(cvInfoG,info_gyro_bias);
        	eg_bias->setInformation(info_gyro_bias);
        	// if(info_gyro_bias(0,0)==0)
        	optimizer.addEdge(eg_bias);

            EdgeDvlIMU *e_di = new EdgeDvlIMU(pKFi->mpDvlPreintegrationKeyFrame);
            e_di->setLevel(1);
            e_di->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
            e_di->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
            e_di->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
            e_di->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
            e_di->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG2));
            e_di->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA2));
            e_di->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
            e_di->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
            e_di->setVertex(8, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VR_b0_w));
            Eigen::Matrix<double,9,9> info = Eigen::Matrix<double,9,9>::Identity();
        	Eigen::Matrix<double,9,9> info_DI=Eigen::Matrix<double,9,9>::Identity();
        	cv::Mat cvInfo = pKFi->mpDvlPreintegrationKeyFrame->C.rowRange(0,9).colRange(0,9).inv(cv::DECOMP_SVD);
        	cv::cv2eigen(cvInfo,info_DI);

            // info.block(0,0,3,3) = Eigen::Matrix3d::Identity() * 100;
            // info(0,0) = info(0,0) * 5e3; // 10_24
            // info(1,1) = info(1,1) * 5e3; // before 10_24
            // info(0,0) = info(0,0) * 1e2; // 10_24
            // info.block(3,3,3,3) = Eigen::Matrix3d::Identity()*100;
            // info.block(6,6,3,3) = Eigen::Matrix3d::Identity()*100;
            e_di->setInformation(info_DI);
            e_di->setId((maxKFid + 1) * 2 + pKFi->mnId);
            optimizer.addEdge(e_di);
            // fixed_bias_edge = e_di;
            // dvlimu_edge.insert(e_di);

        	EdgeDvlGyroBA *ei = new EdgeDvlGyroBA(pKFi->mpDvlPreintegrationKeyFrame);
        	//				ei->setVertex(0, VP1);

        	//			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
        	//			ei->setRobustKernel(rk);
        	//			rk->setDelta(sqrt(7.815));
        	ei->setLevel(0);
        	ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
        	ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
        	ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG2));
        	ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
        	ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
        	Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity();
        	// set first 3*3 of cov to be 100 * identity matrix
        	cov.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * 100;
        	// set last 3*3 of cov to be 1000 * identity matrix
        	cov.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity() * 100;
        	ei->setInformation(cov);
        	ei->setId(pKFi->mnId);
        	dvl_edges.push_back(ei);
        	optimizer.addEdge(ei);

            EdgeDvlIMUGravityRefine* eg =new EdgeDvlIMUGravityRefine(pKFi->mpDvlPreintegrationKeyFrame);
            eg->setLevel(0);
            eg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
            eg->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
            eg->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
            eg->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
            eg->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG2));
            eg->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA2));
            eg->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
            eg->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
            eg->setVertex(8, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VR_b0_w));
            eg->setInformation(Eigen::Matrix<double, 3, 3>::Identity() *100);
            eg->setId((maxKFid+1)*2 + pKFi->mnId);
            optimizer.addEdge(eg);
            // dvlimuG_edge.insert(eg);
        }
    }

    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};

    // Compute error for different scales
    //	std::cout << "start optimization" << std::endl;
    // optimizer.setVerbose(true);

    // ROS_INFO_STREAM("bias before: ");
    // for(auto p:vpab){
    //     int kf_id = p->id() - (maxKFid + 1)*2;
    //     p->setFixed(false);
    //     ROS_INFO_STREAM("KF["<<kf_id<<"] bias: "<<p->estimate().transpose());
    // }
    // for(auto eg:dvlimuG_edge){
    //     eg->setLevel(0);
    // }
    // for(auto edi:dvlimu_edge){
    //     edi->setLevel(1);
    // }
    // optimizer.setVerbose(true);
    // optimizer.initializeOptimization(0);
    // optimizer.optimize(5);
    // ROS_INFO_STREAM("bias after: ");
    // for(auto p:vpab){
    //     int kf_id = p->id() - (maxKFid + 1)*2;
    //     p->setFixed(false);
    //     ROS_INFO_STREAM("KF["<<kf_id<<"] bias: "<<p->estimate().transpose());
    // }
    //
    // for(auto p:vpab){
    //     p->setFixed(true);
    // }
    // for(auto eg:dvlimuG_edge){
    //     eg->setLevel(1);
    // }
    // for(auto edi:dvlimu_edge){
    //     edi->setLevel(0);
    // }
    optimizer.initializeOptimization(0);
    optimizer.optimize(10);
    // for(auto p:vpab){
    //     p->setFixed(false);
    // }
    // optimizer.initializeOptimization(0);
    // optimizer.optimize(2);

    // fixed_bias_edge->setLevel(0);
    // optimizer.initializeOptimization(0);
    // optimizer.optimize(2);


    int visual_edge_num = mono_edges.size() + stereo_edges.size();
    int dvl_edge_num = dvl_edges.size();

    //	std::cout << "end optimization" << std::endl;


    // Recover optimized data
    for(auto pKFi:loss_kfs) {
        int kf_id = pKFi->mnId;
        if (pKFi->mnId > maxKFid) {
            continue;
        }
        //Bias
        // int gyros_bias_vertex_id = kf_id + maxKFid + 1;
        // int acc_bias_vertex_id = kf_id + (maxKFid + 1)*2;
        // VertexGyroBias *v_gb = dynamic_cast<VertexGyroBias *>(optimizer.vertex(gyros_bias_vertex_id));
        // VertexAccBias *v_ab = dynamic_cast<VertexAccBias *>(optimizer.vertex(acc_bias_vertex_id));
        // // bg << v_gb->estimate();
        // IMU::Bias b(v_ab->estimate().x(), v_ab->estimate().y(), v_ab->estimate().z(),
        //             v_gb->estimate().x(), v_gb->estimate().y(), v_gb->estimate().z());
        // pKFi->SetNewBias(b);
        // ROS_INFO_STREAM("recover KF[" << pKFi->mnId << "] bias(gyros acc): " << b.bwx<<","<<b.bwy<<","<<b.bwz<<","<<b.bax<<","<<b.bay<<","<<b.baz);

        // Pose
        VertexPoseDvlIMU *VP = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
        if(VP->fixed()){
            continue;
        }
        Eigen::Quaterniond Rwc(VP->estimate().Rwc);
        Eigen::Vector3d twc = VP->estimate().twc;
        ROS_DEBUG_STREAM("recover KF[" << pKFi->mnId << "] pose: from"<<pKFi->GetPoseInverse().col(3).rowRange(0,3).t()<<" to: " << twc.transpose());
        Eigen::Isometry3d Twc = Eigen::Isometry3d::Identity();
        Twc.pretranslate(twc);
        Twc.rotate(Rwc);
        Eigen::Isometry3d Tcw = Twc.inverse();
        cv::Mat Tcw_cv;
        cv::eigen2cv(Tcw.matrix(), Tcw_cv);
        Tcw_cv.convertTo(Tcw_cv, CV_32F);
        pKFi->SetPose(Tcw_cv);
    }
    // ROS_INFO_STREAM(ss.str());


}

void Optimizer::OptimizationDVLIMU(set<KeyFrame*, KFComparator> &loss_kfs, Atlas* pAtlas, double lamda_DVL)
{
    unique_lock<shared_timed_mutex> lock(pAtlas->GetCurrentMap()->mMutexMapUpdate);
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    optimizer.setAlgorithm(solver);


    // Set KeyFrame vertices (fixed poses and optimizable velocities)
    long maxKFid = (*loss_kfs.rbegin())->mnId;

    stringstream ss;
    ss << "DVL_IMU optimization: " << endl;
    // iterate ls_kf from last to first, and add vertex to optimizer
    for (auto pKFi :loss_kfs) {
        if (pKFi->isBad()){
            ROS_ERROR_STREAM("bad KF");
            assert(-1);
        }
        if(pKFi->GetPose().empty()){
            ROS_ERROR_STREAM("empty pose");
            continue;
        }
        VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
        VP->setId(pKFi->mnId);
        if (pKFi->mnId == (*loss_kfs.begin())->mnId) {
            VP->setFixed(true);
            ss << "fixed: " << pKFi->mnId << endl;
        }
        else if(pKFi->GetMap()!=(*loss_kfs.begin())->GetMap()){
            VP->setFixed(false);
            ss << "optimizable: " << pKFi->mnId << endl;
        }
        else{
            VP->setFixed(true);
            ss << "fixed: " << pKFi->mnId << endl;
        }
        optimizer.addVertex(VP);
    }
    ROS_INFO_STREAM(ss.str());




    // Biases
    vector<VertexGyroBias*> vpgb;
    vector<VertexAccBias*> vpab;
    //todo_tightly
    //	set fixed for debuging
    for(auto pKFi:loss_kfs) {
        VertexGyroBias *VG = new VertexGyroBias(pKFi);
        VG->setId(maxKFid + 1 + pKFi->mnId);
        VG->setFixed(true);
        optimizer.addVertex(VG);
        vpgb.push_back(VG);

        VertexAccBias *VA = new VertexAccBias(pKFi);
        VA->setId((maxKFid + 1)*2 + pKFi->mnId);
        VA->setFixed(true);
        optimizer.addVertex(VA);
        if(pKFi->GetMap()!=(*loss_kfs.begin())->GetMap()){
            vpab.push_back(VA);
        }


        VertexVelocity *VV = new VertexVelocity(pKFi);
        VV->setId((maxKFid + 1)*3 + pKFi->mnId);
        VV->setFixed(true);
        optimizer.addVertex(VV);
    }

    // extrinsic parameter
    g2o::VertexSE3Expmap *vT_d_c = new g2o::VertexSE3Expmap();
    vT_d_c->setEstimate(Converter::toSE3Quat((*loss_kfs.begin())->mImuCalib.mT_dvl_c));
    vT_d_c->setId((maxKFid + 1)*4);
    vT_d_c->setFixed(true);
    optimizer.addVertex(vT_d_c);

    g2o::VertexSE3Expmap *vT_g_d = new g2o::VertexSE3Expmap();
    vT_g_d->setEstimate(Converter::toSE3Quat((*loss_kfs.begin())->mImuCalib.mT_gyro_dvl));
    vT_g_d->setId((maxKFid + 1)*4+1);
    vT_g_d->setFixed(true);
    optimizer.addVertex(vT_g_d);


    Eigen::Matrix3d R_b0_w = pAtlas->getRGravity();
    VertexGDir *VGDir = new VertexGDir(R_b0_w);
    VGDir->setId((maxKFid + 1)*4+2);
    VGDir->setFixed(true);
    optimizer.addVertex(VGDir);

    // add map points
    set<MapPoint *, MapPointComp> LocalMapPoints;
    for(auto pKFi:loss_kfs) {
        vector<MapPoint *> vpMPs = pKFi->GetMapPointMatches();
        for (vector<MapPoint *>::iterator it = vpMPs.begin(); it != vpMPs.end(); it++) {
            MapPoint *pMP = *it;
            if (pMP) {
                //				cout<<"find local map point"<<endl;
                if (!pMP->isBad()) {
                    if(LocalMapPoints.count(pMP)==0){
                        // LocalMapPoints.insert(pMP);
                    }
                }
            }
        }
    }
    int N_map_points = LocalMapPoints.size();


    vector<EdgeMonoBA_DvlGyros *> mono_edges;
    vector<EdgeStereoBA_DvlGyros *> stereo_edges;

    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for (auto pMP: LocalMapPoints) {
            if (pMP) {
                g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
                vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
                int id = pMP->mnId + (maxKFid + 1)*5;
                vPoint->setId(id);
                if (pMP->GetMap()==(*loss_kfs.begin())->GetMap()){
                    vPoint->setFixed(true);
                }
                else{
                    vPoint->setFixed(false);
                }
                vPoint->setMarginalized(true);
                optimizer.addVertex(vPoint);

                const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

                for (auto ob: observations) {
                    KeyFrame *pKFi = ob.first;
                    if(!loss_kfs.count(pKFi)){
                        continue;
                    }
                    if (!pKFi->isBad()) {
                        const int leftIndex = get<0>(ob.second);

                        // Monocular observation
                        if (leftIndex != -1 && pKFi->mvuRight[get<0>(ob.second)] < 0) {
                            const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
                            Eigen::Matrix<double, 2, 1> obs;
                            obs << kpUn.pt.x, kpUn.pt.y;

                            EdgeMonoBA_DvlGyros *e = new EdgeMonoBA_DvlGyros();
                            e->setLevel(0);
                            e->setVertex(0,
                                         dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                            e->setVertex(1, vPoint);
                            e->setMeasurement(obs);
                            const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2 * 0.0001);

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(sqrt(5.991));

                            mono_edges.push_back(e);
                            optimizer.addEdge(e);
                        }
                        else if (leftIndex != -1 && pKFi->mvuRight[get<0>(ob.second)] >= 0) // Stereo observation
                        {
                            const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
                            Eigen::Matrix<double, 3, 1> obs;
                            const float kp_ur = pKFi->mvuRight[get<0>(ob.second)];
                            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                            EdgeStereoBA_DvlGyros *e = new EdgeStereoBA_DvlGyros();
                            e->setLevel(0);
                            e->setVertex(0,
                                         dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                            e->setVertex(1, vPoint);
                            e->setMeasurement(obs);
                            const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2 * 0.0001;
                            e->setInformation(Info);

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(sqrt(7.815));

                            stereo_edges.push_back(e);
                            optimizer.addEdge(e);
                        }

                    }

                }
            }
        }
    }

    // Graph edges
    vector<EdgeDvlGyroBA *> dvl_edges;
    // dvl_edges.reserve(loss_kfs.size());
    vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
    //	vppUsedKF.reserve(OptKFs.size() + FixedKFs.size());
    //	std::cout << "build optimization graph" << std::endl;
    EdgeDvlIMUInit *bias_edge = NULL;
    EdgeDvlIMU *fixed_bias_edge = NULL;
    for(auto pKFi:loss_kfs) {
        if (loss_kfs.count(pKFi->mPrevKF)) {
            if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) {
                continue;
            }
            ROS_DEBUG_STREAM("add dvl-imu edge: " << pKFi->mPrevKF->mnId << " -> " << pKFi->mnId);
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
            g2o::HyperGraph::Vertex *VR_b0_w = optimizer.vertex((maxKFid + 1) * 4 + 2);

            if (!VP1 || !VG || !VP2) {
                cout << "Error" << VP1 << ", " << VG << ", " << VP2 << endl;

                continue;
            }

            std::vector<cv::Point3d> vbias_a, vbias_g;
            vbias_a.push_back(cv::Point3d(pKFi->mPrevKF->GetImuBias().bax, pKFi->mPrevKF->GetImuBias().bay,
                                          pKFi->mPrevKF->GetImuBias().baz));
            vbias_g.push_back(cv::Point3d(pKFi->mPrevKF->GetImuBias().bwx, pKFi->mPrevKF->GetImuBias().bwy,
                                          pKFi->mPrevKF->GetImuBias().bwz));
            cv::Mat acc_prior = cv::Mat(vbias_a);
            cv::Mat gyr_prior = cv::Mat(vbias_g);
            acc_prior.convertTo(acc_prior, CV_32F);
            gyr_prior.convertTo(gyr_prior, CV_32F);
            acc_prior = acc_prior.reshape(1);
            gyr_prior = gyr_prior.reshape(1);
            EdgePriorAcc *epa = new EdgePriorAcc(acc_prior);
            epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
            epa->setLevel(0);
            epa->setInformation(Eigen::Matrix3d::Identity() * 100);
            optimizer.addEdge(epa);

            EdgePriorGyro *epg = new EdgePriorGyro(gyr_prior);
            epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
            epg->setLevel(0);
            epg->setInformation(Eigen::Matrix3d::Identity() * 100);
            optimizer.addEdge(epg);

            //velocity edge
            Eigen::Vector3d dvl_v1;
            pKFi->mPrevKF->GetDvlVelocityMeasurement(dvl_v1);
            Eigen::Vector3d dvl_v2;
            pKFi->GetDvlVelocityMeasurement(dvl_v2);
            EdgeDvlVelocity *ev = new EdgeDvlVelocity(dvl_v1);
            ev->setLevel(0);
            ev->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
            if(pKFi->mbDVL){
                ev->setInformation(Eigen::Matrix3d::Identity()*100);
            }
            else{
                ev->setInformation(Eigen::Matrix3d::Identity()*0.1);
            }
            optimizer.addEdge(ev);
            // add edge for v2
            EdgeDvlVelocity *ev2 = new EdgeDvlVelocity(dvl_v2);
            ev2->setLevel(0);
            ev2->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
            if (pKFi->mbDVL) {
                ev2->setInformation(Eigen::Matrix3d::Identity() * 100);
            }
            else {
                ev2->setInformation(Eigen::Matrix3d::Identity()*0.1);
            }
            optimizer.addEdge(ev2);

            //				EdgeInertialGS *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
            EdgeDvlGyroBA *ei = new EdgeDvlGyroBA(pKFi->mpDvlPreintegrationKeyFrame);
            //				ei->setVertex(0, VP1);

            //			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            //			ei->setRobustKernel(rk);
            //			rk->setDelta(sqrt(7.815));
            ei->setLevel(1);
            ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
            ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
            ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
            ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
            ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
            Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity();
            // set first 3*3 of cov to be 100 * identity matrix
            cov.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * 100;
            // set last 3*3 of cov to be 1000 * identity matrix
            cov.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity() * 100;
            ei->setInformation(cov);
            ei->setId(pKFi->mnId);
            dvl_edges.push_back(ei);
            optimizer.addEdge(ei);

            EdgeDvlIMU *eG = new EdgeDvlIMU(pKFi->mpDvlPreintegrationKeyFrame);
            eG->setLevel(0);
            eG->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
            eG->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
            eG->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
            eG->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
            eG->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
            eG->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
            eG->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
            eG->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
            eG->setVertex(8, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VR_b0_w));
            eG->setInformation(Eigen::Matrix<double, 9, 9>::Identity() *1000);
            eG->setId((maxKFid+1)*2 + pKFi->mnId);
            optimizer.addEdge(eG);
            fixed_bias_edge = eG;
        }
    }

    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};

    // Compute error for different scales
    //	std::cout << "start optimization" << std::endl;
    // optimizer.setVerbose(true);

    optimizer.initializeOptimization(0);
    optimizer.optimize(20);

    // for(auto p:vpab){
    //     p->setFixed(false);
    // }
    // optimizer.initializeOptimization(0);
    // optimizer.optimize(2);
    //
    // fixed_bias_edge->setLevel(0);
    // optimizer.initializeOptimization(0);
    // optimizer.optimize(2);


    int visual_edge_num = mono_edges.size() + stereo_edges.size();
    int dvl_edge_num = dvl_edges.size();

    //	std::cout << "end optimization" << std::endl;


    // Recover optimized data
    for (auto pMap:pAtlas->GetAllMaps()) {
        for(auto pKFi:loss_kfs) {
            int kf_id = pKFi->mnId;
            if (pKFi->mnId > maxKFid) {
                continue;
            }
            if (pKFi->GetMap()!=pMap){
                continue;
            }
            //Bias
            int gyros_bias_vertex_id = kf_id + maxKFid + 1;
            int acc_bias_vertex_id = kf_id + (maxKFid + 1)*2;
            VertexGyroBias *v_gb = dynamic_cast<VertexGyroBias *>(optimizer.vertex(gyros_bias_vertex_id));
            VertexAccBias *v_ab = dynamic_cast<VertexAccBias *>(optimizer.vertex(acc_bias_vertex_id));
            // bg << v_gb->estimate();
            IMU::Bias b(v_ab->estimate().x(), v_ab->estimate().y(), v_ab->estimate().z(),
                        v_gb->estimate().x(), v_gb->estimate().y(), v_gb->estimate().z());
            pKFi->SetNewBias(b);
            ROS_DEBUG_STREAM("recover KF[" << pKFi->mnId << "] bias: " << b);

            // Pose
            VertexPoseDvlIMU *VP = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
            if(VP->fixed()){
                continue;
            }
            Eigen::Quaterniond Rwc(VP->estimate().Rwc);
            Eigen::Vector3d twc = VP->estimate().twc;
            ROS_DEBUG_STREAM("recover KF[" << pKFi->mnId << "] pose: from"<<pKFi->GetPoseInverse().col(3).rowRange(0,3).t()<<" to: " << twc.transpose());
            Eigen::Isometry3d Twc = Eigen::Isometry3d::Identity();
            Twc.pretranslate(twc);
            Twc.rotate(Rwc);
            Eigen::Isometry3d Tcw = Twc.inverse();
            cv::Mat Tcw_cv;
            cv::eigen2cv(Tcw.matrix(), Tcw_cv);
            Tcw_cv.convertTo(Tcw_cv, CV_32F);
            pKFi->SetPose(Tcw_cv);
        }
        for (auto pMP:LocalMapPoints) {
            if(pMP->isBad()){
                continue;
            }
            if (pMP->GetMap()!=pMap){
                continue;
            }
            int id = pMP->mnId + (maxKFid + 1)*5;
            g2o::VertexSBAPointXYZ
                    *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex((maxKFid + 1) * 5 + pMP->mnId));
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
    }
}


int Optimizer::PoseOptimizationWithBA_and_EKF(Frame *pFrame, Frame *pLastFrame, double lamda_visual, double lamda_DVL)
{
	// setup solver
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

	g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);

	// number of edges
	int nInitialCorrespondences = 0;

	// Set Frame vertex
	g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
	vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
	vSE3->setId(0);
	vSE3->setFixed(false);
	optimizer.addVertex(vSE3);

	// Set MapPoint vertices

	// Number of KeyPoints.
	const int N = pFrame->N;

	// setup edges pointer
	vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *> vpEdgesMono;
	vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody *> vpEdgesMono_FHR;
	vector<size_t> vnIndexEdgeMono, vnIndexEdgeRight;
	vpEdgesMono.reserve(N);
	vpEdgesMono_FHR.reserve(N);
	vnIndexEdgeMono.reserve(N);
	vnIndexEdgeRight.reserve(N);

	vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> vpEdgesStereo;
	vector<size_t> vnIndexEdgeStereo;
	vpEdgesStereo.reserve(N);
	vnIndexEdgeStereo.reserve(N);

	// robust kernel
	const float deltaMono = sqrt(5.991);
	const float deltaStereo = sqrt(7.815);

	// add edges
	{
		unique_lock<mutex> lock(MapPoint::mGlobalMutex);

		// iterate all key points
		// N: Number of KeyPoints in the frame
		for (int i = 0; i < N; i++) {
			MapPoint *pMP = pFrame->mvpMapPoints[i];
			// if the key points has its correspending map points created
			if (pMP) {
				//Conventional SLAM
				double visual_lamda = lamda_visual;
				double ekf_lamda = lamda_DVL;
				if (!pFrame->mpCamera2) {
					// Monocular observation
					// add edge
					if (pFrame->mvuRight[i] < 0) {
						nInitialCorrespondences++;
						pFrame->mvbOutlier[i] = false;

						Eigen::Matrix<double, 2, 1> obs;
						//undistorted feature points
						const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
						obs << kpUn.pt.x, kpUn.pt.y;

						ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
						e->setMeasurement(obs);
						const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
						e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

						g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
						// TODO remove setRobustKernel
						e->setRobustKernel(rk);
						rk->setDelta(deltaMono);

						e->pCamera = pFrame->mpCamera;
						// xw: position of the map point under world coordinate
						cv::Mat Xw = pMP->GetWorldPos();
						e->Xw[0] = Xw.at<float>(0);
						e->Xw[1] = Xw.at<float>(1);
						e->Xw[2] = Xw.at<float>(2);

						optimizer.addEdge(e);

						vpEdgesMono.push_back(e);
						vnIndexEdgeMono.push_back(i);
					}
						// Stereo observation
						// add edge, for those key ponits in both left and right
					else {
						nInitialCorrespondences++;
						pFrame->mvbOutlier[i] = false;

						//SET EDGE
						Eigen::Matrix<double, 3, 1> obs;

						//undistorted feature points
						const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
						// kp_ur: the position of x in thr right image
						const float &kp_ur = pFrame->mvuRight[i];
						obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

						g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
						e->setMeasurement(obs);
						const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
						Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
						e->setInformation(Info);

						g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(deltaStereo);

						e->fx = pFrame->fx;
						e->fy = pFrame->fy;
						e->cx = pFrame->cx;
						e->cy = pFrame->cy;
						e->bf = pFrame->mbf;
						cv::Mat Xw = pMP->GetWorldPos();
						e->Xw[0] = Xw.at<float>(0);
						e->Xw[1] = Xw.at<float>(1);
						e->Xw[2] = Xw.at<float>(2);

						optimizer.addEdge(e);

						vpEdgesStereo.push_back(e);
						vnIndexEdgeStereo.push_back(i);
					}
				}
					// SLAM with respect a rigid body
					// using stereo, only in one image
					// add edge, for those key ponits only in left or only in right
				else {
					nInitialCorrespondences++;

					cv::KeyPoint kpUn;

					if (i < pFrame->Nleft) { //Left camera observation
						kpUn = pFrame->mvKeys[i];

						pFrame->mvbOutlier[i] = false;

						Eigen::Matrix<double, 2, 1> obs;
						obs << kpUn.pt.x, kpUn.pt.y;

						ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
						e->setMeasurement(obs);
						const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
						e->setInformation(Eigen::Matrix2d::Identity() * invSigma2 * visual_lamda);

						g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(deltaMono);

						e->pCamera = pFrame->mpCamera;
						cv::Mat Xw = pMP->GetWorldPos();
						e->Xw[0] = Xw.at<float>(0);
						e->Xw[1] = Xw.at<float>(1);
						e->Xw[2] = Xw.at<float>(2);

						optimizer.addEdge(e);

						vpEdgesMono.push_back(e);
						vnIndexEdgeMono.push_back(i);
					}
					else { //Right camera observation
						//continue;
						kpUn = pFrame->mvKeysRight[i - pFrame->Nleft];

						Eigen::Matrix<double, 2, 1> obs;
						obs << kpUn.pt.x, kpUn.pt.y;

						pFrame->mvbOutlier[i] = false;

						ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody
							*e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
						e->setMeasurement(obs);
						const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
						e->setInformation(Eigen::Matrix2d::Identity() * invSigma2 * visual_lamda);

						g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(deltaMono);

						e->pCamera = pFrame->mpCamera2;
						cv::Mat Xw = pMP->GetWorldPos();
						e->Xw[0] = Xw.at<float>(0);
						e->Xw[1] = Xw.at<float>(1);
						e->Xw[2] = Xw.at<float>(2);

						e->mTrl = Converter::toSE3Quat(pFrame->mTrl);

						optimizer.addEdge(e);

						vpEdgesMono_FHR.push_back(e);
						vnIndexEdgeRight.push_back(i);
					}
				}

				// add EKF constrains
				// the position map point under Last camera frame
				Eigen::Vector3d p_ci_test;
				cv::Mat P_c0 = pMP->GetWorldPos();
				Eigen::Vector3d p_c0;
				p_c0 << P_c0.at<float>(0), P_c0.at<float>(1), P_c0.at<float>(2);
				cv::Mat T_ciw = pLastFrame->mTcw;
				Eigen::Isometry3d T_ci_c0 = Eigen::Isometry3d::Identity();
				cv::cv2eigen(T_ciw, T_ci_c0.matrix());
				EdgeSE3DVLPoseOnly *edge = new EdgeSE3DVLPoseOnly(pFrame->mT_e0_ej.inverse(),
				                                                  pLastFrame->mT_e0_ej.inverse(),
				                                                  T_ci_c0,
				                                                  pFrame->mT_e_c,
				                                                  p_c0);
				edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
//					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
//					edge->setRobustKernel(rk);
//					rk->setDelta(deltaMono);
				// edge->setId(index);
				// set information according to covariance of EKF
				edge->setInformation(ekf_lamda * Eigen::Matrix3d::Identity());
				optimizer.addEdge(edge);
			}
		}
	}

	//cout << "PO: vnIndexEdgeMono.size() = " << vnIndexEdgeMono.size() << "   vnIndexEdgeRight.size() = " << vnIndexEdgeRight.size() << endl;
	if (nInitialCorrespondences < 3) {
		return 0;
	}

	// We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
	// At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
	const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
	const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
	const int its[4] = {10, 10, 10, 10};

	int nBad = 0;
	for (size_t it = 0; it < 4; it++) {
		// set estimate of camera vertex as the pose of current frame
		vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
		optimizer.initializeOptimization(0);
		optimizer.optimize(its[it]);

		nBad = 0;

		// 3 for loop for check outlier

		// using mono, or using stereo but feature only in left image
		for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
			ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *e = vpEdgesMono[i];

			const size_t idx = vnIndexEdgeMono[i];

			if (pFrame->mvbOutlier[idx]) {
				e->computeError();
			}

			// ??? what is the meaning of chi2
			const float chi2 = e->chi2();

			if (chi2 > chi2Mono[it]) {
				pFrame->mvbOutlier[idx] = true;
				// ???? what does level work?
				e->setLevel(1);
				nBad++;
			}
			else {
				pFrame->mvbOutlier[idx] = false;
				e->setLevel(0);
			}

			if (it == 2) {
				e->setRobustKernel(0);
			}
		}
		// using stereo, but feature only in right
		for (size_t i = 0, iend = vpEdgesMono_FHR.size(); i < iend; i++) {
			ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody *e = vpEdgesMono_FHR[i];

			const size_t idx = vnIndexEdgeRight[i];

			if (pFrame->mvbOutlier[idx]) {
				e->computeError();
			}

			const float chi2 = e->chi2();

			if (chi2 > chi2Mono[it]) {
				pFrame->mvbOutlier[idx] = true;
				e->setLevel(1);
				nBad++;
			}
			else {
				pFrame->mvbOutlier[idx] = false;
				e->setLevel(0);
			}

			if (it == 2) {
				e->setRobustKernel(0);
			}
		}
		// using stereo, feature in both left and right image
		for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
			g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = vpEdgesStereo[i];

			const size_t idx = vnIndexEdgeStereo[i];

			if (pFrame->mvbOutlier[idx]) {
				e->computeError();
			}

			const float chi2 = e->chi2();

			if (chi2 > chi2Stereo[it]) {
				pFrame->mvbOutlier[idx] = true;
				e->setLevel(1);
				nBad++;
			}
			else {
				e->setLevel(0);
				pFrame->mvbOutlier[idx] = false;
			}

			if (it == 2) {
				e->setRobustKernel(0);
			}
		}

		if (optimizer.edges().size() < 10) {
			break;
		}
	}

	// Recover optimized pose and return number of inliers
	g2o::VertexSE3Expmap *vSE3_recov = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
	g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
	cv::Mat pose = Converter::toCvMat(SE3quat_recov);
	pFrame->SetPose(pose);

	//cout << "[PoseOptimization]: initial correspondences-> " << nInitialCorrespondences << " --- outliers-> " << nBad << endl;

	return nInitialCorrespondences - nBad;
}

int Optimizer::PoseOptimizationWithBA_and_EKF2(Frame *pFrame, Frame *pLastFrame, double lamda_visual, double lamda_DVL)
{
	// setup solver
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

	g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);

	// number of edges
	int nInitialCorrespondences = 0;

	// Set Frame vertex
	g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
	vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
	vSE3->setId(0);
	vSE3->setFixed(false);
	optimizer.addVertex(vSE3);

	// Set MapPoint vertices

	// Number of KeyPoints.
	const int N = pFrame->N;

	// setup edges pointer
	vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *> vpEdgesMono;
	vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody *> vpEdgesMono_FHR;
	vector<size_t> vnIndexEdgeMono, vnIndexEdgeRight;
	vpEdgesMono.reserve(N);
	vpEdgesMono_FHR.reserve(N);
	vnIndexEdgeMono.reserve(N);
	vnIndexEdgeRight.reserve(N);

	vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> vpEdgesStereo;
	vector<size_t> vnIndexEdgeStereo;
	vpEdgesStereo.reserve(N);
	vnIndexEdgeStereo.reserve(N);

	// robust kernel
	const float deltaMono = sqrt(5.991);
	const float deltaStereo = sqrt(7.815);

	// add edges
	{
		unique_lock<mutex> lock(MapPoint::mGlobalMutex);

		// iterate all key points
		// N: Number of KeyPoints in the frame
		for (int i = 0; i < N; i++) {
			MapPoint *pMP = pFrame->mvpMapPoints[i];
			// if the key points has its correspending map points created
			if (pMP) {
				//Conventional SLAM
				double visual_lamda = lamda_visual;
				double ekf_lamda = lamda_DVL;
				if (!pFrame->mpCamera2) {
					// Monocular observation
					// add edge
					if (pFrame->mvuRight[i] < 0) {
						nInitialCorrespondences++;
						pFrame->mvbOutlier[i] = false;

						Eigen::Matrix<double, 2, 1> obs;
						//undistorted feature points
						const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
						obs << kpUn.pt.x, kpUn.pt.y;

						ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
						e->setMeasurement(obs);
						const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
						e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

//                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
						// TODO remove setRobustKernel
//                            e->setRobustKernel(rk);
//                            rk->setDelta(deltaMono);

						e->pCamera = pFrame->mpCamera;
						// xw: position of the map point under world coordinate
						cv::Mat Xw = pMP->GetWorldPos();
						e->Xw[0] = Xw.at<float>(0);
						e->Xw[1] = Xw.at<float>(1);
						e->Xw[2] = Xw.at<float>(2);

						optimizer.addEdge(e);

						vpEdgesMono.push_back(e);
						vnIndexEdgeMono.push_back(i);
					}
						// Stereo observation
						// add edge, for those key ponits in both left and right
					else {
						nInitialCorrespondences++;
						pFrame->mvbOutlier[i] = false;

						//SET EDGE
						Eigen::Matrix<double, 3, 1> obs;

						//undistorted feature points
						const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
						// kp_ur: the position of x in thr right image
						const float &kp_ur = pFrame->mvuRight[i];
						obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

						g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
						e->setMeasurement(obs);
						const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
						Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
						e->setInformation(Info);

//                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
//                            e->setRobustKernel(rk);
//                            rk->setDelta(deltaStereo);

						e->fx = pFrame->fx;
						e->fy = pFrame->fy;
						e->cx = pFrame->cx;
						e->cy = pFrame->cy;
						e->bf = pFrame->mbf;
						cv::Mat Xw = pMP->GetWorldPos();
						e->Xw[0] = Xw.at<float>(0);
						e->Xw[1] = Xw.at<float>(1);
						e->Xw[2] = Xw.at<float>(2);

						optimizer.addEdge(e);

						vpEdgesStereo.push_back(e);
						vnIndexEdgeStereo.push_back(i);
					}
				}
					// SLAM with respect a rigid body
					// using stereo, only in one image
					// add edge, for those key ponits only in left or only in right
				else {
					nInitialCorrespondences++;

					cv::KeyPoint kpUn;

					if (i < pFrame->Nleft) { //Left camera observation
						kpUn = pFrame->mvKeys[i];

						pFrame->mvbOutlier[i] = false;

						Eigen::Matrix<double, 2, 1> obs;
						obs << kpUn.pt.x, kpUn.pt.y;

						ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
						e->setMeasurement(obs);
						const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
						e->setInformation(Eigen::Matrix2d::Identity() * invSigma2 * visual_lamda);

//                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
//                            e->setRobustKernel(rk);
//                            rk->setDelta(deltaMono);

						e->pCamera = pFrame->mpCamera;
						cv::Mat Xw = pMP->GetWorldPos();
						e->Xw[0] = Xw.at<float>(0);
						e->Xw[1] = Xw.at<float>(1);
						e->Xw[2] = Xw.at<float>(2);

						optimizer.addEdge(e);

						vpEdgesMono.push_back(e);
						vnIndexEdgeMono.push_back(i);
					}
					else { //Right camera observation
						//continue;
						kpUn = pFrame->mvKeysRight[i - pFrame->Nleft];

						Eigen::Matrix<double, 2, 1> obs;
						obs << kpUn.pt.x, kpUn.pt.y;

						pFrame->mvbOutlier[i] = false;

						ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody
							*e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
						e->setMeasurement(obs);
						const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
						e->setInformation(Eigen::Matrix2d::Identity() * invSigma2 * visual_lamda);

//                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
//                            e->setRobustKernel(rk);
//                            rk->setDelta(deltaMono);

						e->pCamera = pFrame->mpCamera2;
						cv::Mat Xw = pMP->GetWorldPos();
						e->Xw[0] = Xw.at<float>(0);
						e->Xw[1] = Xw.at<float>(1);
						e->Xw[2] = Xw.at<float>(2);

						e->mTrl = Converter::toSE3Quat(pFrame->mTrl);

						optimizer.addEdge(e);

						vpEdgesMono_FHR.push_back(e);
						vnIndexEdgeRight.push_back(i);
					}
				}

				// add EKF constrains
				// the position map point under Last camera frame
			}
		}

		Eigen::Vector3d p_ci_test;
		cv::Mat T_ciw = pLastFrame->mTcw;
		Eigen::Isometry3d T_ci_c0 = Eigen::Isometry3d::Identity();
		cv::cv2eigen(T_ciw, T_ci_c0.matrix());
		EdgeSE3DVLPoseOnly2 *edge = new EdgeSE3DVLPoseOnly2(pFrame->mT_e0_ej.inverse(),
		                                                    pLastFrame->mT_e0_ej.inverse(),
		                                                    T_ci_c0,
		                                                    pFrame->mT_e_c);
		edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
//					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
//					edge->setRobustKernel(rk);
//					rk->setDelta(deltaMono);
		// edge->setId(index);
		// set information according to covariance of EKF
		edge->setInformation(lamda_DVL * Eigen::Matrix<double, 6, 6>::Identity());
		optimizer.addEdge(edge);
	}

	//cout << "PO: vnIndexEdgeMono.size() = " << vnIndexEdgeMono.size() << "   vnIndexEdgeRight.size() = " << vnIndexEdgeRight.size() << endl;
	if (nInitialCorrespondences < 3) {
		return 0;
	}

	// We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
	// At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
	const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
	const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
	const int its[4] = {10, 10, 10, 10};

	int nBad = 0;
	for (size_t it = 0; it < 4; it++) {
		// set estimate of camera vertex as the pose of current frame
		vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
		optimizer.initializeOptimization(0);
		optimizer.optimize(its[it]);

		nBad = 0;

		// 3 for loop for check outlier

		// using mono, or using stereo but feature only in left image
		for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
			ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *e = vpEdgesMono[i];

			const size_t idx = vnIndexEdgeMono[i];

			if (pFrame->mvbOutlier[idx]) {
				e->computeError();
			}

			// ??? what is the meaning of chi2
			const float chi2 = e->chi2();

			if (chi2 > chi2Mono[it]) {
				pFrame->mvbOutlier[idx] = true;
				// ???? what does level work?
				e->setLevel(1);
				nBad++;
			}
			else {
				pFrame->mvbOutlier[idx] = false;
				e->setLevel(0);
			}

			if (it == 2) {
				e->setRobustKernel(0);
			}
		}
		// using stereo, but feature only in right
		for (size_t i = 0, iend = vpEdgesMono_FHR.size(); i < iend; i++) {
			ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody *e = vpEdgesMono_FHR[i];

			const size_t idx = vnIndexEdgeRight[i];

			if (pFrame->mvbOutlier[idx]) {
				e->computeError();
			}

			const float chi2 = e->chi2();

			if (chi2 > chi2Mono[it]) {
				pFrame->mvbOutlier[idx] = true;
				e->setLevel(1);
				nBad++;
			}
			else {
				pFrame->mvbOutlier[idx] = false;
				e->setLevel(0);
			}

			if (it == 2) {
				e->setRobustKernel(0);
			}
		}
		// using stereo, feature in both left and right image
		for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
			g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = vpEdgesStereo[i];

			const size_t idx = vnIndexEdgeStereo[i];

			if (pFrame->mvbOutlier[idx]) {
				e->computeError();
			}

			const float chi2 = e->chi2();

			if (chi2 > chi2Stereo[it]) {
				pFrame->mvbOutlier[idx] = true;
				e->setLevel(1);
				nBad++;
			}
			else {
				e->setLevel(0);
				pFrame->mvbOutlier[idx] = false;
			}

			if (it == 2) {
				e->setRobustKernel(0);
			}
		}

		if (optimizer.edges().size() < 10) {
			break;
		}
	}

	// Recover optimized pose and return number of inliers
	g2o::VertexSE3Expmap *vSE3_recov = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
	g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
	cv::Mat pose = Converter::toCvMat(SE3quat_recov);
	pFrame->SetPose(pose);

	//cout << "[PoseOptimization]: initial correspondences-> " << nInitialCorrespondences << " --- outliers-> " << nBad << endl;

	return nInitialCorrespondences - nBad;
}

void Optimizer::PoseOptimizationWithEKF(Frame *pFrame, Frame *pLastFrame)
{
	// setup solver
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

	g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);

	// number of edges
	int nInitialCorrespondences = 0;

	// Set Frame vertex
	g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
	vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
	vSE3->setId(0);
	vSE3->setFixed(false);
	optimizer.addVertex(vSE3);

	// Set MapPoint vertices

	// Number of KeyPoints.
	const int N = pFrame->N;

	// setup edges pointer
	vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *> vpEdgesMono;
	vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody *> vpEdgesMono_FHR;
	vector<size_t> vnIndexEdgeMono, vnIndexEdgeRight;
	vpEdgesMono.reserve(N);
	vpEdgesMono_FHR.reserve(N);
	vnIndexEdgeMono.reserve(N);
	vnIndexEdgeRight.reserve(N);

	vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> vpEdgesStereo;
	vector<size_t> vnIndexEdgeStereo;
	vpEdgesStereo.reserve(N);
	vnIndexEdgeStereo.reserve(N);

	// robust kernel
	const float deltaMono = sqrt(5.991);
	const float deltaStereo = sqrt(7.815);

	// add edges
	{
		unique_lock<mutex> lock(MapPoint::mGlobalMutex);

		// iterate all key points
		// N: Number of KeyPoints in the frame
		for (int i = 0; i < N; i++) {
			MapPoint *pMP = pFrame->mvpMapPoints[i];
			// if the key points has its correspending map points created
			if (pMP) {

				// add EKF constrains
				// the position map point under Last camera frame
				Eigen::Vector3d p_ci_test;
				cv::Mat P_c0 = pMP->GetWorldPos();
				Eigen::Vector3d p_c0;
				p_c0 << P_c0.at<float>(0), P_c0.at<float>(1), P_c0.at<float>(2);
				cv::Mat T_ciw = pLastFrame->mTcw;
				Eigen::Isometry3d T_ci_c0 = Eigen::Isometry3d::Identity();
				cv::cv2eigen(T_ciw, T_ci_c0.matrix());
				EdgeSE3DVLPoseOnly *edge = new EdgeSE3DVLPoseOnly(pFrame->mT_e0_ej.inverse(),
				                                                  pLastFrame->mT_e0_ej.inverse(),
				                                                  T_ci_c0,
				                                                  pFrame->mT_e_c,
				                                                  p_c0);
				edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
//					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
//					edge->setRobustKernel(rk);
//					rk->setDelta(deltaMono);
				// edge->setId(index);
				// set information according to covariance of EKF
				edge->setInformation(Eigen::Matrix3d::Identity());
				optimizer.addEdge(edge);
			}
		}
	}
	optimizer.initializeOptimization(0);
	optimizer.optimize(20);



	// Recover optimized pose and return number of inliers
	g2o::VertexSE3Expmap *vSE3_recov = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
	g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
	cv::Mat pose = Converter::toCvMat(SE3quat_recov);
	pFrame->SetPose(pose);

	//cout << "[PoseOptimization]: initial correspondences-> " << nInitialCorrespondences << " --- outliers-> " << nBad << endl;

}

} // namespace ORB_SLAM3
