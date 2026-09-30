//
// Created by da on 08/06/2021.
//

#include "DvlGyroOptimizer.h"

#include <complex>

#include <Eigen/StdVector>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <iostream>
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

#include <mutex>

#include "OptimizableTypes.h"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

namespace ORB_SLAM3
{
void DvlGyroOptimizer::LocalDVLBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, int &num_fixedKF)
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
void DvlGyroOptimizer::LocalDVLGyroBundleAdjustment(KeyFrame *pKF,
													bool *pbStopFlag,
													Map *pMap,
													int &num_fixedKF,
													double lamda_DVL)
{
	Map *pCurrentMap = pKF->GetMap();
	int Nd = std::min(10, (int)pCurrentMap->KeyFramesInMap() - 2);// number of keyframes in current map
	const unsigned long maxKFid = pKF->mnId;

	vector<KeyFrame *> OptKFs;
	OptKFs.reserve(Nd);
	OptKFs.push_back(pKF);
	pKF->mnBALocalForKF = pKF->mnId;

	for (int i = 1; i > Nd; i++) {
		if (OptKFs.back()->mPrevKF) {
			OptKFs.push_back(OptKFs.back()->mPrevKF);
			OptKFs.back()->mnBALocalForKF = pKF->mnId;
		}
		else {
			break;
		}
	}
	int N = OptKFs.size();

	vector<KeyFrame *> FixedKFs;
	if (OptKFs.back()->mPrevKF) {
		FixedKFs.push_back(OptKFs.back()->mPrevKF);
		OptKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
	}
	else {
		OptKFs.back()->mnBALocalForKF = 0;
		OptKFs.back()->mnBAFixedForKF = pKF->mnId;
		FixedKFs.push_back(OptKFs.back());
		OptKFs.pop_back();
	}


	vector<MapPoint *> LocalMapPoints;
	for (int i = 0; i < N; i++) {
		vector<MapPoint *> vpMPs = OptKFs[i]->GetMapPointMatches();
		for (vector<MapPoint *>::iterator it = vpMPs.begin(); it != vpMPs.end(); it++) {
			MapPoint *pMP = *it;
			if (pMP) {
//				cout<<"find local map point"<<endl;
				if (!pMP->isBad()) {
//					cout<<"find local good map point"<<endl;
					if (pMP->mnBALocalForKF != pKF->mnId) {
//						cout<<"find local map point with correct BALocalForKF"<<endl;
						LocalMapPoints.push_back(pMP);
						pMP->mnBALocalForKF = pKF->mnId;
//						cout<<"add local map point to optimize: "<<endl;
					}
				}
			}
		}
	}
	int N_map_points = LocalMapPoints.size();
//	cout << "map point to optimize: " << N_map_points << endl;

	const int maxFixedKF = 30;
	for (vector<MapPoint *>::iterator it = LocalMapPoints.begin(); it != LocalMapPoints.end(); it++) {
		map<KeyFrame *, tuple<int, int>> observations = (*it)->GetObservations();
		for (map<KeyFrame *, tuple<int, int>>::iterator it_ob = observations.begin(); it_ob != observations.end();
			 it_ob++) {
			KeyFrame *pKFi = it_ob->first;
			if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
				pKFi->mnBAFixedForKF = pKF->mnId;
				if (!pKFi->isBad()) {
					FixedKFs.push_back(pKFi);
					break;
				}
			}
		}
		if (FixedKFs.size() >= maxFixedKF) {
			break;
		}
	}
	int N_fixed = FixedKFs.size();


	Verbose::PrintMess("DVL Gyro optimization", Verbose::VERBOSITY_NORMAL);
	int its = 200; // Check number of iterations
//	const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);


	optimizer.setAlgorithm(solver);

	// Set KeyFrame vertices (fixed poses and optimizable velocities)
	for (size_t i = 0; i < OptKFs.size(); i++) {
		KeyFrame *pKFi = OptKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}
		VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
		VP->setId(pKFi->mnId);
		VP->setFixed(false);
		optimizer.addVertex(VP);
	}
	for (int i = 0; i < FixedKFs.size(); i++) {
		KeyFrame *pKFi = FixedKFs[i];
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
	for (int i = 0; i < N; i++) {
		VertexGyroBias *VG = new VertexGyroBias(OptKFs[i]);
		VG->setId(maxKFid + 1 + i);
		VG->setFixed(true);
		optimizer.addVertex(VG);
	}
	for (int i = 0; i < N_fixed; i++) {
		VertexGyroBias *VG = new VertexGyroBias(FixedKFs[i]);
		VG->setId(maxKFid + N + 1 + i);
		VG->setFixed(true);
		optimizer.addVertex(VG);
	}


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
	vT_d_c->setEstimate(Converter::toSE3Quat(pKF->mImuCalib.mT_dvl_c));
	vT_d_c->setId(maxKFid + N + N_fixed + 1);
	vT_d_c->setFixed(true);
	optimizer.addVertex(vT_d_c);

	g2o::VertexSE3Expmap *vT_g_d = new g2o::VertexSE3Expmap();
	vT_g_d->setEstimate(Converter::toSE3Quat(pKF->mImuCalib.mT_gyro_dvl));
	vT_g_d->setId(maxKFid + N + N_fixed + 2);
	vT_g_d->setFixed(true);
	optimizer.addVertex(vT_g_d);

	vector<EdgeMonoBA_DvlGyros *> mono_edges;
	vector<EdgeStereoBA_DvlGyros *> stereo_edges;
	//add map point vertex and visual constrain
	{
		unique_lock<mutex> lock(MapPoint::mGlobalMutex);

		for (int i = 0; i < N_map_points; i++) {
			MapPoint *pMP = LocalMapPoints[i];
			if (pMP) {
				g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
				vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
				int id = pMP->mnId + maxKFid + 1;
				vPoint->setId(maxKFid + N + N_fixed + 2 + 1 + i);
				vPoint->setMarginalized(true);
				optimizer.addVertex(vPoint);

				const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

				for (auto ob: observations) {
					KeyFrame *pKFi = ob.first;
					if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
						continue;
					}

					if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
						const int leftIndex = get<0>(ob.second);

						// Monocular observation
						if (leftIndex != -1 && pKFi->mvuRight[get<0>(ob.second)] < 0) {
							const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
							Eigen::Matrix<double, 2, 1> obs;
							obs << kpUn.pt.x, kpUn.pt.y;

							EdgeMonoBA_DvlGyros *e = new EdgeMonoBA_DvlGyros();

							e->setVertex(0,
										 dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
							e->setVertex(1, vPoint);
							e->setMeasurement(obs);
							const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
							e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

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

							e->setVertex(0,
										 dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
							e->setVertex(1, vPoint);
							e->setMeasurement(obs);
							const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
							Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
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
	dvl_edges.reserve(OptKFs.size());
	vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
//	vppUsedKF.reserve(OptKFs.size() + FixedKFs.size());
//	std::cout << "build optimization graph" << std::endl;

	for (size_t i = 0; i < OptKFs.size(); i++) {
		KeyFrame *pKFi = OptKFs[i];

		if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
			if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) {
				continue;
			}

			VertexPoseDvlIMU *VP1 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mPrevKF->mnId));
//				g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
			VertexPoseDvlIMU *VP2 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
//				g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
			g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid + 1 + i);
			g2o::HyperGraph::Vertex *VT_d_c = optimizer.vertex(maxKFid + N + N_fixed + 1);
			g2o::HyperGraph::Vertex *VT_g_d = optimizer.vertex(maxKFid + N + N_fixed + 2);
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
			EdgeDvlGyroBA *ei = new EdgeDvlGyroBA(pKFi->mpDvlPreintegrationKeyFrame);
//				ei->setVertex(0, VP1);

//			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
//			ei->setRobustKernel(rk);
//			rk->setDelta(sqrt(7.815));
			ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
			ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
			ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
			ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
			ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
			ei->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * lamda_DVL * (stereo_edges.size()+mono_edges.size()));
			ei->setId(pKFi->mnId);


			dvl_edges.push_back(ei);

			vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
			optimizer.addEdge(ei);
		}
	}

	const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
	const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};

	// Compute error for different scales
//	std::cout << "start optimization" << std::endl;
	optimizer.setVerbose(false);

	optimizer.initializeOptimization(0);
	optimizer.optimize(5);

	if(0){
		int mono_outlier;
		int stereo_outlier;
		float visula_chi2, dvl_chi2;
		for (int i = 0; i < 4; i++) {
			optimizer.initializeOptimization(0);
			optimizer.optimize(10);

			mono_outlier = 0;
			stereo_outlier = 0;
			visula_chi2 = 0;
			dvl_chi2 = 0;
			for (auto e_mono: mono_edges) {
				e_mono->computeError();
				const float chi2 = e_mono->chi2();
				if (chi2 > chi2Mono[i]) {
					e_mono->setLevel(1);
				}
				visula_chi2 += chi2;
			}
			for (auto e_stereo: stereo_edges) {
				e_stereo->computeError();
				const float chi2 = e_stereo->chi2();
				if (chi2 > chi2Stereo[i]) {
					e_stereo->setLevel(1);
				}
				visula_chi2 += chi2;
			}
			for (auto e_dvl: dvl_edges) {
				e_dvl->computeError();
				const float chi2 = e_dvl->chi2();
				dvl_chi2 += chi2;
//			cout << "dvl error: " << e_dvl->error().transpose() << endl;
			}
			cout << "iteration: " << i << " visual chi2: " << visula_chi2 << " dvl_gyro chi2: " << dvl_chi2 << endl;
		}
	}





	int visual_edge_num = mono_edges.size() + stereo_edges.size();
	int dvl_edge_num = dvl_edges.size();

//	std::cout << "end optimization" << std::endl;


	// Recover optimized data
	// Biases
	VertexGyroBias *VG = static_cast<VertexGyroBias *>(optimizer.vertex(maxKFid + 1));
	Eigen::Matrix<double, 6, 1> vb;
	Eigen::Vector3d bg;
	vb << VG->estimate(), 0, 0, 0;
	bg << VG->estimate();
//
//	vT_d_c = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(maxKFid + 2));
//
//	Eigen::Isometry3d T_dvl_c = vT_d_c->estimate();
//	Eigen::Isometry3d T_gyros_dvl = vT_g_d->estimate();
//
	IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
//
//	Eigen::Matrix3d R_gt;
//	R_gt << 0, 0, 1,
//		-1, 0, 0,
//		0, -1, 0;
//	cout << "init optimization result: \n"
//		 << "bias_gyro:\n" << bg << "\n"
//		 << "R_dvl_c:\n" << T_dvl_c.rotation() << "\n"
//		 << "R_dvl_c(eular yaw-pitch-roll):" << T_dvl_c.rotation().eulerAngles(2, 1, 0).transpose() << "\n"
//		 << "t_dvl_c:" << T_dvl_c.translation().transpose() << "\n"
//		 << "R_dvl_c distance with R_gt(LogSO3()): " << LogSO3(T_dvl_c.rotation().inverse() * R_gt).transpose() << "\n"
//		 << "R_gyros_dvl:\n" << T_gyros_dvl.rotation() << "\n"
//		 << "R_gyros_dvl(eular yaw-pitch-roll):" << T_gyros_dvl.rotation().eulerAngles(2, 1, 0).transpose() << "\n"
//		 << endl;
//
//
	cv::Mat cvbg = Converter::toCvMat(bg);

	//Keyframes velocities and biases
//	std::cout << "update Keyframes biases" << std::endl;

	for (size_t i = 0; i < N; i++) {
		KeyFrame *pKFi = OptKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}

		VertexPoseDvlIMU *VP = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
		Eigen::Quaterniond Rwc(VP->estimate().Rwc);
		Eigen::Vector3d twc = VP->estimate().twc;
		Eigen::Isometry3d Twc = Eigen::Isometry3d::Identity();
		Twc.pretranslate(twc);
		Twc.rotate(Rwc);
		Eigen::Isometry3d Tcw = Twc.inverse();
		cv::Mat Tcw_cv;
		cv::eigen2cv(Tcw.matrix(), Tcw_cv);
		Tcw_cv.convertTo(Tcw_cv, CV_32F);
		pKFi->SetPose(Tcw_cv);

//		if (cv::norm(pKFi->GetGyroBias() - cvbg) > 0.01) {
//			pKFi->SetNewBias(b);
//			if (pKFi->mpDvlPreintegrationKeyFrame) {
//				pKFi->mpDvlPreintegrationKeyFrame->ReintegrateWithVelocity();
//			}
//		}
//		else {
//			pKFi->SetNewBias(b);
//		}
	}

	for (int i = 0; i < N_map_points; i++) {
		MapPoint *pMP = LocalMapPoints[i];
		g2o::VertexSBAPointXYZ
			*vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(maxKFid + N + N_fixed + 2 + 1 + i));
		pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMP->UpdateNormalAndDepth();
	}
	pMap->IncreaseChangeIndex();
}

void
DvlGyroOptimizer::LocalDVLIMUBundleAdjustment(Atlas* pAtlas, KeyFrame* pKF, bool* pbStopFlag, Map* pMap, int &num_fixedKF,
                                              double lamda_DVL, double lamda_visual)
{
    Map *pCurrentMap = pKF->GetMap();
    int Nd = std::min(10, (int)pCurrentMap->KeyFramesInMap() - 2);// number of keyframes in current map
    const unsigned long maxKFid = pKF->mnId;

    vector<KeyFrame *> OptKFs;
    OptKFs.reserve(Nd);
    OptKFs.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    for (int i = 1; i < Nd; i++) {
        if (OptKFs.back()->mPrevKF) {
            OptKFs.push_back(OptKFs.back()->mPrevKF);
            OptKFs.back()->mnBALocalForKF = pKF->mnId;
        }
        else {
            break;
        }
    }
    int N = OptKFs.size();

    vector<KeyFrame *> FixedKFs;
    if (OptKFs.back()->mPrevKF) {
        FixedKFs.push_back(OptKFs.back()->mPrevKF);
        OptKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
    }
    else {
        OptKFs.back()->mnBALocalForKF = 0;
        OptKFs.back()->mnBAFixedForKF = pKF->mnId;
        FixedKFs.push_back(OptKFs.back());
        OptKFs.pop_back();
    }
    // add more Fixed but connected KF
    auto connectedKF = FixedKFs.back()->GetConnectedKeyFrames();
    for(auto pKFi:connectedKF){
        // check whether pKFi is in OptKFS or FixedKFs
        bool inOptKFs = false;
        bool inFixedKFs = false;
        for(auto pOptKF:OptKFs){
            if(pKFi->mnId == pOptKF->mnId){
                inOptKFs = true;
                break;
            }
        }
        for(auto pFixedKF:FixedKFs){
            if(pKFi->mnId == pFixedKF->mnId){
                inFixedKFs = true;
                break;
            }
        }
        if(!inOptKFs&&!inFixedKFs&&FixedKFs.size()<50){
            pKFi->mnBAFixedForKF = pKF->mnId;
            FixedKFs.push_back(pKFi);
        }
    }


    vector<MapPoint *> LocalMapPoints;
    for (int i = 0; i < N; i++) {
        vector<MapPoint *> vpMPs = OptKFs[i]->GetMapPointMatches();
        for (vector<MapPoint *>::iterator it = vpMPs.begin(); it != vpMPs.end(); it++) {
            MapPoint *pMP = *it;
            if (pMP) {
                //				cout<<"find local map point"<<endl;
                if (!pMP->isBad()) {
                    //					cout<<"find local good map point"<<endl;
                    if (pMP->mnBALocalForKF != pKF->mnId) {
                        //						cout<<"find local map point with correct BALocalForKF"<<endl;
                        LocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                        //						cout<<"add local map point to optimize: "<<endl;
                    }
                }
            }
        }
    }
    vector<MapPoint *> LocalFixedMapPoints;
    for(auto pKFi:FixedKFs){
        auto Mps = pKFi->GetMapPointMatches();
        for(auto pMP:Mps){
            if(!pMP||pMP->isBad()){
                continue;
            }
            if (pMP->mnBALocalForKF != pKF->mnId) {
                //						cout<<"find local map point with correct BALocalForKF"<<endl;
                LocalFixedMapPoints.push_back(pMP);
                LocalMapPoints.push_back(pMP);
                pMP->mnBALocalForKF = pKF->mnId;
                //						cout<<"add local map point to optimize: "<<endl;
            }
        }
    }
    int N_map_points = LocalMapPoints.size();
    //	cout << "map point to optimize: " << N_map_points << endl;

    // const int maxFixedKF = 30;
    // for (vector<MapPoint *>::iterator it = LocalMapPoints.begin(); it != LocalMapPoints.end(); it++) {
    //     map<KeyFrame *, tuple<int, int>> observations = (*it)->GetObservations();
    //     for (map<KeyFrame *, tuple<int, int>>::iterator it_ob = observations.begin(); it_ob != observations.end();
    //          it_ob++) {
    //         KeyFrame *pKFi = it_ob->first;
    //         if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
    //             pKFi->mnBAFixedForKF = pKF->mnId;
    //             if (!pKFi->isBad()) {
    //                 FixedKFs.push_back(pKFi);
    //                 break;
    //             }
    //         }
    //     }
    //     if (FixedKFs.size() >= maxFixedKF) {
    //         break;
    //     }
    // }
    int N_fixed = FixedKFs.size();


    Verbose::PrintMess("DVL Gyro optimization", Verbose::VERBOSITY_NORMAL);
    int its = 200; // Check number of iterations
    //	const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    
    optimizer.setAlgorithm(solver);

    // Set KeyFrame vertices (fixed poses and optimizable velocities)
    // record pose before optimize
    std::map<VertexPoseDvlIMU*, Eigen::Isometry3d> map_pose_original;
    for (size_t i = 0; i < OptKFs.size(); i++) {
        KeyFrame *pKFi = OptKFs[i];
        if (pKFi->mnId > maxKFid) {
            continue;
        }
        VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);
        Eigen::Isometry3d T_c0_cj = Eigen::Isometry3d::Identity();
        T_c0_cj.rotate(VP->estimate().Rwc);
        T_c0_cj.pretranslate(VP->estimate().twc);
        map_pose_original.insert(std::pair<VertexPoseDvlIMU*, Eigen::Isometry3d>(VP, T_c0_cj));
        // ROS_INFO_STREAM("opt KF: "<<pKFi->mnId);

        // DVLGroPreIntegration *pDVLGroPreIntegration2 = new DVLGroPreIntegration();
        // boost::archive::text_iarchive ia1(i_file1);
        // ia1 >> pDVLGroPreIntegration2;
        // oa2 << pDVLGroPreIntegration2;
        // o_file2.close();

    }
    for (int i = 0; i < FixedKFs.size(); i++) {
        KeyFrame *pKFi = FixedKFs[i];
        if (pKFi->mnId > maxKFid) {
            continue;
        }
        VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);
        // ROS_INFO_STREAM("fixed KF: "<<pKFi->mnId);
    }

    // Biases
    vector<VertexGyroBias*> vpgb;
    vector<VertexAccBias*> vpab;
    //todo_tightly
    //	set fixed for debuging
    // Eigen::Vector3d gyros_b(-0.000535183,-0.00224689,0.000705318);//halftank_easy
    // Eigen::Vector3d gyros_b(-0.000535183,-0.00194689,0.000705318);//halftank_medium
    // Eigen::Vector3d gyros_b(0.00317551, -0.00854424, -0.000787445);//Structure_Hard
    // Eigen::Vector3d gyros_b(0.00247442, -0.002438478, 0.00188971);//Structure_Medium
    // Eigen::Vector3d gyros_b(0.00247442, -0.000638478, 0.00188971);//Halftank_Hard
    // Eigen::Vector3d gyros_b(-0.000653851, -0.00174641, 0.0020714);//wholetank_hard
    Eigen::Vector3d gyros_b(0, 0,  0);//halftank_medium

    for(auto pKFi:OptKFs){
        VertexGyroBias *VG = new VertexGyroBias(pKFi);
        // VertexGyroBias *VG = new VertexGyroBias(gyros_b);
        VG->setId(maxKFid + 1 + pKFi->mnId);
        VG->setFixed(false);
        optimizer.addVertex(VG);
        vpgb.push_back(VG);

        VertexAccBias *VA = new VertexAccBias(pKFi);
        VA->setId((maxKFid + 1)*2 + pKFi->mnId);
        if(pAtlas->IsIMUCalibrated())
            VA->setFixed(false);
        else
            VA->setFixed(true);
        optimizer.addVertex(VA);
        vpab.push_back(VA);

        VertexVelocity *VV = new VertexVelocity(pKFi);
        VV->setId((maxKFid + 1)*3 + pKFi->mnId);
        if(pAtlas->IsIMUCalibrated())
            VV->setFixed(false);
        else
            VV->setFixed(true);
        optimizer.addVertex(VV);
    }
    for(auto pKFi:FixedKFs){
        VertexGyroBias *VG = new VertexGyroBias(pKFi);
        // VertexGyroBias *VG = new VertexGyroBias(gyros_b);
        VG->setId(maxKFid + 1 + pKFi->mnId);
        VG->setFixed(true);
        optimizer.addVertex(VG);
        vpgb.push_back(VG);

        VertexAccBias *VA = new VertexAccBias(pKFi);
        VA->setId((maxKFid + 1)*2 + pKFi->mnId);
        VA->setFixed(true);
        optimizer.addVertex(VA);
        // vpab.push_back(VA);

        VertexVelocity *VV = new VertexVelocity(pKFi);
        VV->setId((maxKFid + 1)*3 + pKFi->mnId);
        VV->setFixed(true);
        optimizer.addVertex(VV);

    }

    // extrinsic parameter
    g2o::VertexSE3Expmap *vT_d_c = new g2o::VertexSE3Expmap();
    vT_d_c->setEstimate(Converter::toSE3Quat(pKF->mImuCalib.mT_dvl_c));
    vT_d_c->setId((maxKFid + 1)*4);
    vT_d_c->setFixed(true);
    optimizer.addVertex(vT_d_c);

    g2o::VertexSE3Expmap *vT_g_d = new g2o::VertexSE3Expmap();
    vT_g_d->setEstimate(Converter::toSE3Quat(pKF->mImuCalib.mT_gyro_dvl));
    vT_g_d->setId((maxKFid + 1)*4+1);
    vT_g_d->setFixed(true);
    optimizer.addVertex(vT_g_d);

    VertexGDir *VGDir = new VertexGDir(pAtlas->getRGravity());
    VGDir->setId((maxKFid + 1)*4+2);
    VGDir->setFixed(true);
    optimizer.addVertex(VGDir);

    vector<EdgeMonoBA_DvlGyros *> mono_edges;
    vector<EdgeStereoBA_DvlGyros *> stereo_edges;
    std::map<g2o::VertexSBAPointXYZ*, VertexPoseDvlIMU*> map_point_observation;
    std::set<g2o::VertexSBAPointXYZ*> vertex_point;
    std::map<EdgeMonoBA_DvlGyros*,std::pair<KeyFrame*,MapPoint*>> mono_obs;
    std::map<EdgeStereoBA_DvlGyros*,std::pair<KeyFrame*,MapPoint*>> stereo_obs;
    //add map point vertex and visual constrain
    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for (int i = 0; i < N_map_points; i++) {
            MapPoint *pMP = LocalMapPoints[i];
            if (pMP) {
                g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
                vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
                int id = pMP->mnId + maxKFid + 1;
                vPoint->setId((maxKFid + 1)*5 + i);
                vPoint->setMarginalized(true);
                //check whther pMP is in LocalFixedMapPoints
                if (find(LocalFixedMapPoints.begin(), LocalFixedMapPoints.end(), pMP) != LocalFixedMapPoints.end()) {
                    vPoint->setFixed(true);
                } else {
                    vPoint->setFixed(false);
                }
                optimizer.addVertex(vPoint);

                const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

                for (auto ob: observations) {
                    KeyFrame *pKFi = ob.first;
                    if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
                        continue;
                    }

                    if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
                        std::pair<KeyFrame*,MapPoint*> o = std::make_pair(pKFi,pMP);
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
                            e->setId(optimizer.edges().size());
                            e->setMeasurement(obs);
                            const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                            VertexPoseDvlIMU* v1 = dynamic_cast<VertexPoseDvlIMU*>(e->vertices()[0]);
                            if(v1->estimate().mPoorVision){
                                e->setInformation(Eigen::Matrix<double, 2, 2>::Identity()* invSigma2 * lamda_visual * 1);
                            }
                            else{
                                e->setInformation(Eigen::Matrix<double, 2, 2>::Identity()* invSigma2 * lamda_visual);
                            }
                            // e->setInformation(Eigen::Matrix2d::Identity() * invSigma2* lamda_visual);

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(sqrt(1));

                            mono_edges.push_back(e);
                            optimizer.addEdge(e);

                            mono_obs.insert(std::make_pair(e,o));

                            auto all_verteices = e->vertices();
                            if (VertexPoseDvlIMU* v_pose = dynamic_cast<VertexPoseDvlIMU*>(all_verteices[0])) {
                                if (g2o::VertexSBAPointXYZ* v_point = dynamic_cast<g2o::VertexSBAPointXYZ*>(all_verteices[1])) {
                                    vertex_point.insert(v_point);
                                    if (map_point_observation.find(v_point) == map_point_observation.end()) {
                                        map_point_observation.insert(
                                                std::pair<g2o::VertexSBAPointXYZ*, VertexPoseDvlIMU*>(v_point, v_pose));
                                    }
                                }
                            }
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
                            e->setId(optimizer.edges().size());
                            e->setMeasurement(obs);
                            const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2 * lamda_visual;
                            VertexPoseDvlIMU* v1 = dynamic_cast<VertexPoseDvlIMU*>(e->vertices()[0]);
                            if(v1->estimate().mPoorVision){
                                e->setInformation(Info  * 1);
                            }
                            else{
                                e->setInformation(Info );
                            }

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(sqrt(2));

                            stereo_edges.push_back(e);
                            optimizer.addEdge(e);
                            stereo_obs.insert(std::make_pair(e,o));

                            auto all_verteices = e->vertices();
                            if (VertexPoseDvlIMU* v_pose = dynamic_cast<VertexPoseDvlIMU*>(all_verteices[0])) {
                                if (g2o::VertexSBAPointXYZ* v_point = dynamic_cast<g2o::VertexSBAPointXYZ*>(all_verteices[1])) {
                                    vertex_point.insert(v_point);
                                    if (map_point_observation.find(v_point) == map_point_observation.end()) {
                                        map_point_observation.insert(
                                                std::pair<g2o::VertexSBAPointXYZ*, VertexPoseDvlIMU*>(v_point, v_pose));
                                    }
                                }
                            }
                        }

                    }

                }
            }
        }
    }
    // ROS_INFO_STREAM("visual edge size: "<<(mono_edges.size()+stereo_edges.size()));


    // Graph edges
    vector<EdgeDvlIMU *> dvlimu_edges;
    vector<EdgeSE3DVLIMU *> se3_edges;
    vector<EdgePriorAcc *> acc_edge;
    vector<EdgePriorGyro *> gyro_edge;
    vector<EdgeDvlVelocity *> velocity_edge;
    dvlimu_edges.reserve(OptKFs.size());
    vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
    //	vppUsedKF.reserve(OptKFs.size() + FixedKFs.size());
    //	std::cout << "build optimization graph" << std::endl;
    sort(OptKFs.begin(),OptKFs.end(),KFComparator());
    for (size_t i = 0; i < OptKFs.size(); i++) {
        KeyFrame *pKFi = OptKFs[i];

        if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
            if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) {
                continue;
            }

            // VertexPoseDvlGro *VP1 = dynamic_cast<VertexPoseDvlGro *>(optimizer.vertex(pKFi->mPrevKF->mnId));
            //				g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
            // VertexPoseDvlGro *VP2 = dynamic_cast<VertexPoseDvlGro *>(optimizer.vertex(pKFi->mnId));
            //				g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
            // g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid + 1 + i);
            // g2o::HyperGraph::Vertex *VT_d_c = optimizer.vertex(maxKFid + N + N_fixed + 1);
            // g2o::HyperGraph::Vertex *VT_g_d = optimizer.vertex(maxKFid + N + N_fixed + 2);

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

            if (!VP1|| !VP2 || !VV1 || !VV2 || !VG1 || !VG2 || !VA1 || !VA2 || !VT_d_c || !VT_g_d || !VR_b0_w) {
                ROS_ERROR_STREAM("LocalVAIBA Error");
                assert(0);
            }
            EdgeAccRW* e_bias = new EdgeAccRW();
            e_bias->setLevel(0);
            e_bias->setVertex(0,VA1);
            e_bias->setVertex(1,VA2);
            Eigen::Matrix3d info_acc_bias = Eigen::Matrix3d::Identity()*1e10;
            if(pAtlas->IsIMUCalibrated()){
                cv::Mat cvInfoA = pKFi->mpDvlPreintegrationKeyFrame->C.rowRange(12,15).colRange(12,15).inv(cv::DECOMP_SVD);
                cv::cv2eigen(cvInfoA,info_acc_bias);
            }
            e_bias->setInformation(info_acc_bias);
            // if(info_acc_bias(0,0)==0)
            ROS_DEBUG_STREAM("KF["<<pKFi->mnId<<"] acc bias info:\n"<<info_acc_bias);
            optimizer.addEdge(e_bias);

            EdgeGyroRW* eg_bias = new EdgeGyroRW();
            eg_bias->setLevel(0);
            eg_bias->setVertex(0,VG1);
            eg_bias->setVertex(1,VG2);
            Eigen::Matrix3d info_gyro_bias = Eigen::Matrix3d::Identity()*1e4;
            if(pAtlas->IsIMUCalibrated()){
                cv::Mat cvInfoG = pKFi->mpDvlPreintegrationKeyFrame->C.rowRange(9,12).colRange(9,12).inv(cv::DECOMP_SVD);
                cv::cv2eigen(cvInfoG,info_gyro_bias);
            }
            if(i==0){
                //set robust kernel
                eg_bias->setInformation(info_gyro_bias * 1e-2);
//                ROS_INFO_STREAM("first KF:"<<pKFi->mnId);
//                 g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//                 rk->setDelta(sqrt(16.92));
            }
            eg_bias->setInformation(info_gyro_bias);
            // if(info_gyro_bias(0,0)==0)
            ROS_DEBUG_STREAM("KF["<<pKFi->mnId<<"] gyro bias info:\n"<<info_gyro_bias);

            optimizer.addEdge(eg_bias);

            EdgeDvlIMU2 *eDVL = new EdgeDvlIMU2(pKFi->mPrevKF->mpDvlPreintegrationKeyFrame,pKFi->mpDvlPreintegrationKeyFrame);
            eDVL->setLevel(0);
            eDVL->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
            eDVL->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
            eDVL->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
            eDVL->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
            eDVL->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG2));
            eDVL->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA2));
            eDVL->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
            eDVL->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
            eDVL->setVertex(8, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VR_b0_w));
            eDVL->setId(optimizer.edges().size());
            Eigen::Matrix<double,9,9> info_DVL=Eigen::Matrix<double,9,9>::Identity();
            info_DVL.block(0,0,3,3) = Eigen::Matrix3d::Identity() * 1e5;
            info_DVL.block(3,3,3,3) = Eigen::Matrix3d::Identity() * 1e5;
            info_DVL.block(6,6,3,3) = Eigen::Matrix3d::Identity() * 1e5;
            eDVL->setInformation(info_DVL);
            // if(!pAtlas->IsIMUCalibrated()){
            //     eDVL->setLevel(1);
            // }
            ROS_DEBUG_STREAM("DVL edge info:\n"<<info_DVL);
            optimizer.addEdge(eDVL);



			EdgeDvlIMU *eG = new EdgeDvlIMU(pKFi->mpDvlPreintegrationKeyFrame);
            eG->setLevel(0);
            eG->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
            eG->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
            eG->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
            eG->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
            eG->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG2));
            eG->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA2));
            eG->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
            eG->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
            eG->setVertex(8, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VR_b0_w));
            eG->setId(optimizer.edges().size());
            Eigen::Matrix<double,9,9> info_DI=Eigen::Matrix<double,9,9>::Identity();
            int visual_size = (mono_edges.size()+stereo_edges.size());
            if(pAtlas->IsIMUCalibrated()){
                cv::Mat cvInfo = pKFi->mpDvlPreintegrationKeyFrame->C.rowRange(0,9).colRange(0,9).inv(cv::DECOMP_SVD);
                cv::cv2eigen(cvInfo,info_DI);
                // info_DI=Eigen::Matrix<double,9,9>::Identity();
                // info_DI.block(0,0,3,3) = Eigen::Matrix3d::Identity() * 1e6;
                // info_DI.block(3,3,3,3) = Eigen::Matrix3d::Identity() * 1e4;
                // info_DI.block(6,6,3,3) = Eigen::Matrix3d::Identity() * 1e4;
            }
            else{
                // ROS_DEBUG_STREAM("Poor vision Hign DVL BA");
                // eG->setLevel(1);
                info_DI.block(0,0,3,3) = Eigen::Matrix3d::Identity() * 1;
                info_DI.block(3,3,3,3) = Eigen::Matrix3d::Identity() * 1e3;
                info_DI.block(6,6,3,3) = Eigen::Matrix3d::Identity() * 1e3;
            }
            // info(0,0) = info(0,0)*lamda_DVL * 5e3; // 10_24
            // info_DI(1,1) = 1e10; // before 10_24
            // ROS_INFO_STREAM("info: "<<info_DI);
            eG->setInformation(info_DI);
            ROS_DEBUG_STREAM("IMU edge info:\n"<<info_DI);
            dvlimu_edges.push_back(eG);
            optimizer.addEdge(eG);
        }
    }


    optimizer.initializeOptimization(0);
    optimizer.optimize(20);
    if(pbStopFlag){
        optimizer.setForceStopFlag(pbStopFlag);
    }
    stringstream ss_v_chi2;
    std::set<std::pair<KeyFrame*,MapPoint*>> remove_obs;
    ss_v_chi2<<"mono chi2: ";
    for (auto e: mono_edges) {
        e->computeError();
        ss_v_chi2<<e->chi2()<<", ";
        // e->setInformation(Eigen::Matrix2d::Identity() * lamda_visual);
        if (e->chi2() > 10) {
            e->setLevel(1);
            remove_obs.insert(mono_obs[e]);
        }
        else {
            e->setLevel(0);
            // e->setInformation(Eigen::Matrix2d::Identity() * 1);
        }

    }
    ss_v_chi2<<"\n";
    ss_v_chi2<<"stereo chi2: ";
    for (auto e: stereo_edges) {
        e->computeError();
        ss_v_chi2<<e->chi2()<<", ";
        // e->setInformation(Eigen::Matrix3d::Identity() * lamda_visual);
        if (e->chi2() > 20) {
            e->setLevel(1);
            remove_obs.insert(stereo_obs[e]);
        }
        else {
            e->setLevel(0);
        }
    }
    // ROS_INFO_STREAM(ss_v_chi2.str());
    for(int i=0;i<4;i++){
        if(pbStopFlag){
            if(*pbStopFlag){
                ROS_DEBUG_STREAM("stop BA");
                break;
            }
        }
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);
    }



    // Recover optimized data
    stringstream ss;
    ss<<"LocalVisualAcousticInertial BA"<<"\n";
    unique_lock<shared_timed_mutex> lock(pMap->mMutexMapUpdate);
//    for(auto o:remove_obs){
//        KeyFrame* pKFi = o.first;
//        MapPoint* pMpi = o.second;
//        pKFi->EraseMapPointMatch(pMpi);
//        pMpi->EraseObservation(pKFi);
//        ss<<"remove map point, KF: "<<pKFi->mnId<<", MP: "<<pMpi->mnId<<"\n";
//    }
    //update KF after current KF
    auto all_kf = pMap->GetAllKeyFrames();
    for(auto pkfi:all_kf){
        // Pose
        VertexPoseDvlIMU *VP = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKF->mnId));
        Eigen::Quaterniond Rwci(VP->estimate().Rwc);
        Eigen::Vector3d twci = VP->estimate().twc;
        Eigen::Isometry3d T_w_ci_new = Eigen::Isometry3d::Identity();
        T_w_ci_new.pretranslate(twci);
        T_w_ci_new.rotate(Rwci);

        cv::Mat Twci = pKF->GetPoseInverse();
        Eigen::Isometry3d T_w_ci = Eigen::Isometry3d::Identity();
        cv::cv2eigen(Twci, T_w_ci.matrix());
        Eigen::Isometry3d T_ci_w = T_w_ci.inverse();

        if(pkfi->mnId>pKF->mnId){
            cv::Mat Twcj = pkfi->GetPoseInverse();
            Eigen::Isometry3d T_w_cj = Eigen::Isometry3d::Identity();
            cv::cv2eigen(Twcj,T_w_cj.matrix());
            Eigen::Isometry3d T_ci_cj = T_ci_w * T_w_cj;
            Eigen::Isometry3d T_w_cj_new = T_w_ci_new * T_ci_cj;
            Eigen::Isometry3d T_cj_w_new = T_w_ci_new.inverse();
            cv::Mat Tcw;
            cv::eigen2cv(T_cj_w_new.matrix(),Tcw);
            Tcw.convertTo(Tcw, CV_32F);
            pkfi->SetPose(Tcw);

        }
    }

    for (size_t i = 0; i < N; i++) {
        KeyFrame *pKFi = OptKFs[i];
        int kf_id = pKFi->mnId;
        if (pKFi->mnId > maxKFid) {
            continue;
        }
        // Pose
        VertexPoseDvlIMU *VP = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
        Eigen::Quaterniond Rwc(VP->estimate().Rwc);
        Eigen::Vector3d twc = VP->estimate().twc;
        Eigen::Isometry3d Twc = Eigen::Isometry3d::Identity();
        Twc.pretranslate(twc);
        Twc.rotate(Rwc);
        Eigen::Isometry3d Tcw = Twc.inverse();
        cv::Mat Tcw_cv;
        cv::eigen2cv(Tcw.matrix(), Tcw_cv);
        Tcw_cv.convertTo(Tcw_cv, CV_32F);
        pKFi->SetPose(Tcw_cv);

        //Bias
        int gyros_bias_vertex_id = kf_id + maxKFid + 1;
        int acc_bias_vertex_id = kf_id + (maxKFid + 1)*2;
        VertexGyroBias *v_gb = dynamic_cast<VertexGyroBias *>(optimizer.vertex(gyros_bias_vertex_id));
        VertexAccBias *v_ab = dynamic_cast<VertexAccBias *>(optimizer.vertex(acc_bias_vertex_id));
        // bg << v_gb->estimate();
        IMU::Bias b(v_ab->estimate().x(), v_ab->estimate().y(), v_ab->estimate().z(),
                    v_gb->estimate().x(), v_gb->estimate().y(), v_gb->estimate().z());
        pKFi->SetNewBias(b);
        ROS_DEBUG_STREAM("KF["<<pKFi->mnId<<"] bias[acc gyros]: "<<v_ab->estimate().transpose()<<" "<<v_gb->estimate().transpose());
        // ss<<"KF["<<pKFi->mnId<<"] bias[acc gyros]: "<<v_ab->estimate().transpose()<<" "<<v_gb->estimate().transpose()<<"\n";

        //recover dvl_velocity of pKFi
        int dvl_vertex_id = kf_id + (maxKFid + 1)*3;
        VertexVelocity *v_dvl = dynamic_cast<VertexVelocity *>(optimizer.vertex(dvl_vertex_id));
        Eigen::Vector3d dvl_velocity;
        pKFi->GetDvlVelocityMeasurement(dvl_velocity);
        ss<<"KF["<<pKFi->mnId<<"] "<<pKFi->mTimeStamp<< "old dvl_velocity: "<<dvl_velocity.transpose()<<"\n"
        <<"KF["<<pKFi->mnId<<"] "<<pKFi->mTimeStamp<< "new dvl_velocity: "<<v_dvl->estimate().transpose()<<"\n";
        dvl_velocity = v_dvl->estimate();
        pKFi->SetDvlVelocity(dvl_velocity);


    }


	ROS_DEBUG_STREAM(ss.str());

    for (int i = 0; i < N_map_points; i++) {
        MapPoint *pMP = LocalMapPoints[i];
        if(find(LocalFixedMapPoints.begin(), LocalFixedMapPoints.end(), pMP) !=
           LocalFixedMapPoints.end()){
            continue;
        }
        g2o::VertexSBAPointXYZ
                *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex((maxKFid + 1) * 5 + i));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }
    pMap->IncreaseChangeIndex();
    // ROS_INFO_STREAM("Map change after BA: "<<pMap->GetMapChangeIndex());
    // ROS_INFO_STREAM("Local BA upto KF done: " << pKF->mnId);
}

void DvlGyroOptimizer::LocalDVLIMUBundleAdjustment2(Atlas* pAtlas, KeyFrame* pKF, bool* pbStopFlag, Map* pMap,
                                                    int &num_fixedKF, double lamda_DVL, double lamda_visual)
{
    Map *pCurrentMap = pKF->GetMap();
    const unsigned long maxKFid = pKF->mnId;

    vector<KeyFrame *> OptKFs;
    OptKFs.reserve(200);
    OptKFs.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    auto local_kfs = pKF->GetVectorCovisibleKeyFrames();
    for(auto pkfi:local_kfs){
        pkfi->mnBALocalForKF = pKF->mnId;
        if (!pkfi->isBad() && pkfi->GetMap() == pCurrentMap)
            OptKFs.push_back(pkfi);
    }

    int N = OptKFs.size();

    vector<KeyFrame *> FixedKFs;
    if (OptKFs.back()->mPrevKF) {
        FixedKFs.push_back(OptKFs.back()->mPrevKF);
        OptKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
    }
    else {
        OptKFs.back()->mnBALocalForKF = 0;
        OptKFs.back()->mnBAFixedForKF = pKF->mnId;
        FixedKFs.push_back(OptKFs.back());
        OptKFs.pop_back();
    }
    // add more Fixed but connected KF
    auto connectedKF = FixedKFs.back()->GetConnectedKeyFrames();
    for(auto pKFi:connectedKF){
        // check whether pKFi is in OptKFS or FixedKFs
        bool inOptKFs = false;
        bool inFixedKFs = false;
        for(auto pOptKF:OptKFs){
            if(pKFi->mnId == pOptKF->mnId){
                inOptKFs = true;
                break;
            }
        }
        for(auto pFixedKF:FixedKFs){
            if(pKFi->mnId == pFixedKF->mnId){
                inFixedKFs = true;
                break;
            }
        }
        if(!inOptKFs&&!inFixedKFs&&FixedKFs.size()<50){
            pKFi->mnBAFixedForKF = pKF->mnId;
            FixedKFs.push_back(pKFi);
        }
    }


    vector<MapPoint *> LocalMapPoints;
    for (int i = 0; i < N; i++) {
        vector<MapPoint *> vpMPs = OptKFs[i]->GetMapPointMatches();
        for (vector<MapPoint *>::iterator it = vpMPs.begin(); it != vpMPs.end(); it++) {
            MapPoint *pMP = *it;
            if (pMP) {
                //				cout<<"find local map point"<<endl;
                if (!pMP->isBad()) {
                    //					cout<<"find local good map point"<<endl;
                    if (pMP->mnBALocalForKF != pKF->mnId) {
                        //						cout<<"find local map point with correct BALocalForKF"<<endl;
                        LocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                        //						cout<<"add local map point to optimize: "<<endl;
                    }
                }
            }
        }
    }
    vector<MapPoint *> LocalFixedMapPoints;
    for(auto pKFi:FixedKFs){
        auto Mps = pKFi->GetMapPointMatches();
        for(auto pMP:Mps){
            if(!pMP||pMP->isBad()){
                continue;
            }
            if (pMP->mnBALocalForKF != pKF->mnId) {
                //						cout<<"find local map point with correct BALocalForKF"<<endl;
                LocalFixedMapPoints.push_back(pMP);
                LocalMapPoints.push_back(pMP);
                pMP->mnBALocalForKF = pKF->mnId;
                //						cout<<"add local map point to optimize: "<<endl;
            }
        }
    }
    int N_map_points = LocalMapPoints.size();
    //	cout << "map point to optimize: " << N_map_points << endl;

    // const int maxFixedKF = 30;
    // for (vector<MapPoint *>::iterator it = LocalMapPoints.begin(); it != LocalMapPoints.end(); it++) {
    //     map<KeyFrame *, tuple<int, int>> observations = (*it)->GetObservations();
    //     for (map<KeyFrame *, tuple<int, int>>::iterator it_ob = observations.begin(); it_ob != observations.end();
    //          it_ob++) {
    //         KeyFrame *pKFi = it_ob->first;
    //         if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
    //             pKFi->mnBAFixedForKF = pKF->mnId;
    //             if (!pKFi->isBad()) {
    //                 FixedKFs.push_back(pKFi);
    //                 break;
    //             }
    //         }
    //     }
    //     if (FixedKFs.size() >= maxFixedKF) {
    //         break;
    //     }
    // }
    int N_fixed = FixedKFs.size();


    Verbose::PrintMess("DVL Gyro optimization", Verbose::VERBOSITY_NORMAL);
    int its = 200; // Check number of iterations
    //	const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    optimizer.setAlgorithm(solver);

    // Set KeyFrame vertices (fixed poses and optimizable velocities)
    // record pose before optimize
    std::map<VertexPoseDvlIMU*, Eigen::Isometry3d> map_pose_original;
    for (size_t i = 0; i < OptKFs.size(); i++) {
        KeyFrame *pKFi = OptKFs[i];
        if (pKFi->mnId > maxKFid) {
            continue;
        }
        VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);
        Eigen::Isometry3d T_c0_cj = Eigen::Isometry3d::Identity();
        T_c0_cj.rotate(VP->estimate().Rwc);
        T_c0_cj.pretranslate(VP->estimate().twc);
        map_pose_original.insert(std::pair<VertexPoseDvlIMU*, Eigen::Isometry3d>(VP, T_c0_cj));
        // ROS_INFO_STREAM("opt KF: "<<pKFi->mnId);

        // DVLGroPreIntegration *pDVLGroPreIntegration2 = new DVLGroPreIntegration();
        // boost::archive::text_iarchive ia1(i_file1);
        // ia1 >> pDVLGroPreIntegration2;
        // oa2 << pDVLGroPreIntegration2;
        // o_file2.close();

    }
    for (int i = 0; i < FixedKFs.size(); i++) {
        KeyFrame *pKFi = FixedKFs[i];
        if (pKFi->mnId > maxKFid) {
            continue;
        }
        VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);
        // ROS_INFO_STREAM("fixed KF: "<<pKFi->mnId);
    }

    // Biases
    vector<VertexGyroBias*> vpgb;
    vector<VertexAccBias*> vpab;
    //todo_tightly
    //	set fixed for debuging
    for(auto pKFi:OptKFs){
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

        VertexVelocity *VV = new VertexVelocity(pKFi);
        VV->setId((maxKFid + 1)*3 + pKFi->mnId);
        VV->setFixed(true);
        optimizer.addVertex(VV);
    }
    for(auto pKFi:FixedKFs){
        VertexGyroBias *VG = new VertexGyroBias(pKFi);
        VG->setId(maxKFid + 1 + pKFi->mnId);
        VG->setFixed(true);
        optimizer.addVertex(VG);
        vpgb.push_back(VG);

        VertexAccBias *VA = new VertexAccBias(pKFi);
        VA->setId((maxKFid + 1)*2 + pKFi->mnId);
        VA->setFixed(true);
        optimizer.addVertex(VA);
        // vpab.push_back(VA);

        VertexVelocity *VV = new VertexVelocity(pKFi);
        VV->setId((maxKFid + 1)*3 + pKFi->mnId);
        VV->setFixed(true);
        optimizer.addVertex(VV);

    }

    // extrinsic parameter
    g2o::VertexSE3Expmap *vT_d_c = new g2o::VertexSE3Expmap();
    vT_d_c->setEstimate(Converter::toSE3Quat(pKF->mImuCalib.mT_dvl_c));
    vT_d_c->setId((maxKFid + 1)*4);
    vT_d_c->setFixed(true);
    optimizer.addVertex(vT_d_c);

    g2o::VertexSE3Expmap *vT_g_d = new g2o::VertexSE3Expmap();
    vT_g_d->setEstimate(Converter::toSE3Quat(pKF->mImuCalib.mT_gyro_dvl));
    vT_g_d->setId((maxKFid + 1)*4+1);
    vT_g_d->setFixed(true);
    optimizer.addVertex(vT_g_d);

    VertexGDir *VGDir = new VertexGDir(pAtlas->getRGravity());
    VGDir->setId((maxKFid + 1)*4+2);
    VGDir->setFixed(true);
    optimizer.addVertex(VGDir);

    vector<EdgeMonoBA_DvlGyros *> mono_edges;
    vector<EdgeStereoBA_DvlGyros *> stereo_edges;
    std::map<g2o::VertexSBAPointXYZ*, VertexPoseDvlIMU*> map_point_observation;
    std::set<g2o::VertexSBAPointXYZ*> vertex_point;
    std::map<EdgeMonoBA_DvlGyros*,std::pair<KeyFrame*,MapPoint*>> mono_obs;
    std::map<EdgeStereoBA_DvlGyros*,std::pair<KeyFrame*,MapPoint*>> stereo_obs;
    //add map point vertex and visual constrain
    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for (int i = 0; i < N_map_points; i++) {
            MapPoint *pMP = LocalMapPoints[i];
            if (pMP) {
                g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
                vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
                int id = pMP->mnId + maxKFid + 1;
                vPoint->setId((maxKFid + 1)*5 + i);
                vPoint->setMarginalized(true);
                //check whther pMP is in LocalFixedMapPoints
                if (find(LocalFixedMapPoints.begin(), LocalFixedMapPoints.end(), pMP) != LocalFixedMapPoints.end()) {
                    vPoint->setFixed(true);
                } else {
                    vPoint->setFixed(false);
                }
                optimizer.addVertex(vPoint);

                const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

                for (auto ob: observations) {
                    KeyFrame *pKFi = ob.first;
                    if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
                        continue;
                    }

                    if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
                        std::pair<KeyFrame*,MapPoint*> o = std::make_pair(pKFi,pMP);
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
                            e->setId(optimizer.edges().size());
                            e->setMeasurement(obs);
                            const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                            VertexPoseDvlIMU* v1 = dynamic_cast<VertexPoseDvlIMU*>(e->vertices()[0]);
                            if(v1->estimate().mPoorVision){
                                e->setInformation(Eigen::Matrix<double, 2, 2>::Identity()* invSigma2 * lamda_visual * 1);
                            }
                            else{
                                e->setInformation(Eigen::Matrix<double, 2, 2>::Identity()* invSigma2 * lamda_visual);
                            }
                            // e->setInformation(Eigen::Matrix2d::Identity() * invSigma2* lamda_visual);

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(sqrt(5.991));

                            mono_edges.push_back(e);
                            optimizer.addEdge(e);

                            mono_obs.insert(std::make_pair(e,o));

                            auto all_verteices = e->vertices();
                            if (VertexPoseDvlIMU* v_pose = dynamic_cast<VertexPoseDvlIMU*>(all_verteices[0])) {
                                if (g2o::VertexSBAPointXYZ* v_point = dynamic_cast<g2o::VertexSBAPointXYZ*>(all_verteices[1])) {
                                    vertex_point.insert(v_point);
                                    if (map_point_observation.find(v_point) == map_point_observation.end()) {
                                        map_point_observation.insert(
                                                std::pair<g2o::VertexSBAPointXYZ*, VertexPoseDvlIMU*>(v_point, v_pose));
                                    }
                                }
                            }
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
                            e->setId(optimizer.edges().size());
                            e->setMeasurement(obs);
                            const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2 * lamda_visual;
                            VertexPoseDvlIMU* v1 = dynamic_cast<VertexPoseDvlIMU*>(e->vertices()[0]);
                            if(v1->estimate().mPoorVision){
                                e->setInformation(Info  * 1);
                            }
                            else{
                                e->setInformation(Info );
                            }

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(sqrt(7.815));

                            stereo_edges.push_back(e);
                            optimizer.addEdge(e);
                            stereo_obs.insert(std::make_pair(e,o));

                            auto all_verteices = e->vertices();
                            if (VertexPoseDvlIMU* v_pose = dynamic_cast<VertexPoseDvlIMU*>(all_verteices[0])) {
                                if (g2o::VertexSBAPointXYZ* v_point = dynamic_cast<g2o::VertexSBAPointXYZ*>(all_verteices[1])) {
                                    vertex_point.insert(v_point);
                                    if (map_point_observation.find(v_point) == map_point_observation.end()) {
                                        map_point_observation.insert(
                                                std::pair<g2o::VertexSBAPointXYZ*, VertexPoseDvlIMU*>(v_point, v_pose));
                                    }
                                }
                            }
                        }

                    }

                }
            }
        }
    }
    ROS_INFO_STREAM("visual edge size: "<<(mono_edges.size()+stereo_edges.size()));


    // Graph edges
    vector<EdgeDvlIMU *> dvlimu_edges;
    vector<EdgeSE3DVLIMU *> se3_edges;
    vector<EdgePriorAcc *> acc_edge;
    vector<EdgePriorGyro *> gyro_edge;
    vector<EdgeDvlVelocity *> velocity_edge;
    dvlimu_edges.reserve(OptKFs.size());
    vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
    //	vppUsedKF.reserve(OptKFs.size() + FixedKFs.size());
    //	std::cout << "build optimization graph" << std::endl;

    for (size_t i = 0; i < OptKFs.size(); i++) {
        KeyFrame *pKFi = OptKFs[i];

        if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
            if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) {
                continue;
            }

            // VertexPoseDvlGro *VP1 = dynamic_cast<VertexPoseDvlGro *>(optimizer.vertex(pKFi->mPrevKF->mnId));
            //				g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
            // VertexPoseDvlGro *VP2 = dynamic_cast<VertexPoseDvlGro *>(optimizer.vertex(pKFi->mnId));
            //				g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
            // g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid + 1 + i);
            // g2o::HyperGraph::Vertex *VT_d_c = optimizer.vertex(maxKFid + N + N_fixed + 1);
            // g2o::HyperGraph::Vertex *VT_g_d = optimizer.vertex(maxKFid + N + N_fixed + 2);

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

            if (!VP1|| !VP2 || !VV1 || !VV2 || !VG1 || !VG2 || !VA1 || !VA2 || !VT_d_c || !VT_g_d || !VR_b0_w) {
                ROS_ERROR_STREAM("LocalVAIBA Error");
                return;
            }
            EdgeAccRW* e_bias = new EdgeAccRW();
            e_bias->setVertex(0,VA1);
            e_bias->setVertex(1,VA2);
            e_bias->setInformation(Eigen::Matrix3d::Identity() * 1e6*(mono_edges.size()+stereo_edges.size()) * lamda_DVL);
            optimizer.addEdge(e_bias);
            //velocity edge
            Eigen::Vector3d dvl_v1;
            pKFi->mPrevKF->GetDvlVelocityMeasurement(dvl_v1);
            Eigen::Vector3d dvl_v2;
            pKFi->GetDvlVelocityMeasurement(dvl_v2);
            EdgeDvlVelocity *ev = new EdgeDvlVelocity(dvl_v1);
            ev->setLevel(0);
            ev->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
            ev->setId(optimizer.edges().size());
            auto vv1 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1);
            if(pKFi->mPrevKF->mbDVL){
                vv1->setFixed(false);
                ev->setInformation(Eigen::Matrix3d::Identity()*1e10);
            }
            else{
                vv1->setFixed(false);
                ev->setInformation(Eigen::Matrix3d::Identity()*1e5);
            }
            optimizer.addEdge(ev);
            velocity_edge.push_back(ev);
            // add edge for v2
            EdgeDvlVelocity *ev2 = new EdgeDvlVelocity(dvl_v2);
            ev2->setLevel(0);
            ev2->setId(optimizer.edges().size());
            ev2->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
            auto vv2 = dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2);
            if (pKFi->mbDVL) {
                vv2->setFixed(false);
                ev2->setInformation(Eigen::Matrix3d::Identity()*1e10);
            }
            else {
                vv2->setFixed(false);
                ev2->setInformation(Eigen::Matrix3d::Identity() *1e5);
            }
            optimizer.addEdge(ev2);
            velocity_edge.push_back(ev2);
            //				EdgeInertialGS *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
            // EdgeDvlGyroBA *ei = new EdgeDvlGyroBA(pKFi->mpDvlPreintegrationKeyFrame);
            // ei->setLevel(1);
            // ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
            // ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
            // ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
            // ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
            // ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
            // ei->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * lamda_DVL * (stereo_edges.size()+mono_edges.size()));
            // ei->setId(pKFi->mnId);
            // // dvlimu_edges.push_back(ei);
            // optimizer.addEdge(ei);

            EdgeDvlGyroBA *ei = new EdgeDvlGyroBA(pKFi->mpDvlPreintegrationKeyFrame);
            ei->setLevel(0);
            ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
            ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
            ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG1));
            ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
            ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
            ei->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * lamda_DVL * (stereo_edges.size()+mono_edges.size()));
            ei->setId(pKFi->mnId);
            VertexPoseDvlIMU* v1 = dynamic_cast<VertexPoseDvlIMU*>(ei->vertices()[0]);
            VertexPoseDvlIMU* v2 = dynamic_cast<VertexPoseDvlIMU*>(ei->vertices()[1]);
            Eigen::Matrix<double,6,6> info = Eigen::Matrix<double,6,6>::Identity();
            if(v1->estimate().mPoorVision||v2->estimate().mPoorVision){
                info.block<3,3>(0,0) = info.block<3,3>(0,0) * 1e4;
                info.block<3,3>(3,3) = info.block<3,3>(3,3) * 1e2;
                ei->setInformation(info*lamda_DVL*(mono_edges.size()+stereo_edges.size()));
            }
            else{
                info.block<3,3>(0,0) = info.block<3,3>(0,0) * 1e2;
                info.block<3,3>(3,3) = info.block<3,3>(3,3) * 1;
                ei->setInformation(info* lamda_DVL * (mono_edges.size()+stereo_edges.size()));
            }
            // dvlimu_edges.push_back(ei);
            optimizer.addEdge(ei);

            // EdgeDvlIMU *eG = new EdgeDvlIMU(pKFi->mpDvlPreintegrationKeyFrame);
            // eG->setLevel(1);
            // eG->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
            // eG->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
            // eG->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
            // eG->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
            // eG->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG2));
            // eG->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA2));
            // eG->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
            // eG->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
            // eG->setVertex(8, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VR_b0_w));
            // eG->setId(optimizer.edges().size());
            // Eigen::Matrix<double,9,9> info=Eigen::Matrix<double,9,9>::Identity();
            // // info(0,0) = info(0,0)*500;
            // if(v1->estimate().mPoorVision||v2->estimate().mPoorVision){
            //     info.block(0,0,3,3) = Eigen::Matrix3d::Identity()*100* lamda_DVL;
            //     // info(0,0) =info(0,0) * 5e2 * lamda_DVL; // 10_24
            //     info(1,1) =info(1,1) * 1e2 * lamda_DVL; // before 10_24
            //     info.block(3,3,3,3) = Eigen::Matrix3d::Identity()*100* lamda_DVL;
            //     info.block(6,6,3,3) = Eigen::Matrix3d::Identity()*10 * lamda_DVL;
            //     eG->setInformation(info*(mono_edges.size()+stereo_edges.size()));
            // }
            // else{
            //     info.block(0,0,3,3) = Eigen::Matrix3d::Identity() * lamda_DVL;
            //     // info(0,0) = info(0,0)*lamda_DVL * 5e3; // 10_24
            //     info(1,1) = info(1,1)*lamda_DVL * 5e3; // before 10_24
            //     info.block(3,3,3,3) = Eigen::Matrix3d::Identity()*lamda_DVL*1;
            //     info.block(6,6,3,3) = Eigen::Matrix3d::Identity()*lamda_DVL*1;
            //     eG->setInformation(info  * (mono_edges.size()+stereo_edges.size()));
            // }
            // // eG->setId(maxKFid+1 + pKFi->mnId);
            // dvlimu_edges.push_back(eG);
            // optimizer.addEdge(eG);

            EdgeDvlIMUGravityRefine *eG1 = new EdgeDvlIMUGravityRefine(pKFi->mpDvlPreintegrationKeyFrame);
            eG1->setLevel(0);
            eG1->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
            eG1->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
            eG1->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
            eG1->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
            eG1->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG1));
            eG1->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA1));
            eG1->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
            eG1->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
            eG1->setVertex(8, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VR_b0_w));
            Eigen::Matrix<double,3,3> info_G = Eigen::Matrix<double,3,3>::Identity();
            if(v1->estimate().mPoorVision||v2->estimate().mPoorVision){
                info_G = info_G * 1e4 * lamda_DVL;
                eG1->setInformation(info_G*(mono_edges.size()+stereo_edges.size()));
            }
            else{
                info_G = info_G * lamda_DVL;
                eG1->setInformation(info_G  * (mono_edges.size()+stereo_edges.size()));
            }
            optimizer.addEdge(eG1);




        }
    }

    optimizer.initializeOptimization(0);
    optimizer.optimize(20);
    if(pbStopFlag){
        optimizer.setForceStopFlag(pbStopFlag);
    }
    stringstream ss_v_chi2;
    std::set<std::pair<KeyFrame*,MapPoint*>> remove_obs;
    ss_v_chi2<<"mono chi2: ";
    for (auto e: mono_edges) {
        e->computeError();
        ss_v_chi2<<e->chi2()<<", ";
        // e->setInformation(Eigen::Matrix2d::Identity() * lamda_visual);
        if (e->chi2() > 1) {
            e->setLevel(1);
            remove_obs.insert(mono_obs[e]);
        }
        else {
            e->setLevel(0);
            // e->setInformation(Eigen::Matrix2d::Identity() * 1);
        }

    }
    ss_v_chi2<<"\n";
    ss_v_chi2<<"stereo chi2: ";
    for (auto e: stereo_edges) {
        e->computeError();
        ss_v_chi2<<e->chi2()<<", ";
        // e->setInformation(Eigen::Matrix3d::Identity() * lamda_visual);
        if (e->chi2() > 2) {
            e->setLevel(1);
            remove_obs.insert(stereo_obs[e]);
        }
        else {
            e->setLevel(0);
        }
    }
    // ROS_INFO_STREAM(ss_v_chi2.str());
    for(int i=0;i<4;i++){
        if(pbStopFlag){
            if(*pbStopFlag){
                ROS_INFO_STREAM("stop BA");
                break;
            }
        }
        optimizer.initializeOptimization(0);
        optimizer.optimize(4);
    }



    // Recover optimized data
    stringstream ss;
    ss<<"LocalVisualAcousticInertial BA"<<"\n";
    unique_lock<shared_timed_mutex> lock(pMap->mMutexMapUpdate);
    for(auto o:remove_obs){
        KeyFrame* pKFi = o.first;
        MapPoint* pMpi = o.second;
        pKFi->EraseMapPointMatch(pMpi);
        pMpi->EraseObservation(pKFi);
        ss<<"remove map point, KF: "<<pKFi->mnId<<", MP: "<<pMpi->mnId<<"\n";
    }
    for (size_t i = 0; i < N; i++) {
        KeyFrame *pKFi = OptKFs[i];
        int kf_id = pKFi->mnId;
        if (pKFi->mnId > maxKFid) {
            continue;
        }
        // Pose
        VertexPoseDvlIMU *VP = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
        Eigen::Quaterniond Rwc(VP->estimate().Rwc);
        Eigen::Vector3d twc = VP->estimate().twc;
        Eigen::Isometry3d Twc = Eigen::Isometry3d::Identity();
        Twc.pretranslate(twc);
        Twc.rotate(Rwc);
        Eigen::Isometry3d Tcw = Twc.inverse();
        cv::Mat Tcw_cv;
        cv::eigen2cv(Tcw.matrix(), Tcw_cv);
        Tcw_cv.convertTo(Tcw_cv, CV_32F);
        pKFi->SetPose(Tcw_cv);

        //Bias
        int gyros_bias_vertex_id = kf_id + maxKFid + 1;
        int acc_bias_vertex_id = kf_id + (maxKFid + 1)*2;
        VertexGyroBias *v_gb = dynamic_cast<VertexGyroBias *>(optimizer.vertex(gyros_bias_vertex_id));
        VertexAccBias *v_ab = dynamic_cast<VertexAccBias *>(optimizer.vertex(acc_bias_vertex_id));
        // bg << v_gb->estimate();
        IMU::Bias b(v_ab->estimate().x(), v_ab->estimate().y(), v_ab->estimate().z(),
                    v_gb->estimate().x(), v_gb->estimate().y(), v_gb->estimate().z());
        pKFi->SetNewBias(b);
//        ROS_INFO_STREAM("KF["<<pKFi->mnId<<"] bias[acc gyros]: "<<v_ab->estimate().transpose()<<" "<<v_gb->estimate().transpose());
        // ss<<"KF["<<pKFi->mnId<<"] bias[acc gyros]: "<<v_ab->estimate().transpose()<<" "<<v_gb->estimate().transpose()<<"\n";

        //recover dvl_velocity of pKFi
        int dvl_vertex_id = kf_id + (maxKFid + 1)*3;
        VertexVelocity *v_dvl = dynamic_cast<VertexVelocity *>(optimizer.vertex(dvl_vertex_id));
        Eigen::Vector3d dvl_velocity;
        pKFi->GetDvlVelocity(dvl_velocity);
        ss<<"KF["<<pKFi->mnId<<"] "<<pKFi->mTimeStamp<< "old dvl_velocity: "<<dvl_velocity.transpose()<<"\n"
          <<"KF["<<pKFi->mnId<<"] "<<pKFi->mTimeStamp<< "new dvl_velocity: "<<v_dvl->estimate().transpose()<<"\n";
        dvl_velocity = v_dvl->estimate();
        pKFi->SetDvlVelocity(dvl_velocity);


    }
    ROS_DEBUG_STREAM(ss.str());

    for (int i = 0; i < N_map_points; i++) {
        MapPoint *pMP = LocalMapPoints[i];
        if(find(LocalFixedMapPoints.begin(), LocalFixedMapPoints.end(), pMP) !=
           LocalFixedMapPoints.end()){
            continue;
        }
        g2o::VertexSBAPointXYZ
                *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex((maxKFid + 1) * 5 + i));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }
    pMap->IncreaseChangeIndex();
    // ROS_INFO_STREAM("Map change after BA: "<<pMap->GetMapChangeIndex());
}

void DvlGyroOptimizer::FullDVLIMUBundleAdjustment(Atlas* pAtlas, KeyFrame* pKF, bool* pbStopFlag, Map* pMap,
                                                  const int &num_fixedKF, double lamda_DVL, double lamda_visual)

{
    Map *pCurrentMap = pKF->GetMap();
    int Nd = pCurrentMap->KeyFramesInMap();// number of keyframes in current map
    const unsigned long maxKFid = pKF->mnId;

    vector<KeyFrame *> OptKFs;
    OptKFs.reserve(Nd);
    OptKFs.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    while (OptKFs.back()->mPrevKF) {
        OptKFs.push_back(OptKFs.back()->mPrevKF);
        OptKFs.back()->mnBALocalForKF = pKF->mnId;
    }
    int N = OptKFs.size();

    vector<KeyFrame *> FixedKFs;
    if (OptKFs.back()->mPrevKF) {
        FixedKFs.push_back(OptKFs.back()->mPrevKF);
        OptKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
    }
    else {
        OptKFs.back()->mnBALocalForKF = 0;
        OptKFs.back()->mnBAFixedForKF = pKF->mnId;
        FixedKFs.push_back(OptKFs.back());
        OptKFs.pop_back();
    }
    // add more Fixed but connected KF
    auto connectedKF = FixedKFs.back()->GetConnectedKeyFrames();
    for(auto pKFi:connectedKF){
        // check whether pKFi is in OptKFS or FixedKFs
        bool inOptKFs = false;
        bool inFixedKFs = false;
        for(auto pOptKF:OptKFs){
            if(pKFi->mnId == pOptKF->mnId){
                inOptKFs = true;
                break;
            }
        }
        for(auto pFixedKF:FixedKFs){
            if(pKFi->mnId == pFixedKF->mnId){
                inFixedKFs = true;
                break;
            }
        }
        if(!inOptKFs&&!inFixedKFs&&FixedKFs.size()<50){
            pKFi->mnBAFixedForKF = pKF->mnId;
            FixedKFs.push_back(pKFi);
        }
    }


    set<MapPoint *> LocalMapPoints;
    for (int i = 0; i < N; i++) {
        vector<MapPoint *> vpMPs = OptKFs[i]->GetMapPointMatches();
        for (vector<MapPoint *>::iterator it = vpMPs.begin(); it != vpMPs.end(); it++) {
            MapPoint *pMP = *it;
            if (pMP) {
                //				cout<<"find local map point"<<endl;
                if (!pMP->isBad()) {
                    //					cout<<"find local good map point"<<endl;
                    if(LocalMapPoints.count(pMP)==0){
                        LocalMapPoints.insert(pMP);
                    }
                }
            }
        }
    }
    vector<MapPoint *> LocalFixedMapPoints;
    for(auto pKFi:FixedKFs){
        auto Mps = pKFi->GetMapPointMatches();
        for(auto pMP:Mps){
            if(!pMP||pMP->isBad()){
                continue;
            }
            if(LocalMapPoints.count(pMP)==0){
                LocalFixedMapPoints.push_back(pMP);
            }
        }
    }
    int N_map_points = LocalMapPoints.size();
    ROS_INFO_STREAM("map point to optimize: "<<N_map_points);
    //	cout << "map point to optimize: " << N_map_points << endl;

    // const int maxFixedKF = 30;
    // for (vector<MapPoint *>::iterator it = LocalMapPoints.begin(); it != LocalMapPoints.end(); it++) {
    //     map<KeyFrame *, tuple<int, int>> observations = (*it)->GetObservations();
    //     for (map<KeyFrame *, tuple<int, int>>::iterator it_ob = observations.begin(); it_ob != observations.end();
    //          it_ob++) {
    //         KeyFrame *pKFi = it_ob->first;
    //         if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
    //             pKFi->mnBAFixedForKF = pKF->mnId;
    //             if (!pKFi->isBad()) {
    //                 FixedKFs.push_back(pKFi);
    //                 break;
    //             }
    //         }
    //     }
    //     if (FixedKFs.size() >= maxFixedKF) {
    //         break;
    //     }
    // }
    int N_fixed = FixedKFs.size();


    Verbose::PrintMess("DVL Gyro optimization", Verbose::VERBOSITY_NORMAL);
    int its = 200; // Check number of iterations
    //	const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    optimizer.setAlgorithm(solver);


    for (size_t i = 0; i < OptKFs.size(); i++) {
        KeyFrame *pKFi = OptKFs[i];
        if (pKFi->mnId > maxKFid) {
            continue;
        }
        VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);
        // ROS_INFO_STREAM("opt KF: "<<pKFi->mnId);

        // DVLGroPreIntegration *pDVLGroPreIntegration2 = new DVLGroPreIntegration();
        // boost::archive::text_iarchive ia1(i_file1);
        // ia1 >> pDVLGroPreIntegration2;
        // oa2 << pDVLGroPreIntegration2;
        // o_file2.close();

    }
    for (int i = 0; i < FixedKFs.size(); i++) {
        KeyFrame *pKFi = FixedKFs[i];
        if (pKFi->mnId > maxKFid) {
            continue;
        }
        VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);
        ROS_INFO_STREAM("fixed KF: "<<pKFi->mnId);
    }

    // Biases
    vector<VertexGyroBias*> vpgb;
    vector<VertexAccBias*> vpab;
    //todo_tightly
    //	set fixed for debuging
    // Eigen::Vector3d gyros_b(-0.000535183,-0.00224689,0.000705318);//halftank_easy
    // Eigen::Vector3d gyros_b(-0.000535183,-0.00194689,0.000705318);//halftank_medium
    // Eigen::Vector3d gyros_b(0.00317551, -0.00854424, -0.000787445);//Structure_Hard
    // Eigen::Vector3d gyros_b(0.00247442, -0.002438478, 0.00188971);//Structure_Medium
    // Eigen::Vector3d gyros_b(0.00247442, -0.000638478, 0.00188971);//Halftank_Hard
    // Eigen::Vector3d gyros_b(-0.000653851, -0.00174641, 0.0020714);//wholetank_hard
    Eigen::Vector3d gyros_b(0, 0,  0);//halftank_medium

    for(auto pKFi:OptKFs){
        VertexGyroBias *VG = new VertexGyroBias(pKFi);
        // VertexGyroBias *VG = new VertexGyroBias(gyros_b);
        VG->setId(maxKFid + 1 + pKFi->mnId);
        VG->setFixed(false);
        optimizer.addVertex(VG);
        vpgb.push_back(VG);

        VertexAccBias *VA = new VertexAccBias(pKFi);
        VA->setId((maxKFid + 1)*2 + pKFi->mnId);
        VA->setFixed(false);
        optimizer.addVertex(VA);
        vpab.push_back(VA);

        VertexVelocity *VV = new VertexVelocity(pKFi);
        VV->setId((maxKFid + 1)*3 + pKFi->mnId);
        VV->setFixed(false);
        optimizer.addVertex(VV);
    }
    for(auto pKFi:FixedKFs){
        VertexGyroBias *VG = new VertexGyroBias(pKFi);
        // VertexGyroBias *VG = new VertexGyroBias(gyros_b);
        VG->setId(maxKFid + 1 + pKFi->mnId);
        VG->setFixed(true);
        optimizer.addVertex(VG);
        vpgb.push_back(VG);

        VertexAccBias *VA = new VertexAccBias(pKFi);
        VA->setId((maxKFid + 1)*2 + pKFi->mnId);
        VA->setFixed(true);
        optimizer.addVertex(VA);
        // vpab.push_back(VA);

        VertexVelocity *VV = new VertexVelocity(pKFi);
        VV->setId((maxKFid + 1)*3 + pKFi->mnId);
        VV->setFixed(true);
        optimizer.addVertex(VV);

    }

    // extrinsic parameter
    g2o::VertexSE3Expmap *vT_d_c = new g2o::VertexSE3Expmap();
    vT_d_c->setEstimate(Converter::toSE3Quat(pKF->mImuCalib.mT_dvl_c));
    vT_d_c->setId((maxKFid + 1)*4);
    vT_d_c->setFixed(true);
    optimizer.addVertex(vT_d_c);

    g2o::VertexSE3Expmap *vT_g_d = new g2o::VertexSE3Expmap();
    vT_g_d->setEstimate(Converter::toSE3Quat(pKF->mImuCalib.mT_gyro_dvl));
    vT_g_d->setId((maxKFid + 1)*4+1);
    vT_g_d->setFixed(true);
    optimizer.addVertex(vT_g_d);

    VertexGDir *VGDir = new VertexGDir(pAtlas->getRGravity());
    VGDir->setId((maxKFid + 1)*4+2);
    VGDir->setFixed(true);
    optimizer.addVertex(VGDir);

    vector<EdgeMonoBA_DvlGyros *> mono_edges;
    vector<EdgeStereoBA_DvlGyros *> stereo_edges;
    std::map<g2o::VertexSBAPointXYZ*, VertexPoseDvlIMU*> map_point_observation;
    std::set<g2o::VertexSBAPointXYZ*> vertex_point;
    std::map<EdgeMonoBA_DvlGyros*,std::pair<KeyFrame*,MapPoint*>> mono_obs;
    std::map<EdgeStereoBA_DvlGyros*,std::pair<KeyFrame*,MapPoint*>> stereo_obs;
    //add map point vertex and visual constrain
    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for (auto it:LocalMapPoints) {
            MapPoint *pMP = it;
            if (pMP) {
                g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
                vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
                int id = pMP->mnId + maxKFid + 1;
                vPoint->setId((maxKFid + 1)*5 + pMP->mnId);
                vPoint->setMarginalized(true);
                //check whther pMP is in LocalFixedMapPoints
                if (find(LocalFixedMapPoints.begin(), LocalFixedMapPoints.end(), pMP) != LocalFixedMapPoints.end()) {
                    vPoint->setFixed(true);
                } else {
                    vPoint->setFixed(true);
                }
                optimizer.addVertex(vPoint);

                const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

                for (auto ob: observations) {
                    KeyFrame *pKFi = ob.first;

                    if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
                        std::pair<KeyFrame*,MapPoint*> o = std::make_pair(pKFi,pMP);
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
                            e->setId(optimizer.edges().size());
                            e->setMeasurement(obs);
                            const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                            VertexPoseDvlIMU* v1 = dynamic_cast<VertexPoseDvlIMU*>(e->vertices()[0]);
                            if(v1->estimate().mPoorVision){
                                e->setInformation(Eigen::Matrix<double, 2, 2>::Identity()* invSigma2 * lamda_visual * 1);
                            }
                            else{
                                e->setInformation(Eigen::Matrix<double, 2, 2>::Identity()* invSigma2 * lamda_visual);
                            }
                            // e->setInformation(Eigen::Matrix2d::Identity() * invSigma2* lamda_visual);

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(5);

                            mono_edges.push_back(e);
                            optimizer.addEdge(e);

                            mono_obs.insert(std::make_pair(e,o));

                            auto all_verteices = e->vertices();
                            if (VertexPoseDvlIMU* v_pose = dynamic_cast<VertexPoseDvlIMU*>(all_verteices[0])) {
                                if (g2o::VertexSBAPointXYZ* v_point = dynamic_cast<g2o::VertexSBAPointXYZ*>(all_verteices[1])) {
                                    vertex_point.insert(v_point);
                                    if (map_point_observation.find(v_point) == map_point_observation.end()) {
                                        map_point_observation.insert(
                                                std::pair<g2o::VertexSBAPointXYZ*, VertexPoseDvlIMU*>(v_point, v_pose));
                                    }
                                }
                            }
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
                            e->setId(optimizer.edges().size());
                            e->setMeasurement(obs);
                            const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2 * lamda_visual;
                            VertexPoseDvlIMU* v1 = dynamic_cast<VertexPoseDvlIMU*>(e->vertices()[0]);
                            if(v1->estimate().mPoorVision){
                                e->setInformation(Info  * 1);
                            }
                            else{
                                e->setInformation(Info );
                            }

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(10);

                            stereo_edges.push_back(e);
                            optimizer.addEdge(e);
                            stereo_obs.insert(std::make_pair(e,o));

                            auto all_verteices = e->vertices();
                            if (VertexPoseDvlIMU* v_pose = dynamic_cast<VertexPoseDvlIMU*>(all_verteices[0])) {
                                if (g2o::VertexSBAPointXYZ* v_point = dynamic_cast<g2o::VertexSBAPointXYZ*>(all_verteices[1])) {
                                    vertex_point.insert(v_point);
                                    if (map_point_observation.find(v_point) == map_point_observation.end()) {
                                        map_point_observation.insert(
                                                std::pair<g2o::VertexSBAPointXYZ*, VertexPoseDvlIMU*>(v_point, v_pose));
                                    }
                                }
                            }
                        }

                    }

                }
            }
        }
    }
    ROS_INFO_STREAM("visual edge size: "<<(mono_edges.size()+stereo_edges.size()));


    // Graph edges
    vector<EdgeDvlIMU *> dvlimu_edges;
    vector<EdgeSE3DVLIMU *> se3_edges;
    vector<EdgePriorAcc *> acc_edge;
    vector<EdgePriorGyro *> gyro_edge;
    vector<EdgeDvlVelocity *> velocity_edge;
    dvlimu_edges.reserve(OptKFs.size());
    vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
    //	vppUsedKF.reserve(OptKFs.size() + FixedKFs.size());
    //	std::cout << "build optimization graph" << std::endl;
    sort(OptKFs.begin(),OptKFs.end(),KFComparator());
    for (size_t i = 0; i < OptKFs.size(); i++) {
        KeyFrame *pKFi = OptKFs[i];

        if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
            if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) {
                continue;
            }

            // VertexPoseDvlGro *VP1 = dynamic_cast<VertexPoseDvlGro *>(optimizer.vertex(pKFi->mPrevKF->mnId));
            //				g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
            // VertexPoseDvlGro *VP2 = dynamic_cast<VertexPoseDvlGro *>(optimizer.vertex(pKFi->mnId));
            //				g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
            // g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid + 1 + i);
            // g2o::HyperGraph::Vertex *VT_d_c = optimizer.vertex(maxKFid + N + N_fixed + 1);
            // g2o::HyperGraph::Vertex *VT_g_d = optimizer.vertex(maxKFid + N + N_fixed + 2);

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

            if (!VP1|| !VP2 || !VV1 || !VV2 || !VG1 || !VG2 || !VA1 || !VA2 || !VT_d_c || !VT_g_d || !VR_b0_w) {
                ROS_ERROR_STREAM("LocalVAIBA Error");
                assert(0);
            }
            EdgeAccRW* e_bias = new EdgeAccRW();
            e_bias->setLevel(0);
            e_bias->setVertex(0,VA1);
            e_bias->setVertex(1,VA2);
            Eigen::Matrix3d info_acc_bias = Eigen::Matrix3d::Identity()*1e10;
            if(pAtlas->IsIMUCalibrated()){
                cv::Mat cvInfoA = pKFi->mpDvlPreintegrationKeyFrame->C.rowRange(12,15).colRange(12,15).inv(cv::DECOMP_SVD);
                cv::cv2eigen(cvInfoA,info_acc_bias);
            }
            e_bias->setInformation(info_acc_bias);
            // if(info_acc_bias(0,0)==0)
            ROS_DEBUG_STREAM("KF["<<pKFi->mnId<<"] acc bias info:\n"<<info_acc_bias);
            optimizer.addEdge(e_bias);

            EdgeGyroRW* eg_bias = new EdgeGyroRW();
            eg_bias->setLevel(0);
            eg_bias->setVertex(0,VG1);
            eg_bias->setVertex(1,VG2);
            Eigen::Matrix3d info_gyro_bias = Eigen::Matrix3d::Identity()*1e10;
            if(pAtlas->IsIMUCalibrated()){
                cv::Mat cvInfoG = pKFi->mpDvlPreintegrationKeyFrame->C.rowRange(9,12).colRange(9,12).inv(cv::DECOMP_SVD);
                cv::cv2eigen(cvInfoG,info_gyro_bias);
            }
            if(i==0){
                //set robust kernel
                eg_bias->setInformation(info_gyro_bias * 1e-2);
                //                ROS_INFO_STREAM("first KF:"<<pKFi->mnId);
                //                 g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                //                 rk->setDelta(sqrt(16.92));
            }
            eg_bias->setInformation(info_gyro_bias);
            // if(info_gyro_bias(0,0)==0)
            ROS_DEBUG_STREAM("KF["<<pKFi->mnId<<"] gyro bias info:\n"<<info_gyro_bias);

            optimizer.addEdge(eg_bias);

            EdgeDvlIMU2 *eDVL = new EdgeDvlIMU2(pKFi->mPrevKF->mpDvlPreintegrationKeyFrame,pKFi->mpDvlPreintegrationKeyFrame);
            eDVL->setLevel(0);
            eDVL->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
            eDVL->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
            eDVL->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
            eDVL->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
            eDVL->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG2));
            eDVL->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA2));
            eDVL->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
            eDVL->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
            eDVL->setVertex(8, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VR_b0_w));
            eDVL->setId(optimizer.edges().size());
            Eigen::Matrix<double,9,9> info_DVL=Eigen::Matrix<double,9,9>::Identity();
            info_DVL.block(0,0,3,3) = Eigen::Matrix3d::Identity() * 1e2;
            info_DVL.block(3,3,3,3) = Eigen::Matrix3d::Identity() * 1e2;
            info_DVL.block(6,6,3,3) = Eigen::Matrix3d::Identity() * 1e2;
            eDVL->setInformation(info_DVL);
            ROS_DEBUG_STREAM("DVL edge info:\n"<<info_DVL);
            optimizer.addEdge(eDVL);



            EdgeDvlIMU *eG = new EdgeDvlIMU(pKFi->mpDvlPreintegrationKeyFrame);
            eG->setLevel(0);
            eG->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
            eG->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
            eG->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
            eG->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
            eG->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG2));
            eG->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA2));
            eG->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
            eG->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
            eG->setVertex(8, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VR_b0_w));
            eG->setId(optimizer.edges().size());
            Eigen::Matrix<double,9,9> info_DI=Eigen::Matrix<double,9,9>::Identity();
            int visual_size = (mono_edges.size()+stereo_edges.size());
            if(pAtlas->IsIMUCalibrated()){
                cv::Mat cvInfo = pKFi->mpDvlPreintegrationKeyFrame->C.rowRange(0,9).colRange(0,9).inv(cv::DECOMP_SVD);
                cv::cv2eigen(cvInfo,info_DI);
                // info_DI=Eigen::Matrix<double,9,9>::Identity();
                // info_DI.block(0,0,3,3) = Eigen::Matrix3d::Identity() * 1e6;
                // info_DI.block(3,3,3,3) = Eigen::Matrix3d::Identity() * 1e4;
                // info_DI.block(6,6,3,3) = Eigen::Matrix3d::Identity() * 1e4;
            }
            else{
                // ROS_DEBUG_STREAM("Poor vision Hign DVL BA");
                eG->setLevel(1);
                info_DI.block(0,0,3,3) = Eigen::Matrix3d::Identity() * 1e10;
                info_DI.block(3,3,3,3) = Eigen::Matrix3d::Identity()*1e5;
                info_DI.block(6,6,3,3) = Eigen::Matrix3d::Identity() * 1;
            }
            // info(0,0) = info(0,0)*lamda_DVL * 5e3; // 10_24
            // info_DI(1,1) = 1e10; // before 10_24
            // ROS_INFO_STREAM("info: "<<info_DI);
            eG->setInformation(info_DI);
            ROS_DEBUG_STREAM("IMU edge info:\n"<<info_DI);
            dvlimu_edges.push_back(eG);
            optimizer.addEdge(eG);
        }
    }

    // optimizer.setVerbose(true);
    optimizer.initializeOptimization(0);
    optimizer.optimize(10);
    if(pbStopFlag){
        optimizer.setForceStopFlag(pbStopFlag);
    }
    // stringstream ss_v_chi2;
    // std::set<std::pair<KeyFrame*,MapPoint*>> remove_obs;
    // ss_v_chi2<<"mono chi2: ";
    // for (auto e: mono_edges) {
    //     e->computeError();
    //     ss_v_chi2<<e->chi2()<<", ";
    //     // e->setInformation(Eigen::Matrix2d::Identity() * lamda_visual);
    //     if (e->chi2() > 1) {
    //         e->setLevel(1);
    //         remove_obs.insert(mono_obs[e]);
    //     }
    //     else {
    //         e->setLevel(0);
    //         // e->setInformation(Eigen::Matrix2d::Identity() * 1);
    //     }
    //
    // }
    // ss_v_chi2<<"\n";
    // ss_v_chi2<<"stereo chi2: ";
    // for (auto e: stereo_edges) {
    //     e->computeError();
    //     ss_v_chi2<<e->chi2()<<", ";
    //     // e->setInformation(Eigen::Matrix3d::Identity() * lamda_visual);
    //     if (e->chi2() > 2) {
    //         e->setLevel(1);
    //         remove_obs.insert(stereo_obs[e]);
    //     }
    //     else {
    //         e->setLevel(0);
    //     }
    // }
    // // ROS_INFO_STREAM(ss_v_chi2.str());
    // for(int i=0;i<4;i++){
    //     if(pbStopFlag){
    //         if(*pbStopFlag){
    //             ROS_DEBUG_STREAM("stop BA");
    //             break;
    //         }
    //     }
    //     optimizer.initializeOptimization(0);
    //     optimizer.optimize(10);
    // }



    // Recover optimized data
    stringstream ss;
    ss<<"FULL BA"<<"\n";
    // unique_lock<shared_timed_mutex> lock(pMap->mMutexMapUpdate);
    // for(auto o:remove_obs){
    //     KeyFrame* pKFi = o.first;
    //     MapPoint* pMpi = o.second;
    //     pKFi->EraseMapPointMatch(pMpi);
    //     pMpi->EraseObservation(pKFi);
    //     ss<<"remove map point, KF: "<<pKFi->mnId<<", MP: "<<pMpi->mnId<<"\n";
    // }
    //update KF after current KF
    auto all_kf = pMap->GetAllKeyFrames();
    for(auto pkfi:all_kf){
        // Pose
        VertexPoseDvlIMU *VP = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKF->mnId));
        Eigen::Quaterniond Rwci(VP->estimate().Rwc);
        Eigen::Vector3d twci = VP->estimate().twc;
        Eigen::Isometry3d T_w_ci_new = Eigen::Isometry3d::Identity();
        T_w_ci_new.pretranslate(twci);
        T_w_ci_new.rotate(Rwci);

        cv::Mat Twci = pKF->GetPoseInverse();
        Eigen::Isometry3d T_w_ci = Eigen::Isometry3d::Identity();
        cv::cv2eigen(Twci, T_w_ci.matrix());
        Eigen::Isometry3d T_ci_w = T_w_ci.inverse();

        if(pkfi->mnId>pKF->mnId){
            cv::Mat Twcj = pkfi->GetPoseInverse();
            Eigen::Isometry3d T_w_cj = Eigen::Isometry3d::Identity();
            cv::cv2eigen(Twcj,T_w_cj.matrix());
            Eigen::Isometry3d T_ci_cj = T_ci_w * T_w_cj;
            Eigen::Isometry3d T_w_cj_new = T_w_ci_new * T_ci_cj;
            Eigen::Isometry3d T_cj_w_new = T_w_ci_new.inverse();
            cv::Mat Tcw;
            cv::eigen2cv(T_cj_w_new.matrix(),Tcw);
            Tcw.convertTo(Tcw, CV_32F);
            pkfi->SetPose(Tcw);

        }
    }

    for (size_t i = 0; i < N; i++) {
        KeyFrame *pKFi = OptKFs[i];
        int kf_id = pKFi->mnId;
        if (pKFi->mnId > maxKFid) {
            continue;
        }
        // Pose
        VertexPoseDvlIMU *VP = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
        Eigen::Quaterniond Rwc(VP->estimate().Rwc);
        Eigen::Vector3d twc = VP->estimate().twc;
        Eigen::Isometry3d Twc = Eigen::Isometry3d::Identity();
        Twc.pretranslate(twc);
        Twc.rotate(Rwc);
        Eigen::Isometry3d Tcw = Twc.inverse();
        cv::Mat Tcw_cv;
        cv::eigen2cv(Tcw.matrix(), Tcw_cv);
        Tcw_cv.convertTo(Tcw_cv, CV_32F);
        pKFi->SetPose(Tcw_cv);

        //Bias
        int gyros_bias_vertex_id = kf_id + maxKFid + 1;
        int acc_bias_vertex_id = kf_id + (maxKFid + 1)*2;
        VertexGyroBias *v_gb = dynamic_cast<VertexGyroBias *>(optimizer.vertex(gyros_bias_vertex_id));
        VertexAccBias *v_ab = dynamic_cast<VertexAccBias *>(optimizer.vertex(acc_bias_vertex_id));
        // bg << v_gb->estimate();
        IMU::Bias b(v_ab->estimate().x(), v_ab->estimate().y(), v_ab->estimate().z(),
                    v_gb->estimate().x(), v_gb->estimate().y(), v_gb->estimate().z());
        IMU::Bias b_old = pKFi->GetImuBias();
        ss<<"KF["<<pKFi->mnId<<"] old bias[acc gyros]: "<<b_old.bax<<" "<<b_old.bay<<" "<<b_old.baz<<" "<<b_old.bwx<<" "<<b_old.bwy<<" "<<b_old.bwz<<"\n";
        pKFi->SetNewBias(b);
        ss<<"KF["<<pKFi->mnId<<"] bias[acc gyros]: "<<v_ab->estimate().transpose()<<" "<<v_gb->estimate().transpose()<<"\n";

        //recover dvl_velocity of pKFi
        int dvl_vertex_id = kf_id + (maxKFid + 1)*3;
        VertexVelocity *v_dvl = dynamic_cast<VertexVelocity *>(optimizer.vertex(dvl_vertex_id));
        Eigen::Vector3d dvl_velocity;
        pKFi->GetDvlVelocity(dvl_velocity);
        ss<<"KF["<<pKFi->mnId<<"] "<<pKFi->mTimeStamp<< "old dvl_velocity: "<<dvl_velocity.transpose()<<"\n"
          <<"KF["<<pKFi->mnId<<"] "<<pKFi->mTimeStamp<< "new dvl_velocity: "<<v_dvl->estimate().transpose()<<"\n";
        dvl_velocity = v_dvl->estimate();
        pKFi->SetDvlVelocity(dvl_velocity);


    }


    ROS_DEBUG_STREAM(ss.str());

    for (auto pMP:LocalMapPoints) {
        if(find(LocalFixedMapPoints.begin(), LocalFixedMapPoints.end(), pMP) !=
           LocalFixedMapPoints.end()){
            continue;
        }
        g2o::VertexSBAPointXYZ
                *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex((maxKFid + 1) * 5 + pMP->mnId));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }
    pMap->IncreaseChangeIndex();
    // ROS_INFO_STREAM("Map change after BA: "<<pMap->GetMapChangeIndex());
    ROS_INFO_STREAM("Full BA upto KF["<<pKF->mnId<<"] done");
}

void DvlGyroOptimizer::LocalDVLIMUPoseGraph(Atlas* pAtlas, KeyFrame* pKF, Map* pMap)
{
    Map *pCurrentMap = pKF->GetMap();
    int Nd = std::min(100, (int)pCurrentMap->KeyFramesInMap() - 2);// number of keyframes in current map
    const unsigned long maxKFid = pKF->mnId;

    vector<KeyFrame *> OptKFs;
    OptKFs.reserve(Nd);
    OptKFs.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    for (int i = 1; i < Nd; i++) {
        if (OptKFs.back()->mPrevKF) {
            OptKFs.push_back(OptKFs.back()->mPrevKF);
            OptKFs.back()->mnBALocalForKF = pKF->mnId;
        }
        else {
            break;
        }
    }
    int N = OptKFs.size();

    vector<KeyFrame *> FixedKFs;
    if (OptKFs.back()->mPrevKF) {
        FixedKFs.push_back(OptKFs.back()->mPrevKF);
        OptKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
    }
    else {
        OptKFs.back()->mnBALocalForKF = 0;
        OptKFs.back()->mnBAFixedForKF = pKF->mnId;
        FixedKFs.push_back(OptKFs.back());
        OptKFs.pop_back();
    }


    vector<MapPoint *> LocalMapPoints;
    for (int i = 0; i < N; i++) {
        vector<MapPoint *> vpMPs = OptKFs[i]->GetMapPointMatches();
        for (vector<MapPoint *>::iterator it = vpMPs.begin(); it != vpMPs.end(); it++) {
            MapPoint *pMP = *it;
            if (pMP) {
                //				cout<<"find local map point"<<endl;
                if (!pMP->isBad()) {
                    //					cout<<"find local good map point"<<endl;
                    if (pMP->mnBALocalForKF != pKF->mnId) {
                        //						cout<<"find local map point with correct BALocalForKF"<<endl;
                        LocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                        //						cout<<"add local map point to optimize: "<<endl;
                    }
                }
            }
        }
    }
    int N_map_points = LocalMapPoints.size();

    int N_fixed = FixedKFs.size();


    Verbose::PrintMess("DVL Gyro optimization", Verbose::VERBOSITY_NORMAL);
    int its = 200; // Check number of iterations
    //	const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    optimizer.setAlgorithm(solver);

    // Set KeyFrame vertices (fixed poses and optimizable velocities)
    for (size_t i = 0; i < OptKFs.size(); i++) {
        KeyFrame *pKFi = OptKFs[i];
        if (pKFi->mnId > maxKFid) {
            continue;
        }
        VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);
        // ROS_INFO_STREAM("opt KF: "<<pKFi->mnId);
    }
    for (int i = 0; i < FixedKFs.size(); i++) {
        KeyFrame *pKFi = FixedKFs[i];
        if (pKFi->mnId > maxKFid) {
            continue;
        }
        VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);
        // ROS_INFO_STREAM("fixed KF: "<<pKFi->mnId);
    }

    // Biases
    vector<VertexGyroBias*> vpgb;
    vector<VertexAccBias*> vpab;
    //todo_tightly
    //	set fixed for debuging
    for(auto pKFi:OptKFs){
        VertexGyroBias *VG = new VertexGyroBias(pKFi);
        VG->setId(maxKFid + 1 + pKFi->mnId);
        VG->setFixed(true);
        optimizer.addVertex(VG);
        vpgb.push_back(VG);

        VertexAccBias *VA = new VertexAccBias(pKFi);
        VA->setId((maxKFid + 1)*2 + pKFi->mnId);
        VA->setFixed(false);
        optimizer.addVertex(VA);


        VertexVelocity *VV = new VertexVelocity(pKFi);
        VV->setId((maxKFid + 1)*3 + pKFi->mnId);
        VV->setFixed(true);
        optimizer.addVertex(VV);
    }
    for(auto pKFi:FixedKFs){
        VertexGyroBias *VG = new VertexGyroBias(pKFi);
        VG->setId(maxKFid + 1 + pKFi->mnId);
        VG->setFixed(true);
        optimizer.addVertex(VG);
        vpgb.push_back(VG);

        VertexAccBias *VA = new VertexAccBias(pKFi);
        VA->setId((maxKFid + 1)*2 + pKFi->mnId);
        VA->setFixed(true);
        optimizer.addVertex(VA);
        // vpab.push_back(VA);

        VertexVelocity *VV = new VertexVelocity(pKFi);
        VV->setId((maxKFid + 1)*3 + pKFi->mnId);
        VV->setFixed(true);
        optimizer.addVertex(VV);

    }

    // extrinsic parameter
    g2o::VertexSE3Expmap *vT_d_c = new g2o::VertexSE3Expmap();
    vT_d_c->setEstimate(Converter::toSE3Quat(pKF->mImuCalib.mT_dvl_c));
    vT_d_c->setId((maxKFid + 1)*4);
    vT_d_c->setFixed(true);
    optimizer.addVertex(vT_d_c);

    g2o::VertexSE3Expmap *vT_g_d = new g2o::VertexSE3Expmap();
    vT_g_d->setEstimate(Converter::toSE3Quat(pKF->mImuCalib.mT_gyro_dvl));
    vT_g_d->setId((maxKFid + 1)*4+1);
    vT_g_d->setFixed(true);
    optimizer.addVertex(vT_g_d);

    VertexGDir *VGDir = new VertexGDir(pAtlas->getRGravity());
    VGDir->setId((maxKFid + 1)*4+2);
    VGDir->setFixed(true);
    optimizer.addVertex(VGDir);

    vector<EdgeMonoBA_DvlGyros *> mono_edges;
    vector<EdgeStereoBA_DvlGyros *> stereo_edges;


    // Graph edges
    vector<EdgeDvlGyroBA *> dvl_edges;
    dvl_edges.reserve(OptKFs.size());
    vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
    //	vppUsedKF.reserve(OptKFs.size() + FixedKFs.size());
    //	std::cout << "build optimization graph" << std::endl;

    for (size_t i = 0; i < OptKFs.size(); i++) {
        KeyFrame *pKFi = OptKFs[i];

        if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
            if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) {
                continue;
            }

            // VertexPoseDvlGro *VP1 = dynamic_cast<VertexPoseDvlGro *>(optimizer.vertex(pKFi->mPrevKF->mnId));
            //				g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
            // VertexPoseDvlGro *VP2 = dynamic_cast<VertexPoseDvlGro *>(optimizer.vertex(pKFi->mnId));
            //				g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
            // g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid + 1 + i);
            // g2o::HyperGraph::Vertex *VT_d_c = optimizer.vertex(maxKFid + N + N_fixed + 1);
            // g2o::HyperGraph::Vertex *VT_g_d = optimizer.vertex(maxKFid + N + N_fixed + 2);

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
            pKFi->mPrevKF->GetDvlVelocity(dvl_v1);
            Eigen::Vector3d dvl_v2;
            pKFi->GetDvlVelocity(dvl_v2);
            EdgeDvlVelocity *ev = new EdgeDvlVelocity(dvl_v1);
            ev->setLevel(0);
            ev->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
            if(pKFi->mbDVL){
                ev->setInformation(Eigen::Matrix3d::Identity()*100);
            }
            else{
                ev->setInformation(Eigen::Matrix3d::Identity());
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
                ev2->setInformation(Eigen::Matrix3d::Identity());
            }
            optimizer.addEdge(ev2);



            // g2o::EdgeSE3* edge = new g2o::EdgeSE3();
            // edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(i)));
            // edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(j)));
            // edge->setMeasurement(T_ij);
            // edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
            // optimizer.addEdge(edge);


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
            eG->setInformation(Eigen::Matrix<double, 9, 9>::Identity() * 100);
            eG->setId(maxKFid+1 + pKFi->mnId);
            optimizer.addEdge(eG);

            // eG1->setId(maxKFid+1 + pKFi->mnId);
            // only add first 5 KF for bias optimization
            // if((pKF->mnId - pKFi->mnId)<5){
            // 	ROS_DEBUG_STREAM("add bias edge: "<<pKFi->mPrevKF->mnId<<"->"<<pKFi->mnId);
            //     EdgeDvlIMUGravityRefineWithBias *eG1 = new EdgeDvlIMUGravityRefineWithBias(pKFi->mpDvlPreintegrationKeyFrame);
            //     eG1->setLevel(1);
            //     eG1->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
            //     eG1->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
            //     eG1->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
            //     eG1->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
            //     eG1->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
            //     eG1->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
            //     eG1->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
            //     eG1->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
            //     eG1->setVertex(8, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VR_b0_w));
            //     eG1->setInformation(Eigen::Matrix<double, 9, 9>::Identity() * lamda_DVL * (stereo_edges.size()+mono_edges.size()));
            //     optimizer.addEdge(eG1);
            //     // ROS_INFO_STREAM("add bias refine edge, KF: "<<pKFi->mnId);
            // }



        }
    }

    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};

    // Compute error for different scales
    //	std::cout << "start optimization" << std::endl;
    // optimizer.setVerbose(true);

    optimizer.initializeOptimization(0);
    optimizer.optimize(5);




    // for(auto p:vpab){
    //     p->setFixed(false);
    // }
    // optimizer.initializeOptimization(0);
    // optimizer.optimize(2);


    int visual_edge_num = mono_edges.size() + stereo_edges.size();
    int dvl_edge_num = dvl_edges.size();

    //	std::cout << "end optimization" << std::endl;


    // Recover optimized data
    stringstream ss;
    ss<<"LocalVisualAcousticInertial BA"<<"\n";
    unique_lock<shared_timed_mutex> lock(pMap->mMutexMapUpdate);
    for (size_t i = 0; i < N; i++) {
        KeyFrame *pKFi = OptKFs[i];
        int kf_id = pKFi->mnId;
        if (pKFi->mnId > maxKFid) {
            continue;
        }
        // Pose
        VertexPoseDvlIMU *VP = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
        Eigen::Quaterniond Rwc(VP->estimate().Rwc);
        Eigen::Vector3d twc = VP->estimate().twc;
        Eigen::Isometry3d Twc = Eigen::Isometry3d::Identity();
        Twc.pretranslate(twc);
        Twc.rotate(Rwc);
        Eigen::Isometry3d Tcw = Twc.inverse();
        cv::Mat Tcw_cv;
        cv::eigen2cv(Tcw.matrix(), Tcw_cv);
        Tcw_cv.convertTo(Tcw_cv, CV_32F);
        pKFi->SetPose(Tcw_cv);

        //Bias
        int gyros_bias_vertex_id = kf_id + maxKFid + 1;
        int acc_bias_vertex_id = kf_id + (maxKFid + 1)*2;
        VertexGyroBias *v_gb = dynamic_cast<VertexGyroBias *>(optimizer.vertex(gyros_bias_vertex_id));
        VertexAccBias *v_ab = dynamic_cast<VertexAccBias *>(optimizer.vertex(acc_bias_vertex_id));
        // bg << v_gb->estimate();
        IMU::Bias b(v_ab->estimate().x(), v_ab->estimate().y(), v_ab->estimate().z(),
                    v_gb->estimate().x(), v_gb->estimate().y(), v_gb->estimate().z());
        pKFi->SetNewBias(b);
        // ss<<"KF["<<pKFi->mnId<<"] bias[acc gyros]: "<<v_ab->estimate().transpose()<<" "<<v_gb->estimate().transpose()<<"\n";

        //recover dvl_velocity of pKFi
        int dvl_vertex_id = kf_id + (maxKFid + 1)*3;
        VertexVelocity *v_dvl = dynamic_cast<VertexVelocity *>(optimizer.vertex(dvl_vertex_id));
        Eigen::Vector3d dvl_velocity;
        pKFi->GetDvlVelocity(dvl_velocity);
        ss<<"KF["<<pKFi->mnId<<"] "<<pKFi->mTimeStamp<< "old dvl_velocity: "<<dvl_velocity.transpose()<<"\n"
          <<"KF["<<pKFi->mnId<<"] "<<pKFi->mTimeStamp<< "new dvl_velocity: "<<v_dvl->estimate().transpose()<<"\n";
        dvl_velocity = v_dvl->estimate();
        pKFi->SetDvlVelocity(dvl_velocity);


    }
    // ROS_INFO_STREAM(ss.str());

    // for (int i = 0; i < N_map_points; i++) {
    //     MapPoint *pMP = LocalMapPoints[i];
    //     g2o::VertexSBAPointXYZ
    //             *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex((maxKFid + 1)*5 + i));
    //     pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
    //     pMP->UpdateNormalAndDepth();
    // }
    pMap->IncreaseChangeIndex();
}

void DvlGyroOptimizer::FullDVLGyroBundleAdjustment(bool *pbStopFlag, Map *pMap, double lamda_DVL)
{
	const unsigned long maxKFid = pMap->GetMaxKFid();
	vector<KeyFrame *> OptKFs = pMap->GetAllKeyFrames();
	vector<MapPoint *> AllMapPoints = pMap->GetAllMapPoints();

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);

	// set keyframe vertex
	for (auto pKFi: OptKFs) {
		if (pKFi->mnId > maxKFid) {
			continue;
		}
		if (pKFi->mPrevKF) {
			VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
			VP->setId(pKFi->mnId);
			VP->setFixed(false);
			optimizer.addVertex(VP);
		}
		else {//fix first keyframe pose
			VertexPoseDvlIMU *VP = new VertexPoseDvlIMU(pKFi);
			VP->setId(pKFi->mnId);
			VP->setFixed(true);
			optimizer.addVertex(VP);

		}
	}
	int N = OptKFs.size();

	int N_map_points = AllMapPoints.size();
//	cout << "map point to optimize: " << N_map_points << endl;


	Verbose::PrintMess("DVL Gyro optimization", Verbose::VERBOSITY_NORMAL);
	int its = 200; // Check number of iterations
//	const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();




	// Biases
	//todo_tightly
	//	set fixed for debuging
	for (int i = 0; i < N; i++) {
		VertexGyroBias *VG = new VertexGyroBias(OptKFs[i]);
		VG->setId(maxKFid + 1 + OptKFs[i]->mnId);
		VG->setFixed(true);
		optimizer.addVertex(VG);
	}



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
	vT_d_c->setEstimate(Converter::toSE3Quat(OptKFs.front()->mImuCalib.mT_dvl_c));
	vT_d_c->setId(maxKFid + maxKFid + 1 + 1);
	vT_d_c->setFixed(true);
	optimizer.addVertex(vT_d_c);

	g2o::VertexSE3Expmap *vT_g_d = new g2o::VertexSE3Expmap();
	vT_g_d->setEstimate(Converter::toSE3Quat(OptKFs.front()->mImuCalib.mT_gyro_dvl));
	vT_g_d->setId(maxKFid + maxKFid + 1 + 2);
	vT_g_d->setFixed(true);
	optimizer.addVertex(vT_g_d);

	vector<EdgeMonoBA_DvlGyros *> mono_edges;
	vector<EdgeStereoBA_DvlGyros *> stereo_edges;
	//add map point vertex and visual constrain
	{
		unique_lock<mutex> lock(MapPoint::mGlobalMutex);

		for (int i = 0; i < N_map_points; i++) {
			MapPoint *pMP = AllMapPoints[i];
			if (pMP) {
				g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
				vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
				int id = pMP->mnId + maxKFid + maxKFid + 1 + 2 + 1;
				vPoint->setId(id);
				vPoint->setMarginalized(true);
				optimizer.addVertex(vPoint);

				const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

				for (auto ob: observations) {
					KeyFrame *pKFi = ob.first;

					if (!pKFi->isBad()) {
						const int leftIndex = get<0>(ob.second);

						// Monocular observation
						if (leftIndex != -1 && pKFi->mvuRight[get<0>(ob.second)] < 0) {
							const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
							Eigen::Matrix<double, 2, 1> obs;
							obs << kpUn.pt.x, kpUn.pt.y;

							EdgeMonoBA_DvlGyros *e = new EdgeMonoBA_DvlGyros();

							e->setVertex(0,
										 dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
							e->setVertex(1, vPoint);
							e->setMeasurement(obs);
							const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
							e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

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

							e->setVertex(0,
										 dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
							e->setVertex(1, vPoint);
							e->setMeasurement(obs);
							const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
							Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
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
	dvl_edges.reserve(OptKFs.size());
	vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
//	vppUsedKF.reserve(OptKFs.size() + FixedKFs.size());
//	std::cout << "build optimization graph" << std::endl;

	for (size_t i = 0; i < OptKFs.size(); i++) {
		KeyFrame *pKFi = OptKFs[i];

		if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
			if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) {
				continue;
			}

			VertexPoseDvlIMU *VP1 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mPrevKF->mnId));
//				g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
			VertexPoseDvlIMU *VP2 = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
//				g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
			g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid + 1 + pKFi->mnId);
			g2o::HyperGraph::Vertex *VT_d_c = optimizer.vertex(maxKFid + maxKFid + 1 + 1);
			g2o::HyperGraph::Vertex *VT_g_d = optimizer.vertex(maxKFid + maxKFid + 1 + 2);
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
//			EdgeDvlGyroInit *ei = new EdgeDvlGyroInit(pKFi->mpDvlPreintegrationKeyFrame);
			EdgeDvlGyroBA *ei = new EdgeDvlGyroBA(pKFi->mpDvlPreintegrationKeyFrame);
//				ei->setVertex(0, VP1);

//			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
//			ei->setRobustKernel(rk);
//			rk->setDelta(sqrt(7.815));
			ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
			ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
			ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
			ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_d_c));
			ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VT_g_d));
			ei->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * lamda_DVL);
			ei->setId(pKFi->mnId);


			dvl_edges.push_back(ei);

			vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
			optimizer.addEdge(ei);
		}
	}

	const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
	const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};

	// Compute error for different scales
//	std::cout << "start optimization" << std::endl;
	optimizer.setVerbose(false);


	int mono_outlier;
	int stereo_outlier;
	float visula_chi2, dvl_chi2;
	for (int i = 0; i < 4; i++) {
		optimizer.initializeOptimization(0);
		optimizer.optimize(10);

		mono_outlier = 0;
		stereo_outlier = 0;
		visula_chi2 = 0;
		dvl_chi2 = 0;
		for (auto e_mono: mono_edges) {
			e_mono->computeError();
			const float chi2 = e_mono->chi2();
			if (chi2 > chi2Mono[i]) {
				e_mono->setLevel(1);
			}
			visula_chi2 += chi2;
		}
		for (auto e_stereo: stereo_edges) {
			e_stereo->computeError();
			const float chi2 = e_stereo->chi2();
			if (chi2 > chi2Stereo[i]) {
				e_stereo->setLevel(1);
			}
			visula_chi2 += chi2;
		}
		for (auto e_dvl: dvl_edges) {
			e_dvl->computeError();
			const float chi2 = e_dvl->chi2();
			dvl_chi2 += chi2;
//			cout << "dvl error: " << e_dvl->error().transpose() << endl;
		}
//		cout << "iteration: " << i << " visual chi2: " << visula_chi2 << " dvl_gyro chi2: " << dvl_chi2 << endl;
	}


	int visual_edge_num = mono_edges.size() + stereo_edges.size();
	int dvl_edge_num = dvl_edges.size();

//	std::cout << "end optimization" << std::endl;


	// Recover optimized data
	// Biases
	VertexGyroBias *VG = static_cast<VertexGyroBias *>(optimizer.vertex(maxKFid + 1 + OptKFs[0]->mnId));
//	Eigen::Matrix<double, 6, 1> vb;
	Eigen::Vector3d bg;
//	vb << VG->estimate(), 0, 0, 0;
	bg << VG->estimate();
//
//	vT_d_c = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(maxKFid + 2));
//
//	Eigen::Isometry3d T_dvl_c = vT_d_c->estimate();
//	Eigen::Isometry3d T_gyros_dvl = vT_g_d->estimate();
//
	IMU::Bias b(0, 0, 0, bg[0], bg[1], bg[2]);
//
//	Eigen::Matrix3d R_gt;
//	R_gt << 0, 0, 1,
//		-1, 0, 0,
//		0, -1, 0;
//	cout << "init optimization result: \n"
//		 << "bias_gyro:\n" << bg << "\n"
//		 << "R_dvl_c:\n" << T_dvl_c.rotation() << "\n"
//		 << "R_dvl_c(eular yaw-pitch-roll):" << T_dvl_c.rotation().eulerAngles(2, 1, 0).transpose() << "\n"
//		 << "t_dvl_c:" << T_dvl_c.translation().transpose() << "\n"
//		 << "R_dvl_c distance with R_gt(LogSO3()): " << LogSO3(T_dvl_c.rotation().inverse() * R_gt).transpose() << "\n"
//		 << "R_gyros_dvl:\n" << T_gyros_dvl.rotation() << "\n"
//		 << "R_gyros_dvl(eular yaw-pitch-roll):" << T_gyros_dvl.rotation().eulerAngles(2, 1, 0).transpose() << "\n"
//		 << endl;
//
//
	cv::Mat cvbg = Converter::toCvMat(bg);

	//Keyframes velocities and biases
//	std::cout << "update Keyframes biases" << std::endl;

	for (size_t i = 0; i < N; i++) {
		KeyFrame *pKFi = OptKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}

		VertexPoseDvlIMU *VP = dynamic_cast<VertexPoseDvlIMU *>(optimizer.vertex(pKFi->mnId));
		Eigen::Quaterniond Rwc(VP->estimate().Rwc);
		Eigen::Vector3d twc = VP->estimate().twc;
		Eigen::Isometry3d Twc = Eigen::Isometry3d::Identity();
		Twc.pretranslate(twc);
		Twc.rotate(Rwc);
		Eigen::Isometry3d Tcw = Twc.inverse();
		cv::Mat Tcw_cv;
		cv::eigen2cv(Tcw.matrix(), Tcw_cv);
		Tcw_cv.convertTo(Tcw_cv, CV_32F);
		pKFi->SetPose(Tcw_cv);

//		if (cv::norm(pKFi->GetGyroBias() - cvbg) > 0.01) {
//			pKFi->SetNewBias(b);
//			if (pKFi->mpDvlPreintegrationKeyFrame) {
//				pKFi->mpDvlPreintegrationKeyFrame->ReintegrateWithVelocity();
//			}
//		}
//		else {
//			pKFi->SetNewBias(b);
//		}
	}

	for (int i = 0; i < N_map_points; i++) {
		MapPoint *pMP = AllMapPoints[i];
		g2o::VertexSBAPointXYZ
			*vPoint =
			static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(maxKFid + maxKFid + 2 + 1 + 1 + pMP->mnId));
		pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMP->UpdateNormalAndDepth();
	}
	pMap->IncreaseChangeIndex();
}

int DvlGyroOptimizer::PoseDvlGyrosOPtimizationLastFrame(Frame *pFrame, bool bRecInit)
{
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

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
	EdgeDvlGyroInit *ei = new EdgeDvlGyroInit(pFrame->mpDvlPreintegrationFrame);

	ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP));
	ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
	ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
	ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vT_d_c));
	ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vT_g_d));
	ei->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100);
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
int DvlGyroOptimizer::PoseDvlGyrosOPtimizationLastKeyFrame(Frame *pFrame, bool bRecInit)
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

					EdgeMonoOnlyPose_DvlGyros *e = new EdgeMonoOnlyPose_DvlGyros(pMP->GetWorldPos(), 0);

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

				// Right monocular observation
				if (bRight && i >= Nleft) {
					nInitialMonoCorrespondences++;
					pFrame->mvbOutlier[i] = false;

					kpUn = pFrame->mvKeysRight[i - Nleft];
					Eigen::Matrix<double, 2, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y;

					EdgeMonoOnlyPose_DvlGyros *e = new EdgeMonoOnlyPose_DvlGyros(pMP->GetWorldPos(), 1);

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


	KeyFrame *pFK = pFrame->mpLastKeyFrame;
	VertexPoseDvlIMU *VP2 = new VertexPoseDvlIMU(pFK);
	VP2->setId(1);
	VP2->setFixed(true);
	optimizer.addVertex(VP2);

	// set DVL_Gyros constrain
	//todo_tightly
	//	maybe add velocity to optimization
	EdgeDvlGyroInit *ei = new EdgeDvlGyroInit(pFrame->mpDvlPreintegrationFrame);

	ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP));
	ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
	ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
	ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vT_d_c));
	ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vT_g_d));
	ei->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 100);
	ei->setId(pFrame->mnId);


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
	pFrame->SetDvlPoseVelocity(Converter::toCvMat(R_w_g),
							   Converter::toCvMat(t_w_d),
							   pFrame->GetDvlVelocity());
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
void DvlGyroOptimizer::DvlGyroInitOptimization(Map *pMap,
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
	Eigen::Isometry3d T_gyros_c = T_gyros_dvl * T_dvl_c;

	IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);


	cout << "init optimization result: \n"
		 << "bias_gyro:\n" << bg << "\n"
		 << "R_dvl_c:\n" << T_dvl_c.rotation() << "\n"
		 << "t_dvl_c:" << T_dvl_c.translation().transpose() << "\n"
		 << "R_gyros_c:\n" << T_gyros_c.rotation() << "\n"
		 << endl;


	cv::Mat cvbg = Converter::toCvMat(bg);

	//Keyframes velocities and biases
	std::cout << "update Keyframes biases" << std::endl;

	const int N = vpKFs.size();
	for (size_t i = 0; i < N; i++) {
		KeyFrame *pKFi = vpKFs[i];
		if (pKFi->mnId > maxKFid) {
			continue;
		}

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
}