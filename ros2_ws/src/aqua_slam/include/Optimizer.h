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


#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include <math.h>

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

namespace ORB_SLAM3
{

class LoopClosing;
class Atlas;

class Optimizer
{
public:
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    void static FullInertialBA(Map *pMap, int its, const bool bFixLocal=false, const unsigned long nLoopKF=0, bool *pbStopFlag=NULL, bool bInit=false, float priorG = 1e2, float priorA=1e6, Eigen::VectorXd *vSingVal = NULL, bool *bHess=NULL);

    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, vector<KeyFrame*> &vpNonEnoughOptKFs);
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF);
    void static LocalDVLBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF);
	void static LocalDVLRefinement(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF);
    void static MergeBundleAdjustmentVisual(KeyFrame* pCurrentKF, vector<KeyFrame*> vpWeldingKFs, vector<KeyFrame*> vpFixedKFs, bool *pbStopFlag);

    int static PoseOptimization(Frame* pFrame);
    int static PoseOptimizationWithBA_and_EKF(Frame *pFrame,Frame *pLastFrame, double lamda_visual=1, double lamda_DVL=1);
	int static PoseOptimizationWithBA_and_EKF2(Frame *pFrame,Frame *pLastFrame, double lamda_visual=1, double lamda_DVL=1);
	void static PoseOptimizationWithEKF(Frame *pFrame,Frame *pLastFrame);
    void static PoseOnlyOptimizationDVLIMU(set<KeyFrame*, KFComparator> &loss_kfs, Atlas* pAtlas, int& optimized_kf_id);
    void static OptimizationDVLIMU(set<KeyFrame*, KFComparator> &loss_kfs, Atlas* pAtlas, double lamda_DVL);

    int static PoseInertialOptimizationLastKeyFrame(Frame* pFrame, bool bRecInit = false);
    int static PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit = false);

    int static PoseDvlGyrosOPtimizationLastFrame(Frame *pFrame, double lamda_DVL, bool bRecInit = false);
	int static PoseDvlGyrosOPtimizationLastKeyFrame(Frame *pFrame,double lamda_DVL, bool bRecInit = false);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);
    void static OptimizeEssentialGraph6DoF(KeyFrame* pCurKF, vector<KeyFrame*> &vpFixedKFs, vector<KeyFrame*> &vpFixedCorrectedKFs,
                                           vector<KeyFrame*> &vpNonFixedKFs, vector<MapPoint*> &vpNonCorrectedMPs, double scale);
    /***
    *
    * @param pCurKF current KeyFrame
    * @param vpFixedKFs pervious KFs in previous map optimized LocalBA
    * @param vpFixedCorrectedKFs current KFs but already corrected by loop transformation and optimized by LocalBA
    * @param vpNonFixedKFs others KFs haven't optimized by LocalBA in current map
    * @param vpNonCorrectedMPs others Map Points haven't optimized by LocalBA in current map
    * @param bFinished
    * @param bReSet
    */
    void static OptimizeEssentialGraph(KeyFrame *pCurKF, vector<KeyFrame *> &vpFixedKFs, vector<KeyFrame *> &vpFixedCorrectedKFs,
                                       vector<KeyFrame *> &vpNonFixedKFs, vector<MapPoint *> &vpNonCorrectedMPs, bool &bFinished, bool* bReSet);

    /***
    *
    * @param pCurKF current KeyFrame
    * @param vpFixedKFs pervious KFs in previous map optimized LocalBA
    * @param vpFixedCorrectedKFs current KFs but already corrected by loop transformation and optimized by LocalBA
    * @param vpNonFixedKFs others KFs haven't optimized by LocalBA in current map
    * @param vpNonCorrectedMPs others Map Points haven't optimized by LocalBA in current map
    * @param pAtlas pointer to the atlas
    */
    void static GlobalVAPoseGraphOptimization(KeyFrame *pCurKF, vector<KeyFrame *> &vpFixedKFs, vector<KeyFrame *> &vpFixedCorrectedKFs,vector<KeyFrame *> &vpNonFixedKFs,
                                              vector<MapPoint *> &vpNonCorrectedMPs, Atlas* pAtlas);

    void static OptimizeEssentialGraph(KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3);
    // For inetial loopclosing
    void static OptimizeEssentialGraph4DoF(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono) (OLD)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono) (NEW)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale,
                            Eigen::Matrix<double,7,7> &mAcumHessian, const bool bAllPoints=false);
    static int OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, vector<KeyFrame*> &vpMatches1KF,
                     g2o::Sim3 &g2oS12, const float th2, const bool bFixScale, Eigen::Matrix<double,7,7> &mAcumHessian,
                     const bool bAllPoints = false);

    // For inertial systems

    void static LocalInertialBA(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, bool bLarge = false, bool bRecInit = false);

    void static MergeInertialBA(KeyFrame* pCurrKF, KeyFrame* pMergeKF, bool *pbStopFlag, Map *pMap, LoopClosing::KeyFrameAndPose &corrPoses);

    // Local BA in welding area when two maps are merged
    void static LocalBundleAdjustment(KeyFrame* pMainKF,vector<KeyFrame*> vpAdjustKF, vector<KeyFrame*> vpFixedKF, bool *pbStopFlag);

    // Marginalize block element (start:end,start:end). Perform Schur complement.
    // Marginalized elements are filled with zeros.
    static Eigen::MatrixXd Marginalize(const Eigen::MatrixXd &H, const int &start, const int &end);
    // Condition block element (start:end,start:end). Fill with zeros.
    static Eigen::MatrixXd Condition(const Eigen::MatrixXd &H, const int &start, const int &end);
    // Remove link between element 1 and 2. Given elements 1,2 and 3 must define the whole matrix.
    static Eigen::MatrixXd Sparsify(const Eigen::MatrixXd &H, const int &start1, const int &end1, const int &start2, const int &end2);

    // Inertial pose-graph
	// used for IMU initialization
    void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale, Eigen::Vector3d &bg, Eigen::Vector3d &ba, bool bMono, Eigen::MatrixXd  &covInertial, bool bFixedVel=false, bool bGauss=false, float priorG = 1e2, float priorA = 1e6);
    void static InertialOptimization(Map *pMap, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG = 1e2, float priorA = 1e6);
    void static InertialOptimization(vector<KeyFrame*> vpKFs, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG = 1e2, float priorA = 1e6);
    void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale);

    //fixed gyros bias
	void static DvlGyroInitOptimization(Map *pMap, Eigen::Vector3d &bg, bool bMono, float priorG = 1e2);
	//optimize gyros bias
	void static DvlGyroInitOptimization2(Map *pMap, Eigen::Vector3d &bg, bool bMono, float priorG = 1e2);
	// fix bias, opt extrinsic para
	void static DvlGyroInitOptimization3(Map *pMap, Eigen::Vector3d &bg, bool bMono, float priorG = 1e2);
	// opt bias, opt extrinsic para
	void static DvlGyroInitOptimization4(Map *pMap, Eigen::Vector3d &bg, bool bMono, float priorG = 1e2);
	// fix bias, opt beam orientation
	void static DvlGyroInitOptimization5(Map *pMap, Eigen::Vector3d &bg, bool bMono, float priorG = 1e2);
	// fix bias extrinsic, opt velocity
	void static DvlGyroInitOptimization6(Map *pMap, Eigen::Vector3d &bg, bool bMono, float priorG = 1e2);

	double static DvlIMUInitOptimization(Map *pMap, double priori_g, double priori_a);

    void static DvlIMURefineOptimization(Atlas *pAtlas);


	void static DvlBeamOptimization(Map *pMap);
	void static DvlBeamOptimization_dvl(Map *pMap);
};

} //namespace ORB_SLAM3

#endif // OPTIMIZER_H
