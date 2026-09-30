//
// Created by da on 08/06/2021.
//

#ifndef DVLGYROOPTIMIZER_H
#define DVLGYROOPTIMIZER_H

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

class DvlGyroOptimizer
{
public:


	void static LocalDVLBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF);
	void static LocalDVLGyroBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, double lamda_DVL);
    void static LocalDVLIMUBundleAdjustment(Atlas* pAtlas, KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, double lamda_DVL, double lamda_visual = 1.0);
    void static LocalDVLIMUBundleAdjustment2(Atlas* pAtlas, KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, double lamda_DVL, double lamda_visual = 1.0);
    void static FullDVLIMUBundleAdjustment(Atlas* pAtlas, KeyFrame* pKF, bool *pbStopFlag, Map *pMap,const int& num_fixedKF, double lamda_DVL, double lamda_visual = 1.0);
    void static LocalDVLIMUPoseGraph(Atlas* pAtlas, KeyFrame* pKF, Map *pMap);
	void static FullDVLGyroBundleAdjustment(bool *pbStopFlag, Map *pMap, double lamda_DVL);


	int static PoseDvlGyrosOPtimizationLastFrame(Frame *pFrame, bool bRecInit = false);
	int static PoseDvlGyrosOPtimizationLastKeyFrame(Frame *pFrame, bool bRecInit = false);



	void static DvlGyroInitOptimization(Map *pMap, Eigen::Vector3d &bg, bool bMono, float priorG = 1e2);

};

} //namespace ORB_SLAM3


#endif //DVLGYROOPTIMIZER_H
