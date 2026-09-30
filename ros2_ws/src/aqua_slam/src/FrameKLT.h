//
// Created by da on 17/03/2022.
//

#ifndef FRAMEKLT_H
#define FRAMEKLT_H
#include <vector>
#include <iostream>
#include <thread>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>


namespace ORB_SLAM3{

class FrameKLT
{
public:
	FrameKLT(){}

public:
	// pose T_cj_c0
	cv::Mat mTcw;
	// Frame timestamp.
	double mTimeStamp;
	// image
	cv::Mat imgLeft, imgRight;

	// ID
	long mID;

	// KeyPoints
	std::map<long,cv::Point2d> mKPWithID;
	std::vector<cv::KeyPoint> mKP_l,mKP_r;
	cv::Mat mDsp_l,mDsp_r;
	// feature number
	int N;
	std::vector<float> mvDepth;

};

}



#endif //FRAMEKLT_H
