//
// Created by da on 19/12/2021.
//

#include "LKTracker.h"
#include "Frame.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "FrameKLT.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
namespace ORB_SLAM3
{
LKTracker::LKTracker()
{
	pTrack_img_pub = boost::make_shared<image_transport::Publisher>(
		image_transport::create_publisher(aqua_slam::GetNode().get(), "~/lk_tracker/track_img"));
}
LKTracker::LKTracker(bool bStereo)
	: stereo_cam(bStereo)
{
	pTrack_img_pub = boost::make_shared<image_transport::Publisher>(
		image_transport::create_publisher(aqua_slam::GetNode().get(), "~/lk_tracker/track_img"));
}
void LKTracker::drawTrack(const cv::Mat &imLeft,
                          const cv::Mat &imRight,
                          vector<int> &curLeftIds,
                          vector<cv::Point2f> &curLeftPts,
                          vector<cv::Point2f> &curRightPts,
                          map<int, cv::Point2f> &prevLeftPtsMap)
{
	//int rows = imLeft.rows;
	int cols = imLeft.cols;
	if (!imRight.empty() && stereo_cam) {
		cv::hconcat(imLeft, imRight, imTrack);
	}
	else {
		imTrack = imLeft.clone();
	}
	cv::cvtColor(imTrack, imTrack, cv::COLOR_GRAY2RGB);

	for (size_t j = 0; j < curLeftPts.size(); j++) {
		double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
		cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
	}
	if (!imRight.empty() && stereo_cam) {
		for (size_t i = 0; i < curRightPts.size(); i++) {
			cv::Point2f rightPt = curRightPts[i];
			rightPt.x += cols;
			cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
			//cv::Point2f leftPt = curLeftPtsTrackRight[i];
			//cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
		}
	}

	map<int, cv::Point2f>::iterator mapIt;
	for (size_t i = 0; i < curLeftIds.size(); i++) {
		int id = curLeftIds[i];
		mapIt = prevLeftPtsMap.find(id);
		if (mapIt != prevLeftPtsMap.end()) {
			cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
		}
	}

	//draw prediction
	/*
	for(size_t i = 0; i < predict_pts_debug.size(); i++)
	{
		cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
	}
	*/
	//printf("predict pts size %d \n", (int)predict_pts_debug.size());

	//cv::Mat imCur2Compress;
	//cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
	std_msgs::Header header; // empty header
	header.stamp = ros::Time::now();
	cv_bridge::CvImage img_bridge(header, sensor_msgs::image_encodings::BGR8, imTrack);
	pTrack_img_pub->publish(*img_bridge.toImageMsg());
}
map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> LKTracker::trackImage(double _cur_time,
                                                                               const cv::Mat &_img,
                                                                               const cv::Mat &_img1)
{
//	TicToc t_r;
	bool hasPrediction = false;
	cur_time = _cur_time;
	cur_img = _img;
	row = cur_img.rows;
	col = cur_img.cols;
	cv::Mat rightImg = _img1;

	/*
	{
		cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
		clahe->apply(cur_img, cur_img);
		if(!rightImg.empty())
			clahe->apply(rightImg, rightImg);
	}
	*/
	cur_pts.clear();

	if (prev_pts.size() > 0) {
		vector<uchar> status;
		vector<float> err;
		if (hasPrediction) {
			cur_pts = predict_pts;
			cv::calcOpticalFlowPyrLK(prev_img,
			                         cur_img,
			                         prev_pts,
			                         cur_pts,
			                         status,
			                         err,
			                         cv::Size(21, 21),
			                         1,
			                         cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
			                         cv::OPTFLOW_USE_INITIAL_FLOW);

			int succ_num = 0;
			for (size_t i = 0; i < status.size(); i++) {
				if (status[i]) {
					succ_num++;
				}
			}
			if (succ_num < 10) {
				cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
			}
		}
		else {
			cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
		}
		// reverse check
		if (FLOW_BACK) {
			vector<uchar> reverse_status;
			vector<cv::Point2f> reverse_pts = prev_pts;
			cv::calcOpticalFlowPyrLK(cur_img,
			                         prev_img,
			                         cur_pts,
			                         reverse_pts,
			                         reverse_status,
			                         err,
			                         cv::Size(21, 21),
			                         1,
			                         cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
			                         cv::OPTFLOW_USE_INITIAL_FLOW);
			//cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3);
			for (size_t i = 0; i < status.size(); i++) {
				if (status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5) {
					status[i] = 1;
				}
				else {
					status[i] = 0;
				}
			}
		}

		for (int i = 0; i < int(cur_pts.size()); i++)
			if (status[i] && !inBorder(cur_pts[i], col, row)) {
				status[i] = 0;
			}
		reduceVector(prev_pts, status);
		reduceVector(cur_pts, status);
		reduceVector(ids, status);
		reduceVector(track_cnt, status);
//		ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
		//printf("track cnt %d\n", (int)ids.size());
	}

	for (auto &n: track_cnt)
		n++;

	if (1) {
		//rejectWithF();
		ROS_DEBUG("set mask begins");
		// set a
		setMask();

		ROS_DEBUG("detect feature begins");
		int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
		if (n_max_cnt > 0) {
			if (mask.empty()) {
				cout << "mask is empty " << endl;
			}
			if (mask.type() != CV_8UC1) {
				cout << "mask type wrong " << endl;
			}
			cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
		}
		else {
			n_pts.clear();
		}

		// add new points
		for (auto &p: n_pts) {
			cur_pts.push_back(p);
			ids.push_back(n_id++);
			track_cnt.push_back(1);
		}
		//printf("feature cnt after add %d\n", (int)ids.size());
	}

	cur_un_pts = cur_pts;
	pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

	if (!_img1.empty() && stereo_cam) {
		ids_right.clear();
		cur_right_pts.clear();
		cur_un_right_pts.clear();
		right_pts_velocity.clear();
		cur_un_right_pts_map.clear();
		if (!cur_pts.empty()) {
			//printf("stereo image; track feature on right image\n");
			vector<cv::Point2f> reverseLeftPts;
			vector<uchar> status, statusRightLeft;
			vector<float> err;
			// cur left ---- cur right
			cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);
			// reverse check cur right ---- cur left
			if (FLOW_BACK) {
				cv::calcOpticalFlowPyrLK(rightImg,
				                         cur_img,
				                         cur_right_pts,
				                         reverseLeftPts,
				                         statusRightLeft,
				                         err,
				                         cv::Size(21, 21),
				                         3);
				for (size_t i = 0; i < status.size(); i++) {
					if (status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i], col, row)
						&& distance(cur_pts[i], reverseLeftPts[i]) <= 0.5) {
						status[i] = 1;
					}
					else {
						status[i] = 0;
					}
				}
			}

			ids_right = ids;
			reduceVector(cur_right_pts, status);
			reduceVector(ids_right, status);
			// only keep left-right pts
			/*
			reduceVector(cur_pts, status);
			reduceVector(ids, status);
			reduceVector(track_cnt, status);
			reduceVector(cur_un_pts, status);
			reduceVector(pts_velocity, status);
			*/
			cur_un_right_pts = cur_right_pts;
			right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
		}
		prev_un_right_pts_map = cur_un_right_pts_map;
	}
	if (SHOW_TRACK) {
		drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);
	}

	prev_img = cur_img;
	prev_pts = cur_pts;
	prev_un_pts = cur_un_pts;
	prev_un_pts_map = cur_un_pts_map;
	prev_time = cur_time;
	hasPrediction = false;

	prevLeftPtsMap.clear();
	for (size_t i = 0; i < cur_pts.size(); i++)
		prevLeftPtsMap[ids[i]] = cur_pts[i];

	map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
	for (size_t i = 0; i < ids.size(); i++) {
		int feature_id = ids[i];
		double x, y, z;
		x = cur_un_pts[i].x;
		y = cur_un_pts[i].y;
		z = 1;
		double p_u, p_v;
		p_u = cur_pts[i].x;
		p_v = cur_pts[i].y;
		int camera_id = 0;
		double velocity_x, velocity_y;
		velocity_x = pts_velocity[i].x;
		velocity_y = pts_velocity[i].y;

		Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
		// ??? difference between cur_un_pts and cur_pts
		xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
		featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
	}

	if (!_img1.empty() && stereo_cam) {
		for (size_t i = 0; i < ids_right.size(); i++) {
			int feature_id = ids_right[i];
			double x, y, z;
			x = cur_un_right_pts[i].x;
			y = cur_un_right_pts[i].y;
			z = 1;
			double p_u, p_v;
			p_u = cur_right_pts[i].x;
			p_v = cur_right_pts[i].y;
			int camera_id = 1;
			double velocity_x, velocity_y;
			velocity_x = right_pts_velocity[i].x;
			velocity_y = right_pts_velocity[i].y;

			Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
			xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
			featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
		}
	}

	//printf("feature track whole time %f\n", t_r.toc());
	return featureFrame;
}
vector<cv::Point2f> LKTracker::ptsVelocity(vector<int> &ids,
                                           vector<cv::Point2f> &pts,
                                           map<int, cv::Point2f> &cur_id_pts,
                                           map<int, cv::Point2f> &prev_id_pts)
{
	vector<cv::Point2f> pts_velocity;
	cur_id_pts.clear();
	for (unsigned int i = 0; i < ids.size(); i++) {
		cur_id_pts.insert(make_pair(ids[i], pts[i]));
	}

	// caculate points velocity
	if (!prev_id_pts.empty()) {
		double dt = cur_time - prev_time;

		for (unsigned int i = 0; i < pts.size(); i++) {
			std::map<int, cv::Point2f>::iterator it;
			it = prev_id_pts.find(ids[i]);
			if (it != prev_id_pts.end()) {
				double v_x = (pts[i].x - it->second.x) / dt;
				double v_y = (pts[i].y - it->second.y) / dt;
				pts_velocity.push_back(cv::Point2f(v_x, v_y));
			}
			else {
				pts_velocity.push_back(cv::Point2f(0, 0));
			}

		}
	}
	else {
		for (unsigned int i = 0; i < cur_pts.size(); i++) {
			pts_velocity.push_back(cv::Point2f(0, 0));
		}
	}
	return pts_velocity;
}
double LKTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
	//printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
	double dx = pt1.x - pt2.x;
	double dy = pt1.y - pt2.y;
	return sqrt(dx * dx + dy * dy);
}
void LKTracker::setMask()
{
	mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

	// prefer to keep features that are tracked for long time
	//(tracked_times, (point_2d, id))
	vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

	for (unsigned int i = 0; i < cur_pts.size(); i++)
		cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

	sort(cnt_pts_id.begin(),
	     cnt_pts_id.end(),
	     [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
	     {
		     return a.first > b.first;
	     });

	cur_pts.clear();
	ids.clear();
	track_cnt.clear();

	for (auto &it: cnt_pts_id) {
		// it.second.first 2d-point, pixel position of
		if (mask.at<uchar>(it.second.first) == 255) {
			cur_pts.push_back(it.second.first);
			ids.push_back(it.second.second);
			track_cnt.push_back(it.first);
			cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
		}
	}
}
bool LKTracker::inBorder(const cv::Point2f &pt, int col, int row)
{
	const int BORDER_SIZE = 1;
	int img_x = cvRound(pt.x);
	int img_y = cvRound(pt.y);
	return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}
void LKTracker::reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
	int j = 0;
	for (int i = 0; i < int(v.size()); i++)
		if (status[i]) {
			v[j++] = v[i];
		}
	v.resize(j);
}
void LKTracker::reduceVector(vector<int> &v, vector<uchar> status)
{
	int j = 0;
	for (int i = 0; i < int(v.size()); i++)
		if (status[i]) {
			v[j++] = v[i];
		}
	v.resize(j);
}
bool LKTracker::trackFrame(Frame &cur_frame, const Frame &prev_frame)
{
	/*matching steps
	 * 1. get pixel position of map points on previous frame and their prediction on current image
	 * 2. track pixel position of map points
	 * 3. link orb feature points extracted on current image to tracked points
	 * 4. adjust orb feature point position
	 * 5. update orb feature desciptor(optional)
	 * */


	cv::Mat cur_img_rgb = cur_frame.imgLeft.clone();
	cv::Mat prev_img_rgb = prev_frame.imgLeft.clone();
	if (cur_img_rgb.channels() > 1) {
		cv::cvtColor(cur_img_rgb, cur_img, cv::COLOR_BGR2GRAY);
	}
	if (prev_img_rgb.channels() > 1) {
		cv::cvtColor(prev_img_rgb, prev_img, cv::COLOR_BGR2GRAY);
	}

	prev_pts.clear();
	cur_pts.clear();
	vector<int> prev_feature_ids;
	vector<int> map_point_ids;


	// get camera pose
	int nmatches = 0;
	Eigen::Isometry3d T_cj_c0, T_ci_c0, T_c0_cj, T_c0_ci;
	cv::Mat Tcw, Tlw;
	Tcw = cur_frame.mTcw.clone();
	Tlw = prev_frame.mTcw.clone();
	if (!Tcw.empty()) {
//		cv2Eigen_3d(Tcw, T_cj_c0);
		cv::cv2eigen(Tcw, T_cj_c0.matrix());
	}
	else {
		ROS_ERROR_STREAM('LK_Tracker: cannot get current camera pose!');
		return false;
	}
	T_c0_cj = T_cj_c0.inverse();
	if (!Tlw.empty()) {
//		cv2Eigen_3d(Tlw, T_ci_c0);
		cv::cv2eigen(Tlw, T_ci_c0.matrix());
	}
	else {
		ROS_ERROR_STREAM('LK_Tracker: cannot get previous camera pose!');
		return false;
	}
	T_c0_ci = T_ci_c0.inverse();

	//get pixel position of map points on previous frame and their prediction on current image
	for (int i = 0; i < prev_frame.N; i++) {
		MapPoint *pMP = prev_frame.mvpMapPoints[i];
		if (pMP) {
			if (!prev_frame.mvbOutlier[i]) {
				//get map point 3D position in camera0 frame
				cv::Mat p_w_cv = pMP->GetWorldPos();
				//get map point 3D position in current camera frame
				Eigen::Vector3d p_c0(p_w_cv.at<float>(0), p_w_cv.at<float>(1), p_w_cv.at<float>(2));
				Eigen::Vector3d p_cj = T_cj_c0 * p_c0;
				cv::Mat p_c_cv = (cv::Mat_<float>(3, 1) << p_cj.x(), p_cj.y(), p_cj.z());
				//get pixel position on current image and previous image
				cv::Point2f uv_cur = cur_frame.mpCamera->project(p_c_cv);
				if (uv_cur.x < cur_frame.mnMinX || uv_cur.x > cur_frame.mnMaxX) {
					continue;
				}
				if (uv_cur.y < cur_frame.mnMinY || uv_cur.y > cur_frame.mnMaxY) {
					continue;
				}
				cur_pts.push_back(uv_cur);

				// add points
				if (i < prev_frame.Nleft || prev_frame.Nleft == -1) {
					cv::Point2f uv_pre_l = prev_frame.mvKeys[i].pt;
					prev_pts.push_back(uv_pre_l);
				}
				else {
					cv::Point2f uv_pre_r = prev_frame.mvKeysRight[i].pt;
					//todo_lktracker handle previous right
//					prev_pts.push_back(uv_pre_r);
				}

				prev_feature_ids.push_back(i);

			}
		}
	}

	// 2. track pixel position of map points
	/*todo_lktracker
	 * -add cur left to previous right
	 * -only match cur left to cur left here
	 */
	vector<uchar> status;
	vector<float> matching_err;
	cv::calcOpticalFlowPyrLK(prev_img,
	                         cur_img,
	                         prev_pts,
	                         cur_pts,
	                         status,
	                         matching_err,
	                         cv::Size(15, 15),
	                         1,
	                         cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
	                         cv::OPTFLOW_USE_INITIAL_FLOW);
	int succ_num = 0;
	for (size_t i = 0; i < status.size(); i++) {
		if (status[i]) {
			succ_num++;
		}
	}
	if (succ_num < 10) {
		cv::calcOpticalFlowPyrLK(prev_img,
		                         cur_img,
		                         prev_pts,
		                         cur_pts,
		                         status,
		                         matching_err,
		                         cv::Size(21, 21),
		                         3);
	}
//	ROS_INFO_STREAM("LKTracker: first tracked points =" << succ_num);

	if (FLOW_BACK) {
		vector<uchar> reverse_status;
		vector<cv::Point2f> reverse_pts = prev_pts;
		cv::calcOpticalFlowPyrLK(cur_img,
		                         prev_img,
		                         cur_pts,
		                         reverse_pts,
		                         reverse_status,
		                         matching_err,
		                         cv::Size(15, 15),
		                         3,
		                         cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
		                         cv::OPTFLOW_USE_INITIAL_FLOW);
		//cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3);
//		succ_num = 0;
//		for (size_t i = 0; i < reverse_status.size(); i++) {
//			if (reverse_status[i]) {
//				succ_num++;
//			}
//		}
//		ROS_INFO_STREAM("LKTracker: BackFlow tracked points =" << succ_num);
		for (size_t i = 0; i < status.size(); i++) {
			double dis = distance(prev_pts[i], reverse_pts[i]);
//			ROS_INFO_STREAM("LKTracker: BackFlow tracked points distance=" << dis);
			if (status[i] && reverse_status[i] && dis <= 1.0) {
				status[i] = 1;
			}
			else {
				status[i] = 0;
			}
		}
//		succ_num = 0;
//		for (size_t i = 0; i < status.size(); i++) {
//			if (status[i]) {
//				succ_num++;
//			}
//		}
//		ROS_INFO_STREAM("LKTracker: Foward-BackFlow tracked points =" << succ_num);
	}

	//3. link orb feature points extracted on current image to tracked points
	//4. adjust orb feature point position
	for (int i = 0; i < status.size(); i++) {
		if (status[i]) {
			cv::Point2f uv_cur_tracked = cur_pts[i];
			unsigned long best_index = 0;
			double best_distance = 256;
			auto indices = cur_frame.GetFeaturesInArea(uv_cur_tracked.x, uv_cur_tracked.y, 30);
			if (indices.empty()) {
				status[i] = 0;
				continue;
			}
			for (auto index: indices) {
				if (i < cur_frame.Nleft || cur_frame.Nleft == -1) {
					cv::Point2f uv_cur_orb = cur_frame.mvKeys[i].pt;
					double dis = distance(uv_cur_tracked, uv_cur_orb);
					if (dis < best_distance) {
						best_index = i;
						best_distance = dis;
					}
				}
			}
			if (best_index == 0) {
				status[i] = 0;
				continue;
			}

			cur_frame.mvpMapPoints[best_index] = prev_frame.mvpMapPoints[prev_feature_ids[i]];
			cur_frame.mvKeys[best_index].pt = uv_cur_tracked;
			nmatches++;
		}
	}
//	ROS_INFO_STREAM("LKTracker: final tracked points =" << nmatches);

	// draw and publish track result
//	drawTrackFrame(cur_frame, prev_frame, imTrack);
//	std_msgs::Header header; // empty header
//	header.stamp = ros::Time::now();
//	cv_bridge::CvImage img_bridge(header, sensor_msgs::image_encodings::BGR8, imTrack);
//	pTrack_img_pub->publish(*img_bridge.toImageMsg());

	return nmatches > 10;
}
void LKTracker::cv2Eigen_3d(const cv::Mat &T, Eigen::Isometry3d &T_eigen)
{
	cv::cv2eigen(T, T_eigen.matrix());
}
void LKTracker::eigen2CV_3d(const Eigen::Isometry3d &T_eigen, cv::Mat &T)
{
	cv::eigen2cv(T_eigen.matrix(), T);
}
void LKTracker::drawTrackFrame(Frame &cur_frame, const Frame &prev_frame, cv::Mat &result)
{
	cv::Mat result_cur = cur_frame.imgLeft.clone();
	cv::Mat result_prev = prev_frame.imgLeft.clone();
	if (result_cur.channels() == 1) {
		cv::cvtColor(result_cur, result_cur, cv::COLOR_GRAY2BGR);
	}
	if (result_prev.channels() == 1) {
		cv::cvtColor(result_prev, result_prev, cv::COLOR_GRAY2BGR);
	}

	for (int i = 0; i < cur_frame.N; i++) {
		auto p = cur_frame.mvpMapPoints[i];
		if (p) {
			cv::Point2f uv = cur_frame.mvKeys[i].pt;
			cv::circle(result_cur, uv, 3, cv::Scalar(0, 255, 0), 2);
//			cv::circle(result_cur, uv, 3, cv::Scalar(0, 255, 0), 2);
			cv::putText(result_cur, to_string(p->mnId), uv, cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2);
		}
	}
	for (int i = 0; i < prev_frame.N; i++) {
		auto p = prev_frame.mvpMapPoints[i];
		if (p) {
			cv::Point2f uv = prev_frame.mvKeys[i].pt;
			cv::circle(result_prev, uv, 3, cv::Scalar(0, 255, 0), 2);
//			cv::circle(result_prev, uv, 3, cv::Scalar(0, 255, 0), 2);
			cv::putText(result_prev, to_string(p->mnId), uv, cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2);
		}
	}
	for (auto p: cur_pts) {
		cv::circle(result_cur, p, 3, cv::Scalar(255, 0, 0), 1);
	}
	for (auto p: prev_pts) {
		cv::circle(result_prev, p, 3, cv::Scalar(255, 0, 0), 1);
	}
	cv::hconcat(result_prev, result_cur, result);
}
bool LKTracker::TrackReferenceKeyFrameKLT(KeyFrame *pKF, const Frame &cur_frame)
{
	/*matching steps
	 * 1. get pixel position of map points on previous frame and their prediction on current image
	 * 2. track pixel position of map points
	 * 3. link orb feature points extracted on current image to tracked points
	 * 4. adjust orb feature point position
	 * 5. update orb feature desciptor(optional)
	 * */


	cv::Mat cur_img_rgb = cur_frame.imgLeft.clone();
	cv::Mat prev_img_rgb = pKF->imgLeft.clone();
	if (cur_img_rgb.channels() > 1) {
		cv::cvtColor(cur_img_rgb, cur_img, cv::COLOR_BGR2GRAY);
	}
	if (prev_img_rgb.channels() > 1) {
		cv::cvtColor(prev_img_rgb, prev_img, cv::COLOR_BGR2GRAY);
	}

	prev_pts.clear();
	cur_pts.clear();
	vector<int> prev_feature_ids;
	vector<int> map_point_ids;


	// get camera pose
	int nmatches = 0;
	Eigen::Isometry3d T_cj_c0, T_ci_c0, T_c0_cj, T_c0_ci;
	cv::Mat Tcw, Tlw;
	Tcw = cur_frame.mTcw.clone();
	// Tcw
	Tlw = pKF->GetPose();
	if (!Tcw.empty()) {
//		cv2Eigen_3d(Tcw, T_cj_c0);
		cv::cv2eigen(Tcw, T_cj_c0.matrix());
	}
	else {
		ROS_ERROR_STREAM('LK_Tracker: cannot get current camera pose!');
		return false;
	}
	T_c0_cj = T_cj_c0.inverse();
	if (!Tlw.empty()) {
//		cv2Eigen_3d(Tlw, T_ci_c0);
		cv::cv2eigen(Tlw, T_ci_c0.matrix());
	}
	else {
		ROS_ERROR_STREAM('LK_Tracker: cannot get previous camera pose!');
		return false;
	}
	T_c0_ci = T_ci_c0.inverse();

	//get pixel position of map points on previous frame and their prediction on current image
	for (int i = 0; i < pKF->N; i++) {
		MapPoint *pMP = pKF->GetMapPoint(i);
		if (pMP) {
			if (!pMP->isBad()) {
				//get map point 3D position in camera0 frame
				cv::Mat p_w_cv = pMP->GetWorldPos();
				//get map point 3D position in current camera frame
				Eigen::Vector3d p_c0(p_w_cv.at<float>(0), p_w_cv.at<float>(1), p_w_cv.at<float>(2));
				Eigen::Vector3d p_cj = T_cj_c0 * p_c0;
				cv::Mat p_c_cv = (cv::Mat_<float>(3, 1) << p_cj.x(), p_cj.y(), p_cj.z());
				//get pixel position on current image and previous image
				cv::Point2f uv_cur = cur_frame.mpCamera->project(p_c_cv);
				if (uv_cur.x < cur_frame.mnMinX || uv_cur.x > cur_frame.mnMaxX) {
					continue;
				}
				if (uv_cur.y < cur_frame.mnMinY || uv_cur.y > cur_frame.mnMaxY) {
					continue;
				}
				cur_pts.push_back(uv_cur);

				// add points
				if (i < pKF->NLeft || pKF->NLeft == -1) {
					cv::Point2f uv_pre_l = pKF->mvKeys[i].pt;
					prev_pts.push_back(uv_pre_l);
				}
				else {
					cv::Point2f uv_pre_r = pKF->mvKeysRight[i].pt;
					//todo_lktracker handle previous right
//					prev_pts.push_back(uv_pre_r);
				}

				prev_feature_ids.push_back(i);

			}
		}
	}

	// 2. track pixel position of map points
	/*todo_lktracker
	 * -add cur left to previous right
	 * -only match cur left to cur left here
	 */
	vector<uchar> status;
	vector<float> matching_err;
	cv::calcOpticalFlowPyrLK(prev_img,
	                         cur_img,
	                         prev_pts,
	                         cur_pts,
	                         status,
	                         matching_err,
	                         cv::Size(15, 15),
	                         1,
	                         cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
	                         cv::OPTFLOW_USE_INITIAL_FLOW);
	int succ_num = 0;
	for (size_t i = 0; i < status.size(); i++) {
		if (status[i]) {
			succ_num++;
		}
	}
	if (succ_num < 10) {
		cv::calcOpticalFlowPyrLK(prev_img,
		                         cur_img,
		                         prev_pts,
		                         cur_pts,
		                         status,
		                         matching_err,
		                         cv::Size(21, 21),
		                         3);
	}
//	ROS_INFO_STREAM("LKTracker: first tracked points =" << succ_num);

	if (FLOW_BACK) {
		vector<uchar> reverse_status;
		vector<cv::Point2f> reverse_pts = prev_pts;
		cv::calcOpticalFlowPyrLK(cur_img,
		                         prev_img,
		                         cur_pts,
		                         reverse_pts,
		                         reverse_status,
		                         matching_err,
		                         cv::Size(15, 15),
		                         3,
		                         cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
		                         cv::OPTFLOW_USE_INITIAL_FLOW);
		//cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3);
//		succ_num = 0;
//		for (size_t i = 0; i < reverse_status.size(); i++) {
//			if (reverse_status[i]) {
//				succ_num++;
//			}
//		}
//		ROS_INFO_STREAM("LKTracker: BackFlow tracked points =" << succ_num);
		for (size_t i = 0; i < status.size(); i++) {
			double dis = distance(prev_pts[i], reverse_pts[i]);
//			ROS_INFO_STREAM("LKTracker: BackFlow tracked points distance=" << dis);
			if (status[i] && reverse_status[i] && dis <= 1.0) {
				status[i] = 1;
			}
			else {
				status[i] = 0;
			}
		}
//		succ_num = 0;
//		for (size_t i = 0; i < status.size(); i++) {
//			if (status[i]) {
//				succ_num++;
//			}
//		}
//		ROS_INFO_STREAM("LKTracker: Foward-BackFlow tracked points =" << succ_num);
	}

	//3. link orb feature points extracted on current image to tracked points
	//4. adjust orb feature point position
//	for (int i = 0; i < status.size(); i++) {
//		if (status[i]) {
//			cv::Point2f uv_cur_tracked = cur_pts[i];
//			unsigned long best_index = 0;
//			double best_distance = 256;
//			auto indices = cur_frame.GetFeaturesInArea(uv_cur_tracked.x, uv_cur_tracked.y, 30);
//			if (indices.empty()) {
//				status[i] = 0;
//				continue;
//			}
//			for (auto index: indices) {
//				if (i < cur_frame.Nleft || cur_frame.Nleft == -1) {
//					cv::Point2f uv_cur_orb = cur_frame.mvKeys[i].pt;
//					double dis = distance(uv_cur_tracked, uv_cur_orb);
//					if (dis < best_distance) {
//						best_index = i;
//						best_distance = dis;
//					}
//				}
//			}
//			if (best_index == 0) {
//				status[i] = 0;
//				continue;
//			}
//
//			cur_frame.mvpMapPoints[best_index] = prev_frame.mvpMapPoints[prev_feature_ids[i]];
//			cur_frame.mvKeys[best_index].pt = uv_cur_tracked;
//			nmatches++;
//		}
//	}

//	ROS_INFO_STREAM("LKTracker: final tracked points =" << nmatches);

	// draw and publish track result
//	drawTrackFrame(cur_frame, prev_frame, imTrack);
//	std_msgs::Header header; // empty header
//	header.stamp = ros::Time::now();
//	cv_bridge::CvImage img_bridge(header, sensor_msgs::image_encodings::BGR8, imTrack);
//	pTrack_img_pub->publish(*img_bridge.toImageMsg());

	return nmatches > 10;
}
bool LKTracker::InitializeFrame(Frame &cur_f)
{
//	for (int i = 0; i < cur_f.N; i++) {
//		float z = cur_f.mvDepth[i];
//		if (z > 0) {
//			// x3D.at<fload>(col,row)
//			cv::Mat x3D = cur_f.UnprojectStereo(i);
//			// calculate the map point in world coordinate
//			MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
//			pNewMP->AddObservation(pKFini, i);
//			pKFini->AddMapPoint(pNewMP, i);
//			pNewMP->ComputeDistinctiveDescriptors();
//			pNewMP->UpdateNormalAndDepth();
//			mpAtlas->AddMapPoint(pNewMP);
//
//			mCurrentFrame.mvpMapPoints[i] = pNewMP;
//		}
//	}
}
void LKTracker::DetectORBPoints(FrameKLT &f)
{

}
void LKTracker::DetectFeature(FrameKLT &f, int max_num)
{
	cv::Mat img_l = f.imgLeft.clone(), img_r = f.imgRight.clone();
	if (img_l.channels() > 1) {
		cv::cvtColor(img_l, img_l, cv::COLOR_BGR2GRAY);
	}
	if (img_r.channels() > 1) {
		cv::cvtColor(img_r, img_r, cv::COLOR_BGR2GRAY);
	}
	cv::Ptr<cv::ORB> detector = cv::ORB::create(max_num * 5);
	vector<cv::KeyPoint> kp_l, kp_r;
	detector->detect(img_l, kp_l);
	detector->detect(img_r, kp_r);
	ssc(kp_l, max_num, 0.1, img_l.cols, img_r.rows, f.mKP_l);
	ssc(kp_r, max_num, 0.1, img_r.cols, img_r.rows, f.mKP_r);

}
void LKTracker::ComputeFeatureDescriptor(FrameKLT &f, int max_num)
{
	cv::Mat img_l = f.imgLeft.clone(), img_r = f.imgRight.clone();
	if (img_l.channels() > 1) {
		cv::cvtColor(img_l, img_l, cv::COLOR_BGR2GRAY);
	}
	if (img_r.channels() > 1) {
		cv::cvtColor(img_r, img_r, cv::COLOR_BGR2GRAY);
	}

	cv::Ptr<cv::ORB> detector = cv::ORB::create(max_num);

	detector->compute(img_l, f.mKP_l, f.mDsp_l);
	detector->compute(img_r, f.mKP_r, f.mDsp_r);
}

void LKTracker::ssc(vector<cv::KeyPoint> keyPoints, int numRetPoints,
                    float tolerance, int cols, int rows, vector<cv::KeyPoint> &out)
{
	// several temp expression variables to simplify solution equation
	int exp1 = rows + cols + 2 * numRetPoints;
	long long exp2 =
		((long long)4 * cols + (long long)4 * numRetPoints +
			(long long)4 * rows * numRetPoints + (long long)rows * rows +
			(long long)cols * cols - (long long)2 * rows * cols +
			(long long)4 * rows * cols * numRetPoints);
	double exp3 = sqrt(exp2);
	double exp4 = numRetPoints - 1;

	double sol1 = -round((exp1 + exp3) / exp4); // first solution
	double sol2 = -round((exp1 - exp3) / exp4); // second solution

	// binary search range initialization with positive solution
	int high = (sol1 > sol2) ? sol1 : sol2;
	int low = floor(sqrt((double)keyPoints.size() / numRetPoints));
	low = max(1, low);

	int width;
	int prevWidth = -1;

	vector<int> ResultVec;
	bool complete = false;
	unsigned int K = numRetPoints;
	unsigned int Kmin = round(K - (K * tolerance));
	unsigned int Kmax = round(K + (K * tolerance));

	vector<int> result;
	result.reserve(keyPoints.size());
	while (!complete) {
		width = low + (high - low) / 2;
		if (width == prevWidth ||
			low >
				high) { // needed to reassure the same radius is not repeated again
			ResultVec = result; // return the keypoints from the previous iteration
			break;
		}
		result.clear();
		double c = (double)width / 2.0; // initializing Grid
		int numCellCols = floor(cols / c);
		int numCellRows = floor(rows / c);
		vector<vector<bool>> coveredVec(numCellRows + 1,
		                                vector<bool>(numCellCols + 1, false));

		for (unsigned int i = 0; i < keyPoints.size(); ++i) {
			int row =
				floor(keyPoints[i].pt.y /
					c); // get position of the cell current point is located at
			int col = floor(keyPoints[i].pt.x / c);
			if (coveredVec[row][col] == false) { // if the cell is not covered
				result.push_back(i);
				int rowMin = ((row - floor(width / c)) >= 0)
				             ? (row - floor(width / c))
				             : 0; // get range which current radius is covering
				int rowMax = ((row + floor(width / c)) <= numCellRows)
				             ? (row + floor(width / c))
				             : numCellRows;
				int colMin =
					((col - floor(width / c)) >= 0) ? (col - floor(width / c)) : 0;
				int colMax = ((col + floor(width / c)) <= numCellCols)
				             ? (col + floor(width / c))
				             : numCellCols;
				for (int rowToCov = rowMin; rowToCov <= rowMax; ++rowToCov) {
					for (int colToCov = colMin; colToCov <= colMax; ++colToCov) {
						if (!coveredVec[rowToCov][colToCov]) {
							coveredVec[rowToCov][colToCov] =
								true;
						} // cover cells within the square bounding box with width
						// w
					}
				}
			}
		}

		if (result.size() >= Kmin && result.size() <= Kmax) { // solution found
			ResultVec = result;
			complete = true;
		}
		else if (result.size() < Kmin) {
			high = width - 1; // update binary search range
		}
		else {
			low = width + 1;
		}
		prevWidth = width;
	}
	// retrieve final keypoints

	for (unsigned int i = 0; i < ResultVec.size(); i++)
		out.push_back(keyPoints[ResultVec[i]]);

}

}
