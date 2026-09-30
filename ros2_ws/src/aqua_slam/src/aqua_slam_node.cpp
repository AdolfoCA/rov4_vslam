/**
 * AQUA-SLAM stereo + DVL + gyro node, ROS 2.
 *
 * Port of upstream's src/ros_stereo_DVL_tighly.cc (ROS 1). The measurement handling is
 * kept as it was - in particular ImageGrabber::SyncWithImu(), which pairs the stereo
 * images, collects the IMU and DVL samples up to the left image stamp and hands them to
 * System::TrackStereoGroDVL() - so results stay comparable with the original. What
 * changed:
 *
 *  - rclcpp subscriptions, best-effort QoS to match the bluerov2 drivers.
 *  - The DVL input is bluerov2_msgs/DVLReport (our driver) instead of
 *    waterlinked_a50_ros_driver/DVL. Our driver rotates the velocity from the A50's FRD
 *    axes into ROS FLU by default; AQUA-SLAM's T_dvl_c and beam model are defined in
 *    the instrument's own FRD frame, so the velocity is rotated back (parameter
 *    dvl_velocity_frame). Reports with fewer than 4 beams are dropped rather than read
 *    out of bounds.
 *  - Optional stereo rectification (parameter rectify), as in the ORB-SLAM3 stereo
 *    examples, from LEFT.* / RIGHT.* in the settings file. Upstream's datasets were
 *    already rectified; the multi-camera streams are not.
 *  - Topics, file paths and frames are ROS parameters instead of hard-coded paths.
 *  - The sync loop sleeps when idle instead of spinning a core at 100 %.
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <bluerov2_msgs/msg/dvl_report.hpp>

#include "ImuTypes.h"
#include "System.h"
#include "ros1_compat.h"

using namespace std;
namespace logging = boost::log;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;

using ImageMsg = sensor_msgs::msg::Image;
using ImuMsg = sensor_msgs::msg::Imu;
using DvlMsg = bluerov2_msgs::msg::DVLReport;

namespace
{
double StampToSec(const builtin_interfaces::msg::Time &t)
{
	return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

/**
 * The core logs every frame's IMU/DVL bookkeeping through BOOST_LOG_TRIVIAL - a lot of
 * output. Upstream wrote it to a file in a hard-coded directory; here it goes to
 * <log_dir>/aqua_slam_%N.log if log_dir is set, and is discarded otherwise (boost's
 * default would print all of it to the console).
 */
void InitBoostLogging(const string &log_dir)
{
	if (log_dir.empty()) {
		logging::core::get()->set_logging_enabled(false);
		return;
	}
	logging::add_file_log(
		keywords::file_name = log_dir + "/aqua_slam_%N.log",
		keywords::rotation_size = 20 * 1024 * 1024,
		keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0),
		keywords::format = "%Message%",
		keywords::auto_flush = true);
	logging::core::get()->set_filter(logging::trivial::severity >= logging::trivial::info);
}
}  // namespace

class ImuGrabber
{
public:
	void GrabImu(const ImuMsg::ConstSharedPtr &imu_msg)
	{
		std::lock_guard<std::mutex> lock(mBufMutex);
		imuBuf.push(imu_msg);
	}

	queue<ImuMsg::ConstSharedPtr> imuBuf;
	std::mutex mBufMutex;
};

class DVLGrabber
{
public:
	explicit DVLGrabber(bool velocity_is_flu) : mVelocityIsFlu(velocity_is_flu) {}

	void GrabDVL(const DvlMsg::ConstSharedPtr &msg)
	{
		if (msg->beams.size() < 4) {
			BOOST_LOG_TRIVIAL(warning) << "skip DVL report with " << msg->beams.size() << " beams";
			return;
		}
		if (!mVelocityIsFlu) {
			std::lock_guard<std::mutex> lock(mBufMutex);
			dvlBuf.push(msg);
			return;
		}
		// FLU -> FRD: 180 degrees about x. Beam velocities are along-beam scalars and
		// need no rotation.
		auto frd = std::make_shared<DvlMsg>(*msg);
		frd->velocity.y = -msg->velocity.y;
		frd->velocity.z = -msg->velocity.z;
		std::lock_guard<std::mutex> lock(mBufMutex);
		dvlBuf.push(frd);
	}

	queue<DvlMsg::ConstSharedPtr> dvlBuf;
	std::mutex mBufMutex;

private:
	bool mVelocityIsFlu;
};

class ImageGrabber
{
public:
	ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, DVLGrabber *pDvlGb)
		: mpSLAM(pSLAM), mpImuGb(pImuGb), mpDvlGb(pDvlGb)
	{}

	/** Loads LEFT.* / RIGHT.* from the settings file; returns false if any is missing. */
	bool InitRectification(const string &settings_path);

	void GrabImageLeft(const ImageMsg::ConstSharedPtr &msg)
	{
		std::lock_guard<std::mutex> lock(mBufMutexLeft);
		imgLeftBuf.push(msg);
	}
	void GrabImageRight(const ImageMsg::ConstSharedPtr &msg)
	{
		std::lock_guard<std::mutex> lock(mBufMutexRight);
		imgRightBuf.push(msg);
	}
	cv::Mat GetImage(const ImageMsg::ConstSharedPtr &img_msg);
	void SyncWithImu();

	queue<ImageMsg::ConstSharedPtr> imgLeftBuf, imgRightBuf;
	std::mutex mBufMutexLeft, mBufMutexRight;

	ORB_SLAM3::System *mpSLAM;
	ImuGrabber *mpImuGb;
	DVLGrabber *mpDvlGb;

	bool do_rectify = false;
	cv::Mat M1l, M2l, M1r, M2r;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("aqua_slam");

	// --- parameters ---------------------------------------------------------------
	const string voc_path = node->declare_parameter<string>("vocabulary_path", "/opt/aqua_slam/Vocabulary/ORBvoc.txt");
	const string settings_path = node->declare_parameter<string>("settings_path", "");
	const string left_topic = node->declare_parameter<string>("left_image_topic", "/bluerov2/multicam/aux_left/image_raw");
	const string right_topic = node->declare_parameter<string>("right_image_topic", "/bluerov2/multicam/aux_right/image_raw");
	const string imu_topic = node->declare_parameter<string>("imu_topic", "/bluerov2/imu/data_raw");
	const string dvl_topic = node->declare_parameter<string>("dvl_topic", "/bluerov2/dvl/report");
	// "raw" in the same container; "compressed" if the images come over a slow link.
	const string transport = node->declare_parameter<string>("image_transport", "raw");
	// Frame of the DVLReport velocity: "flu" if bluerov2_dvl runs with rotate_to_flu
	// (its default), "frd" if it publishes the instrument's native axes.
	const string dvl_frame = node->declare_parameter<string>("dvl_velocity_frame", "flu");
	const bool rectify = node->declare_parameter<bool>("rectify", false);
	const string log_dir = node->declare_parameter<string>("log_dir", "");
	// read by the SLAM core (RosHandling, System, DenseMapper)
	node->declare_parameter<string>("map_frame", "aqua_slam_map");
	node->declare_parameter<string>("body_frame", "aqua_slam_camera");
	node->declare_parameter<bool>("is_load_map", false);
	node->declare_parameter<string>("out_path", "");
	node->declare_parameter<string>("traj_path", "");
	node->declare_parameter<string>("map_file", "");

	if (settings_path.empty()) {
		RCLCPP_FATAL(node->get_logger(), "parameter settings_path is required");
		return 1;
	}
	if (dvl_frame != "flu" && dvl_frame != "frd") {
		RCLCPP_FATAL(node->get_logger(), "dvl_velocity_frame must be 'flu' or 'frd', got '%s'", dvl_frame.c_str());
		return 1;
	}

	InitBoostLogging(log_dir);
	aqua_slam::SetNode(node);

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM3::System SLAM(voc_path, settings_path, ORB_SLAM3::System::DVL_STEREO, false);
	ImuGrabber imugb;
	DVLGrabber dvlgb(dvl_frame == "flu");
	ImageGrabber igb(&SLAM, &imugb, &dvlgb);

	if (rectify) {
		if (!igb.InitRectification(settings_path)) {
			RCLCPP_FATAL(node->get_logger(), "rectify:=true but LEFT.* / RIGHT.* calibration is missing in %s",
			             settings_path.c_str());
			return 1;
		}
		igb.do_rectify = true;
	}

	// The drivers publish best-effort (sensor data QoS); a reliable subscription would
	// never match them. Queues are deep enough for the IMU at 100 Hz.
	auto sensor_qos = rclcpp::SensorDataQoS().keep_last(200);
	auto sub_imu = node->create_subscription<ImuMsg>(
		imu_topic, sensor_qos, [&imugb](ImuMsg::ConstSharedPtr m) { imugb.GrabImu(m); });
	auto sub_dvl = node->create_subscription<DvlMsg>(
		dvl_topic, sensor_qos, [&dvlgb](DvlMsg::ConstSharedPtr m) { dvlgb.GrabDVL(m); });

	rmw_qos_profile_t image_qos = rmw_qos_profile_sensor_data;
	image_qos.depth = 10;
	auto sub_l = image_transport::create_subscription(
		node.get(), left_topic, [&igb](const ImageMsg::ConstSharedPtr &m) { igb.GrabImageLeft(m); }, transport, image_qos);
	auto sub_r = image_transport::create_subscription(
		node.get(), right_topic, [&igb](const ImageMsg::ConstSharedPtr &m) { igb.GrabImageRight(m); }, transport, image_qos);

	RCLCPP_INFO(node->get_logger(),
	            "AQUA-SLAM running\n  left  %s\n  right %s\n  imu   %s\n  dvl   %s (velocity in %s)\n  transport %s, rectify %s",
	            left_topic.c_str(), right_topic.c_str(), imu_topic.c_str(), dvl_topic.c_str(), dvl_frame.c_str(),
	            transport.c_str(), rectify ? "on" : "off");

	std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

	// Callbacks on several threads: the SLAM core publishes and serves ~/save etc.
	// from its own threads, and a long service call must not stall the sensor queues.
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();

	rclcpp::shutdown();
	sync_thread.join();
	// The ORB-SLAM3 worker threads (local mapping, loop closing, map publishing) loop
	// forever and are never joined upstream either; leave without running static
	// destructors underneath them.
	std::quick_exit(0);
}

bool ImageGrabber::InitRectification(const string &settings_path)
{
	cv::FileStorage fs(settings_path, cv::FileStorage::READ);
	cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
	fs["LEFT.K"] >> K_l;
	fs["RIGHT.K"] >> K_r;
	fs["LEFT.P"] >> P_l;
	fs["RIGHT.P"] >> P_r;
	fs["LEFT.R"] >> R_l;
	fs["RIGHT.R"] >> R_r;
	fs["LEFT.D"] >> D_l;
	fs["RIGHT.D"] >> D_r;
	const int rows_l = fs["LEFT.height"];
	const int cols_l = fs["LEFT.width"];
	const int rows_r = fs["RIGHT.height"];
	const int cols_r = fs["RIGHT.width"];
	if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() ||
	    D_r.empty() || rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
		return false;
	}
	cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F,
	                            M1l, M2l);
	cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F,
	                            M1r, M2r);
	return true;
}

cv::Mat ImageGrabber::GetImage(const ImageMsg::ConstSharedPtr &img_msg)
{
	// Copy the ros image message to cv::Mat.
	try {
		return cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8)->image.clone();
	}
	catch (cv_bridge::Exception &e) {
		RCLCPP_ERROR(aqua_slam::GetLogger(), "cv_bridge exception: %s", e.what());
		return cv::Mat();
	}
}

void ImageGrabber::SyncWithImu()
{
	const double maxTimeDiff = 0.1;
	while (rclcpp::ok()) {
		cv::Mat imLeft, imRight;
		double tImLeft = 0, tImRight = 0;
		bool have_data;
		{
			std::scoped_lock lock(mBufMutexLeft, mBufMutexRight, mpImuGb->mBufMutex);
			have_data = !imgLeftBuf.empty() && !imgRightBuf.empty() && !mpImuGb->imuBuf.empty();
		}
		if (!have_data) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}

		{
			std::scoped_lock lock(mBufMutexLeft, mBufMutexRight);
			tImLeft = StampToSec(imgLeftBuf.front()->header.stamp);
			tImRight = StampToSec(imgRightBuf.front()->header.stamp);

			// right timestamp is too smaller than left
			while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1) {
				imgRightBuf.pop();
				tImRight = StampToSec(imgRightBuf.front()->header.stamp);
			}
			// left timestamp is too smaller than right
			while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1) {
				imgLeftBuf.pop();
				tImLeft = StampToSec(imgLeftBuf.front()->header.stamp);
			}
		}

		// cannot find image with good timestamp diff, waiting for the following image
		if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff) {
			BOOST_LOG_TRIVIAL(warning) << "big time difference bwtween left and right img\nleft img:" << tImLeft
			                           << "\nright img:" << tImRight;
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}

		// keep saving IMU data until the timestamp of lasted IMU data > the timestamp of lasted Image
		{
			std::lock_guard<std::mutex> lock(mpImuGb->mBufMutex);
			if (tImLeft > StampToSec(mpImuGb->imuBuf.back()->header.stamp)) {
				have_data = false;
			}
		}
		if (!have_data) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}

		{
			std::lock_guard<std::mutex> lock(mBufMutexLeft);
			imLeft = GetImage(imgLeftBuf.front());
			imgLeftBuf.pop();
		}
		{
			std::lock_guard<std::mutex> lock(mBufMutexRight);
			imRight = GetImage(imgRightBuf.front());
			imgRightBuf.pop();
		}
		if (imLeft.empty() || imRight.empty()) {
			continue;
		}
		if (do_rectify) {
			cv::remap(imLeft, imLeft, M1l, M2l, cv::INTER_LINEAR);
			cv::remap(imRight, imRight, M1r, M2r, cv::INTER_LINEAR);
		}

		vector<ORB_SLAM3::IMU::ImuPoint> vImuMeas;
		vector<ORB_SLAM3::IMU::DvlPoint> vDVLMeas;
		vector<ORB_SLAM3::IMU::GyroDvlPoint> vGyroDVLMeas;
		{
			std::lock_guard<std::mutex> lock(mpImuGb->mBufMutex);
			// save all IMU data between last Frame and current Frame to current Frame
			while (!mpImuGb->imuBuf.empty() && StampToSec(mpImuGb->imuBuf.front()->header.stamp) <= tImLeft) {
				const auto &m = mpImuGb->imuBuf.front();
				double t = StampToSec(m->header.stamp);
				cv::Point3f acc(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z);
				cv::Point3f gyr(m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z);
				vImuMeas.push_back(ORB_SLAM3::IMU::ImuPoint(acc, gyr, t));
				vGyroDVLMeas.push_back(
					ORB_SLAM3::IMU::GyroDvlPoint(acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z, 0, 0, 0, 0, 0, 0, 0, t));
				mpImuGb->imuBuf.pop();
			}
		}
		{
			std::lock_guard<std::mutex> lock(mpDvlGb->mBufMutex);
			while (!mpDvlGb->dvlBuf.empty() && StampToSec(mpDvlGb->dvlBuf.front()->header.stamp) <= tImLeft) {
				const auto &m = mpDvlGb->dvlBuf.front();
				double t = StampToSec(m->header.stamp);
				//skip bad DVL
				if (!m->velocity_valid) {
					BOOST_LOG_TRIVIAL(warning) << "skip bad DVL data";
					mpDvlGb->dvlBuf.pop();
					continue;
				}
				Eigen::Vector3d v(m->velocity.x, m->velocity.y, m->velocity.z);
				if (v.norm() > 1.0) {
					BOOST_LOG_TRIVIAL(warning) << std::fixed << std::setprecision(9) << "skip bad DVL data time:" << t
					                           << " v:" << v.transpose();
					mpDvlGb->dvlBuf.pop();
					continue;
				}
				vDVLMeas.push_back(ORB_SLAM3::IMU::DvlPoint(m->velocity.x, m->velocity.y, m->velocity.z,
				                                            m->beams[0].velocity, m->beams[1].velocity,
				                                            m->beams[2].velocity, m->beams[3].velocity, t));
				vGyroDVLMeas.push_back(ORB_SLAM3::IMU::GyroDvlPoint(0, 0, 0, m->velocity.x, m->velocity.y,
				                                                    m->velocity.z, m->beams[0].velocity,
				                                                    m->beams[1].velocity, m->beams[2].velocity,
				                                                    m->beams[3].velocity, t));
				mpDvlGb->dvlBuf.pop();
			}
		}

		while (vDVLMeas.size() >= 2) {
			BOOST_LOG_TRIVIAL(warning) << "there are two DVL measurement between Frames!";
			vDVLMeas.pop_back();
		}

		if (vImuMeas.empty()) {
			BOOST_LOG_TRIVIAL(warning) << "no IMU measurement between Frames!";
			continue;
		}

		//sort by timestamp
		sort(vGyroDVLMeas.begin(), vGyroDVLMeas.end(),
		     [](const ORB_SLAM3::IMU::GyroDvlPoint &a, const ORB_SLAM3::IMU::GyroDvlPoint &b) { return a.t < b.t; });

		BOOST_LOG_TRIVIAL(info) << fixed << setprecision(9) << "Track Stereo!\n"
		                        << "left image time: " << tImLeft;
		mpSLAM->TrackStereoGroDVL(imLeft, imRight, tImLeft, vGyroDVLMeas, !vDVLMeas.empty());
	}
}
