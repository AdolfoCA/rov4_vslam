/**
 * ROS 1 -> ROS 2 compatibility layer for the AQUA-SLAM core.
 *
 * The upstream code is ROS 1 (Noetic), and ROS calls are spread through the SLAM core
 * itself (Tracking, LoopClosing, Optimizer, ...), mostly as ROS_*_STREAM logging,
 * ros::Time stamps and message type names. Rewriting ~400 of those call sites would
 * make every future merge from upstream painful, so this header maps just those
 * mechanical pieces onto ROS 2:
 *
 *   - ROS_{DEBUG,INFO,WARN,ERROR}[_STREAM]  ->  RCLCPP_* on the "aqua_slam" logger
 *   - ros::Time, ros::Duration              ->  small value types that convert to
 *                                               builtin_interfaces Time / Duration
 *   - std_msgs::Header, sensor_msgs::Image, ...  ->  aliases of the ROS 2 msg:: types
 *
 * Everything with real ROS semantics - node handles, publishers, services, TF,
 * parameters, subscriptions - is written against rclcpp directly, not shimmed.
 *
 * The SLAM threads publish through one shared node, set once by the executable with
 * aqua_slam::SetNode() before the ORB_SLAM3::System is constructed.
 */

#ifndef AQUA_SLAM_ROS1_COMPAT_H
#define AQUA_SLAM_ROS1_COMPAT_H

#include <cmath>
#include <cstdint>

#include <boost/make_shared.hpp>  // ROS 1 headers used to pull this in for the core

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace aqua_slam
{
/** Sets the node the SLAM core publishes through. Call once, before building System. */
void SetNode(const rclcpp::Node::SharedPtr &node);

/** The node set by SetNode(); throws std::runtime_error if it was never set. */
rclcpp::Node::SharedPtr GetNode();

/** Logger for the SLAM core: the node's logger once set, "aqua_slam" before that. */
rclcpp::Logger GetLogger();

/** Current time from the node's clock (sim time aware), or the system clock before SetNode(). */
rclcpp::Time Now();
}  // namespace aqua_slam

namespace ros
{
/** Seconds since the epoch, as ROS 1's ros::Time; converts to a message stamp. */
class Time
{
public:
	Time() = default;
	explicit Time(double sec) : m_sec(sec) {}
	explicit Time(const rclcpp::Time &t) : m_sec(t.seconds()) {}

	static Time now() { return Time(aqua_slam::Now()); }

	double toSec() const { return m_sec; }
	uint64_t toNSec() const { return static_cast<uint64_t>(std::llround(m_sec * 1e9)); }

	operator builtin_interfaces::msg::Time() const
	{
		builtin_interfaces::msg::Time t;
		const int64_t ns = static_cast<int64_t>(std::llround(m_sec * 1e9));
		t.sec = static_cast<int32_t>(ns / 1000000000LL);
		t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
		return t;
	}

private:
	double m_sec = 0.0;
};

class Duration
{
public:
	explicit Duration(double sec) : m_sec(sec) {}

	operator builtin_interfaces::msg::Duration() const
	{
		builtin_interfaces::msg::Duration d;
		const int64_t ns = static_cast<int64_t>(std::llround(m_sec * 1e9));
		d.sec = static_cast<int32_t>(ns / 1000000000LL);
		d.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
		return d;
	}

private:
	double m_sec;
};
}  // namespace ros

// ROS 1 message type names used by the core, as aliases of the ROS 2 types.
namespace std_msgs
{
using Header = msg::Header;
}
namespace sensor_msgs
{
using Image = msg::Image;
using ImagePtr = msg::Image::SharedPtr;
using ImageConstPtr = msg::Image::ConstSharedPtr;
using Imu = msg::Imu;
using ImuConstPtr = msg::Imu::ConstSharedPtr;
using PointCloud2 = msg::PointCloud2;
}
namespace geometry_msgs
{
using PoseStamped = msg::PoseStamped;
}
namespace nav_msgs
{
using Odometry = msg::Odometry;
using OdometryConstPtr = msg::Odometry::ConstSharedPtr;
using Path = msg::Path;
}
namespace visualization_msgs
{
using Marker = msg::Marker;
using MarkerArray = msg::MarkerArray;
}

#define ROS_DEBUG_STREAM(args) RCLCPP_DEBUG_STREAM(aqua_slam::GetLogger(), args)
#define ROS_INFO_STREAM(args) RCLCPP_INFO_STREAM(aqua_slam::GetLogger(), args)
#define ROS_WARN_STREAM(args) RCLCPP_WARN_STREAM(aqua_slam::GetLogger(), args)
#define ROS_ERROR_STREAM(args) RCLCPP_ERROR_STREAM(aqua_slam::GetLogger(), args)
#define ROS_DEBUG(...) RCLCPP_DEBUG(aqua_slam::GetLogger(), __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(aqua_slam::GetLogger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(aqua_slam::GetLogger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(aqua_slam::GetLogger(), __VA_ARGS__)

#endif  // AQUA_SLAM_ROS1_COMPAT_H
