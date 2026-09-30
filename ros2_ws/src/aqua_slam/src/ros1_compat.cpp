#include "ros1_compat.h"

#include <atomic>
#include <memory>
#include <stdexcept>

namespace aqua_slam
{
namespace
{
std::shared_ptr<rclcpp::Node> g_node;
std::atomic<bool> g_node_set{false};
}  // namespace

void SetNode(const rclcpp::Node::SharedPtr &node)
{
	g_node = node;
	g_node_set = static_cast<bool>(node);
}

rclcpp::Node::SharedPtr GetNode()
{
	if (!g_node_set) {
		throw std::runtime_error("aqua_slam::SetNode() must be called before the SLAM system is created");
	}
	return g_node;
}

rclcpp::Logger GetLogger()
{
	return g_node_set ? g_node->get_logger() : rclcpp::get_logger("aqua_slam");
}

rclcpp::Time Now()
{
	return g_node_set ? g_node->now() : rclcpp::Clock(RCL_SYSTEM_TIME).now();
}
}  // namespace aqua_slam
