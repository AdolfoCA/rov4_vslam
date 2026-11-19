"""Pier calibration (bluerov2_navigation/pier_calibration_node.py).

    ros2 launch bluerov2_navigation pier_calibration.launch.py [settings:=<AQUA-SLAM yaml>] [auto_arm:=true]
        [output_dir:=/media/.../<day>] [use_sim_time:=true]

Arm it with the ROV still at the pier:  ros2 service call /pier_calibration/arm std_srvs/srv/Trigger
Pier point, nose direction and thresholds: config/pier_calibration.yaml.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument("settings", default_value=PathJoinSubstitution(
            [FindPackageShare("aqua_slam"), "config", "bluerov2_rov4_15hz_10-11.yaml"]),
            description="AQUA-SLAM settings yaml (extrinsics IMU-camera-DVL, DVL beam model)"),
        DeclareLaunchArgument("auto_arm", default_value="false", description="arm at start-up (bag replays)"),
        DeclareLaunchArgument("output_dir", default_value="", description="write the result there as JSON"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]
    node = Node(
        package="bluerov2_navigation", executable="pier_calibration", name="pier_calibration", output="screen",
        parameters=[
            PathJoinSubstitution([FindPackageShare("bluerov2_navigation"), "config", "pier_calibration.yaml"]),
            {"slam_settings": LaunchConfiguration("settings"),
             "auto_arm": ParameterValue(LaunchConfiguration("auto_arm"), value_type=bool),
             "output_dir": LaunchConfiguration("output_dir"),
             "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
        ],
    )
    return LaunchDescription(args + [node])
