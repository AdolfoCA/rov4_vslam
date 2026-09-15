"""Run the Foxglove bridge on its own.

    ros2 launch bluerov2_bringup foxglove.launch.py

Use this when the drivers are already running - started by hand, by
``bluerov2.launch.py foxglove:=false``, or in another container - and you just want a
websocket to point Foxglove at. The main bringup launch starts the bridge itself by
default, so you rarely need this one.

What the bridge is
------------------
``foxglove_bridge`` is a ROS 2 node that exposes the whole ROS graph over a single
websocket, speaking the Foxglove WebSocket protocol. Foxglove (the desktop app or the
web app at app.foxglove.dev) connects to that one socket and can then subscribe to any
topic, read and set parameters, and call services - without needing ROS installed, and
without the DDS discovery traffic ever leaving this machine. That last part is why it
is the right tool here: DDS multicast discovery across a tether or a lab network is
fragile, whereas one outbound TCP connection is not.

Arguments:

    port:=8765                   websocket port
    address:=0.0.0.0             bind address; 0.0.0.0 accepts remote clients
    params_file:=<path>          override the parameter file
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("bluerov2_bringup")
    default_params = PathJoinSubstitution([pkg_share, "config", "foxglove_bridge.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="8765"),
            DeclareLaunchArgument("address", default_value="0.0.0.0"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        # 'port' is declared as an integer by the node; a bare
                        # substitution would arrive as a string and be rejected.
                        "port": ParameterValue(
                            LaunchConfiguration("port"), value_type=int
                        ),
                        "address": ParameterValue(
                            LaunchConfiguration("address"), value_type=str
                        ),
                    },
                ],
                respawn=True,
                respawn_delay=2.0,
            ),
        ]
    )
