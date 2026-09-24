"""Bring up the BlueROV2 Heavy sensor drivers.

    ros2 launch bluerov2_bringup bluerov2.launch.py

Launch arguments (all optional):

    imu:=true|false          start the MAVLink IMU driver
    dvl:=true|false          start the Water Linked DVL driver
    camera:=true|false       start the nose camera driver, and the video relay that lets
                             QGroundControl receive the same stream (see below)
    foxglove:=true|false     start foxglove_bridge (default true)
    foxglove_port:=8765      websocket port for Foxglove clients
    compressed_video:=auto   publish JPEG alongside raw video; see below
    namespace:=bluerov2      ROS namespace for every node
    params_file:=<path>      override the driver parameter file
    extrinsics_file:=<path>  override the sensor extrinsics file
    static_tf:=true|false    publish the sensor extrinsics on /tf_static
    rov_ip:=192.168.2.2      convenience override, see below
    dvl_ip:=192.168.2.95     convenience override, see below
    log_level:=info          rcl logging level

    Multi-camera system (Blue Atlas, 4 cameras, H.265 streams to 192.168.1.1):
    aux_left:=true           camera node for Aux Left       (UDP 5700)
    aux_right:=true          camera node for Aux Right      (UDP 5701)
    stereo_bottom:=false     camera node for Stereo Bottom  (UDP 5702)
    bottom_most:=false       camera node for Bottom Most    (UDP 5703)

Multi-camera topics: <namespace>/multicam/<camera>/image_raw, .../image_raw/compressed
and .../camera_info. Ports, codec and frame ids are in config/bluerov2.yaml. These nodes
only receive the streams: the camera system has to be streaming already (it is started
on the camera system with its new_attach_streaming_consumer service).

Video over Foxglove
-------------------
``compressed_video`` defaults to ``auto``, which turns JPEG republishing on exactly
when the Foxglove bridge is running. The reason is bandwidth: raw ``bgr8`` at 1080p30
is about 186 MB/s, which no websocket will carry, whereas the same frames as JPEG are
roughly 5 MB/s. Foxglove's Image panel reads ``CompressedImage`` natively, so pointing
it at ``camera/image_raw/compressed`` gives smooth video where the raw topic would stall
the connection and eventually get the client dropped. Force it either way with
``compressed_video:=true`` or ``compressed_video:=false``.

``rov_ip`` and ``dvl_ip`` exist so that a field session can be re-pointed without
editing YAML: when set they override ``connection_url`` and the DVL ``host``. Leave
them empty to use whatever is in the parameter file.

Frames
------
The sensor extrinsics are published by ``bluerov2_tf``'s ``static_tf_node``, configured
in ``config/extrinsics.yaml``. Frame layout (REP-103/REP-105):

    base_link ──> imu_link              Navigator, inside the electronics enclosure
              ──> dvl_link              A50, looking down
              ──> camera_link ──> camera_link_optical

The values in that file are placeholders until measured on the vehicle; the node warns
at startup about any transform still left at exact identity.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Multi-camera system cameras, with whether each one's node starts by default. The top
# pair is the one currently used for datasets.
MULTICAM_CAMERAS = {
    "aux_left": "true",
    "aux_right": "true",
    "stereo_bottom": "false",
    "bottom_most": "false",
}


def _driver_nodes(context, *_args, **_kwargs):
    """Build the driver nodes, applying the ip overrides if they were given."""
    params_file = LaunchConfiguration("params_file").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    rov_ip = LaunchConfiguration("rov_ip").perform(context).strip()
    dvl_ip = LaunchConfiguration("dvl_ip").perform(context).strip()

    imu_overrides = {}
    if rov_ip:
        # BlueOS pushes MAVLink to topside, so the usual endpoint is a local listener.
        # Naming the ROV here switches to an outbound connection instead, which is what
        # you want when BlueOS is configured to wait for a client.
        imu_overrides["connection_url"] = f"udpout:{rov_ip}:14550"

    dvl_overrides = {"host": dvl_ip} if dvl_ip else {}

    # 'auto' means: compress when something is going to watch over a websocket.
    foxglove_on = LaunchConfiguration("foxglove").perform(context).lower() in (
        "true", "1", "yes", "on",
    )
    compressed = LaunchConfiguration("compressed_video").perform(context).strip().lower()
    if compressed in ("auto", ""):
        compress_video = foxglove_on
    else:
        compress_video = compressed in ("true", "1", "yes", "on")
    camera_overrides = {"publish_compressed": compress_video}

    common = ["--ros-args", "--log-level", log_level]

    return [
        Node(
            package="bluerov2_imu",
            executable="imu_node",
            name="imu_node",
            output="screen",
            parameters=[params_file, imu_overrides] if imu_overrides else [params_file],
            arguments=common,
            condition=IfCondition(LaunchConfiguration("imu")),
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="bluerov2_dvl",
            executable="dvl_node",
            name="dvl_node",
            output="screen",
            parameters=[params_file, dvl_overrides] if dvl_overrides else [params_file],
            arguments=common,
            condition=IfCondition(LaunchConfiguration("dvl")),
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="bluerov2_camera",
            executable="camera_node",
            name="camera_node",
            output="screen",
            parameters=[params_file, camera_overrides],
            arguments=common,
            condition=IfCondition(LaunchConfiguration("camera")),
            respawn=True,
            respawn_delay=2.0,
        ),
    ] + [
        # Nose-camera video relay. BlueOS sends the H.264 RTP stream once, to
        # 192.168.2.1:5600, and a UDP port feeds only one program - but both
        # QGroundControl and the camera node above want it. The relay binds the tether
        # address specifically (192.168.2.1:5600, which Linux prefers over QGC's
        # 0.0.0.0:5600) and copies each packet, without decoding, to 127.0.0.1:5600 for
        # QGroundControl and to the camera node's port (5602, bluerov2.yaml).
        ExecuteProcess(
            cmd=[
                "gst-launch-1.0", "-q",
                "udpsrc", "address=192.168.2.1", "port=5600", "reuse=true",
                "buffer-size=4194304",
                "!", "multiudpsink", "clients=127.0.0.1:5600,127.0.0.1:5602", "sync=false",
            ],
            name="nose_video_relay",
            output="screen",
            condition=IfCondition(LaunchConfiguration("camera")),
            respawn=True,
            respawn_delay=2.0,
        ),
    ] + [
        # Multi-camera system: the same camera driver, one node per camera, with codec
        # and port from the params file. The node publishes camera/image_raw etc.; the
        # remappings give each camera its own topics under multicam/<camera>/.
        Node(
            package="bluerov2_camera",
            executable="camera_node",
            name=camera,
            namespace="multicam",
            output="screen",
            parameters=[params_file],
            arguments=common,
            remappings=[
                ("camera/image_raw", f"{camera}/image_raw"),
                ("camera/image_raw/compressed", f"{camera}/image_raw/compressed"),
                ("camera/camera_info", f"{camera}/camera_info"),
            ],
            condition=IfCondition(LaunchConfiguration(camera)),
            respawn=True,
            respawn_delay=2.0,
        )
        for camera in MULTICAM_CAMERAS
    ]


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("bluerov2_bringup")
    default_params = PathJoinSubstitution([pkg_share, "config", "bluerov2.yaml"])

    arguments = [
        DeclareLaunchArgument("imu", default_value="true"),
        DeclareLaunchArgument("dvl", default_value="true"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        DeclareLaunchArgument("compressed_video", default_value="auto"),
        DeclareLaunchArgument("namespace", default_value="bluerov2"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("static_tf", default_value="true"),
        DeclareLaunchArgument(
            "extrinsics_file",
            default_value=PathJoinSubstitution([pkg_share, "config", "extrinsics.yaml"]),
        ),
        DeclareLaunchArgument("rov_ip", default_value=""),
        DeclareLaunchArgument("dvl_ip", default_value=""),
        DeclareLaunchArgument("log_level", default_value="info"),
    ] + [
        DeclareLaunchArgument(camera, default_value=default)
        for camera, default in MULTICAM_CAMERAS.items()
    ]

    static_tf = Node(
        package="bluerov2_tf",
        executable="static_tf_node",
        name="static_tf_node",
        output="screen",
        parameters=[LaunchConfiguration("extrinsics_file")],
        condition=IfCondition(LaunchConfiguration("static_tf")),
    )

    # The bridge stays outside the namespace as well. It serves the whole graph, not one
    # subsystem, and a client connects to it by address and port rather than by node
    # name - so namespacing it would only make it harder to find in `ros2 node list`.
    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "foxglove_bridge.yaml"]),
            # A bare substitution resolves to a string, and foxglove_bridge declares
            # 'port' as an integer - without the explicit type the node rejects it at
            # startup with InvalidParameterTypeException.
            {"port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int)},
        ],
        condition=IfCondition(LaunchConfiguration("foxglove")),
        respawn=True,
        respawn_delay=2.0,
    )

    # Only the drivers go inside the namespace. The static TF node stays at the root on
    # purpose: TF is a single global tree, and pushing it into a namespace would remap
    # /tf_static to /bluerov2/tf_static, quietly cutting every namespaced frame off from
    # the rest of the system.
    return LaunchDescription(
        arguments
        + [
            GroupAction(
                [
                    PushRosNamespace(LaunchConfiguration("namespace")),
                    OpaqueFunction(function=_driver_nodes),
                ]
            ),
            static_tf,
            foxglove,
        ]
    )
