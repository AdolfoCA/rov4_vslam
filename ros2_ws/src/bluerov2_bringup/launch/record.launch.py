"""Record the BlueROV2 sensor topics into a rosbag.

    ros2 launch bluerov2_bringup record.launch.py

Run it next to the drivers (``bluerov2.launch.py``), in a second shell. Stop it with
Ctrl-C: the recorder closes the bag cleanly on SIGINT.

By default every topic of every sensor is recorded. Turn a sensor off with its argument:

    ros2 launch bluerov2_bringup record.launch.py camera:=false
    ros2 launch bluerov2_bringup record.launch.py camera_raw:=false     # keep JPEG only

Launch arguments (all optional):

    imu:=true|false                record every IMU topic
    dvl:=true|false                record every DVL topic
    camera:=true|false             record the camera at all (camera_info always with it)
    camera_raw:=true|false         record camera/image_raw           (~187 MB/s!)
    camera_compressed:=true|false  record camera/image_raw/compressed (~15 MB/s)
    tf:=true|false                 record /tf and /tf_static
    logs:=true|false               record /rosout, i.e. every node's log output
    namespace:=bluerov2            namespace the drivers were launched in
    output_dir:=/home/rosdev/data  parent directory; bind-mounted to rov4_vslam/data
    bag_name:=<name>               default rov_YYYYmmdd_HHMMSS
    storage:=mcap|sqlite3          bag format
    max_bag_duration:=60           split into files of this many seconds; 0 = no split

What gets recorded
------------------
    imu     imu/data  imu/data_raw  imu/mag  imu/pressure  imu/temperature
    dvl     dvl/velocity  dvl/report  dvl/altitude
            dvl/dead_reckoning  dvl/dead_reckoning_odometry
    camera  camera/camera_info  camera/image_raw  camera/image_raw/compressed
    tf      /tf  /tf_static
    logs    /rosout

A topic that is not being published is simply absent from the bag, it is not an error.
In particular ``camera/image_raw/compressed`` only exists when the drivers were started
with the Foxglove bridge or ``compressed_video:=true``.

Timestamps
----------
Everything goes into ONE bag, on the one ROS clock, so IMU, DVL and camera share a
common time base. Nothing is resampled or dropped to force alignment: every message is
kept at its own rate, and carries two times - ``header.stamp``, set by the driver
(the IMU maps the vehicle clock onto ROS time; the DVL and camera stamp on arrival),
and the time the recorder received it. Align sensors offline by ``header.stamp``.

Why the rosbag2 recorder and not a Python node
----------------------------------------------
Raw 1080p video is ~187 MB/s. rosbag2's recorder is C++ and writes serialized messages
without ever deserializing them; a Python subscriber would have to convert every frame
and falls far behind (see the camera fps notes in CHECKPOINT_2026-09-15.txt). Humble has
no composable recorder node, so it runs as a process.

Disk
----
Raw video fills about 11 GB per minute. The free space on the output disk is printed at
start; check it before a long recording.
"""

import os
import shutil
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

IMU_TOPICS = [
    "imu/data",
    "imu/data_raw",
    "imu/mag",
    "imu/pressure",
    "imu/temperature",
]
DVL_TOPICS = [
    "dvl/velocity",
    "dvl/report",
    "dvl/altitude",
    "dvl/dead_reckoning",
    "dvl/dead_reckoning_odometry",
]
TF_TOPICS = ["/tf", "/tf_static"]
LOG_TOPICS = ["/rosout"]


def _flag(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).strip().lower() in (
        "true", "1", "yes", "on",
    )


def _recorder(context, *_args, **_kwargs):
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")

    def ns(topic: str) -> str:
        # Driver topics are relative and live under the namespace; TF and /rosout are
        # global and are listed with a leading slash.
        if topic.startswith("/") or not namespace:
            return topic if topic.startswith("/") else "/" + topic
        return f"/{namespace}/{topic}"

    topics = []
    if _flag(context, "imu"):
        topics += IMU_TOPICS
    if _flag(context, "dvl"):
        topics += DVL_TOPICS
    if _flag(context, "camera"):
        topics.append("camera/camera_info")
        if _flag(context, "camera_raw"):
            topics.append("camera/image_raw")
        if _flag(context, "camera_compressed"):
            topics.append("camera/image_raw/compressed")
    if _flag(context, "tf"):
        topics += TF_TOPICS
    if _flag(context, "logs"):
        topics += LOG_TOPICS
    topics = [ns(t) for t in topics]

    if not topics:
        raise RuntimeError("record.launch.py: every sensor is disabled, nothing to record")

    output_dir = os.path.expanduser(LaunchConfiguration("output_dir").perform(context))
    bag_name = LaunchConfiguration("bag_name").perform(context).strip()
    if not bag_name:
        bag_name = datetime.now().strftime("rov_%Y%m%d_%H%M%S")
    bag_path = os.path.join(output_dir, bag_name)
    if os.path.exists(bag_path):
        raise RuntimeError(f"record.launch.py: {bag_path} already exists, pick another bag_name")
    os.makedirs(output_dir, exist_ok=True)

    free_gb = shutil.disk_usage(output_dir).free / 1e9

    cmd = [
        "ros2", "bag", "record",
        "-s", LaunchConfiguration("storage").perform(context),
        "-o", bag_path,
    ]
    max_duration = int(LaunchConfiguration("max_bag_duration").perform(context))
    if max_duration > 0:
        cmd += ["-d", str(max_duration)]
    cmd += topics

    return [
        LogInfo(msg=f"Recording to {bag_path}  ({free_gb:.1f} GB free)"),
        LogInfo(msg="Topics:\n  " + "\n  ".join(topics)),
        ExecuteProcess(
            cmd=cmd,
            name="bag_recorder",
            output="screen",
            # Closing a bag flushes the write cache and, for MCAP, writes the summary
            # section. With raw video in the cache that can take a while, so do not let
            # launch escalate to SIGTERM/SIGKILL after the default few seconds.
            sigterm_timeout="30",
            sigkill_timeout="30",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("imu", default_value="true"),
            DeclareLaunchArgument("dvl", default_value="true"),
            DeclareLaunchArgument("camera", default_value="true"),
            DeclareLaunchArgument("camera_raw", default_value="true"),
            DeclareLaunchArgument("camera_compressed", default_value="true"),
            DeclareLaunchArgument("tf", default_value="true"),
            DeclareLaunchArgument("logs", default_value="true"),
            DeclareLaunchArgument("namespace", default_value="bluerov2"),
            DeclareLaunchArgument("output_dir", default_value="/home/rosdev/data"),
            DeclareLaunchArgument("bag_name", default_value=""),
            DeclareLaunchArgument("storage", default_value="mcap"),
            DeclareLaunchArgument("max_bag_duration", default_value="60"),
            OpaqueFunction(function=_recorder),
        ]
    )
