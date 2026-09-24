"""Record the BlueROV2 and multi-camera topics into a rosbag, after checking the streams.

    ros2 launch bluerov2_bringup record.launch.py

Run it next to the drivers (``bluerov2.launch.py``), in a second shell. Stop it with
Ctrl-C: the recorder closes the bag cleanly on SIGINT.

Choose what to record with one switch per sensor. By default: IMU, DVL and the top pair
of the multi-camera system (aux_left, aux_right). The nose camera and the bottom pair
are off.

    ros2 launch bluerov2_bringup record.launch.py camera:=true           # add nose camera
    ros2 launch bluerov2_bringup record.launch.py stereo_bottom:=true bottom_most:=true
    ros2 launch bluerov2_bringup record.launch.py check_only:=true       # just check

Stream check
------------
Before the recorder starts, every selected sensor is checked: the script
``check_streams.py`` waits for data from each one and measures its rate. If any sensor
is silent or slower than its minimum rate, it prints which one and the recording is
NOT started. ``check_only:=true`` runs only the check (useful at the start of a
session); ``check:=false`` skips it.

    sensor          checked on                      minimum
    imu             imu/data_raw                    50 Hz
    dvl             dvl/report                       1 Hz   (reports arrive in air too)
    camera          camera/camera_info              10 Hz
    <multicam>      multicam/<camera>/camera_info   10 Hz

Launch arguments (all optional):

    Sensors
    imu:=true|false                record every IMU topic              (default true)
    dvl:=true|false                record every DVL topic              (default true)
    camera:=true|false             record the ROV nose camera, camera_info always with it
                                   (default false)
    camera_raw:=true|false         nose camera/image_raw            (~187 MB/s!)
    camera_compressed:=true|false  nose camera/image_raw/compressed (~15 MB/s)
    aux_left:=true|false           multi-camera Aux Left       (default true)
    aux_right:=true|false          multi-camera Aux Right      (default true)
    stereo_bottom:=true|false      multi-camera Stereo Bottom  (default false)
    bottom_most:=true|false        multi-camera Bottom Most    (default false)
    multicam_raw:=true|false       multi-camera image_raw, 960x540 (~39 MB/s per camera)
                                   (default false)
    multicam_compressed:=true|false  multi-camera image_raw/compressed, JPEG
                                   (default true)
    tf:=true|false                 record /tf and /tf_static
    logs:=true|false               record /rosout, i.e. every node's log output

    Check
    check:=true|false              check the selected streams before recording
    check_only:=true|false         only check, do not record
    check_timeout:=10              seconds to wait for the first message of each stream

    Output
    namespace:=bluerov2            namespace the drivers were launched in
    output_dir:=/home/rosdev/data  parent directory; bind-mounted to rov4_vslam/data
    bag_name:=<name>               default rov_YYYYmmdd_HHMMSS
    storage:=mcap|sqlite3          bag format
    max_bag_duration:=60           split into files of this many seconds; 0 = no split

A multi-camera switch only records that camera if its node is running: start it with
the matching argument of bluerov2.launch.py (aux_left and aux_right are on by default).

Timestamps
----------
Everything goes into ONE bag, on the one ROS clock, so IMU, DVL and all cameras share a
common time base. Nothing is resampled or dropped to force alignment: every message is
kept at its own rate, and carries two times - ``header.stamp``, set by the driver
(the IMU maps the vehicle clock onto ROS time; the DVL and all cameras stamp on
arrival), and the time the recorder received it. Align sensors offline by
``header.stamp``. The four multi-camera streams are not synchronised with each other
by the camera system.

Why the rosbag2 recorder and not a Python node
----------------------------------------------
Raw 1080p video is ~187 MB/s. rosbag2's recorder is C++ and writes serialized messages
without ever deserializing them; a Python subscriber would fall far behind. Humble has
no composable recorder node, so it runs as a process.

Disk
----
Raw nose-camera video fills about 11 GB per minute. The free space on the output disk
is printed at start; check it before a long recording.
"""

import os
import shutil
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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

# Multi-camera system cameras and whether each is recorded by default. Keep the names
# in step with MULTICAM_CAMERAS in bluerov2.launch.py.
MULTICAM_CAMERAS = {
    "aux_left": "true",
    "aux_right": "true",
    "stereo_bottom": "false",
    "bottom_most": "false",
}

# What the stream check subscribes to for each sensor, and the minimum rate that counts
# as "on". Cameras are checked on camera_info, published once per frame.
CAMERA_INFO = "sensor_msgs/msg/CameraInfo"
CHECKS = {
    "imu": ("imu/data_raw", "sensor_msgs/msg/Imu", 50),
    "dvl": ("dvl/report", "bluerov2_msgs/msg/DVLReport", 1),
    "camera": ("camera/camera_info", CAMERA_INFO, 10),
}
MULTICAM_MIN_HZ = 10


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
    checks = []  # (label, topic, type, min_hz)
    if _flag(context, "imu"):
        topics += IMU_TOPICS
        checks.append(("imu", *CHECKS["imu"]))
    if _flag(context, "dvl"):
        topics += DVL_TOPICS
        checks.append(("dvl", *CHECKS["dvl"]))
    if _flag(context, "camera"):
        topics.append("camera/camera_info")
        if _flag(context, "camera_raw"):
            topics.append("camera/image_raw")
        if _flag(context, "camera_compressed"):
            topics.append("camera/image_raw/compressed")
        checks.append(("camera", *CHECKS["camera"]))
    for camera in MULTICAM_CAMERAS:
        if not _flag(context, camera):
            continue
        base = f"multicam/{camera}"
        topics.append(f"{base}/camera_info")
        if _flag(context, "multicam_raw"):
            topics.append(f"{base}/image_raw")
        if _flag(context, "multicam_compressed"):
            topics.append(f"{base}/image_raw/compressed")
        checks.append((camera, f"{base}/camera_info", CAMERA_INFO, MULTICAM_MIN_HZ))
    if _flag(context, "tf"):
        topics += TF_TOPICS
    if _flag(context, "logs"):
        topics += LOG_TOPICS
    topics = [ns(t) for t in topics]

    if not topics:
        raise RuntimeError("record.launch.py: every sensor is disabled, nothing to record")

    check_only = _flag(context, "check_only")
    run_check = (_flag(context, "check") or check_only) and bool(checks)

    checker = None
    if run_check:
        checker = Node(
            package="bluerov2_bringup",
            executable="check_streams.py",
            name="stream_check",
            output="screen",
            arguments=[
                "--timeout", LaunchConfiguration("check_timeout").perform(context),
                *[f"{label}={ns(topic)}:{type_name}:{min_hz}"
                  for label, topic, type_name, min_hz in checks],
            ],
        )

    if check_only:
        return [
            checker,
            RegisterEventHandler(OnProcessExit(
                target_action=checker,
                on_exit=[EmitEvent(event=Shutdown(reason="check_only: done"))],
            )),
        ]

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

    recording = [
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

    if checker is None:
        return recording

    def after_check(event, _context):
        if event.returncode == 0:
            return recording
        return [
            LogInfo(msg="Recording NOT started: a selected stream is missing or too slow "
                        "(see the table above). Start that sensor, or turn its switch off."),
            EmitEvent(event=Shutdown(reason="stream check failed")),
        ]

    return [
        checker,
        RegisterEventHandler(OnProcessExit(target_action=checker, on_exit=after_check)),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("imu", default_value="true"),
            DeclareLaunchArgument("dvl", default_value="true"),
            DeclareLaunchArgument("camera", default_value="false"),
            DeclareLaunchArgument("camera_raw", default_value="true"),
            DeclareLaunchArgument("camera_compressed", default_value="true"),
        ]
        + [
            DeclareLaunchArgument(camera, default_value=default)
            for camera, default in MULTICAM_CAMERAS.items()
        ]
        + [
            DeclareLaunchArgument("multicam_raw", default_value="false"),
            DeclareLaunchArgument("multicam_compressed", default_value="true"),
            DeclareLaunchArgument("tf", default_value="true"),
            DeclareLaunchArgument("logs", default_value="true"),
            DeclareLaunchArgument("check", default_value="true"),
            DeclareLaunchArgument("check_only", default_value="false"),
            DeclareLaunchArgument("check_timeout", default_value="10"),
            DeclareLaunchArgument("namespace", default_value="bluerov2"),
            DeclareLaunchArgument("output_dir", default_value="/home/rosdev/data"),
            DeclareLaunchArgument("bag_name", default_value=""),
            DeclareLaunchArgument("storage", default_value="mcap"),
            DeclareLaunchArgument("max_bag_duration", default_value="60"),
            OpaqueFunction(function=_recorder),
        ]
    )
