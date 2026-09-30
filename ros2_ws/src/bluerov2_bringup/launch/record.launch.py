"""Record the BlueROV2 and multi-camera topics into a rosbag, after checking the streams.

    ros2 launch bluerov2_bringup record.launch.py

Run it next to the drivers (``bluerov2.launch.py``), in a second shell. Stop it with
Ctrl-C: the recorder closes the bag cleanly on SIGINT.

What is recorded is set in ``config/recording.yaml``: topics grouped by sensor, each
group enabled or not, and the stream each group is checked on. Edit and save it; no
rebuild is needed. Any group can be switched from the command line as well:

    ros2 launch bluerov2_bringup record.launch.py nose_camera:=true
    ros2 launch bluerov2_bringup record.launch.py stereo_bottom:=true bottom_most:=true
    ros2 launch bluerov2_bringup record.launch.py check_only:=true       # just check
    ros2 launch bluerov2_bringup record.launch.py config:=/path/other.yaml

Stream check
------------
Before the recorder starts, every enabled group that has a ``check`` entry is checked:
``check_streams.py`` waits for data on that topic and measures its rate. If any is
silent or slower than its ``min_hz``, it prints which one and the recording is NOT
started. ``check_only:=true`` runs only the check; ``check:=false`` skips it.

Launch arguments (all optional):

    config:=<path>                 topic selection (default: config/recording.yaml)
    <group>:=true|false            override one group's `enabled` from the config
    check:=true|false              check the selected streams before recording
    check_only:=true|false         only check, do not record
    check_timeout:=10              seconds to wait for the first message of each stream
    output_dir:=/home/rosdev/data  parent directory; bind-mounted to the project's data/
    bag_name:=<name>               default rov_YYYYmmdd_HHMMSS
    namespace:=, storage:=, max_bag_duration:=   override the config's values

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

import yaml
from ament_index_python.packages import get_package_share_directory
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

DEFAULT_CONFIG = os.path.join(
    get_package_share_directory("bluerov2_bringup"), "config", "recording.yaml")


def _flag(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).strip().lower() in (
        "true", "1", "yes", "on",
    )


def _override(context, name: str, default):
    """The command-line value of ``name`` if one was given, else ``default``."""
    value = context.launch_configurations.get(name, "").strip()
    return value if value else default


def _recorder(context, *_args, **_kwargs):
    config_path = os.path.expanduser(LaunchConfiguration("config").perform(context))
    with open(config_path) as f:
        config = yaml.safe_load(f) or {}
    groups = config.get("groups") or {}
    output = config.get("output") or {}
    namespace = str(_override(context, "namespace", config.get("namespace", ""))).strip("/")

    def ns(topic: str) -> str:
        # Driver topics are relative and live under the namespace; TF and /rosout are
        # global and are listed with a leading slash.
        if topic.startswith("/") or not namespace:
            return topic if topic.startswith("/") else "/" + topic
        return f"/{namespace}/{topic}"

    unknown = [k for k in context.launch_configurations
               if k not in groups and k not in KNOWN_ARGUMENTS]
    if unknown:
        raise RuntimeError(
            f"record.launch.py: unknown argument(s) {', '.join(unknown)}; groups in "
            f"{config_path} are: {', '.join(groups)}")

    topics = []
    checks = []  # (label, topic, type, min_hz)
    for name, group in groups.items():
        enabled = str(_override(context, name, group.get("enabled", False))).lower()
        if enabled not in ("true", "1", "yes", "on"):
            continue
        topics += group.get("topics") or []
        check = group.get("check")
        if check:
            checks.append((name, check["topic"], check["type"], check["min_hz"]))
    topics += config.get("extra_topics") or []
    topics = list(dict.fromkeys(ns(t) for t in topics))   # unique, in order

    if not topics:
        raise RuntimeError(f"record.launch.py: nothing enabled in {config_path}")

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
        "-s", str(_override(context, "storage", output.get("storage", "mcap"))),
        "-o", bag_path,
    ]
    max_duration = int(_override(context, "max_bag_duration",
                                 output.get("max_bag_duration", 60)))
    if max_duration > 0:
        cmd += ["-d", str(max_duration)]
    cmd += topics

    recording = [
        LogInfo(msg=f"Recording to {bag_path}  ({free_gb:.1f} GB free), "
                    f"topics from {config_path}"),
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


# Arguments that are not group names. Anything else given on the command line must
# be a group of the config, so a typo fails loudly instead of recording the wrong set.
KNOWN_ARGUMENTS = {
    "config", "check", "check_only", "check_timeout", "namespace", "output_dir",
    "bag_name", "storage", "max_bag_duration",
}


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=DEFAULT_CONFIG),
            DeclareLaunchArgument("check", default_value="true"),
            DeclareLaunchArgument("check_only", default_value="false"),
            DeclareLaunchArgument("check_timeout", default_value="10"),
            DeclareLaunchArgument("output_dir", default_value="/home/rosdev/data"),
            DeclareLaunchArgument("bag_name", default_value=""),
            # Empty = take it from the config.
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("storage", default_value=""),
            DeclareLaunchArgument("max_bag_duration", default_value=""),
            OpaqueFunction(function=_recorder),
        ]
    )
