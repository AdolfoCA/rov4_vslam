"""Start a session: wait for the vehicle, start drivers and cameras, check everything.

    ros2 launch bluerov2_bringup session_start.launch.py

Run it inside the container right after plugging in the battery. It keeps running (it
owns the drivers); record from a second shell with record.launch.py, and end the
session with session_stop.launch.py. Ctrl-C here stops the drivers only - the camera
streams keep running on the multi-camera computer until session_stop.

Sequence - each step must succeed before the next one starts:

    1. preflight   wait (up to wait_timeout) until Pi, IMU heartbeat, DVL and the
                   multi-camera computer (ping + camera software) are all reachable
    2. drivers     bluerov2.launch.py, with the camera switches below
    3. cameras     multicam start <enabled multi-cameras>
    4. lights      lights_test_power % for lights_test_seconds, then `lights` (default 0)
    5. check       check_streams.py on every started sensor
    6.             SESSION READY

If a step fails, it prints SESSION NOT READY and which step; the drivers keep running
so you can look at what is wrong.

Launch arguments (all optional):

    wait_timeout:=180          seconds preflight waits for everything to be reachable
    check_pi/check_imu/check_dvl/check_nvidia:=true   which preflight checks run
    camera:=true               nose camera driver (+ the relay for QGroundControl)
    aux_left:=true aux_right:=true stereo_bottom:=false bottom_most:=false
                               multi-camera nodes, and which streams are started
    foxglove:=true             Foxglove bridge
    lights_test:=true          flash the lights to show the session is starting
    lights_test_power:=50      % during the test
    lights_test_seconds:=5
    lights:=0                  lights level left on after the start (0 = off)
    check_timeout:=15          seconds the stream check waits for each sensor

Lights can be changed at any time afterwards with `multicam lights <0-100>`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

MULTICAM_CAMERAS = {
    "aux_left": "true",
    "aux_right": "true",
    "stereo_bottom": "false",
    "bottom_most": "false",
}
NAMESPACE = "bluerov2"
CAMERA_INFO = "sensor_msgs/msg/CameraInfo"


def _flag(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).strip().lower() in (
        "true", "1", "yes", "on",
    )


def _value(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context).strip()


def _ros_run(label: str, executable: str, *args: str) -> tuple:
    return label, ExecuteProcess(
        cmd=["ros2", "run", "bluerov2_bringup", executable, *args],
        name=label.replace(" ", "_"),
        output="screen",
    )


def _sequence(steps, final_actions):
    """Run (label, ExecuteProcess) steps one after the other; each must exit 0."""
    if not steps:
        return final_actions
    (label, action), rest = steps[0], steps[1:]

    def after(event, _context):
        if event.returncode == 0:
            return _sequence(rest, final_actions)
        return [LogInfo(msg=f"\n*** SESSION NOT READY: step '{label}' failed "
                            f"(exit code {event.returncode}). See the output above. ***\n")]

    return [action, RegisterEventHandler(OnProcessExit(target_action=action, on_exit=after))]


def _session(context, *_args, **_kwargs):
    cameras = [c for c in MULTICAM_CAMERAS if _flag(context, c)]
    nose = _flag(context, "camera")

    # 1. preflight
    checks = [c for c in ("pi", "imu", "dvl", "nvidia") if _flag(context, f"check_{c}")]
    preflight = ("preflight", ExecuteProcess(
        cmd=["ros2", "run", "bluerov2_bringup", "preflight.py",
             "--timeout", _value(context, "wait_timeout"), *checks],
        name="preflight", output="screen",
    )) if checks else None

    # 2. drivers
    drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("bluerov2_bringup"), "launch", "bluerov2.launch.py")),
        launch_arguments={
            "camera": _value(context, "camera"),
            "foxglove": _value(context, "foxglove"),
            **{c: _value(context, c) for c in MULTICAM_CAMERAS},
        }.items(),
    )

    # 3-5. cameras, lights, stream check
    steps = []
    if cameras:
        steps.append(_ros_run("start cameras", "multicam.py", "start", *cameras))
    if _flag(context, "lights_test"):
        steps.append(_ros_run("lights on", "multicam.py", "lights",
                              _value(context, "lights_test_power")))
        steps.append(("lights test wait", ExecuteProcess(
            cmd=["sleep", _value(context, "lights_test_seconds")], name="lights_wait")))
    if _flag(context, "lights_test") or _value(context, "lights") not in ("", "0"):
        steps.append(_ros_run("lights level", "multicam.py", "lights", _value(context, "lights")))

    specs = [f"imu=/{NAMESPACE}/imu/data_raw:sensor_msgs/msg/Imu:50",
             f"dvl=/{NAMESPACE}/dvl/report:bluerov2_msgs/msg/DVLReport:1"]
    if nose:
        specs.append(f"camera=/{NAMESPACE}/camera/camera_info:{CAMERA_INFO}:10")
    specs += [f"{c}=/{NAMESPACE}/multicam/{c}/camera_info:{CAMERA_INFO}:10" for c in cameras]
    steps.append(_ros_run("stream check", "check_streams.py",
                          "--timeout", _value(context, "check_timeout"), *specs))

    ready = [LogInfo(msg=(
        "\n"
        "==================================================================\n"
        "  SESSION READY\n"
        f"  sensors: imu, dvl{', nose camera' if nose else ''}"
        f"{', ' + ', '.join(cameras) if cameras else ''}\n"
        "  record (second shell):  ros2 launch bluerov2_bringup record.launch.py\n"
        "  lights:                 multicam lights <0-100>\n"
        "  end of session:         ros2 launch bluerov2_bringup session_stop.launch.py\n"
        "==================================================================\n"))]

    after_drivers = _sequence(steps, ready)
    if preflight is None:
        return [drivers] + after_drivers

    label, action = preflight

    def after_preflight(event, _context):
        if event.returncode == 0:
            return [drivers] + after_drivers
        return [LogInfo(msg="\n*** SESSION NOT READY: preflight failed, drivers NOT started. "
                            "See the table above. ***\n")]

    return [action, RegisterEventHandler(OnProcessExit(target_action=action, on_exit=after_preflight))]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("wait_timeout", default_value="180"),
            DeclareLaunchArgument("check_pi", default_value="true"),
            DeclareLaunchArgument("check_imu", default_value="true"),
            DeclareLaunchArgument("check_dvl", default_value="true"),
            DeclareLaunchArgument("check_nvidia", default_value="true"),
            DeclareLaunchArgument("camera", default_value="true"),
            DeclareLaunchArgument("foxglove", default_value="true"),
        ]
        + [DeclareLaunchArgument(c, default_value=d) for c, d in MULTICAM_CAMERAS.items()]
        + [
            DeclareLaunchArgument("lights_test", default_value="true"),
            DeclareLaunchArgument("lights_test_power", default_value="50"),
            DeclareLaunchArgument("lights_test_seconds", default_value="5"),
            DeclareLaunchArgument("lights", default_value="0"),
            DeclareLaunchArgument("check_timeout", default_value="15"),
            OpaqueFunction(function=_session),
        ]
    )
