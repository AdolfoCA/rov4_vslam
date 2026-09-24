"""End a session: stop the camera streams, lights off, power off the multi-camera computer.

    ros2 launch bluerov2_bringup session_stop.launch.py

Run it inside the container at the end of the day, then unplug the battery when it
prints "safe to unplug". It runs `multicam shutdown`, which:

    1. stops every multi-camera stream        (/ba/camera_feed_manager/detach_...)
    2. sets the lights to 0 on channels 1-3   (/ba/light_controller/set_brightness)
    3. powers off the multi-camera computer   (ssh nvidia@10.42.0.5 sudo poweroff)
    4. waits until it stops answering ping, plus a margin for its disks

Steps 1-2 only warn if they fail; the power-off is what matters. The login is read from
rov4_vslam/data/.multicam_ssh (host=, user=, password=), which git ignores.

BlueOS and the ROV are NOT powered off. Stop session_start.launch.py (Ctrl-C in its
shell) before or after this; the camera nodes only log "No frames" once the streams stop.
"""

from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def generate_launch_description() -> LaunchDescription:
    shutdown = ExecuteProcess(
        cmd=["ros2", "run", "bluerov2_bringup", "multicam.py", "shutdown"],
        name="multicam_shutdown",
        output="screen",
    )

    def after(event, _context):
        if event.returncode == 0:
            banner = (
                "\n"
                "==================================================================\n"
                "  SESSION STOPPED\n"
                "  camera streams stopped, lights off, multi-camera computer OFF\n"
                "  -> safe to unplug the battery (BlueOS was left on)\n"
                "==================================================================\n")
        else:
            banner = (
                "\n"
                "==================================================================\n"
                "  *** NOT SAFE TO UNPLUG YET ***\n"
                "  the multi-camera computer was not confirmed off - see above.\n"
                "  Check the login in data/.multicam_ssh, or power it off by hand.\n"
                "==================================================================\n")
        return [LogInfo(msg=banner), EmitEvent(event=Shutdown(reason="session_stop done"))]

    return LaunchDescription([
        shutdown,
        RegisterEventHandler(OnProcessExit(target_action=shutdown, on_exit=after)),
    ])
