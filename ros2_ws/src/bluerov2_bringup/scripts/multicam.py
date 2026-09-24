#!/usr/bin/env python3
"""Start/stop the multi-camera streams and set its lights, via the camera system's services.

    multicam status                        # cameras available/streaming/recording, lights
    multicam start [CAMERA ...|all]        # default: aux_left aux_right
    multicam stop  [CAMERA ...|all]        # default: aux_left aux_right;
                                           # all = every camera that is streaming
    multicam lights BRIGHTNESS [--channel N]   # 0-100, channel 1-3 (default 3)
    multicam shutdown                      # stop all streams, lights off, power off
                                           # the multi-camera computer (end of day)

    (inside the container `multicam` is an alias for
     `ros2 run bluerov2_bringup multicam.py`)

CAMERA is one of aux_left, aux_right, stereo_bottom, bottom_most.

Why this switches middleware
----------------------------
The camera system (Blue Atlas, its own NVIDIA computer) runs its ROS 2 nodes on
CycloneDDS. Its topics can be read from Fast DDS, but its services do not answer calls
made from Fast DDS. So this script re-executes itself with
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp, for its own calls only; the drivers and the
recorder stay on Fast DDS.

CycloneDDS is restricted to the tether address (192.168.1.1 by default, or
$MULTICAM_TOPSIDE_IP). Advertising the Wi-Fi or Docker address as well makes the camera
system send its replies to an address it cannot reach, and they never arrive.

What the calls do on the camera system
--------------------------------------
start   /ba/camera_feed_manager/new_attach_streaming_consumer, once per camera:
        {camera_device_path: /dev/video_<camera>, ip_address: <topside>, port: <port>}
        The camera system then sends H.265 RTP to <topside>:<port>. Ports come from
        bluerov2_bringup/config/bluerov2.yaml (udp_port of each camera), so they match
        the receiving camera nodes.
stop    /ba/camera_feed_manager/detach_streaming_consumer {camera_device_path}
status  /ba/camera_feed_manager/camera_status, and the lifecycle state of
        /ba/light_controller
lights  /ba/light_controller/set_brightness {channel, brightness}, after bringing the
        light controller (a lifecycle node) to 'active' with configure/activate if
        needed: while 'unconfigured' it answers done=True but the lights stay dark.

The streams do not survive a reboot of the camera computer: run `multicam start` after
every power cycle (checked 2026-09-24).
"""

import argparse
import os
import sys

TOPSIDE_IP = os.environ.get("MULTICAM_TOPSIDE_IP", "192.168.1.1")
CAMERAS = ("aux_left", "aux_right", "stereo_bottom", "bottom_most")
DEFAULT_CAMERAS = ("aux_left", "aux_right")
# Used only if bluerov2.yaml cannot be read.
FALLBACK_PORTS = {"aux_left": 5700, "aux_right": 5701, "stereo_bottom": 5702, "bottom_most": 5703}

SERVICE_ATTACH = "/ba/camera_feed_manager/new_attach_streaming_consumer"
SERVICE_DETACH = "/ba/camera_feed_manager/detach_streaming_consumer"
SERVICE_STATUS = "/ba/camera_feed_manager/camera_status"
SERVICE_BRIGHTNESS = "/ba/light_controller/set_brightness"
SERVICE_LIGHT_STATE = "/ba/light_controller/get_state"
SERVICE_LIGHT_CHANGE_STATE = "/ba/light_controller/change_state"


def _reexec_with_cyclonedds() -> None:
    """Run this script again with CycloneDDS, bound to the tether address."""
    if os.environ.get("MULTICAM_CYCLONE_ACTIVE") == "1":
        return
    env = dict(os.environ)
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    env["MULTICAM_CYCLONE_ACTIVE"] = "1"
    env["CYCLONEDDS_URI"] = (
        "<CycloneDDS><Domain id='any'><General><Interfaces>"
        f"<NetworkInterface address='{TOPSIDE_IP}'/>"
        "</Interfaces></General></Domain></CycloneDDS>"
    )
    os.execvpe(sys.executable, [sys.executable] + sys.argv, env)


def _ports() -> dict:
    """udp_port of each camera, from the bringup parameter file."""
    try:
        import yaml
        from ament_index_python.packages import get_package_share_directory

        path = os.path.join(
            get_package_share_directory("bluerov2_bringup"), "config", "bluerov2.yaml"
        )
        with open(path, encoding="utf-8") as handle:
            params = yaml.safe_load(handle)
        return {
            camera: int(params[f"/**/{camera}"]["ros__parameters"]["udp_port"])
            for camera in CAMERAS
        }
    except Exception as exc:  # noqa: BLE001 - fall back and say so
        print(f"(could not read ports from bluerov2.yaml: {exc}; using defaults)")
        return dict(FALLBACK_PORTS)


def _cameras(names) -> list:
    if not names:
        return list(DEFAULT_CAMERAS)
    if names == ["all"]:
        return list(CAMERAS)
    unknown = [n for n in names if n not in CAMERAS]
    if unknown:
        sys.exit(f"unknown camera(s): {', '.join(unknown)}. Use: {', '.join(CAMERAS)} or all")
    return names


class Caller:
    def __init__(self, timeout: float):
        import rclpy

        self._rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("multicam_ctl")
        self.timeout = timeout

    def call(self, service, srv_type, request):
        client = self.node.create_client(srv_type, service)
        try:
            if not client.wait_for_service(timeout_sec=self.timeout):
                return None, f"service {service} not found within {self.timeout:.0f} s"
            future = client.call_async(request)
            self._rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
            if not future.done():
                return None, f"no reply from {service} within {self.timeout:.0f} s"
            return future.result(), None
        finally:
            self.node.destroy_client(client)

    def close(self):
        self.node.destroy_node()
        self._rclpy.shutdown()


def cmd_status(caller, _args) -> int:
    from ba_msgs.srv import CameraStatus
    from lifecycle_msgs.srv import GetState

    status, err = caller.call(SERVICE_STATUS, CameraStatus, CameraStatus.Request())
    if err:
        print(f"camera status: FAILED - {err}")
        return 1
    print(f"camera system : {status.status}")
    print(f"  available   : {', '.join(status.cameras_available) or '-'}")
    print(f"  streaming   : {', '.join(status.cameras_streaming) or '-'}")
    print(f"  recording   : {', '.join(status.cameras_recording) or '-'}")
    print(f"  failed      : {', '.join(status.cameras_failed) or '-'}")
    print(f"  save path   : {status.save_path or '-'}")
    state, err = caller.call(SERVICE_LIGHT_STATE, GetState, GetState.Request())
    print(f"light controller: {err if err else state.current_state.label}")
    return 0


def _attach_or_detach(caller, args, start: bool) -> int:
    from ba_msgs.srv import DetachConsumer, NewAttachStreamingConsumer

    ports = _ports()
    failed = 0
    for camera in _cameras(args.cameras):
        device = f"/dev/video_{camera}"
        if start:
            request = NewAttachStreamingConsumer.Request(
                camera_device_path=device, ip_address=TOPSIDE_IP, port=str(ports[camera])
            )
            result, err = caller.call(SERVICE_ATTACH, NewAttachStreamingConsumer, request)
            what = f"start {camera:<14} -> {TOPSIDE_IP}:{ports[camera]}"
        else:
            request = DetachConsumer.Request(camera_device_path=device)
            result, err = caller.call(SERVICE_DETACH, DetachConsumer, request)
            what = f"stop  {camera:<14}"
        ok = err is None and result.succeeded
        failed += 0 if ok else 1
        print(f"{what}  {'OK' if ok else 'FAILED'}{'  - ' + err if err else ''}")
    print("")
    cmd_status(caller, args)
    return 1 if failed else 0


def cmd_start(caller, args) -> int:
    return _attach_or_detach(caller, args, start=True)


def _streaming_cameras(caller):
    """Names of the cameras streaming right now, or None if camera_status fails."""
    from ba_msgs.srv import CameraStatus

    status, err = caller.call(SERVICE_STATUS, CameraStatus, CameraStatus.Request())
    if err:
        return None
    return [d[len("/dev/video_"):] for d in status.cameras_streaming
            if d.startswith("/dev/video_")]


def cmd_stop(caller, args) -> int:
    # "all" means every camera that is streaming: detaching a camera that is not
    # streaming is refused by the camera system (succeeded=False), which would show
    # up as a false FAILED.
    if args.cameras == ["all"]:
        streaming = _streaming_cameras(caller)
        if streaming is None:
            print("(camera status unavailable - trying to stop every camera)")
        elif not streaming:
            print("no camera is streaming - nothing to stop")
            return 0
        else:
            args.cameras = streaming
    return _attach_or_detach(caller, args, start=False)


def _ensure_lights_active(caller) -> bool:
    """Bring /ba/light_controller to the 'active' lifecycle state if it is not there.

    The light controller is a lifecycle node. After the camera computer boots it is
    'unconfigured': it still answers set_brightness with done=True, but the lights stay
    dark (checked 2026-09-24). It has to be configured and then activated first.
    """
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState, GetState

    steps = {
        "unconfigured": ("configure", Transition.TRANSITION_CONFIGURE),
        "inactive": ("activate", Transition.TRANSITION_ACTIVATE),
    }
    for _ in range(3):
        state, err = caller.call(SERVICE_LIGHT_STATE, GetState, GetState.Request())
        if err:
            print(f"light controller: FAILED to read its state - {err}")
            return False
        label = state.current_state.label
        if label == "active":
            return True
        if label not in steps:
            print(f"light controller is '{label}', cannot bring it to 'active'")
            return False
        name, transition_id = steps[label]
        request = ChangeState.Request()
        request.transition.id = transition_id
        result, err = caller.call(SERVICE_LIGHT_CHANGE_STATE, ChangeState, request)
        ok = err is None and result.success
        print(f"light controller: {label} -> {name}  {'OK' if ok else 'FAILED'}"
              f"{'  - ' + err if err else ''}")
        if not ok:
            return False
    return False


def cmd_lights(caller, args) -> int:
    from ba_msgs.srv import SetBrightness

    if not 0 <= args.brightness <= 100:
        sys.exit("brightness must be 0-100")
    if not 1 <= args.channel <= 3:
        sys.exit("channel must be 1-3")
    if not _ensure_lights_active(caller):
        return 1
    request = SetBrightness.Request(channel=args.channel, brightness=args.brightness)
    result, err = caller.call(SERVICE_BRIGHTNESS, SetBrightness, request)
    ok = err is None and result.done
    print(f"lights channel {args.channel} -> {args.brightness}%  "
          f"{'OK' if ok else 'FAILED'}{'  - ' + err if err else ''}")
    return 0 if ok else 1


SSH_FILE = os.path.expanduser(os.environ.get("MULTICAM_SSH_FILE", "~/data/.multicam_ssh"))


def _read_ssh_file() -> dict:
    """host/user/password for the multi-camera computer, from data/.multicam_ssh."""
    login = {"host": "10.42.0.5"}
    try:
        with open(SSH_FILE, encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    key, value = line.split("=", 1)
                    login[key.strip()] = value.strip()
    except OSError as exc:
        sys.exit(f"cannot read {SSH_FILE} ({exc}).\nCreate it from the template, on the "
                 "host, in rov4_vslam/:\n"
                 "  cp ros2_ws/src/bluerov2_bringup/config/multicam_ssh.example "
                 "data/.multicam_ssh\n"
                 "  chmod 600 data/.multicam_ssh   # then put the real password in it")
    missing = [k for k in ("user", "password") if not login.get(k)]
    if missing:
        sys.exit(f"{SSH_FILE} has no {', '.join(missing)}")
    if login["password"] == "CHANGE_ME":
        sys.exit(f"{SSH_FILE} still has the template password: put the real one in it "
                 "(see the lab's multi-camera user guide).")
    return login


def _ping(host: str) -> bool:
    import subprocess

    return subprocess.run(["ping", "-c", "1", "-W", "1", host],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0


def cmd_shutdown(caller, args) -> int:
    """Stop every stream, lights off, power off the multi-camera computer, wait for it."""
    import shlex
    import subprocess
    import time

    login = _read_ssh_file()
    host = login["host"]

    print("=== 1/3 stop the camera streams that are running")
    args.cameras = ["all"]
    if cmd_stop(caller, args) != 0:
        print("    (not every stream stopped - continuing, the power-off matters more)")

    print("\n=== 2/3 lights off")
    from ba_msgs.srv import SetBrightness

    for channel in (1, 2, 3):
        result, err = caller.call(SERVICE_BRIGHTNESS, SetBrightness,
                                  SetBrightness.Request(channel=channel, brightness=0))
        ok = err is None and result.done
        print(f"lights channel {channel} -> 0%  {'OK' if ok else 'FAILED'}"
              f"{'  - ' + err if err else ''}")

    print(f"\n=== 3/3 power off {login['user']}@{host}")
    if not _ping(host):
        print(f"{host} does not answer ping: already off, or not reachable.")
        return 0
    remote = f"echo {shlex.quote(login['password'])} | sudo -S -p '' poweroff"
    result = subprocess.run(
        ["sshpass", "-e", "ssh",
         "-o", "StrictHostKeyChecking=accept-new", "-o", "ConnectTimeout=8",
         "-o", "PreferredAuthentications=password", "-o", "PubkeyAuthentication=no",
         f"{login['user']}@{host}", remote],
        env=dict(os.environ, SSHPASS=login["password"]),
        capture_output=True, text=True, timeout=40,
    )
    output = (result.stdout + result.stderr).strip()
    # The connection may be cut by the power-off itself (exit 255): that is fine.
    # A refused login or sudo is not.
    if result.returncode == 5 or any(s in output.lower() for s in (
            "permission denied", "incorrect password", "sorry, try again")):
        print(f"power-off FAILED: login or sudo refused (exit {result.returncode})\n{output}")
        return 1
    print(f"poweroff sent (ssh exit {result.returncode}). Waiting for {host} to go silent...")

    deadline = time.monotonic() + 120
    misses = 0
    while time.monotonic() < deadline:
        misses = 0 if _ping(host) else misses + 1
        if misses >= 3:
            break
        time.sleep(2)
    else:
        print(f"FAILED: {host} still answers after 120 s - it did not power off.")
        return 1
    print(f"{host} stopped answering. Waiting 10 s more for its disks to finish...")
    time.sleep(10)
    print("NVIDIA is off - safe to unplug.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        prog="multicam", description="Control the multi-camera system's streams and lights."
    )
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="seconds to wait for each service (default 10)")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("status", help="cameras available/streaming/recording, lights state")
    for name, helptext in (("start", "start streaming"), ("stop", "stop streaming")):
        p = sub.add_parser(name, help=f"{helptext} (default: aux_left aux_right)")
        p.add_argument("cameras", nargs="*", metavar="CAMERA",
                       help=f"{', '.join(CAMERAS)} or all")
    p = sub.add_parser("lights", help="set the lights brightness")
    p.add_argument("brightness", type=int, help="0-100 (0 = off)")
    p.add_argument("--channel", type=int, default=3, help="light channel 1-3 (default 3)")
    sub.add_parser("shutdown", help="stop all streams, lights off, power off the "
                                    "multi-camera computer (login from data/.multicam_ssh)")
    args, _ros_args = parser.parse_known_args()

    _reexec_with_cyclonedds()

    caller = Caller(args.timeout)
    try:
        return {"status": cmd_status, "start": cmd_start, "stop": cmd_stop,
                "lights": cmd_lights, "shutdown": cmd_shutdown}[args.command](caller, args)
    finally:
        caller.close()


if __name__ == "__main__":
    sys.exit(main())
