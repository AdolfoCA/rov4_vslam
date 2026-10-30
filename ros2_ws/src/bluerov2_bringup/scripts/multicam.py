#!/usr/bin/env python3
"""Start/stop the multi-camera streams and set its lights, via the camera system's services.

    multicam status                        # cameras available/streaming/recording, lights
    multicam start [CAMERA ...|all]        # default: aux_left aux_right; fails unless
                                           # every one is then listed as streaming
    multicam stop  [CAMERA ...|all]        # default: aux_left aux_right;
                                           # all = every camera that is streaming
    multicam lights BRIGHTNESS [--channel N]   # 0-100, channel 1-3 (default 3)
    multicam shutdown                      # stop all streams (ours too), lights off,
                                           # power off the multi-camera computer
    multicam sync-start                    # aux pair from our own pipeline (what
                                           # session_start does by default)
    multicam sync-stop                     # back to the Blue Atlas camera software
    multicam sync-status                   # which of the two runs; exit 0 if the
                                           # camera computer is ready (preflight)
    multicam timesync-setup                # once: chrony on the camera computer, time
                                           # from the topside (capture timestamps need it)
    multicam timesync-check                # is its clock within ~2 ms of the topside
                                           # (0.5 ms + half the tether round trip)

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
        the receiving camera nodes. An attach can answer succeeded=True and the camera
        still not stream, so start then polls camera_status (up to --verify-timeout)
        and exits 1 unless every requested camera is listed as streaming.
stop    /ba/camera_feed_manager/detach_streaming_consumer {camera_device_path}
status  /ba/camera_feed_manager/camera_status, and the lifecycle state of
        /ba/light_controller
lights  /ba/light_controller/set_brightness {channel, brightness}, after bringing the
        light controller (a lifecycle node) to 'active' with configure/activate if
        needed: while 'unconfigured' it answers done=True but the lights stay dark.

The streams do not survive a reboot of the camera computer: run `multicam start` after
every power cycle (checked 2026-09-24).

sync-start / sync-stop (over SSH, login from data/.multicam_ssh)
-----------------------------------------------------------------
The Blue Atlas software re-times every camera to its own 30 Hz tick, so every ~355 s
each aux camera repeats or drops a frame and the left/right pairing can shift by one
frame, invisibly in the timestamps (2026-10-30). sync-start replaces it, for the aux
pair only, with jetson/aux_sync_pipeline.py: both cameras in one pipeline, kernel
capture timestamps, the same RTP timestamp for the same capture. It
  1. copies the script to ~/rov4_vslam/ on the camera computer;
  2. if the pipeline already runs and holds both cameras, stops here (so running
     session_start twice is harmless);
  3. waits for the Blue Atlas camera service to finish starting (right after power-on
     it may still be starting, and must not start after us and take the cameras), then
     stops it. Lights, battery and leak detector are separate services and keep
     running. The program ignores SIGTERM, so systemd kills it after 30 s: this step
     takes ~30 s, then up to 20 s more until both cameras are free;
  4. starts the script in the background; it counts as started once, for
     --verify-seconds in a row, the process runs and holds both cameras.
If a step fails it stops the pipeline and starts the Blue Atlas service again.
sync-stop stops the script and starts the Blue Atlas service; shutdown stops it before
the power-off. Nothing is enabled at boot and no Blue Atlas file is changed: a power
cycle always brings the Blue Atlas software back, as the colleague who shares the ROV
expects. While our pipeline runs, `multicam start/stop/status` find no camera service
(expected). Start the right camera node with stamp_reference_topic (bluerov2.launch.py
aux_pipeline:=sync, the default, does it) so that both cameras of a pair carry the
same stamp.
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

BA_CAMERA_UNIT = "ba-app-video-camera-feed-manager-new.service"
TIMESYNC_DROPIN = "/etc/systemd/timesyncd.conf.d/rov4_vslam.conf"   # first attempt, removed
CHRONY_DEB = os.path.expanduser(os.environ.get(
    "MULTICAM_CHRONY_DEB", "~/data/jetson_pkgs/chrony_3.5-6ubuntu6.2_arm64.deb"))
# Remote snippet: the topside address as the camera computer sees it (the SSH client).
TOPSIDE_FROM_JETSON = 'TOPSIDE=${SSH_CLIENT%% *}' 
REMOTE_DIR = "rov4_vslam"             # in the camera computer's home
PIPELINE_SCRIPT = "aux_sync_pipeline.py"
# A leftover pipeline is found by its command line.
LEFTOVER = "PIPE='[p]ython3 aux_sync_pipeline.py'"
# Remote shell function: true when our pipeline runs and holds both aux cameras.
PIPELINE_CHECK = f"""
pipeline_ok() {{
  pid=$(cat ~/{REMOTE_DIR}/aux_sync_pipeline.pid 2>/dev/null)
  [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null || return 1
  for d in /dev/video_aux_left /dev/video_aux_right; do
    fuser "$(readlink -f $d)" 2>/dev/null | grep -qw "$pid" || return 1
  done
}}
"""


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
    cameras = _cameras(args.cameras)
    failed = 0
    for camera in cameras:
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
    if start and not _wait_streaming(caller, cameras, args.verify_timeout):
        failed += 1
    print("")
    cmd_status(caller, args)
    return 1 if failed else 0


def _wait_streaming(caller, cameras, timeout: float) -> bool:
    """True once every camera is in the camera system's streaming list.

    An attach that answers succeeded=True does not guarantee the camera is streaming,
    so this asks camera_status until all are listed or `timeout` seconds pass.
    """
    import time

    deadline = time.monotonic() + timeout
    while True:
        streaming = _streaming_cameras(caller)
        missing = [c for c in cameras if streaming is None or c not in streaming]
        if not missing:
            print(f"streaming check  OK  ({', '.join(cameras)})")
            return True
        if time.monotonic() >= deadline:
            why = "camera status unavailable" if streaming is None else "not streaming"
            print(f"streaming check  FAILED  - {why}: {', '.join(missing)}")
            return False
        time.sleep(1.0)


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
    if not _stop_pipeline(login):
        # Not our aux pipeline: streams of the Blue Atlas software, if any.
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


def _ssh(login: dict, remote: str, stdin=None, timeout: float = 30):
    """Run a shell command on the camera computer; (exit code, stdout + stderr).

    stdin may be text or bytes (a file to copy over).
    """
    import subprocess

    binary = isinstance(stdin, bytes)
    try:
        result = subprocess.run(
            ["sshpass", "-e", "ssh",
             "-o", "StrictHostKeyChecking=accept-new", "-o", "ConnectTimeout=8",
             "-o", "PreferredAuthentications=password", "-o", "PubkeyAuthentication=no",
             f"{login['user']}@{login['host']}", remote],
            input=stdin, env=dict(os.environ, SSHPASS=login["password"]),
            capture_output=True, text=not binary, timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return 124, f"no answer within {timeout:.0f} s"
    out = result.stdout + result.stderr
    if binary:
        out = out.decode(errors="replace")
    return result.returncode, out.strip()


def _pipeline_script() -> str:
    from ament_index_python.packages import get_package_share_directory

    path = os.path.join(get_package_share_directory("bluerov2_bringup"), "jetson",
                        PIPELINE_SCRIPT)
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def _restart_blue_atlas(login: dict) -> bool:
    code, out = _ssh(login, f"sudo -n systemctl start {BA_CAMERA_UNIT}; sleep 3; "
                            f"systemctl is-active {BA_CAMERA_UNIT}")
    ok = code == 0 and out.splitlines()[-1:] == ["active"]
    print(f"Blue Atlas camera service started again  {'OK' if ok else 'FAILED: ' + out}")
    return ok


def cmd_sync_start(_caller, args) -> int:
    login = _read_ssh_file()
    ports = _ports()
    pl, pr = ports["aux_left"], ports["aux_right"]

    print(f"=== 1/3 copy {PIPELINE_SCRIPT} to {login['host']}:~/{REMOTE_DIR}/")
    code, out = _ssh(login, f"mkdir -p ~/{REMOTE_DIR} && cat > ~/{REMOTE_DIR}/{PIPELINE_SCRIPT}",
                     stdin=_pipeline_script())
    if code != 0:
        print(f"FAILED: {out}")
        return 1

    # Already running (session_start run again without a power cycle): leave it alone.
    code, out = _ssh(login, PIPELINE_CHECK + "\npipeline_ok && echo running")
    if code == 0 and out.endswith("running"):
        print("=== aux pipeline already running, both cameras open by it - nothing to do")
        return 0

    # Right after power-on the Blue Atlas service may still be starting: wait for it to
    # settle, so it cannot start after us and take the cameras. Then stop it. The program
    # ignores SIGTERM and systemd kills it after the unit's TimeoutStopSec (30 s), so the
    # stop takes ~30 s; then wait until both cameras are free (up to 20 s more).
    print(f"=== 2/3 stop the Blue Atlas camera service ({BA_CAMERA_UNIT}), ~30-60 s")
    code, out = _ssh(login, f"""
        {LEFTOVER}
        pkill -INT -f "$PIPE" ; sleep 1
        for i in $(seq 1 45); do
          [ "$(systemctl is-active {BA_CAMERA_UNIT})" = activating ] || break; sleep 2
        done
        echo "Blue Atlas camera service was: $(systemctl is-active {BA_CAMERA_UNIT})"
        sudo -n systemctl stop {BA_CAMERA_UNIT} || exit 2
        for i in $(seq 1 20); do
          busy=""
          for d in /dev/video_aux_left /dev/video_aux_right; do
            fuser -s "$(readlink -f $d)" && busy="$busy $d"
          done
          [ -z "$busy" ] && break; sleep 1
        done
        if [ -n "$busy" ]; then echo "still open by another program:$busy"; exit 3; fi
        echo "stopped ($(systemctl is-active {BA_CAMERA_UNIT})), cameras free" """,
                     timeout=200)
    print(out)
    if code != 0:
        print(f"FAILED (exit {code})")
        _restart_blue_atlas(login)
        return 1

    # Counts as started only when, for verify_seconds in a row, the pipeline process is
    # alive, it holds both cameras and the Blue Atlas service has not come back.
    # The pipeline sends the capture time as wall-clock time of the camera computer.
    code, report = _ssh(login, CLOCK_REPORT, timeout=30)
    print(report)
    if not _clock_ok(report):
        print("WARNING: the camera computer's clock is not within 0.5 ms + half the round "
              "trip of the topside. "
              "The stamps carry that error; if it is large the camera nodes notice and fall "
              "back to arrival-based stamps. Check: multicam timesync-check.")

    print(f"=== 3/3 start the aux pipeline -> {TOPSIDE_IP}:{pl} (left), :{pr} (right)")
    code, out = _ssh(login, PIPELINE_CHECK + f"""
        cd ~/{REMOTE_DIR} || exit 2
        nohup setsid python3 {PIPELINE_SCRIPT} {TOPSIDE_IP} {pl} {pr} > aux_sync_pipeline.log 2>&1 < /dev/null &
        echo $! > aux_sync_pipeline.pid
        good=0
        for i in $(seq 1 {int(args.verify_seconds) + 15}); do
          sleep 1
          if pipeline_ok; then good=$((good + 1)); else good=0; fi
          kill -0 "$(cat aux_sync_pipeline.pid)" 2>/dev/null || break
          [ $good -ge {int(args.verify_seconds)} ] && break
        done
        if [ $good -ge {int(args.verify_seconds)} ]; then echo "running, both cameras open by it"
        else
          echo "the pipeline did not come up:"; tail -n 20 aux_sync_pipeline.log
          echo "Blue Atlas camera service: $(systemctl is-active {BA_CAMERA_UNIT})"; exit 4
        fi""", timeout=60 + args.verify_seconds)
    print(out)
    if code != 0:
        print(f"FAILED (exit {code})")
        _stop_pipeline(login)
        _restart_blue_atlas(login)
        return 1
    print("aux pipeline running. It stays on until session_stop (power-off) or "
          "`multicam sync-stop`.")
    return 0


def _stop_pipeline(login: dict) -> bool:
    """Stop our aux pipeline (end of stream, then kill); True if it was running."""
    code, out = _ssh(login, f"""
        {LEFTOVER}
        cd ~/{REMOTE_DIR} 2>/dev/null
        pid=$(cat aux_sync_pipeline.pid 2>/dev/null)
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
          kill -INT "$pid"
          for i in $(seq 1 10); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
          kill -KILL "$pid" 2>/dev/null
          echo "aux pipeline stopped"
        else echo "aux pipeline was not running"; fi
        pkill -KILL -f "$PIPE" ; rm -f aux_sync_pipeline.pid; true""")
    print(out if code == 0 else f"stopping the aux pipeline FAILED: {out}")
    return code == 0 and out.endswith("aux pipeline stopped")


def cmd_sync_stop(_caller, _args) -> int:
    login = _read_ssh_file()
    print("=== 1/2 stop the aux pipeline")
    _stop_pipeline(login)
    print("=== 2/2 start the Blue Atlas camera service")
    return 0 if _restart_blue_atlas(login) else 1


def cmd_sync_status(_caller, _args) -> int:
    """Print what runs; exit 0 if the camera computer is ready for (or running) the pipeline.

    Ready = SSH login works and both aux cameras exist. preflight.py uses this exit code.
    """
    login = _read_ssh_file()
    code, out = _ssh(login, PIPELINE_CHECK + f"""
        echo "Blue Atlas camera service: $(systemctl is-active {BA_CAMERA_UNIT})"
        if pipeline_ok; then echo "aux pipeline: running, both cameras open by it"
        else echo "aux pipeline: not running"; fi
        for d in /dev/video_aux_left /dev/video_aux_right; do
          [ -e "$d" ] || {{ echo "missing camera: $d"; exit 5; }}
        done""", timeout=15)
    print(out)
    return code


# Remote: offset of the topside clock from this one, from the NTP query with the
# smallest round trip out of 30. ntpdate's single estimate is off by several ms on the
# tether (round trip 4-16 ms, 2026-10-30); the fastest exchange is the least skewed.
NTP_MIN_DELAY = r"""
python3 - "$TOPSIDE" <<'PY'
import socket, struct, sys, time
host, epoch, best = sys.argv[1], 2208988800, None
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(1)
for _ in range(30):
    try:
        t1 = time.time()
        s.sendto(b"\x23" + 47 * b"\0", (host, 123))
        d = s.recvfrom(48)[0]
        t4 = time.time()
    except OSError:
        continue
    t2, t3 = [a - epoch + b / 2**32 for a, b in (struct.unpack("!II", d[32:40]),
                                                     struct.unpack("!II", d[40:48]))]
    delay, offset = (t4 - t1) - (t3 - t2), ((t2 - t1) + (t3 - t4)) / 2
    if best is None or delay < best[0]:
        best = (delay, offset)
    time.sleep(0.2)
print("offset to the topside %+.6f s (round trip %.1f ms)" % (best[1], best[0] * 1e3)
      if best else "offset to the topside unknown s (no NTP answer)")
PY
"""
CLOCK_REPORT = f"""
{TOPSIDE_FROM_JETSON}
synced=$(timedatectl show -p NTPSynchronized --value)
echo "camera computer clock: synchronised=$synced, $({NTP_MIN_DELAY})"
"""


def _clock_ok(report: str) -> bool:
    """True if synchronised and |offset| <= 0.5 ms + half the round trip.

    NTP cannot tell how the round trip splits between the two directions, so an offset
    measured over the tether (round trip ~3.5 ms) is only known to +- half of it.
    """
    import re

    m = re.search(r"synchronised=(\w+), offset to the topside ([-+0-9.]+) s "
                  r"\(round trip ([0-9.]+) ms\)", report)
    if not m or m.group(1) != "yes":
        return False
    return abs(float(m.group(2))) <= 0.0005 + float(m.group(3)) / 2e3


def cmd_timesync_setup(_caller, _args) -> int:
    """Once per camera computer: install chrony there and take the time from the topside.

    The aux pipeline sends the capture time as wall-clock time, so the camera computer's
    clock must agree with the topside's. It has no internet and its clock runs ~94 ppm
    slow (0.34 s an hour, 2026-10-30). systemd-timesyncd, tried first, let it drift 5-7 ms
    between its ever rarer polls; chrony learns the frequency error and keeps only the
    exchanges with the shortest round trip. This copies the chrony package (Ubuntu 20.04
    arm64, CHRONY_DEB, checked against the Ubuntu index) over SSH, installs it without
    downloading anything, replacing systemd-timesyncd (it stops if apt plans anything
    else, e.g. upgrading systemd), writes /etc/chrony/chrony.conf
    with the topside as the only server (the original is kept as chrony.conf.debian) and
    waits for the clock to settle. The topside must serve NTP on the tether (chrony,
    TOPSIDE_SETUP.md 2g). Away from that topside chrony just lets the clock run free, as
    before. To undo: sudo apt-get install systemd-timesyncd (offline: it is in
    /var/cache/apt/archives if apt kept it) or simply sudo apt-get remove chrony.
    """
    login = _read_ssh_file()
    try:
        with open(CHRONY_DEB, "rb") as handle:
            deb = handle.read()
    except OSError as exc:
        sys.exit(f"cannot read {CHRONY_DEB} ({exc}). Download it on the host:\n"
                 "  curl -O http://ports.ubuntu.com/pool/main/c/chrony/"
                 "chrony_3.5-6ubuntu6.2_arm64.deb   (into rov4_vslam/data/jetson_pkgs/)")

    print("=== 1/3 copy the chrony package to the camera computer")
    code, out = _ssh(login, "cat > /tmp/chrony_rov4.deb", stdin=deb, timeout=60)
    if code != 0:
        print(f"FAILED: {out}")
        return 1

    print("=== 2/3 install and configure chrony (topside = only time server)")
    code, out = _ssh(login, f"""
        {TOPSIDE_FROM_JETSON}
        set -e
        if ! dpkg-query -W -f='${{Status}}' chrony 2>/dev/null | grep -q 'install ok installed'; then
          # Offline install: the package goes into apt's cache. apt must be told to remove
          # systemd-timesyncd, or it tries to upgrade systemd instead (2026-10-30). Check
          # the plan first and touch nothing unless it is exactly these two steps.
          sudo -n cp /tmp/chrony_rov4.deb /var/cache/apt/archives/{os.path.basename(CHRONY_DEB)}
          plan=$(apt-get install -s --no-download chrony systemd-timesyncd- 2>&1 | grep -E '^(Inst|Remv)')
          echo "apt plan:"; echo "$plan"
          if [ "$(echo "$plan" | grep -c .)" != 2 ] || ! echo "$plan" | grep -q '^Inst chrony ' \
             || ! echo "$plan" | grep -q '^Remv systemd-timesyncd '; then
            echo "unexpected apt plan: nothing installed"; exit 4
          fi
          sudo -n DEBIAN_FRONTEND=noninteractive apt-get install -y --no-download chrony systemd-timesyncd-
        fi
        [ -f /etc/chrony/chrony.conf.debian ] || sudo -n cp /etc/chrony/chrony.conf /etc/chrony/chrony.conf.debian
        sudo -n tee /etc/chrony/chrony.conf >/dev/null <<CONF
# rov4_vslam (multicam timesync-setup): time from the topside laptop on the tether.
# The laptop serves it with chrony. Away from it this clock just runs free.
# maxdelay drops exchanges slowed down by the tether (round trip 4-16 ms).
server $TOPSIDE iburst minpoll 4 maxpoll 6 maxdelay 0.02
keyfile /etc/chrony/chrony.keys
driftfile /var/lib/chrony/chrony.drift
logdir /var/log/chrony
maxupdateskew 100.0
rtcsync
makestep 1 3
CONF
        sudo -n rm -f {TIMESYNC_DROPIN}
        sudo -n systemctl restart chrony
        systemctl is-active chrony
        rm -f /tmp/chrony_rov4.deb""", timeout=180)
    print(out)
    if code != 0:
        print(f"FAILED (exit {code})")
        return 1

    print("=== 3/3 wait for the clock to settle (up to ~2 min)")
    code, out = _ssh(login, f"""
        chronyc waitsync 120 0.0005 0 1 >/dev/null; chronyc tracking | grep -E 'Reference|System time|Frequency|Leap'
        {CLOCK_REPORT}""", timeout=200)
    print(out)
    if not _clock_ok(out):
        print("Not within 0.5 ms + half the round trip yet - chrony keeps converging; "
              "check again in a few minutes with `multicam timesync-check`.")
        return 1
    print("OK")
    return 0


def cmd_timesync_check(_caller, _args) -> int:
    login = _read_ssh_file()
    code, out = _ssh(login, f"""
        command -v chronyc >/dev/null && chronyc tracking | grep -E 'System time|Frequency|Leap'
        {CLOCK_REPORT}""", timeout=60)
    print(out)
    return 0 if code == 0 and _clock_ok(out) else 1


SSH_COMMANDS = {"sync-start": cmd_sync_start, "sync-stop": cmd_sync_stop,
                "sync-status": cmd_sync_status, "timesync-setup": cmd_timesync_setup,
                "timesync-check": cmd_timesync_check}


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
        if name == "start":
            p.add_argument("--verify-timeout", type=float, default=10.0,
                           help="seconds to wait for every camera to be listed as "
                                "streaming (default 10)")
    p = sub.add_parser("lights", help="set the lights brightness")
    p.add_argument("brightness", type=int, help="0-100 (0 = off)")
    p.add_argument("--channel", type=int, default=3, help="light channel 1-3 (default 3)")
    sub.add_parser("shutdown", help="stop all streams, lights off, power off the "
                                    "multi-camera computer (login from data/.multicam_ssh)")
    p = sub.add_parser("sync-start", help="stream the aux pair from our own pipeline "
                                          "(capture timestamps) instead of Blue Atlas")
    p.add_argument("--verify-seconds", type=float, default=5.0,
                   help="seconds in a row the pipeline must run holding both cameras "
                        "to count as started (default 5)")
    sub.add_parser("sync-stop", help="stop our aux pipeline, start Blue Atlas again")
    sub.add_parser("timesync-setup", help="once: install chrony on the camera computer, "
                                          "time from the topside (needs chrony there too)")
    sub.add_parser("timesync-check", help="is the camera computer's clock synchronised "
                                          "to the topside (exit 0 if within 0.5 ms + half "
                                          "the round trip)")
    sub.add_parser("sync-status", help="is Blue Atlas or our aux pipeline running; exit 0 "
                                       "if the camera computer answers and has both aux cameras")
    args, _ros_args = parser.parse_known_args()

    if args.command in SSH_COMMANDS:  # SSH only: no camera system services involved
        return SSH_COMMANDS[args.command](None, args)

    _reexec_with_cyclonedds()

    caller = Caller(args.timeout)
    try:
        return {"status": cmd_status, "start": cmd_start, "stop": cmd_stop,
                "lights": cmd_lights, "shutdown": cmd_shutdown}[args.command](caller, args)
    finally:
        caller.close()


if __name__ == "__main__":
    sys.exit(main())
