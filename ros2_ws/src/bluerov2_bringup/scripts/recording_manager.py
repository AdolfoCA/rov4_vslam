#!/usr/bin/env python3
"""Start and stop recordings from Foxglove (or any ROS client).

Runs ``record.launch.py`` - the same stream check and recorder as from the terminal -
as a child process. Started by bluerov2.launch.py (``recording_control:=true``).

Services (std_srvs/Trigger), in the node namespace:

    recording/start    start a recording (stream check first; see recording/status)
    recording/stop     stop it; the bag is closed cleanly (SIGINT)

Topics:

    recording/set_folder  std_msgs/String in: where the next recording goes.
                          A relative name is a subfolder of the data directory
                          (/home/rosdev/data = the project's data/ folder on the host),
                          e.g. "pool_test" or "dives/day1". Empty = the data directory.
    recording/active      std_msgs/Bool out (latched): true while the recorder runs
    recording/status      std_msgs/String out (latched): what is happening, in words

Parameters: ``data_dir`` (default /home/rosdev/data), ``folder`` (same as set_folder),
``record_args`` (extra record.launch.py arguments, e.g. "camera:=true stereo_bottom:=true").
"""

import os
import shlex
import shutil
import signal
import subprocess
import threading
from datetime import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class RecordingManager(Node):

    def __init__(self):
        super().__init__("recording_manager")
        self.declare_parameter("data_dir", "/home/rosdev/data")
        self.declare_parameter("folder", "")
        self.declare_parameter("record_args", "")
        self.declare_parameter("namespace", "bluerov2")

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_active = self.create_publisher(Bool, "recording/active", latched)
        self._pub_status = self.create_publisher(String, "recording/status", latched)
        self.create_subscription(String, "recording/set_folder", self._on_set_folder, 10)
        self.create_service(Trigger, "recording/start", self._on_start)
        self.create_service(Trigger, "recording/stop", self._on_stop)

        self._lock = threading.Lock()
        self._proc = None
        self._bag_path = ""
        self._recording = False
        self._set_status(False, f"idle - next recording goes to {self._output_dir()}")

    # -- Helpers ------------------------------------------------------------------

    def _output_dir(self) -> str:
        data_dir = str(self.get_parameter("data_dir").value)
        folder = str(self.get_parameter("folder").value).strip()
        return os.path.normpath(os.path.join(data_dir, os.path.expanduser(folder)))

    def _set_status(self, active: bool, text: str) -> None:
        self._pub_active.publish(Bool(data=active))
        self._pub_status.publish(String(data=text))
        self.get_logger().info(text)

    # -- Callbacks ----------------------------------------------------------------

    def _on_set_folder(self, msg: String) -> None:
        folder = msg.data.strip()
        self.set_parameters([rclpy.parameter.Parameter("folder", value=folder)])
        target = self._output_dir()
        data_dir = os.path.normpath(str(self.get_parameter("data_dir").value))
        note = "" if target.startswith(data_dir) else \
            " - WARNING: outside the data directory, lost when the container is removed"
        with self._lock:
            busy = self._proc is not None
        self._set_status(busy, f"{'recording; next' if busy else 'idle - next'} recording "
                               f"goes to {target}{note}")

    def _on_start(self, _request, response):
        with self._lock:
            if self._proc is not None:
                response.success = False
                response.message = f"already recording to {self._bag_path}"
                return response
            output_dir = self._output_dir()
            try:
                os.makedirs(output_dir, exist_ok=True)
            except OSError as exc:
                response.success = False
                response.message = f"cannot create {output_dir}: {exc}"
                self._set_status(False, response.message)
                return response
            bag_name = datetime.now().strftime("rov_%Y%m%d_%H%M%S")
            self._bag_path = os.path.join(output_dir, bag_name)
            free_gb = shutil.disk_usage(output_dir).free / 1e9
            cmd = ["ros2", "launch", "bluerov2_bringup", "record.launch.py",
                   f"output_dir:={output_dir}", f"bag_name:={bag_name}",
                   f"namespace:={self.get_parameter('namespace').value}"]
            cmd += shlex.split(str(self.get_parameter("record_args").value))
            # Own process group, so stop reaches ros2 launch and everything under it.
            self._proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                start_new_session=True)
            self._recording = False
        threading.Thread(target=self._watch, args=(self._proc,), daemon=True).start()
        response.success = True
        response.message = f"checking streams, then recording to {self._bag_path}"
        self._set_status(True, f"{response.message} ({free_gb:.0f} GB free)")
        return response

    def _on_stop(self, _request, response):
        with self._lock:
            proc = self._proc
        if proc is None:
            response.success = False
            response.message = "not recording"
            return response
        os.killpg(proc.pid, signal.SIGINT)
        response.success = True
        response.message = f"stopping; closing {self._bag_path}"
        self._set_status(True, response.message)
        return response

    def _watch(self, proc) -> None:
        """Follow the recorder's output: report the check result and the exit."""
        reason = ""
        for line in proc.stdout:
            line = line.rstrip()
            if "Recording to" in line:
                self._recording = True
                self._set_status(True, f"RECORDING to {self._bag_path}")
            elif "Recording NOT started" in line or "FAIL" in line or "Error" in line:
                reason = line.split("]: ", 1)[-1]
                self.get_logger().warning(line)
        proc.wait()
        with self._lock:
            self._proc = None
        if self._recording:
            self._set_status(False, f"stopped - saved {self._bag_path} - next recording goes "
                                    f"to {self._output_dir()}")
        else:
            self._set_status(False, "recording NOT started: " + (reason or "see the log"))

    def shutdown(self) -> None:
        with self._lock:
            proc = self._proc
        if proc is not None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=35)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)


def _raise_keyboard_interrupt(*_args):
    raise KeyboardInterrupt


def main():
    rclpy.init()
    # launch stops nodes with SIGINT, then SIGTERM. Treat SIGTERM like Ctrl-C, so the
    # recorder is still stopped cleanly and never left running on its own.
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    node = RecordingManager()
    try:
        # Spin in short slices: rclpy.spin() blocks in C, where a Python signal handler
        # (the SIGTERM one above) never gets to run.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
