#!/usr/bin/env python3
"""Run the DVL calibration from Foxglove (or any ROS client).

Runs ``dvl_calibrate`` - the same stillness check, sound speed, gyro calibration and
dead-reckoning reset as from the terminal - as a child process. Started by
bluerov2.launch.py (``dvl_calibration:=true``).

The vehicle must be at the surface, disarmed and still. The stillness check still runs:
if the vehicle moves, the calibration aborts and nothing is changed on the DVL.

Service (std_srvs/Trigger), in the node namespace:

    dvl/calibration/start    start a calibration; it takes ~20 s (see dvl/calibration/status)

Topics:

    dvl/calibration/set_water  std_msgs/String in: "<water temp degC> <salinity ppt>",
                               e.g. "12 0" (fresh) or "10 35" (sea), so the DVL's sound
                               speed is set. Empty = leave the sound speed unchanged.
    dvl/calibration/active     std_msgs/Bool out (latched): true while it runs
    dvl/calibration/status     std_msgs/String out (latched): what is happening, in words

Parameters: ``water`` (same as set_water, default "" = sound speed unchanged),
``extra_args`` (more dvl_calibrate arguments, e.g. "--settle 15").
"""

import os
import shlex
import subprocess
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def parse_water(text: str):
    """Return (temp_c, salinity_ppt) from "12 0", None for "", ValueError otherwise."""
    fields = text.replace(",", " ").split()
    if not fields:
        return None
    if len(fields) != 2:
        raise ValueError(f"expected '<temp degC> <salinity ppt>', got {text!r}")
    temp, salinity = (float(f) for f in fields)
    if not (-2.0 <= temp <= 40.0 and 0.0 <= salinity <= 45.0):
        raise ValueError(f"out of range: {temp} degC, {salinity} ppt")
    return temp, salinity


class DvlCalibration(Node):

    def __init__(self):
        super().__init__("dvl_calibration")
        self.declare_parameter("water", "")
        self.declare_parameter("extra_args", "")

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_active = self.create_publisher(Bool, "dvl/calibration/active", latched)
        self._pub_status = self.create_publisher(String, "dvl/calibration/status", latched)
        self.create_subscription(String, "dvl/calibration/set_water", self._on_set_water, 10)
        self.create_service(Trigger, "dvl/calibration/start", self._on_start)

        self._lock = threading.Lock()
        self._proc = None
        self._set_status(False, f"idle - {self._water_text()}")

    # -- Helpers ------------------------------------------------------------------

    def _water(self):
        return parse_water(str(self.get_parameter("water").value))

    def _water_text(self) -> str:
        try:
            water = self._water()
        except ValueError as exc:
            return f"water setting invalid ({exc})"
        if water is None:
            return "sound speed unchanged - no water set"
        return f"water {water[0]:g} degC, salinity {water[1]:g} ppt"

    def _set_status(self, active: bool, text: str) -> None:
        self._pub_active.publish(Bool(data=active))
        self._pub_status.publish(String(data=text))
        self.get_logger().info(text)

    def _topic(self, name: str) -> str:
        ns = self.get_namespace().rstrip("/")
        return f"{ns}/{name}"

    # -- Callbacks ----------------------------------------------------------------

    def _on_set_water(self, msg: String) -> None:
        try:
            parse_water(msg.data)
        except ValueError as exc:
            self.get_logger().warning(f"set_water ignored: {exc}")
            with self._lock:
                busy = self._proc is not None
            self._set_status(busy, f"set_water ignored: {exc}")
            return
        self.set_parameters([rclpy.parameter.Parameter("water", value=msg.data.strip())])
        with self._lock:
            busy = self._proc is not None
        self._set_status(busy, f"{'calibrating; next run' if busy else 'idle'} - "
                               f"{self._water_text()}")

    def _on_start(self, _request, response):
        with self._lock:
            if self._proc is not None:
                response.success = False
                response.message = "a calibration is already running"
                return response
            try:
                water = self._water()
            except ValueError as exc:
                response.success = False
                response.message = f"water setting invalid: {exc}"
                self._set_status(False, response.message)
                return response
            cmd = ["ros2", "run", "bluerov2_dvl", "dvl_calibrate",
                   "--imu-topic", self._topic("imu/data"),
                   "--dr-reset-service", self._topic("dead_reckoning/reset")]
            if water is not None:
                cmd += ["--water-temp", f"{water[0]:g}", "--salinity", f"{water[1]:g}"]
            cmd += shlex.split(str(self.get_parameter("extra_args").value))
            # Unbuffered, so its progress lines arrive while it runs.
            env = dict(os.environ, PYTHONUNBUFFERED="1")
            self._proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env,
                start_new_session=True)
        threading.Thread(target=self._watch, args=(self._proc,), daemon=True).start()
        response.success = True
        response.message = f"calibrating ({self._water_text()}) - KEEP THE VEHICLE STILL"
        self._set_status(True, response.message)
        return response

    def _watch(self, proc) -> None:
        """Forward the calibration's progress to the status topic."""
        reason = ""
        record = ""
        for line in proc.stdout:
            line = line.rstrip()
            if not line:
                continue
            self.get_logger().info(f"dvl_calibrate: {line}")
            text = line.strip()
            if text.startswith("[") and "/6]" in text:
                self._set_status(True, f"calibrating: {text}")
            elif text.startswith(("ABORT", "ERROR", "NOT STILL", "WARNING")):
                reason = text
                self._set_status(True, f"calibrating: {text}")
            elif text.startswith("Record written to"):
                record = text[len("Record written to "):]
            elif text.startswith("Dry run"):
                reason = text
        code = proc.wait()
        with self._lock:
            self._proc = None
        if code == 0 and reason.startswith("Dry run"):
            self._set_status(False, "DRY RUN - nothing changed on the DVL")
        elif code == 0:
            note = f" - record {record}" if record else ""
            self._set_status(False, f"DONE - DVL calibrated, dead reckoning reset{note}")
        elif code == 1 and record:
            self._set_status(False, f"DONE WITH WARNING - {reason or 'see the log'}")
        elif code == 2:
            # dvl_calibrate exits 2 only before it changes anything on the DVL.
            self._set_status(False, f"ABORTED, nothing changed - {reason or 'see the log'}")
        else:
            self._set_status(False, f"FAILED (exit {code}) - {reason or 'see the log'}")

    def shutdown(self) -> None:
        with self._lock:
            proc = self._proc
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()


def main():
    rclpy.init()
    node = DvlCalibration()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
