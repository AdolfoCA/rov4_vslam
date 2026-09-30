#!/usr/bin/env python3
"""Calibrate the Water Linked DVL-A50 with the vehicle floating still at the surface.

    ros2 run bluerov2_dvl dvl_calibrate                          # check, calibrate, reset
    ros2 run bluerov2_dvl dvl_calibrate --water-temp 12 --salinity 0   # also sound speed
    ros2 run bluerov2_dvl dvl_calibrate --dry-run                # check only, change nothing

What it does, in order:

1. ``get_config``: print the DVL's current configuration.
2. Stillness check (``--settle`` seconds). The gyro calibration measures the gyro bias,
   so any rotation while it runs ends up in the bias. Two signals are used:
   the vehicle IMU (``/bluerov2/imu/data`` angular rate), when the IMU driver runs,
   and the DVL velocity, when the DVL has bottom lock. With neither, the check
   cannot be made and the script stops unless ``--force`` is given.
3. ``set_config speed_of_sound`` (only if ``--speed-of-sound`` or ``--water-temp`` is
   given). The DVL scales every velocity by the sound speed, so a wrong value is a
   scale error on all velocities: fresh water at 10 degC is ~1447 m/s, sea water at
   10 degC ~1490 m/s, a 3 % difference.
4. ``calibrate_gyro``: the DVL's own gyro bias calibration. Keep the vehicle still.
5. ``reset_dead_reckoning``: the DVL's dead-reckoned position restarts at zero here,
   so ``dvl/dead_reckoning`` is relative to the surface start point. The IMU + DVL
   track of ``dead_reckoning_node`` is reset too, if that node runs.
6. A summary, also written as JSON to ``--output`` (default ``~/dvl_calibration``).

The robot must be at the surface. The script cannot check that: the IMU driver does not
publish depth. The DVL must be in water (the transducers must not run in air).

The script opens its own TCP connection to the DVL, so ``dvl_node`` can keep running.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import statistics
import sys
import threading
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from bluerov2_dvl.protocol import encode_command, is_response_to, speed_of_sound, split_lines

DEFAULT_HOST = os.environ.get("DVL_IP", "192.168.2.128")
DEFAULT_PORT = int(os.environ.get("DVL_PORT", "16171"))


class DvlLink:
    """A command connection to the DVL that also keeps its velocity reports."""

    def __init__(self, host: str, port: int, timeout: float = 5.0) -> None:
        self._sock = socket.create_connection((host, port), timeout=timeout)
        self._sock.settimeout(0.5)
        self._buffer = b""
        self.velocity_reports: List[Dict[str, Any]] = []

    def close(self) -> None:
        self._sock.close()

    def read(self, duration: float) -> List[Dict[str, Any]]:
        """Read for ``duration`` seconds; keep velocity reports, return all others."""
        others: List[Dict[str, Any]] = []
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            try:
                chunk = self._sock.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError("DVL closed the connection")
            self._buffer += chunk
            lines, self._buffer = split_lines(self._buffer)
            for line in lines:
                try:
                    report = json.loads(line.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue
                if report.get("type") == "velocity":
                    self.velocity_reports.append(report)
                else:
                    others.append(report)
                    if report.get("type") == "response":
                        return others
        return others

    def request(
        self, command: str, parameters: Optional[Dict[str, Any]] = None,
        timeout: float = 10.0,
    ) -> Dict[str, Any]:
        """Send a command and wait for its response."""
        self._sock.sendall(encode_command(command, parameters))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for report in self.read(deadline - time.monotonic()):
                if is_response_to(report, command):
                    if not report.get("success", False):
                        raise RuntimeError(
                            f"{command} failed: {report.get('error_message') or 'no reason given'}"
                        )
                    return report
        raise TimeoutError(f"no response to {command} within {timeout:.0f} s")


class GyroMonitor(Node):
    """Collect the vehicle IMU's angular rate magnitude."""

    def __init__(self, topic: str, reset_service: str) -> None:
        super().__init__("dvl_calibrate")
        self._lock = threading.Lock()
        self._rates: List[float] = []
        self.create_subscription(Imu, topic, self._on_imu, qos_profile_sensor_data)
        self.reset_client = self.create_client(Trigger, reset_service)

    def _on_imu(self, msg: Imu) -> None:
        w = msg.angular_velocity
        with self._lock:
            self._rates.append(math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z))

    def reset_dead_reckoning(self) -> Optional[str]:
        """Call dead_reckoning_node's reset; None when that node is not running."""
        if not self.reset_client.wait_for_service(timeout_sec=1.0):
            return None
        future = self.reset_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.result().message if future.done() else "no answer"

    def take(self) -> List[float]:
        with self._lock:
            rates, self._rates = self._rates, []
        return rates


def motion_summary(gyro: List[float], velocity: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Summarise how much the vehicle moved over one window."""
    valid = [r for r in velocity if r.get("velocity_valid")]
    speeds = [math.sqrt(r["vx"] ** 2 + r["vy"] ** 2 + r["vz"] ** 2) for r in valid]
    altitudes = [r["altitude"] for r in valid if r.get("altitude", -1) > 0]
    return {
        "imu_samples": len(gyro),
        "gyro_rate_mean_dps": math.degrees(statistics.fmean(gyro)) if gyro else None,
        "gyro_rate_max_dps": math.degrees(max(gyro)) if gyro else None,
        "dvl_reports": len(velocity),
        "dvl_valid_reports": len(valid),
        "dvl_speed_mean_mps": statistics.fmean(speeds) if speeds else None,
        "dvl_velocity_mean_mps": [
            statistics.fmean(r[k] for r in valid) for k in ("vx", "vy", "vz")
        ] if valid else None,
        "dvl_fom_mean_mps": statistics.fmean(r["fom"] for r in valid) if valid else None,
        "altitude_mean_m": statistics.fmean(altitudes) if altitudes else None,
    }


def check_still(summary: Dict[str, Any], max_gyro_dps: float, max_speed: float) -> List[str]:
    """Return the reasons the vehicle is not still enough; empty if it is."""
    problems = []
    if summary["gyro_rate_mean_dps"] is not None and summary["gyro_rate_mean_dps"] > max_gyro_dps:
        problems.append(
            f"IMU angular rate {summary['gyro_rate_mean_dps']:.2f} deg/s "
            f"> {max_gyro_dps:.2f} deg/s"
        )
    if summary["dvl_speed_mean_mps"] is not None and summary["dvl_speed_mean_mps"] > max_speed:
        problems.append(
            f"DVL speed {summary['dvl_speed_mean_mps']:.3f} m/s > {max_speed:.3f} m/s"
        )
    return problems


def fmt(value: Any, spec: str = ".3f") -> str:
    return "n/a" if value is None else format(value, spec)


def print_summary(title: str, s: Dict[str, Any]) -> None:
    print(f"  {title}")
    print(f"    IMU:  {s['imu_samples']} samples, angular rate mean "
          f"{fmt(s['gyro_rate_mean_dps'], '.2f')} deg/s, max {fmt(s['gyro_rate_max_dps'], '.2f')}")
    print(f"    DVL:  {s['dvl_valid_reports']}/{s['dvl_reports']} reports with bottom lock, "
          f"speed {fmt(s['dvl_speed_mean_mps'])} m/s, fom {fmt(s['dvl_fom_mean_mps'])} m/s, "
          f"altitude {fmt(s['altitude_mean_m'], '.2f')} m")


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate the DVL with the vehicle floating still at the surface.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--imu-topic", default="/bluerov2/imu/data")
    parser.add_argument("--dr-reset-service", default="/bluerov2/dead_reckoning/reset",
                        help="dead_reckoning_node reset, called with reset_dead_reckoning")
    parser.add_argument("--settle", type=float, default=10.0,
                        help="stillness check before calibrating [s]")
    parser.add_argument("--hold", type=float, default=5.0,
                        help="keep still this long after calibrate_gyro answers [s]")
    parser.add_argument("--max-gyro", type=float, default=1.0,
                        help="max mean IMU angular rate to count as still [deg/s]")
    parser.add_argument("--max-speed", type=float, default=0.05,
                        help="max mean DVL speed to count as still [m/s]")
    sos = parser.add_mutually_exclusive_group()
    sos.add_argument("--speed-of-sound", type=float,
                     help="set this sound speed on the DVL [m/s]")
    sos.add_argument("--water-temp", type=float,
                     help="water temperature [degC]; sound speed is computed from it")
    parser.add_argument("--salinity", type=float, default=0.0,
                        help="with --water-temp: salinity [ppt], 0 fresh, ~35 sea")
    parser.add_argument("--no-gyro", action="store_true", help="skip calibrate_gyro")
    parser.add_argument("--no-reset", action="store_true", help="skip reset_dead_reckoning")
    parser.add_argument("--force", action="store_true",
                        help="calibrate even when stillness cannot be checked or fails")
    parser.add_argument("--dry-run", action="store_true",
                        help="only read the config and check stillness")
    parser.add_argument("--output", default=os.path.expanduser("~/dvl_calibration"),
                        help="directory for the JSON record ('' to skip)")
    return parser.parse_args(argv)


def run(args: argparse.Namespace, gyro: GyroMonitor) -> int:
    record: Dict[str, Any] = {"time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                              "host": args.host, "steps": []}

    print(f"[1/6] Connecting to DVL at {args.host}:{args.port}")
    link = DvlLink(args.host, args.port)
    try:
        config = link.request("get_config").get("result") or {}
        record["config_before"] = config
        print("      config: " + ", ".join(f"{k}={v}" for k, v in config.items()))
        if config.get("acoustic_enabled") is False:
            print("      WARNING: acoustics are disabled: no velocity, no bottom lock.")

        print(f"[2/6] Stillness check: keep the vehicle still for {args.settle:.0f} s")
        gyro.take()
        link.velocity_reports.clear()
        link.read(args.settle)
        before = motion_summary(gyro.take(), link.velocity_reports)
        record["before"] = before
        print_summary("before calibration", before)

        if before["imu_samples"] == 0 and before["dvl_valid_reports"] == 0:
            msg = (f"no IMU data on {args.imu_topic} and no DVL bottom lock: "
                   "stillness cannot be checked")
            if not (args.force or args.dry_run):
                print(f"      ABORT: {msg}. Start the IMU driver, or pass --force.")
                return 2
            print(f"      WARNING: {msg}.")
        elif before["dvl_valid_reports"] == 0:
            print("      No DVL bottom lock (normal if the bottom is out of range): "
                  "using the IMU only.")
        problems = check_still(before, args.max_gyro, args.max_speed)
        if problems:
            print("      NOT STILL: " + "; ".join(problems))
            if not (args.force or args.dry_run):
                print("      ABORT: wait for calmer water or hold the vehicle, then rerun "
                      "(or --force).")
                return 2
        else:
            print("      Still enough.")

        if args.dry_run:
            print("Dry run: nothing changed on the DVL.")
            return 0

        target = args.speed_of_sound
        if args.water_temp is not None:
            target = speed_of_sound(args.water_temp, args.salinity)
        if target is not None:
            print(f"[3/6] set_config speed_of_sound = {target:.1f} m/s "
                  f"(was {config.get('speed_of_sound', 'unknown')})")
            link.request("set_config", {"speed_of_sound": round(target, 1)})
            record["steps"].append({"set_config": {"speed_of_sound": round(target, 1)}})
        else:
            print(f"[3/6] Sound speed unchanged ({config.get('speed_of_sound', 'unknown')} m/s); "
                  "pass --water-temp/--salinity to set it")

        if args.no_gyro:
            print("[4/6] calibrate_gyro skipped")
        else:
            print("[4/6] calibrate_gyro: KEEP STILL")
            gyro.take()
            link.velocity_reports.clear()
            link.request("calibrate_gyro", timeout=30.0)
            print(f"      DVL answered; holding still {args.hold:.0f} s more")
            link.read(args.hold)
            during = motion_summary(gyro.take(), link.velocity_reports)
            record["during_gyro_calibration"] = during
            record["steps"].append("calibrate_gyro")
            print_summary("during calibration", during)
            problems = check_still(during, args.max_gyro, args.max_speed)
            if problems:
                print("      WARNING: vehicle moved during calibration ("
                      + "; ".join(problems) + "). Rerun the script.")
                record["warning"] = "moved during gyro calibration"

        if args.no_reset:
            print("[5/6] reset_dead_reckoning skipped")
        else:
            link.request("reset_dead_reckoning")
            record["steps"].append("reset_dead_reckoning")
            print("[5/6] reset_dead_reckoning: DVL position is now (0, 0, 0) here")
            answer = gyro.reset_dead_reckoning()
            print("      IMU + DVL track: " + (answer or "dead_reckoning_node not running"))

        record["config_after"] = link.request("get_config").get("result") or {}
    finally:
        link.close()

    print("[6/6] Done.")
    if args.output:
        os.makedirs(args.output, exist_ok=True)
        path = os.path.join(args.output, f"dvl_calibration_{time.strftime('%Y%m%d_%H%M%S')}.json")
        with open(path, "w") as f:
            json.dump(record, f, indent=2)
        print(f"      Record written to {path}")
    return 1 if "warning" in record else 0


def main(argv: Optional[List[str]] = None) -> None:
    argv = sys.argv if argv is None else argv
    args = parse_args(rclpy.utilities.remove_ros_args(argv)[1:])

    rclpy.init(args=argv)
    gyro = GyroMonitor(args.imu_topic, args.dr_reset_service)
    executor = SingleThreadedExecutor()
    executor.add_node(gyro)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        code = run(args, gyro)
    except (OSError, RuntimeError, TimeoutError) as exc:
        print(f"ERROR: {exc}")
        code = 1
    except KeyboardInterrupt:
        code = 130
    finally:
        executor.shutdown()
        spinner.join(timeout=2.0)
        gyro.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
