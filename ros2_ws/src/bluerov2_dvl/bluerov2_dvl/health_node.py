#!/usr/bin/env python3
"""Live verdict on the DVL data, in plain words, refreshed every 2 s.

    ros2 run bluerov2_dvl dvl_health            # vehicle moving or still
    ros2 run bluerov2_dvl dvl_health --still    # also check noise: keep the vehicle still

Reads /bluerov2/dvl/report. The checks and what they mean are in bluerov2_dvl.health.

bluerov2.launch.py starts one instance with ``--publish --no-print``: it publishes the
verdicts as diagnostic_msgs/DiagnosticArray on /bluerov2/dvl/health, which Foxglove's
Diagnostics panels show as a colour-coded checklist. status[0] is the overall verdict,
then one status per check. Levels: 0 OK (GOOD), 1 WARN (CHECK), 2 ERROR (BAD).
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.qos import qos_profile_sensor_data

from bluerov2_dvl.health import BAD, GOOD, WARN, assess
from bluerov2_msgs.msg import DVLReport

COLOR = {GOOD: "\033[32m", WARN: "\033[33m", BAD: "\033[31m"}
RESET = "\033[0m"
DIAG_LEVEL = {GOOD: DiagnosticStatus.OK, WARN: DiagnosticStatus.WARN, BAD: DiagnosticStatus.ERROR}


class HealthNode(Node):

    def __init__(self, topic: str, window: float, still: bool,
                 health_topic: str = "", show: bool = True) -> None:
        super().__init__("dvl_health")
        self._window = window
        self._still = still
        self._print = show
        self._pub = self.create_publisher(DiagnosticArray, health_topic, 10) \
            if health_topic else None
        self._reports: deque = deque()
        self.create_subscription(DVLReport, topic, self._on_report, qos_profile_sensor_data)
        self.create_timer(2.0, self._show)

    def _on_report(self, msg: DVLReport) -> None:
        self._reports.append((time.monotonic(), {
            "vx": msg.velocity.x, "vy": msg.velocity.y, "vz": msg.velocity.z,
            "fom": msg.fom, "velocity_valid": msg.velocity_valid,
            "altitude": msg.altitude, "status": msg.status,
            "beams": [{"distance": b.distance, "beam_valid": b.beam_valid} for b in msg.beams],
        }))

    def _show(self) -> None:
        now = time.monotonic()
        while self._reports and now - self._reports[0][0] > self._window:
            self._reports.popleft()
        verdicts = assess([r for _, r in self._reports], self._window, self._still)
        worst = BAD if any(v[0] == BAD for v in verdicts) else \
            WARN if any(v[0] == WARN for v in verdicts) else GOOD
        if self._pub is not None:
            self._publish(worst, verdicts)
        if not self._print:
            return
        print(f"\n=== DVL {time.strftime('%H:%M:%S')}  last {self._window:.0f} s  "
              f"{COLOR[worst]}{worst}{RESET}")
        for level, name, text in verdicts:
            print(f"  {COLOR[level]}{level:5}{RESET} {name:12} {text}")

    def _publish(self, worst, verdicts) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        overall = DiagnosticStatus(
            level=DIAG_LEVEL[worst], name="DVL: overall", hardware_id="dvl_a50",
            message=worst if worst == GOOD else "; ".join(
                name for level, name, _ in verdicts if level != GOOD))
        array.status.append(overall)
        for level, name, text in verdicts:
            array.status.append(DiagnosticStatus(
                level=DIAG_LEVEL[level], name=f"DVL: {name}", hardware_id="dvl_a50",
                message=text))
        self._pub.publish(array)


def main(argv=None) -> None:
    argv = sys.argv if argv is None else argv
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--topic", default="/bluerov2/dvl/report")
    parser.add_argument("--window", type=float, default=5.0, help="seconds judged each time")
    parser.add_argument("--still", action="store_true",
                        help="the vehicle is still: also check velocity noise and bias")
    parser.add_argument("--publish", nargs="?", const="/bluerov2/dvl/health", default="",
                        help="publish the verdicts as DiagnosticArray on this topic")
    parser.add_argument("--no-print", action="store_true", help="do not print to the terminal")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(argv)[1:])
    rclpy.init(args=argv)
    node = HealthNode(args.topic, args.window, args.still, args.publish, not args.no_print)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
