#!/usr/bin/env python3
"""Check that sensor streams are publishing, before a recording starts.

    ros2 run bluerov2_bringup check_streams.py --timeout 10 --window 3 \
        imu=/bluerov2/imu/data_raw:sensor_msgs/msg/Imu:50 \
        dvl=/bluerov2/dvl/report:bluerov2_msgs/msg/DVLReport:1

Each argument is LABEL=TOPIC:TYPE:MIN_HZ. The script subscribes to every topic, waits
until each has delivered a first message (or --timeout expires), then counts messages
for --window seconds. It prints one line per stream and exits 0 only if every stream
reached its minimum rate, 1 otherwise. record.launch.py runs it before the recorder.

Camera streams are checked on camera_info, not on the image itself: it is published
from the same callback as every frame, so it has the frame rate, but it is a few
hundred bytes instead of megabytes.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def parse_stream(spec: str):
    label, _, rest = spec.partition("=")
    topic, type_name, min_hz = rest.split(":", 2)
    return label, topic, type_name, float(min_hz)


class StreamCheck(Node):
    def __init__(self, streams):
        super().__init__("stream_check")
        self.counts = {}
        self.errors = {}
        self._subs = []
        for label, topic, type_name, _ in streams:
            self.counts[label] = 0
            try:
                msg_type = get_message(type_name)
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                self.errors[label] = f"unknown type {type_name} ({exc})"
                continue
            self._subs.append(
                self.create_subscription(
                    msg_type, topic, self._make_callback(label), qos_profile_sensor_data
                )
            )

    def _make_callback(self, label):
        def callback(_msg):
            self.counts[label] += 1

        return callback

    def spin_for(self, seconds, stop_when=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if stop_when is not None and stop_when():
                return


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("streams", nargs="+", help="LABEL=TOPIC:TYPE:MIN_HZ")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="seconds to wait for the first message of every stream")
    parser.add_argument("--window", type=float, default=3.0,
                        help="seconds over which the rate is measured")
    args, _ros_args = parser.parse_known_args()

    streams = [parse_stream(s) for s in args.streams]
    rclpy.init()
    node = StreamCheck(streams)
    try:
        # Phase 1: discovery. Wait until every stream has delivered something.
        node.spin_for(
            args.timeout,
            stop_when=lambda: all(node.counts[s[0]] > 0 for s in streams if s[0] not in node.errors),
        )
        seen = {label: node.counts[label] > 0 for label, *_ in streams}

        # Phase 2: rate. Count messages over a fixed window.
        for label in node.counts:
            node.counts[label] = 0
        started = time.monotonic()
        node.spin_for(args.window)
        elapsed = time.monotonic() - started
    finally:
        node.destroy_node()
        rclpy.shutdown()

    failed = []
    print("")
    print(f"{'STREAM':<16}{'TOPIC':<46}{'RATE':>10}{'MIN':>8}  RESULT")
    for label, topic, _, min_hz in streams:
        if label in node.errors:
            rate_txt, result = "-", f"FAIL  {node.errors[label]}"
        elif not seen[label]:
            rate_txt, result = "0.0 Hz", f"FAIL  no data within {args.timeout:.0f} s"
        else:
            rate = node.counts[label] / elapsed
            rate_txt = f"{rate:.1f} Hz"
            result = "OK" if rate >= min_hz else "FAIL  rate too low"
        if not result.startswith("OK"):
            failed.append(label)
        print(f"{label:<16}{topic:<46}{rate_txt:>10}{min_hz:>7.0f}  {result}")
    print("")
    if failed:
        print(f"STREAM CHECK FAILED: {', '.join(failed)}")
        return 1
    print(f"STREAM CHECK PASSED: all {len(streams)} streams are publishing")
    return 0


if __name__ == "__main__":
    sys.exit(main())
