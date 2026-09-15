#!/usr/bin/env python3
"""ROS 2 driver for a Water Linked DVL-A50 on a BlueROV2 Heavy.

Protocol
--------
The A50 exposes a very simple interface: a TCP server on port 16171 that pushes
newline-delimited JSON objects, one per line, with no request needed. Two object types
matter here, distinguished by their ``type`` field:

``velocity``
    The bottom-track solution: a 3D velocity, a figure of merit, a 3x3 covariance, the
    altitude above the bottom, and per-transducer detail (along-beam velocity, slant
    range, RSSI, noise floor, validity). Arrives at up to ~15 Hz depending on altitude,
    since the ping period is bounded by the two-way travel time.

``position_local``
    The instrument's own dead-reckoned pose, integrated internally. Useful as a
    reference to compare an external estimator against; not a navigation solution.

Frames
------
The A50 reports velocity in its own body frame, which follows the FRD convention
(x forward, y right, z down when the instrument looks at the seabed). ROS REP-103 wants
FLU, so by default this driver rotates every vector and the covariance by 180 degrees
about x before publishing. Set ``rotate_to_flu`` to false if you would rather handle
the convention downstream.

Covariance
----------
When the instrument supplies a covariance, it is used as-is (rotated). When it does not,
or when the values are non-finite, the driver falls back to an isotropic covariance
built from the figure of merit, which the A50 defines as the standard deviation of the
velocity estimate. That fallback is the honest choice: ``fom`` is the only uncertainty
the instrument is willing to commit to.

The angular block of the published TwistWithCovariance is marked unknown with a large
diagonal, because a DVL measures no rotation at all.
"""

from __future__ import annotations

import json
import math
import socket
import threading
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from bluerov2_dvl.protocol import (
    parse_beams,
    rotate_vector_frd_to_flu,
    split_lines,
    velocity_covariance,
)
from bluerov2_msgs.msg import DVLBeam, DVLDeadReckoning, DVLReport

#: Diagonal value used to mark a degree of freedom the sensor cannot observe.
UNOBSERVED_VARIANCE = 1.0e6

#: Read chunk size. A velocity report is a few hundred bytes, so this is generous.
RECV_CHUNK = 4096


def _quat_from_euler(roll: float, pitch: float, yaw: float):
    """Return an ``(x, y, z, w)`` quaternion from an intrinsic Z-Y-X euler triple."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class WaterLinkedDvlNode(Node):
    """Stream a Water Linked DVL's JSON reports onto ROS 2 topics."""

    def __init__(self) -> None:
        super().__init__("dvl_node")

        # --- Parameters ------------------------------------------------------------
        # TODO(deployment): 192.168.2.95 is the address Water Linked ships the A50 with
        # when it is wired into the BlueROV2's 192.168.2.0/24 network. Confirm it with
        # the DVL's own web UI before a field trial, and set it here.
        self.declare_parameter("host", "192.168.2.95")
        self.declare_parameter("port", 16171)

        self.declare_parameter("frame_id", "dvl_link")
        self.declare_parameter("odom_frame_id", "dvl_odom")
        self.declare_parameter("rotate_to_flu", True)

        self.declare_parameter("publish_dead_reckoning", True)
        self.declare_parameter("publish_range", True)

        # Fallback uncertainty when the instrument sends none. 0.02 m/s is a plausible
        # order of magnitude for an A50 with good bottom lock; measure your own.
        self.declare_parameter("fallback_velocity_stddev", 0.02)
        self.declare_parameter("dead_reckoning_position_stddev", 0.1)

        # A50 acoustics: 4 beams on a 22.5 degree cone, valid roughly 0.05-50 m.
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 50.0)
        self.declare_parameter("beam_field_of_view", 0.3927)  # 22.5 deg in rad

        self.declare_parameter("connect_timeout_s", 5.0)
        self.declare_parameter("read_timeout_s", 2.0)
        self.declare_parameter("reconnect_period_s", 3.0)

        # If true, drop velocity reports the instrument itself flags as invalid instead
        # of publishing them with a large covariance. Most estimators are happier with a
        # gap than with a confidently wrong measurement, so this defaults to true.
        self.declare_parameter("drop_invalid_velocity", True)

        p = self.get_parameter
        self._frame_id: str = p("frame_id").value
        self._odom_frame_id: str = p("odom_frame_id").value
        self._rotate: bool = bool(p("rotate_to_flu").value)
        self._fallback_var = float(p("fallback_velocity_stddev").value) ** 2
        self._dr_var = float(p("dead_reckoning_position_stddev").value) ** 2
        self._drop_invalid = bool(p("drop_invalid_velocity").value)

        # --- Publishers --------------------------------------------------------------
        self._pub_twist = self.create_publisher(
            TwistWithCovarianceStamped, "dvl/velocity", qos_profile_sensor_data
        )
        self._pub_report = self.create_publisher(
            DVLReport, "dvl/report", qos_profile_sensor_data
        )
        self._pub_range = self.create_publisher(
            Range, "dvl/altitude", qos_profile_sensor_data
        )
        self._pub_dr = self.create_publisher(
            DVLDeadReckoning, "dvl/dead_reckoning", qos_profile_sensor_data
        )
        self._pub_odom = self.create_publisher(
            Odometry, "dvl/dead_reckoning_odometry", qos_profile_sensor_data
        )

        # --- State --------------------------------------------------------------------
        self._socket: Optional[socket.socket] = None
        self._stop = threading.Event()
        self._buffer = b""
        self._counters: Dict[str, int] = {}
        self._counter_lock = threading.Lock()
        self._invalid_streak = 0

        self._rx_thread = threading.Thread(target=self._link_loop, daemon=True)
        self._rx_thread.start()
        self._stats_timer = self.create_timer(5.0, self._log_statistics)

        self.get_logger().info(
            f"DVL driver starting, target {p('host').value}:{p('port').value}, "
            f"frame {self._frame_id}, rotate_to_flu={self._rotate}"
        )

    # -- Link management ----------------------------------------------------------

    def _link_loop(self) -> None:
        reconnect_period = float(self.get_parameter("reconnect_period_s").value)
        while not self._stop.is_set() and rclpy.ok():
            try:
                self._connect()
                self._pump()
            except Exception as exc:  # noqa: BLE001 - a driver must survive any link fault
                self.get_logger().error(f"DVL link failed: {exc}")
            finally:
                self._close()
            if not self._stop.is_set():
                self.get_logger().info(f"Reconnecting in {reconnect_period:.1f} s")
                self._stop.wait(reconnect_period)

    def _connect(self) -> None:
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        connect_timeout = float(self.get_parameter("connect_timeout_s").value)
        read_timeout = float(self.get_parameter("read_timeout_s").value)

        self.get_logger().info(f"Connecting to DVL at {host}:{port}")
        sock = socket.create_connection((host, port), timeout=connect_timeout)
        sock.settimeout(read_timeout)
        # Bottom-track reports are small and latency matters, so do not let Nagle
        # coalesce them.
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._socket = sock
        self._buffer = b""
        self.get_logger().info("DVL connected")

    def _close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None

    def _pump(self) -> None:
        """Read the socket and dispatch every complete JSON line."""
        while not self._stop.is_set() and rclpy.ok():
            sock = self._socket
            if sock is None:
                raise ConnectionError("DVL socket closed")

            try:
                chunk = sock.recv(RECV_CHUNK)
            except socket.timeout:
                # A silent DVL is not necessarily a broken one (it stops reporting when
                # it cannot ping), so keep waiting rather than tearing the link down.
                self.get_logger().warning(
                    "No data from the DVL for a full read timeout",
                    throttle_duration_sec=10.0,
                )
                continue

            if not chunk:
                raise ConnectionError("DVL closed the connection")

            self._buffer += chunk
            lines, self._buffer = split_lines(self._buffer)
            for line in lines:
                self._handle_line(line)

    def _handle_line(self, line: bytes) -> None:
        try:
            report = json.loads(line.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Discarding malformed DVL line: {exc}")
            return

        report_type = report.get("type")
        if report_type == "velocity":
            self._on_velocity(report)
        elif report_type == "position_local":
            self._on_position_local(report)
        else:
            self.get_logger().debug(f"Ignoring DVL report of type {report_type!r}")
            return
        self._count(str(report_type))

    # -- Report handlers ----------------------------------------------------------

    def _on_velocity(self, report: Dict[str, Any]) -> None:
        stamp = self.get_clock().now().to_msg()

        velocity_valid = bool(report.get("velocity_valid", False))
        if not velocity_valid:
            self._invalid_streak += 1
            if self._invalid_streak in (1, 10) or self._invalid_streak % 100 == 0:
                self.get_logger().warning(
                    f"DVL reports no bottom lock ({self._invalid_streak} consecutive "
                    f"invalid reports)"
                )
        else:
            if self._invalid_streak:
                self.get_logger().info(
                    f"DVL regained bottom lock after {self._invalid_streak} invalid reports"
                )
            self._invalid_streak = 0

        velocity = (
            float(report.get("vx", 0.0)),
            float(report.get("vy", 0.0)),
            float(report.get("vz", 0.0)),
        )
        if self._rotate:
            velocity = rotate_vector_frd_to_flu(velocity)
        vx, vy, vz = velocity

        fom = float(report.get("fom", float("nan")))
        covariance = velocity_covariance(report, self._fallback_var, self._rotate)
        altitude = float(report.get("altitude", -1.0))

        # Full report: always published, even when the lock is bad, because a
        # diagnostics or logging consumer wants to see the bad ones too.
        msg = DVLReport()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.velocity.x, msg.velocity.y, msg.velocity.z = vx, vy, vz
        msg.covariance = covariance
        msg.fom = fom if math.isfinite(fom) else -1.0
        msg.velocity_valid = velocity_valid
        msg.altitude = altitude
        msg.beams = [self._to_beam_msg(beam) for beam in parse_beams(report)]
        msg.time_delta = float(report.get("time", 0.0)) * 1.0e-3  # instrument sends ms
        msg.status = int(report.get("status", 0))
        msg.format = str(report.get("format", ""))
        msg.time_of_validity = int(report.get("time_of_validity", 0))
        msg.time_of_transmission = int(report.get("time_of_transmission", 0))
        self._pub_report.publish(msg)

        if velocity_valid or not self._drop_invalid:
            twist = TwistWithCovarianceStamped()
            twist.header.stamp = stamp
            twist.header.frame_id = self._frame_id
            twist.twist.twist.linear.x = vx
            twist.twist.twist.linear.y = vy
            twist.twist.twist.linear.z = vz
            # Row-major 6x6: linear block from the DVL, angular block marked unobserved.
            full = [0.0] * 36
            for i in range(3):
                for j in range(3):
                    full[6 * i + j] = covariance[3 * i + j]
            for i in range(3, 6):
                full[6 * i + i] = UNOBSERVED_VARIANCE
            twist.twist.covariance = full
            self._pub_twist.publish(twist)

        if bool(self.get_parameter("publish_range").value) and altitude > 0.0:
            rng = Range()
            rng.header.stamp = stamp
            rng.header.frame_id = self._frame_id
            rng.radiation_type = Range.ULTRASOUND
            rng.field_of_view = float(self.get_parameter("beam_field_of_view").value)
            rng.min_range = float(self.get_parameter("range_min").value)
            rng.max_range = float(self.get_parameter("range_max").value)
            rng.range = altitude
            self._pub_range.publish(rng)

    @staticmethod
    def _to_beam_msg(beam: Dict[str, Any]) -> DVLBeam:
        msg = DVLBeam()
        msg.id = beam["id"]
        msg.velocity = beam["velocity"]
        msg.distance = beam["distance"]
        msg.rssi = beam["rssi"]
        msg.nsd = beam["nsd"]
        msg.beam_valid = beam["beam_valid"]
        return msg

    def _on_position_local(self, report: Dict[str, Any]) -> None:
        if not bool(self.get_parameter("publish_dead_reckoning").value):
            return

        stamp = self.get_clock().now().to_msg()
        x = float(report.get("x", 0.0))
        y = float(report.get("y", 0.0))
        z = float(report.get("z", 0.0))
        roll = math.radians(float(report.get("roll", 0.0)))
        pitch = math.radians(float(report.get("pitch", 0.0)))
        yaw = math.radians(float(report.get("yaw", 0.0)))
        std = float(report.get("std", -1.0))

        if self._rotate:
            y, z = -y, -z
            pitch, yaw = -pitch, -yaw

        dr = DVLDeadReckoning()
        dr.header.stamp = stamp
        dr.header.frame_id = self._odom_frame_id
        dr.x, dr.y, dr.z = x, y, z
        dr.position_std = std
        dr.roll, dr.pitch, dr.yaw = roll, pitch, yaw
        dr.status = int(report.get("status", 0))
        dr.format = str(report.get("format", ""))
        self._pub_dr.publish(dr)

        variance = std * std if std > 0.0 else self._dr_var
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame_id
        odom.child_frame_id = self._frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        qx, qy, qz, qw = _quat_from_euler(roll, pitch, yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        for i in range(3):
            odom.pose.covariance[7 * i] = variance
        for i in range(3, 6):
            odom.pose.covariance[7 * i] = UNOBSERVED_VARIANCE
        for i in range(6):
            odom.twist.covariance[7 * i] = UNOBSERVED_VARIANCE
        self._pub_odom.publish(odom)

    # -- Helpers ------------------------------------------------------------------

    def _count(self, report_type: str) -> None:
        with self._counter_lock:
            self._counters[report_type] = self._counters.get(report_type, 0) + 1

    def _log_statistics(self) -> None:
        with self._counter_lock:
            counters = dict(self._counters)
            self._counters.clear()
        if not counters:
            self.get_logger().warning(
                "No DVL reports in the last 5 s", throttle_duration_sec=10.0
            )
            return
        rates = ", ".join(
            f"{name} {count / 5.0:.1f} Hz" for name, count in sorted(counters.items())
        )
        self.get_logger().info(rates)

    def destroy_node(self) -> bool:
        self._stop.set()
        self._close()
        if self._rx_thread.is_alive():
            self._rx_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaterLinkedDvlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
