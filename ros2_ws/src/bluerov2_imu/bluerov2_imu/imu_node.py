#!/usr/bin/env python3
"""ROS 2 driver publishing the BlueROV2 Heavy's IMU over MAVLink.

Where the data comes from
-------------------------
The BlueROV2 Heavy carries a Navigator flight controller: an ICM-20602 accelerometer +
gyroscope, an MMC5983MA magnetometer and a BMP280 barometer, all read by ArduSub
running on the ROV's Raspberry Pi. There is no way to talk to those chips directly from
topside - ArduSub owns the I2C/SPI buses - so the IMU reaches us as MAVLink messages
that BlueOS forwards over the tether.

This node therefore:

1. opens a MAVLink link to the vehicle (UDP by default, see ``connection_url``);
2. asks ArduSub for the messages it wants, at the rates it wants, using
   MAV_CMD_SET_MESSAGE_INTERVAL - without this the default stream rates are far too low
   for anything odometry-like;
3. converts every quantity from ArduPilot's NED/FRD conventions into ROS REP-103
   ENU/FLU (see :mod:`bluerov2_imu.frames`);
4. re-stamps messages onto the ROS clock using a minimum-filter offset estimator
   (see :mod:`bluerov2_imu.time_sync`) rather than stamping on arrival.

Published topics (relative to the node's namespace)
---------------------------------------------------
``imu/data``          sensor_msgs/Imu           orientation + rates + accelerations
``imu/data_raw``      sensor_msgs/Imu           rates + accelerations only, no orientation
``imu/mag``           sensor_msgs/MagneticField magnetometer, if enabled
``imu/pressure``      sensor_msgs/FluidPressure barometer (in-hull), if enabled
``imu/temperature``   sensor_msgs/Temperature   IMU die temperature, if reported

A note on covariances
---------------------
ArduSub does not publish any uncertainty for these messages, so the covariances here
come from the ``*_stddev`` parameters. The defaults are order-of-magnitude figures for
a consumer MEMS IMU; if you are going to fuse this into a SLAM back-end, replace them
with values from an Allan variance run on your own unit rather than trusting them.
"""

from __future__ import annotations

import threading
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Imu, MagneticField, Temperature

from bluerov2_imu.frames import frd_to_flu, ned_frd_to_enu_flu, to_ros_quat
from bluerov2_imu.time_sync import ClockOffsetEstimator

try:
    from pymavlink import mavutil
except ImportError as exc:  # pragma: no cover - surfaced at runtime with a clear message
    raise ImportError(
        "pymavlink is not installed. Inside the container it comes from "
        "requirements.txt; outside, run 'pip3 install pymavlink'."
    ) from exc


# --- Unit conversions -------------------------------------------------------------
# ArduPilot fills RAW_IMU / SCALED_IMU2 with integer-scaled values, documented in the
# common MAVLink dialect. These are the factors that bring them into SI.
MILLI_G_TO_M_S2 = 9.80665e-3      # xacc etc. are in mg (milli-standard-gravity)
MILLI_RAD_TO_RAD = 1.0e-3         # xgyro etc. are in mrad/s
MILLI_GAUSS_TO_TESLA = 1.0e-7     # xmag etc. are in mgauss; 1 gauss = 1e-4 T
CENTI_DEG_TO_DEG = 1.0e-2         # temperature is in cdegC
HPA_TO_PA = 1.0e2                 # SCALED_PRESSURE press_abs is in hPa

UNKNOWN_COVARIANCE = -1.0         # REP-145: covariance[0] = -1 means "not available"


class BlueRov2ImuNode(Node):
    """Bridge ArduSub's inertial MAVLink stream onto ROS 2 topics."""

    def __init__(self) -> None:
        super().__init__("imu_node")

        # --- Parameters ------------------------------------------------------------
        # TODO(deployment): confirm these against your own vehicle before a field trial.
        # 'udpin:0.0.0.0:14550' listens for the stream BlueOS pushes to topside by
        # default. If BlueOS is configured to expect a client instead, use
        # 'udpout:<rov-ip>:14550'; for a direct SITL / serial link use 'tcp:<host>:5760'
        # or '/dev/ttyACM0'.
        self.declare_parameter("connection_url", "udpin:0.0.0.0:14550")
        self.declare_parameter("source_system", 255)
        self.declare_parameter("source_component", 190)
        self.declare_parameter("target_system", 1)
        self.declare_parameter("target_component", 1)
        self.declare_parameter("frame_id", "imu_link")

        # Which raw inertial message to use. ArduSub publishes RAW_IMU for the primary
        # IMU and SCALED_IMU2 / SCALED_IMU3 for the others; on a Navigator the primary
        # is the ICM-20602 you almost certainly want.
        self.declare_parameter("raw_imu_message", "RAW_IMU")

        self.declare_parameter("attitude_rate_hz", 50.0)
        self.declare_parameter("raw_imu_rate_hz", 100.0)
        self.declare_parameter("pressure_rate_hz", 10.0)

        self.declare_parameter("publish_magnetic_field", True)
        self.declare_parameter("publish_pressure", True)
        self.declare_parameter("publish_temperature", True)

        self.declare_parameter("use_mavlink_time", True)
        self.declare_parameter("clock_window_seconds", 30.0)

        self.declare_parameter("linear_acceleration_stddev", 0.05)   # m/s^2
        self.declare_parameter("angular_velocity_stddev", 0.005)     # rad/s
        self.declare_parameter("orientation_stddev", 0.02)           # rad
        self.declare_parameter("magnetic_field_stddev", 1.0e-6)      # T

        self.declare_parameter("connection_timeout_s", 10.0)
        self.declare_parameter("reconnect_period_s", 3.0)
        self.declare_parameter("stream_request_period_s", 10.0)

        p = self.get_parameter
        self._frame_id: str = p("frame_id").value
        self._use_mavlink_time: bool = p("use_mavlink_time").value
        self._raw_imu_message: str = str(p("raw_imu_message").value).upper()

        self._accel_var = float(p("linear_acceleration_stddev").value) ** 2
        self._gyro_var = float(p("angular_velocity_stddev").value) ** 2
        self._orientation_var = float(p("orientation_stddev").value) ** 2
        self._mag_var = float(p("magnetic_field_stddev").value) ** 2

        # --- Publishers --------------------------------------------------------------
        self._pub_imu = self.create_publisher(Imu, "imu/data", qos_profile_sensor_data)
        self._pub_imu_raw = self.create_publisher(
            Imu, "imu/data_raw", qos_profile_sensor_data
        )
        self._pub_mag = self.create_publisher(
            MagneticField, "imu/mag", qos_profile_sensor_data
        )
        self._pub_pressure = self.create_publisher(
            FluidPressure, "imu/pressure", qos_profile_sensor_data
        )
        self._pub_temperature = self.create_publisher(
            Temperature, "imu/temperature", qos_profile_sensor_data
        )

        # --- State --------------------------------------------------------------------
        self._clock = ClockOffsetEstimator(float(p("clock_window_seconds").value))
        self._conn = None
        self._conn_lock = threading.Lock()
        self._stop = threading.Event()
        self._counters: Dict[str, int] = {}
        self._counter_lock = threading.Lock()

        self._latest_accel_flu = (0.0, 0.0, 0.0)
        self._have_accel = False

        self._rx_thread = threading.Thread(target=self._link_loop, daemon=True)
        self._rx_thread.start()

        self._heartbeat_timer = self.create_timer(1.0, self._send_heartbeat)
        self._stats_timer = self.create_timer(5.0, self._log_statistics)

        self.get_logger().info(
            f"IMU driver starting on {p('connection_url').value} "
            f"(raw source: {self._raw_imu_message}, frame: {self._frame_id})"
        )

    # -- Link management ----------------------------------------------------------

    def _link_loop(self) -> None:
        """Connect, pump messages, and reconnect on failure until shutdown."""
        reconnect_period = float(self.get_parameter("reconnect_period_s").value)
        while not self._stop.is_set() and rclpy.ok():
            try:
                self._connect()
                self._pump()
            except Exception as exc:  # noqa: BLE001 - a driver must survive any link fault
                self.get_logger().error(f"MAVLink link failed: {exc}")
            finally:
                with self._conn_lock:
                    if self._conn is not None:
                        try:
                            self._conn.close()
                        except Exception:  # noqa: BLE001
                            pass
                        self._conn = None
                self._clock.reset()
            if not self._stop.is_set():
                self.get_logger().info(f"Reconnecting in {reconnect_period:.1f} s")
                self._stop.wait(reconnect_period)

    def _connect(self) -> None:
        url = str(self.get_parameter("connection_url").value)
        timeout = float(self.get_parameter("connection_timeout_s").value)

        self.get_logger().info(f"Opening MAVLink connection: {url}")
        conn = mavutil.mavlink_connection(
            url,
            source_system=int(self.get_parameter("source_system").value),
            source_component=int(self.get_parameter("source_component").value),
            dialect="ardupilotmega",
            autoreconnect=True,
        )

        heartbeat = conn.wait_heartbeat(timeout=timeout)
        if heartbeat is None:
            conn.close()
            raise TimeoutError(
                f"No MAVLink heartbeat within {timeout:.0f} s. Check that the ROV is "
                f"powered, that the tether is up, and that BlueOS is forwarding "
                f"MAVLink to this host."
            )

        self.get_logger().info(
            f"Heartbeat from system {conn.target_system}, component "
            f"{conn.target_component} (autopilot type {heartbeat.autopilot})"
        )
        with self._conn_lock:
            self._conn = conn
        self._request_streams()

    def _pump(self) -> None:
        """Block on the link and dispatch messages until it dies."""
        handlers = {
            "ATTITUDE_QUATERNION": self._on_attitude_quaternion,
            "RAW_IMU": self._on_raw_imu,
            "SCALED_IMU2": self._on_raw_imu,
            "SCALED_IMU3": self._on_raw_imu,
            "SCALED_PRESSURE": self._on_scaled_pressure,
        }
        wanted = list(handlers.keys())
        last_request = self.get_clock().now()
        request_period = float(self.get_parameter("stream_request_period_s").value)

        while not self._stop.is_set() and rclpy.ok():
            conn = self._conn
            if conn is None:
                raise ConnectionError("MAVLink connection dropped")

            msg = conn.recv_match(type=wanted, blocking=True, timeout=1.0)

            # ArduSub occasionally forgets a stream request (for instance after a
            # BlueOS restart on the other end of the tether), so re-assert it.
            now = self.get_clock().now()
            if (now - last_request).nanoseconds > request_period * 1e9:
                self._request_streams()
                last_request = now

            if msg is None:
                continue
            if msg.get_type() == "BAD_DATA":
                continue

            handler = handlers.get(msg.get_type())
            if handler is not None:
                handler(msg)
                self._count(msg.get_type())

    def _request_streams(self) -> None:
        """Ask ArduSub for each message at the configured rate."""
        requests = [
            (
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
                float(self.get_parameter("attitude_rate_hz").value),
            ),
            (
                self._raw_imu_message_id(),
                float(self.get_parameter("raw_imu_rate_hz").value),
            ),
        ]
        if bool(self.get_parameter("publish_pressure").value):
            requests.append(
                (
                    mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE,
                    float(self.get_parameter("pressure_rate_hz").value),
                )
            )

        for message_id, rate_hz in requests:
            if rate_hz <= 0.0:
                continue
            self._send_message_interval(message_id, rate_hz)

    def _raw_imu_message_id(self) -> int:
        table = {
            "RAW_IMU": mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
            "SCALED_IMU2": mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2,
            "SCALED_IMU3": mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU3,
        }
        if self._raw_imu_message not in table:
            self.get_logger().warning(
                f"Unknown raw_imu_message '{self._raw_imu_message}', using RAW_IMU"
            )
            self._raw_imu_message = "RAW_IMU"
        return table[self._raw_imu_message]

    def _send_message_interval(self, message_id: int, rate_hz: float) -> None:
        interval_us = int(round(1.0e6 / rate_hz))
        with self._conn_lock:
            conn = self._conn
            if conn is None:
                return
            conn.mav.command_long_send(
                int(self.get_parameter("target_system").value),
                int(self.get_parameter("target_component").value),
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,               # confirmation
                float(message_id),
                float(interval_us),
                0.0, 0.0, 0.0, 0.0,
                0.0,             # response target: default
            )

    def _send_heartbeat(self) -> None:
        """Announce ourselves so ArduSub keeps streaming to this endpoint."""
        with self._conn_lock:
            conn = self._conn
            if conn is None:
                return
            try:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(f"Heartbeat send failed: {exc}")

    # -- Message handlers ---------------------------------------------------------

    def _on_attitude_quaternion(self, msg) -> None:
        stamp_ns = self._stamp_ns(getattr(msg, "time_boot_ms", 0) * 1_000_000)

        q_enu_flu = ned_frd_to_enu_flu((msg.q1, msg.q2, msg.q3, msg.q4))
        rates_flu = frd_to_flu((msg.rollspeed, msg.pitchspeed, msg.yawspeed))

        imu = Imu()
        imu.header.stamp = self._to_ros_stamp(stamp_ns)
        imu.header.frame_id = self._frame_id

        qx, qy, qz, qw = to_ros_quat(q_enu_flu)
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.orientation_covariance = self._diagonal(self._orientation_var)

        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = rates_flu
        imu.angular_velocity_covariance = self._diagonal(self._gyro_var)

        if self._have_accel:
            ax, ay, az = self._latest_accel_flu
            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az
            imu.linear_acceleration_covariance = self._diagonal(self._accel_var)
        else:
            # Be explicit that we have no acceleration yet rather than publishing zeros.
            imu.linear_acceleration_covariance[0] = UNKNOWN_COVARIANCE

        self._pub_imu.publish(imu)

    def _on_raw_imu(self, msg) -> None:
        if msg.get_type() != self._raw_imu_message:
            return

        # RAW_IMU carries time_usec; SCALED_IMU2/3 carry time_boot_ms.
        if hasattr(msg, "time_usec"):
            vehicle_ns = int(msg.time_usec) * 1_000
        else:
            vehicle_ns = int(getattr(msg, "time_boot_ms", 0)) * 1_000_000
        stamp_ns = self._stamp_ns(vehicle_ns)
        stamp = self._to_ros_stamp(stamp_ns)

        accel_flu = frd_to_flu(
            (
                msg.xacc * MILLI_G_TO_M_S2,
                msg.yacc * MILLI_G_TO_M_S2,
                msg.zacc * MILLI_G_TO_M_S2,
            )
        )
        gyro_flu = frd_to_flu(
            (
                msg.xgyro * MILLI_RAD_TO_RAD,
                msg.ygyro * MILLI_RAD_TO_RAD,
                msg.zgyro * MILLI_RAD_TO_RAD,
            )
        )
        self._latest_accel_flu = accel_flu
        self._have_accel = True

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self._frame_id
        # REP-145: an orientation covariance of -1 marks "this message has no orientation".
        imu.orientation_covariance[0] = UNKNOWN_COVARIANCE
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gyro_flu
        imu.angular_velocity_covariance = self._diagonal(self._gyro_var)
        (
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        ) = accel_flu
        imu.linear_acceleration_covariance = self._diagonal(self._accel_var)
        self._pub_imu_raw.publish(imu)

        if bool(self.get_parameter("publish_magnetic_field").value):
            mag_flu = frd_to_flu(
                (
                    msg.xmag * MILLI_GAUSS_TO_TESLA,
                    msg.ymag * MILLI_GAUSS_TO_TESLA,
                    msg.zmag * MILLI_GAUSS_TO_TESLA,
                )
            )
            mag = MagneticField()
            mag.header.stamp = stamp
            mag.header.frame_id = self._frame_id
            mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z = mag_flu
            mag.magnetic_field_covariance = self._diagonal(self._mag_var)
            self._pub_mag.publish(mag)

        # ArduPilot sends temperature = 0 when the sensor does not report one.
        temperature_cdeg = int(getattr(msg, "temperature", 0))
        if temperature_cdeg != 0 and bool(
            self.get_parameter("publish_temperature").value
        ):
            temp = Temperature()
            temp.header.stamp = stamp
            temp.header.frame_id = self._frame_id
            temp.temperature = temperature_cdeg * CENTI_DEG_TO_DEG
            temp.variance = 0.0  # unknown
            self._pub_temperature.publish(temp)

    def _on_scaled_pressure(self, msg) -> None:
        if not bool(self.get_parameter("publish_pressure").value):
            return
        stamp_ns = self._stamp_ns(int(getattr(msg, "time_boot_ms", 0)) * 1_000_000)

        # NOTE: this is the Navigator's own barometer, sealed inside the electronics
        # enclosure. It measures internal hull pressure, NOT depth. Depth on a BlueROV2
        # comes from the external Bar30 (SCALED_PRESSURE2 / VFR_HUD.alt), which is a
        # separate concern from this IMU driver.
        pressure = FluidPressure()
        pressure.header.stamp = self._to_ros_stamp(stamp_ns)
        pressure.header.frame_id = self._frame_id
        pressure.fluid_pressure = float(msg.press_abs) * HPA_TO_PA
        pressure.variance = 0.0
        self._pub_pressure.publish(pressure)

    # -- Helpers ------------------------------------------------------------------

    def _stamp_ns(self, vehicle_time_ns: int) -> int:
        now_ns = self.get_clock().now().nanoseconds
        if not self._use_mavlink_time:
            return now_ns
        return self._clock.update(vehicle_time_ns, now_ns)

    def _to_ros_stamp(self, nanoseconds: int):
        return rclpy.time.Time(nanoseconds=nanoseconds).to_msg()

    @staticmethod
    def _diagonal(variance: float) -> list:
        return [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance,
        ]

    def _count(self, message_type: str) -> None:
        with self._counter_lock:
            self._counters[message_type] = self._counters.get(message_type, 0) + 1

    def _log_statistics(self) -> None:
        with self._counter_lock:
            counters = dict(self._counters)
            self._counters.clear()
        if not counters:
            self.get_logger().warning(
                "No inertial MAVLink messages in the last 5 s - link down or streams "
                "not granted by the autopilot",
                throttle_duration_sec=10.0,
            )
            return
        rates = ", ".join(f"{name} {count / 5.0:.0f} Hz" for name, count in sorted(counters.items()))
        offset = self._clock.offset_ns
        offset_text = f", clock offset {offset / 1e9:+.3f} s" if offset is not None else ""
        self.get_logger().info(f"{rates}{offset_text}")

    def destroy_node(self) -> bool:
        self._stop.set()
        with self._conn_lock:
            if self._conn is not None:
                try:
                    self._conn.close()
                except Exception:  # noqa: BLE001
                    pass
                self._conn = None
        if self._rx_thread.is_alive():
            self._rx_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlueRov2ImuNode()
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
