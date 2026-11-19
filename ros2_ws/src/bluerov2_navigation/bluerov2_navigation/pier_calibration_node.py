"""Pier calibration node: places the AQUA-SLAM map in the harbour frame (porto) before a mission.

Usage at the harbour (see pier_calibration.py for the method):
    ros2 launch bluerov2_navigation pier_calibration.launch.py
    ros2 service call /pier_calibration/arm std_srvs/srv/Trigger    # stern on the pier, nose out, still
    # hold still >= still_s (together with dvl_calibrate), then release and drive to the work area

Subscribes (header stamps are the time base, so it also runs on a bag replay):
    /bluerov2/imu/data_raw      sensor_msgs/Imu              gyro + accelerometer
    /bluerov2/dvl/report        bluerov2_msgs/DVLReport      beam velocities
    /bluerov2/depth/ardusub_z   geometry_msgs/PointStamped   ROV depth (z, negative below the surface), optional
    /aqua_slam/orb_pose         geometry_msgs/PoseStamped    camera pose in the SLAM map
    /aqua_slam/tracking_info    bluerov2_msgs/TrackingInfo   tracking state and map id

Publishes:
    ~/status    std_msgs/String (transient local)  state and what to do
    ~/pose      geometry_msgs/PoseStamped          IMU (body) pose in porto: dead reckoning while bridging,
                                                   SLAM pose x calibration afterwards (only in the calibrated map)
    /tf_static  porto -> aqua_slam_map             once the calibration is done

Services: ~/arm, ~/reset (std_srvs/Trigger).
"""
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster

from bluerov2_msgs.msg import DVLReport, TrackingInfo
from bluerov2_navigation.geo import latlon_to_utm
from bluerov2_navigation.pier_calibration import (BRIDGE, DONE, Config, PierCalibration, beam_matrix,
                                                  quat_to_rot, transform)

TRACKING_OK = (TrackingInfo.STATE_OK, TrackingInfo.STATE_OK_KLT)


def _t(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def rot_to_quat(R: np.ndarray):
    """(x, y, z, w) of a rotation matrix."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


def load_extrinsics(settings: str):
    """T_imu_cam, R and lever arm of the DVL in the IMU frame, beam matrix: from the AQUA-SLAM settings yaml."""
    fs = cv2.FileStorage(settings, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"cannot open the SLAM settings {settings}")
    T_imu_cam, T_dvl_cam = fs.getNode("T_gyro_c").mat(), fs.getNode("T_dvl_c").mat()
    T_imu_dvl = T_imu_cam @ np.linalg.inv(T_dvl_cam)
    node = lambda k, d: fs.getNode(k).mat().ravel() if not fs.getNode(k).empty() else np.full(4, d)  # noqa: E731
    return T_imu_cam, T_imu_dvl[:3, :3], T_imu_dvl[:3, 3], beam_matrix(node("DVL.Alpha", 1.178), node("DVL.Beta", 0.785))


class PierCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("pier_calibration")
        share = get_package_share_directory("aqua_slam")
        p = self.declare_parameter
        p("slam_settings", os.path.join(share, "config", "bluerov2_rov4_15hz_10-11.yaml"))
        p("origin_lat", 55.76121385912307)     # origin of the porto frame (start point on the pier)
        p("origin_lon", 12.600330511855978)
        p("pier_lat", 55.76121385912307)       # where the stern touches the pier
        p("pier_lon", 12.600330511855978)
        p("nose_yaw_deg", 3.0)                  # nose at the pier, deg CCW from grid East (east pier edge: true East)
        p("stern_offset", 0.23)
        p("still_s", 10.0)
        p("still_std_max", 0.02)
        p("motion_std", 0.03)
        p("average_s", 3.0)
        p("max_bridge_s", 180.0)
        p("porto_frame", "porto")
        p("slam_frame", "aqua_slam_map")
        p("auto_arm", False)                    # arm at start-up (bag replays)
        p("output_dir", "")                     # if set, the result is also written there as JSON
        g = lambda k: self.get_parameter(k).value  # noqa: E731

        T_imu_cam, R_dvl, r_dvl, K = load_extrinsics(g("slam_settings"))
        e0, n0 = latlon_to_utm(g("origin_lat"), g("origin_lon"))
        e1, n1 = latlon_to_utm(g("pier_lat"), g("pier_lon"))
        cfg = Config(pier_xy=(e1 - e0, n1 - n0), nose_yaw=math.radians(g("nose_yaw_deg")),
                     stern_offset=g("stern_offset"), still_s=g("still_s"), still_std_max=g("still_std_max"),
                     motion_std=g("motion_std"), average_s=g("average_s"), max_bridge_s=g("max_bridge_s"))
        self.cal = PierCalibration(cfg, T_imu_cam, R_dvl, r_dvl, K)
        self.porto, self.slam = g("porto_frame"), g("slam_frame")
        self.out_dir = g("output_dir")
        self.get_logger().info(f"pier at ({cfg.pier_xy[0]:+.2f}, {cfg.pier_xy[1]:+.2f}) m in {self.porto}, "
                               f"nose {g('nose_yaw_deg'):+.1f} deg; SLAM settings {g('slam_settings')}")

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_status = self.create_publisher(String, "~/status", latched)
        self.pub_pose = self.create_publisher(PoseStamped, "~/pose", 10)
        self.tf_static = StaticTransformBroadcaster(self)
        self._info = {}                         # stamp -> (state, map_id)
        self._last_status = None
        self._T_porto_slam = None

        self.create_subscription(Imu, "/bluerov2/imu/data_raw", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(DVLReport, "/bluerov2/dvl/report", self._on_dvl, qos_profile_sensor_data)
        self.create_subscription(PointStamped, "/bluerov2/depth/ardusub_z", self._on_depth, qos_profile_sensor_data)
        self.create_subscription(TrackingInfo, "/aqua_slam/tracking_info", self._on_info, 50)
        self.create_subscription(PoseStamped, "/aqua_slam/orb_pose", self._on_pose, 50)
        self.create_service(Trigger, "~/arm", self._srv_arm)
        self.create_service(Trigger, "~/reset", self._srv_reset)
        if g("auto_arm"):
            self.cal.arm()
        self._publish_status()

    # ------------------------------------------------------------------ services
    def _srv_arm(self, _req, res):
        self.cal.arm()
        self._T_porto_slam = None
        self._publish_status()
        res.success, res.message = True, self.cal.message
        return res

    def _srv_reset(self, _req, res):
        self.cal.reset()
        self._T_porto_slam = None
        self._publish_status()
        res.success, res.message = True, "reset"
        return res

    # ------------------------------------------------------------------ inputs
    def _on_imu(self, m: Imu) -> None:
        w, a = m.angular_velocity, m.linear_acceleration
        self.cal.add_imu(_t(m.header), np.array([w.x, w.y, w.z]), np.array([a.x, a.y, a.z]))
        if self.cal.state == DONE and self._T_porto_slam is None:    # finished on a queued SLAM pose
            self._on_done()
        if self.cal.state == BRIDGE and self.cal._hist:
            t, R, p = self.cal._hist[-1]
            if not hasattr(self, "_t_pose") or t - self._t_pose >= 0.1:   # 10 Hz is enough for display/control
                self._t_pose = t
                self._publish_pose(m.header.stamp, R, p)
        self._publish_status()

    def _on_dvl(self, m: DVLReport) -> None:
        self.cal.add_dvl(_t(m.header), [b.velocity for b in m.beams], bool(m.velocity_valid))

    def _on_depth(self, m: PointStamped) -> None:
        self.cal.add_depth(_t(m.header), m.point.z)

    def _on_info(self, m: TrackingInfo) -> None:
        self._info[_t(m.header)] = (m.state, m.map_id)
        while len(self._info) > 200:
            self._info.pop(next(iter(self._info)))

    def _on_pose(self, m: PoseStamped) -> None:
        t = _t(m.header)
        state, map_id = self._info.get(t, (TrackingInfo.STATE_OK, 0))
        ok = state in TRACKING_OK
        p, q = m.pose.position, m.pose.orientation
        if self.cal.state != DONE:
            self.cal.add_slam_pose(t, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), ok, map_id)
            if self.cal.state == DONE and self._T_porto_slam is None:
                self._on_done()
            self._publish_status()
            return
        if self._T_porto_slam is not None and ok and map_id == self.cal.result.map_id:
            T = self._T_porto_slam @ transform(quat_to_rot(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z])) @ self.cal.T_cam_imu
            self._publish_pose(m.header.stamp, T[:3, :3], T[:3, 3])
        elif ok and map_id != self.cal.result.map_id and self._last_status != "map_changed":
            self._last_status = "map_changed"
            msg = (f"SLAM map changed ({self.cal.result.map_id} -> {map_id}) after a tracking loss: the calibration "
                   "holds only for the old map; ~/pose stops")
            self.get_logger().warn(msg)
            self.pub_status.publish(String(data="map_changed: " + msg))

    # ------------------------------------------------------------------ outputs
    def _on_done(self) -> None:
        r = self.cal.result
        c, s = math.cos(r.yaw), math.sin(r.yaw)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        self._T_porto_slam = transform(R, r.xyz)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = self.porto, self.slam
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, r.xyz)
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(r.yaw / 2), math.cos(r.yaw / 2)
        self.tf_static.sendTransform(tf)
        self.get_logger().info(self.cal.message)
        if self.out_dir:
            os.makedirs(self.out_dir, exist_ok=True)
            path = os.path.join(self.out_dir, time.strftime("pier_calibration_%Y%m%d_%H%M%S.json"))
            json.dump({"yaw_deg": math.degrees(r.yaw), "x": float(r.xyz[0]), "y": float(r.xyz[1]), "z": float(r.xyz[2]),
                       "yaw_spread_deg": r.yaw_spread_deg, "xyz_spread_m": r.xyz_spread.tolist(), "bridge_s": r.bridge_s,
                       "map_id": r.map_id, "gyro_bias_rad_s": r.bias.tolist(), "roll_deg": math.degrees(r.roll),
                       "pitch_deg": math.degrees(r.pitch), "still_window": list(r.still_window),
                       "pier_xy": list(self.cal.cfg.pier_xy), "nose_yaw_deg": math.degrees(self.cal.cfg.nose_yaw),
                       "frames": [self.porto, self.slam]}, open(path, "w"), indent=1)
            self.get_logger().info(f"written {path}")

    def _publish_pose(self, stamp, R, p) -> None:
        msg = PoseStamped()
        msg.header.stamp, msg.header.frame_id = stamp, self.porto
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p)
        (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
         msg.pose.orientation.w) = rot_to_quat(R)
        self.pub_pose.publish(msg)

    def _publish_status(self) -> None:
        text = f"{self.cal.state}: {self.cal.message}"
        if text != self._last_status:
            self._last_status = text
            self.pub_status.publish(String(data=text))
            self.get_logger().info(text)


def main() -> None:
    rclpy.init()
    node = PierCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
