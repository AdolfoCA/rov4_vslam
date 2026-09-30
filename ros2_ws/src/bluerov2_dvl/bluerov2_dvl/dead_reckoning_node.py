#!/usr/bin/env python3
"""IMU + DVL dead reckoning, for display and as a sanity check of both sensors.

Subscribes (relative to the node namespace):

    imu/data              sensor_msgs/Imu, orientation in ENU (bluerov2_imu)
    dvl/velocity          TwistWithCovarianceStamped, only with bottom lock (bluerov2_dvl)
    dvl/dead_reckoning    the DVL's own dead reckoning (bluerov2_dvl)

Publishes:

    dead_reckoning/odometry   nav_msgs/Odometry, odom -> base_link
    dead_reckoning/path       nav_msgs/Path, the IMU + DVL track
    dead_reckoning/dvl_path   nav_msgs/Path, the DVL's own track, placed in odom
    /tf                       odom -> base_link (parameter publish_tf)

Service ``dead_reckoning/reset`` (std_srvs/Trigger) puts the vehicle back at the odom
origin and clears both paths.

The two tracks differ in heading source: the first uses the vehicle IMU (ArduSub,
magnetometer-referenced), the second the DVL's internal gyro. Where they diverge, one
of the headings is off. Both drift; neither is a navigation solution.

The mounting of the IMU and the DVL comes from TF (``bluerov2_tf`` static_tf_node).
When the DVL loses bottom lock, ``dvl/velocity`` stops and the position is held: a gap
longer than ``max_dt`` is not integrated.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from bluerov2_dvl.dead_reckoning import (
    IDENTITY,
    TrackAligner,
    base_velocity,
    looks_like_reset,
    quat_conjugate,
    quat_from_yaw,
    quat_multiply,
    quat_rotate,
    yaw_of,
)
from bluerov2_msgs.msg import DVLDeadReckoning


class DeadReckoningNode(Node):

    def __init__(self) -> None:
        super().__init__("dead_reckoning_node")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("dvl_frame", "dvl_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("max_dt", 1.0)             # s, longer gaps are not integrated
        self.declare_parameter("path_min_spacing", 0.05)  # m between stored path poses
        self.declare_parameter("path_max_poses", 20000)
        self.declare_parameter("path_publish_rate", 2.0)  # Hz

        p = self.get_parameter
        self._odom_frame = str(p("odom_frame").value)
        self._base_frame = str(p("base_frame").value)
        self._dvl_frame = str(p("dvl_frame").value)
        self._publish_tf = bool(p("publish_tf").value)
        self._max_dt = float(p("max_dt").value)
        self._spacing = float(p("path_min_spacing").value)
        self._max_poses = int(p("path_max_poses").value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        self._mounts = {}   # child frame -> (rotation, translation) in base_frame

        self._q_world_base: Optional[tuple] = None
        self._omega_base = (0.0, 0.0, 0.0)
        self._position = [0.0, 0.0, 0.0]
        self._last_dvl_stamp: Optional[float] = None
        self._gaps = 0

        self._aligner = TrackAligner()
        self._last_dvl_dr: Optional[tuple] = None

        self._path = self._new_path()
        self._dvl_path = self._new_path()
        self._paths_dirty = False

        self._pub_odom = self.create_publisher(Odometry, "dead_reckoning/odometry", 10)
        # Latched, so Foxglove gets the whole track when it connects mid-dive.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_path = self.create_publisher(Path, "dead_reckoning/path", latched)
        self._pub_dvl_path = self.create_publisher(Path, "dead_reckoning/dvl_path", latched)

        self.create_subscription(Imu, "imu/data", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(TwistWithCovarianceStamped, "dvl/velocity",
                                 self._on_velocity, qos_profile_sensor_data)
        self.create_subscription(DVLDeadReckoning, "dvl/dead_reckoning",
                                 self._on_dvl_dead_reckoning, qos_profile_sensor_data)
        self.create_service(Trigger, "dead_reckoning/reset", self._on_reset)
        self.create_timer(1.0 / float(p("path_publish_rate").value), self._publish_paths)

        self.get_logger().info(
            f"IMU + DVL dead reckoning in {self._odom_frame} -> {self._base_frame}")

    # -- Helpers ------------------------------------------------------------------

    def _new_path(self) -> Path:
        path = Path()
        path.header.frame_id = self._odom_frame
        return path

    def _mount(self, frame: str):
        """Rotation and translation of ``frame`` in base_frame, from static TF."""
        if frame in self._mounts:
            return self._mounts[frame]
        if frame == self._base_frame:
            self._mounts[frame] = (IDENTITY, (0.0, 0.0, 0.0))
            return self._mounts[frame]
        try:
            tf = self._tf_buffer.lookup_transform(self._base_frame, frame, Time())
        except TransformException:
            self.get_logger().warning(
                f"No TF {self._base_frame} -> {frame} yet; is static_tf_node running?",
                throttle_duration_sec=10.0)
            return None
        r, t = tf.transform.rotation, tf.transform.translation
        self._mounts[frame] = ((r.x, r.y, r.z, r.w), (t.x, t.y, t.z))
        self.get_logger().info(f"Mounting of {frame} taken from TF")
        return self._mounts[frame]

    @staticmethod
    def _seconds(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def _append(self, path: Path, stamp, position, orientation=IDENTITY) -> None:
        if path.poses:
            last = path.poses[-1].pose.position
            if math.dist((last.x, last.y, last.z), position) < self._spacing:
                return
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._odom_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = orientation
        path.poses.append(pose)
        if len(path.poses) > self._max_poses:
            del path.poses[: len(path.poses) - self._max_poses]
        self._paths_dirty = True

    # -- Callbacks ----------------------------------------------------------------

    def _on_imu(self, msg: Imu) -> None:
        mount = self._mount(msg.header.frame_id)
        if mount is None:
            return
        q_base_imu, _ = mount
        o = msg.orientation
        q_world_imu = (o.x, o.y, o.z, o.w)
        self._q_world_base = quat_multiply(q_world_imu, quat_conjugate(q_base_imu))
        w = msg.angular_velocity
        self._omega_base = quat_rotate(q_base_imu, (w.x, w.y, w.z))

    def _on_velocity(self, msg: TwistWithCovarianceStamped) -> None:
        if self._q_world_base is None:
            self.get_logger().warning("DVL velocity but no IMU orientation yet",
                                      throttle_duration_sec=10.0)
            return
        mount = self._mount(msg.header.frame_id)
        if mount is None:
            return
        q_base_dvl, r_base_dvl = mount

        now = self._seconds(msg.header.stamp)
        dt = None if self._last_dvl_stamp is None else now - self._last_dvl_stamp
        self._last_dvl_stamp = now

        lin = msg.twist.twist.linear
        v_base = base_velocity((lin.x, lin.y, lin.z), q_base_dvl, r_base_dvl,
                               self._omega_base)

        if dt is not None and 0.0 < dt <= self._max_dt:
            v_world = quat_rotate(self._q_world_base, v_base)
            for i in range(3):
                self._position[i] += v_world[i] * dt
        elif dt is not None:
            self._gaps += 1
            self.get_logger().info(
                f"DVL gap of {dt:.1f} s (no bottom lock?): position held",
                throttle_duration_sec=5.0)

        self._publish_odometry(msg.header.stamp, v_base)
        self._append(self._path, msg.header.stamp, tuple(self._position),
                     self._q_world_base)

    def _on_dvl_dead_reckoning(self, msg: DVLDeadReckoning) -> None:
        p_dvl = (msg.x, msg.y, msg.z)
        if self._last_dvl_dr is not None and looks_like_reset(self._last_dvl_dr, p_dvl):
            self.get_logger().info("DVL dead reckoning was reset: re-anchoring its track")
            self._aligner.reset()
        self._last_dvl_dr = p_dvl

        if not self._aligner.anchored:
            # The report is in the DVL's own odom frame; its heading is dvl_link's.
            mount = self._mount(self._dvl_frame)
            if self._q_world_base is None or mount is None:
                return
            q_world_dvl = quat_multiply(self._q_world_base, mount[0])
            p_dvl_in_odom = [self._position[i] + quat_rotate(self._q_world_base, mount[1])[i]
                             for i in range(3)]
            self._aligner.anchor(p_dvl_in_odom, yaw_of(q_world_dvl), p_dvl, msg.yaw)
        self._append(self._dvl_path, msg.header.stamp, self._aligner.to_odom(p_dvl),
                     quat_from_yaw(msg.yaw))

    def _on_reset(self, _request, response):
        self._position = [0.0, 0.0, 0.0]
        self._last_dvl_stamp = None
        self._aligner.reset()
        self._path = self._new_path()
        self._dvl_path = self._new_path()
        self._paths_dirty = True
        response.success = True
        response.message = "dead reckoning reset to the odom origin"
        self.get_logger().info(response.message)
        return response

    # -- Output -------------------------------------------------------------------

    def _publish_odometry(self, stamp, v_base) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        pos = odom.pose.pose.position
        pos.x, pos.y, pos.z = self._position
        q = odom.pose.pose.orientation
        q.x, q.y, q.z, q.w = self._q_world_base
        lin = odom.twist.twist.linear
        lin.x, lin.y, lin.z = v_base
        ang = odom.twist.twist.angular
        ang.x, ang.y, ang.z = self._omega_base
        self._pub_odom.publish(odom)

        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x, tf.transform.translation.y, \
                tf.transform.translation.z = self._position
            tf.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf)

    def _publish_paths(self) -> None:
        if not self._paths_dirty:
            return
        self._paths_dirty = False
        stamp = self.get_clock().now().to_msg()
        for path, pub in ((self._path, self._pub_path), (self._dvl_path, self._pub_dvl_path)):
            path.header.stamp = stamp
            pub.publish(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DeadReckoningNode()
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
