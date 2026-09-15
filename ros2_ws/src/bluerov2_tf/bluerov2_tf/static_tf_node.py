#!/usr/bin/env python3
"""Publish the vehicle's sensor extrinsics on /tf_static from a single config file.

Why a node instead of several static_transform_publisher processes
------------------------------------------------------------------
``tf2_ros static_transform_publisher`` is fine for one throwaway transform, but a
vehicle's extrinsics are neither throwaway nor singular. Putting them here buys:

* **One place to edit.** The numbers live in a YAML file next to the other
  configuration, not spread across launch arguments, so re-measuring the DVL lever arm
  is a config change rather than a launch-file edit.
* **Validation at startup.** A frame accidentally given two parents, a cycle, a
  non-unit quaternion, a typo'd frame name - all caught with a message naming the
  offending transform, instead of producing a TF tree that resolves lookups wrongly.
* **Degrees where degrees make sense.** Mounting angles usually come off a drawing or a
  protractor; being able to write ``-90`` instead of ``-1.5707963`` removes a class of
  transcription error.
* **One process.** Four static publishers means four nodes in the graph and four sets
  of logs to read.

How /tf_static works
--------------------
Static transforms are published exactly once, on a latched (transient-local) topic.
Any subscriber that joins later still receives them, which is why nothing needs to be
republished periodically - and also why changing an extrinsic means restarting this
node rather than editing a live value.
"""

from __future__ import annotations

from typing import Dict, List

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from bluerov2_tf.extrinsics import ExtrinsicsError, parse_extrinsics

#: Fields read for each named transform in the parameter file.
_FIELDS = ("parent", "child", "translation", "rpy", "quaternion", "units")


class StaticTfNode(Node):
    """Read named transforms from parameters, validate them, publish them once."""

    def __init__(self) -> None:
        # Parameters are nested per transform name and therefore not known ahead of
        # time, so let rclpy declare whatever the YAML provides.
        super().__init__(
            "static_tf_node",
            automatically_declare_parameters_from_overrides=True,
        )

        if not self.has_parameter("transforms"):
            self.declare_parameter("transforms", [""])

        names = [
            name for name in (self.get_parameter("transforms").value or []) if name
        ]
        if not names:
            self.get_logger().error(
                "No transforms configured. Set the 'transforms' parameter to the list "
                "of transform names to publish, each with its own parent/child/"
                "translation/rpy block. Nothing will be published."
            )
            return

        specs = {name: self._read_spec(name) for name in names}

        try:
            transforms, warnings = parse_extrinsics(specs)
        except ExtrinsicsError as exc:
            # Refusing to publish is the right failure mode: a partial or wrong TF tree
            # is harder to debug than an absent one.
            self.get_logger().fatal(f"Invalid extrinsics, publishing nothing: {exc}")
            raise

        for warning in warnings:
            self.get_logger().warning(warning)

        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(
            [self._to_message(transform) for transform in transforms]
        )

        self.get_logger().info(
            f"Published {len(transforms)} static transforms:\n"
            + "\n".join(self._describe(transform) for transform in transforms)
        )

    def _read_spec(self, name: str) -> Dict[str, object]:
        """Collect the parameters belonging to one named transform."""
        spec: Dict[str, object] = {}
        for field in _FIELDS:
            key = f"{name}.{field}"
            if self.has_parameter(key):
                value = self.get_parameter(key).value
                if value is not None:
                    spec[field] = value
        return spec

    def _to_message(self, transform: Dict[str, object]) -> TransformStamped:
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(transform["parent"])
        message.child_frame_id = str(transform["child"])

        translation: List[float] = transform["translation"]  # type: ignore[assignment]
        message.transform.translation.x = translation[0]
        message.transform.translation.y = translation[1]
        message.transform.translation.z = translation[2]

        rotation: List[float] = transform["rotation"]  # type: ignore[assignment]
        message.transform.rotation.x = rotation[0]
        message.transform.rotation.y = rotation[1]
        message.transform.rotation.z = rotation[2]
        message.transform.rotation.w = rotation[3]
        return message

    @staticmethod
    def _describe(transform: Dict[str, object]) -> str:
        x, y, z = transform["translation"]  # type: ignore[misc]
        qx, qy, qz, qw = transform["rotation"]  # type: ignore[misc]
        return (
            f"  {transform['parent']} -> {transform['child']}  "
            f"xyz=({x:+.4f}, {y:+.4f}, {z:+.4f})  "
            f"quat=({qx:+.4f}, {qy:+.4f}, {qz:+.4f}, {qw:+.4f})"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = StaticTfNode()
    except ExtrinsicsError:
        # The message has already been logged; exit rather than spin doing nothing.
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)

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
