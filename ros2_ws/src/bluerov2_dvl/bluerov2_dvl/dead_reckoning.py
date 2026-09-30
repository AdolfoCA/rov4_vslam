"""Dead-reckoning math with no ROS dependencies: IMU attitude + DVL velocity -> position.

Quaternions are ``(x, y, z, w)`` tuples, the ROS message order. All frames are REP-103:
the world (``odom``) frame is ENU, bodies are FLU.

The estimate is the textbook one:

    v_base  = R_base_dvl v_dvl - omega_base x r_base_dvl    (lever-arm correction)
    p      += R_world_base v_base dt

It has no correction of any kind, so it drifts: a heading error of 1 degree is a
cross-track error of 1.7 % of the distance travelled. It is a check on the sensors and
a baseline for a real estimator, not a navigation solution.
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

Quaternion = Tuple[float, float, float, float]
Vector3 = Tuple[float, float, float]

IDENTITY: Quaternion = (0.0, 0.0, 0.0, 1.0)


def quat_multiply(a: Sequence[float], b: Sequence[float]) -> Quaternion:
    """Hamilton product ``a * b`` of two ``(x, y, z, w)`` quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conjugate(q: Sequence[float]) -> Quaternion:
    return (-q[0], -q[1], -q[2], q[3])


def quat_rotate(q: Sequence[float], v: Sequence[float]) -> Vector3:
    """Rotate vector ``v`` by the unit quaternion ``q``."""
    x, y, z, _ = quat_multiply(quat_multiply(q, (v[0], v[1], v[2], 0.0)), quat_conjugate(q))
    return (x, y, z)


def quat_from_yaw(yaw: float) -> Quaternion:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_of(q: Sequence[float]) -> float:
    """Heading of ``q`` about the world z axis [rad], counter-clockwise from x."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def cross(a: Sequence[float], b: Sequence[float]) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def base_velocity(
    v_dvl: Sequence[float],
    q_base_dvl: Sequence[float],
    r_base_dvl: Sequence[float],
    omega_base: Sequence[float],
) -> Vector3:
    """Velocity of the base_link origin, in base_link, from a DVL velocity.

    The DVL sits at ``r_base_dvl`` from the base_link origin, so while the vehicle turns
    it sees an extra ``omega x r`` that the base_link origin does not have.
    """
    v = quat_rotate(q_base_dvl, v_dvl)
    lever = cross(omega_base, r_base_dvl)
    return (v[0] - lever[0], v[1] - lever[1], v[2] - lever[2])


class TrackAligner:
    """Place the DVL's own dead-reckoned track in the ``odom`` frame.

    The DVL integrates in its own local frame: origin where it was last reset, heading
    from its own gyro. To overlay it on the IMU + DVL track, one anchor is taken - the
    DVL's pose and the odom pose of the same instant - and every later DVL position is
    rotated about z by the heading difference and shifted by the position difference.
    """

    def __init__(self) -> None:
        self.anchored = False
        self._p_odom: Vector3 = (0.0, 0.0, 0.0)
        self._p_dvl: Vector3 = (0.0, 0.0, 0.0)
        self._cos = 1.0
        self._sin = 0.0

    def anchor(
        self,
        p_odom: Sequence[float],
        yaw_odom: float,
        p_dvl: Sequence[float],
        yaw_dvl: float,
    ) -> None:
        """``yaw_odom`` is the heading of the DVL frame in odom, ``yaw_dvl`` as reported."""
        offset = yaw_odom - yaw_dvl
        self._cos, self._sin = math.cos(offset), math.sin(offset)
        self._p_odom = tuple(float(v) for v in p_odom)
        self._p_dvl = tuple(float(v) for v in p_dvl)
        self.anchored = True

    def reset(self) -> None:
        self.anchored = False

    def to_odom(self, p_dvl: Sequence[float]) -> Vector3:
        dx = p_dvl[0] - self._p_dvl[0]
        dy = p_dvl[1] - self._p_dvl[1]
        dz = p_dvl[2] - self._p_dvl[2]
        return (
            self._p_odom[0] + self._cos * dx - self._sin * dy,
            self._p_odom[1] + self._sin * dx + self._cos * dy,
            self._p_odom[2] + dz,
        )


def looks_like_reset(previous: Sequence[float], current: Sequence[float],
                     far: float = 0.5, near: float = 0.05) -> bool:
    """True when the DVL position jumped from away-from-origin back to the origin."""
    return math.dist(previous, (0.0, 0.0, 0.0)) > far and \
        math.dist(current, (0.0, 0.0, 0.0)) < near
