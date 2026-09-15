"""Frame conventions: converting ArduPilot (NED / FRD) data into ROS (ENU / FLU).

Why this module exists
----------------------
Autopilots and ROS disagree about which way the axes point, and getting this wrong is
the single most common cause of a robot that "almost" navigates:

* ArduPilot expresses body-frame quantities in **FRD**: x forward, y right, z down.
  Its attitude is the rotation from the **NED** world frame (x north, y east, z down)
  to that body frame.
* ROS REP-103 expresses body-frame quantities in **FLU**: x forward, y left, z up,
  relative to an **ENU** world frame (x east, y north, z up).

So two separate rotations are involved, and they compose in a specific order:

    q_enu_flu  =  q_ned->enu  *  q_ned_frd  *  q_frd->flu

* ``q_ned->enu`` is a 180 degree rotation about the axis (1, 1, 0)/sqrt(2). It swaps
  north and east and flips down to up.
* ``q_frd->flu`` is a 180 degree rotation about the body x axis. It flips right to
  left and down to up.

For plain vectors (accelerations, angular rates, magnetic field) only the body-frame
part applies, and a 180 degree rotation about x is simply ``(x, -y, -z)``.

Quaternions here are ``(w, x, y, z)`` tuples, matching the MAVLink field order of
ATTITUDE_QUATERNION. ROS messages use ``(x, y, z, w)``, so use :func:`to_ros_quat`
at the boundary.
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

Quaternion = Tuple[float, float, float, float]
Vector3 = Tuple[float, float, float]

_SQRT_HALF = math.sqrt(0.5)

#: 180 degree rotation about (1, 1, 0)/sqrt(2): takes an NED world frame to ENU.
Q_NED_TO_ENU: Quaternion = (0.0, _SQRT_HALF, _SQRT_HALF, 0.0)

#: 180 degree rotation about the body x axis: takes an FRD body frame to FLU.
#: A 180 degree rotation is its own inverse (up to sign, which does not change the
#: rotation it represents), so the same constant is used on both sides.
Q_FRD_TO_FLU: Quaternion = (0.0, 1.0, 0.0, 0.0)


def quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    """Hamilton product of two ``(w, x, y, z)`` quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_normalize(q: Quaternion) -> Quaternion:
    """Return ``q`` scaled to unit norm, or the identity if it is degenerate."""
    w, x, y, z = q
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / norm, x / norm, y / norm, z / norm)


def ned_frd_to_enu_flu(q_ned_frd: Quaternion) -> Quaternion:
    """Convert an ArduPilot attitude quaternion into the ROS ENU/FLU convention."""
    return quat_normalize(
        quat_multiply(quat_multiply(Q_NED_TO_ENU, q_ned_frd), Q_FRD_TO_FLU)
    )


def frd_to_flu(v: Sequence[float]) -> Vector3:
    """Rotate a body-frame vector from FRD to FLU (180 degrees about x)."""
    return (float(v[0]), -float(v[1]), -float(v[2]))


def frd_to_flu_covariance(cov: Sequence[float]) -> list:
    """Rotate a row-major 3x3 covariance from FRD to FLU.

    With ``R = diag(1, -1, -1)`` the similarity transform ``R C R^T`` leaves the
    diagonal untouched and flips the sign of the xy and xz cross terms.
    """
    c = [float(v) for v in cov]
    signs = (1.0, -1.0, -1.0)
    return [c[3 * i + j] * signs[i] * signs[j] for i in range(3) for j in range(3)]


def to_ros_quat(q: Quaternion) -> Tuple[float, float, float, float]:
    """Reorder a ``(w, x, y, z)`` quaternion into the ROS ``(x, y, z, w)`` order."""
    w, x, y, z = q
    return (x, y, z, w)


def euler_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Build a ``(w, x, y, z)`` quaternion from an intrinsic Z-Y-X euler triple [rad]."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )
