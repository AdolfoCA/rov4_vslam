"""Tests for the NED/FRD -> ENU/FLU conversions.

These are the cheapest bugs in the whole stack to write and the most expensive to find
in the water, so they are pinned down with cases whose answers can be reasoned out by
hand rather than by running the code.
"""

import math

from bluerov2_imu.frames import (
    euler_to_quat,
    frd_to_flu,
    frd_to_flu_covariance,
    ned_frd_to_enu_flu,
    quat_multiply,
    quat_normalize,
    to_ros_quat,
)

SQRT_HALF = math.sqrt(0.5)


def assert_same_rotation(actual, expected, tol=1e-9):
    """Compare quaternions up to sign, since q and -q are the same rotation."""
    same = all(abs(a - b) < tol for a, b in zip(actual, expected))
    opposite = all(abs(a + b) < tol for a, b in zip(actual, expected))
    assert same or opposite, f"{actual} is not the rotation {expected}"


def test_identity_attitude_becomes_ninety_degree_yaw():
    """A vehicle level and facing north.

    In NED/FRD that is the identity: body axes coincide with north/east/down.
    In ENU/FLU, facing north means the body x axis points along +y, so the answer must
    be a +90 degree rotation about z.
    """
    result = ned_frd_to_enu_flu((1.0, 0.0, 0.0, 0.0))
    assert_same_rotation(result, (SQRT_HALF, 0.0, 0.0, SQRT_HALF))


def test_north_east_yaw_becomes_identity():
    """A vehicle level and facing east.

    Facing east is a +90 degree yaw about the NED down axis. East is +x in ENU, so in
    ENU/FLU the vehicle is aligned with the world frame: the identity.
    """
    q_ned_frd = (SQRT_HALF, 0.0, 0.0, SQRT_HALF)
    result = ned_frd_to_enu_flu(q_ned_frd)
    assert_same_rotation(result, (1.0, 0.0, 0.0, 0.0))


def test_roll_keeps_its_sign():
    """Roll is about the forward axis, which both conventions agree points forward.

    A +30 degree roll (starboard side down) in FRD is still a +30 degree roll about the
    body x axis in FLU, and with the vehicle facing north the ENU/FLU result is that
    roll composed with the +90 degree yaw of the first test.
    """
    roll = math.radians(30.0)
    q_ned_frd = euler_to_quat(roll, 0.0, 0.0)
    result = ned_frd_to_enu_flu(q_ned_frd)

    expected = quat_multiply(
        (SQRT_HALF, 0.0, 0.0, SQRT_HALF),   # yaw +90 about z
        euler_to_quat(roll, 0.0, 0.0),      # then roll +30 about the body x axis
    )
    assert_same_rotation(result, expected)


def test_pitch_flips_sign():
    """Pitch is about the y axis, and FRD's y (right) is the opposite of FLU's y (left).

    So a +10 degree nose-up pitch in ArduPilot's convention must come out as -10 degrees
    about the FLU y axis.
    """
    pitch = math.radians(10.0)
    result = ned_frd_to_enu_flu(euler_to_quat(0.0, pitch, 0.0))
    expected = quat_multiply(
        (SQRT_HALF, 0.0, 0.0, SQRT_HALF),
        euler_to_quat(0.0, -pitch, 0.0),
    )
    assert_same_rotation(result, expected)


def test_output_is_normalised():
    result = ned_frd_to_enu_flu((0.9, 0.1, 0.2, 0.3))
    norm = math.sqrt(sum(component * component for component in result))
    assert abs(norm - 1.0) < 1e-12


def test_degenerate_quaternion_falls_back_to_identity():
    assert quat_normalize((0.0, 0.0, 0.0, 0.0)) == (1.0, 0.0, 0.0, 0.0)


def test_vector_conversion_flips_y_and_z():
    assert frd_to_flu((1.0, 2.0, 3.0)) == (1.0, -2.0, -3.0)


def test_gravity_points_the_right_way():
    """An IMU at rest measures specific force, i.e. +1 g along its own up axis.

    At rest in FRD that reads as -9.81 on z (z points down), and in FLU it must become
    +9.81 on z (z points up).
    """
    accel_flu = frd_to_flu((0.0, 0.0, -9.80665))
    assert abs(accel_flu[2] - 9.80665) < 1e-9


def test_covariance_rotation_preserves_diagonal_and_flips_cross_terms():
    cov = [
        1.0, 2.0, 3.0,
        2.0, 4.0, 5.0,
        3.0, 5.0, 6.0,
    ]
    rotated = frd_to_flu_covariance(cov)
    # Diagonal unchanged.
    assert [rotated[0], rotated[4], rotated[8]] == [1.0, 4.0, 6.0]
    # xy and xz flip sign, yz does not (both y and z flip, so the signs cancel).
    assert rotated[1] == -2.0 and rotated[3] == -2.0
    assert rotated[2] == -3.0 and rotated[6] == -3.0
    assert rotated[5] == 5.0 and rotated[7] == 5.0


def test_covariance_rotation_is_an_involution():
    """Applying the same 180 degree rotation twice must return the original."""
    cov = [1.0, 0.3, -0.2, 0.3, 2.0, 0.1, -0.2, 0.1, 3.0]
    assert frd_to_flu_covariance(frd_to_flu_covariance(cov)) == cov


def test_ros_quat_reorders_to_xyzw():
    assert to_ros_quat((1.0, 2.0, 3.0, 4.0)) == (2.0, 3.0, 4.0, 1.0)
