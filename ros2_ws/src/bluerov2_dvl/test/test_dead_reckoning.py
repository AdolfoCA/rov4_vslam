"""Tests for the dead-reckoning math."""

import math

import pytest

from bluerov2_dvl.dead_reckoning import (
    IDENTITY,
    TrackAligner,
    base_velocity,
    looks_like_reset,
    quat_from_yaw,
    quat_multiply,
    quat_rotate,
    yaw_of,
)


def approx(values, expected):
    return all(a == pytest.approx(b, abs=1e-9) for a, b in zip(values, expected))


def test_yaw_90_rotates_x_to_y():
    assert approx(quat_rotate(quat_from_yaw(math.pi / 2), (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))


def test_yaw_of_round_trip_and_composition():
    q = quat_multiply(quat_from_yaw(0.3), quat_from_yaw(0.4))
    assert yaw_of(q) == pytest.approx(0.7)
    assert yaw_of(IDENTITY) == 0.0


def test_no_lever_arm_effect_without_rotation():
    v = base_velocity((0.5, 0.0, 0.0), IDENTITY, (0.2, 0.0, -0.1), (0.0, 0.0, 0.0))
    assert approx(v, (0.5, 0.0, 0.0))


def test_lever_arm_removes_rotation_induced_velocity():
    # Turning in place at 1 rad/s about z with the DVL 0.2 m ahead: the DVL sees 0.2 m/s
    # to the left, but the base_link origin is not moving.
    v = base_velocity((0.0, 0.2, 0.0), IDENTITY, (0.2, 0.0, 0.0), (0.0, 0.0, 1.0))
    assert approx(v, (0.0, 0.0, 0.0))


def test_dvl_mounted_with_yaw_offset():
    # DVL turned 90 deg left: its x is the vehicle's y.
    v = base_velocity((1.0, 0.0, 0.0), quat_from_yaw(math.pi / 2), (0, 0, 0), (0, 0, 0))
    assert approx(v, (0.0, 1.0, 0.0))


def test_aligner_rotates_and_shifts_dvl_track():
    aligner = TrackAligner()
    # At the anchor the DVL reports (1, 0) heading 0; in odom it is at (5, 5) heading 90 deg.
    aligner.anchor((5.0, 5.0, 0.0), math.pi / 2, (1.0, 0.0, 0.0), 0.0)
    assert approx(aligner.to_odom((1.0, 0.0, 0.0)), (5.0, 5.0, 0.0))
    # Moving 2 m along the DVL's x means moving 2 m along odom y.
    assert approx(aligner.to_odom((3.0, 0.0, -1.0)), (5.0, 7.0, -1.0))


def test_looks_like_reset():
    assert looks_like_reset((3.0, 1.0, 0.0), (0.0, 0.0, 0.0))
    assert not looks_like_reset((0.2, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert not looks_like_reset((3.0, 1.0, 0.0), (3.1, 1.0, 0.0))
