"""Pier calibration on a synthetic run: still at the pier, then a turn and a straight leg, then the
SLAM starts in a map with a known transform from porto. The transform must come back."""
import math

import numpy as np
import pytest

from bluerov2_navigation.geo import latlon_to_utm
from bluerov2_navigation.pier_calibration import (BRIDGE, DONE, FAILED, STILL, WAIT_STILL, Config,
                                                  PierCalibration, beam_matrix, expm_so3, rot_zyx, transform)

ALPHA, BETA = np.full(4, 1.178), np.full(4, 0.785)


def _E():
    K = beam_matrix(ALPHA, BETA)
    return np.linalg.pinv(K)          # beam velocities from a DVL velocity: v_beams = E v


def _rot_to_quat(R):
    w = math.sqrt(max(0.0, 1 + np.trace(R))) / 2
    return (math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2]),
            math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0]),
            math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1]), w)


def run(slam_start=40.0, still=12.0, slam_at_pier=False, noise=0.002, seed=0):
    rng = np.random.default_rng(seed)
    cfg = Config(pier_xy=(0.0, 0.0), nose_yaw=math.radians(3.0), still_s=10.0)
    T_imu_cam = transform(rot_zyx(-math.pi / 2, 0, -math.pi / 2), np.array([0.15, 0.0, -0.05]))
    R_dvl, r_dvl = rot_zyx(math.pi, 0, math.pi), np.array([-0.09, 0.03, -0.32])
    cal = PierCalibration(cfg, T_imu_cam, R_dvl, r_dvl, beam_matrix(ALPHA, BETA))
    E = _E()
    bias = np.array([0.004, -0.003, 0.006])
    T_true = transform(rot_zyx(math.radians(25.0), 0, 0), np.array([2.5, -1.5, -1.2]))   # porto <- slam
    cal.arm()
    assert cal.state == WAIT_STILL

    R = rot_zyx(cfg.nose_yaw, 0.0, 0.0)
    p = np.array([cfg.stern_offset * math.cos(cfg.nose_yaw), cfg.stern_offset * math.sin(cfg.nose_yaw), 0.0])
    dt, t, t_end = 0.01, 0.0, slam_start + 6.0
    states = set()
    while t < t_end:
        moving = t > still
        tm = t - still
        w = np.array([0.0, 0.0, 0.25 if 0 < tm < 4 else 0.0]) if moving else np.zeros(3)
        v = np.array([0.3 if moving else 0.0, 0.05 if moving else 0.0, 0.0])          # body velocity
        if moving and tm < 3:                                                         # a few wobbles to start
            w = w + np.array([0.05 * math.sin(7 * tm), 0.04 * math.cos(5 * tm), 0.0])
        R = R @ expm_so3(w * dt)
        p = p + R @ v * dt
        acc = R.T @ np.array([0, 0, 9.81])
        cal.add_imu(t, w + bias + rng.normal(0, noise, 3), acc + rng.normal(0, 0.01, 3))
        if round(t / dt) % 20 == 0:                                                   # DVL 5 Hz
            v_dvl = R_dvl.T @ (v + np.cross(w, r_dvl))
            cal.add_dvl(t, E @ v_dvl + rng.normal(0, 0.002, 4), True)
        slam_on = t >= slam_start or (slam_at_pier and t >= 2.0)
        if slam_on and round(t / dt) % 7 == 0:                                        # SLAM ~15 Hz
            T_slam_cam = np.linalg.inv(T_true) @ transform(R, p) @ T_imu_cam
            cal.add_slam_pose(t, T_slam_cam[:3, 3], _rot_to_quat(T_slam_cam[:3, :3]), True, 0)
        states.add(cal.state)
        t += dt
    return cal, T_true, states


def test_recovers_transform_after_bridge():
    cal, T_true, states = run()
    assert {STILL, BRIDGE, DONE} <= states, states
    r = cal.result
    assert abs(math.degrees(r.yaw) - 25.0) < 0.5
    assert np.linalg.norm(r.xyz[:2] - T_true[:2, 3]) < 0.10
    assert 25 < r.bridge_s < 30


def test_slam_already_tracking_at_the_pier():
    cal, T_true, _ = run(slam_at_pier=True)
    assert cal.state == DONE
    assert abs(math.degrees(cal.result.yaw) - 25.0) < 0.2
    assert np.linalg.norm(cal.result.xyz[:2] - T_true[:2, 3]) < 0.02


def test_not_still_never_bridges():
    cal, _, states = run(noise=0.05)              # hand shaking: never still
    assert STILL not in states and cal.state == WAIT_STILL


def test_fails_if_slam_never_starts():
    cal, _, states = run(slam_start=200.0)       # max_bridge_s 180: the SLAM comes too late
    assert BRIDGE in states and cal.state == FAILED and cal.result is None


def test_utm_known_point():
    e, n = latlon_to_utm(55.76121385912307, 12.600330511855978)
    assert e == pytest.approx(725872.719, abs=0.01) and n == pytest.approx(6185373.820, abs=0.01)   # data/guidance/utm.py
