"""Pier calibration: where the AQUA-SLAM map lies in the harbour frame (porto), from a start at the pier.

Procedure: the ROV is held still by hand with its stern against the pier edge, nose perpendicular
to it (a known direction in porto), together with the DVL calibration; then it is released and
driven away. The SLAM usually starts tracking only some metres later, away from the pier.

1. Still: while armed, the last ``still_s`` seconds of gyro are checked continuously. When they
   are still (norm of the per-axis std < ``still_std_max``), they give the gyro bias (mean gyro)
   and roll/pitch (mean accelerometer = gravity); yaw is the known nose direction and the
   position is the pier point plus the IMU offset from the stern.
2. Bridge: when the ROV starts moving, the body pose in porto is propagated from the end of the
   last still window with the gyro (bias removed) and the DVL velocity rebuilt from the 4 beams
   (AQUA-SLAM's beam model; the velocity vector of the report is not used, its frame is not
   reliable on this ROV).
3. Anchor: for the first ``average_s`` seconds of SLAM poses in one map (tracking OK), each pose
   gives T_porto_slam = T_porto_body(t) * inv(T_slam_body(t)); yaw (circular mean) and
   translation are averaged. The SLAM map is gravity aligned, so yaw and translation are enough.

Same algorithm as data/guidance/validazione/calibrazione_molo.py (offline), here incremental.
Frames: body = IMU (FLU); SLAM poses are of the rectified left camera, T_imu_cam from the SLAM yaml.
"""
import math
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

IDLE, WAIT_STILL, STILL, BRIDGE, DONE, FAILED = "idle", "wait_still", "still", "bridge", "done", "failed"


def expm_so3(w: np.ndarray) -> np.ndarray:
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def rot_zyx(yaw: float, pitch: float, roll: float) -> np.ndarray:
    cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def transform(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def yaw_of(R: np.ndarray) -> float:
    return math.atan2(R[1, 0], R[0, 0])


def beam_matrix(alpha, beta) -> np.ndarray:
    """AQUA-SLAM's DVL beam model (DVLGroPreIntegration.cpp): V = K v_beams, K = (E^T E)^-1 E^T."""
    a, b = np.asarray(alpha, float), np.asarray(beta, float)
    E = np.array([[-math.cos(b[0]) * math.cos(a[0]), math.sin(b[0]) * math.cos(a[0]), math.sin(a[0])],
                  [-math.cos(b[1]) * math.cos(a[1]), -math.sin(b[1]) * math.cos(a[1]), math.sin(a[1])],
                  [math.cos(b[2]) * math.cos(a[2]), -math.sin(b[2]) * math.cos(a[2]), math.sin(a[2])],
                  [math.cos(b[3]) * math.cos(a[3]), math.sin(b[3]) * math.cos(a[3]), math.sin(a[3])]])
    return np.linalg.inv(E.T @ E) @ E.T


@dataclass
class Config:
    pier_xy: Tuple[float, float]            # pier point in porto [m]
    nose_yaw: float                         # ROV nose at the pier, rad CCW from porto x (grid East)
    stern_offset: float = 0.23              # IMU distance from the stern [m]
    still_s: float = 10.0                   # length of the still window [s]
    still_std_max: float = 0.02             # gyro std norm allowed in the still window [rad/s]
    motion_std: float = 0.03                # gyro std norm over 1 s that means "moving" [rad/s]
    average_s: float = 3.0                  # seconds of SLAM poses averaged
    max_bridge_s: float = 180.0             # give up if the SLAM does not start within this time
    dvl_max_gap: float = 1.0                # do not integrate over DVL gaps longer than this [s]
    onset_margin_s: float = 1.5             # the still window used ends this long before motion is detected


@dataclass
class Result:
    yaw: float
    xyz: np.ndarray
    yaw_spread_deg: float
    xyz_spread: np.ndarray
    bridge_s: float
    map_id: int
    bias: np.ndarray
    roll: float
    pitch: float
    still_window: Tuple[float, float]


@dataclass
class _Still:
    t0: float
    t1: float
    bias: np.ndarray
    acc: np.ndarray
    std: float


class PierCalibration:
    """Feed IMU, DVL, depth and SLAM samples in time order; read ``state``, ``message`` and ``result``."""

    def __init__(self, cfg: Config, T_imu_cam: np.ndarray, R_imu_dvl: np.ndarray, r_imu_dvl: np.ndarray,
                 beam_K: np.ndarray):
        self.cfg, self.T_imu_cam, self.R_dvl, self.r_dvl, self.K = cfg, T_imu_cam, R_imu_dvl, r_imu_dvl, beam_K
        self.T_cam_imu = np.linalg.inv(T_imu_cam)
        self.reset()

    # ------------------------------------------------------------------ control
    def reset(self) -> None:
        self.state, self.message, self.result = IDLE, "not armed", None
        self._imu = deque()                         # (t, w[3], a[3]) for the still check
        self._still: Optional[_Still] = None
        self._R = self._p = None
        self._t_last_imu = self._w_last = None
        self._t_last_dvl = None
        self._v_body = None
        self._hist = deque()                        # (t, R, p) of the body in porto
        self._pairs: List[Tuple[float, np.ndarray]] = []
        self._map_id = None
        self._t_bridge0 = None
        self._depth = None                          # (t, z porto)
        self._pending = deque()                     # SLAM poses newer than the integrated IMU, processed later

    def arm(self) -> None:
        self.reset()
        self.state, self.message = WAIT_STILL, f"armed: hold the ROV still at the pier for {self.cfg.still_s:.0f} s"

    # ------------------------------------------------------------------ inputs
    def add_imu(self, t: float, w: np.ndarray, a: np.ndarray) -> None:
        if self.state in (WAIT_STILL, STILL):
            self._imu.append((t, np.asarray(w, float), np.asarray(a, float)))
            while self._imu and self._imu[0][0] < t - self.cfg.still_s - self.cfg.onset_margin_s - 2.0:
                self._imu.popleft()
            self._check_still(t)
            self._flush_pending(t)
        elif self.state == BRIDGE:
            self._propagate(t, np.asarray(w, float))
            if t - self._t_bridge0 > self.cfg.max_bridge_s:
                self.state = FAILED
                self.message = (f"no SLAM tracking within {self.cfg.max_bridge_s:.0f} s of leaving the pier: "
                                "the dead reckoning would have drifted too much; arm again at the pier")

    def add_dvl(self, t: float, beam_velocities, valid: bool) -> None:
        vb = list(beam_velocities)
        ok = valid and len(vb) == 4 and all(v != 0.0 for v in vb)
        if self.state != BRIDGE:
            return
        if not ok:
            self._v_body = None                      # hold the position over invalid reports
            return
        w = self._w_last if self._w_last is not None else np.zeros(3)
        self._v_body = self.R_dvl @ (self.K @ np.array(vb)) - np.cross(w, self.r_dvl)
        self._t_last_dvl = t

    def add_depth(self, t: float, z_porto: float) -> None:
        """ROV depth as porto z (0 = sea level, negative below)."""
        self._depth = (t, float(z_porto))

    def add_slam_pose(self, t: float, p_cam, q_cam_xyzw, tracking_ok: bool, map_id: int) -> None:
        if self.state not in (STILL, BRIDGE) or not tracking_ok:
            return
        if t > self._t_imu_latest() + 0.05:
            self._pending.append((t, p_cam, q_cam_xyzw, tracking_ok, map_id))   # IMU not there yet
            while len(self._pending) > 100:
                self._pending.popleft()
            return
        if self._map_id is None:
            self._map_id = map_id
        elif map_id != self._map_id:                 # tracking lost while averaging: restart on the new map
            self._map_id, self._pairs = map_id, []
        T_porto_body = self._body_at(t)
        if T_porto_body is None:
            return
        T_slam_body = transform(quat_to_rot(*q_cam_xyzw), np.asarray(p_cam, float)) @ self.T_cam_imu
        T = T_porto_body @ np.linalg.inv(T_slam_body)
        self._pairs.append((t, T))
        if t - self._pairs[0][0] >= self.cfg.average_s:
            self._finish()

    # ------------------------------------------------------------------ internals
    def _t_imu_latest(self) -> float:
        if self.state == BRIDGE and self._hist:
            return self._hist[-1][0]
        return self._imu[-1][0] if self._imu else -math.inf

    def _flush_pending(self, t: float) -> None:
        while self._pending and self._pending[0][0] <= t and self.state in (STILL, BRIDGE):
            self.add_slam_pose(*self._pending.popleft())

    def _check_still(self, t: float) -> None:
        win = [s for s in self._imu if s[0] >= t - self.cfg.still_s]
        if len(win) < 20 or win[-1][0] - win[0][0] < 0.95 * self.cfg.still_s:
            return
        W = np.array([s[1] for s in win])
        std = float(np.linalg.norm(W.std(0)))
        if std < self.cfg.still_std_max:
            self._still = _Still(win[0][0], win[-1][0], W.mean(0), np.array([s[2] for s in win]).mean(0), std)
            if self.state != STILL:
                self.state = STILL
                self.message = f"still at the pier (gyro std {std:.4f} rad/s): release the ROV when ready"
            return
        if self.state == STILL:
            last = np.array([s[1] for s in self._imu if s[0] >= t - 1.0])
            if len(last) >= 5 and np.linalg.norm(last.std(0)) > self.cfg.motion_std:
                self._trim_still(t)
                self._start_bridge()

    def _trim_still(self, t_detect: float) -> None:
        """The last accepted still window can already hold the first instants of motion (a slow start keeps the
        10 s std low), which biases the gyro bias and loses the first rotation. Use the window that ends
        onset_margin_s before the motion was detected, if it is still; the bridge starts from its end and
        integrates the buffered IMU through the onset."""
        t1 = t_detect - self.cfg.onset_margin_s
        win = [x for x in self._imu if t1 - self.cfg.still_s <= x[0] <= t1]
        if len(win) < 20 or win[-1][0] - win[0][0] < 0.9 * self.cfg.still_s:
            return
        W = np.array([x[1] for x in win])
        std = float(np.linalg.norm(W.std(0)))
        if std < self.cfg.still_std_max:
            self._still = _Still(win[0][0], win[-1][0], W.mean(0), np.array([x[2] for x in win]).mean(0), std)

    def _pier_pose(self):
        """Body pose in porto while still at the pier: known yaw, roll/pitch from gravity, pier point + offset."""
        s, c = self._still, self.cfg
        roll = math.atan2(s.acc[1], s.acc[2])
        pitch = math.atan2(-s.acc[0], math.hypot(s.acc[1], s.acc[2]))
        xy = np.array(c.pier_xy) + c.stern_offset * np.array([math.cos(c.nose_yaw), math.sin(c.nose_yaw)])
        z = self._depth[1] if self._depth is not None else 0.0
        return rot_zyx(c.nose_yaw, pitch, roll), np.array([xy[0], xy[1], z]), roll, pitch

    def _start_bridge(self) -> None:
        s = self._still
        self._R, self._p, self._roll, self._pitch = self._pier_pose()
        # pose known from the start of the still window; then integrate the buffered IMU after its end
        self._hist.append((s.t0, self._R.copy(), self._p.copy()))
        self._hist.append((s.t1, self._R.copy(), self._p.copy()))
        self._t_last_imu, self._w_last, self._t_bridge0 = s.t1, None, s.t1
        self.state = BRIDGE
        self.message = "moving: dead reckoning gyro + DVL until the SLAM tracks"
        for (t, w, _) in [x for x in self._imu if x[0] > s.t1]:
            self._propagate(t, w)
        self._imu.clear()

    def _propagate(self, t: float, w_raw: np.ndarray) -> None:
        w = w_raw - self._still.bias
        dt = t - self._t_last_imu
        if dt <= 0:
            return
        w_mid = w if self._w_last is None else 0.5 * (w + self._w_last)
        self._R = self._R @ expm_so3(w_mid * dt)
        if self._v_body is not None and self._t_last_dvl is not None and t - self._t_last_dvl <= self.cfg.dvl_max_gap:
            self._p = self._p + self._R @ self._v_body * dt
        if self._depth is not None and t - self._depth[0] < 1.0:
            self._p[2] = self._depth[1]
        self._t_last_imu, self._w_last = t, w
        self._hist.append((t, self._R.copy(), self._p.copy()))
        while len(self._hist) > 2 and self._hist[1][0] < t - 30.0:
            self._hist.popleft()
        self._flush_pending(t)

    def _body_at(self, t: float) -> Optional[np.ndarray]:
        if self.state == STILL:                     # the SLAM already tracks at the pier: the pose is the pier pose
            if self._still is None or t < self._still.t0:
                return None
            R, p, self._roll, self._pitch = self._pier_pose()
            return transform(R, p)
        if not self._hist or t < self._hist[0][0] or t > self._hist[-1][0] + 0.05:
            return None
        ts = [h[0] for h in self._hist]
        i = int(np.searchsorted(ts, t))
        i = min(max(i, 0), len(ts) - 1)
        return transform(self._hist[i][1], self._hist[i][2])

    def _finish(self) -> None:
        yaws = np.array([yaw_of(T[:3, :3]) for _, T in self._pairs])
        xyz = np.array([T[:3, 3] for _, T in self._pairs])
        y = math.atan2(np.sin(yaws).mean(), np.cos(yaws).mean())
        s = self._still
        bridge_s = max(0.0, self._pairs[0][0] - self._t_bridge0) if self._t_bridge0 is not None else 0.0
        self.result = Result(y, xyz.mean(0), float(np.degrees(np.std(np.unwrap(yaws)))), xyz.std(0), bridge_s, int(self._map_id), s.bias, self._roll, self._pitch,
                             (s.t0, s.t1))
        self.state = DONE
        r = self.result
        self.message = (f"porto -> SLAM map {r.map_id}: yaw {math.degrees(r.yaw):+.2f} deg, "
                        f"x {r.xyz[0]:+.2f} y {r.xyz[1]:+.2f} z {r.xyz[2]:+.2f} m (dead reckoning {r.bridge_s:.1f} s, "
                        f"spread {r.yaw_spread_deg:.2f} deg)")
