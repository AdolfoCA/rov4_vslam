"""Estimating the offset between the vehicle's clock and the ROS clock.

MAVLink messages carry ``time_boot_ms`` or ``time_usec``, counted from the moment the
autopilot booted. The topside computer's ROS clock knows nothing about that epoch, and
the two clocks also drift relative to each other. Two naive options both hurt:

* stamping with ``now()`` on arrival folds the whole tether + serial + scheduling
  latency into the timestamp, and that latency is *jittery*, which is much worse for a
  filter than a constant bias;
* using the raw vehicle timestamp puts every message decades away from ROS time.

The estimator below takes the middle road. For each message it computes

    offset = t_ros_arrival - t_vehicle

Every sample of that offset is inflated by exactly the transport delay of that one
message, so the *smallest* offset seen in a recent window is the sample that suffered
the least delay - the closest thing to the true clock offset that passive observation
can give. Keeping a running minimum over a sliding window therefore removes the jitter
while tracking slow drift. This is the same idea as the minimum-filter used in NTP's
clock filter algorithm.
"""

from __future__ import annotations

from collections import deque
from typing import Deque, Optional, Tuple


class ClockOffsetEstimator:
    """Sliding-window minimum filter over ``t_ros - t_vehicle``."""

    def __init__(self, window_seconds: float = 30.0, max_samples: int = 4096) -> None:
        self._window_ns = int(window_seconds * 1e9)
        self._max_samples = max_samples
        self._samples: Deque[Tuple[int, int]] = deque()  # (t_ros_ns, offset_ns)
        self._offset_ns: Optional[int] = None

    @property
    def offset_ns(self) -> Optional[int]:
        """Current best estimate of ``t_ros - t_vehicle`` in nanoseconds."""
        return self._offset_ns

    @property
    def ready(self) -> bool:
        return self._offset_ns is not None

    def update(self, vehicle_time_ns: int, ros_time_ns: int) -> int:
        """Feed one observation and return the ROS-time stamp for this message."""
        if vehicle_time_ns <= 0:
            # The autopilot did not fill the field in; fall back to arrival time.
            return ros_time_ns

        self._samples.append((ros_time_ns, ros_time_ns - vehicle_time_ns))
        self._prune(ros_time_ns)
        self._offset_ns = min(offset for _, offset in self._samples)
        return vehicle_time_ns + self._offset_ns

    def _prune(self, now_ns: int) -> None:
        horizon = now_ns - self._window_ns
        while self._samples and self._samples[0][0] < horizon:
            self._samples.popleft()
        while len(self._samples) > self._max_samples:
            self._samples.popleft()

    def reset(self) -> None:
        self._samples.clear()
        self._offset_ns = None
