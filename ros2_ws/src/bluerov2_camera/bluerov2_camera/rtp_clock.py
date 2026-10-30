"""Map RTP-based buffer times (PTS) onto the ROS clock.

Kept free of ROS and GStreamer so it can be checked offline against recorded streams.

Two steps, applied to every frame:

``FrameGrid.correct(pts)``
    The cameras run off a steady frame clock, so consecutive PTS differ by a whole number
    of frame periods. With the aux pipeline on the camera computer (v4l2src, kernel
    capture timestamps) a few PTS come out a fraction of a millisecond to ~11 ms off the
    grid (measured 2026-10-30: 3 frames in 56 s), most likely when v4l2src is preempted
    while it converts the kernel timestamp to its pipeline clock. A PTS more than
    ``tolerance_ns`` away from the predicted one is replaced by the prediction, unless
    ``confirm`` frames in a row disagree the same way (then the grid really moved and is
    followed). PTS that are already regular (the Blue Atlas pipeline) pass unchanged.

``EnvelopeClock.to_ros(pts, arrival_ns)``
    ROS time = PTS + offset, where the offset tracks the lower envelope of
    arrival - PTS: the frames that reached the node fastest carry only the constant part
    of the latency. The offset drops at once to a faster frame and otherwise creeps
    upwards at ``creep_us_per_s``, to follow the drift between the camera computer's
    clock and the topside clock. The creep has to be faster than that drift, or the
    stamps fall further and further behind (at 20 us/s: ~44 ms over 14 minutes, on
    2026-10-27); the drift was 36-94 us/s on 2026-10-30.
    The limit of this method (2026-10-30, 5-minute test with the packets captured):
    the latency from capture to arrival varies slowly by 0-40 ms on the camera
    computer's side (encoder and sending queue), so for seconds at a time no frame comes
    in near the true minimum. Meanwhile the offset creeps up and the stamps run a few
    ms late, up to ~10 ms in the worst 1 %. Fitting the drift instead of a fixed creep
    did worse: with minima that noisy the slope takes minutes to settle. Stamps exact
    to the capture would need the camera computer's clock synchronised to the topside.
"""

from __future__ import annotations

from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np

NS = 1_000_000_000


class FrameGrid:
    def __init__(self, tolerance_ns: int = 500_000, confirm: int = 3) -> None:
        self.tolerance_ns = tolerance_ns
        self.confirm = confirm
        self.reset()

    def reset(self) -> None:
        self._last: Optional[int] = None      # last corrected PTS
        self._prev_raw: Optional[int] = None  # last PTS as it came in
        self._steps: Deque[int] = deque(maxlen=301)
        self._disagree = 0

    def _period(self) -> Optional[float]:
        # Median of every recent step, not only of the steps already accepted: right
        # after the pipeline starts the first ~15 frames come 25.7 ms apart (2026-10-30),
        # and a period learnt from those alone would never be corrected again.
        if len(self._steps) < 10:
            return None
        return float(np.median(self._steps))

    def correct(self, pts: int) -> int:
        if self._prev_raw is not None and pts > self._prev_raw:
            self._steps.append(pts - self._prev_raw)
        self._prev_raw = pts
        last, period = self._last, self._period()
        if last is None or period is None:
            self._last = pts
            return pts
        n = max(1, round((pts - last) / period))
        predicted = int(round(last + n * period))
        if abs(pts - predicted) <= self.tolerance_ns:
            self._disagree = 0
            out = pts
        else:
            self._disagree += 1
            if self._disagree >= self.confirm:  # the grid itself moved: follow it
                self._disagree = 0
                out = pts
            else:
                out = predicted
        self._last = out
        return out


class EnvelopeClock:
    def __init__(self, creep_us_per_s: float = 100.0) -> None:
        self.creep = creep_us_per_s * 1e-6
        self.reset()

    def reset(self) -> None:
        self._offset: Optional[int] = None
        self._last: Optional[Tuple[int, int]] = None   # (pts, arrival) of the last frame

    def to_ros(self, pts: int, arrival_ns: int) -> int:
        last = self._last
        if last is not None and not (0 < pts - last[0] < 2 * NS):
            self._offset = None  # PTS went backwards or jumped: a new timeline
        raw = arrival_ns - pts
        if self._offset is None:
            self._offset = raw
        else:
            creep = int((arrival_ns - last[1]) * self.creep)
            self._offset = min(self._offset + creep, raw)
        self._last = (pts, arrival_ns)
        return pts + self._offset
