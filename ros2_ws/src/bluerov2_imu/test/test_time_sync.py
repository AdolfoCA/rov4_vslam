"""Tests for the MAVLink clock offset estimator."""

from bluerov2_imu.time_sync import ClockOffsetEstimator

SECOND = 1_000_000_000


def test_first_sample_defines_the_offset():
    estimator = ClockOffsetEstimator()
    stamp = estimator.update(vehicle_time_ns=5 * SECOND, ros_time_ns=1005 * SECOND)
    assert estimator.offset_ns == 1000 * SECOND
    assert stamp == 1005 * SECOND


def test_minimum_filter_rejects_transport_jitter():
    """Delayed messages must not drag the estimate along.

    A message that took 80 ms longer to arrive shows an offset 80 ms larger. The
    minimum over the window is the least-delayed sample, so the estimate should stay
    at the clean value and the delayed message should be stamped *earlier* than it
    arrived - which is the whole point.
    """
    estimator = ClockOffsetEstimator(window_seconds=30.0)
    estimator.update(1 * SECOND, 101 * SECOND)          # offset 100 s, no extra delay
    delayed = estimator.update(2 * SECOND, 102 * SECOND + 80_000_000)  # +80 ms late

    assert estimator.offset_ns == 100 * SECOND
    assert delayed == 102 * SECOND
    assert delayed < 102 * SECOND + 80_000_000


def test_window_expires_old_samples():
    """A slow clock drift should be tracked once the old samples fall out of the window."""
    estimator = ClockOffsetEstimator(window_seconds=10.0)
    estimator.update(1 * SECOND, 101 * SECOND)            # offset 100 s
    assert estimator.offset_ns == 100 * SECOND

    # 60 s later the vehicle clock has slipped 1 s behind; the old sample is long gone.
    estimator.update(60 * SECOND, 161 * SECOND)           # offset 101 s
    assert estimator.offset_ns == 101 * SECOND


def test_missing_vehicle_time_falls_back_to_arrival():
    estimator = ClockOffsetEstimator()
    assert estimator.update(0, 42 * SECOND) == 42 * SECOND
    assert not estimator.ready


def test_reset_clears_state():
    estimator = ClockOffsetEstimator()
    estimator.update(1 * SECOND, 2 * SECOND)
    assert estimator.ready
    estimator.reset()
    assert not estimator.ready
