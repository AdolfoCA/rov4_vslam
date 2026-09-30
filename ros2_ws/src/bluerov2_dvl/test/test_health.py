"""Tests for the DVL health verdicts."""

import math

from bluerov2_dvl.health import BAD, GOOD, WARN, assess


def report(valid=True, altitude=1.85, distance=2.0, fom=0.003, v=(0.0, 0.0, 0.0), beams_ok=4):
    return {
        "vx": v[0], "vy": v[1], "vz": v[2], "fom": fom if valid else 10.0,
        "velocity_valid": valid, "altitude": altitude if valid else -1.0, "status": 0,
        "beams": [{"distance": distance if valid else -1.0, "beam_valid": valid and i < beams_ok}
                  for i in range(4)],
    }


def levels(verdicts):
    return {name: level for level, name, _ in verdicts}


def test_no_reports_is_bad():
    assert levels(assess([], 5.0)) == {"reports": BAD}


def test_no_lock_is_bad_and_stops_there():
    result = levels(assess([report(valid=False)] * 20, 5.0))
    assert result == {"rate": GOOD, "bottom lock": BAD}


def test_good_lock_all_good():
    alt = 2.0 * math.cos(math.radians(22.5))
    result = levels(assess([report(altitude=alt)] * 20, 5.0, still=True))
    assert set(result.values()) == {GOOD}, result


def test_bad_beam_and_altitude_mismatch_flagged():
    result = levels(assess([report(altitude=3.0, beams_ok=3)] * 20, 5.0))
    assert result["beam 3"] == BAD
    assert result["altitude"] == WARN


def test_moving_during_still_test_flagged():
    reports = [report(altitude=1.848, v=(0.1 * (i % 2), 0.0, 0.0)) for i in range(20)]
    assert levels(assess(reports, 5.0, still=True))["still test"] == WARN
