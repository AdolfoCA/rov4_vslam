"""Tests for the Water Linked JSON decoding."""

import json

from bluerov2_dvl.protocol import (
    extract_covariance,
    parse_beams,
    rotate_covariance_frd_to_flu,
    rotate_vector_frd_to_flu,
    split_lines,
    velocity_covariance,
)

FALLBACK_VARIANCE = 0.0004  # (0.02 m/s)^2

# A realistically shaped velocity report, trimmed to the fields the driver reads.
VELOCITY_REPORT = json.loads(
    """
    {
      "time": 105.7,
      "vx": 0.3, "vy": -0.1, "vz": 0.05,
      "fom": 0.002,
      "covariance": [[1e-05, 0.0, 0.0], [0.0, 1e-05, 0.0], [0.0, 0.0, 4e-05]],
      "altitude": 3.4,
      "transducers": [
        {"id": 0, "velocity": 0.1, "distance": 3.5, "rssi": -30.1, "nsd": -88.5,
         "beam_valid": true},
        {"id": 1, "velocity": -0.2, "distance": 3.6, "rssi": -31.0, "nsd": -88.1,
         "beam_valid": true},
        {"id": 2, "velocity": 0.0, "distance": 0.0, "rssi": -90.0, "nsd": -88.0,
         "beam_valid": false},
        {"id": 3, "velocity": 0.15, "distance": 3.4, "rssi": -29.5, "nsd": -88.9,
         "beam_valid": true}
      ],
      "velocity_valid": true,
      "status": 0,
      "format": "json_v3.1",
      "type": "velocity",
      "time_of_validity": 1638191471563017,
      "time_of_transmission": 1638191471752336
    }
    """
)


# -- Frame conversion ---------------------------------------------------------------


def test_vector_rotation_flips_y_and_z():
    assert rotate_vector_frd_to_flu((0.3, -0.1, 0.05)) == (0.3, 0.1, -0.05)


def test_downward_motion_becomes_negative_z():
    """A DVL descending reports +vz in FRD (z down); in FLU that must read negative."""
    assert rotate_vector_frd_to_flu((0.0, 0.0, 0.4))[2] < 0.0


def test_covariance_rotation_is_an_involution():
    cov = [1.0, 0.3, -0.2, 0.3, 2.0, 0.1, -0.2, 0.1, 3.0]
    assert rotate_covariance_frd_to_flu(rotate_covariance_frd_to_flu(cov)) == cov


def test_covariance_rotation_preserves_symmetry_and_trace():
    cov = [1.0, 0.3, -0.2, 0.3, 2.0, 0.1, -0.2, 0.1, 3.0]
    rotated = rotate_covariance_frd_to_flu(cov)
    assert rotated[1] == rotated[3] and rotated[2] == rotated[6] and rotated[5] == rotated[7]
    assert sum(rotated[0:1] + rotated[4:5] + rotated[8:9]) == 6.0


# -- Covariance selection -------------------------------------------------------------


def test_instrument_covariance_is_preferred():
    result = velocity_covariance(VELOCITY_REPORT, FALLBACK_VARIANCE, rotate_to_flu=False)
    assert result == [1e-05, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 4e-05]


def test_all_zero_covariance_is_rejected():
    """An all-zero covariance claims a perfect measurement, which is never true."""
    report = dict(VELOCITY_REPORT, covariance=[[0.0] * 3] * 3)
    assert extract_covariance(report) is None


def test_falls_back_to_the_figure_of_merit():
    report = dict(VELOCITY_REPORT)
    del report["covariance"]
    result = velocity_covariance(report, FALLBACK_VARIANCE, rotate_to_flu=False)
    expected_variance = 0.002 ** 2
    assert result[0] == expected_variance
    assert result[4] == expected_variance
    assert result[8] == expected_variance


def test_falls_back_to_the_parameter_when_fom_is_useless():
    report = dict(VELOCITY_REPORT, fom=-1.0)
    del report["covariance"]
    result = velocity_covariance(report, FALLBACK_VARIANCE, rotate_to_flu=False)
    assert result[0] == FALLBACK_VARIANCE


def test_non_finite_covariance_is_rejected():
    report = dict(VELOCITY_REPORT, covariance=[[float("nan")] * 3] * 3)
    assert extract_covariance(report) is None


def test_malformed_covariance_shapes_are_rejected():
    for bad in ([], [[1.0, 2.0]], [[1.0, 2.0, 3.0]] * 2, "not a matrix", None):
        assert extract_covariance(dict(VELOCITY_REPORT, covariance=bad)) is None


# -- Beams --------------------------------------------------------------------------


def test_all_four_beams_are_parsed():
    beams = parse_beams(VELOCITY_REPORT)
    assert len(beams) == 4
    assert [beam["id"] for beam in beams] == [0, 1, 2, 3]
    assert [beam["beam_valid"] for beam in beams] == [True, True, False, True]
    assert beams[0]["rssi"] == -30.1


def test_missing_beam_fields_get_defaults():
    beams = parse_beams({"transducers": [{"id": 2}]})
    assert beams[0]["velocity"] == 0.0
    # -1 rather than 0, because 0 m would be a valid (if implausible) range.
    assert beams[0]["distance"] == -1.0
    assert beams[0]["beam_valid"] is False


def test_absent_or_junk_transducers_do_not_raise():
    assert parse_beams({}) == []
    assert parse_beams({"transducers": None}) == []
    assert parse_beams({"transducers": ["nonsense", 42]}) == []


# -- TCP framing ----------------------------------------------------------------------


def test_complete_lines_are_split_and_the_tail_is_kept():
    lines, remainder = split_lines(b'{"a": 1}\n{"b": 2}\n{"c": 3')
    assert lines == [b'{"a": 1}', b'{"b": 2}']
    assert remainder == b'{"c": 3'


def test_a_partial_line_is_held_until_it_completes():
    """The realistic case: one recv returns half a report and the next finishes it."""
    lines, remainder = split_lines(b'{"type": "velo')
    assert lines == []

    lines, remainder = split_lines(remainder + b'city"}\n')
    assert lines == [b'{"type": "velocity"}']
    assert remainder == b""


def test_blank_lines_are_dropped():
    lines, remainder = split_lines(b"\n\n{}\n\n")
    assert lines == [b"{}"]
    assert remainder == b""
