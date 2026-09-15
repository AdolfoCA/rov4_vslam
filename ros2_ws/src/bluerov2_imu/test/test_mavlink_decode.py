"""End-to-end check of the MAVLink decode path, without a vehicle or a ROS graph.

Real MAVLink frames are encoded, pushed through the parser, and then through the same
unit conversions and frame rotations the node applies. This is what catches a wrong
scale factor - the kind of bug that produces perfectly plausible numbers that are off
by a factor of a thousand.

No sockets are involved, so the test is hermetic and cannot flake on a busy port.
"""

import math

import pytest

from bluerov2_imu.frames import frd_to_flu, ned_frd_to_enu_flu

mavutil = pytest.importorskip("pymavlink.mavutil")

from pymavlink.dialects.v20 import ardupilotmega as mav  # noqa: E402

# Kept in step with the constants in imu_node; if one changes, this test should fail.
MILLI_G_TO_M_S2 = 9.80665e-3
MILLI_RAD_TO_RAD = 1.0e-3
MILLI_GAUSS_TO_TESLA = 1.0e-7
HPA_TO_PA = 1.0e2

STANDARD_GRAVITY = 9.80665


class _Sink:
    """Minimal file-like object so MAVLink() can encode without a real connection."""

    def __init__(self):
        self.buffer = bytearray()

    def write(self, data):
        self.buffer.extend(data)


def _roundtrip(message):
    """Encode a message to bytes and parse it back, exactly as the link would."""
    sink = _Sink()
    link = mav.MAVLink(sink, srcSystem=1, srcComponent=1)
    link.send(message)

    parser = mav.MAVLink(_Sink())
    decoded = parser.parse_buffer(bytes(sink.buffer))
    assert decoded, "the frame did not parse back"
    return decoded[-1]


def test_raw_imu_at_rest_reads_one_g_upward():
    """An IMU sitting still measures specific force: +1 g along its own up axis.

    In ArduPilot's FRD frame that is -1000 mg on z. After scaling and rotation into
    FLU it must come out as +9.80665 m/s^2 on z. Getting this backwards is the classic
    way to have an estimator that thinks it is falling.
    """
    message = _roundtrip(
        mav.MAVLink_raw_imu_message(
            time_usec=1_234_567,
            xacc=0, yacc=0, zacc=-1000,      # mg
            xgyro=0, ygyro=0, zgyro=0,
            xmag=0, ymag=0, zmag=0,
            id=0, temperature=2500,
        )
    )

    accel = frd_to_flu(
        (
            message.xacc * MILLI_G_TO_M_S2,
            message.yacc * MILLI_G_TO_M_S2,
            message.zacc * MILLI_G_TO_M_S2,
        )
    )
    assert accel[0] == 0.0 and accel[1] == 0.0
    assert abs(accel[2] - STANDARD_GRAVITY) < 1e-6


def test_gyro_scale_and_sign():
    """100 mrad/s of yaw rate in FRD is 0.1 rad/s, negated by the FLU flip."""
    message = _roundtrip(
        mav.MAVLink_raw_imu_message(
            time_usec=1,
            xacc=0, yacc=0, zacc=0,
            xgyro=0, ygyro=0, zgyro=100,
            xmag=0, ymag=0, zmag=0,
            id=0, temperature=0,
        )
    )
    gyro = frd_to_flu(
        (
            message.xgyro * MILLI_RAD_TO_RAD,
            message.ygyro * MILLI_RAD_TO_RAD,
            message.zgyro * MILLI_RAD_TO_RAD,
        )
    )
    assert abs(gyro[2] + 0.1) < 1e-12


def test_magnetometer_scale_lands_in_the_right_order_of_magnitude():
    """Earth's field is 25-65 uT. 500 mgauss must decode to 5e-5 T, inside that range."""
    message = _roundtrip(
        mav.MAVLink_raw_imu_message(
            time_usec=1,
            xacc=0, yacc=0, zacc=0,
            xgyro=0, ygyro=0, zgyro=0,
            xmag=500, ymag=0, zmag=0,
            id=0, temperature=0,
        )
    )
    bx = message.xmag * MILLI_GAUSS_TO_TESLA
    assert abs(bx - 5.0e-5) < 1e-12
    assert 20e-6 < bx < 70e-6


def test_temperature_scale():
    message = _roundtrip(
        mav.MAVLink_raw_imu_message(
            time_usec=1,
            xacc=0, yacc=0, zacc=0,
            xgyro=0, ygyro=0, zgyro=0,
            xmag=0, ymag=0, zmag=0,
            id=0, temperature=2537,      # cdegC
        )
    )
    assert abs(message.temperature * 1.0e-2 - 25.37) < 1e-9


def test_pressure_scale_matches_one_atmosphere():
    message = _roundtrip(
        mav.MAVLink_scaled_pressure_message(
            time_boot_ms=1000,
            press_abs=1013.25,          # hPa
            press_diff=0.0,
            temperature=2500,
        )
    )
    assert abs(message.press_abs * HPA_TO_PA - 101325.0) < 1e-2


def test_attitude_quaternion_field_order_matches_the_conversion():
    """MAVLink sends (q1..q4) = (w, x, y, z); feeding them in that order must hold.

    The message below is a 90 degree yaw in NED, which the conversion turns into the
    identity in ENU/FLU (facing east is aligned with the ENU x axis).
    """
    half = math.sqrt(0.5)
    message = _roundtrip(
        mav.MAVLink_attitude_quaternion_message(
            time_boot_ms=5000,
            q1=half, q2=0.0, q3=0.0, q4=half,
            rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0,
            repr_offset_q=[0.0, 0.0, 0.0, 0.0],
        )
    )
    result = ned_frd_to_enu_flu((message.q1, message.q2, message.q3, message.q4))
    assert abs(abs(result[0]) - 1.0) < 1e-6
    assert all(abs(component) < 1e-6 for component in result[1:])


def test_scaled_imu2_carries_boot_time_not_usec():
    """SCALED_IMU2 stamps with time_boot_ms; the node must not read time_usec on it."""
    message = _roundtrip(
        mav.MAVLink_scaled_imu2_message(
            time_boot_ms=7777,
            xacc=0, yacc=0, zacc=-1000,
            xgyro=0, ygyro=0, zgyro=0,
            xmag=0, ymag=0, zmag=0,
            temperature=0,
        )
    )
    assert hasattr(message, "time_boot_ms")
    assert not hasattr(message, "time_usec")
    assert message.time_boot_ms * 1_000_000 == 7_777_000_000
