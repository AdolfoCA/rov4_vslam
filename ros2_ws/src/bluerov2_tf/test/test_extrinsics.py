"""Tests for extrinsics parsing and TF tree validation."""

import math

import pytest

from bluerov2_tf.extrinsics import (
    ExtrinsicsError,
    build_transform,
    normalize_quaternion,
    parse_extrinsics,
    rpy_to_quaternion,
    validate_tree,
)

SQRT_HALF = math.sqrt(0.5)


def spec(parent, child, **kwargs):
    base = {"parent": parent, "child": child}
    base.update(kwargs)
    return base


# -- Rotation conversion ---------------------------------------------------------------


def test_zero_rpy_is_identity():
    assert rpy_to_quaternion(0.0, 0.0, 0.0) == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_ninety_degree_yaw():
    """A quarter turn about z: only qz and qw are non-zero, both sqrt(1/2)."""
    q = rpy_to_quaternion(0.0, 0.0, math.pi / 2)
    assert q == pytest.approx((0.0, 0.0, SQRT_HALF, SQRT_HALF))


def test_camera_optical_rotation_is_the_rep103_one():
    """rpy (-90, 0, -90) must map robot axes (x fwd, y left, z up) to optical axes
    (x right, y down, z forward). Check it by rotating the basis vectors."""
    qx, qy, qz, qw = rpy_to_quaternion(-math.pi / 2, 0.0, -math.pi / 2)

    def rotate(v):
        # q * v * q^-1, written out for a unit quaternion.
        x, y, z = v
        tx = 2 * (qy * z - qz * y)
        ty = 2 * (qz * x - qx * z)
        tz = 2 * (qx * y - qy * x)
        return (
            x + qw * tx + (qy * tz - qz * ty),
            y + qw * ty + (qz * tx - qx * tz),
            z + qw * tz + (qx * ty - qy * tx),
        )

    # The optical frame's z axis (forward) must land on the robot frame's x axis.
    assert rotate((0.0, 0.0, 1.0)) == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)
    # The optical frame's x axis (right) must land on the robot frame's -y.
    assert rotate((1.0, 0.0, 0.0)) == pytest.approx((0.0, -1.0, 0.0), abs=1e-9)
    # The optical frame's y axis (down) must land on the robot frame's -z.
    assert rotate((0.0, 1.0, 0.0)) == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)


def test_degrees_and_radians_agree():
    from_degrees = build_transform(
        "t", spec("a", "b", rpy=[0.0, 0.0, 90.0], units="degrees")
    )
    from_radians = build_transform(
        "t", spec("a", "b", rpy=[0.0, 0.0, math.pi / 2], units="radians")
    )
    assert from_degrees["rotation"] == pytest.approx(from_radians["rotation"])


def test_radians_are_the_default():
    implicit = build_transform("t", spec("a", "b", rpy=[0.0, 0.0, 1.0]))
    explicit = build_transform("t", spec("a", "b", rpy=[0.0, 0.0, 1.0], units="radians"))
    assert implicit["rotation"] == explicit["rotation"]


def test_unknown_units_are_rejected():
    with pytest.raises(ExtrinsicsError, match="unknown units"):
        build_transform("t", spec("a", "b", rpy=[0.0, 0.0, 90.0], units="gradians"))


# -- Quaternion input -------------------------------------------------------------------


def test_quaternion_input_is_normalised():
    result = build_transform("t", spec("a", "b", quaternion=[0.0, 0.0, 2.0, 2.0]))
    assert result["rotation"] == pytest.approx([0.0, 0.0, SQRT_HALF, SQRT_HALF])


def test_zero_quaternion_is_rejected():
    with pytest.raises(ExtrinsicsError, match="near-zero norm"):
        normalize_quaternion([0.0, 0.0, 0.0, 0.0])


def test_wrong_length_quaternion_is_rejected():
    with pytest.raises(ExtrinsicsError, match="4 elements"):
        normalize_quaternion([0.0, 0.0, 1.0])


def test_giving_both_rpy_and_quaternion_is_rejected():
    with pytest.raises(ExtrinsicsError, match="not both"):
        build_transform(
            "t", spec("a", "b", rpy=[0.0, 0.0, 0.0], quaternion=[0.0, 0.0, 0.0, 1.0])
        )


# -- Field validation ---------------------------------------------------------------------


def test_missing_parent_is_rejected():
    with pytest.raises(ExtrinsicsError, match="'parent'"):
        build_transform("t", {"child": "b"})


def test_self_parenting_is_rejected():
    with pytest.raises(ExtrinsicsError, match="cannot"):
        build_transform("t", spec("a", "a"))


def test_bad_translation_length_is_rejected():
    with pytest.raises(ExtrinsicsError, match="translation"):
        build_transform("t", spec("a", "b", translation=[1.0, 2.0]))


def test_non_finite_translation_is_rejected():
    with pytest.raises(ExtrinsicsError, match="non-finite"):
        build_transform("t", spec("a", "b", translation=[1.0, float("inf"), 0.0]))


def test_error_message_names_the_offending_transform():
    with pytest.raises(ExtrinsicsError, match="dvl"):
        build_transform("dvl", spec("base_link", "dvl_link", translation=[1.0]))


# -- Tree validation ------------------------------------------------------------------------


def test_two_parents_for_one_frame_is_fatal():
    """The classic TF failure: tf2 accepts both and then resolves lookups by whichever
    message arrived last."""
    transforms = [
        build_transform("a", spec("base_link", "dvl_link", translation=[0, 0, -0.1])),
        build_transform("b", spec("imu_link", "dvl_link", translation=[0, 0, -0.2])),
    ]
    with pytest.raises(ExtrinsicsError, match="2 parents"):
        validate_tree(transforms)


def test_cycle_is_fatal():
    transforms = [
        build_transform("a", spec("x", "y", translation=[1, 0, 0])),
        build_transform("b", spec("y", "z", translation=[1, 0, 0])),
        build_transform("c", spec("z", "x", translation=[1, 0, 0])),
    ]
    with pytest.raises(ExtrinsicsError, match="cycle"):
        validate_tree(transforms)


def test_disconnected_trees_warn_but_do_not_raise():
    transforms = [
        build_transform("a", spec("base_link", "imu_link", translation=[0.1, 0, 0])),
        build_transform("b", spec("other_root", "thing", translation=[0.1, 0, 0])),
    ]
    warnings = validate_tree(transforms)
    assert any("disconnected" in w for w in warnings)


def test_identity_placeholder_is_flagged():
    transforms = [build_transform("imu", spec("base_link", "imu_link"))]
    warnings = validate_tree(transforms)
    assert any("identity" in w and "imu" in w for w in warnings)


def test_a_measured_transform_is_not_flagged():
    transforms = [
        build_transform("dvl", spec("base_link", "dvl_link", translation=[0.0, 0.0, -0.1]))
    ]
    assert validate_tree(transforms) == []


# -- The shipped configuration ------------------------------------------------------------


def test_the_default_vehicle_tree_is_valid():
    """The frame layout the stack ships with must parse and form one connected tree."""
    specs = {
        "imu": spec("base_link", "imu_link", translation=[0.0, 0.0, 0.0], units="degrees"),
        "dvl": spec("base_link", "dvl_link", translation=[0.0, 0.0, -0.10], units="degrees"),
        "camera": spec(
            "base_link", "camera_link", translation=[0.20, 0.0, 0.0], units="degrees"
        ),
        "camera_optical": spec(
            "camera_link",
            "camera_link_optical",
            translation=[0.0, 0.0, 0.0],
            rpy=[-90.0, 0.0, -90.0],
            units="degrees",
        ),
    }
    transforms, warnings = parse_extrinsics(specs)

    assert len(transforms) == 4
    # One root, so the tree is connected.
    assert not any("disconnected" in w for w in warnings)
    # The unmeasured placeholders should be called out - that is the point of the check.
    assert any("imu" in w for w in warnings)
    # The optical transform is a real rotation and must not be flagged.
    assert not any("camera_optical" in w for w in warnings)
