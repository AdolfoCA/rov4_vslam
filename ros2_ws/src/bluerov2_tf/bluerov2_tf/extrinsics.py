"""Parsing and validating sensor extrinsics, with no ROS dependencies.

An *extrinsic* is the rigid transform between two frames: where a sensor sits on the
vehicle and how it is oriented. TF (the ROS transform library) stores these as a tree -
every frame has exactly one parent, and asking for the transform between any two frames
means walking the tree between them.

That one-parent rule is the reason this module validates rather than just converts. A
frame given two parents, a typo that leaves a frame disconnected, or a quaternion that
is not unit-length all produce a TF tree that looks fine in a topic listing and then
silently returns wrong answers - or nothing at all - deep inside whatever is consuming
it. Catching those at startup with a readable message is much cheaper.

Rotation input
--------------
Two forms are accepted per transform:

* ``rpy: [roll, pitch, yaw]`` - the ROS convention, an intrinsic Z-Y-X sequence: yaw
  about z first, then pitch about the new y, then roll about the new x. ``units`` may
  be ``radians`` (default) or ``degrees``, because mounting angles measured with a
  protractor are more readable in degrees.
* ``quaternion: [x, y, z, w]`` - for values that came out of a calibration routine,
  where converting to euler angles would only lose precision and invite sign mistakes.
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Sequence, Tuple

Quaternion = Tuple[float, float, float, float]  # (x, y, z, w), the ROS ordering

#: Below this, a quaternion is too close to zero to normalise meaningfully.
_MIN_QUAT_NORM = 1e-9

#: A translation and rotation both this close to zero are almost certainly an
#: unmeasured placeholder rather than a real coincident mounting.
_PLACEHOLDER_EPS = 1e-12


class ExtrinsicsError(ValueError):
    """Raised when a transform specification cannot be turned into a valid transform."""


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert an intrinsic Z-Y-X euler triple [rad] to an ``(x, y, z, w)`` quaternion."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def normalize_quaternion(quaternion: Sequence[float]) -> Quaternion:
    """Return a unit-length ``(x, y, z, w)`` quaternion, or raise if that is impossible."""
    if len(quaternion) != 4:
        raise ExtrinsicsError(
            f"quaternion must have 4 elements (x, y, z, w), got {len(quaternion)}"
        )
    values = [float(v) for v in quaternion]
    if not all(math.isfinite(v) for v in values):
        raise ExtrinsicsError(f"quaternion contains non-finite values: {values}")

    norm = math.sqrt(sum(v * v for v in values))
    if norm < _MIN_QUAT_NORM:
        raise ExtrinsicsError(
            f"quaternion {values} has near-zero norm and cannot be normalised"
        )
    return (values[0] / norm, values[1] / norm, values[2] / norm, values[3] / norm)


def build_transform(name: str, fields: Dict[str, object]) -> Dict[str, object]:
    """Turn one raw specification into a validated transform.

    Returns a dict with ``name``, ``parent``, ``child``, ``translation`` (3 floats) and
    ``rotation`` (4 floats, x-y-z-w). Raises :class:`ExtrinsicsError` with a message
    naming the offending transform if anything is wrong.
    """
    parent = fields.get("parent")
    child = fields.get("child")
    if not isinstance(parent, str) or not parent:
        raise ExtrinsicsError(f"transform '{name}': 'parent' must be a non-empty string")
    if not isinstance(child, str) or not child:
        raise ExtrinsicsError(f"transform '{name}': 'child' must be a non-empty string")
    if parent == child:
        raise ExtrinsicsError(
            f"transform '{name}': parent and child are both '{parent}'; a frame cannot "
            f"be its own parent"
        )

    translation = fields.get("translation", [0.0, 0.0, 0.0])
    if not isinstance(translation, (list, tuple)) or len(translation) != 3:
        raise ExtrinsicsError(
            f"transform '{name}': 'translation' must be [x, y, z] in metres"
        )
    try:
        xyz = [float(v) for v in translation]
    except (TypeError, ValueError) as exc:
        raise ExtrinsicsError(
            f"transform '{name}': translation contains a non-numeric value"
        ) from exc
    if not all(math.isfinite(v) for v in xyz):
        raise ExtrinsicsError(f"transform '{name}': translation contains non-finite values")

    has_rpy = "rpy" in fields and fields["rpy"] is not None
    has_quat = "quaternion" in fields and fields["quaternion"] is not None
    if has_rpy and has_quat:
        raise ExtrinsicsError(
            f"transform '{name}': give either 'rpy' or 'quaternion', not both"
        )

    if has_quat:
        rotation = normalize_quaternion(fields["quaternion"])  # type: ignore[arg-type]
    else:
        rpy = fields.get("rpy", [0.0, 0.0, 0.0])
        if not isinstance(rpy, (list, tuple)) or len(rpy) != 3:
            raise ExtrinsicsError(
                f"transform '{name}': 'rpy' must be [roll, pitch, yaw]"
            )
        try:
            angles = [float(v) for v in rpy]
        except (TypeError, ValueError) as exc:
            raise ExtrinsicsError(
                f"transform '{name}': rpy contains a non-numeric value"
            ) from exc
        if not all(math.isfinite(v) for v in angles):
            raise ExtrinsicsError(f"transform '{name}': rpy contains non-finite values")

        units = str(fields.get("units", "radians")).strip().lower()
        if units in ("deg", "degree", "degrees"):
            angles = [math.radians(a) for a in angles]
        elif units not in ("rad", "radian", "radians"):
            raise ExtrinsicsError(
                f"transform '{name}': unknown units '{units}'; use 'radians' or 'degrees'"
            )
        rotation = rpy_to_quaternion(*angles)

    return {
        "name": name,
        "parent": parent,
        "child": child,
        "translation": xyz,
        "rotation": list(rotation),
    }


def validate_tree(transforms: Iterable[Dict[str, object]]) -> List[str]:
    """Check the set of transforms forms a single valid TF tree.

    Returns a list of human-readable warnings for things that are suspicious but not
    fatal. Raises :class:`ExtrinsicsError` for the two conditions that genuinely break
    TF: a frame with more than one parent, and a cycle.
    """
    transforms = list(transforms)
    warnings: List[str] = []

    # A frame with two parents is the classic TF failure. tf2 will accept the messages
    # and then resolve lookups inconsistently depending on which arrived last.
    parents_of: Dict[str, List[str]] = {}
    for transform in transforms:
        parents_of.setdefault(str(transform["child"]), []).append(str(transform["parent"]))
    for child, parents in parents_of.items():
        if len(parents) > 1:
            raise ExtrinsicsError(
                f"frame '{child}' is given {len(parents)} parents ({', '.join(parents)}). "
                f"In TF every frame has exactly one parent."
            )

    # Walk upward from each frame; if we revisit a frame we are in a cycle.
    for child in parents_of:
        seen = {child}
        cursor = parents_of[child][0]
        while cursor in parents_of:
            if cursor in seen:
                raise ExtrinsicsError(
                    f"the transforms form a cycle through frame '{cursor}'; a TF tree "
                    f"must be acyclic"
                )
            seen.add(cursor)
            cursor = parents_of[cursor][0]

    # More than one root means a disconnected tree: lookups across the gap will fail.
    roots = {str(t["parent"]) for t in transforms} - set(parents_of)
    if len(roots) > 1:
        warnings.append(
            f"the transforms describe {len(roots)} disconnected trees rooted at "
            f"{sorted(roots)}; lookups between them will fail"
        )

    # An unmeasured placeholder is worth saying out loud - it will produce results that
    # look plausible and are wrong.
    for transform in transforms:
        translation = transform["translation"]
        rotation = transform["rotation"]
        is_zero_translation = all(abs(float(v)) < _PLACEHOLDER_EPS for v in translation)  # type: ignore[union-attr]
        is_identity_rotation = abs(abs(float(rotation[3])) - 1.0) < _PLACEHOLDER_EPS  # type: ignore[index]
        if is_zero_translation and is_identity_rotation:
            warnings.append(
                f"transform '{transform['name']}' ({transform['parent']} -> "
                f"{transform['child']}) is exactly identity - if that is a placeholder "
                f"rather than a measurement, anything using it will be quietly wrong"
            )

    return warnings


def parse_extrinsics(
    specs: Dict[str, Dict[str, object]],
) -> Tuple[List[Dict[str, object]], List[str]]:
    """Build and validate every transform. Returns ``(transforms, warnings)``."""
    transforms = [build_transform(name, fields) for name, fields in specs.items()]
    return transforms, validate_tree(transforms)
