"""Pure decoding of the Water Linked JSON protocol, with no ROS dependencies.

Keeping the wire format separate from the node has two payoffs: the parsing can be
unit-tested without a ROS environment or a DVL, and the awkward parts (frame rotation,
covariance fallbacks, TCP framing) live in one place where they can be reasoned about.

Everything here works on plain Python types. The node turns the results into messages.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

#: Sign flips that take a vector from the DVL's FRD axes to ROS FLU axes.
_FLU_SIGNS = (1.0, -1.0, -1.0)


def rotate_vector_frd_to_flu(vector: Sequence[float]) -> Tuple[float, float, float]:
    """Rotate a vector 180 degrees about x: (x, y, z) -> (x, -y, -z)."""
    return (float(vector[0]), -float(vector[1]), -float(vector[2]))


def rotate_covariance_frd_to_flu(covariance: Sequence[float]) -> List[float]:
    """Rotate a row-major 3x3 covariance 180 degrees about x.

    With ``R = diag(1, -1, -1)``, ``R C R^T`` leaves the diagonal alone and flips the
    sign of the xy and xz cross terms. The yz term keeps its sign because both of its
    axes flip.
    """
    c = [float(v) for v in covariance]
    return [c[3 * i + j] * _FLU_SIGNS[i] * _FLU_SIGNS[j] for i in range(3) for j in range(3)]


def isotropic_covariance(variance: float) -> List[float]:
    return [
        variance, 0.0, 0.0,
        0.0, variance, 0.0,
        0.0, 0.0, variance,
    ]


def extract_covariance(report: Dict[str, Any]) -> Optional[List[float]]:
    """Return the instrument's 3x3 covariance as a flat list, or None if unusable.

    The A50 sends ``covariance`` as a nested 3x3 list. It is rejected here when it is
    the wrong shape, contains non-finite values, or is entirely zero - an all-zero
    covariance claims a perfect measurement, which no acoustic instrument can make.
    """
    raw = report.get("covariance")
    if not isinstance(raw, list) or len(raw) != 3:
        return None
    if not all(isinstance(row, list) and len(row) == 3 for row in raw):
        return None

    try:
        flat = [float(value) for row in raw for value in row]
    except (TypeError, ValueError):
        return None

    if not all(math.isfinite(value) for value in flat):
        return None
    if all(value == 0.0 for value in flat):
        return None
    return flat


def velocity_covariance(
    report: Dict[str, Any],
    fallback_variance: float,
    rotate_to_flu: bool = True,
) -> List[float]:
    """Best available 3x3 velocity covariance for one velocity report.

    Preference order, most to least informative:

    1. the covariance the instrument sent;
    2. an isotropic covariance built from the figure of merit, which Water Linked
       define as the standard deviation of the velocity estimate;
    3. the caller's fallback.
    """
    flat = extract_covariance(report)

    if flat is None:
        fom = report.get("fom")
        try:
            fom_value = float(fom)
        except (TypeError, ValueError):
            fom_value = float("nan")
        if math.isfinite(fom_value) and fom_value > 0.0:
            flat = isotropic_covariance(fom_value * fom_value)
        else:
            flat = isotropic_covariance(fallback_variance)

    return rotate_covariance_frd_to_flu(flat) if rotate_to_flu else flat


def parse_beams(report: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Normalise the ``transducers`` array into plain dicts with defaults filled in."""
    beams: List[Dict[str, Any]] = []
    for entry in report.get("transducers") or []:
        if not isinstance(entry, dict):
            continue
        beams.append(
            {
                "id": _as_int(entry.get("id"), 0),
                "velocity": _as_float(entry.get("velocity"), 0.0),
                # -1 rather than 0 for a missing range: 0 m would be a valid reading.
                "distance": _as_float(entry.get("distance"), -1.0),
                "rssi": _as_float(entry.get("rssi"), 0.0),
                "nsd": _as_float(entry.get("nsd"), 0.0),
                "beam_valid": bool(entry.get("beam_valid", False)),
            }
        )
    return beams


def split_lines(buffer: bytes) -> Tuple[List[bytes], bytes]:
    """Split a receive buffer into complete newline-terminated lines plus a remainder.

    TCP gives no message boundaries: one ``recv`` can return half a report, or two and
    a half. Only whole lines are handed on; the tail is kept for the next read.
    """
    if b"\n" not in buffer:
        return [], buffer
    *complete, remainder = buffer.split(b"\n")
    return [line.strip() for line in complete if line.strip()], remainder


def _as_float(value: Any, default: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _as_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default
