"""Judge a window of DVL reports: is the data good? No ROS dependencies.

How a DVL-A50 measures, in short
--------------------------------
It has 4 acoustic beams, each tilted 22.5 degrees from the DVL's z axis. Each beam
pings the bottom and gets two things back from the echo:

* the **range** along the beam (from the echo's travel time): ``beams[i].distance``;
* the **velocity** along the beam (from the Doppler shift of the echo).

From these the DVL computes:

* **velocity**: 4 along-beam velocities, 3 unknowns (vx, vy, vz). The spare beam is a
  consistency check; how well the 4 agree becomes the figure of merit ``fom``.
* **altitude**: the beam ranges projected onto the z axis, about
  ``distance * cos(22.5 deg)``.

``velocity_valid`` and ``beam_valid`` are the DVL's own verdicts. Everything else here
is a cross-check on top of them.
"""

from __future__ import annotations

import math
import statistics
from typing import Any, Dict, List, Sequence, Tuple

BEAM_TILT = math.radians(22.5)

GOOD, WARN, BAD = "GOOD", "CHECK", "BAD"

Verdict = Tuple[str, str, str]   # (level, check name, explanation)


def _beams(report: Dict[str, Any]) -> List[Dict[str, Any]]:
    return report.get("beams") or []


def assess(reports: Sequence[Dict[str, Any]], window_s: float,
           still: bool = False) -> List[Verdict]:
    """Verdicts for the reports received in the last ``window_s`` seconds.

    Each report is a dict with the fields of bluerov2_msgs/DVLReport (velocity as
    ``vx, vy, vz``, beams as dicts). ``still`` adds a noise check that only makes sense
    when the vehicle is not moving.
    """
    out: List[Verdict] = []
    n = len(reports)
    rate = n / window_s if window_s > 0 else 0.0
    if n == 0:
        return [(BAD, "reports", "no reports at all: driver not running or DVL unreachable")]
    out.append((GOOD if rate >= 2.0 else WARN, "rate",
                f"{rate:.1f} Hz (normal 2-15 Hz; it drops as altitude grows, since the DVL "
                "waits for each echo before pinging again)"))

    valid = [r for r in reports if r.get("velocity_valid")]
    share = len(valid) / n
    if share >= 0.9:
        level = GOOD
    elif share > 0.0:
        level = WARN
    else:
        level = BAD
    out.append((level, "bottom lock",
                f"{share:.0%} of reports valid" + (
                    "" if valid else " - NO LOCK: out of water, bottom out of range, "
                    "or air/bubbles under the transducers")))

    if not valid:
        return out

    # Beams: the DVL's own per-beam verdict.
    all_beams_ok = True
    for i in range(4):
        ok = sum(1 for r in valid if len(_beams(r)) > i and _beams(r)[i].get("beam_valid"))
        frac = ok / len(valid)
        if frac < 0.9:
            all_beams_ok = False
            out.append((WARN if frac > 0.5 else BAD, f"beam {i}",
                        f"valid in {frac:.0%} of locked reports - something blocks or "
                        "confuses this beam (hull, cable, wall, weeds)"))
    if all_beams_ok:
        out.append((GOOD, "beams", "all 4 beams valid"))

    # Figure of merit: the DVL's own standard deviation of the velocity.
    fom = statistics.fmean(r["fom"] for r in valid)
    level = GOOD if fom < 0.01 else WARN if fom < 0.05 else BAD
    out.append((level, "fom",
                f"{fom:.4f} m/s mean (the DVL's velocity std; <0.01 good, <0.05 usable)"))

    # Altitude against the beam ranges it is computed from.
    last = valid[-1]
    ranges = [b["distance"] for b in _beams(last) if b.get("beam_valid") and b["distance"] > 0]
    if ranges and last.get("altitude", -1) > 0:
        expected = statistics.fmean(ranges) * math.cos(BEAM_TILT)
        err = abs(last["altitude"] - expected) / expected
        out.append((GOOD if err < 0.1 else WARN, "altitude",
                    f"{last['altitude']:.2f} m; from beam ranges {expected:.2f} m "
                    "(mean range x cos 22.5)"))
        spread = (max(ranges) - min(ranges)) / statistics.fmean(ranges)
        out.append((GOOD if spread < 0.25 else WARN, "beam ranges",
                    " / ".join(f"{d:.2f}" for d in ranges) + f" m, spread {spread:.0%} "
                    "(large on a flat floor = vehicle tilted, or a beam hits a wall/object)"))
    else:
        out.append((WARN, "altitude", "no altitude in the last locked report"))

    speeds = [math.sqrt(r["vx"] ** 2 + r["vy"] ** 2 + r["vz"] ** 2) for r in valid]
    if max(speeds) > 1.0:
        out.append((WARN, "speed", f"max {max(speeds):.2f} m/s - AQUA-SLAM discards >1 m/s"))
    if still and len(valid) >= 3:
        noise = max(statistics.pstdev(r[k] for r in valid) for k in ("vx", "vy", "vz"))
        mean = [statistics.fmean(r[k] for r in valid) for k in ("vx", "vy", "vz")]
        level = GOOD if noise < 0.01 and max(map(abs, mean)) < 0.02 else WARN
        out.append((level, "still test",
                    f"noise {noise:.4f} m/s, mean ({mean[0]:+.3f}, {mean[1]:+.3f}, "
                    f"{mean[2]:+.3f}) m/s - should both be ~0 when not moving"))

    if any(r.get("status", 0) == 1 for r in reports):
        out.append((WARN, "temperature", "DVL reports high temperature (status 1)"))
    return out
