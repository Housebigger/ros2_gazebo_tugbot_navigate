"""Frame reconciliation + error decomposition for Task A POSEDIAG lines (offline, ROS-free).

The solver logs three raw poses per tick; this module turns them into along-track / lateral /
yaw error so the along-track-lag verdict is testable. See the localization-root-cause spec."""
from __future__ import annotations
import math
import re
from typing import Dict, Optional, Tuple

Pose2D = Tuple[float, float, float]

_STAMP = re.compile(r"\[(\d+\.\d+)\]")
_TRIP = re.compile(
    r"POSEDIAG gt=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\) "
    r"odom=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\) "
    r"solver=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)")


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def parse_posediag_line(line: str) -> Optional[Dict]:
    ms, mt = _STAMP.search(line), _TRIP.search(line)
    if not (ms and mt):
        return None
    g = [float(x) for x in mt.groups()]
    return {"t": float(ms.group(1)),
            "gt": (g[0], g[1], g[2]),
            "odom": (g[3], g[4], g[5]),
            "solver": (g[6], g[7], g[8])}


def decompose_error(gt: Pose2D, other: Pose2D) -> Dict[str, float]:
    """Error (gt - other) expressed in gt's body frame: +along = gt is ahead of `other` along
    gt's heading; +lateral = gt is to gt's left of `other`; yaw = wrapped(gt_yaw - other_yaw)."""
    dx, dy = gt[0] - other[0], gt[1] - other[1]
    c, s = math.cos(gt[2]), math.sin(gt[2])
    return {"along": c * dx + s * dy,
            "lateral": -s * dx + c * dy,
            "yaw": _wrap(gt[2] - other[2])}
