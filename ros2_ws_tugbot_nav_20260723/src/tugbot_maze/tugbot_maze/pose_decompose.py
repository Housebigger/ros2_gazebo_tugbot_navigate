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
    try:
        g = [float(x) for x in mt.groups()]
    except ValueError:
        return None
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


# --- frame reconciliation ---------------------------------------------------
# POSEDIAG carries gt in the maze WORLD frame but odom/solver in the entrance-anchored MAP frame.
# The two frames differ by a fixed rigid transform (a pure translation ~+entrance, yaw ~0). Because
# /odom is world-anchored ground truth (up to that fixed transform), the offset (odom - gt) recovers
# world->map. We derive it from the data (never hardcode) and add it to gt to move gt into the map
# frame, so gt-odom becomes a pure "does odom drift?" signal and gt-solver the true belief error.

def frame_offset(row: Dict) -> Pose2D:
    """world->map offset (odom - gt) implied by one POSEDIAG row: (dx, dy, wrapped dyaw)."""
    gx, gy, gth = row["gt"]
    ox, oy, oth = row["odom"]
    return (ox - gx, oy - gy, _wrap(oth - gth))


def offset_consistency(rows) -> Dict:
    """Compare the world->map offset from the FIRST vs LAST row. If odom is truly world-anchored,
    the two agree; disagreement means /odom drifts (a finding, not a bug to hide). Returns the two
    offsets, the position delta (m) and the wrapped yaw delta (rad)."""
    off_first = frame_offset(rows[0])
    off_last = frame_offset(rows[-1])
    dpos = math.hypot(off_first[0] - off_last[0], off_first[1] - off_last[1])
    dyaw = abs(_wrap(off_first[2] - off_last[2]))
    return {"off_first": off_first, "off_last": off_last, "dpos": dpos, "dyaw": dyaw}


def apply_offset(pose: Pose2D, off: Pose2D) -> Pose2D:
    """gt_map = gt + off (translation added in world axes; yaw wrapped). Exact when yaw offset ~0."""
    return (pose[0] + off[0], pose[1] + off[1], _wrap(pose[2] + off[2]))


def align_gt_to_map(rows):
    """Reconcile every row's gt from the WORLD frame into the entrance-anchored MAP frame by adding
    the fixed world->map offset derived from the FIRST row. Returns new row dicts with gt replaced
    by gt_map; odom and solver (already map-frame) are copied through untouched. Empty in -> empty
    out. After this, decompose_error(gt_map, odom/solver) is same-frame and meaningful."""
    if not rows:
        return []
    off = frame_offset(rows[0])
    out = []
    for r in rows:
        nr = dict(r)
        nr["gt"] = apply_offset(r["gt"], off)
        out.append(nr)
    return out
