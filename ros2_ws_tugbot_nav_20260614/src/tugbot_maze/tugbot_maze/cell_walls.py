"""Sense a maze cell's N/E/S/W walls directly from a live LaserScan.

For each MAP-frame cardinal we look at the LIDAR beams pointing that way (rotated
into the scan frame by the robot's yaw) and estimate the PERPENDICULAR distance to
an axis-aligned wall in that direction: each in-window beam range r is projected
onto the cardinal axis (r*cos(angle-from-cardinal)), which equals the perpendicular
distance to a flat wall regardless of the robot's lateral offset, and we take the
MEDIAN over the window. The median rejects oblique corner/edge grazes -- the lone
short slant returns that fooled an earlier min-over-window estimator into reading an
OPEN edge as a wall when the robot was off-centre (the (1,4) dead-end trap). A wall
on a 2 m cell's edge sits ~0.88 m away perpendicular, so a distance below
`wall_dist_m` means WALL, otherwise OPEN. Sensing from the live scan is immediate and
reliable -- no SLAM-map lag/sparsity -- and matches the offline ground-truth at every
cell, at the centre AND off-centre by up to 0.45 m (validated by test). A 360-degree
scan sees every direction, so there is no UNKNOWN.

Replaces the earlier SLAM-occupancy-grid sensing, which under-detected walls from the
sparse live map and stalled the robot at the first turn.
"""
from __future__ import annotations
import math
from typing import Dict, List

# MAP-frame angle of each cell direction (E=+x, N=+y, W=-x, S=-y).
_DIR_MAP_ANGLE = {'E': 0.0, 'N': math.pi / 2.0, 'W': math.pi, 'S': -math.pi / 2.0}


def _norm(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _median(vals: List[float]) -> float:
    s = sorted(vals)
    n = len(s)
    mid = n // 2
    return s[mid] if n % 2 else 0.5 * (s[mid - 1] + s[mid])


def cell_wall_perp_dist(ranges, angle_min, angle_inc, yaw, *,
                        half_window_rad: float = math.radians(22),
                        max_range: float = 12.0) -> Dict[str, float]:
    """Robust perpendicular distance (m, capped at max_range) to a wall within
    +/-half_window of each MAP cardinal. `yaw` is the robot heading in the map frame;
    beam i points in map direction yaw + (angle_min + i*angle_inc). Each in-window beam
    is projected onto the cardinal axis (r*cos(off)) -- the perpendicular distance to a
    flat axis-aligned wall, invariant to the robot's lateral offset -- and the
    per-direction MEDIAN is returned, which rejects oblique corner/edge grazes. A
    no-return beam counts as max_range (open). Exposed for diagnostics; the returned
    floats are what sense_cell_walls / cell_center_offset threshold."""
    proj: Dict[str, List[float]] = {d: [] for d in _DIR_MAP_ANGLE}
    targets = {d: _norm(a - yaw) for d, a in _DIR_MAP_ANGLE.items()}   # scan-frame angle per dir
    for i in range(len(ranges)):
        r = ranges[i]
        if r is None or not math.isfinite(r) or r <= 0.0:
            r = max_range                                  # no return => that beam sees open
        else:
            r = min(float(r), max_range)
        ang = _norm(angle_min + i * angle_inc)
        for d, ta in targets.items():
            off = _norm(ang - ta)
            if abs(off) <= half_window_rad:
                proj[d].append(r * math.cos(off))          # project onto the cardinal axis
    return {d: (_median(v) if v else max_range) for d, v in proj.items()}


def sense_cell_walls(ranges, angle_min, angle_inc, yaw, *, wall_dist_m: float = 1.3,
                     half_window_rad: float = math.radians(22),
                     max_range: float = 12.0) -> Dict[str, bool]:
    """Return {'N'/'S'/'E'/'W': True if a wall is within wall_dist_m (perpendicular) in
    that MAP direction, else False}. `yaw` is the robot heading in the map frame;
    LaserScan beam i points in map direction yaw + (angle_min + i*angle_inc)."""
    d = cell_wall_perp_dist(ranges, angle_min, angle_inc, yaw,
                            half_window_rad=half_window_rad, max_range=max_range)
    return {k: d[k] < wall_dist_m for k in _DIR_MAP_ANGLE}
