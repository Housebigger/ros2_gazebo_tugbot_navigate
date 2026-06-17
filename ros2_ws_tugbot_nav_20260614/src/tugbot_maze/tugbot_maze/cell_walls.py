"""Sense a maze cell's N/E/S/W walls directly from a live LaserScan.

The robot sits near a cell center; for each MAP-frame cardinal direction we look at the
LIDAR beams pointing that way (rotated into the scan frame by the robot's yaw) and take
the minimum range. A wall on a 2 m cell's edge is ~1 m away, so a min range below
`wall_dist_m` means WALL, otherwise OPEN. Sensing from the live scan is immediate and
reliable -- no SLAM-map lag/sparsity -- and matches the offline ground-truth at every
cell (validated by test). A 360-degree scan sees every direction, so there is no UNKNOWN.

Replaces the earlier SLAM-occupancy-grid sensing, which under-detected walls from the
sparse live map and stalled the robot at the first turn.
"""
from __future__ import annotations
import math
from typing import Dict

# MAP-frame angle of each cell direction (E=+x, N=+y, W=-x, S=-y).
_DIR_MAP_ANGLE = {'E': 0.0, 'N': math.pi / 2.0, 'W': math.pi, 'S': -math.pi / 2.0}


def _norm(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def cell_wall_min_ranges(ranges, angle_min, angle_inc, yaw, *,
                         half_window_rad: float = math.radians(22),
                         max_range: float = 12.0) -> Dict[str, float]:
    """Min LIDAR range (m, capped at max_range) within +/-half_window of each MAP
    cardinal. `yaw` is the robot heading in the map frame; beam i points in map
    direction yaw + (angle_min + i*angle_inc). Returned per-direction floats are what
    sense_cell_walls thresholds; exposed separately for diagnostics."""
    mins = {d: max_range for d in _DIR_MAP_ANGLE}
    targets = {d: _norm(a - yaw) for d, a in _DIR_MAP_ANGLE.items()}   # scan-frame angle per dir
    for i in range(len(ranges)):
        r = ranges[i]
        if r is None or not math.isfinite(r) or r <= 0.0:
            continue
        r = min(float(r), max_range)
        ang = _norm(angle_min + i * angle_inc)
        for d, ta in targets.items():
            if r < mins[d] and abs(_norm(ang - ta)) <= half_window_rad:
                mins[d] = r
    return mins


def sense_cell_walls(ranges, angle_min, angle_inc, yaw, *, wall_dist_m: float = 1.3,
                     half_window_rad: float = math.radians(22),
                     max_range: float = 12.0) -> Dict[str, bool]:
    """Return {'N'/'S'/'E'/'W': True if a wall is within wall_dist_m in that MAP
    direction, else False}. `yaw` is the robot heading in the map frame; LaserScan
    beam i points in map direction yaw + (angle_min + i*angle_inc)."""
    mins = cell_wall_min_ranges(ranges, angle_min, angle_inc, yaw,
                                half_window_rad=half_window_rad, max_range=max_range)
    return {d: mins[d] < wall_dist_m for d in _DIR_MAP_ANGLE}
