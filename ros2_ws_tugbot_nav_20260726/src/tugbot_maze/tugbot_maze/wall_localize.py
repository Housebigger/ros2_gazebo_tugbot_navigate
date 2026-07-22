"""ROS-free wall-referenced localization for the maze cell grid.

`cell_center_offset` estimates the robot's offset from the current cell's true center
(map axes) from the min LIDAR range to each cardinal wall; `heading_snap` returns the
nearest cardinal. Used to re-center the robot against the physical walls each cell,
making the solver immune to wheel-odometry drift. Deterministic; no ROS/time/I/O.
"""
from __future__ import annotations
import math
from typing import Dict, Optional, Tuple

from tugbot_maze.cell_walls import cell_wall_perp_dist
from tugbot_maze.flood_fill_brain import CELL_SIZE_M

HALF_CORRIDOR_M = 0.88   # cell half-width (1.0) minus wall half-thickness (0.12)
WALL_DIST_M = 1.3        # a min range below this in a cardinal window => wall present

# perimeter wall NEAR-SURFACE coordinate (centerline -/+ wall_half_thickness 0.12).
_PERIM_SURF: Dict[str, float] = {
    'E': 21.01 - 0.12,
    'W': 1.02 + 0.12,
    'N': 19.02 - 0.12,
    'S': -0.97 + 0.12,
}
PERIM_MAX_M = 2.5       # trust the perimeter return only within this perp distance
_E_WALL_MAX_CY = 8      # E perimeter wall ends at y~17.1 -> cells cy<=8 see it; cy=9 uses N


def perimeter_walls_for(cell: Tuple[int, int]):
    """The perimeter cardinals this boundary cell faces (grid cx 1..10, cy 0..9)."""
    cx, cy = cell
    walls = []
    if cx == 10 and cy <= _E_WALL_MAX_CY:
        walls.append('E')
    if cx == 1:
        walls.append('W')
    if cy == 9:
        walls.append('N')
    if cy == 0:
        walls.append('S')
    return walls


def perimeter_offset(ranges, angle_min, angle_inc, yaw, cell, *,
                     max_range: float = 12.0) -> Dict[int, Tuple[float, int]]:
    """For each perimeter wall `cell` faces, return the ABSOLUTE true coordinate on
    that axis (drift-immune, inferred from the outer wall distance) and the implied
    cell index. Returns {} if the cell has no facing perimeter wall or the wall is
    out of PERIM_MAX_M range.

    axis 0 = x (E/W), axis 1 = y (N/S).
    Result: {axis: (true_coordinate, implied_cell_index)}.
    """
    walls = perimeter_walls_for(cell)
    if not walls:
        return {}
    perp = cell_wall_perp_dist(ranges, angle_min, angle_inc, yaw, max_range=max_range)
    out: Dict[int, Tuple[float, int]] = {}
    for d in walls:
        dist = perp[d]
        if not (dist < max_range and dist <= PERIM_MAX_M):
            continue
        if d == 'E':
            true, axis = _PERIM_SURF['E'] - dist, 0
        elif d == 'W':
            true, axis = _PERIM_SURF['W'] + dist, 0
        elif d == 'N':
            true, axis = _PERIM_SURF['N'] - dist, 1
        else:   # 'S'
            true, axis = _PERIM_SURF['S'] + dist, 1
        out[axis] = (true, int(round(true / CELL_SIZE_M)))
    return out


def cell_center_offset(ranges, angle_min, angle_inc, yaw, *,
                       half_corridor_m: float = HALF_CORRIDOR_M,
                       wall_dist_m: float = WALL_DIST_M
                       ) -> Tuple[Optional[float], Optional[float]]:
    """Robot position minus true cell center, MAP axes (+x=E, +y=N). A component is
    None if that axis is an open corridor (no wall to reference)."""
    r = cell_wall_perp_dist(ranges, angle_min, angle_inc, yaw)

    def axis(d_pos, d_neg):
        pos, neg = d_pos < wall_dist_m, d_neg < wall_dist_m
        if pos and neg:
            return (d_neg - d_pos) / 2.0
        if pos:
            return half_corridor_m - d_pos
        if neg:
            return d_neg - half_corridor_m
        return None

    return (axis(r['E'], r['W']), axis(r['N'], r['S']))


def gate_offset_against_pose(ox: Optional[float], oy: Optional[float],
                             pose_x: float, pose_y: float,
                             center_x: float, center_y: float, *,
                             tol: float = 0.35
                             ) -> Tuple[Optional[float], Optional[float], bool]:
    """Cross-check the wall-referenced center offset against the pose-derived one.

    Returns (ox, oy, clean): a wall-derived component disagreeing with
    (pose - cell_center) by more than tol is dropped to None and clean goes
    False. The two disagree when a pipeline stall leaves the scan stale while
    the robot keeps moving (20260722 forensics: two 6.4s MATCH gaps vs the
    5.00s cadence bracketed a wall graze at (10,6) -- the stale east wall
    read ~0.7m far and flipped ox from +0.31 to -0.42, steering INTO the
    wall; at the quadruped's 0.23 m/s the same stall stayed inside tolerance,
    the kinematic buggy's 0.4 m/s doubled it). Clean-run agreement is
    ~0.02-0.15 m, the stale contradiction 0.73 m; tol=0.35 splits them.
    The absolute pose is trustworthy here post the odom-yaw gate (c52e951);
    consumers use `clean` to fail CLOSED on the sense-commit quality gate."""
    clean = True
    px, py = pose_x - center_x, pose_y - center_y
    if ox is not None and abs(ox - px) > tol:
        ox, clean = None, False
    if oy is not None and abs(oy - py) > tol:
        oy, clean = None, False
    return ox, oy, clean


def heading_snap(yaw: float) -> Tuple[float, float]:
    """Nearest cardinal yaw and the signed, normalized rotation to reach it."""
    snapped = round(yaw / (math.pi / 2.0)) * (math.pi / 2.0)
    dyaw = math.atan2(math.sin(snapped - yaw), math.cos(snapped - yaw))
    return (math.atan2(math.sin(snapped), math.cos(snapped)), dyaw)
