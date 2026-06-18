"""ROS-free wall-referenced localization for the maze cell grid.

`cell_center_offset` estimates the robot's offset from the current cell's true center
(map axes) from the min LIDAR range to each cardinal wall; `heading_snap` returns the
nearest cardinal. Used to re-center the robot against the physical walls each cell,
making the solver immune to wheel-odometry drift. Deterministic; no ROS/time/I/O.
"""
from __future__ import annotations
import math
from typing import Optional, Tuple

from tugbot_maze.cell_walls import cell_wall_min_ranges

HALF_CORRIDOR_M = 0.88   # cell half-width (1.0) minus wall half-thickness (0.12)
WALL_DIST_M = 1.3        # a min range below this in a cardinal window => wall present


def cell_center_offset(ranges, angle_min, angle_inc, yaw, *,
                       half_corridor_m: float = HALF_CORRIDOR_M,
                       wall_dist_m: float = WALL_DIST_M
                       ) -> Tuple[Optional[float], Optional[float]]:
    """Robot position minus true cell center, MAP axes (+x=E, +y=N). A component is
    None if that axis is an open corridor (no wall to reference)."""
    r = cell_wall_min_ranges(ranges, angle_min, angle_inc, yaw)

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


def heading_snap(yaw: float) -> Tuple[float, float]:
    """Nearest cardinal yaw and the signed, normalized rotation to reach it."""
    snapped = round(yaw / (math.pi / 2.0)) * (math.pi / 2.0)
    dyaw = math.atan2(math.sin(snapped - yaw), math.cos(snapped - yaw))
    return (math.atan2(math.sin(snapped), math.cos(snapped)), dyaw)
