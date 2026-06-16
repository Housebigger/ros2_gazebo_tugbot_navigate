"""ROS-free 2D pose-tracking helpers for odometry-based localization.

`quat_to_yaw` extracts the planar yaw from a quaternion; `compose_2d` composes
two planar rigid transforms (x, y, yaw). These let flood_fill_solver track the
robot on the maze grid from wheel odometry (`odom->base_link`) anchored by a
`map->odom` offset frozen once at startup -- sidestepping slam_toolbox's live
pose, which aliases and degrades under heavy exploration in the narrow,
repetitive corridors. (In sim, wheel odometry barely drifts and never aliases.)
"""
from __future__ import annotations
import math
from typing import Tuple

Pose2D = Tuple[float, float, float]


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Planar yaw (rad) of a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def compose_2d(parent_to_mid: Pose2D, mid_to_child: Pose2D) -> Pose2D:
    """Compose planar transforms parent->mid and mid->child into parent->child."""
    x1, y1, t1 = parent_to_mid
    x2, y2, t2 = mid_to_child
    c, s = math.cos(t1), math.sin(t1)
    return (x1 + c * x2 - s * y2, y1 + s * x2 + c * y2, t1 + t2)
