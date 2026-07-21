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


def inverse_2d(p: Pose2D) -> Pose2D:
    """Inverse of a planar rigid transform."""
    x, y, t = p
    c, s = math.cos(t), math.sin(t)
    return (-(c * x + s * y), (s * x - c * y), -t)


def odom_prior(last_corrected: Pose2D, last_odom: Pose2D, cur_odom: Pose2D) -> Pose2D:
    """Propagate the last corrected map pose by the odom motion since the last tick.

    delta = (odom_last)^-1 o (odom_cur)  is the body-frame motion; applying it to
    the last corrected map pose gives the prior for the current tick's scan match.
    """
    delta = compose_2d(inverse_2d(last_odom), cur_odom)
    return compose_2d(last_corrected, delta)


def apply_odom_yaw_gate(est: Pose2D, odom_map: Pose2D, prior: Pose2D,
                        bound: float) -> Tuple[Pose2D, bool]:
    """Reject an ICP-accepted pose whose yaw disagrees with the drift-free odom yaw by more
    than `bound` rad, returning the odom-propagated prior instead. `odom_map` is the pure-odom
    pose in the map frame (compose_2d(entrance_anchor, odom_base)); its yaw is drift-free.

    This compares against the RAW odom orientation, unlike the inner ICP's per-step yaw clamp
    which compares against the previous corrected pose (itself drifting) -- so it catches the
    cumulative alias migration (sub-threshold accepts walking the pose into a ~90-degree/one-cell
    grid alias) that the per-step clamp misses. Returns (pose, gated: bool)."""
    d = est[2] - odom_map[2]
    if abs(math.atan2(math.sin(d), math.cos(d))) > bound:
        return prior, True
    return est, False


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    """(x, y, z, w) quaternion for a planar yaw (rad)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def map_to_odom(map_to_base: Pose2D, odom_to_base: Pose2D) -> Pose2D:
    """map->odom offset such that compose_2d(result, odom_to_base) == map_to_base.

    The solver holds map->base (its ICP pose) and reads odom->base from TF; publishing
    this offset as the map->odom transform makes the full TF tree resolve to the ICP pose.
    """
    return compose_2d(map_to_base, inverse_2d(odom_to_base))
