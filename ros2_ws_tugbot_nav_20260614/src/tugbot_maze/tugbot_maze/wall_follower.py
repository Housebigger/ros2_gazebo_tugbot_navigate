"""ROS-free reactive wall-following core for the perfect-maze solver.

A perfect maze (a tree: fully connected, no loops) with entrance and exit on the
outer boundary is provably solved by the wall-following rule: keep one hand on a
wall and you traverse the connected boundary until you reach an opening — the
exit. Wall-following holds a fixed lateral offset from the followed wall via
continuous LIDAR control, so it cannot wedge (no point goal is ever aimed into a
corner) and cannot loop forever (a tree has no cycles).

This module is pure policy: deterministic, no ROS / time / I/O. The node feeds it
a `Sectors` snapshot each tick and applies the returned `Command` to /cmd_vel.

Angle convention (robot frame): forward = 0, left = +pi/2, right = -pi/2.
`+w` = turn left (CCW). `+v` = forward.
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import math


@dataclass
class Sectors:
    front: float        # min range straight ahead (m)
    side: float         # min range on the followed side (m)
    front_side: float   # min range on the forward-diagonal toward the followed side (m)


@dataclass
class Command:
    v: float            # linear velocity (m/s), + forward
    w: float            # angular velocity (rad/s), + CCW (turn left)


class State(Enum):
    FIND_WALL = 'find_wall'
    FOLLOW = 'follow'
    TURN_AWAY = 'turn_away'
    CORNER = 'corner'


def _normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def sectorize(ranges, angle_min, angle_increment, follow_side, *,
              front_half_rad=math.radians(25),
              side_half_rad=math.radians(30),
              diag_half_rad=math.radians(25),
              max_range=12.0):
    """Reduce a LaserScan ranges array to (front, side, front_side) minima.

    front window: |ang| <= front_half_rad.
    side window (right-hand): |ang - (-pi/2)| <= side_half_rad; (left-hand): +pi/2.
    front_side window (right-hand): |ang - (-pi/4)| <= diag_half_rad; (left-hand): +pi/4.
    Non-finite or non-positive ranges are treated as max_range (no nearer hit).
    """
    if follow_side not in ('right', 'left'):
        raise ValueError("follow_side must be 'right' or 'left'")
    side_center = -math.pi / 2 if follow_side == 'right' else math.pi / 2
    diag_center = -math.pi / 4 if follow_side == 'right' else math.pi / 4
    front = side = front_side = max_range
    for i in range(len(ranges)):
        r = ranges[i]
        if r is None or not math.isfinite(r) or r <= 0.0:
            continue
        r = min(float(r), max_range)
        ang = _normalize_angle(angle_min + i * angle_increment)
        if abs(ang) <= front_half_rad:
            front = min(front, r)
        if abs(_normalize_angle(ang - side_center)) <= side_half_rad:
            side = min(side, r)
        if abs(_normalize_angle(ang - diag_center)) <= diag_half_rad:
            front_side = min(front_side, r)
    return Sectors(front=front, side=side, front_side=front_side)
