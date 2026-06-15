"""ROS-free node-support helpers for the wall-following solver: exit detection,
entry-distance check, and a stall watchdog. Time is injected (seconds, float) so
these stay pure and unit-testable.
"""
from __future__ import annotations
import math
from typing import Optional, Tuple


def exit_reached(pose, exit_xy, radius_m) -> bool:
    """True if the robot pose (x, y, [yaw]) is within radius_m of the exit."""
    return math.hypot(pose[0] - exit_xy[0], pose[1] - exit_xy[1]) <= radius_m


def entering_done(start_xy, cur_xy, distance_m) -> bool:
    """True once the robot has travelled >= distance_m (straight-line) from start."""
    return math.hypot(cur_xy[0] - start_xy[0], cur_xy[1] - start_xy[1]) >= distance_m


class StallWatchdog:
    """Flags a stall: no net progress (>= progress_eps_m) for >= stall_s seconds.

    Anchors on the first sample; re-anchors whenever the robot moves at least
    progress_eps_m from the current anchor. `update(t, x, y)` returns True when the
    robot has sat within progress_eps_m of the anchor for at least stall_s.
    """

    def __init__(self, *, stall_s=4.0, progress_eps_m=0.2):
        self.stall_s = stall_s
        self.progress_eps_m = progress_eps_m
        self._anchor_t: Optional[float] = None
        self._anchor_xy: Optional[Tuple[float, float]] = None

    def update(self, t, x, y) -> bool:
        if self._anchor_xy is None:
            self._anchor_t, self._anchor_xy = t, (x, y)
            return False
        moved = math.hypot(x - self._anchor_xy[0], y - self._anchor_xy[1])
        if moved >= self.progress_eps_m:
            self._anchor_t, self._anchor_xy = t, (x, y)
            return False
        return (t - self._anchor_t) >= self.stall_s

    def reset(self, t, x, y) -> None:
        self._anchor_t, self._anchor_xy = t, (x, y)
