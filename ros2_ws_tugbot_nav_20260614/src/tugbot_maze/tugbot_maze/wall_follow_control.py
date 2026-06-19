"""ROS-free node-support helpers for the wall-following solver: exit detection,
entry-distance check, a stall watchdog, and entrance-seal geometry. Time is
injected (seconds, float) so these stay pure and unit-testable.
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


def entrance_seal_segment(center_xy, width_m, opening_side) -> Tuple[float, float, float, float]:
    """Return a map-frame segment (x0, y0, x1, y1) that fills a boundary opening.

    Used to "close the door" so the reactive follower treats the entrance as a
    solid wall once the robot is inside. `opening_side` 'left'/'right' is a
    vertical boundary wall, so the seal spans y; 'top'/'bottom' is a horizontal
    wall, so the seal spans x. width_m is the opening width.
    """
    cx, cy = float(center_xy[0]), float(center_xy[1])
    half = float(width_m) / 2.0
    if opening_side in ('left', 'right'):
        return (cx, cy - half, cx, cy + half)
    if opening_side in ('top', 'bottom'):
        return (cx - half, cy, cx + half, cy)
    raise ValueError("opening_side must be 'left', 'right', 'top', or 'bottom'")


def fuse_virtual_segment(ranges, angle_min, angle_inc, pose, segment, *,
                         wall_half_thickness_m=0.12, max_range=12.0) -> list[float]:
    """Fuse a single virtual wall `segment` into a LaserScan `ranges` list.

    Each beam range becomes min(real_range, distance-to-segment). This mirrors
    maze_sim's ray-segment intersection and half-thickness convention so the
    live "sealed door" matches the offline guarantee. `pose` = (x, y, yaw) and
    `segment` = (x0, y0, x1, y1) share the same (map) frame. Beams that miss the
    segment keep their real range. Non-finite / non-positive real ranges are
    treated as `max_range` before fusing. Returns a new list.
    """
    px, py, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    x0, y0, x1, y1 = (float(c) for c in segment)
    ex, ey = x1 - x0, y1 - y0
    wx, wy = x0 - px, y0 - py
    out = []
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r <= 0.0:
            r = max_range
        else:
            r = min(float(r), max_range)
        ang = yaw + angle_min + i * angle_inc
        dx, dy = math.cos(ang), math.sin(ang)
        denom = dx * ey - dy * ex
        if abs(denom) > 1e-12:
            t = (wx * ey - wy * ex) / denom          # ray param == distance (d is unit)
            u = (wx * dy - wy * dx) / denom          # segment param in [0, 1]
            if t >= 0.0 and 0.0 <= u <= 1.0:
                # t < half-thickness only if the robot is essentially inside the
                # virtual wall (shouldn't happen — the seal arms after entry); clamp to 0.
                seal_r = max(0.0, min(t - wall_half_thickness_m, max_range))
                r = min(r, seal_r)
        out.append(r)
    return out
