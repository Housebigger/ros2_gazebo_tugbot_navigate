#!/usr/bin/env python3
"""Guided Corridor Navigation (GCN) for the tugbot maze.

Uses a pre-computed BFS solution path from the 2D maze image to guide
the robot through the maze corridor-by-corridor, instead of blind DFS
exploration.  Each corridor is defined by a direction, guide coordinate,
and target endpoint.  The navigator tracks progress through the corridor
sequence and provides goal targets and preferred heading directions.

Coordinate convention: map frame with entrance at approximately (0, 0).
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

Point = Tuple[float, float]

# Direction angle constants (map frame yaw in radians).
DIR_ANGLE: Dict[str, float] = {
    'N': math.pi / 2,
    'S': -math.pi / 2,
    'E': 0.0,
    'W': math.pi,
}

# Each corridor is a dict with:
#   id        – sequential corridor number (1-based)
#   dir       – 'N', 'S', 'E', or 'W'
#   guide     – (axis, value) e.g. ('x', 2.0) means keep x ≈ 2.0
#   start     – (x, y) start point in map frame
#   end       – (x, y) end point in map frame
#   len       – corridor length in metres
#
# Derived from BFS on the 360×360 maze image (maze_20260528.png).
# The maze has a 10×9 cell grid.  Cell column centers: x = 2,4,…,20.
# Cell row centers: y = 1.0, 4.0, 6.05, 8.05, 10.05, 12.0, 14.0, 16.0, 18.0.
#
# Previous version used wall positions as guides (x=1.0 is the LEFT EXTERIOR
# WALL, not a corridor center).  This version uses cell CENTER coordinates.
#
# BFS shortest path: 30 cells, entrance at cell (0,0) x=2 y=1,
# exit at cell (9,8) x=20 y=18 through right exterior wall gap.

MAZE_CORRIDORS: List[Dict] = [
    # C1: Entrance approach → cells (0,0)→(0,1)
    {
        'id': 1,
        'dir': 'N',
        'guide': ('x', 2.0),
        'start': (2.0, 0.0),
        'end': (2.0, 4.0),
        'len': 4.0,
    },
    # C2: East along row 1 → cells (0,1)→(1,1)→(2,1)→(3,1)→(4,1)
    {
        'id': 2,
        'dir': 'E',
        'guide': ('y', 4.0),
        'start': (2.0, 4.0),
        'end': (10.0, 4.0),
        'len': 8.0,
    },
    # C3: North through col 4 → cells (4,1)→(4,2)→(4,3)→(4,4)
    {
        'id': 3,
        'dir': 'N',
        'guide': ('x', 10.0),
        'start': (10.0, 4.0),
        'end': (10.0, 10.05),
        'len': 6.05,
    },
    # C4: West along row 4 → cells (4,4)→(3,4)→(2,4)
    {
        'id': 4,
        'dir': 'W',
        'guide': ('y', 10.05),
        'start': (10.0, 10.05),
        'end': (6.0, 10.05),
        'len': 4.0,
    },
    # C5: North through col 2 → cells (2,4)→(2,5)→(2,6)→(2,7)
    {
        'id': 5,
        'dir': 'N',
        'guide': ('x', 6.0),
        'start': (6.0, 10.05),
        'end': (6.0, 16.0),
        'len': 5.95,
    },
    # C6: East along row 7 → cells (2,7)→(3,7)→(4,7)
    {
        'id': 6,
        'dir': 'E',
        'guide': ('y', 16.0),
        'start': (6.0, 16.0),
        'end': (10.0, 16.0),
        'len': 4.0,
    },
    # C7: North to top row → cell (4,7)→(4,8)
    {
        'id': 7,
        'dir': 'N',
        'guide': ('x', 10.0),
        'start': (10.0, 16.0),
        'end': (10.0, 18.0),
        'len': 2.0,
    },
    # C8: East along top row → cells (4,8)→(5,8)→(6,8)
    {
        'id': 8,
        'dir': 'E',
        'guide': ('y', 18.0),
        'start': (10.0, 18.0),
        'end': (14.0, 18.0),
        'len': 4.0,
    },
    # C9: South through col 6 → cell (6,8)→(6,7)
    {
        'id': 9,
        'dir': 'S',
        'guide': ('x', 14.0),
        'start': (14.0, 18.0),
        'end': (14.0, 16.0),
        'len': 2.0,
    },
    # C10: East to col 7 → cell (6,7)→(7,7)
    {
        'id': 10,
        'dir': 'E',
        'guide': ('y', 16.0),
        'start': (14.0, 16.0),
        'end': (16.0, 16.0),
        'len': 2.0,
    },
    # C11: South through col 7 → cell (7,7)→(7,6)
    {
        'id': 11,
        'dir': 'S',
        'guide': ('x', 16.0),
        'start': (16.0, 16.0),
        'end': (16.0, 14.0),
        'len': 2.0,
    },
    # C12: East to col 8 → cell (7,6)→(8,6)
    {
        'id': 12,
        'dir': 'E',
        'guide': ('y', 14.0),
        'start': (16.0, 14.0),
        'end': (18.0, 14.0),
        'len': 2.0,
    },
    # C13: South through col 8 → cell (8,6)→(8,5)
    {
        'id': 13,
        'dir': 'S',
        'guide': ('x', 18.0),
        'start': (18.0, 14.0),
        'end': (18.0, 12.0),
        'len': 2.0,
    },
    # C14: East to col 9 → cell (8,5)→(9,5)
    {
        'id': 14,
        'dir': 'E',
        'guide': ('y', 12.0),
        'start': (18.0, 12.0),
        'end': (20.0, 12.0),
        'len': 2.0,
    },
    # C15: North through col 9 → cells (9,5)→(9,6)→(9,7)
    {
        'id': 15,
        'dir': 'N',
        'guide': ('x', 20.0),
        'start': (20.0, 12.0),
        'end': (20.0, 16.0),
        'len': 4.0,
    },
    # C16: West to col 8 → cell (9,7)→(8,7)
    {
        'id': 16,
        'dir': 'W',
        'guide': ('y', 16.0),
        'start': (20.0, 16.0),
        'end': (18.0, 16.0),
        'len': 2.0,
    },
    # C17: North through col 8 to top row → cell (8,7)→(8,8)
    {
        'id': 17,
        'dir': 'N',
        'guide': ('x', 18.0),
        'start': (18.0, 16.0),
        'end': (18.0, 18.0),
        'len': 2.0,
    },
    # C18: East to exit → cell (8,8)→(9,8) + exit through right wall gap
    {
        'id': 18,
        'dir': 'E',
        'guide': ('y', 18.0),
        'start': (18.0, 18.0),
        'end': (21.0, 18.0),
        'len': 3.0,
    },
]


class CorridorNavigator:
    """Tracks progress through the pre-computed corridor sequence.

    Usage:
        nav = CorridorNavigator()
        while not nav.finished:
            goal = nav.current_goal(robot_xy)
            # navigate to goal ...
            if nav.check_advance(robot_xy):
                nav.advance()
    """

    def __init__(self) -> None:
        self.corridors = MAZE_CORRIDORS
        self.idx: int = 0          # current corridor index (0-based)
        self.sub_idx: int = 0      # current sub-segment index within corridor
        self.entry_xy: Optional[Point] = None  # robot xy when entering current corridor
        self.nav2_fail_count: int = 0  # consecutive Nav2 failures in current corridor
        self.reactive_count: int = 0   # reactive drive attempts in current corridor
        self.completed: bool = False

    # ── public queries ──────────────────────────────────────────────

    @property
    def finished(self) -> bool:
        return self.completed or self.idx >= len(self.corridors)

    @property
    def current(self) -> Dict:
        """Current corridor dict."""
        return self.corridors[self.idx]

    @property
    def current_dir(self) -> str:
        """Current movement direction ('N', 'S', 'E', 'W')."""
        c = self.current
        sub = c.get('sub_segs')
        if sub and self.sub_idx < len(sub):
            return sub[self.sub_idx][0]
        return c['dir']

    @property
    def current_angle(self) -> float:
        """Current heading in map frame (radians)."""
        return DIR_ANGLE[self.current_dir]

    @property
    def current_target(self) -> Point:
        """Target point for the current (sub-)segment end."""
        c = self.current
        sub = c.get('sub_segs')
        if sub and self.sub_idx < len(sub):
            return sub[self.sub_idx][2]
        return c['end']

    @property
    def corridor_label(self) -> str:
        c = self.current
        label = f'C{c["id"]}:{c["dir"]}'
        sub = c.get('sub_segs')
        if sub and self.sub_idx < len(sub):
            label += f'[{self.sub_idx+1}/{len(sub)}]'
        return label

    @property
    def progress_pct(self) -> float:
        """Overall progress 0..100."""
        n = len(self.corridors)
        if n == 0:
            return 100.0
        # Count fully completed corridors plus fractional current
        done = self.idx
        if not self.finished:
            c = self.current
            done += self._corridor_fraction(c)
        return min(100.0, done / n * 100.0)

    # ── goal generation ─────────────────────────────────────────────

    def current_goal(self, robot_xy: Point, max_step: float = 2.5) -> Point:
        """Return the next Nav2 goal for the current corridor.

        The goal is a point along the corridor direction, up to max_step m
        ahead of the robot (capped at the corridor end).  The guide coordinate
        is enforced to keep the robot in the corridor.
        """
        c = self.current
        target = self.current_target
        step = min(max_step, self._dist_to_target(robot_xy, target))
        step = max(step, 0.8)  # don't send sub-metre goals

        gx, gy = robot_xy
        d = self.current_dir
        guide_axis, guide_val = c['guide']

        if d in ('N', 'S'):
            sign = 1.0 if d == 'N' else -1.0
            gy = gy + sign * step
            gx = guide_val  # snap to guide x
        else:
            sign = 1.0 if d == 'E' else -1.0
            gx = gx + sign * step
            gy = guide_val  # snap to guide y

        # Clamp to corridor end
        tx, ty = target
        gx = self._clamp_progress(gx, robot_xy[0], tx)
        gy = self._clamp_progress(gy, robot_xy[1], ty)

        return (round(gx, 2), round(gy, 2))

    # ── progress / advance ──────────────────────────────────────────

    def _dist_to_target(self, robot_xy: Point, target: Point) -> float:
        dx = target[0] - robot_xy[0]
        dy = target[1] - robot_xy[1]
        return math.sqrt(dx * dx + dy * dy)

    def _clamp_progress(self, val: float, robot_val: float, target_val: float) -> float:
        """Clamp val so it doesn't overshoot the target relative to robot."""
        if target_val > robot_val:
            return min(val, target_val)
        elif target_val < robot_val:
            return max(val, target_val)
        return val

    def _corridor_fraction(self, corridor: Dict) -> float:
        """Estimate 0..1 progress through the current corridor (placeholder)."""
        # Simplified: sub-segment index / total sub-segments
        sub = corridor.get('sub_segs')
        if sub:
            return self.sub_idx / len(sub)
        return 0.0

    def dist_to_current_target(self, robot_xy: Point) -> float:
        """Distance from robot to current (sub-)segment target."""
        return self._dist_to_target(robot_xy, self.current_target)

    def check_advance(self, robot_xy: Point, tolerance: float = 1.5) -> bool:
        """Check if the robot has reached (within tolerance) the current target.

        For N/S corridors, checks if y is within tolerance of target y.
        For E/W corridors, checks if x is within tolerance of target x.
        """
        if self.finished:
            return False
        target = self.current_target
        c = self.current
        d = self.current_dir

        if d in ('N', 'S'):
            diff = abs(robot_xy[1] - target[1])
        else:
            diff = abs(robot_xy[0] - target[0])

        return diff <= tolerance

    def advance(self) -> None:
        """Move to the next (sub-)segment or corridor."""
        c = self.current
        sub = c.get('sub_segs')

        if sub and self.sub_idx + 1 < len(sub):
            self.sub_idx += 1
            self.nav2_fail_count = 0
            self.reactive_count = 0
            return

        # Next corridor
        self.idx += 1
        self.sub_idx = 0
        self.nav2_fail_count = 0
        self.reactive_count = 0
        if self.idx >= len(self.corridors):
            self.completed = True

    def record_nav2_failure(self) -> None:
        self.nav2_fail_count += 1

    def record_reactive_drive(self) -> None:
        self.reactive_count += 1

    @property
    def should_try_reactive(self) -> bool:
        """True if we should attempt reactive drive (≥2 Nav2 failures, <5 reactive)."""
        return self.nav2_fail_count >= 2 and self.reactive_count < 5

    @property
    def corridor_exhausted(self) -> bool:
        """True if this corridor seems stuck (many failures)."""
        return self.nav2_fail_count >= 6 and self.reactive_count >= 5

    def set_entry(self, robot_xy: Point) -> None:
        """Record the robot position when entering a new corridor."""
        self.entry_xy = robot_xy

    def relative_progress(self, robot_xy: Point) -> float:
        """Metres of forward progress since entering the current corridor.

        Uses the corridor direction to compute signed progress.
        """
        if self.entry_xy is None:
            return 0.0
        d = self.current_dir
        if d == 'N':
            return robot_xy[1] - self.entry_xy[1]
        elif d == 'S':
            return self.entry_xy[1] - robot_xy[1]
        elif d == 'E':
            return robot_xy[0] - self.entry_xy[0]
        else:  # W
            return self.entry_xy[0] - robot_xy[0]

    def summary(self) -> str:
        """One-line status summary for logging."""
        if self.finished:
            return 'GCN: FINISHED (all corridors completed)'
        c = self.current
        return (
            f'GCN: corridor {self.corridor_label} '
            f'({self.progress_pct:.0f}%) '
            f'target=({self.current_target[0]:.1f},{self.current_target[1]:.1f}) '
            f'nav2_fail={self.nav2_fail_count} reactive={self.reactive_count}'
        )
