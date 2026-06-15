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


class WallFollower:
    """Stateful-but-pure right/left-hand wall-following policy.

    `update(Sectors) -> Command` is a deterministic function of (self.state,
    Sectors). No ROS, time, or progress input — stall recovery lives in the node.

    Decision order each tick (right-hand; mirror for left via `open_turn`):
      1. front < front_block_m -> TURN_AWAY: rotate toward the open side in place.
      2. FIND_WALL (initial) and nothing within engage_m -> creep forward. (CORNER
         is gated out of FIND_WALL so an open start creeps instead of spinning.)
      3. Once engaged, wall lost (side > wall_lost_m) with hysteresis -> CORNER: arc
         toward the wall while creeping, until the wall is regained (side <= target)
         and the minimum dwell has elapsed.
      4. FOLLOW: PID on (side - target_wall_m); slow down while turning hard.
    """

    def __init__(self, *, target_wall_m=0.6, front_block_m=0.7, wall_lost_m=1.2,
                 engage_m=1.0, cruise_v=0.3, corner_v=0.18, corner_w=0.6,
                 turn_w=0.7, w_max=0.8, kp=1.5, kd=0.4, follow_side='right',
                 min_state_ticks=2):
        if follow_side not in ('right', 'left'):
            raise ValueError("follow_side must be 'right' or 'left'")
        self.target_wall_m = target_wall_m
        self.front_block_m = front_block_m
        self.wall_lost_m = wall_lost_m
        self.engage_m = engage_m
        self.cruise_v = cruise_v
        self.corner_v = corner_v
        self.corner_w = corner_w
        self.turn_w = turn_w
        self.w_max = w_max
        self.kp = kp
        self.kd = kd
        self.follow_side = follow_side
        self.min_state_ticks = min_state_ticks
        # +1 means "turn toward the open side" is +w (CCW). Right-hand: open side
        # is the LEFT, so open_turn = +1. Left-hand: open side is the right, -1.
        self.open_turn = 1.0 if follow_side == 'right' else -1.0
        self.state = State.FIND_WALL
        self._prev_err = 0.0
        self._ticks_in_state = 0

    def _enter(self, state: State) -> None:
        if state != self.state:
            self.state = state
            self._ticks_in_state = 0

    def update(self, s: Sectors) -> Command:
        self._ticks_in_state += 1

        # 1. Safety: blocked ahead always preempts -> rotate toward the open side
        #    in place (inside corner / T-stem).
        if s.front < self.front_block_m:
            self._enter(State.TURN_AWAY)
            return Command(v=0.0, w=self.open_turn * self.turn_w)

        # 2. FIND_WALL: nothing engaged yet -> creep straight until any wall comes
        #    within engage range. CORNER is gated out here, or an open start (every
        #    side beyond wall_lost_m) would arc in a circle instead of advancing.
        if self.state == State.FIND_WALL:
            if min(s.front, s.side, s.front_side) > self.engage_m:
                return Command(v=self.cruise_v, w=0.0)
            # a wall is near -> engage by falling through to FOLLOW
        else:
            # 3. Wall lost (only once engaged): arc back toward the wall to round an
            #    outside corner / junction. Hysteresis: once cornering, keep arcing
            #    until the wall is regained (side <= target) AND min dwell elapsed.
            cornering = self.state == State.CORNER
            if (not cornering and s.side > self.wall_lost_m) or \
               (cornering and (s.side > self.target_wall_m
                               or self._ticks_in_state < self.min_state_ticks)):
                self._enter(State.CORNER)
                return Command(v=self.corner_v, w=-self.open_turn * self.corner_w)

        # 4. FOLLOW: PID on lateral offset. err > 0 (too far) -> steer toward wall.
        self._enter(State.FOLLOW)
        err = s.side - self.target_wall_m
        derr = err - self._prev_err
        self._prev_err = err
        w = -self.open_turn * (self.kp * err + self.kd * derr)
        w = max(-self.w_max, min(self.w_max, w))
        v = self.cruise_v * (1.0 - 0.5 * min(1.0, abs(w) / self.w_max))
        return Command(v=v, w=w)
