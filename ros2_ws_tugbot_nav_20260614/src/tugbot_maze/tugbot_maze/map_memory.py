"""Dedicated manager for the explored cell-map memory.

A targeted gateway around FloodFillBrain owning the two integrity behaviors the scattered
in-FSM marking lacks (run 20260625_082913: 33% collisions, 54% planner<->odom cell desync):
  (a) persistent-desync odom reconcile -- snap the planner cell to the odom cell when they stay
      disagreed while the robot is stuck (the (4,9)/(5,9) doorway straddle), and
  (b) mark-guard -- suppress a failure-driven WALL mark that is a lateral-pin CONTROL failure
      (open front, jammed on a side wall well off the centerline), not a real wall.
Sensing-driven marks stay in MazeMotion._center (already quality-gated). ROS-free; no time/IO
beyond the monotonic t passed in.
"""
from __future__ import annotations
from typing import Optional

from tugbot_maze.flood_fill_brain import FloodFillBrain, pose_to_cell, in_grid, Cell


class MapMemory:
    def __init__(self, brain: FloodFillBrain, *, front_open_m: float = 1.3,
                 pin_cross_m: float = 0.5, reconcile_persist_s: float = 8.0):
        self.brain = brain
        self.front_open_m = front_open_m          # a forward reading > this is "open" (no wall ahead)
        self.pin_cross_m = pin_cross_m            # |cross_track| beyond this is "well off centerline"
        self.reconcile_persist_s = reconcile_persist_s
        self._desync_since: Optional[float] = None
        self.suppressed = 0                       # observable: false WALL marks suppressed
        self.reconciles = 0                       # observable: forced odom snaps

    def is_lateral_pin(self, perp_front, near, cross_track, safety_radius) -> bool:
        """A hop failure is a lateral pin (control failure, NOT a wall) when the forward path is
        open but the robot is jammed against a side wall well off the corridor centerline."""
        return (perp_front is not None and perp_front > self.front_open_m
                and near is not None and near < safety_radius
                and abs(cross_track) > self.pin_cross_m)

    def mark_wall_on_failure(self, cell: Cell, d: str, *, perp_front, near,
                             cross_track, safety_radius) -> bool:
        """Gateway for hop-failure WALL marks. Suppress (return False) when it's a lateral pin;
        otherwise mark the wall on the brain (return True)."""
        if self.is_lateral_pin(perp_front, near, cross_track, safety_radius):
            self.suppressed += 1
            return False
        self.brain.mark(cell, d, is_wall=True)
        return True

    def observe(self, dcell: Cell, x: float, y: float, t: float) -> None:
        """Call every tick. Track how long the planner cell has disagreed with the odom cell."""
        odom = pose_to_cell(x, y)
        if (not in_grid(odom)) or odom == dcell:
            self._desync_since = None
        elif self._desync_since is None:
            self._desync_since = t

    def reconcile_target(self, dcell: Cell, x: float, y: float, t: float) -> Cell:
        """Return the odom cell to adopt when the desync has persisted >= reconcile_persist_s,
        else dcell unchanged. Call at a safe settled point (center phase). The bounded 1-cell
        hysteretic re-anchor remains the gentle path; this is the forced fallback for a stuck
        boundary-straddle (the (4,9) trap)."""
        odom = pose_to_cell(x, y)
        if (in_grid(odom) and odom != dcell and self._desync_since is not None
                and (t - self._desync_since) >= self.reconcile_persist_s):
            self._desync_since = None
            self.reconciles += 1
            return odom
        return dcell
