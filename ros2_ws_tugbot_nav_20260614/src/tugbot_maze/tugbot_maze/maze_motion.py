"""ROS-free maze motion controller for the cell-grid flood-fill solver.

`MazeMotion` is the whole motion layer: a phase FSM (center -> turn -> drive) that drives the
robot cell-to-cell to the exit using wall-referenced sensing (cell_walls) and odom-relative
motion, plus the FloodFillBrain for routing. It takes (pose, scan, t) each tick and returns
(v, w, done). No ROS / time / I/O beyond the monotonic `t` passed in, so it can be driven
end-to-end through the offline inertia+collision maze_sim for validation. The ROS node is a
thin adapter that feeds pose/scan and publishes (v, w).

Phases:
  center : 1-axis re-center on referenced walls (along-track re-anchor), settle, sense once,
           pick next_cell, set the hop's target cardinal -> turn.
  turn   : rotate in place to the target cardinal on odom yaw; declared done only after
           `turn_settle_ticks` consecutive in-tolerance ticks (settle check) -> drive.
  drive  : corridor pure-pursuit drive toward the cardinal until arrival (dead-reckoned ~one
           cell) -> advance cell -> center; or a real front wall -> mark wall -> center; an
           open-front timeout is a transient stall (retry, bounded) not a wall.
"""
from __future__ import annotations
import math
from typing import Optional, Tuple

from tugbot_maze.flood_fill_brain import (
    FloodFillBrain, ENTRANCE_CELL, EXIT_CELL, CELL_SIZE_M)
from tugbot_maze.cell_walls import sense_cell_walls, cell_wall_perp_dist
from tugbot_maze.wall_localize import cell_center_offset
from tugbot_maze.hop_controller import (
    centering_command, cross_track_offset, corridor_drive_command)


def _norm(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _dir_name(d) -> str:
    return {(1, 0): 'E', (-1, 0): 'W', (0, 1): 'N', (0, -1): 'S'}[d]


class MazeMotion:
    def __init__(self, brain: Optional[FloodFillBrain] = None, *, cruise_v: float = 0.3,
                 w_max: float = 0.5, kp_turn: float = 1.2, center_tol_m: float = 0.10,
                 yaw_tol_rad: float = 0.10, turn_settle_ticks: int = 3,
                 hop_arrive_slack_m: float = 0.05, front_block_m: float = 0.7,
                 lookahead_m: float = 0.7, max_cross_track_m: float = 0.6,
                 wedge_slow_m: float = 0.50, wedge_stop_m: float = 0.40,
                 wedge_v_floor: float = 0.10, hop_timeout_s: float = 25.0,
                 settle_s: float = 0.4, max_open_timeouts: int = 3):
        self.brain = brain if brain is not None else FloodFillBrain(exit_cell=EXIT_CELL)
        self.cruise_v = cruise_v; self.w_max = w_max; self.kp_turn = kp_turn
        self.center_tol_m = center_tol_m; self.yaw_tol_rad = yaw_tol_rad
        self.turn_settle_ticks = turn_settle_ticks; self.hop_arrive_slack_m = hop_arrive_slack_m
        self.front_block_m = front_block_m; self.lookahead_m = lookahead_m
        self.max_cross_track_m = max_cross_track_m; self.wedge_slow_m = wedge_slow_m
        self.wedge_stop_m = wedge_stop_m; self.wedge_v_floor = wedge_v_floor
        self.hop_timeout_s = hop_timeout_s; self.settle_s = settle_s
        self.max_open_timeouts = max_open_timeouts
        self.cell = ENTRANCE_CELL
        self.phase = 'center'
        self.sensed = set()
        self.hop_dir = None; self.hop_target = None; self.hop_start = None
        self.target_cardinal = 0.0
        self.hop_deadline = 0.0; self.settle_until = 0.0; self.turn_in_tol = 0
        self.stall_key = None; self.stall_count = 0

    def step(self, pose, scan, t) -> Tuple[float, float, bool]:
        if self.phase == 'done' or self.cell == EXIT_CELL:
            self.phase = 'done'
            return (0.0, 0.0, True)
        if self.phase == 'stuck':
            return (0.0, 0.0, False)
        if self.phase == 'center':
            return self._center(pose, scan, t)
        if self.phase == 'turn':
            return self._turn(pose, t)
        if self.phase == 'drive':
            return self._drive(pose, scan, t)
        return (0.0, 0.0, False)

    def _center(self, pose, scan, t):
        x, y, yaw = pose
        ranges, amin, ainc = scan
        ox, oy = cell_center_offset(ranges, amin, ainc, yaw)
        v, w, centered = centering_command((x, y, yaw), ox, oy,
                                           tol=self.center_tol_m, yaw_tol=self.yaw_tol_rad)
        if not centered:
            self.settle_until = t + self.settle_s
            return (v, w, False)
        if t < self.settle_until:
            return (0.0, 0.0, False)
        if self.cell not in self.sensed:
            for d, is_wall in sense_cell_walls(ranges, amin, ainc, yaw).items():
                self.brain.mark(self.cell, d, is_wall)
            self.sensed.add(self.cell)
        nxt = self.brain.next_cell(self.cell)
        if nxt is None:
            self.phase = 'stuck'
            return (0.0, 0.0, False)
        if nxt != self.hop_target:                       # count a Tremaux traversal once per new hop
            self.brain.mark_traversal(self.cell, nxt)
        self.hop_target = nxt
        self.hop_dir = (nxt[0] - self.cell[0], nxt[1] - self.cell[1])
        self.target_cardinal = math.atan2(self.hop_dir[1], self.hop_dir[0])
        self.hop_start = (x, y)
        self.turn_in_tol = 0
        self.phase = 'turn'
        return (0.0, 0.0, False)

    def _turn(self, pose, t):
        yaw = pose[2]
        err = _norm(self.target_cardinal - yaw)
        if abs(err) <= self.yaw_tol_rad:
            self.turn_in_tol += 1
            if self.turn_in_tol >= self.turn_settle_ticks:
                self.hop_start = (pose[0], pose[1])       # measure the drive from here
                self.hop_deadline = t + self.hop_timeout_s
                self.phase = 'drive'
            return (0.0, 0.0, False)
        self.turn_in_tol = 0
        w = max(-self.w_max, min(self.w_max, self.kp_turn * err))
        return (0.0, w, False)

    def _drive(self, pose, scan, t):
        x, y, yaw = pose
        ranges, amin, ainc = scan
        moved = math.hypot(x - self.hop_start[0], y - self.hop_start[1])
        if moved >= CELL_SIZE_M - self.hop_arrive_slack_m:           # arrived
            self.cell = self.hop_target
            self.stall_key = None; self.stall_count = 0
            self.settle_until = t + self.settle_s
            self.phase = 'center'
            return (0.0, 0.0, False)
        perp = cell_wall_perp_dist(ranges, amin, ainc, yaw)
        dirn = _dir_name(self.hop_dir)
        if perp[dirn] < self.front_block_m and moved > 0.3:         # real wall ahead -> mark
            self.brain.mark(self.cell, dirn, is_wall=True)
            self.stall_key = None; self.stall_count = 0
            self.settle_until = t + self.settle_s
            self.phase = 'center'
            return (0.0, 0.0, False)
        if t >= self.hop_deadline:                                  # open-front timeout = stall
            key = (self.cell, dirn)
            self.stall_count = self.stall_count + 1 if key == self.stall_key else 1
            self.stall_key = key
            if self.stall_count >= self.max_open_timeouts:          # give up: mark (last resort)
                self.brain.mark(self.cell, dirn, is_wall=True)
                self.stall_key = None; self.stall_count = 0
                self.settle_until = t + self.settle_s
                self.phase = 'center'
                return (0.0, 0.0, False)
            self.hop_deadline = t + self.hop_timeout_s              # retry: keep hop_start, drive on
        ox, oy = cell_center_offset(ranges, amin, ainc, yaw)
        cross = cross_track_offset(ox, oy, self.hop_dir)
        if abs(cross) > self.max_cross_track_m:                    # bogus/desync -> hold cardinal
            cross = 0.0
        if self.hop_dir[1] != 0:                                   # N/S travel: sides are E/W
            near = min(perp['E'], perp['W'])
        else:                                                      # E/W travel: sides are N/S
            near = min(perp['N'], perp['S'])
        v, w = corridor_drive_command(yaw, self.target_cardinal, cross, near,
                                      v_max=self.cruise_v, w_max=self.w_max,
                                      lookahead_m=self.lookahead_m, wedge_slow_m=self.wedge_slow_m,
                                      wedge_stop_m=self.wedge_stop_m, wedge_v_floor=self.wedge_v_floor)
        return (v, w, False)
