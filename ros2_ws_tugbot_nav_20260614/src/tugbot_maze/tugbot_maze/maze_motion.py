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
    FloodFillBrain, ENTRANCE_CELL, EXIT_CELL, CELL_SIZE_M, pose_to_cell, in_grid)
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
                 w_max: float = 0.5, kp_turn: float = 0.7, kd_turn: float = 0.6,
                 turn_w_max: float = 0.35, center_tol_m: float = 0.10,
                 yaw_tol_rad: float = 0.10, turn_settle_ticks: int = 3,
                 hop_arrive_slack_m: float = 0.05, front_block_m: float = 0.7,
                 lookahead_m: float = 0.7, max_cross_track_m: float = 0.6,
                 wedge_slow_m: float = 0.50, wedge_stop_m: float = 0.40,
                 wedge_v_floor: float = 0.10, hop_timeout_s: float = 25.0,
                 settle_s: float = 0.4, max_hop_attempts: int = 3,
                 center_timeout_s: float = 4.0, turn_timeout_s: float = 5.0):
        self.brain = brain if brain is not None else FloodFillBrain(exit_cell=EXIT_CELL)
        self.cruise_v = cruise_v; self.w_max = w_max; self.kp_turn = kp_turn
        self.kd_turn = kd_turn          # derivative damping on rotate-in-place (vs latency overshoot)
        self.turn_w_max = turn_w_max    # gentler cap for rotate-in-place (less latency overshoot)
        self.center_tol_m = center_tol_m; self.yaw_tol_rad = yaw_tol_rad
        self.turn_settle_ticks = turn_settle_ticks; self.hop_arrive_slack_m = hop_arrive_slack_m
        self.front_block_m = front_block_m; self.lookahead_m = lookahead_m
        self.max_cross_track_m = max_cross_track_m; self.wedge_slow_m = wedge_slow_m
        self.wedge_stop_m = wedge_stop_m; self.wedge_v_floor = wedge_v_floor
        self.hop_timeout_s = hop_timeout_s; self.settle_s = settle_s
        self.max_hop_attempts = max_hop_attempts; self.center_timeout_s = center_timeout_s
        self.turn_timeout_s = turn_timeout_s
        self.cell = ENTRANCE_CELL
        self.phase = 'center'
        self.sensed = set()
        self.hop_dir = None; self.hop_target = None; self.hop_start = None
        self.target_cardinal = 0.0
        self.hop_deadline = 0.0; self.settle_until = 0.0; self.turn_in_tol = 0
        self.hop_attempts = {}          # (cell,dir) -> failed-hop count (transient, NOT walls)
        self.turn_start = None          # when the current turn episode began (for the timeout)
        self.center_start = None        # when the current center episode began (for the timeout)
        self.prev_yaw = None; self.prev_t = None; self.yaw_rate = 0.0   # for PD damping
        self.dbg = {}                   # last-tick diagnostics (offset, walls, near) for logging

    def step(self, pose, scan, t) -> Tuple[float, float, bool]:
        yaw = pose[2]                                   # measured yaw rate (for PD damping)
        if self.prev_t is not None and t > self.prev_t:
            self.yaw_rate = _norm(yaw - self.prev_yaw) / (t - self.prev_t)
        self.prev_yaw = yaw; self.prev_t = t
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
        if self.center_start is None:
            self.center_start = t
        ox, oy = cell_center_offset(ranges, amin, ainc, yaw)
        v, w, centered = centering_command((x, y, yaw), ox, oy,
                                           tol=self.center_tol_m, yaw_tol=self.yaw_tol_rad,
                                           w_max=self.turn_w_max, kp_ang=self.kp_turn,
                                           kd_ang=self.kd_turn, yaw_rate=self.yaw_rate)
        # Keep re-centering until converged OR the attempt times out. The timeout matters:
        # diff-drive overshoot can make 1-axis centering oscillate and never report
        # "centered"; rather than deadlock, sense anyway -- projection-median sensing is
        # robust to ~0.45 m off-centre, so tight centering is not required to sense/plan.
        if not centered and (t - self.center_start) < self.center_timeout_s:
            self.settle_until = t + self.settle_s
            return (v, w, False)
        # Snap yaw to the nearest cardinal before sensing. At all-open junctions there is no
        # wall for centering_command to face, so the robot would otherwise sense at its
        # (off-cardinal) arrival yaw -- the Gazebo mis-sense source. PD-damped like the other
        # rotate-in-place; bounded by the same center timeout (sense anyway if it can't settle).
        snapped = round(yaw / (math.pi / 2.0)) * (math.pi / 2.0)
        yaw_err = _norm(snapped - yaw)
        if abs(yaw_err) > self.yaw_tol_rad and (t - self.center_start) < self.center_timeout_s:
            w = self.kp_turn * yaw_err - self.kd_turn * self.yaw_rate
            self.settle_until = t + self.settle_s
            return (0.0, max(-self.turn_w_max, min(self.turn_w_max, w)), False)
        if t < self.settle_until:
            return (0.0, 0.0, False)
        # Re-anchor the discrete cell to the odom-derived cell -- but only as a BOUNDED
        # correction (within 1 cell of the dead-reckoned cell). This corrects the slow
        # 1-cell dead-reckoning desync seen in the tight Gazebo interior (where odom is
        # accurate, drift ~0.25 m) without letting a large odom excursion hijack tracking
        # (so it stays drift-immune like pure dead-reckoning under extreme odom drift).
        anchored = pose_to_cell(x, y)
        if in_grid(anchored) and abs(anchored[0] - self.cell[0]) + abs(anchored[1] - self.cell[1]) <= 1:
            self.cell = anchored
        if self.cell == EXIT_CELL:
            self.phase = 'done'
            return (0.0, 0.0, True)
        if self.cell not in self.sensed:
            walls = sense_cell_walls(ranges, amin, ainc, yaw)
            for d, is_wall in walls.items():
                self.brain.mark(self.cell, d, is_wall)
            self.sensed.add(self.cell)
            self.dbg = {'cell': self.cell, 'pose': (round(x, 2), round(y, 2), round(yaw, 2)),
                        'off': (None if ox is None else round(ox, 2),
                                None if oy is None else round(oy, 2)),
                        'centered': centered, 'walls': walls}
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
        self.turn_start = t
        self.center_start = None
        self.phase = 'turn'
        return (0.0, 0.0, False)

    def _turn(self, pose, t):
        yaw = pose[2]
        err = _norm(self.target_cardinal - yaw)
        settled = abs(err) <= self.yaw_tol_rad
        self.turn_in_tol = self.turn_in_tol + 1 if settled else 0
        # Proceed to drive when the turn has settled, OR when it times out: diff-drive
        # overshoot can make the in-place turn oscillate around the cardinal and never reach
        # the settle count; the corridor drive holds heading, so starting slightly off is fine.
        timed_out = self.turn_start is not None and (t - self.turn_start) >= self.turn_timeout_s
        if (settled and self.turn_in_tol >= self.turn_settle_ticks) or timed_out:
            self.hop_start = (pose[0], pose[1])           # measure the drive from here
            self.hop_deadline = t + self.hop_timeout_s
            self.turn_start = None
            self.phase = 'drive'
            return (0.0, 0.0, False)
        w = self.kp_turn * err - self.kd_turn * self.yaw_rate        # PD: damp latency overshoot
        return (0.0, max(-self.turn_w_max, min(self.turn_w_max, w)), False)

    def _drive(self, pose, scan, t):
        x, y, yaw = pose
        ranges, amin, ainc = scan
        moved = math.hypot(x - self.hop_start[0], y - self.hop_start[1])
        if moved >= CELL_SIZE_M - self.hop_arrive_slack_m:           # arrived
            self.cell = self.hop_target
            self.hop_attempts.clear()                                # fresh cell -> reset attempts
            self.settle_until = t + self.settle_s
            self.center_start = None
            self.phase = 'center'
            return (0.0, 0.0, False)
        perp = cell_wall_perp_dist(ranges, amin, ainc, yaw)
        dirn = _dir_name(self.hop_dir)
        front_blocked = perp[dirn] < self.front_block_m and moved > 0.3
        if front_blocked or t >= self.hop_deadline:
            # A hop toward a sensed-OPEN edge failed (a front wall appeared, or it stalled).
            # Do NOT trust this as a wall: walls come from SENSING (robust projection-median),
            # NOT from locomotion failures -- those were silently marking false walls and
            # corrupting the map -> next_cell None -> stuck (the ~13-14 cell plateau). Treat it
            # as transient: count the attempt, force a fresh cardinal-aligned RE-SENSE of this
            # cell (so the re-read, not the failure, decides if the edge is a wall), and re-plan.
            # Mark a wall only as a last resort after max_hop_attempts genuine retries.
            key = (self.cell, dirn)
            self.hop_attempts[key] = self.hop_attempts.get(key, 0) + 1
            if self.hop_attempts[key] >= self.max_hop_attempts:
                self.brain.mark(self.cell, dirn, is_wall=True)
                self.hop_attempts.pop(key, None)
            else:
                self.sensed.discard(self.cell)                       # re-sense from a clean position
            self.settle_until = t + self.settle_s
            self.center_start = None
            self.phase = 'center'
            return (0.0, 0.0, False)
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
