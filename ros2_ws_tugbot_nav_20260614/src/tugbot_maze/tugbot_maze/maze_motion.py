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
    FloodFillBrain, ENTRANCE_CELL, EXIT_CELL, CELL_SIZE_M, pose_to_cell, in_grid, DIRS, OPP,
    open_exits)
from tugbot_maze.cell_walls import sense_cell_walls, cell_wall_perp_dist
from tugbot_maze.wall_localize import cell_center_offset
from tugbot_maze.hop_controller import (
    centering_command, grid_cross_track,
    side_distances, corridor_follow_command, profiled_turn_command, backout_command)


def _norm(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _dir_name(d) -> str:
    return {(1, 0): 'E', (-1, 0): 'W', (0, 1): 'N', (0, -1): 'S'}[d]


def within_commit_offset(ox, oy, tol: float) -> bool:
    """True if every REFERENCED axis offset (non-None) is within tol. Open axes (None) impose
    no constraint -- they cannot be wall-referenced."""
    return (ox is None or abs(ox) <= tol) and (oy is None or abs(oy) <= tol)


def within_cell_core(coord: float, cell_idx: int, margin: float) -> bool:
    """True if `coord` (map metres on one axis) is in the central core of the cell -- at least
    `margin` from either cell boundary. Rejects boundary-straddling poses (sensing) and gates
    hysteretic re-anchoring."""
    return abs(coord - CELL_SIZE_M * cell_idx) <= (CELL_SIZE_M / 2.0 - margin)


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
                 center_timeout_s: float = 4.0, turn_timeout_s: float = 5.0,
                 wedge_detect_s: float = 3.0, wedge_move_eps: float = 0.08,
                 recover_v: float = 0.15, recover_s: float = 1.8, recover_w: float = 0.0,
                 ang_decel: float = 1.2, wall_seen_m: float = 1.3, half_corridor_m: float = 0.88,
                 align_timeout_s: float = 6.0, commit_offset_tol_m: float = 0.40,
                 boundary_margin_m: float = 0.40, k_corroborate: int = 2,
                 backout_v: float = 0.30, backout_timeout_s: float = 12.0,
                 max_backout_attempts: int = 2,
                 grid_fallback_max_m: float = 0.40):
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
        self.wedge_detect_s = wedge_detect_s; self.wedge_move_eps = wedge_move_eps
        self.recover_v = recover_v; self.recover_s = recover_s; self.recover_w = recover_w
        self.ang_decel = ang_decel; self.wall_seen_m = wall_seen_m
        self.half_corridor_m = half_corridor_m
        self.align_timeout_s = align_timeout_s
        self.commit_offset_tol = commit_offset_tol_m
        self.boundary_margin_m = boundary_margin_m
        self.k_corroborate = k_corroborate
        self.backout_v = backout_v; self.backout_timeout_s = backout_timeout_s
        self.max_backout_attempts = max_backout_attempts
        self.grid_fallback_max_m = grid_fallback_max_m   # clamp on the open-junction odom fallback
        self.cell = ENTRANCE_CELL
        self.phase = 'center'
        self.sensed = set()
        self.committed = set()          # cells sensed from a good (gated) pose -- frozen, never re-sensed
        self.align_start = None         # independent timeout baseline for the cardinal-align sub-phase
        self.latched_cardinal = None    # cardinal latched once at align entry (no 45-deg basin jitter)
        self.corrob = {}                # cell -> (walls, agreeing-good-read count) for commit
        self.locomotion_walls = set()   # (cell,dir) WALLs stamped by hop-failure (re-openable by _unstick)
        self.reopened = set()           # (cell,dir) WALL edges re-opened by _unstick (monotonic bound)
        self.backout_target = None; self.backout_cardinal = 0.0
        self.backout_start = None; self.backout_deadline = 0.0
        self.backout_attempts = {}      # dead-end cell -> backout timeout count (bounds the loop)
        self.backout_count = 0          # observable: back-outs initiated (tests / regression)
        self.hop_dir = None; self.hop_target = None; self.hop_start = None
        self.target_cardinal = 0.0
        self.hop_deadline = 0.0; self.settle_until = 0.0; self.turn_in_tol = 0
        self.hop_attempts = {}          # (cell,dir) -> failed-hop count (transient, NOT walls)
        self.progress_pose = None; self.progress_t = None; self.recover_until = 0.0  # wedge recovery
        self.turn_start = None          # when the current turn episode began (for the timeout)
        self.center_start = None        # when the current center episode began (for the timeout)
        self.prev_yaw = None; self.prev_t = None; self.yaw_rate = 0.0   # for PD damping
        self.dbg = {}                   # last-tick diagnostics (offset, walls, near) for logging
        # --- no-progress watchdog + escalating escape + failed-hop deprioritizer ---
        self.last_seen_cell = ENTRANCE_CELL   # for cell-change detection (prev_cell)
        self.prev_cell = None                 # cell occupied just before the current one (reverse target)
        self.visited = {ENTRANCE_CELL}        # cells ever occupied (monotonic; seeds the entrance)
        self.explore_t = None                 # sim-time of last visited-growth (None = unseeded)
        self.recent = []                      # [(t, cell)] appended on cell-change; rolling confinement window
        self.escape_tier = 0                  # current escalation level (0 = none)
        self.escape_count = 0                 # MONOTONIC observable (calibration/regression gate)
        self._escape_backout = False          # tags an escape reverse (vs a dead-end back-out)
        self.no_progress_s = 90.0             # window (calibrated in Task 6)
        self.confine_k = 6                    # max distinct cells in-window to count as "confined" (Task 6)
        self.max_escape_tier = 2
        self.failed_hops = {}                 # (cell,dir) -> consecutive cross-occupation failed-hop count
        self.failed_hop_limit = 3
        self._latched = False                 # NH14 dead-maze permanent stop (set in _escape, Task 4)
        self.events = []                      # DIAG: structured STALL/ESCAPE/LATCH/UNSTICK strings (node drains)
        self._last_drive_v = 0.0              # DIAG: last corridor v (to spot wedge_stop, v~=0)
        self.wedge_realign_yaw = 0.5          # |yaw_err|>=this => follower turns in place (v~=0), not a pin

    def step(self, pose, scan, t) -> Tuple[float, float, bool]:
        yaw = pose[2]                                   # measured yaw rate (for PD damping)
        if self.prev_t is not None and t > self.prev_t:
            self.yaw_rate = _norm(yaw - self.prev_yaw) / (t - self.prev_t)
        self.prev_yaw = yaw; self.prev_t = t
        self._track_cell(t)
        if (self.phase != 'done' and self.cell != EXIT_CELL and not self._latched
                and not self._escape_backout              # don't re-fire mid-escape reverse
                and self.explore_t is not None
                and (t - self.explore_t) > self.no_progress_s
                and self._confined(t)):
            return self._escape(pose, t)
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
        if self.phase == 'recover':
            return self._recover(pose, t)
        if self.phase == 'backout':
            return self._backout(pose, t)
        return (0.0, 0.0, False)

    def _center(self, pose, scan, t):
        x, y, yaw = pose
        ranges, amin, ainc = scan
        # Fast-path: a committed cell is trusted -- skip centering/align/sensing; re-anchor
        # (hysteresis) and route. Revisits must not re-run the 4 s centering.
        if self.cell in self.committed:
            self._reanchor(x, y)
            if self.cell == EXIT_CELL:
                self.phase = 'done'
                return (0.0, 0.0, True)
            if self.cell in self.committed:
                return self._route(x, y, t)
            # else re-anchored onto a non-committed neighbour -> fall through and sense it
        # Lateral centering (unchanged primitive).
        if self.center_start is None:
            self.center_start = t
        ox, oy = cell_center_offset(ranges, amin, ainc, yaw)
        v, w, centered = centering_command((x, y, yaw), ox, oy,
                                           tol=self.center_tol_m, yaw_tol=self.yaw_tol_rad,
                                           w_max=self.turn_w_max, kp_ang=self.kp_turn,
                                           kd_ang=self.kd_turn, yaw_rate=self.yaw_rate)
        if not centered and (t - self.center_start) < self.center_timeout_s:
            self.settle_until = t + self.settle_s
            return (v, w, False)
        # Cardinal align as a POSE-FREEZE before sensing: latch the cardinal ONCE (no 45-deg basin
        # jitter), rotate in place via the decel-limited profile to within yaw_tol. Independent
        # align_start/align_timeout (NOT shared with center_start -- that sharing was the bug that
        # let a timed-out center sense off-cardinal).
        if self.align_start is None:
            self.align_start = t
            self.latched_cardinal = round(yaw / (math.pi / 2.0)) * (math.pi / 2.0)
        yaw_err = _norm(self.latched_cardinal - yaw)
        aligned = abs(yaw_err) <= self.yaw_tol_rad
        if not aligned and (t - self.align_start) < self.align_timeout_s:
            self.settle_until = t + self.settle_s
            w = profiled_turn_command(yaw, self.latched_cardinal, self.yaw_rate,
                                      ang_decel=self.ang_decel, turn_w_max=self.turn_w_max,
                                      kd=self.kd_turn)
            return (0.0, w, False)
        if t < self.settle_until:
            return (0.0, 0.0, False)
        # Re-anchor (bounded + hysteresis), then exit/committed checks.
        self._reanchor(x, y)
        if self.cell == EXIT_CELL:
            self.phase = 'done'
            return (0.0, 0.0, True)
        if self.cell in self.committed:
            return self._route(x, y, t)
        # Quality gate (BINDING = position), atomic with the sense below. The (3,4) flicker is
        # position-driven (boundary-straddle), not yaw -- so position is the commit criterion.
        ox, oy = cell_center_offset(ranges, amin, ainc, yaw)
        pos_ok = within_commit_offset(ox, oy, self.commit_offset_tol)
        taxis = 0 if (self.hop_dir and self.hop_dir[0] != 0) else 1
        coord = x if taxis == 0 else y
        not_straddling = within_cell_core(coord, self.cell[taxis], self.boundary_margin_m)
        good = aligned and pos_ok and not_straddling
        # Sense iff never sensed (first read, any quality) OR this is a GOOD read (improve/commit).
        # A POOR re-entry of an already-sensed cell does NOT re-sense -> bad poses cannot churn.
        if (self.cell not in self.committed) and (self.cell not in self.sensed or good):
            walls = sense_cell_walls(ranges, amin, ainc, yaw)
            for d, is_wall in walls.items():
                self.brain.mark(self.cell, d, is_wall)
                if not is_wall:
                    self.locomotion_walls.discard((self.cell, d))   # good re-sense clears a stale loco wall
            was_sensed = self.cell in self.sensed
            self.sensed.add(self.cell)
            if good:                                    # PER-CELL corroboration across visits
                prev = self.corrob.get(self.cell)
                count = prev[1] + 1 if (prev is not None and prev[0] == walls) else 1
                self.corrob[self.cell] = (walls, count)
                if count >= self.k_corroborate:
                    self.committed.add(self.cell)
            self.dbg = {'cell': self.cell, 'pose': (round(x, 2), round(y, 2), round(yaw, 2)),
                        'off': (None if ox is None else round(ox, 2),
                                None if oy is None else round(oy, 2)),
                        'good': good, 'yaw_err': round(yaw_err, 3),
                        'pos_ok': pos_ok, 'straddle': not not_straddling, 'walls': walls,
                        'committed': self.cell in self.committed, 'resensed': was_sensed,
                        'corrob': self.corrob.get(self.cell, (None, 0))[1]}
        return self._route(x, y, t)

    def _reanchor(self, x, y):
        """Bounded (1-cell) re-anchor of self.cell to the odom cell, with hysteresis: only flip
        across a boundary when the pose is clearly inside the new cell (within_cell_core), so a
        boundary-straddling pose can't make self.cell (and next_cell) flip."""
        anchored = pose_to_cell(x, y)
        if not in_grid(anchored):
            return
        if abs(anchored[0] - self.cell[0]) + abs(anchored[1] - self.cell[1]) != 1:
            return                                       # same cell (0) or too far (>1) -> keep
        ax = 0 if anchored[0] != self.cell[0] else 1
        coord = x if ax == 0 else y
        if within_cell_core(coord, anchored[ax], self.boundary_margin_m):
            self.cell = anchored

    def _track_cell(self, t):
        """Maintain the exploration-progress clock + confinement record. The seed and visited-update
        blocks run UNCONDITIONALLY (NOT gated on a cell change): at startup cell == last_seen_cell ==
        ENTRANCE_CELL, so a change-gated seed would never fire -> explore_t stays None -> the C2
        watchdog would be permanently disabled (F2 returns). Idempotent across the two calls
        (step() and the top of _route, which closes the in-tick _reanchor lag)."""
        if self.explore_t is None:                       # seed (absolute Gazebo clock ~1.78e9)
            self.explore_t = t
        if self.cell not in self.visited:                # NEW GROUND
            self.visited.add(self.cell)
            self.explore_t = t
            self.escape_tier = 0                         # real progress clears in-flight escalation
        if self.cell != self.last_seen_cell:             # cell-change bookkeeping
            self.prev_cell = self.last_seen_cell
            self.last_seen_cell = self.cell
            self.recent.append((t, self.cell))

    def _confined(self, t):
        """True iff the robot stayed within <= confine_k distinct cells over the last no_progress_s
        window. A tight ping-pong keeps a small rolling footprint (confined); a long legitimate
        backtrack sweeps many distinct cells (not confined -> the watchdog stays silent)."""
        self.recent = [(t2, c) for (t2, c) in self.recent if t2 >= t - self.no_progress_s]
        footprint = {c for (_, c) in self.recent}
        footprint.add(self.cell)
        return len(footprint) <= self.confine_k

    def _route(self, x, y, t):
        """Pick next_cell and set up the hop. Run the unstick backstop if boxed in (next_cell None)
        OR if the exit is unreachable from here -- a false WALL can cut the map while leaving an
        open neighbour, in which case next_cell still returns a non-None inf-distance cell and the
        robot would wander a disconnected pocket forever."""
        self._track_cell(t)                              # catch an in-tick _reanchor cell change
        nxt = self.brain.next_cell(self.cell)
        reachable = self.brain.flood().get(self.cell, math.inf) != math.inf
        if nxt is None or not reachable:
            return self._unstick(t)
        # Dead-end -> decisive straight back-out, but ONLY when the lone open exit is the edge we
        # drove in through (came_from). This excludes the entrance / out-of-grid-entered cells,
        # whose single in-grid open exit is the FORWARD route. Bounded so a pin can't livelock.
        came_from = OPP[_dir_name(self.hop_dir)] if self.hop_dir is not None else None
        open_dirs = open_exits(self.brain, self.cell)
        if (self.cell != ENTRANCE_CELL and came_from is not None and len(open_dirs) == 1
                and open_dirs[0] == came_from
                and self.backout_attempts.get(self.cell, 0) < self.max_backout_attempts):
            dx, dy = DIRS[came_from]
            self.backout_target = (self.cell[0] + dx, self.cell[1] + dy)
            # Trémaux: count the dead-end retreat edge exactly as a normal routed retreat would.
            # The forward entry already counted this edge once; bringing it to the <2 cap stops
            # next_cell from re-selecting the dead-end, so the back-out cannot livelock against the
            # proven routing (which, without this, would re-route into the only "legal" edge -- the
            # dead-end -- whenever the genuine exit-ward edge is already twice-traversed/capped).
            self.brain.mark_traversal(self.cell, self.backout_target)
            self.backout_cardinal = math.atan2(-dy, -dx)   # face INTO the dead-end; reverse to parent
            self.backout_start = (x, y)
            self.backout_deadline = t + self.backout_timeout_s
            self.backout_count += 1
            self.center_start = None
            self.phase = 'backout'
            return (0.0, 0.0, False)
        self.hop_target = nxt
        self.hop_dir = (nxt[0] - self.cell[0], nxt[1] - self.cell[1])
        self.target_cardinal = math.atan2(self.hop_dir[1], self.hop_dir[0])
        self.hop_start = (x, y)
        self.turn_in_tol = 0
        self.turn_start = t
        self.center_start = None
        self.align_start = None; self.latched_cardinal = None
        self.phase = 'turn'
        return (0.0, 0.0, False)

    def _reachable_component(self, start):
        """Cells reachable from `start` over non-WALL edges (OPEN or UNKNOWN are traversable; only
        WALL blocks). Used by _unstick to find the walls that cut the robot off from the exit."""
        seen = {start}; stack = [start]
        while stack:
            c = stack.pop()
            for d, (dx, dy) in DIRS.items():
                nb = (c[0] + dx, c[1] + dy)
                if in_grid(nb) and not self.brain.is_wall(c, d) and nb not in seen:
                    seen.add(nb); stack.append(nb)
        return seen

    def _stamp_loco_wall(self, cell, d):
        """Record a hop-failure WALL on edge (cell,d) for BOTH directed reps, so _unstick's tier-1
        (locomotion) check finds it regardless of which side it keys the cut from."""
        self.locomotion_walls.add((cell, d))
        nb = (cell[0] + DIRS[d][0], cell[1] + DIRS[d][1])
        if in_grid(nb):
            self.locomotion_walls.add((nb, OPP[d]))

    def _unstick(self, t):
        """Boxed (next_cell None) or exit-unreachable -> a false WALL cuts the robot's reachable
        component off from the exit. Re-open the CUT edges (walls from the component to a cell
        outside it) in ASCENDING trust -- locomotion (hop-failure) -> non-committed sensed (poor
        reads) -> committed (2x-corroborated, trusted last). Within a tier, prefer edges INCIDENT to
        self.cell (re-sensable on the very next center tick) so recovery stays inside the bounded
        loop. Re-open is OPTIMISTIC -> OPEN (flood routes through it); the cell is un-committed but
        KEPT in `sensed`, so the gate re-WALLs it only from a GOOD re-sense (never a poor pose --
        that poor re-WALL was the bug that could re-seal the false wall). Bounded by self.reopened
        (both reps): when no un-reopened cut edge remains, the map is disconnected -> 'stuck'."""
        R = self._reachable_component(self.cell)
        cut = [(c, d) for c in R for d, (dx, dy) in DIRS.items()
               if self.brain.is_wall(c, d) and (c, d) not in self.reopened
               and in_grid((c[0] + dx, c[1] + dy)) and (c[0] + dx, c[1] + dy) not in R]
        for pick in (lambda e: e in self.locomotion_walls,
                     lambda e: e not in self.locomotion_walls and e[0] not in self.committed,
                     lambda e: e[0] in self.committed):
            cand = [e for e in cut if pick(e)]
            if cand:
                incident = [e for e in cand if e[0] == self.cell]
                for (c, d) in (incident or cand):
                    nb = (c[0] + DIRS[d][0], c[1] + DIRS[d][1])
                    self.brain.mark(c, d, is_wall=False)     # re-open optimistically; a GOOD re-sense re-confirms
                    self.reopened.add((c, d)); self.reopened.add((nb, OPP[d]))      # both reps -> clean 1x bound
                    self.locomotion_walls.discard((c, d)); self.locomotion_walls.discard((nb, OPP[d]))
                    self.failed_hops.pop((c, d), None); self.failed_hops.pop((nb, OPP[d]), None)
                    self.committed.discard(c); self.corrob.pop(c, None)             # keep `sensed`: re-WALL needs good
                self.center_start = None
                self.align_start = None; self.latched_cardinal = None
                self.settle_until = t + self.settle_s
                self.escape_tier = 0                          # map mutation -> retry the cheap Tier-1 first
                self.events.append("UNSTICK reopen cell=%s n=%d" % (self.cell, len(incident or cand)))  # DIAG
                self.phase = 'center'                          # explicit: re-route / re-sense next tick
                return (0.0, 0.0, False)
        self.events.append("UNSTICK exhausted -> stuck cell=%s" % (self.cell,))  # DIAG
        self.phase = 'stuck'
        return (0.0, 0.0, False)

    def _maze_exhausted(self):
        """Latched terminal: exit truly unreachable AND every reachable cell already visited AND no
        un-reopened cut edge remains (nothing left to reopen). Avoids a benign forever-oscillation in
        an unsolvable/fully-explored maze. Does not arise in the real solvable maze."""
        if self.brain.flood().get(self.cell, math.inf) != math.inf:
            return False                                  # exit still reachable -> not exhausted
        R = self._reachable_component(self.cell)
        if any(c not in self.visited for c in R):
            return False                                  # reachable unexplored ground remains
        cut = [(c, d) for c in R for d, (dx, dy) in DIRS.items()
               if self.brain.is_wall(c, d) and (c, d) not in self.reopened
               and in_grid((c[0] + dx, c[1] + dy)) and (c[0] + dx, c[1] + dy) not in R]
        return not cut                                    # nothing left to reopen -> latch

    def _escape(self, pose, t):
        """No-progress escape (top-of-step watchdog fired: no new ground for no_progress_s while
        confined). Escalates: Tier 1 = decisive one-cell reverse to the adjacent known-open prev_cell
        (fresh re-approach clears a position-dependent false front_block); Tier 2 = ALSO give up the
        blocked forward edge (real brain wall, reopenable by _unstick) so flood/next_cell route to the
        nearest optimistic frontier (flood already IS 'retreat to nearest unexplored junction'). No
        valid reverse -> hand to _unstick AT MOST ONCE this tick (terminal; C2 revives next tick)."""
        x, y, yaw = pose
        if self._maze_exhausted():
            self.events.append("LATCH cell=%s -> permanent stuck (maze_exhausted)" % (self.cell,))  # DIAG
            self.phase = 'stuck'; self._latched = True
            return (0.0, 0.0, False)
        self.escape_count += 1
        self.escape_tier = min(self.escape_tier + 1, self.max_escape_tier)
        self.explore_t = t                                   # reset on EVERY entry (no busy-re-fire)
        pc = self.prev_cell
        man = None if pc is None else abs(pc[0] - self.cell[0]) + abs(pc[1] - self.cell[1])
        d_prev = (_dir_name((pc[0] - self.cell[0], pc[1] - self.cell[1]))
                  if (pc is not None and man == 1) else None)
        can_reverse = d_prev is not None and not self.brain.is_wall(self.cell, d_prev)
        gave_up = self.escape_tier >= 2 and self.hop_dir is not None
        if gave_up:                                              # Tier 2+: GIVE UP the blocked edge
            dirn = _dir_name(self.hop_dir)
            self.brain.mark(self.cell, dirn, is_wall=True)       # the real routing change (symmetric)
            self._stamp_loco_wall(self.cell, dirn)               # provenance: _unstick reopens loco first
            self.committed.discard(self.cell)
            self.failed_hops.pop((self.cell, dirn), None)
        self.events.append("ESCAPE tier=%d count=%d cell=%s prev=%s can_reverse=%s gave_up_edge=%s"  # DIAG
                           % (self.escape_tier, self.escape_count, self.cell, self.prev_cell,
                              can_reverse, gave_up))
        if can_reverse:                                          # Tier 1 & 2: one-cell reverse to prev
            dx, dy = DIRS[d_prev]
            self.backout_target = pc
            self.backout_cardinal = math.atan2(-dy, -dx)         # face away from prev -> reverse into it
            self.backout_start = (x, y)
            self.backout_deadline = t + self.backout_timeout_s
            self.center_start = None
            self._escape_backout = True
            self.phase = 'backout'
            return (0.0, 0.0, False)
        return self._unstick(t)                                  # no reverse -> _unstick once (MF5)

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
            self.progress_pose = (pose[0], pose[1]); self.progress_t = t   # wedge-detector baseline
            self.turn_start = None
            self.phase = 'drive'
            return (0.0, 0.0, False)
        w = profiled_turn_command(yaw, self.target_cardinal, self.yaw_rate,
                                  ang_decel=self.ang_decel, turn_w_max=self.turn_w_max,
                                  kd=self.kd_turn)        # decel profile: no latency overshoot
        return (0.0, w, False)

    def _stall_event(self, reason, perp, moved, marked, x, y, yaw):
        """DIAG: record WHY a drive hop stalled + the sensor state, to pinpoint the doorway pin
        mechanism (false front_block vs side-wall wedge_stop vs off-center vs real pin)."""
        if self.hop_dir and self.hop_dir[1] != 0:           # N/S travel -> side walls are E/W
            near = min(perp.get('E', -1.0), perp.get('W', -1.0))
        elif self.hop_dir:                                  # E/W travel -> side walls are N/S
            near = min(perp.get('N', -1.0), perp.get('S', -1.0))
        else:
            near = -1.0
        ct = (grid_cross_track(x, y, self.cell, self.hop_dir, cell_size_m=CELL_SIZE_M)
              if self.hop_dir else 0.0)
        dn = _dir_name(self.hop_dir) if self.hop_dir else '?'
        key = (self.cell, dn)
        self.events.append(
            "STALL reason=%s cell=%s dir=%s perp_front=%.2f near=%.2f cross_track=%.2f yaw_err=%.2f "
            "moved=%.2f last_v=%.2f hop_att=%d failed=%d marked=%s"
            % (reason, self.cell, dn, perp.get(dn, -1.0), near, ct,
               _norm(self.target_cardinal - yaw), moved, self._last_drive_v,
               self.hop_attempts.get(key, 0), self.failed_hops.get(key, 0), marked))

    def _drive(self, pose, scan, t):
        x, y, yaw = pose
        ranges, amin, ainc = scan
        moved = math.hypot(x - self.hop_start[0], y - self.hop_start[1])
        if moved >= CELL_SIZE_M - self.hop_arrive_slack_m:           # arrived
            self.brain.mark_traversal(self.cell, self.hop_target)    # count the COMPLETED traversal (was: at pick)
            _dirn = _dir_name(self.hop_dir)                          # clear failed_hops for THIS edge,
            self.failed_hops.pop((self.cell, _dirn), None)           #   keyed on the PRE-advance cell (MF6)
            self.failed_hops.pop((self.hop_target, OPP[_dirn]), None)
            self.cell = self.hop_target
            self.hop_attempts.clear()                                # fresh cell -> reset attempts
            self.settle_until = t + self.settle_s
            self.center_start = None
            self.align_start = None; self.latched_cardinal = None
            self.phase = 'center'
            return (0.0, 0.0, False)
        dirn = _dir_name(self.hop_dir)
        perp = cell_wall_perp_dist(ranges, amin, ainc, yaw)          # DIAG: computed early so stall events have it
        # Wedge detector: while driving, if the robot makes no real progress for wedge_detect_s
        # it is physically PINNED (the ~13-14 cell plateau: pose frozen, can't move in the tight
        # interior). Recover by reversing to free it, rather than grinding the hop timeout. Count
        # it as a hop attempt; after max_hop_attempts give up on this edge (mark wall, re-route).
        if math.hypot(x - self.progress_pose[0], y - self.progress_pose[1]) > self.wedge_move_eps:
            self.progress_pose = (x, y); self.progress_t = t
        elif abs(_norm(self.target_cardinal - yaw)) >= self.wedge_realign_yaw:  # heading far off cardinal =>
            self.progress_pose = (x, y); self.progress_t = t  # follower turns in place (v~=0), not a pin -> no false wedge
        elif (t - self.progress_t) > self.wedge_detect_s:
            key = (self.cell, dirn)
            self.hop_attempts[key] = self.hop_attempts.get(key, 0) + 1
            self.failed_hops[key] = self.failed_hops.get(key, 0) + 1   # cross-occupation accumulator
            marked = (self.hop_attempts[key] >= self.max_hop_attempts
                      or self.failed_hops[key] >= self.failed_hop_limit)  # per-dwell OR cross-visit
            self._stall_event('wedge', perp, moved, marked, x, y, yaw)    # DIAG
            self.center_start = None
            self.align_start = None; self.latched_cardinal = None
            self.settle_until = t + self.settle_s
            if marked:
                self.brain.mark(self.cell, dirn, is_wall=True)
                self._stamp_loco_wall(self.cell, dirn)               # re-openable by _unstick (both reps)
                self.committed.discard(self.cell)                    # un-commit; cell stays in `sensed`
                self.hop_attempts.pop(key, None)                     # so a poor re-entry won't re-sense
                self.failed_hops.pop(key, None)
                self.phase = 'center'                                # (and clobber) the WALL it just set
            else:
                self.recover_until = t + self.recover_s              # reverse to un-wedge
                self.phase = 'recover'
            return (0.0, 0.0, False)
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
            self.failed_hops[key] = self.failed_hops.get(key, 0) + 1   # cross-occupation accumulator
            marked = (self.hop_attempts[key] >= self.max_hop_attempts
                      or self.failed_hops[key] >= self.failed_hop_limit)
            self._stall_event('front_block' if front_blocked else 'deadline', perp, moved, marked, x, y, yaw)  # DIAG
            if marked:
                self.brain.mark(self.cell, dirn, is_wall=True)
                self._stamp_loco_wall(self.cell, dirn)               # re-openable by _unstick (both reps)
                self.committed.discard(self.cell)                    # un-commit; cell stays in `sensed`
                self.hop_attempts.pop(key, None)                     # so a poor re-entry won't re-sense
                self.failed_hops.pop(key, None)
            # non-max: just retry (re-center); re-sensing happens only from a GOOD re-entry, never
            # from this off-position post-failure pose (that off-pose re-sense was the (3,4) churn).
            self.settle_until = t + self.settle_s
            self.center_start = None
            self.align_start = None; self.latched_cardinal = None
            self.phase = 'center'
            return (0.0, 0.0, False)
        fallback = max(-self.grid_fallback_max_m,
                       min(self.grid_fallback_max_m,
                           grid_cross_track(x, y, self.cell, self.hop_dir, cell_size_m=CELL_SIZE_M)))  # odom grid centerline (valid in open junctions), clamped to the smooth creep-and-steer regime
        d_left, d_right = side_distances(perp, self.hop_dir)
        if self.hop_dir[1] != 0:                                   # N/S travel: sides are E/W
            near = min(perp['E'], perp['W'])
        else:                                                      # E/W travel: sides are N/S
            near = min(perp['N'], perp['S'])
        self.dbg['sides'] = (round(d_left, 2), round(d_right, 2))  # live centerline diagnostics
        self.dbg['near'] = round(near, 2)
        v, w = corridor_follow_command(yaw, self.target_cardinal, d_left, d_right, near,
                                       fallback_cross=fallback, wall_seen_m=self.wall_seen_m,
                                       half_corridor_m=self.half_corridor_m,
                                       max_cross_track_m=self.max_cross_track_m,
                                       v_max=self.cruise_v, w_max=self.w_max,
                                       lookahead_m=self.lookahead_m, wedge_slow_m=self.wedge_slow_m,
                                       wedge_stop_m=self.wedge_stop_m, wedge_v_floor=self.wedge_v_floor)
        self._last_drive_v = v                                       # DIAG: spot wedge_stop (v~=0)
        return (v, w, False)

    def _recover(self, pose, t):
        # Un-wedge: reverse straight back (recover_w=0 default -- the near-solve config; reverse
        # was enough for the breakthrough run's wedges, and adding rotation disoriented the robot
        # off-cardinal -> more wedges/churn). recover_w>0 adds a turn for corner/side-wall pins
        # but trades churn. Then re-center / re-sense / re-plan; hop_attempts bounds repeats.
        if t < self.recover_until:
            return (-self.recover_v, self.recover_w, False)
        # Do NOT discard `sensed`: re-sensing from this just-reversed (often still off-position)
        # pose is the (3,4) churn. The cell keeps its read; a GOOD re-entry re-senses it. hop_attempts
        # already bounds repeated wedge->recover->retry cycles (-> giveup marks the edge a WALL).
        self.center_start = None
        self.align_start = None; self.latched_cardinal = None
        self.phase = 'center'
        return (0.0, 0.0, False)

    def _backout(self, pose, t):
        """Decisive straight reverse out of a dead-end to the parent cell (one cell). Holds the
        cardinal and backs at backout_v -- no turn-in-place, no wedge_detect_s wait. On arrival
        (~one cell) advance self.cell to the parent and resume the FSM. On timeout WITHOUT arrival
        (a physical pin while reversing) count an attempt and resume center; after
        max_backout_attempts the _route gate stops re-arming back-out and falls through to the
        normal turn+drive toward the parent, where the wedge detector + hop-attempt cap escalate
        (mark wall -> _unstick) exactly as before -- so the timeout path always makes progress and
        can never deterministically re-enter back-out on the same pinned cell."""
        x, y, yaw = pose
        moved = math.hypot(x - self.backout_start[0], y - self.backout_start[1])
        arrived = moved >= CELL_SIZE_M - self.hop_arrive_slack_m
        if arrived or t >= self.backout_deadline:
            if arrived:
                self.cell = self.backout_target
            elif not self._escape_backout:                # an escape-reverse timeout must NOT charge
                self.backout_attempts[self.cell] = self.backout_attempts.get(self.cell, 0) + 1  # the dead-end budget
            self._escape_backout = False                  # clear on leaving backout (arrival or timeout)
            self.settle_until = t + self.settle_s
            self.center_start = None
            self.align_start = None; self.latched_cardinal = None
            self.phase = 'center'
            return (0.0, 0.0, False)
        v, w = backout_command(yaw, self.backout_cardinal, backout_v=self.backout_v,
                               w_max=self.w_max, kp_ang=self.kp_turn)
        return (v, w, False)
