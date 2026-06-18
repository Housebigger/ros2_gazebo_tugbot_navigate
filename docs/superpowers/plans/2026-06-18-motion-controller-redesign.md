# Motion-Controller Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the flood-fill solver's fragile motion layer with a pure, offline-testable `MazeMotion` controller (corridor pure-pursuit drive + rotate-in-place turn + 1-axis re-anchoring) that drives the full maze to the exit without wedging, turn-stalling, or `dcell` desync.

**Architecture:** Extract the phase FSM + controllers + brain into a ROS-free `MazeMotion` class driven end-to-end through `maze_sim` (inertia + collision + odom drift) for validation; `flood_fill_solver.py` becomes a thin ROS adapter. Spec: `docs/superpowers/specs/2026-06-18-motion-controller-redesign.md`.

**Tech Stack:** Python 3.12, ROS 2 Jazzy (rclpy), pytest, numpy; `tugbot_maze` package in `ros2_ws_tugbot_nav_20260614`.

All paths below are under `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/`. Run tests from that directory: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest <file> -q`. Commits go on branch `flood-fill-maze-solver` (never pushed); end each commit message with the `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>` trailer.

---

### Task 1: `corridor_drive_command` (corridor pure-pursuit drive)

**Files:**
- Modify: `tugbot_maze/hop_controller.py`
- Test: `test/test_hop_controller.py`

- [ ] **Step 1: Write the failing tests**

Append to `test/test_hop_controller.py`:

```python
from tugbot_maze.hop_controller import corridor_drive_command


def test_corridor_drive_steers_toward_centerline():
    # facing the N cardinal, 0.3 m LEFT of centre -> steer RIGHT (negative w), still driving
    v, w = corridor_drive_command(math.pi / 2, math.pi / 2, 0.3)
    assert w < 0.0 and v > 0.0


def test_corridor_drive_goes_straight_on_centerline():
    v, w = corridor_drive_command(math.pi / 2, math.pi / 2, 0.0)
    assert v > 0.0 and abs(w) < 1e-9


def test_corridor_drive_turns_in_place_when_badly_misaligned():
    # heading 90 deg off the cardinal -> v throttled to ~0 (turn first), w turns toward cardinal
    v, w = corridor_drive_command(0.0, math.pi / 2, 0.0)
    assert v < 0.05 and w > 0.0


def test_corridor_drive_slows_near_wall_but_never_zero():
    # aligned & centred but a side wall at the stop distance -> v floored, not zero (no freeze)
    v, w = corridor_drive_command(math.pi / 2, math.pi / 2, 0.0, near_wall_m=0.40)
    assert 0.0 < v <= 0.12        # ~wedge_v_floor, never 0
    v2, _ = corridor_drive_command(math.pi / 2, math.pi / 2, 0.0, near_wall_m=0.30)  # past stop
    assert 0.0 < v2 <= 0.12


def test_corridor_drive_full_speed_when_far_from_walls():
    v, _ = corridor_drive_command(math.pi / 2, math.pi / 2, 0.0, near_wall_m=1.5)
    assert v > 0.25               # ~cruise


def test_corridor_drive_within_envelope():
    v, w = corridor_drive_command(0.0, math.pi / 2, 2.0, near_wall_m=0.35)  # extreme inputs
    assert -0.5 <= v <= 0.5 and -0.5 <= w <= 0.5
```

- [ ] **Step 2: Run to verify they fail**

Run: `python3 -m pytest test/test_hop_controller.py -q`
Expected: FAIL — `ImportError: cannot import name 'corridor_drive_command'`.

- [ ] **Step 3: Implement**

Add to `tugbot_maze/hop_controller.py` (after `hop_drive_command`):

```python
def corridor_drive_command(yaw: float, cardinal_yaw: float, cross_track: float,
                           near_wall_m: Optional[float] = None, *, v_max: float = 0.3,
                           w_max: float = 0.5, kp_ang: float = 1.5, lookahead_m: float = 0.7,
                           slow_angle: float = 0.6, wedge_slow_m: float = 0.50,
                           wedge_stop_m: float = 0.40, wedge_v_floor: float = 0.10
                           ) -> Tuple[float, float]:
    """Drive forward along `cardinal_yaw` while converging onto the corridor centerline.

    cross_track = signed lateral offset (+ = robot LEFT of centre). The robot aims at the
    centerline point `lookahead_m` ahead, so it CURVES back toward centre as it nears a side
    wall (inherently anti-wedge). Forward speed is throttled to ~0 only by a large HEADING
    error (turn first). If `near_wall_m` (min perpendicular distance to the two side walls)
    is given, speed is additionally slowed toward `wedge_v_floor` as the nearer wall
    approaches `wedge_stop_m` -- a NEVER-zero floor (the heading term can still zero v), so
    the robot keeps creeping while steering away rather than freezing or wedging."""
    setpoint = cardinal_yaw + math.atan2(-cross_track, lookahead_m)
    err = _norm(setpoint - yaw)
    w = max(-w_max, min(w_max, kp_ang * err))
    throttle = max(0.0, 1.0 - abs(err) / slow_angle)              # heading: -> 0 when misaligned
    if near_wall_m is not None:                                  # wedge: slow near a wall, never 0
        wedge = max(0.0, min(1.0, (near_wall_m - wedge_stop_m) / (wedge_slow_m - wedge_stop_m)))
        throttle *= max(wedge_v_floor / v_max, wedge)
    v = max(0.0, min(v_max, v_max * throttle))
    return (v, w)
```

- [ ] **Step 4: Run to verify they pass**

Run: `python3 -m pytest test/test_hop_controller.py -q`
Expected: PASS (all hop_controller tests).

- [ ] **Step 5: Commit**

```bash
git add tugbot_maze/hop_controller.py test/test_hop_controller.py
git commit -m "feat: corridor_drive_command (pure-pursuit centerline drive + anti-wedge floor)"
```

---

### Task 2: `MazeMotion` — pure phase FSM + controllers + brain

**Files:**
- Create: `tugbot_maze/maze_motion.py`
- Test: `test/test_maze_motion.py`

- [ ] **Step 1: Write the failing tests** (use `maze_sim` raycaster to make realistic scans)

Create `test/test_maze_motion.py`:

```python
import math
from tugbot_maze.maze_motion import MazeMotion
from tugbot_maze.maze_sim import MazeSim, load_segments
from tugbot_maze.flood_fill_brain import ENTRANCE_CELL, EXIT_CELL, cell_center


def _scan_at(sim):
    return sim.scan(n_beams=360, fov_rad=2 * math.pi)


def test_starts_in_center_at_entrance():
    m = MazeMotion()
    assert m.cell == ENTRANCE_CELL and m.phase == 'center'


def test_center_senses_then_turns_toward_a_chosen_neighbor():
    # At the entrance cell centre, MazeMotion should sense, pick next_cell, and enter 'turn'.
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0)
    m = MazeMotion()
    t = 0.0
    for _ in range(60):                       # a few ticks to settle + sense + plan
        v, w, done = m.step(sim.pose, _scan_at(sim), t)
        t += 0.1
        if m.phase in ('turn', 'drive'):
            break
    assert m.phase in ('turn', 'drive')
    assert m.hop_target is not None and m.hop_dir is not None


def test_turn_rotates_then_settles_to_drive():
    # Force a turn target 90 deg from current heading; stepping with a rotating yaw must
    # eventually settle into 'drive'.
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0)
    m = MazeMotion()
    m.phase = 'turn'
    m.hop_dir = (0, 1)
    m.target_cardinal = math.pi / 2
    m.hop_start = cell_center(ENTRANCE_CELL)
    yaw = 0.0
    t = 0.0
    for _ in range(200):
        v, w, _ = m.step((0.0, 0.0, yaw), _scan_at(sim), t)
        assert abs(v) < 1e-9                  # no translation during a turn
        yaw = math.atan2(math.sin(yaw + w * 0.1), math.cos(yaw + w * 0.1))
        t += 0.1
        if m.phase == 'drive':
            break
    assert m.phase == 'drive'
    assert abs(math.atan2(math.sin(math.pi / 2 - yaw), math.cos(math.pi / 2 - yaw))) <= 0.11


def test_done_when_cell_is_exit():
    m = MazeMotion()
    m.cell = EXIT_CELL
    sim = MazeSim(load_segments(), cell_center(EXIT_CELL), 0.0)
    v, w, done = m.step(sim.pose, _scan_at(sim), 0.0)
    assert done is True and (v, w) == (0.0, 0.0)
```

- [ ] **Step 2: Run to verify they fail**

Run: `python3 -m pytest test/test_maze_motion.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'tugbot_maze.maze_motion'`.

- [ ] **Step 3: Implement `tugbot_maze/maze_motion.py`**

```python
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
```

- [ ] **Step 4: Run to verify they pass**

Run: `python3 -m pytest test/test_maze_motion.py -q`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add tugbot_maze/maze_motion.py test/test_maze_motion.py
git commit -m "feat: pure MazeMotion phase FSM (center/turn/drive) + brain"
```

---

### Task 3: End-to-end offline validation through the inertia+collision sim (+ gain tuning)

**Files:**
- Test: `test/test_maze_motion_sim.py`
- Modify (tuning only, if needed): `tugbot_maze/maze_motion.py`, `tugbot_maze/hop_controller.py`

- [ ] **Step 1: Write the end-to-end test**

Create `test/test_maze_motion_sim.py`:

```python
"""End-to-end validation: drive MazeMotion through the inertia+collision maze_sim to the exit.

This is the strong, fast (offline) validator the motion layer needs -- it exercises the real
diff-drive accel limits, the LIDAR raycaster, COLLISIONS, and injectable odom drift, so gains
can be tuned without Gazebo. Asserts: reaches the exit cell; never collides (anti-wedge); and
the discrete cell tracker stays synced to the physical cell.
"""
import math
import pytest
from tugbot_maze.maze_motion import MazeMotion
from tugbot_maze.maze_sim import MazeSim, load_segments
from tugbot_maze.flood_fill_brain import (
    ENTRANCE_CELL, EXIT_CELL, cell_center, pose_to_cell)


def _run(drift, dt=0.1, max_steps=30000):
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0,
                  inertia=True, odom_drift_per_m=drift)
    m = MazeMotion()
    t = 0.0
    collided = False
    max_desync = 0
    for _ in range(max_steps):
        scan = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        v, w, done = m.step(sim.reported_pose, scan, t)   # navigate on (drifting) reported pose
        if done:
            return True, collided, max_desync
        sim.step(v, w, dt)
        if sim.collides(sim.x, sim.y):                    # body entered a wall margin (true pose)
            collided = True
        if m.phase == 'center':                           # tracker-vs-physical sync at rest
            tc = pose_to_cell(sim.x, sim.y)
            max_desync = max(max_desync, abs(tc[0] - m.cell[0]) + abs(tc[1] - m.cell[1]))
        t += dt
    return (m.cell == EXIT_CELL), collided, max_desync


@pytest.mark.parametrize("drift", [0.0, 0.05, 0.10])
def test_reaches_exit_without_collision_or_desync(drift):
    reached, collided, max_desync = _run(drift)
    assert reached, f"did not reach the exit cell (drift={drift})"
    assert not collided, f"robot body collided with a wall (drift={drift})"
    assert max_desync <= 1, f"dcell desynced from the physical cell by {max_desync} (drift={drift})"
```

- [ ] **Step 2: Run it**

Run: `python3 -m pytest test/test_maze_motion_sim.py -q`
Expected: initially may FAIL (not reaching exit, a collision, or desync) — this is the tuning loop.

- [ ] **Step 3: Tune until green**

Iterate on the `MazeMotion` / `corridor_drive_command` gains (`lookahead_m`, `kp_ang`, `kp_turn`, `turn_settle_ticks`, `center_tol_m`, `hop_arrive_slack_m`, `wedge_slow_m`/`wedge_stop_m`/`wedge_v_floor`, `front_block_m`) until all three drift cases pass: reaches exit, zero collisions, `max_desync ≤ 1`. Re-run after each change. Keep changes minimal and record any non-obvious gain choice in a code comment.

- [ ] **Step 4: Confirm the rest of the offline suite is still green**

Run: `python3 -m pytest test/test_hop_controller.py test/test_maze_motion.py test/test_cell_walls.py test/test_wall_localize.py test/test_wall_relocalize_sim.py test/test_maze_sim.py test/test_flood_fill_brain.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add test/test_maze_motion_sim.py tugbot_maze/maze_motion.py tugbot_maze/hop_controller.py
git commit -m "test: end-to-end MazeMotion drive through inertia+collision sim (reaches exit, no collision, dcell synced under drift); tune gains"
```

---

### Task 4: Refactor `flood_fill_solver.py` into a thin adapter

**Files:**
- Modify: `tugbot_maze/flood_fill_solver.py`
- Test: `test/test_flood_fill_solver_smoke.py`

- [ ] **Step 1: Confirm the smoke test currently passes (baseline)**

Run: `python3 -m pytest test/test_flood_fill_solver_smoke.py -q`
Expected: PASS.

- [ ] **Step 2: Refactor the node to delegate control to `MazeMotion`**

Edit `tugbot_maze/flood_fill_solver.py`: keep the ROS plumbing (params, `/scan` sub, `/cmd_vel_nav` pub, TF, `_lookup_pose` with `odom_locked`, the `startup` and `entering` phases, the entry drive, and `sense_debug` logging). Replace the `center`/`hop`/`backup` control logic with a `MazeMotion` instance.

- In `__init__`, after params, construct it (forwarding the tunable params):
```python
from tugbot_maze.maze_motion import MazeMotion
...
self.motion = MazeMotion(
    self.brain, cruise_v=self.cruise_v, center_tol_m=self.center_tol_m,
    yaw_tol_rad=self.yaw_tol_rad, hop_arrive_slack_m=self.hop_arrive_slack_m,
    front_block_m=self.front_block_m, hop_timeout_s=self.hop_timeout_s)
```
- Keep `self.phase` for `startup`/`entering`/`done`. When `entering` completes, set `self.motion.cell = ENTRANCE_CELL` and `self.phase = 'driving'`.
- In `_control_tick`, the `'driving'` branch delegates:
```python
if self.phase == 'driving':
    if self.scan_msg is None:
        return
    s = self.scan_msg
    v, w, done = self.motion.step(
        pose, (s.ranges, s.angle_min, s.angle_increment), t)
    self._publish_cmd(v, w)
    if done:
        self.phase = 'done'
        self.get_logger().info('EXIT_REACHED (flood_fill_solver)')
        self.goal_events_pub.publish(String(data='EXIT_REACHED'))
    return
```
- `_diag_tick` logs `self.motion.cell` and `self.motion.phase`. Remove now-dead helpers/state that only the old `center`/`hop` logic used (e.g. `hop_target`/`hop_dir`/`hop_start`/`stall_*` on the node, `_dir_name` if unused). Keep `_sense`'s `sense_debug` block if you still want per-cell logging — but sensing now happens inside `MazeMotion`, so either drop `_sense` or keep only the debug logging path gated on `sense_debug`.

- [ ] **Step 3: Run the smoke test**

Run: `python3 -m pytest test/test_flood_fill_solver_smoke.py -q`
Expected: PASS (node constructs and ticks; no control logic regressions).

- [ ] **Step 4: Byte-compile the node**

Run: `python3 -m py_compile tugbot_maze/flood_fill_solver.py`
Expected: no output (success).

- [ ] **Step 5: Commit**

```bash
git add tugbot_maze/flood_fill_solver.py test/test_flood_fill_solver_smoke.py
git commit -m "refactor: flood_fill_solver is a thin adapter around MazeMotion"
```

---

### Task 5: Gazebo confirmation run (validation, not TDD)

**Files:** none (build + run).

- [ ] **Step 1: Build the package**

Run: `cd ros2_ws_tugbot_nav_20260614 && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select tugbot_maze`
Expected: `1 package finished`.

- [ ] **Step 2: Ensure a clean sim environment**

If a prior sim is running, request the user run `! pkill -9 -f "gz sim|ruby.*gz|parameter_bridge|flood_fill_solver|ros2 launch"` (agent pkill is hook-blocked). Confirm 0 leftover procs.

- [ ] **Step 3: Run and watch**

Run (background): `tools/run_flood_fill_maze.sh 1200 true false odom_locked true`
Watch the newest `log/flood_fill_run_*/launch.log` for the unique cells sensed, `HOP_BLOCKED`/`HOP_STALL`, and `EXIT_REACHED`. Compare against the offline prediction (should flow well past the run-4/5 plateau at ~7 cells).

- [ ] **Step 4: Assess + iterate or finish**

If it reaches `EXIT_REACHED`: proceed to finish the branch (superpowers:finishing-a-development-branch). If it wedges/stalls/desyncs at a new spot, capture the DIAG trajectory + SCANDUMP, reproduce/extend the offline sim test to cover it, fix, and re-run. Record findings in the `flood-fill-maze-solver` memory.

---

## Self-review notes

- **Spec coverage:** Task 1 = corridor drive; Task 2 = MazeMotion FSM (center/turn/drive) + along-track re-anchor (via `centering_command`) + stall handling; Task 3 = end-to-end offline validation (reaches-exit / no-collision / dcell-synced under drift) + tuning; Task 4 = thin-adapter node; Task 5 = Gazebo. All spec components covered.
- **Type consistency:** `MazeMotion.step(pose, scan, t)` and `(v, w, done)` used consistently in tests, the class, and the node; `corridor_drive_command(yaw, cardinal_yaw, cross_track, near_wall_m=None, …)` matches its tests and the `_drive` call site; `cross_track_offset(ox, oy, hop_dir)` and `centering_command(pose, ox, oy, …)` match their existing signatures.
- **No placeholders:** every code step shows complete code; Task 3 Step 3 is an explicit tuning loop, not a placeholder (the acceptance is the green end-to-end test).
