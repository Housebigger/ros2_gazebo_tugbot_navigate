# Wall-Referenced Re-Localization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the flood-fill maze solver drift-immune — track the cell index discretely and re-center against the corridor walls each cell — so it reaches the exit despite the confirmed ~0.7 m wheel-odometry drift.

**Architecture:** A new ROS-free `wall_localize` module computes the robot's offset from the true cell center from the live LIDAR. The solver tracks `self.cell` discretely (advanced by committed hops, not `pose_to_cell(odom)`), re-centers physically against the walls before sensing (drift-immune relative moves + cardinal heading snap), drives hops as relative ~2 m forward moves, and checks the exit discretely. An offline drift-injection harness in `maze_sim` validates the logic before any Gazebo run.

**Tech Stack:** ROS 2 Jazzy (rclpy), Python 3.12, pytest. Gazebo Harmonic for runtime validation. All new logic is ROS-free except the solver node.

**Spec:** `docs/superpowers/specs/2026-06-18-wall-referenced-relocalization-design.md`

---

## File Structure

- Create: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/wall_localize.py` — offset-from-center + heading snap (ROS-free).
- Create: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_wall_localize.py` — unit tests.
- Modify: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_sim.py` — add odom-drift injection (true vs reported pose).
- Create: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_wall_relocalize_sim.py` — offline drift guarantee.
- Modify: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/flood_fill_solver.py` — discrete cell tracking, wall-referenced `center` phase, relative hop, discrete exit check.

All paths below are relative to `ros2_ws_tugbot_nav_20260614/`. Run pytest from `src/tugbot_maze/`. Build from the workspace root with `source /opt/ros/jazzy/setup.bash && colcon build --packages-select tugbot_maze --symlink-install`.

---

### Task 1: `wall_localize` — offset-from-center + heading snap

**Files:**
- Create: `src/tugbot_maze/tugbot_maze/wall_localize.py`
- Test: `src/tugbot_maze/test/test_wall_localize.py`

- [ ] **Step 1: Write the failing test**

```python
# src/tugbot_maze/test/test_wall_localize.py
import math
from tugbot_maze.wall_localize import cell_center_offset, heading_snap, HALF_CORRIDOR_M

H = HALF_CORRIDOR_M  # 0.88


def _scan(dN, dE, dS, dW, n=360):
    """360-beam scan (angle_min=-pi, inc=2pi/n); one beam set per map cardinal, rest far."""
    amin, ainc = -math.pi, 2 * math.pi / n
    ranges = [12.0] * n
    for ang, d in [(0.0, dE), (math.pi / 2, dN), (math.pi, dW), (-math.pi / 2, dS)]:
        ranges[int(round((ang - amin) / ainc)) % n] = d
    return ranges, amin, ainc


def test_centered_gives_zero_offset():
    ox, oy = cell_center_offset(*_scan(H, H, H, H), yaw=0.0)
    assert math.isclose(ox, 0.0, abs_tol=1e-6)
    assert math.isclose(oy, 0.0, abs_tol=1e-6)


def test_offset_east_detected_from_both_walls():
    # robot 0.2 m east of center: closer to E wall, farther from W wall
    ox, oy = cell_center_offset(*_scan(H, H - 0.2, H, H + 0.2), yaw=0.0)
    assert math.isclose(ox, 0.2, abs_tol=1e-6)
    assert math.isclose(oy, 0.0, abs_tol=1e-6)


def test_offset_from_single_east_wall():
    # only E wall present (W open): off_x = H - dE
    ox, oy = cell_center_offset(*_scan(12.0, H - 0.2, 12.0, 12.0), yaw=0.0)
    assert math.isclose(ox, 0.2, abs_tol=1e-6)
    assert oy is None        # N and S open -> no y reference


def test_open_axis_is_none():
    # N-S corridor: N,S open; E,W walls -> x correctable, y open
    ox, oy = cell_center_offset(*_scan(12.0, H, 12.0, H), yaw=0.0)
    assert math.isclose(ox, 0.0, abs_tol=1e-6)
    assert oy is None


def test_offset_respects_yaw():
    # facing north (yaw=pi/2): map-East wall is on the robot's right; still resolves map x.
    ox, oy = cell_center_offset(*_scan(H, H - 0.2, H, H + 0.2), yaw=math.pi / 2)
    assert math.isclose(ox, 0.2, abs_tol=1e-6)


def test_heading_snap():
    s, d = heading_snap(0.1)
    assert math.isclose(s, 0.0, abs_tol=1e-9) and math.isclose(d, -0.1, abs_tol=1e-9)
    s, d = heading_snap(1.5)
    assert math.isclose(s, math.pi / 2, abs_tol=1e-9) and math.isclose(d, math.pi / 2 - 1.5, abs_tol=1e-9)
    s, d = heading_snap(-3.10)
    assert math.isclose(abs(s), math.pi, abs_tol=1e-9)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/tugbot_maze && python3 -m pytest test/test_wall_localize.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'tugbot_maze.wall_localize'`

- [ ] **Step 3: Write minimal implementation**

```python
# src/tugbot_maze/tugbot_maze/wall_localize.py
"""ROS-free wall-referenced localization for the maze cell grid.

`cell_center_offset` estimates the robot's offset from the current cell's true center
(map axes) from the min LIDAR range to each cardinal wall; `heading_snap` returns the
nearest cardinal. Used to re-center the robot against the physical walls each cell,
making the solver immune to wheel-odometry drift. Deterministic; no ROS/time/I/O.
"""
from __future__ import annotations
import math
from typing import Optional, Tuple

from tugbot_maze.cell_walls import cell_wall_min_ranges

HALF_CORRIDOR_M = 0.88   # cell half-width (1.0) minus wall half-thickness (0.12)
WALL_DIST_M = 1.3        # a min range below this in a cardinal window => wall present


def cell_center_offset(ranges, angle_min, angle_inc, yaw, *,
                       half_corridor_m: float = HALF_CORRIDOR_M,
                       wall_dist_m: float = WALL_DIST_M
                       ) -> Tuple[Optional[float], Optional[float]]:
    """Robot position minus true cell center, MAP axes (+x=E, +y=N). A component is
    None if that axis is an open corridor (no wall to reference)."""
    r = cell_wall_min_ranges(ranges, angle_min, angle_inc, yaw)

    def axis(d_pos, d_neg):
        pos, neg = d_pos < wall_dist_m, d_neg < wall_dist_m
        if pos and neg:
            return (d_neg - d_pos) / 2.0
        if pos:
            return half_corridor_m - d_pos
        if neg:
            return d_neg - half_corridor_m
        return None

    return (axis(r['E'], r['W']), axis(r['N'], r['S']))


def heading_snap(yaw: float) -> Tuple[float, float]:
    """Nearest cardinal yaw and the signed, normalized rotation to reach it."""
    snapped = round(yaw / (math.pi / 2.0)) * (math.pi / 2.0)
    dyaw = math.atan2(math.sin(snapped - yaw), math.cos(snapped - yaw))
    return (math.atan2(math.sin(snapped), math.cos(snapped)), dyaw)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/tugbot_maze && python3 -m pytest test/test_wall_localize.py -q`
Expected: PASS (6 passed)

- [ ] **Step 5: Commit**

```bash
git -C <repo-root> add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/wall_localize.py \
                       ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_wall_localize.py
git -C <repo-root> commit -m "feat(flood-fill): wall_localize cell-center offset + heading snap

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: `maze_sim` odom-drift injection

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/maze_sim.py`
- Test: `src/tugbot_maze/test/test_maze_sim.py` (append)

**Context:** `MazeSim` already tracks true pose `self.x, self.y, self.yaw` and integrates motion in `step(v, w, dt)`. We add a separate *reported* (odom) pose that accumulates a configurable per-meter drift, so tests can reproduce the confirmed ~0.7 m drift. Sensing/raycasting continues to use the TRUE pose (the real lidar sees true geometry).

- [ ] **Step 1: Write the failing test** (append to `test/test_maze_sim.py`)

```python
def test_odom_drift_accumulates_with_distance():
    from tugbot_maze.maze_sim import MazeSim
    # No walls needed for the drift model; empty segment list.
    sim = MazeSim([], (0.0, 0.0), 0.0, odom_drift_per_m=0.1)   # 0.1 m drift per m driven
    # drive forward 1 m (v=0.5 for 2 s) with no inertia
    for _ in range(20):
        sim.step(0.5, 0.0, 0.1)
    tx, ty, _ = sim.pose
    ox, oy, _ = sim.reported_pose
    assert abs(tx - 1.0) < 0.05                 # true pose moved ~1 m
    assert abs((ox - tx)) > 0.05                # reported pose drifted from true
    assert abs((ox - tx) - 0.1 * tx) < 0.02     # ~0.1 m drift per m (along x here)


def test_reported_pose_equals_true_with_zero_drift():
    from tugbot_maze.maze_sim import MazeSim
    sim = MazeSim([], (0.0, 0.0), 0.0)          # default: no drift
    for _ in range(10):
        sim.step(0.5, 0.0, 0.1)
    assert sim.reported_pose == sim.pose
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/tugbot_maze && python3 -m pytest test/test_maze_sim.py -q -k drift`
Expected: FAIL — `MazeSim.__init__() got an unexpected keyword argument 'odom_drift_per_m'`

- [ ] **Step 3: Write minimal implementation** (in `maze_sim.py`)

In `MazeSim.__init__`, add the parameter and drift state (place beside the existing inertia fields):
```python
                 odom_drift_per_m=0.0,                 # add to the __init__ signature
```
```python
        self.odom_drift_per_m = odom_drift_per_m
        self._drift_x = 0.0                            # accumulated odom drift (map frame)
        self._drift_y = 0.0
        self._last_x, self._last_y = self.x, self.y
```

At the end of `step(...)`, after the true pose is updated, accumulate drift proportional to distance moved (bias along the direction of travel):
```python
        dx, dy = self.x - self._last_x, self.y - self._last_y
        dist = math.hypot(dx, dy)
        if dist > 1e-9 and self.odom_drift_per_m:
            self._drift_x += self.odom_drift_per_m * dx     # bias grows along travel
            self._drift_y += self.odom_drift_per_m * dy
        self._last_x, self._last_y = self.x, self.y
```

Add the reported-pose property (beside the existing `pose` property):
```python
    @property
    def reported_pose(self):
        """Odom pose = true pose + accumulated drift (what the solver would believe)."""
        return (self.x + self._drift_x, self.y + self._drift_y, self.yaw)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/tugbot_maze && python3 -m pytest test/test_maze_sim.py -q`
Expected: PASS (all existing + 2 new)

- [ ] **Step 5: Commit**

```bash
git -C <repo-root> add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_sim.py \
                       ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_sim.py
git -C <repo-root> commit -m "feat(maze-sim): inject odom drift (true vs reported pose)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: Offline wall-referenced re-localization guarantee

**Files:**
- Test: `src/tugbot_maze/test/test_wall_relocalize_sim.py` (create)

**Context:** Prove offline that discrete cell tracking + wall-referenced re-centering reaches the exit even when odom drifts. The harness mirrors the solver loop: it tracks the cell discretely, re-centers the TRUE pose using `wall_localize` on a true-geometry raycast, senses ground-truth walls, and hops ~2 m. Drift is injected; the re-centering must keep the true pose centered each cell.

- [ ] **Step 1: Write the failing test**

```python
# src/tugbot_maze/test/test_wall_relocalize_sim.py
import math
from tugbot_maze.maze_sim import MazeSim, load_segments
from tugbot_maze.flood_fill_brain import (
    FloodFillBrain, ENTRANCE_CELL, EXIT_CELL, cell_center)
from tugbot_maze.cell_walls import sense_cell_walls
from tugbot_maze.wall_localize import cell_center_offset


def _recenter(sim):
    """Null the wall-referenced offset by moving the TRUE pose (models the drift-immune
    relative re-center). Open axes (no wall reference) are left unchanged."""
    ranges, amin, ainc = sim.scan(n_beams=360, fov_rad=2 * math.pi)
    ox, oy = cell_center_offset(ranges, amin, ainc, sim.yaw)
    if ox is not None:
        sim.x -= ox
    if oy is not None:
        sim.y -= oy


def _sense(sim, brain, cell):
    ranges, amin, ainc = sim.scan(n_beams=360, fov_rad=2 * math.pi)
    for d, is_wall in sense_cell_walls(ranges, amin, ainc, sim.yaw).items():
        brain.mark(cell, d, is_wall)


def test_reaches_exit_under_injected_drift():
    # Drift 0.15 m/m matches the observed ~0.7 m by cell (1,1) (~5 m driven).
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0, odom_drift_per_m=0.15)
    brain = FloodFillBrain()
    cell = ENTRANCE_CELL
    for _hop in range(400):
        _recenter(sim)                 # corrects the walled axes from the TRUE raycast
        if cell == EXIT_CELL:
            return
        _sense(sim, brain, cell)       # senses from the (re-centered) TRUE pose
        nxt = brain.next_cell(cell)
        assert nxt is not None
        brain.mark_traversal(cell, nxt)
        # Hop: face the cardinal, drive until the REPORTED (drifted) odom shows ~2 m.
        # Drift makes the TRUE move differ, so the robot lands off the true next-center;
        # the next _recenter must recover the cross-corridor component. This faithfully
        # exercises drift (the true pose is NOT teleported to center).
        sim.yaw = math.atan2(nxt[1] - cell[1], nxt[0] - cell[0])
        rx0, ry0, _ = sim.reported_pose
        for _ in range(300):
            sim.step(0.5, 0.0, 0.1)
            rx, ry, _ = sim.reported_pose
            if math.hypot(rx - rx0, ry - ry0) >= 2.0:
                break
        cell = nxt
    assert cell == EXIT_CELL, "did not reach exit under injected drift"
```

> **Risk / contingency (longitudinal drift):** the perpendicular re-center cannot
> correct the *along-corridor* axis (no walls there); that residual is corrected only at
> the next cell with a perpendicular wall (a turn). If this test FAILS under drift 0.15
> because a long straight run accumulates enough longitudinal error to mis-sense, the fix
> needs **longitudinal correction** — drive the hop until the lidar shows entry into the
> next cell (a side-wall transition / forward-wall distance) instead of a fixed reported
> 2 m. Add that as a follow-up task only if the test reveals it; the twisty perfect maze
> may make turns frequent enough that the simple fix passes. Tune `odom_drift_per_m` to
> the observed ~0.14 if needed.

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/tugbot_maze && python3 -m pytest test/test_wall_relocalize_sim.py -q`
Expected: FAIL initially if `_recenter` logic or `cell_center_offset` interaction is off; iterate Step 3 until green. (If it passes immediately, that is acceptable — the components already exist from Tasks 1-2; the value is the regression guarantee.)

- [ ] **Step 3: Fix/iterate the harness** until it reaches the exit (the test *is* the implementation — no production code changes). If it fails due to longitudinal accumulation, apply the risk/contingency note above (longitudinal correction).

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/tugbot_maze && python3 -m pytest test/test_wall_relocalize_sim.py -q`
Expected: PASS (1 passed)

- [ ] **Step 5: Commit**

```bash
git -C <repo-root> add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_wall_relocalize_sim.py
git -C <repo-root> commit -m "test(flood-fill): offline wall-referenced re-localization reaches exit under drift

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 4: Solver — discrete cell tracking + wall-referenced center + relative hop + discrete exit

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/flood_fill_solver.py`

**Context:** The node currently derives the working cell from `pose_to_cell(odom)`, senses in a `center` phase that drives to the odom-grid center, hops to an absolute `cell_center(next)`, and checks the exit by absolute pose radius. Drift breaks all of these. Rework to discrete tracking + wall-referenced centering + relative hop + discrete exit. No unit test (thin ROS node); validated by Task 3 offline + Task 5 Gazebo.

- [ ] **Step 1: Import + state.** Add `from tugbot_maze.wall_localize import cell_center_offset, heading_snap`. In `__init__`, add `self.cell = ENTRANCE_CELL` and `self.hop_start = None`. Add params `self.center_tol_m` (keep existing 0.12) and `self.yaw_tol_rad = float(self.declare_parameter('yaw_tol_rad', 0.10).value)`.

- [ ] **Step 2: Entry → discrete init.** When `entering` completes, set `self.cell = ENTRANCE_CELL`, `self.phase = 'center'`, and seed `self.settle_until`.

- [ ] **Step 3: Wall-referenced `center` phase.** Replace the odom-grid centering with:

```python
        if self.phase == 'center':
            if self.scan_msg is None:
                return
            s = self.scan_msg
            ox, oy = cell_center_offset(s.ranges, s.angle_min, s.angle_increment, pose[2])
            snap_yaw, dyaw = heading_snap(pose[2])
            # rotate to the nearest cardinal first
            if abs(dyaw) > self.yaw_tol_rad and t < self.hop_deadline:
                self._publish_cmd(0.0, max(-0.5, min(0.5, 1.5 * dyaw)))
                self.settle_until = t + self.settle_s
                return
            # null the correctable offsets with a relative move (drift-immune)
            need = (ox is not None and abs(ox) > self.center_tol_m) or \
                   (oy is not None and abs(oy) > self.center_tol_m)
            if need and t < self.hop_deadline:
                tx = pose[0] - (ox or 0.0)
                ty = pose[1] - (oy or 0.0)
                v, w, _ = hop_command(pose, (tx, ty), arrive_m=0.06)
                self._publish_cmd(v, w)
                self.settle_until = t + self.settle_s
                return
            self._publish_cmd(0.0, 0.0)
            if t < self.settle_until:
                return
            if self.cell not in self.sensed:
                self._sense(self.cell, pose)        # NOTE: sense uses self.cell, not pose_to_cell
                self.sensed.add(self.cell)
            if self.cell == EXIT_CELL:
                return                              # exit handled at top of tick (discrete, below)
            nxt = self.brain.next_cell(self.cell)
            if nxt is None:
                self._publish_cmd(0.0, 0.0); return
            self.brain.mark_traversal(self.cell, nxt)
            self.next_cell = nxt
            self.hop_dir = (nxt[0] - self.cell[0], nxt[1] - self.cell[1])   # unit cardinal
            self.hop_start = (pose[0], pose[1])
            self.hop_deadline = t + self.hop_timeout_s
            self.phase = 'hop'
            return
```

- [ ] **Step 4: Relative `hop` phase.** Replace the absolute-target hop with a turn-to-cardinal + drive ~`CELL_SIZE_M` forward measured from `self.hop_start`:

```python
        if self.phase == 'hop':
            target_yaw = math.atan2(self.hop_dir[1], self.hop_dir[0])
            dyaw = math.atan2(math.sin(target_yaw - pose[2]), math.cos(target_yaw - pose[2]))
            if abs(dyaw) > self.yaw_tol_rad:
                self._publish_cmd(0.0, max(-0.5, min(0.5, 1.5 * dyaw))); return
            moved = math.hypot(pose[0] - self.hop_start[0], pose[1] - self.hop_start[1])
            if moved >= CELL_SIZE_M - 0.10:
                self.cell = self.next_cell          # DISCRETE advance
                self.phase = 'center'; self.settle_until = t + self.settle_s
                self.hop_deadline = t + self.hop_timeout_s; return
            if t >= self.hop_deadline:              # blocked -> back up, mark wall, re-plan
                self.brain.mark(self.cell, _dir_name(self.hop_dir), is_wall=True)
                self.phase = 'backup'; self.backup_until = t + self.backup_s
                self.goal_events_pub.publish(String(data='HOP_BACKUP')); return
            self._publish_cmd(self.cruise_v, 0.0)
            return
```
Add a small helper `_dir_name(d)` mapping `(1,0)->'E'`, `(-1,0)->'W'`, `(0,1)->'N'`, `(0,-1)->'S'` (module-level or via `DIRS`/`OPP`). Import `CELL_SIZE_M` from `flood_fill_brain`.

- [ ] **Step 5: Discrete exit check.** At the top of `_control_tick`, replace the absolute `exit_reached(...)` test with: `if self.phase != 'startup' and self.cell == EXIT_CELL:` → set done, publish `EXIT_REACHED`, stop. Keep `exit_reached` as a secondary OR-confirm only if desired.

- [ ] **Step 6: `backup` → re-center.** After backup, set `self.phase = 'center'` (re-center in the current cell and re-plan around the marked wall).

- [ ] **Step 7: Build + offline suite green.**

Run:
```bash
cd <repo-root>/ros2_ws_tugbot_nav_20260614
python3 -c "import ast; ast.parse(open('src/tugbot_maze/tugbot_maze/flood_fill_solver.py').read()); print('OK')"
source /opt/ros/jazzy/setup.bash && colcon build --packages-select tugbot_maze --symlink-install
cd src/tugbot_maze && python3 -m pytest test/ -q
```
Expected: parse OK, build finished, all offline tests pass (the solver node isn't unit-tested; the brain/wall_localize/maze_sim/relocalize tests must stay green).

- [ ] **Step 8: Commit**

```bash
git -C <repo-root> add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/flood_fill_solver.py
git -C <repo-root> commit -m "feat(flood-fill): discrete cell tracking + wall-referenced re-centering (drift-immune)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 5: Gazebo runtime validation

**Files:** none (verification only).

- [ ] **Step 1: Run.** `tools/run_flood_fill_maze.sh 900 true false odom_locked true` (background).

- [ ] **Step 2: Confirm drift is now bounded.** During the run, read ground truth and compare to the solver's `DIAG` odom poses:
`gz topic -e -t /world/tugbot_maze_world_20260528_clean_scaled2x/dynamic_pose/info -n 30 | grep -A4 'name: "tugbot"'` → convert world→solver `(+11,+9)`; the gap to the re-centered position should stay small (≪ the old ~0.7 m) at each cell.

- [ ] **Step 3: Confirm progress.** `DIAG` should show the robot escaping the SW block (cells with cx≥4) and `dist_to_exit` descending; success = `EXIT_REACHED` in the log.

- [ ] **Step 4: If it still wedges/drifts**, capture `SENSE`/`SCANDUMP` + GT at the failure cell and iterate (likely tune `center_tol_m`, `yaw_tol_rad`, or the re-center P-gains). Record findings in `flood-fill-maze-solver` memory.

- [ ] **Step 5: On success**, update the `flood-fill-maze-solver` memory (drift fixed; autonomous interior 通关 achieved) and finish the branch per `superpowers:finishing-a-development-branch`.

---

## Notes for the implementer

- `<repo-root>` = `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`. Use `git -C "<repo-root>"` and absolute paths (cwd is not stable across calls). End commit messages with the `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>` line. **Merge locally only — never push.**
- The agent `pkill`/`kill` is hook-blocked; the run script self-cleans, but stray sims need `! pkill -9 -f "gz sim|..."` from the user.
- Keep `pose_source=odom_locked` (SLAM degrades). The brain, `cell_walls`, `hop_command`, entry drive, and the grid transform are verified correct — do not change them.
