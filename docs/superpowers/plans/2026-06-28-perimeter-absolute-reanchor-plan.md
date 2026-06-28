# Perimeter Absolute Re-anchor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Break the NE near-exit plateau by using the maze perimeter walls as drift-immune absolute references to correct centering offset + dcell in boundary cells. Spec: `docs/superpowers/specs/2026-06-28-perimeter-absolute-reanchor-design.md`.

**Architecture:** (A) `perimeter_offset` in `wall_localize.py` — pure function returning the absolute boundary-axis coordinate + implied cell from the perimeter wall. (B) integrate into `MazeMotion._center` — boundary-axis centering-offset override + unbounded dcell self-consistency correction. Branch `footprint-oracle-baseline` (continue; off `main`/`8661d4a` + the oracle baseline `d63c88d`).

**Tech Stack:** Python 3.12, ROS 2 Jazzy, pytest. Trailer ends `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`. **No backticks in `git commit -m`.** Diagnostics stay IN. Never push.

**Conventions:**
```bash
PKG=/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260614/src/tugbot_maze
# tests:  cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest <args>
```
Full regression (all tasks):
```bash
cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_footprint.py test/test_map_memory.py test/test_maze_motion.py \
  test/test_maze_motion_sim.py test/test_hop_controller.py test/test_flood_fill_brain.py \
  test/test_junction_log.py test/test_cell_walls.py test/test_maze_sim.py test/test_wall_localize.py -q
```
Baseline before this plan: **175 passed, 3 xfailed** (the 3 `[0.05-*]` rear-corner xfails stay xfailed throughout).

---

## Task 1: `perimeter_offset` (pure function, `wall_localize.py`)

**Files:** Modify `tugbot_maze/wall_localize.py`; Test `test/test_wall_localize.py`.

- [ ] **Step 1: READ** `wall_localize.py` — `cell_center_offset` (note exactly HOW it calls `cell_wall_perp_dist`: the arg list incl. any `half_window_rad`/`max_range`, since `perimeter_offset` must call it the same way) and the **sign convention** of the `(ox, oy)` it returns (robot-minus-center, or center-minus-robot — Task 2 must match it). Note the imports (`cell_wall_perp_dist`; `CELL_SIZE_M` from `flood_fill_brain` if not already imported).

- [ ] **Step 2: Write failing tests** — create `test/test_wall_localize.py` (or append if it exists):
```python
import math
from tugbot_maze.maze_sim import MazeSim, load_segments
from tugbot_maze.wall_localize import perimeter_offset


def _scan_at(x, y, yaw=0.0):
    sim = MazeSim(load_segments(), (x, y), yaw)
    ranges, amin, ainc = sim.scan(n_beams=360, fov_rad=2 * math.pi)
    return ranges, amin, ainc, yaw


def test_perimeter_offset_recovers_true_x_when_off_center_in_boundary_cell():
    # Cell (10,5) center=(20,10); robot 0.6 m off-center toward the interior (true x=19.4).
    # E perimeter wall surface ~20.89 -> perp_E ~1.49 m -> x_true ~19.4, implied cell index 10.
    r, amin, ainc, yaw = _scan_at(19.4, 10.0)
    out = perimeter_offset(r, amin, ainc, yaw, (10, 5))
    assert 0 in out                                   # x-axis (axis 0) returned
    true_x, implied = out[0]
    assert abs(true_x - 19.4) < 0.10                  # absolute true x, NOT the cell center (20) or a drifted belief
    assert implied == 10


def test_perimeter_offset_flags_cell_desync():
    # Robot physically in cell (9,5) at (18.6,10) but caller believes (10,5). Perimeter E wall is
    # ~2.29 m (< PERIM_MAX_M 2.5) -> x_true ~18.6 -> implied index 9 != believed 10 (desync signal).
    r, amin, ainc, yaw = _scan_at(18.6, 10.0)
    out = perimeter_offset(r, amin, ainc, yaw, (10, 5))
    assert 0 in out
    true_x, implied = out[0]
    assert abs(true_x - 18.6) < 0.12 and implied == 9


def test_perimeter_offset_abstains_when_far_from_perimeter():
    # Robot well inside (cell (9,5) center, ~2.89 m from the E wall > PERIM_MAX_M) believing (10,5):
    # the perimeter is out of trusted range -> abstain (never fire on a far reading).
    r, amin, ainc, yaw = _scan_at(18.0, 10.0)
    assert perimeter_offset(r, amin, ainc, yaw, (10, 5)) == {}


def test_perimeter_offset_empty_for_interior_cell():
    r, amin, ainc, yaw = _scan_at(10.0, 10.0)
    assert perimeter_offset(r, amin, ainc, yaw, (5, 5)) == {}   # not a boundary cell


def test_perimeter_offset_north_axis_in_top_row():
    # Cell (5,9) center=(10,18); N perimeter wall surface ~18.90; robot at true y=18.5 -> perp_N ~0.40
    # -> y_true ~18.5 (axis 1), implied index 9.
    r, amin, ainc, yaw = _scan_at(10.0, 18.5)
    out = perimeter_offset(r, amin, ainc, yaw, (5, 9))
    assert 1 in out
    true_y, implied = out[1]
    assert abs(true_y - 18.5) < 0.12 and implied == 9
```

- [ ] **Step 3: Run → FAIL** `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_wall_localize.py -q` (no `perimeter_offset`).

- [ ] **Step 4: Implement `perimeter_offset`** in `wall_localize.py` (adapt the `cell_wall_perp_dist` call to match `cell_center_offset`'s from Step 1):
```python
# perimeter wall NEAR-SURFACE coordinate (centerline -/+ wall_half_thickness 0.12), from load_segments.
_PERIM_SURF = {'E': 21.01 - 0.12, 'W': 1.02 + 0.12, 'N': 19.02 - 0.12, 'S': -0.97 + 0.12}
PERIM_MAX_M = 2.5            # trust the perimeter return only within this perp distance
_E_WALL_MAX_CY = 8          # E perimeter wall ends at y~17.1 -> cells cy<=8 see it; cy=9 uses N


def perimeter_walls_for(cell):
    """The perimeter cardinals this boundary cell faces (grid is cx 1..10, cy 0..9)."""
    cx, cy = cell
    walls = []
    if cx == 10 and cy <= _E_WALL_MAX_CY:
        walls.append('E')
    if cx == 1:
        walls.append('W')
    if cy == 9:
        walls.append('N')
    if cy == 0:
        walls.append('S')
    return walls


def perimeter_offset(ranges, angle_min, angle_inc, yaw, cell, *, max_range: float = 12.0):
    """For each perimeter `cell` faces, the ABSOLUTE true coordinate on that axis (from the perimeter
    wall, drift-immune) and the implied cell index. {} if no perimeter or the wall is out of range.
    axis 0 = x (E/W), axis 1 = y (N/S)."""
    walls = perimeter_walls_for(cell)
    if not walls:
        return {}
    perp = cell_wall_perp_dist(ranges, angle_min, angle_inc, yaw, max_range=max_range)  # match cell_center_offset's call
    out = {}
    for d in walls:
        dist = perp[d]
        if not (dist < max_range and dist <= PERIM_MAX_M):
            continue
        if d == 'E':
            true, axis = _PERIM_SURF['E'] - dist, 0
        elif d == 'W':
            true, axis = _PERIM_SURF['W'] + dist, 0
        elif d == 'N':
            true, axis = _PERIM_SURF['N'] - dist, 1
        else:  # 'S'
            true, axis = _PERIM_SURF['S'] + dist, 1
        out[axis] = (true, int(round(true / CELL_SIZE_M)))
    return out
```
(Ensure `CELL_SIZE_M` is imported from `flood_fill_brain`.) If `test_perimeter_offset_recovers_true_x...` is off because the synthetic perp differs from the hand-computed value (window/median effects), adjust the TEST's tolerance (keep < 0.15) — do NOT fudge the formula; the formula is fixed by the spec geometry.

- [ ] **Step 5: Run → PASS** (the 5 new tests). Then the full regression — additive only (Task 1 is a new pure function, no integration yet): the 5 new tests pass, NO previously-passing test regresses, the 3 `[0.05-*]` stay xfailed. (If `test_wall_localize.py` already existed, its prior tests must still pass too.) Report the actual pass/xfail counts.

- [ ] **Step 6: Commit** `wall_localize.py test/test_wall_localize.py` — `feat: perimeter_offset (absolute boundary-axis re-anchor from the maze outer walls)`.

---

## Task 2: Integrate into `MazeMotion._center` (`maze_motion.py`)

**Files:** Modify `tugbot_maze/maze_motion.py`; Test `test/test_maze_motion_sim.py`.

- [ ] **Step 1: READ** `maze_motion.py` `_center` (lines ~174-260) and `_reanchor` (~262): how `ox, oy = cell_center_offset(...)` is consumed (the centering command + the commit gate), the `(ox, oy)` sign convention (from Task 1 Step 1), and how `self.cell` is set in `_reanchor`. Note `self.boundary_margin_m` (=0.40) and `CELL_SIZE_M` (=2.0).

- [ ] **Step 2: Write the failing offline-recovery guard** — append to `test/test_maze_motion_sim.py`:
```python
def test_perimeter_reanchor_recovers_offcenter_in_boundary_cell():
    """In a boundary cell (10,5), started 0.6 m off-center, the perimeter re-anchor must let
    _center recover to true center -- the existing corridor-only centering cannot pull 0.6 m."""
    import math
    sim = MazeSim(load_segments(), (19.4, 10.0), 0.0, inertia=True)   # cell (10,5) center (20,10), 0.6 m off in x
    m = MazeMotion()
    m.cell = (10, 5)
    m.phase = 'center'
    t = 0.0
    for _ in range(120):                                   # 12 s to settle
        scan = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        v, w, _ = m.step(sim.reported_pose, scan, t)
        sim.step(v, w, 0.1)
        t += 0.1
    assert abs(sim.x - 20.0) < 0.20, f"did not recenter via perimeter anchor: x={sim.x:.3f}"
    assert not sim.collides(sim.x, sim.y, sim.yaw)
```

- [ ] **Step 3: Run → FAIL** `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest "test/test_maze_motion_sim.py::test_perimeter_reanchor_recovers_offcenter_in_boundary_cell" -q` — expect the robot does NOT recover (ends well off x=20) WITHOUT the integration. (If it happens to pass already, the test isn't exercising the gap — STOP and report so we strengthen it before implementing.)

- [ ] **Step 4: Implement the integration in `_center`.** Add `from tugbot_maze.wall_localize import cell_center_offset, perimeter_offset` (extend the existing import). After `ox, oy = cell_center_offset(ranges, amin, ainc, yaw)` (both call sites in `_center`, ~line 197 and ~231), insert:
```python
        # Perimeter absolute re-anchor: in a boundary cell, the maze outer wall is a drift-immune
        # absolute reference. Override the boundary-axis centering offset (trustworthy even when
        # badly off-center, unlike the mis-associated interior walls) and correct a 1-cell desync.
        peri = perimeter_offset(ranges, amin, ainc, yaw, self.cell)
        for axis, (true_coord, implied) in peri.items():
            # dcell self-consistency: correct (unbounded) only when the implied cell is confidently core
            if implied != self.cell[axis] and abs(true_coord - CELL_SIZE_M * implied) <= (CELL_SIZE_M / 2.0 - self.boundary_margin_m):
                self.cell = (implied, self.cell[1]) if axis == 0 else (self.cell[0], implied)
            # absolute centering offset on this axis (match cell_center_offset's sign convention):
            abs_off = true_coord - CELL_SIZE_M * self.cell[axis]   # robot-minus-center; FLIP if cell_center_offset uses center-minus-robot
            if axis == 0:
                ox = abs_off
            else:
                oy = abs_off
```
IMPORTANT: confirm the sign in Step 1. If `cell_center_offset` returns center-minus-robot, use `abs_off = CELL_SIZE_M * self.cell[axis] - true_coord`. Place this block so the overridden `ox/oy` flow into BOTH the centering command and the commit gate (i.e. right after each `cell_center_offset` call). Do NOT alter non-boundary behavior (when `peri` is `{}`, `ox/oy` are unchanged).

- [ ] **Step 5: Run the guard → PASS**, then the FULL regression. The guard passes (robot recenters to x≈20). Full regression: the new guard passes, NO previously-passing test regresses, and the 3 `[0.05-*]` xfails MUST remain xfailed (perimeter re-anchor doesn't target them). If a previously-passing case flips, STOP and report (do not weaken). Confirm the non-boundary tests (`test_symmetric_following...`, `test_grid_centerline...`) still pass (perimeter_offset returns `{}` for their non-boundary cells → unchanged). Report the actual counts.

- [ ] **Step 6: Commit** `maze_motion.py test/test_maze_motion_sim.py` — `feat: perimeter absolute re-anchor in _center (boundary-axis offset override + dcell fix)`.

---

## Task 2 — REVISION (2026-06-28): global drift-reset + odom cross-check gate

Empirical findings during execution (verified): `perimeter_offset` (a) returns `{}` for interior cells (the bulk of the NE struggle, cx 8,9, gets no help), and (b) **mis-fires** when `self.cell` is a boundary cell but the robot is physically one cell short with a walled edge — it reads the interior wall as the perimeter (robot at (18.6,10) in cell (9,5) believing (10,5) → returns `(20.61, cell 10)`). `pose_to_cell(odom)` is correct there `(9,5)` and disagrees with the perimeter's implied cell → an **odom cross-check catches the mis-fire**. The user chose the **GLOBAL DRIFT-RESET** integration (resets accumulated odom drift on a boundary fix → downstream interior cells benefit, not just the boundary leg). This SUPERSEDES the Task 2 Step 4 "local offset override only" above.

Revised Task 2:
- **`__init__`:** add `self.pose_corr = [0.0, 0.0]` (accumulated drift correction, map frame).
- **top of `step()`:** apply it before dispatch — `pose = (pose[0] + self.pose_corr[0], pose[1] + self.pose_corr[1], pose[2])` — so EVERY phase uses the corrected pose. (On this baseline `step` is the plain dispatch — no footprint wrapper.)
- **in `_center` (boundary cells), after `ox,oy = cell_center_offset(...)`:**
```python
        peri = perimeter_offset(ranges, amin, ainc, yaw, self.cell)
        for axis, (true_coord, implied) in peri.items():
            pc = pose_to_cell(x, y)                      # x,y are the CORRECTED pose in _center
            if implied != pc[axis]:
                continue                                 # odom-gate: perimeter saw an interior wall (mis-fire) -> skip
            self.pose_corr[axis] += true_coord - (x if axis == 0 else y)   # GLOBAL drift reset on this axis
            off = true_coord - CELL_SIZE_M * self.cell[axis]               # LOCAL recenter offset (robot-minus-center)
            if axis == 0: ox = off
            else:         oy = off
```
  (Confirm `x, y` in `_center` are already the corrected pose; if `_center` re-reads `pose[0]/pose[1]`, ensure those are the corrected values. Match the `ox/oy` sign to `cell_center_offset` = **robot-minus-center**, confirmed in T1.) The global reset makes the corrected pose match the absolute perimeter coordinate → subsequent ticks (and `_reanchor`, which uses the corrected pose) are accurate, fixing the 1-cell desync downstream WITHOUT a separate perimeter dcell-correction.
- **Tests** (`test_maze_motion_sim.py`):
  1. **recenter guard** (as in Step 2 above): robot 0.6 m off-center in (10,5) recenters to x≈20 via the local offset.
  2. **global drift-reset:** with injected odom drift, drive the robot into a boundary cell; after `_center`, `MazeMotion`'s corrected pose matches the true `sim` pose within tol (drift removed) — FAILS without the reset.
  3. **mis-fire guard:** robot physically in (9,5) believing (10,5) on a walled-edge row → `pose_corr` unchanged and no false dcell flip (the odom-gate skipped the bad reading).
- Commit message: `feat: perimeter global drift-reset re-anchor in _center (odom-gated)`.

---

## Task 3: Final regression + build sanity + Gazebo handoff

**Files:** none (verification only).

- [ ] **Step 1: Full regression** (header command) → ALL PASS (record count + the 3 `[0.05-*]` still xfailed; nothing newly failing).
- [ ] **Step 2: Build + sanity**
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260614 \
  && source /opt/ros/jazzy/setup.bash && colcon build --packages-select tugbot_maze --symlink-install 2>&1 | tail -2
B=build/tugbot_maze/tugbot_maze
echo -n "perimeter_offset present: "; grep -c "def perimeter_offset" "$B/wall_localize.py"
echo -n "wired into _center: "; grep -c "perimeter_offset" "$B/maze_motion.py"
```
- [ ] **Step 3: Final whole-change review** over `git diff d63c88d HEAD` (the oracle baseline → HEAD; covers Tasks 1-2). Confirm: only `wall_localize` (new fn) + `_center` (boundary block) + tests changed; non-boundary navigation untouched; the perimeter constants match `load_segments`.
- [ ] **Step 4: Report Gazebo-run readiness** (user-initiated — do NOT auto-launch):
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260614 \
  && export DISPLAY=:1 && bash tools/run_flood_fill_maze.sh 1800 false true odom_locked true
```
Headline (vs the run-2 NE plateau): does the robot break past dist 5.65 m toward the exit (10,9), with NE `SENSE` walls now matching ground truth at (10,5)/(10,8)/(·,9) (analyze the log against `ground_truth_edge_open`), and the true-oracle collision rate in the NE down? Confirm `/scan` frame_id is the sensor frame (`scan_omni`) at startup.

---

## Self-review notes (author)
- **Coverage:** spec A→T1 (`perimeter_offset` + 5 tests incl. recover/desync/abstain/interior/north); spec B→T2 (`_center` integration: offset override + dcell correction + the off-center recovery guard); validation→T3.
- **Names/consts:** `_PERIM_SURF` (E 20.89, W 1.14, N 18.90, S −0.85), `PERIM_MAX_M`=2.5, `_E_WALL_MAX_CY`=8, `CELL_SIZE_M`=2.0 used consistently; axis 0=x, 1=y throughout.
- **Sign-convention flag** (T1S1 + T2S4): the one correctness hinge — the absolute offset sign must match `cell_center_offset`. Called out explicitly with the flip alternative.
- **Regression-catcher has teeth:** the T2 guard FAILS pre-integration (existing centering can't pull 0.6 m) and PASSES after — and T2S3 says STOP if it passes pre-integration (so the test genuinely exercises the gap).
- **xfails preserved** (T2S5 + T3S1): the 3 `[0.05-*]` rear-corner cases stay xfailed; perimeter re-anchor is out of their scope.
- **Read-first** flagged where line numbers are approximate (`_center`, `cell_center_offset` call/sign) and the synthetic-scan tolerance may need adjusting (not the formula).
