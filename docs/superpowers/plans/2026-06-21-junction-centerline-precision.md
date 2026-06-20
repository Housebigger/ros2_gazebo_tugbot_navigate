# Odom-Anchored Centerline Through Open Junctions — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the corridor follower a valid lateral reference in the open-junction zone — replace the blind `0.0` fallback with the robot's clamped odom offset from the grid corridor centerline — so it stops drifting into junction corners.

**Architecture:** One pure helper `grid_cross_track` (odom/grid centerline offset, same sign convention as `cross_track_offset` but always valid), validated in-loop by an offline real-physics convergence test, then wired into `_drive` (clamped to 0.40 m) as the open-junction fallback. The proven wall-balanced centering (when ≥1 side wall is visible) is untouched.

**Tech Stack:** Python 3.12, pytest; ROS-free motion/sim layer. Pairs with spec `docs/superpowers/specs/2026-06-20-junction-centerline-precision-design.md` (read its *Revisions* section).

**Conventions for every task:**
- `PKG = /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260614/src/tugbot_maze`
- `REPO = /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`
- Run tests: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest <path> -v`
- Work on branch `junction-centerline-precision` (already created off `main`; **never push**).
- Commit messages end with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`. **No backticks inside `git commit -m`.** Stage only each task's files via `git -C "$REPO" add <path>` (NEVER `git add -A` — the repo has unrelated `20260522/build` noise).

---

## File Structure

| File | Responsibility | Task |
|------|----------------|------|
| `tugbot_maze/hop_controller.py` | + pure `grid_cross_track(...)` helper | 1 |
| `test/test_hop_controller.py` | + `grid_cross_track` unit tests | 1 |
| `test/test_maze_motion_sim.py` | + open-junction convergence + large-offset repro tests | 2 |
| `tugbot_maze/maze_motion.py` | `_drive` uses clamped `grid_cross_track`; new `grid_fallback_max_m` tunable; import swap | 3 |

---

## Task 1: `grid_cross_track` helper (`hop_controller.py`)

**Files:**
- Modify: `tugbot_maze/hop_controller.py` (append after `cross_track_offset`, or anywhere among the pure helpers)
- Test: `test/test_hop_controller.py` (append + extend imports)

- [ ] **Step 1: Write the failing tests** — add `grid_cross_track` to the imports at the top of `test/test_hop_controller.py`, then append:

```python
def test_grid_cross_track_all_dirs():
    # cell (5,5) centre (10,10); robot 0.4 m off-centre on the lateral axis
    assert abs(grid_cross_track(9.6, 10.0, (5, 5), (0, 1)) - 0.4) < 1e-9    # N: west=left  => +
    assert abs(grid_cross_track(10.4, 10.0, (5, 5), (0, 1)) + 0.4) < 1e-9   # N: east=right => -
    assert abs(grid_cross_track(10.4, 10.0, (5, 5), (0, -1)) - 0.4) < 1e-9  # S: east=left  => +
    assert abs(grid_cross_track(10.0, 10.4, (5, 5), (1, 0)) - 0.4) < 1e-9   # E: north=left => +
    assert abs(grid_cross_track(10.0, 10.4, (5, 5), (-1, 0)) + 0.4) < 1e-9  # W: north=right=> -


def test_grid_cross_track_on_centerline_is_zero():
    assert grid_cross_track(10.0, 10.0, (5, 5), (0, 1)) == 0.0
    assert grid_cross_track(10.0, 10.0, (5, 5), (1, 0)) == 0.0
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_hop_controller.py -k grid_cross_track -v`
Expected: FAIL — `ImportError: cannot import name 'grid_cross_track'`.

- [ ] **Step 3: Implement** — append to `tugbot_maze/hop_controller.py`:

```python
def grid_cross_track(pose_x: float, pose_y: float, cell, hop_dir, *,
                     cell_size_m: float = 2.0) -> float:
    """Signed lateral offset of the robot to the LEFT of the hop direction (+ = robot left of
    centre), measured from the GRID corridor centerline (the cell-centre line) using the odom pose.
    Unlike cross_track_offset (wall-derived, 0 when no side wall is visible), this is ALWAYS valid,
    so it gives the corridor follower a centerline reference through an open junction where both side
    walls vanish. Sign convention matches cross_track_offset. `cell` is the hop's SOURCE cell; it
    shares the perpendicular coordinate with the target (the hop is along a cardinal), so it defines
    the corridor centerline for the whole hop. The convergence target is the ODOM centerline (off the
    true centre by the bounded drift residual); the caller clamps the result, so a small drift error
    can only nudge, never swing."""
    dx, dy = hop_dir
    if dy != 0:                                       # N/S travel -> lateral is the x axis
        off = pose_x - cell_size_m * cell[0]          # +E
        return -off if dy > 0 else off                # N: east=right=>-; S: east=left=>+
    off = pose_y - cell_size_m * cell[1]              # +N
    return off if dx > 0 else -off                    # E: north=left=>+; W: north=right=>-
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_hop_controller.py -v`
Expected: PASS (existing + 2 new).

- [ ] **Step 5: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/hop_controller.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_hop_controller.py
git -C "$REPO" commit -m "feat: add grid_cross_track odom-grid centerline helper

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Offline open-junction repro tests (`test_maze_motion_sim.py`)

These validate the helper *in the control loop under real collision physics*, and are the
discriminating proof of the fix's value (they fail with the old `0.0` fallback). They call
`grid_cross_track` + `corridor_follow_command` directly, so they depend on Task 1 only.

**Files:**
- Test: `test/test_maze_motion_sim.py` (append + extend the `hop_controller` import to include `grid_cross_track`)

- [ ] **Step 1: Write the tests** — extend the existing
`from tugbot_maze.hop_controller import side_distances, corridor_follow_command` to also import
`grid_cross_track`, then append:

```python
def test_grid_centerline_holds_through_open_junction():
    """Open zone: side walls at x=+-2.0 are beyond sensing range (face ~1.5-1.9 m > wall_seen_m=1.3,
    so both UNSEEN -> the fallback path runs) yet collidable (a wrong-sign fallback diverges into a
    wall). Start 0.4 m right of the grid centerline (x = 2*0 = 0) and drive N: the odom-grid fallback
    must re-centre. With the old fallback=0.0 the robot holds the 0.4 m offset and the convergence
    assertion fails."""
    walls = [(-2.0, -5.0, -2.0, 8.0), (2.0, -5.0, 2.0, 8.0)]
    sim = MazeSim(walls, (0.4, 0.0), math.pi / 2, inertia=True)
    collided = False
    for _ in range(120):
        ranges, amin, ainc = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        perp = cell_wall_perp_dist(ranges, amin, ainc, sim.yaw)
        d_left, d_right = side_distances(perp, (0, 1))
        fallback = max(-0.40, min(0.40, grid_cross_track(sim.x, sim.y, (0, 0), (0, 1))))
        v, w = corridor_follow_command(sim.yaw, math.pi / 2, d_left, d_right, None, fallback_cross=fallback)
        sim.step(v, w, 0.1)
        if sim.collides(sim.x, sim.y):
            collided = True
    assert abs(sim.x) < 0.15, f"did not converge via grid fallback: x={sim.x:.3f}"
    assert not collided


def test_grid_centerline_recovers_from_large_offset():
    """0.55 m offset (beyond the 0.40 clamp): must re-centre without stalling/wedging or hitting the
    x=+-2.0 walls -- characterises the clamp's creep-and-steer regime."""
    walls = [(-2.0, -5.0, -2.0, 8.0), (2.0, -5.0, 2.0, 8.0)]
    sim = MazeSim(walls, (0.55, 0.0), math.pi / 2, inertia=True)
    collided = False
    for _ in range(160):
        ranges, amin, ainc = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        perp = cell_wall_perp_dist(ranges, amin, ainc, sim.yaw)
        d_left, d_right = side_distances(perp, (0, 1))
        fallback = max(-0.40, min(0.40, grid_cross_track(sim.x, sim.y, (0, 0), (0, 1))))
        v, w = corridor_follow_command(sim.yaw, math.pi / 2, d_left, d_right, None, fallback_cross=fallback)
        sim.step(v, w, 0.1)
        if sim.collides(sim.x, sim.y):
            collided = True
    assert not collided
    assert abs(sim.x) < 0.2, f"did not recover from large offset: x={sim.x:.3f}"
```

- [ ] **Step 2: Run to verify they pass**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion_sim.py -k grid_centerline -v`
Expected: PASS (both). If `test_grid_centerline_recovers_from_large_offset` collides or fails to converge, that is the offline signal to tighten the clamp (try 0.35) here and in Task 3 — do not weaken the assertions.

- [ ] **Step 3: Sanity-check the test has teeth** (optional, do-not-commit): temporarily set `fallback = 0.0` in `test_grid_centerline_holds_through_open_junction` and confirm it now FAILS the `abs(sim.x) < 0.15` assertion (the robot holds ~0.4). Revert to the `grid_cross_track` line. This confirms the test is discriminating, not vacuous.

- [ ] **Step 4: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_motion_sim.py
git -C "$REPO" commit -m "test: offline open-junction centerline convergence + large-offset repro

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Wire the clamped odom fallback into `_drive` (`maze_motion.py`)

**Files:**
- Modify: `tugbot_maze/maze_motion.py` (import, `__init__` param, `_drive` fallback)

- [ ] **Step 1: Import swap.** In the `from tugbot_maze.hop_controller import (...)` block, replace `cross_track_offset` with `grid_cross_track`. First confirm `cross_track_offset` is used nowhere else in the file: `cd "$PKG" && grep -n cross_track_offset tugbot_maze/maze_motion.py` (expect only the import and the one `_drive` use). The block becomes:

```python
from tugbot_maze.hop_controller import (
    centering_command, grid_cross_track,
    side_distances, corridor_follow_command, profiled_turn_command, backout_command)
```
(`cell_center_offset` is imported from `wall_localize` and still used by `_center` — leave it.)

- [ ] **Step 2: New `__init__` tunable.** Add `grid_fallback_max_m: float = 0.40` to the `MazeMotion.__init__` signature (e.g. right after `max_backout_attempts: int = 2`), and assign it near the other back-out assignments:

```python
        self.grid_fallback_max_m = grid_fallback_max_m   # clamp on the open-junction odom fallback
```

- [ ] **Step 3: Swap the `_drive` fallback.** In `_drive`, find the open-junction fallback (the two lines):

```python
        ox, oy = cell_center_offset(ranges, amin, ainc, yaw)
        fallback = cross_track_offset(ox, oy, self.hop_dir)        # open-junction fallback
```

and replace BOTH with:

```python
        fallback = max(-self.grid_fallback_max_m,
                       min(self.grid_fallback_max_m,
                           grid_cross_track(x, y, self.cell, self.hop_dir, cell_size_m=CELL_SIZE_M)))  # odom grid centerline (valid in open junctions), clamped to the smooth creep-and-steer regime
```
(`x, y` come from `x, y, yaw = pose` at the top of `_drive`; `self.cell`, `self.hop_dir`, `CELL_SIZE_M` are already in scope. The `ox, oy` were used only here, so the `cell_center_offset` call is removed.)

- [ ] **Step 4: Run the full offline suite to verify no regression**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion_sim.py test/test_maze_motion.py test/test_hop_controller.py -v`
Expected: ALL PASS — the 5 full-solve `test_reaches_exit_without_collision_or_desync` cases still reach the exit collision-free with `max_desync ≤ 1`, `test_backout_is_exercised_end_to_end` still holds, and the new junction tests pass. The fallback change only affects the open-junction zone, so the solve must not regress.

- [ ] **Step 5: Compile check**

Run: `cd "$PKG" && python3 -m py_compile tugbot_maze/maze_motion.py && echo COMPILE_OK`
Expected: `COMPILE_OK`.

- [ ] **Step 6: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_motion.py
git -C "$REPO" commit -m "feat: drive uses clamped odom grid centerline as the open-junction fallback

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Final validation (after all tasks)

1. **Full ROS-free suite green:**
   `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py test/test_maze_motion_sim.py test/test_hop_controller.py test/test_flood_fill_brain.py test/test_junction_log.py -q`
   Expected: all pass.
2. **Rebuild + Gazebo confirmation** (user-initiated; agent `pkill` is hook-blocked — the user clears
   stray sims first via `! pkill -9 -f "gz sim|ruby.*gz|parameter_bridge|flood_fill_solver|ros2 launch"`).
   Rebuild `tugbot_maze` (`colcon build --packages-select tugbot_maze`), then
   `tools/run_flood_fill_maze.sh 1800 false true odom_locked true`. Success signal: the (6,4)-class
   junction wedge-recovers drop sharply vs. the prior run (24), and deeper exit-ward progress than the
   prior best (dist 9.78 m). `EXIT_REACHED` is the stretch goal.
3. Do **not** merge to `main` or push — banking/merge is a separate, user-approved step after the
   Gazebo run.
