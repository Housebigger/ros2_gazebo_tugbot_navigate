# Tighter Corridor Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace `MazeMotion`'s `drive` and `turn` laws with continuous symmetric wall-following (true-centerline, drift-immune) and a decel-limited profiled turn (no latency overshoot), to eliminate the physical-wedge plateau that gates a consistent autonomous maze solve.

**Architecture:** Add three pure helpers to `hop_controller.py` (`side_distances`, `centerline_cross`, `corridor_follow_command`) plus `profiled_turn_command`; `corridor_follow_command` reuses the proven `corridor_drive_command` steering/throttle by feeding it a symmetric-wall cross-track. Wire them into `maze_motion.py`'s `_drive` and `_turn`. Everything else (brain, sensing, `_center`, re-anchor, wedge detector, `_recover`) is untouched.

**Tech Stack:** Python 3.12, pytest, ROS 2 Jazzy / Gazebo Harmonic (Gazebo only for the final confirmation run). All controller code is ROS-free and unit-testable.

**Spec:** `docs/superpowers/specs/2026-06-19-tighter-corridor-controller-design.md`

**Branch:** `tighter-corridor-controller` (already created off `main`). Local only — **never push**.

**Standing constraints:**
- Commit messages end with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
- Run pytest from the package dir. Absolute path:
  `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260614/src/tugbot_maze`
- Agent `pkill`/`kill` is hook-blocked; the user clears stray Gazebo sims via the `!` prefix.
- Foreground `sleep` is blocked; background only.

---

## File Structure

- **Modify** `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/hop_controller.py`
  — add `side_distances`, `centerline_cross`, `corridor_follow_command`, `profiled_turn_command`.
  Keep all existing functions.
- **Modify** `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_motion.py`
  — new `__init__` tunables; `_turn` uses `profiled_turn_command`; `_drive` uses
  `side_distances` + `corridor_follow_command`; update the `hop_controller` import.
- **Modify** `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_hop_controller.py`
  — unit tests for the four new functions (add `import pytest`).
- **Modify** `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_motion_sim.py`
  — add the offline centerline-convergence test; keep the existing end-to-end regression.

Shorthand below: `PKG = ros2_ws_tugbot_nav_20260614/src/tugbot_maze`, run all commands from
`PKG` (absolute path above).

---

## Task 1: `side_distances` helper

**Files:**
- Modify: `PKG/tugbot_maze/hop_controller.py`
- Test: `PKG/test/test_hop_controller.py`

- [ ] **Step 1: Write the failing test**

Add to `test/test_hop_controller.py` (extend the existing import line to include `side_distances`):

```python
from tugbot_maze.hop_controller import (
    hop_command, centering_command, hop_drive_command, cross_track_offset)
from tugbot_maze.hop_controller import corridor_drive_command
from tugbot_maze.hop_controller import side_distances


def test_side_distances_maps_left_right_by_direction():
    # robot-frame left = +90 deg from heading
    perp = {'E': 1.0, 'W': 2.0, 'N': 3.0, 'S': 4.0}
    assert side_distances(perp, (0, 1)) == (2.0, 1.0)    # N: left=W, right=E
    assert side_distances(perp, (0, -1)) == (1.0, 2.0)   # S: left=E, right=W
    assert side_distances(perp, (1, 0)) == (3.0, 4.0)    # E: left=N, right=S
    assert side_distances(perp, (-1, 0)) == (4.0, 3.0)   # W: left=S, right=N
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_hop_controller.py::test_side_distances_maps_left_right_by_direction -q`
Expected: FAIL with `ImportError`/`cannot import name 'side_distances'`.

- [ ] **Step 3: Write the implementation**

Add to `tugbot_maze/hop_controller.py` (after `cross_track_offset`):

```python
def side_distances(perp, hop_dir) -> Tuple[float, float]:
    """Map the per-cardinal perpendicular-distance dict (keys 'E','W','N','S', from
    cell_wall_perp_dist) to the robot's (left, right) side-wall distances for travel along
    hop_dir. Robot-frame left is +90 deg from heading."""
    dx, dy = hop_dir
    if dy > 0:                       # N: left=W, right=E
        return perp['W'], perp['E']
    if dy < 0:                       # S: left=E, right=W
        return perp['E'], perp['W']
    if dx > 0:                       # E: left=N, right=S
        return perp['N'], perp['S']
    return perp['S'], perp['N']      # W: left=S, right=N
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_hop_controller.py::test_side_distances_maps_left_right_by_direction -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/hop_controller.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_hop_controller.py
git commit -m "feat: side_distances maps hop direction to left/right wall distances" \
  -m "Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: `centerline_cross` (3-tier symmetric centering)

**Files:**
- Modify: `PKG/tugbot_maze/hop_controller.py`
- Test: `PKG/test/test_hop_controller.py`

- [ ] **Step 1: Write the failing test**

Add `import pytest` at the top of `test/test_hop_controller.py` (it currently imports only
`math`), add `centerline_cross` to the imports, and add:

```python
def test_centerline_cross_both_walls_balances():
    # equidistant -> on centre (0); convention: + = robot LEFT of centre
    assert centerline_cross(0.88, 0.88) == 0.0
    # closer to the LEFT wall (d_left small) -> robot is LEFT of centre (+)
    assert centerline_cross(0.5, 1.2) > 0.0
    assert centerline_cross(0.6, 1.0) == pytest.approx(0.2)   # (d_right - d_left)/2


def test_centerline_cross_single_wall_holds_half_corridor():
    # only RIGHT wall seen (left open at 2.0), far from it -> robot LEFT of centre (+)
    assert centerline_cross(2.0, 1.2, half_corridor_m=0.88) == pytest.approx(1.2 - 0.88)
    # only LEFT wall seen, hugging it (0.5) -> robot LEFT of centre (+)
    assert centerline_cross(0.5, 2.0, half_corridor_m=0.88) == pytest.approx(0.88 - 0.5)


def test_centerline_cross_neither_wall_uses_fallback():
    assert centerline_cross(2.0, 2.0, fallback_cross=0.25) == 0.25
    assert centerline_cross(2.0, 2.0) == 0.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_hop_controller.py -k centerline_cross -q`
Expected: FAIL with `cannot import name 'centerline_cross'`.

- [ ] **Step 3: Write the implementation**

Add to `tugbot_maze/hop_controller.py` (after `side_distances`):

```python
def centerline_cross(d_left: float, d_right: float, *, fallback_cross: float = 0.0,
                     wall_seen_m: float = 1.3, half_corridor_m: float = 0.88) -> float:
    """Signed lateral offset of the robot to the LEFT of the corridor centerline (+ = left),
    from the left/right side-wall distances. A side counts as a seen wall when < wall_seen_m.
      both seen   -> balance the two (the drift-immune true centerline -- THE key change),
      one seen    -> hold half_corridor_m from that wall,
      neither     -> the caller's fallback (cell-grid/odom estimate), else 0 (hold heading)."""
    left_seen = d_left < wall_seen_m
    right_seen = d_right < wall_seen_m
    if left_seen and right_seen:
        return (d_right - d_left) / 2.0
    if right_seen:
        return d_right - half_corridor_m
    if left_seen:
        return half_corridor_m - d_left
    return fallback_cross
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_hop_controller.py -k centerline_cross -q`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/hop_controller.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_hop_controller.py
git commit -m "feat: centerline_cross derives lateral offset from both side walls (drift-immune)" \
  -m "Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: `corridor_follow_command` (symmetric wall-following drive)

**Files:**
- Modify: `PKG/tugbot_maze/hop_controller.py`
- Test: `PKG/test/test_hop_controller.py`

- [ ] **Step 1: Write the failing test**

Add `corridor_follow_command` to the imports and add:

```python
def test_corridor_follow_centres_between_walls():
    # both walls equal -> straight (on centre), driving forward
    v, w = corridor_follow_command(math.pi / 2, math.pi / 2, 0.88, 0.88)
    assert v > 0.0 and abs(w) < 1e-9


def test_corridor_follow_steers_toward_farther_wall():
    # facing N, closer to the RIGHT (east) wall (d_right small) -> right of centre ->
    # steer LEFT (+w) back toward centre, still driving
    v, w = corridor_follow_command(math.pi / 2, math.pi / 2, 1.2, 0.5)
    assert w > 0.0 and v > 0.0


def test_corridor_follow_single_wall_holds_offset():
    # only LEFT wall seen, robot off-center toward it (0.6) -> left of centre -> steer RIGHT (-w),
    # still driving (a modest offset stays under corridor_drive_command's turn-first throttle)
    v, w = corridor_follow_command(math.pi / 2, math.pi / 2, 0.6, 2.0)
    assert w < 0.0 and v > 0.0


def test_corridor_follow_no_walls_uses_fallback():
    # both open -> fallback_cross drives steering (0.3 left -> steer right)
    _, w = corridor_follow_command(math.pi / 2, math.pi / 2, 2.0, 2.0, fallback_cross=0.3)
    assert w < 0.0
    _, w0 = corridor_follow_command(math.pi / 2, math.pi / 2, 2.0, 2.0, fallback_cross=0.0)
    assert abs(w0) < 1e-9


def test_corridor_follow_slows_near_wall_never_zero():
    v, _ = corridor_follow_command(math.pi / 2, math.pi / 2, 0.88, 0.88, near_wall_m=0.40)
    assert 0.0 < v <= 0.12        # ~wedge_v_floor, never frozen


def test_corridor_follow_within_envelope():
    v, w = corridor_follow_command(0.0, math.pi / 2, 0.3, 0.3, near_wall_m=0.35)  # extreme
    assert -0.5 <= v <= 0.5 and -0.5 <= w <= 0.5
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_hop_controller.py -k corridor_follow -q`
Expected: FAIL with `cannot import name 'corridor_follow_command'`.

- [ ] **Step 3: Write the implementation**

Add to `tugbot_maze/hop_controller.py` (after `corridor_drive_command`):

```python
def corridor_follow_command(yaw: float, cardinal_yaw: float, d_left: float, d_right: float,
                            near_wall_m: Optional[float] = None, *, fallback_cross: float = 0.0,
                            wall_seen_m: float = 1.3, half_corridor_m: float = 0.88,
                            max_cross_track_m: float = 0.6, v_max: float = 0.3,
                            w_max: float = 0.5, kp_ang: float = 1.5, lookahead_m: float = 0.7,
                            slow_angle: float = 0.6, wedge_slow_m: float = 0.50,
                            wedge_stop_m: float = 0.40, wedge_v_floor: float = 0.10
                            ) -> Tuple[float, float]:
    """Symmetric wall-following straight drive. Derives the lateral centerline offset from the
    two side-wall distances (centerline_cross), clamps it, then steers/throttles with the proven
    pure-pursuit + never-zero wedge-floor law (corridor_drive_command). Keeps the 0.35 m robot on
    the PHYSICAL centerline using both walls (drift-immune) instead of the cell-grid offset that
    vanished at openings -- closing the off-center-entry -> wedge path."""
    cross = centerline_cross(d_left, d_right, fallback_cross=fallback_cross,
                             wall_seen_m=wall_seen_m, half_corridor_m=half_corridor_m)
    cross = max(-max_cross_track_m, min(max_cross_track_m, cross))
    return corridor_drive_command(yaw, cardinal_yaw, cross, near_wall_m, v_max=v_max,
                                  w_max=w_max, kp_ang=kp_ang, lookahead_m=lookahead_m,
                                  slow_angle=slow_angle, wedge_slow_m=wedge_slow_m,
                                  wedge_stop_m=wedge_stop_m, wedge_v_floor=wedge_v_floor)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_hop_controller.py -k corridor_follow -q`
Expected: PASS (6 tests).

- [ ] **Step 5: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/hop_controller.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_hop_controller.py
git commit -m "feat: corridor_follow_command -- symmetric wall-following straight drive" \
  -m "Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: `profiled_turn_command` (decel-limited turn)

**Files:**
- Modify: `PKG/tugbot_maze/hop_controller.py`
- Test: `PKG/test/test_hop_controller.py`

- [ ] **Step 1: Write the failing test**

Add `profiled_turn_command` to the imports and add:

```python
def test_profiled_turn_full_rate_when_far():
    # 90 deg error -> sqrt(2*1.2*1.57) ~ 1.94, capped to turn_w_max
    w = profiled_turn_command(0.0, math.pi / 2, ang_decel=1.2, turn_w_max=0.35)
    assert abs(w - 0.35) < 1e-9


def test_profiled_turn_decelerates_when_close():
    # small error -> w = sqrt(2*ang_decel*|err|), shrinks toward 0 as err closes (no overshoot)
    w_small = profiled_turn_command(math.pi / 2 - 0.05, math.pi / 2, ang_decel=1.2, turn_w_max=0.35)
    w_tiny = profiled_turn_command(math.pi / 2 - 0.01, math.pi / 2, ang_decel=1.2, turn_w_max=0.35)
    assert 0.0 < w_tiny < w_small <= 0.35


def test_profiled_turn_sign_toward_target():
    assert profiled_turn_command(0.0, 0.3) > 0.0     # target to the LEFT (+)
    assert profiled_turn_command(0.0, -0.3) < 0.0    # target to the RIGHT (-)


def test_profiled_turn_within_cap():
    w = profiled_turn_command(0.0, math.pi, turn_w_max=0.35)
    assert -0.35 <= w <= 0.35
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_hop_controller.py -k profiled_turn -q`
Expected: FAIL with `cannot import name 'profiled_turn_command'`.

- [ ] **Step 3: Write the implementation**

Add to `tugbot_maze/hop_controller.py` (after `corridor_follow_command`):

```python
def profiled_turn_command(yaw: float, target_cardinal: float, yaw_rate: float = 0.0, *,
                          ang_decel: float = 1.2, turn_w_max: float = 0.35,
                          kd: float = 0.0) -> float:
    """Rotate-in-place toward target_cardinal with a DECEL-LIMITED angular profile:
    |w| = min(turn_w_max, sqrt(2*ang_decel*|err|)) ramps to 0 as the heading error closes, so a
    delayed (command-latency) command lands while the robot is already decelerating -> no
    overshoot (the PD law's failure mode). Optional kd*yaw_rate adds light damping. Returns w
    only (the caller drives v=0)."""
    err = _norm(target_cardinal - yaw)
    w_mag = min(turn_w_max, math.sqrt(2.0 * ang_decel * abs(err)))
    w = math.copysign(w_mag, err) - kd * yaw_rate
    return max(-turn_w_max, min(turn_w_max, w))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_hop_controller.py -k profiled_turn -q`
Expected: PASS (4 tests). Then run the whole file to confirm no regressions:
`python3 -m pytest test/test_hop_controller.py -q` → all green.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/hop_controller.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_hop_controller.py
git commit -m "feat: profiled_turn_command -- decel-limited rotate-in-place (no latency overshoot)" \
  -m "Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: Offline centerline-convergence guarantee

**Files:**
- Test: `PKG/test/test_maze_motion_sim.py`

This is the spec's "with both side walls present, symmetric following holds the robot within the
centerline envelope" assertion — proven offline against the real raycaster + inertia, no Gazebo.

- [ ] **Step 1: Write the failing test**

Add to the imports at the top of `test/test_maze_motion_sim.py`:

```python
from tugbot_maze.cell_walls import cell_wall_perp_dist
from tugbot_maze.hop_controller import side_distances, corridor_follow_command
```

Add the test:

```python
def test_symmetric_following_converges_to_centerline():
    """Straight N-S corridor (side walls at x=+/-1.0, faces ~0.88 m from centre); start 0.4 m
    off-centre and drive N with symmetric wall-following -> converges onto the centerline
    (|x| shrinks well below the start) without colliding. The offline, deterministic form of the
    centerline-envelope guarantee the Gazebo physical-wedge needs."""
    walls = [(-1.0, -5.0, -1.0, 5.0), (1.0, -5.0, 1.0, 5.0)]   # two parallel side walls
    sim = MazeSim(walls, (0.4, 0.0), math.pi / 2, inertia=True)
    for _ in range(120):                                       # 12 s at dt=0.1
        ranges, amin, ainc = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        perp = cell_wall_perp_dist(ranges, amin, ainc, sim.yaw)
        d_left, d_right = side_distances(perp, (0, 1))
        v, w = corridor_follow_command(sim.yaw, math.pi / 2, d_left, d_right)
        sim.step(v, w, 0.1)
    assert abs(sim.x) < 0.2, f"did not converge to centerline: x={sim.x:.3f}"
    assert not sim.collides(sim.x, sim.y)
```

- [ ] **Step 2: Run test to verify it passes**

Run: `python3 -m pytest test/test_maze_motion_sim.py::test_symmetric_following_converges_to_centerline -q`
Expected: PASS. (Tasks 1 & 3 already provide the imported functions, so this should pass
immediately — it is a guarantee test, not a red-green for new code. If it FAILS by a small
margin, the controller is converging but slowly: extend the loop to 160 steps before changing
any tolerance; do NOT relax the `< 0.2` envelope.)

- [ ] **Step 3: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_motion_sim.py
git commit -m "test: offline guarantee that symmetric following converges to the centerline" \
  -m "Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: Wire the new laws into MazeMotion

**Files:**
- Modify: `PKG/tugbot_maze/maze_motion.py`
- Regression: `PKG/test/test_maze_motion_sim.py` (existing `test_reaches_exit_*`)

- [ ] **Step 1: Confirm the regression baseline is green BEFORE editing**

Run: `python3 -m pytest test/test_maze_motion_sim.py -q`
Expected: PASS (the existing end-to-end solve + the new convergence test). This is the baseline
the integration must preserve.

- [ ] **Step 2: Update the `hop_controller` import in `maze_motion.py`**

Replace:

```python
from tugbot_maze.hop_controller import (
    centering_command, cross_track_offset, corridor_drive_command)
```

with:

```python
from tugbot_maze.hop_controller import (
    centering_command, cross_track_offset,
    side_distances, corridor_follow_command, profiled_turn_command)
```

(`corridor_drive_command` is no longer called directly by `maze_motion` — it is invoked inside
`corridor_follow_command` within `hop_controller` — so it is dropped from this import.)

- [ ] **Step 3: Add the new tunables to `MazeMotion.__init__`**

Change the end of the `__init__` signature from:

```python
                 recover_v: float = 0.15, recover_s: float = 1.8, recover_w: float = 0.0):
```

to:

```python
                 recover_v: float = 0.15, recover_s: float = 1.8, recover_w: float = 0.0,
                 ang_decel: float = 1.2, wall_seen_m: float = 1.3, half_corridor_m: float = 0.88):
```

And add the stores next to the existing `self.recover_*` assignments:

```python
        self.recover_v = recover_v; self.recover_s = recover_s; self.recover_w = recover_w
        self.ang_decel = ang_decel; self.wall_seen_m = wall_seen_m
        self.half_corridor_m = half_corridor_m
```

- [ ] **Step 4: Replace the PD turn in `_turn`**

In `_turn`, replace:

```python
        w = self.kp_turn * err - self.kd_turn * self.yaw_rate        # PD: damp latency overshoot
        return (0.0, max(-self.turn_w_max, min(self.turn_w_max, w)), False)
```

with:

```python
        w = profiled_turn_command(yaw, self.target_cardinal, self.yaw_rate,
                                  ang_decel=self.ang_decel, turn_w_max=self.turn_w_max,
                                  kd=self.kd_turn)        # decel profile: no latency overshoot
        return (0.0, w, False)
```

(The settle-ticks / `turn_timeout` handoff block above this is unchanged.)

- [ ] **Step 5: Replace the drive law in `_drive`**

In `_drive`, replace:

```python
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

with:

```python
        ox, oy = cell_center_offset(ranges, amin, ainc, yaw)
        fallback = cross_track_offset(ox, oy, self.hop_dir)        # open-junction fallback
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
        return (v, w, False)
```

- [ ] **Step 6: Run the offline regression — must stay green**

Run: `python3 -m pytest test/test_maze_motion_sim.py -q`
Expected: PASS (`test_reaches_exit_*` for all drift×latency cases + the convergence test).

If a `test_reaches_exit_*` case FAILS, the new law changed the solve. Tune **in this order**,
re-running after each change, until green (do NOT relax the test asserts):
1. `lookahead_m` (raise toward 0.9 for gentler, more stable centerline pursuit).
2. `ang_decel` (raise toward 1.6 for snappier turn settle; lower toward 0.9 if it overshoots).
3. `kp_ang` of the drive — if needed, pass an explicit lower value into the
   `corridor_follow_command` call (e.g. `kp_ang=1.2`).
Record the final values in the commit message.

- [ ] **Step 7: Run the FULL offline suite**

Run: `python3 -m pytest test/test_flood_fill_*.py test/test_cell_walls.py test/test_hop_controller.py test/test_maze_sim.py test/test_maze_motion.py test/test_maze_motion_sim.py test/test_wall_localize.py test/test_wall_relocalize_sim.py -q`
Expected: ALL PASS (the prior 75 + the new unit/convergence tests).

- [ ] **Step 8: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_motion.py
git commit -m "feat: MazeMotion drive=symmetric wall-following, turn=decel profile" \
  -m "Drive now centers on both physical walls (drift-immune) with cell-grid fallback only at \
open junctions; turn uses a decel-limited profile (no latency overshoot). Eliminates the \
off-center-entry -> physical-wedge path. Offline regression stays green." \
  -m "Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 7: Gazebo confirmation run (validation, not code)

**Files:** none (execution + observation).

- [ ] **Step 1: Ensure no stray sim is running**

Check (read-only): `ps -eo args | grep -E "gz sim|parameter_bridge|flood_fill_solver|ros2 launch" | grep -v grep`
If any are present, ask the user to clear them via the `!` prefix:
`! pkill -9 -f "gz sim|ruby.*gz|parameter_bridge|flood_fill_solver|ros2 launch"`

- [ ] **Step 2: (If needed) rebuild the package**

With `--symlink-install`, Python edits are picked up live. If the run script uses an installed
copy, rebuild: from `ros2_ws_tugbot_nav_20260614`, `colcon build --packages-select tugbot_maze`.

- [ ] **Step 3: Launch the confirmation run (alone, detached)**

Run ONLY the script (no prepended commands — a prefix kills its monitor `sleep`):
`tools/run_flood_fill_maze.sh 1800 false true odom_locked true`
(1800 s budget, GUI on, RViz on, `odom_locked`, `sense_debug`.)

- [ ] **Step 4: Observe against the success criteria**

From the run artifact / logs, evaluate:
- **Stage success (minimal):** physical-wedge class eliminated — recover events from physical
  pins ≈ 0; `dbg['sides']` shows the robot held near the centerline (each side ≈ `half_corridor_m`,
  i.e. `|d_left − d_right|` small); the robot progresses past the historical ~14-cell plateau
  toward the exit; dcell stays synced.
- **Full success:** `EXIT_REACHED` (cell `(10,9)`).
- **If wedges persist:** STOP and escalate to the deferred SDF-fidelity offline reproduction
  (import the real SDF doorway geometry into `maze_sim`, reproduce an off-center pin, tune
  offline) before further Gazebo runs — do not grind Gazebo blindly.

- [ ] **Step 5: Record the outcome**

Update the `flood-fill-maze-solver` memory with the run result (cells reached, min dist, recover
count, centerline behavior, EXIT_REACHED or the next blocker). No code commit.

---

## Final Review

After all tasks: dispatch a final code reviewer over the whole `tighter-corridor-controller`
branch diff (vs `main`), then use `superpowers:finishing-a-development-branch`. Per the standing
decision, the branch is **local only and not pushed**; merge into `main` locally only if the user
asks.
