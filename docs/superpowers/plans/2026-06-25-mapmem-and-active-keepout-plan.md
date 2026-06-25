# Map-Memory Manager + Active Keep-Out Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop the doorway-corner collisions at the source (active wall repulsion + radius 0.70) and stop the explored-map from being corrupted by control-failure pins (a dedicated `MapMemory` gateway: persistent-desync odom reconcile + lateral-pin mark-guard).

**Architecture:** Two independent, ROS-free, offline-sim-testable components. (1) Repulsion term added to `corridor_follow_command`. (2) New `MapMemory` class wrapping `FloodFillBrain`, wired into `MazeMotion` (observe every tick, reconcile in `_center`, mark-guard at the two `_drive` giveup sites) and constructed by the node. Spec: `docs/superpowers/specs/2026-06-25-mapmem-and-active-keepout-design.md`.

**Tech Stack:** Python 3.12, ROS 2 Jazzy (node is a thin adapter), pytest. Branch `no-progress-watchdog`. Diagnostics stay IN. Commit trailer ends with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`. **No backticks inside `git commit -m`.** Diagnostic-instrumentation commit `2fe9e39` is reverted only before any future merge — NOT in this plan.

**Conventions for every command below:**
```bash
PKG=/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260614/src/tugbot_maze
# run tests:   cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest <args>
```
Full regression suite (used at the end of Tasks 3, 4, 5):
```bash
cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_map_memory.py test/test_maze_motion.py \
  test/test_maze_motion_sim.py test/test_hop_controller.py test/test_flood_fill_brain.py \
  test/test_junction_log.py test/test_cell_walls.py test/test_maze_sim.py -q
```

---

## Task 1: `MapMemory` module (mark-guard + odom reconcile)

**Files:**
- Create: `src/tugbot_maze/tugbot_maze/map_memory.py`
- Test: `src/tugbot_maze/test/test_map_memory.py`

- [ ] **Step 1: Write the failing tests** — create `test/test_map_memory.py`:

```python
from tugbot_maze.flood_fill_brain import FloodFillBrain
from tugbot_maze.map_memory import MapMemory


def test_is_lateral_pin_true_for_open_front_side_pin():
    m = MapMemory(FloodFillBrain())
    assert m.is_lateral_pin(perp_front=4.0, near=0.31, cross_track=1.22, safety_radius=0.70) is True


def test_is_lateral_pin_false_for_real_front_wall():
    m = MapMemory(FloodFillBrain())
    assert m.is_lateral_pin(perp_front=0.6, near=0.31, cross_track=1.22, safety_radius=0.70) is False


def test_is_lateral_pin_false_when_not_near_a_wall():
    m = MapMemory(FloodFillBrain())
    assert m.is_lateral_pin(perp_front=4.0, near=0.85, cross_track=1.22, safety_radius=0.70) is False


def test_is_lateral_pin_false_when_on_centerline():
    m = MapMemory(FloodFillBrain())
    assert m.is_lateral_pin(perp_front=4.0, near=0.31, cross_track=0.10, safety_radius=0.70) is False


def test_mark_wall_on_failure_suppresses_lateral_pin():
    b = FloodFillBrain(); m = MapMemory(b)
    marked = m.mark_wall_on_failure((4, 9), 'E', perp_front=4.0, near=0.31,
                                    cross_track=1.22, safety_radius=0.70)
    assert marked is False
    assert b.is_wall((4, 9), 'E') is False
    assert m.suppressed == 1


def test_mark_wall_on_failure_marks_genuine_front_block():
    b = FloodFillBrain(); m = MapMemory(b)
    marked = m.mark_wall_on_failure((4, 9), 'N', perp_front=0.5, near=0.31,
                                    cross_track=1.22, safety_radius=0.70)
    assert marked is True
    assert b.is_wall((4, 9), 'N') is True
    assert m.suppressed == 0


def test_reconcile_no_snap_before_persist():
    m = MapMemory(FloodFillBrain(), reconcile_persist_s=8.0)
    m.observe((4, 9), 9.23, 18.0, 100.0)                       # odom cell (5,9) != planner (4,9)
    assert m.reconcile_target((4, 9), 9.23, 18.0, 105.0) == (4, 9)   # 5s < 8s -> no snap
    assert m.reconciles == 0


def test_reconcile_snaps_after_persist():
    m = MapMemory(FloodFillBrain(), reconcile_persist_s=8.0)
    m.observe((4, 9), 9.23, 18.0, 100.0)
    m.observe((4, 9), 9.23, 18.0, 108.5)
    assert m.reconcile_target((4, 9), 9.23, 18.0, 108.5) == (5, 9)
    assert m.reconciles == 1


def test_observe_resets_desync_when_synced():
    m = MapMemory(FloodFillBrain(), reconcile_persist_s=8.0)
    m.observe((4, 9), 9.23, 18.0, 100.0)                       # desync starts
    m.observe((5, 9), 9.23, 18.0, 101.0)                       # planner==odom -> reset
    assert m.reconcile_target((5, 9), 9.23, 18.0, 110.0) == (5, 9)   # synced -> no snap
    assert m.reconciles == 0
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_map_memory.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'tugbot_maze.map_memory'`.

- [ ] **Step 3: Write the module** — create `tugbot_maze/map_memory.py`:

```python
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
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_map_memory.py -q`
Expected: PASS (9 passed).

- [ ] **Step 5: Commit**

```bash
cd "$PKG" && git add tugbot_maze/map_memory.py test/test_map_memory.py
git commit -m "feat: MapMemory gateway (lateral-pin mark-guard + odom reconcile)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Active keep-out repulsion + radius 0.70 (`hop_controller.py`)

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/hop_controller.py`
- Test: `src/tugbot_maze/test/test_hop_controller.py`

- [ ] **Step 1: Write the failing tests** — append to `test/test_hop_controller.py`:

```python
def test_repulsion_steers_away_from_near_right_wall():
    import math
    from tugbot_maze.hop_controller import corridor_follow_command
    yaw = math.pi / 2; cardinal = math.pi / 2                      # N travel; for N: left=W, right=E
    _, w0 = corridor_follow_command(yaw, cardinal, 0.85, 0.85, 0.85, safety_radius=0.70)  # centered
    _, w1 = corridor_follow_command(yaw, cardinal, 0.85, 0.30, 0.30, safety_radius=0.70)  # near right (E)
    assert w1 > w0                                                 # near right -> steer LEFT (+w, CCW toward W)


def test_repulsion_steers_away_from_near_left_wall():
    import math
    from tugbot_maze.hop_controller import corridor_follow_command
    yaw = math.pi / 2; cardinal = math.pi / 2
    _, w0 = corridor_follow_command(yaw, cardinal, 0.85, 0.85, 0.85, safety_radius=0.70)
    _, w1 = corridor_follow_command(yaw, cardinal, 0.30, 0.85, 0.30, safety_radius=0.70)  # near left (W)
    assert w1 < w0                                                 # near left -> steer RIGHT (-w)


def test_repulsion_noop_when_centered_and_clear():
    import math
    from tugbot_maze.hop_controller import corridor_follow_command
    yaw = math.pi / 2; cardinal = math.pi / 2
    v, w = corridor_follow_command(yaw, cardinal, 0.88, 0.88, 0.88, safety_radius=0.70)
    assert abs(w) < 1e-9 and v > 0.0                               # both sides > radius -> bias 0


def test_repulsion_stays_within_cross_track_envelope():
    import math
    from tugbot_maze.hop_controller import corridor_follow_command
    yaw = math.pi / 2; cardinal = math.pi / 2
    # huge gain + a wall almost touching: cross is clamped to max_cross_track_m, so |w| <= w_max
    _, w = corridor_follow_command(yaw, cardinal, 0.85, 0.05, 0.05, safety_radius=0.70,
                                   keepout_repulse_gain=5.0, max_cross_track_m=0.6, w_max=0.5)
    assert abs(w) <= 0.5 + 1e-9
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_hop_controller.py -k repulsion -v`
Expected: FAIL — `corridor_follow_command() got an unexpected keyword argument 'keepout_repulse_gain'` (and the directional asserts fail because there is no repulsion yet).

- [ ] **Step 3: Implement** — in `hop_controller.py`:

3a. Bump the `safety_radius` default `0.60 -> 0.70` in BOTH `corridor_drive_command` (line ~144) and `corridor_follow_command` (line ~176). Each currently reads:
```python
                           safety_radius: float = 0.60, keepout_max_cross_steer: float = 0.8
```
Change the literal to `0.70` in both.

3b. Add the `keepout_repulse_gain` kwarg to `corridor_follow_command`'s signature (line ~176-177), so it ends:
```python
                            safety_radius: float = 0.70, keepout_max_cross_steer: float = 0.8,
                            keepout_repulse_gain: float = 0.6
                            ) -> Tuple[float, float]:
```

3c. Replace the body of `corridor_follow_command` between the docstring and the `return corridor_drive_command(...)` call. The current body is:
```python
    cross = centerline_cross(d_left, d_right, fallback_cross=fallback_cross,
                             wall_seen_m=wall_seen_m, half_corridor_m=half_corridor_m)
    cross = max(-max_cross_track_m, min(max_cross_track_m, cross))
```
Replace with:
```python
    cross = centerline_cross(d_left, d_right, fallback_cross=fallback_cross,
                             wall_seen_m=wall_seen_m, half_corridor_m=half_corridor_m)
    # Active keep-out repulsion: a side wall within safety_radius pushes the centerline target
    # AWAY from it (toward the open side), proportional to the intrusion depth. centerline_cross
    # only converges to CENTER; this repels so the 0.35 m robot won't graze the corner while
    # creeping through a doorway. cross is +left-of-centre and the follower steers to reduce it,
    # so near-LEFT (d_left<d_right) adds +bias (steer right, away); near-RIGHT adds -bias.
    near_side = min(d_left, d_right)
    if near_side < safety_radius:
        bias = keepout_repulse_gain * (safety_radius - near_side)
        cross += bias if d_left < d_right else -bias
    cross = max(-max_cross_track_m, min(max_cross_track_m, cross))
```
(The existing `return corridor_drive_command(...)` call is unchanged — it already forwards `safety_radius` and `keepout_max_cross_steer`.)

- [ ] **Step 4: Run the new tests, then the whole file**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_hop_controller.py -k repulsion -v`
Expected: PASS (4 passed).

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_hop_controller.py -q`
Expected: PASS. If a pre-existing keep-out test now fails because it relied on the old `0.60` default (e.g. a case with a side wall between 0.60 and 0.70, or one asserting an exact `w` that the repulsion now shifts), make it explicit: pass `safety_radius=0.60` and/or `keepout_repulse_gain=0.0` to that call so it tests the original behavior, OR update its expected value if the new repulsion is correct for that scenario. Do not weaken an assertion to make it pass — pin the parameters instead.

- [ ] **Step 5: Commit**

```bash
cd "$PKG" && git add tugbot_maze/hop_controller.py test/test_hop_controller.py
git commit -m "feat: active keep-out repulsion + safety radius 0.70

Steer away from a near side wall proportional to intrusion (not just cap
cross-steer and slow), and widen the keep-out envelope 0.60->0.70.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Wire `MapMemory` + repulsion into `MazeMotion`

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/maze_motion.py`
- Test: `src/tugbot_maze/test/test_maze_motion.py`

- [ ] **Step 1: Write the failing tests** — append to `test/test_maze_motion.py`:

```python
def test_center_reconciles_cell_to_odom_after_persistent_desync():
    import math
    from tugbot_maze.maze_motion import MazeMotion
    m = MazeMotion()
    m.cell = (4, 9); m.phase = 'center'
    n = 16; scan = ([3.0] * n, -math.pi, 2 * math.pi / n)         # open all around (no walls)
    m.mem.observe((4, 9), 9.23, 18.0, 100.0)                      # desync starts at t=100 (odom (5,9))
    m._center((9.23, 18.0, math.pi / 2), scan, 100.0 + m.mem.reconcile_persist_s + 0.5)
    assert m.cell == (5, 9)                                       # snapped to odom
    assert any(e.startswith("RECONCILE") for e in m.events)


def test_drive_giveup_routes_wall_mark_through_mapmemory():
    import math
    from tugbot_maze.maze_motion import MazeMotion
    from tugbot_maze.map_memory import MapMemory
    calls = []

    class Spy(MapMemory):
        def mark_wall_on_failure(self, cell, d, **kw):
            calls.append((cell, d)); return False                 # simulate suppression

    m = MazeMotion()
    m.mem = Spy(m.brain)
    m.cell = (4, 9); m.hop_dir = (1, 0); m.hop_target = (5, 9); m.target_cardinal = 0.0  # E hop
    m.phase = 'drive'; m.hop_start = (8.0, 18.0); m.hop_deadline = 1e12
    m.progress_pose = (8.2, 18.0); m.progress_t = 0.0
    m.visited = {(4, 9)}                                          # (5,9) NOT visited -> giveup attempts a mark
    m.hop_attempts[((4, 9), 'E')] = m.max_hop_attempts - 1        # next failure marks
    n = 16; scan = ([3.0] * n, -math.pi, 2 * math.pi / n)
    m._drive((8.2, 18.0, 0.0), scan, m.wedge_detect_s + 5.0)      # no progress, aligned -> wedge giveup
    assert calls == [((4, 9), 'E')]                               # routed through the gateway
    assert m.brain.is_wall((4, 9), 'E') is False                 # suppressed -> not walled


def test_drive_giveup_marks_wall_when_not_lateral_pin():
    import math
    from tugbot_maze.maze_motion import MazeMotion
    m = MazeMotion()
    m.cell = (4, 9); m.hop_dir = (1, 0); m.hop_target = (5, 9); m.target_cardinal = 0.0
    m.phase = 'drive'; m.hop_start = (8.0, 18.0); m.hop_deadline = 1e12
    m.progress_pose = (8.2, 18.0); m.progress_t = 0.0
    m.visited = {(4, 9)}
    m.hop_attempts[((4, 9), 'E')] = m.max_hop_attempts - 1
    n = 16; scan = ([3.0] * n, -math.pi, 2 * math.pi / n)         # near ~3.0 > safety -> NOT a pin
    m._drive((8.2, 18.0, 0.0), scan, m.wedge_detect_s + 5.0)
    assert m.brain.is_wall((4, 9), 'E') is True                  # real giveup -> marked
    assert m.mem.suppressed == 0
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py -k "reconcile or giveup_routes or not_lateral_pin" -v`
Expected: FAIL — `AttributeError: 'MazeMotion' object has no attribute 'mem'`.

- [ ] **Step 3a: Import `MapMemory`** — in `maze_motion.py`, after the `from tugbot_maze.hop_controller import (...)` block (line ~30), add:
```python
from tugbot_maze.map_memory import MapMemory
```

- [ ] **Step 3b: Accept and store `mem`** — in `MazeMotion.__init__`, add a keyword-only param `mem` (put it right after `grid_fallback_max_m: float = 0.40` in the signature, line ~72):
```python
                 grid_fallback_max_m: float = 0.40,
                 mem: Optional['MapMemory'] = None):
```
Then, immediately after `self.brain = brain if brain is not None else FloodFillBrain(exit_cell=EXIT_CELL)` (line ~73), add:
```python
        self.mem = mem if mem is not None else MapMemory(self.brain)
```

- [ ] **Step 3c: Bump radius + add repulse gain** — in `__init__`, change line ~98 `self.safety_radius = 0.60` to `0.70`, and immediately after the `self.keepout_max_cross_steer = 0.8` line (~99) add:
```python
        self.keepout_repulse_gain = 0.6      # active keep-out: steer-away strength within safety_radius
```

- [ ] **Step 3d: Observe every tick** — in `step`, change the `self._track_cell(t)` line (~145) to:
```python
        self._track_cell(t)
        self.mem.observe(self.cell, pose[0], pose[1], t)
```

- [ ] **Step 3e: Reconcile at the top of `_center`** — in `_center`, immediately after `ranges, amin, ainc = scan` (line ~171) and BEFORE the committed-cell fast-path comment, insert:
```python
        target = self.mem.reconcile_target(self.cell, x, y, t)
        if target != self.cell:
            self.events.append("RECONCILE %s -> %s (desync>%.0fs)"
                               % (self.cell, target, self.mem.reconcile_persist_s))
            self.cell = target                              # relabel only; committed wall-knowledge unchanged
            self.hop_attempts.clear(); self.hop_dir = None
            self.center_start = None; self.align_start = None; self.latched_cardinal = None
```

- [ ] **Step 3f: Route both `_drive` giveup marks through the gateway** — the two giveup sites (wedge ~line 532-540, front_block/deadline ~560-567) share an IDENTICAL inner block. Replace it in BOTH places (use `replace_all`). Old block (appears twice):
```python
                if nb not in self.visited:                           # never wall an edge to a VISITED cell
                    self.brain.mark(self.cell, dirn, is_wall=True)
                    self._stamp_loco_wall(self.cell, dirn)           # re-openable by _unstick (both reps)
                    self.committed.discard(self.cell)                # un-commit; cell stays in `sensed`
```
New block:
```python
                if nb not in self.visited:                           # never wall an edge to a VISITED cell
                    ct = grid_cross_track(x, y, self.cell, self.hop_dir, cell_size_m=CELL_SIZE_M)
                    if self.mem.mark_wall_on_failure(self.cell, dirn, perp_front=perp[dirn],
                                                     near=near, cross_track=ct,
                                                     safety_radius=self.safety_radius):  # suppress lateral pins
                        self._stamp_loco_wall(self.cell, dirn)       # re-openable by _unstick (both reps)
                        self.committed.discard(self.cell)            # un-commit; cell stays in `sensed`
```
(`x, y, perp, near` are all in scope in `_drive`; `grid_cross_track`, `CELL_SIZE_M`, `DIRS` are already imported.)

- [ ] **Step 3g: Thread `keepout_repulse_gain` into the follower call** — in `_drive`, the `corridor_follow_command(...)` call (line ~582-591), add the kwarg to the end of the call:
```python
                                       safety_radius=self.safety_radius,
                                       keepout_max_cross_steer=self.keepout_max_cross_steer,
                                       keepout_repulse_gain=self.keepout_repulse_gain)
```

- [ ] **Step 4: Run the new tests, then the full regression**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py -k "reconcile or giveup_routes or not_lateral_pin" -v`
Expected: PASS (3 passed).

Run the full regression suite (command in the header).
Expected: PASS. If a `test_maze_motion.py` test asserted `self.safety_radius == 0.60`, update it to `0.70`. If an offline-sim case in `test_maze_motion_sim.py` regresses, investigate before weakening any assertion — the esc/desync/collision invariants are the gate.

- [ ] **Step 5: Commit**

```bash
cd "$PKG" && git add tugbot_maze/maze_motion.py test/test_maze_motion.py
git commit -m "feat: wire MapMemory reconcile + mark-guard and repulsion into MazeMotion

observe() every tick, reconcile_target() snap at the top of _center,
mark-guard at both _drive giveup sites, repulse-gain threaded to the
corridor follower, safety_radius 0.70.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: Construct `MapMemory` in the node + log observables

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/flood_fill_solver.py`

- [ ] **Step 1: Read the node** to find the brain/MazeMotion construction and the DIAG log line.

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -c "import ast,sys; print(open('tugbot_maze/flood_fill_solver.py').read())" | head -120`
Locate: (a) the `FloodFillBrain(...)` + `MazeMotion(...)` construction (around lines 57-64), and (b) the line that emits `DIAG pose=... dcell=... odomcell=... dist_to_exit=... phase=...`.

- [ ] **Step 2: Add the import** — with the other `tugbot_maze` imports at the top of `flood_fill_solver.py`, add:
```python
from tugbot_maze.map_memory import MapMemory
```

- [ ] **Step 3: Construct `MapMemory` and pass it to `MazeMotion`** — at the construction site, change (adapt to the exact existing variable names found in Step 1):
```python
        self.brain = FloodFillBrain(exit_cell=EXIT_CELL)
        self.motion = MazeMotion(self.brain, ...)
```
to:
```python
        self.brain = FloodFillBrain(exit_cell=EXIT_CELL)
        self.mem = MapMemory(self.brain)
        self.motion = MazeMotion(self.brain, ..., mem=self.mem)
```
(Keep all existing `MazeMotion(...)` kwargs; just add `mem=self.mem`.)

- [ ] **Step 4: Append observables to the DIAG line** — extend the DIAG format string and its args with the two counters, e.g. append ` mem_supp=%d mem_rec=%d` to the format and `self.motion.mem.suppressed, self.motion.mem.reconciles` to the args. The final DIAG line should read like:
```
DIAG pose=(..) dcell=(..) odomcell=(..) dist_to_exit=.. phase=../.. mem_supp=N mem_rec=N
```

- [ ] **Step 5: Verify import + build + full regression**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -c "import tugbot_maze.flood_fill_solver"`
Expected: no error (module imports; ROS imports may warn but must not fail under the workspace env — if `rclpy` import fails outside a sourced env, instead assert the module parses: `python3 -m py_compile tugbot_maze/flood_fill_solver.py`).

Run the full regression suite (header command). Expected: PASS.

Run the colcon build:
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260614 \
  && source /opt/ros/jazzy/setup.bash && colcon build --packages-select tugbot_maze --symlink-install 2>&1 | tail -4
```
Expected: `Finished <<< tugbot_maze`.

- [ ] **Step 6: Commit**

```bash
cd "$PKG" && git add tugbot_maze/flood_fill_solver.py
git commit -m "feat: construct MapMemory in the node and log mem_supp/mem_rec in DIAG

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: Final regression gate + validation handoff

**Files:** none (verification only).

- [ ] **Step 1: Full regression suite** — run the header command. Expected: ALL PASS. Record the pass count.

- [ ] **Step 2: Sanity — installed module reflects the change**

Run:
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260614
grep -n "keepout_repulse_gain\|self.safety_radius = 0.70" build/tugbot_maze/tugbot_maze/maze_motion.py | head
grep -n "class MapMemory" build/tugbot_maze/tugbot_maze/map_memory.py
```
Expected: the repulse gain + `safety_radius = 0.70` present, and `MapMemory` installed.

- [ ] **Step 3: Hand off for Gazebo validation** — do NOT auto-launch. Report readiness and the expected outcomes for the user-initiated run (diagnostics IN): collision rate well below 33 %, desync well below 54 %, the `(4,9)`-class doorway resolved by a RECONCILE within ~8 s (watch `mem_rec`), false walls avoided (`mem_supp`), and depth at least matching the 8.39 m record — ideally `EXIT_REACHED`. The run command (when the user clears sims and confirms):
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260614 \
  && export DISPLAY=:1 && bash tools/run_flood_fill_maze.sh 1800 false true odom_locked true
```

---

## Self-review notes (author)
- **Spec coverage:** Component 1 (repulsion + radius) → Task 2 + Task 3c/3g. Component 2a (reconcile) → Task 1 + Task 3d/3e. Component 2b (mark-guard) → Task 1 + Task 3f. Node ownership + DIAG observables → Task 4. All spec sections map to a task.
- **Type/name consistency:** `MapMemory(brain, *, front_open_m, pin_cross_m, reconcile_persist_s)`; `observe(dcell,x,y,t)`, `reconcile_target(dcell,x,y,t)`, `mark_wall_on_failure(cell,d,*,perp_front,near,cross_track,safety_radius)`, `is_lateral_pin(perp_front,near,cross_track,safety_radius)` — identical across Tasks 1, 3, 4. `keepout_repulse_gain` identical in hop_controller (Task 2) and MazeMotion (Task 3).
- **No placeholders:** every code/step is concrete. The only "adapt to existing names" is Task 4 Step 3 (the node's exact MazeMotion kwargs are not in the plan's context — Step 1 reads them first).
