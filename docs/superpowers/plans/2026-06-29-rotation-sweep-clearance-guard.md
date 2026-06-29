# Rotation-Sweep Clearance Guard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Guarantee the motion FSM never rotates the true footprint into a known wall: gate the in-place turn on a swept-clearance check, and when a turn would graze, translate toward the known cell center (collision-safe) first. Eliminates the residual cell-(3,9)-type rear-corner graze.

**Architecture:** A new ROS-free `collision_geometry.py` holds the single source of footprint-vs-wall collision truth (extracted from `maze_sim.collides`), exposing `rect_hits_segments` + `FootprintClearance.collides/.swept_collides`. `maze_sim.collides` is refactored to call it (behavior-identical). `MazeMotion` gains a `segments` arg → builds a `FootprintClearance`, gates `_route`'s turn-entry, and adds a `preturn` make-room phase. `flood_fill_solver` passes `load_segments()` in. `segments=None` keeps every existing caller unchanged (gate is a no-op).

**Tech Stack:** Python 3.12, NumPy, pytest. No new deps.

**Spec:** `docs/superpowers/specs/2026-06-29-rotation-sweep-clearance-guard-design.md`

**Branch:** `rotation-sweep-clearance-guard` (off main). **Test cmd (from package root):**
`cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest <path> -v`

**Commit convention:** on branch `rotation-sweep-clearance-guard`; message ends with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`; NO backticks in `git commit -m`; stage ONLY the task's files.

---

## File Structure

- **Create** `src/tugbot_maze/tugbot_maze/collision_geometry.py` — shared collision core + `FootprintClearance`.
- **Create** `src/tugbot_maze/test/test_collision_geometry.py` — equivalence + ground-truth + swept tests.
- **Modify** `src/tugbot_maze/tugbot_maze/maze_sim.py` — `collides` yaw-branch calls `rect_hits_segments`.
- **Modify** `src/tugbot_maze/tugbot_maze/maze_motion.py` — `segments` arg, `_route` gate, `_preturn` phase.
- **Modify** `src/tugbot_maze/test/test_maze_motion_sim.py` — reproduce-then-fix + regression.
- **Modify** `src/tugbot_maze/tugbot_maze/flood_fill_solver.py` — pass `load_segments()` to `MazeMotion`.

Paths relative to repo root `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`.

---

## Task 1: `collision_geometry.py` core + refactor `maze_sim.collides`

**Files:**
- Create: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/collision_geometry.py`
- Create: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_collision_geometry.py`
- Modify: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_sim.py`

- [ ] **Step 1: Write the failing test** — `test/test_collision_geometry.py`:

```python
import math
import numpy as np
from tugbot_maze.collision_geometry import rect_hits_segments, FootprintClearance
from tugbot_maze.maze_sim import MazeSim, load_segments
from tugbot_maze.footprint import FOOT_X_FRONT, FOOT_X_REAR, FOOT_HALF_W


def test_clearance_collides_matches_maze_sim_oracle():
    """FootprintClearance.collides == MazeSim.collides (the refactor is the same truth)."""
    segs = load_segments()
    fc = FootprintClearance(segs)
    sim = MazeSim(segs, (0.0, 0.0), 0.0)
    rng = np.random.default_rng(0)
    mismatches = 0
    for _ in range(2000):
        x = rng.uniform(0.0, 21.0); y = rng.uniform(0.0, 19.0); yaw = rng.uniform(-math.pi, math.pi)
        if fc.collides(x, y, yaw) != sim.collides(x, y, yaw):
            mismatches += 1
    assert mismatches == 0


def test_cell_3_9_center_is_rotation_safe_but_offcenter_grazes():
    """Ground truth from the investigation: center (6,18) clears all headings; 0.4 m north grazes."""
    fc = FootprintClearance(load_segments())
    assert not any(fc.collides(6.0, 18.0, math.radians(h)) for h in range(-180, 180, 5))
    assert not fc.collides(6.0, 18.2, math.radians(-120))      # 0.2 m off still clear
    assert fc.collides(6.04, 18.40, math.radians(-120))        # 0.4 m off -> rear corner grazes


def test_swept_collides_center_clear_offcenter_grazes():
    fc = FootprintClearance(load_segments())
    # full turn at center is clear for any target
    assert not fc.swept_collides(6.0, 18.0, math.pi / 2, -2.4)
    # the observed off-center turn grazes somewhere along the arc
    assert fc.swept_collides(6.04, 18.40, math.pi / 2, -2.4)


def test_swept_collides_detects_midarc_only_collision():
    """A start and end both clear, but a mid-arc heading collides -> swept must catch it."""
    fc = FootprintClearance(load_segments())
    x, y = 6.04, 18.40
    # pick endpoints that individually clear but whose arc passes through the grazing band
    assert not fc.collides(x, y, math.radians(90))
    assert not fc.collides(x, y, math.radians(10))
    assert fc.swept_collides(x, y, math.radians(90), math.radians(-150))
```

- [ ] **Step 2: Run to verify it fails** — `cd .../src/tugbot_maze && python3 -m pytest test/test_collision_geometry.py -v`
Expected: FAIL `ModuleNotFoundError: No module named 'tugbot_maze.collision_geometry'`.

- [ ] **Step 3: Implement** — create `tugbot_maze/collision_geometry.py`:

```python
"""Footprint-vs-wall collision geometry: the single source of collision truth, shared by the
offline maze_sim and the runtime rotation-sweep guard. ROS-free, pure NumPy.

rect_hits_segments tests the TRUE asymmetric footprint rectangle (base_link box
[x0,x1]x[y0,y1]) at pose (x,y,yaw) against wall-segment centerlines: collision iff the body
comes within half_thickness of a centerline (exact -- Liang-Barsky intersection, else min
distance). Moved verbatim from maze_sim.collides' yaw-given branch.
"""
from __future__ import annotations
import math

import numpy as np

from tugbot_maze.footprint import FOOT_X_FRONT, FOOT_X_REAR, FOOT_HALF_W


def rect_hits_segments(segs, x, y, yaw, x0, x1, y0, y1, half_thickness) -> bool:
    segs = np.asarray(segs, dtype=float).reshape(-1, 4)
    if segs.shape[0] == 0:
        return False
    c, s = math.cos(yaw), math.sin(yaw)
    ax = segs[:, 0] - x; ay = segs[:, 1] - y
    bx = segs[:, 2] - x; by = segs[:, 3] - y
    Ax = c * ax + s * ay; Ay = -s * ax + c * ay          # seg endpoint A in base_link
    Bx = c * bx + s * by; By = -s * bx + c * by          # seg endpoint B in base_link
    dx = Bx - Ax; dy = By - Ay
    # (1) Liang-Barsky: does the segment intersect the rectangle? -> distance 0
    t0 = np.zeros(Ax.shape); t1 = np.ones(Ax.shape)
    intersects = np.ones(Ax.shape, dtype=bool)
    for pi, qi in ((-dx, Ax - x0), (dx, x1 - Ax), (-dy, Ay - y0), (dy, y1 - Ay)):
        parallel = np.abs(pi) < 1e-12
        intersects &= ~(parallel & (qi < 0))             # parallel & outside slab -> miss
        with np.errstate(divide='ignore', invalid='ignore'):
            r = qi / pi
        t0 = np.where((pi < 0) & ~parallel, np.maximum(t0, r), t0)
        t1 = np.where((pi > 0) & ~parallel, np.minimum(t1, r), t1)
    intersects &= (t0 <= t1)
    # (2) disjoint case: min of seg-endpoints-to-box and box-corners-to-seg
    def _pt_to_box(px, py):
        ox = np.maximum(np.maximum(x0 - px, 0.0), px - x1)
        oy = np.maximum(np.maximum(y0 - py, 0.0), py - y1)
        return np.hypot(ox, oy)

    def _corner_to_seg(cx, cy):
        wx = cx - Ax; wy = cy - Ay
        L2 = dx * dx + dy * dy
        tt = np.where(L2 > 1e-12, (wx * dx + wy * dy) / np.maximum(L2, 1e-12), 0.0)
        tt = np.clip(tt, 0.0, 1.0)
        return np.hypot(cx - (Ax + tt * dx), cy - (Ay + tt * dy))

    mind = np.minimum.reduce([_pt_to_box(Ax, Ay), _pt_to_box(Bx, By),
                              _corner_to_seg(x0, y0), _corner_to_seg(x1, y0),
                              _corner_to_seg(x1, y1), _corner_to_seg(x0, y1)])
    mind = np.where(intersects, 0.0, mind)
    return bool(np.any(mind < half_thickness))


class FootprintClearance:
    """Runtime clearance checker over a fixed wall map and the true footprint box."""

    def __init__(self, segments, *, wall_half_thickness_m: float = 0.12,
                 x0: float = FOOT_X_REAR, x1: float = FOOT_X_FRONT,
                 y0: float = -FOOT_HALF_W, y1: float = FOOT_HALF_W):
        self._segs = np.asarray(segments, dtype=float).reshape(-1, 4)
        self.m = float(wall_half_thickness_m)
        self.x0, self.x1, self.y0, self.y1 = x0, x1, y0, y1

    def collides(self, x, y, yaw) -> bool:
        return rect_hits_segments(self._segs, x, y, yaw, self.x0, self.x1, self.y0, self.y1, self.m)

    def swept_collides(self, x, y, yaw0, yaw1, *, step_rad: float = math.radians(5.0)) -> bool:
        """True if the footprint collides at any heading along the SHORTEST arc yaw0->yaw1."""
        err = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))   # shortest signed arc
        n = max(1, int(math.ceil(abs(err) / step_rad)))
        for i in range(n + 1):
            if self.collides(x, y, yaw0 + err * (i / n)):
                return True
        return False
```

- [ ] **Step 4: Run to verify it passes** — `cd .../src/tugbot_maze && python3 -m pytest test/test_collision_geometry.py -v`
Expected: 4 passed. (If `test_swept_collides_detects_midarc_only_collision`'s chosen endpoints don't bracket a grazing mid-arc, adjust ONLY those two heading constants so the two `collides` are False and the `swept_collides` is True — the investigation shows the grazing band is roughly [-145°,-100°] ∪ [-80°,-35°] at (6.04,18.40).)

- [ ] **Step 5: Refactor `maze_sim.collides` to use the shared core (behavior-identical).**

In `tugbot_maze/maze_sim.py`, add to the imports near the top (after the existing `from tugbot_maze.footprint import ...`):
```python
from tugbot_maze.collision_geometry import rect_hits_segments
```
Then in `MazeSim.collides`, replace the ENTIRE yaw-given branch — the block starting at the comment `# yaw given: EXACT min distance ...` through the final `return bool(np.any(mind < m))` — with:
```python
        # yaw given: TRUE asymmetric footprint rectangle vs wall centerlines (shared core).
        return rect_hits_segments(self.segs, x, y, yaw, FOOT_X_REAR, FOOT_X_FRONT,
                                  -FOOT_HALF_W, FOOT_HALF_W, m)
```
(The `yaw is None` bounding-circle branch and `m = self.wall_half_thickness_m` above it are unchanged.)

- [ ] **Step 6: Verify the refactor is behavior-identical** — run the collision-geometry tests AND the existing maze_sim suite:
`cd .../src/tugbot_maze && python3 -m pytest test/test_collision_geometry.py test/test_maze_sim.py -v`
Expected: all pass (the equivalence test confirms identical truth; maze_sim's own tests stay green).

- [ ] **Step 7: Commit**
```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/collision_geometry.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_collision_geometry.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_sim.py
git commit -m "feat: shared collision_geometry (footprint vs walls) + swept_collides; maze_sim refactor

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: `MazeMotion` swept-clearance gate + `_preturn` make-room phase

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_motion.py`
- Test: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_motion_sim.py`

**Context:** `_route(self, x, y, t)` sets up the hop and at line 361 does `self.phase = 'turn'`. It has no `yaw` arg, but `step()` stores the current yaw in `self.prev_yaw` (line 148) before dispatch, so the gate uses `self.prev_yaw`. `CELL_SIZE_M` is already imported. Cell center = `(CELL_SIZE_M*cx, CELL_SIZE_M*cy)`. Existing callers that construct `MazeMotion()` without `segments` must be unaffected (`self.clearance is None` → gate skipped).

- [ ] **Step 1: Write the failing test** — append to `test/test_maze_motion_sim.py`:

```python
from tugbot_maze.collision_geometry import FootprintClearance


def test_preturn_makes_room_then_turns_without_grazing():
    """Robot 0.4 m north of center in boundary cell (3,9), facing north (arrival heading),
    with a pending turn to a heading whose sweep grazes the north wall. The preturn guard must
    translate to clearance and complete the turn with ZERO collision."""
    segs = load_segments()
    assert FootprintClearance(segs).swept_collides(6.04, 18.40, math.pi / 2, -2.4)  # baseline grazes
    sim = MazeSim(segs, (6.04, 18.40), math.pi / 2, inertia=True)
    m = MazeMotion(segments=segs)
    m.cell = (3, 9); m.target_cardinal = -2.4; m.phase = 'preturn'; m.preturn_start = 0.0
    t = 0.0
    collided = False
    reached = False
    for _ in range(600):
        scan = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        v, w, _done = m.step(sim.reported_pose, scan, t)
        sim.step(v, w, 0.1)
        if sim.collides(sim.x, sim.y, sim.yaw):
            collided = True
        if abs(_wrap_pi(sim.yaw - (-2.4))) <= 0.10:    # reached target heading at some point
            reached = True
        if m.phase == 'drive':                          # turn finished -> moving on
            break
        t += 0.1
    assert not collided, "footprint grazed a wall during the guarded boundary turn"
    assert reached, "did not complete the turn to the target heading"


def _wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))
```

- [ ] **Step 2: Run to verify it fails** — `cd .../src/tugbot_maze && python3 -m pytest test/test_maze_motion_sim.py -k preturn -v`
Expected: FAIL — `MazeMotion()` has no `segments` kwarg (TypeError) / no `preturn` phase.

- [ ] **Step 3: Implement — `__init__` segments + params.**

In `maze_motion.py`, add to the imports (after the `from tugbot_maze.map_memory import MapMemory` line):
```python
from tugbot_maze.collision_geometry import FootprintClearance
```
Add params to `__init__` signature (just before `mem: Optional['MapMemory'] = None):`):
```python
                 preturn_v: float = 0.12, preturn_timeout_s: float = 4.0,
                 sweep_step_rad: float = math.radians(5.0), segments=None,
```
In `__init__` body, after the line `self.grid_fallback_max_m = grid_fallback_max_m   # clamp on the open-junction odom fallback`, add:
```python
        self.preturn_v = preturn_v
        self.preturn_timeout_s = preturn_timeout_s
        self.sweep_step_rad = sweep_step_rad
        self.preturn_start = None
        self.clearance = FootprintClearance(segments) if segments is not None else None
```

- [ ] **Step 4: Implement — dispatch + `_route` gate + `_preturn`.**

In `step()`, add a dispatch line right after the `if self.phase == 'turn': return self._turn(pose, t)` line:
```python
        if self.phase == 'preturn':
            return self._preturn(pose, t)
```
In `_route`, replace the turn-entry tail:
```python
        self.center_start = None
        self.align_start = None; self.latched_cardinal = None
        self.phase = 'turn'
        return (0.0, 0.0, False)
```
with:
```python
        self.center_start = None
        self.align_start = None; self.latched_cardinal = None
        # Rotation-sweep clearance guard: never rotate the true footprint into a known wall.
        if (self.clearance is not None and self.clearance.swept_collides(
                x, y, self.prev_yaw, self.target_cardinal, step_rad=self.sweep_step_rad)):
            self.preturn_start = t
            self.phase = 'preturn'
            return (0.0, 0.0, False)
        self.phase = 'turn'
        return (0.0, 0.0, False)
```
Add the `_preturn` method (place it right after `_route`):
```python
    def _preturn(self, pose, t):
        """Make room before a turn that would sweep the footprint into a wall: translate ALONG
        the current heading toward the known cell center (no rotation -> collision-safe), then
        re-gate. Bounded; on timeout fall back to _unstick rather than force a grazing turn.
        Rotating at the cell center is geometrically always safe, so this converges."""
        x, y, yaw = pose
        if not self.clearance.swept_collides(x, y, yaw, self.target_cardinal,
                                             step_rad=self.sweep_step_rad):
            self.turn_in_tol = 0
            self.turn_start = t
            self.phase = 'turn'
            return (0.0, 0.0, False)
        if self.preturn_start is None:
            self.preturn_start = t
        if t - self.preturn_start > self.preturn_timeout_s:
            self.events.append('PRETURN exhausted -> unstick cell=%s' % (self.cell,))
            return self._unstick(t)
        cx_ctr = CELL_SIZE_M * self.cell[0]; cy_ctr = CELL_SIZE_M * self.cell[1]
        s = (cx_ctr - x) * math.cos(yaw) + (cy_ctr - y) * math.sin(yaw)   # along-heading dist to center
        if abs(s) < 0.02:                          # at center on this axis but still blocked (rare
            return (0.0, 0.0, False)               # perpendicular case) -> wait for timeout -> unstick
        v = self.preturn_v if s > 0 else -self.preturn_v
        return (v, 0.0, False)
```

- [ ] **Step 5: Run to verify it passes** — `cd .../src/tugbot_maze && python3 -m pytest test/test_maze_motion_sim.py -k preturn -v`
Expected: PASS (1 passed). If `reached` fails because the robot reverses past center and stalls, confirm `s` flips sign correctly (reverse when center is behind); if collisions appear, the gate/`swept_collides` is being bypassed — verify `self.clearance` is built and `self.prev_yaw` is set. Do NOT weaken the `not collided` assertion.

- [ ] **Step 6: Regression — existing motion + scan_match tests unaffected** (they construct `MazeMotion()` with no segments → gate is a no-op):
`cd .../src/tugbot_maze && python3 -m pytest test/test_maze_motion_sim.py -q`
Expected: all pass; the 3 `[0.05-*]` xfails remain xfailed; the new preturn test passes.

- [ ] **Step 7: Commit**
```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_motion.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_motion_sim.py
git commit -m "feat: rotation-sweep clearance gate + preturn make-room phase in MazeMotion

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Wire the known map into the node

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/flood_fill_solver.py`

**Context:** `MazeMotion` is constructed at `flood_fill_solver.py:60-66`; `load_segments` is already imported (Task 4 of the localization work). Passing `segments=load_segments()` activates the guard at runtime (with `segments=None` default elsewhere keeping tests unaffected).

- [ ] **Step 1: Implement — pass segments to MazeMotion.**

In `flood_fill_solver.py`, change the `self.motion = MazeMotion(...)` construction to add `segments=load_segments()` as the final kwarg. The current call is:
```python
        self.motion = MazeMotion(self.brain, cruise_v=self.cruise_v,
                                 center_tol_m=self.center_tol_m,
                                 yaw_tol_rad=self.yaw_tol_rad,
                                 hop_arrive_slack_m=self.hop_arrive_slack_m,
                                 front_block_m=self.front_block_m,
                                 hop_timeout_s=self.hop_timeout_s,
                                 mem=self.mem)
```
Change the last line to:
```python
                                 mem=self.mem, segments=load_segments())
```

- [ ] **Step 2: Verify import-safety + suite.**
`cd .../src/tugbot_maze && python3 -c "import tugbot_maze.flood_fill_solver"` → exit 0.
`cd .../src/tugbot_maze && python3 -m pytest test/test_maze_motion_sim.py test/test_collision_geometry.py test/test_scan_match_localizer.py -q` → all pass (3 `[0.05-*]` xfails remain xfailed).

- [ ] **Step 3: Commit**
```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/flood_fill_solver.py
git commit -m "feat: pass known wall map to MazeMotion (activates rotation-sweep guard at runtime)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: Full-suite green + Gazebo acceptance (user-gated)

**Files:** none (verification + handoff). The Gazebo run is launched by the user ("set up the run"); do not auto-launch.

- [ ] **Step 1: Focused + core ROS-free suites.**
`cd .../src/tugbot_maze && python3 -m pytest test/test_collision_geometry.py test/test_maze_motion_sim.py test/test_scan_match_localizer.py test/test_pose_tracking.py test/test_maze_sim.py -q`
Expected: all pass; 3 `[0.05-*]` xfails remain xfailed; no regressions.

- [ ] **Step 2: Core unit regression (no guard-induced breakage).**
`cd .../src/tugbot_maze && python3 -m pytest test/test_flood_fill_brain.py test/test_cell_walls.py test/test_hop_controller.py test/test_wall_localize.py -q`
Expected: all pass.

- [ ] **Step 3: Build for the Gazebo run.**
`cd .../ros2_ws_tugbot_nav_20260614 && source /opt/ros/jazzy/setup.bash && colcon build --packages-select tugbot_maze --symlink-install`
Expected: build OK; then `python3 -c "from tugbot_maze.collision_geometry import FootprintClearance"` resolves from install.

- [ ] **Step 4: Document the Gazebo acceptance (user-gated).**
The user clears stray sims and triggers (default pose source is already `scan_match`):
`cd .../ros2_ws_tugbot_nav_20260614 && export DISPLAY=:1 && bash tools/batch_diagnose_floodfill.sh 8 1200 false true`
Then replay the recorded trajectories through the true-footprint oracle (the `/tmp/replay_collisions.py`-style script over the batch manifest — `MazeSim.collides` now routes through `collision_geometry`). **Pass criteria:** **0** collision samples (was 6/1808; specifically 0 at cell (3,9)), still 8/8 EXIT_REACHED, ~560 s. Watch for any new `PRETURN exhausted` events (should be rare/none).

- [ ] **Step 5: Hand off.** Report offline results + present the Gazebo command. On a clean batch, finish the branch (bank to main per project convention).

---

## Self-Review

**Spec coverage:** §3 invariant+maneuver → Task 2 (`_route` gate + `_preturn`). §4 components → Task 1 (`collision_geometry`, `maze_sim` refactor), Task 2 (`MazeMotion`), Task 3 (node). §6 testing → Task 1 (equivalence, ground-truth, swept), Task 2 (reproduce-then-fix + regression), refactor safety (Task 1 Step 6). §7 acceptance → Task 4. All covered.

**Placeholder scan:** No TBD; complete code in every step; the one tunable note (Task 1 Step 4 mid-arc heading constants) is bounded by the investigation's grazing band, not open-ended.

**Type/name consistency:** `rect_hits_segments(segs,x,y,yaw,x0,x1,y0,y1,half_thickness)` and `FootprintClearance(segments,*,wall_half_thickness_m,x0,x1,y0,y1).collides/.swept_collides(...,step_rad=)` are used identically in Task 1 (def), Task 2 (`self.clearance`), and the tests. `_preturn` uses `self.preturn_v/preturn_timeout_s/sweep_step_rad/preturn_start/clearance` all defined in Task 2 Step 3. `_route` gate uses `self.prev_yaw` (set in `step()`), `self.target_cardinal`, `CELL_SIZE_M` (imported). `segments=None` default consistent across `MazeMotion.__init__` (Task 2) and the node passing `load_segments()` (Task 3).

**Invariant check:** the gate prevents the grazing turn regardless of whether `_preturn` makes room (timeout → `_unstick`), matching the spec's safety contract.
