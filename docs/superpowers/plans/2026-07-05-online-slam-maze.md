# Online self-built-map maze solving (no prior map) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Solve the same 20260528 maze in a new clean workspace `ros2_ws_tugbot_nav_20260705` **without the wall map fed** — localize online by scan-matching against a *growing* set of confirmed walls (known perimeter + the flood-fill brain's committed interior walls), grid-snapped so the pose stays absolute and drift-free.

**Architecture:** Seed the new workspace from the completed 20260614 stack (flood-fill + MazeMotion + the ICP) and change only the localization layer. A new `OnlineScanMatchLocalizer` wraps the **unchanged** `ScanMatchLocalizer` ICP, rebuilding it against `perimeter + confirmed-interior-walls` whenever that set grows. The flood-fill brain both consumes the pose and produces the confirmed-wall set (a feedback loop bounded by its existing commit gate).

**Tech Stack:** Python 3, NumPy, pytest, ROS 2 Jazzy, Gazebo Harmonic. ROS-free core validated offline in `maze_sim` before any Gazebo run.

**Design:** `docs/superpowers/specs/2026-07-05-online-slam-maze-design.md`

**Branch:** create `online-slam-maze` off `main` before Task 1 (do NOT implement on `main`):
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git checkout -b online-slam-maze
```

All paths below are relative to the repo root. The new workspace is `ros2_ws_tugbot_nav_20260705`. Offline tests run from `ros2_ws_tugbot_nav_20260705/src/tugbot_maze`.

---

## File Structure

- `ros2_ws_tugbot_nav_20260705/` — NEW workspace, seeded from `ros2_ws_tugbot_nav_20260614`, pruned (Task 1).
- `.../src/tugbot_maze/tugbot_maze/maze_sim.py` — MODIFY: add `outer_segments()` (perimeter centerlines) (Task 2).
- `.../src/tugbot_maze/tugbot_maze/online_scan_match_localizer.py` — NEW: `confirmed_wall_segments()` + `OnlineScanMatchLocalizer` (Tasks 3–4).
- `.../src/tugbot_maze/tugbot_maze/flood_fill_solver.py` — MODIFY: add `online_slam` pose_source (Task 5).
- `.../src/tugbot_maze/test/test_online_scan_match_localizer.py` — NEW: unit tests (Tasks 3–4).
- `.../src/tugbot_maze/test/test_online_slam_sim.py` — NEW: offline end-to-end, map withheld (Task 6).
- `.../tools/run_flood_fill_maze.sh` + workspace `README.md` — MODIFY: document/enable `online_slam` (Task 7).

---

## Task 1: Seed the clean `ros2_ws_tugbot_nav_20260705` workspace

**Files:** create the workspace by copy; no code yet.

- [ ] **Step 1: Copy the 20260614 workspace**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
cp -r ros2_ws_tugbot_nav_20260614 ros2_ws_tugbot_nav_20260705
```

- [ ] **Step 2: Prune regenerated dirs + historical cruft**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260705
rm -rf build install log
rm -rf archive doc/archive src/tugbot_maze/tugbot_maze/__pycache__
# one-shot phaseNN visual-overlay diagnostics (dead code from the long bring-up):
find src -name 'phase[0-9]*_*.py' -delete
```

- [ ] **Step 3: Build the seeded workspace (must be green)**

Run:
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260705
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
```
Expected: build succeeds. If the `phase[0-9]*` deletion broke an import/build (some file references one), restore just those files (`git checkout -- <file>` is not applicable pre-commit — instead re-copy the specific file from `../ros2_ws_tugbot_nav_20260614/`), then rebuild until green. Record which (if any) were kept.

- [ ] **Step 4: Offline suite green (proves the stack is intact)**

Run:
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260705/src/tugbot_maze
python3 -m pytest test/test_hop_controller.py test/test_maze_motion_sim.py test/test_maze_sim.py \
  test/test_flood_fill_brain.py test/test_cell_walls.py test/test_scan_match_localizer.py \
  test/test_pose_tracking.py test/test_wall_localize.py -q
```
Expected: all pass (same counts as 20260614 — the fed-map `test_maze_motion_sim` drift=0.05 cases already pass after the reverse-to-center fix). This confirms the clean seed carries the whole proven stack.

- [ ] **Step 5: Commit the seeded workspace**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260705
git commit -m "$(cat <<'EOF'
feat: seed ros2_ws_tugbot_nav_20260705 from 20260614 (clean)

Clean-seed the next-phase workspace: copy the proven flood-fill + scan-match
+ MazeMotion stack, drop build/install/log, archive/, and one-shot phaseNN
diagnostics. Same 20260528 world for direct comparison. Build + offline suite
green. Localization layer changes come next (online self-built-map SLAM).

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: `outer_segments()` — the known perimeter reference

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260705/src/tugbot_maze/tugbot_maze/maze_sim.py` (add after `outer_boundary_box`, ~line 75)
- Test: `ros2_ws_tugbot_nav_20260705/src/tugbot_maze/test/test_maze_sim.py` (append)

- [ ] **Step 1: Write the failing test**

Append to `test/test_maze_sim.py`:

```python
def test_outer_segments_is_the_known_perimeter_subset():
    from tugbot_maze.maze_sim import outer_segments, load_segments
    outer = outer_segments()
    full = load_segments()
    assert len(outer) > 0
    assert len(outer) < len(full)                     # interior walls are NOT included
    full_set = {tuple(round(v, 3) for v in s) for s in full}
    for s in outer:                                   # every perimeter seg is a real map wall
        assert tuple(round(v, 3) for v in s) in full_set
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd ros2_ws_tugbot_nav_20260705/src/tugbot_maze && python3 -m pytest test/test_maze_sim.py::test_outer_segments_is_the_known_perimeter_subset -q`
Expected: FAIL with `ImportError: cannot import name 'outer_segments'`.

- [ ] **Step 3: Implement `outer_segments`**

In `maze_sim.py`, add directly after `outer_boundary_box` (mirrors it but returns the segments):

```python
def outer_segments(path: Optional[str] = None) -> List[Segment]:
    """Return the maze's OUTER boundary wall centerlines (segments flagged `outer: true`),
    in map frame. This is the legitimately-known perimeter used to seed online localization
    when the interior wall map is withheld -- NOT the full map (`load_segments`)."""
    if path is None:
        path = default_segments_path()
    with open(path) as f:
        doc = yaml.safe_load(f)
    segs: List[Segment] = []
    for s in doc['segments']:
        if s.get('outer'):
            x0, y0 = px_to_map(float(s['p0_px'][0]), float(s['p0_px'][1]))
            x1, y1 = px_to_map(float(s['p1_px'][0]), float(s['p1_px'][1]))
            segs.append((x0, y0, x1, y1))
    if not segs:
        raise RuntimeError(f"no 'outer: true' segments found in {path!r}")
    return segs
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd ros2_ws_tugbot_nav_20260705/src/tugbot_maze && python3 -m pytest test/test_maze_sim.py -q`
Expected: PASS (all `test_maze_sim.py` tests green).

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260705/src/tugbot_maze/tugbot_maze/maze_sim.py \
        ros2_ws_tugbot_nav_20260705/src/tugbot_maze/test/test_maze_sim.py
git commit -m "$(cat <<'EOF'
feat: maze_sim.outer_segments() -- the known perimeter (not the full map)

Returns only the outer:true boundary centerlines, to seed online localization
when the interior wall map is withheld.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: `confirmed_wall_segments()` — committed walls → grid-snapped segments

**Files:**
- Create: `ros2_ws_tugbot_nav_20260705/src/tugbot_maze/tugbot_maze/online_scan_match_localizer.py`
- Test: `ros2_ws_tugbot_nav_20260705/src/tugbot_maze/test/test_online_scan_match_localizer.py`

- [ ] **Step 1: Write the failing test**

Create `test/test_online_scan_match_localizer.py`:

```python
import math
from tugbot_maze.flood_fill_brain import FloodFillBrain
from tugbot_maze.online_scan_match_localizer import confirmed_wall_segments


def _canon(seg):
    a, b = (round(seg[0], 3), round(seg[1], 3)), (round(seg[2], 3), round(seg[3], 3))
    return (a, b) if a <= b else (b, a)


def test_committed_wall_becomes_grid_snapped_segment():
    brain = FloodFillBrain()
    brain.mark((3, 4), 'N', True)                       # a wall on the N edge of cell (3,4)
    segs = confirmed_wall_segments(brain, {(3, 4)})
    # cell (3,4) centre = (6,8); N edge y=9, x in [5,7] -> (5,9,7,9)
    assert [_canon(s) for s in segs] == [((5.0, 9.0), (7.0, 9.0))]


def test_uncommitted_and_non_wall_edges_excluded():
    brain = FloodFillBrain()
    brain.mark((3, 4), 'N', True)                       # wall, but its cell is NOT committed
    assert confirmed_wall_segments(brain, set()) == []
    brain.mark((3, 4), 'E', False)                      # OPEN edge in a committed cell
    segs = confirmed_wall_segments(brain, {(3, 4)})
    assert len(segs) == 1                               # only the N wall, not the E-open edge


def test_symmetric_edge_deduplicated():
    brain = FloodFillBrain()
    brain.mark((3, 4), 'N', True)                       # also marks (3,5) S == the same wall
    segs = confirmed_wall_segments(brain, {(3, 4), (3, 5)})
    assert len(segs) == 1                               # one physical wall, not two
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd ros2_ws_tugbot_nav_20260705/src/tugbot_maze && python3 -m pytest test/test_online_scan_match_localizer.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'tugbot_maze.online_scan_match_localizer'`.

- [ ] **Step 3: Implement `confirmed_wall_segments` (+ `_edge_segment`)**

Create `online_scan_match_localizer.py`:

```python
"""Online self-built-map localization: scan-match against a GROWING set of confirmed
walls (known perimeter + the flood-fill brain's committed interior walls) instead of a
prior full map. Because the grid is known, a confirmed wall's centerline is grid-snapped
and exact, so the reused ICP still yields an absolute, drift-free pose.
See docs/superpowers/specs/2026-07-05-online-slam-maze-design.md.
"""
from __future__ import annotations
from typing import Iterable, List, Tuple

from tugbot_maze.flood_fill_brain import CELL_SIZE_M, DIRS
from tugbot_maze.scan_match_localizer import ScanMatchLocalizer

Segment = Tuple[float, float, float, float]
Cell = Tuple[int, int]


def _edge_segment(cell: Cell, d: str) -> Segment:
    """Grid-snapped centerline of the wall on edge (cell, d). Cell centre = (2c, 2r);
    each wall lies half a cell (1.0 m) from centre and spans the cell width."""
    c, r = cell
    cx, cy = CELL_SIZE_M * c, CELL_SIZE_M * r
    h = CELL_SIZE_M / 2.0
    if d == 'N':
        return (cx - h, cy + h, cx + h, cy + h)
    if d == 'S':
        return (cx - h, cy - h, cx + h, cy - h)
    if d == 'E':
        return (cx + h, cy - h, cx + h, cy + h)
    return (cx - h, cy - h, cx - h, cy + h)              # 'W'


def _canonical(seg: Segment) -> Segment:
    a, b = (seg[0], seg[1]), (seg[2], seg[3])
    return (a[0], a[1], b[0], b[1]) if a <= b else (b[0], b[1], a[0], a[1])


def confirmed_wall_segments(brain, committed_cells: Iterable[Cell]) -> List[Segment]:
    """Segments for every WALL edge of a committed cell, de-duplicated (a wall is shared by
    two cells). Only committed cells contribute, so a poorly-sensed wall never enters the
    localization reference."""
    seen = set()
    for cell in committed_cells:
        for d in DIRS:
            if brain.is_wall(cell, d):
                seen.add(_canonical(_edge_segment(cell, d)))
    return [tuple(s) for s in seen]
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd ros2_ws_tugbot_nav_20260705/src/tugbot_maze && python3 -m pytest test/test_online_scan_match_localizer.py -q`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260705/src/tugbot_maze/tugbot_maze/online_scan_match_localizer.py \
        ros2_ws_tugbot_nav_20260705/src/tugbot_maze/test/test_online_scan_match_localizer.py
git commit -m "$(cat <<'EOF'
feat: confirmed_wall_segments() -- committed walls to grid-snapped segments

Materialises the brain's committed WALL edges as grid-snapped centerline
segments (deduped), the growing interior reference for online localization.
Only committed cells contribute, so a poor sense can't poison localization.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: `OnlineScanMatchLocalizer` — reuse the ICP against a dynamic reference

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260705/src/tugbot_maze/tugbot_maze/online_scan_match_localizer.py`
- Test: `ros2_ws_tugbot_nav_20260705/src/tugbot_maze/test/test_online_scan_match_localizer.py` (append)

**Design:** wrap the unchanged `ScanMatchLocalizer`. Hold the perimeter; each `correct()` merges `perimeter + interior_segments`; rebuild the inner ICP only when the interior set changes (cheap signature compare). `**icp_kwargs` pass through (e.g. `scan_offset_x=0.0` for the offline sim, which scans from body centre).

- [ ] **Step 1: Write the failing test**

Append to `test/test_online_scan_match_localizer.py`:

```python
from tugbot_maze.online_scan_match_localizer import OnlineScanMatchLocalizer

_NOSCAN = ([float('inf')] * 8, -math.pi, 2 * math.pi / 8)   # all-inf beams -> ICP returns prior


def test_online_localizer_starts_with_perimeter_only():
    loc = OnlineScanMatchLocalizer([(0.0, 0.0, 10.0, 0.0)], scan_offset_x=0.0)
    assert loc._icp._a.shape[0] == 1                    # one perimeter segment
    assert loc._icp.scan_offset_x == 0.0                # kwargs passed through


def test_online_localizer_rebuilds_when_interior_grows_and_caches():
    loc = OnlineScanMatchLocalizer([(0.0, 0.0, 10.0, 0.0)])
    icp0 = loc._icp
    loc.correct((0.0, 0.0, 0.0), *_NOSCAN, [])          # empty interior -> no rebuild
    assert loc._icp is icp0
    loc.correct((0.0, 0.0, 0.0), *_NOSCAN, [(1.0, 1.0, 1.0, 3.0)])   # new wall -> rebuild
    assert loc._icp is not icp0
    assert loc._icp._a.shape[0] == 2                    # perimeter + 1 interior
    icp1 = loc._icp
    loc.correct((0.0, 0.0, 0.0), *_NOSCAN, [(1.0, 1.0, 1.0, 3.0)])   # same set -> cached
    assert loc._icp is icp1


def test_online_localizer_correct_returns_pose_and_info():
    loc = OnlineScanMatchLocalizer([(0.0, 0.0, 10.0, 0.0)])
    pose, info = loc.correct((0.5, 0.5, 0.1), *_NOSCAN, [])
    assert len(pose) == 3 and isinstance(info, dict)    # delegates to the ICP contract
    assert info['rejected'] is True                     # no inliers from all-inf beams
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd ros2_ws_tugbot_nav_20260705/src/tugbot_maze && python3 -m pytest test/test_online_scan_match_localizer.py -q`
Expected: FAIL with `ImportError: cannot import name 'OnlineScanMatchLocalizer'`.

- [ ] **Step 3: Implement `OnlineScanMatchLocalizer`**

Append to `online_scan_match_localizer.py`:

```python
class OnlineScanMatchLocalizer:
    """Scan-match against `perimeter + confirmed-interior-walls`, rebuilding the inner
    ScanMatchLocalizer only when the interior set changes. The ICP itself is reused
    unchanged; only the segment source is dynamic."""

    def __init__(self, perimeter_segments, **icp_kwargs):
        self._perimeter = [tuple(s) for s in perimeter_segments]
        self._icp_kwargs = dict(icp_kwargs)
        self._sig = frozenset()
        self._rebuild([])

    def _rebuild(self, interior_segments) -> None:
        segs = self._perimeter + [tuple(s) for s in interior_segments]
        self._icp = ScanMatchLocalizer(segs, **self._icp_kwargs)

    def correct(self, prior_pose, ranges, angle_min, angle_inc, interior_segments):
        sig = frozenset(tuple(s) for s in interior_segments)
        if sig != self._sig:
            self._rebuild(interior_segments)
            self._sig = sig
        return self._icp.correct(prior_pose, ranges, angle_min, angle_inc)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd ros2_ws_tugbot_nav_20260705/src/tugbot_maze && python3 -m pytest test/test_online_scan_match_localizer.py -q`
Expected: PASS (6 tests total).

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260705/src/tugbot_maze/tugbot_maze/online_scan_match_localizer.py \
        ros2_ws_tugbot_nav_20260705/src/tugbot_maze/test/test_online_scan_match_localizer.py
git commit -m "$(cat <<'EOF'
feat: OnlineScanMatchLocalizer -- reuse the ICP against a dynamic reference

Wraps the unchanged ScanMatchLocalizer; rebuilds it against
perimeter + confirmed-interior-walls only when that set grows. **icp_kwargs
pass through (e.g. scan_offset_x for the offline sim).

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: Wire `online_slam` into the solver node

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260705/src/tugbot_maze/tugbot_maze/flood_fill_solver.py`

**Design:** reuse the existing `_lookup_scan_match()` scaffold (bootstrap, per-scan idempotency, odom prior) for both `scan_match` and `online_slam`; branch only at the `correct()` call to pass confirmed interior segments. This is an integration change; it is verified by build + import + the offline end-to-end (Task 6) and the Gazebo gate — no new unit test here.

- [ ] **Step 1: Add imports**

In `flood_fill_solver.py`, the current import (line ~25–26) is:
```python
from tugbot_maze.scan_match_localizer import ScanMatchLocalizer
from tugbot_maze.maze_sim import load_segments
```
Replace with:
```python
from tugbot_maze.scan_match_localizer import ScanMatchLocalizer
from tugbot_maze.maze_sim import load_segments, outer_segments
from tugbot_maze.online_scan_match_localizer import (
    OnlineScanMatchLocalizer, confirmed_wall_segments)
```

- [ ] **Step 2: Construct the right localizer for the pose_source**

Replace the construction (current lines ~76–77):
```python
        self.localizer = (ScanMatchLocalizer(load_segments())
                          if self.pose_source == 'scan_match' else None)
```
with:
```python
        if self.pose_source == 'scan_match':
            self.localizer = ScanMatchLocalizer(load_segments())          # fed the full map (baseline)
        elif self.pose_source == 'online_slam':
            self.localizer = OnlineScanMatchLocalizer(outer_segments())   # perimeter only; interior grows online
        else:
            self.localizer = None
```

- [ ] **Step 3: Route `online_slam` through the scan-match lookup**

Replace the dispatch head of `_lookup_pose` (current lines ~108–110):
```python
    def _lookup_pose(self):
        if self.pose_source == 'scan_match':
            return self._lookup_scan_match()
```
with:
```python
    def _lookup_pose(self):
        if self.pose_source in ('scan_match', 'online_slam'):
            return self._lookup_scan_match()
```

- [ ] **Step 4: Pass confirmed interior segments in the `correct()` call**

In `_lookup_scan_match`, replace the `correct()` call (current lines ~149–153):
```python
        try:
            est, info = self.localizer.correct(prior, s.ranges, s.angle_min, s.angle_increment)
        except Exception as e:           # never let a localizer hiccup kill the node mid-batch
            self.get_logger().warning('scan_match correct() failed: %r; using odom prior' % (e,))
            est, info = prior, {'rejected': True, 'error': repr(e)}
```
with:
```python
        try:
            if self.pose_source == 'online_slam':
                interior = confirmed_wall_segments(self.brain, self.motion.committed)
                est, info = self.localizer.correct(
                    prior, s.ranges, s.angle_min, s.angle_increment, interior)
            else:
                est, info = self.localizer.correct(
                    prior, s.ranges, s.angle_min, s.angle_increment)
        except Exception as e:           # never let a localizer hiccup kill the node mid-batch
            self.get_logger().warning('%s correct() failed: %r; using odom prior'
                                      % (self.pose_source, e))
            est, info = prior, {'rejected': True, 'error': repr(e)}
```

Also make the info-logging guard (current line ~231, `if self.pose_source == 'scan_match' and self._sm_info is not None:`) fire for both by replacing that condition with:
```python
        if self.pose_source in ('scan_match', 'online_slam') and self._sm_info is not None:
```

- [ ] **Step 5: Verify build + import + dispatch**

Run:
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260705
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select tugbot_maze
source install/setup.bash
python3 -c "import tugbot_maze.flood_fill_solver as m; print('import OK')"
```
Expected: build green; `import OK`. (Node runtime is exercised in the Gazebo gate; `self.motion.committed` is the committed-cell set the brain builds — confirm the attribute exists: `python3 -c "from tugbot_maze.maze_motion import MazeMotion; print(hasattr(MazeMotion(), 'committed'))"` → `True`.)

- [ ] **Step 6: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260705/src/tugbot_maze/tugbot_maze/flood_fill_solver.py
git commit -m "$(cat <<'EOF'
feat: online_slam pose_source -- localize against self-built confirmed walls

Routes online_slam through the existing scan-match lookup scaffold (bootstrap,
per-scan idempotency, odom prior), branching only to pass the brain's confirmed
interior walls to OnlineScanMatchLocalizer. scan_match (fed map) stays as the
A/B baseline.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: Offline end-to-end — solve the maze with the map WITHHELD

**Files:**
- Create: `ros2_ws_tugbot_nav_20260705/src/tugbot_maze/test/test_online_slam_sim.py`

**This is the crux validation.** `maze_sim` raycasts/collides against the TRUE maze (`load_segments()`), but the solver's localizer is given only the **perimeter** (`outer_segments()`) + the brain's **online-committed** interior walls — never the full map. The harness closes the online-localization loop exactly as the node does. A FAILURE here is the expected iteration point (tune bootstrap seeding / gate thresholds, or escalate to the spec §6 slam fallback) — not a mechanical defect.

- [ ] **Step 1: Write the failing test**

Create `test/test_online_slam_sim.py`:

```python
import math
from tugbot_maze.maze_motion import MazeMotion
from tugbot_maze.maze_sim import MazeSim, load_segments, outer_segments
from tugbot_maze.flood_fill_brain import ENTRANCE_CELL, EXIT_CELL, cell_center
from tugbot_maze.pose_tracking import odom_prior
from tugbot_maze.online_scan_match_localizer import (
    OnlineScanMatchLocalizer, confirmed_wall_segments)


def _run_online_slam(drift, latency=0, dt=0.1, max_steps=30000):
    """Drive MazeMotion on the ONLINE-localized pose. The sim knows the true walls; the
    localizer is seeded with the perimeter only and grows its interior reference from the
    brain's committed walls. Returns (reached, collided, max_loc_err_m)."""
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0,
                  inertia=True, odom_drift_per_m=drift, cmd_latency_steps=latency)
    m = MazeMotion()
    loc = OnlineScanMatchLocalizer(outer_segments(), scan_offset_x=0.0)   # sim scans from body centre
    corrected = (sim.x, sim.y, sim.yaw)              # cold-start bootstrap = true start pose
    last_odom = sim.reported_pose
    t, collided, max_loc_err = 0.0, False, 0.0
    for _ in range(max_steps):
        scan = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        cur_odom = sim.reported_pose
        prior = odom_prior(corrected, last_odom, cur_odom)
        interior = confirmed_wall_segments(m.brain, m.committed)
        corrected, _info = loc.correct(prior, scan[0], scan[1], scan[2], interior)
        last_odom = cur_odom
        max_loc_err = max(max_loc_err, math.hypot(corrected[0] - sim.x, corrected[1] - sim.y))
        v, w, done = m.step(corrected, scan, t)
        if done:
            return True, collided, max_loc_err
        sim.step(v, w, dt)
        if sim.collides(sim.x, sim.y, sim.yaw):
            collided = True
        t += dt
    return (m.cell == EXIT_CELL), collided, max_loc_err


def test_online_slam_reaches_exit_with_map_withheld():
    reached, collided, max_loc_err = _run_online_slam(drift=0.0)
    assert reached, "did not reach the exit on the online self-built map"
    assert not collided, "robot body collided with a wall"
    assert max_loc_err < 0.5, f"online localization error unbounded ({max_loc_err:.2f} m)"
```

- [ ] **Step 2: Run test to verify it fails first for the right reason**

Run: `cd ros2_ws_tugbot_nav_20260705/src/tugbot_maze && python3 -m pytest test/test_online_slam_sim.py -q`
Expected initially: FAIL at import if run before Tasks 3–4 are present; once those exist, it exercises the real loop. If it fails on `reached`/`max_loc_err`, that is the **research signal** — proceed to Step 3.

- [ ] **Step 3: Make it pass (iterate on the localizer, not the test)**

The pieces (Tasks 2–5) are built; this step is where the approach is proven. If Step 2 shows drift/no-exit, iterate in this order, re-running the test after each change, and record what was needed:
1. **Bootstrap:** confirm `corrected` is seeded to the true start pose and the perimeter anchors the entrance region (it should — `outer_segments()` includes the entrance-side boundary).
2. **Gate thresholds:** if the interior reference is too sparse mid-maze and the pose drifts, the ICP's per-DOF observability gate (`min_eig`, `min_yaw_info`) should already drop to the odom prior; if it is instead accepting bad corrections, raise `min_inliers`/`min_eig` via `OnlineScanMatchLocalizer(outer_segments(), min_eig=..., min_inliers=...)` (kwargs pass straight through).
3. **Nominal-vs-map offset:** the grid-snapped interior segments are within scan-match tolerance (~0.12 + 0.30 m) of the real walls, and the real-frame perimeter anchors the pose. If a systematic bias appears, calibrate one constant offset from the known perimeter (compare `outer_segments()` to the nominal boundary edges) and apply it in `_edge_segment` — do NOT read interior walls from the map.
4. If reliable interior localization still can't be reached offline, escalate per spec §6 (a minimal `slam_toolbox`-pose fallback inside the gate) — flag for a design revisit rather than forcing the test.

Target: `test_online_slam_reaches_exit_with_map_withheld` passes (reaches exit, no collision, `max_loc_err < 0.5`). Then add a drift-stress case mirroring the existing suite:

```python
def test_online_slam_under_moderate_drift():
    reached, collided, _ = _run_online_slam(drift=0.03)
    assert reached and not collided
```

- [ ] **Step 4: Run the full offline suite (regression guard)**

Run:
```bash
cd ros2_ws_tugbot_nav_20260705/src/tugbot_maze && python3 -m pytest \
  test/test_online_slam_sim.py test/test_online_scan_match_localizer.py \
  test/test_maze_motion_sim.py test/test_maze_sim.py test/test_hop_controller.py \
  test/test_scan_match_localizer.py test/test_flood_fill_brain.py test/test_cell_walls.py \
  test/test_pose_tracking.py test/test_wall_localize.py -q
```
Expected: all green (the fed-map `test_maze_motion_sim` cases must stay green — the online path is additive).

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260705/src/tugbot_maze/test/test_online_slam_sim.py \
        ros2_ws_tugbot_nav_20260705/src/tugbot_maze/tugbot_maze/online_scan_match_localizer.py
git commit -m "$(cat <<'EOF'
test: offline end-to-end online-SLAM maze solve (map withheld)

Closes the online-localization loop in maze_sim: the sim knows the true walls,
the localizer is seeded with the perimeter only and grows its interior
reference from the brain's committed walls. Asserts exit reached, no collision,
bounded localization error. <record any localizer tuning needed to pass>.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: Enable `online_slam` in the runners + workspace README

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260705/tools/run_flood_fill_maze.sh` (comment/usage only)
- Modify: `ros2_ws_tugbot_nav_20260705/README.md`

- [ ] **Step 1: Document `online_slam` in the run script usage**

`run_flood_fill_maze.sh` already passes arg 4 (`POSE_SOURCE`) straight through to the launch, so `online_slam` is already runnable as `bash tools/run_flood_fill_maze.sh 1200 true false online_slam true`. Update the usage comment (the `POSE_SOURCE:` line, ~line 7) to list it:
```bash
#   POSE_SOURCE: scan_match (fed the full known map; A/B upper bound) | online_slam
#                (self-built map: perimeter + committed walls, no prior interior map)
#                | odom_locked | slam
```

- [ ] **Step 2: Rewrite the workspace README headline for the new phase**

Replace the top of `ros2_ws_tugbot_nav_20260705/README.md` (the 20260614 headline block) with a short section stating: this workspace solves the same 20260528 maze with **no interior wall map fed** via `pose_source:=online_slam` (online self-built-map localization); grid geometry + entrance/exit + perimeter are known, interior walls discovered online; `pose_source:=scan_match` (fed map) is the A/B upper-bound baseline; how to run:
```bash
bash tools/run_flood_fill_maze.sh 1200 false true online_slam true   # visual, no map fed
bash tools/batch_diagnose_floodfill.sh 8 1200 true false online_slam true   # controlled batch
python3 tools/replay_collision_oracle.py log/batch_diag_<stamp>              # collision check
```
Keep the rest of the README (architecture, diagnostic tooling) accurate; note that only the localization layer changed from 20260614.

- [ ] **Step 3: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260705/tools/run_flood_fill_maze.sh ros2_ws_tugbot_nav_20260705/README.md
git commit -m "$(cat <<'EOF'
docs: enable + document online_slam in the 20260705 runners and README

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Post-implementation: Gazebo acceptance gate (with the user)

Not autonomous — needs the user to clear stray sims and say "set up the run". On the `online-slam-maze` branch after Tasks 1–7:

1. Build: `cd ros2_ws_tugbot_nav_20260705 && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash`.
2. **Two controlled batches (16 runs), no map fed:** `bash tools/batch_diagnose_floodfill.sh 8 1200 true false online_slam true` ×2.
3. Oracle over both: `python3 tools/replay_collision_oracle.py log/batch_diag_<s1> log/batch_diag_<s2>`.
4. A/B upper bound: one `scan_match` (fed-map) batch for reference.

**Acceptance (the honest gate):**
- **≥ 14/16 EXIT_REACHED** with no interior map fed.
- Collision rate comparable to the 20260614 fed-map baseline (oracle replay).
- If it regresses / cannot complete → iterate on the localizer per Task 6 Step 3, or park the branch with an honest write-up (as the rotation-sweep guard was). Merge to `main` only after the gate passes (via `superpowers:finishing-a-development-branch`).

---

## Self-Review

**Spec coverage:** §3.1 workspace seed → Task 1. §3.2 online localizer (confirmed set, ICP reuse, gate, fallback) → Tasks 3–4 (+ perimeter Task 2). §3.3 feedback loop / §3.2 wiring → Task 5. §5.1 unit → Tasks 2–4. §5.2 offline map-withheld → Task 6. §5.3 Gazebo gate → Post-impl. §4 carries-over-vs-new → Tasks 3–5 (new) with everything else untouched. §6 risks → Task 6 Step 3 iteration order. All spec sections map to a task.

**Placeholder scan:** the one place with a genuine unknown is Task 6 (does the approach converge) — handled explicitly with a concrete test + a prioritised iteration order + an escalation path, not a "TODO". No other placeholders; every code step shows complete code and every run step an exact command + expected output.

**Type/name consistency:** `confirmed_wall_segments(brain, committed_cells)`, `OnlineScanMatchLocalizer(perimeter_segments, **icp_kwargs)`, `.correct(prior, ranges, angle_min, angle_inc, interior_segments)`, `_edge_segment(cell, d)`, `outer_segments(path=None)` are used identically across Tasks 2–6. The solver uses `self.motion.committed` (verified attribute) and `self.brain`. Segment format is the 4-tuple `(x1,y1,x2,y2)` everywhere, matching `ScanMatchLocalizer`'s `reshape(-1,4)`.
