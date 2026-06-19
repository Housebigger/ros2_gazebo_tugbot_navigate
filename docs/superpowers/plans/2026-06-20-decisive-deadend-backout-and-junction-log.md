# Decisive Dead-End Back-Out + Passive Junction Log — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the flood-fill maze solver a decisive 1-cell dead-end back-out (no 180°-in-cell, no 3 s stall) and a passive per-run junction record, without disturbing the proven routing.

**Architecture:** Two independent components. (1) A new `backout` FSM phase in `MazeMotion`, gated on the single open exit being the edge the robot drove in through (`came_from`), with a bounded escalating timeout. (2) A node-only passive junction logger built from pure helpers (`open_exits`, `JunctionLog`, `update_junctions`) that reads existing motion/brain state and writes `junctions.json`.

**Tech Stack:** Python 3.12, ROS 2 Jazzy (node only), pytest. The motion/brain/log logic is ROS-free and unit-tested offline; the node is a thin adapter.

**Spec:** `docs/superpowers/specs/2026-06-20-decisive-deadend-backout-and-junction-log-design.md` (hardened by adversarial review — read its *Revisions* section).

**Conventions for every task:**
- `PKG = /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260614/src/tugbot_maze`
- Run tests with: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest <path> -v` (the package dir holds the importable `tugbot_maze/` module + `test/`).
- Work on branch `deadend-backout-junction-log` (already created off `main`; **never push**).
- Each commit message ends with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`. **No backticks inside `git commit -m`** (bash command-substitution can drop a word). Use absolute paths / `git -C`.
- Repo root for git: `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate` (`REPO`). Stage only the files each task touches (the repo has unrelated `20260522/build` noise — never `git add -A`).

---

## File Structure

| File | Responsibility | Task |
|------|----------------|------|
| `tugbot_maze/flood_fill_brain.py` | + pure `open_exits(brain, cell)` helper (additive, no routing change) | 1 |
| `tugbot_maze/hop_controller.py` | + pure `backout_command(...)` reverse command | 2 |
| `tugbot_maze/maze_motion.py` | + `backout` FSM phase (detection, `_backout`, state, `backout_count`) | 3 |
| `tugbot_maze/junction_log.py` | **new** — `JunctionLog` class + `update_junctions` pure glue | 4 |
| `tugbot_maze/flood_fill_solver.py` | node: own `JunctionLog`, call `update_junctions`, guarded flush | 5 |
| `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py` | + `junction_log_dir` launch arg → node param | 6 |
| `tools/run_flood_fill_maze.sh` | pass `junction_log_dir:=$ART`; grep `JUNCTION` | 7 |
| `test/test_flood_fill_brain.py` | + `open_exits` tests | 1 |
| `test/test_hop_controller.py` | + `backout_command` tests | 2 |
| `test/test_maze_motion.py` | + back-out FSM tests (incl. entrance no-fire, timeout escalate) | 3 |
| `test/test_junction_log.py` | **new** — `JunctionLog` + `update_junctions` tests | 4 |
| `test/test_maze_motion_sim.py` | + assert back-out fired end-to-end | 8 |

---

## Task 1: `open_exits` helper (`flood_fill_brain.py`)

**Files:**
- Modify: `tugbot_maze/flood_fill_brain.py` (append a module-level function after the `FloodFillBrain` class)
- Test: `test/test_flood_fill_brain.py` (append)

- [ ] **Step 1: Write the failing tests** — append to `test/test_flood_fill_brain.py`:

```python
def test_open_exits_counts_ingrid_open_only():
    from tugbot_maze.flood_fill_brain import FloodFillBrain, open_exits
    b = FloodFillBrain()
    b.mark((5, 5), 'N', True)
    b.mark((5, 5), 'E', True)
    assert sorted(open_exits(b, (5, 5))) == ['S', 'W']


def test_open_exits_excludes_out_of_grid():
    from tugbot_maze.flood_fill_brain import FloodFillBrain, open_exits
    b = FloodFillBrain()
    b.mark((1, 0), 'E', True)            # entrance: S=(1,-1) and W=(0,0) out of grid
    assert open_exits(b, (1, 0)) == ['N']
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_flood_fill_brain.py -k open_exits -v`
Expected: FAIL — `ImportError: cannot import name 'open_exits'`.

- [ ] **Step 3: Implement** — append to the END of `tugbot_maze/flood_fill_brain.py`:

```python
def open_exits(brain, cell):
    """In-grid, not-walled directions out of `cell` (its open exits). Pure, read-only. The in_grid
    filter is load-bearing: a border cell's out-of-grid side must NOT count as an exit."""
    return [d for d, (dx, dy) in DIRS.items()
            if in_grid((cell[0] + dx, cell[1] + dy)) and not brain.is_wall(cell, d)]
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_flood_fill_brain.py -v`
Expected: PASS (all existing brain tests + the 2 new ones).

- [ ] **Step 5: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/flood_fill_brain.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_flood_fill_brain.py
git -C "$REPO" commit -m "feat: add pure open_exits(brain, cell) helper

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: `backout_command` (`hop_controller.py`)

**Files:**
- Modify: `tugbot_maze/hop_controller.py` (append after `profiled_turn_command`)
- Test: `test/test_hop_controller.py` (append + extend the import block)

- [ ] **Step 1: Write the failing tests** — append to `test/test_hop_controller.py`, and add `from tugbot_maze.hop_controller import backout_command` to the imports at the top:

```python
def test_backout_command_reverses_straight_when_aligned():
    v, w = backout_command(math.pi / 2, math.pi / 2)
    assert v == -0.30 and abs(w) < 1e-9


def test_backout_command_holds_heading():
    # yaw ahead of hold_cardinal -> steer back (negative w), still reversing
    v, w = backout_command(0.2, 0.0)
    assert v < 0.0 and w < 0.0


def test_backout_command_custom_speed():
    v, _ = backout_command(0.0, 0.0, backout_v=0.22)
    assert v == -0.22
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_hop_controller.py -k backout -v`
Expected: FAIL — `ImportError: cannot import name 'backout_command'`.

- [ ] **Step 3: Implement** — append to `tugbot_maze/hop_controller.py` after `profiled_turn_command`:

```python
def backout_command(yaw: float, hold_cardinal: float, *, backout_v: float = 0.30,
                    w_max: float = 0.5, kp_ang: float = 1.5) -> Tuple[float, float]:
    """Firm straight reverse out of a dead-end while holding `hold_cardinal`. Returns (v, w) with
    v = -backout_v (decisive, ~cruise speed -- NOT the 0.15 un-wedge crawl). Proportional steering
    on the heading error keeps the robot backing straight along the cardinal; the steering sign is
    correct for reverse travel because w rotates the body toward hold_cardinal regardless of the
    sign of v. No cross-track term -- the reverse is a single ~2 m cell retracing a path just
    driven centered, so heading-hold suffices."""
    err = _norm(hold_cardinal - yaw)
    w = max(-w_max, min(w_max, kp_ang * err))
    return (-backout_v, w)
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_hop_controller.py -v`
Expected: PASS (existing + 3 new).

- [ ] **Step 5: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/hop_controller.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_hop_controller.py
git -C "$REPO" commit -m "feat: add backout_command firm-reverse helper

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Back-out FSM phase (`maze_motion.py`)

**Files:**
- Modify: `tugbot_maze/maze_motion.py` (import, `__init__`, `step`, `_route`, new `_backout`)
- Test: `test/test_maze_motion.py` (append)

- [ ] **Step 1: Write the failing tests** — append to `test/test_maze_motion.py`:

```python
def _deadend_brain(d_cell, open_dir):
    """Brain where d_cell is walled on every in-grid side except open_dir."""
    from tugbot_maze.flood_fill_brain import FloodFillBrain, DIRS, in_grid
    b = FloodFillBrain()
    for d, (dx, dy) in DIRS.items():
        nb = (d_cell[0] + dx, d_cell[1] + dy)
        if in_grid(nb) and d != open_dir:
            b.mark(d_cell, d, True)
    return b


def test_deadend_triggers_backout():
    b = _deadend_brain((5, 5), 'S')            # open only S -> parent (5,4)
    m = MazeMotion(b)
    m.cell = (5, 5)
    m.hop_dir = (0, 1)                          # entered going N => came_from = 'S'
    m._route(10.0, 10.0, 0.0)
    assert m.phase == 'backout'
    assert m.backout_target == (5, 4)
    assert abs(m.backout_cardinal - math.pi / 2) < 1e-9   # face N (into the dead-end)
    assert m.backout_count == 1


def test_backout_reverses_then_advances():
    b = _deadend_brain((5, 5), 'S')
    m = MazeMotion(b)
    m.cell = (5, 5); m.hop_dir = (0, 1)
    m._route(10.0, 10.0, 0.0)                   # -> phase 'backout', start (10,10)
    v, w, _ = m._backout((10.0, 10.0, math.pi / 2), 0.1)
    assert v < 0.0 and abs(w) < 0.05            # firm straight reverse
    m._backout((10.0, 8.0, math.pi / 2), 0.2)   # moved one cell south into (5,4)
    assert m.cell == (5, 4) and m.phase == 'center'


def test_entrance_does_not_backout():
    from tugbot_maze.flood_fill_brain import FloodFillBrain
    b = FloodFillBrain()
    b.mark((1, 0), 'E', True)                   # entrance: only in-grid open exit is N (forward)
    m = MazeMotion(b)
    m.cell = (1, 0); m.hop_dir = None           # no came-from at the start
    m._route(2.0, 0.0, 0.0)
    assert m.phase == 'turn' and m.hop_target is not None


def test_backout_requires_exit_equals_came_from():
    b = _deadend_brain((5, 5), 'S')             # lone open exit is 'S'
    m = MazeMotion(b)
    m.cell = (5, 5); m.hop_dir = (0, -1)        # entered going S => came_from = 'N' != 'S'
    m._route(10.0, 10.0, 0.0)
    assert m.phase != 'backout'


def test_backout_timeout_escalates():
    b = _deadend_brain((5, 5), 'S')
    m = MazeMotion(b)
    m.cell = (5, 5); m.hop_dir = (0, 1)
    for _ in range(m.max_backout_attempts):     # each: enter backout, then time out (no arrival)
        m._route(10.0, 10.0, 0.0)
        assert m.phase == 'backout'
        m._backout((10.0, 10.0, math.pi / 2), m.backout_deadline + 0.1)
        assert m.phase == 'center'
    m._route(10.0, 10.0, 0.0)                   # attempts exhausted -> normal turn, no re-arm
    assert m.phase == 'turn'
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py -k "backout or entrance" -v`
Expected: FAIL — `AttributeError: 'MazeMotion' object has no attribute '_backout'` / `backout_count`.

- [ ] **Step 3a: Implement — import.** In `tugbot_maze/maze_motion.py` change the `flood_fill_brain` import (lines 23-24) to add `open_exits`:

```python
from tugbot_maze.flood_fill_brain import (
    FloodFillBrain, ENTRANCE_CELL, EXIT_CELL, CELL_SIZE_M, pose_to_cell, in_grid, DIRS, OPP,
    open_exits)
```

- [ ] **Step 3b: Implement — `__init__` params.** In the `__init__` signature, change the tail `boundary_margin_m: float = 0.40, k_corroborate: int = 2):` to:

```python
                 boundary_margin_m: float = 0.40, k_corroborate: int = 2,
                 backout_v: float = 0.30, backout_timeout_s: float = 12.0,
                 max_backout_attempts: int = 2):
```

- [ ] **Step 3c: Implement — `__init__` assignments + state.** After `self.k_corroborate = k_corroborate` add:

```python
        self.backout_v = backout_v; self.backout_timeout_s = backout_timeout_s
        self.max_backout_attempts = max_backout_attempts
```

And after `self.reopened = set()` add:

```python
        self.backout_target = None; self.backout_cardinal = 0.0
        self.backout_start = None; self.backout_deadline = 0.0
        self.backout_attempts = {}      # dead-end cell -> backout timeout count (bounds the loop)
        self.backout_count = 0          # observable: back-outs initiated (tests / regression)
```

- [ ] **Step 3d: Implement — dispatch.** In `step`, after the `recover` branch (`if self.phase == 'recover': return self._recover(pose, t)`), add:

```python
        if self.phase == 'backout':
            return self._backout(pose, t)
```

- [ ] **Step 3e: Implement — detection in `_route`.** Immediately after the unstick guard line `if nxt is None or not reachable: return self._unstick(t)`, insert:

```python
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
            self.backout_cardinal = math.atan2(-dy, -dx)   # face INTO the dead-end; reverse to parent
            self.backout_start = (x, y)
            self.backout_deadline = t + self.backout_timeout_s
            self.backout_count += 1
            self.center_start = None
            self.phase = 'backout'
            return (0.0, 0.0, False)
```

- [ ] **Step 3f: Implement — `_backout` method.** Add after `_recover` (end of class):

```python
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
            else:
                self.backout_attempts[self.cell] = self.backout_attempts.get(self.cell, 0) + 1
            self.settle_until = t + self.settle_s
            self.center_start = None
            self.align_start = None; self.latched_cardinal = None
            self.phase = 'center'
            return (0.0, 0.0, False)
        v, w = backout_command(yaw, self.backout_cardinal, backout_v=self.backout_v,
                               w_max=self.w_max, kp_ang=self.kp_turn)
        return (v, w, False)
```

Also add `backout_command` to the `hop_controller` import block at the top of `maze_motion.py` (it currently imports `centering_command, cross_track_offset, side_distances, corridor_follow_command, profiled_turn_command`):

```python
from tugbot_maze.hop_controller import (
    centering_command, cross_track_offset,
    side_distances, corridor_follow_command, profiled_turn_command, backout_command)
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py -v`
Expected: PASS — the 6 new tests plus all pre-existing ones (especially `test_center_senses_then_turns_toward_a_chosen_neighbor`, which the entrance gate preserves).

- [ ] **Step 5: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_motion.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_motion.py
git -C "$REPO" commit -m "feat: came-from-gated decisive dead-end back-out phase

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: `JunctionLog` + `update_junctions` (`junction_log.py`)

**Files:**
- Create: `tugbot_maze/junction_log.py`
- Test: `test/test_junction_log.py` (new)

- [ ] **Step 1: Write the failing tests** — create `test/test_junction_log.py`:

```python
import json


def test_observe_dedups_and_refreshes():
    from tugbot_maze.junction_log import JunctionLog
    jl = JunctionLog()
    e1, new1 = jl.observe((3, 4), ['N', 'E', 'W'], 2.0)
    assert new1 and e1['discovery_index'] == 1 and e1['exit_count'] == 3
    e2, new2 = jl.observe((3, 4), ['N', 'E', 'W', 'S'], 5.0)   # refresh exits, keep order/time
    assert not new2 and e2['discovery_index'] == 1 and e2['first_seen_s'] == 2.0
    assert e2['exit_count'] == 4 and jl.count == 1


def test_visit_counts_and_reflects_in_record():
    from tugbot_maze.junction_log import JunctionLog
    jl = JunctionLog()
    jl.visit((3, 4)); jl.visit((3, 4))         # 2 entries before record
    e, _ = jl.observe((3, 4), ['N', 'E', 'S'], 1.0)
    assert e['visits'] == 2
    jl.visit((3, 4))
    assert jl._j[(3, 4)]['visits'] == 3


def test_discovery_order_and_to_dict():
    from tugbot_maze.junction_log import JunctionLog
    jl = JunctionLog()
    jl.observe((1, 1), ['N', 'E', 'S'], 1.0)
    jl.observe((2, 2), ['N', 'E', 'W'], 2.0)
    d = jl.to_dict()
    assert d['junction_count'] == 2
    assert [j['cell'] for j in d['junctions']] == [[1, 1], [2, 2]]


def test_flush_writes_json(tmp_path):
    from tugbot_maze.junction_log import JunctionLog
    jl = JunctionLog()
    jl.observe((3, 4), ['N', 'E', 'W'], 1.0)
    path = str(tmp_path / 'sub' / 'junctions.json')
    jl.flush(path)
    data = json.loads(open(path).read())
    assert data['junction_count'] == 1 and data['junctions'][0]['cell'] == [3, 4]


def test_update_junctions_records_and_counts():
    from tugbot_maze.flood_fill_brain import FloodFillBrain
    from tugbot_maze.junction_log import JunctionLog, update_junctions
    b = FloodFillBrain()
    b.mark((5, 5), 'N', True)                  # (5,5): open S,E,W -> junction (3 exits)
    jl = JunctionLog()
    prev, j = update_junctions(jl, b, (5, 5), None, {(5, 5)}, 1.0)
    assert prev == (5, 5) and j is not None and j['exit_count'] == 3
    assert jl.count == 1 and jl._j[(5, 5)]['visits'] == 1
    prev, j2 = update_junctions(jl, b, (5, 5), prev, {(5, 5)}, 1.1)   # stationary: no re-record/visit
    assert j2 is None and jl._j[(5, 5)]['visits'] == 1


def test_update_junctions_ignores_non_junction_and_unsensed():
    from tugbot_maze.flood_fill_brain import FloodFillBrain
    from tugbot_maze.junction_log import JunctionLog, update_junctions
    jl = JunctionLog()
    b = FloodFillBrain()
    b.mark((5, 5), 'N', True); b.mark((5, 5), 'E', True)    # only 2 exits -> not a junction
    _, j = update_junctions(jl, b, (5, 5), None, {(5, 5)}, 1.0)
    assert j is None and jl.count == 0
    b2 = FloodFillBrain(); b2.mark((5, 5), 'N', True)        # junction, but NOT sensed
    _, j2 = update_junctions(jl, b2, (5, 5), None, set(), 1.0)
    assert j2 is None and jl.count == 0
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_junction_log.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'tugbot_maze.junction_log'`.

- [ ] **Step 3: Implement** — create `tugbot_maze/junction_log.py`:

```python
"""ROS-free per-run record of discovered junctions (cells with >= 3 open exits).

A passive memory asset / analyzable dataset: the JunctionLog stores each junction's coordinate,
open exits, discovery order, first-seen time, and visit count, and persists to JSON. update_junctions
is the pure per-tick glue the node calls; it owns the detection predicate so the node stays thin.
Routing is NOT affected by anything here.
"""
import json
import os
from typing import Dict, List, Optional, Tuple

from tugbot_maze.flood_fill_brain import open_exits

Cell = Tuple[int, int]


class JunctionLog:
    def __init__(self):
        self._j: Dict[Cell, dict] = {}
        self._order = 0
        self._visits: Dict[Cell, int] = {}

    @property
    def count(self) -> int:
        return len(self._j)

    def visit(self, cell: Cell) -> None:
        self._visits[cell] = self._visits.get(cell, 0) + 1
        if cell in self._j:
            self._j[cell]['visits'] = self._visits[cell]

    def observe(self, cell: Cell, exits: List[str], t: float,
                cell_size_m: float = 2.0) -> Tuple[dict, bool]:
        """Record on first sight (assign discovery_index + first_seen_s); on later sights refresh
        exits/exit_count while keeping discovery order, first-seen time, and visit count. Returns
        (entry, is_new)."""
        is_new = cell not in self._j
        if is_new:
            self._order += 1
            self._j[cell] = {
                'cell': [cell[0], cell[1]],
                'center_xy': [round(cell_size_m * cell[0], 3), round(cell_size_m * cell[1], 3)],
                'first_seen_s': round(t, 2),
                'discovery_index': self._order,
                'visits': self._visits.get(cell, 0),
            }
        e = self._j[cell]
        e['exits'] = list(exits)
        e['exit_count'] = len(exits)
        return e, is_new

    def to_dict(self) -> dict:
        return {
            'junction_count': len(self._j),
            'junctions': sorted(self._j.values(), key=lambda r: r['discovery_index']),
        }

    def flush(self, path: str) -> None:
        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)
        os.replace(tmp, path)   # atomic


def update_junctions(jlog: JunctionLog, brain, cell: Cell, prev_cell: Optional[Cell],
                     sensed, t: float) -> Tuple[Optional[Cell], Optional[dict]]:
    """Per-tick junction bookkeeping (ROS-free, unit-testable). Counts a visit on cell change; if
    `cell` is sensed with >= 3 in-grid open exits, observe() it. Returns
    (new_prev_cell, newly_recorded_entry_or_None) -- the node logs a JUNCTION line iff non-None."""
    if cell != prev_cell:
        jlog.visit(cell)
        prev_cell = cell
    new_entry = None
    if cell in sensed:
        exits = open_exits(brain, cell)
        if len(exits) >= 3:
            entry, is_new = jlog.observe(cell, exits, t)
            if is_new:
                new_entry = entry
    return prev_cell, new_entry
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_junction_log.py -v`
Expected: PASS (all 6).

- [ ] **Step 5: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/junction_log.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_junction_log.py
git -C "$REPO" commit -m "feat: add JunctionLog + update_junctions passive junction record

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: Node wiring (`flood_fill_solver.py`)

**Files:**
- Modify: `tugbot_maze/flood_fill_solver.py`

This is integration glue; the detection/visit/flush logic is already unit-tested in Task 4. Verify with a syntax compile + the existing import smoke test.

- [ ] **Step 1: Implement — imports.** Add `import os` next to `import math` (top of file), and add after the `from tugbot_maze.maze_motion import MazeMotion` line:

```python
from tugbot_maze.junction_log import JunctionLog, update_junctions
```

- [ ] **Step 2: Implement — params + state in `__init__`.** After `self.front_block_m = float(self.declare_parameter('front_block_m', 0.7).value)` add:

```python
        self.junction_log_dir = str(self.declare_parameter('junction_log_dir', '').value)
        self.junction_log_path = os.path.join(self.junction_log_dir or 'log', 'junctions.json')
```

And after `self.motion = MazeMotion(...)` (the multi-line constructor call) add:

```python
        self.junctions = JunctionLog()
        self._prev_motion_cell = None
```

- [ ] **Step 3: Implement — guarded flush helper.** Add a method (e.g. right after `_publish_cmd`):

```python
    def _flush_junctions(self):
        try:
            self.junctions.flush(self.junction_log_path)
        except Exception as e:                       # never let artifact IO kill the solver
            self.get_logger().warning('junction log flush failed: %r' % e)
```

- [ ] **Step 4: Implement — detect/record in the driving branch.** In `_control_tick`, in the `if self.phase == 'driving':` block, right after `self._publish_cmd(v, w)`, add:

```python
            self._prev_motion_cell, j = update_junctions(
                self.junctions, self.brain, self.motion.cell, self._prev_motion_cell,
                self.motion.sensed, t)
            if j is not None:
                self.get_logger().info('JUNCTION cell=%s exits=%s order=%d visits=%d'
                                       % (tuple(j['cell']), j['exits'],
                                          j['discovery_index'], j['visits']))
```

- [ ] **Step 5: Implement — flush on EXIT_REACHED.** In the same branch, after `self.goal_events_pub.publish(String(data='EXIT_REACHED'))`, add:

```python
                self._flush_junctions()
```

- [ ] **Step 6: Implement — periodic flush.** Make `_flush_junctions` run every diag tick: insert it as the FIRST statement of `_diag_tick` (before the `pose = self._lookup_pose()` line, so the early `return pose is None` cannot skip it):

```python
    def _diag_tick(self):
        self._flush_junctions()
        pose = self._lookup_pose()
```

- [ ] **Step 7: Verify (syntax + import)**

Run: `cd "$PKG" && python3 -m py_compile tugbot_maze/flood_fill_solver.py && echo COMPILE_OK`
Expected: `COMPILE_OK` (no SyntaxError).
Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_flood_fill_solver_smoke.py -v`
Expected: PASS or SKIP (skips only if `rclpy` is unavailable; if it runs, it confirms the module imports cleanly with the new code).

- [ ] **Step 8: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/flood_fill_solver.py
git -C "$REPO" commit -m "feat: wire passive junction logging into flood_fill_solver node

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: Launch arg (`tugbot_maze_explore.launch.py`)

**Files:**
- Modify: `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`

- [ ] **Step 1: Implement — pass the param to the node.** In `flood_fill_solver_node` (around line 203-205), add `junction_log_dir` to the parameters dict so it reads:

```python
        parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                     'pose_source': LaunchConfiguration('pose_source'),
                     'sense_debug': ParameterValue(LaunchConfiguration('sense_debug'), value_type=bool),
                     'junction_log_dir': LaunchConfiguration('junction_log_dir')}],
```

- [ ] **Step 2: Implement — declare the launch arg.** Immediately after the `DeclareLaunchArgument('sense_debug', ...)` line (~line 272), add:

```python
        DeclareLaunchArgument('junction_log_dir', default_value='', description='flood_fill: directory for the per-run junctions.json artifact (empty -> <cwd>/log).'),
```

- [ ] **Step 3: Verify (syntax)**

Run: `cd "$PKG/../tugbot_bringup/launch" && python3 -m py_compile tugbot_maze_explore.launch.py && echo COMPILE_OK`
Expected: `COMPILE_OK`.

- [ ] **Step 4: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_bringup/launch/tugbot_maze_explore.launch.py
git -C "$REPO" commit -m "feat: add junction_log_dir launch arg for flood_fill solver

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 7: Run script (`run_flood_fill_maze.sh`)

**Files:**
- Modify: `tools/run_flood_fill_maze.sh`

- [ ] **Step 1: Implement — pass the artifact dir.** In the `ros2 launch ...` invocation, add `junction_log_dir:="$ART"` to the args. The block becomes:

```bash
ros2 launch tugbot_bringup tugbot_maze_explore.launch.py \
    headless:="${HEADLESS}" use_rviz:="${USE_RVIZ}" \
    explorer_type:=flood_fill entry_direct_distance_m:=2.0 \
    pose_source:="${POSE_SOURCE}" sense_debug:="${SENSE_DEBUG}" \
    junction_log_dir:="$ART" \
    > "$ART/launch.log" 2>&1 &
```

- [ ] **Step 2: Implement — surface JUNCTION lines.** Add `JUNCTION` to the tail grep alternation (line ~60):

```bash
grep -aE "EXIT_REACHED|HOP_BACKUP|JUNCTION|DIAG|SENSE|flood_fill_solver" "$ART/launch.log" | tail -80 > "$ART/flood_fill_tail.txt" 2>/dev/null
```

- [ ] **Step 3: Verify (shell syntax)**

Run: `bash -n "$PKG/../../tools/run_flood_fill_maze.sh" && echo SH_OK`
Expected: `SH_OK`.

- [ ] **Step 4: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/tools/run_flood_fill_maze.sh
git -C "$REPO" commit -m "feat: run script passes junction_log_dir and greps JUNCTION

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 8: Offline regression proves back-out fired (`test_maze_motion_sim.py`)

**Files:**
- Modify: `test/test_maze_motion_sim.py`

The existing regression asserts only outcome (reached/collided/desync), so a broken or absent
back-out could stay green. Instrument `_run` and add a dedicated assertion that back-out actually
ran end-to-end and advanced the cell.

- [ ] **Step 1: Modify `_run` to track back-out and return it.** Replace the body of `_run` with (keep the signature):

```python
def _run(drift, latency=0, dt=0.1, max_steps=30000):
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0,
                  inertia=True, odom_drift_per_m=drift, cmd_latency_steps=latency)
    m = MazeMotion()
    t = 0.0
    collided = False
    max_desync = 0
    prev_phase = m.phase
    backout_cell = None
    backout_advances = 0
    for _ in range(max_steps):
        scan = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        v, w, done = m.step(sim.reported_pose, scan, t)   # navigate on (drifting) reported pose
        if prev_phase != 'backout' and m.phase == 'backout':
            backout_cell = m.cell                         # cell where this back-out began
        elif prev_phase == 'backout' and m.phase != 'backout':
            if backout_cell is not None and m.cell != backout_cell:
                backout_advances += 1                     # a back-out moved us to a new (parent) cell
        prev_phase = m.phase
        if done:
            return True, collided, max_desync, m.backout_count, backout_advances
        sim.step(v, w, dt)
        if sim.collides(sim.x, sim.y):                    # body entered a wall margin (true pose)
            collided = True
        if m.phase == 'center':                           # tracker-vs-physical sync at rest
            tc = pose_to_cell(sim.x, sim.y)
            max_desync = max(max_desync, abs(tc[0] - m.cell[0]) + abs(tc[1] - m.cell[1]))
        t += dt
    return (m.cell == EXIT_CELL), collided, max_desync, m.backout_count, backout_advances
```

- [ ] **Step 2: Update the parametrized test to the 5-tuple** (ignore the back-out fields here):

```python
@pytest.mark.parametrize("drift,latency", [(0.0, 0), (0.03, 0), (0.05, 0), (0.05, 2), (0.05, 3)])
def test_reaches_exit_without_collision_or_desync(drift, latency):
    reached, collided, max_desync, _, _ = _run(drift, latency)
    assert reached, f"did not reach the exit cell (drift={drift}, latency={latency})"
    assert not collided, f"robot body collided with a wall (drift={drift}, latency={latency})"
    assert max_desync <= 1, f"dcell desynced by {max_desync} (drift={drift}, latency={latency})"
```

- [ ] **Step 3: Add the back-out-fired regression** (deterministic baseline):

```python
def test_backout_is_exercised_end_to_end():
    """The decisive dead-end back-out must actually fire during the offline solve and advance the
    robot to the parent cell -- guards against a silently-disabled/regressed back-out that still
    happens to reach the exit via the fallback path."""
    reached, collided, _, backout_count, backout_advances = _run(0.0, 0)
    assert reached and not collided
    assert backout_count > 0, "dead-end back-out never fired in the offline solve"
    assert backout_advances > 0, "no back-out advanced the robot to the parent cell"
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion_sim.py -v`
Expected: PASS (all parametrized cases + `test_backout_is_exercised_end_to_end` + the unchanged centerline test).
*If `backout_count == 0`:* investigate whether the solve path reaches a genuine came-from dead-end (the review confirmed cells like (2,5)/(3,4) are entered); a 0 means the gate is too strict or the path changed — do not weaken the assertion without understanding why.

- [ ] **Step 5: Run the FULL suite to confirm no regressions**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py test/test_maze_motion_sim.py test/test_hop_controller.py test/test_flood_fill_brain.py test/test_junction_log.py -v`
Expected: ALL PASS.

- [ ] **Step 6: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_motion_sim.py
git -C "$REPO" commit -m "test: assert dead-end back-out fires end-to-end in offline regression

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Final validation (after all tasks)

1. **Full offline suite green** (Task 8 Step 5 command) — the strongest fast gate.
2. **Gazebo confirmation run** (manual, user-initiated; agent `pkill` is hook-blocked — the user
   clears stray sims first via `! pkill -9 -f "gz sim|ruby.*gz|parameter_bridge|flood_fill_solver|ros2 launch"`):
   `tools/run_flood_fill_maze.sh 1800 false true odom_locked true`
   Watch for: decisive reversals at dead-ends (no 3 s stalls, no body clips) and a populated
   `log/flood_fill_run_<stamp>/junctions.json` plus `JUNCTION` lines in `launch.log`.
3. Do **not** merge to `main` or push — banking/merge is a separate, user-approved step.
