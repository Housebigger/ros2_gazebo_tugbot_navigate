# Cell-Grid Flood-Fill (Micromouse) Maze Solver — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the stateless wall-follower with a memory-based micromouse flood-fill solver that autonomously discovers the maze on the known 2 m cell grid and reaches the exit cell while staying inside the perimeter.

**Architecture:** A ROS-free `FloodFillBrain` holds per-cell wall knowledge and flood-fills distance-to-exit (UNKNOWN edges optimistically open), choosing each next cell toward the known exit and re-flooding when a wall is sensed. `cell_walls` senses a cell's N/E/S/W edges from the SLAM occupancy grid; `hop_controller` drives cell-center→cell-center within the real motion envelope. A thin `flood_fill_solver` node wires `/map` + pose + sensing + brain + hops + the entrance seal. An inertia-aware `maze_sim` proves it offline (reaches the exit cell, stays inside) before Gazebo.

**Tech Stack:** Python 3.12, ROS 2 Jazzy (`rclpy`, `nav_msgs/OccupancyGrid`, `tf2_ros`), `pytest`, NumPy (offline sim only). All decision logic is ROS-free + unit-tested.

**Workspace:** `ros2_ws_tugbot_nav_20260614`. Branch `flood-fill-maze-solver` (already created, off the seal branch — so `entrance_seal_segment`, `outer_boundary_box`, `fuse_virtual_segment` are present). Run unit tests from `ros2_ws_tugbot_nav_20260614/src/tugbot_maze`; run git from the repo root `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`.

**Pre-verified facts (from a prototype against the real maze segments):** the 2 m grid has cell `(cx,cy)` centered at map `(2·cx, 2·cy)`; interior cells `cx∈1..10, cy∈0..9` (100 cells); entrance cell `(1,0)`, exit cell `(10,9)` (center (20,18), within 1.2 m of the exit point (21.07,18.08)). An edge is open iff the straight center→center segment is collision-free under `MazeSim.collides` (robot_radius 0.35 + wall_half_thickness 0.12). The maze is a verified tree (99 open edges over 100 cells). Flood-fill with ground-truth sensing reaches the exit in ~52 hops visiting ~41 cells (max 3 revisits). Real motion envelope: `v,w ≤ 0.5`, linear accel ≤ 0.5 m/s², angular accel ≤ 0.8 rad/s².

---

## File structure

| File | Responsibility |
|---|---|
| `tugbot_maze/flood_fill_brain.py` (new) | `FloodFillBrain`: per-cell wall memory + flood-fill + next-cell policy. ROS-free. |
| `tugbot_maze/cell_walls.py` (new) | `sense_cell_walls(grid_view, cell, ...)`: SLAM occupancy → per-edge WALL/OPEN/UNKNOWN. ROS-free given an `OccupancyGridView`. |
| `tugbot_maze/hop_controller.py` (new) | `hop_command(pose, target_xy, ...)`: cell-center→center drive command within the envelope. ROS-free. |
| `tugbot_maze/maze_sim.py` (modify) | + `ground_truth_edge_open(...)` (cell-edge model) and an inertia-aware step mode. |
| `tugbot_maze/flood_fill_solver.py` (new) | Thin node `explorer_type:=flood_fill`: `/map`→`OccupancyGridView`, pose→cell, sense, brain, hop, seal, exit-check. |
| `test/test_flood_fill_brain.py` (new) | Brain unit tests + real-maze cell-level proof. |
| `test/test_cell_walls.py` (new) | Sensing unit tests vs synthetic grids. |
| `test/test_hop_controller.py` (new) | Locomotion primitive unit tests. |
| `test/test_flood_fill_maze_sim.py` (new) | End-to-end offline guarantee (brain+sense+hop+inertia). |
| `test/test_flood_fill_solver_smoke.py` (new) | Node import smoke test. |
| `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py` (modify) | `explorer_type:=flood_fill` branch. |
| `tools/run_flood_fill_maze.sh` (new) | Runner (mirrors run_wall_follower_maze.sh). |

Shared constants (define once in `flood_fill_brain.py`, import elsewhere):
`CELL_SIZE_M = 2.0`, `CX_MIN, CX_MAX = 1, 10`, `CY_MIN, CY_MAX = 0, 9`, `ENTRANCE_CELL = (1, 0)`, `EXIT_CELL = (10, 9)`, `DIRS = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'W': (-1, 0)}`, `OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}`.

---

### Task 1: `FloodFillBrain` (memory + flood-fill policy)

**Files:**
- Create: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/flood_fill_brain.py`
- Test: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_flood_fill_brain.py`

- [ ] **Step 1: Write the failing tests**

Create `test/test_flood_fill_brain.py`:

```python
from tugbot_maze.flood_fill_brain import (
    FloodFillBrain, CELL_SIZE_M, ENTRANCE_CELL, EXIT_CELL, DIRS, OPP,
    cell_center, pose_to_cell)


def test_cell_center_and_pose_to_cell_roundtrip():
    assert cell_center((1, 0)) == (2.0, 0.0)
    assert cell_center((10, 9)) == (20.0, 18.0)
    assert pose_to_cell(2.1, -0.2) == (1, 0)
    assert pose_to_cell(19.6, 18.3) == (10, 9)


def test_flood_open_maze_distances_to_exit():
    # No walls known -> every edge optimistically open -> Manhattan distance to exit.
    b = FloodFillBrain()
    dist = b.flood()
    assert dist[EXIT_CELL] == 0
    assert dist[(9, 9)] == 1
    assert dist[(10, 8)] == 1
    assert dist[ENTRANCE_CELL] == abs(10 - 1) + abs(9 - 0)   # 18


def test_next_cell_heads_toward_exit_when_open():
    b = FloodFillBrain()
    # from entrance (1,0), open maze -> step that reduces Manhattan distance (N or E)
    nxt = b.next_cell((1, 0))
    assert nxt in {(1, 1), (2, 0)}


def test_mark_wall_is_symmetric_and_blocks():
    b = FloodFillBrain()
    b.mark((1, 0), 'N', is_wall=True)
    assert b.is_wall((1, 0), 'N') is True
    assert b.is_wall((1, 1), 'S') is True          # symmetric
    # with N walled at (1,0), next_cell must not choose (1,1)
    assert b.next_cell((1, 0)) != (1, 1)


def test_reflood_reroutes_around_new_wall():
    b = FloodFillBrain()
    # Wall off the whole exit column except via the top: force a detour by walling
    # the direct E step from (1,0); next_cell should then prefer N.
    b.mark((1, 0), 'E', is_wall=True)
    assert b.next_cell((1, 0)) == (1, 1)


def test_backtracks_when_only_open_edge_increases_distance():
    # At (5,5) wall off N, E, S -> only W is open. next_cell must still return the
    # one open neighbor (4,5) even though it steps AWAY from the exit (backtracking),
    # rather than getting stuck.
    b = FloodFillBrain()
    for d in ('N', 'E', 'S'):
        b.mark((5, 5), d, is_wall=True)
    assert b.next_cell((5, 5)) == (4, 5)


def test_is_done_only_at_exit():
    b = FloodFillBrain()
    assert b.is_done(EXIT_CELL) is True
    assert b.is_done(ENTRANCE_CELL) is False
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_flood_fill_brain.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'tugbot_maze.flood_fill_brain'`.

- [ ] **Step 3: Implement `flood_fill_brain.py`**

Create `tugbot_maze/flood_fill_brain.py`:

```python
"""ROS-free micromouse flood-fill brain for the 2 m maze cell grid.

Memory = per-edge wall knowledge (WALL / OPEN / UNKNOWN). flood() computes
distance-to-exit by BFS from the exit over edges that are not WALL (UNKNOWN is
optimistically passable). next_cell() steps toward the exit along the lowest-
distance non-WALL neighbor. Deterministic; no ROS / time / I/O.
"""
from __future__ import annotations
import collections
import math
from typing import Dict, Optional, Tuple

Cell = Tuple[int, int]

CELL_SIZE_M = 2.0
CX_MIN, CX_MAX = 1, 10
CY_MIN, CY_MAX = 0, 9
ENTRANCE_CELL: Cell = (1, 0)
EXIT_CELL: Cell = (10, 9)
DIRS = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'W': (-1, 0)}
OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}

WALL, OPEN, UNKNOWN = 'wall', 'open', 'unknown'


def cell_center(cell: Cell) -> Tuple[float, float]:
    return (CELL_SIZE_M * cell[0], CELL_SIZE_M * cell[1])


def pose_to_cell(x: float, y: float) -> Cell:
    return (int(round(x / CELL_SIZE_M)), int(round(y / CELL_SIZE_M)))


def in_grid(cell: Cell) -> bool:
    return CX_MIN <= cell[0] <= CX_MAX and CY_MIN <= cell[1] <= CY_MAX


class FloodFillBrain:
    def __init__(self, exit_cell: Cell = EXIT_CELL):
        self.exit_cell = exit_cell
        self._edge: Dict[Tuple[Cell, str], str] = {}   # (cell, dir) -> WALL/OPEN

    def _state(self, cell: Cell, d: str) -> str:
        return self._edge.get((cell, d), UNKNOWN)

    def is_wall(self, cell: Cell, d: str) -> bool:
        return self._state(cell, d) == WALL

    def mark(self, cell: Cell, d: str, is_wall: bool) -> None:
        st = WALL if is_wall else OPEN
        self._edge[(cell, d)] = st
        dx, dy = DIRS[d]
        nb = (cell[0] + dx, cell[1] + dy)
        self._edge[(nb, OPP[d])] = st            # keep knowledge symmetric

    def _passable(self, cell: Cell, d: str) -> bool:
        return self._state(cell, d) != WALL       # OPEN or UNKNOWN are passable

    def flood(self) -> Dict[Cell, int]:
        dist = {self.exit_cell: 0}
        q = collections.deque([self.exit_cell])
        while q:
            c = q.popleft()
            for d, (dx, dy) in DIRS.items():
                nb = (c[0] + dx, c[1] + dy)
                if in_grid(nb) and self._passable(c, d) and nb not in dist:
                    dist[nb] = dist[c] + 1
                    q.append(nb)
        return dist

    def next_cell(self, cur: Cell) -> Optional[Cell]:
        if cur == self.exit_cell:
            return None
        dist = self.flood()
        best, best_d = None, math.inf
        for d, (dx, dy) in DIRS.items():
            nb = (cur[0] + dx, cur[1] + dy)
            if in_grid(nb) and self._passable(cur, d) and dist.get(nb, math.inf) < best_d:
                best, best_d = nb, dist[nb]
        return best

    def is_done(self, cell: Cell) -> bool:
        return cell == self.exit_cell
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_flood_fill_brain.py -q`
Expected: PASS (7 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/flood_fill_brain.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_flood_fill_brain.py
git commit -m "feat(flood-fill): add FloodFillBrain (memory + distance-to-exit flood-fill)"
```

---

### Task 2: Ground-truth cell-edge model + inertia-aware sim (in `maze_sim.py`)

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_sim.py`
- Test: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_sim.py` (append)

- [ ] **Step 1: Write the failing tests**

Append to `test/test_maze_sim.py`:

```python
def test_ground_truth_edge_open_matches_perfect_maze_tree():
    # The 20260528 maze is a perfect maze: 100 cells, 99 undirected open edges.
    from tugbot_maze.maze_sim import MazeSim, load_segments, ground_truth_edge_open
    from tugbot_maze.flood_fill_brain import in_grid, DIRS, cell_center
    sim = MazeSim(load_segments(), (2.0, 0.0), 0.0)
    undirected = set()
    for cx in range(1, 11):
        for cy in range(0, 10):
            for d, (dx, dy) in DIRS.items():
                nb = (cx + dx, cy + dy)
                if in_grid(nb) and ground_truth_edge_open(sim, (cx, cy), nb):
                    undirected.add(frozenset({(cx, cy), nb}))
    assert len(undirected) == 99


def test_inertia_step_rate_limits_and_clamps():
    from tugbot_maze.maze_sim import MazeSim, load_segments
    sim = MazeSim(load_segments(), (10.0, 10.0), 0.0, inertia=True)
    # command beyond the envelope; one 0.1 s step can change v by <= 0.5*0.1 = 0.05
    sim.step(1.0, 1.0, 0.1)
    assert sim.v_cur == pytest.approx(0.05, abs=1e-9)     # accel-limited toward clamp(1.0)->0.5
    assert sim.w_cur == pytest.approx(0.08, abs=1e-9)     # ang accel 0.8 * 0.1
    # eventually saturates at the clamp, not the commanded 1.0
    for _ in range(200):
        sim.step(1.0, 1.0, 0.1)
    assert sim.v_cur == pytest.approx(0.5, abs=1e-6)
    assert sim.w_cur == pytest.approx(0.5, abs=1e-6)
```

(`test/test_maze_sim.py` already imports `pytest`; if not, add `import pytest` at the top.)

- [ ] **Step 2: Run to verify they fail**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_maze_sim.py -q`
Expected: FAIL (`cannot import name 'ground_truth_edge_open'`; `MazeSim` has no `inertia`/`v_cur`).

- [ ] **Step 3: Implement in `maze_sim.py`**

Add this module-level function (after `outer_boundary_box`):

```python
def ground_truth_edge_open(sim: "MazeSim", a, b, samples: int = 10) -> bool:
    """True if the robot can traverse from cell-center a to cell-center b without
    colliding (samples the straight connector against the real wall segments).
    a, b are (cx, cy) cells with centers at (2*cx, 2*cy)."""
    ax, ay = 2.0 * a[0], 2.0 * a[1]
    bx, by = 2.0 * b[0], 2.0 * b[1]
    for i in range(samples + 1):
        t = i / samples
        if sim.collides(ax + (bx - ax) * t, ay + (by - ay) * t):
            return False
    return True
```

Add inertia to `MazeSim`. In `__init__`, add the parameter and state (keep the default `inertia=False` so existing tests are unaffected):

```python
    def __init__(self, segments, start_xy, start_yaw, *, robot_radius_m=0.35,
                 wall_half_thickness_m=0.12, max_range_m=12.0, inertia=False,
                 v_max=0.5, w_max=0.5, lin_accel=0.5, ang_accel=0.8):
        # ... existing body unchanged ...
        self.inertia = inertia
        self.v_max = v_max
        self.w_max = w_max
        self.lin_accel = lin_accel
        self.ang_accel = ang_accel
        self.v_cur = 0.0
        self.w_cur = 0.0
```

Replace `step` so that, when `inertia=True`, the commanded `(v, w)` is clamped to
the envelope and rate-limited before integrating:

```python
    def step(self, v, w, dt):
        if self.inertia:
            v_cmd = max(-self.v_max, min(self.v_max, v))
            w_cmd = max(-self.w_max, min(self.w_max, w))
            self.v_cur += max(-self.lin_accel * dt, min(self.lin_accel * dt, v_cmd - self.v_cur))
            self.w_cur += max(-self.ang_accel * dt, min(self.ang_accel * dt, w_cmd - self.w_cur))
            v, w = self.v_cur, self.w_cur
        nx = self.x + v * math.cos(self.yaw) * dt
        ny = self.y + v * math.sin(self.yaw) * dt
        if not self.collides(nx, ny):
            self.x, self.y = nx, ny
        self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))
```

- [ ] **Step 4: Run to verify pass**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_maze_sim.py -q`
Expected: PASS (existing maze_sim tests + the 2 new ones; the 99-edge assertion confirms the cell model).

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_sim.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_sim.py
git commit -m "feat(flood-fill): ground-truth cell-edge model + inertia-aware sim step"
```

---

### Task 3: `cell_walls` — sense edges from the SLAM occupancy grid

**Files:**
- Create: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/cell_walls.py`
- Test: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_cell_walls.py`

- [ ] **Step 1: Write the failing tests**

Create `test/test_cell_walls.py`:

```python
from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView
from tugbot_maze.cell_walls import sense_cell_walls


def _grid(occupied_cells, width=24, height=22, res=1.0, ox=-1.0, oy=-1.0):
    # build a fine occupancy grid; mark given (gx,gy) fine-cells occupied (=100)
    data = [0] * (width * height)
    for gx, gy in occupied_cells:
        data[gy * width + gx] = 100
    return OccupancyGridView(OccupancyGridInfo(width, height, res, ox, oy), data)


def test_open_neighbourhood_reports_all_open():
    # all-free grid around maze-cell (1,0) center (2,0) -> every edge OPEN (False=not wall)
    view = _grid([])
    walls = sense_cell_walls(view, (1, 0))
    assert walls == {'N': False, 'S': False, 'E': False, 'W': False}


def test_wall_on_north_edge_detected():
    # occupy the fine cells along the north edge of maze-cell (1,0): map y≈1.0, x in [1,3]
    # origin (-1,-1), res 1.0 -> map (mx,my) -> fine ( mx+1, my+1 ); north edge y=1 -> gy=2
    occ = [(gx, 2) for gx in range(2, 5)]    # x≈1..3 -> gx 2..4
    view = _grid(occ)
    walls = sense_cell_walls(view, (1, 0))
    assert walls['N'] is True
    assert walls['E'] is False


def test_unknown_edge_returns_none():
    # mark the north-edge fine cells UNKNOWN (-1); cell_walls leaves that edge None
    view = _grid([])
    for gx in range(2, 5):
        view.data[2 * view.info.width + gx] = -1
    walls = sense_cell_walls(view, (1, 0))
    assert walls['N'] is None
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_cell_walls.py -q`
Expected: FAIL (`No module named 'tugbot_maze.cell_walls'`).

- [ ] **Step 3: Implement `cell_walls.py`**

Create `tugbot_maze/cell_walls.py`:

```python
"""Sense a maze cell's N/E/S/W edges from a SLAM OccupancyGridView.

The maze cell grid is 2 m (cell (cx,cy) centered at (2*cx, 2*cy)). Each edge is
the boundary line ~1 m from the center toward a neighbor. We sample occupancy at
points across that edge: mostly-occupied -> WALL (True), clearly free -> OPEN
(False), too-unknown -> None (leave UNKNOWN, re-sense later).
"""
from __future__ import annotations
from typing import Dict, Optional

from tugbot_maze.flood_fill_brain import CELL_SIZE_M, DIRS


def sense_cell_walls(grid_view, cell, *, cell_size: float = CELL_SIZE_M,
                     n_samples: int = 5, occupied_frac: float = 0.34,
                     unknown_frac: float = 0.6) -> Dict[str, Optional[bool]]:
    cx, cy = cell
    ccx, ccy = cell_size * cx, cell_size * cy
    half = cell_size / 2.0
    out: Dict[str, Optional[bool]] = {}
    for d, (dx, dy) in DIRS.items():
        # edge midpoint, and the in-plane axis to sample along
        emx, emy = ccx + dx * half, ccy + dy * half
        ax, ay = (0.0, 1.0) if dx != 0 else (1.0, 0.0)   # sample perpendicular to travel
        occ = unk = 0
        for i in range(n_samples):
            t = (i / (n_samples - 1) - 0.5) * cell_size    # -half..+half along the edge
            px, py = emx + ax * t, emy + ay * t
            gcell = grid_view.world_to_cell(px, py)
            if grid_view.is_unknown(gcell):
                unk += 1
            elif grid_view.is_occupied(gcell):
                occ += 1
        if unk / n_samples >= unknown_frac:
            out[d] = None
        else:
            out[d] = (occ / n_samples) >= occupied_frac
    return out
```

- [ ] **Step 4: Run to verify pass**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_cell_walls.py -q`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/cell_walls.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_cell_walls.py
git commit -m "feat(flood-fill): cell_walls edge sensing from the occupancy grid"
```

---

### Task 4: `hop_controller` — cell-center→center drive within the envelope

**Files:**
- Create: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/hop_controller.py`
- Test: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_hop_controller.py`

- [ ] **Step 1: Write the failing tests**

Create `test/test_hop_controller.py`:

```python
import math
from tugbot_maze.hop_controller import hop_command


def test_arrived_when_within_tolerance():
    v, w, arrived = hop_command((2.0, 0.0, 0.0), (2.0, 0.0))
    assert arrived is True
    assert (v, w) == (0.0, 0.0)


def test_drives_forward_when_aligned():
    v, w, arrived = hop_command((0.0, 0.0, 0.0), (2.0, 0.0))   # target straight ahead
    assert arrived is False
    assert v > 0.0
    assert abs(w) < 1e-6


def test_turns_toward_target_and_slows():
    # target to the left (90 deg) -> turn left (+w), and v throttled near 0 while mis-aligned
    v, w, arrived = hop_command((0.0, 0.0, 0.0), (0.0, 2.0))
    assert w > 0.0
    assert v < 0.1


def test_commands_within_envelope():
    v, w, _ = hop_command((0.0, 0.0, 0.0), (0.0, 2.0))   # large heading error
    assert -0.5 <= v <= 0.5
    assert -0.5 <= w <= 0.5
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_hop_controller.py -q`
Expected: FAIL (`No module named 'tugbot_maze.hop_controller'`).

- [ ] **Step 3: Implement `hop_controller.py`**

Create `tugbot_maze/hop_controller.py`:

```python
"""ROS-free point-to-point drive command for a single cell hop, within the real
motion envelope (v,w <= 0.5). P-control on heading; forward speed is throttled by
heading error so the robot turns toward the target before driving (no overshoot)."""
from __future__ import annotations
import math
from typing import Tuple


def _norm(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def hop_command(pose, target_xy, *, v_max: float = 0.5, w_max: float = 0.5,
                kp_ang: float = 1.5, slow_angle: float = 0.6,
                arrive_m: float = 0.25) -> Tuple[float, float, bool]:
    px, py, yaw = pose
    dx, dy = target_xy[0] - px, target_xy[1] - py
    dist = math.hypot(dx, dy)
    if dist <= arrive_m:
        return (0.0, 0.0, True)
    err = _norm(math.atan2(dy, dx) - yaw)
    w = max(-w_max, min(w_max, kp_ang * err))
    throttle = max(0.0, 1.0 - abs(err) / slow_angle)   # 0 when |err|>=slow_angle
    v = max(0.0, min(v_max, v_max * throttle))
    return (v, w, False)
```

- [ ] **Step 4: Run to verify pass**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_hop_controller.py -q`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/hop_controller.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_hop_controller.py
git commit -m "feat(flood-fill): hop_controller for in-envelope cell-center drives"
```

---

### Task 5: End-to-end offline guarantee (brain + sensing + hop + inertia)

This is the gate: the brain, ground-truth edge sensing, the hop controller, and the
inertia-aware unicycle, integrated — must reach the exit cell staying inside.

**Files:**
- Test: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_flood_fill_maze_sim.py`

- [ ] **Step 1: Write the guarantee test**

Create `test/test_flood_fill_maze_sim.py`:

```python
import math
from tugbot_maze.maze_sim import MazeSim, load_segments, outer_boundary_box, ground_truth_edge_open
from tugbot_maze.flood_fill_brain import (
    FloodFillBrain, ENTRANCE_CELL, EXIT_CELL, DIRS, in_grid, cell_center, pose_to_cell)
from tugbot_maze.hop_controller import hop_command

OUTSIDE_TOL_M = 0.4


def _sense(sim, brain, cell):
    for d, (dx, dy) in DIRS.items():
        nb = (cell[0] + dx, cell[1] + dy)
        brain.mark(cell, d, is_wall=not (in_grid(nb) and ground_truth_edge_open(sim, cell, nb)))


def run(max_hops=200, dt=0.1, max_ticks_per_hop=400):
    segs = load_segments()
    bx0, bx1, by0, by1 = outer_boundary_box()
    start = cell_center(ENTRANCE_CELL)
    sim = MazeSim(segs, start, 0.0, inertia=True)
    brain = FloodFillBrain()
    outside = 0
    for hop in range(max_hops):
        cur = pose_to_cell(sim.x, sim.y)
        _sense(sim, brain, cur)
        if brain.is_done(cur):
            return True, hop, outside
        nxt = brain.next_cell(cur)
        if nxt is None:
            return False, hop, outside
        target = cell_center(nxt)
        for _ in range(max_ticks_per_hop):
            v, w, arrived = hop_command((sim.x, sim.y, sim.yaw), target)
            sim.step(v, w, dt)
            if max(bx0 - sim.x, sim.x - bx1, by0 - sim.y, sim.y - by1) > OUTSIDE_TOL_M:
                outside += 1
            if arrived:
                break
    return pose_to_cell(sim.x, sim.y) == EXIT_CELL, max_hops, outside


def test_flood_fill_reaches_exit_staying_inside():
    reached, hops, outside = run()
    assert reached, f"flood-fill did not reach the exit cell (hops={hops})"
    assert outside == 0, f"left the maze {outside} times"
```

- [ ] **Step 2: Run to verify it fails, then passes**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_flood_fill_maze_sim.py -q`
Expected before Tasks 1–4 exist: import error. After them: **PASS** — the robot reaches `(10,9)` (≈50–60 hops) with `outside == 0`. If it fails to reach or `outside > 0`, STOP and report (the hop controller or sensing needs work) — do not weaken the assertion.

- [ ] **Step 3: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_flood_fill_maze_sim.py
git commit -m "test(flood-fill): inertia-aware offline guarantee reaches exit, stays inside"
```

---

### Task 6: `flood_fill_solver` node

Mirror the structure of `wall_follow_solver.py` (same ROS plumbing: `/cmd_vel_nav`
publisher, `goal_events` String publisher, map→base_link pose, control timer, DIAG
timer, the entrance seal, exit self-check). Replace the wall-follower control with the
cell loop.

**Files:**
- Create: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/flood_fill_solver.py`
- Modify: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/setup.py` (add console_script)
- Test: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_flood_fill_solver_smoke.py`

- [ ] **Step 1: Write the smoke test**

Create `test/test_flood_fill_solver_smoke.py`:

```python
import pytest


def test_node_module_imports():
    pytest.importorskip("rclpy")
    from tugbot_maze import flood_fill_solver
    assert hasattr(flood_fill_solver, "FloodFillSolver")
    assert hasattr(flood_fill_solver, "main")
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_flood_fill_solver_smoke.py -q`
Expected: FAIL (`No module named 'tugbot_maze.flood_fill_solver'`).

- [ ] **Step 3: Implement `flood_fill_solver.py`**

Create `tugbot_maze/flood_fill_solver.py`:

```python
"""Thin ROS 2 node: cell-grid flood-fill (micromouse) maze solver.

Subscribes /map (-> OccupancyGridView) and uses map->base_link TF for pose. Each
control tick: localize to a maze cell, sense its edges (cell_walls), ask the
FloodFillBrain for the next cell, and drive there with hop_command. Owns the entry
drive, a per-hop stall watchdog, and the exit self-check. No LIDAR entrance seal is
needed here: the brain only ever targets interior cells (cx 1..10), so the robot is
confined to the interior and cannot reach the entrance opening or the exterior (a
stronger guarantee than the wall-follower's seal). ROS plumbing mirrors
wall_follow_solver.py.
"""
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import tf2_ros

from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView
from tugbot_maze.flood_fill_brain import (
    FloodFillBrain, ENTRANCE_CELL, EXIT_CELL, DIRS, in_grid, cell_center, pose_to_cell)
from tugbot_maze.cell_walls import sense_cell_walls
from tugbot_maze.hop_controller import hop_command
from tugbot_maze.wall_follow_control import exit_reached, entering_done


class FloodFillSolver(Node):
    def __init__(self):
        super().__init__('flood_fill_solver')
        self.scan_topic = self.declare_parameter('map_topic', '/map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.goal_events_topic = self.declare_parameter('goal_events_topic', '/maze/goal_events').value
        self.exit_x = float(self.declare_parameter('exit_x', 21.072562).value)
        self.exit_y = float(self.declare_parameter('exit_y', 18.083566).value)
        self.exit_radius = float(self.declare_parameter('exit_radius', 1.2).value)
        self.entry_direct_distance_m = float(self.declare_parameter('entry_direct_distance_m', 2.0).value)
        self.startup_delay_sec = float(self.declare_parameter('startup_delay_sec', 3.0).value)
        self.hop_timeout_s = float(self.declare_parameter('hop_timeout_s', 25.0).value)
        self.backup_s = float(self.declare_parameter('backup_s', 1.0).value)
        self.backup_v = float(self.declare_parameter('backup_v', -0.15).value)
        self.cruise_v = float(self.declare_parameter('cruise_v', 0.3).value)

        self.brain = FloodFillBrain(exit_cell=EXIT_CELL)
        self.map_view: Optional[OccupancyGridView] = None
        self.create_subscription(OccupancyGrid, self.scan_topic, self._map_cb, self._map_qos())
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.goal_events_pub = self.create_publisher(String, self.goal_events_topic, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.phase = 'startup'        # startup | entering | hop | backup | done
        self.start_xy = None
        self.target_xy = None
        self.hop_deadline = 0.0
        self.backup_until = 0.0
        self.create_timer(0.1, self._control_tick)
        self.create_timer(5.0, self._diag_tick)
        self.start_time = self.get_clock().now()
        self.get_logger().info('flood_fill_solver started.')

    def _map_qos(self):
        return QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=QoSReliabilityPolicy.RELIABLE,
                          history=QoSHistoryPolicy.KEEP_LAST)

    def _map_cb(self, msg):
        self.map_view = OccupancyGridView(
            info=OccupancyGridInfo(width=int(msg.info.width), height=int(msg.info.height),
                                   resolution=float(msg.info.resolution),
                                   origin_x=float(msg.info.origin.position.x),
                                   origin_y=float(msg.info.origin.position.y)),
            data=list(msg.data))

    def _lookup_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def _publish_cmd(self, v, w):
        m = Twist(); m.linear.x = float(v); m.angular.z = float(w); self.cmd_vel_pub.publish(m)

    def _sense(self, cur):
        if self.map_view is None:
            return
        walls = sense_cell_walls(self.map_view, cur)
        for d, is_wall in walls.items():
            if is_wall is not None:
                self.brain.mark(cur, d, is_wall)

    def _control_tick(self):
        now = self.get_clock().now(); t = now.nanoseconds / 1e9
        pose = self._lookup_pose()
        if pose is not None and exit_reached(pose, (self.exit_x, self.exit_y), self.exit_radius):
            if self.phase != 'done':
                self.phase = 'done'
                self.get_logger().info('EXIT_REACHED (flood_fill_solver)')
                self.goal_events_pub.publish(String(data='EXIT_REACHED'))
                self._publish_cmd(0.0, 0.0)
            return
        if self.phase == 'done':
            return
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if self.phase == 'startup':
            if elapsed >= self.startup_delay_sec and pose is not None:
                self.start_xy = (pose[0], pose[1]); self.phase = 'entering'
            return
        if pose is None:
            return
        if self.phase == 'entering':
            if entering_done(self.start_xy, (pose[0], pose[1]), self.entry_direct_distance_m):
                self.phase = 'hop'; self.target_xy = None
            else:
                self._publish_cmd(self.cruise_v, 0.0)
            return
        if self.phase == 'backup':
            if t >= self.backup_until:
                self.phase = 'hop'; self.target_xy = None
            else:
                self._publish_cmd(self.backup_v, 0.0)
            return
        # phase == 'hop'
        cur = pose_to_cell(pose[0], pose[1])
        if self.target_xy is None:
            self._sense(cur)
            nxt = self.brain.next_cell(cur)
            if nxt is None:
                self._publish_cmd(0.0, 0.0); return     # at exit (caught above) or stuck
            self.target_xy = cell_center(nxt)
            self.hop_deadline = t + self.hop_timeout_s
        v, w, arrived = hop_command(pose, self.target_xy)
        if arrived:
            self.target_xy = None                       # re-sense + choose next at the new cell
            return
        if t >= self.hop_deadline:                      # wedged -> back up and retry
            self.phase = 'backup'; self.backup_until = t + self.backup_s
            self.goal_events_pub.publish(String(data='HOP_BACKUP')); return
        self._publish_cmd(v, w)

    def _diag_tick(self):
        pose = self._lookup_pose()
        if pose is None:
            return
        cur = pose_to_cell(pose[0], pose[1])
        dist = math.hypot(pose[0] - self.exit_x, pose[1] - self.exit_y)
        self.get_logger().info('DIAG pose=(%.2f, %.2f) cell=%s dist_to_exit=%.2f phase=%s'
                               % (pose[0], pose[1], cur, dist, self.phase))


def main(args=None):
    rclpy.init(args=args)
    node = FloodFillSolver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Add the console_script in `setup.py`**

In `setup.py`, add to `entry_points['console_scripts']` (after the `wall_follow_solver` line):

```python
            'flood_fill_solver = tugbot_maze.flood_fill_solver:main',
```

- [ ] **Step 5: Run the smoke test + full ROS-free suite**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_flood_fill_brain.py test/test_cell_walls.py test/test_hop_controller.py test/test_flood_fill_maze_sim.py test/test_flood_fill_solver_smoke.py test/test_maze_sim.py -q`
Expected: PASS (the smoke test skips if rclpy unavailable; passes under colcon). If rclpy is sourced, also: `python3 -c "from tugbot_maze import flood_fill_solver; print('imports OK')"`.

- [ ] **Step 6: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/flood_fill_solver.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/setup.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_flood_fill_solver_smoke.py
git commit -m "feat(flood-fill): flood_fill_solver node (explorer_type:=flood_fill)"
```

---

### Task 7: Launch branch + runner

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260614/src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`
- Create: `ros2_ws_tugbot_nav_20260614/tools/run_flood_fill_maze.sh`

- [ ] **Step 1: Add the `flood_fill` node to the launch**

In `tugbot_maze_explore.launch.py`, mirror the existing `wall_follow_solver_node` block. Add a node guarded by `explorer_type == 'flood_fill'`:

```python
    flood_fill_solver_node = Node(
        package='tugbot_maze', executable='flood_fill_solver', name='flood_fill_solver',
        output='screen', parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", explorer_type, "' == 'flood_fill'"])),
    )
```

Add `flood_fill_solver_node` to the same `TimerAction(period=13.0, ...)` actions list that launches `wall_follow_solver_node`. (Find that block by searching for `wall_follow_solver_node` and add the new node alongside it.)

- [ ] **Step 2: Create the runner `tools/run_flood_fill_maze.sh`**

Copy `tools/run_wall_follower_maze.sh` to `tools/run_flood_fill_maze.sh` and change only: the `ART` prefix to `flood_fill_run_`, the log tag to `[FLOODFILL]`, and the launch line's explorer args to:

```bash
ros2 launch tugbot_bringup tugbot_maze_explore.launch.py \
    headless:="${HEADLESS}" use_rviz:="${USE_RVIZ}" \
    explorer_type:=flood_fill entry_direct_distance_m:=2.0 \
    > "$ART/launch.log" 2>&1 &
```

Drop the `FOLLOW_SIDE` argument (flood-fill has no hand). Keep the same SHM/process hygiene (`kill_all_sim`, `rm -f /dev/shm/fastrtps_*`, unique `ROS_DOMAIN_ID`) and the EXIT_REACHED/EXHAUSTED/DDS_SHM_FAIL monitor loop. Make it executable: `chmod +x tools/run_flood_fill_maze.sh`.

- [ ] **Step 3: Build + sanity-check the launch parses**

Run: `cd ros2_ws_tugbot_nav_20260614 && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select tugbot_maze tugbot_bringup`
Expected: 2 packages finished.
Run: `source install/setup.bash && ros2 launch tugbot_bringup tugbot_maze_explore.launch.py explorer_type:=flood_fill headless:=true --show-args 2>/dev/null | head -1 || true`
(Just confirms the launch file imports/parses; full run is the Gazebo validation step.)

- [ ] **Step 4: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260614/src/tugbot_bringup/launch/tugbot_maze_explore.launch.py \
        ros2_ws_tugbot_nav_20260614/tools/run_flood_fill_maze.sh
git commit -m "feat(flood-fill): launch explorer_type:=flood_fill + run_flood_fill_maze.sh"
```

---

## Final verification (after all tasks)

- [ ] Full ROS-free suite green:

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_flood_fill_brain.py test/test_cell_walls.py test/test_hop_controller.py test/test_flood_fill_maze_sim.py test/test_maze_sim.py -q`
Expected: all PASS — including the end-to-end guarantee reaching the exit cell with `outside == 0`.

- [ ] Rebuild: `cd ros2_ws_tugbot_nav_20260614 && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install` → 6 packages finished.

- [ ] **Gazebo re-validation (acceptance gate, user-watched):** GUI run via `DISPLAY=:1 bash tools/run_flood_fill_maze.sh 1800 false true`, then regenerate the trajectory-vs-maze plot. Acceptance = `EXIT_REACHED` from the interior **AND 0 trajectory samples outside the outer-wall box**, ideally repeatable (≥4/5). This is the real-vs-offline gate (the offline guarantee senses ground-truth edges; Gazebo exercises `cell_walls` against the live SLAM grid — the one piece not covered offline). Tracked as a separate task; run after the branch is implemented.
