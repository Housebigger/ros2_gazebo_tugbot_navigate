# Trémaux Autonomous Maze Solver — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a clean, unit-testable autonomous maze-solving ROS 2 node (`explorer_type:=tremaux`) in a new `ros2_ws_tugbot_nav_20260614` workspace that reaches the maze exit fully autonomously (≥4/5 runs) by reliably backing out of dead ends and rerouting into unexplored passages.

**Architecture:** A thin ROS node (`maze_solver.py`) wires three pieces: a ROS-free Trémaux brain (`tremaux_solver.py`) that decides the next action over a junction graph with per-edge traversal counts (never blacklisting a real corridor on a nav timeout); a reactive locomotion pilot (`reactive_pilot.py`, extracted from the proven GCN drive) that executes short centerline goals, waypoint reroutes, and dead-end back-outs reliably; and the reused perception/topology/Nav2/SLAM stack. GCN and the legacy `maze_dfs` explorer are retained as fallbacks.

**Tech Stack:** ROS 2 Jazzy, Python (rclpy), Nav2 (NavigateToPose), slam_toolbox, Gazebo Harmonic, pytest. Spec: `docs/superpowers/specs/2026-06-14-maze-solving-exploration-design.md`.

---

## File Structure

All paths are under `ros2_ws_tugbot_nav_20260614/` (a fork of `20260522`).

**New files:**
- `src/tugbot_maze/tugbot_maze/tremaux_solver.py` — ROS-free brain (Action types + `TremauxSolver`). One responsibility: decide the next action from local topology + graph.
- `src/tugbot_maze/tugbot_maze/dead_end_classifier.py` — ROS-free: true wall vs transient nav failure.
- `src/tugbot_maze/tugbot_maze/reactive_pilot.py` — ROS locomotion layer (`drive_to`, `follow_path`, `back_out`), extracted from `maze_explorer.py`.
- `src/tugbot_maze/tugbot_maze/maze_solver.py` — thin ROS node + control loop.
- `src/tugbot_maze/test/test_tremaux_solver.py` — brain unit tests.
- `src/tugbot_maze/test/test_tremaux_pocket_escape.py` — SE-pocket regression.
- `src/tugbot_maze/test/test_dead_end_classifier.py` — classifier unit tests.
- `tools/run_solver_maze.sh`, `tools/run_solver_reliability.sh` — run/validation harnesses.

**Modified files:**
- `src/tugbot_maze/tugbot_maze/maze_topology.py` — add `visit_count` to `TopoEdge`.
- `src/tugbot_maze/setup.py` — add `maze_solver` console_script.
- `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py` — add `explorer_type=='tremaux'` Node; extend the `explorer_type` arg description.

**Reused unchanged:** `maze_perception.py`, `grid_utils.py`, `corridor_navigator.py`, `maze_goal_monitor.py`, the Nav2/SLAM/Gazebo configs and the rest of the launch.

---

## Milestone M0 — Workspace bootstrap

### Task 0: Fork the workspace, gitignore artifacts, verify build & fallbacks

**Files:**
- Create: `ros2_ws_tugbot_nav_20260614/` (copied `src/` only)
- Create: `ros2_ws_tugbot_nav_20260614/.gitignore`

- [ ] **Step 1: Copy source only (no build/install/log) from 20260522**

Run from repo root:
```bash
SRC=ros2_ws_tugbot_nav_20260522
DST=ros2_ws_tugbot_nav_20260614
mkdir -p "$DST"
rsync -a --exclude build/ --exclude install/ --exclude log/ --exclude '.pytest_cache/' \
      --exclude '__pycache__/' "$SRC/src" "$DST/"
ls "$DST/src"
```
Expected: `tugbot_bringup tugbot_description tugbot_exploration tugbot_gazebo tugbot_maze tugbot_navigation`

- [ ] **Step 2: Add a real .gitignore for the new workspace**

Create `ros2_ws_tugbot_nav_20260614/.gitignore`:
```gitignore
build/
install/
log/
.pytest_cache/
__pycache__/
*.pyc
```

- [ ] **Step 3: Build the new workspace**

Run:
```bash
cd ros2_ws_tugbot_nav_20260614
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```
Expected: `Summary: 6 packages finished` with no failures.

- [ ] **Step 4: Verify the existing explorers still launch (sanity, ~40s headless)**

Run (uses the proven GCN path; just confirming the fork is runnable):
```bash
cd ros2_ws_tugbot_nav_20260614
source install/setup.bash
timeout 40 ros2 launch tugbot_bringup tugbot_maze_explore.launch.py \
  headless:=true use_rviz:=false explorer_type:=maze_dfs guided_corridor_mode:=true \
  > /tmp/m0_smoke.log 2>&1 ; grep -c "maze_explorer\|Nav2\|lifecycle" /tmp/m0_smoke.log
```
Expected: non-zero count (nodes start). No need to reach the exit — this only proves the fork builds and launches.

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260614/src ros2_ws_tugbot_nav_20260614/.gitignore
git commit -m "feat(20260614): fork workspace from 20260522 (src only) + gitignore"
```

---

## Milestone M1 — The Trémaux brain (ROS-free, the correctness core)

### Task 1: Add `visit_count` to `TopoEdge`

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_topology.py` (the `TopoEdge` dataclass, ~line 124)
- Test: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_tremaux_solver.py`

- [ ] **Step 1: Write the failing test**

Create `test_tremaux_solver.py` with:
```python
from tugbot_maze.maze_topology import MazeTopology, IN_PROGRESS


def test_topoedge_has_visit_count_default_zero():
    topo = MazeTopology()
    a = topo.find_or_create_node(0.0, 0.0)
    b = topo.find_or_create_node(2.0, 0.0)
    edge = topo.connect_nodes(a.node_id, b.node_id, state=IN_PROGRESS)
    assert edge.visit_count == 0
    edge.visit_count = 2
    assert topo.edges[edge.edge_id].visit_count == 2
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python -m pytest test/test_tremaux_solver.py::test_topoedge_has_visit_count_default_zero -v`
Expected: FAIL — `TypeError`/`AttributeError` (no `visit_count` field).

- [ ] **Step 3: Add the field**

In `maze_topology.py`, change the `TopoEdge` dataclass:
```python
@dataclass
class TopoEdge:
    edge_id: int
    start_node_id: int
    end_node_id: int
    state: str = IN_PROGRESS
    visit_count: int = 0
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest test/test_tremaux_solver.py::test_topoedge_has_visit_count_default_zero -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_topology.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_tremaux_solver.py
git commit -m "feat(tremaux): add visit_count Trémaux mark to TopoEdge"
```

### Task 2: `TremauxSolver` — node/edge bookkeeping + Action types

**Files:**
- Create: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/tremaux_solver.py`
- Test: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_tremaux_solver.py`

The brain is duck-typed on `local`: any object with `.kind` (str) and `.open_directions` (list of objects with `.angle_rad`, `.target_xy`, `.distance_m`). Tests use a tiny stub so they stay fully ROS-free.

- [ ] **Step 1: Write the failing test (first explore at a junction)**

Append to `test_tremaux_solver.py`:
```python
import math
from collections import namedtuple
from tugbot_maze.tremaux_solver import TremauxSolver, EXPLORE, REROUTE, BACK_OUT, DONE

Dir = namedtuple('Dir', 'angle_rad target_xy distance_m')
Local = namedtuple('Local', 'kind open_directions')


def _local(kind, dirs):
    return Local(kind=kind, open_directions=[Dir(a, t, d) for (a, t, d) in dirs])


def test_first_update_at_junction_returns_explore():
    solver = TremauxSolver(exit_xy=(20.0, 18.0))
    # Junction at origin with 3 openings: east, north, south.
    local = _local('junction', [
        (0.0, (1.5, 0.0), 1.5),
        (math.pi / 2, (0.0, 1.5), 1.5),
        (-math.pi / 2, (0.0, -1.5), 1.5),
    ])
    action = solver.update((0.0, 0.0), 0.0, local)
    assert action.kind == EXPLORE
    assert action.target_xy is not None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest test/test_tremaux_solver.py::test_first_update_at_junction_returns_explore -v`
Expected: FAIL — `ModuleNotFoundError: tremaux_solver`.

- [ ] **Step 3: Write the brain (bookkeeping + Action + update skeleton with explore)**

Create `tremaux_solver.py`:
```python
"""ROS-free Trémaux maze-solving brain.

Decides the next exploration action from the current local topology and a
persistent junction graph with per-edge traversal counts (Trémaux marks). A
corridor is never permanently discarded because of a navigation failure — only a
physically-traversed-twice edge or a geometry-confirmed wall is excluded. This is
the correctness core; it has no ROS dependencies and is fully unit-tested.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import List, Optional, Tuple

from . import maze_perception as perception
from .maze_topology import (
    MazeTopology, TopoNode, BranchOption,
    UNTRIED, IN_PROGRESS, EXPLORED,
    DEAD_END as BRANCH_DEAD_END,
)

Point = Tuple[float, float]

EXPLORE = 'explore'
REROUTE = 'reroute'
BACK_OUT = 'back_out'
DONE = 'done'

# Pilot outcomes reported back to the brain.
OUT_SUCCESS = 'success'
OUT_WALL = 'wall'
OUT_WEDGED = 'wedged'


@dataclass
class Action:
    kind: str
    target_xy: Optional[Point] = None
    path_xy: List[Point] = field(default_factory=list)
    yaw: Optional[float] = None
    reason: str = ''


class TremauxSolver:
    def __init__(
        self,
        exit_xy: Point,
        *,
        junction_merge_radius_m: float = 0.75,
        exit_bias_weight: float = 0.3,
        exploration_bonus_weight: float = 0.6,
        distance_to_exit_weight: float = 0.15,
        max_soft_failures: int = 3,
    ) -> None:
        self.exit_xy = (float(exit_xy[0]), float(exit_xy[1]))
        self.topology = MazeTopology(
            junction_merge_radius_m=junction_merge_radius_m,
            exit_bias_weight=exit_bias_weight,
            exploration_bonus_weight=exploration_bonus_weight,
            distance_to_exit_weight=distance_to_exit_weight,
        )
        self.max_soft_failures = int(max_soft_failures)
        self.active_start_node_id: Optional[int] = None
        self.active_branch: Optional[BranchOption] = None

    # ---- helpers -------------------------------------------------------
    @staticmethod
    def _yaw(a: Point, b: Point) -> float:
        return math.atan2(b[1] - a[1], b[0] - a[0])

    def _edge_between(self, a_id: int, b_id: int):
        for edge in self.topology.edges.values():
            if {edge.start_node_id, edge.end_node_id} == {a_id, b_id}:
                return edge
        return None

    def _register_node(self, robot_xy: Point, local) -> TopoNode:
        if local.kind == perception.JUNCTION and len(local.open_directions) >= 3:
            center = perception.compute_junction_center(robot_xy, local.open_directions)
        else:
            center = (robot_xy[0], robot_xy[1])
        node_type = (
            'dead_end' if local.kind == perception.DEAD_END
            else 'junction' if local.kind == perception.JUNCTION
            else 'corridor'
        )
        return self.topology.find_or_create_node(center[0], center[1], node_type=node_type)

    def _set_branches(self, node: TopoNode, local) -> None:
        options = [
            BranchOption(angle_rad=d.angle_rad, target_xy=(d.target_xy[0], d.target_xy[1]))
            for d in local.open_directions
        ]
        self.topology.set_branch_options(node.node_id, options)

    # ---- main loop -----------------------------------------------------
    def update(self, robot_xy: Point, robot_yaw: float, local) -> Action:
        node = self._register_node(robot_xy, local)
        edge = None
        if self.active_start_node_id is not None and node.node_id != self.active_start_node_id:
            edge = self.topology.connect_nodes(self.active_start_node_id, node.node_id, state=EXPLORED)
            edge.visit_count = max(edge.visit_count, 1)
            if self.active_branch is not None:
                self.active_branch.state = EXPLORED
        self.topology.visit_node(node.node_id)
        self._set_branches(node, local)

        if local.kind == perception.DEAD_END:
            if edge is not None:
                edge.visit_count = 2
            if self.active_branch is not None:
                self.active_branch.state = BRANCH_DEAD_END
            if self.active_start_node_id is not None:
                prev_xy = self.topology.nodes[self.active_start_node_id].xy
            else:
                prev_xy = (robot_xy[0] - math.cos(robot_yaw), robot_xy[1] - math.sin(robot_yaw))
            self.active_branch = None
            self.active_start_node_id = node.node_id
            return Action(BACK_OUT, target_xy=prev_xy, yaw=self._yaw(robot_xy, prev_xy),
                          reason='dead end: reverse out')

        self.active_branch = None
        chosen = self.topology.choose_next_branch(node.node_id, exit_xy=self.exit_xy)
        if chosen is not None:
            self.active_start_node_id = node.node_id
            self.active_branch = chosen
            self.topology.mark_branch_state(node.node_id, chosen, IN_PROGRESS)
            return Action(EXPLORE, target_xy=chosen.target_xy, yaw=chosen.angle_rad,
                          reason='explore untried branch')

        result = self.topology.dijkstra_nearest_unexplored(node.node_id)
        if result is not None:
            path_ids, _target = result
            path = [self.topology.nodes[i].xy for i in path_ids]
            # Reroute spans already-known edges; clear the active start so the
            # next update() does not add a spurious shortcut edge on arrival.
            self.active_start_node_id = None
            self.active_branch = None
            return Action(REROUTE, path_xy=path, reason='reroute to nearest untried junction')

        return Action(DONE, reason='full coverage: no untried branch reachable')
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest test/test_tremaux_solver.py::test_first_update_at_junction_returns_explore -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/tremaux_solver.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_tremaux_solver.py
git commit -m "feat(tremaux): TremauxSolver brain — explore/reroute/back-out/done"
```

### Task 3: Brain — explored branches, dead-end back-out, reroute, termination

**Files:**
- Modify: `tremaux_solver.py` (add `report_outcome`)
- Test: `test_tremaux_solver.py`

- [ ] **Step 1: Write the failing tests (the four behaviors)**

Append to `test_tremaux_solver.py`:
```python
def test_explored_branch_not_rechosen_after_traversal():
    solver = TremauxSolver(exit_xy=(20.0, 18.0))
    j = _local('junction', [
        (0.0, (1.5, 0.0), 1.5),
        (math.pi / 2, (0.0, 1.5), 1.5),
        (-math.pi / 2, (0.0, -1.5), 1.5),
    ])
    first = solver.update((0.0, 0.0), 0.0, j)          # picks a branch -> IN_PROGRESS
    # Arrive at the far end (a corridor), then come back to the junction.
    solver.update(first.target_xy, first.yaw, _local('corridor', [
        (first.yaw, (first.target_xy[0] + 1.0, first.target_xy[1]), 1.0),
        (first.yaw + math.pi, (0.0, 0.0), 1.0),
    ]))
    second = solver.update((0.0, 0.0), 0.0, j)          # back at junction
    assert second.kind == EXPLORE
    assert second.target_xy != first.target_xy           # a *different* branch


def test_dead_end_returns_back_out_toward_previous_node():
    solver = TremauxSolver(exit_xy=(20.0, 18.0))
    j = _local('junction', [
        (0.0, (1.5, 0.0), 1.5),
        (math.pi / 2, (0.0, 1.5), 1.5),
        (-math.pi / 2, (0.0, -1.5), 1.5),
    ])
    a = solver.update((0.0, 0.0), 0.0, j)
    # The chosen branch leads to a dead end (1 opening = the way we came).
    back = solver.update(a.target_xy, a.yaw,
                         _local('dead_end', [(a.yaw + math.pi, (0.0, 0.0), 1.0)]))
    assert back.kind == BACK_OUT
    # Back-out target is the junction we came from (origin).
    assert math.hypot(back.target_xy[0], back.target_xy[1]) < 0.8


def test_done_when_single_corridor_dead_ends_both_ways():
    solver = TremauxSolver(exit_xy=(20.0, 18.0))
    # Start in a corridor, only one new direction; it dead-ends; nothing left.
    solver.update((0.0, 0.0), 0.0, _local('dead_end', [(0.0, (1.0, 0.0), 1.0)]))
    final = solver.update((0.0, 0.0), 0.0, _local('dead_end', [(0.0, (1.0, 0.0), 1.0)]))
    assert final.kind in (DONE, BACK_OUT)


def test_report_outcome_wall_marks_branch_dead_end():
    solver = TremauxSolver(exit_xy=(20.0, 18.0))
    j = _local('junction', [
        (0.0, (1.5, 0.0), 1.5),
        (math.pi / 2, (0.0, 1.5), 1.5),
        (-math.pi / 2, (0.0, -1.5), 1.5),
    ])
    a = solver.update((0.0, 0.0), 0.0, j)
    solver.report_outcome(OUT_WALL)
    assert solver.active_branch is None
    # Re-evaluating the same junction must not re-pick the walled branch.
    nxt = solver.update((0.0, 0.0), 0.0, j)
    assert nxt.kind == EXPLORE and nxt.target_xy != a.target_xy
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest test/test_tremaux_solver.py -v`
Expected: the new tests FAIL (`report_outcome` missing; back-out / re-choice not yet correct).

- [ ] **Step 3: Implement `report_outcome` and confirm logic**

Add to `TremauxSolver` (the `update` already handles back-out/reroute/done from Task 2; this adds outcome reporting for non-arrival results):
```python
    def report_outcome(self, outcome: str) -> None:
        """Called by the node after a pilot action completes without arriving
        at a new node (wall confirmed, or wedged/timeout). SUCCESS is handled
        implicitly by the next update() connecting the edge.
        """
        branch = self.active_branch
        if branch is None:
            return
        if outcome == OUT_WALL:
            branch.state = BRANCH_DEAD_END
            self.active_branch = None
        elif outcome == OUT_WEDGED:
            branch.failures += 1
            if branch.failures >= self.max_soft_failures:
                # Give up on this branch (treat as explored) WITHOUT blacklisting
                # the corridor — Dijkstra simply won't target it; the geometry is
                # never globally poisoned.
                branch.state = EXPLORED
            else:
                branch.state = UNTRIED  # retry on a later visit
            self.active_branch = None
        # OUT_SUCCESS: no-op (handled by next update()).
```
Note: `choose_next_branch` only selects `UNTRIED` branches and applies no blacklist (we never call `blacklist_goal`), so `EXPLORED`/`DEAD_END` branches are naturally skipped, and `dijkstra_nearest_unexplored` reroutes to any node that still has an `UNTRIED` branch.

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest test/test_tremaux_solver.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/tremaux_solver.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_tremaux_solver.py
git commit -m "feat(tremaux): outcome reporting + explored/dead-end/reroute/done behaviors"
```

### Task 4: The SE-pocket regression test (the core proof)

**Files:**
- Create: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_tremaux_pocket_escape.py`

This encodes the actual v8 failure: the robot enters a SE pocket that dead-ends, while a northward passage at an earlier junction was *never taken* (it would previously have been blacklisted after Nav2 timeouts). Assert the brain backs out and reroutes toward that northward passage instead of oscillating.

- [ ] **Step 1: Write the regression test**

Create `test_tremaux_pocket_escape.py`:
```python
import math
from collections import namedtuple

from tugbot_maze.tremaux_solver import (
    TremauxSolver, EXPLORE, REROUTE, BACK_OUT, OUT_WALL,
)

Dir = namedtuple('Dir', 'angle_rad target_xy distance_m')
Local = namedtuple('Local', 'kind open_directions')


def _local(kind, dirs):
    return Local(kind=kind, open_directions=[Dir(a, t, d) for (a, t, d) in dirs])


def test_escapes_se_pocket_to_northward_passage():
    """
    Topology (map frame, approx):
      J1 (10,9): junction with EAST (toward pocket) and NORTH (toward exit).
      Robot takes EAST into the pocket mouth J2 (15,9), which has EAST again.
      EAST from J2 dead-ends at D (19,9) — the SE pocket wall.
      The NORTH passage at J1 was never taken.
    Expected: at D -> BACK_OUT; then the brain reroutes/【explores】 toward the
    NORTH passage at J1, never re-selecting the walled EAST chain.
    """
    solver = TremauxSolver(exit_xy=(21.0, 18.0))

    # J1: at (10,9), openings EAST and NORTH (and the entry WEST).
    j1 = _local('junction', [
        (0.0, (11.5, 9.0), 1.5),            # east -> pocket
        (math.pi / 2, (10.0, 10.5), 1.5),   # north -> toward exit
        (math.pi, (8.5, 9.0), 1.5),         # west -> entry
    ])
    a1 = solver.update((10.0, 9.0), 0.0, j1)
    assert a1.kind == EXPLORE
    # Exit-bias makes EAST or NORTH plausible; force the historical bad path:
    # drive EAST toward the pocket regardless of which the score picked, by
    # reporting arrival at J2 along the east corridor.
    # J2: pocket mouth at (15,9): EAST (deeper) + WEST (back to J1).
    j2 = _local('junction', [
        (0.0, (16.5, 9.0), 1.5),            # east -> deeper into pocket
        (math.pi, (13.5, 9.0), 1.5),        # west -> back to J1
    ])
    solver.update((15.0, 9.0), 0.0, j2)
    a2 = solver.update((15.0, 9.0), 0.0, j2)
    assert a2.kind in (EXPLORE, REROUTE)

    # Drive deeper east; it dead-ends at D (19,9): one opening = the way back.
    dead = solver.update((19.0, 9.0), 0.0,
                         _local('dead_end', [(math.pi, (17.5, 9.0), 1.5)]))
    assert dead.kind == BACK_OUT

    # After backing out to the pocket-mouth region, the brain must steer the
    # robot back toward J1's still-untried NORTH passage — never DONE, never a
    # re-selection of the (now walled) east chain.
    nxt = solver.update((15.0, 9.0), 0.0, j2)
    assert nxt.kind in (EXPLORE, REROUTE)
    # Eventually, back at J1, NORTH is the only untried branch -> chosen.
    at_j1 = solver.update((10.0, 9.0), 0.0, j1)
    assert at_j1.kind == EXPLORE
    assert at_j1.target_xy[1] > 9.5   # heading NORTH (toward the exit), not EAST


def test_pocket_never_blacklists_the_corridor():
    """A Nav2 timeout in the pocket must NOT poison the corridor: after a WEDGED
    report the corridor stays reachable (UNTRIED) for a later retry."""
    solver = TremauxSolver(exit_xy=(21.0, 18.0))
    from tugbot_maze.tremaux_solver import OUT_WEDGED
    j = _local('junction', [
        (0.0, (1.5, 0.0), 1.5),
        (math.pi / 2, (0.0, 1.5), 1.5),
        (math.pi, (-1.5, 0.0), 1.5),
    ])
    solver.update((0.0, 0.0), 0.0, j)
    solver.report_outcome(OUT_WEDGED)   # one transient failure
    # No branch should be in a terminal "blocked/blacklisted" state.
    node = solver.topology.nodes[solver.active_start_node_id or 1]
    assert all(b.state != 'blacklisted' and b.state != 'blocked' for b in node.branches)
```

- [ ] **Step 2: Run the regression to verify it fails (or passes) honestly**

Run: `python -m pytest test/test_tremaux_pocket_escape.py -v`
Expected: PASS if the Task 2–3 logic is correct. If it FAILS, do **not** weaken the test — fix `tremaux_solver.py` until the brain genuinely escapes the pocket. This test is the contract for the whole effort.

- [ ] **Step 3: If needed, fix the brain (no test weakening)**

Likely fix points if it fails: ensure `dijkstra_nearest_unexplored` is reached when the current node has no untried branch, and that `choose_next_branch` skips `EXPLORED`/`DEAD_END`. (No code shown here because the brain from Tasks 2–3 should already satisfy it; only adjust if a real assertion fails.)

- [ ] **Step 4: Run the full brain suite**

Run: `python -m pytest test/test_tremaux_solver.py test/test_tremaux_pocket_escape.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_tremaux_pocket_escape.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/tremaux_solver.py
git commit -m "test(tremaux): SE-pocket escape regression — the core correctness proof"
```

---

## Milestone M2 — Reactive pilot + dead-end classifier

### Task 5: Dead-end classifier (ROS-free)

**Files:**
- Create: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/dead_end_classifier.py`
- Test: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_dead_end_classifier.py`

A node is a *true* dead end only if perception says `DEAD_END` **and** the forward scan shows a wall within ~corridor-length. A Nav2 timeout alone never qualifies.

- [ ] **Step 1: Write the failing tests**

Create `test_dead_end_classifier.py`:
```python
import math
from tugbot_maze.dead_end_classifier import is_true_dead_end


def test_wall_ahead_with_single_opening_is_true_dead_end():
    # perception kind dead_end + min forward range below corridor length
    assert is_true_dead_end(perception_kind='dead_end',
                            forward_min_range_m=0.6,
                            corridor_length_m=1.76) is True


def test_open_ahead_is_not_dead_end_even_if_kind_says_so():
    # Scan shows lots of room ahead -> a Nav2 timeout false alarm, not a wall.
    assert is_true_dead_end(perception_kind='dead_end',
                            forward_min_range_m=2.5,
                            corridor_length_m=1.76) is False


def test_junction_kind_is_never_dead_end():
    assert is_true_dead_end(perception_kind='junction',
                            forward_min_range_m=0.3,
                            corridor_length_m=1.76) is False


def test_missing_scan_defers_to_not_dead_end():
    assert is_true_dead_end(perception_kind='dead_end',
                            forward_min_range_m=None,
                            corridor_length_m=1.76) is False
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest test/test_dead_end_classifier.py -v`
Expected: FAIL — `ModuleNotFoundError`.

- [ ] **Step 3: Implement the classifier**

Create `dead_end_classifier.py`:
```python
"""Distinguish a geometry-confirmed dead end from a transient navigation failure.

Pure Python so it is unit-testable without ROS. A Nav2 goal timeout is NEVER a
dead-end signal on its own; only perception saying DEAD_END together with a scan
that confirms a wall within ~corridor length counts.
"""
from __future__ import annotations

from typing import Optional

DEAD_END = 'dead_end'


def is_true_dead_end(
    perception_kind: str,
    forward_min_range_m: Optional[float],
    corridor_length_m: float = 1.76,
    wall_margin_m: float = 0.4,
) -> bool:
    if perception_kind != DEAD_END:
        return False
    if forward_min_range_m is None:
        return False  # no scan corroboration -> do not condemn the corridor
    return forward_min_range_m <= (corridor_length_m - wall_margin_m)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest test/test_dead_end_classifier.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/dead_end_classifier.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_dead_end_classifier.py
git commit -m "feat(pilot): geometry-confirmed dead-end classifier (kills false dead-ends)"
```

### Task 6: `ReactivePilot` — extract the proven reactive drive + own Nav2 + back_out

**Files:**
- Create: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/reactive_pilot.py`
- Source to extract from: `…/20260522/src/tugbot_maze/tugbot_maze/maze_explorer.py`
  - `_execute_reactive_drive_step` (4429–4580): the `rotating`/`driving`/`backup` state machine.
  - `_start_reactive_drive` (4603–4662): arming + laser pre-check.
  - `_finish_reactive_drive` (4582–4601): cleanup (drop the GCN/eastward/frontier coupling — pilot is mode-free).
  - laser helpers `_is_laser_clear_ahead`, `_is_laser_clear_in_direction` (grep for their defs).
  - Nav2 dispatch pattern `_send_goal` (3683+) and `action_client` usage (488).

Per the spec, the pilot OWNS locomotion end-to-end: `drive_to`/`follow_path` try a short Nav2 `NavigateToPose` goal and fall back to the reactive state machine; `back_out` reverses out of a dead end. The node decides *where*; the pilot decides *how*. It holds a ref to the owning node (clock, `cmd_vel_pub`, `scan_msg`, `_lookup_pose`, `map_frame`, logger) and a Nav2 action client. **Do not** copy the GCN-specific branches from `_finish_reactive_drive`.

- [ ] **Step 1: Scaffold the class + unified state**

Create `reactive_pilot.py`:
```python
"""Reliable locomotion for the Trémaux solver.

Owns end-to-end motion: drive_to/follow_path try a short Nav2 NavigateToPose
goal and fall back to a 10 Hz reactive /cmd_vel_nav state machine (rotate ->
drive) with watchdog + unwedge; back_out reverses out of a dead end. Extracted
from the proven GCN reactive drive in maze_explorer.py. Mode-free.
"""
from __future__ import annotations

import math
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from tugbot_maze.maze_perception import normalize_angle

SUCCESS = 'success'
WALL_AHEAD = 'wall_ahead'
WEDGED = 'wedged'


class ReactivePilot:
    def __init__(self, node, action_client, *, nav_timeout_sec=18.0,
                 max_seconds=12.0, no_progress_sec=3.0, no_rot_sec=3.0,
                 forward_speed=0.2, turn_speed=0.5, reverse_speed=0.15):
        self.node = node
        self.action_client = action_client
        self.nav_timeout_sec = nav_timeout_sec
        self.max_seconds = max_seconds
        self.no_progress_sec = no_progress_sec
        self.no_rot_sec = no_rot_sec
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.reverse_speed = reverse_speed
        self._reset()

    def _reset(self):
        self.state = 'idle'        # idle | nav | rotating | driving | backup
        self.queue = []            # remaining waypoints for follow_path
        self.result = None         # SUCCESS | WALL_AHEAD | WEDGED on finish
        self.target_xy = None
        self.target_yaw = 0.0
        self.target_distance = 0.0
        self.start_xy = None
        self.start_time = None
        self.progress_ref_xy = None
        self.progress_ref_time = None
        self.backup_ref_xy = None
        self.rot_ref_yaw = None
        self.rot_ref_time = None
        # nav sub-state
        self.nav_done = True
        self.nav_succeeded = False
        self.nav_deadline = None
        self._nav_goal_handle = None

    def is_active(self):
        return self.state != 'idle'
```

- [ ] **Step 2: Public API — drive_to / follow_path / back_out / reactive_drive**

```python
    def drive_to(self, target_xy):
        self.queue = [(float(target_xy[0]), float(target_xy[1]))]
        self.result = None
        self._begin_next()

    def follow_path(self, waypoints):
        self.queue = [(float(x), float(y)) for (x, y) in waypoints]
        self.result = None
        self._begin_next()

    def back_out(self, distance, robot_pose):
        self.queue = []
        self.result = None
        self._begin_reactive(robot_pose[2], distance, reverse=True)

    def reactive_drive(self, angle, distance):
        """Forced-reactive forward push (used for maze entry, no Nav2)."""
        self.queue = []
        self.result = None
        self._begin_reactive(angle, distance)

    def _begin_next(self):
        if not self.queue:
            self._finish(SUCCESS)
            return
        self.target_xy = self.queue[0]
        self._dispatch_nav(self.target_xy)

    def _dispatch_nav(self, target_xy):
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self._fallback_reactive(); return
        pose = self.node._lookup_pose()
        yaw = math.atan2(target_xy[1] - pose[1], target_xy[0] - pose[0]) if pose else 0.0
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.node.map_frame
        goal.pose.pose.position.x = float(target_xy[0])
        goal.pose.pose.position.y = float(target_xy[1])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.nav_done = False
        self.nav_succeeded = False
        self.nav_deadline = self.node.get_clock().now().nanoseconds / 1e9 + self.nav_timeout_sec
        self.state = 'nav'
        fut = self.action_client.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.nav_done = True; self.nav_succeeded = False; return
        self._nav_goal_handle = gh
        gh.get_result_async().add_done_callback(
            lambda f: (setattr(self, 'nav_succeeded', f.result().status == 4),
                       setattr(self, 'nav_done', True)))

    def _fallback_reactive(self):
        pose = self.node._lookup_pose()
        if pose is None or self.target_xy is None:
            self._finish(WEDGED); return
        ang = math.atan2(self.target_xy[1] - pose[1], self.target_xy[0] - pose[0])
        dist = max(0.3, math.hypot(self.target_xy[0] - pose[0], self.target_xy[1] - pose[1]))
        self._begin_reactive(ang, dist)
```

- [ ] **Step 3: Port the reactive state machine + the nav-polling tick**

Port `maze_explorer.py:4429–4662` into `_begin_reactive(angle, distance, reverse=False)` and the reactive branches of `tick`, with these mechanical adaptations: `self.cmd_vel_pub`→`self.node.cmd_vel_pub`; `self.get_clock()`→`self.node.get_clock()`; `self.get_logger()`→`self.node.get_logger()`; `self.scan_msg`→`self.node.scan_msg`; the `reactive_drive_*` attrs → the `self.*` fields from Step 1; generalize the `backup` branch to back off `self.target_distance` (not a fixed 0.4 m) with rear-cone safety via `_is_laser_clear_in_direction(math.pi, …)`. On each terminal reactive branch call `self._finish(SUCCESS | WALL_AHEAD | WEDGED)`; on a successful `driving` completion, advance the queue via `_on_hop_reached`.

```python
    def _begin_reactive(self, angle, distance, reverse=False):
        self.target_yaw = angle
        self.target_distance = distance
        self.start_xy = None
        self.start_time = self.node.get_clock().now()
        self.progress_ref_xy = None; self.progress_ref_time = None
        self.backup_ref_xy = None
        self.rot_ref_yaw = None; self.rot_ref_time = None
        self.state = 'backup' if reverse else 'rotating'

    def tick(self, robot_pose):
        if self.state == 'nav':
            now = self.node.get_clock().now().nanoseconds / 1e9
            if self.nav_done:
                if self.nav_succeeded:
                    self._on_hop_reached(robot_pose)
                else:
                    self._fallback_reactive()
            elif now > self.nav_deadline:
                self._cancel_nav(); self._fallback_reactive()
            return
        # rotating | driving | backup : PORT of _execute_reactive_drive_step,
        # except: where the original 'driving' branch finishes on reaching
        # target_distance, call self._on_hop_reached(robot_pose) instead of a
        # bare finish; 'backup' completion and WALL/WEDGED call self._finish(...).
        ...

    def _on_hop_reached(self, robot_pose):
        self.node.cmd_vel_pub.publish(Twist())
        if self.queue:
            self.queue.pop(0)
        if self.queue:
            self._begin_next()
        else:
            self._finish(SUCCESS)

    def _cancel_nav(self):
        try:
            if self._nav_goal_handle is not None:
                self._nav_goal_handle.cancel_goal_async()
        except Exception:
            pass
        self.nav_done = True; self.nav_succeeded = False

    def _finish(self, result):
        self.node.cmd_vel_pub.publish(Twist())
        self._reset()
        self.result = result
```
(Engineer: copy the cited reactive body into `tick`; this block names the exact adaptations so the port is mechanical.)

- [ ] **Step 4: Build to verify it imports cleanly**

Run:
```bash
cd ros2_ws_tugbot_nav_20260614 && source /opt/ros/jazzy/setup.bash && colcon build --packages-select tugbot_maze --symlink-install
source install/setup.bash && python -c "from tugbot_maze.reactive_pilot import ReactivePilot, SUCCESS, WALL_AHEAD, WEDGED; print('ok')"
```
Expected: `ok`

- [ ] **Step 5: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/reactive_pilot.py
git commit -m "feat(pilot): ReactivePilot owns Nav2 short-goal + reactive fallback + back_out"
```

---

## Milestone M3 — The `maze_solver` node + wiring

### Task 7: Thin `maze_solver.py` node + control loop

**Files:**
- Create: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_solver.py`
- Reference for ROS plumbing: `maze_explorer.py` `__init__` (lines 90–630): action client (488), subscriptions (496–499), `cmd_vel_pub` (505), `_map_callback`/`_scan_callback` (677/692), `_lookup_robot_pose`, `_send_goal` (3683).

The node owns ROS I/O and a small state machine that drives the brain↔pilot loop. It is intentionally thin; all decisions are the brain's, all motion is the pilot's.

- [ ] **Step 1: Node skeleton — params, I/O, map/scan/pose**

Create `maze_solver.py` (port the cited plumbing; new control logic shown in full in Step 2):
```python
"""Thin ROS 2 node: Trémaux brain + reactive pilot autonomous maze solver."""
from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
import tf2_ros

from tugbot_maze.grid_utils import OccupancyGridView
from tugbot_maze import maze_perception as perception
from tugbot_maze.tremaux_solver import (
    TremauxSolver, EXPLORE, REROUTE, BACK_OUT, DONE,
    OUT_SUCCESS, OUT_WALL, OUT_WEDGED,
)
from tugbot_maze.reactive_pilot import ReactivePilot, SUCCESS, WALL_AHEAD, WEDGED
from tugbot_maze.dead_end_classifier import is_true_dead_end


class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver')
        # --- params (lean subset; mirror names used by the launch) ---
        self.map_topic = self.declare_parameter('map_topic', '/map').value
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.action_name = self.declare_parameter('action_name', '/navigate_to_pose').value
        self.goal_events_topic = self.declare_parameter('goal_events_topic', '/maze/goal_events').value
        self.exit_x = float(self.declare_parameter('exit_x', 21.07).value)
        self.exit_y = float(self.declare_parameter('exit_y', 18.08).value)
        self.exit_radius = float(self.declare_parameter('exit_radius', 1.2).value)
        self.entrance_yaw = float(self.declare_parameter('entrance_yaw', 0.0).value)
        self.entry_direct_distance_m = float(self.declare_parameter('entry_direct_distance_m', 2.0).value)
        self.clearance_radius_m = float(self.declare_parameter('clearance_radius_m', 0.35).value)
        self.lookahead_m = float(self.declare_parameter('open_direction_lookahead_m', 1.5).value)
        self.min_open_distance_m = float(self.declare_parameter('min_open_distance_m', 0.45).value)
        self.goal_step_m = float(self.declare_parameter('branch_goal_step_m', 1.5).value)
        self.goal_timeout_sec = float(self.declare_parameter('goal_timeout_sec', 18.0).value)
        self.max_goals = int(self.declare_parameter('max_goals', 400).value)
        self.startup_delay_sec = float(self.declare_parameter('startup_delay_sec', 3.0).value)

        # --- I/O ---
        self.action_client = ActionClient(self, NavigateToPose, self.action_name)
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.goal_events_pub = self.create_publisher(String, self.goal_events_topic, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- state ---
        self.map_view: Optional[OccupancyGridView] = None
        self.scan_msg: Optional[LaserScan] = None
        self.brain = TremauxSolver(exit_xy=(self.exit_x, self.exit_y))
        self.pilot = ReactivePilot(self, self.action_client, nav_timeout_sec=self.goal_timeout_sec)
        self.goal_count = 0
        self.phase = 'startup'          # startup | entering | deciding | busy | done
        self.pending_action = None
        self.entered = False

        self.create_timer(0.1, self._reactive_tick)   # 10 Hz pilot
        self.create_timer(0.5, self._control_tick)     # 2 Hz brain
        self.start_time = self.get_clock().now()
        self.get_logger().info('maze_solver started (Trémaux autonomous).')

    # --- callbacks / helpers (port from maze_explorer) ---
    def _map_cb(self, msg):
        self.map_view = OccupancyGridView.from_msg(msg)   # see grid_utils API

    def _scan_cb(self, msg):
        self.scan_msg = msg

    def _lookup_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def _forward_min_range(self):
        """Min laser range within a ±20° forward cone (for dead-end confirm)."""
        if self.scan_msg is None:
            return None
        s = self.scan_msg
        best = None
        n = len(s.ranges)
        for i, r in enumerate(s.ranges):
            ang = s.angle_min + i * s.angle_increment
            if abs(math.atan2(math.sin(ang), math.cos(ang))) <= math.radians(20):
                if r and r == r and r > 0.05 and (best is None or r < best):
                    best = r
        return best
```
(`OccupancyGridView.from_msg` — confirm the exact constructor in `grid_utils.py`; `maze_explorer._map_callback:677` shows the existing call.)

- [ ] **Step 2: Control loop — brain↔pilot state machine**

Add the control logic:
```python
    def _publish_event(self, text):
        self.goal_events_pub.publish(String(data=text))

    def _reactive_tick(self):
        if not self.pilot.is_active():
            return
        pose = self._lookup_pose()
        if pose is not None:
            self.pilot.tick(pose)

    def _control_tick(self):
        pose = self._lookup_pose()
        if pose is None or self.map_view is None or self.scan_msg is None:
            return
        # exit self-check
        if math.hypot(pose[0] - self.exit_x, pose[1] - self.exit_y) <= self.exit_radius:
            if self.phase != 'done':
                self.phase = 'done'
                self.get_logger().info('EXIT_REACHED (maze_solver)')
                self._publish_event('EXIT_REACHED')
            return
        if self.phase == 'done':
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if self.phase == 'startup':
            if elapsed >= self.startup_delay_sec:
                self.phase = 'entering'
            return
        if self.goal_count >= self.max_goals:
            self.get_logger().warn('FAILED_EXHAUSTED: max_goals reached')
            self._publish_event('FAILED_EXHAUSTED'); self.phase = 'done'; return

        # The pilot owns all motion; wait while it is active.
        if self.pilot.is_active():
            return

        # Entry: a forced-reactive forward push so SLAM starts mapping.
        if self.phase == 'entering':
            if self.pilot.result is None:
                self.pilot.reactive_drive(self.entrance_yaw, self.entry_direct_distance_m)
                return
            self.pilot.result = None
            self.entered = True
            self.phase = 'deciding'
            return

        # A pilot action just finished -> feed the outcome back to the brain.
        if self.pilot.result is not None:
            self._handle_result()
            return

        if self.phase == 'deciding':
            self._decide(pose)

    def _decide(self, pose):
        local = perception.classify_local_topology(
            self.map_view, pose, lookahead_m=self.lookahead_m,
            clearance_radius_m=self.clearance_radius_m,
            min_open_distance_m=self.min_open_distance_m,
        )
        # Confirm dead ends with the scan before the brain commits to back-out.
        if local.kind == perception.DEAD_END and not is_true_dead_end(
                local.kind, self._forward_min_range()):
            return  # false alarm: re-sample next tick instead of condemning it
        action = self.brain.update((pose[0], pose[1]), pose[2], local)
        self.goal_count += 1
        self.pending_action = action
        self._publish_event('explore goal #%d kind=%s' % (self.goal_count, action.kind))
        if action.kind == DONE:
            self.get_logger().warn('FAILED_EXHAUSTED: brain reports full coverage w/o exit')
            self._publish_event('FAILED_EXHAUSTED'); self.phase = 'done'; return
        self.phase = 'busy'
        if action.kind == BACK_OUT:
            self.pilot.back_out(self.goal_step_m, pose)
        elif action.kind == REROUTE:
            self.pilot.follow_path(action.path_xy[1:])   # skip the current node
        else:  # EXPLORE
            self.pilot.drive_to(action.target_xy)

    def _handle_result(self):
        res = self.pilot.result
        self.pilot.result = None
        act = self.pending_action
        self.pending_action = None
        # Only an EXPLORE that ended without arriving reports an outcome; a
        # BACK_OUT / REROUTE is re-evaluated by the next brain update().
        if act is not None and act.kind == EXPLORE:
            if res == WALL_AHEAD:
                self.brain.report_outcome(OUT_WALL)
            elif res == WEDGED:
                self.brain.report_outcome(OUT_WEDGED)
        self.phase = 'deciding'


def main(args=None):
    rclpy.init(args=args)
    node = MazeSolver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```
(Engineer: verify `OccupancyGridView.from_msg`, the `_lookup_pose` quaternion order, and Nav2 result `status == 4` against `grid_utils.py` / `maze_explorer.py:3683`+ ; adjust call names if the existing code differs.)

- [ ] **Step 3: Build**

Run:
```bash
cd ros2_ws_tugbot_nav_20260614 && source /opt/ros/jazzy/setup.bash
colcon build --packages-select tugbot_maze --symlink-install
source install/setup.bash && python -c "import tugbot_maze.maze_solver; print('ok')"
```
Expected: `ok`

- [ ] **Step 4: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_solver.py
git commit -m "feat(solver): thin maze_solver node wiring brain + pilot + perception"
```

### Task 8: Register the node + launch wiring

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/setup.py` (console_scripts, ~line 30)
- Modify: `ros2_ws_tugbot_nav_20260614/src/tugbot_bringup/launch/tugbot_maze_explore.launch.py` (after the `maze_dfs_explorer` Node, ~line 150; and the `explorer_type` arg, ~line 213; and the `TimerAction`, ~line 274)

- [ ] **Step 1: Add the console_script**

In `setup.py`, add to `console_scripts`:
```python
            'maze_solver = tugbot_maze.maze_solver:main',
```

- [ ] **Step 2: Add the launch Node (mirror maze_dfs, lean params)**

In `tugbot_maze_explore.launch.py`, after the `maze_dfs_explorer` Node definition add:
```python
    maze_solver_node = Node(
        package='tugbot_maze',
        executable='maze_solver',
        name='maze_solver',
        output='screen',
        condition=IfCondition(PythonExpression(["'", explorer_type, "' == 'tremaux'"])),
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'map_topic': '/map',
            'scan_topic': '/scan',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'action_name': '/navigate_to_pose',
            'goal_events_topic': LaunchConfiguration('goal_events_topic'),
            'exit_x': ParameterValue(LaunchConfiguration('exit_x'), value_type=float),
            'exit_y': ParameterValue(LaunchConfiguration('exit_y'), value_type=float),
            'exit_radius': ParameterValue(LaunchConfiguration('exit_radius'), value_type=float),
            'entrance_yaw': ParameterValue(LaunchConfiguration('entrance_yaw'), value_type=float),
            'entry_direct_distance_m': ParameterValue(LaunchConfiguration('entry_direct_distance_m'), value_type=float),
            'clearance_radius_m': ParameterValue(LaunchConfiguration('clearance_radius_m'), value_type=float),
            'open_direction_lookahead_m': ParameterValue(LaunchConfiguration('open_direction_lookahead_m'), value_type=float),
            'min_open_distance_m': ParameterValue(LaunchConfiguration('min_open_distance_m'), value_type=float),
            'branch_goal_step_m': ParameterValue(LaunchConfiguration('branch_goal_step_m'), value_type=float),
            'goal_timeout_sec': ParameterValue(LaunchConfiguration('goal_timeout_sec'), value_type=float),
            'max_goals': ParameterValue(LaunchConfiguration('max_goals'), value_type=int),
        }],
    )
```

- [ ] **Step 3: Update the explorer_type arg description and the TimerAction**

Change the `explorer_type` `DeclareLaunchArgument` description to mention `tremaux`, and add `maze_solver_node` to the delayed start `TimerAction` (line ~274):
```python
        TimerAction(period=13.0, actions=[maze_dfs_explorer, frontier_explorer, maze_solver_node]),
```

- [ ] **Step 4: Build & confirm the node is selectable**

Run:
```bash
cd ros2_ws_tugbot_nav_20260614 && source /opt/ros/jazzy/setup.bash
colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install
source install/setup.bash
ros2 pkg executables tugbot_maze | grep maze_solver
```
Expected: `tugbot_maze maze_solver`

- [ ] **Step 5: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/setup.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_bringup/launch/tugbot_maze_explore.launch.py
git commit -m "feat(solver): register maze_solver node + explorer_type:=tremaux launch wiring"
```

### Task 9: Bounded sim smoke test

**Files:**
- Create: `ros2_ws_tugbot_nav_20260614/tools/run_solver_maze.sh`

- [ ] **Step 1: Create the run harness (fork of run_dfs_maze.sh)**

Create `tools/run_solver_maze.sh` by copying `…/20260522/tools/run_dfs_maze.sh` and changing only:
- `explorer_type:=maze_dfs` → `explorer_type:=tremaux`
- `guided_corridor_mode:=false` (unchanged)
- artifact prefix `dfs_run_` → `solver_run_`
- keep the SHM purge + unique `ROS_DOMAIN_ID` + `kill_all_sim` hygiene verbatim.
Make it executable: `chmod +x tools/run_solver_maze.sh`.

- [ ] **Step 2: Run a short bounded smoke (headless, ~180s)**

Run:
```bash
cd ros2_ws_tugbot_nav_20260614 && ./tools/run_solver_maze.sh 180 true false 30
```
Expected: the robot enters the maze and executes multiple `explore goal #N` events; check the artifact:
```bash
tail -40 log/solver_run_*/launch.log | grep -E "explore goal|REACTIVE|BACK_OUT|EXIT_REACHED|FAILED"
```
PASS criteria for the smoke: ≥5 `explore goal #` events, at least one successful reactive drive or Nav2 success, and **zero** "blacklist" log lines. (Reaching the exit in 180 s is not required yet.)

- [ ] **Step 3: If the robot stalls at entry, debug before proceeding**

Use the closest-approach check:
```bash
grep -aoE "pose=\([-0-9.]+, [-0-9.]+\)" log/solver_run_*/launch.log | tail -5
```
Confirm the robot moved off the start. Common fixes: entry distance, `min_clearance` in the entry `start()` call, or the 13 s warmup. Adjust params (not test weakening) and re-run.

- [ ] **Step 4: Commit the harness**

```bash
git add ros2_ws_tugbot_nav_20260614/tools/run_solver_maze.sh
git commit -m "test(solver): bounded sim smoke harness run_solver_maze.sh"
```

---

## Milestone M4 — Full-maze validation & tuning

### Task 10: Reliability harness + drive to ≥4/5

**Files:**
- Create: `ros2_ws_tugbot_nav_20260614/tools/run_solver_reliability.sh`

- [ ] **Step 1: Create the reliability harness (fork of run_gcn_reliability.sh)**

Create `tools/run_solver_reliability.sh` that runs `run_solver_maze.sh` N times (default 5), each headless with a full budget (e.g. 900 s, max_goals 400), and tallies `EXIT_REACHED` vs `FAILED_EXHAUSTED`/`TIMEOUT`:
```bash
#!/usr/bin/env bash
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"; cd "$WS"
N="${1:-5}"; SECS="${2:-900}"; PASS=0
for i in $(seq 1 "$N"); do
  echo "=== solver run $i/$N ==="
  ./tools/run_solver_maze.sh "$SECS" true false 400
  R=$(cat log/solver_run_*/result.txt 2>/dev/null | tail -1)
  [ "$R" = "EXIT_REACHED" ] && PASS=$((PASS+1))
  echo "run $i: $R (cumulative pass=$PASS/$i)"
done
echo "RELIABILITY: $PASS/$N reached EXIT"
```
Make executable.

- [ ] **Step 2: Run the reliability suite**

Run: `cd ros2_ws_tugbot_nav_20260614 && ./tools/run_solver_reliability.sh 5 900`
Expected target: `RELIABILITY: >=4/5 reached EXIT`.

- [ ] **Step 3: Iterate on real failures (tuning loop, no test weakening)**

For each non-EXIT run, inspect `log/solver_run_*/launch.log`:
- If trapped/oscillating: capture the topology decision sequence; if it reveals a brain logic gap, add a failing unit test to `test_tremaux_solver.py` reproducing it, fix the brain, re-run M1 tests, then re-run sim.
- If a pilot wedge/timeout: tune `ReactivePilot` thresholds or `goal_timeout_sec` / `goal_step_m`.
- If a false dead-end: tune `is_true_dead_end` margins (add a classifier unit test first).
Repeat until ≥4/5.

- [ ] **Step 4: Commit the harness + any tuning**

```bash
git add ros2_ws_tugbot_nav_20260614/tools/run_solver_reliability.sh \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze
git commit -m "test(solver): reliability harness; tune Trémaux solver to >=4/5"
```

---

## Milestone M5 — Document & memory

### Task 11: Document the result and update memory

**Files:**
- Create: `ros2_ws_tugbot_nav_20260614/README_TREMAUX.md`
- Update memory: `…/memory/maze-autonomy-status.md`, `…/memory/MEMORY.md`

- [ ] **Step 1: Write a short README**

Create `ros2_ws_tugbot_nav_20260614/README_TREMAUX.md` documenting: how to run (`tools/run_solver_maze.sh`, `explorer_type:=tremaux`), the architecture (brain/pilot/node), the reliability result (N/5), and how it fixes the false-dead-end + back-out problems.

- [ ] **Step 2: Update memory with the outcome**

Update `maze-autonomy-status.md` to record whether autonomous 通关 is now achieved (with the run tally), referencing the new workspace and the Trémaux design. Add a one-line pointer in `MEMORY.md` if a new memory file is created.

- [ ] **Step 3: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/README_TREMAUX.md
git commit -m "docs(solver): document Trémaux autonomous solver + reliability result"
```

- [ ] **Step 4: Final verification**

Run the full unit suite once more:
```bash
cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python -m pytest test/test_tremaux_solver.py test/test_tremaux_pocket_escape.py test/test_dead_end_classifier.py -v
```
Expected: all PASS. Confirm the reliability tally meets ≥4/5.

---

## Notes for the implementer

- **TDD discipline:** the brain (M1) and classifier (M2) are pure Python — never weaken a failing test to make it pass; fix the logic. The SE-pocket regression (Task 4) is the contract for the entire effort.
- **Extraction fidelity (Task 6):** the reactive drive is proven; port it mechanically from the cited line ranges, changing only state ownership and dropping GCN/mode coupling. Preserve the 10 Hz cadence, centerline aiming, watchdog (12 s) and unwedge.
- **Verify reused APIs before relying on them:** `OccupancyGridView.from_msg`, `_is_laser_clear_*`, Nav2 result status — confirm exact names in `grid_utils.py` / `maze_explorer.py` and adjust.
- **Sim cost:** front-load correctness into fast unit tests; only spend full 15-minute sim runs once M1–M3 gates pass.
- **Fallbacks intact:** GCN (`guided_corridor_mode:=true`) and `maze_dfs` remain runnable in 20260614 throughout.
