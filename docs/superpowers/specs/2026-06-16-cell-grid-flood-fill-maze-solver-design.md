# Cell-Grid Flood-Fill (Micromouse) Maze Solver — Design

**Date:** 2026-06-16
**Workspace:** `ros2_ws_tugbot_nav_20260614`
**Status:** Approved (design). New autonomous exploration strategy; supersedes the
reactive wall-follower as the intended autonomous solver.

## Motivation

The reactive wall-follower is abandoned as the autonomous solver. Its earlier
"5/5 通关" was a cheat (it followed the **outside** of the perimeter wall between the
two boundary openings). Sealing the entrance forced an interior solve, which exposed
the real flaw: the follower is **stateless** — when it loses the followed wall (real
LIDAR noise + the 0.5 rad/s angular cap + corridor geometry make this inevitable at
some junctions) it re-references on an arbitrary wall and **loops in a sub-region**.
This was confirmed in Gazebo even after re-tuning inside the real motion envelope
(re-tuning removed the wedging — 0 stall-backups — but not the looping). The earlier
DFS/Trémaux/frontier explorers failed from the other side: Nav2 point-goals **wedge**
in the 1.76 m corridors, and node aliasing kept reroutes local.

The fix needs two properties the wall-follower lacks:
1. **Memory** of where it has been (kill statelessness/looping).
2. **Reliable corridor locomotion** (kill wedging) — centerline driving, the one
   thing that has always worked here (GCN).

The maze is a known-size **perfect maze** (a tree) on a regular grid, with a **known
exit cell**. That makes the classic **micromouse flood-fill** the natural fit: it has
explicit per-cell memory and is provably goal-reaching in a grid maze with a known
goal while discovering walls online.

## The cell grid (concrete)

A fixed 2 m grid in the map frame. Cell `(cx, cy)` is centered at map
`(2·cx, 2·cy)`. Columns `cx ∈ 0..10`, rows `cy ∈ 0..9` (covers the outer-wall box
x ∈ [0.95, 21.07], y ∈ [−1.04, 19.09]; confirmed by the GCN route coordinates, which
land on even map coordinates at 2 m spacing).

- **Entrance interior cell = `(1, 0)`** — center (2, 0) m, where the ENTRY_DIRECT
  drive lands after entering through the opening at map (0, 0).
- **Exit cell = `(10, 9)`** — center (20, 18) m; its east edge is the exit opening at
  map (21, 18). Mission success = robot within `exit_radius` (1.2 m) of
  (21.072562, 18.083566), i.e. arriving at cell (10, 9).
- Directions: N = +y (cy+1), S = −y, E = +x (cx+1), W = −x.

## Real motion envelope (for locomotion limits + the inertia sim)

Measured from the stack (velocity_smoother + diff-drive plugin):
`v ≤ 0.5 m/s`, `|w| ≤ 0.5 rad/s` (the smoother angular cap — the old follower's
`w_max=1.0` was fiction), linear accel/decel `≤ 0.5 m/s²` (diff-drive, the binding
limit), angular accel `≤ 0.8 rad/s²`. The locomotion controller commands within this
envelope; the offline sim rate-limits to it (see Offline validation).

## Components

All decision logic is ROS-free and unit-tested, mirroring the existing module style
(`tremaux_solver.py`, `grid_utils.py`).

| Unit | Responsibility | ROS-free? |
|---|---|---|
| `flood_fill_brain.py` | Memory + policy. Per-cell edge knowledge `walls[(cx,cy)][dir] ∈ {OPEN, WALL, UNKNOWN}`; `flood()` = BFS distance-to-exit over non-WALL edges (UNKNOWN treated as passable); `next_cell(cur)` = adjacent cell with lowest distance whose edge is not WALL; `mark(cell, dir, is_wall)` records a sensed edge and flags a re-flood; `is_done(cell)`. Deterministic. | yes |
| `cell_walls.py` | `sense_walls(grid_view, cell) -> dict[dir, bool]`: from a `grid_utils.OccupancyGridView`, decide WALL vs OPEN on each of N/E/S/W of the current cell by sampling occupancy along/just past the shared edge (`is_occupied` / `line_min_clearance`). Returns only edges confidently sensed (others stay UNKNOWN). | yes (given a grid view) |
| `flood_fill_solver.py` | Thin node (`explorer_type:=flood_fill`): `/map` → `OccupancyGridView`; map→base_link pose → current cell; sense walls; brain → next cell; drive to its center; repeat. Owns the entrance seal + exit self-check + a per-move stall watchdog. | no |

## Flood-fill algorithm

State: `walls` (per-cell, per-edge knowledge, symmetric — marking edge `(c, N)`
also marks `(c+N, S)`); `goal = (10, 9)`.

Loop each arrival at a cell `cur`:
1. **Sense:** `cell_walls.sense_walls(grid_view, cur)` → mark each confidently-sensed
   edge OPEN/WALL in `walls` (symmetric).
2. **Flood:** BFS from `goal` over edges that are **not WALL** (UNKNOWN counts as
   passable — optimistic), assigning each reachable cell its distance-to-goal.
3. **Choose:** `next = ` the neighbor of `cur` reachable by a non-WALL edge with the
   **smallest** distance-to-goal (tie-break deterministically, e.g. by direction
   order). If `cur == goal` → DONE. If no non-WALL neighbor reduces distance / goal
   unreachable under current knowledge → report STUCK (shouldn't happen in a connected
   maze; surfaces a sensing bug).
4. **Drive** to `next`'s center; on arrival, repeat.

Because UNKNOWN edges are optimistically passable and the goal is known, the robot
always heads toward the exit along believed-open edges, discovers the real walls, and
re-floods when blocked — the standard micromouse guarantee of reaching the goal in a
connected grid maze. Memory lives in `walls` (integer-indexed cells → no aliasing).

## Wall sensing

At `cur` (center known in map frame), for each direction the shared edge with the
neighbor is a 2 m-spaced line ~1 m from the center. Sample the SLAM occupancy grid
across that edge (a few points via `OccupancyGridView.is_occupied` /
`line_min_clearance` with the robot radius): mostly-occupied → WALL; clear corridor →
OPEN; too-unknown → leave UNKNOWN (re-sensed on a later visit). The robot also senses
opportunistically while driving, but the authoritative sense is at cell centers.

## Locomotion — cell-center → cell-center

Each move is a 2 m hop to an adjacent cell center. A short **reactive centerline
drive** (reuse `maze_perception`'s centered-target + the GCN reactive `/cmd_vel_nav`
drive) aims at the next center within the real envelope (v, w ≤ 0.5; accel-limited),
keeping clear of side walls. No Nav2 point-goals → no wedging. On arrival the node
re-localizes via `world_to_cell` and re-senses (self-correcting against SLAM drift;
2 m cells tolerate sub-0.5 m drift). A per-move watchdog (no progress for T s) backs
up and retries the hop.

## Offline validation (the guarantee — faithful this time)

Extend `maze_sim.py` with an **inertia-aware** mode: rate-limit commanded `(v, w)` to
the real envelope (clamp to ±0.5; linear accel ≤ 0.5 m/s², angular accel ≤ 0.8 rad/s²)
before integrating. (This is the model that, in prototyping, correctly flipped the
wall-follower from "solved" to "loops" — i.e. it reproduces real behavior the old
idealized sim could not.)

Two offline test layers:
1. **Brain unit tests** (`test_flood_fill_brain.py`): on a hand-built known maze, the
   distance field and chosen path are correct; a dead-end forces backtracking; marking
   a new wall re-floods and reroutes; `is_done` at the goal.
2. **End-to-end guarantee** (`test_flood_fill_maze_sim.py`): derive **ground-truth
   cell walls** from the real `maze_wall_segments_20260528.yaml`; run the brain +
   `cell_walls` (sensing against the ground-truth walls) + the inertia-aware unicycle
   driving cell-to-cell; assert it **reaches cell (10, 9), stays inside the outer-wall
   box** (reuse `outer_boundary_box`), within a step budget. `cell_walls` unit-tested
   against synthetic occupancy patterns.

The Gazebo run is gated on these passing first.

## Reuse vs new

**Reuse:** `grid_utils.OccupancyGridView` (occupancy → cell, free/occupied/clearance);
`maze_perception` (centerline / centered-goal target); the GCN reactive drive
(`corridor_navigator` / `maze_explorer` drive logic) for cell hops; the **entrance
seal** (`entrance_seal_segment` + the solver's seal arming — belt-and-suspenders,
though centerline hops keep the robot away from the entrance); `outer_boundary_box`
(stay-inside check); `maze_sim` segments (ground-truth walls). GCN
(`guided_corridor_mode`) stays as the guaranteed fallback.

**New:** `flood_fill_brain.py`, `cell_walls.py`, `flood_fill_solver.py`, the
inertia-aware mode in `maze_sim.py`, and the two offline test files; plus an
`explorer_type:=flood_fill` branch in the launch and a `tools/run_flood_fill_maze.sh`
runner.

## Success criteria

- Offline: the brain unit tests pass, and the inertia-aware end-to-end guarantee
  reaches cell (10, 9) staying inside the box.
- Gazebo: autonomously reaches the exit from the interior, **never leaving the
  perimeter**, reliably — target **≥ 4/5** GUI/headless runs `EXIT_REACHED`, confirmed
  by a trajectory-vs-maze plot (0 samples outside the box).

## Risks

1. **SLAM-drift localization:** mis-mapping pose→cell near boundaries. Mitigated by
   2 m cells + re-localize/re-sense at each center + cell-center driving that
   self-corrects. If drift exceeds ~1 m this needs attention.
2. **Wall sensing from a sparse early map:** the SLAM grid may be UNKNOWN ahead before
   the robot is close. Handled by UNKNOWN = optimistically-passable (the robot drives
   in and senses), and by re-sensing on later visits; a cell isn't committed OPEN
   until confidently sensed.
3. **Cell-hop locomotion robustness:** a 2 m reactive hop must stay centered in a
   1.76 m corridor. Validated in the inertia-aware sim and then Gazebo; the watchdog
   backs up and retries a failed hop.

## Out of scope

- Optimal/shortest-path refinement (just reach the exit; a second flood-fill "speed
  run" is a future option).
- Dynamic obstacles; changing the Gazebo world or the known grid geometry.
- Re-litigating the wall-follower: its seal + honest guarantee + the diagnostic record
  remain as history (the inertia model is promoted here as reusable infrastructure).
