# Autonomous Maze Navigation — `ros2_ws_tugbot_nav_20260614`

ROS 2 (Jazzy) + Gazebo Harmonic stack that drives the Tugbot from the maze entrance
(map `(0, 0)`) to the exit (map `(21.07, 18.08)`). Several explorer strategies are
selectable via the `explorer_type` launch argument; the **reliable autonomous** one is
reactive **wall-following**.

## Explorer modes

| `explorer_type` | Strategy | Status |
|---|---|---|
| **`wall_follower`** | Reactive right/left-hand wall-following (ROS-free core) | ✅ **Reliable autonomous 通关 — 5/5 headless** |
| `tremaux` + `guided_corridor_mode:=true` (GCN) | Follow a pre-computed BFS corridor sequence | ✅ Reliable, but *guided* (not autonomous discovery) |
| `tremaux` (`guided_corridor_mode:=false`) | Junction-graph Trémaux DFS / SLAM-frontier coverage | ⚠️ Earlier autonomous attempt — explores untrapped but plateaus ~8.7 m from the exit |
| `frontier` | Inherited frontier/perimeter explorer | ⚠️ Earlier attempt |
| `maze_dfs` | Original greedy+novelty DFS explorer (launch default) | ⚠️ Earlier attempt |

**Why wall-following is the reliable autonomous solver.** The maze is a *perfect maze*
(a tree — 100 cells, 99 edges, no loops), with entrance and exit both on the outer
boundary. The left/right-hand rule is therefore *mathematically guaranteed* to reach the
exit, and it structurally avoids the failure that capped the goal-based explorers below:
it holds a **fixed lateral offset** from one wall via continuous LIDAR control — no Nav2
point-goals aimed into the 1.76 m corridors (so it **cannot wedge**), and a tree has no
cycles (so it **cannot loop forever**).

## How to run

```bash
cd ros2_ws_tugbot_nav_20260614
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash

# Reliable autonomous solver — reactive wall-following (headless, 1500 s, left hand):
./tools/run_wall_follower_maze.sh 1500 true false left
#   Artifacts: log/wall_follower_run_<stamp>/  (result.txt, launch.log; DIAG pose/dist_to_exit every 5 s)

# Reliability batch (N runs, tallies EXIT_REACHED; target ≥ 4/5):
./tools/run_wall_follower_reliability.sh 5 1500 left

# Offline guarantee proof + full unit suite (pure Python, no ROS needed, ~3 s):
cd src/tugbot_maze && python3 -m pytest test/test_wall_follower.py test/test_maze_sim.py \
  test/test_wall_follow_control.py test/test_wall_follow_maze_sim.py -q
```

Launch directly:
`ros2 launch tugbot_bringup tugbot_maze_explore.launch.py explorer_type:=wall_follower follow_side:=left`.
`follow_side` defaults to `left` — ~5.4× faster than `right` here, because it traces the
short outer-boundary arc to the NE exit rather than the long interior route (1721 vs 9320
sim-steps in the offline proof, which selects the faster hand objectively).

## Wall-following architecture

A thin ROS node delegates **all** control to a ROS-free reactive core, validated offline
by a ROS-free maze simulator *before* any Gazebo run:

| Unit | Role | ROS-free? | Tests |
|---|---|---|---|
| `wall_follower.py` | Reactive core — `sectorize` (LaserScan → front/side/front_side minima) + `WallFollower` state machine (FIND_WALL/FOLLOW/TURN_AWAY/CORNER; PID on lateral wall offset; **all** angular outputs clamped to `w_max`) | yes | 16 |
| `wall_follow_control.py` | Node-support helpers — `exit_reached`, `entering_done`, `StallWatchdog` (time injected) | yes | 7 |
| `maze_sim.py` | **Test-only** raycaster + unicycle integrator over the real `20260528` wall segments — proves exit-reaching offline and picks the faster hand | yes | 9 + 6 guarantee |
| `wall_follow_solver.py` | Thin node (`explorer_type:=wall_follower`) — `/scan` → `sectorize` → `WallFollower` → `/cmd_vel_nav` at 10 Hz; entry + stall recovery + exit self-check | no | smoke |

`slam_toolbox` runs only for the `map→base_link` pose used by the exit check; **Nav2,
costmaps, and frontier are not in the control loop** (they stay available behind the other
`explorer_type` flags). Robust tuning: CORNER orbit radius `R = corner_v/corner_w ≈ 0.75 m`
(`corner_v=0.45, corner_w=0.6`), `turn_w=1.0`, `w_max=1.0`, `follow_side='left'`. See
`docs/superpowers/specs/2026-06-15-wall-following-solver-design.md` for the full design and
the robustness lesson (a first tuning passed the offline proof with overfit parameters; a
robustness sweep caught it and the validated R≈0.75 point passes 54/54 start perturbations).

## Prior autonomous attempts (Trémaux / frontier / DFS)

Before wall-following, three goal-based explorers were built and tuned to push past the
`20260522` ceiling (which trapped ~7.8 m from the exit, oscillating). All of them explore
the maze **untrapped** and relocate correctly, but **plateau ~8.7–11 m from the NE-corner
exit** within a 1500 s budget — Nav2 point-goals wedge in the 1.76 m corridors. They remain
selectable behind their `explorer_type` flags as a comparison baseline. The decision logic
is clean and ROS-free (millisecond unit tests); the gap was always the narrow-corridor
*navigation/efficiency* layer, which wall-following sidesteps entirely.

```bash
# Trémaux / frontier-coverage autonomous solver (the best of the prior attempts):
./tools/run_solver_maze.sh 1500 true false 400
./tools/run_solver_reliability.sh 5 1500 400
cd src/tugbot_maze && python3 -m pytest test/test_tremaux_solver.py test/test_tremaux_pocket_escape.py \
  test/test_reactive_pilot.py test/test_dead_end_classifier.py test/test_frontier_coverage.py -v
```

### Architecture (prior attempts)

A thin node delegated *decisions* to a strategy module and *motion* to a reactive pilot:

| Unit | Role | ROS-free? | Tests |
|---|---|---|---|
| `maze_solver.py` | Thin node + control loop (startup→entering→deciding→busy→done); ROS I/O; publishes `/maze_boundary_map`; exit self-check; DIAG log | no | (sim) |
| `frontier_coverage.py` | Primary prior explorer — SLAM-grid frontier detect → cluster → select nearest exit-biased max-clearance goal (coverage-complete, no node aliasing) | yes | 12 |
| `tremaux_solver.py` | Junction-graph Trémaux DFS with edge visit-counts (behind `use_frontier_coverage:=false`) | yes | 13 |
| `reactive_pilot.py` | Locomotion — short Nav2 goal + 10 Hz reactive `/cmd_vel_nav` fallback + watchdog/unwedge + back-out | no | 10 |
| `dead_end_classifier.py` | Geometry-confirmed dead end vs transient nav failure (no false blacklists) | yes | 4 |

Reused unchanged by the prior attempts and the GCN path: Nav2 (NavigateToPose),
`slam_toolbox`, Gazebo, `maze_perception`, `grid_utils`, `corridor_navigator` (GCN),
`maze_goal_monitor`.

### Results across efforts

| | 20260522 (DFS) | 20260614 Trémaux/frontier | 20260614 **wall-following** |
|---|---|---|---|
| Behavior | trapped / oscillating | explores untrapped, relocates | follows the boundary to the exit |
| Closest to exit | ~7.8 m (stuck) | 8.69 m (still progressing at timeout) | **reaches it** |
| Completion | no | no | ✅ reliable (5/5) |

### Defects found & fixed while bringing up the prior attempts

Kept as a record (each with a regression test where unit-testable):

1. Entry-corridor deadlock (map-unknown-ahead vs scan-open) → bounded re-sample + forward nudge.
2. Nav2 never planned — node didn't publish `/maze_boundary_map` (the global-costmap `maze_boundary_layer`) → ported the publisher.
3. Brain livelock re-committing one branch → advance-on-SUCCESS.
4. Local oscillation (Trémaux) — came-from branch stayed UNTRIED → mark it EXPLORED on arrival.
5. `active_branch` idempotency guard for a desync.
6. Frontier livelock — excluded an unreachable frontier by *centroid* but re-selected it by *goal* → exclude by goal point.
7. Frontier wedging — goals sat near walls → choose the max-clearance cell in each cluster.

### Why they plateaued (the open problem that motivated wall-following)

Full autonomous completion (`≥4/5 EXIT_REACHED`) was never reached by the goal-based
explorers: the robot gets ~8.7–11 m from the exit from various sides but not to the precise
NE-corner exit within budget, because (a) narrow-corridor **wedging** dominates (many goals
end in a wedge→exclude→relocate cycle, capping coverage to ~40–55 goals/run) and (b) the
exit corner is a hard, precise approach the coverage rarely maps in time. A loop-free maze
made the fix structural rather than heuristic — see the wall-following section above. (A maze
*with* loops would instead need Trémaux/Pledge — out of scope for this maze.)
