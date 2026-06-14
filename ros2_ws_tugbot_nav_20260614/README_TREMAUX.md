# Autonomous Maze Solver (`ros2_ws_tugbot_nav_20260614`)

A clean, unit-tested autonomous maze-exploration stack for the Tugbot, built to push
past the `20260522` ceiling (which trapped at ~7.8 m from the exit, oscillating).

**Status (2026-06-15): autonomous exploration works and is untrapped — best approach
8.69 m from the exit, covering both the north (y≈17.8) and east (x≈20.1) of the maze —
but full autonomous completion (reaching the exit at map (21.07, 18.08)) is NOT yet
reliably achieved.** The reliable way to *complete* the maze remains Guided Corridor
Navigation (`guided_corridor_mode:=true`); see the parent project notes.

## How to run

```bash
cd ros2_ws_tugbot_nav_20260614
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash

# Autonomous solver (frontier coverage, headless), 1500s, max 400 goals:
./tools/run_solver_maze.sh 1500 true false 400
# Artifacts: log/solver_run_<stamp>/  (result.txt, launch.log; DIAG pose/dist_to_exit every 5s)

# Reliability batch (N runs, tallies EXIT_REACHED):
./tools/run_solver_reliability.sh 5 1500 400

# Unit tests (pure Python, fast):
cd src/tugbot_maze && python3 -m pytest test/test_tremaux_solver.py test/test_tremaux_pocket_escape.py \
  test/test_reactive_pilot.py test/test_dead_end_classifier.py test/test_frontier_coverage.py -v
```

Launch directly: `ros2 launch tugbot_bringup tugbot_maze_explore.launch.py explorer_type:=tremaux guided_corridor_mode:=false`.

## Architecture

Thin ROS node delegates *decisions* to a strategy module and *motion* to a reliable pilot:

| Unit | Role | ROS-free? | Tests |
|---|---|---|---|
| `maze_solver.py` | Thin node + control loop (startup→entering→deciding→busy→done); ROS I/O; publishes `/maze_boundary_map`; exit self-check; DIAG log | no | (sim) |
| `frontier_coverage.py` | **Primary explorer** — SLAM-grid frontier detect → cluster → select nearest exit-biased max-clearance goal (coverage-complete, no node aliasing) | yes | 12 |
| `tremaux_solver.py` | Alternative explorer — junction-graph Trémaux DFS with edge visit-counts (behind `use_frontier_coverage:=false`) | yes | 13 |
| `reactive_pilot.py` | Locomotion — short Nav2 goal + 10 Hz reactive `/cmd_vel_nav` fallback + watchdog/unwedge + back-out | no | 10 |
| `dead_end_classifier.py` | Geometry-confirmed dead end vs transient nav failure (no false blacklists) | yes | 4 |

Reused unchanged: Nav2 (NavigateToPose), slam_toolbox, Gazebo, `maze_perception`, `grid_utils`, `corridor_navigator` (GCN), `maze_goal_monitor`.

**Default exploration: frontier coverage** (`use_frontier_coverage:=true`). The robot drives
to the nearest exit-biased frontier (edge of known space) until the map is complete; once the
exit cell is mapped free it **dashes** straight there via Nav2. Failed/unreachable frontiers are
excluded (by goal point) so the robot relocates instead of freezing.

## Results vs. the prior effort

| | 20260522 (DFS) | 20260614 (this) |
|---|---|---|
| Behavior | trapped / oscillating | explores untrapped, relocates correctly |
| Closest to exit | ~7.8 m (stuck) | **8.69 m, still progressing at timeout** |
| Coverage | SE pocket only | reaches y≈17.8 *and* x≈20.1 (both sides of the exit) |

## Defects found & fixed during bring-up (each with a regression test where unit-testable)

1. Entry-corridor deadlock (map-unknown-ahead vs scan-open) → bounded re-sample + forward nudge.
2. Nav2 never planned — node didn't publish `/maze_boundary_map` (the global-costmap `maze_boundary_layer`) → ported the publisher.
3. Brain livelock re-committing one branch → advance-on-SUCCESS.
4. Local oscillation (Trémaux) — came-from branch stayed UNTRIED → mark it EXPLORED on arrival.
5. `active_branch` idempotency guard for a desync.
6. Frontier livelock — excluded an unreachable frontier by *centroid* but re-selected it by *goal* → exclude by goal point.
7. Frontier wedging — goals sat near walls → choose the max-clearance cell in each cluster.

## Open problem (future work)

Full autonomous completion (`≥4/5 EXIT_REACHED`) is **not** achieved. The robot reaches ~8.7–11 m
from the exit from various sides but not the **NE-corner exit (21, 18)** within a 1500 s budget.
Blockers and untried levers:

- **Narrow-corridor wedging** still dominates (~40–55 frontier goals/run, many ending in a wedge→exclude→relocate cycle). Levers: a smarter local controller for 1.76 m corridors, or recovery behaviors; tuning the reactive give-up was tried and regressed (reverted).
- **The exit corner is a hard, precise approach** the coverage rarely maps within budget, so the exit-dash (sound, but gated on the exit cell being mapped free) seldom fires. Levers: a longer budget, faster controller, or an explicit "drive toward the exit corridor mouth" heuristic.
- A coverage run is stochastic (covers a different side each run).

The decision strategies are clean and ROS-free (millisecond unit tests). The remaining gap is the
narrow-corridor *navigation/efficiency* layer, not the exploration logic.
