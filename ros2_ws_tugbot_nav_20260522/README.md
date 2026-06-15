# Maze Navigation — `ros2_ws_tugbot_nav_20260522`

ROS 2 (Jazzy) + Gazebo Harmonic stack that drives the Tugbot from the maze entrance
(map `(0, 0)`) to the exit (map `(21.07, 18.08)`, radius `1.2 m`). This workspace evolved
from `ros2_ws_tugbot_nav_20260514` and reuses its proven Gazebo + `ros_gz_bridge` +
`slam_toolbox` + Nav2 chain. Its **reliable maze completion is Guided Corridor Navigation
(GCN)**; fully-autonomous discovery was attempted at length but plateaus short of the exit.

> **Successor:** the fully-autonomous traversal problem was later *solved* in
> `../ros2_ws_tugbot_nav_20260614` via reactive **wall-following** (the maze is a perfect
> maze — a tree — so the left/right-hand rule is guaranteed to reach a boundary exit). See
> that workspace's `README.md`. This `20260522` workspace remains the GCN-completion + the
> autonomous-DFS baseline that motivated it.

## Modes

| Configuration | Strategy | Status |
|---|---|---|
| **GCN** — `explorer_type:=maze_dfs guided_corridor_mode:=true` | Follow a pre-computed 18-corridor BFS route (`corridor_navigator.py`, `MAZE_CORRIDORS`); each ~1.5 m centerline-snapped goal via Nav2, with a 10 Hz reactive `/cmd_vel_nav` fallback | ✅ **Reliable completion — 4/4 clean runs** |
| Autonomous DFS — `guided_corridor_mode:=false` | SLAM + junction-topology Trémaux-like DFS discovery (`maze_explorer.py` + `maze_topology.py`) | ⚠️ Explores untrapped but **plateaus ~7.8 m from the exit** (oscillates in an SE pocket) |
| `explorer_type:=frontier` | Inherited 0514 frontier/perimeter explorer | ⚠️ Baseline / fallback |

**Why GCN completes where autonomous DFS stalls.** GCN exploits the *fully-known* maze
layout: short straight centerline goals keep Nav2 reliable in the 1.76 m corridors. Blind
DFS + SLAM drift + Nav2 MPPI timeouts in those narrow corridors get lost in the maze's
complex second half and trap the robot ~7.8 m from the NE-corner exit. (The successor
workspace's wall-following sidesteps both failure modes entirely.)

## How to run

```bash
cd ros2_ws_tugbot_nav_20260522
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash

# Reliable maze completion — GCN, headless, 600 s budget:
./tools/run_gcn_maze.sh 600 true
#   Artifacts: log/gcn_run_<stamp>/  (result.txt = EXIT_REACHED/EXHAUSTED/TIMEOUT, launch.log)

# GCN reliability batch (N runs, tallies EXIT_REACHED):
./tools/run_gcn_reliability.sh 4 540

# Visual acceptance — GCN with Gazebo GUI + RViz (3rd arg = use_rviz):
./tools/run_gcn_maze.sh 600 false true

# Autonomous DFS baseline (discovers the maze on its own; plateaus ~7.8 m short):
./tools/run_dfs_maze.sh 900 true false 400

# Verify the BFS corridor path clears robot-radius against the source maze PNG (no ROS):
python3 tools/verify_gcn_path.py
```

Launch directly (e.g. to watch GCN live):
`ros2 launch tugbot_bringup tugbot_maze_explore.launch.py explorer_type:=maze_dfs guided_corridor_mode:=true headless:=false use_rviz:=true`.

`tools/` keeps only these four current entry points; the ~206 one-shot `phaseNN`
diagnostic scripts from the long bring-up are preserved under `tools/archive/`.

## Active world & maze source files

The human-accepted active Gazebo world and its source maze (see `doc/ACTIVE_MAZE_WORLD.md`):

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf   # active world (launch default)
src/tugbot_maze/assets/maze_20260528.png                                 # clean 2D binary source image
src/tugbot_maze/config/maze_20260528_scaled_instance.yaml                # scaled maze metadata (entrance/exit/frame)
src/tugbot_maze/config/maze_wall_segments_20260528.yaml                  # wall segments (used by path verification)
```

Geometry: the map frame = original maze coords × 2 + `(11.011281, 9.025070)`; the exit at
world `(10.061, 9.058)` maps to `(21.073, 18.084)`. Deprecated artifacts (the decorative
`maze_20260522.jpg`, scaffold worlds, unscaled candidate) live under
`archive/deprecated_maze_worlds/` and `doc/archive/phase_old_maze_artifacts/`.

## GCN architecture

GCN delegates the *route* to a corridor table and the *motion* to the explorer node:

| Unit | Role |
|---|---|
| `tugbot_maze/corridor_navigator.py` | `MAZE_CORRIDORS` — the 18-corridor BFS route entrance→exit; hands the node centerline-snapped ~1.5 m waypoints |
| `tugbot_maze/maze_explorer.py` | The node + control loop: dispatches each corridor goal to Nav2; 10 Hz reactive `/cmd_vel_nav` fallback; watchdog/unwedge; exit self-check |
| `tugbot_maze/maze_goal_monitor.py` | Independent EXIT_REACHED detector (pose within exit radius) |
| Nav2 (`NavigateToPose`) + `slam_toolbox` | Per-goal planning/control + online `map`/pose |

Four fixes made GCN reliable (early runs were ~1-in-4): a **10 Hz reactive control loop**
(the 0.5 Hz main tick stuttered against `velocity_smoother`'s 1.0 s timeout — the single
biggest fix); reactive heading aimed at the **centerline-snapped waypoint** (pulls the robot
off a wall it has jammed on); a **watchdog + 0.4 m back-out unwedge**; and **disabling the
periodic 360° relocalization spin** in GCN mode (harmful to narrow-corridor scan-matching).

## Repository layout

```text
src/            6 ROS 2 packages: tugbot_{bringup,description,gazebo,maze,navigation,exploration}
tools/          4 current runners (run_gcn_maze.sh, run_dfs_maze.sh, run_gcn_reliability.sh,
                verify_gcn_path.py); tools/archive/ = ~206 historical phaseNN diagnostics
doc/            ACTIVE_MAZE_WORLD.md, doc_experience/ (lessons), the obstacle-reproduction
                workflow; doc/archive/ = per-phase design reviews + reports + old maze artifacts
docs/superpowers/   current spec/plan convention (e.g. the entry-direct-bypass design)
```

`build/`, `install/`, and `log/` are colcon-generated and git-ignored (regenerated by
`colcon build`); they are not tracked.
