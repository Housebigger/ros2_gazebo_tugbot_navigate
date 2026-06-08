# Phase33F Final Marker Orientation Gazebo Visual Re-check Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `VISUAL_ONLY_RUN_ACTIVE_PENDING_HUMAN_ACCEPTANCE`
Phase33E accepted status: `PASS_AS_VISUAL_MARKER_ORIENTATION_CORRECTED_NOT_PROMOTED`

## Scope

Phase33F starts a visual-only Gazebo/RViz re-check for the final marker orientation in:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

The purpose is human visual confirmation that Phase33E's yaw-only correction now
makes:

- `maze_entrance_arrow_visual` point into the maze;
- `maze_exit_finish_band_visual` appear as the desired horizontal black/white
  checker finish band;
- `maze_exit_marker` remain hidden and not cover the finish band.

No navigation conclusion is made in this phase.

## Guardrails held

- no autonomous navigation launched;
- no SLAM/Nav2 exploration launched;
- no `maze_explorer` launched;
- no Nav2/MPPI/controller parameter changes;
- no fallback / terminal-acceptance continuation;
- no wall segment geometry changes;
- no `scale_factor` changes;
- no entrance/exit coordinate changes;
- no Tugbot initial pose changes;
- no overwrite of `src/tugbot_gazebo/worlds/tugbot_maze_world.sdf`;
- no overwrite of `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf`;
- no promotion of scaled candidate world;
- no navigation conclusion.

## Wrapper

Created visual-only wrapper:

```text
tools/run_phase33f_final_marker_orientation_visual_recheck.sh
```

Wrapper behavior:

- creates `log/phase33f_final_marker_orientation_visual_recheck/`;
- verifies the final scaled SDF exists;
- refuses to launch scaffold/unscaled clean world as Phase33F world;
- records hashes and final marker SDF parse before launch;
- launches visible Gazebo with `headless:=false`;
- launches RViz only for TF/RobotModel/topic viewing;
- records ROS graph, Gazebo graph, processes, screenshot, and manual checklist;
- leaves Gazebo/RViz active for human observation.

Launch command:

```bash
ros2 launch tugbot_gazebo tugbot_gazebo.launch.py \
  world_sdf:=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf \
  headless:=false \
  use_sim_time:=true
```

RViz:

```text
install/tugbot_bringup/share/tugbot_bringup/rviz/tugbot_nav.rviz
```

RViz is for viewing only. Do not send navigation goals.

## Active visual run

The visual-only run is active for human inspection.

Launch PID:

```text
1549788
```

## Final marker checks

SDF parse confirms:

```text
world_name tugbot_maze_world_20260528_clean_scaled2x
wall_count 53
```

Entrance marker:

```text
model: maze_entrance_arrow_visual
style: triangle_arrow
pose: -10.661 -9.025 0.005 0.000 0.000 3.142
expected: green triangle arrow, visually points into maze
collisions: 0
visuals: 3
```

Exit finish band:

```text
model: maze_exit_finish_band_visual
style: rotated_checker_finish_band
pose: 10.061 9.058 0.005 0.000 0.000 -3.141
expected: black/white checker finish band in final human-approved orientation
collisions: 0
visuals: 8
```

Exit marker:

```text
model: maze_exit_marker
style: hidden_semantic_marker
pose: 10.061 9.058 0.010 0.000 0.000 0.000
expected: no visible green disk, not covering finish band
collisions: 0
visuals: 1 tiny/transparent semantic marker
```

Tugbot pose remains:

```text
-11.011 -9.025 0.000 0.000 0.000 0.000
```

## Runtime verification snapshot

ROS nodes observed:

```text
/camera_link_static_tf
/camera_optical_frame_static_tf
/ros_gz_bridge
/rviz
/scan_omni_static_tf
/transform_listener_impl_648f6fa6ed80
```

Forbidden autonomous nodes were absent:

```text
maze_explorer absent
controller_server absent
planner_server absent
bt_navigator absent
slam_toolbox absent
map_server absent
amcl absent
behavior_server absent
waypoint_follower absent
velocity_smoother absent
smoother_server absent
```

Visual-only processes observed:

```text
ros2 launch tugbot_gazebo tugbot_gazebo.launch.py ... tugbot_maze_world_20260528_clean_scaled2x.sdf headless:=false
gz sim -r ... tugbot_maze_world_20260528_clean_scaled2x.sdf
gz sim server
gz sim gui
rviz2 -d .../tugbot_nav.rviz
ros_gz_bridge
static_transform_publisher nodes
```

Gazebo world services confirm loaded final scaled world:

```text
/world/tugbot_maze_world_20260528_clean_scaled2x/control
/world/tugbot_maze_world_20260528_clean_scaled2x/scene/info
/world/tugbot_maze_world_20260528_clean_scaled2x/set_pose
/world/tugbot_maze_world_20260528_clean_scaled2x/state
```

## Artifacts

Artifact directory:

```text
log/phase33f_final_marker_orientation_visual_recheck/
```

Files:

```text
phase33f_final_marker_orientation_visual_recheck_preflight.txt
phase33f_final_marker_orientation_visual_recheck_launch.log
phase33f_final_marker_orientation_visual_recheck_launch.pid
phase33f_final_marker_orientation_visual_recheck_rviz.log
phase33f_final_marker_orientation_visual_recheck_rviz.pid
phase33f_final_marker_orientation_visual_recheck_manual_checklist.txt
phase33f_final_marker_orientation_visual_recheck_ros_graph.txt
phase33f_final_marker_orientation_visual_recheck_gz_graph.txt
phase33f_final_marker_orientation_visual_recheck_processes.txt
phase33f_final_marker_orientation_visual_recheck_verification.txt
phase33f_final_marker_orientation_visual_recheck_display_root.xwd
```

Screenshot:

```text
phase33f_final_marker_orientation_visual_recheck_display_root.xwd
```

## Preservation checks

Hashes at verification:

```text
5867385cd3eb72202c25e8bdd16c63591df6b947facada139d8fa8bab403d3a5  src/tugbot_gazebo/worlds/tugbot_maze_world.sdf
da33e9e58c6c07089b6bb7156315759b4bceb989db2f1ace18f0b1af0b0c0301  src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf
d7687388a9c91d2466c52518d3b9e2b93dcac433c21c971d9f4b2101bd03d655  src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

Diff guards:

```text
git diff -- src/tugbot_gazebo/worlds/tugbot_maze_world.sdf
# empty

git diff -- src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf
# empty

git diff -- src/tugbot_navigation/config
# empty
```

## Manual observation checklist

Please inspect Gazebo/RViz and answer:

1. `maze_entrance_arrow_visual` 是否为绿色三角箭头，并指向迷宫内部；
2. `maze_exit_finish_band_visual` 是否为横向黑白格子终点带；
3. `maze_exit_marker` 是否不再显示绿色圆盘、不遮挡终点带；
4. 墙体布局、尺度、入口/出口位置、Tugbot 初始位姿是否保持正确。

## Current status

```text
VISUAL_ONLY_RUN_ACTIVE_PENDING_HUMAN_ACCEPTANCE
```

The visual-only Gazebo/RViz run remains active. No acceptance/rejection or
navigation conclusion is made until human feedback is reported.
