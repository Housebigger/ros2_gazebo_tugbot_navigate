# Phase33E Refined Marker Visual Re-check Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `VISUAL_ONLY_RUN_ACTIVE_PENDING_HUMAN_ACCEPTANCE`
Phase33D accepted status: `PASS_AS_VISUAL_MARKER_STYLE_REFINED_NOT_PROMOTED`

## Scope

Phase33E starts a visual-only Gazebo/RViz re-check for refined marker style in:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

The purpose is human visual confirmation that:

- the entrance marker now reads as a green isosceles triangle arrow;
- the entrance arrow points +X into the maze;
- the exit finish band is a rotated black/white checker band spanning the exit
  width;
- the green circular exit marker is hidden and no longer covers the finish band.

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
- no overwrite of `src/tugbot_gazebo/worlds/tugbot_maze_world.sdf`;
- no overwrite of `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf`;
- no promotion of scaled candidate world;
- no navigation conclusion.

## Wrapper

Created visual-only wrapper:

```text
tools/run_phase33e_refined_marker_visual_recheck.sh
```

Wrapper behavior:

- creates `log/phase33e_refined_marker_visual_recheck/`;
- verifies the refined scaled SDF exists;
- refuses to launch scaffold/unscaled clean world as Phase33E world;
- records hashes and refined SDF parse before launch;
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

Wrapper session:

```text
proc_ad1a02d5cbda
```

Wrapper exited after starting long-lived Gazebo/RViz processes.

Launch PID:

```text
1453020
```

## Refined marker checks

SDF parse confirms:

```text
world_name tugbot_maze_world_20260528_clean_scaled2x
wall_count 53
```

Entrance marker:

```text
model: maze_entrance_arrow_visual
style: triangle_arrow
expected style: green_isosceles_triangle_arrow
pose: -10.661 -9.025 0.005 0.000 0.000 0.000
direction: +X into maze
collisions: 0
visuals: 3
```

Exit finish band:

```text
model: maze_exit_finish_band_visual
style: rotated_checker_finish_band
expected style: rotated black/white checker band
pose: 10.061 9.058 0.005 0.000 0.000 1.571
yaw: approx 1.570796 rad
collisions: 0
visuals: 8
```

Exit marker:

```text
model: maze_exit_marker
style: hidden_semantic_marker
expected: semantic-only hidden marker, no visible green ground disk
pose: 10.061 9.058 0.010 0.000 0.000 0.000
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
/transform_listener_impl_62c1c4751ed0
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

Gazebo world services confirm loaded refined scaled world:

```text
/world/tugbot_maze_world_20260528_clean_scaled2x/control
/world/tugbot_maze_world_20260528_clean_scaled2x/scene/info
/world/tugbot_maze_world_20260528_clean_scaled2x/pose/info
/world/tugbot_maze_world_20260528_clean_scaled2x/stats
```

## Artifacts

Artifact directory:

```text
log/phase33e_refined_marker_visual_recheck/
```

Files:

```text
phase33e_refined_marker_visual_recheck_preflight.txt
phase33e_refined_marker_visual_recheck_launch.log
phase33e_refined_marker_visual_recheck_launch.pid
phase33e_refined_marker_visual_recheck_rviz.log
phase33e_refined_marker_visual_recheck_rviz.pid
phase33e_refined_marker_visual_recheck_manual_checklist.txt
phase33e_refined_marker_visual_recheck_ros_graph.txt
phase33e_refined_marker_visual_recheck_gz_graph.txt
phase33e_refined_marker_visual_recheck_processes.txt
phase33e_refined_marker_visual_recheck_verification.txt
phase33e_refined_marker_visual_recheck_display_root.xwd
```

Screenshot:

```text
phase33e_refined_marker_visual_recheck_display_root.xwd
```

## Preservation checks

Hashes at verification:

```text
5867385cd3eb72202c25e8bdd16c63591df6b947facada139d8fa8bab403d3a5  src/tugbot_gazebo/worlds/tugbot_maze_world.sdf
da33e9e58c6c07089b6bb7156315759b4bceb989db2f1ace18f0b1af0b0c0301  src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf
7ea6084b49aa1b0585a87ab6e6f1380e25be5bd8375590f9a774e990fa0c7aa6  src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
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

1. 入口三角箭头是否清楚，不再像正方形；
2. 入口三角箭头是否指向迷宫内部 +X；
3. 出口黑白终点带是否横向铺满出口 opening；
4. 绿色圆形 exit marker 是否已隐藏且不遮挡终点带；
5. 墙体布局、尺度、入口/出口位置是否保持不变；
6. 是否可进入下一步 SLAM/map smoke。

## Current status

```text
VISUAL_ONLY_RUN_ACTIVE_PENDING_HUMAN_ACCEPTANCE
```

The visual-only Gazebo/RViz run remains active. No acceptance/rejection or
navigation conclusion is made until human feedback is reported.
