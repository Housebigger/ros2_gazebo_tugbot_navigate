# Phase33C Corrected Marker Gazebo/RViz Visual Re-check Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `VISUAL_ONLY_RUN_ACTIVE_PENDING_HUMAN_ACCEPTANCE`
Phase33B accepted status: `PASS_AS_MARKER_ALIGNMENT_CORRECTED_NOT_PROMOTED`

## Scope

Phase33C starts a visual-only Gazebo/RViz re-check for the corrected scaled clean
world:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

The purpose is human visual confirmation that Phase33B's corrected entrance
arrow, Tugbot initial pose, exit finish band, and exit marker align with the
actual boundary openings.

No navigation conclusion is made in this phase.

## Guardrails held

- no autonomous navigation launched;
- no SLAM/Nav2 exploration launched;
- no `maze_explorer` launched;
- no Nav2/MPPI/controller parameter changes;
- no fallback / terminal-acceptance continuation;
- no navigation strategy changes;
- no wall segment geometry changes;
- no `scale_factor` changes;
- no overwrite of `src/tugbot_gazebo/worlds/tugbot_maze_world.sdf`;
- no overwrite of `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf`;
- no promotion of scaled candidate world.

## Wrapper

Created wrapper:

```text
tools/run_phase33c_corrected_marker_visual_recheck.sh
```

Wrapper behavior:

- creates `log/phase33c_corrected_marker_visual_recheck/`;
- verifies the corrected scaled SDF exists;
- refuses to launch scaffold/unscaled clean world as the Phase33C world;
- records hashes and SDF marker parse before launch;
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
proc_c6e44096381f
```

Wrapper exited after starting long-lived Gazebo/RViz processes.

Launch PID:

```text
849613
```

## Corrected target checks

SDF parse confirms:

```text
world_name tugbot_maze_world_20260528_clean_scaled2x
wall_count 53
```

Tugbot initial pose:

```text
expected: (-11.011281337, -9.025069638, yaw=0)
observed: -11.011 -9.025 0.000 0.000 0.000 0.000
```

Entrance arrow:

```text
model: maze_entrance_arrow_visual
expected pose approx: (-10.661281337, -9.025069638, z=0.005)
observed pose: -10.661 -9.025 0.005 0.000 0.000 0.000
expected direction: +X into maze
collisions: 0
```

Exit finish band:

```text
model: maze_exit_finish_band_visual
expected pose approx: (10.061281337, 9.058495822, z=0.005)
observed pose: 10.061 9.058 0.005 0.000 0.000 0.000
collisions: 0
```

Exit marker:

```text
model: maze_exit_marker
expected pose approx: (10.061281337, 9.058495822, z=0.010)
observed pose: 10.061 9.058 0.010 0.000 0.000 0.000
collisions: 0
```

## Runtime verification snapshot

ROS nodes observed:

```text
/camera_link_static_tf
/camera_optical_frame_static_tf
/ros_gz_bridge
/rviz
/scan_omni_static_tf
/transform_listener_impl_607462cc22b0
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

Gazebo world services confirm loaded corrected scaled world:

```text
/world/tugbot_maze_world_20260528_clean_scaled2x/control
/world/tugbot_maze_world_20260528_clean_scaled2x/scene/info
/world/tugbot_maze_world_20260528_clean_scaled2x/pose/info
/world/tugbot_maze_world_20260528_clean_scaled2x/stats
```

## Artifacts

Artifact directory:

```text
log/phase33c_corrected_marker_visual_recheck/
```

Files:

```text
phase33c_corrected_marker_visual_recheck_preflight.txt
phase33c_corrected_marker_visual_recheck_launch.log
phase33c_corrected_marker_visual_recheck_launch.pid
phase33c_corrected_marker_visual_recheck_rviz.log
phase33c_corrected_marker_visual_recheck_rviz.pid
phase33c_corrected_marker_visual_recheck_manual_checklist.txt
phase33c_corrected_marker_visual_recheck_ros_graph.txt
phase33c_corrected_marker_visual_recheck_gz_graph.txt
phase33c_corrected_marker_visual_recheck_processes.txt
phase33c_corrected_marker_visual_recheck_verification.txt
phase33c_corrected_marker_visual_recheck_display_root.xwd
```

Screenshot:

```text
phase33c_corrected_marker_visual_recheck_display_root.xwd
```

## Preservation checks

Hashes at verification:

```text
5867385cd3eb72202c25e8bdd16c63591df6b947facada139d8fa8bab403d3a5  src/tugbot_gazebo/worlds/tugbot_maze_world.sdf
da33e9e58c6c07089b6bb7156315759b4bceb989db2f1ace18f0b1af0b0c0301  src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf
4d64778340a48fa1bf95d3d46eb861734fbe62c83a92f472bfc7abd9f24a1d47  src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
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

1. 入口是否正确；
2. 出口是否正确；
3. 右上 opening 是否确实是你希望的终点；
4. 箭头和终点带是否贴地、无碰撞、可见；
5. Tugbot 是否摆在入口外侧且朝向入口；
6. 是否可进入下一步 SLAM/map smoke。

## Current status

```text
VISUAL_ONLY_RUN_ACTIVE_PENDING_HUMAN_ACCEPTANCE
```

The visual-only Gazebo/RViz run remains active. No acceptance/rejection or
navigation conclusion is made until human feedback is reported.
