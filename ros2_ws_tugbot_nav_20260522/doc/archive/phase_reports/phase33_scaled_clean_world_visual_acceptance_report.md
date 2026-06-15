# Phase33 Gazebo/RViz Visual Acceptance of Scaled Clean World Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `VISUAL_ONLY_RUN_ACTIVE_PENDING_HUMAN_ACCEPTANCE`
Phase32B accepted status: `PASS_AS_SCALED_CLEAN_WORLD_CANDIDATE_GENERATED_NOT_PROMOTED`

## Scope

Phase33 starts a visual-only Gazebo/RViz run for the 2x scaled clean candidate
world:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

The purpose is human visual acceptance of:

- 2x scaled 3D maze shape;
- Tugbot size relative to corridor width;
- entrance green arrow;
- exit black/white finish band;
- wall height/thickness/scale.

This phase does not run autonomous navigation and does not make navigation
behavior conclusions.

## Guardrails held

- no autonomous navigation launched;
- no SLAM/Nav2 exploration launched;
- no `maze_explorer` launched;
- no Nav2/MPPI/controller parameter changes;
- no fallback / terminal-acceptance continuation;
- no navigation strategy changes;
- no overwrite of `src/tugbot_gazebo/worlds/tugbot_maze_world.sdf`;
- no overwrite of `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf`;
- no promotion of scaled candidate world as default;
- no navigation conclusion from this visual-only phase.

## Wrapper

Created visual-only wrapper:

```text
tools/run_phase33_scaled_clean_world_visual_acceptance.sh
```

Wrapper behavior:

- creates `log/phase33_scaled_clean_world_visual_acceptance/`;
- sources ROS Jazzy and workspace setup;
- verifies the scaled candidate SDF exists;
- refuses to use scaffold/unscaled clean world as the scaled candidate output;
- records SHA256 of scaffold, unscaled clean, scaled clean, and Phase29 worlds;
- starts visible Gazebo with the scaled candidate world;
- starts RViz only for visual/TF/topic inspection;
- records ROS nodes/topics, Gazebo topics/services, processes, launch log,
  screenshot, and checklist;
- does not start Nav2, SLAM, or `maze_explorer`.

Launch command used by wrapper:

```bash
ros2 launch tugbot_gazebo tugbot_gazebo.launch.py \
  world_sdf:=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf \
  headless:=false \
  use_sim_time:=true
```

RViz was opened with:

```text
install/tugbot_bringup/share/tugbot_bringup/rviz/tugbot_nav.rviz
```

RViz is visual/TF/RobotModel/topic inspection only. Do not send goals.

## Active visual run

The visual-only run is active for human inspection.

Hermes background session used to start wrapper:

```text
proc_8e1d6d513a1c
```

Wrapper exited after starting the long-lived visual processes. The active visual
launch PID is recorded here:

```text
log/phase33_scaled_clean_world_visual_acceptance/phase33_scaled_clean_world_visual_acceptance_launch.pid
```

At verification time the launch PID was:

```text
381283
```

## Candidate world and markers

Candidate world:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

World name observed in Gazebo services/topics:

```text
tugbot_maze_world_20260528_clean_scaled2x
```

SDF parse summary:

```text
wall_count: 53
tugbot_pose: -12.000 -2.000 0.000 0.000 0.000 0.000
```

Entrance / arrow:

```text
Tugbot entrance pose: (-12.0, -2.0, yaw=0)
maze_entrance_arrow_visual: present
arrow pose: -11.200 -2.000 0.005 0.000 0.000 0.000
arrow collisions: 0
```

Finish / exit:

```text
maze_exit_finish_band_visual: present
finish band pose: 0.000 12.000 0.005 0.000 0.000 0.000
finish band collisions: 0
maze_exit_marker pose: 0.000 12.000 0.010 0.000 0.000 0.000
```

Wall scale from Phase32B:

```text
scale_factor: 2.0
wall count preserved: 53
wall thickness: 0.24m
wall height: 1.2m
```

## Logs and artifacts

Artifact directory:

```text
log/phase33_scaled_clean_world_visual_acceptance/
```

Files:

```text
phase33_scaled_clean_world_visual_acceptance_launch.log
phase33_scaled_clean_world_visual_acceptance_launch.pid
phase33_scaled_clean_world_visual_acceptance_rviz.log
phase33_scaled_clean_world_visual_acceptance_rviz.pid
phase33_scaled_clean_world_visual_acceptance_manual_checklist.txt
phase33_scaled_clean_world_visual_acceptance_ros_nodes.txt
phase33_scaled_clean_world_visual_acceptance_ros_topics.txt
phase33_scaled_clean_world_visual_acceptance_gz_topics.txt
phase33_scaled_clean_world_visual_acceptance_gz_services.txt
phase33_scaled_clean_world_visual_acceptance_processes.txt
phase33_scaled_clean_world_visual_acceptance_verification.txt
phase33_display_root.xwd
```

Screenshot note:

- `phase33_display_root.xwd` was captured as an X11 root-window screenshot.
- PNG conversion was not required for this phase; the XWD artifact is retained.

## Verification snapshot

ROS nodes observed:

```text
/camera_link_static_tf
/camera_optical_frame_static_tf
/ros_gz_bridge
/rviz
/scan_omni_static_tf
/transform_listener_impl_5c7fe6f9b390
```

Forbidden autonomous nodes were absent:

```text
/maze_explorer absent
/controller_server absent
/planner_server absent
/bt_navigator absent
/slam_toolbox absent
/map_server absent
/amcl absent
/behavior_server absent
/waypoint_follower absent
/velocity_smoother absent
/smoother_server absent
```

Visual processes observed:

```text
ros2 launch tugbot_gazebo tugbot_gazebo.launch.py ... tugbot_maze_world_20260528_clean_scaled2x.sdf headless:=false
ruby ... gz sim -r ... tugbot_maze_world_20260528_clean_scaled2x.sdf
gz sim -r ... tugbot_maze_world_20260528_clean_scaled2x.sdf
gz sim server
gz sim gui
rviz2 -d .../tugbot_nav.rviz
ros_gz_bridge
static_transform_publisher nodes
```

Gazebo world services/topics confirmed:

```text
/world/tugbot_maze_world_20260528_clean_scaled2x/control
/world/tugbot_maze_world_20260528_clean_scaled2x/scene/info
/world/tugbot_maze_world_20260528_clean_scaled2x/pose/info
/world/tugbot_maze_world_20260528_clean_scaled2x/stats
/world/tugbot_maze_world_20260528_clean_scaled2x/model/tugbot/...
```

Hash guard:

```text
5867385cd3eb72202c25e8bdd16c63591df6b947facada139d8fa8bab403d3a5  src/tugbot_gazebo/worlds/tugbot_maze_world.sdf
da33e9e58c6c07089b6bb7156315759b4bceb989db2f1ace18f0b1af0b0c0301  src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf
298eeaa8e8d030f9c008247efbb10869184e168291f572d97e96b959f2455f60  src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
b4ced2a422fd976f2a9a12455e06d6626526b1cebfc748b0c4820f5149c423dc  src/tugbot_gazebo/worlds/tugbot_maze_world_image_faithful.sdf
```

Diff guards:

```text
git diff -- src/tugbot_gazebo/worlds/tugbot_maze_world.sdf | cat
# empty

git diff -- src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf | cat
# empty

git diff -- src/tugbot_navigation/config | cat
# empty
```

## Manual observation checklist

Please inspect Gazebo/RViz and answer these items:

1. 2x scale 是否正确？
2. Tugbot 与通道宽度比例是否合理？
3. 入口箭头是否指向迷宫内部？
4. 终点带是否位于期望出口？
5. 墙体布局是否仍与 `maze_20260528.png` 一致？
6. 是否有明显断墙、错墙、重叠墙、封死入口/出口？
7. 墙高 `1.2m`、墙厚 `0.24m` 是否合理？
8. 是否可进入 SLAM/map smoke？

## Current status

```text
VISUAL_ONLY_RUN_ACTIVE_PENDING_HUMAN_ACCEPTANCE
```

The visual-only Gazebo/RViz run remains active for human observation. No final
accept/reject conclusion is made in Phase33 until the human visual inspection is
reported.
