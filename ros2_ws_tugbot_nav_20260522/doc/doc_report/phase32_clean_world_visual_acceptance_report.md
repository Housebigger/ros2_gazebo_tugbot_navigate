# Phase32 Gazebo Visual Acceptance of Clean 2D Candidate World Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `VISUAL_ONLY_RUN_ACTIVE_PENDING_HUMAN_ACCEPTANCE`
Phase31 accepted status: `PASS_AS_CLEAN_2D_IMAGE_WORLD_CANDIDATE_GENERATED_NOT_PROMOTED`

## Scope

Phase32 starts a visual-only Gazebo/RViz run for the clean 2D candidate world:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf
```

The purpose is human visual acceptance of whether the 3D world matches the clean
source image:

```text
src/tugbot_maze/assets/maze_20260528.png
```

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
- no promotion of `tugbot_maze_world_20260528_clean.sdf` as default;
- no navigation conclusion from this visual-only phase.

## Wrapper

Created visual-only wrapper:

```text
tools/run_phase32_clean_world_visual_acceptance.sh
```

Wrapper behavior:

- creates `log/phase32_clean_world_visual_acceptance/`;
- sources ROS Jazzy and workspace setup;
- verifies the clean candidate SDF exists;
- refuses to launch if the candidate path equals the scaffold world path;
- records SHA256 of scaffold, clean candidate, and Phase29 candidate worlds;
- starts visible Gazebo with the clean candidate world;
- starts RViz only for visual/TF/topic inspection;
- records ROS nodes/topics, Gazebo topics/services, processes, launch log,
  screenshot, and checklist;
- does not start Nav2, SLAM, or `maze_explorer`.

Launch command used by wrapper:

```bash
ros2 launch tugbot_gazebo tugbot_gazebo.launch.py \
  world_sdf:=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf \
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
proc_c34f8594e776
```

Wrapper exited after starting the long-lived visual processes. The active visual
launch PID is recorded here:

```text
log/phase32_clean_world_visual_acceptance/phase32_clean_world_visual_acceptance_launch.pid
```

At verification time the launch PID was:

```text
250019
```

## Candidate world and markers

Candidate world:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf
```

World name observed in Gazebo services/topics:

```text
tugbot_maze_world_20260528_clean
```

Candidate entrance from Phase31:

```text
(-6.0, -1.0, yaw=0)
```

The clean candidate world includes Tugbot at this entrance pose:

```text
tugbot_pose -6.000 -1.000 0 0 0 0.000
```

Exit marker from Phase31:

```text
(0.0, 6.0), radius=0.6
```

The clean candidate world includes `maze_exit_marker`.

Clean candidate wall count from Phase31:

```text
53 total maze wall models
49 inner wall models
4 outer wall models
```

## Logs and artifacts

Artifact directory:

```text
log/phase32_clean_world_visual_acceptance/
```

Files:

```text
phase32_clean_world_visual_acceptance_launch.log
phase32_clean_world_visual_acceptance_launch.pid
phase32_clean_world_visual_acceptance_rviz.log
phase32_clean_world_visual_acceptance_rviz.pid
phase32_clean_world_visual_acceptance_manual_checklist.txt
phase32_clean_world_visual_acceptance_ros_nodes.txt
phase32_clean_world_visual_acceptance_ros_topics.txt
phase32_clean_world_visual_acceptance_gz_topics.txt
phase32_clean_world_visual_acceptance_gz_services.txt
phase32_clean_world_visual_acceptance_processes.txt
phase32_clean_world_visual_acceptance_verification.txt
phase32_display_root.xwd
```

Screenshot note:

- `phase32_display_root.xwd` was captured as an X11 root-window screenshot.
- PNG conversion was not required for this phase; the XWD artifact is retained.

## Verification snapshot

ROS nodes observed:

```text
/camera_link_static_tf
/camera_optical_frame_static_tf
/ros_gz_bridge
/rviz
/scan_omni_static_tf
/transform_listener_impl_5d1f25397780
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
ros2 launch tugbot_gazebo tugbot_gazebo.launch.py ... tugbot_maze_world_20260528_clean.sdf headless:=false
ruby ... gz sim -r ... tugbot_maze_world_20260528_clean.sdf
gz sim -r ... tugbot_maze_world_20260528_clean.sdf
gz sim server
gz sim gui
rviz2 -d .../tugbot_nav.rviz
ros_gz_bridge
static_transform_publisher nodes
```

Gazebo world services/topics confirmed:

```text
/world/tugbot_maze_world_20260528_clean/control
/world/tugbot_maze_world_20260528_clean/scene/info
/world/tugbot_maze_world_20260528_clean/pose/info
/world/tugbot_maze_world_20260528_clean/stats
/world/tugbot_maze_world_20260528_clean/model/tugbot/...
```

Hash guard:

```text
5867385cd3eb72202c25e8bdd16c63591df6b947facada139d8fa8bab403d3a5  src/tugbot_gazebo/worlds/tugbot_maze_world.sdf
da33e9e58c6c07089b6bb7156315759b4bceb989db2f1ace18f0b1af0b0c0301  src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf
b4ced2a422fd976f2a9a12455e06d6626526b1cebfc748b0c4820f5149c423dc  src/tugbot_gazebo/worlds/tugbot_maze_world_image_faithful.sdf
```

Diff guards:

```text
git diff -- src/tugbot_gazebo/worlds/tugbot_maze_world.sdf | cat
# empty

git diff -- src/tugbot_navigation/config | cat
# empty

git diff -- src/tugbot_gazebo/worlds/tugbot_maze_world_image_faithful.sdf | cat
# empty
```

## Manual observation checklist

Please inspect Gazebo/RViz and answer these items:

1. 迷宫整体是否贴合 `maze_20260528.png`？
2. 是否有明显漏墙、错墙、重叠墙？
3. 入口 `(-6.0, -1.0, yaw=0)` 是否符合人的预期？
4. 出口 marker `(0.0, 6.0), radius=0.6` 是否符合人的预期？
5. Tugbot 是否能放进入口？
6. 通道宽度是否足够 Tugbot 通过？
7. 墙高、墙厚、尺度是否合理？
8. 是否需要修改 `src/tugbot_maze/config/maze_wall_segments_20260528.yaml`？
9. 是否可进入下一阶段 clean world map/SLAM smoke？

## Current status

```text
VISUAL_ONLY_RUN_ACTIVE_PENDING_HUMAN_ACCEPTANCE
```

The visual-only Gazebo/RViz run remains active for human observation. No final
accept/reject conclusion is made in Phase32 until the human visual inspection is
reported.
