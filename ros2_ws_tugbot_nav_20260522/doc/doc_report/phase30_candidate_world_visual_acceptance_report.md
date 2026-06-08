# Phase30 Candidate Image-Faithful World Visual Acceptance Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `VISUAL_ONLY_RUN_ACTIVE_PENDING_HUMAN_ACCEPTANCE`
Phase29 accepted status: `PASS_AS_HYBRID_IMAGE_FAITHFUL_CANDIDATE_GENERATED_NOT_PROMOTED`

## Scope

Phase30 starts a visual-only Gazebo/RViz run for the Phase29 candidate world:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_image_faithful.sdf
```

The purpose is human visual acceptance of maze shape, wall coverage,
entrance/exit placement, Tugbot scale, and corridor width. This phase does not
run autonomous navigation and does not make navigation behavior conclusions.

## Guardrails held

- no autonomous navigation launched;
- no SLAM launched;
- no Nav2 exploration launched;
- no `maze_explorer` launched;
- no Nav2/MPPI/controller parameter changes;
- no fallback / terminal-acceptance continuation;
- no navigation strategy changes;
- no candidate promotion;
- no overwrite of `src/tugbot_gazebo/worlds/tugbot_maze_world.sdf`;
- no navigation conclusion from this visual-only phase.

## Wrapper

Created visual-only wrapper:

```text
tools/run_phase30_candidate_world_visual_acceptance.sh
```

Wrapper behavior:

- creates artifact directory;
- sources ROS Jazzy and workspace setup;
- verifies candidate world exists;
- refuses to use baseline scaffold path as candidate;
- starts visible Gazebo with the candidate world;
- starts RViz only for visual/TF/topic inspection;
- records ROS node/topic snapshots, Gazebo topic snapshots, process snapshot,
  launch log, hashes, and checklist;
- does not start Nav2, SLAM, or `maze_explorer`.

Launch command used by wrapper:

```bash
ros2 launch tugbot_gazebo tugbot_gazebo.launch.py \
  world_sdf:=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_gazebo/worlds/tugbot_maze_world_image_faithful.sdf \
  headless:=false \
  use_sim_time:=true
```

RViz was opened with:

```text
install/tugbot_bringup/share/tugbot_bringup/rviz/tugbot_nav.rviz
```

RViz is for visual/TF/RobotModel/topic inspection only. Do not send goals.

## Active visual run

The visual-only run is active for human inspection.

Hermes background session used to start wrapper:

```text
proc_2b0f5ea745d9
```

Wrapper exited after starting the long-lived visual processes. The active visual
launch PID is recorded here:

```text
log/phase30_candidate_world_visual_acceptance/phase30_candidate_world_visual_acceptance_launch.pid
```

At verification time the launch PID was:

```text
129939
```

## Candidate world and markers

Candidate world:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_image_faithful.sdf
```

World name observed in Gazebo services/topics:

```text
tugbot_maze_world_image_faithful
```

Candidate entrance from Phase29:

```text
(-4.0, -3.0, yaw=0)
```

The candidate world includes Tugbot at this entrance pose.

Exit marker from Phase29:

```text
(4.0, 3.0), radius=0.6
```

The candidate world includes `maze_exit_marker` as a green disk-style visual.

Candidate wall count from Phase29:

```text
63 total maze wall models
59 inner wall models
4 outer wall models
```

## Logs and artifacts

Artifact directory:

```text
log/phase30_candidate_world_visual_acceptance/
```

Files:

```text
phase30_candidate_world_visual_acceptance_launch.log
phase30_candidate_world_visual_acceptance_launch.pid
phase30_candidate_world_visual_acceptance_rviz.log
phase30_candidate_world_visual_acceptance_rviz.pid
phase30_candidate_world_visual_acceptance_manual_checklist.txt
phase30_candidate_world_visual_acceptance_ros_nodes.txt
phase30_candidate_world_visual_acceptance_ros_topics.txt
phase30_candidate_world_visual_acceptance_gz_topics.txt
phase30_candidate_world_visual_acceptance_processes.txt
phase30_candidate_world_visual_acceptance_verification.txt
phase30_display_root.xwd
```

Screenshot note:

- `phase30_display_root.xwd` was captured as an X11 root-window screenshot.
- PNG conversion was not available from the current shell tools, so the XWD file
  is retained as the static screenshot artifact.

## Verification snapshot

ROS nodes observed:

```text
/camera_link_static_tf
/camera_optical_frame_static_tf
/ros_gz_bridge
/rviz
/scan_omni_static_tf
/transform_listener_impl_5e4e0033fac0
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
ros2 launch tugbot_gazebo tugbot_gazebo.launch.py ... tugbot_maze_world_image_faithful.sdf headless:=false
ruby ... gz sim -r ... tugbot_maze_world_image_faithful.sdf
gz sim -r ... tugbot_maze_world_image_faithful.sdf
gz sim server
gz sim gui
rviz2 -d .../tugbot_nav.rviz
ros_gz_bridge
static_transform_publisher nodes
```

Gazebo world services/topics confirmed:

```text
/world/tugbot_maze_world_image_faithful/control
/world/tugbot_maze_world_image_faithful/scene/info
/world/tugbot_maze_world_image_faithful/pose/info
/world/tugbot_maze_world_image_faithful/stats
/world/tugbot_maze_world_image_faithful/model/tugbot/...
```

Hash guard:

```text
5867385cd3eb72202c25e8bdd16c63591df6b947facada139d8fa8bab403d3a5  src/tugbot_gazebo/worlds/tugbot_maze_world.sdf
b4ced2a422fd976f2a9a12455e06d6626526b1cebfc748b0c4820f5149c423dc  src/tugbot_gazebo/worlds/tugbot_maze_world_image_faithful.sdf
```

Diff guards:

```text
git diff -- src/tugbot_gazebo/worlds/tugbot_maze_world.sdf | cat
# empty

git diff -- src/tugbot_navigation/config | cat
# empty
```

## Manual observation checklist

Please inspect Gazebo/RViz and answer these items:

1. 迷宫整体是否接近原图？
2. 入口是否位于期望 lower-left / `(-4.0, -3.0)` 附近？
3. 出口 marker 是否位于期望 upper-right / `(4.0, 3.0)` 附近？
4. 是否有明显断墙、漏墙、重叠墙？
5. Tugbot 是否比通道宽，是否明显无法通过？
6. 墙高、墙厚、整体尺度是否合理？
7. 是否存在明显封死入口或出口的问题？
8. 是否需要 Phase31 继续做 candidate world geometry edits，还是可接受进入后续 controlled map/SLAM validation？

## Current status

```text
VISUAL_ONLY_RUN_ACTIVE_PENDING_HUMAN_ACCEPTANCE
```

The visual-only Gazebo/RViz run remains active for human observation. No final
accept/reject conclusion is made in Phase30 until the human visual inspection is
reported.
