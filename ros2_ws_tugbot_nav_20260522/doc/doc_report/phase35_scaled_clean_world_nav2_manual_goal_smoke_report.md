# Phase35 Scaled Clean World Nav2 Manual Goal Smoke Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `PASS_AS_SCALED_CLEAN_WORLD_NAV2_MANUAL_GOAL_SMOKE_ACCEPTED`
Phase35-pre accepted status: `PASS_AS_ACTIVE_CLEAN_WORLD_SELECTED_AND_OLD_ARTIFACTS_ARCHIVED`

## Final human acceptance

Human RViz/Gazebo observation accepted Phase35 with conclusion:

```text
PASS_AS_SCALED_CLEAN_WORLD_NAV2_MANUAL_GOAL_SMOKE_ACCEPTED
```

Human-observed acceptance points:

- User used RViz manual `Nav2 Goal` to navigate Tugbot continuously to the area
  in front of the exit.
- `/map` is basically consistent with the Gazebo maze layout.
- `/plan` can be generated.
- Tugbot can pass representative corridors and turns.
- Local costmap / footprint / LaserScan behavior is basically acceptable.

Important limitations:

- This phase does not declare autonomous exploration success.
- This phase does not claim autonomous exit-reaching success.
- This phase did not start `maze_explorer`.
- This phase did not run autonomous exploration.
- This phase did not modify Nav2/MPPI/controller parameters.
- This phase did not continue fallback / terminal-acceptance work.

## Scope

Phase35 started the current active scaled clean maze world with live SLAM, Nav2,
and RViz for human RViz manual-goal smoke testing:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

The purpose was to let a human set a small number of RViz `Nav2 Goal` poses and
observe:

- global planning;
- local obstacle avoidance / local costmap behavior;
- footprint/costmap alignment near walls;
- representative corridor and opening passability.

This phase intentionally did not start `maze_explorer`, did not run autonomous
exploration, and does not claim autonomous-navigation success.

## Guardrails held

- no `maze_explorer` launched;
- no `tugbot_maze_explore.launch.py` launched;
- no autonomous exploration;
- no Nav2/MPPI/controller parameter tuning;
- no fallback / terminal-acceptance continuation;
- no navigation strategy changes;
- no old scaffold world/map;
- no static-map replay / AMCL;
- no autonomous navigation-success claim;
- active world remained `tugbot_maze_world_20260528_clean_scaled2x.sdf`.

## Phase35-pre reclosure

Before launching Phase35, Phase35-pre was rechecked and accepted:

```text
PASS_AS_ACTIVE_CLEAN_WORLD_SELECTED_AND_OLD_ARTIFACTS_ARCHIVED
```

Reclosure artifact:

```text
log/phase35_pre_map_world_artifact_cleanup/phase35_pre_reclosure_checks.txt
```

Verified:

- active world/source/config/map snapshot artifacts exist;
- current maze SLAM / SLAM+Nav2 / explore launch defaults point to the active
  scaled2x world;
- current maze launch defaults do not silently reference the old scaffold world;
- `git diff -- src/tugbot_navigation/config` is empty;
- launch syntax checks pass;
- cleanup process check was empty before Phase35.

Updated status docs:

```text
doc/ACTIVE_MAZE_WORLD.md
doc/doc_report/phase35_pre_map_world_artifact_cleanup_report.md
```

## Wrapper

Created:

```text
tools/run_phase35_scaled_clean_world_nav2_manual_goal_smoke.sh
```

Launch command used by the wrapper:

```bash
ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py \
  world_sdf:=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf \
  slam_params_file:=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_navigation/config/slam_toolbox_params.yaml \
  params_file:=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_navigation/config/nav2_slam_params.yaml \
  headless:=false \
  use_rviz:=true \
  use_sim_time:=true \
  autostart:=true
```

The wrapper recorded preflight guards, ROS/Gazebo graph snapshots, one-shot data
samples for map/scan/odom/TF/costmaps, lifecycle state, and an XWD root-display
screenshot while leaving the visual Nav2 run active for human observation.

## Manual-goal run

The Phase35 SLAM+Nav2+RViz run was started for human manual-goal testing.

Launch PID:

```text
1879871
```

Artifact directory:

```text
log/phase35_scaled_clean_world_nav2_manual_goal_smoke/
```

Key artifacts:

```text
phase35_scaled_clean_world_nav2_manual_goal_smoke_preflight.txt
phase35_scaled_clean_world_nav2_manual_goal_smoke_launch.log
phase35_scaled_clean_world_nav2_manual_goal_smoke_launch.pid
phase35_scaled_clean_world_nav2_manual_goal_smoke_manual_checklist.txt
phase35_scaled_clean_world_nav2_manual_goal_smoke_ros_graph_initial.txt
phase35_scaled_clean_world_nav2_manual_goal_smoke_gz_graph_initial.txt
phase35_scaled_clean_world_nav2_manual_goal_smoke_map_once.txt
phase35_scaled_clean_world_nav2_manual_goal_smoke_map_metadata_once.txt
phase35_scaled_clean_world_nav2_manual_goal_smoke_scan_once.txt
phase35_scaled_clean_world_nav2_manual_goal_smoke_odom_once.txt
phase35_scaled_clean_world_nav2_manual_goal_smoke_tf_once.txt
phase35_scaled_clean_world_nav2_manual_goal_smoke_local_costmap_once.txt
phase35_scaled_clean_world_nav2_manual_goal_smoke_global_costmap_once.txt
phase35_scaled_clean_world_nav2_manual_goal_smoke_hidden_action_topics.txt
phase35_scaled_clean_world_nav2_manual_goal_smoke_initial_verification.txt
phase35_scaled_clean_world_nav2_manual_goal_smoke_processes_current_filtered.txt
phase35_scaled_clean_world_nav2_manual_goal_smoke_display_root_initial.xwd
phase35_scaled_clean_world_nav2_manual_goal_smoke_cleanup.txt
```

## Initial runtime verification

Initial verification result:

```text
RUN_ACTIVE_READY_FOR_HUMAN_RVIZ_NAV2_GOALS
```

Required nodes observed:

```text
/slam_toolbox: present
/controller_server: present
/planner_server: present
/bt_navigator: present
/behavior_server: present
/waypoint_follower: present
/velocity_smoother: present
/smoother_server: present
/ros_gz_bridge: present
/rviz2: present
```

Forbidden explorer nodes were absent:

```text
maze_explorer: absent
frontier_explorer: absent
```

Expected live data topics were present:

```text
/map
/scan
/odom
/local_costmap/costmap
/global_costmap/costmap
/plan
```

`/navigate_to_pose` action was available:

```text
Action: /navigate_to_pose
Action clients: 3
Action servers: 1
    /bt_navigator
```

Hidden action topics were also confirmed with `ros2 topic list --include-hidden-topics`:

```text
/navigate_to_pose/_action/feedback
/navigate_to_pose/_action/status
```

Lifecycle state checks:

```text
/slam_toolbox: active [3]
/controller_server: active [3]
/planner_server: active [3]
/bt_navigator: active [3]
/behavior_server: active [3]
/waypoint_follower: active [3]
/velocity_smoother: active [3]
/smoother_server: active [3]
```

Costmap samples were received:

```text
/local_costmap/costmap: nav_msgs/msg/OccupancyGrid, publisher count 1
/global_costmap/costmap: nav_msgs/msg/OccupancyGrid, publisher count 1
```

## Preflight preservation evidence

Active world and config hashes recorded before launch:

```text
d7687388a9c91d2466c52518d3b9e2b93dcac433c21c971d9f4b2101bd03d655  src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
dc5566f11e261cb4beb6ada6ee6dd45484ddd764c34c2e8c33625fdba2d92e8f  src/tugbot_navigation/config/slam_toolbox_params.yaml
e8c2e68690fd359ddad20237523cea8c68f725f8319de0d33381ee4010106d93  src/tugbot_navigation/config/nav2_slam_params.yaml
```

Archived deprecated world hashes were recorded for provenance:

```text
5867385cd3eb72202c25e8bdd16c63591df6b947facada139d8fa8bab403d3a5  archive/deprecated_maze_worlds/worlds/manual_simplified_first_pass_scaffold/tugbot_maze_world.sdf
da33e9e58c6c07089b6bb7156315759b4bceb989db2f1ace18f0b1af0b0c0301  archive/deprecated_maze_worlds/worlds/unscaled_clean_candidate/tugbot_maze_world_20260528_clean.sdf
```

`git diff -- src/tugbot_navigation/config` was empty in preflight. Current maze
launch default grep showed only:

```text
tugbot_maze_world_20260528_clean_scaled2x.sdf
```

## Launch-log notes

Notable non-fatal log lines are archived in:

```text
phase35_scaled_clean_world_nav2_manual_goal_smoke_launch_log_notable_lines.txt
```

Observed categories:

- Gazebo warnings for custom visual marker metadata attributes and marker
  transparency SDF extension fields inherited from visual-marker phases;
- EGL/GL display warnings from Gazebo/RViz rendering;
- Nav2 startup warning: global costmap inflation radius `0.350` is slightly
  smaller than computed footprint inscribed radius `0.353`;
- early startup TF waiting messages before odom/base_link became available;
- RViz GL shader warning similar to Phase34.

No Nav2/MPPI/controller parameter changes were made in response to these notes.
They are recorded for human smoke context only.

## Cleanup

After human acceptance, the Phase35 Gazebo/SLAM/Nav2/RViz run was stopped.

Cleanup artifact:

```text
log/phase35_scaled_clean_world_nav2_manual_goal_smoke/phase35_scaled_clean_world_nav2_manual_goal_smoke_cleanup.txt
```

Cleanup sequence:

- sent SIGTERM to the launch parent and relevant Gazebo / bridge / static TF /
  SLAM / Nav2 / RViz child processes;
- remaining `gz sim` processes were explicitly killed;
- ROS daemon was refreshed after stale Nav2 nodes were still visible in the graph;
- final process and ROS-node cleanup checks were empty.

Final cleanup result:

```text
phase35 cleanup final check passed
```

Final relevant process check:

```text
(empty)
```

Final ROS relevant node check after daemon refresh:

```text
(empty)
```

## Result

```text
PASS_AS_SCALED_CLEAN_WORLD_NAV2_MANUAL_GOAL_SMOKE_ACCEPTED
```

Phase35 is complete. The active scaled clean world supports RViz manual Nav2 Goal
smoke at this phase's scope: manual goals could generate plans and drive Tugbot
through representative corridors/turns to the area in front of the exit, with
acceptable observed map/costmap/footprint/LaserScan behavior. The run has been
stopped and cleanup passed.

No autonomous exploration was started, no `maze_explorer` was launched, no
Nav2/MPPI/controller parameters were modified, no fallback/terminal acceptance
work continued, and no autonomous exploration success is claimed.
