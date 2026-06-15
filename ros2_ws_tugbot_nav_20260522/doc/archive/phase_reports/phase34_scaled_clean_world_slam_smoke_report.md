# Phase34 Scaled Clean World SLAM / Map Smoke Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `PASS_AS_SCALED_CLEAN_WORLD_SLAM_DATAFLOW_AND_MAP_VISUAL_ACCEPTED`
Phase33F accepted status: `PASS_AS_SCALED_CLEAN_WORLD_VISUALLY_ACCEPTED`

## Final human acceptance

Human Gazebo/RViz observation accepted Phase34 with conclusion:

```text
PASS_AS_SCALED_CLEAN_WORLD_SLAM_DATAFLOW_AND_MAP_VISUAL_ACCEPTED
```

Human-observed acceptance points:

- RViz `/map` is basically consistent with Gazebo wall orientation.
- LaserScan can see walls.
- TF / RobotModel near the start pose is normal.
- The local map has no obvious large-area displacement.

Important limitation:

- This phase does not declare navigation success.
- This phase did not enter Nav2/manual goal testing.
- This phase did not run autonomous exploration.

## Scope

Phase34 started a minimal Gazebo + SLAM/RViz smoke for the visually accepted scaled clean world:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

The goal was to verify live SLAM/map data flow and allow human RViz inspection of
whether `/map` reflects the Gazebo maze wall layout. This phase did not perform
autonomous exploration and does not make any navigation success claim.

## Guardrails held

- no Nav2/MPPI/controller parameter changes;
- no fallback / terminal-acceptance continuation;
- no navigation strategy changes;
- no world promotion;
- no long autonomous exploration;
- no `maze_explorer`;
- no `frontier_explorer`;
- no Nav2 navigation stack;
- no Nav2/manual goal;
- no map-server / AMCL static replay;
- no navigation success conclusion.

A bounded manual `/cmd_vel` rotate pulse was sent only after initial passive data
capture because the initial stationary scan sample was all `.inf`. This was used
to verify scan/map sensing flow near the accepted start pose, not as navigation,
not as manual goal execution, and not as autonomous exploration.

## Wrapper

Created:

```text
tools/run_phase34_scaled_clean_world_slam_smoke.sh
```

Wrapper behavior:

- explicitly launches `tugbot_maze_slam.launch.py` with the scaled world path;
- uses `slam_toolbox_params.yaml`;
- launches Gazebo visible (`headless:=false`);
- launches RViz (`use_rviz:=true`);
- does not launch `tugbot_maze_explore.launch.py`, `tugbot_maze_slam_nav.launch.py`, or Nav2 `navigation_launch.py`;
- records `/map`, `/map_metadata`, `/tf`, `/scan`, `/odom` one-shot samples;
- records `/scan` and `/map` frequency samples;
- records `slam_toolbox` lifecycle state;
- saves map snapshots via `map_saver_cli` when available;
- captures root-display XWD screenshot;
- after human acceptance, cleanup stops the visual SLAM run and records a final cleanup check.

Launch command used:

```bash
ros2 launch tugbot_bringup tugbot_maze_slam.launch.py \
  world_sdf:=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf \
  slam_params_file:=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_navigation/config/slam_toolbox_params.yaml \
  headless:=false \
  use_rviz:=true \
  use_sim_time:=true \
  autostart:=true
```

## Runtime graph during smoke

ROS nodes observed during the active smoke:

```text
/camera_link_static_tf
/camera_optical_frame_static_tf
/launch_ros_1663889
/ros_gz_bridge
/rviz2
/scan_omni_static_tf
/slam_toolbox
/transform_listener_impl_5561fbda5bc0
/transform_listener_impl_5aada0173ca0
```

Forbidden nodes absent during the active smoke:

```text
maze_explorer absent
frontier_explorer absent
controller_server absent
planner_server absent
bt_navigator absent
map_server absent
amcl absent
behavior_server absent
waypoint_follower absent
velocity_smoother absent
smoother_server absent
```

Live visual/SLAM processes during the smoke:

```text
ros2 launch tugbot_bringup tugbot_maze_slam.launch.py ... scaled2x.sdf ...
gz sim -r ... tugbot_maze_world_20260528_clean_scaled2x.sdf
gz sim server
gz sim gui
ros_gz_bridge
slam_toolbox async_slam_toolbox_node
rviz2 -d .../tugbot_nav.rviz
static_transform_publisher nodes
```

## Data-flow artifacts

Artifact directory:

```text
log/phase34_scaled_clean_world_slam_smoke/
```

Core artifacts:

```text
phase34_scaled_clean_world_slam_smoke_preflight.txt
phase34_scaled_clean_world_slam_smoke_launch.log
phase34_scaled_clean_world_slam_smoke_launch.pid
phase34_scaled_clean_world_slam_smoke_manual_checklist.txt
phase34_scaled_clean_world_slam_smoke_ros_graph_initial.txt
phase34_scaled_clean_world_slam_smoke_gz_graph_initial.txt
phase34_scaled_clean_world_slam_smoke_processes.txt
phase34_scaled_clean_world_slam_smoke_map_once.txt
phase34_scaled_clean_world_slam_smoke_map_metadata_once.txt
phase34_scaled_clean_world_slam_smoke_scan_once.txt
phase34_scaled_clean_world_slam_smoke_odom_once.txt
phase34_scaled_clean_world_slam_smoke_tf_once.txt
phase34_scaled_clean_world_slam_smoke_scan_hz.txt
phase34_scaled_clean_world_slam_smoke_map_hz.txt
phase34_scaled_clean_world_slam_smoke_slam_lifecycle.txt
phase34_scaled_clean_world_slam_smoke_map_saver.log
phase34_scaled_clean_world_slam_smoke_map_snapshot.yaml
phase34_scaled_clean_world_slam_smoke_map_snapshot.pgm
phase34_scaled_clean_world_slam_smoke_display_root.xwd
phase34_scaled_clean_world_slam_smoke_verification.txt
phase34_scaled_clean_world_slam_smoke_cleanup.txt
```

Post-pulse artifacts:

```text
phase34_scaled_clean_world_slam_smoke_cmd_vel_pulse.log
phase34_scaled_clean_world_slam_smoke_odom_before_cmd_pulse.txt
phase34_scaled_clean_world_slam_smoke_odom_after_cmd_pulse.txt
phase34_scaled_clean_world_slam_smoke_scan_after_cmd_pulse.txt
phase34_scaled_clean_world_slam_smoke_map_after_cmd_pulse.txt
phase34_scaled_clean_world_slam_smoke_map_snapshot_after_cmd_pulse.yaml
phase34_scaled_clean_world_slam_smoke_map_snapshot_after_cmd_pulse.pgm
phase34_scaled_clean_world_slam_smoke_display_root_after_cmd_pulse.xwd
phase34_scaled_clean_world_slam_smoke_post_motion_metrics.txt
```

## Evidence summary

`slam_toolbox` launched and registered the Lidar:

```text
slam_toolbox: Activating
Registering sensor: [Custom Described Lidar]
```

Bridge topics created:

```text
/clock
/cmd_vel
/odom
/scan
/tf
/camera/image_raw
/camera/camera_info
```

Map metadata sample:

```text
resolution: 0.05000000074505806
width: 62
height: 243
origin: present
```

Saved map snapshot:

```text
phase34_scaled_clean_world_slam_smoke_map_snapshot.yaml
phase34_scaled_clean_world_slam_smoke_map_snapshot.pgm
```

Map PGM:

```text
Netpbm image data, size = 62 x 243, rawbits, greymap
```

TF sample included:

```text
frame_id: map
child_frame_id: odom
```

Odom sample included:

```text
frame_id: odom
child_frame_id: base_link
position: present
orientation: present
```

Initial passive scan sample existed but was all `.inf` near the accepted start
pose. After a bounded manual rotate pulse, scan flow showed finite wall returns:

```text
scan_count 128
scan_finite_count 128
scan_min 1.3347588777542114
scan_median 2.7540814876556396
scan_under_10m 128
```

Post-pulse map sample remained bounded and available:

```text
map_data_count 128
map_known_count 128
map_occupied_count 2
map_known_ratio 1.0
```

Interpretation:

- Gazebo + bridge + slam_toolbox + RViz launched against the explicit scaled world.
- `/map`, `/tf`, `/scan`, and `/odom` samples were captured.
- LaserScan can observe finite wall returns after a minimal non-autonomous rotate pulse.
- A saved map artifact exists for human inspection.
- Human observation accepted local map visual quality: `/map` wall direction is basically consistent with Gazebo, TF/RobotModel is normal, LaserScan sees walls, and no obvious large-area local-map displacement is visible.
- The smoke does not validate navigation success and does not validate full autonomous traversal.

## Visual/RViz notes

RViz produced a GL shader warning while trying to render a map:

```text
active samplers with a different type refer to the same texture image unit
```

Human observation nevertheless accepted the Gazebo/RViz map smoke at the Phase34
scope: data flow is valid and local map visual alignment is acceptable.

## Cleanup

Cleanup artifact:

```text
log/phase34_scaled_clean_world_slam_smoke/phase34_scaled_clean_world_slam_smoke_cleanup.txt
```

Cleanup was requested after human acceptance. The original launch parent exited
first, but several child processes remained; these were explicitly terminated:

```text
gz sim / gz sim server / gz sim gui
ros_gz_bridge
static_transform_publisher nodes
slam_toolbox async_slam_toolbox_node
rviz2
```

Final cleanup check result:

```text
phase34 cleanup final check passed
```

The final cleanup section shows no remaining relevant processes and no forbidden
ROS nodes for:

```text
ros2 launch
gz sim
rviz2
slam_toolbox
maze_explorer
frontier_explorer
controller_server
planner_server
bt_navigator
ros_gz_bridge
parameter_bridge
static_transform_publisher
```

## Preservation checks

Hashes at verification:

```text
5867385cd3eb72202c25e8bdd16c63591df6b947facada139d8fa8bab403d3a5  src/tugbot_gazebo/worlds/tugbot_maze_world.sdf
da33e9e58c6c07089b6bb7156315759b4bceb989db2f1ace18f0b1af0b0c0301  src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf
d7687388a9c91d2466c52518d3b9e2b93dcac433c21c971d9f4b2101bd03d655  src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
dc5566f11e261cb4beb6ada6ee6dd45484ddd764c34c2e8c33625fdba2d92e8f  src/tugbot_navigation/config/slam_toolbox_params.yaml
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

## Final status

```text
PASS_AS_SCALED_CLEAN_WORLD_SLAM_DATAFLOW_AND_MAP_VISUAL_ACCEPTED
```

Phase34 is complete. The Gazebo/SLAM/RViz run has been stopped. No autonomous
navigation was started, no Nav2/manual goal was entered, no Nav2/MPPI/controller
parameters were modified, no fallback/terminal acceptance work continued, and no
navigation success is claimed.
