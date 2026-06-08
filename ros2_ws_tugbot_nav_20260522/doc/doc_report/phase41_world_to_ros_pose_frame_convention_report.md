# Phase41 World-to-ROS Pose Frame Convention Fix Report

Generated: 2026-06-01T09:47:52+08:00

## Verdict

Conclusion: `MAP_FRAME_TRUTH_ALIGNED_READY_FOR_PHASE37_RERUN`

Phase41 fixed the coordinate convention conflict by selecting Convention A: ROS `map` frame follows the SLAM startup pose, so the runtime maze truth is expressed in `map_frame_truth` while Gazebo/SDF spawn and visual markers remain in `world_frame_truth`.

This phase does not claim autonomous exploration success and did not enter Phase37 rerun.

Classification reasons:

- map_frame_truth exit equals world exit minus world entrance
- map_frame_truth entrance is at ROS map origin
- runtime map->base_link is 0.000 m from map_frame entrance
- exit_map distance is reasonable at 27.768 m

## Guardrails preserved

- Phase37 conclusion preserved: `BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH`.
- Phase40 conclusion preserved: `METADATA_WORLD_FRAME_CONVENTION_CONFLICT`.
- No autonomous exploration success claim.
- No Phase37 rerun.
- Bounded startup pose-only runtime check only.
- No `maze_explorer` launch in Phase41 runtime wrapper.
- No goal dispatch.
- No fallback / terminal acceptance continuation.
- No old scaffold world/map.
- No Nav2/MPPI/controller tuning changes.
- No `maze_explorer` strategy changes.

Preflight guardrail evidence:

```text
# Phase41 World-to-ROS Pose Frame Convention Fix
timestamp=2026-06-01T09:46:19+08:00
world=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
maze_config=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_maze/config/maze_20260528_scaled_instance.yaml
slam_params=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_navigation/config/slam_toolbox_params.yaml
nav2_params=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_navigation/config/nav2_slam_params.yaml
run_timeout_sec=75
recorder_duration_sec=45
snapshots=15,30,45
selected_convention=A_map_frame_follows_slam_startup_pose
world_frame_truth=entrance_x=-11.011281 entrance_y=-9.025070 exit_x=10.061281 exit_y=9.058496
map_frame_truth=entrance_x=0.0 entrance_y=0.0 exit_x=21.072562 exit_y=18.083566 exit_radius=1.2
preserved_phase37_conclusion=BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH
preserved_phase40_conclusion=METADATA_WORLD_FRAME_CONVENTION_CONFLICT
guardrails=pose-only bounded startup check; no maze_explorer; no goal dispatch; no autonomous success claim; no Phase37 rerun; no Nav2/MPPI/controller parameter edits; no maze_explorer strategy edits; no fallback/terminal acceptance continuation; no old scaffold world/map; no long run.
nav2_and_maze_strategy_diff_begin
nav2_and_maze_strategy_diff_end
```

## Coordinate convention audit

Phase40 evidence showed the key conflict:

- Active SDF Tugbot spawn and active metadata entrance were aligned in Gazebo world coordinates.
- Runtime ROS TF `map->base_link` still started near `(0, 0)`.
- `slam_toolbox map_start_pose` did not shift runtime `map->odom` into Gazebo world coordinates.

Phase41 therefore adopted Convention A:

- `world_frame_truth` keeps accepted Gazebo/SDF coordinates:
  - entrance = `(-11.011281, -9.025070, yaw=0.0)`
  - exit = `(10.061281, 9.058496, radius=1.2)`
- `map_frame_truth` is derived by subtracting the world entrance offset:
  - entrance_map = `(0.0, 0.0, yaw=0.0)`
  - exit_map = `(21.072562, 18.083566, radius=1.2)`
- SDF spawn and markers remain unchanged in Gazebo world coordinates.
- `tugbot_maze_explore.launch.py` passes map-frame truth to `maze_explorer` and `maze_goal_monitor`.
- `slam_toolbox_params.yaml` no longer attempts `map_start_pose` world-coordinate forcing.

## Files added or changed

- `src/tugbot_maze/config/maze_20260528_scaled_instance.yaml`
  - Added `coordinate_convention`, `world_frame_truth`, and `map_frame_truth`.
  - Preserved legacy top-level `entrance` / `exit` as world-frame coordinates for marker/SDF compatibility.
- `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`
  - Default `entrance_x/y/yaw` and `exit_x/y/radius` now use map-frame truth.
- `src/tugbot_navigation/config/slam_toolbox_params.yaml`
  - Removed Phase40 `map_start_pose` probe because Convention A does not force SLAM map into Gazebo world coordinates.
- `tools/record_phase41_world_to_ros_pose_frame_convention.py`
  - Added bounded TF/odom recorder and offline analyzer for Phase41.
- `tools/run_phase41_world_to_ros_pose_frame_convention_diagnostics.sh`
  - Added bounded runtime wrapper using active world + SLAM + Nav2 only; no explorer and no goals.
- `tools/check_phase36_autonomous_readiness.py`
  - Updated readiness check to use `map_frame_truth` when present.
- `tools/run_phase37_scaled_clean_world_maze_explorer_bounded_smoke.sh`
  - Updated future Phase37 wrapper truth arguments to map-frame convention only. This phase did not execute Phase37.
- Tests updated/added:
  - `src/tugbot_maze/test/test_phase41_world_to_ros_pose_frame_convention.py`
  - `src/tugbot_maze/test/test_phase36_autonomous_readiness.py`
  - `src/tugbot_maze/test/test_phase37_bounded_smoke_wrapper.py`
  - `src/tugbot_maze/test/test_phase40_start_pose_entrance_frame_reconciliation.py`

## Bounded runtime evidence

Runtime command:

```bash
./tools/run_phase41_world_to_ros_pose_frame_convention_diagnostics.sh
```

Runtime summary:

- artifact dir: `log/phase41_world_to_ros_pose_frame_convention/`
- snapshots: `3` at 15/30/45 seconds
- `map->base_link`: available in all snapshots
- `odom->base_link`: available in all snapshots
- `map->odom`: available in all snapshots
- latest `map->base_link`:
  - x = `1.0685934496749354e-11`
  - y = `-1.0400802301430532e-24`
  - yaw = `2.7401554489864275e-13`
- robot to map-frame entrance distance = `1.0685934496749354e-11` m
- acceptance threshold = `0.75` m
- exit_map distance = `27.768115321717463` m

Runtime acceptance is satisfied because robot pose is effectively at map-frame entrance `(0,0)` and exit_map distance is reasonable.

## Artifacts

- `log/phase41_world_to_ros_pose_frame_convention/phase41_world_to_ros_pose_frame_convention.json`
- `log/phase41_world_to_ros_pose_frame_convention/phase41_world_to_ros_pose_frame_convention_full_data.json`
- `log/phase41_world_to_ros_pose_frame_convention/world_to_ros_map_frame_truth_overlay.png`
- `log/phase41_world_to_ros_pose_frame_convention/phase41_world_to_ros_pose_frame_convention_preflight.txt`
- `log/phase41_world_to_ros_pose_frame_convention/phase41_world_to_ros_pose_frame_convention_launch.log`
- `log/phase41_world_to_ros_pose_frame_convention/phase41_world_to_ros_pose_frame_convention_cleanup_processes_after.txt`

## Verification

Focused py_compile + pytest:

```text
python3 -m py_compile tools/record_phase41_world_to_ros_pose_frame_convention.py tools/check_phase36_autonomous_readiness.py src/tugbot_maze/test/test_phase36_autonomous_readiness.py src/tugbot_maze/test/test_phase37_bounded_smoke_wrapper.py src/tugbot_maze/test/test_phase40_start_pose_entrance_frame_reconciliation.py src/tugbot_maze/test/test_phase41_world_to_ros_pose_frame_convention.py
python3 -m pytest -p no:cacheprovider src/tugbot_maze/test/test_phase36_autonomous_readiness.py src/tugbot_maze/test/test_phase37_bounded_smoke_wrapper.py src/tugbot_maze/test/test_phase38_initial_local_topology.py src/tugbot_maze/test/test_phase39_startup_map_sufficiency.py src/tugbot_maze/test/test_phase40_start_pose_entrance_frame_reconciliation.py src/tugbot_maze/test/test_phase41_world_to_ros_pose_frame_convention.py -q

30 passed in 0.60s
```

Colcon focused verification:

```text
colcon build --symlink-install --packages-select tugbot_maze
colcon test --packages-select tugbot_maze --pytest-args -k 'phase41_world_to_ros_pose_frame_convention or phase36_autonomous_readiness or phase37_bounded_smoke_wrapper or phase40_start_pose_entrance_frame_reconciliation'
colcon test-result --verbose

Summary: 24 tests, 0 errors, 0 failures, 0 skipped
```

## Cleanup

Wrapper cleanup artifact after runtime:

```text
(empty: no matching ROS/Gazebo/Nav2/Phase41 recorder processes remained)
```

Additional live process check after runtime only matched the transient `pgrep` command itself; no persistent ROS/Gazebo/Nav2/recorder process remained.

## Stop condition

Phase41 stops here for human acceptance. Do not treat this as autonomous exploration success. Do not enter Phase37 rerun unless explicitly approved in a later phase.
