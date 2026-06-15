# Phase40 Start Pose / Entrance Frame Reconciliation Report

Generated: 2026-06-01T09:30:08+08:00

## Verdict

Conclusion: `METADATA_WORLD_FRAME_CONVENTION_CONFLICT`

Phase40 remains a bounded start-pose / entrance-frame reconciliation diagnostic. It does not claim autonomous exploration success, does not change Nav2/MPPI/controller tuning, and does not change `maze_explorer` strategy.

Required runtime acceptance criterion was not met:

- expected: robot pose to active entrance distance `< 0.75 m`
- observed: `14.237 m`

Therefore this phase stops with evidence rather than proceeding to Phase37 rerun.

Classification reasons from artifact JSON:

- active SDF Tugbot spawn equals metadata entrance
- slam_toolbox map_start_pose equals metadata entrance
- runtime robot pose remains 14.237 m from active entrance despite aligned SDF/SLAM start pose


## Guardrails preserved

- Phase37 conclusion preserved: `BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH`
- Phase39 conclusion preserved: `FRAME_ALIGNMENT_ISSUE_CONFIRMED`
- No autonomous exploration success claim.
- Bounded startup pose-only runtime check only.
- No long run.
- No `maze_explorer` launch in Phase40 runtime wrapper.
- No goal dispatch.
- No fallback / terminal acceptance continuation.
- No old scaffold world/map.
- No Nav2/MPPI/controller tuning changes.
- No `maze_explorer` strategy changes.

Preflight guardrail file confirms:

```text
# Phase40 Start Pose / Entrance Frame Reconciliation
timestamp=2026-06-01T09:24:23+08:00
world=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
maze_config=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_maze/config/maze_20260528_scaled_instance.yaml
slam_params=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_navigation/config/slam_toolbox_params.yaml
nav2_params=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_navigation/config/nav2_slam_params.yaml
run_timeout_sec=75
recorder_duration_sec=45
snapshots=15,30,45
active_truth=entrance_x=-11.011281 entrance_y=-9.025070 entrance_yaw=0.0 exit_x=10.061281 exit_y=9.058496 exit_radius=1.2
preserved_phase37_conclusion=BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH
preserved_phase39_conclusion=FRAME_ALIGNMENT_ISSUE_CONFIRMED
guardrails=pose-only bounded startup check; no maze_explorer; no goal dispatch; no autonomous success claim; no Nav2/MPPI/controller parameter edits; no maze_explorer strategy edits; no fallback/terminal acceptance continuation; no old scaffold world/map; no long run.
nav2_and_maze_strategy_diff_begin
nav2_and_maze_strategy_diff_end
```

## Files added or changed

- `tools/record_phase40_start_pose_alignment.py`
  - rclpy/tf2 recorder for `/odom` plus TF lookups `map->base_link`, `odom->base_link`, `map->odom`.
  - Offline analyzer for full JSON payloads.
  - Generates `phase40_start_pose_entrance_frame_reconciliation.json`, `phase40_start_pose_full_data.json`, and overlay PNG.
- `tools/run_phase40_start_pose_entrance_frame_reconciliation_diagnostics.sh`
  - Bounded wrapper using active scaled2x world + SLAM + Nav2 startup only.
  - Launches `tugbot_maze_slam_nav.launch.py`; intentionally does not launch `tugbot_maze_explore.launch.py`.
  - No `max_goals`, no action goal send, no `NavigateToPose` request.
- `src/tugbot_maze/test/test_phase40_start_pose_entrance_frame_reconciliation.py`
  - Contract tests for active SDF spawn, metadata entrance, launch guardrails, marker/wall preservation, Nav2/strategy guardrails, tools, and offline analyzer classification labels.
- `src/tugbot_navigation/config/slam_toolbox_params.yaml`
  - Added `map_start_pose: [-11.011281, -9.025070, 0.0]` as start-pose/frame wiring evidence.
  - This did not change Nav2/MPPI/controller behavior and did not change `maze_explorer` strategy.
- `doc/doc_report/phase40_start_pose_entrance_frame_reconciliation_report.md`
  - This report.

## Active truth and static reconciliation evidence

Active metadata:

- entrance: `x=-11.011281`, `y=-9.02507`, `yaw=0.0`
- exit: `x=10.061281`, `y=9.058496`, `radius=1.2`

Active scaled2x SDF Tugbot include:

- uri: `model://tugbot`
- name: `tugbot`
- pose: `[-11.011, -9.025, 0.0, 0.0, 0.0, 0.0]`
- SDF spawn to active entrance distance: `0.000290 m`

SLAM params evidence:

- `map_start_pose`: `[-11.011281, -9.02507, 0.0]`
- `map_start_pose` to active entrance distance: `0.000000 m`

Static inspection result:

- The active SDF Tugbot spawn is already aligned to the active metadata entrance within sub-millimeter rounding tolerance (`~0.00029 m`).
- Marker/wall geometry is preserved by contract tests.
- Exit/marker geometry was not modified.
- Launch path for bounded runtime uses active scaled2x world and active metadata; no old scaffold world/map.

## Bounded runtime evidence

Runtime command:

```bash
./tools/run_phase40_start_pose_entrance_frame_reconciliation_diagnostics.sh
```

Runtime result summary:

- artifact dir: `log/phase40_start_pose_entrance_frame_reconciliation/`
- snapshots: `3` at 15/30/45 seconds
- `map->base_link`: available in all snapshots
- `odom->base_link`: available in all snapshots
- `map->odom`: available in all snapshots
- latest `map->base_link`: `x=0.000000000011`, `y=-0.000000000000`, `yaw=0.000000000000`
- robot to active entrance distance: `14.237 m`

Timeline:

| elapsed_s | robot_x_map | robot_y_map | yaw_rad | distance_to_entrance_m | map->base_link | odom->base_link | map->odom |
|---:|---:|---:|---:|---:|---|---|---|
| 15.006 | 0.000000 | -0.000000 | 0.000000 | 14.237 | True | True | True |
| 30.006 | 0.000000 | -0.000000 | 0.000000 | 14.237 | True | True | True |
| 45.006 | 0.000000 | -0.000000 | 0.000000 | 14.237 | True | True | True |

Interpretation:

- Static world/metadata reconciliation is OK: SDF spawn and metadata entrance are aligned.
- The runtime ROS frame chain still reports the robot near map/odom origin, not near the active metadata/SDF entrance.
- `map->odom` remains effectively identity, and `odom->base_link` remains effectively origin, so `map->base_link` remains effectively origin.
- Adding `slam_toolbox` `map_start_pose` did not make online SLAM publish `map->odom` offset to the active world/metadata entrance during this bounded startup run.
- This is a frame-convention / runtime-frame reconciliation issue rather than evidence of autonomous exploration success or failure.

## Artifacts

- `log/phase40_start_pose_entrance_frame_reconciliation/phase40_start_pose_entrance_frame_reconciliation.json`
- `log/phase40_start_pose_entrance_frame_reconciliation/phase40_start_pose_full_data.json`
- `log/phase40_start_pose_entrance_frame_reconciliation/start_pose_entrance_alignment_overlay.png`
- `log/phase40_start_pose_entrance_frame_reconciliation/phase40_start_pose_entrance_frame_reconciliation_preflight.txt`
- `log/phase40_start_pose_entrance_frame_reconciliation/phase40_start_pose_entrance_frame_reconciliation_launch.log`
- `log/phase40_start_pose_entrance_frame_reconciliation/phase40_start_pose_entrance_frame_reconciliation_recorder_stdout.json`
- `log/phase40_start_pose_entrance_frame_reconciliation/phase40_start_pose_entrance_frame_reconciliation_recorder_stderr.log`
- `log/phase40_start_pose_entrance_frame_reconciliation/phase40_start_pose_entrance_frame_reconciliation_cleanup_processes_after.txt`

## Tests and verification

Executed and passed:

```text
python3 -m py_compile tools/record_phase40_start_pose_alignment.py src/tugbot_maze/test/test_phase40_start_pose_entrance_frame_reconciliation.py
python3 -m pytest -p no:cacheprovider src/tugbot_maze/test/test_phase40_start_pose_entrance_frame_reconciliation.py -q
# 7 passed in 0.04s

python3 -m pytest -p no:cacheprovider   src/tugbot_maze/test/test_phase36_autonomous_readiness.py   src/tugbot_maze/test/test_phase37_bounded_smoke_wrapper.py   src/tugbot_maze/test/test_phase38_initial_local_topology.py   src/tugbot_maze/test/test_phase39_startup_map_sufficiency.py   src/tugbot_maze/test/test_phase40_start_pose_entrance_frame_reconciliation.py -q
# 22 passed in 0.57s

. /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select tugbot_maze
colcon test --packages-select tugbot_maze --pytest-args -k phase40_start_pose_entrance_frame_reconciliation
colcon test-result --verbose
# Summary: 7 tests, 0 errors, 0 failures, 0 skipped
```

RED test evidence:

- The new Phase40 contract test was run before implementation and failed as expected because Phase40 tools/evidence were not yet present.
- After implementing recorder/wrapper/contracts and frame-wiring evidence, the focused Phase40 contract suite passed.

## Cleanup evidence

Wrapper cleanup file:

```text
(empty: no matching ROS/Gazebo/Nav2/Phase40 recorder processes after cleanup)
```

A post-run process check also only matched the `pgrep` command itself, with no actual `ros2 launch`, `gz sim`, `slam_toolbox`, Nav2, `maze_explorer`, bridge, or Phase40 recorder process left running.

## Final conclusion

`METADATA_WORLD_FRAME_CONVENTION_CONFLICT`

Phase40 confirms that the active SDF spawn and metadata entrance are aligned, and the SLAM parameter file now contains the same entrance pose as frame-wiring evidence. However, bounded runtime TF still places the robot at map/odom origin and about `14.237 m` from the active entrance. The `<0.75 m` runtime acceptance criterion is not met.

Stop here for human review. Do not proceed to Phase37 rerun until the frame-convention conflict is accepted or a Phase41 fix is explicitly requested.
