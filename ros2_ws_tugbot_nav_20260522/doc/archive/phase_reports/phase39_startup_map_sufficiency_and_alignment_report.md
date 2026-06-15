# Phase39 Startup Map Sufficiency and Entrance Frame Alignment Evidence

## Scope

Phase39 is a bounded startup diagnostics phase. It hardens evidence around the Phase38 finding `INSUFFICIENT_MAP_AT_START` by recording complete startup map/costmap/scan/TF/explorer evidence before and around the first maze_explorer topology decision.

This phase intentionally does not tune Nav2, MPPI, controller, progress checker, or maze_explorer strategy. It also does not continue fallback or terminal-acceptance work and does not claim autonomous exploration success.

## Guardrails

Runtime guardrails recorded in `phase39_startup_map_sufficiency.json`:

- bounded_startup_diagnostics_only: true
- nav2_mppi_controller_params_modified: false
- maze_explorer_strategy_modified: false
- fallback_or_terminal_acceptance_continued: false
- old_scaffold_world_used: false
- long_run_performed: false
- relied_on_ros2_topic_echo_truncated_map: false
- autonomous_success_claimed: false
- Phase37 semantics preserved: `BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH`
- Phase38 conclusion preserved: `INSUFFICIENT_MAP_AT_START`

Wrapper bounds:

- `PHASE39_RUN_TIMEOUT_SEC` defaults to 150 and is guardrailed to 120..180 seconds.
- `PHASE39_RECORDER_DURATION_SEC` defaults to 95 seconds.
- Fixed evidence targets: 30, 60, and 90 seconds.
- `PHASE39_MAX_GOALS` defaults to 1 so the first topology analysis can occur, but the wrapper refuses values above 1.
- `near_exit_fallback_enabled:=false` in the Phase39 wrapper.

## Files added

- `tools/record_phase39_startup_map_sufficiency.py`
  - rclpy recorder for full `/map`, `/local_costmap/costmap`, `/global_costmap/costmap`, `/scan`, `/tf` lookups, `/maze/explorer_state`, and `/maze/goal_events` evidence.
  - Captures immediate snapshots at first explorer_state and first `FAILED_EXHAUSTED`, plus scheduled 30/60/90s snapshots.
  - Generates summary JSON, full-data JSON, and overlay PNG files.
- `tools/run_phase39_startup_map_sufficiency_diagnostics.sh`
  - Bounded startup diagnostics wrapper for the active scaled2x world + SLAM + Nav2 + maze_explorer.
  - Performs preflight, launch, recorder, timeout, summary printing, and cleanup.
- `src/tugbot_maze/test/test_phase39_startup_map_sufficiency.py`
  - Contract tests for recorder, wrapper guardrails, JSON schema, allowed conclusions, and overlay artifacts.

## Active world and metadata

- Active world: `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf`
- Active metadata: `src/tugbot_maze/config/maze_20260528_scaled_instance.yaml`
- Active entrance: x=-11.011281, y=-9.025070, yaw=0.000
- Active exit: x=10.061281, y=9.058496, radius=1.200

## Runtime command

Executed from workspace root:

```bash
./tools/run_phase39_startup_map_sufficiency_diagnostics.sh
```

The final wrapper run exited 0 and printed:

```json
{
  "artifact_dir": "log/phase39_startup_map_sufficiency",
  "autonomous_success_claimed": false,
  "conclusion": "FRAME_ALIGNMENT_ISSUE_CONFIRMED",
  "failed_exhausted_before_map_sufficient": true,
  "run_id": "phase39_startup_map_sufficiency",
  "snapshot_count": 5
}
```

A cleanup-stage warning occurred once:

```text
pkill: killing pid 82100 failed: Operation not permitted
```

Follow-up cleanup verification showed no actual ROS/Gazebo/Nav2/recorder process leftovers; the only `pgrep` hit was the `pgrep` shell command itself. The recorded cleanup-after file is empty: `log/phase39_startup_map_sufficiency/phase39_startup_map_sufficiency_cleanup_processes_after.txt`.

## Artifacts

Generated under `log/phase39_startup_map_sufficiency/`:

- `phase39_startup_map_sufficiency.json`
- `phase39_startup_full_data.json`
- `robot_vs_entrance_frame_alignment_overlay.png`
- `local_map_known_free_overlay.png`
- `topology_sampling_after_full_map_overlay.png`
- `phase39_startup_map_sufficiency_preflight.txt`
- `phase39_startup_map_sufficiency_launch.log`
- `phase39_startup_map_sufficiency_recorder_stderr.log`
- `phase39_startup_map_sufficiency_recorder_stdout.json`
- `phase39_startup_map_sufficiency_cleanup_processes_after.txt`

Overlay sizes from the final artifact set:

- robot_vs_entrance_frame_alignment_overlay.png: 6524 bytes
- local_map_known_free_overlay.png: 2083 bytes
- topology_sampling_after_full_map_overlay.png: 6451 bytes

## Snapshot coverage

The recorder captured five snapshots:

- first explorer_state immediate snapshot: 14.574s
- first FAILED_EXHAUSTED immediate snapshot: 14.575s
- scheduled startup evidence: 30.006s
- scheduled startup evidence: 60.006s
- scheduled startup evidence: 90.006s

Recorder stderr confirms all scheduled captures:

```text
captured Phase39 snapshot target=30.0 actual=30.006s
captured Phase39 snapshot target=60.0 actual=60.006s
captured Phase39 snapshot target=90.0 actual=90.006s
```

## Frame alignment evidence

TF lookups were available, but they place the robot near the map/odom origin instead of the configured active entrance.

Latest robot pose in map:

- source: tf2 map->base_link
- x=1.06859454167e-11
- y=-1.04007723772e-24
- yaw=2.74053630454e-13

TF samples:

- map->base_link available: true, translation x=1.06859454167e-11, y=-1.04007723772e-24, z=0.000
- map->odom available: true, translation x=0, y=1.83670992316e-40, z=0.000
- odom->base_link available: true, translation x=1.06859454167e-11, y=-1.04007723772e-24, z=0.000

Active entrance from metadata:

- x=-11.011281
- y=-9.025070

Frame-alignment distance:

- robot pose to active entrance: 14.237 m
- alignment_ok_threshold_m: 0.75
- frame_issue_threshold_m: 2.00

This exceeds the frame-issue threshold and is the decisive Phase39 classifier reason: `robot pose is 14.237 m from active entrance`.

## Map/costmap/scan evidence

Full grid data was recorded and serialized; the map/costmap evidence below is derived from full arrays, not from truncated `ros2 topic echo` samples.

Grid metadata from the final snapshot:

- `/map`: frame=map, width=62, height=243, resolution=0.0500000007451, origin=(-0.185499999989, -1.07636939634), data_len=15066
- `/local_costmap/costmap`: frame=odom, width=60, height=60, resolution=0.0500000007451, origin=(-1.45000000037, -1.45000000037), data_len=3600
- `/global_costmap/costmap`: frame=map, width=62, height=243, resolution=0.0500000007451, origin=(-0.185499999989, -1.07636939634), data_len=15066

Whole-grid occupancy ratios from full arrays:

- `/map`: free=2374/15066 (0.158), occupied=75/15066 (0.005), unknown=12617/15066 (0.837)
- local costmap: free=2593/3600 (0.720), occupied=1007/3600 (0.280), unknown=0/3600 (0.000)
- global costmap: free=2447/15066 (0.162), occupied=1838/15066 (0.122), unknown=10781/15066 (0.716)

Near-robot map sufficiency at first topology/FAILED_EXHAUSTED time (14.574s):

- `/map` near robot: known_ratio=0.402, free_ratio=0.398, unknown_ratio=0.598, out_of_bounds_cells=1341
- local costmap near robot: known_ratio=1.000, free_ratio=0.959, occupied_ratio=0.041
- global costmap near robot: known_ratio=0.433, free_ratio=0.339, unknown_ratio=0.567, out_of_bounds_cells=1341
- scan: finite_count=322/900, finite_ratio=0.358, nearest_obstacle_m=1.478

Near-robot map sufficiency remained false at 90s:

- `/map` near robot at 90s: known_ratio=0.402, free_ratio=0.398, unknown_ratio=0.598
- local costmap near robot at 90s: known_ratio=1.000, free_ratio=0.959
- global costmap near robot at 90s: known_ratio=0.433, free_ratio=0.339, unknown_ratio=0.567
- `first_sufficient_snapshot`: null

## maze_explorer startup state evidence

First explorer_state:

- elapsed_sec=14.574
- mode=FAILED_EXHAUSTED
- last_terminal_reason=no untried branches remain
- goal_count=0
- goal_success_count=0
- goal_failure_count=0
- current_node_id=2
- known_junctions=1
- last_local_topology_kind=unknown
- last_open_direction_count=0
- last_candidate_count=0
- near_exit_fallback_enabled=false

Goal event samples:

- count=0

This is not a dispatch-success or autonomous-success run. maze_explorer reached `FAILED_EXHAUSTED` before any sufficient map snapshot existed:

- `failed_exhausted_before_map_sufficient`: true

## Recomputed topology sampling evidence

At the first topology/FAILED_EXHAUSTED timestamp, recomputed four-direction local topology sampling found no open direction:

- attempted: true
- sampled_direction_count=4
- open_direction_count_recomputed=0
- block_reason_counts={"clearance_radius_blocked": 4}
- parameters: lookahead_m=1.60, min_open_distance_m=0.55, clearance_radius_m=0.38, angle_step_deg=90.0

All four directions were immediately clearance-blocked around the robot pose near the map/odom origin. This explains the zero candidate count without requiring any Nav2/controller tuning hypothesis.

## Conclusion classification

Selected Phase39 classification:

```text
FRAME_ALIGNMENT_ISSUE_CONFIRMED
```

Reason:

```text
robot pose is 14.237 m from active entrance
```

Interpretation:

- Phase38 remains valid: startup evidence is not sufficient for the first topology analysis.
- Phase39 hardens the result by showing the active entrance frame alignment problem: TF consistently places the robot near map/odom origin, while active entrance metadata is 14.237 m away.
- Sensor flow was not completely absent at topology time: `/map`, local costmap, scan, and TF were available, with scan finite_count=322. Therefore the stronger diagnosis is not `SENSOR_FLOW_NOT_READY_AT_TOPOLOGY_TIME`.
- `START_POSE_ALIGNMENT_OK_BUT_MAP_NOT_READY` is not selected because start pose alignment is not OK.
- `MAP_SUFFICIENCY_DELAY_REQUIRED` is not selected as the final category because delaying alone would not resolve the 14.237 m robot-vs-active-entrance mismatch; the 90s snapshot still had no sufficient map snapshot.
- `INSUFFICIENT_EVIDENCE` is not selected because the bounded run produced full map/costmap/scan/TF/explorer evidence and overlays.

## Verification

Focused contract/regression tests:

```text
python3 -m pytest -p no:cacheprovider   src/tugbot_maze/test/test_phase36_autonomous_readiness.py   src/tugbot_maze/test/test_phase37_bounded_smoke_wrapper.py   src/tugbot_maze/test/test_phase38_initial_local_topology.py   src/tugbot_maze/test/test_phase39_startup_map_sufficiency.py -q

15 passed in 0.62s
```

Phase39 colcon build/test available item after cleaning the stale package build directory:

```text
rm -rf build/tugbot_maze install/tugbot_maze log/latest_test/tugbot_maze
. /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select tugbot_maze
colcon test --packages-select tugbot_maze --event-handlers console_direct+ --pytest-args -k phase39_startup_map_sufficiency
colcon test-result --verbose

Summary: 1 package finished [0.93s]
3 passed, 237 deselected in 0.77s
Summary: 3 tests, 0 errors, 0 failures, 0 skipped
```

A broader `colcon test --packages-select tugbot_maze` was also attempted. It ran 240 tests and produced 212 passed / 28 failed. The failures are pre-existing or unrelated asset-history failures around missing older scaffold artifacts such as `maze_20260522.jpg`, `maze_wall_segments_20260522.yaml`, `tugbot_maze_world.sdf`, and unaligned old Phase32/33 expectations. Phase39 tests passed inside that run.

A later package build initially failed because the previous failed full-package test left a stale `build/tugbot_maze/config/maze_instance.yaml` install-manifest reference. Removing `build/tugbot_maze`, `install/tugbot_maze`, and `log/latest_test/tugbot_maze` fixed the stale build state; the focused Phase39 colcon build/test then passed.

Targeted diff check for forbidden runtime/strategy tuning files:

```text
git diff -- src/tugbot_navigation/config src/tugbot_bringup/launch src/tugbot_maze/tugbot_maze/maze_explorer.py
```

No diff was produced for these paths.

Cleanup verification:

```text
pgrep -af 'ros2 launch|gz sim|gzserver|gzclient|rviz2|slam_toolbox|nav2_|bt_navigator|controller_server|planner_server|behavior_server|waypoint_follower|maze_explorer|record_phase39_startup_map_sufficiency' || true
```

The only match was the shell executing the `pgrep` command itself, not an actual ROS/Gazebo/Nav2/recorder process.

## Next recommended phase

Do not treat Phase37 as autonomous exploration success. Phase37 remains `BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH`.

Recommended next step is a bounded start-pose / entrance-frame reconciliation phase, still without Nav2/MPPI/controller tuning: verify whether the active scaled2x world robot spawn, active metadata entrance, and SLAM/map origin should be aligned by spawn pose, metadata transform, or launch initial-pose wiring before re-running bounded topology diagnostics.
