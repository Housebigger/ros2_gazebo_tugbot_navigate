# Phase42 Map-frame Truth Maze Explorer Bounded Autonomous Smoke Report

Generated: 2026-06-01T10:15:04

## Scope

Phase42 reran a bounded autonomous smoke on the active scaled2x world after Phase41 selected map-frame truth convention A. This phase is not a complete exit-success acceptance phase. It only checks whether map-frame truth lets `maze_explorer` start, publish state/events, and dispatch at least one autonomous goal under bounded conditions.

Preserved upstream conclusions:
- Phase37 remains `BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH`; do not rewrite it as success.
- Phase41 remains `MAP_FRAME_TRUTH_ALIGNED_READY_FOR_PHASE37_RERUN`.

Guardrails honored:
- No Nav2/MPPI/controller parameter tuning.
- No `maze_explorer` strategy changes.
- No fallback/terminal acceptance continuation.
- No old scaffold world/map.
- No claim of complete autonomous exploration success.
- Bounded timeout and max goal budget used.
- Cleanup completed.

## Implementation / tooling

Added:
- `tools/run_phase42_map_frame_truth_bounded_smoke.sh`
- `tools/analyze_phase42_map_frame_truth_bounded_smoke.py`
- `src/tugbot_maze/test/test_phase42_map_frame_truth_bounded_smoke.py`

The Phase37 wrapper was not overwritten. Phase42 uses its own run id and log directory:
- run id: `phase42_map_frame_truth_bounded_smoke`
- artifact dir: `log/phase42_map_frame_truth_bounded_smoke/`

## Explicit launch contract

Preflight and launch use:
- world_sdf: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf`
- maze_config: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_maze/config/maze_20260528_scaled_instance.yaml`
- truth frame: `map`
- entrance: (0.0, 0.0, 0.0)
- exit: (21.072562, 18.083566)
- exit_radius: 1.2
- timeout_sec: 900.0
- max_goals: 4
- near_exit_fallback_enabled: false
- headless default: true

Old scaffold truth/use:
- legacy truth seen in launch log: False
- active map-frame truth used: True

## Verification before runtime

- Phase42 focused test: `6 passed in 0.00s`
- Phase36/37/41/42 focused regression: `23 passed in 0.35s`
- readiness analyzer decision: `READY_FOR_BOUNDED_AUTONOMOUS_EXPLORATION_SMOKE`
- readiness blockers: `[]`
- `git diff -- src/tugbot_navigation/config`: empty

Note: Phase42 did not modify Nav2 config files.

## Runtime evidence

Acceptance analysis:
- classification: `BOUNDED_SMOKE_NO_DISPATCH_TOPOLOGY_SAMPLING`
- complete_autonomous_success_claimed: False
- bounded_smoke_partial_pass: False
- maze_explorer_started: True
- explorer_state_samples: 10
- goal_events_samples: 0
- dispatch_events: 0
- outcome_events: 0
- exit_reached_events: 0
- Nav2 action server available: True
- cleanup_empty: True

Final explorer state excerpt:
- mode: `FAILED_EXHAUSTED`
- terminal reason: `no untried branches remain`
- goal_count: 0
- goal_success_count: 0
- goal_failure_count: 0
- nav2_failure_count: 0
- last_local_topology_kind: `unknown`
- last_open_direction_count: 0
- last_candidate_count: 0
- known_junctions: 0
- edges: 0
- exit_distance_m: 27.768115321709352
- near_exit_fallback_enabled: False

Classification reasons:
- topology failed before dispatch: mode=FAILED_EXHAUSTED, topology=unknown, open=0, candidates=0, terminal=no untried branches remain


Interpretation:
- Phase41 map-frame truth was used and old -4/-3/4/3/0.6 truth was not used.
- `maze_explorer` started and `/maze/explorer_state` had data.
- `/navigate_to_pose` action server was available.
- No `/maze/goal_events` rows were emitted because no dispatch occurred.
- The run reached `FAILED_EXHAUSTED` before dispatch with topology `unknown`, open direction count 0, and candidate count 0.
- This is a no-dispatch bounded smoke, classified as topology sampling rather than a Nav2 action failure.

## Required artifacts

Artifact directory: `log/phase42_map_frame_truth_bounded_smoke/`

Key files:
- `phase42_map_frame_truth_bounded_smoke_launch.log`
- `phase42_map_frame_truth_bounded_smoke_explorer_state.jsonl`
- `phase42_map_frame_truth_bounded_smoke_goal_events.jsonl`
- `phase42_map_frame_truth_bounded_smoke_map_once.txt`
- `phase42_map_frame_truth_bounded_smoke_scan_once.txt`
- `phase42_map_frame_truth_bounded_smoke_odom_once.txt`
- `phase42_map_frame_truth_bounded_smoke_tf_once.txt`
- `phase42_map_frame_truth_bounded_smoke_local_costmap_once.txt`
- `phase42_map_frame_truth_bounded_smoke_global_costmap_once.txt`
- `phase42_map_frame_truth_bounded_smoke_navigate_to_pose_action_info.txt`
- `phase42_map_frame_truth_bounded_smoke_summary.json`
- `phase42_map_frame_truth_bounded_smoke_acceptance_analysis.json`
- `phase42_map_frame_truth_bounded_smoke_cleanup_processes_after.txt`
- `phase42_preflight_readiness.json`
- `phase42_nav2_config_diff.txt`

## Cleanup

Wrapper cleanup artifact is empty: True

Post-report live process check actual matched residual process count: 0

Raw live pgrep output only contained the pgrep/shell command itself when non-empty:
```
466237 /bin/sh -c pgrep -af 'ros2 launch|gz sim|rviz2|slam_toolbox|maze_explorer|frontier_explorer|controller_server|planner_server|bt_navigator|behavior_server|waypoint_follower|velocity_smoother|smoother_server|ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|record_explorer_state_series.py.*phase42' || true
```

## Conclusion

`BOUNDED_SMOKE_NO_DISPATCH_TOPOLOGY_SAMPLING`

This is not complete autonomous exploration success. It is a bounded smoke result showing that the corrected map-frame truth is in use, but the explorer still does not dispatch a goal. Next diagnostics should focus on initial topology sampling around the map-frame entrance under the now-correct frame convention: compare `/map`, local/global costmaps, scan, TF, and the explorer's sampled branch candidates/open-direction rejection reasons at the first topology analysis. Do not tune Nav2/MPPI/controller parameters or alter strategy until that evidence isolates the cause.
