# Phase72 Multi-Goal Bounded Rerun From Inner Ingress / Re-dispatch Validation

Run id: `phase72_multigoal_bounded_rerun_from_inner_ingress`

## Scope

Phase72 validates the Phase71 conclusion `POST_SUCCESS_EXHAUSTED_PREMATURELY` by raising the bounded autonomous `maze_explorer` budget from `max_goals=1` to `max_goals=3` and starting from the accepted Phase65 inner ingress waypoint `(2.0, 0.0, 0.0)`.

This phase is diagnostics/runtime validation only:

- no Nav2/MPPI/controller parameter edits
- no inflation/robot_radius/clearance_radius_m/map threshold tuning
- no branch scoring change
- no additional centerline gate relaxation beyond the Phase70 `balance_first` mode
- no cmd_vel corridor-following
- no fallback/terminal acceptance change
- no autonomous exploration success claim
- no exit success claim
- stop after Phase72; do not enter Phase73 / 不进入 Phase73

## Added artifacts and tools

Source files:

- `tools/run_phase72_multigoal_bounded_rerun_from_inner_ingress.sh`
- `tools/analyze_phase72_multigoal_bounded_rerun_from_inner_ingress.py`
- `src/tugbot_maze/test/test_phase72_multigoal_bounded_rerun_from_inner_ingress.py`
- `doc/doc_report/phase72_multigoal_bounded_rerun_from_inner_ingress_report.md`

Runtime artifact root:

- `log/phase72_multigoal_bounded_rerun_from_inner_ingress/`
- summary JSON: `log/phase72_multigoal_bounded_rerun_from_inner_ingress/phase72_multigoal_bounded_rerun_from_inner_ingress.json`

Per-replay runtime streams include:

- `/maze/goal_events` JSONL
- `/maze/explorer_state` JSONL
- Phase72 runtime timeline JSONL
- controller dynamics JSONL
- Nav2 feedback JSONL
- local costmap samples JSONL
- global plan samples JSONL
- collision monitor state JSONL

## Runtime configuration

Bounded runtime was executed with:

- `PHASE72_REPLAY_COUNT=2`
- `PHASE72_MAX_GOALS=3`
- `PHASE72_GOAL_TIMEOUT_SEC=100`
- `PHASE72_EXPLORER_OBSERVE_SEC=180`
- `PHASE72_RUNTIME_RECORD_TIMEOUT_SEC=215`
- `PHASE72_RUN_TIMEOUT_SEC=300`
- inner ingress waypoint: `(2.0, 0.0, 0.0)`
- fallback disabled: `near_exit_fallback_enabled=false`
- startup warmup no-dispatch disabled: `startup_warmup_no_dispatch=false`
- centerline mode preserved: `centerline_target_refinement_gate_mode=balance_first`

The analyzer reports:

- classification: `MULTIGOAL_NO_REDISPATCH_AFTER_SUCCESS`
- replay_count: `2`
- max_goals: `3`
- guardrail_violation: `false`
- nav2_config_diff_empty: `true`
- cleanup_empty: `true`
- complete_autonomous_success_claimed: `false`
- exit_success_claimed: `false`

## Summary metrics

From `phase72_multigoal_bounded_rerun_from_inner_ingress.json`:

- inner_ingress_goal_success: `true`
- goal1_success_replay_count: `2`
- redispatch_after_goal1_success_observed: `false`
- goal2_dispatch_replay_count: `0`
- dispatch_count_total: `2`
- outcome_count_total: `2`
- timeout_count_total: `0`
- local_cost_risk_observed: `true`
- exit_reached_by_existing_state: `false`

Interpretation:

- Raising `max_goals` to 3 removed the Phase71 goal-budget terminal condition, but did not produce Goal2 dispatch in either replay.
- Both replays completed Goal1 successfully and returned to `WAIT_FOR_DISPATCH_ENTRY_READINESS`.
- The post-success gate then remained blocked by `local_costmap_sufficient=false`, specifically `local_costmap_ratio_or_bounds_insufficient`.
- Candidate/topology evidence was still available after Goal1 success: `junction`, raw open directions 4, filtered open directions 4, candidate count 4.
- Therefore this run is not a premature `goal budget reached` exhaustion anymore; it is a post-success re-dispatch readiness hold dominated by local costmap sufficiency.
- Because no Goal2 dispatch occurred, Phase72 cannot yet evaluate Goal2 timeout behavior in a real dispatched second goal.

## Replay timeline evidence

### replay_01

High-level:

- inner ingress action: success
- dispatch_count: `1`
- outcome_count: `1`
- timeout_count: `0`
- Goal1 success observed: `true`
- Goal2 dispatch observed: `false`
- dispatch_after_goal1_success_count: `0`
- final_mode: `WAIT_FOR_DISPATCH_ENTRY_READINESS`
- final_goal_count: `1`
- last_terminal_reason: `null`
- last_failure_reason: `null`

Mode transitions:

1. `WAIT_FOR_DISPATCH_ENTRY_READINESS`, goal_count 0
2. `NAVIGATING`, active_goal_sequence_id 1, goal_count 1
3. `SETTLING`, goal_count 1
4. `WAIT_FOR_DISPATCH_ENTRY_READINESS`, goal_count 1

Goal1:

- target: `[2.5958592076479667, 0.19489997253619254]`
- outcome_event: `success`
- result_reason: `succeeded`
- Nav2 recoveries max: `0`
- Nav2 distance_remaining final: `0.2739926278591156 m`
- near-goal tolerance band observed: `true` (configured band 0.30 m)
- robot pose progress: total_motion `0.464353656470634 m`, distance_improvement `0.44046753595053884 m`, robot_stuck `false`
- final robot-to-target distance: `0.3296785546794773 m`
- local-cost risk observed: `true`
  - target_cost_max final/max: `99/99`
  - target_footprint_max final/max: `99/99`
  - target_footprint_lethal_count final/max: `77/77`
  - front_wedge_cost_max final/max: `100/100`
  - front_wedge_high_cost_count final/max: `233/233`

Post-success readiness:

- passed: `false`
- blocking_reasons: `["local_costmap_sufficient"]`
- checks:
  - map_sufficient: `true`
  - scan_sufficient: `true`
  - tf_sufficient: `true`
  - nav2_lifecycle_active: `true`
  - navigate_to_pose_action_ready: `true`
  - goal_pose_subscriber_ready: `true`
  - local_costmap_sufficient: `false`
- local_costmap reason: `local_costmap_ratio_or_bounds_insufficient`
- local_costmap known_ratio: `1.0`
- local_costmap free_ratio: `0.431637519872814` (minimum required 0.5)
- local_costmap occupied_ratio: `0.568362480127186`
- local_costmap sample_age_sec: `0.123`
- robot_in_bounds: `true`

Candidate/topology after Goal1 success:

- local_topology_kind: `junction`
- raw_open_direction_count: `4`
- filtered_open_direction_count: `4`
- candidate_after_filter_count: `4`
- blocked_branch_count: `0`
- blacklisted_goal_count: `0`

### replay_02

High-level:

- inner ingress action: success
- dispatch_count: `1`
- outcome_count: `1`
- timeout_count: `0`
- Goal1 success observed: `true`
- Goal2 dispatch observed: `false`
- dispatch_after_goal1_success_count: `0`
- final_mode: `WAIT_FOR_DISPATCH_ENTRY_READINESS`
- final_goal_count: `1`
- last_terminal_reason: `null`
- last_failure_reason: `null`

Mode transitions:

1. `WAIT_FOR_MAP`, goal_count 0
2. `WAIT_FOR_DISPATCH_ENTRY_READINESS`, goal_count 0
3. `NAVIGATING`, active_goal_sequence_id 1, goal_count 1
4. `SETTLING`, goal_count 1
5. `WAIT_FOR_DISPATCH_ENTRY_READINESS`, goal_count 1

Goal1:

- target: `[2.6097417047332763, 0.1698233818452811]`
- outcome_event: `success`
- result_reason: `succeeded`
- Nav2 recoveries max: `0`
- Nav2 distance_remaining final: `0.2396574765443802 m`
- near-goal tolerance band observed: `true` (configured band 0.30 m)
- robot pose progress: total_motion `0.4386875012179515 m`, distance_improvement `0.42323848776281453 m`, robot_stuck `false`
- final robot-to-target distance: `0.341614439714457 m`
- local-cost risk observed: `true`
  - target_cost_max final/max: `99/99`
  - target_footprint_max final/max: `100/100`
  - target_footprint_lethal_count final/max: `88/88`
  - front_wedge_cost_max final/max: `100/100`
  - front_wedge_high_cost_count final/max: `221/221`

Post-success readiness:

- passed: `false`
- blocking_reasons: `["local_costmap_sufficient"]`
- checks:
  - map_sufficient: `true`
  - scan_sufficient: `true`
  - tf_sufficient: `true`
  - nav2_lifecycle_active: `true`
  - navigate_to_pose_action_ready: `true`
  - goal_pose_subscriber_ready: `true`
  - local_costmap_sufficient: `false`
- local_costmap reason: `local_costmap_ratio_or_bounds_insufficient`
- local_costmap known_ratio: `1.0`
- local_costmap free_ratio: `0.4350597609561753` (minimum required 0.5)
- local_costmap occupied_ratio: `0.5649402390438247`
- local_costmap sample_age_sec: `0.159`
- robot_in_bounds: `true`

Candidate/topology after Goal1 success:

- local_topology_kind: `junction`
- raw_open_direction_count: `4`
- filtered_open_direction_count: `4`
- candidate_after_filter_count: `4`
- blocked_branch_count: `0`
- blacklisted_goal_count: `0`

## Diagnosis

Phase72 classification is `MULTIGOAL_NO_REDISPATCH_AFTER_SUCCESS`.

Operational subtype: post-success re-dispatch was blocked by readiness, not by goal budget, candidate exhaustion, branch scoring, blacklist growth, or terminal acceptance.

Evidence:

- Goal1 succeeded in both replays.
- `final_goal_count=1` despite `max_goals=3`.
- `final_mode=WAIT_FOR_DISPATCH_ENTRY_READINESS`, not `FAILED_EXHAUSTED`.
- `last_terminal_reason=null`, so Phase71 `goal budget reached` no longer explains the stop.
- `goal2_dispatch_observed=false` and `dispatch_after_goal1_success_count=0` in both replays.
- Candidate formation/topology had usable evidence after success: junction with 4 open directions / 4 candidates.
- Readiness held because `local_costmap_sufficient=false`; map, scan, TF, Nav2 lifecycle, action server, and goal_pose subscriber checks were all true.
- The local costmap evidence was fresh and in-bounds, but the near-robot free ratio was below threshold in both replays: about 0.432 and 0.435 versus required 0.5.

Timeout interpretation:

- Phase70 replay_02 showed a Goal1 timeout with not-high target local risk; Phase72 did not reproduce timeout because neither replay dispatched Goal2.
- There were no timeout events in Phase72 (`timeout_count_total=0`).
- The current bounded evidence therefore cannot answer where Goal2+ timeout would occur; the dominant blocker is before Goal2 dispatch.

Local-cost interpretation:

- Local cost risk remains visible around the successful Goal1 target and front wedge in both replays.
- This report does not recommend tuning inflation, radius, clearance, map thresholds, MPPI/controller, branch scoring, centerline runtime behavior, fallback, or terminal acceptance.
- The evidence should be treated as readiness-state-machine/local-costmap-sufficiency diagnosis only.

## Verification

Completed verification:

- `python3 -m py_compile tools/analyze_phase72_multigoal_bounded_rerun_from_inner_ingress.py src/tugbot_maze/test/test_phase72_multigoal_bounded_rerun_from_inner_ingress.py` passed.
- `bash -n tools/run_phase72_multigoal_bounded_rerun_from_inner_ingress.sh` passed.
- `pytest -q src/tugbot_maze/test/test_phase72_multigoal_bounded_rerun_from_inner_ingress.py` passed: `5 passed in 0.01s`.
- `colcon build --symlink-install --packages-select tugbot_maze tugbot_bringup` passed: `Summary: 2 packages finished [1.04s]`.
- Nav2 config diff artifact is empty: `nav2_config_diff_bytes=0`.
- Cleanup artifact is empty and live process grep returned no ROS/Gazebo/Nav2/maze_explorer/RViz leftovers: `cleanup_processes_after_bytes=0`.

## Stop condition

Phase72 stops here. Do not enter Phase73 / 不进入 Phase73.

No autonomous exploration success is claimed. No exit success is claimed.
