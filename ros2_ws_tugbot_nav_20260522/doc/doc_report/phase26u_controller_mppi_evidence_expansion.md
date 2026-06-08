# Phase26U diagnostics-only controller/MPPI evidence expansion

Date: 2026-05-26
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## Goal

Phase26U stops repeating the same compact-summary smokes and adds new evidence dimensions to explain first `cmd_vel_nav` near-zero.

Guardrails:

- diagnostics-only
- no Phase27 intervention
- no branch-selection changes
- no Nav2/controller parameter changes
- no candidate promotion/rejection
- no new long run

Source runs analyzed:

- `phase26r_baseline_summary_run1`
- `phase26r_baseline_summary_run2`
- `phase26r_candidate_summary_run1`

## New tools

1. `tools/analyze_phase26u_controller_log_signals.py`

Reads launch logs plus timeline artifacts and searches dispatch/recovery/first-cmd-near-zero/timeout windows for:

- no valid control
- invalid trajectory
- collision / near collision
- constraint
- CostCritic / PathAlign / PathFollow / GoalCritic keywords
- Failed to make progress
- controller abort
- zero velocity

Output:

- `log/phase26u_controller_log_signals.json`

2. `tools/audit_phase26u_mppi_topic_semantics.py`

Audits installed Nav2 Jazzy MPPI headers/library strings plus compact evidence rows for:

- `/trajectories`
- `/optimal_trajectory`
- `/transformed_global_plan`

Output:

- `log/phase26u_mppi_topic_semantics_audit.json`

3. `tools/analyze_phase26u_optimal_path_local_cost_join.py`

Attempts offline join between compact MPPI path summaries and local-cost / post-recovery snapshots around first cmd-near-zero ±1s.

Output:

- `log/phase26u_optimal_path_local_cost_join.json`

## TDD

Tests added:

- `src/tugbot_maze/test/test_phase26u_diagnostics.py`

RED:

- `python3 -m pytest src/tugbot_maze/test/test_phase26u_diagnostics.py -q`
- initial result: `3 failed`
- expected reason: the three Phase26U tools did not exist.

GREEN:

- `3 passed in 0.11s`

## Controller log signal analysis

Command:

```bash
python3 tools/analyze_phase26u_controller_log_signals.py \
  --log-dir log \
  --run-ids phase26r_baseline_summary_run1 phase26r_baseline_summary_run2 phase26r_candidate_summary_run1 \
  --output-json log/phase26u_controller_log_signals.json
```

Result summary:

- case_count: `3`
- condition_hypothesis_counts:
  - `progress_failure_controller_abort_only_no_specific_mppi_reason`: `3`
- specific_log_signal_case_count: `0`
- phase27_candidate_signal: `not_supported`

Per case:

- all 3 cases contain:
  - `Failed to make progress`: `1`
  - controller abort: `1`
- all 3 cases have zero hits for:
  - no valid control
  - invalid trajectory
  - collision
  - near collision
  - constraint
  - zero velocity
  - CostCritic / PathAlign / PathFollow / GoalCritic in the near-zero window

Interpretation:

The existing launch logs confirm progress failure + controller abort near the timing window, but do not expose the internal MPPI reason for the near-zero command. There is no textual evidence for no-valid-control, invalid trajectory, collision, near-collision, or a specific critic pathway.

## MPPI topic semantics audit

Command:

```bash
python3 tools/audit_phase26u_mppi_topic_semantics.py \
  --ros-prefix /opt/ros/jazzy \
  --sample-evidence log/phase26r_candidate_summary_run1_mppi_evidence_summary.jsonl \
  --output-json log/phase26u_mppi_topic_semantics_audit.json
```

Result summary:

- `/trajectories`
  - message_type: `visualization_msgs/msg/MarkerArray`
  - has `trajectories_publisher_` in `trajectory_visualizer.hpp`
  - has candidate-trajectory add API: `add(const models::Trajectories & trajectories, ...)`
  - library string `/trajectories` available
  - sample_marker_count: `7656`
  - sample_point_count: `0`
  - sample_representative_path_length: `null`
  - geometry_limit: `markerarray_points_empty_in_compact_summary`

- `/optimal_trajectory`
  - message_type: `nav_msgs/msg/Path`
  - has `optimal_path_pub_`
  - has optimal trajectory add API using `xt::xtensor<float, 2>`
  - sample_point_count: `56`
  - sample_path_displacement: `0.048858368921298155` in first candidate sample

- `/transformed_global_plan`
  - message_type: `nav_msgs/msg/Path`
  - has `transformed_path_pub_`
  - sample_point_count: `18`
  - sample_path_displacement: `0.45676911761926187` in first candidate sample

Key limitation:

Compact summaries keep counts/length/displacement but not raw path poses. Therefore they cannot support spatial join between optimal trajectory geometry and high-cost cells. `/trajectories` MarkerArray currently appears as many markers but zero points in compact summary, so it also cannot reconstruct candidate trajectory geometry.

Missing evidence from semantics audit:

- raw or sampled optimal trajectory poses
- raw or sampled transformed plan poses
- MarkerArray marker types/points or selected trajectory ID

## Offline optimal path / local-cost join

Command:

```bash
python3 tools/analyze_phase26u_optimal_path_local_cost_join.py \
  --log-dir log \
  --run-ids phase26r_baseline_summary_run1 phase26r_baseline_summary_run2 phase26r_candidate_summary_run1 \
  --output-json log/phase26u_optimal_path_local_cost_join.json
```

Result summary:

- case_count: `3`
- condition_hypothesis_counts:
  - `near_zero_with_local_path_high_cost_but_optimal_path_geometry_missing`: `3`
- high_cost_choke_case_count: `3`
- spatial_join_possible_case_count: `0`
- phase27_candidate_signal: `not_supported`

Per case:

1. `phase26r_baseline_summary_run1`
   - near_zero_path_ahead_1_0m_cost_max: `99`
   - near_zero_high_cost_count: `8`
   - near_zero_robot_to_path_distance_m: `0.08667487702616955`
   - optimal path displacement min/max: `0.00450001984383581` / `0.006050366085208177`
   - zero_displacement_path_sample_count: `2`
   - spatial_join_possible: `false`

2. `phase26r_baseline_summary_run2`
   - near_zero_path_ahead_1_0m_cost_max: `100`
   - near_zero_high_cost_count: `11`
   - near_zero_robot_to_path_distance_m: `0.09814595897362753`
   - optimal path displacement min/max: `0.00004747471604189324` / `0.005196660711260056`
   - zero_displacement_path_sample_count: `2`
   - spatial_join_possible: `false`

3. `phase26r_candidate_summary_run1`
   - near_zero_path_ahead_1_0m_cost_max: `99`
   - near_zero_high_cost_count: `6`
   - near_zero_robot_to_path_distance_m: `0.07961140563889703`
   - optimal path displacement min/max: `0.000052764136478960286` / `0.005071020571039716`
   - zero_displacement_path_sample_count: `3`
   - spatial_join_possible: `false`

Interpretation:

All three runs show near-zero coinciding with local path-ahead high-cost evidence and near-zero/near-stationary optimal trajectory summary. This is stronger than Phase26T's broad `trajectory_evidence_present_without_critic_stats` condition.

However, Phase26U still cannot prove that the optimal trajectory itself crosses the high-cost choke, because compact summaries do not contain raw path poses. Therefore this is still a constrained diagnostic hypothesis, not an intervention-ready controller/critic condition.

## Phase26U conclusion

Specific controller/MPPI condition found?

No.

What Phase26U adds:

- It rules out available textual launch-log evidence for no-valid-control, invalid trajectory, near-collision, collision, constraint, or specific critic keyword pathways in the near-zero windows.
- It confirms installed MPPI publishes the relevant debug topics through `TrajectoryVisualizer`, but compact summaries are insufficient for geometry reconstruction.
- It identifies a repeated diagnostic pattern across baseline and candidate:
  - near-zero cmd window
  - local path-ahead high cost (`99/100`)
  - small robot-to-path distance (`~0.08-0.10 m`)
  - near-zero/near-stationary optimal path displacement
  - missing raw path geometry prevents spatial proof

Phase27 remains blocked.

Reason:

- no controller log signal for a concrete MPPI failure mode
- no critic/validator/no-valid-control evidence
- no raw optimal path geometry for spatial high-cost join
- evidence still cannot distinguish whether the near-zero command is due to CostCritic, PathAlign/PathFollow geometry, trajectory validator, or another controller pathway

## Required next evidence

If continuing to Phase26V, keep diagnostics-only and run a bounded evidence collection that records sampled geometry rather than full raw payloads:

1. Modify compact recorder to store sampled path poses for `/optimal_trajectory` and `/transformed_global_plan` in first cmd-near-zero windows, bounded by max points per message.
2. Add MarkerArray type/action summary for `/trajectories`, not full payload:
   - marker type counts
   - action counts
   - namespace counts
   - pose/scale/color summary
   - optional bounded first N marker point samples if points exist
3. Join sampled path poses with near-zero high-cost points to prove or refute whether the optimal path/plan intersects the local high-cost choke.
4. Still do not tune Nav2/controller parameters or branch selection until a stable, specific condition is reviewed.

## Verification

- Phase26U focused tests: `3 passed in 0.11s`
- Phase26P/Q/R/S/T/U focused tests: `18 passed in 0.56s`
- Phase24/25/26 regression: `91 passed in 1.78s`
- py_compile Phase26 tools and launch: passed
- `bash -n tools/run_phase21_controller_diagnostics_smoke.sh`: passed
- `colcon build --symlink-install --packages-select tugbot_bringup tugbot_navigation tugbot_maze`: `3 packages finished [0.93s]`
- `colcon test --packages-select tugbot_maze`: `162 passed in 2.15s`; `colcon test-result --verbose`: `0 errors, 0 failures`
