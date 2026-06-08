# Phase26V sampled path / local-cost spatial join

Date: 2026-05-26
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## Goal

Phase26V stays diagnostics-only. It fills the Phase26U geometry gap by recording bounded sampled poses for MPPI path topics and then spatially joining those sampled path points with near-zero local high-cost points.

Forbidden in this phase:

- Enter Phase27
- Modify branch selection
- Modify Nav2/controller parameters
- Promote or reject candidate `CostCritic.cost_weight=2.75`

## Code changes

### Recorder bounded geometry summary

Updated:

- `tools/record_phase26p_mppi_evidence.py`

Additions:

- `--sample-points`, default `30`
- `/optimal_trajectory` and `/transformed_global_plan` path summaries now include:
  - `sampled_point_count`
  - `sampled_points`, bounded to at most 30 per message
- `/trajectories` MarkerArray remains summary-only, no raw payload:
  - `marker_type_counts`
  - `marker_action_counts`
  - `marker_namespace_counts`
  - `scale_summary`
  - `color_alpha_summary`
  - bounded `sampled_marker_points` if marker points exist

### Phase26V join analyzer

Added:

- `tools/analyze_phase26v_sampled_path_local_cost_join.py`

Inputs:

- `${RUN_ID}_mppi_evidence_summary.jsonl`
- `${RUN_ID}_phase26p_single_goal_timeline.json`
- `${RUN_ID}_post_recovery_enriched.json`

Output per case:

- `optimal_path_intersects_high_cost`
- `transformed_plan_intersects_high_cost`
- `optimal_path_min_distance_to_high_cost`
- `transformed_plan_min_distance_to_high_cost`
- path length/displacement/sample counts
- `condition_hypothesis`

### Wrapper run IDs

Updated:

- `tools/run_phase21_controller_diagnostics_smoke.sh`

Added diagnostics-only aliases:

- `phase26v_baseline_geometry_runN`
- `phase26v_candidate_geometry_runN`

These reuse Phase26P/26R diagnostics profiles, run Phase26R coverage, then run Phase26V sampled geometry join. They do not change branch selection or controller/Nav2 semantics.

## TDD

Added:

- `src/tugbot_maze/test/test_phase26v_sampled_geometry_join.py`

RED result:

- `3 failed`

Expected failures:

- `summarize_path(..., sample_limit=...)` not implemented
- `tools/analyze_phase26v_sampled_path_local_cost_join.py` missing
- wrapper missing Phase26V run IDs

GREEN result:

- `3 passed in 0.04s`

## Runtime evidence

### Baseline bounded geometry smoke

Command:

```bash
PHASE_RUN_MAX_GOALS=2 \
PHASE_RUN_STATE_MAX_SAMPLES=90 \
PHASE_RUN_GOAL_EVENTS_MAX_SAMPLES=70 \
PHASE_RUN_TIMEOUT_SEC=190 \
PHASE_RUN_SNAPSHOT_TIMEOUT_SEC=210 \
bash tools/run_phase21_controller_diagnostics_smoke.sh phase26v_baseline_geometry_run1
```

Runtime proof:

- `/controller_server` loaded `FollowPath.CostCritic.cost_weight = 3.81`

Outcome:

- `final_mode`: `FAILED_EXHAUSTED`
- `goal_count`: 2
- `goal_success_count`: 1
- `goal_failure_count`: 1
- `timeout_cancel_count`: 1
- `blocked_branch_count`: 0
- `blacklisted_goal_count`: 0
- `exit_distance_m`: 4.35796340571367

Evidence file:

- `log/phase26v_baseline_geometry_run1_mppi_evidence_summary.jsonl`
- size: 247,769 bytes, below 5 MB
- message rows: 227
- topic rows:
  - `/optimal_trajectory`: 77
  - `/trajectories`: 77
  - `/transformed_global_plan`: 76
  - `/docking_trajectory`: 1

Coverage:

- `log/phase26v_baseline_geometry_run1_phase26r_summary_evidence_coverage.json`
- `pass_coverage`: true
- evidence size: 247,769 bytes
- first cmd-near-zero ±1s covered:
  - `/trajectories`: 3 samples
  - `/optimal_trajectory`: 3 samples
  - `/transformed_global_plan`: 3 samples

Phase26V join:

- `log/phase26v_baseline_geometry_run1_phase26v_sampled_path_local_cost_join.json`

Result:

- `condition_hypothesis`: `sampled_paths_do_not_intersect_high_cost_choke`
- `optimal_path_intersects_high_cost`: false
- `transformed_plan_intersects_high_cost`: false
- `local_high_cost.path_ahead_1_0m_cost_max`: 100
- `local_high_cost.high_cost_point_count`: 11
- `robot_to_path_distance_m`: 0.09821121125305839
- optimal path min distance to high-cost: 0.9252544447085893 m
- transformed plan min distance to high-cost: 0.9201603035889996 m
- optimal sampled points: 90 across 3 rows, max 30 per message
- transformed sampled points: 36 across 3 rows, max 12 per message
- optimal path displacement: 0.0013068230407379678 to 0.004735652186361809 m
- optimal path length max: 0.009164119946129383 m

### Candidate bounded geometry smoke

Baseline coverage passed and file size stayed small, so one candidate evidence comparison run was executed. This is evidence-only and not promotion/rejection.

Command:

```bash
PHASE_RUN_MAX_GOALS=2 \
PHASE_RUN_STATE_MAX_SAMPLES=90 \
PHASE_RUN_GOAL_EVENTS_MAX_SAMPLES=70 \
PHASE_RUN_TIMEOUT_SEC=190 \
PHASE_RUN_SNAPSHOT_TIMEOUT_SEC=210 \
bash tools/run_phase21_controller_diagnostics_smoke.sh phase26v_candidate_geometry_run1
```

Runtime proof:

- `/controller_server` loaded `FollowPath.CostCritic.cost_weight = 2.75`

Outcome:

- `final_mode`: `FAILED_EXHAUSTED`
- `goal_count`: 2
- `goal_success_count`: 2
- `goal_failure_count`: 0
- `timeout_cancel_count`: 0
- `blocked_branch_count`: 0
- `blacklisted_goal_count`: 0
- `exit_distance_m`: 4.456074527515507

This bounded outcome is not a promotion/rejection signal.

Evidence file:

- `log/phase26v_candidate_geometry_run1_mppi_evidence_summary.jsonl`
- size: 179,062 bytes, below 5 MB
- message rows: 161
- topic rows:
  - `/optimal_trajectory`: 55
  - `/trajectories`: 55
  - `/transformed_global_plan`: 54
  - `/docking_trajectory`: 1

Phase26V join:

- `log/phase26v_candidate_geometry_run1_phase26v_sampled_path_local_cost_join.json`

Result:

- `condition_hypothesis`: `sampled_paths_do_not_intersect_high_cost_choke`
- `optimal_path_intersects_high_cost`: false
- `transformed_plan_intersects_high_cost`: false
- `local_high_cost.path_ahead_1_0m_cost_max`: 99
- `local_high_cost.high_cost_point_count`: 8
- `robot_to_path_distance_m`: 0.08301535617542133
- optimal path min distance to high-cost: 0.8789217638504578 m
- transformed plan min distance to high-cost: 0.8372596836406253 m
- optimal sampled points: 90 across 3 rows, max 30 per message
- transformed sampled points: 31 across 3 rows, max 11 per message
- optimal path displacement: 0.03571561905314619 to 0.06959679795195603 m
- optimal path length max: 0.0696026202272791 m

## Combined baseline + candidate join

Artifact:

- `log/phase26v_baseline_candidate_sampled_path_local_cost_join.json`

Summary:

- `case_count`: 2
- `sampled_geometry_case_count`: 2
- `intersection_case_count`: 0
- `condition_hypothesis_counts`:
  - `sampled_paths_do_not_intersect_high_cost_choke`: 2
- `phase27_candidate_signal`: `not_supported`
- `intervention_allowed`: false

## Interpretation

Phase26V proves the sampled path geometry gap from Phase26U is closed for the tested windows:

- sampled `/optimal_trajectory` points are present
- sampled `/transformed_global_plan` points are present
- local high-cost points are present
- spatial join is possible

The join does not support the earlier high-cost-choke hypothesis:

- Baseline optimal path min distance to high-cost is about 0.925 m
- Baseline transformed plan min distance to high-cost is about 0.920 m
- Candidate optimal path min distance to high-cost is about 0.879 m
- Candidate transformed plan min distance to high-cost is about 0.837 m

Therefore, in these two bounded cases, the sampled optimal path and sampled transformed plan did not pass through the local high-cost choke points despite local path-ahead high cost being present near cmd-near-zero.

The repeated concrete condition is now:

- `sampled_paths_do_not_intersect_high_cost_choke`

This is useful negative evidence. It argues against a simple explanation that near-zero cmd is caused by sampled optimal path or transformed global plan directly crossing the high-cost choke points captured by the local-cost recorder.

## Phase27 gate

Phase27 remains blocked.

Reason:

- No sampled optimal-path high-cost intersection
- No sampled transformed-plan high-cost intersection
- No critic-specific MPPI evidence remains available in installed Jazzy MPPI
- Controller logs still only show progress failure/controller abort, not specific no-valid-control/collision/critic reason
- Candidate evidence is comparison-only and cannot promote/reject `2.75`

Still prohibited:

- Branch-selection changes
- Nav2/controller parameter changes
- Candidate promotion
- Candidate rejection

## Remaining evidence gap

Phase26V removes the raw path geometry gap, but the reason for near-zero command is still not proven.

Missing evidence:

1. Per-cycle selected control / command candidate reason from MPPI
   - The sampled optimal path is near-stationary but not intersecting high-cost points.
   - Need evidence explaining why MPPI chooses or outputs near-zero despite available transformed plan geometry.

2. Better alignment between local-cost high-cost points and MPPI cost evaluation frame
   - Current local high-cost points come from post-recovery local-cost snapshots/path-ahead summaries.
   - They may not correspond to the critic's evaluated footprint/cost samples for the selected trajectory.

3. Critic-level or optimizer-level diagnostic signal
   - `critics_stats` remains unavailable in installed Jazzy MPPI.
   - Alternative evidence may require source-level instrumentation or a bounded custom recorder around controller inputs/outputs.

No tuning is recommended from Phase26V.

## Verification

Focused Phase26P/Q/R/S/T/U/V tests:

- `21 passed in 0.57s`

Phase24/25/26 regression:

- `94 passed in 1.85s`

Static checks:

- `py_compile` passed for Phase26P/U/V tools and related analyzers
- `bash -n tools/run_phase21_controller_diagnostics_smoke.sh` passed

Build:

- `colcon build --symlink-install --packages-select tugbot_bringup tugbot_navigation tugbot_maze`
- `3 packages finished [0.94s]`

Colcon test:

- `colcon test --packages-select tugbot_maze`
- `165 passed in 2.20s`
- `colcon test-result --verbose`: `0 errors, 0 failures`

Cleanup:

- No residual ROS/Gazebo/Nav2/recorder processes found.
