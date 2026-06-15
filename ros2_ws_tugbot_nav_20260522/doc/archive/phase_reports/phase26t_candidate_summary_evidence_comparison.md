# Phase26T candidate compact-summary evidence comparison

Date: 2026-05-26
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## Goal

Phase26T remains diagnostics-only. It compares candidate compact-summary evidence against the Phase26S baseline compact-summary repeats.

It does not:

- change branch selection
- tune Nav2/controller parameters
- promote `CostCritic.cost_weight=2.75`
- reject `CostCritic.cost_weight=2.75`
- enter Phase27 intervention

Even if a candidate difference appears, Phase26T can only mark it as review-only evidence.

## Tool updates

Updated:

- `tools/aggregate_phase26s_summary_repeats.py`

New `--phase` option:

- `--phase 26S` for baseline repeat aggregation
- `--phase 26T` for baseline + candidate evidence comparison

Phase26T-specific output:

- `summary.candidate_comparison`
- `decision.candidate_promotion_allowed: false`
- `decision.candidate_rejection_allowed: false`
- guardrail: `do_not_promote_or_reject_candidate_from_phase26t`

Added tests:

- `src/tugbot_maze/test/test_phase26t_candidate_summary_comparison.py`

## TDD

RED observed before implementation:

- `python3 -m pytest src/tugbot_maze/test/test_phase26t_candidate_summary_comparison.py src/tugbot_maze/test/test_phase26s_summary_repeat_aggregator.py -q`
- initial result: `5 failed`
- expected reason: aggregator did not yet accept `--phase 26T` / `--phase 26S`

GREEN:

- `5 passed in 0.19s`

## Candidate bounded smoke

Command:

```bash
PHASE_RUN_MAX_GOALS=2 \
PHASE_RUN_STATE_MAX_SAMPLES=90 \
PHASE_RUN_GOAL_EVENTS_MAX_SAMPLES=70 \
PHASE_RUN_TIMEOUT_SEC=190 \
PHASE_RUN_SNAPSHOT_TIMEOUT_SEC=210 \
bash tools/run_phase21_controller_diagnostics_smoke.sh phase26r_candidate_summary_run1
```

Runtime profile proof:

- run_id: `phase26r_candidate_summary_run1`
- runtime `/controller_server` loaded `FollowPath.CostCritic.cost_weight: 2.75`
- runtime_semantics_ok in aggregate: `true`

Runtime bounded outcome:

- final_mode: `FAILED_EXHAUSTED`
- goal_count: `2`
- goal_success_count: `1`
- goal_failure_count: `1`
- timeout_cancel_count: `1`
- blocked_branch_count: `0`
- blacklisted_goal_count: `0`
- exit_distance_m: `4.348221686587794`

This bounded outcome is not used for candidate promotion/rejection.

## Candidate compact evidence

Artifact:

- `log/phase26r_candidate_summary_run1_mppi_evidence_summary.jsonl`

Size:

- `72,875` bytes

Rows:

- message rows: `170`
- topic_discovered rows: `4`
- summary rows: `1`

Topic counts:

- `/optimal_trajectory`: `58`
- `/trajectories`: `58`
- `/transformed_global_plan`: `57`
- `/docking_trajectory`: `1`

## Candidate first cmd-near-zero ±1s coverage

Artifact:

- `log/phase26r_candidate_summary_run1_phase26r_summary_evidence_coverage.json`

Result:

- pass_coverage: `true`
- evidence_size_bytes: `72,875`
- case_count: `1`
- all_required_topics_covered_case_count: `1`
- trajectory_summary_metrics_present_case_count: `1`

Candidate case:

- goal_sequence: `2`
- first_cmd_near_zero_time: `1779765834.018047`
- join window: `[1779765833.018047, 1779765835.018047]`
- evidence_row_count: `9`

Required topic coverage:

- `/trajectories`: `3`
- `/optimal_trajectory`: `3`
- `/transformed_global_plan`: `3`

Trajectory summary metrics:

- sample_count: `3`
- marker_count_max: `7656.0`
- degenerate_trajectory_count_max: `0.0`
- representative_path_length_max: `null`

Condition:

- `trajectory_evidence_present_without_critic_stats`

## Baseline + candidate aggregate

Command:

```bash
python3 tools/aggregate_phase26s_summary_repeats.py \
  --phase 26T \
  --log-dir log \
  --run-ids phase26r_baseline_summary_run1 phase26r_baseline_summary_run2 phase26r_candidate_summary_run1 \
  --output-json log/phase26t_baseline_candidate_summary_comparison.json
```

Artifact:

- `log/phase26t_baseline_candidate_summary_comparison.json`

Aggregate summary:

- run_count: `3`
- baseline_run_count: `2`
- candidate_run_count: `1`
- all_runs_passed_coverage: `true`
- all_runtime_semantics_ok: `true`
- runtime_cost_weights_by_profile:
  - baseline: `[3.81, 3.81]`
  - candidate: `[2.75]`

Condition counts:

- `trajectory_evidence_present_without_critic_stats`: `3`

Stable condition:

- stable_condition: `trajectory_evidence_present_without_critic_stats`
- stable_condition_count: `3`
- stable_condition_is_specific: `false`

Candidate comparison:

- mode: `evidence_only_not_promotion`
- baseline_top_condition: `trajectory_evidence_present_without_critic_stats`
- candidate_top_condition: `trajectory_evidence_present_without_critic_stats`
- candidate_condition_counts:
  - `trajectory_evidence_present_without_critic_stats`: `1`
- candidate_has_specific_condition: `false`
- candidate_condition_differs_from_baseline: `false`

Trajectory summary metric ranges across all three runs:

- sample_count:
  - min: `2.0`
  - max: `3.0`
  - mean: `2.6666666666666665`
- marker_count_max:
  - min: `7656.0`
  - max: `7656.0`
  - mean: `7656.0`
- degenerate_trajectory_count_max:
  - min: `0.0`
  - max: `0.0`
  - mean: `0.0`
- representative_path_length_max:
  - count: `0`
  - min/max/mean: `null`

## Interpretation

Candidate compact-summary evidence does not introduce a stable, specific, interpretable condition.

Candidate and baseline are identical at the current condition-hypothesis level:

- baseline: `trajectory_evidence_present_without_critic_stats`
- candidate: `trajectory_evidence_present_without_critic_stats`

This means the candidate run also has trajectory/path evidence in the first cmd-near-zero window, but still lacks critic/validator/no-valid-control pathway specificity.

Candidate does not show repeated or even single-run specific condition such as:

- CostCritic collision / near-collision
- PathAlign occupancy conflict
- PathFollow / GoalCritic geometry conflict
- trajectory validator reject
- no valid control / invalid control pathway
- degenerate trajectory summary

The candidate's `degenerate_trajectory_count_max` is `0.0`, same as baseline.

## Decision

Phase27 remains blocked.

Aggregator decision:

- phase27_candidate_signal: `not_supported`
- intervention_allowed: `false`
- candidate_promotion_allowed: `false`
- candidate_rejection_allowed: `false`
- reason: `candidate_condition_not_specific_or_not_different`

Do not from Phase26T:

- enter Phase27
- change branch selection
- tune Nav2/controller params
- promote `2.75`
- reject `2.75`

## Verification

- Phase26T + Phase26S focused tests: `5 passed in 0.19s`
- Candidate bounded smoke: exit `0`
- Aggregator execution: exit `0`
- Candidate coverage: `pass_coverage: true`
- Runtime profile proof: candidate `CostCritic.cost_weight=2.75`
- Cleanup check: no matching ROS/Gazebo/Nav2/recorder processes remained after smoke.

## Next recommendation

Stay diagnostics-only.

Recommended Phase26U:

- Stop collecting more identical compact-summary smokes unless adding new evidence dimensions.
- The current compact evidence has reached a ceiling: it proves trajectory/path evidence is present but not why command goes near-zero.
- Next useful direction should be a new diagnostics-only dimension, for example:
  - controller log parsing for no-valid-control / invalid trajectory strings,
  - MPPI source-level parameter audit for trajectory visualization semantics,
  - or an offline geometry/costmap join around the selected optimal path summary.

Do not proceed to Phase27 until a stable, specific condition is identified and reviewed.
