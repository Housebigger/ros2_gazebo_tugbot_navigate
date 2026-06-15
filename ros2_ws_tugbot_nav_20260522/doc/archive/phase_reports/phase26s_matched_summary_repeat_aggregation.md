# Phase26S matched repeat compact-summary aggregation

Date: 2026-05-26
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## Goal

Phase26S remains diagnostics-only. It does not enter intervention, does not tune Nav2/controller parameters, does not change branch selection, and does not promote/reject the `CostCritic.cost_weight=2.75` candidate.

Goal: run matched repeat compact-summary smokes and aggregate Phase26R-style coverage + trajectory_summary metrics. Only matched repeats with a stable, specific, interpretable condition may justify a later human review discussion about Phase27.

## New tool

Added:

- `tools/aggregate_phase26s_summary_repeats.py`

It aggregates per-run artifacts:

- `${RUN_ID}_phase26r_summary_evidence_coverage.json`
- `${RUN_ID}_phase26p_mppi_evidence_analysis.json`
- `${RUN_ID}_runtime_params/controller_server_summary.json`
- `${RUN_ID}_mppi_evidence_summary.jsonl`

It reports:

- coverage pass/fail by run
- runtime CostCritic semantics by profile
- condition_hypothesis counts
- stable condition and whether it is specific/actionable enough for review
- trajectory_summary metric ranges
- guardrails that keep Phase27 blocked unless a stable specific condition exists and receives human review

## TDD

RED observed before implementation:

- `python3 -m pytest src/tugbot_maze/test/test_phase26s_summary_repeat_aggregator.py -q`
- initial result: `3 failed`
- expected cause: `tools/aggregate_phase26s_summary_repeats.py` did not exist

GREEN:

- `3 passed in 0.11s`

## Runtime repeats

Used existing Phase26R run:

- `phase26r_baseline_summary_run1`

Ran matched baseline repeat:

```bash
PHASE_RUN_MAX_GOALS=2 \
PHASE_RUN_STATE_MAX_SAMPLES=90 \
PHASE_RUN_GOAL_EVENTS_MAX_SAMPLES=70 \
PHASE_RUN_TIMEOUT_SEC=190 \
PHASE_RUN_SNAPSHOT_TIMEOUT_SEC=210 \
bash tools/run_phase21_controller_diagnostics_smoke.sh phase26r_baseline_summary_run2
```

Then aggregated:

```bash
python3 tools/aggregate_phase26s_summary_repeats.py \
  --log-dir log \
  --run-ids phase26r_baseline_summary_run1 phase26r_baseline_summary_run2 \
  --output-json log/phase26s_baseline_summary_repeat_aggregate.json
```

## Run-level results

### phase26r_baseline_summary_run1

Runtime semantics:

- profile: baseline
- runtime `FollowPath.CostCritic.cost_weight`: `3.81`
- runtime_semantics_ok: `true`

Coverage:

- pass_coverage: `true`
- evidence_size_bytes: `61,567`
- all required topics covered: `true`

Window sample counts:

- `/trajectories`: `2`
- `/optimal_trajectory`: `2`
- `/transformed_global_plan`: `2`

Trajectory summary metrics:

- sample_count: `2`
- marker_count_max: `7656.0`
- degenerate_trajectory_count_max: `0.0`
- representative_path_length_max: `null`

Condition:

- `trajectory_evidence_present_without_critic_stats`

### phase26r_baseline_summary_run2

Runtime semantics:

- profile: baseline
- runtime `FollowPath.CostCritic.cost_weight`: `3.81`
- runtime_semantics_ok: `true`

Coverage:

- pass_coverage: `true`
- evidence_size_bytes: `72,895`
- all required topics covered: `true`

Window sample counts:

- `/trajectories`: `3`
- `/optimal_trajectory`: `2`
- `/transformed_global_plan`: `2`

Trajectory summary metrics:

- sample_count: `3`
- marker_count_max: `7656.0`
- degenerate_trajectory_count_max: `0.0`
- representative_path_length_max: `null`

Condition:

- `trajectory_evidence_present_without_critic_stats`

## Aggregated result

Artifact:

- `log/phase26s_baseline_summary_repeat_aggregate.json`

Summary:

- run_count: `2`
- baseline_run_count: `2`
- candidate_run_count: `0`
- all_runs_passed_coverage: `true`
- all_runtime_semantics_ok: `true`
- runtime_cost_weights_by_profile:
  - baseline: `[3.81, 3.81]`
  - candidate: `[]`
- condition_hypothesis_counts:
  - `trajectory_evidence_present_without_critic_stats`: `2`
- stable_condition: `trajectory_evidence_present_without_critic_stats`
- stable_condition_count: `2`
- stable_condition_is_specific: `false`

Trajectory summary metric ranges:

- sample_count:
  - min: `2.0`
  - max: `3.0`
  - mean: `2.5`
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

Phase26S validates the compact-summary repeat workflow:

1. Both baseline compact-summary runs passed coverage.
2. Both preserved runtime baseline semantics: `CostCritic.cost_weight=3.81`.
3. Both captured `/trajectories`, `/optimal_trajectory`, and `/transformed_global_plan` around first cmd-near-zero.
4. Both produced small artifacts: `61,567` and `72,895` bytes.
5. Both produced the same broad condition: `trajectory_evidence_present_without_critic_stats`.

However, the repeated condition is not specific enough to justify intervention. It means evidence exists but lacks critic/validator/no-valid-control specificity. It does not identify:

- CostCritic collision / near-collision
- PathAlign occupancy conflict
- PathFollow / GoalCritic geometry conflict
- trajectory validator reject
- no valid control / invalid control pathway

The stable metrics also do not indicate degenerate trajectory summaries:

- degenerate_trajectory_count_max: `0.0` in both runs

## Decision

Phase27 remains blocked.

Aggregator decision:

- phase27_candidate_signal: `not_supported`
- intervention_allowed: `false`
- reason: `no_stable_specific_condition`

Do not from Phase26S:

- change branch selection
- tune Nav2/controller parameters
- promote/reject `CostCritic.cost_weight=2.75`
- infer intervention from baseline-only repeat summary evidence

## Verification

- Phase26S focused tests: `3 passed in 0.11s`
- Aggregator runtime execution: exit `0`
- Both compact-summary runs produced coverage artifacts with `pass_coverage: true`
- Runtime cleanup check: no matching ROS/Gazebo/Nav2/recorder processes remained after the repeat smoke.

## Next recommendation

Stay diagnostics-only.

Recommended Phase26T:

- Optionally run candidate compact-summary repeat(s), but only for evidence comparison, not promotion/rejection.
- Aggregate baseline + candidate summary evidence.
- Compare whether the broad condition remains identical or whether candidate introduces a specific repeated condition.
- Still block Phase27 unless matched repeats show a stable, specific, interpretable condition and human review accepts it as sufficient.
