# Phase26R summary-evidence bounded smoke

Date: 2026-05-26
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## Goal

Phase26R stays analysis-first and diagnostics-only. It does not change branch selection, does not tune Nav2/controller parameters, and does not start Phase27 intervention.

The only goal is to verify that the compact MPPI evidence recorder from Phase26Q can cover the real first `cmd_vel_nav` near-zero ±1s window without producing a large raw MarkerArray artifact.

Required coverage in the first cmd-near-zero ±1s window:

- `/trajectories` summary
- `/optimal_trajectory` summary
- `/transformed_global_plan` summary
- analyzer trajectory_summary metrics
- small `${RUN_ID}_mppi_evidence_summary.jsonl`

## Changes

Added a Phase26R coverage checker:

- `tools/check_phase26r_summary_evidence_coverage.py`

Extended the existing smoke wrapper:

- `tools/run_phase21_controller_diagnostics_smoke.sh`

New run IDs:

- `phase26r_baseline_summary_runN`
- `phase26r_candidate_summary_runN`

These are diagnostics-only aliases for the existing Phase26P MPPI diagnostics profiles, plus Phase26R compact-evidence coverage checks. They do not introduce a new intervention profile.

Added tests:

- `src/tugbot_maze/test/test_phase26r_summary_evidence_smoke.py`

## Bounded smoke command

```bash
PHASE_RUN_MAX_GOALS=2 \
PHASE_RUN_STATE_MAX_SAMPLES=90 \
PHASE_RUN_GOAL_EVENTS_MAX_SAMPLES=70 \
PHASE_RUN_TIMEOUT_SEC=190 \
PHASE_RUN_SNAPSHOT_TIMEOUT_SEC=210 \
bash tools/run_phase21_controller_diagnostics_smoke.sh phase26r_baseline_summary_run1
```

## Runtime profile proof

Runtime `/controller_server` parameter dump confirmed the baseline diagnostics profile preserved the canonical baseline MPPI CostCritic semantics:

- `FollowPath.CostCritic.cost_weight`: `3.81`

This is diagnostics-only evidence collection, not a tuning run.

## Runtime outcome

Run ID: `phase26r_baseline_summary_run1`

Bounded smoke summary:

- final mode: `FAILED_EXHAUSTED`
- goal_count: `2`
- goal_success_count: `1`
- goal_failure_count: `1`
- timeout_cancel_count: `1`
- blocked_branch_count: `0`
- blacklisted_goal_count: `0`
- exit_distance_m: `4.406597239573055`

This bounded smoke result is not used to accept/reject any Nav2 profile.

## Compact evidence artifact size

Artifact:

- `log/phase26r_baseline_summary_run1_mppi_evidence_summary.jsonl`

Size:

- `61,567` bytes

Recorder totals:

- message rows: `143`
- topic_discovered rows: `4`
- summary rows: `1`

Topics recorded:

- `/optimal_trajectory`: `49`
- `/trajectories`: `49`
- `/transformed_global_plan`: `48`
- `/docking_trajectory`: `1`

Summary kinds:

- `/trajectories`: `marker_array_trajectory_summary`
- `/optimal_trajectory`: `path_summary`
- `/transformed_global_plan`: `path_summary`

This passes the Phase26R small-artifact requirement and avoids the Phase26P raw MarkerArray blowup.

## First cmd-near-zero ±1s coverage

Coverage artifact:

- `log/phase26r_baseline_summary_run1_phase26r_summary_evidence_coverage.json`

Result:

- `pass_coverage`: `true`
- case_count: `1`
- all_required_topics_covered_case_count: `1`
- trajectory_summary_metrics_present_case_count: `1`
- evidence_size_bytes: `61,567`
- max_evidence_bytes: `5,000,000`

Analyzed case:

- run_id: `phase26r_baseline_summary_run1`
- goal_sequence: `2`
- first_cmd_near_zero_time: `1779763866.9752023`
- join window: `[1779763865.9752023, 1779763867.9752023]`
- evidence_row_count in window: `6`

Required topic coverage in that window:

- `/trajectories`
  - covered: `true`
  - sample_count: `2`
  - summary_kind: `marker_array_trajectory_summary`
- `/optimal_trajectory`
  - covered: `true`
  - sample_count: `2`
  - summary_kind: `path_summary`
- `/transformed_global_plan`
  - covered: `true`
  - sample_count: `2`
  - summary_kind: `path_summary`

## Analyzer output

Analyzer artifact:

- `log/phase26r_baseline_summary_run1_phase26p_mppi_evidence_analysis.json`

Summary:

- case_count: `1`
- real_mppi_evidence_case_count: `1`
- condition_hypothesis_counts:
  - `trajectory_evidence_present_without_critic_stats`: `1`

Trajectory summary metrics:

- sample_count: `2`
- marker_count_max: `7656.0`
- point_count_max: `0.0`
- degenerate_trajectory_count_max: `0.0`
- representative_path_length_max: `null`

Optimal trajectory metrics:

- sample_count: `2`
- path_sample_count: `2`
- path_displacement_min: `0.00450001984383581`
- zero_displacement_path_sample_count: `2`
- velocity_sample_count: `0`

Critic/validator evidence:

- critic_stats sample_count: `0`
- trajectory_validator sample_count: `0`

## Interpretation

Phase26R successfully verifies the evidence plumbing:

1. Compact summary evidence stays small.
2. The real first cmd-near-zero ±1s window contains all required summary topics:
   - `/trajectories`
   - `/optimal_trajectory`
   - `/transformed_global_plan`
3. Analyzer emits trajectory_summary metrics.
4. Runtime params confirm no semantic tuning was introduced.

However, Phase26R does not locate a stable actionable condition.

What it can say:

- MPPI trajectory/path evidence is available exactly in the first near-zero command window.
- The local evidence classification remains `trajectory_evidence_present_without_critic_stats`.
- critics_stats remains unavailable in the installed Jazzy MPPI build, consistent with Phase26Q.

What it cannot yet prove:

- CostCritic collision / near-collision
- PathAlign occupancy conflict
- PathFollow / GoalCritic geometry conflict
- trajectory validator reject
- no valid control / invalid control pathway

## Decision

Phase27 remains blocked.

Do not from Phase26R:

- change branch selection
- tune Nav2/controller parameters
- promote or reject `CostCritic.cost_weight=2.75`
- infer a stable controller intervention from this single bounded smoke

The Phase26R coverage checker hard-codes:

- `phase27_candidate_signal: not_supported`
- `intervention_allowed: false`

## Verification

TDD RED observed before implementation:

- `test_phase26r_summary_evidence_smoke.py`: initial `3 failed`

GREEN/static verification:

- `python3 -m pytest src/tugbot_maze/test/test_phase26r_summary_evidence_smoke.py -q`
  - `3 passed in 0.06s`
- `python3 -m py_compile tools/check_phase26r_summary_evidence_coverage.py`
  - passed
- `bash -n tools/run_phase21_controller_diagnostics_smoke.sh`
  - passed
- `colcon build --symlink-install --packages-select tugbot_bringup tugbot_navigation tugbot_maze`
  - `3 packages finished [1.07s]`

Runtime cleanup check after smoke:

- no matching ROS/Gazebo/Nav2/recorder process remained.

## Next recommendation

Stay diagnostics-only unless matched repeats show a stable, specific, interpretable condition.

Recommended Phase26S direction:

- Run matched repeat compact-summary smokes, not interventions:
  - at least baseline summary repeat(s)
  - optionally candidate summary repeat(s), still not for promotion
- Aggregate Phase26R-style coverage and trajectory_summary metrics across repeats.
- Only discuss Phase27 if repeated windows show a stable condition such as:
  - degenerate selected/optimal trajectory consistently coincident with near-zero cmd,
  - transformed plan/optimal trajectory collapse near recovery,
  - or a specific controller log pathway consistently aligned with near-zero cmd.
