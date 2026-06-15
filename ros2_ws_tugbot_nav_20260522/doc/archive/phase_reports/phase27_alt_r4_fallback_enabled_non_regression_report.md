# Phase27-alt-R4 Fallback Enabled Non-Regression Repeat Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `PASS_AS_FALLBACK_ENABLED_NON_REGRESSION_REPEAT`
R3 accepted status: `PASS_AS_TARGETED_TERMINAL_ACCEPTANCE_COVERAGE_DESIGN`
R2 coverage result preserved: `NOT_COVERED_TERMINAL_ACCEPTANCE_BRANCH`

## Scope

Phase27-alt-R4 runs two bounded runtime repeats with:

```text
near_exit_fallback_enabled:=true
```

All other Phase27-alt fallback thresholds remain at their defaults. R4 does not
try to force terminal_acceptance runtime coverage. R4 only checks whether enabling
the fallback creates regressions:

- non-near-exit fallback triggers;
- topology pollution;
- blocked/blacklisted counter increases caused by fallback;
- fallback event schema gaps;
- unrecorded final modes / exit distances.

## Guardrails

Held:

- Did not enter Phase26Y/Phase26Z.
- Did not fetch `nav2_mppi_controller` source.
- Did not modify Nav2/MPPI/controller parameter semantics.
- Did not promote or reject `CostCritic=2.75`.
- Did not change the normal DFS/topology branch-selection strategy.
- Did not relax the local-cost safety gate.
- Did not claim MPPI selected-control near-zero root cause.
- Did not force terminal_acceptance runtime coverage.
- Did not use a fallback behavior success label.

## Files added / updated

Added:

- `tools/aggregate_phase27_alt_r4_non_regression.py`
- `src/tugbot_maze/test/test_phase27_alt_r4_non_regression.py`
- `log/phase27_alt_r4_enabled_run1_goal_events.jsonl`
- `log/phase27_alt_r4_enabled_run1_explorer_state.jsonl`
- `log/phase27_alt_r4_enabled_run1_launch.log`
- `log/phase27_alt_r4_enabled_run1_phase27_alt_fallback_analysis.json`
- `log/phase27_alt_r4_enabled_run2_goal_events.jsonl`
- `log/phase27_alt_r4_enabled_run2_explorer_state.jsonl`
- `log/phase27_alt_r4_enabled_run2_launch.log`
- `log/phase27_alt_r4_enabled_run2_phase27_alt_fallback_analysis.json`
- `log/phase27_alt_r4_non_regression_summary.json`

Updated:

- `tools/run_phase27_alt_r1_bounded_smoke.sh`
  - accepts `phase27_alt_r4_enabled_runN` run IDs while preserving the R1 run ID pattern.

## Runtime execution

Bounded smoke wrapper:

```bash
PHASE27_ALT_R1_TIMEOUT_SEC=420 \
PHASE27_ALT_R1_MAX_GOALS=12 \
  bash tools/run_phase27_alt_r1_bounded_smoke.sh <run_id>
```

Runs:

1. `phase27_alt_r4_enabled_run1`
2. `phase27_alt_r4_enabled_run2`

Both explicitly enabled the fallback via launch arg:

```text
near_exit_fallback_enabled:=true
```

No Nav2 config file was changed.

## Aggregated acceptance

From `log/phase27_alt_r4_non_regression_summary.json`:

```json
{
  "no_non_near_exit_fallback_trigger": true,
  "topology_non_pollution": true,
  "blocked_blacklisted_not_increased_by_fallback": true,
  "all_fallback_events_schema_complete": true,
  "final_modes_and_exit_distances_recorded": true,
  "src_tugbot_navigation_config_no_diff": true
}
```

R4 conclusion:

```text
PASS_AS_FALLBACK_ENABLED_NON_REGRESSION_REPEAT
```

## Per-run results

### phase27_alt_r4_enabled_run1

Artifacts:

- `log/phase27_alt_r4_enabled_run1_goal_events.jsonl`
- `log/phase27_alt_r4_enabled_run1_explorer_state.jsonl`
- `log/phase27_alt_r4_enabled_run1_phase27_alt_fallback_analysis.json`

Summary:

```json
{
  "final_mode": "NAVIGATING",
  "final_exit_distance_m": 0.7816547799762239,
  "fallback_event_count": 2,
  "fallback_triggered_count": 0,
  "fallback_actions": {"no_action": 2},
  "fallback_reasons": {"micro_goal_local_cost_lethal": 2},
  "topology_non_pollution": true,
  "last_blocked_branch_count": 0,
  "last_blacklisted_goal_count": 0,
  "schema_complete": true
}
```

Interpretation:

- No fallback action was triggered.
- Two near-exit fallback evaluations were safely rejected by default local-cost gate:
  - `micro_goal_local_cost_lethal`.
- No non-near-exit fallback trigger occurred.
- No topology pollution occurred.
- Run ended bounded at goal budget while still `NAVIGATING`, with final exit distance recorded.
- This is evidence, not a prompt for tuning.

### phase27_alt_r4_enabled_run2

Artifacts:

- `log/phase27_alt_r4_enabled_run2_goal_events.jsonl`
- `log/phase27_alt_r4_enabled_run2_explorer_state.jsonl`
- `log/phase27_alt_r4_enabled_run2_phase27_alt_fallback_analysis.json`

Summary:

```json
{
  "final_mode": "FAILED_EXHAUSTED",
  "final_exit_distance_m": 0.7034997055891423,
  "fallback_event_count": 1,
  "fallback_triggered_count": 1,
  "fallback_actions": {"micro_goal": 1},
  "fallback_reasons": {"micro_goal_candidate": 1},
  "topology_non_pollution": true,
  "last_blocked_branch_count": 0,
  "last_blacklisted_goal_count": 0,
  "schema_complete": true
}
```

Interpretation:

- One near-exit micro-goal fallback was triggered within near-exit radius:
  - `robot_exit_dist = 0.7570325215231576` in the per-run fallback analysis.
- No non-near-exit fallback trigger occurred.
- No topology pollution occurred.
- blocked/blacklisted counters remained 0.
- The run ended `FAILED_EXHAUSTED` at bounded goal budget with final exit distance recorded.
- Per R4 rules, this is recorded as evidence only. No tuning or parameter change is proposed.
- The aggregate data does not show fallback-caused topology pollution. It also does not prove the final mode was caused by fallback; that would require a separate diagnostics phase if desired.

## Final modes / exit distances

```json
{
  "final_modes": {
    "NAVIGATING": 1,
    "FAILED_EXHAUSTED": 1
  },
  "final_exit_distances_m": {
    "phase27_alt_r4_enabled_run1": 0.7816547799762239,
    "phase27_alt_r4_enabled_run2": 0.7034997055891423
  }
}
```

Neither repeat reached `EXIT_REACHED` inside the bounded smoke. This is not used
to tune Nav2, branch selection, fallback thresholds, or the local-cost gate.

## Verification

TDD:

- RED: `src/tugbot_maze/test/test_phase27_alt_r4_non_regression.py` failed because the aggregator did not exist.
- GREEN: R4 focused tests passed after adding the aggregator.

Commands verified:

```bash
python3 -m pytest src/tugbot_maze/test/test_phase27_alt_r4_non_regression.py -q
python3 -m pytest \
  src/tugbot_maze/test/test_phase27_alt_r4_non_regression.py \
  src/tugbot_maze/test/test_phase27_alt_r3_decision_path_coverage.py \
  src/tugbot_maze/test/test_phase27_alt_terminal_acceptance_coverage.py \
  src/tugbot_maze/test/test_phase27_alt_runtime_analysis.py \
  src/tugbot_maze/test/test_phase27_alt_near_exit_fallback.py -q
python3 -m py_compile \
  tools/aggregate_phase27_alt_r4_non_regression.py \
  tools/analyze_phase27_alt_fallback_runtime.py \
  tools/analyze_phase27_alt_r3_decision_path_coverage.py \
  src/tugbot_maze/test/test_phase27_alt_r4_non_regression.py \
  src/tugbot_maze/tugbot_maze/maze_explorer.py
bash -n tools/run_phase27_alt_r1_bounded_smoke.sh tools/run_phase21_controller_diagnostics_smoke.sh
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Observed verification results:

- Focused R4: `3 passed in 0.10s`
- Phase27-alt regression: `20 passed in 0.37s`
- `py_compile`: passed
- `bash -n`: passed
- `colcon build --symlink-install`: passed, 6 packages finished
- `colcon test-result --verbose`: `Summary: 190 tests, 0 errors, 0 failures, 0 skipped`
- cleanup process check: empty
- `git diff -- src/tugbot_navigation/config | cat`: empty

## Conclusion

Phase27-alt-R4 result:

```text
PASS_AS_FALLBACK_ENABLED_NON_REGRESSION_REPEAT
```

The two bounded repeats do not show non-near-exit fallback triggers, topology
pollution, fallback-caused blocked/blacklist counter increases, or fallback event
schema gaps. Final modes and exit distances were recorded. Since neither run
reached `EXIT_REACHED`, those outcomes are retained as evidence only; R4 does not
recommend tuning, does not change parameters, and does not interpret MPPI
selected-control near-zero root cause.
