# Phase27-alt-R1 Bounded Runtime Validation Smoke Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Run id: `phase27_alt_r1_smoke1`
Status: `PASS_AS_NO_TRIGGER_RUNTIME_VALIDATION`

## Scope

Phase27-alt-R1 is a bounded runtime validation smoke for the Phase27-alt default-off near-exit fallback. The run explicitly enabled the fallback:

```text
near_exit_fallback_enabled:=true
```

All other Phase27-alt fallback thresholds stayed at their implementation defaults. This phase only validates runtime event coverage, gate behavior, topology non-pollution, and final-mode / exit-distance evidence. It does not explain MPPI selected-control near-zero root cause.

## Guardrails held

- Did not enter Phase26Y/26Z.
- Did not fetch or overlay `nav2_mppi_controller` source.
- Did not modify Nav2 / MPPI / controller parameter semantics.
- Did not promote or reject `CostCritic.cost_weight=2.75`.
- Did not change the general DFS/topology branch-selection strategy.
- Did not claim MPPI selected-control near-zero root cause.
- `git diff -- src/tugbot_navigation/config | cat` produced no output during post-run checks.

## New/updated artifacts

- Runtime wrapper:
  - `tools/run_phase27_alt_r1_bounded_smoke.sh`
- Analyzer:
  - `tools/analyze_phase27_alt_fallback_runtime.py`
- Tests:
  - `src/tugbot_maze/test/test_phase27_alt_runtime_analysis.py`
- Runtime logs:
  - `log/phase27_alt_r1_smoke1_goal_events.jsonl`
  - `log/phase27_alt_r1_smoke1_explorer_state.jsonl`
  - `log/phase27_alt_r1_smoke1_launch.log`
- Analysis output:
  - `log/phase27_alt_r1_smoke1_phase27_alt_fallback_analysis.json`
- This report:
  - `doc/doc_report/phase27_alt_r1_bounded_runtime_validation_report.md`

## Runtime command

```bash
PHASE27_ALT_R1_TIMEOUT_SEC=420 \
PHASE27_ALT_R1_MAX_GOALS=12 \
bash tools/run_phase27_alt_r1_bounded_smoke.sh phase27_alt_r1_smoke1
```

The wrapper records `/maze/goal_events` and `/maze/explorer_state`, launches headless Gazebo/Nav2/maze explorer with `near_exit_fallback_enabled:=true`, waits for terminal state or bounded timeout, stops launch/recorders, and runs the Phase27-alt-R1 analyzer.

## Analyzer result summary

Source: `log/phase27_alt_r1_smoke1_phase27_alt_fallback_analysis.json`

```json
{
  "conclusion": "runtime_no_fallback_trigger_observed",
  "counts": {
    "goal_event_rows": 22,
    "explorer_state_rows": 261
  },
  "fallback": {
    "event_count": 1,
    "triggered_count": 0,
    "no_action_event_count": 1,
    "no_trigger_reason": "micro_goal_local_cost_lethal",
    "min_robot_exit_dist_observed": 0.770878138232428
  },
  "final_state": {
    "final_mode": "EXIT_REACHED",
    "final_exit_distance_m": 0.5408336208400051,
    "goal_count": 9,
    "goal_success_count": 5,
    "goal_failure_count": 3,
    "blocked_branch_count": 0,
    "blacklisted_goal_count": 0
  },
  "topology_non_pollution": {
    "passed": true,
    "first_blocked_branch_count": 0,
    "last_blocked_branch_count": 0,
    "first_blacklisted_goal_count": 0,
    "last_blacklisted_goal_count": 0,
    "violations": []
  }
}
```

## Event coverage

Fallback diagnostic fields were present in `/maze/goal_events`:

- `near_exit_fallback_triggered`
- `fallback_reason`
- `robot_exit_dist`
- `cmd_near_zero_duration`
- `last_nav2_result`
- `robot_to_path_distance`
- `action`

Analyzer schema result:

```text
required_fields_present: true
triggered_event_required_fields_present: true
```

## Gate behavior observed

One `near_exit_fallback` event was recorded near the exit:

```json
{
  "event": "near_exit_fallback",
  "near_exit_fallback_triggered": false,
  "fallback_reason": "micro_goal_local_cost_lethal",
  "action": "no_action",
  "robot_exit_dist": 0.770878138232428,
  "last_nav2_result": "goal_canceled_after_timeout",
  "near_exit_fallback_attempts": 0,
  "near_exit_fallback_max_attempts": 1,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0
}
```

Associated diagnostics explain why the fallback did not trigger:

- Geometry was not the blocker:
  - `target_cell_occupancy: 0`
  - `line_of_sight_occupied_count: 0`
  - `line_of_sight_unknown_count: 0`
  - `target_clearance_m: 0.3500000052154064`
- Local cost blocked the micro-goal:
  - `dispatch_target_local_cost: 99`
  - `dispatch_target_local_cost_max_radius: 100`
  - `dispatch_path_local_cost_max: 99`
  - `dispatch_path_local_cost_mean: 94.0`
  - `dispatch_local_cost_sample_age_sec: 0.297`
  - `dispatch_local_cost_sample_coverage_ratio: 1.0`

Interpretation for this phase only: Phase27-alt runtime gate coverage is working, and the default safety feasibility gate correctly refused a near-exit micro-goal when local-cost evidence was lethal. This is a no-trigger validation, not a fallback-success validation.

## Topology non-pollution

No near-exit micro-goal was dispatched in this run. The analyzer still checked counters across the run:

```text
blocked_branch_count: 0 -> 0
blacklisted_goal_count: 0 -> 0
topology_non_pollution.passed: true
```

There is no evidence of topology pollution in this bounded smoke.

## Final mode and exit-distance evidence

The run reached the configured exit region without needing to execute a fallback micro-goal:

```text
final_mode: EXIT_REACHED
final_exit_distance_m: 0.5408336208400051
```

This is below the normal exit radius / terminal region and is runtime evidence that the task ended at the exit coordinates. It is not evidence about MPPI selected-control near-zero root cause.

## Verification

TDD and focused checks:

```text
python3 -m pytest src/tugbot_maze/test/test_phase27_alt_runtime_analysis.py -q
3 passed in 0.10s
```

Relevant Phase27-alt checks:

```text
python3 -m pytest \
  src/tugbot_maze/test/test_phase27_alt_runtime_analysis.py \
  src/tugbot_maze/test/test_phase27_alt_near_exit_fallback.py -q
10 passed in 0.10s
```

Relevant regression subset:

```text
python3 -m pytest \
  src/tugbot_maze/test/test_maze_explorer_phase14.py \
  src/tugbot_maze/test/test_maze_explorer_phase19.py \
  src/tugbot_maze/test/test_phase26x_selected_control_debug.py -q
11 passed in 0.07s
```

Static checks:

```text
python3 -m py_compile \
  tools/analyze_phase27_alt_fallback_runtime.py \
  tools/record_explorer_state_series.py \
  src/tugbot_maze/test/test_phase27_alt_runtime_analysis.py \
  src/tugbot_maze/tugbot_maze/maze_explorer.py \
  src/tugbot_bringup/launch/tugbot_maze_explore.launch.py

bash -n tools/run_phase27_alt_r1_bounded_smoke.sh
bash -n tools/run_phase21_controller_diagnostics_smoke.sh
```

Build/test:

```text
colcon build --symlink-install
Summary: 6 packages finished [1.24s]

colcon test --event-handlers console_direct+
colcon test-result --verbose
Summary: 180 tests, 0 errors, 0 failures, 0 skipped
```

Cleanup/process check:

```text
pgrep -af 'ros2 launch|gz sim|controller_server|planner_server|bt_navigator|slam_toolbox|maze_explorer|record_explorer_state_series|run_phase27_alt_r1' | grep -v pgrep || true
```

Output was empty in the final cleanup check.

Nav2 config guard:

```text
git diff -- src/tugbot_navigation/config | cat
```

Output was empty.

## Conclusion

Phase27-alt-R1 produced bounded runtime evidence for:

- `/maze/goal_events` fallback field coverage: PASS
- near-exit fallback gate behavior: PASS, observed `micro_goal_local_cost_lethal` no-action gate
- topology non-pollution counters: PASS, `blocked_branch_count=0`, `blacklisted_goal_count=0`
- final mode / exit-distance evidence: PASS, `EXIT_REACHED`, `0.5408m`
- Nav2 config non-modification: PASS
- MPPI root-cause claim: NOT EVALUATED

Phase27-alt-R1 result: `runtime_no_fallback_trigger_observed` with complete no-trigger evidence. The fallback was explicitly enabled, evaluated near the exit, and refused to act because default safety local-cost gates detected lethal/high-cost micro-goal conditions. The run still reached the exit under the normal exploration/Nav2 flow.

Stop here for human acceptance before any further runtime validation.
