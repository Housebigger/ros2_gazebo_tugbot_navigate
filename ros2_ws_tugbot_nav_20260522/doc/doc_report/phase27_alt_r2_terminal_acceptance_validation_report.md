# Phase27-alt-R2 Terminal Acceptance Branch Validation Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `PASS_AS_STATIC_REPLAY_BRANCH_COVERAGE_ANALYSIS`
Coverage result: `NOT_COVERED_TERMINAL_ACCEPTANCE_BRANCH`

## Scope

Phase27-alt-R2 validates whether existing Phase27-alt-R1 replay/static logs cover the `terminal_acceptance` branch:

```text
robot_exit_dist <= 0.6m
recent timeout/cancel/failure evidence present
near_exit_fallback event: action=terminal_acceptance
mode transitions to EXIT_REACHED
```

This phase is replay/static first. It does not run a new smoke, does not relax local-cost safety gates, does not modify branch selection, and does not modify Nav2/MPPI/controller parameters.

## R1 status label update

Per human acceptance, Phase27-alt-R1 is recorded as:

```text
PASS_AS_NO_TRIGGER_RUNTIME_VALIDATION
```

## Guardrails held

- Did not enter Phase26Y/26Z.
- Did not fetch `nav2_mppi_controller` source.
- Did not modify Nav2 / MPPI / controller parameter semantics.
- Did not promote or reject `CostCritic.cost_weight=2.75`.
- Did not alter general DFS/topology branch-selection strategy.
- Did not claim MPPI selected-control near-zero root cause.
- This report does not claim MPPI selected-control near-zero root cause; R2 only checks terminal_acceptance branch coverage.
- Did not relax the local-cost safety gate to force a micro-goal.
- `git diff -- src/tugbot_navigation/config | cat` output was empty.

## New artifacts

- Analyzer:
  - `tools/analyze_phase27_alt_terminal_acceptance_coverage.py`
- Tests:
  - `src/tugbot_maze/test/test_phase27_alt_terminal_acceptance_coverage.py`
- Replay/static output:
  - `log/phase27_alt_r2_terminal_acceptance_coverage.json`
- Updated accepted R1 report label:
  - `doc/doc_report/phase27_alt_r1_bounded_runtime_validation_report.md`
- This report:
  - `doc/doc_report/phase27_alt_r2_terminal_acceptance_validation_report.md`

## Replay/static inputs

R2 analyzed existing R1 artifacts only:

- `log/phase27_alt_r1_smoke1_goal_events.jsonl`
- `log/phase27_alt_r1_smoke1_explorer_state.jsonl`

Command:

```bash
python3 tools/analyze_phase27_alt_terminal_acceptance_coverage.py \
  --goal-events log/phase27_alt_r1_smoke1_goal_events.jsonl \
  --explorer-state log/phase27_alt_r1_smoke1_explorer_state.jsonl \
  --output-json log/phase27_alt_r2_terminal_acceptance_coverage.json
```

## Replay/static finding

Analyzer result:

```json
{
  "coverage_status": "not_covered",
  "conclusion": "NOT_COVERED_TERMINAL_ACCEPTANCE_BRANCH",
  "not_covered_reason": "exit_radius_state_seen_but_no_terminal_acceptance_event",
  "terminal_acceptance_event_count": 0
}
```

R1 logs do show the robot entering the terminal radius:

```json
{
  "state_count_at_or_below_terminal_radius": 13,
  "min_exit_distance_m": 0.5408336208400051,
  "first_state_at_or_below_terminal_radius": {
    "mode": "NAVIGATING",
    "exit_distance_m": 0.5876039015539865,
    "elapsed_sec": 179.70400953292847
  },
  "last_state_at_or_below_terminal_radius": {
    "mode": "EXIT_REACHED",
    "exit_distance_m": 0.5408336208400051,
    "elapsed_sec": 187.70488595962524
  }
}
```

But R1 logs do not contain a `near_exit_fallback` event with:

```text
action = terminal_acceptance
fallback_reason = terminal_acceptance_radius
near_exit_fallback_triggered = true
```

Therefore Phase27-alt-R2 cannot claim terminal_acceptance branch coverage from R1 replay/static evidence.

## Important event context from R1

R1 did contain one near-exit fallback evaluation, but it was a micro-goal no-action gate:

```json
{
  "event": "near_exit_fallback",
  "near_exit_fallback_triggered": false,
  "fallback_reason": "micro_goal_local_cost_lethal",
  "action": "no_action",
  "robot_exit_dist": 0.770878138232428,
  "last_nav2_result": "goal_canceled_after_timeout",
  "near_exit_fallback_attempts": 0,
  "near_exit_fallback_max_attempts": 1
}
```

This supports the R1 accepted status `PASS_AS_NO_TRIGGER_RUNTIME_VALIDATION`, not terminal_acceptance coverage.

## Final state from R1 replay/static logs

```json
{
  "final_mode": "EXIT_REACHED",
  "final_exit_distance_m": 0.5408336208400051,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "last_goal_event": "terminal_cancel_result",
  "last_goal_mode": "EXIT_REACHED"
}
```

The robot reached the exit under normal flow. That does not prove the Phase27-alt terminal_acceptance branch executed.

## Runtime smoke recommendation

Because replay/static R1 logs do not cover the branch, the analyzer recommends a future bounded runtime smoke only if human acceptance requests it:

```json
{
  "recommended": true,
  "reason": "R1 replay/static logs do not cover the terminal_acceptance branch",
  "keep_thresholds_default": true,
  "do_not_relax_local_cost_gate": true,
  "do_not_change_branch_selection": true,
  "do_not_change_nav2_mppi_controller_params": true
}
```

No new R2 runtime smoke was executed in this phase.

## Verification

RED:

```text
python3 -m pytest src/tugbot_maze/test/test_phase27_alt_terminal_acceptance_coverage.py -q
3 failed, expected because analyzer did not exist yet
```

GREEN focused:

```text
python3 -m pytest src/tugbot_maze/test/test_phase27_alt_terminal_acceptance_coverage.py -q
3 passed in 0.10s
```

Phase27-alt regression:

```text
python3 -m pytest \
  src/tugbot_maze/test/test_phase27_alt_terminal_acceptance_coverage.py \
  src/tugbot_maze/test/test_phase27_alt_runtime_analysis.py \
  src/tugbot_maze/test/test_phase27_alt_near_exit_fallback.py -q
13 passed in 0.21s
```

Static compile:

```text
python3 -m py_compile \
  tools/analyze_phase27_alt_terminal_acceptance_coverage.py \
  tools/analyze_phase27_alt_fallback_runtime.py \
  src/tugbot_maze/test/test_phase27_alt_terminal_acceptance_coverage.py \
  src/tugbot_maze/tugbot_maze/maze_explorer.py \
  src/tugbot_bringup/launch/tugbot_maze_explore.launch.py
```

Build/test:

```text
colcon build --symlink-install
Summary: 6 packages finished [1.22s]

colcon test --event-handlers console_direct+
colcon test-result --verbose
Summary: 183 tests, 0 errors, 0 failures, 0 skipped
```

Cleanup:

```text
pgrep -af 'ros2 launch|gz sim|controller_server|planner_server|bt_navigator|slam_toolbox|maze_explorer|record_explorer_state_series|run_phase27_alt_r1' | grep -v pgrep || true
```

Final cleanup output was empty.

## Conclusion

Phase27-alt-R2 replay/static validation result:

```text
NOT_COVERED_TERMINAL_ACCEPTANCE_BRANCH
```

R1 logs entered `robot_exit_dist <= 0.6m` and finished with `EXIT_REACHED`, but they do not contain a `terminal_acceptance` fallback event. The only fallback evaluation in R1 was `micro_goal_local_cost_lethal` / `no_action` at `robot_exit_dist=0.770878138232428`.

Therefore terminal_acceptance branch validation remains pending. If continued, the next step should be a bounded runtime smoke designed to observe this branch while keeping default thresholds, preserving local-cost safety gates, preserving DFS/topology branch selection, and leaving Nav2/MPPI/controller params unchanged.

Stop here for human acceptance.
