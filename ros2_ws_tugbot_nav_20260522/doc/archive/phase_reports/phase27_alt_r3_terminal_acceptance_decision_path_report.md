# Phase27-alt-R3 Terminal Acceptance Decision-Path Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `PASS_AS_TARGETED_TERMINAL_ACCEPTANCE_COVERAGE_DESIGN`
R2 accepted status: `PASS_AS_STATIC_REPLAY_BRANCH_COVERAGE_ANALYSIS`
R2 coverage result preserved: `NOT_COVERED_TERMINAL_ACCEPTANCE_BRANCH`

## Scope

Phase27-alt-R3 is a static/replay decision-path audit. It explains why R1 could
enter `robot_exit_dist <= 0.6m` and still emit no runtime `terminal_acceptance`
fallback event. It also decides whether another runtime smoke is needed to cover
that branch.

No new Gazebo/Nav2 runtime was started in R3.

## Guardrails

Held:

- Did not enter Phase26Y/Phase26Z.
- Did not fetch `nav2_mppi_controller` source.
- Did not modify Nav2/MPPI/controller parameter semantics.
- Did not promote or reject `CostCritic=2.75`.
- Did not change the normal DFS/topology branch-selection strategy.
- Did not relax the local-cost safety gate.
- Did not claim MPPI selected-control near-zero root cause.
- Did not use a success label for fallback behavior.

## Inputs

- Source decision path:
  - `src/tugbot_maze/tugbot_maze/maze_explorer.py`
- R2 replay/static coverage JSON:
  - `log/phase27_alt_r2_terminal_acceptance_coverage.json`
- R3 analyzer output:
  - `log/phase27_alt_r3_decision_path_coverage.json`

## R2 acceptance label update

Per human acceptance, the R2 report now uses:

```text
PASS_AS_STATIC_REPLAY_BRANCH_COVERAGE_ANALYSIS
```

while preserving the branch coverage result:

```text
NOT_COVERED_TERMINAL_ACCEPTANCE_BRANCH
```

This is intentionally a branch-coverage result, not a fallback behavior success label.

## Decision-path audit

`_explore_once()` orders the relevant checks as follows:

1. Terminal-state guard returns immediately if already in `EXIT_REACHED` or
   `FAILED_EXHAUSTED`.
2. Map/TF prerequisites are checked.
3. The normal terminal monitor runs:
   - `if self._exit_reached(robot_pose):`
   - `_enter_terminal_state(EXIT_REACHED, terminal_reason='exit_reached', ...)`
   - publish state and return.
4. If a goal is active, the node handles active-goal state / timeout and returns.
5. If goal-settle cooldown is active, the node returns.
6. Goal budget and Nav2 server checks run.
7. Only then does the fallback decision run:
   - `self.mode = AT_NODE_ANALYZE`
   - `if self._maybe_apply_near_exit_fallback(robot_pose): ... return`
8. Normal `_analyze_and_dispatch(robot_pose)` runs only after fallback declined.

Analyzer booleans:

```json
{
  "normal_exit_monitor_precedes_near_exit_fallback": true,
  "near_exit_fallback_called_only_after_no_active_goal_and_settle": true,
  "near_exit_fallback_precedes_normal_branch_dispatch": true,
  "normal_exit_path_cancels_active_goal_before_fallback": true,
  "fallback_terminal_acceptance_requires_recent_problem_evidence": true,
  "fallback_terminal_acceptance_contract_present": true,
  "terminal_acceptance_action_enters_exit_reached": true,
  "failure_path_sets_last_failure_reason": true,
  "timeout_cancel_result_sets_recent_failure_reason": true
}
```

## Why R1 entered <=0.6m without fallback terminal_acceptance event

R1/R2 replay facts:

- R1 state entered terminal radius:
  - `state_count_at_or_below_terminal_radius = 13`
  - `min_exit_distance_m = 0.5408336208400051`
- R1 final mode:
  - `EXIT_REACHED`
- R2 branch coverage result:
  - `NOT_COVERED_TERMINAL_ACCEPTANCE_BRANCH`
- Terminal acceptance fallback event count:
  - `0`

Decision-path interpretation:

```text
robot_entered_terminal_radius_while_goal_active_normal_exit_monitor_executed_before_fallback_decision
```

That is, ordinary runtime radius crossing is expected to be handled by the normal
exit monitor before the fallback terminal_acceptance branch is reachable. The
fallback branch is evaluated only later, when no active goal is in progress and
the node is about to analyze/dispatch again. Therefore, R1 naturally bypassed the
fallback terminal_acceptance event while still correctly reaching `EXIT_REACHED`.

## Terminal acceptance fallback branch nature

The fallback `terminal_acceptance` branch is an abnormal/recovery branch, not the
ordinary successful exit path. It requires all of the following before it can run:

- `near_exit_fallback_enabled = true`;
- robot pose within `near_exit_fallback_trigger_radius_m`;
- clean topology if required:
  - `blocked_branch_count == 0`;
  - blacklist length `== 0`;
- recent problem evidence or command stall evidence:
  - `GOAL_TIMEOUT`, `BLOCKED_NAV2`, `GOAL_CANCELED_AFTER_TIMEOUT`, or
    `GOAL_REJECTED`; or
  - cmd near-zero duration above threshold;
- `robot_exit_dist <= near_exit_terminal_acceptance_radius_m`;
- execution reaches `AT_NODE_ANALYZE`, i.e. after active-goal handling and settle
  cooldown, and before the normal branch dispatcher.

If a robot simply drives into the exit radius during an active Nav2 goal, the
normal monitor is designed to win first and enter `EXIT_REACHED`.

## Host-level terminal_acceptance coverage

R3 supplemented/reviewed host-level tests to ensure the branch contract is
covered statically:

- `near_exit_fallback_enabled` exists and defaults false;
- terminal radius is default `0.6`;
- decision helper includes recent failure evidence:
  - `GOAL_TIMEOUT`, `BLOCKED_NAV2`, `GOAL_CANCELED_AFTER_TIMEOUT`,
    `GOAL_REJECTED`;
- clean-topology gates are present;
- terminal branch condition is present:
  - `robot_exit_dist <= self.near_exit_terminal_acceptance_radius_m`;
- branch event payload is present:
  - `near_exit_fallback_triggered = true`;
  - `fallback_reason = terminal_acceptance_radius`;
  - `action = terminal_acceptance`;
- action implementation publishes the fallback event and calls:
  - `_enter_terminal_state(EXIT_REACHED, terminal_reason='near_exit_terminal_acceptance', ...)`.

## Runtime coverage recommendation

R3 does not recommend a new runtime execution now.

Analyzer result:

```json
{
  "targeted_runtime_needed": false,
  "execute_runtime_now": false,
  "recommended_path": "host_level_coverage_sufficient_for_abnormal_branch",
  "reason": "normal exit monitor naturally handles ordinary radius crossing before fallback decision path can run"
}
```

If a future reviewer still asks for runtime coverage, the required window would
be narrow and abnormal:

```text
robot already within <=0.6m after recent timeout/cancel/failure, no active goal,
after settle cooldown, before normal exit monitor has already entered EXIT_REACHED
```

Such a runtime scenario must still keep default thresholds, must not relax the
local-cost gate, must not change Nav2/MPPI/controller parameters, and must not
change branch selection. The audit indicates that natural coverage may be hard
because the normal exit monitor is expected to win ordinary runtime races.

## Files added/updated

Added:

- `tools/analyze_phase27_alt_r3_decision_path_coverage.py`
- `src/tugbot_maze/test/test_phase27_alt_r3_decision_path_coverage.py`
- `log/phase27_alt_r3_decision_path_coverage.json`

Updated:

- `src/tugbot_maze/test/test_phase27_alt_near_exit_fallback.py`
- `doc/doc_report/phase27_alt_r2_terminal_acceptance_validation_report.md`

## Verification summary

Focused R3/R27 tests initially failed RED because the analyzer was missing and
host terminal-acceptance contract coverage was incomplete. After implementation,
focused tests passed.

Final verification is recorded in the session result and should include:

- focused tests;
- Phase27-alt regression;
- `py_compile`;
- `colcon build --symlink-install`;
- `colcon test --event-handlers console_direct+`;
- `colcon test-result --verbose`;
- cleanup process check;
- Nav2 config diff check.

## Conclusion

Phase27-alt-R3 result:

```text
PASS_AS_TARGETED_TERMINAL_ACCEPTANCE_COVERAGE_DESIGN
```

R2 coverage result remains:

```text
NOT_COVERED_TERMINAL_ACCEPTANCE_BRANCH
```

R1 entered the terminal radius but did not emit a terminal_acceptance fallback
event because the normal terminal monitor precedes the fallback decision path and
is expected to handle ordinary successful radius crossings first. The
fallback terminal_acceptance branch is therefore an abnormal recovery branch; R3
recommends host-level coverage as sufficient unless a future reviewer explicitly
requires a designed abnormal runtime scenario.

This report does not claim MPPI selected-control near-zero root cause.
