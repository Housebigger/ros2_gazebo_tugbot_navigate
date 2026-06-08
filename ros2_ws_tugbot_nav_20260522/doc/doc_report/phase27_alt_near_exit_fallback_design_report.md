# Phase27-alt Near-Exit Fallback Design Report

Date: 2026-05-27
Workspace: ros2_ws_tugbot_nav_20260522
Status: implemented_static_verified_pending_runtime

## Summary

Phase27-alt route B is now implemented as a default-off, upper-layer `maze_explorer` fallback for near-exit stalls. It is intentionally not a continuation of Phase26Y/26Z and does not explain the MPPI selected-control near-zero root cause.

The fallback is scoped to the engineering goal: Tugbot autonomously explores in ROS 2 Jazzy + Gazebo and reaches the configured exit coordinates. It uses the configured exit coordinates as the success criterion and does not use the maze image as a runtime path answer.

## Guardrails preserved

- No `nav2_mppi_controller` source fetch.
- No MPPI/Nav2/controller parameter semantic changes.
- No promotion or rejection of `CostCritic.cost_weight=2.75`.
- No change to the normal DFS/topology branch-selection strategy.
- No long Gazebo/Nav2 run in this phase.
- Fallback is default-off via `near_exit_fallback_enabled:=false`.
- Fallback decisions are emitted to `/maze/goal_events` as structured diagnostics.

## Design proposal

The design-first proposal was written before implementation:

```text
doc/doc_proposal/phase27_alt_near_exit_fallback_design.md
```

It defines trigger gates, terminal acceptance behavior, bounded micro-goal behavior, event schema, rollback, and verification plan.

## Files changed

```text
src/tugbot_maze/tugbot_maze/maze_explorer.py
src/tugbot_bringup/launch/tugbot_maze_explore.launch.py
src/tugbot_maze/test/test_phase27_alt_near_exit_fallback.py
doc/doc_proposal/phase27_alt_near_exit_fallback_design.md
doc/doc_report/phase27_alt_near_exit_fallback_design_report.md
README.md
```

No files under `src/tugbot_navigation/config/*.yaml` were changed.

## Implementation details

### Runtime switch and bounds

`MazeExplorer` now declares these Phase27-alt parameters:

```text
near_exit_fallback_enabled=false
near_exit_fallback_trigger_radius_m=0.9
near_exit_terminal_acceptance_radius_m=0.6
near_exit_micro_goal_min_step_m=0.20
near_exit_micro_goal_max_step_m=0.35
near_exit_fallback_max_attempts=1
near_exit_fallback_require_clean_topology=true
near_exit_fallback_require_path_alignment=true
near_exit_fallback_robot_to_path_max_m=0.15
near_exit_fallback_cmd_near_zero_min_sec=3.0
```

The launch file exposes the same parameters and keeps the enable switch defaulted to `false`.

### Decision helper

Added `_near_exit_fallback_decision(...)` as a host-testable decision function. It returns a dictionary with:

```text
near_exit_fallback_triggered
fallback_reason
robot_exit_dist
cmd_near_zero_duration
last_nav2_result
robot_to_path_distance
action
near_exit_fallback_enabled
near_exit_fallback_attempts
near_exit_fallback_max_attempts
```

The current minimal runtime call uses known in-node evidence:

- robot pose / exit distance;
- `last_failure_reason` from the previous Nav2 timeout/failure/cancel path;
- topology counters from `MazeTopology`.

`cmd_near_zero_duration` and `robot_to_path_distance` are supported in the decision/event schema and remain `null` in this minimal in-node implementation unless future runtime diagnostics pass them in. This avoids fabricating evidence.

### Trigger gates

The fallback can trigger only when all relevant gates pass:

1. `near_exit_fallback_enabled == true`.
2. `robot_exit_dist <= 0.9m`.
3. Recent problem evidence exists through either:
   - `last_failure_reason in {GOAL_TIMEOUT, BLOCKED_NAV2, GOAL_CANCELED_AFTER_TIMEOUT, GOAL_REJECTED}`; or
   - future `cmd_near_zero_duration >= near_exit_fallback_cmd_near_zero_min_sec`.
4. If `near_exit_fallback_require_clean_topology == true`:
   - `blocked_branch_count == 0`;
   - `blacklisted_goal_count == 0`.
5. If path-distance evidence is available and path-alignment gate is enabled:
   - `robot_to_path_distance <= 0.15m`.

### Action A: terminal acceptance

If gates pass and:

```text
robot_exit_dist <= near_exit_terminal_acceptance_radius_m
```

then the node emits a `near_exit_fallback` event with:

```text
action=terminal_acceptance
fallback_reason=terminal_acceptance_radius
```

Then it enters:

```text
final mode = EXIT_REACHED
terminal_reason = near_exit_terminal_acceptance
```

This uses the existing terminal-state path, including active-goal cancellation if needed.

### Action B: bounded micro-goal

If gates pass and:

```text
0.6m < robot_exit_dist <= 0.9m
near_exit_fallback_attempts < near_exit_fallback_max_attempts
```

then the node computes a short target toward the configured exit vector using:

```text
step = clamp(distance_to_exit - terminal_acceptance_radius, 0.20, 0.35)
```

The target is accepted only if existing map/local-cost diagnostics pass:

- target occupancy is not occupied;
- line of sight has no occupied cells;
- target clearance is at least `clearance_radius_m` where available;
- local target/path cost is not lethal when local costmap evidence is available.

If feasible, the node emits `near_exit_fallback` with:

```text
action=micro_goal
fallback_reason=micro_goal_candidate
goal_kind=near_exit_micro_goal
```

Then it sends one bounded `NavigateToPose` goal with `goal_kind='near_exit_micro_goal'`.

### Topology non-pollution

Micro-goal success returns to the normal exit-check / exploration loop.

Micro-goal failure does not call:

```text
record_branch_failure
mark_branch_state
blacklist updates
```

It preserves DFS topology state and uses the ordinary settle cooldown.

### Event schema

All normal goal events now include Phase27-alt fields, defaulting to false/null when not a fallback event:

```text
near_exit_fallback_triggered
fallback_reason
robot_exit_dist
cmd_near_zero_duration
last_nav2_result
robot_to_path_distance
action
near_exit_fallback_enabled
near_exit_fallback_attempts
near_exit_fallback_max_attempts
micro_goal_geometry
micro_goal_local_cost
```

Fallback decisions use the existing `/maze/goal_events` stream; wrappers that record goal events will therefore capture fallback diagnostics in `log/<run_id>_goal_events.jsonl`.

## TDD record

RED was verified after adding `test_phase27_alt_near_exit_fallback.py` and before implementation:

```text
5 failed, 2 passed
```

Expected failures covered missing parameters, launch switch, event schema, decision helper, and micro-goal topology guard.

GREEN after implementation:

```text
7 passed in 0.01s
```

## Verification

Commands run from workspace root with ROS Jazzy sourced:

```bash
. /opt/ros/jazzy/setup.sh && python3 -m pytest src/tugbot_maze/test/test_phase27_alt_near_exit_fallback.py -q
```

Result:

```text
7 passed in 0.01s
```

Focused regression:

```bash
. /opt/ros/jazzy/setup.sh && python3 -m pytest \
  src/tugbot_maze/test/test_phase27_alt_near_exit_fallback.py \
  src/tugbot_maze/test/test_maze_explorer_phase14.py \
  src/tugbot_maze/test/test_maze_explorer_phase19.py -q
```

Result:

```text
15 passed in 0.01s
```

Static compile:

```bash
. /opt/ros/jazzy/setup.sh && python3 -m py_compile \
  src/tugbot_maze/tugbot_maze/maze_explorer.py \
  src/tugbot_bringup/launch/tugbot_maze_explore.launch.py \
  src/tugbot_maze/test/test_phase27_alt_near_exit_fallback.py
```

Result: passed.

Build/test:

```bash
. /opt/ros/jazzy/setup.sh && colcon build --symlink-install
. /opt/ros/jazzy/setup.sh && colcon test --event-handlers console_direct+
. /opt/ros/jazzy/setup.sh && colcon test-result --verbose
```

Result:

```text
Summary: 6 packages finished
Summary: 177 tests, 0 errors, 0 failures, 0 skipped
```

Cleanup/process check:

```bash
pgrep -af 'ros2 launch|gz sim|controller_server|planner_server|bt_navigator|slam_toolbox|maze_explorer|apt-get source|apt source'
```

Result: no matching runtime/source-fetch processes after filtering the check command itself.

## Current conclusion

Phase27-alt status:

```text
implemented_static_verified_pending_runtime
```

The default-off fallback is implemented and statically/build verified. Runtime behavior is intentionally not yet claimed because no bounded Gazebo/Nav2 validation run was performed in this phase.

The MPPI near-zero selected-control cause remains:

```text
insufficient_evidence
```

This phase does not tune Nav2/controller parameters and does not explain or resolve the MPPI internal selected-control mechanism. It provides a controlled engineering fallback for the near-exit symptom when enabled explicitly.

## Human acceptance checkpoint

Stop here for review. Recommended next step, only after human approval, is a single bounded Phase27-alt validation smoke with:

```text
near_exit_fallback_enabled:=true
```

and goal-event recording enabled. Acceptance should be based on fallback event coverage, no topology pollution, and final exit-distance/final-mode evidence, not on MPPI root-cause claims.
