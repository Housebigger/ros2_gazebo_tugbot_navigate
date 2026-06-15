# Phase27-alt Near-Exit Recovery / Terminal Acceptance Fallback Design

Date: 2026-05-27
Workspace: ros2_ws_tugbot_nav_20260522
Status: design-before-implementation

## Goal

Implement a small, reversible, diagnostics-rich upper-layer fallback for the Tugbot maze task:

- Success criterion: robot reaches the configured exit coordinates / radius.
- Runtime path source: live ROS observations only (`/map`, TF pose, Nav2 action state, existing local costmap diagnostics). The maze image is not used as a runtime path answer.
- Scope: `maze_explorer` upper-level exploration logic only.
- Out of scope: MPPI/Nav2/controller parameter changes, branch-selection main strategy changes, Phase26Y/26Z source-overlay work, and CostCritic candidate promotion/rejection.

This is an engineering fallback, not an explanation of the Phase26 MPPI selected-control near-zero root cause.

## Constraints

1. Do not use the maze image as a runtime path answer.
2. Allow configured exit coordinates as the terminal success criterion.
3. Trigger only when the robot is already near the exit.
4. Do not modify Nav2/MPPI parameters.
5. Do not change the normal DFS/topology branch-selection strategy.
6. Provide an explicit enable switch with the default recorded in state/events.
7. Emit every decision into `/maze/goal_events` as structured JSON fields.
8. Do not pollute topology: fallback failures must not mark normal DFS branches blocked and must not blacklist ordinary branch targets.
9. For Phase27-alt initial delivery, run only host-level tests/build/static checks; no long Gazebo/Nav2 run.

## Proposed parameters

Add to `MazeExplorer` and expose in `tugbot_maze_explore.launch.py`:

- `near_exit_fallback_enabled` (bool, default `false`)
  - Default-off preserves existing baseline behavior and makes activation explicit.
- `near_exit_fallback_trigger_radius_m` (float, default `0.90`)
  - Hard outer gate for fallback consideration.
- `near_exit_terminal_acceptance_radius_m` (float, default `0.60`)
  - If the robot is this close to the exit after a qualifying stall/failure, accept terminal success.
  - This should match the existing `exit_radius` default but remains a separate fallback parameter for auditability.
- `near_exit_micro_goal_min_step_m` (float, default `0.20`)
- `near_exit_micro_goal_max_step_m` (float, default `0.35`)
  - Bound micro-goal size to avoid creating a hidden path planner.
- `near_exit_fallback_max_attempts` (int, default `1`)
  - Keep the initial behavior bounded and reversible.
- `near_exit_fallback_require_clean_topology` (bool, default `true`)
  - Require `blocked_branch_count == 0` and `blacklisted_goal_count == 0`.
- `near_exit_fallback_require_path_alignment` (bool, default `true`)
  - Require `robot_to_path_distance <= 0.15m` when a measured path-distance signal exists.
- `near_exit_fallback_robot_to_path_max_m` (float, default `0.15`)
- `near_exit_fallback_cmd_near_zero_min_sec` (float, default `3.0`)
  - Used when `cmd_near_zero_duration` evidence is available.

## Initial implementation strategy

Use the existing `goal_events` stream rather than adding a second JSONL publisher. Runtime recording wrappers already support `/maze/goal_events`; adding a new event type keeps the artifact path compatible with existing recorders:

- recorded by wrapper as `log/<run_id>_goal_events.jsonl`;
- every event has `near_exit_fallback_triggered`, `fallback_reason`, `robot_exit_dist`, `cmd_near_zero_duration`, `last_nav2_result`, `robot_to_path_distance`, and `action` fields;
- action values: `terminal_acceptance`, `micro_goal`, `no_action`.

A dedicated `log/<run_id>_fallback_events.jsonl` can be added later by recorder filtering if needed, but Phase27-alt minimum implementation keeps one source of truth.

## Trigger evidence model

The fallback decision must be conservative. It can run only after the normal navigation machinery has produced a failure/stall signal.

Required hard gates:

1. `near_exit_fallback_enabled == true`.
2. `robot_exit_dist <= near_exit_fallback_trigger_radius_m`.
3. Recent Nav2/problem evidence exists:
   - `last_failure_reason` is one of `GOAL_TIMEOUT`, `BLOCKED_NAV2`, `GOAL_CANCELED_AFTER_TIMEOUT`, or `GOAL_REJECTED`; or
   - a late-silent/cmd-near-zero subtype is available in diagnostics; or
   - `cmd_near_zero_duration >= near_exit_fallback_cmd_near_zero_min_sec`.
4. If clean-topology gate is enabled: `blocked_branch_count == 0` and `blacklisted_goal_count == 0`.
5. If path-alignment gate is enabled and `robot_to_path_distance` is known: `robot_to_path_distance <= near_exit_fallback_robot_to_path_max_m`.

Unknown optional fields should be recorded as `null`, not fabricated.

## Action selection

### A. Terminal acceptance

If all hard gates pass and:

- `robot_exit_dist <= near_exit_terminal_acceptance_radius_m`,

then:

- publish a `near_exit_fallback` goal event with `action='terminal_acceptance'`;
- enter `EXIT_REACHED` with `terminal_reason='near_exit_terminal_acceptance'`;
- cancel any active Nav2 goal through existing terminal-cancel path if needed;
- do not mark/blacklist any branch.

### B. Bounded near-exit micro-goal

If all hard gates pass and:

- `near_exit_terminal_acceptance_radius_m < robot_exit_dist <= near_exit_fallback_trigger_radius_m`,
- attempts used < `near_exit_fallback_max_attempts`,
- a local/map feasibility check passes,

then:

- compute a micro-goal from the current robot pose toward the configured exit vector;
- step length is clamped to `[0.20, 0.35]` and never overshoots terminal radius;
- validate with existing occupancy/local-cost geometry checks:
  - target cell in map is not occupied;
  - line of sight has zero occupied cells;
  - target clearance is at least `clearance_radius_m` where available;
  - local cost target/path samples are in-bounds when local costmap is available, with no lethal target cost;
- publish `near_exit_fallback` with `action='micro_goal'` and diagnostics fields;
- send a Nav2 goal with `goal_kind='near_exit_micro_goal'`.

Micro-goal success should behave like a bounded recovery step and then return to normal exploration/exit checks. Micro-goal failure must publish diagnostics and settle, but must not call `record_branch_failure`, must not blacklist, and must not mutate DFS branch state.

### C. No action

If any gate fails or feasibility fails:

- publish `near_exit_fallback` with `action='no_action'` and a specific `fallback_reason`;
- return to normal DFS/topology flow unchanged.

## Minimal code touch points

1. `src/tugbot_maze/tugbot_maze/maze_explorer.py`
   - Add parameters and runtime counters.
   - Add a pure-ish decision helper for host-level tests:
     - `_near_exit_fallback_decision(robot_pose, robot_to_path_distance=None, cmd_near_zero_duration=None)` returning a dictionary.
   - Add `_maybe_apply_near_exit_fallback(robot_pose)` called after goal settle and before ordinary `_analyze_and_dispatch`.
   - Add `_publish_near_exit_fallback_event(decision)` that reuses `goal_events_pub`.
   - Treat `active_goal_kind == 'near_exit_micro_goal'` as neither branch explore nor backtrack in success/failure handlers.
2. `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`
   - Expose the new parameters with default `false` for the enable switch.
3. `src/tugbot_maze/test/test_phase27_alt_near_exit_fallback.py`
   - Host-level tests for default-disabled guard, trigger/no-trigger/boundary behavior, event schema, launch default, and topology non-pollution guard.
4. `doc/doc_report/phase27_alt_near_exit_fallback_design_report.md`
   - Implementation/verification report.
5. `README.md`
   - Current phase status update.

## Acceptance tests

Host-level tests should cover:

1. The launch default explicitly exposes `near_exit_fallback_enabled=false`.
2. `maze_explorer` declares the enable switch and bounded thresholds.
3. Event schema includes:
   - `near_exit_fallback_triggered`
   - `fallback_reason`
   - `robot_exit_dist`
   - `cmd_near_zero_duration`
   - `last_nav2_result`
   - `robot_to_path_distance`
   - `action`
4. Disabled switch yields `action='no_action'` and no terminal/micro-goal side effect.
5. Distance boundary: `<=0.60m` terminal acceptance; `(0.60, 0.90]m` micro-goal candidate; `>0.90m` no action.
6. Topology guard: blocked/blacklisted count nonzero prevents fallback when clean-topology gate is enabled.
7. Micro-goal failure path does not call `record_branch_failure`, `mark_branch_state`, or blacklist for `near_exit_micro_goal`.
8. No Nav2/MPPI parameter file is changed by Phase27-alt.

## Verification plan

Run only static/host/build checks:

```bash
. /opt/ros/jazzy/setup.sh
python3 -m pytest src/tugbot_maze/test/test_phase27_alt_near_exit_fallback.py -q
python3 -m pytest src/tugbot_maze/test/test_maze_explorer_phase14.py src/tugbot_maze/test/test_maze_explorer_phase19.py -q
python3 -m py_compile src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_bringup/launch/tugbot_maze_explore.launch.py
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

No Gazebo/Nav2 long run in this phase.

## Rollback plan

- Runtime rollback: keep `near_exit_fallback_enabled:=false` (default).
- Code rollback: revert Phase27-alt edits to `maze_explorer.py`, `tugbot_maze_explore.launch.py`, Phase27 tests/docs/README.
- Diagnostic rollback: ignore `near_exit_fallback` events; normal goal event fields remain unchanged.

## Expected final conclusion format

The report must state one of:

- `implemented_static_verified_pending_runtime`: code/tests/build pass, no long run yet;
- `design_only_pending_implementation`: proposal written but implementation deferred;
- `blocked`: with specific blocker.

It must not claim MPPI near-zero root cause resolution and must not propose controller tuning.
