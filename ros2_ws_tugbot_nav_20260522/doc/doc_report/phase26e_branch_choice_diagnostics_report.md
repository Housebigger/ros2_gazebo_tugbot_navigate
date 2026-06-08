# Phase 26E runtime branch-choice diagnostics

Date: 2026-05-25
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## Goal

Add runtime branch-choice diagnostics to `/maze/goal_events` dispatch events only.

Acceptance constraints:

- Do not change branch-choice behavior.
- Add diagnostics fields to dispatch goal events.
- TDD contract tests prove the new fields exist.
- Smoke run verifies artifact coverage only.
- Do not use the Phase26E smoke to promote/reject Nav2, controller, local-cost, or branch-choice parameters.

## Implementation summary

Changed `src/tugbot_maze/tugbot_maze/maze_explorer.py`.

Added dispatch-event fields:

- `chosen_branch_rank`
- `chosen_branch_score_components`
- `candidate_branch_count`
- `candidate_branches[]`
- `selected_due_to_context`

Each `candidate_branches[]` entry includes:

- `branch_angle`
- `target`
- `target_exit_dist`
- `exit_progress_delta_m`
- `target_clearance_m`
- `path_corridor_min_clearance_m`
- `dispatch_path_local_cost_max`
- `dispatch_path_local_cost_mean`
- `target_local_cost`
- `target_local_cost_max_radius`
- `is_reverse_candidate`
- `is_backtrack_context`
- `is_near_exit_candidate`
- `rejection_reason`

Additional useful diagnostic-only fields inside each candidate entry:

- `rank`
- `state`
- `score_components`

Added helper logic:

- `_empty_branch_choice_diagnostics()`
- `_build_branch_choice_diagnostics()`
- `_branch_score_components()`
- `_branch_option_payload()`

Added smoke/artifact coverage helper:

- `tools/check_phase26e_branch_diagnostics_coverage.py`

Updated smoke wrapper run-id allowlist:

- `tools/run_phase21_controller_diagnostics_smoke.sh` accepts `phase26e_branch_diagnostics_smoke`.

## Behavior preservation notes

The existing behavior still calls:

```python
chosen = self.topology.choose_next_branch(node.node_id, exit_xy=(self.exit_x, self.exit_y))
```

Diagnostics are built only after `choose_next_branch()` has already returned `chosen` and before `_send_goal()` publishes the dispatch event.

The diagnostics compute a ranked mirror of the existing `BranchOption.score_for_exit()` formula for observability, but this ranking is not used to select or replace `chosen`.

No Nav2 parameter, controller parameter, local-cost profile, branch scoring policy, or branch rejection policy was changed.

## TDD / contract validation

RED was verified first:

```bash
python3 -m pytest -q src/tugbot_maze/test/test_phase26e_branch_choice_diagnostics.py -v
```

Initial result before implementation: 4 failures for missing Phase26E fields/helpers/run-id support.

GREEN after implementation:

```bash
python3 -m pytest -q src/tugbot_maze/test/test_phase26e_branch_choice_diagnostics.py -v
```

Result:

- 6 passed

Targeted regression set:

```bash
. /opt/ros/jazzy/setup.sh && python3 -m pytest -q \
  src/tugbot_maze/test/test_phase26e_branch_choice_diagnostics.py \
  src/tugbot_maze/test/test_maze_explorer_phase14.py \
  src/tugbot_maze/test/test_maze_explorer_phase17.py \
  src/tugbot_maze/test/test_maze_explorer_phase19.py \
  src/tugbot_maze/test/test_phase23b_local_cost_runtime_diagnostics.py \
  src/tugbot_maze/test/test_maze_topology.py \
  src/tugbot_maze/test/test_maze_topology_phase4.py
```

Result:

- 30 passed

Static workspace contract:

```bash
. /opt/ros/jazzy/setup.sh && python3 /home/hyh/.hermes/skills/software-development/ros2-gazebo-navigation-workspaces/scripts/check_ros2_workspace_contracts.py .
```

Result:

- ROS 2 workspace static contracts passed

Build and full test:

```bash
. /opt/ros/jazzy/setup.sh && colcon build --symlink-install
. /opt/ros/jazzy/setup.sh && colcon test --event-handlers console_direct+
. /opt/ros/jazzy/setup.sh && colcon test-result --verbose
```

Final result after adding the coverage helper:

- `colcon build --symlink-install`: 6 packages finished
- `colcon test`: 120 passed in `tugbot_maze`
- `colcon test-result --verbose`: 120 tests, 0 errors, 0 failures, 0 skipped

## Phase26E smoke run

Command:

```bash
. /opt/ros/jazzy/setup.sh && . install/setup.sh && \
  bash tools/run_phase21_controller_diagnostics_smoke.sh phase26e_branch_diagnostics_smoke
```

The first direct invocation failed with permission denied because the wrapper is not executable when called directly. Re-run via `bash ...` succeeded.

Smoke result summary from wrapper:

- `run_id`: `phase26e_branch_diagnostics_smoke`
- `final_mode`: `EXIT_REACHED`
- `goal_count`: 8
- `goal_success_count`: 5
- `goal_failure_count`: 2
- `timeout_cancel_count`: 2
- `blocked_branch_count`: 0
- `blacklisted_goal_count`: 0
- `exit_distance_m`: 0.5572765014159342

Important: these navigation metrics are recorded only as context. They are not Phase26E promotion/rejection evidence.

## Artifact coverage validation

Coverage command:

```bash
python3 tools/check_phase26e_branch_diagnostics_coverage.py \
  --goal-events log/phase26e_branch_diagnostics_smoke_goal_events.jsonl \
  --output-json log/phase26e_branch_diagnostics_smoke_branch_diagnostics_coverage.json
```

Coverage result:

- `pass_coverage`: true
- `total_events`: 19
- `dispatch_events`: 8
- `dispatch_events_with_nonempty_candidate_branches`: 8
- `candidate_branch_rows`: 29
- top-level missing counts: all 0
- candidate field missing counts: all 0
- `chosen_branch_ranks`: `[1, 1, 1, 1, 1, 1, 1, 1]`
- `candidate_branch_counts`: `[4, 3, 3, 4, 4, 4, 4, 3]`
- `selected_due_to_context_values`: `["topology_exit_bias_score"]`

Coverage artifact:

- `log/phase26e_branch_diagnostics_smoke_branch_diagnostics_coverage.json`

Primary smoke artifacts:

- `log/phase26e_branch_diagnostics_smoke_goal_events.jsonl`
- `log/phase26e_branch_diagnostics_smoke_explorer_state.jsonl`
- `log/phase26e_branch_diagnostics_smoke_launch.log`
- `log/phase26e_branch_diagnostics_smoke_goal_nav2_analysis.json`
- `log/phase26e_branch_diagnostics_smoke_geometry_nav2_summary.json`
- `log/phase26e_branch_diagnostics_smoke_goal_event_cost_summary.json`
- `log/phase26e_branch_diagnostics_smoke_goal_controller_dynamics.json`
- `log/phase26e_branch_diagnostics_smoke_branch_diagnostics_coverage.json`

## Cleanup verification

After smoke completion, checked for residual ROS/Gazebo/recorder processes:

```bash
ps -eo pid,cmd | grep -E 'ros2 launch tugbot_bringup|record_explorer_state_series|record_controller_dynamics|record_post_recovery_snapshots|ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|maze_explorer|slam_toolbox|controller_server|planner_server|bt_navigator|gz sim|ruby .*gz sim' | grep -v grep || true
```

Result: no matching residual processes.

## Decision

Phase 26E acceptance: PASS.

Reason:

- The new dispatch diagnostics fields exist and are covered by TDD contract tests.
- Existing branch-choice behavior is preserved: selection still comes from `MazeTopology.choose_next_branch()`.
- Smoke artifacts prove coverage for dispatch events and candidate branch payloads.
- No parameter promotion/rejection decision was made from this smoke.

## Next-stage recommendation

Proceed to Phase 26F as analysis-only, using Phase26E diagnostics from repeated baseline/candidate runs to answer route-divergence questions before any branch-selection change.

Suggested Phase 26F scope:

1. Add a post-run branch-choice analyzer over `/maze/goal_events`:
   - group dispatch events by run and goal sequence;
   - summarize chosen rank, candidate count, chosen target exit delta, and rejected candidate alternatives;
   - flag dispatches where the chosen target moves away from exit while a rejected candidate moves toward exit;
   - separate reverse, backtrack, near-exit, local-cost-constrained, and blacklisted/terminal contexts.

2. Run matched diagnostics-only repeats if needed:
   - baseline profile;
   - candidate profile if still under investigation;
   - do not tune parameters during this phase.

3. Only after Phase26F evidence exists, decide whether Phase 27 should test a narrow branch-selection intervention.

Potential Phase 27 gates, if reached later:

- no regression in `blocked_branch_count` / `blacklisted_goal_count`;
- route divergence decreases;
- exit reach rate or final exit distance improves across matched repeats;
- candidate branch decision is explained by diagnostics, not by a single lucky smoke.
