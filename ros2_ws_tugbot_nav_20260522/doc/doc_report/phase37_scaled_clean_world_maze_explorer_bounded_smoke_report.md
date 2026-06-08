# Phase37 Scaled Clean World Maze Explorer Bounded Autonomous Smoke Report

Date: 2026-05-28 13:34:19 UTC

## Scope

Phase37 was the first bounded autonomous `maze_explorer` smoke on the active scaled2x clean world after Phase36B metadata wiring was accepted.

This phase was deliberately not a full autonomous exit-success validation. It used a timeout shell and `max_goals` limit, collected artifacts, and stopped for review.

## Inputs

- Phase36B human acceptance: `READY_FOR_BOUNDED_AUTONOMOUS_EXPLORATION_SMOKE`
- Active world: `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf`
- Active metadata: `src/tugbot_maze/config/maze_20260528_scaled_instance.yaml`
- Active truth:
  - entrance_x = -11.011281
  - entrance_y = -9.025070
  - entrance_yaw = 0.0
  - exit_x = 10.061281
  - exit_y = 9.058496
  - exit_radius = 1.2

## Files added

- `tools/run_phase37_scaled_clean_world_maze_explorer_bounded_smoke.sh`
- `src/tugbot_maze/test/test_phase37_bounded_smoke_wrapper.py`

## TDD / static verification

Focused wrapper tests were added first and initially failed because the wrapper did not exist:

```text
3 failed in 0.02s
Phase37 bounded smoke wrapper must exist
```

After adding the wrapper and adjusting the test to permit legacy-token detection code while forbidding legacy launch arguments, focused tests passed:

```text
python3 -m pytest -p no:cacheprovider   src/tugbot_maze/test/test_phase36_autonomous_readiness.py   src/tugbot_maze/test/test_phase37_bounded_smoke_wrapper.py -q

9 passed in 0.37s
```

Other static verification:

```text
bash -n tools/run_phase37_scaled_clean_world_maze_explorer_bounded_smoke.sh: PASS
python3 -m py_compile tools/check_phase36_autonomous_readiness.py src/tugbot_maze/test/test_phase36_autonomous_readiness.py src/tugbot_maze/test/test_phase37_bounded_smoke_wrapper.py tools/record_explorer_state_series.py: PASS
Phase36/37 readiness analyzer: READY_FOR_BOUNDED_AUTONOMOUS_EXPLORATION_SMOKE, blockers=0
git diff -- src/tugbot_navigation/config: empty
```

## Bounded smoke command

```bash
PHASE37_RUN_TIMEOUT_SEC=720 PHASE37_MAX_GOALS=4 PHASE37_SNAPSHOT_DELAY_SEC=60 PHASE37_STATE_RECORDER_TIMEOUT_SEC=750 tools/run_phase37_scaled_clean_world_maze_explorer_bounded_smoke.sh
```

The wrapper launched:

- Gazebo with the active scaled2x world
- SLAM
- Nav2
- `maze_explorer`
- `maze_goal_monitor`
- recorders for `/maze/explorer_state` and `/maze/goal_events`

RViz was not started (`PHASE37_USE_RVIZ=false`).

## Artifacts

Artifact directory:

- `log/phase37_scaled_clean_world_maze_explorer_bounded_smoke/`

Key files:

- `phase37_scaled_clean_world_maze_explorer_bounded_smoke_launch.log`
- `phase37_scaled_clean_world_maze_explorer_bounded_smoke_explorer_state.jsonl`
- `phase37_scaled_clean_world_maze_explorer_bounded_smoke_goal_events.jsonl`
- `phase37_scaled_clean_world_maze_explorer_bounded_smoke_ros_graph_initial.txt`
- `phase37_scaled_clean_world_maze_explorer_bounded_smoke_navigate_to_pose_action_info.txt`
- `phase37_scaled_clean_world_maze_explorer_bounded_smoke_map_once.txt`
- `phase37_scaled_clean_world_maze_explorer_bounded_smoke_scan_once.txt`
- `phase37_scaled_clean_world_maze_explorer_bounded_smoke_odom_once.txt`
- `phase37_scaled_clean_world_maze_explorer_bounded_smoke_tf_once.txt`
- `phase37_scaled_clean_world_maze_explorer_bounded_smoke_local_costmap_once.txt`
- `phase37_scaled_clean_world_maze_explorer_bounded_smoke_global_costmap_once.txt`
- `phase37_scaled_clean_world_maze_explorer_bounded_smoke_summary.json`
- `phase37_scaled_clean_world_maze_explorer_bounded_smoke_acceptance_analysis.json`
- `phase37_scaled_clean_world_maze_explorer_bounded_smoke_cleanup_processes_after_manual_kill.txt`

## Runtime observations

`maze_explorer` started and used the active scaled2x truth:

```text
maze_explorer started: map_topic=/map action_name=/navigate_to_pose state_topic=/maze/explorer_state entrance=(-11.011, -9.025, 0.000) exit=(10.061, 9.058) radius=1.200 ... max_goals=4
```

`maze_goal_monitor` also used the active exit/radius:

```text
maze_goal_monitor started: map_frame=map base_frame=base_link exit=(10.061, 9.058) radius=1.200 topic=/maze/exit_reached
```

ROS graph/action evidence:

```text
/navigate_to_pose action server available via /bt_navigator
Action clients include /maze_explorer
```

`/maze/explorer_state` data was recorded:

```text
explorer_state_samples = 10
final_mode = FAILED_EXHAUSTED
last_terminal_reason = no untried branches remain
goal_count = 0
known_junctions = 1
last_local_topology_kind = unknown
last_open_direction_count = 0
last_candidate_count = 0
exit_distance_m = 13.538305844630706
```

`/maze/goal_events` did not record a dispatch/outcome event:

```text
goal_events_samples = 0
dispatch_events = 0
outcome_events = 0
exit_reached_events = 0
```

The run reached `FAILED_EXHAUSTED` before dispatching any autonomous goal. Final state reported:

```text
mode = FAILED_EXHAUSTED
last_terminal_reason = no untried branches remain
goal_count = 0
last_local_topology_kind = unknown
last_open_direction_count = 0
last_candidate_count = 0
```

Map/scan/odom/tf/costmap sample files were saved. The `/map` sample existed but was small/mostly unknown around startup; `/scan` and `/odom` samples existed. This supports that the stack came up, but not that autonomous branch dispatch behavior succeeded.

## Acceptance check summary

```json
{
  "active_truth_used": true,
  "at_least_one_dispatch": false,
  "bounded_not_complete_success_claim": true,
  "cleanup_empty": true,
  "goal_events_topic_has_data": false,
  "maze_explorer_started": true,
  "nav2_action_server_available": true,
  "state_topic_has_data": true
}
```

Phase37 smoke decision:

```text
BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH
```

Failures / unmet smoke goals:

```text
goal_events_topic_has_data, at_least_one_dispatch
```

Interpretation:

- PASS: `maze_explorer` launched.
- PASS: active scaled2x entrance/exit/radius were used; no legacy `-4/-3/4/3/0.6` truth appeared in the launch log.
- PASS: `/maze/explorer_state` published data.
- PASS: Nav2 `/navigate_to_pose` action server was available and `/maze_explorer` appeared as an action client.
- FAIL: no `/maze/goal_events` dispatch/outcome data was recorded.
- FAIL: no autonomous goal dispatch occurred (`goal_count=0`, `dispatch_events=0`).
- PASS: cleanup check after manual parent-session kill was empty.

## Cleanup

The Hermes background process was manually killed after evidence showed `maze_explorer` had already reached `FAILED_EXHAUSTED` with no dispatch. A follow-up cleanup removed ROS/Gazebo/Nav2/explorer processes.

Post-cleanup process file:

```text
(empty)
```

Cache cleanup:

```text
(empty)
```

## Guardrail confirmation

- Nav2/MPPI/controller parameters were not modified.
- Navigation strategy was not modified.
- Fallback / terminal acceptance was not continued; `near_exit_fallback_enabled:=false` was used.
- No old scaffold world/map was used.
- The run was bounded by timeout and `max_goals=4`.
- Cleanup was performed and verified empty.
- No full autonomous exploration success is claimed.

## Conclusion

Phase37 did not pass the full bounded-smoke acceptance target because no autonomous goal dispatch was recorded and `/maze/goal_events` stayed empty.

Conclusion:

```text
BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH
```

This is useful first-runtime evidence: the active scaled2x launch wiring works, `maze_explorer` starts with the correct metadata, Nav2 action infrastructure is available, and `/maze/explorer_state` publishes. However, `maze_explorer` immediately exhausts in the new world before finding any dispatchable branch. A later phase should investigate why the initial local topology is `unknown` with zero open directions/candidates in the active scaled2x map before any strategy or Nav2 parameter changes.

Stop here for human review.
