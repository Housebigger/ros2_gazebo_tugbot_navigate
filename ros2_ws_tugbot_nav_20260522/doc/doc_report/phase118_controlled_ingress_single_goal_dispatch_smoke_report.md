# Phase118 controlled ingress single-goal dispatch smoke report

Status: PHASE118_CONTROLLED_INGRESS_SINGLE_GOAL_DISPATCH_SMOKE_COMPLETE_STOP_BEFORE_PHASE119

## Goal

Phase118 implements and runs a controlled visible-stack single-goal dispatch smoke wrapper after the Phase117 design review.

The wrapper must first run the current preflight. It may send exactly one explicit inner-ingress NavigateToPose goal only when the strict preflight and action-server preconditions are all true:

```text
preflight.passed == true
preflight.failed_gates == []
preflight.ingress_preflight_reject_reason is null/absent
action_server_ready == true
```

The only permitted goal is:

```text
frame_id=map
x=2.0
y=0.0
yaw=0.0
```

If preflight fails, the wrapper must not dispatch. If a future dispatch succeeds, it must still stop and must not start maze_explorer.

## Scope and hard guardrails

Phase118 allowed:

- Add a single-goal dispatch smoke runner.
- Add a Phase118 analyzer.
- Add focused static tests.
- Launch visible Gazebo/RViz/SLAM/Nav2 stack.
- Run the current preflight.
- Send the one explicit inner-ingress NavigateToPose goal only if strict preconditions pass.
- Bounded wait for result if the goal is accepted.
- Record artifacts and cleanup.

Phase118 forbidden actions preserved:

- No maze_explorer was started.
- No Goal1/carry-over/staging/branch/centerline/fallback/terminal/exit goal was sent.
- No Nav2/MPPI/controller/config tuning was performed.
- No exploration strategy was changed.
- Preflight was not removed.
- No autonomous exploration success or exit success is claimed.
- Phase119 not entered.

## Delivered files

Runner:

```text
tools/run_phase118_controlled_ingress_single_goal_dispatch_smoke.py
```

Analyzer:

```text
tools/analyze_phase118_controlled_ingress_single_goal_dispatch_smoke.py
```

Focused tests:

```text
src/tugbot_maze/test/test_phase118_controlled_ingress_single_goal_dispatch_smoke.py
```

Runtime artifacts:

```text
log/phase118_controlled_ingress_single_goal_dispatch_smoke/
```

## Implementation summary

The runner implements `single_goal_dispatch_smoke_only` and records the required dispatch schema even when no dispatch occurs.

Strict dispatch precondition helper:

```text
evaluate_dispatch_preconditions(preflight, action_server_ready)
```

Locked goal helper:

```text
locked_goal_pose(stamp=...)
```

Initial dispatch state:

```text
ingress_goal_sent=false before send
maze_explorer_started=false
```

State transition rule:

```text
ingress_goal_sent=true only after accepted
```

Therefore:

- If preflight fails, `dispatch_attempted=false` and `ingress_goal_sent=false`.
- If action server is not ready, `dispatch_attempted=false` and `ingress_goal_sent=false`.
- If a goal response rejects the goal, `dispatch_attempted=true`, `accepted=false`, `rejected=true`, and `ingress_goal_sent=false`.
- If accepted, only then `ingress_goal_sent=true`.
- timeout/abort/cancel are diagnostic fail.

Classifications implemented:

```text
PREFLIGHT_FAILED_NO_DISPATCH
INGRESS_DISPATCH_REJECTED_DIAGNOSTIC_FAIL
INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER
INGRESS_RESULT_ABORTED_DIAGNOSTIC_FAIL
INGRESS_RESULT_CANCELED_DIAGNOSTIC_FAIL
INGRESS_RESULT_TIMEOUT_DIAGNOSTIC_FAIL
INGRESS_DISPATCH_INSUFFICIENT_EVIDENCE
```

## TDD evidence

Focused tests were added before the runner/analyzer existed.

RED observed:

```text
4 failed, 1 deselected
missing required Phase118 file: tools/run_phase118_controlled_ingress_single_goal_dispatch_smoke.py
```

The tests cover:

- Strict dispatch preconditions.
- Locked explicit inner-ingress goal identity.
- `ingress_goal_sent` state transition.
- Classification behavior for rejected, timeout, succeeded, aborted, and canceled results.
- Analyzer schema validation.
- No-maze/no-overclaim guardrails.
- Report guardrails.

GREEN after runner/analyzer implementation:

```text
4 passed, 1 deselected in 0.02s
```

Static regression before runtime:

```text
50 passed, 9 deselected in 0.47s
```

## Runtime execution

Run id:

```text
phase118_controlled_ingress_single_goal_dispatch_smoke
```

Pre-runtime cleanup:

```json
{
  "phase": "Phase118",
  "cleanup": "pre_runtime",
  "before_count": 0,
  "after_count": 0
}
```

Visible stack command:

```bash
ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py \
  headless:=false \
  use_rviz:=true \
  use_sim_time:=true \
  autostart:=true \
  use_composition:=False \
  use_respawn:=False
```

ROS graph readiness before smoke:

```text
READY [('/navigate_to_pose action', True, ''), ('/map topic', True, ''), ('/scan topic', True, '')]
```

Phase118 runner command wrote:

```text
log/phase118_controlled_ingress_single_goal_dispatch_smoke/phase118_controlled_ingress_single_goal_dispatch_smoke_artifact.json
log/phase118_controlled_ingress_single_goal_dispatch_smoke/phase118_controlled_ingress_single_goal_dispatch_smoke_preflight.json
log/phase118_controlled_ingress_single_goal_dispatch_smoke/phase118_controlled_ingress_single_goal_dispatch_smoke_runner_stdout.json
log/phase118_controlled_ingress_single_goal_dispatch_smoke/phase118_controlled_ingress_single_goal_dispatch_smoke_runner_stderr.log
```

Runner stdout summary:

```json
{
  "accepted": false,
  "classification": "PREFLIGHT_FAILED_NO_DISPATCH",
  "ingress_goal_sent": false,
  "maze_explorer_started": false,
  "preflight_passed": false,
  "result_status_label": null
}
```

## Runtime classification

Final analyzer classification:

```text
PREFLIGHT_FAILED_NO_DISPATCH
```

This is the correct fail-closed classification because Phase118 preflight did not pass.

Analyzer validity:

```text
valid: True
```

Analyzer summary:

```text
preflight_passed: False
preflight_failed_gates: ['ingress_raw_snapshot_cross_check_failed', 'ingress_lifecycle_ambiguous', 'ingress_map_base_tf_missing', 'ingress_map_odom_tf_stale', 'ingress_odom_base_tf_stale', 'ingress_scan_transform_unstable', 'ingress_controller_robot_pose_unavailable', 'ingress_preflight_timeout']
preflight_reject_reason: ingress_preflight_timeout
dispatch_attempted: False
action_server_ready: False
accepted: False
rejected: False
result_status_label: None
abort_text: None
bounded_goal_result_wait_sec: 45.0
cancel_requested: False
cancel_result: None
ingress_goal_sent: False
maze_explorer_started: False
```

Because strict preflight did not pass, the wrapper did not send the inner-ingress goal.

## Required artifact fields

The artifact records the required Phase118 fields.

Preflight artifact:

```text
log/phase118_controlled_ingress_single_goal_dispatch_smoke/phase118_controlled_ingress_single_goal_dispatch_smoke_preflight.json
```

Dispatch artifact:

```text
log/phase118_controlled_ingress_single_goal_dispatch_smoke/phase118_controlled_ingress_single_goal_dispatch_smoke_artifact.json
```

Required dispatch schema status:

```text
required_dispatch_fields_present: true
missing_dispatch_fields: []
```

Recorded dispatch fields include:

```text
goal_pose
frame_id
stamp
x
y
yaw
action_server_ready
send_time
send_wall_time_sec
send_ros_time_sec
accepted
rejected
result_status
result_status_label
abort_text
bounded_goal_result_wait_sec
cancel_requested
cancel_result
ingress_goal_sent
maze_explorer_started
```

Goal identity in artifact:

```json
{
  "frame_id": "map",
  "stamp": null,
  "x": 2.0,
  "y": 0.0,
  "yaw": 0.0
}
```

`stamp` is null because dispatch was not attempted. The locked explicit inner-ingress goal identity was still recorded for auditability.

## Direct cause of no dispatch

Phase118 did not dispatch because the current preflight failed. The preflight artifact showed:

```text
passed=false
reject_reason=ingress_preflight_timeout
last_specific_reject_reason=ingress_lifecycle_ambiguous
```

Relevant lifecycle evidence during the failed preflight:

```text
bt_navigator.active_confirmed=false
bt_navigator.ambiguous=true
bt_navigator.ambiguous_detail=lifecycle subprocess timed out; missing strict active confirmation sources: launch_active_marker
controller_server.active_confirmed=true
navigate_to_pose_action_ready=true
```

A post-fail lifecycle snapshot recorded:

```text
/controller_server: inactive [2]
/bt_navigator: unconfigured [1]
/navigate_to_pose action: present
```

Launch log marker search found no `Managed nodes are active` marker in this Phase118 run before preflight timeout. Therefore the Phase115/116 strict lifecycle confirmation rule correctly remained fail-closed and prevented dispatch.

This Phase118 outcome is a diagnostic fail/no-dispatch smoke result, not an ingress navigation result.

## Guardrails

Final guardrails from analyzer:

```text
ingress_goal_sent=false
maze_explorer_started=false
no_maze_explorer_auto_start_guard_valid=true
nav2_config_tuned=false
exploration_strategy_changed=false
preflight_removed=false
no_autonomous_success_claim=true
no_exit_success_claim=true
```

No NavigateToPose goal was sent in the real run because the dispatch precondition failed before dispatch.

No Goal1/carry-over/staging/branch/centerline/fallback/terminal/exit goal was sent.

No maze_explorer was started.

No autonomous exploration success or exit success is claimed.

## Cleanup

Post-runtime cleanup:

```json
{
  "phase": "Phase118",
  "cleanup": "post_runtime",
  "before_count": 12,
  "after_count": 0
}
```

Killed/cleaned process categories included:

```text
ros2 launch tugbot_bringup
gz sim
ros_gz_bridge
slam_toolbox
controller_server
planner_server
bt_navigator
rviz2
```

Final runtime process guard was run after cleanup and is expected to be empty.

## Interpretation

Phase118 successfully enforced the Phase117 design contract.

The real visible-stack smoke did not reach dispatch because strict preflight failed. This is an acceptable Phase118 diagnostic outcome under the rules:

```text
PREFLIGHT_FAILED_NO_DISPATCH
ingress_goal_sent=false
maze_explorer_started=false
```

No timeout/failure was converted into success. No autonomous exploration success or exit success is claimed.

## Stop condition

Phase118 is complete and stopped.

Phase119 not entered.
