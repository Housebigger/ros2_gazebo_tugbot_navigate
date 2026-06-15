# Phase117: Controlled ingress goal dispatch design

Status: DESIGN_ONLY_PENDING_HUMAN_ACCEPTANCE

Phase117 is DESIGN_ONLY and doc-only. It defines how a future Phase118 may send exactly one controlled explicit inner-ingress goal after the Phase116 preflight pass evidence, but Phase117 itself does not implement dispatch code and does not run the simulation.

Phase117 boundaries:

- do not send NavigateToPose goal.
- do not start maze_explorer.
- do not tune Nav2/MPPI/controller/config.
- do not change exploration strategy.
- do not remove preflight.
- do not claim autonomous exploration success.
- do not claim exit success.
- Phase118 not entered.

## Background

Phase116 completed a visible-stack no-goal preflight-only rerun with:

```text
preflight.passed == true
preflight.failed_gates == []
reject_reason == null
classification == TF_SCAN_CONTROLLER_GATES_PASSED_NO_GOAL
```

Phase116 also diagnosed earlier Phase115 TF/scan/controller failures as startup TF buffer/tree-connection delay rather than permanent frame mismatch, sim-time age error, or persistent TF absence.

That evidence permits a design review for a future single-goal dispatch smoke phase. It does not itself authorize autonomous exploration, Goal1 staging validation, or exit success claims.

## Non-goals and hard guardrails

- DESIGN_ONLY: no runtime code change in Phase117.
- doc-only: no launch, no Gazebo/RViz, no Nav2 runtime, no action dispatch.
- do not send NavigateToPose goal in Phase117.
- do not start maze_explorer in Phase117.
- do not tune Nav2/MPPI/controller/config.
- do not tune Nav2/MPPI/controller/inflation/robot_radius/clearance_radius/map threshold/config.
- do not change exploration strategy.
- do not change branch scoring.
- do not change exploration order.
- do not change centerline gate.
- do not change directional readiness/fallback/terminal acceptance.
- do not remove preflight.
- do not add exploration goal.
- do not claim autonomous exploration success.
- do not claim exit success.
- Phase118 not entered.

## Design objective

Define a future Phase118 controlled dispatch wrapper that performs only this bounded sequence:

```text
project cleanup
  -> launch visible stack
  -> run unchanged Phase105/114/115/116 preflight
  -> if preflight.passed == true and preflight.failed_gates == [] and reject_reason is absent:
       dispatch exactly the explicit inner-ingress goal
       wait for bounded result
       record outcome
       stop without starting maze_explorer
     else:
       no dispatch
       classify preflight failure
       stop
```

The future Phase118 wrapper must be single-goal dispatch smoke only. It must not validate exploration success, Goal1 carry-over, staging, autonomous exploration, or exit success.

## Dispatch preconditions

Future Phase118 may enter dispatch only when all strict preconditions are true in the same artifact/run:

```text
preflight.passed == true
preflight.failed_gates == []
preflight.ingress_preflight_reject_reason is null or absent
preflight.evaluated == true
action_server_ready == true
```

Additional required guard semantics:

- no dispatch when failed_gates is non-empty.
- no dispatch when reject_reason is present.
- no dispatch when preflight evidence is missing or malformed.
- no dispatch when the preflight artifact is from a prior run ID or stale run context.
- no dispatch when no-goal guard cannot be transitioned safely.
- fail-closed on ambiguity.

If any precondition fails, the future wrapper must classify:

```text
dispatch_precondition_failed
PREFLIGHT_FAILED_NO_DISPATCH
```

and must leave:

```text
ingress_goal_sent=false
maze_explorer_started=false
```

## Locked goal identity

Future Phase118 must send only the existing explicit inner-ingress goal. It must not create, derive, search, stage, or optimize any new goal.

Locked goal:

```text
explicit inner-ingress goal
frame_id=map
x=2.0
y=0.0
yaw=0.0
```

Forbidden in Phase118 dispatch smoke:

- do not add exploration goal.
- do not send Goal1.
- do not send any carry-over, staging, branch, centerline, fallback, terminal, or exit goal.
- do not start maze_explorer automatically.
- do not set `max_goals` above zero. The Phase118 smoke wrapper should treat `max_goals=0` as the intended exploration setting because maze exploration is out of scope.

## State transition contract

Future Phase118 must treat `ingress_goal_sent` as a precise state transition, not a broad intention flag.

Required state sequence:

```text
ingress_goal_sent=false before send
  -> action server ready checked
  -> send_goal_async called for the locked explicit inner-ingress goal
  -> goal response received
  -> if accepted == true:
       ingress_goal_sent=true only after accepted
     else:
       ingress_goal_sent remains false
```

Important details:

- `ingress_goal_sent=true only after accepted` means the action server accepted the goal handle.
- A rejected goal is still a dispatch attempt, but it is not a sent/accepted ingress goal.
- A timeout before goal response must keep `ingress_goal_sent=false`.
- A future artifact may separately record `dispatch_attempted=true` to avoid ambiguity.

## Required dispatch artifact schema

Future Phase118 must write a dispatch artifact section even when no dispatch occurs. Recommended top-level shape:

```json
{
  "phase": "Phase118",
  "mode": "single_goal_dispatch_smoke_only",
  "preflight": {},
  "dispatch": {},
  "cleanup": {},
  "classification": "...",
  "guardrails": {}
}
```

Required `dispatch` fields:

```text
dispatch_attempted
preflight_pass_required
preflight_failed_gates_required_empty
preflight_reject_reason_required_absent
goal_pose
frame_id
stamp
x
y
yaw
action_name
action_server_ready
action_server_ready_checked_at_wall_time
send_time
send_wall_time_sec
send_ros_time_sec
goal_response_wall_time_sec
accepted
rejected
goal_handle_available
result_wait_started_wall_time_sec
bounded_goal_result_wait_sec
result_received
result_status
result_status_label
abort_text
cancel_requested
cancel_result
exception_text
traceback_tail
```

Required `goal_pose` content:

```json
{
  "frame_id": "map",
  "stamp": "use current ROS time at send",
  "x": 2.0,
  "y": 0.0,
  "yaw": 0.0
}
```

Required guardrail fields:

```text
ingress_goal_sent
maze_explorer_started
no_maze_explorer_auto_start_guard_valid
nav2_config_tuned=false
exploration_strategy_changed=false
preflight_removed=false
autonomous_success_claimed=false
exit_success_claimed=false
```

## Bounded wait and result handling

Future Phase118 must set an explicit bounded goal result wait:

```text
bounded_goal_result_wait_sec: finite, documented, and short enough for smoke validation
```

The wait may be long enough for the robot to reach the inner-ingress pose under normal conditions, but it must not become an unbounded autonomous run.

Outcome handling:

- `SUCCEEDED`: diagnostic success for the single ingress goal only; stop without starting maze_explorer.
- `ABORTED`: diagnostic fail; stop without starting maze_explorer.
- `CANCELED`: diagnostic fail; stop without starting maze_explorer.
- result timeout: diagnostic fail; optionally cancel active goal; stop without starting maze_explorer.
- goal response timeout: diagnostic fail; stop without starting maze_explorer.
- action server unavailable: diagnostic fail; stop without starting maze_explorer.

Mandatory phrase for implementation acceptance:

```text
timeout/abort/cancel are diagnostic fail
```

Timeout, abort, cancel, or rejection must never be converted into success and must never continue into `maze_explorer`.

## Cleanup/stop design

Future Phase118 must use cleanup/stop rules that are independent of dispatch result:

1. Always write the preflight artifact before dispatch decision.
2. Always write the dispatch artifact after dispatch decision.
3. If an active goal exists and bounded wait expires, request cancel and record cancel evidence.
4. Do not automatically start maze_explorer on success.
5. never auto-start maze_explorer.
6. Stop after single-goal dispatch result classification.
7. Cleanup scoped processes unless the user explicitly asks to keep Gazebo/RViz visible for screenshots.
8. Verify process guard and Nav2 config diff guard.

The cleanup/stop policy must apply equally to success, rejected, aborted, canceled, timeout, and insufficient-evidence outcomes.

## Outcome classifications

Allowed Phase118 wrapper classifications:

```text
PREFLIGHT_FAILED_NO_DISPATCH
INGRESS_DISPATCH_REJECTED_DIAGNOSTIC_FAIL
INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER
INGRESS_RESULT_ABORTED_DIAGNOSTIC_FAIL
INGRESS_RESULT_CANCELED_DIAGNOSTIC_FAIL
INGRESS_RESULT_TIMEOUT_DIAGNOSTIC_FAIL
INGRESS_DISPATCH_INSUFFICIENT_EVIDENCE
```

Classification meanings:

### `PREFLIGHT_FAILED_NO_DISPATCH`

Use when preflight does not satisfy all strict dispatch preconditions.

Required fields:

```text
dispatch_attempted=false
ingress_goal_sent=false
maze_explorer_started=false
preflight.failed_gates=<recorded list>
preflight.reject_reason=<recorded token or null>
```

### `INGRESS_DISPATCH_REJECTED_DIAGNOSTIC_FAIL`

Use when preflight passed and a goal request was sent to the action server, but the action server rejected the goal handle.

Required fields:

```text
dispatch_attempted=true
accepted=false
rejected=true
ingress_goal_sent=false
maze_explorer_started=false
```

### `INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER`

Use only when the locked explicit inner-ingress goal is accepted and returns a succeeded result within the bounded wait.

This means only:

```text
single explicit inner-ingress goal reached or reported succeeded by Nav2
```

It does not mean:

```text
autonomous exploration success
Goal1 carry-over success
staging success
exit success
```

Required fields:

```text
accepted=true
ingress_goal_sent=true
result_status=SUCCEEDED
maze_explorer_started=false
```

### `INGRESS_RESULT_ABORTED_DIAGNOSTIC_FAIL`

Use when the accepted goal returns aborted.

Required fields:

```text
accepted=true
ingress_goal_sent=true
result_status=ABORTED
abort_text=<Nav2/error text if available>
maze_explorer_started=false
```

### `INGRESS_RESULT_CANCELED_DIAGNOSTIC_FAIL`

Use when the accepted goal returns canceled or wrapper cancellation completes.

Required fields:

```text
accepted=true
ingress_goal_sent=true
result_status=CANCELED
cancel_requested=<true/false>
cancel_result=<recorded evidence>
maze_explorer_started=false
```

### `INGRESS_RESULT_TIMEOUT_DIAGNOSTIC_FAIL`

Use when the result does not arrive within `bounded_goal_result_wait_sec`.

Required fields:

```text
accepted=true
ingress_goal_sent=true
result_received=false
bounded_goal_result_wait_sec=<finite value>
cancel_requested=true unless impossible
maze_explorer_started=false
```

### `INGRESS_DISPATCH_INSUFFICIENT_EVIDENCE`

Use when the artifact cannot support a stronger classification, for example missing goal response evidence, malformed action result, inconsistent timestamps, or missing wrapper state.

Required fields:

```text
insufficient_evidence_reason
maze_explorer_started=false
```

## Phase118 allowed scope

Phase118 allowed scope is deliberately narrow:

```text
single-goal dispatch smoke only
```

Allowed in Phase118:

- project cleanup.
- visible stack launch.
- run the current preflight.
- if and only if preflight passed, dispatch the locked explicit inner-ingress goal once.
- wait for a bounded result.
- record goal dispatch/result artifacts.
- stop.

Forbidden in Phase118:

- does not validate exploration success.
- does not validate Goal1 carry-over.
- does not validate staging.
- does not validate exit success.
- does not start maze_explorer.
- does not run autonomous exploration.
- does not tune Nav2/MPPI/controller/config.
- does not change exploration strategy.
- does not treat single-goal success as autonomous success.

The expected Phase118 exploration setting is:

```text
max_goals=0
```

because exploration must remain disabled.

## Suggested future Phase118 runbook outline

This is a design outline only, not a command to execute in Phase117.

1. Cleanup scoped old ROS/Gazebo/Nav2/RViz/preflight processes.
2. Launch visible stack with `headless:=false`, `use_rviz:=true`, and `use_sim_time:=true`.
3. Wait for graph readiness.
4. Run preflight.
5. Evaluate strict dispatch preconditions.
6. If preconditions fail:
   - classify `PREFLIGHT_FAILED_NO_DISPATCH`.
   - stop.
7. If preconditions pass:
   - check `/navigate_to_pose` action server readiness immediately before dispatch.
   - send exactly `frame_id=map, x=2.0, y=0.0, yaw=0.0`.
   - mark `ingress_goal_sent=true` only after accepted.
   - wait bounded result.
   - classify result.
   - stop without `maze_explorer`.
8. Cleanup or keep visible only if explicitly requested for screenshot observation.
9. Verify no Nav2 config diff and no residual runtime processes.

## Acceptance checklist for future Phase118 implementation

A future Phase118 implementation is acceptable only if artifacts prove:

- preflight was evaluated in the same run.
- dispatch occurred only after `preflight.passed == true` and `preflight.failed_gates == []`.
- goal pose exactly matched the locked explicit inner-ingress goal.
- action server readiness was checked immediately before sending.
- `ingress_goal_sent` remained false before accepted goal response.
- `ingress_goal_sent` changed to true only after accepted goal response.
- result wait was bounded.
- timeout/abort/cancel are diagnostic fail.
- `maze_explorer_started=false` for every classification.
- no autonomous exploration success or exit success was claimed.

## Stop condition

Phase117 stops after this design proposal, design report, and focused static tests are complete. It does not implement dispatch and does not enter Phase118.
