# Phase117 controlled ingress goal dispatch design report

Status: PHASE117_CONTROLLED_INGRESS_GOAL_DISPATCH_DESIGN_COMPLETE_STOP_BEFORE_PHASE118

## Goal

Phase117 is a design-only review for a future controlled ingress dispatch smoke phase. It defines how Phase118 may send exactly one explicit inner-ingress goal after a strict preflight pass, and how that future run must record, bound, clean up, and classify outcomes.

Phase117 does not implement dispatch code and does not run Gazebo/RViz/Nav2.

## Work performed

No implementation was performed.

No runtime rerun was performed.

No NavigateToPose goal was sent.

No maze_explorer was started.

No Nav2/MPPI/controller/config tuning was performed.

No exploration strategy was changed.

Preflight was not removed.

No autonomous exploration success or exit success is claimed.

Phase118 not entered.

Phase117 produced:

- `doc/doc_proposal/phase117_controlled_ingress_goal_dispatch_design.md`
- `doc/doc_report/phase117_controlled_ingress_goal_dispatch_design_report.md`
- `src/tugbot_maze/test/test_phase117_controlled_ingress_goal_dispatch_design.py`

## Context carried from Phase116

Phase116 completed TF/scan/controller gate focused diagnosis. A real visible-stack no-goal preflight-only rerun passed:

```text
failed_gates=[]
passed=true
reject_reason=null
classification=TF_SCAN_CONTROLLER_GATES_PASSED_NO_GOAL
```

Phase116 concluded that the remaining Phase115 gates came from startup TF buffer/tree-connection delay, not permanent frame mismatch, sim-time age error, or persistent TF absence.

Phase117 treats this as enough to design a future single-goal smoke, not enough to claim autonomous exploration, Goal1 carry-over/staging, or exit success.

## Design summary

The proposal defines a future Phase118 sequence:

```text
cleanup
  -> visible stack
  -> current preflight
  -> dispatch only if preflight.passed == true and preflight.failed_gates == []
  -> send exactly the explicit inner-ingress goal
  -> wait bounded result
  -> classify
  -> stop without maze_explorer
```

Locked goal:

```text
explicit inner-ingress goal
frame_id=map
x=2.0
y=0.0
yaw=0.0
```

Strict dispatch preconditions:

```text
preflight.passed == true
preflight.failed_gates == []
preflight.ingress_preflight_reject_reason absent/null
action_server_ready == true
```

If any precondition fails:

```text
dispatch_precondition_failed
PREFLIGHT_FAILED_NO_DISPATCH
ingress_goal_sent=false
maze_explorer_started=false
```

## Required artifact design

Future Phase118 must record at least:

- `goal_pose`
- `frame_id`
- `stamp`
- `x`
- `y`
- `yaw`
- `action_server_ready`
- `send_time`
- `send_wall_time_sec`
- `send_ros_time_sec`
- `accepted`
- `rejected`
- `result_status`
- `result_status_label`
- `abort_text`
- `bounded_goal_result_wait_sec`
- `cancel_requested`
- `cancel_result`
- `ingress_goal_sent`
- `maze_explorer_started`

Critical state rule:

```text
ingress_goal_sent=false before send
ingress_goal_sent=true only after accepted
```

## Outcome classifications designed

The proposal defines these future Phase118 classifications:

```text
PREFLIGHT_FAILED_NO_DISPATCH
INGRESS_DISPATCH_REJECTED_DIAGNOSTIC_FAIL
INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER
INGRESS_RESULT_ABORTED_DIAGNOSTIC_FAIL
INGRESS_RESULT_CANCELED_DIAGNOSTIC_FAIL
INGRESS_RESULT_TIMEOUT_DIAGNOSTIC_FAIL
INGRESS_DISPATCH_INSUFFICIENT_EVIDENCE
```

Rules:

- timeout/abort/cancel are diagnostic fail.
- rejected goal is diagnostic fail.
- goal response timeout is diagnostic fail.
- any insufficient evidence is diagnostic fail.
- even single-goal success stops with `maze_explorer_started=false`.
- no classification means autonomous exploration success.
- no classification means exit success.

## Phase118 allowed scope

The proposal defines Phase118 allowed scope as:

```text
single-goal dispatch smoke only
```

Phase118 may:

- cleanup.
- launch visible stack.
- run current preflight.
- if strict preconditions pass, dispatch the locked explicit inner-ingress goal once.
- wait for bounded result.
- record artifacts.
- stop.

Phase118 may not:

- start maze_explorer.
- validate exploration success.
- validate Goal1 carry-over.
- validate staging.
- validate exit success.
- run autonomous exploration.
- tune Nav2/MPPI/controller/config.
- change exploration strategy.

The proposal explicitly uses:

```text
max_goals=0
```

as the intended exploration-disabled setting for the future smoke.

## TDD evidence

Focused static tests were added first:

```text
src/tugbot_maze/test/test_phase117_controlled_ingress_goal_dispatch_design.py
```

RED was observed before proposal/report creation:

```text
7 failed
missing required Phase117 document: doc/doc_proposal/phase117_controlled_ingress_goal_dispatch_design.md
missing required Phase117 document: doc/doc_report/phase117_controlled_ingress_goal_dispatch_design_report.md
```

These focused static tests check:

- Phase117 is design-only/doc-only/no-goal.
- strict preflight pass gate is required before dispatch.
- locked inner-ingress goal identity is preserved.
- dispatch artifact schema is specified.
- `ingress_goal_sent` state transition is specified.
- bounded result wait and diagnostic-fail classifications are specified.
- cleanup/stop never starts maze_explorer.
- Phase118 scope is single-goal dispatch smoke only.
- no success/exit claims are allowed.

## Verification

Final verification command:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile src/tugbot_maze/test/test_phase117_controlled_ingress_goal_dispatch_design.py && \
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase117_controlled_ingress_goal_dispatch_design.py
```

Observed result is recorded in the final guard artifact after this report is written.

## Guardrails

- Design-only; no runtime behavior changed.
- No implementation was performed.
- No NavigateToPose goal was sent.
- No maze_explorer was started.
- No Nav2/MPPI/controller/config tuning was performed.
- No Nav2/MPPI/controller/inflation/robot_radius/clearance_radius/map threshold/config tuning was performed.
- No exploration strategy was changed.
- No branch scoring changed.
- No exploration order changed.
- No centerline gate changed.
- No directional readiness/fallback/terminal acceptance changed.
- Preflight was not removed.
- No autonomous exploration success or exit success is claimed.
- Phase118 not entered.

## Stop condition

Phase117 stops after the design proposal, design report, focused static tests, and guards complete. It waits for human acceptance and does not enter Phase118.
