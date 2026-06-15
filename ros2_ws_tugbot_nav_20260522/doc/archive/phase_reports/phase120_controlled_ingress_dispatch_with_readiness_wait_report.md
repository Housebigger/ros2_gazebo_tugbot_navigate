# Phase120 controlled ingress dispatch with managed-active readiness wait report

Status: PHASE120_CONTROLLED_INGRESS_DISPATCH_WITH_READINESS_WAIT_COMPLETE_STOP_BEFORE_PHASE121

## Goal

Phase120 adds a minimal managed-active readiness wait before the existing Phase118 controlled ingress single-goal dispatch smoke. The purpose is to avoid starting preflight before Nav2 lifecycle activation reaches a stable same-run readiness point.

This phase is still a controlled single-goal dispatch smoke only. A successful result means only that the explicit inner-ingress goal succeeded after readiness wait and strict preflight. It is not autonomous exploration success and not exit success.

## Scope

Allowed:

- update the Phase118 runner/analyzer/tests for Phase120 readiness-wait evidence;
- launch visible Gazebo/RViz/SLAM/Nav2;
- after readiness wait and strict same-run preflight pass, send exactly one explicit inner-ingress goal: `frame_id=map`, `x=2.0`, `y=0.0`, `yaw=0.0`;
- bounded wait for result;
- cleanup after the run.

Forbidden / unchanged:

- no `maze_explorer` startup;
- no Goal1/carry-over/staging/branch/centerline/fallback/terminal/exit goal;
- no Nav2/MPPI/controller/config tuning;
- no exploration strategy change;
- no preflight removal;
- no autonomous exploration success or exit success claim;
- Phase121 not entered.

## Delivered files

Updated runner:

```text
tools/run_phase118_controlled_ingress_single_goal_dispatch_smoke.py
```

Updated analyzer:

```text
tools/analyze_phase118_controlled_ingress_single_goal_dispatch_smoke.py
```

Phase120 focused tests:

```text
src/tugbot_maze/test/test_phase120_controlled_ingress_dispatch_with_readiness_wait.py
```

Updated Phase119 report test to preserve the manual-cleanup / pycache-not-empty status:

```text
src/tugbot_maze/test/test_phase119_nav2_lifecycle_activation_stability.py
```

Runtime artifacts:

```text
log/phase120_controlled_ingress_dispatch_with_readiness_wait/
```

Report:

```text
doc/doc_report/phase120_controlled_ingress_dispatch_with_readiness_wait_report.md
```

## Implementation summary

The Phase118 runner now supports Phase120 mode via `--enable-readiness-wait`.

Phase120 mode:

```text
controlled_ingress_dispatch_with_managed_active_readiness_wait
```

Readiness wait rule:

- before preflight, wait for either:
  - launch log contains `Managed nodes are active`, or
  - multi-source readiness is satisfied by action readiness plus active lifecycle states;
- if readiness wait times out, do not run preflight and classify:
  - `READY_WAIT_TIMEOUT_NO_DISPATCH`;
- if readiness wait passes, run the current strict preflight;
- if preflight fails, classify:
  - `PREFLIGHT_FAILED_NO_DISPATCH`;
- only if preflight passes strictly and action server is ready may the runner send the single explicit inner-ingress goal;
- `ingress_goal_sent=true` only after the action goal is accepted;
- success is classified only as:
  - `INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER`.

## Required classifications

```text
READY_WAIT_TIMEOUT_NO_DISPATCH
PREFLIGHT_FAILED_NO_DISPATCH
INGRESS_DISPATCH_REJECTED_DIAGNOSTIC_FAIL
INGRESS_RESULT_TIMEOUT_DIAGNOSTIC_FAIL
INGRESS_RESULT_ABORTED_DIAGNOSTIC_FAIL
INGRESS_RESULT_CANCELED_DIAGNOSTIC_FAIL
INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER
```

## Required readiness wait evidence

The Phase120 artifact records:

```text
readiness_wait_start
readiness_wait_end
readiness_wait_start_wall_time_sec
readiness_wait_end_wall_time_sec
readiness_wait_elapsed_sec
readiness_wait_timeout_sec
marker_found
marker_found_wall_time_sec
multi_source_ready
timed_out
lifecycle_states
action_server_ready
samples
```

## TDD and static verification

RED before implementation:

```text
4 failed, 1 deselected
```

Expected RED reasons included missing Phase120 readiness-wait helpers and missing `READY_WAIT_TIMEOUT_NO_DISPATCH` support.

Focused GREEN after implementation:

```text
4 passed, 1 deselected in 0.01s
5 passed in 0.02s
```

Phase118 + Phase120 focused tests:

```text
10 passed in 0.03s
```

Phase116-120 static regression:

```text
27 passed in 0.10s
```

A Phase119 test was updated intentionally because Phase119 final status had already been corrected to the true manual-cleanup / pycache-not-empty state:

```text
PHASE119_NAV2_LIFECYCLE_ACTIVATION_STABILITY_ANALYSIS_COMPLETE_MANUAL_PROCESS_CLEANUP_CONFIRMED_PYCACHE_GUARD_NOT_EMPTY_STOP_BEFORE_PHASE120
```

This prevents Phase120 regression tests from falsely requiring the older Phase119 `COMPLETE_STOP_BEFORE_PHASE120` status.

## Runtime execution

Run id:

```text
phase120_controlled_ingress_dispatch_with_readiness_wait
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

Runner command used Phase120 readiness wait:

```bash
python3 tools/run_phase118_controlled_ingress_single_goal_dispatch_smoke.py \
  --output log/phase120_controlled_ingress_dispatch_with_readiness_wait/phase120_controlled_ingress_dispatch_with_readiness_wait_artifact.json \
  --preflight-output log/phase120_controlled_ingress_dispatch_with_readiness_wait/phase120_controlled_ingress_dispatch_with_readiness_wait_preflight.json \
  --run-id phase120_controlled_ingress_dispatch_with_readiness_wait \
  --launch-log-path log/phase120_controlled_ingress_dispatch_with_readiness_wait/phase120_controlled_ingress_dispatch_with_readiness_wait_launch.log \
  --enable-readiness-wait \
  --readiness-wait-timeout-sec 60 \
  --readiness-wait-sample-period-sec 1.0 \
  --preflight-timeout-sec 35 \
  --tf-stability-window-sec 2.0 \
  --sample-period-sec 0.5 \
  --startup-grace-sec 2.0 \
  --action-server-wait-sec 10 \
  --bounded-goal-result-wait-sec 45
```

## Runtime result

Final classification:

```text
INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER
```

Analyzer validity:

```text
valid=true
classification_matches_dispatch=true
```

Readiness wait result:

```text
enabled=true
marker_found=true
multi_source_ready=false
timed_out=false
readiness_wait_timeout_sec=60.0
readiness_wait_elapsed_sec=17.718641757965088
action_server_ready=true
```

Lifecycle state samples during readiness wait still showed direct CLI lifecycle query timeouts under visible-stack load:

```json
{
  "controller_server": "query_timeout",
  "bt_navigator": "query_timeout",
  "planner_server": "query_timeout",
  "behavior_server": "query_timeout"
}
```

This matches the Phase119 diagnosis: direct CLI lifecycle queries can time out under visible-stack load, while the launch-active marker and preflight multi-source evidence can still safely establish readiness.

Launch log readiness evidence:

```text
Activating bt_navigator
Server bt_navigator connected with bond
Managed nodes are active
```

The `Managed nodes are active` marker appeared at launch-log line 307.

Preflight result after readiness wait:

```text
evaluated=true
passed=true
failed_gates=[]
ingress_preflight_reject_reason=null
bounded_wait_elapsed_sec=20.415148259024136
ingress_goal_sent=true
maze_explorer_started=false
```

The preflight lifecycle source evidence confirmed active by the same Phase115/116 multi-source rule:

```text
bt_navigator.active_confirmed=true
bt_navigator.active_confirmed_by=multi_source_query_timeout_override
controller_server.active_confirmed=true
controller_server.active_confirmed_by=multi_source_query_timeout_override
launch_active_marker.present=true
navigate_to_pose_action_ready=true
```

Dispatch result:

```text
dispatch_attempted=true
action_server_ready=true
accepted=true
rejected=false
result_received=true
result_status=4
result_status_label=SUCCEEDED
abort_text=
bounded_goal_result_wait_sec=45.0
cancel_requested=false
cancel_result=null
ingress_goal_sent=true
maze_explorer_started=false
```

Sent goal identity:

```json
{
  "frame_id": "map",
  "x": 2.0,
  "y": 0.0,
  "yaw": 0.0,
  "stamp": {
    "sec": 0,
    "nanosec": 0
  }
}
```

The action result succeeded; the launch log includes:

```text
Goal succeeded
```

## Important interpretation boundary

This is not an autonomous maze-navigation success.

Phase120 success means only:

```text
The single explicit inner-ingress NavigateToPose goal succeeded after managed-active readiness wait and strict preflight pass.
```

It does not mean:

- Goal1 dispatched;
- carry-over/staging was evaluated;
- exploration succeeded;
- exit was reached;
- autonomous navigation succeeded.

## Artifacts

Key artifacts:

```text
log/phase120_controlled_ingress_dispatch_with_readiness_wait/phase120_controlled_ingress_dispatch_with_readiness_wait_launch.log
log/phase120_controlled_ingress_dispatch_with_readiness_wait/phase120_controlled_ingress_dispatch_with_readiness_wait_artifact.json
log/phase120_controlled_ingress_dispatch_with_readiness_wait/phase120_controlled_ingress_dispatch_with_readiness_wait_preflight.json
log/phase120_controlled_ingress_dispatch_with_readiness_wait/phase120_controlled_ingress_dispatch_with_readiness_wait_analysis.json
log/phase120_controlled_ingress_dispatch_with_readiness_wait/phase120_controlled_ingress_dispatch_with_readiness_wait_analysis_final_guard.json
log/phase120_controlled_ingress_dispatch_with_readiness_wait/phase120_controlled_ingress_dispatch_with_readiness_wait_minimal_summary.md
log/phase120_controlled_ingress_dispatch_with_readiness_wait/phase120_controlled_ingress_dispatch_with_readiness_wait_minimal_summary_final_guard.md
log/phase120_controlled_ingress_dispatch_with_readiness_wait/phase120_controlled_ingress_dispatch_with_readiness_wait_final_test_and_analyzer_bundle.txt
log/phase120_controlled_ingress_dispatch_with_readiness_wait/phase120_controlled_ingress_dispatch_with_readiness_wait_final_guard_bundle.txt
```

## Cleanup and final guards

The Phase120 launch background process was stopped after the bounded smoke completed. Final guards passed:

```text
runtime process guard: empty
tracked Nav2 config diff guard: empty
preflight implementation Phase120 diff guard: empty
Phase120 scoped pycache guard: empty
```

No Nav2 config changes were made:

```text
src/tugbot_navigation/config diff: empty
```

No preflight implementation changes were made in Phase120:

```text
tools/phase105_inner_ingress_tf_controller_preflight.py diff: empty
```

## Guardrails

- No maze_explorer was started.
- `maze_explorer_started=false`.
- No Goal1/carry-over/staging/branch/centerline/fallback/terminal/exit goal was sent.
- No Nav2/MPPI/controller/config tuning was performed.
- No exploration strategy was changed.
- Preflight was not removed.
- No autonomous exploration success or exit success is claimed.
- Phase121 not entered.

## Conclusion

Phase120 confirms the Phase119 diagnosis: adding a managed-active readiness wait prevents the Phase118-style preflight-too-early failure for this run.

The same visible stack reached `Managed nodes are active`; the strict same-run preflight then passed; the runner sent exactly one locked explicit inner-ingress goal; Nav2 accepted it and returned `SUCCEEDED`; the runner stopped without starting `maze_explorer`.

Final status:

```text
PHASE120_CONTROLLED_INGRESS_DISPATCH_WITH_READINESS_WAIT_COMPLETE_STOP_BEFORE_PHASE121
```

Stop here and wait for human acceptance. Do not enter Phase121.
