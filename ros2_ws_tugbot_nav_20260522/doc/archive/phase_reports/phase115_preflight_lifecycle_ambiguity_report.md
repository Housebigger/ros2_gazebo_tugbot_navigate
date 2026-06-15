# Phase115 preflight lifecycle ambiguity report

Status: PHASE115_PREFLIGHT_LIFECYCLE_AMBIGUITY_COMPLETE_STOP_BEFORE_PHASE116

## Goal

Phase115 diagnoses and minimally fixes the Phase114 remaining lifecycle ambiguity: `ros2 lifecycle get` subprocesses time out while other readiness evidence says Nav2 is up. The phase remains no-goal, fail-closed, and limited to preflight tooling/diagnostics.

This phase does not send a NavigateToPose goal, does not start `maze_explorer`, does not tune Nav2/controller/config, and does not claim autonomous exploration or exit success.

## Scope and guardrails

Allowed:

- Modify `tools/phase105_inner_ingress_tf_controller_preflight.py`.
- Add Phase115 focused tests and analyzer.
- Run one real visible-stack no-goal preflight-only rerun.

Guarded / not changed:

- No NavigateToPose goal was sent.
- `ingress_goal_sent=false`.
- `maze_explorer_started=false`.
- No maze_explorer was started.
- No Nav2/MPPI/controller/config tuning was performed.
- No Nav2/MPPI/controller/inflation/robot_radius/clearance_radius/map threshold/config tuning was performed.
- No exploration strategy, branch scoring, exploration order, centerline gate, readiness/fallback/terminal acceptance logic was changed.
- Preflight was not removed.
- This phase does not prove autonomous exploration success.
- This phase does not prove exit success.
- Phase116 not entered.

## Root-cause hypothesis from Phase114

Phase114 recovered sim-time `/scan` and internal TF callback flow. The final Phase114 no-goal rerun still failed closed:

- `reject_reason=ingress_preflight_timeout`
- `failed_gates` included `ingress_lifecycle_ambiguous`
- `/scan` callback age was fresh.
- internal TF resolved `map->base_link`, `map->odom`, `odom->base_link`.

Phase115 focused on the remaining contradiction: lifecycle subprocess query timeout vs. node/action/service/launch readiness.

## Implementation summary

Updated preflight tool:

- `_ros_lifecycle_query(node_name, timeout_sec=...)` now records raw subprocess evidence:
  - command
  - timeout_sec
  - duration_sec
  - returncode
  - state
  - stdout
  - stderr
  - timed_out
  - query_error
- TimeoutExpired is handled explicitly and records partial stdout/stderr when present.
- Added `PreflightConfig.lifecycle_query_timeout_sec` and CLI `--lifecycle-query-timeout-sec`.
- Added optional launch active marker input:
  - `PreflightConfig.launch_log_path`
  - CLI `--launch-log-path`
- Runtime sample now records lifecycle diagnostics under both `controller_pose_check.lifecycle_sources` and `internal_sampling_diagnostics.lifecycle`.
- Runtime lifecycle sources now include:
  - node graph presence
  - `/navigate_to_pose` action availability
  - lifecycle service presence (`/controller_server/get_state`, `/bt_navigator/get_state`, etc.)
  - launch active marker presence from launch log
  - raw lifecycle query evidence
- Minimal multi-source active rule:
  - If lifecycle query explicitly returns active and graph/action agree, active is confirmed as `lifecycle_query_active`.
  - If lifecycle query times out, active is confirmed only when all strict sources agree:
    - node graph present
    - action available
    - lifecycle service present
    - launch active marker present
  - The timeout override is recorded as `active_confirmed_by=multi_source_query_timeout_override`.
  - Missing any strict source remains `ambiguous` and fail-closed.

No safety gate was weakened beyond this strict multi-source confirmation rule. The tool remains fail-closed when evidence is incomplete.

## Focused TDD / static verification

RED evidence:

- Phase115 focused tests initially failed because:
  - `_ros_lifecycle_query` did not accept `timeout_sec` and did not record command/duration/timed_out.
  - `derive_lifecycle_confirmation` did not accept lifecycle service or launch marker evidence.
  - `tools/analyze_phase115_preflight_lifecycle_ambiguity.py` did not exist.

GREEN evidence after implementation:

- `PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile tools/phase105_inner_ingress_tf_controller_preflight.py tools/analyze_phase115_preflight_lifecycle_ambiguity.py src/tugbot_maze/test/test_phase115_preflight_lifecycle_ambiguity.py`
- `PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase115_preflight_lifecycle_ambiguity.py -k 'not report_guardrails'`
- Result: `6 passed, 1 deselected in 0.05s`.

Static regression bundle before runtime:

- py_compile passed for Phase105/110/111/113/114/115 tools and Phase109-115 tests.
- Focused pytest without report guards:
  - `35 passed, 7 deselected in 0.38s`.

Phase115 analyzer added:

- `tools/analyze_phase115_preflight_lifecycle_ambiguity.py`
- It reports per-node lifecycle details for `controller_server` and `bt_navigator`, including timeout diagnostics and multi-source active confirmation evidence.

## Runtime cleanup and launch evidence

Artifact directory:

- `log/phase115_preflight_lifecycle_ambiguity/`

Pre-rerun cleanup:

- `phase115_preflight_lifecycle_ambiguity_pre_rerun_cleanup_summary.json`
- before_count=0
- after_count=0

Visible stack launch:

- Launch log: `phase115_preflight_lifecycle_ambiguity_launch.log`
- Gazebo visible (`headless:=false`).
- RViz enabled (`use_rviz:=true`).
- Stack `use_sim_time:=true`.
- No `maze_explorer` launch.
- No goal dispatch.

Readiness artifact:

- `phase115_preflight_lifecycle_ambiguity_ros_graph_ready.txt`

Readiness summary:

- `/navigate_to_pose` action available: yes.
- `/map` available: yes.
- `/scan` available: yes.
- lifecycle launch marker present: yes (`Managed nodes are active`).
- lifecycle services present:
  - `/controller_server/get_state`
  - `/controller_server/change_state`
  - `/bt_navigator/get_state`
  - `/bt_navigator/change_state`
- external `map->base_link` TF readiness did not become available during the readiness wait (`tf=0`). This limited the final overall preflight pass result, but did not block lifecycle-focused validation.

## Final no-goal preflight-only rerun

Preflight artifact:

- `phase115_preflight_lifecycle_ambiguity_ingress_preflight.json`

Analyzer artifact:

- `phase115_preflight_lifecycle_ambiguity_analysis.json`

Minimal summary:

- `phase115_preflight_lifecycle_ambiguity_minimal_summary.md`

Preflight stdout:

```json
{"failed_gates": ["ingress_map_base_tf_missing", "ingress_map_odom_tf_stale", "ingress_odom_base_tf_stale", "ingress_scan_transform_unstable", "ingress_controller_robot_pose_unavailable", "ingress_preflight_timeout"], "passed": false, "reject_reason": "ingress_preflight_timeout"}
```

Final Phase115 analyzer classification:

- `LIFECYCLE_AMBIGUITY_RESOLVED_FAIL_CLOSED_NEW_REASON`

Important: the overall preflight still failed closed because TF/scan/controller evidence was not sufficient in this runtime window. This is not a navigation success claim. The lifecycle-specific ambiguity from Phase114 was resolved by strict multi-source active confirmation.

## Per-node lifecycle diagnostics

Controller server:

- active_confirmed: true
- active_confirmed_by: `multi_source_query_timeout_override`
- query_command: `['ros2', 'lifecycle', 'get', '/controller_server']`
- query_timeout_sec: `2.0`
- query_duration_sec: `2.0076219880138524`
- query_returncode: `null`
- query_timed_out: true
- query_stdout: empty
- query_stderr: empty
- query_error_text: `timeout after 2.0 sec`
- node_graph_present: true
- action_available: true
- lifecycle_service_present: true
- launch_active_marker_present: true
- launch_active_marker_token: `Managed nodes are active`
- ambiguous_detail: `lifecycle subprocess timed out but graph/action/service/launch-active marker agree`

BT navigator:

- active_confirmed: true
- active_confirmed_by: `multi_source_query_timeout_override`
- query_command: `['ros2', 'lifecycle', 'get', '/bt_navigator']`
- query_timeout_sec: `2.0`
- query_duration_sec: `2.0073159619932994`
- query_returncode: `null`
- query_timed_out: true
- query_stdout: empty
- query_stderr: empty
- query_error_text: `timeout after 2.0 sec`
- node_graph_present: true
- action_available: true
- lifecycle_service_present: true
- launch_active_marker_present: true
- launch_active_marker_token: `Managed nodes are active`
- ambiguous_detail: `lifecycle subprocess timed out but graph/action/service/launch-active marker agree`

## Required Phase115 judgments

1. Lifecycle query command/timeout/stdout/stderr/returncode/duration recorded:
   - Yes, for both `controller_server` and `bt_navigator`.

2. Controller server and BT navigator diagnosed separately:
   - Yes, analyzer reports separate lifecycle sections for each node.

3. Subprocess timeout classified as query_error/ambiguous_detail rather than blocking all evidence interpretation:
   - Yes. Both lifecycle query timeouts remain visible as `query_error=true` and `query_timed_out=true`, but strict multisource evidence confirms active state.

4. Minimal multi-source active_confirmed rule implemented fail-closed:
   - Yes. Timeout override requires node graph + action + lifecycle service + launch active marker. Focused tests verify missing marker remains ambiguous.

5. If timeout remains, artifact gives interpretable reason:
   - Yes. Timeout duration, command, timeout value, stdout/stderr, returncode, and `ambiguous_detail` are recorded.

6. No-goal guard maintained:
   - Yes. `ingress_goal_sent=false`, `maze_explorer_started=false`, `no_goal_dispatch_guard_valid=true`.

## Cleanup after runtime

Final cleanup artifact:

- `phase115_preflight_lifecycle_ambiguity_final_cleanup_summary.json`
- before_count=17
- after_count=0
- unrelated_processes_targeted=false

## Final conclusion

Phase115 completed the lifecycle ambiguity diagnosis and minimal fix. The lifecycle subprocess timeout remains observable, but it no longer causes lifecycle ambiguity when strict independent evidence agrees that Nav2 is active. The real no-goal rerun classified this as `LIFECYCLE_AMBIGUITY_RESOLVED_FAIL_CLOSED_NEW_REASON`.

The overall preflight did not pass in this runtime window because TF/scan/controller evidence was still insufficient, and external `map->base_link` TF readiness was not achieved before the preflight-only run. This phase therefore stops with a lifecycle-focused fix only. It does not send any goal, does not start exploration, does not tune Nav2, and does not claim navigation success.

Stop here for human acceptance. Phase116 not entered.
