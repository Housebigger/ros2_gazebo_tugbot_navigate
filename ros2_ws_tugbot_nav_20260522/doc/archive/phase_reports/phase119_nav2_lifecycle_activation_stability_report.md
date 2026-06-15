# Phase119 Nav2 lifecycle activation stability diagnosis report

Status: PHASE119_NAV2_LIFECYCLE_ACTIVATION_STABILITY_ANALYSIS_COMPLETE_MANUAL_PROCESS_CLEANUP_CONFIRMED_PYCACHE_GUARD_NOT_EMPTY_STOP_BEFORE_PHASE120

## Goal

Phase119 diagnoses why Phase116 could pass the no-goal ingress preflight while the Phase118 visible-stack run failed closed before dispatch with lifecycle evidence not strictly active.

This phase is diagnosis-only. It focuses on Nav2 bringup/lifecycle readiness stability and timing. It does not send a NavigateToPose goal, does not start `maze_explorer`, does not tune Nav2, and does not change exploration strategy.

## Scope and guardrails

Allowed in Phase119:

- Add a Nav2 lifecycle activation stability analyzer.
- Add focused static tests.
- Add launch/readiness/lifecycle timeline capture.
- Run a real visible-stack no-goal preflight-only rerun.
- Compare Phase116 pass evidence, Phase118 no-dispatch failure evidence, and Phase119 rerun evidence.

Guarded / not changed:

- No NavigateToPose goal was sent.
- `ingress_goal_sent=false`.
- `maze_explorer_started=false`.
- No maze_explorer was started.
- No Nav2/MPPI/controller/config tuning was performed.
- No Nav2/MPPI/controller/inflation/robot_radius/clearance_radius/map threshold/config tuning was performed.
- No exploration strategy, branch scoring, exploration order, centerline gate, readiness/fallback/terminal acceptance logic was changed.
- Preflight was not removed.
- No autonomous exploration success or exit success is claimed.
- Phase120 not entered.

## Delivered files

Analyzer:

```text
tools/analyze_phase119_nav2_lifecycle_activation_stability.py
```

Focused tests:

```text
src/tugbot_maze/test/test_phase119_nav2_lifecycle_activation_stability.py
```

Runtime artifacts:

```text
log/phase119_nav2_lifecycle_activation_stability/
```

Report:

```text
doc/doc_report/phase119_nav2_lifecycle_activation_stability_report.md
```

## Phase118 baseline failure context

Phase118 visible-stack run correctly classified:

```text
PREFLIGHT_FAILED_NO_DISPATCH
```

Important Phase118 evidence:

- `/navigate_to_pose` action was present in the readiness snapshot.
- Preflight failed closed:
  - `preflight_passed=false`
  - `reject_reason=ingress_preflight_timeout`
  - `failed_gates` included `ingress_lifecycle_ambiguous` and `ingress_preflight_timeout`.
- Dispatch was blocked:
  - `dispatch_attempted=false`
  - `ingress_goal_sent=false`
  - `maze_explorer_started=false`
- Strict lifecycle evidence was not satisfied:
  - `bt_navigator.active_confirmed=false`
  - missing `launch_active_marker`
- Post-fail snapshot showed lifecycle not ready enough for dispatch:
  - `/controller_server`: `inactive [2]`
  - `/bt_navigator`: `unconfigured [1]`
- Phase118 launch log did not contain `Managed nodes are active` before the preflight failed.
- The log ended around Nav2 configuration/collision monitor service response timeout, before lifecycle activation completion.

Interpretation:

```text
/navigate_to_pose action present is not equivalent to bt_navigator active.
```

Phase118 correctly did not send a goal.

## Phase116 pass contrast

Phase116 visible-stack no-goal preflight-only rerun passed:

```json
{"failed_gates": [], "passed": true, "reject_reason": null}
```

Phase116 lifecycle evidence included:

- `Managed nodes are active` marker present.
- `bt_navigator.active_confirmed=true`.
- `controller_server.active_confirmed=true`.
- `bt_navigator.active_confirmed_by=multi_source_query_timeout_override` when direct `ros2 lifecycle get` timed out but graph/action/service/launch marker agreed.
- `ingress_goal_sent=false`.
- `maze_explorer_started=false`.

This means Phase116 pass was not based on action presence alone; it required the launch-active marker plus other evidence.

## Phase119 implementation summary

The analyzer reads a Phase119 artifact containing:

- Launch args:
  - `autostart`
  - `use_sim_time`
  - `use_composition`
  - `use_respawn`
  - `headless`
  - `use_rviz`
- Lifecycle/readiness timeline samples for:
  - `lifecycle_manager_navigation`
  - `controller_server`
  - `bt_navigator`
  - `planner_server`
  - `behavior_server`
  - `map_server`
  - `slam_toolbox`
- Launch-log lifecycle transition events and `Managed nodes are active` marker.
- Preflight start/end window and result.
- Guardrails:
  - `ingress_goal_sent=false`
  - `maze_explorer_started=false`

Analyzer classifications implemented/used:

```text
NAV2_LIFECYCLE_ACTIVATION_STABLE_PREFLIGHT_PASS
NAV2_LIFECYCLE_ACTIVE_MARKER_PREFLIGHT_PASS_QUERY_TIMEOUTS
NAV2_ACTIVATION_DELAY_OR_PREFLIGHT_TOO_EARLY
ACTION_PRESENT_WITH_INACTIVE_LIFECYCLE_WINDOW
MANAGED_NODES_NOT_ACTIVE_IN_CAPTURE
NAV2_LIFECYCLE_STABILITY_INSUFFICIENT_EVIDENCE
```

Important distinction added during Phase119 analysis:

- `inactive` / `unconfigured` are treated as definite not-active states.
- repeated `ros2 lifecycle get` timeout is treated separately as CLI query timeout evidence, not automatically as inactive/unconfigured.

## TDD evidence

RED observed before analyzer existed:

```text
4 failed, 1 deselected
missing required Phase119 analyzer: tools/analyze_phase119_nav2_lifecycle_activation_stability.py
```

The tests cover:

- action-present but lifecycle-inactive contradiction detection.
- stable activation with launch marker before preflight.
- activation delay / preflight-too-early classification.
- launch log parser extraction of lifecycle transition, marker, and bond/restart failure events.
- report no-goal/no-maze/no-overclaim guardrails.

GREEN after analyzer implementation:

```text
4 passed, 1 deselected in 0.01s
```

Static regression before runtime:

```text
19 passed, 3 deselected in 0.09s
```

## Runtime execution

Run id:

```text
phase119_nav2_lifecycle_activation_stability
```

Pre-runtime cleanup:

```json
{
  "before_count": 6,
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

Actual launch args recorded:

```text
autostart=true
use_sim_time=true
use_composition=False
use_respawn=False
headless=false
use_rviz=true
```

Phase119 artifacts include:

```text
log/phase119_nav2_lifecycle_activation_stability/phase119_nav2_lifecycle_activation_stability_launch.log
log/phase119_nav2_lifecycle_activation_stability/phase119_nav2_lifecycle_activation_stability_lifecycle_timeline.jsonl
log/phase119_nav2_lifecycle_activation_stability/phase119_nav2_lifecycle_activation_stability_artifact.json
log/phase119_nav2_lifecycle_activation_stability/phase119_nav2_lifecycle_activation_stability_artifact_enriched.json
log/phase119_nav2_lifecycle_activation_stability/phase119_nav2_lifecycle_activation_stability_preflight.json
log/phase119_nav2_lifecycle_activation_stability/phase119_nav2_lifecycle_activation_stability_analysis.json
log/phase119_nav2_lifecycle_activation_stability/phase119_nav2_lifecycle_activation_stability_minimal_summary.md
```

## Phase119 runtime result

Preflight result:

```text
passed=true
failed_gates=[]
reject_reason=null
sample_count=3
bounded_wait_elapsed_sec=19.356734289001906
```

Guardrails:

```text
ingress_goal_sent=false
maze_explorer_started=false
```

Final analyzer classification:

```text
NAV2_LIFECYCLE_ACTIVE_MARKER_PREFLIGHT_PASS_QUERY_TIMEOUTS
```

Analyzer validity:

```text
valid=true
```

Analyzer findings:

```text
LIFECYCLE_CLI_QUERY_TIMEOUT_WITH_ACTION_PRESENT
MANAGED_NODES_NEVER_ALL_ACTIVE_IN_CAPTURE
QUERY_RACE_OR_CLI_TIMEOUT_DESPITE_MULTI_SOURCE_PREFLIGHT_PASS
```

Important: `MANAGED_NODES_NEVER_ALL_ACTIVE_IN_CAPTURE` here refers only to the external timeline sampler's direct `ros2 lifecycle get` results. The preflight itself confirmed active by the Phase115/116 multi-source rule because direct lifecycle CLI calls timed out while graph/action/service/launch marker agreed.

## Launch log lifecycle timeline

Phase119 launch log shows Nav2 lifecycle progression completed:

```text
Configuring bt_navigator
Activating controller_server
Server controller_server connected with bond
Activating planner_server
Server planner_server connected with bond
Activating behavior_server
Server behavior_server connected with bond
Activating bt_navigator
Server bt_navigator connected with bond
Managed nodes are active
```

Key lines from the launch log:

```text
line 224: Configuring bt_navigator
line 237: Activating controller_server
line 246: Server controller_server connected with bond
line 251: Activating planner_server
line 263: Server planner_server connected with bond
line 268: Activating behavior_server
line 276: Server behavior_server connected with bond
line 285: Activating bt_navigator
line 288: Server bt_navigator connected with bond
line 297: Managed nodes are active
```

No bond loss, node restart, or lifecycle activation failure was observed in the Phase119 launch log.

## Action-present vs active-state window

Phase119 external timeline observed `/navigate_to_pose` action present while direct `ros2 lifecycle get` calls repeatedly timed out for managed nodes.

Counts from analyzer:

```text
action_present_bt_not_active_count=0
action_present_any_required_not_active_count=0
action_present_cli_query_timeout_count=8
```

This is not the same as the Phase118 failure. Phase118 had action present plus missing launch-active marker and inactive/unconfigured post-fail states. Phase119 had action present plus launch-active marker plus preflight multi-source active confirmation, but external direct CLI lifecycle queries timed out.

Diagnosis:

```text
Phase119 reproduced lifecycle CLI query race/timeout behavior, but not a persistent inactive/unconfigured lifecycle state.
```

## Root-cause diagnosis for Phase116 vs Phase118 difference

The Phase118 no-dispatch failure is best explained as lifecycle activation/readiness timing rather than permanent Nav2 failure.

Evidence:

1. Phase118 launched the same visible stack args but preflight started while the launch log had not reached `Managed nodes are active`.
2. Phase118 post-fail lifecycle snapshot showed `controller_server inactive [2]` and `bt_navigator unconfigured [1]` while `/navigate_to_pose` action was already present.
3. Phase119 repeated visible no-goal preflight-only run with the same launch args and observed the launch active marker plus successful preflight.
4. Phase119 launch log shows lifecycle manager completed activation and all relevant bond connections.
5. Direct `ros2 lifecycle get` remained timeout-prone under load, but the preflight multi-source confirmation handled this when marker/action/service/graph agreed.

Classification of likely cause:

```text
activation delay / preflight-too-early in Phase118, plus lifecycle CLI query race/timeout under visible-stack load.
```

Not supported by Phase119 evidence:

- persistent Nav2 lifecycle failure
- permanent frame mismatch
- sim-time age error
- continuous TF absence
- bond loss or managed node restart
- need for Nav2/MPPI/controller/config tuning

## Phase119 answer to required focus points

1. lifecycle_manager/controller_server/bt_navigator/planner_server/behavior_server/map_server/slam_toolbox timeline:
   - recorded in `phase119_nav2_lifecycle_activation_stability_lifecycle_timeline.jsonl` and analyzer output.
   - launch log confirms lifecycle_manager activated controller_server, planner_server, behavior_server, bt_navigator and reached `Managed nodes are active`.
   - map_server is not expected in this SLAM launch; SLAM toolbox is present.

2. launch log lifecycle transition and marker timeline:
   - recorded in launch log and analyzer marker timeline.
   - `Managed nodes are active` present in Phase119; absent in Phase118 before failure.

3. `/navigate_to_pose` action present vs bt_navigator active contradiction:
   - Phase118: contradiction existed with inactive/unconfigured post-fail states and missing launch marker.
   - Phase119: no definite inactive/unconfigured contradiction; instead direct lifecycle CLI timed out while marker and preflight multi-source evidence passed.

4. autostart/use_sim_time/use_composition/use_respawn actual values:
   - `autostart=true`
   - `use_sim_time=true`
   - `use_composition=False`
   - `use_respawn=False`

5. preflight start/end managed-node state:
   - external direct CLI sampler timed out for lifecycle gets at start/end under visible-stack load.
   - preflight internal lifecycle sources confirmed active by multi-source query-timeout override because graph/action/service/launch marker agreed.

6. inactive/unconfigured cause classification:
   - Phase118 inactive/unconfigured evidence was caused by preflight starting before lifecycle manager completed activation / launch active marker appeared.
   - Phase119 did not reproduce inactive/unconfigured after marker; it reproduced direct CLI query timeout only.

7. no-goal/no-maze preserved:
   - `ingress_goal_sent=false`
   - `maze_explorer_started=false`

## Cleanup and final guards

Phase119 runtime cleanup history must distinguish Hermes-attempted cleanup from manual external cleanup.

Hermes cleanup attempts:

- Hermes attempted scoped Phase119 final cleanup after the runtime diagnosis.
- The execution environment blocked the cleanup command with `BLOCKED: User denied this command`.
- Hermes did not retry, rephrase, or bypass that blocked cleanup command.
- Therefore the report must not claim that Hermes automatically completed final cleanup.

Manual external cleanup:

- The user reported that Phase119 residual processes were manually cleaned from an external terminal.
- Hermes then performed only read-only verification, as requested.

Read-only verification after manual cleanup:

```text
runtime process guard: empty
tracked Nav2 config diff guard: empty
Phase119 scoped pycache guard: not empty
```

Phase119 scoped pycache still present in the read-only guard:

```text
tools/__pycache__/analyze_phase119_nav2_lifecycle_activation_stability.cpython-312.pyc
src/tugbot_maze/test/__pycache__/test_phase119_nav2_lifecycle_activation_stability.cpython-312.pyc
```

Because the latest allowed scope was read-only checks plus document patching, Hermes did not delete these pycache files. As a result, the process cleanup was confirmed externally and read-only process/config guards passed, but the scoped pycache guard did not pass.

Prior guard artifacts remain available in:

```text
log/phase119_nav2_lifecycle_activation_stability/phase119_nav2_lifecycle_activation_stability_final_guard_bundle.txt
```

## Conclusion

Phase119 diagnoses the Phase118 failure as a lifecycle activation readiness timing problem, not a persistent Nav2 activation failure.

The key acceptance boundary remains unchanged:

```text
Do not treat /navigate_to_pose action presence alone as sufficient for dispatch.
```

A future phase that attempts dispatch should wait for the same-run preflight to pass strictly, which in practice requires either direct lifecycle active state or multi-source agreement including the `Managed nodes are active` marker. If the marker is missing and lifecycle states are inactive/unconfigured, the correct behavior remains no-dispatch.

Phase119 is complete. Stop here and wait for human acceptance. Do not enter Phase120.
