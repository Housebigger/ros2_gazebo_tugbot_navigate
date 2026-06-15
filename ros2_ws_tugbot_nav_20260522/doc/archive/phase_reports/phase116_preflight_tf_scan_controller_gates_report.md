# Phase116 preflight TF/scan/controller gates report

Status: PHASE116_PREFLIGHT_TF_SCAN_CONTROLLER_GATES_COMPLETE_STOP_BEFORE_PHASE117

## Goal

Phase116 diagnoses the TF/scan/controller gates that remained after Phase115 resolved lifecycle ambiguity. This phase is diagnosis-only and no-goal: it may add lightweight preflight instrumentation, focused tests, an analyzer, and one real visible-stack preflight-only rerun, but it must not send a NavigateToPose goal, start `maze_explorer`, tune Nav2, or claim navigation success.

## Scope and guardrails

Allowed:

- Add focused tests.
- Add `tools/analyze_phase116_preflight_tf_scan_controller_gates.py`.
- Add lightweight diagnostic instrumentation to `tools/phase105_inner_ingress_tf_controller_preflight.py`.
- Run one visible-stack no-goal preflight-only rerun.

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
- Phase117 not entered.

## Phase115 carry-over context

Phase115 real no-goal rerun resolved lifecycle ambiguity by strict multi-source confirmation:

- `active_confirmed_by=multi_source_query_timeout_override`
- node graph + action + lifecycle service + launch active marker all agreed.

But the overall Phase115 preflight remained fail-closed with TF/scan/controller related gates:

- `ingress_map_base_tf_missing`
- `ingress_map_odom_tf_stale`
- `ingress_odom_base_tf_stale`
- `ingress_scan_transform_unstable`
- `ingress_controller_robot_pose_unavailable`
- `ingress_preflight_timeout`

Phase116 therefore targeted whether these were startup delay, TF buffer fill delay, frame mismatch, sim-time stamp age, or real TF absence.

## Implementation summary

Updated preflight instrumentation in `tools/phase105_inner_ingress_tf_controller_preflight.py`:

- TF diagnostics now record per lookup:
  - `sample_wall_time_sec`
  - `sample_ros_time_sec`
  - `can_transform_duration_sec`
  - `lookup_duration_sec`
  - `failure_reason`
  - `buffer_frames`
- `internal_sampling_diagnostics.tf_buffer_frame_list` records the TF buffer frame list.
- `scan_transform_check` records:
  - `age_sec`
  - `transform_age_sec`
  - `failure_reason`
  - `scan_transform_failure_reason`
- `controller_pose_check` records:
  - `robot_pose_unavailable_reason`
- New helper:
  - `derive_controller_pose_unavailable_reason(...)`
- The helper distinguishes:
  - `map_base_tf_missing`
  - `map_base_tf_stale`
  - `map_base_tf_lookup_exception_or_cache`
  - `controller_action_or_state_insufficient`
  - `map_base_tf_non_finite`
  - `robot_pose_unavailable_unknown`

This is instrumentation only. It does not weaken or change pass/fail gates.

Added analyzer:

- `tools/analyze_phase116_preflight_tf_scan_controller_gates.py`

Analyzer outputs:

- internal TF timeline for:
  - `map->base_link`
  - `map->odom`
  - `odom->base_link`
- optional external TF timeline comparison from readiness-window `tf2_echo` captures.
- scan frame/age/transform timeline.
- controller direct reason sequence.
- no-goal guard validation.

Added focused tests:

- `src/tugbot_maze/test/test_phase116_preflight_tf_scan_controller_gates.py`

## TDD evidence

RED evidence:

- Initial Phase116 tests failed as expected:
  - missing TF diagnostic fields (`sample_wall_time_sec`, `sample_ros_time_sec`, `can_transform_duration_sec`, `lookup_duration_sec`, `failure_reason`, `tf_buffer_frame_list`, `robot_pose_unavailable_reason`, `scan_transform_failure_reason`)
  - missing `derive_controller_pose_unavailable_reason`
  - missing Phase116 analyzer file

GREEN evidence after implementation:

- `PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile tools/phase105_inner_ingress_tf_controller_preflight.py tools/analyze_phase116_preflight_tf_scan_controller_gates.py src/tugbot_maze/test/test_phase116_preflight_tf_scan_controller_gates.py`
- `PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase116_preflight_tf_scan_controller_gates.py -k 'not report_guardrails'`
- Result: `4 passed, 1 deselected in 0.05s`.

Static regression before runtime:

- Phase109-116 focused tests excluding report guards:
  - `39 passed, 8 deselected in 0.43s`

## Runtime artifacts

Artifact directory:

- `log/phase116_preflight_tf_scan_controller_gates/`

Pre-rerun cleanup:

- `phase116_preflight_tf_scan_controller_gates_pre_rerun_cleanup_summary.json`
- before_count=0
- after_count=0

Visible stack launch:

- `phase116_preflight_tf_scan_controller_gates_launch.log`
- Gazebo visible (`headless:=false`).
- RViz enabled (`use_rviz:=true`).
- Stack `use_sim_time:=true`.
- No `maze_explorer` launch.
- No goal dispatch.

Readiness and external TF timeline:

- `phase116_preflight_tf_scan_controller_gates_ros_graph_ready.txt`
- `phase116_preflight_tf_scan_controller_gates_external_tf_timeline.json`

External readiness summary:

- `/navigate_to_pose` action available: yes.
- `/map` topic available: yes.
- `/scan` topic available: yes.
- lifecycle services present: yes.
- launch active marker present: yes.
- external `tf2_echo` timeline captured for:
  - `map->base_link`
  - `map->odom`
  - `odom->base_link`

External TF observations:

- `map->base_link` was not available in the external `tf2_echo` readiness timeline; latest external sample still reported invalid/missing `map` for that pair.
- `map->odom` became available in external `tf2_echo` output.
- `odom->base_link` became available in external `tf2_echo` output.

Real no-goal preflight-only artifact:

- `phase116_preflight_tf_scan_controller_gates_ingress_preflight.json`

Analyzer artifact:

- `phase116_preflight_tf_scan_controller_gates_analysis.json`

Minimal summary:

- `phase116_preflight_tf_scan_controller_gates_minimal_summary.md`

Preflight stdout:

```json
{"failed_gates": [], "passed": true, "reject_reason": null}
```

Important: this is only a no-goal preflight pass. It does not send the inner-ingress goal, does not start maze exploration, and does not prove navigation/autonomous exploration/exit success.

## Final analyzer classification

Classification:

- `TF_SCAN_CONTROLLER_GATES_PASSED_NO_GOAL`

Findings:

- `STARTUP_TF_BUFFER_FILL_DELAY`
- `SCAN_TRANSFORM_RECOVERED_AFTER_TF_BUFFER_FILL`
- `CONTROLLER_POSE_RECOVERED_AFTER_MAP_BASE_TF_AVAILABLE`
- `SCAN_FRAME_NAMESPACED_BUT_MATCHED_BY_TF_WHEN_BUFFER_FILLED`

## Internal TF/scan/controller diagnosis

Preflight sample count:

- 3

Bounded elapsed:

- 16.349758066004142 sec

TF timeline:

- `map->base_link`
  - initial_missing_internal: true
  - recovered_internal: true
  - ever_available_internal: true
  - latest internal age: 0.006000000000000227 sec
  - external `tf2_echo` timeline did not observe this pair available during readiness sampling.
- `map->odom`
  - initial_missing_internal: false
  - recovered_internal: false
  - ever_available_internal: true
  - external `tf2_echo` observed this pair available.
- `odom->base_link`
  - initial_missing_internal: true
  - recovered_internal: true
  - ever_available_internal: true
  - external `tf2_echo` observed this pair available.

Initial internal `map->base_link` failure:

- lookup_exception: `Could not find a connection between 'map' and 'base_link' because they are not part of the same tree.Tf has two or more unconnected trees.`
- sample_ros_time_sec: 0.0
- buffer_frames_present: true

This points to startup / buffer fill / tree-connection delay, not a permanent frame name mismatch.

Scan timeline:

- scan frame: `tugbot/scan_omni/scan_omni`
- first frame elapsed: 8.85162564300117 sec
- latest scan age: 0.009000000000000341 sec
- latest map->scan transform age: 0.006000000000000227 sec
- transform_available: recovered to true

Controller direct reason sequence:

- first sample: `map_base_tf_lookup_exception`
- later samples: null
- ever_robot_pose_available: true

This means `controller_robot_pose_unavailable` was directly caused by early `map->base_link` lookup failure, and recovered once internal TF connected.

## Required Phase116 judgments

1. Readiness-window external tf2_echo timeline captured:
   - Yes. Stored in `phase116_preflight_tf_scan_controller_gates_external_tf_timeline.json` for `map->base_link`, `map->odom`, and `odom->base_link`.

2. Preflight internal TF can_transform/lookup per sample captured with failure reason/timestamps/age/buffer frame list:
   - Yes. Stored in `internal_sampling_diagnostics.tf_buffer` and `internal_sampling_diagnostics.tf_buffer_frame_list`.

3. `/scan` frame and map->scan_frame transform availability/age/failure reason captured:
   - Yes. Scan frame was `tugbot/scan_omni/scan_omni`; latest scan age was about 0.009 sec; latest transform age was about 0.006 sec.

4. `controller_robot_pose_unavailable` direct reason captured:
   - Yes. Initial reason was `map_base_tf_lookup_exception`; later samples recovered to null once robot pose became available.

5. Startup delay vs buffer fill vs mismatch vs sim-time age vs true TF absence distinguished:
   - Best-supported diagnosis: startup TF buffer/tree-connection fill delay.
   - No evidence of sim-time age problem: final scan/TF ages were fresh.
   - No final frame name mismatch: namespaced scan frame matched when TF buffer filled.
   - No persistent true TF absence inside preflight: internal `map->base_link`, `map->odom`, and `odom->base_link` all became available.
   - External `tf2_echo map->base_link` did not observe availability during readiness sampling, so external CLI evidence remained weaker than internal preflight evidence for that pair.

6. No-goal guard maintained:
   - Yes. `ingress_goal_sent=false`, `maze_explorer_started=false`, `no_goal_dispatch_guard_valid=true`.

## Cleanup after runtime

Final cleanup artifact:

- `phase116_preflight_tf_scan_controller_gates_final_cleanup_summary.json`
- before_count=12
- after_count=0
- unrelated_processes_targeted=false

## Final conclusion

Phase116 completed the TF/scan/controller gate diagnosis. The real no-goal preflight-only rerun passed after internal TF buffer fill, scan transform recovery, and controller robot pose recovery. The remaining Phase115 gates are best explained as startup TF buffer/tree-connection delay rather than permanent frame mismatch, sim-time age error, or persistent TF absence.

This phase deliberately stops at preflight-only validation. No goal was sent, no maze explorer was started, no Nav2 configuration was tuned, and no navigation/exploration/exit success is claimed.

Stop here for human acceptance. Phase117 not entered.
