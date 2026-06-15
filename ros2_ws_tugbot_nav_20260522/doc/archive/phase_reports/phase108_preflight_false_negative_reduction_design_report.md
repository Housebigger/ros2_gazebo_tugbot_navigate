# Phase108 ingress preflight false-negative reduction design report

Status: COMPLETE_DESIGN_REVIEW_ONLY_STOP_BEFORE_PHASE109

## Goal

Phase108 reviews how to keep the Phase105 ingress preflight fail-closed while reducing false-negative risk observed in Phase106/107.

Phase107 conclusion carried into this phase:

```text
PREFLIGHT_TOOL_FALSE_NEGATIVE
contributing: RAW_CAPTURE_LATER_THAN_PREFLIGHT
```

The Phase105 fail-closed direction remains correct: do not send the explicit inner-ingress goal when TF/controller/scan/goal-transform prerequisites are missing, stale, unstable, or ambiguous. Phase108 only designs better evidence handling so a future implementation is less likely to mislabel ambiguous or late-arriving readiness as definitive inactive/missing state.

## Work performed

No implementation was performed.

No Phase106 rerun was performed.

No ingress goal was sent.

No `maze_explorer` process was started.

No Nav2/MPPI/controller tuning was performed.

No exploration strategy was changed.

Phase108 produced design-only documents and optional static contract tests:

- `doc/doc_proposal/phase108_preflight_false_negative_reduction_design.md`
- `doc/doc_report/phase108_preflight_false_negative_reduction_design_report.md`
- `src/tugbot_maze/test/test_phase108_preflight_false_negative_reduction_design.py`

## Required context read

- `doc/doc_report/phase107_preflight_raw_capture_discrepancy_diagnosis_report.md`
- `doc/doc_report/phase106_preflighted_carry_over_bounded_goal1_staging_validation_report.md`
- `doc/doc_report/phase105_inner_ingress_tf_controller_preflight_hardening_minimal_implementation_report.md`
- `doc/doc_proposal/phase104_inner_ingress_tf_controller_preflight_hardening_design_review.md`
- `tools/phase105_inner_ingress_tf_controller_preflight.py`

## Design summary

The proposal preserves Phase105's fail-closed role, but recommends these future changes for Phase109-or-later implementation review:

1. `lifecycle_multi_source_confirmation`
   - Compare `ros2 lifecycle get`, node graph, `/navigate_to_pose` action availability, lifecycle-manager log markers, bond/managed-node markers, and related topic/service presence.
   - Confirm active only when sources agree.
   - If subprocess result conflicts with action/log/node evidence, classify as ambiguous, not definitive inactive.

2. `tf_consecutive_sampling_stable_window`
   - Require consecutive passing TF samples across a stable window.
   - Record `map->base_link`, `map->odom`, and `odom->base_link` availability, age, finite transform, timestamp regression, and pose jump evidence per sample.
   - Avoid treating a single early miss as the final hard reason when later samples recover.

3. `scan_wait_for_first_sample`
   - Subscribe to `/scan` immediately.
   - During bounded startup grace, classify no scan as `waiting_for_first_scan`.
   - Reject as `ingress_first_scan_timeout` only if the bounded first-scan wait expires.

4. `per_sample_failure_reason_history`
   - Preserve full `sample_history`, not only a compact tail.
   - Require `failed_gates` and `failure_reasons_by_gate` for every sample.
   - Keep analyzer-ready fields for lifecycle, TF, scan, goal transform, and raw cross-check evidence.

5. `raw_style_snapshot_cross_check`
   - Take a raw-style snapshot immediately before reject.
   - Record scan/map/local_costmap/odom/TF/action/node availability in the same preflight artifact.
   - If it contradicts the gated preflight failure, emit ambiguity/contradiction diagnostics but still fail closed.

6. `lifecycle_ambiguous_not_inactive`
   - Do not flatten lifecycle subprocess timeout/error into `controller_server_active=false` when other evidence says active/ready.
   - Future artifact should distinguish `active_confirmed`, `inactive_confirmed`, `ambiguous`, `unavailable`, and `query_error`.

7. `startup_grace_plus_stable_window`
   - Split bounded wait into explicit `startup_grace_sec` and `stable_window_sec` semantics.
   - Allow TF buffer fill and first scan during startup grace.
   - Still require continuous hard-gate pass before any future ingress goal can be sent.

## Future reject and ambiguity tokens

The design keeps existing Phase105 tokens:

```text
ingress_tf_unstable
ingress_map_base_tf_missing
ingress_map_odom_tf_stale
ingress_odom_base_tf_stale
ingress_scan_transform_unstable
ingress_controller_robot_pose_unavailable
ingress_goal_pose_transform_unavailable
ingress_preflight_timeout
```

The design proposes future diagnostic tokens:

```text
ingress_lifecycle_ambiguous
ingress_first_scan_timeout
ingress_tf_stable_window_not_met
ingress_raw_snapshot_cross_check_failed
```

These are fail-closed diagnostic tokens. They do not authorize sending a goal.

## Explicit safety statements

This Phase108 review does not prove the Phase105 bug is fixed.

This Phase108 review does not prove it is safe to send an ingress goal.

This Phase108 review does not validate Phase101 carry-over or Goal1 staging.

A future preflight pass would only be a prerequisite for sending the unchanged explicit inner-ingress goal in a later phase. It would not be autonomous exploration success or exit success.

## Guardrails

- Design-only; no runtime behavior changed.
- Preflight remains fail-closed.
- Preflight is not removed.
- No Phase106 rerun.
- No ingress goal sent.
- No `maze_explorer` started.
- No Phase88/92/101/105 runtime logic changed.
- No branch scoring changed.
- No exploration order changed.
- No centerline gate changed.
- No directional readiness/fallback/terminal acceptance changed.
- No Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No autonomous exploration success claim.
- No exit success claim.
- Phase109 not entered.

## Optional static tests

A design-only static contract test was added to ensure the proposal/report keep the required Phase108 boundaries and design topics:

```text
src/tugbot_maze/test/test_phase108_preflight_false_negative_reduction_design.py
```

Initial RED observed before creating proposal/report:

```text
5 failed
missing required Phase108 document: doc/doc_proposal/phase108_preflight_false_negative_reduction_design.md
missing required Phase108 document: doc/doc_report/phase108_preflight_false_negative_reduction_design_report.md
```

Final verification is recorded below.

## Verification

Static test and syntax commands:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase108_preflight_false_negative_reduction_design.py && \
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile src/tugbot_maze/test/test_phase108_preflight_false_negative_reduction_design.py
```

Observed result:

```text
.....                                                                    [100%]
5 passed in 0.01s
```

Guard/cleanup commands:

```bash
git diff -- tools/phase105_inner_ingress_tf_controller_preflight.py src/tugbot_navigation/config && \
ps -eo pid,cmd | grep -E 'phase108|phase107_preflight|phase106_preflighted|record_explorer_state_series|maze_explorer|ros2 launch tugbot_bringup tugbot_maze_slam_nav|gz sim|rviz2|slam_toolbox|controller_server|planner_server|bt_navigator|ros_gz_bridge|parameter_bridge' | grep -v grep || true && \
find . -name __pycache__ -type d
```

Observed result after deleting test-created `src/tugbot_maze/test/__pycache__`: empty output for diff/process/cache checks. This means no Phase105 source diff, no Nav2 config diff, no matching ROS/Gazebo/Nav2/maze_explorer runtime residues, and no remaining `__pycache__` directories were found.

`git status --short` still reports broad untracked project paths such as `doc/doc_proposal/`, `doc/doc_report/`, `src/tugbot_maze/test/`, `src/tugbot_navigation/config/`, and `tools/phase105_inner_ingress_tf_controller_preflight.py` because this workspace/repo has many existing untracked phase artifacts and source paths. The Phase108 guard used `git diff` for modified tracked content and found no changes to Phase105 preflight source or Nav2 config.

## Conclusion

Phase108 is complete as a design review. The recommended future direction is to preserve fail-closed ingress preflight behavior while reducing false negatives through multi-source lifecycle confirmation, explicit ambiguity states, bounded first-scan wait, consecutive TF stable-window sampling, per-sample failure artifacts, pre-reject raw-style snapshot cross-check, and separate startup-grace/stable-window timing.

The phase is waiting for human acceptance before Phase109. Phase109 not entered.
