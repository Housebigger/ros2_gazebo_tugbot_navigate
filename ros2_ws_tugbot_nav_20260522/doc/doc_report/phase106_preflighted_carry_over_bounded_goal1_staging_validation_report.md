# Phase106 preflighted carry-over bounded Goal1 staging validation report

Status: RUNTIME_COMPLETED_PHASE105_PREFLIGHT_REJECTED_BEFORE_INGRESS

## Goal

Use the Phase102 wrapper flow after Phase105 fail-closed ingress preflight hardening to rerun Goal1 carry-over bounded validation. This phase validates only; it does not change `maze_explorer` strategy or any Phase88/92/101/105 runtime logic.

## Required reading completed

- `doc/doc_report/phase105_inner_ingress_tf_controller_preflight_hardening_minimal_implementation_report.md`
- `doc/doc_report/phase101_staging_corridor_evidence_carry_over_minimal_implementation_report.md`
- `doc/doc_report/phase102_carry_over_bounded_goal1_staging_validation_report.md`
- `doc/doc_report/phase103_inner_ingress_goal_failure_diagnosis_report.md`
- `doc/doc_report/phase99_goal1_staging_evidence_path_diagnosis_report.md`
- `doc/doc_report/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_report.md`
- `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`

## Added Phase106 files

- `src/tugbot_maze/test/test_phase106_preflighted_carry_over_bounded_goal1_staging_validation.py`
- `tools/analyze_phase106_preflighted_carry_over_bounded_goal1_staging_validation.py`
- `tools/record_phase106_preflighted_carry_over_validation_evidence.py`
- `tools/run_phase106_preflighted_carry_over_bounded_goal1_staging_validation.sh`
- `doc/doc_report/phase106_preflighted_carry_over_bounded_goal1_staging_validation_runbook.md`
- `doc/doc_report/phase106_preflighted_carry_over_bounded_goal1_staging_validation_report.md`

## Classification set

The Phase106 analyzer emits exactly one of:

- `PREFLIGHTED_CARRY_OVER_APPLIED_STAGING_APPLIED`
- `PREFLIGHTED_CARRY_OVER_APPLIED_STAGING_SAFETY_REJECTED`
- `PREFLIGHTED_CARRY_OVER_REJECTED`
- `PREFLIGHTED_CARRY_OVER_NOT_TRIGGERED`
- `PHASE105_PREFLIGHT_REJECTED_BEFORE_INGRESS`
- `PREFLIGHTED_INGRESS_FAILED_AFTER_PREFLIGHT_PASS`
- `PREFLIGHTED_GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE`

## Runtime command

```bash
PHASE106_MAX_GOALS=1 \
PHASE106_INGRESS_TIMEOUT_SEC=90 \
PHASE106_GOAL_TIMEOUT_SEC=190.0 \
PHASE106_VALIDATION_WINDOW_SEC=260 \
PHASE106_RUNTIME_RECORD_TIMEOUT_SEC=320 \
PHASE106_CLEANUP_ON_EXIT=0 \
PYTHONDONTWRITEBYTECODE=1 \
tools/run_phase106_preflighted_carry_over_bounded_goal1_staging_validation.sh
```

Runtime wrapper exit code: `0`

Runtime stdout:

```json
{"classification":"PHASE105_PREFLIGHT_REJECTED_BEFORE_INGRESS","evidence_gaps":["phase105_preflight_rejected_before_ingress", "ingress_preflight_rejected_explorer_not_started", "missing_goal_events", "missing_goal1_events", "missing_goal1_dispatch"]}
```

## Runtime classification

Final classification: `PHASE105_PREFLIGHT_REJECTED_BEFORE_INGRESS`

Interpretation: Phase105 ingress preflight rejected before ingress. The wrapper preserved the fail-closed contract: it did not send the inner-ingress goal and did not start `maze_explorer`. Therefore Goal1 was not dispatched and carry-over/staging could not be evaluated in this runtime.

This is a validation result, not an autonomous exploration success and not an exit success.

## Cleanup evidence

Initial cleanup artifact:
`log/phase106_preflighted_carry_over_bounded_goal1_staging_validation/phase106_preflighted_carry_over_bounded_goal1_staging_validation_initial_cleanup_summary.json`

- scope: `project-scoped Gazebo/RViz/SLAM/Nav2/maze_explorer/recorder/ros2 launch processes only`
- before_count: `13`
- after_count: `0`
- unrelated_processes_targeted: `false`

Post-run cleanup artifact:
`log/phase106_preflighted_carry_over_bounded_goal1_staging_validation/phase106_preflighted_carry_over_bounded_goal1_staging_validation_post_cleanup_summary.json`

- before_count: `25`
- after_count: `0`
- unrelated_processes_targeted: `false`

A final `ps` verification after post-run cleanup found no matching Phase106/Gazebo/RViz/SLAM/Nav2/maze_explorer/recorder process residues.

## Ingress preflight evidence

Ingress preflight artifact:
`log/phase106_preflighted_carry_over_bounded_goal1_staging_validation/phase106_preflighted_carry_over_bounded_goal1_staging_validation_ingress_preflight.json`

Ingress result artifact:
`log/phase106_preflighted_carry_over_bounded_goal1_staging_validation/phase106_preflighted_carry_over_bounded_goal1_staging_validation_ingress_result.json`

- evaluated: `true`
- passed: `false`
- ingress_goal_sent: `false`
- maze_explorer_started: `false`
- reject token: `ingress_preflight_timeout`
- last_specific_reject_reason: `ingress_map_base_tf_missing`
- ingress_preflight_timeout_sec: `20.0`
- bounded_wait_elapsed_sec: `28.905702845018823`
- inner-ingress goal if allowed by preflight: frame_id=`map`, x=`2.0`, y=`0.0`, yaw=`0.0`

Failed gates:

- `ingress_map_base_tf_missing`
- `ingress_map_odom_tf_stale`
- `ingress_odom_base_tf_stale`
- `ingress_scan_transform_unstable`
- `ingress_controller_robot_pose_unavailable`
- `ingress_goal_pose_transform_unavailable`
- `ingress_preflight_timeout`

Controller/TF/scan details:

- navigate_to_pose_action_ready: `true`
- bt_navigator_active: `false`
- controller_server_active: `false`
- robot_pose_available: `false`
- map_base_tf_available: `false`
- map_base_tf_stable: `false`
- map_odom_tf_age_sec: `None`
- odom_base_tf_age_sec: `1780476298.763681`
- scan_available: `false`
- scan_transform_available: `false`
- scan_transform_stable: `false`
- goal transform to controller frame available: `false`
- goal transform to global costmap available: `true`
- tf_detector.goal_pose_transform_failure_count: `2`
- tf_detector.robot_pose_unavailable_count: `2`
- tf_detector.tf_jump_count: `0`
- tf_detector.cache_drop_count: `0`

## Goal1 carry-over/staging field summary

Because preflight rejected before ingress, no `/maze/goal_events` were published for Goal1 and the Goal1 carry-over/staging fields are non-triggered evidence, not a negative algorithm verdict.

- goal_event_count: `0`
- Goal1 dispatch observed: `false`
- corridor_evidence_carry_over: `{}`
- carry_over_source: `null`
- carry_over_applied: `false`
- carry_over_reject_reason: `None`
- source_forward_window: `{}`
- staging_window: `{}`
- safety_evidence_recomputed: `false`
- two_step_staging_plan: `{}`
- staging_executability_check: `{}`
- staging_applied: `false`
- staging_reject_reason: `None`
- branch_scoring_changed: `false`
- fallback_terminal_acceptance_used: `false`
- nav2_feedback.sample_count: `0`
- local_cost.sample_count: `0`

## Direct validation answers

- Phase105 preflight passed? `false`
- Was the ingress goal sent only after preflight pass? `true`
- Did ingress succeed? `false`
- Did Goal1 dispatch? `false`
- Was original `missing_two_side_wall_evidence` fixed by carry-over? `false`
- Does `carry_over_applied=true` only mean corridor-level evidence reuse? `false`
- Is `safety_evidence_recomputed` true? `false`
- If staging still rejects, is the new reject reason `staging_safety_recompute_failed` or another token? `not reached; staging_reject_reason_after_carry_over=None`

## Raw capture evidence

Raw capture artifact:
`log/phase106_preflighted_carry_over_bounded_goal1_staging_validation/phase106_preflighted_carry_over_bounded_goal1_staging_validation_raw_capture.json`

- raw_capture_summary.available: `true`
- local_costmap_available: `true`
- map_available: `true`
- odom_available: `true`
- scan_available: `true`
- tf_available: `true`

Raw capture includes `/local_costmap/costmap`, `/scan`, `/map`, `/odom`, `/local_costmap/published_footprint`, and TF samples. These are retained only as preflight-reject context because `maze_explorer` did not run.

## Evidence gaps

- `phase105_preflight_rejected_before_ingress`
- `ingress_preflight_rejected_explorer_not_started`
- `missing_goal_events`
- `missing_goal1_events`
- `missing_goal1_dispatch`

These gaps are expected under the fail-closed Phase105 reject branch.

## Guardrails

- No maze_explorer strategy changed.
- No Phase88/92/101/105 logic changed.
- No branch scoring changed.
- No exploration order changed.
- No centerline gate changed.
- No directional readiness/fallback/terminal acceptance changed.
- No Nav2/MPPI/controller tuning.
- No inflation/robot_radius/clearance_radius_m/map threshold tuning.
- Timeout/failure is not success.
- No autonomous exploration success claimed.
- No exit success claimed.
- Phase107 not entered.

Analyzer guardrail fields:

```json
{
  "algorithm_changed": false,
  "branch_scoring_changed": false,
  "fallback_terminal_acceptance_used": false,
  "nav2_config_changed": false,
  "no_exit_success_claimed": true,
  "no_success_claimed": true,
  "phase107_entered": false,
  "phase88_92_101_105_logic_changed": false
}
```

## Verification

Focused tests:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase106_preflighted_carry_over_bounded_goal1_staging_validation.py
```

Observed result:

```text
......                                                                   [100%]
6 passed in 0.05s
```

Build command used for this phase:

```bash
source /opt/ros/jazzy/setup.bash && colcon build --packages-select tugbot_maze --symlink-install
```

Observed result:

```text
Starting >>> tugbot_maze
Finished <<< tugbot_maze [0.87s]

Summary: 1 package finished [0.99s]
```

Post-cleanup process verification command:

```bash
ps -eo pid,cmd | grep -E 'phase106_preflighted|record_explorer_state_series|record_phase106|maze_explorer|ros2 launch tugbot_bringup tugbot_maze_slam_nav|gz sim|rviz2|slam_toolbox|controller_server|planner_server|bt_navigator|ros_gz_bridge|parameter_bridge' | grep -v grep || true
```

Observed result: empty output, meaning no matching project-scoped runtime residues were found.

Nav2 config guard command:

```bash
git diff -- src/tugbot_navigation/config
```

Observed result: empty output.

## Conclusion

Phase106 completed at the Phase105 fail-closed branch:

- Phase105 preflight did not pass.
- The unchanged inner-ingress goal `(map, x=2.0, y=0.0, yaw=0.0)` was not sent because preflight failed.
- `maze_explorer` was not started.
- Goal1 was not dispatched.
- Carry-over/staging did not trigger; therefore the original `missing_two_side_wall_evidence` question remains unevaluated in this run.
- No algorithm or Nav2/controller configuration changes were made.
- No autonomous exploration success or exit success is claimed.
- Work stops in Phase106; Phase107 is not entered.
