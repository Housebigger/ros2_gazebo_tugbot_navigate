# Phase103 inner-ingress goal failure diagnosis report

Status: COMPLETE_STOP_BEFORE_PHASE104

## Goal

Diagnose why Phase102 failed before Goal1 dispatch: explicit inner-ingress Nav2 goal was accepted but returned status=6 / success=false / error_code=102 with last distance_remaining about 1.99m.

This phase is diagnosis-only. No Phase101 carry-over changes. No Phase88/92 logic changes. No maze_explorer strategy changes. No Nav2 parameter tuning.

## Cleanup

Initial cleanup artifact:

```text
log/phase103_inner_ingress_goal_failure_diagnosis/phase103_initial_cleanup_summary.json
```

Project-scoped cleanup only; no unrelated processes targeted. Cleanup before_count=10 after_count=0. One stubborn project-scoped planner_server required SIGKILL after SIGTERM. RViz hung-window handling was included in the scoped scan; remaining matching process count after cleanup was 0.

Post-analysis process scan:

```text
log/phase103_inner_ingress_goal_failure_diagnosis/phase103_post_analysis_process_scan.json
```

Final process scan after validation:

```text
log/phase103_inner_ingress_goal_failure_diagnosis/phase103_final_process_scan.json
```

Final matching project process count: 0.

## Classification

`INNER_INGRESS_CONTROLLER_EXECUTION_FAILED`

Allowed classifications were:

- INNER_INGRESS_GOAL_POSE_BLOCKED
- INNER_INGRESS_START_POSE_OR_FRAME_MISMATCH
- INNER_INGRESS_NAV2_PLANNING_FAILED
- INNER_INGRESS_CONTROLLER_EXECUTION_FAILED
- INNER_INGRESS_REUSE_VISIBLE_STACK_STATE_DIRTY
- INNER_INGRESS_DIAGNOSIS_INSUFFICIENT_EVIDENCE

## References compared

- Phase102 failed ingress artifact directory: `log/phase102_carry_over_bounded_goal1_staging_validation`
- Phase96-fix successful ingress reference: `log/phase96_fix_ingress_guided_startup_correction`
- Phase97 successful ingress reference: `log/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke`

## Analyzer artifacts

- JSON: `log/phase103_inner_ingress_goal_failure_diagnosis/phase103_inner_ingress_goal_failure_diagnosis_analysis.json`
- Minimal summary: `log/phase103_inner_ingress_goal_failure_diagnosis/phase103_inner_ingress_goal_failure_diagnosis_minimal_summary.md`
- Analyzer: `tools/analyze_phase103_inner_ingress_goal_failure_diagnosis.py`
- Focused tests: `src/tugbot_maze/test/test_phase103_inner_ingress_goal_failure_diagnosis.py`

## Key evidence

Phase102 ingress:

- goal_sent=True
- goal_accepted=True
- result_received=True
- status=6
- success=False
- error_code=102
- feedback_count=15

Pose/readiness:

- ingress goal pose: `{'frame_id': 'map', 'x_m': 2.0, 'y_m': 0.0, 'yaw_rad': 0.0}`
- start pose: `{'available': True, 'robot_pose_map': [1.0685968489295284e-11, -1.0400709138586546e-24, 2.7411664263445953e-13]}`
- map frame OK: `{'expected_frame_id': 'map', 'frame_ok': True, 'goal_frame_id': 'map'}`
- Nav2 readiness: `{'action_server_available': True, 'bt_navigator_active': True, 'controller_server_active': True, 'navigate_to_pose_action_ready': True, 'planner_server_active': True}`
- Nav2 lifecycle ready: `True`

Feedback:

- distance_remaining_last=1.9926457405090332
- navigation_time_last_sec=0.375
- recoveries_max=0
- immediate_abort_like=True

Log-derived failure markers:

- controller_goal_received=True
- controller_unable_to_transform_goal_pose=True
- follow_path_abort=True
- bt_robot_pose_unavailable=True
- bt_goal_failed=True
- tf_jump_back_count=793
- scan_transform_cache_drop=True
- scan_frame_transform_error=True

The action error diagnosis interprets error_code=102 as `TF_ERROR-compatible error code surfaced through NavigateToPose result`, likely source `controller_or_bt_subaction_tf_error`.

## Comparison with successful ingress phases

Phase96-fix:

- ingress_success=True
- status=succeeded
- error_code=0
- goal_pose=`{'frame_id': 'map', 'x_m': 2.0, 'y_m': 0.0, 'yaw_rad': 0.0}`
- feedback_sample_count=455
- scan/map/local_costmap/odom/tf available=True/True/True/True/True

Phase97:

- ingress_success=True
- status=succeeded
- error_code=0
- goal_pose=`{'frame_id': 'map', 'x_m': 2.0, 'y_m': 0.0, 'yaw_rad': 0.0}`
- feedback_sample_count=459
- scan/map/local_costmap/odom/tf available=True/True/True/True/True

The goal pose matches Phase96-fix and Phase97 (`map`, x=2.0, y=0.0, yaw=0.0), so the failure is not explained by a different inner-ingress target. Phase102 readiness markers were present, but execution aborted almost immediately with TF/pose transform failures.

## Reuse-visible-stack note

Phase102 final rerun used reuse_visible_stack=1, but the archived first attempt had reuse_visible_stack=0 and also failed. Therefore dirty reuse is recorded as a contextual risk but not the primary classification:

`{'attempt1_failure_artifact_present': True, 'attempt1_ingress_failed': True, 'attempt1_preflight_reuse_line': 'reuse_visible_stack=0', 'attempt1_reuse_visible_stack': False, 'clean_attempt_also_failed': True, 'dirty_reuse_detected': False, 'dirty_reuse_present_but_not_primary': True, 'preflight_reuse_line': 'reuse_visible_stack=1', 'reuse_visible_stack': True}`

## Interpretation

The best supported Phase103 classification is controller execution failure: the action was accepted and controller_server received FollowPath, then logs show transform/pose errors, inability to transform the goal pose into the costmap frame, FollowPath abort, bt_navigator robot pose unavailable, and Goal failed. This is narrower than a pure planning failure despite missing global-plan recorder samples.

This does not validate Phase101 carry-over. Phase102 never reached Goal1 dispatch and produced no Goal1 carry-over/staging evidence.

## Verification

Final verification commands run:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase103_inner_ingress_goal_failure_diagnosis.py
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile tools/analyze_phase103_inner_ingress_goal_failure_diagnosis.py
```

Additional guard checks were run after report generation:

```text
git diff -- src/tugbot_navigation/config | wc -l
git diff -- src/tugbot_maze/tugbot_maze | wc -l
find . -name __pycache__ -type d
```

## Guardrails

- No Phase101 carry-over changes.
- No Phase88/92 logic changes.
- No maze_explorer strategy changes.
- No Nav2 parameter tuning.
- No inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No autonomous exploration success declared.
- No exit success declared.
- Phase104 not entered.
