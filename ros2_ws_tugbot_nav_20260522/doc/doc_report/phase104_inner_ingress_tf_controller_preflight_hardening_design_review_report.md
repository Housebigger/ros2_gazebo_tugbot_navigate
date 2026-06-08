# Phase104 inner-ingress TF/controller preflight hardening design review report

Status: FINAL_DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_PHASE105_IMPLEMENTATION

## Scope

Phase104 is design review only. It designs future fail-closed TF/controller preflight hardening before sending the explicit inner-ingress Nav2 goal.

No runtime simulation was launched. No Phase105 implementation was started.

## Required reading completed

- `doc/doc_report/phase103_inner_ingress_goal_failure_diagnosis_report.md`
- `doc/doc_report/phase102_carry_over_bounded_goal1_staging_validation_report.md`
- `doc/doc_report/phase101_staging_corridor_evidence_carry_over_minimal_implementation_report.md`
- `doc/doc_report/phase96_fix_ingress_guided_startup_correction_report.md`
- `doc/doc_report/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_report.md`
- `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`
- top-level design docs under `doc/doc_proposal`: Phase100, Phase91, Phase87, Phase83, Phase76, Phase27-alt-R5, and Phase27-alt near-exit fallback.

## Background evidence

Phase103 classified Phase102 as:

```text
INNER_INGRESS_CONTROLLER_EXECUTION_FAILED
```

Phase102 inner-ingress facts:

```text
goal_sent=True
goal_accepted=True
result_received=True
status=6
success=False
error_code=102
recoveries_max=0
navigation_time_last_sec≈0.375
last_distance_remaining≈1.9926457405090332
```

Phase103 log-derived markers:

```text
controller_goal_received=True
controller_unable_to_transform_goal_pose=True
follow_path_abort=True
bt_robot_pose_unavailable=True
tf_jump_back_count=793
scan_transform_cache_drop=True
scan_frame_transform_error=True
```

Phase96-fix and Phase97 both succeeded with the same explicit inner-ingress goal:

```text
frame_id=map
x=2.0
y=0.0
yaw=0.0
```

Therefore Phase104 does not design a new target. It designs a preflight gate to avoid sending that unchanged target while TF/controller/pose prerequisites are unstable.

## Added files

- Design proposal: `doc/doc_proposal/phase104_inner_ingress_tf_controller_preflight_hardening_design_review.md`
- Report: `doc/doc_report/phase104_inner_ingress_tf_controller_preflight_hardening_design_review_report.md`
- Focused static tests: `src/tugbot_maze/test/test_phase104_inner_ingress_tf_controller_preflight_hardening_design_review.py`

## TDD evidence

Focused static tests were written before the Phase104 proposal/report existed.

Observed RED:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase104_inner_ingress_tf_controller_preflight_hardening_design_review.py
FFFFFF [100%]
6 failed
```

Expected RED cause: missing Phase104 design document and report.

Final focused static tests:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase104_inner_ingress_tf_controller_preflight_hardening_design_review.py
...... [100%]
6 passed in 0.01s
```

## Design summary

The recommended future Phase105 direction is a fail-closed preflight in the visible/bounded ingress wrapper before the explicit inner-ingress `NavigateToPose` goal is sent:

```text
launch visible stack
  -> wait for Nav2 lifecycle/action readiness
  -> collect bounded TF/controller/scan stability samples
  -> require all hard preflight gates pass in one continuous window
  -> if pass: send explicit inner-ingress NavigateToPose goal unchanged
  -> if reject/timeout: ingress_goal_sent=false; record ingress_preflight_reject_reason
  -> do not start maze_explorer after rejected preflight
```

Key principle:

```text
Do not send the ingress goal until every hard gate passes in the same bounded preflight window.
```

## Fail-closed gates

Phase104 design requires these future hard gates:

1. `map->base_link TF continuous stability window`
   - continuous stable TF window for N seconds;
   - finite map-frame robot pose;
   - no timestamp regression or TF jump marker.

2. `map->odom timestamp age gate`
   - `map -> odom` available and fresh.

3. `odom->base_link timestamp age gate`
   - `odom -> base_link` available and fresh.

4. `scan frame transform stability gate`
   - recent scan exists;
   - scan frame can transform to map / global-costmap frame;
   - no cache drop / extrapolation pattern in the preflight window.

5. `controller robot pose availability gate`
   - controller_server active;
   - bt_navigator active;
   - NavigateToPose action ready;
   - robot pose available to controller/global costmap context;
   - no fresh `Robot pose is not available` marker.

6. `inner-ingress goal pose transform gate`
   - unchanged goal pose can transform from `map` into global costmap and controller frames.

7. `TF jump / cache-drop / robot-pose-unavailable detector gate`
   - detects Phase103-like jump/cache/drop/pose/goal-transform errors before sending the action.

8. `bounded wait then fail-closed`
   - brief bounded wait is allowed for transient instability;
   - timeout rejects and does not send the ingress goal.

## Reject tokens

The complete future reject-token inventory is:

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

Reject contract:

```text
ingress_preflight.passed=false
ingress_goal_sent=false
ingress_preflight_reject_reason=<one token from inventory>
maze_explorer_started=false
```

Preflight rejection is not maze_explorer failure, not carry-over failure, not staging failure, not branch failure, and not autonomous success.

## Future diagnostics/artifacts contract

Phase104 design requires a future artifact such as:

```text
log/<phase>/ingress_preflight.json
```

Required fields include:

```text
ingress_preflight
tf_stability_window_sec
tf_jump_count
scan_transform_check
controller_pose_check
goal_pose_transform_check
ingress_goal_sent=false when rejected
ingress_preflight_reject_reason
map_base_tf_age_sec
map_odom_tf_age_sec
odom_base_tf_age_sec
bounded_wait_elapsed_sec
```

The artifact records the unchanged explicit inner-ingress goal for audit:

```text
frame_id=map
x_m=2.0
y_m=0.0
yaw_rad=0.0
```

## Relationship to Phase101 carry-over

Phase104 does not validate Phase101 carry-over. Phase102 never reached Goal1 dispatch, so carry-over/staging fields were absent. The proposed preflight only restores the prerequisite reliability for the known ingress-guided startup path. Any future carry-over validation still requires a separate explicit bounded phase.

## Guardrails

- Do not change the inner-ingress target.
- No runtime code changed.
- No Phase88/92/101 logic changed.
- No maze_explorer strategy changed.
- No branch scoring changed.
- No exploration order changed.
- No centerline gate changed.
- No directional readiness changed.
- No fallback/terminal acceptance changed.
- No Nav2/MPPI/controller tuning.
- No inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No autonomous exploration success claimed.
- No exit success claimed.
- Phase105 not entered.

## Verification

Focused static tests:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase104_inner_ingress_tf_controller_preflight_hardening_design_review.py
...... [100%]
6 passed in 0.01s
```

Static syntax check:

```text
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile src/tugbot_maze/test/test_phase104_inner_ingress_tf_controller_preflight_hardening_design_review.py
exit 0
```

Nav2 config diff guard:

```text
nav2_config_diff=0
```

maze runtime logic diff guard:

```text
maze_logic_diff=0
```

Pycache cleanup:

```text
pycache_count=0
```

Validation artifact:

```text
log/phase104_inner_ingress_tf_controller_preflight_hardening_validation_summary.txt
```

## Decision

`DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_PHASE105_IMPLEMENTATION`

Phase104 recommends a future fail-closed TF/controller preflight before the explicit inner-ingress Nav2 goal. The preflight must require continuous stable TF, fresh map/odom/base transforms, stable scan transform, controller robot pose availability, and goal-pose transformability before sending the unchanged ingress goal. If those gates do not pass within a bounded wait, the future wrapper should reject with a specific token and `ingress_goal_sent=false`.

Stop here. Phase105 not entered.
