# Phase105 inner-ingress TF/controller preflight hardening minimal implementation report

Status: PHASE105_IMPLEMENTED_STATIC_UNIT_VALIDATED_STOP_BEFORE_PHASE106

## Scope

Phase105 minimally implements a fail-closed TF/controller preflight before the explicit inner-ingress Nav2 goal is sent by the Phase102/afterward validation wrapper/tooling.

This is wrapper/tooling hardening only. It does not validate Phase101 carry-over. It only prevents the Phase102/103 class of accepted-then-immediate-controller-TF abort by rejecting before the action is sent when the startup TF/controller prerequisites are unstable.

## Required reading completed

- `doc/doc_proposal/phase104_inner_ingress_tf_controller_preflight_hardening_design_review.md`
- `doc/doc_report/phase104_inner_ingress_tf_controller_preflight_hardening_design_review_report.md`
- `doc/doc_report/phase103_inner_ingress_goal_failure_diagnosis_report.md`
- `doc/doc_report/phase102_carry_over_bounded_goal1_staging_validation_report.md`
- `doc/doc_report/phase96_fix_ingress_guided_startup_correction_report.md`
- `doc/doc_report/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_report.md`

## Background

Phase103 classified Phase102 as:

```text
INNER_INGRESS_CONTROLLER_EXECUTION_FAILED
```

Phase102 ingress evidence:

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

Phase96-fix and Phase97 succeeded with the same explicit inner-ingress target:

```text
map, x=2.0, y=0.0, yaw=0.0
```

Therefore Phase105 preserves the target and adds only a pre-send preflight.

## Files added or changed

Added:

- `tools/phase105_inner_ingress_tf_controller_preflight.py`
- `src/tugbot_maze/test/test_phase105_inner_ingress_tf_controller_preflight_hardening.py`
- `doc/doc_report/phase105_inner_ingress_tf_controller_preflight_hardening_minimal_implementation_report.md`

Changed wrapper/tooling only:

- `tools/run_phase102_carry_over_bounded_goal1_staging_validation.sh`
  - adds `INGRESS_PREFLIGHT_JSON`
  - runs Phase105 preflight before `send_ingress_goal`
  - writes fail-closed ingress result and trigger on reject
  - does not call `send_ingress_goal` or `start_explorer` on reject

- `tools/analyze_phase102_carry_over_bounded_goal1_staging_validation.py`
  - reads `*_ingress_preflight.json`
  - surfaces `ingress_preflight` in analysis output
  - classifies preflight reject as `GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE`, because no Goal1 dispatch occurs

## Implemented fail-closed preflight gates

The Phase105 preflight tool evaluates these gates before allowing the wrapper to send the explicit inner-ingress goal:

1. `map->base_link TF continuous stability window`
   - requires repeated passing samples across `tf_stability_window_sec`
   - rejects missing `map->base_link` as `ingress_map_base_tf_missing`
   - rejects jump/unstable/stale map-base evidence as `ingress_tf_unstable`

2. `map->odom timestamp age`
   - rejects stale/missing age as `ingress_map_odom_tf_stale`

3. `odom->base_link timestamp age`
   - rejects stale/missing age as `ingress_odom_base_tf_stale`

4. `scan frame transform stability`
   - requires scan availability and transformability to the map/global frame
   - rejects cache drops/exceptions/unavailable transform as `ingress_scan_transform_unstable`

5. `controller_server / bt_navigator / NavigateToPose ready plus robot pose available`
   - rejects missing lifecycle/action/pose evidence as `ingress_controller_robot_pose_unavailable`

6. `inner-ingress goal pose transformability`
   - verifies the unchanged goal in `map` can be transformed into controller/global costmap context
   - rejects as `ingress_goal_pose_transform_unavailable`

7. `TF jump / cache drop / robot pose unavailable detector`
   - records jump/cache-drop/robot-pose/goal-transform failure counts
   - contributes to the relevant specific reject tokens

8. `bounded wait then fail-closed`
   - if no continuous stable window is achieved before the deadline, rejects as `ingress_preflight_timeout`

## Reject token inventory

Implemented reject tokens:

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
maze_explorer_started=false
ingress_preflight_reject_reason=<specific token>
```

A preflight reject is not a maze_explorer failure, not a carry-over failure, not a staging failure, not a branch-selection failure, not autonomous exploration success, and not exit success.

## Artifact contract

The wrapper writes:

```text
log/phase102_carry_over_bounded_goal1_staging_validation/phase102_carry_over_bounded_goal1_staging_validation_ingress_preflight.json
```

The artifact contains:

```text
ingress_preflight
tf_stability_window_sec
tf_jump_count
scan_transform_check
controller_pose_check
goal_pose_transform_check
map_base_tf_age_sec
map_odom_tf_age_sec
odom_base_tf_age_sec
bounded_wait_elapsed_sec
ingress_goal_sent=false when rejected
maze_explorer_started=false when rejected
ingress_preflight_reject_reason
```

The artifact also records the unchanged explicit inner-ingress goal:

```text
frame_id=map
x_m=2.0
y_m=0.0
yaw_rad=0.0
```

## Wrapper behavior

New wrapper flow:

```text
launch visible stack
wait_for_readiness
start recorders
run_ingress_preflight
  if preflight passes:
      mark ingress_goal_sent=true, maze_explorer_started=false
      send explicit inner-ingress NavigateToPose goal
      if ingress succeeds:
          start maze_explorer
          mark maze_explorer_started=true
      else:
          trigger ingress_failed_explorer_not_started
  else:
      mark ingress_goal_sent=false, maze_explorer_started=false
      write ingress_result status=preflight_rejected
      trigger ingress_preflight_rejected_explorer_not_started
run analyzer
```

## TDD evidence

Focused tests were written before the preflight tool/report/wrapper integration existed.

Observed RED:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase105_inner_ingress_tf_controller_preflight_hardening.py
FFFFFF [100%]
6 failed
```

Intermediate result after adding tool/analyzer integration but before report/wrapper-order cleanup:

```text
4 passed, 2 failed
```

Final validation is recorded below.

## Validation

Focused and regression tests:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q \
  src/tugbot_maze/test/test_phase102_carry_over_bounded_goal1_staging_validation.py \
  src/tugbot_maze/test/test_phase105_inner_ingress_tf_controller_preflight_hardening.py
........... [100%]
11 passed in 0.06s
```

Phase105 focused subset:

```text
6 passed
```

Phase102 regression subset:

```text
5 passed
```

Static syntax checks:

```text
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile \
  tools/phase105_inner_ingress_tf_controller_preflight.py \
  tools/analyze_phase102_carry_over_bounded_goal1_staging_validation.py \
  src/tugbot_maze/test/test_phase105_inner_ingress_tf_controller_preflight_hardening.py \
  src/tugbot_maze/test/test_phase102_carry_over_bounded_goal1_staging_validation.py
py_compile=passed
```

Wrapper syntax check:

```text
bash -n tools/run_phase102_carry_over_bounded_goal1_staging_validation.sh
bash_n=passed
```

Synthetic dry-run artifacts:

```text
synthetic_pass_artifact=log/phase105_inner_ingress_tf_controller_preflight_hardening/phase105_synthetic_pass_ingress_preflight.json
synthetic_reject_artifact=log/phase105_inner_ingress_tf_controller_preflight_hardening/phase105_synthetic_reject_ingress_preflight.json
```

Synthetic reject artifact confirmed:

```text
ingress_preflight.passed=false
ingress_preflight_reject_reason=ingress_map_base_tf_missing
ingress_goal_sent=false
maze_explorer_started=false
inner_ingress_goal_pose={frame_id: map, x_m: 2.0, y_m: 0.0, yaw_rad: 0.0}
```

Diff guards:

```text
nav2_config_diff=0
maze_logic_diff=0
```

Pycache cleanup:

```text
pycache_count=0
```

Validation artifact:

```text
log/phase105_inner_ingress_tf_controller_preflight_hardening_validation_summary.txt
```

## Guardrails

- No Phase101 carry-over change.
- No Phase88/92 logic change.
- No maze_explorer strategy change.
- No branch scoring change.
- No exploration order change.
- No centerline gate change.
- No directional readiness change.
- No fallback/terminal acceptance change.
- No Nav2/MPPI/controller tuning.
- No inflation tuning.
- No robot_radius tuning.
- No clearance_radius_m tuning.
- No map threshold tuning.
- No autonomous exploration success claim.
- No exit success claim.
- No long autonomous exploration was run.
- Phase106 not entered.

## Decision

`PHASE105_IMPLEMENTED_STATIC_UNIT_VALIDATED_STOP_BEFORE_PHASE106`

Phase105 minimally implements fail-closed preflight hardening before sending the explicit inner-ingress goal. The target remains unchanged. Any preflight rejection now records a specific reject token, keeps `ingress_goal_sent=false`, keeps `maze_explorer_started=false`, and leaves Phase101 carry-over validation for a later explicit bounded phase.
