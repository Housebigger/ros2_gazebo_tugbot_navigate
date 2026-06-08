# Phase102 carry-over bounded Goal1 staging validation report

Status: COMPLETED_INSUFFICIENT_EVIDENCE_INGRESS_FAILED_SCENE_HELD

## Goal

Reproduce the Phase97/98 Goal1 scene with ingress-guided startup and validate whether the Phase101 carry-over implementation fixes the original `staging_reject_reason=missing_two_side_wall_evidence` evidence-path failure.

This phase was validation only. No maze_explorer strategy, Phase88/92/101 logic, branch scoring, exploration order, centerline gate, directional readiness/fallback/terminal acceptance, or Nav2/controller/inflation/robot footprint/clearance/map thresholds were changed.

## Required reading completed

- `doc/doc_report/phase101_staging_corridor_evidence_carry_over_minimal_implementation_report.md`
- `doc/doc_report/phase100_staging_corridor_evidence_carry_over_design_review_report.md`
- `doc/doc_report/phase99_goal1_staging_evidence_path_diagnosis_report.md`
- `doc/doc_report/phase98_goal1_recovery_dominant_failure_root_cause_diagnosis_report.md`
- `doc/doc_report/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_report.md`
- `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`

## Initial cleanup

Cleanup artifact:

```text
log/phase102_carry_over_bounded_goal1_staging_validation/phase102_initial_cleanup_summary.json
```

Cleanup summary:

```text
before_count=10
after_count=0
unrelated_processes_targeted=false
scope=project-scoped Gazebo/RViz/SLAM/Nav2/maze_explorer/recorder/phase wrapper processes only
```

The first cleanup removed old Phase93/94-era project-scoped launch/Gazebo/RViz/SLAM/Nav2 processes from this workspace and did not target unrelated processes.

## Added Phase102 files

- `src/tugbot_maze/test/test_phase102_carry_over_bounded_goal1_staging_validation.py`
- `tools/analyze_phase102_carry_over_bounded_goal1_staging_validation.py`
- `tools/record_phase102_carry_over_validation_evidence.py`
- `tools/run_phase102_carry_over_bounded_goal1_staging_validation.sh`
- `doc/doc_report/phase102_carry_over_bounded_goal1_staging_validation_runbook.md`
- `doc/doc_report/phase102_carry_over_bounded_goal1_staging_validation_report.md`

## TDD evidence

Focused tests were written before Phase102 tooling/report existed.

Observed RED:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase102_carry_over_bounded_goal1_staging_validation.py
FFFFF [100%]
5 failed in 0.07s
```

Final focused tests:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase102_carry_over_bounded_goal1_staging_validation.py
5 passed
```

## Runtime command

Bounded visible validation command used:

```bash
PHASE102_MAX_GOALS=1 PHASE102_INGRESS_TIMEOUT_SEC=90 PHASE102_GOAL_TIMEOUT_SEC=190.0 PHASE102_VALIDATION_WINDOW_SEC=260 PHASE102_RUNTIME_RECORD_TIMEOUT_SEC=320 PHASE102_CLEANUP_ON_EXIT=0 PYTHONDONTWRITEBYTECODE=1 tools/run_phase102_carry_over_bounded_goal1_staging_validation.sh
```

A second bounded attempt reused the visible stack after the first attempt showed the ingress sender returned `status=6` immediately. That second attempt also failed before maze_explorer start. No long unbounded exploration was run.

## Runtime artifact directory

```text
log/phase102_carry_over_bounded_goal1_staging_validation
```

Key artifacts:

- `phase102_carry_over_bounded_goal1_staging_validation_analysis.json`
- `phase102_carry_over_bounded_goal1_staging_validation_minimal_field_summary.md`
- `phase102_carry_over_bounded_goal1_staging_validation_ingress_result.json`
- `phase102_carry_over_bounded_goal1_staging_validation_raw_capture.json`
- `phase102_failed_ingress_hold_scene_summary.json`
- `attempt1_ingress_failed_before_rerun/`

## Classification set

The Phase102 analyzer emits exactly one of:

- `CARRY_OVER_APPLIED_STAGING_APPLIED`
- `CARRY_OVER_APPLIED_STAGING_SAFETY_REJECTED`
- `CARRY_OVER_REJECTED`
- `CARRY_OVER_NOT_TRIGGERED`
- `GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE`

## Runtime result

Final classification:

```text
GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE
```

Evidence gaps:

```text
['ingress_failed_explorer_not_started', 'missing_goal_events', 'missing_goal1_events', 'missing_goal1_dispatch']
```

Trigger:

```json
{
  "trigger": "ingress_failed_explorer_not_started"
}
```

Ingress result summary:

```text
goal_sent=True
goal_accepted=True
result_received=True
status=6
status_text=6
success=False
error_code=102
feedback_count=15
last_distance_remaining=1.9926457405090332
```

Interpretation: the explicit inner-ingress Nav2 goal was accepted but returned `status=6` / `success=false` almost immediately. Therefore `maze_explorer` was not started and no `/maze/goal_events` Goal1 dispatch was produced. Phase102 cannot validate carry-over behavior from this run.

## Required Goal1 fields

Because Goal1 never dispatched, all Goal1 carry-over fields are absent/not observed:

```text
corridor_evidence_carry_over={}
carry_over_source=null
carry_over_applied=false
carry_over_reject_reason=null
source_forward_window={}
staging_window={}
safety_evidence_recomputed=false
staging_executability_check={}
staging_applied=false
staging_reject_reason=null
two_step_staging_plan={}
branch_scoring_changed=false
fallback_terminal_acceptance_used=false
```

## Direct validation answers

- Was original `missing_two_side_wall_evidence` fixed by carry-over?
  - Not validated. Goal1 did not dispatch; no staging/carry-over context was observed.
- Does `carry_over_applied=true` only mean corridor-level evidence reuse?
  - Not observed in runtime. Static Phase101 tests still enforce that contract, but Phase102 runtime did not reach this field.
- Is `safety_evidence_recomputed=true`?
  - Not observed. Goal1 did not dispatch.
- If staging still rejects, is the new reject reason `staging_safety_recompute_failed` or something else?
  - Not observed. No Goal1 staging reject occurred.

## Raw capture availability

Despite missing Goal1 events, raw scene evidence was captured:

```text
scan_available=True
map_available=True
local_costmap_available=True
odom_available=True
tf_available=True
```

## Scene hold / screenshot guidance

Failure/stuck behavior was followed: Gazebo/RViz/SLAM/Nav2 visible stack was preserved for screenshots.

Hold-scene artifact:

```text
log/phase102_carry_over_bounded_goal1_staging_validation/phase102_failed_ingress_hold_scene_summary.json
```

Visible preserved process count:

```text
9
```

Suggested screenshots:

- Gazebo wide view of robot at/near start and entrance corridor.
- RViz map + robot pose + TF view after failed inner-ingress action.
- RViz local costmap around robot footprint and front wedge.
- RViz Nav2 goal/plan display if any plan marker remains from the failed ingress action.

## Verification

- Focused Phase102 tests: `5 passed`.
- `py_compile`: passed for analyzer, recorder, and test.
- `bash -n`: passed for Phase102 wrapper.
- `nav2_config_diff=0`.
- `maze_logic_diff=0`.

## Guardrails

- No maze_explorer strategy changed.
- No Phase88/92/101 logic changed.
- No branch scoring changed.
- No exploration order changed.
- No centerline gate changed.
- No directional readiness/fallback/terminal acceptance changed.
- No Nav2/MPPI/controller tuning.
- No inflation/robot_radius/clearance_radius_m/map threshold tuning.
- Timeout/failure is not success.
- No autonomous exploration success claimed.
- No exit success claimed.
- Phase103 not entered.
