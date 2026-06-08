# Phase102 carry-over bounded Goal1 staging validation runbook

Status: RUNBOOK_CREATED_FOR_BOUNDED_VISIBLE_VALIDATION

## Goal

Validate whether Phase101 corridor evidence carry-over fixes the original Goal1 `staging_reject_reason=missing_two_side_wall_evidence` path observed in Phase97/98/99.

This is validation only. Do not change algorithms or parameters.

## Required startup flow

1. Scoped cleanup of old project Gazebo/RViz/SLAM/Nav2/maze_explorer/recorder processes.
2. Start visible Gazebo/RViz/SLAM/Nav2 with:
   - `headless:=false`
   - `use_rviz:=true`
3. Wait for:
   - `/navigate_to_pose`
   - `/map`
   - `/scan`
   - `/local_costmap/costmap`
   - `map->base_link` TF
4. Send explicit inner-ingress Nav2 goal before `maze_explorer`.
5. Start `maze_explorer` bounded with `max_goals:=1`.
6. Stop at the first of:
   - Goal1 staging/carry-over context observed,
   - Goal1 terminal outcome,
   - bounded validation window expiration,
   - explicit failure.

## Command

```bash
PHASE102_MAX_GOALS=1 \
PHASE102_INGRESS_TIMEOUT_SEC=90 \
PHASE102_GOAL_TIMEOUT_SEC=190.0 \
PHASE102_VALIDATION_WINDOW_SEC=260 \
PHASE102_RUNTIME_RECORD_TIMEOUT_SEC=320 \
PHASE102_CLEANUP_ON_EXIT=0 \
PYTHONDONTWRITEBYTECODE=1 \
tools/run_phase102_carry_over_bounded_goal1_staging_validation.sh
```

If a visible stack is already deliberately running from the same phase after a failed first attempt, use only with caution:

```bash
PHASE102_REUSE_VISIBLE_STACK=1 PHASE102_SKIP_BUILD=1 ... tools/run_phase102_carry_over_bounded_goal1_staging_validation.sh
```

## Artifacts

Artifact directory:

```text
log/phase102_carry_over_bounded_goal1_staging_validation
```

Expected files include:

- `phase102_initial_cleanup_summary.json`
- `phase102_carry_over_bounded_goal1_staging_validation_goal_events.jsonl`
- `phase102_carry_over_bounded_goal1_staging_validation_nav2_feedback.jsonl`
- `phase102_carry_over_bounded_goal1_staging_validation_local_costmap_samples.jsonl`
- `phase102_carry_over_bounded_goal1_staging_validation_runtime_timeline.jsonl`
- `phase102_carry_over_bounded_goal1_staging_validation_raw_capture.json`
- `phase102_carry_over_bounded_goal1_staging_validation_analysis.json`
- `phase102_carry_over_bounded_goal1_staging_validation_minimal_field_summary.md`

## Classification set

The analyzer emits exactly one of:

- `CARRY_OVER_APPLIED_STAGING_APPLIED`
- `CARRY_OVER_APPLIED_STAGING_SAFETY_REJECTED`
- `CARRY_OVER_REJECTED`
- `CARRY_OVER_NOT_TRIGGERED`
- `GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE`

## Required Goal1 fields

Analyze and report:

- `corridor_evidence_carry_over`
- `carry_over_source`
- `carry_over_applied`
- `carry_over_reject_reason`
- `source_forward_window`
- `staging_window`
- `safety_evidence_recomputed`
- `staging_executability_check`
- `staging_applied`
- `staging_reject_reason`
- `two_step_staging_plan`
- `branch_scoring_changed=false`
- `fallback_terminal_acceptance_used=false`

## Direct validation questions

1. Was original `missing_two_side_wall_evidence` fixed by carry-over?
2. Does `carry_over_applied=true` only mean corridor-level evidence reuse?
3. Is `safety_evidence_recomputed=true`?
4. If staging still rejects, is the new reject reason `staging_safety_recompute_failed` or something else?

## Failure / stuck behavior

If the run fails or gets stuck, preserve Gazebo/RViz for screenshots and output only a minimal summary plus screenshot suggestions:

- Gazebo wide view of robot, wall/corridor, and target direction.
- RViz local costmap around robot footprint/front wedge.
- RViz goal marker / Nav2 plan near Goal1.
- RViz TF/map view if evidence is missing.

## Guardrails

- No maze_explorer strategy change.
- No Phase88/92/101 logic change.
- No branch scoring change.
- No exploration order change.
- No centerline gate change.
- No directional readiness/fallback/terminal acceptance change.
- No Nav2/MPPI/controller tuning.
- No inflation/robot_radius/clearance_radius_m/map threshold tuning.
- Do not treat timeout/failure as success.
- No autonomous exploration success claimed.
- No exit success claimed.
- Phase103 not entered.
