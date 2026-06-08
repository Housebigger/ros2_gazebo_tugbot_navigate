# Phase89 safety-first refinement bounded Goal2 validation runbook

Status: READY_FOR_BOUNDED_REPRODUCTION

Purpose: validate Phase88 safety-first multi-candidate forward search on the Goal2-equivalent segment using bounded visible reproduction. This runbook is validation-only and does not change navigation strategy.

## Command

From workspace root:

```bash
PHASE89_HOLD_AFTER_TRIGGER_SEC=600 PHASE89_CLEANUP_ON_EXIT=0 PYTHONDONTWRITEBYTECODE=1 \
  tools/run_phase89_safety_first_refinement_bounded_goal2_validation.sh
```

Bounded defaults:

- visible Gazebo/RViz: `headless:=false`, `use_rviz:=true`
- max goals: `PHASE89_MAX_GOALS=2` and must remain 2
- ingress goal first, then bounded `maze_explorer`
- `goal_timeout_sec=45.0`
- no long autonomous exploration
- scene held after trigger for screenshots

## Artifacts

The wrapper writes under:

`log/phase89_safety_first_refinement_bounded_goal2_validation/`

Expected evidence:

- `phase89_safety_first_refinement_bounded_goal2_validation_goal_events.jsonl`
- `phase89_safety_first_refinement_bounded_goal2_validation_nav2_feedback.jsonl`
- `phase89_safety_first_refinement_bounded_goal2_validation_local_costmap_samples.jsonl`
- `phase89_safety_first_refinement_bounded_goal2_validation_runtime_timeline.jsonl`
- `phase89_safety_first_refinement_bounded_goal2_validation_raw_capture.json`
- `phase89_safety_first_refinement_bounded_goal2_validation_analysis.json`
- `phase89_safety_first_refinement_bounded_goal2_validation_minimal_field_summary.md`
- `phase89_safety_first_refinement_bounded_goal2_validation_SCENE_HELD_WAITING_FOR_USER_SCREENSHOT.txt`

## Goal2-equivalent dispatch context fields

The analyzer checks:

- `multi_candidate_forward_search`
- `candidate_family`
- `candidate_count`
- `hard_safety_pass_candidate_count`
- `selected_candidate_index`
- `selected_candidate_target`
- `selected_candidate_yaw`
- `selection_priority_trace`
- `rejected_candidate_summaries`
- `refinement_applied`
- `refinement_reject_reason`
- `original_target_preserved_on_reject`
- `branch_scoring_changed=false`
- `fallback_terminal_acceptance_used=false`

## Classification set

- `PHASE88_REFINEMENT_APPLIED_GOAL2_IMPROVED`
- `PHASE88_REFINEMENT_APPLIED_BUT_GOAL2_TIMEOUT`
- `PHASE88_REFINEMENT_STILL_REJECTED`
- `PHASE88_VALIDATION_INSUFFICIENT_EVIDENCE`

## Screenshot suggestions if scene is held

Use 1-4 screenshots only:

1. Gazebo wide view: robot near Goal2-equivalent corridor, nearest wall, and target direction.
2. RViz local costmap: robot footprint and front wedge around selected candidate approach.
3. RViz goal/tolerance view: original target vs selected candidate/dispatch target if visible.
4. Recovery/timeout pose: local-cost blockage or recovery loop if still present.

## Guardrails

No maze_explorer strategy changed.
No branch scoring changed.
No exploration order changed.
No centerline gate changed.
No directional readiness changed.
No fallback/terminal acceptance changed.
No Nav2/MPPI/controller tuning.
No inflation/robot_radius/clearance_radius_m/map threshold tuning.
No timeout-as-success interpretation.
No autonomous exploration success claimed.
No exit success claimed.
Phase90 not entered.
