# Phase87 safety-first multi-candidate forward search design review report

Status: DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_IMPLEMENTATION_PHASE

## Scope

Phase87 is design review only. It documents a future safety-first multi-candidate forward search concept for corridor-aligned intermediate-goal refinement. It does not implement runtime behavior and does not modify navigation strategy.

Design review complete. Phase88 not entered.

## Required inputs read

The following required sources were read before writing the design:

- `doc/doc_report/phase86_lethal_cost_regression_reject_root_cause_report.md`
- `doc/doc_report/phase85_goal2_corridor_aligned_refinement_bounded_validation_report.md`
- `doc/doc_report/phase84_corridor_aligned_intermediate_goal_refinement_minimal_implementation_report.md`
- `doc/doc_proposal/phase83_corridor_aligned_intermediate_goal_design_review.md`
- top-level `doc/doc_proposal/*.md` design/workflow files, including:
  - `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`
  - `doc/doc_proposal/phase76_obstacle_triggered_visual_root_cause_workflow.md`
  - `doc/doc_proposal/phase27_alt_r5_footprint_terminal_acceptance_design.md`
  - `doc/doc_proposal/phase27_alt_near_exit_fallback_design.md`

## Phase86 basis

Phase86 diagnosed the Phase85 `lethal_cost_regression` reject as:

`CENTERLINE_PROJECTION_DISTANCE_OR_POSE_TOO_AGGRESSIVE`

with secondary support for:

- `MULTI_CANDIDATE_FORWARD_SEARCH_NEEDED`
- `FORWARD_EXECUTABILITY_CHECK_TOO_CONSERVATIVE`

Key evidence carried into Phase87:

- `candidate_count: 21`
- `two_side_wall_candidate_count: 21`
- `strict_eligible_candidate_count: 0`
- `eligible_candidate_count: 0`
- `hard_safety_pass_candidate_count: 1`
- `high_or_lethal_candidate_count: 12`
- `lethal_regression_candidate_count: 12`
- best-balance candidates all failed clearance or safety floor

Interpretation: exact/best-balance centerline candidates were too aggressive. Future design should search a bounded local family and rank safety before centering.

## New Phase87 files

Design proposal:

- `doc/doc_proposal/phase87_safety_first_multi_candidate_forward_search_design_review.md`

Focused static tests:

- `src/tugbot_maze/test/test_phase87_safety_first_multi_candidate_forward_search_design_review.py`

This report:

- `doc/doc_report/phase87_safety_first_multi_candidate_forward_search_design_review_report.md`

## Design summary

The proposed future Phase88 helper remains scoped to the already-selected explore branch candidate and runs only before Nav2 dispatch.

Core future flow:

```text
already-selected candidate target
  -> same-corridor evidence
  -> two-side-wall evidence
  -> safety-first multi-candidate forward search
  -> candidate safety/quality ranking
  -> selected refined Nav2 goal OR reject keeps original target unchanged
```

Candidate family required by the design:

- centerline projection
- small lateral offsets
- multiple forward offsets
- corridor heading variants

The family is bounded and local. It is not route replanning and not branch reselection.

## Safety-first priority order

The design records this priority order for any future implementation:

1. hard safety pass
2. no footprint/front-wedge lethal regression
3. safety_floor_ok
4. forward_progress_ok
5. clearance better
6. balance error smaller

Important interpretation:

- A best-centered target must lose to a less-centered but safer target.
- Balance error is a late tie-breaker, not the primary selection objective.
- No candidate can be accepted by lowering a safety boundary.

## Reject/apply contract

Future reject path:

- `refinement_applied=false`
- `selected_candidate_index=null`
- `selected_candidate_target=null`
- Nav2 receives the original target
- `original_target_preserved_on_reject=true`
- `refinement_reject_reason=<specific token>`
- reject keeps original target unchanged

Future apply path:

- selected candidate must have `hard_safety_pass=true`
- selected candidate must not increase footprint/front-wedge lethal counts
- selected candidate must pass safety floor and forward-progress checks
- selected candidate may be less centered than a best-balance candidate if it is safer
- Nav2 receives selected candidate target and yaw
- `branch_scoring_changed=false`
- `fallback_terminal_acceptance_used=false`

## Future goal_events contract

Future dispatch diagnostics should include:

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

## Future test contract

The design proposal specifies future Phase88 tests:

- `test_candidate_family_generation_contract`
- `test_safety_first_priority_selects_hard_safe_candidate_before_best_balance`
- `test_reject_preserves_original_target_when_no_candidate_passes`
- `test_guardrails_no_strategy_or_parameter_tuning`

Additional tests should cover missing same-corridor evidence, missing two-side-wall evidence, ambiguous baseline risk, exact centerline lethal candidates losing to safer offset candidates, heading variants that worsen front wedge, selection rank trace, and rejected candidate summaries.

## Guardrails

No safety boundary is lowered.

No inflation/robot_radius/clearance_radius_m/MPPI/controller tuning.

No branch scoring changed.

No exploration order changed.

No centerline gate changed.

No directional readiness override changed.

No fallback/terminal acceptance changed.

No autonomous exploration success claimed.

No exit success claimed.

Phase88 not entered.

No runtime strategy changed.

## Validation

Validation commands are run after this report is written:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase87_safety_first_multi_candidate_forward_search_design_review.py
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile src/tugbot_maze/test/test_phase87_safety_first_multi_candidate_forward_search_design_review.py
git diff -- src/tugbot_navigation/config src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_maze/tugbot_maze/maze_perception.py
```

Final command outputs are recorded in the final assistant response for Phase87.

## Stop condition

Phase87 stops after design document, report, focused static tests, py_compile, guard diff, and pycache cleanup. Phase88 not entered.
