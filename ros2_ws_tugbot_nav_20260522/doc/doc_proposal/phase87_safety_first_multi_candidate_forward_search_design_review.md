# Phase87: Safety-first multi-candidate forward search design review

Status: `DESIGN_REVIEW_ONLY_NOT_RUNTIME_ENABLED`

Phase87 is a design-review-only phase for a future safety-first multi-candidate forward search refinement. It does not modify runtime code, navigation strategy, branch scoring, centerline gates, directional readiness, fallback, terminal acceptance, Nav2, MPPI, controller, inflation, robot radius, clearance radius, or map thresholds.

Phase88 implementation phase may implement this design only after explicit user approval. Do not implement in Phase87.

## Purpose

Phase86 diagnosed the Phase85 `lethal_cost_regression` reject as:

`CENTERLINE_PROJECTION_DISTANCE_OR_POSE_TOO_AGGRESSIVE`

with secondary support for:

- `MULTI_CANDIDATE_FORWARD_SEARCH_NEEDED`
- `FORWARD_EXECUTABILITY_CHECK_TOO_CONSERVATIVE`

The Phase86 key finding is that Phase84's best-balance centerline candidates were too aggressive for the Goal2 local-cost / footprint / front-wedge geometry. The future design must not simply chase the most centered point. It must search a small candidate family and select the safest candidate first.

This proposal designs that future Phase88 behavior. It is not a runtime patch.

## Upstream evidence basis

Required upstream context read before this design:

- Phase83: corridor-aligned intermediate goal design should refine only the already-selected candidate before Nav2 dispatch.
- Phase84: minimal implementation added `original_target`, `centerline_projected_target`, `corridor_heading_yaw`, `refinement_applied`, `refinement_reject_reason`, `forward_executability_check`, `branch_scoring_changed=false`, and `fallback_terminal_acceptance_used=false`.
- Phase85: refinement evaluated the Goal2-equivalent target but rejected with `lethal_cost_regression`; Nav2 received the original target.
- Phase86: all 21 candidates had same-corridor and two-side-wall evidence, but no candidate was eligible; 12/21 were high/lethal, 12/21 were lethal regressions, 1 candidate passed hard safety, and all four best-balance candidates failed clearance or safety floor.

Design implication:

- Do not lower safety boundaries.
- Do not tune Nav2 / MPPI / controller / inflation / robot_radius / clearance_radius_m / map thresholds.
- Do not choose the best centered candidate before proving safety.
- Search a bounded family of nearby candidate poses and rank by safety first, then execution quality, then centering.

## Non-goals and hard guardrails

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

The future algorithm must stay scoped to the already-selected explore candidate. It must not choose a different branch and must not mutate DFS topology state.

## Future core flow

The future Phase88 helper should run only after the existing exploration logic has already selected a branch candidate and before Nav2 dispatch:

```text
already-selected candidate target
  -> same-corridor evidence
  -> two-side-wall evidence
  -> safety-first multi-candidate forward search
  -> candidate safety/quality ranking
  -> selected refined Nav2 goal OR reject keeps original target unchanged
```

If any required evidence is missing, if every candidate fails hard safety, or if ranking cannot distinguish a safe candidate, reject keeps original target unchanged and records a specific reject reason.

## Candidate family design

The future helper should generate a small, bounded family around the Phase84 centerline target concept. This is not route replanning. The family is local to the already-selected target and corridor.

Required candidate family dimensions:

1. centerline projection
   - Start from the already-selected candidate point.
   - Project to the corridor centerline estimated from same-corridor and two-side-wall evidence.
   - Keep the projected point within a bounded local neighborhood of the original candidate.

2. small lateral offsets
   - Add small offsets around the centerline instead of forcing exact centerline.
   - Example future candidate set: `[-0.15, -0.10, -0.05, 0.0, +0.05, +0.10, +0.15] m`, subject to existing geometry scale and tests.
   - This addresses Phase86 evidence that the exact/best-balance centerline pose can be too aggressive.

3. multiple forward offsets
   - Add multiple forward offsets along the corridor heading.
   - Example future candidate set: `[0.0, +0.05, +0.10, +0.15, +0.20] m`, bounded by original candidate neighborhood and same-corridor evidence.
   - Forward offsets must never move behind dispatch pose, reduce progress, or cross walls.

4. corridor heading variants
   - Evaluate the primary corridor heading yaw.
   - Optionally evaluate small heading variants around the corridor axis, for example `[-5 deg, 0 deg, +5 deg]`, only if raw evidence shows the variants remain aligned with the same corridor.
   - Heading variants are for footprint/front-wedge executability, not for changing branch direction.

Candidate count must remain bounded and deterministic. Future Phase88 should record the exact offsets used and why.

## Safety-first screening priority

The future selection priority is ordered and must be implemented exactly in this conceptual order:

1. hard safety pass
   - Candidate must pass the whole hard-safety bundle before quality ranking.
   - Required evidence includes same-corridor evidence, two-side-wall evidence, free occupancy, target clearance, lethal-regression checks, safety-floor checks, and forward-progress checks.
   - This item is a bundle gate; the next priority layers define how to interpret the main hard-safety subitems without lowering any boundary.

2. no footprint/front-wedge lethal regression
   - Candidate must not increase target footprint lethal count or front-wedge lethal count relative to original target metrics.
   - If baseline is already risky, the future helper must classify the comparison as ambiguous and reject unless a clear strictly safer candidate is available.

3. safety_floor_ok
   - Candidate must satisfy the existing safety floor.
   - Do not lower `clearance_radius_m`, local-cost thresholds, robot radius, inflation radius, or map thresholds to make a candidate pass.

4. forward_progress_ok
   - Candidate must preserve or improve forward progress along the selected corridor branch.
   - Candidate must not create a sideways-only near-goal finish or a target behind the dispatch pose.

5. clearance better
   - Among hard-safe candidates, prefer higher minimum clearance and lower local-cost radius / footprint / front-wedge risk.
   - Clearance improvement is a tie-breaker after hard safety, not a reason to accept lethal regression.

6. balance error smaller
   - Only after hard safety and clearance are acceptable, prefer smaller corridor balance error.
   - A more centered target must lose to a less centered but safer target.

This ordering intentionally reverses the failure mode observed in Phase86: best-balance centerline candidates must not be selected before safety is proven.

## Candidate metrics contract

Each future candidate should record:

- `candidate_index`
- `target_xy`
- `target_yaw`
- `lateral_offset_m`
- `forward_offset_m`
- `heading_offset_rad`
- `same_corridor`
- `two_side_wall_evidence`
- `occupancy_free`
- `target_has_clearance`
- `safety_floor_ok`
- `forward_progress_ok`
- `forward_progress_m`
- `footprint_lethal_count`
- `front_wedge_lethal_count`
- `footprint_lethal_not_increased`
- `front_wedge_lethal_not_increased`
- `local_cost_max_radius`
- `local_cost_mean_radius`
- `min_clearance_m`
- `balance_error_m`
- `hard_safety_pass`
- `selection_rank_tuple`
- `candidate_reject_reasons`

The `selection_rank_tuple` should be diagnostics-only and deterministic. It should show why a candidate was selected or rejected.

## Future reject reasons

If the future helper rejects, it must preserve the original target unchanged and record a specific reason. Recommended future tokens:

- `missing_same_corridor_evidence`
- `missing_two_side_wall_evidence`
- `no_candidate_family_generated`
- `all_candidates_failed_hard_safety`
- `all_candidates_lethal_cost_regression`
- `baseline_already_risky_comparison_ambiguous`
- `no_forward_progress_candidate`
- `insufficient_local_cost_or_tf_evidence`
- `safety_first_search_no_apply`

Reject path contract:

- `refinement_applied=false`
- `selected_candidate_index=null`
- `selected_candidate_target=null`
- `centerline_projected_target=null` unless a diagnostic-only projection exists; do not send it to Nav2.
- `nav2_goal_target=original_target`
- `original_target_preserved_on_reject=true`
- `refinement_reject_reason=<specific token>`

## Future apply path

If the future helper applies a refined goal:

- selected candidate must have `hard_safety_pass=true`.
- selected candidate must not increase footprint/front-wedge lethal counts.
- selected candidate must pass safety floor and forward-progress checks.
- selected candidate may be less centered than the best-balance candidate if it is safer.
- Nav2 receives `selected_candidate_target` and `selected_candidate_yaw`.
- `branch_scoring_changed=false` remains recorded.
- `fallback_terminal_acceptance_used=false` remains recorded.

## Future goal_events fields

Future Phase88 dispatch events should include both top-level fields and a nested payload:

Top-level fields:

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

Nested `multi_candidate_forward_search` payload:

```text
{
  enabled: true,
  candidate_family: {
    lateral_offsets_m: [...],
    forward_offsets_m: [...],
    heading_offsets_rad: [...],
    bounded_local_search: true
  },
  original_target: [x, y],
  original_metrics: {...},
  candidates: [candidate_metrics...],
  selected_candidate_index: int|null,
  selected_candidate_target: [x, y]|null,
  selected_candidate_yaw: float|null,
  selection_priority_trace: [ordered decision notes],
  rejected_candidate_summaries: [compact candidate reject summaries],
  refinement_applied: bool,
  refinement_reject_reason: string|null,
  original_target_preserved_on_reject: bool,
  branch_scoring_changed: false,
  fallback_terminal_acceptance_used: false
}
```

Candidate summaries should be compact enough for JSONL but include the required hard-safety subitems.

## Future test contract

Minimum Phase88 test names and behaviors:

- `test_candidate_family_generation_contract`
  - verifies centerline projection + small lateral offsets + multiple forward offsets + corridor heading variants are generated deterministically and bounded.

- `test_safety_first_priority_selects_hard_safe_candidate_before_best_balance`
  - constructs a best-balance candidate that fails safety and a less-centered candidate that passes safety.
  - expects the less-centered hard-safe candidate to be selected.

- `test_reject_preserves_original_target_when_no_candidate_passes`
  - constructs candidates that all fail hard safety.
  - expects `refinement_applied=false`, `nav2_goal_target=original_target`, `original_target_preserved_on_reject=true`, and a specific reject reason.

- `test_guardrails_no_strategy_or_parameter_tuning`
  - verifies no Nav2 config, MPPI/controller, inflation, robot radius, clearance radius, map threshold, branch scoring, centerline gate, directional readiness, fallback, or terminal acceptance changes are part of the implementation.

Additional recommended tests:

- missing same-corridor evidence rejects.
- missing two-side-wall evidence rejects.
- baseline-already-risky comparison rejects or classifies ambiguous.
- exact centerline lethal candidate loses to safer lateral-offset candidate.
- heading variant that worsens front wedge is rejected.
- selected candidate records rank trace.
- rejected candidate summaries include all hard-safety subitems.

## Phase88 implementation handoff outline

Phase88, if explicitly opened later, should be narrow:

1. Add tests first.
2. Implement candidate-family generation in the existing Phase84 refinement helper, after branch candidate selection and before Nav2 dispatch.
3. Keep Phase84 single-candidate behavior available as a fallback diagnostics branch if evidence is missing.
4. Add safety-first candidate ranking without touching branch scoring.
5. Emit the Phase87 goal_events contract.
6. Run focused static tests plus Phase84/85/86 compatibility tests.
7. Run only bounded validation after static approval.

This handoff does not authorize implementation now.

## Acceptance criteria for this design review

- Phase87 proposal exists under `doc/doc_proposal`.
- Phase87 report exists under `doc/doc_report`.
- Static tests verify candidate-family tokens, priority order, future goal_events fields, future test contract, and guardrails.
- Guard diff for Nav2/config and runtime strategy files remains empty.
- No runtime simulation or long autonomous exploration is run.
- Phase88 not entered.

## Phase87 decision

`DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_IMPLEMENTATION_PHASE`

The accepted design direction is safety-first multi-candidate forward search: generate a bounded local family around corridor projection, filter by hard safety first, reject if no safe candidate exists, and only then use clearance and balance quality as tie-breakers.
