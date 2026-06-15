# Phase86 lethal_cost_regression reject root-cause diagnosis report

Status: COMPLETE_DIAGNOSIS_STOPPED

## Scope

Phase86 diagnoses why Phase85 observed Phase84 corridor-aligned refinement evaluating Goal2-equivalent candidates but rejecting the refinement with:

- `refinement_applied=false`
- `refinement_reject_reason=lethal_cost_regression`
- `forward_executability_check.passed=false`

This phase is diagnostic only. It reads Phase85 artifacts and earlier Phase80/81/82/83/84 reports. It does not change navigation behavior.

## Required sources read

- `doc/doc_report/phase85_goal2_corridor_aligned_refinement_bounded_validation_report.md`
- `doc/doc_report/phase84_corridor_aligned_intermediate_goal_refinement_minimal_implementation_report.md`
- `doc/doc_proposal/phase83_corridor_aligned_intermediate_goal_design_review.md`
- `doc/doc_report/phase80_goal2_near_goal_forward_open_corridor_diagnostic_classification_report.md`
- `doc/doc_report/phase81_goal2_forward_open_machine_evidence_capture_report.md`
- `doc/doc_report/phase82_goal2_local_cost_scan_mismatch_root_cause_report.md`
- `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`
- Phase80/81/82/85 JSON and minimal-summary artifacts under `log/`

## New Phase86 files

Analyzer:

- `tools/analyze_phase86_lethal_cost_regression_reject_root_cause.py`

Focused tests:

- `src/tugbot_maze/test/test_phase86_lethal_cost_regression_reject_root_cause.py`

Artifacts:

- `log/phase86_lethal_cost_regression_reject_root_cause/phase86_lethal_cost_regression_reject_root_cause.json`
- `log/phase86_lethal_cost_regression_reject_root_cause/phase86_lethal_cost_regression_reject_root_cause_minimal_summary.md`

This report:

- `doc/doc_report/phase86_lethal_cost_regression_reject_root_cause_report.md`

## Analyzer inputs

The analyzer reads the Phase85 bounded-validation artifacts:

- `phase85_goal2_corridor_aligned_refinement_bounded_validation_goal_events.jsonl`
- `phase85_goal2_corridor_aligned_refinement_bounded_validation_local_costmap_samples.jsonl`
- `phase85_goal2_corridor_aligned_refinement_bounded_validation_raw_capture.json`
- `phase85_goal2_corridor_aligned_refinement_bounded_validation_analysis.json`

The analyzer treats missing fields as evidence gaps. It does not infer unavailable metrics and does not fabricate missing evidence.

## Allowed conservative classifications

Phase86 analyzer emits exactly one primary classification from:

- `REFINED_TARGET_IN_HIGH_COST_BAND`
- `CENTERLINE_PROJECTION_DISTANCE_OR_POSE_TOO_AGGRESSIVE`
- `FORWARD_EXECUTABILITY_CHECK_TOO_CONSERVATIVE`
- `BASELINE_ALREADY_RISKY_COMPARISON_AMBIGUOUS`
- `MULTI_CANDIDATE_FORWARD_SEARCH_NEEDED`
- `INSUFFICIENT_REJECT_DIAGNOSTIC_EVIDENCE`

## Phase86 result

Primary classification:

`CENTERLINE_PROJECTION_DISTANCE_OR_POSE_TOO_AGGRESSIVE`

Classification reason:

- `best_balance_centerline_projection_candidates_failed_clearance_or_safety_floor`

Secondary supported classifications:

- `MULTI_CANDIDATE_FORWARD_SEARCH_NEEDED`
- `FORWARD_EXECUTABILITY_CHECK_TOO_CONSERVATIVE`

Evidence gaps:

- `[]`

Interpretation:

The Phase85 evidence is sufficient to say that the lethal_cost_regression reject was not caused by missing same-corridor or two-side-wall evidence. All 21 candidates had same-corridor and two-side-wall evidence. The conservative root-cause classification is that the candidate/projection pose family searched in Phase84 is too aggressive around the centerline/pose offsets for this local-cost geometry: best-balance candidates reduce centerline balance error but fail clearance/safety-floor checks, while many candidates enter high/lethal local-cost bands.

This is not an autonomous success claim and not an exit success claim.

## Source Phase85 context

Source Phase85 classification:

- `REFINEMENT_REJECTED_OR_NOT_TRIGGERED`

Lethal reject context:

- `refinement_applied: false`
- `refinement_reject_reason: lethal_cost_regression`
- `forward_executability_reason: lethal_cost_regression`
- `validation_goal_sequence: 1`
- `used_target_match_fallback: true`
- `candidate_count: 21`
- `two_side_wall_candidate_count: 21`
- `strict_eligible_candidate_count: 0`
- `eligible_candidate_count: 0`
- `balance_first_eligible_candidate_count: 0`
- `original_target: [2.084224301395533, 1.023027384249055]`
- `centerline_projected_target: null`
- `nav2_goal_target: [2.084224301395533, 1.023027384249055]`
- `corridor_heading_yaw: 1.5660394713676706`

## Reject subitem decomposition

The analyzer decomposed these required subitems:

### safety_floor_ok

- forward_check_value: `false`
- candidate_true_count: `1`
- candidate_false_count: `20`
- candidate_missing_count: `0`

Interpretation: only one candidate met the safety floor. The best-balance centerline-projection candidates did not pass the safety floor.

### footprint_lethal_not_increased

- forward_check_value: `false`
- candidate_true_count: `12`
- candidate_false_count: `9`
- candidate_missing_count: `0`

Interpretation: footprint lethal regression is present for 9 candidates, but not all candidates. This supports diagnosis at candidate-family/selection level rather than a blanket lack of footprint evidence.

### front_wedge_lethal_not_increased

- forward_check_value: `false`
- candidate_true_count: `9`
- candidate_false_count: `12`
- candidate_missing_count: `0`

Interpretation: front-wedge lethal regression is the strongest lethal-cost contributor: 12 of 21 candidates increased front-wedge lethal count.

### forward_progress_not_lowered

- forward_check_value: `false`
- candidate_true_count: `21`
- candidate_false_count: `0`
- candidate_missing_count: `0`

Interpretation: every recorded candidate says forward progress was not lowered, but the aggregate forward check says false. This mismatch is why `FORWARD_EXECUTABILITY_CHECK_TOO_CONSERVATIVE` is retained as a secondary supported classification. Phase86 does not change the check.

### target_has_clearance

- forward_check_value: `true`
- candidate_true_count: `14`
- candidate_false_count: `7`
- candidate_missing_count: `0`

Interpretation: many candidates have target clearance, but best-balance forward-offset centerline candidates include clearance failures.

### occupancy_free

- forward_check_value: `true`
- candidate_true_count: `21`
- candidate_false_count: `0`
- candidate_missing_count: `0`

Interpretation: occupancy grid free-space is not the rejecting factor.

### same_corridor

- forward_check_value: `true`
- candidate_true_count: `21`
- candidate_false_count: `0`
- candidate_missing_count: `0`

Interpretation: same-corridor evidence is not the rejecting factor.

### two_side_wall_evidence

- forward_check_value: `true`
- candidate_true_count: `21`
- candidate_false_count: `0`
- candidate_missing_count: `0`

Interpretation: two-side-wall evidence is not the rejecting factor.

## Candidate aggregate evidence

Candidate aggregate from Phase85:

- `candidate_count: 21`
- `eligible_candidate_count: 0`
- `balance_first_eligible_candidate_count: 0`
- `hard_safety_pass_candidate_count: 1`
- `high_or_lethal_candidate_count: 12`
- `lethal_regression_candidate_count: 12`
- `best_balance_error_m: 0.0`
- `best_balance_candidate_count: 4`
- `best_balance_candidates_all_clearance_failed: true`
- `best_balance_candidates_all_high_or_lethal: false`
- `original_local_cost_max_radius: 54.0`
- `original_front_wedge_lethal_count: 0.0`
- `original_footprint_lethal_count: 0.0`
- `original_min_clearance_m: 0.42720019363165534`
- `lateral_offset_range_m: [-0.45, 0.45]`
- `forward_offset_range_m: [0.0, 0.2]`
- `local_cost_max_radius_range: [54.0, 99.0]`
- `front_wedge_lethal_count_range: [0.0, 80.0]`
- `footprint_lethal_count_range: [0.0, 53.0]`
- `min_clearance_range_m: [0.05000000074505806, 0.45000000670552254]`

Key diagnostic points:

1. Baseline comparison is not obviously already lethal:
   - original local_cost_max_radius is 54.0
   - original front_wedge_lethal_count is 0.0
   - original footprint_lethal_count is 0.0

2. Candidate search found many high/lethal poses:
   - 12 of 21 candidates are high/lethal
   - 12 of 21 candidates are lethal regressions

3. But the result is not simply “all refined targets are unsafe”:
   - 1 candidate passes all hard safety subitems
   - 12 candidates do not increase footprint lethal count
   - 9 candidates do not increase front-wedge lethal count

4. Best-balance projection is suspicious:
   - 4 best-balance candidates reached balance_error_m 0.0
   - all best-balance candidates failed clearance or safety floor

This supports the primary classification `CENTERLINE_PROJECTION_DISTANCE_OR_POSE_TOO_AGGRESSIVE` with secondary support for `MULTI_CANDIDATE_FORWARD_SEARCH_NEEDED` and `FORWARD_EXECUTABILITY_CHECK_TOO_CONSERVATIVE`.

## Runtime local-cost sample evidence

Phase85 local-cost samples for validation goal sequence 1:

- sample_count: `190`
- robot_footprint_lethal_count:
  - min: `0.0`
  - max: `54.0`
  - last: `52.0`
- target_footprint_lethal_count:
  - min: `0.0`
  - max: `0.0`
  - last: `0.0`
- front_wedge_lethal_count:
  - min: `0.0`
  - max: `148.0`
  - last: `70.0`
- front_wedge_cost_max:
  - min: `54.0`
  - max: `100.0`
  - last: `99.0`
- target_radius_cost_max:
  - min: `54.0`
  - max: `63.0`
  - last: `63.0`

Interpretation:

The executed original target itself remains target-footprint safe in local-cost samples, but robot footprint and front wedge become lethal during execution. This is consistent with the Phase80/81/82 diagnosis that execution-time footprint/front-wedge local cost is the active problem. Phase86 does not use this to justify parameter tuning.

## Raw capture availability

Phase85 raw capture fields were present:

- scan_available: `true`
- local_costmap_available: `true`
- footprint_available: `true`
- odom_available: `true`
- tf_available: `true`

No missing-evidence fallback was needed for the Phase86 primary classification.

## Relation to Phase80/81/82

Phase80 showed near-goal lateral residual with forward-open corridor suspicion.

Phase81 added machine evidence that raw scan was forward-open but local-cost execution corridor was high/lethal.

Phase82 classified the root cause as:

- primary: `FOOTPRINT_OR_WEDGE_PROJECTION_SUSPECTED`
- secondary: `INFLATION_SPILLOVER_SUSPECTED`

Phase86 is consistent with those results: the candidate family is inside a geometry where centerline/pose refinements easily encounter front-wedge or footprint lethal-cost bands. Phase86 does not prove that changing Nav2/inflation/controller parameters would solve the issue, and no such tuning was performed.

## Conservative conclusion

The Phase85 `lethal_cost_regression` reject is best diagnosed as:

`CENTERLINE_PROJECTION_DISTANCE_OR_POSE_TOO_AGGRESSIVE`

with secondary evidence for:

- `MULTI_CANDIDATE_FORWARD_SEARCH_NEEDED`
- `FORWARD_EXECUTABILITY_CHECK_TOO_CONSERVATIVE`

Meaning:

The attempted centerline/balance refinement candidates appear too aggressive for the local-cost/footprint/front-wedge geometry near Goal2. Some safer candidates exist, but the current candidate family and check combination produces no eligible candidate. The evidence supports a future design discussion around multi-candidate forward search and/or check interpretation, but Phase86 intentionally makes no strategy change.

## Guardrails

Phase86 did not modify runtime navigation strategy.

- No maze_explorer strategy changed
- No branch scoring changed
- No centerline gate changed
- No directional readiness override changed
- No fallback/terminal acceptance changed
- No Nav2/MPPI/controller tuning
- No inflation/robot_radius/clearance_radius_m/map threshold tuning
- No autonomous exploration success claimed
- No exit success claimed
- Phase87 not entered

## Validation

Focused tests and static validation were run after this report was created.

Expected validation commands:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase86_lethal_cost_regression_reject_root_cause.py
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile tools/analyze_phase86_lethal_cost_regression_reject_root_cause.py src/tugbot_maze/test/test_phase86_lethal_cost_regression_reject_root_cause.py
git diff -- src/tugbot_navigation/config src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_maze/tugbot_maze/maze_perception.py
```

Final command outputs are recorded in the final assistant response for Phase86.
