# Phase91: Two-step corridor alignment staging goal design review report

Status: COMPLETE_DESIGN_REVIEW_ONLY_STOP_BEFORE_PHASE92
Date: 2026-06-03
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## Scope

Phase91 is Design review only. It designs a future two-step corridor alignment staging goal approach:

```text
corridor-alignment / staging pose -> fresh evidence -> forward exploration goal
```

No runtime strategy changed. No navigation code or Nav2 configuration was changed. No Phase92 implementation started.

## Required inputs read

Phase91 read the required upstream evidence and design context:

1. `doc/doc_report/phase90_rejected_candidate_failure_landscape_diagnosis_report.md`
2. `log/phase90_rejected_candidate_failure_landscape_diagnosis/phase90_rejected_candidate_failure_landscape_diagnosis.json`
3. `doc/doc_report/phase89_safety_first_refinement_bounded_goal2_validation_report.md`
4. `doc/doc_report/phase88_safety_first_multi_candidate_forward_search_minimal_implementation_report.md`
5. `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`
6. Top-level `doc/doc_proposal` design documents:
   - `doc/doc_proposal/phase87_safety_first_multi_candidate_forward_search_design_review.md`
   - `doc/doc_proposal/phase83_corridor_aligned_intermediate_goal_design_review.md`
   - `doc/doc_proposal/phase76_obstacle_triggered_visual_root_cause_workflow.md`
   - `doc/doc_proposal/phase27_alt_r5_footprint_terminal_acceptance_design.md`
   - `doc/doc_proposal/phase27_alt_near_exit_fallback_design.md`

## Upstream evidence summary

Phase90 conclusion:

- classification: `STAGING_ALIGNMENT_GOAL_NEEDED`
- secondary classifications:
  - `SAFETY_FLOOR_DOMINANT_BLOCKER`
  - `CANDIDATE_FAMILY_TOO_LOCAL`

Phase89 / Phase88 Goal2-equivalent evidence:

- `candidate_count=63`
- `hard_safety_pass_candidate_count=0`
- `refinement_applied=false`
- `refinement_reject_reason=lethal_cost_regression`
- `original_target_preserved_on_reject=true`
- `branch_scoring_changed=false`
- `fallback_terminal_acceptance_used=false`
- outcome remained timeout / `goal_timeout`, not success

Phase90 failure landscape:

- safety_floor false count: 63 / 63
- footprint lethal regression false count: 27 / 63
- front-wedge lethal regression false count: 35 / 63
- clearance insufficient false count: 21 / 63
- forward_progress false count: 0 / 63
- occupancy false count: 0 / 63
- same_corridor false count: 0 / 63
- two_side_wall false count: 0 / 63

Interpretation:

The single-step forward refined goal family was locally unexecutable from the observed Goal2 pose neighborhood. The blocker was not a lack of same-corridor evidence, two-side-wall evidence, occupancy-free targets, or forward progress. It was primarily safety floor / clearance margin, with execution-time footprint/front-wedge risk still active near the final pose.

## New Phase91 files

- Design document:
  - `doc/doc_proposal/phase91_two_step_corridor_alignment_staging_goal_design_review.md`
- Report:
  - `doc/doc_report/phase91_two_step_corridor_alignment_staging_goal_design_review_report.md`
- Focused static test:
  - `src/tugbot_maze/test/test_phase91_two_step_corridor_alignment_staging_goal_design_review.py`

## Design review result

Design review complete.

Phase91 recommends a future two-step flow for explicit Phase92 consideration only:

1. Trigger only when the Phase90-style bundle is present:
   - near-goal lateral residual
   - single-step forward-search no hard-safety-pass
   - safety_floor dominant blocker
   - execution-time footprint/front-wedge risk

2. Generate a future staging pose as a local, short, hard-safe adjustment:
   - short-distance staging pose
   - low-risk staging motion
   - corridor heading
   - reduce lateral residual
   - reduce front-wedge risk
   - preserve same-corridor and two-side-wall evidence
   - avoid hidden branch changes or route replanning

3. Dispatch staging only if its executability check passes all hard-safety gates.

4. After staging success, gather fresh evidence:
   - re-acquire scan
   - re-acquire local costmap
   - re-acquire TF

5. Only after fresh evidence is available, run second-step forward goal refinement. The second-step forward exploration goal must be generated only after fresh evidence passes safety gates.

## Staging is not fallback or terminal acceptance

Phase91 explicitly preserves these semantics:

- staging is not fallback
- staging is not terminal acceptance
- timeout is not success
- staging success is not branch success
- staging success is not exit success
- second-step timeout remains timeout
- no fallback/terminal acceptance semantics are introduced

## Reject semantics

Future reject semantics should remain conservative:

- original target preserved on reject in ordinary runtime mode
- stop and wait for visual diagnosis in explicit visual/root-cause handoff mode
- do not lower safety floor
- do not lower local-cost, geometry, or clearance safety boundaries
- record a specific `staging_reject_reason`
- keep `branch_scoring_changed=false`
- keep `fallback_terminal_acceptance_used=false`

Phase91 proposed future reject tokens including:

- `trigger_bundle_not_satisfied`
- `missing_near_goal_lateral_residual_evidence`
- `single_step_forward_search_not_attempted`
- `single_step_forward_search_had_hard_safe_candidate`
- `missing_same_corridor_evidence`
- `missing_two_side_wall_evidence`
- `staging_candidate_family_empty`
- `all_staging_candidates_failed_hard_safety`
- `staging_safety_floor_blocked`
- `staging_footprint_lethal_regression`
- `staging_front_wedge_lethal_regression`
- `staging_not_bounded_short_distance`
- `staging_would_change_branch_or_score`
- `insufficient_fresh_tf_scan_or_local_costmap_evidence`
- `second_step_fresh_evidence_missing`
- `second_step_no_hard_safe_forward_goal`
- `visual_diagnosis_required_before_more_algorithm_changes`

## Future goal_events contract

Phase91 defines these future `/maze/goal_events` fields for Phase92 consideration:

- `two_step_staging_plan`
- `staging_goal_pose`
- `staging_reason`
- `staging_executability_check`
- `second_step_forward_goal`
- `staging_applied`
- `staging_reject_reason`
- `branch_scoring_changed=false`
- `fallback_terminal_acceptance_used=false`

Required nested diagnostics include:

- `two_step_staging_plan.enabled`
- trigger bundle booleans for near-goal lateral residual, no hard-safety-pass single-step search, safety-floor dominance, and execution-time footprint/front-wedge risk
- source single-step summary: candidate count, hard-safety-pass count, reject reason, original-target preservation
- `staging_executability_check.hard_safety_pass`
- `staging_executability_check.same_corridor`
- `staging_executability_check.two_side_wall_evidence`
- `staging_executability_check.safety_floor_ok`
- `staging_executability_check.footprint_lethal_not_increased`
- `staging_executability_check.front_wedge_lethal_not_increased`
- fresh evidence age fields
- `second_step_forward_goal.generated_after_fresh_evidence`
- fresh scan/local-cost/TF booleans for second-step gating

## Relationship to obstacle-triggered visual workflow

Phase91 incorporates the user’s workflow principle from Phase76/78:

When future exploration meets unresolved obstacles such as `goal_timeout`, `FAILED_EXHAUSTED`, `no_candidate`, `local_cost_risk`, recovery loop / `recovery_count>0`, near-goal outside tolerance, or re-dispatch readiness blocked, do not continue blind algorithm changes. Start or hold a bounded visual root-cause reproduction and wait for user/ChatGPT discussion before the next repair phase.

For the future two-step flow, if staging is rejected or the second step still times out/re-enters recovery/local-cost risk, the correct response is to stop and wait for visual diagnosis rather than keep expanding algorithm complexity.

## Guardrails

Phase91 preserved the requested boundaries:

- Design review only
- Phase92 not entered
- No runtime strategy changed
- No navigation code changed
- staging is not fallback
- staging is not terminal acceptance
- timeout is not success
- original target preserved on reject
- stop and wait for visual diagnosis when unresolved obstacles remain
- do not lower safety floor
- No Nav2/MPPI/controller tuning
- No inflation/robot_radius/clearance_radius_m/map threshold tuning
- No branch scoring changed
- No exploration order changed
- No centerline gate changed
- No directional readiness changed
- No fallback/terminal acceptance changed
- No autonomous exploration success claimed
- No exit success claimed

## Verification

TDD / static verification performed:

1. RED focused test first:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase91_two_step_corridor_alignment_staging_goal_design_review.py
```

Observed RED result before the proposal/report existed:

```text
6 failed in 0.03s
```

2. Proposal and report were written after RED.

3. Final focused tests:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase91_two_step_corridor_alignment_staging_goal_design_review.py
```

Observed:

```text
6 passed in 0.01s
```

4. Python syntax compile and runtime/config guard diff:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile src/tugbot_maze/test/test_phase91_two_step_corridor_alignment_staging_goal_design_review.py

git diff -- src/tugbot_navigation/config \
  src/tugbot_maze/tugbot_maze/maze_explorer.py \
  src/tugbot_maze/tugbot_maze/maze_perception.py
```

Observed:

```text
exit 0
empty guard diff output
```

5. `__pycache__` cleanup:

```text
removed_pycache_count= 1
removed src/tugbot_maze/test/__pycache__
remaining_pycache_count= 0
```

## Stop condition

Phase91 stops here. Phase92 not entered. No implementation was started.
