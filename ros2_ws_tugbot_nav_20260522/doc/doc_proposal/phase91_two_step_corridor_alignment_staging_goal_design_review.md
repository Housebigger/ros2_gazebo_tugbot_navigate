# Phase91: Two-step corridor alignment staging goal design review

Status: `DESIGN_REVIEW_ONLY_NOT_RUNTIME_ENABLED`

Phase91 is a design-review-only phase for a future two-step corridor alignment staging flow. It does not modify runtime code, navigation strategy, branch scoring, exploration order, centerline gates, directional readiness, fallback, terminal acceptance, Nav2, MPPI, controller, inflation, robot radius, `clearance_radius_m`, or map thresholds.

Phase92 may consider implementation only after explicit approval. Do not implement in Phase91.

## Evidence basis

Phase90 classification: STAGING_ALIGNMENT_GOAL_NEEDED.

Required Phase91 inputs read before this design:

- `doc/doc_report/phase90_rejected_candidate_failure_landscape_diagnosis_report.md`
- `log/phase90_rejected_candidate_failure_landscape_diagnosis/phase90_rejected_candidate_failure_landscape_diagnosis.json`
- `doc/doc_report/phase89_safety_first_refinement_bounded_goal2_validation_report.md`
- `doc/doc_report/phase88_safety_first_multi_candidate_forward_search_minimal_implementation_report.md`
- `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`
- top-level design documents under `doc/doc_proposal`

Key upstream findings:

- Phase89 Goal2-equivalent outcome was timeout / `goal_timeout`, not success.
- Phase88 multi-candidate forward search emitted 63 candidate diagnostics.
- `hard_safety_pass_candidate_count=0`.
- `refinement_applied=false` and `refinement_reject_reason=lethal_cost_regression`.
- `original_target_preserved_on_reject=true`.
- Phase90 found safety floor failed for all 63/63 candidates.
- same-corridor, two-side-wall, occupancy, and forward-progress were not the main blocker.
- execution-time footprint/front-wedge risk remained present near the final/near-goal pose.
- The local single-step forward refined goal is currently unexecutable from the observed Goal2 pose neighborhood.

Design implication:

A later implementation should not keep widening the one-step forward refined-goal search around the same near-goal pose. Instead, it should first attempt a short-distance staging pose that improves the robot body pose relative to the corridor, then re-evaluate the forward exploration goal from fresh scan/local-cost/TF evidence.

## Non-goals and hard guardrails

Design review only.

- Phase92 not entered.
- No runtime strategy changed.
- No navigation code changed.
- No Nav2/MPPI/controller tuning.
- No inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No branch scoring changed.
- No exploration order changed.
- No centerline gate changed.
- No directional readiness changed.
- No fallback/terminal acceptance changed.
- No autonomous exploration success claimed.
- No exit success claimed.
- timeout is not success.

The staging step is a future candidate target-generation/refinement layer only. It is not a fallback. It is not terminal acceptance. It is not a success shortcut. It must not mark branches blocked, blacklist goals, mutate DFS topology, or change the selected branch.

## Trigger conditions for a future two-step staging flow

A future Phase92 implementation may evaluate the two-step flow only when the already-selected branch/candidate is in the same failure class observed by Phase90. Suggested trigger bundle:

1. near-goal lateral residual
   - The robot is near the dispatch target, but the residual is dominated by lateral/cross-corridor error or final orientation mismatch.
   - The residual is treated as a target-generation/execution issue, not a reason to accept timeout.

2. single-step forward-search no hard-safety-pass
   - Phase88-style safety-first multi-candidate forward search ran.
   - `candidate_count > 0` and `hard_safety_pass_candidate_count == 0`.
   - `refinement_applied=false` and the original target was preserved.

3. safety_floor dominant blocker
   - Candidate diagnostics show `safety_floor_ok=false` dominates the local candidate family.
   - same-corridor and two-side-wall evidence remain required before any staging pose is considered.
   - occupancy and forward progress should not be the primary failure dimensions.

4. execution-time footprint/front-wedge risk
   - Runtime samples, `/maze/goal_events`, or visual workflow evidence show footprint or front-wedge high/lethal risk near the near-goal/terminal pose.
   - The staging objective is to reduce body-envelope risk before asking Nav2 to finish the forward exploration goal.

If any required trigger evidence is missing, the future helper should not fabricate it. It should preserve the original target or stop and wait for visual diagnosis, depending on the phase/run mode.

## Future two-step core flow

The future flow remains scoped to the already-selected exploration branch:

```text
already-selected candidate target
  -> Phase88 single-step forward-search diagnostics
  -> trigger bundle says local forward family is unexecutable
  -> generate short-distance staging pose
  -> staging executability check
  -> apply staging goal only if hard-safe
  -> staging success
  -> re-acquire scan
  -> re-acquire local costmap
  -> re-acquire TF
  -> forward goal refinement
  -> dispatch second-step forward exploration goal only if the new evidence passes safety gates
```

The second step must not reuse stale dispatch-time local-cost evidence for the second step; in short, do not reuse stale dispatch-time local-cost evidence for the second step. The future implementation must explicitly re-acquire scan, re-acquire local costmap, and re-acquire TF after staging success, because the purpose of staging is to change the robot pose/body envelope before re-evaluating the forward goal.

The second-step forward exploration goal is generated only after fresh evidence passes safety gates.

## Staging pose generation design

The staging pose is a short-distance staging pose, not a longer route and not an alternate branch. It should be generated in a small local neighborhood of the current robot pose and already-selected corridor axis.

Objectives:

- low-risk staging motion;
- corridor heading;
- reduce lateral residual;
- reduce front-wedge risk;
- reduce robot footprint/body-envelope squeeze;
- keep forward progress non-negative;
- keep same-corridor and two-side-wall evidence valid;
- stay bounded so this remains a staging adjustment, not a hidden replanner.

Candidate staging pose components:

1. XY position
   - Start from the current robot pose at/near the failing target execution point.
   - Estimate the same corridor axis using existing same-corridor/two-side-wall evidence.
   - Move only a short distance along a vector that reduces lateral offset to the corridor centerline.
   - Allow a small forward component only if it does not worsen front-wedge/footprint local-cost risk.
   - Do not move behind the dispatch pose or toward a different branch.

2. Yaw
   - Use corridor heading, not the vector from the robot to the original target if that vector would force a lateral finishing motion.
   - The goal orientation should align the Tugbot front wedge with the traversable corridor.
   - Heading variants may be considered only as diagnostic candidates and must be rejected if they worsen footprint/front-wedge risk.

3. Distance bounds
   - The staging pose should be shorter than a normal forward exploration goal.
   - It exists only to improve body alignment and lateral placement, not to solve the maze by a hidden micro-planner.
   - A future implementation should expose its exact candidate offsets in diagnostics, but Phase91 does not choose numeric tuning values.

4. Risk model
   - The staging pose must pass the same hard-safety family used by Phase88 where applicable: occupancy, same-corridor, two-side-wall, safety floor, footprint lethal regression, front-wedge lethal regression, forward progress, and sufficient fresh local-cost/TF evidence.
   - do not lower safety floor.
   - Do not tune local-cost thresholds, inflation, robot radius, or `clearance_radius_m` to make staging pass.

## Staging executability check

A future staging executability check should record both pass/fail and the reason. Minimum conceptual gates:

- `same_corridor=true`
- `two_side_wall_evidence=true`
- `occupancy_free=true`
- `target_has_clearance=true`
- `safety_floor_ok=true`
- `footprint_lethal_not_increased=true`
- `front_wedge_lethal_not_increased=true`
- `forward_progress_ok=true` or `non_negative_progress_for_alignment=true`
- TF/local-cost/scan samples fresh enough by the existing diagnostics policy
- staging pose is bounded to the local pose/corridor neighborhood

If all gates pass, a future implementation may dispatch the staging goal. If any gate fails, reject staging and either preserve the original target or stop and wait for visual diagnosis. The choice between those two reject responses should be phase/run-mode dependent:

- ordinary autonomous runtime mode: original target preserved on reject, with `staging_applied=false` and a specific `staging_reject_reason`.
- visual/root-cause workflow mode after repeated unresolved obstacles: stop and wait for visual diagnosis, keeping the scene available for user observation.

## Second-step forward exploration goal after staging

The second-step forward goal is not precomputed at the original dispatch pose. It is generated only after staging success.

Required sequence:

1. Confirm staging success using Nav2 result plus robot pose evidence.
2. re-acquire scan.
3. re-acquire local costmap.
4. re-acquire TF.
5. Re-run same-corridor and two-side-wall evidence checks from the staged pose.
6. Re-run Phase88-style safety-first forward goal refinement from fresh evidence.
7. Dispatch the second-step forward exploration goal only if the selected candidate passes hard safety.

Second-step reject semantics:

- If no hard-safe second-step forward candidate exists, preserve the relevant original target for that step or stop for visual diagnosis in an explicit handoff mode.
- Do not lower safety floor.
- Do not treat staging success as branch success, exit success, or terminal acceptance.
- Do not treat a second-step timeout as success.

## Reject reasons for a future implementation

Recommended future `staging_reject_reason` tokens:

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

Reject path contract:

- `staging_applied=false`
- `staging_goal_pose=null`
- `second_step_forward_goal=null`
- `original target preserved on reject`
- `staging_reject_reason=<specific token>`
- `branch_scoring_changed=false`
- `fallback_terminal_acceptance_used=false`

## Future goal_events contract

A future Phase92 implementation should emit the following fields in `/maze/goal_events` dispatch/staging/second-step events.

Top-level required fields:

- `two_step_staging_plan`
- `staging_goal_pose`
- `staging_reason`
- `staging_executability_check`
- `second_step_forward_goal`
- `staging_applied`
- `staging_reject_reason`
- `branch_scoring_changed=false`
- `fallback_terminal_acceptance_used=false`

Suggested nested shape:

```text
two_step_staging_plan.enabled: bool
two_step_staging_plan.trigger_conditions: {
  near_goal_lateral_residual: bool,
  single_step_forward_search_no_hard_safety_pass: bool,
  safety_floor_dominant_blocker: bool,
  execution_time_footprint_front_wedge_risk: bool
}
two_step_staging_plan.source_single_step: {
  candidate_count: int,
  hard_safety_pass_candidate_count: int,
  refinement_applied: bool,
  refinement_reject_reason: string|null,
  original_target_preserved_on_reject: bool
}
staging_goal_pose: {x: float, y: float, yaw: float}|null
staging_reason: string|null
staging_executability_check.hard_safety_pass: bool
staging_executability_check.same_corridor: bool
staging_executability_check.two_side_wall_evidence: bool
staging_executability_check.safety_floor_ok: bool
staging_executability_check.footprint_lethal_not_increased: bool
staging_executability_check.front_wedge_lethal_not_increased: bool
staging_executability_check.forward_progress_ok: bool
staging_executability_check.local_costmap_stamp_age_sec: float|null
staging_executability_check.tf_stamp_age_sec: float|null
staging_applied: bool
staging_reject_reason: string|null
second_step_forward_goal.generated_after_fresh_evidence: bool
second_step_forward_goal.fresh_scan_received: bool
second_step_forward_goal.fresh_local_costmap_received: bool
second_step_forward_goal.fresh_tf_received: bool
second_step_forward_goal.selected_candidate_target: [x, y]|null
second_step_forward_goal.selected_candidate_yaw: float|null
second_step_forward_goal.hard_safety_pass_candidate_count: int|null
branch_scoring_changed=false
fallback_terminal_acceptance_used=false
```

## Relationship to visual root-cause workflow

Phase76/78 visual workflow remains the governance stop for unresolved obstacles. If a future Phase92 staging implementation rejects for safety reasons or still produces a timeout/recovery loop, the correct next action is a bounded visible RViz/Gazebo reproduction, not blind algorithm edits.

The future staging flow should therefore be integrated with the obstacle handoff policy:

- repeated `goal_timeout`, `local_cost_risk`, recovery loop, near-goal outside tolerance, `no_candidate`, `FAILED_EXHAUSTED`, or re-dispatch readiness blocked -> stop and wait for visual diagnosis before further changes.
- the visual workflow should show dispatch target, staging target, second-step target if any, robot trajectory, terminal pose, footprint, front wedge, nearest wall, local-cost high/lethal cells, corridor centerline, and goal tolerance circle.

## Future Phase92 test contract

If Phase92 is explicitly opened later, minimum tests should cover:

- trigger bundle positive case from Phase90-like diagnostics.
- trigger bundle negative case when single-step forward search still has a hard-safe candidate.
- staging candidate generation is short-distance and same-corridor.
- staging yaw uses corridor heading.
- staging candidate that lowers lateral residual but worsens front-wedge risk is rejected.
- staging candidate that lowers front-wedge risk but violates safety floor is rejected.
- successful staging requires fresh scan/local-cost/TF before second-step forward refinement.
- stale evidence prevents second-step forward goal generation.
- reject preserves original target or enters explicit visual diagnosis wait mode; it never lowers safety floor.
- `branch_scoring_changed=false` and `fallback_terminal_acceptance_used=false` remain recorded.

## Phase91 decision

`DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_IMPLEMENTATION_PHASE`

Phase91 recommends a conservative two-step direction for a later explicit Phase92: first generate a hard-safe short corridor-alignment staging pose to improve body orientation and lateral placement; after staging success, reacquire evidence and then run the forward exploration goal refinement again. This design remains inactive in Phase91.
