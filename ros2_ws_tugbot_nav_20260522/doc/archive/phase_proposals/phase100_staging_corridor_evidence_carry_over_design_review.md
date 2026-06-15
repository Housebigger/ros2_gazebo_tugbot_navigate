# Phase100: Staging corridor evidence carry-over design review

Status: `DESIGN_REVIEW_ONLY_NOT_RUNTIME_ENABLED`

Decision: `DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_PHASE101_IMPLEMENTATION`

Phase100 is a design-review-only phase. No runtime code is changed. No Phase88/92 logic is changed. Phase101 not entered.

## Purpose

Phase99 diagnosed Goal1 Phase92 staging rejection as:

`STAGING_CANDIDATE_TOO_SHORT_OR_TOO_NEAR_FOR_WALL_EVIDENCE`

The key artifact-level evidence was:

- Phase88 forward Goal1 target/candidate window had same-corridor and two-side-wall evidence.
- Phase92 staging candidates were very short and near the dispatch/entrance pose.
- Phase92 staging window missed two-side-wall evidence, so staging was rejected with `missing_two_side_wall_evidence` even though the Phase88 forward source window had corridor-level evidence.

Phase100 designs a future explicit Phase101 implementation direction: Phase88 corridor evidence carry-over / evidence reuse for Phase92 staging eligibility. This design is intentionally narrow and does not authorize implementation in Phase100.

## Required reading completed

- `doc/doc_report/phase99_goal1_staging_evidence_path_diagnosis_report.md`
- `doc/doc_report/phase98_goal1_recovery_dominant_failure_root_cause_diagnosis_report.md`
- `doc/doc_report/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_report.md`
- `doc/doc_report/phase92_two_step_corridor_alignment_staging_goal_minimal_implementation_report.md`
- `doc/doc_report/phase88_safety_first_multi_candidate_forward_search_minimal_implementation_report.md`
- `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`
- top-level design docs under `doc/doc_proposal`, including Phase91, Phase87, Phase83, Phase76, Phase27-alt-R5, and Phase27-alt near-exit fallback designs.

## Evidence basis from Phase99

Phase99 established:

```text
Phase92 staging candidates:
  candidate_count=9
  dispatch_pose≈[1.86054, 0.02512, -0.00722]
  staging y range≈0.024..0.124
  max staging distance≈0.180 m
  same_corridor_true_count=9
  two_side_wall_true_count=0
  staging_reject_reason=missing_two_side_wall_evidence

Phase88 forward candidates:
  candidate_count=63
  forward y range≈0.921..1.127
  same_corridor_true_count=63
  two_side_wall_true_count=63
  hard_safety_pass_candidate_count=0
  refinement_reject_reason=lethal_cost_regression

Window separation:
  Phase88_min_y - Phase92_max_y≈0.796 m
```

Interpretation:

- Phase92 did not fail because the selected branch globally lacked corridor evidence.
- Phase92 failed because the staging evidence path used a short near-dispatch window that did not see two-side-wall evidence.
- Phase88 had corridor-level same-corridor / two-side-wall evidence in the forward window.
- A future implementation may consider reusing only the corridor-level Phase88 evidence for staging eligibility, while recomputing all staging safety/execution evidence from the staging candidate itself.

## Non-goals and hard guardrails

Phase100 does not authorize any runtime behavior change.

- No runtime code is changed.
- No Phase88/92 logic is changed.
- No maze_explorer strategy change.
- No branch scoring change.
- No exploration order change.
- No centerline gate change.
- No directional readiness change.
- No fallback/terminal acceptance change.
- No Nav2/MPPI/controller tuning.
- No inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No autonomous exploration success claim.
- No exit success claim.
- Timeout/failure remains timeout/failure.
- Staging success, if implemented later, must not become branch success, exploration success, exit success, fallback, or terminal acceptance.

## Design principle

Carry-over is allowed only for corridor-level geometric evidence that describes the selected corridor relation between the forward Phase88 source window and the candidate branch.

Carry-over must never set or imply hard_safety_pass=true, safety_floor_ok=true, occupancy_free=true, target_has_clearance=true, footprint_lethal_not_increased=true, front_wedge_lethal_not_increased=true, or fresh scan/local_costmap/TF availability.

The staging itself must recompute all safety/execution evidence from fresh or current staging-context inputs before it can apply.

## Evidence allowed to carry over

Only these corridor-level evidence items may be carried over from the Phase88 forward window into Phase92 staging eligibility:

1. `same_corridor`
   - Source meaning: the Phase88 forward target/candidate window was in the same selected corridor as the dispatch pose and branch direction.
   - Carry-over meaning: staging may treat the source corridor identity as supporting evidence only if staging geometry remains coherent with the same selected corridor.

2. `two_side_wall_evidence`
   - Source meaning: Phase88 forward window observed both side-wall/cost-boundary contexts for the selected corridor.
   - Carry-over meaning: staging may use the source forward two-side-wall corridor context when the staging window is too short/near-dispatch to see both walls by itself.

3. `corridor_heading`
   - Source meaning: Phase88 estimated a forward corridor heading for the already-selected branch.
   - Carry-over meaning: staging yaw may use this corridor heading only if heading agreement gates pass.

4. `wall clearance context`
   - Source meaning: source forward window recorded left/right wall hit/clearance context and corridor balance evidence.
   - Carry-over meaning: staging may record this as corridor context, not as a staging safety floor or clearance pass.

5. `source_forward_window`
   - Source meaning: the bounded Phase88 spatial window from which the corridor-level evidence was observed.
   - Carry-over meaning: diagnostics must expose the source window so later analyzers can audit whether the evidence is relevant to the staging window.

## Evidence forbidden to carry over

These safety/execution facts are not corridor-level evidence and must never be carried over:

- `hard_safety_pass`
- `safety_floor_ok`
- `occupancy_free`
- `target_has_clearance`
- `footprint_lethal_not_increased`
- `front_wedge_lethal_not_increased`
- `fresh scan/local_costmap/TF`

Additional forbidden implications:

- A source Phase88 `two_side_wall_evidence=true` must not imply the staging target cell is free.
- A source Phase88 wall clearance context must not imply the staging footprint is clear.
- A source Phase88 corridor heading must not imply the staging front wedge is safe.
- A source Phase88 candidate that failed hard safety must not make staging hard-safe.
- Carry-over must not weaken or bypass any local-cost, map, TF, scan, footprint, front-wedge, or clearance check.

## Staging safety/execution evidence must be recomputed

A future Phase101 implementation may use carry-over only to satisfy the corridor-context part of staging eligibility. Staging itself must recompute:

- `hard_safety_pass`
- `safety_floor_ok`
- `occupancy_free`
- `target_has_clearance`
- `footprint_lethal_not_increased`
- `front_wedge_lethal_not_increased`
- local-cost target/path/footprint/front-wedge samples
- map occupancy / unknown policy checks
- scan consistency if available
- TF transform and timestamp age
- local_costmap timestamp age
- bounded short-distance check
- forward/non-negative progress check
- branch-consistency check

If recomputed staging evidence fails, staging must reject even when corridor-level carry-over is available.

## Trigger conditions for carry-over evaluation

A future implementation may evaluate corridor evidence carry-over only when all of these are true:

1. Phase92 staging trigger bundle is satisfied.
   - near-goal lateral residual
   - single-step forward search no hard-safety-pass
   - safety_floor dominant blocker
   - execution-time footprint/front-wedge risk

2. The staging window is missing corridor evidence:
   - staging window missing two-side-wall, or staging candidate family has `two_side_wall_evidence=false` due to short/near-dispatch geometry.

3. The Phase88 forward window has same-corridor/two-side-wall evidence:
   - Phase88 forward window has same-corridor/two-side-wall.
   - `same_corridor=true` and `two_side_wall_evidence=true` are present in the Phase88 forward source metrics or candidate summaries.

4. Robot/staging/forward target geometry coherent:
   - robot/staging/forward target geometry coherent with one selected corridor.
   - staging candidate remains between dispatch pose and forward target or inside the bounded corridor segment.
   - staging candidate does not cross behind dispatch pose or switch to another branch.
   - staging yaw/heading agrees with the Phase88 corridor heading within an explicit bounded heading tolerance.
   - staging candidate remains within a bounded near-dispatch/staging neighborhood and the source forward window remains relevant to that corridor.

5. Source evidence is auditable:
   - source event sequence and timestamp are available.
   - source frame ID is known and compatible.
   - source forward window bounds are recorded.
   - source candidate/metrics fields explicitly show the corridor-level pass.

If any trigger member is missing, carry-over is not applied and the original Phase92 rejection semantics remain intact.

## Reject conditions

A future implementation must reject carry-over and record a specific `carry_over_reject_reason` if any of these occur:

- `carry_over_source_stale`
  - source event is too old for the current dispatch/staging decision or cannot be tied to the current selected branch.

- `frame_mismatch`
  - source evidence and staging candidate use incompatible frames, missing transforms, or inconsistent frame IDs.

- `heading_mismatch`
  - staging yaw/corridor direction diverges from the source corridor heading beyond the accepted tolerance.

- `forward_window_not_trustworthy`
  - source forward window has missing source metrics, ambiguous wall evidence, unsafe/contradictory source geometry, or insufficient candidate diagnostics.

- `staging_not_consistent_with_source_corridor`
  - staging pose is not between dispatch and forward target, crosses a wall/branch boundary, reverses progress, or lies outside the selected corridor geometry.

- `source_missing_same_corridor_or_two_side_wall`
  - source forward window does not explicitly show both source corridor evidence booleans.

- `source_branch_or_goal_sequence_mismatch`
  - source evidence came from a different goal sequence, branch, or selected target context.

- `staging_safety_recompute_failed`
  - corridor carry-over was available, but recomputed staging hard-safety / occupancy / clearance / footprint / front-wedge checks failed.

- `insufficient_carry_over_evidence`
  - required source/staging diagnostics are unavailable. Missing evidence must not be fabricated.

## Proposed future goal_events contract

A future Phase101 implementation should extend `/maze/goal_events` without removing Phase88/92 fields.

Required fields:

```text
corridor_evidence_carry_over: {
  evaluated: bool,
  eligible: bool,
  allowed_fields: [
    "same_corridor",
    "two_side_wall_evidence",
    "corridor_heading",
    "wall_clearance_context",
    "source_forward_window"
  ],
  forbidden_fields: [
    "hard_safety_pass",
    "safety_floor_ok",
    "occupancy_free",
    "target_has_clearance",
    "footprint_lethal_not_increased",
    "front_wedge_lethal_not_increased",
    "fresh_scan_local_costmap_tf"
  ]
}
carry_over_source: {
  goal_sequence: int|null,
  source_event_type: string|null,
  source_timestamp: float|null,
  frame_id: string|null,
  same_corridor: bool|null,
  two_side_wall_evidence: bool|null,
  corridor_heading: float|null,
  wall_clearance_context: dict|null,
  source_forward_window: dict|null
}
carry_over_applied: bool
carry_over_reject_reason: string|null
source_forward_window: {
  target_xy: [float, float]|null,
  candidate_count: int|null,
  y_range: {min: float|null, max: float|null},
  x_range: {min: float|null, max: float|null},
  corridor_heading: float|null,
  same_corridor_count: int|null,
  two_side_wall_count: int|null
}
staging_window: {
  candidate_count: int|null,
  y_range: {min: float|null, max: float|null},
  x_range: {min: float|null, max: float|null},
  max_staging_distance_m: float|null,
  two_side_wall_count: int|null,
  same_corridor_count: int|null
}
safety_evidence_recomputed=true
branch_scoring_changed=false
fallback_terminal_acceptance_used=false
```

Compatibility notes:

- Existing `two_step_staging_plan`, `staging_goal_pose`, `staging_executability_check`, `staging_applied`, and `staging_reject_reason` fields remain required.
- `carry_over_applied=true` can only mean corridor-level evidence was reused. It must not mean staging goal was applied.
- `staging_applied=true` still requires recomputed staging hard safety.
- `safety_evidence_recomputed=true` must be present when carry-over is evaluated or applied.
- `branch_scoring_changed=false` and `fallback_terminal_acceptance_used=false` must remain explicit.

## Future decision flow

```text
Phase92 trigger bundle satisfied
  -> generate staging candidate family
  -> staging local corridor evidence missing two-side-wall
  -> inspect Phase88 source forward window
  -> if source corridor evidence available and geometry coherent:
       corridor_evidence_carry_over.evaluated=true
       carry_over_applied=true only for same_corridor/two_side_wall/corridor_heading context
       recompute staging safety/execution evidence
       if recomputed hard safety passes:
           staging_applied=true
       else:
           staging_applied=false; staging_reject_reason=<safety reason>
     else:
       carry_over_applied=false; carry_over_reject_reason=<specific reason>
       preserve current Phase92 reject semantics
```

## Future tests for Phase101 implementation

If Phase101 is explicitly opened, minimum tests should be written before implementation:

1. Positive corridor carry-over eligibility
   - Phase92 staging window missing two-side-wall.
   - Phase88 forward window has same_corridor/two_side_wall/corridor_heading.
   - robot/staging/forward target geometry coherent.
   - carry-over applies only corridor-level fields.

2. Safety still recomputed
   - carry-over source has corridor evidence.
   - staging hard safety fails.
   - expect `carry_over_applied=true`, `safety_evidence_recomputed=true`, `staging_applied=false`, and a safety-specific `staging_reject_reason`.

3. Forbidden safety carry-over
   - source hard_safety_pass or safety_floor_ok cannot make staging pass.
   - assert staging recomputation is required and recorded.

4. Stale/source mismatch rejection
   - stale source, frame mismatch, heading mismatch, or source branch mismatch rejects carry-over with specific reason.

5. Goal event schema
   - all future fields appear in dispatch/staging diagnostics.
   - `branch_scoring_changed=false` and `fallback_terminal_acceptance_used=false` remain false.

6. Guardrails
   - no Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map threshold changes.
   - no branch scoring/exploration order/centerline gate/directional readiness/fallback/terminal acceptance changes.

## Relationship to obstacle reproduction handoff

Carry-over is not a license for blind algorithm changes after a failure. If staging with corridor carry-over still leads to timeout, recovery loop, local-cost risk, no candidate, failed exhausted, near-goal outside tolerance, or re-dispatch readiness block, the obstacle reproduction handoff workflow remains the governance stop:

- run bounded visible reproduction only;
- keep Gazebo/RViz observable at the problem scene;
- collect minimal field summary;
- ask for screenshot/user judgment;
- execute any confirmed repair only in a later explicit phase.

## Acceptance criteria for Phase100

- Phase100 design document exists under `doc/doc_proposal`.
- Phase100 report exists under `doc/doc_report`.
- Minimal static tests verify the design-only decision, allowed/forbidden carry-over fields, trigger/reject conditions, future goal_events fields, and guardrails.
- Nav2 config diff guard remains zero.
- Runtime strategy files are not edited in Phase100.
- No simulation, runtime, or long exploration run is started.
- Phase101 not entered.

## Phase100 decision

`DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_PHASE101_IMPLEMENTATION`

Phase100 recommends a narrow future design: reuse only Phase88 corridor-level evidence for Phase92 staging eligibility when the staging window is too short/near-dispatch to see two-side-wall evidence by itself, while requiring staging to recompute all safety/execution evidence. This preserves safety boundaries and diagnostics while addressing the Phase99 evidence-path gap.

Phase101 not entered.
