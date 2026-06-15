# Phase132 corridor alignment staging contract design

Status: DESIGN_ONLY

## Purpose

Phase132 is a doc-only/design-only contract review after Phase131. It defines the contract status of `corridor_alignment_staging` in first-goal smoke work: whether it is the first exploration goal, a front-end staging dispatch before the first exploration goal, or a separate staging-smoke classification.

This phase does not implement, replay runtime, tune parameters, or change behavior. It defines the future smoke/artifact interpretation contract that Phase133 may use after human acceptance.

## Scope and hard guardrails

Phase132 is doc-only/design-only.

Required negative guardrails:

- No Gazebo/RViz/Nav2 runtime may be launched.
- No NavigateToPose goal may be sent.
- No maze_explorer may be started.
- No exploration/corridor/staging goal may be sent.
- No Nav2/MPPI/controller/goal checker/config tuning may be performed.
- No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change may be made.
- No autonomous exploration success or exit success may be claimed.
- No direct staging disablement is authorized.
- No Nav2 parameter tuning is authorized.
- Phase133 not entered.

Allowed work:

- Write this design proposal.
- Write a completion report.
- Add focused static tests that verify the design contract and guardrails.
- Read `maze_explorer.py` only to understand current staging field semantics.

## Inputs from Phase131

Phase131 observed this dispatch-kind sequence from existing artifacts:

- Phase124: first dispatch `goal_kind=explore`.
- Phase125: first dispatch `goal_kind=explore`.
- Phase129: first dispatch `goal_kind=corridor_alignment_staging`.

Phase131 classification:

`FIRST_DISPATCH_KIND_CHANGED_BY_POSE_TOPOLOGY_DRIFT`

Phase129 staging evidence:

- `staging_applied=true`.
- `two_step_stage_dispatch_requested=true`.
- `staging_reason=reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal`.
- `post_ingress_context_active=false`.
- `single_open_exception_applied=false`.
- an explore-like `original_target` was transformed into a near-robot staging target.

Phase127 remains unchanged:

`FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED`

That remains diagnostic-only and must not be turned into success or repaired by this design phase.

## Source semantics from read-only inspection

`maze_explorer.py` currently supports this field-level interpretation:

- `_maybe_plan_corridor_alignment_staging(...)` is only applicable when the requested `goal_kind` is `explore`; it returns `not_applicable` for non-explore goals or when two-step staging is skipped.
- When staging is applied inside `_send_goal(...)`, the requested explore target remains recorded as `original_target`, the outgoing target becomes the staging target, and the outgoing `goal_kind` is rewritten to `corridor_alignment_staging`.
- `_send_goal(...)` records `two_step_stage_dispatch_requested`, `staging_goal_pose`, `staging_reason`, `staging_executability_check`, `second_step_forward_goal`, `staging_applied`, `staging_reject_reason`, `source_forward_window`, `staging_window`, and related safety fields into the dispatch event context.
- `pending_corridor_alignment_second_step` records the pending relationship between the staging dispatch and a possible later explore second step.
- On staging success, `_handle_goal_success(...)` does not mark the branch explored; it logs that corridor-alignment staging succeeded and waits for fresh scan/local_costmap/TF before a second-step forward goal.

## Contract decision

`corridor_alignment_staging is not goal_kind=explore`.

The contract decision for future first-goal smoke interpretation is:

- `corridor_alignment_staging` is a front-end staging dispatch before a possible second-step exploration dispatch.
- A `corridor_alignment_staging` dispatch may be the literal first dispatch in a smoke artifact.
- The first exploration goal is the later second-step goal_kind=explore dispatch, not the staging dispatch.
- Therefore, the first dispatch may be a staging dispatch while the first exploration goal has not yet been attempted.
- A first-dispatch smoke and a first-exploration-goal smoke are not the same contract when staging is enabled.
- A staging dispatch must not be mixed with first explore-goal timeout replay.

Practical implication:

- Phase124/125 style first-explore contract remains valid only when the first dispatch itself is `goal_kind=explore`.
- Phase129 style first literal dispatch should be classified under staging-smoke vocabulary when `goal_kind=corridor_alignment_staging` appears before an explore second step.
- A staging accepted or succeeded outcome proves only that the staging dispatch was accepted or completed; it does not prove that the exploration branch was attempted, that the maze was explored, or that the exit was reached.

## Two-step staging artifact contract

A future staging-contract smoke or artifact replay must treat the staging path as a two-step relationship, not as a single explore-goal timeout. The artifact contract should preserve these fields explicitly and mark missing values as `missing` rather than coercing them to false/zero.

Required dispatch fields:

- `goal_sequence`
- `goal_kind`
- `dispatch_pose`
- `target`
- `original_target`
- `refined_target`
- `staging target`
- `branch_angle`
- `current_node_id`
- `start_node_id`
- `local_topology`
- `candidate_branch_count`
- `last_open_direction_count`
- `last_candidate_count`
- `goal_count_before_dispatch`
- `max_goals` or an equivalent first-dispatch/staging budget guard

Required staging mechanism fields:

- `two_step_stage_dispatch_requested`
- `staging_applied`
- `staging_reason`
- `staging_reject_reason`
- `staging_goal_pose`
- `staging_lateral_residual_before_m`
- `staging_lateral_residual_after_m`
- `staging_executability_check`
- `second_step_forward_goal`
- `pending_corridor_alignment_second_step`
- `source_forward_window`
- `staging_window`
- `front_wedge_risk`
- `local_cost_radius_m` or equivalent local-cost radius context
- `front_wedge_radius_m` and `front_wedge_half_angle_rad` or equivalent front-wedge geometry context
- `target_local_cost_max_radius` when available
- `path_corridor_min_clearance_m` when available

Required pending-second-step fields:

- `original_goal_kind`
- `original_target`
- `direction_rad`
- `start_node_id`
- `active_branch` identity or serializable branch context
- `staging_plan`
- `fresh_scan_received`
- `fresh_local_costmap_received`
- `fresh_tf_received`

Required outcome fields for a bounded smoke:

- Nav2 accepted/rejected for the staging dispatch.
- terminal result status if waiting for staging result is in scope.
- timeout flag if waiting for staging result is in scope.
- whether `pending_corridor_alignment_second_step` was created.
- whether a second-step explore goal was attempted.
- if no second step was attempted, the explicit stop reason.

## Future smoke classification vocabulary

Phase133 or later staging-contract smoke should use this vocabulary when the first literal dispatch is under review:

### FIRST_DISPATCH_EXPLORE_ACCEPTED_STOP

Use when the first literal dispatch is `goal_kind=explore`, Nav2 accepts it, and the smoke intentionally stops at dispatch acceptance.

Contract notes:

- This is the Phase124-style contract.
- It may be used as a first exploration goal dispatch smoke.
- It does not claim exit success.

### FIRST_DISPATCH_STAGING_ACCEPTED_STOP

Use when the first literal dispatch is `goal_kind=corridor_alignment_staging`, staging fields are coherent, Nav2 accepts the staging goal, and the smoke intentionally stops at staging acceptance.

Contract notes:

- FIRST_DISPATCH_STAGING_ACCEPTED_STOP is not exploration success.
- It is not exit success.
- It is not evidence that the second-step explore goal was sent.
- It should record `STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP` as a stop annotation if the bounded design intentionally stops before second step.

### FIRST_DISPATCH_STAGING_REJECTED_DIAGNOSTIC_FAIL

Use when the first literal dispatch is `goal_kind=corridor_alignment_staging`, the artifact fields are coherent, but Nav2 rejects the staging goal.

Contract notes:

- This is a staging diagnostic failure.
- It must not be reported as an exploration branch failure unless a later accepted explore goal existed.
- It must preserve staging geometry/local-cost fields for follow-up diagnosis.

### FIRST_DISPATCH_STAGING_TIMEOUT_DIAGNOSTIC_FAIL

Use when the first literal dispatch is `goal_kind=corridor_alignment_staging`, Nav2 accepts it, and a bounded staging-result wait times out.

Contract notes:

- FIRST_DISPATCH_STAGING_TIMEOUT_DIAGNOSTIC_FAIL is not FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED.
- It is a staging-goal timeout, not a first explore-goal timeout.
- It does not strengthen Phase127 unless a later separate first explore-goal artifact exists.

### STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP

Use as a primary classification or stop annotation when a bounded smoke intentionally stops after staging acceptance/success or after classifying the first staging dispatch and does not attempt the second-step explore goal.

Contract notes:

- This is a success of the bounded stop condition, not exploration success.
- It should explicitly record whether `pending_corridor_alignment_second_step` existed.
- It should explicitly record that no second explore goal was sent.

### DISPATCH_KIND_CONTRACT_AMBIGUOUS

Use when the artifact cannot tell whether the first literal dispatch is an explore goal, a staging goal, a rewritten target, or a filtered/hidden event.

Examples:

- missing `goal_kind`
- missing `original_target` vs `refined_target`
- missing `staging_applied`
- missing Nav2 acceptance/rejection status
- inconsistent first-dispatch ordering
- a staging target is present but no `pending_corridor_alignment_second_step` or equivalent relationship field exists

## Phase133 boundary

Phase133 may only be a staging-contract smoke or artifact replay.

Phase133 allowed shapes after human acceptance:

1. Artifact replay only:
   - read existing Phase129/131 artifacts and classify under the Phase132 vocabulary;
   - do not start runtime;
   - do not send any goal.

2. Bounded staging-contract smoke:
   - explicitly target the first literal dispatch contract;
   - allow only enough runtime to classify the first dispatch as explore or staging under the vocabulary;
   - stop after first classified staging or first classified explore dispatch outcome;
   - preserve max_goals or an equivalent first-dispatch/staging budget guard;
   - if the accepted design chooses acceptance-only, stop immediately after first dispatch acceptance.

Strict Phase133 limits:

- Phase133 must not run full autonomous exploration.
- Phase133 must not send a second exploration goal unless the accepted design explicitly chooses a bounded second-step contract smoke.
- Phase133 must preserve max_goals or an equivalent first-dispatch/staging budget guard.
- Phase133 must stop after the first classified staging or first classified explore dispatch outcome.
- Phase133 must not tune Nav2 or disable staging.
- Phase133 must not reinterpret staging accepted/succeeded as exploration success or exit success.

## Non-goals and forbidden interpretations

Forbidden interpretations:

- Do not count `corridor_alignment_staging` as `goal_kind=explore`.
- Do not treat staging acceptance as autonomous exploration success.
- Do not treat staging success as exit success.
- Do not use a staging timeout to overwrite Phase127 `FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED`.
- Do not use a staging accepted/succeeded artifact to claim the Phase125/127 timeout is fixed.
- Do not directly disable staging as a Phase132 outcome.
- Do not tune MPPI, controller, progress checker, or goal checker parameters as a Phase132 outcome.

## Acceptance criteria for this design phase

Phase132 is acceptable if:

- The proposal and report exist.
- Focused static tests verify the design contract and guardrails.
- No runtime runner is added.
- No protected navigation/exploration config or source behavior is changed.
- The report records final test/guard output.
- The work stops before Phase133.
