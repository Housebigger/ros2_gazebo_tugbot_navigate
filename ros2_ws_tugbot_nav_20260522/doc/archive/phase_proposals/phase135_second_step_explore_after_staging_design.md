# Phase135 second-step explore after staging design

Status: DESIGN_ONLY

## Purpose

Phase135 is a doc-only/design-only review after Phase134. Phase134 proved only that the first literal `corridor_alignment_staging` dispatch can be executed and can return `result_status_label=SUCCEEDED` under a bounded visible-stack smoke. It did not attempt the second-step exploration dispatch, did not prove autonomous exploration success, did not prove exit success, and did not repair Phase127 `FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED`.

This phase defines the future Phase136 contract for allowing exactly one bounded second-step `goal_kind=explore` dispatch after staging accepted/succeeded, and for recording and classifying that dispatch. Phase135 does not implement the runner, does not start runtime, and does not send any goal.

## Scope and hard guardrails

Phase135 is doc-only/design-only.

Allowed work:

- Write this design proposal.
- Write a completion report.
- Add focused static tests that verify the design contract and guardrails.
- Read `maze_explorer.py` only to understand `pending_corridor_alignment_second_step` semantics.

Required negative guardrails:

- No Gazebo/RViz/Nav2 runtime may be launched.
- No NavigateToPose goal may be sent.
- No maze_explorer may be started.
- No staging/explore/second-step goal may be sent.
- No Nav2/MPPI/controller/goal checker/config tuning may be performed.
- No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change may be made.
- No direct staging disablement is authorized.
- No autonomous exploration success or exit success may be claimed.
- Phase136 not entered.

## Inputs from Phase134

Phase134 final classification:

`FIRST_DISPATCH_STAGING_ACCEPTED_STOP`

Relevant Phase134 result fields:

- first literal dispatch `goal_kind=corridor_alignment_staging`.
- `staging_applied=true`.
- `two_step_stage_dispatch_requested=true`.
- `staging_reason=reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal`.
- staging dispatch accepted by Nav2.
- staging dispatch terminal result `result_status_label=SUCCEEDED`.
- `second_step_attempted=false`.
- `second_goal_dispatched=false`.
- bounded runner stopped at the staging result.

Interpretation boundary:

- Phase134 proves only that the staging dispatch was executable in that bounded smoke.
- Phase134 does not prove that an exploration branch was attempted.
- Phase134 does not prove autonomous exploration success.
- Phase134 does not prove exit success.
- Phase134 does not repair or override Phase127 `FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED`.

## Read-only source semantics

Read-only inspection of `maze_explorer.py` supports this Phase135 design without source modification:

- `_send_goal(..., goal_kind='explore')` may rewrite the outgoing literal dispatch to `goal_kind='corridor_alignment_staging'` when two-step corridor-alignment staging is applied.
- When staging is applied, `pending_corridor_alignment_second_step` is created with `original_goal_kind`, `original_target`, `direction_rad`, `start_node_id`, `active_branch`, `staging_plan`, and freshness flags initialized to false.
- `_scan_callback` marks `fresh_scan_received=true` while a pending second step exists.
- `_local_costmap_callback` marks `fresh_local_costmap_received=true` while a pending second step exists.
- `_dispatch_second_step_after_corridor_alignment_staging(robot_pose)` marks `fresh_tf_received=true` and calls `generate_second_step_forward_goal_after_staging(...)` with the fresh evidence flags.
- The second step is not dispatched unless `generated_after_fresh_evidence=true` and `selected_candidate_target` is present.
- When ready, `_dispatch_second_step_after_corridor_alignment_staging(...)` calls `_send_goal(..., 'explore', skip_two_step_staging=True)`, so the second-step dispatch contract is `goal_kind=explore` and it must not recursively create a new staging dispatch.
- `_handle_goal_success(...)` logs that corridor-alignment staging succeeded and waits for fresh scan/local_costmap/TF before second-step forward goal.

## Second-step preconditions

A future Phase136 runner may permit the second-step dispatch only after all of these preconditions are explicitly satisfied and serialized:

1. Staging outcome precondition:
   - staging accepted/succeeded;
   - first literal dispatch `goal_kind=corridor_alignment_staging`;
   - first literal dispatch had `staging_applied=true` and `two_step_stage_dispatch_requested=true`;
   - staging `result_status_label=SUCCEEDED` or equivalent Nav2 success status;
   - no prior second-step `goal_kind=explore` dispatch exists in the same bounded smoke.

2. Fresh evidence precondition:
   - fresh scan/local_costmap/TF after staging;
   - `fresh_scan_received=true` after the staging result time;
   - `fresh_local_costmap_received=true` after the staging result time;
   - `fresh_tf_received=true` at second-step evaluation;
   - freshness timestamps or sample counters must be recorded, not merely inferred.

3. Pending relationship precondition:
   - `pending_corridor_alignment_second_step present`;
   - pending payload records `original_goal_kind=explore`;
   - pending payload records `original_target`;
   - pending payload records `staging_plan` and staging target;
   - pending payload records `start_node_id` and branch context when available;
   - missing pending evidence is a fail closed condition.

4. Second-step target precondition:
   - `second_step_forward_goal valid`;
   - `generated_after_fresh_evidence=true`;
   - `selected_candidate_target` is present and finite;
   - selected yaw is finite or falls back to the stored direction;
   - the resulting outgoing dispatch must be `goal_kind=explore`;
   - the outgoing dispatch must use `skip_two_step_staging=True` or equivalent recursion guard.

If any precondition is missing, stale, or contradictory, Phase136 must fail closed with:

`STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED`

The fail closed branch must not send a second-step goal, must not send a replacement goal, and must not tune parameters or alter exploration strategy.

## Second-step artifact contract

A future Phase136 artifact must preserve the first staging dispatch, the staging result, the readiness window after staging, and the second-step dispatch/result as separate records.

Required top-level fields:

- phase name and run id;
- bounded mode name;
- guardrails summary;
- first staging dispatch summary;
- staging terminal outcome summary;
- second-step readiness summary;
- second-step dispatch summary;
- classification;
- stop reason;
- no-success-claim flags.

Required first staging dispatch fields carried forward:

- `original_target`;
- `staging target`;
- `staging_applied`;
- `two_step_stage_dispatch_requested`;
- `staging_reason`;
- `staging_executability_check`;
- `front_wedge_risk` before staging;
- `lateral residual after` from the staging plan;
- staging Nav2 accepted/rejected/result/timeout status.

Required readiness-after-staging fields:

- `pending_corridor_alignment_second_step`;
- `freshness after staging`;
- `fresh_scan_received`;
- `fresh_scan_sample_time_sec` or sample index after staging;
- `fresh_local_costmap_received`;
- `fresh_local_costmap_sample_time_sec` or sample index after staging;
- `fresh_tf_received`;
- `fresh_tf_sample_time_sec` or evaluation time;
- `front_wedge_risk_after_staging`;
- `lateral residual after`;
- local costmap availability and timestamp after staging;
- robot pose at second-step readiness evaluation;
- readiness decision and fail-closed reason if not ready.

Required second-step dispatch fields:

- `second_step_forward_goal`;
- `goal_kind=explore`;
- `selected_candidate_target`;
- selected candidate yaw;
- original target preserved from the staging plan;
- target selected from fresh evidence;
- local/global cost context used for readiness;
- `second_step_goal_count=1`;
- `third_goal_dispatched=false`;
- `skip_two_step_staging=true` or equivalent recursion guard;
- `accepted`;
- `rejected`;
- `timeout`;
- `result_status_label`;
- `abort_text`;
- Nav2 feedback summary while waiting, if result wait is in scope.

Required stop flags:

- one and only one second-step `goal_kind=explore` may be sent;
- no third goal may be sent;
- no full autonomous exploration may continue after second-step accepted/rejected/timeout/result;
- no autonomous exploration success claim;
- no exit success claim;
- no Phase127 timeout-fixed claim.

## Phase136 allowed scope

Phase136 allowed scope, after human acceptance of Phase135, is a bounded second-step smoke only:

- It may start the visible stack only if explicitly approved in Phase136.
- It may reuse the Phase134 staging prelude only to produce one bounded staging succeeded context.
- It may proceed only after a bounded staging result succeeded.
- It may wait for fresh scan/local_costmap/TF evidence after staging.
- It may send exactly one second-step goal_kind=explore when the preconditions pass.
- It must stop after accepted/rejected/timeout/result for that one second-step goal.
- It must preserve the first staging dispatch evidence and the second-step evidence separately.
- It must not send a third goal.
- It must not run full autonomous exploration.
- It must not tune Nav2 parameters.
- It must not change exploration strategy.
- It must not disable staging.
- It must not alter branch scoring, centerline, fallback, or terminal acceptance.
- It must not claim autonomous exploration success.
- It must not claim exit success.

The phrase "stop after accepted/rejected/timeout/result" means the accepted Phase136 runner must choose and document a bounded wait policy before runtime starts. It may stop at acceptance or wait for one bounded terminal result, but it must stop immediately after the selected boundary is observed.

## Classification vocabulary for Phase136

### STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_ACCEPTED_STOP

Use when staging accepted/succeeded, second-step readiness passed, exactly one second-step `goal_kind=explore` dispatch was sent, Nav2 accepted it, and the bounded smoke intentionally stopped at acceptance.

Contract notes:

- second-step accepted is not autonomous exploration success.
- It is not exit success.
- It does not imply the branch was completed.

### STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_REJECTED_DIAGNOSTIC_FAIL

Use when staging accepted/succeeded, readiness passed, exactly one second-step `goal_kind=explore` dispatch was sent, and Nav2 rejected the second-step goal.

Contract notes:

- Preserve the second-step target, readiness, cost, and branch geometry evidence.
- Do not send a replacement goal.
- Do not tune Nav2 parameters in the same phase.

### STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_TIMEOUT_DIAGNOSTIC_FAIL

Use when staging accepted/succeeded, readiness passed, exactly one second-step `goal_kind=explore` dispatch was accepted, the bounded result wait timed out, and the runner stopped after timeout handling.

Contract notes:

- This is a second-step explore timeout diagnostic failure.
- It is not Phase127 `FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED` unless a separate accepted design explicitly compares artifacts later.
- second-step result must not be used to claim Phase127 timeout is fixed.

### STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_RESULT_SUCCEEDED_STOP

Use when staging accepted/succeeded, readiness passed, exactly one second-step `goal_kind=explore` dispatch was sent, and the bounded result wait observed Nav2 success.

Contract notes:

- second-step succeeded is not exit success.
- It is only the result of the one bounded second-step exploration goal.
- It is not autonomous exploration success because Phase136 must stop before any third goal or full exploration continuation.

### STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED

Use when staging accepted/succeeded, but Phase136 cannot prove readiness for the second-step explore dispatch.

Fail-closed examples:

- no `pending_corridor_alignment_second_step present` evidence;
- no valid `second_step_forward_goal`;
- no `generated_after_fresh_evidence=true`;
- missing fresh scan/local_costmap/TF after staging;
- missing or invalid selected candidate target;
- artifact cannot prove the second-step would be `goal_kind=explore`;
- recursion guard for staging is absent.

This classification must not send a second-step goal.

### SECOND_STEP_CONTRACT_AMBIGUOUS

Use when the artifact cannot distinguish staging result, pending relationship, readiness, second-step target, dispatch kind, or second-step outcome.

Examples:

- first dispatch order is ambiguous;
- staging result is missing or not terminal;
- second dispatch exists but its `goal_kind` is missing;
- more than one post-staging dispatch exists;
- a third goal appears;
- acceptance/result fields are contradictory;
- success wording appears without explicit no-success-claim boundaries.

## Forbidden interpretations

The following interpretations are forbidden in Phase135 and must remain forbidden in Phase136:

- Do not treat staging accepted/succeeded as exploration success.
- Do not treat second-step accepted as autonomous exploration success; second-step accepted is not autonomous exploration success.
- Do not treat second-step succeeded as exit success; second-step succeeded is not exit success.
- Do not use second-step accepted/succeeded to claim the maze has been autonomously explored.
- Do not use a second-step timeout or success to overwrite Phase127.
- Do not claim Phase127 timeout is fixed; second-step result must not be used to claim Phase127 timeout is fixed.
- Do not send a third goal.
- Do not run full autonomous exploration.
- Do not tune MPPI, controller, progress checker, or goal checker parameters.
- Do not alter exploration strategy, branch scoring, centerline, fallback, or terminal acceptance.
- Do not disable corridor-alignment staging.

## Completion boundary

Phase135 stops at this design review. Phase136 requires explicit human acceptance and a new phase authorization before any runtime, implementation, or second-step dispatch is attempted.
