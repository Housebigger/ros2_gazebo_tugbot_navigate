# Phase132 corridor alignment staging contract design report

Status: PHASE132_CORRIDOR_ALIGNMENT_STAGING_CONTRACT_DESIGN_COMPLETE_STOP_BEFORE_PHASE133

## Summary

Phase132 completed a Corridor alignment staging contract design review.

This phase was DESIGN_ONLY. It did not implement, run, tune, or repair navigation behavior. It defines how future first-dispatch and first-exploration-goal smoke work should interpret `corridor_alignment_staging` after Phase131 showed that Phase124/125 first dispatch was `goal_kind=explore` while Phase129 first dispatch was `goal_kind=corridor_alignment_staging`.

Final design decision:

- corridor_alignment_staging is not goal_kind=explore.
- It is a front-end staging dispatch before a possible second-step exploration dispatch.
- The first exploration goal is the later second-step `goal_kind=explore` dispatch, if and only if that second step is attempted.
- A first literal dispatch may be staging; that should be classified under staging-smoke vocabulary, not mixed into first explore-goal timeout replay.
- staging accepted/succeeded is not exploration success.
- staging accepted/succeeded is not exit success.

Phase127 FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED remains diagnostic-only. Phase132 does not strengthen it, repair it, overwrite it, or convert it into success.

## Scope actually performed

Allowed work performed:

- Wrote `doc/doc_proposal/phase132_corridor_alignment_staging_contract_design.md`.
- Wrote this report: `doc/doc_report/phase132_corridor_alignment_staging_contract_design_report.md`.
- Added focused static tests: `src/tugbot_maze/test/test_phase132_corridor_alignment_staging_contract_design.py`.
- Read `maze_explorer.py` source only to understand staging semantics.
- Read Phase130/131 documents and the Phase131 reusable reference.

Guardrails preserved:

- No Gazebo/RViz/Nav2 runtime was launched.
- No NavigateToPose goal was sent.
- No maze_explorer was started.
- No exploration/corridor/staging goal was sent.
- No Nav2/MPPI/controller/goal checker/config tuning was performed.
- No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change was made.
- No autonomous exploration success or exit success was claimed.
- No direct staging disablement was performed or authorized.
- No Nav2 parameter tuning was performed or authorized.
- Phase133 not entered.

## Source semantics reviewed read-only

`maze_explorer.py` shows this current staging relationship:

- `_maybe_plan_corridor_alignment_staging(...)` is applicable only for requested `goal_kind='explore'` and returns not-applicable for non-explore goals or when two-step staging is skipped.
- `_send_goal(...)` preserves the requested explore target as `original_target` while rewriting the outgoing dispatch target to the staging pose when `staging_applied=true`.
- When staging is applied, outgoing `goal_kind` is changed to `corridor_alignment_staging`.
- `pending_corridor_alignment_second_step` records `original_goal_kind`, `original_target`, `direction_rad`, `start_node_id`, `active_branch`, the `staging_plan`, and freshness flags for scan/local_costmap/TF.
- Dispatch context records `two_step_stage_dispatch_requested`, `staging_goal_pose`, `staging_reason`, `staging_executability_check`, `second_step_forward_goal`, `staging_applied`, `staging_reject_reason`, `source_forward_window`, `staging_window`, and safety/local-cost evidence.
- On staging success, the code logs that corridor-alignment staging succeeded and waits for fresh scan/local_costmap/TF before second-step forward goal; it does not mark the branch explored merely because staging succeeded.

This supports the design contract that staging is a precursor to an exploration dispatch, not the exploration dispatch itself.

## Contract definitions

### Relationship to goal_kind=explore

`corridor_alignment_staging` is a distinct dispatch kind. It is not `goal_kind=explore`.

It may originate from a requested explore branch context, but after two-step staging is applied, the literal outgoing dispatch is a staging dispatch. Therefore a future smoke must distinguish:

- first literal dispatch: the first goal actually sent to Nav2;
- first exploration goal: the first literal dispatch whose `goal_kind=explore`;
- staging dispatch: a first literal dispatch whose `goal_kind=corridor_alignment_staging` and whose artifact links back to `original_target` plus pending second-step context.

### Two-step staging artifact contract

Future smoke or artifact replay should require and preserve:

- `original_target`: the requested explore target before staging rewrite.
- `refined_target`: the actual outgoing target after staging or centerline refinement.
- staging target: the near-robot alignment target sent under `goal_kind=corridor_alignment_staging`.
- `pending_corridor_alignment_second_step`: the relationship record linking staging to the possible later explore second step.
- `second_step_forward_goal`: candidate second-step goal after fresh scan/local_costmap/TF if available.
- `staging_reason`: why staging was selected.
- `staging_reject_reason`: why staging was not applied or was rejected, if applicable.
- `staging_lateral_residual_before_m` and `staging_lateral_residual_after_m`: alignment effect evidence.
- `front_wedge_risk`: front-wedge cost/safety risk evidence.
- `staging_executability_check`: explicit safety/executability check.
- `source_forward_window` and `staging_window`: local/global/cost evidence windows if present.
- `two_step_stage_dispatch_requested` and `staging_applied`: boolean staging mechanism fields.

Missing values must remain explicit as missing; they must not be silently coerced to false or zero.

## Future smoke classification vocabulary

Phase132 defines these future smoke classifications:

- FIRST_DISPATCH_EXPLORE_ACCEPTED_STOP
- FIRST_DISPATCH_STAGING_ACCEPTED_STOP
- FIRST_DISPATCH_STAGING_REJECTED_DIAGNOSTIC_FAIL
- FIRST_DISPATCH_STAGING_TIMEOUT_DIAGNOSTIC_FAIL
- STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP
- DISPATCH_KIND_CONTRACT_AMBIGUOUS

Interpretation:

- FIRST_DISPATCH_EXPLORE_ACCEPTED_STOP: first literal dispatch is `goal_kind=explore`, Nav2 accepts, and smoke stops at acceptance.
- FIRST_DISPATCH_STAGING_ACCEPTED_STOP: first literal dispatch is `goal_kind=corridor_alignment_staging`, staging artifact fields are coherent, Nav2 accepts, and smoke stops at staging acceptance. This is not exploration success or exit success.
- FIRST_DISPATCH_STAGING_REJECTED_DIAGNOSTIC_FAIL: first literal dispatch is staging, but Nav2 rejects it. This is a staging diagnostic failure, not an exploration branch failure.
- FIRST_DISPATCH_STAGING_TIMEOUT_DIAGNOSTIC_FAIL: first literal dispatch is staging, accepted by Nav2, but a bounded staging-result wait times out. This is not Phase127 FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED and must not be mixed with first explore-goal timeout replay.
- STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP: a bounded smoke intentionally stops after staging acceptance/success or after first staging classification without attempting the explore second step. This is a bounded stop condition, not exploration success.
- DISPATCH_KIND_CONTRACT_AMBIGUOUS: required fields are missing/inconsistent enough that first dispatch kind or staging relationship cannot be classified.

## Phase133 boundary

Phase133 may only be a staging-contract smoke or artifact replay after human acceptance.

Allowed Phase133 shapes:

1. Artifact replay only:
   - read existing Phase129/131 artifacts;
   - classify using the Phase132 vocabulary;
   - no runtime and no goals.

2. Bounded staging-contract smoke:
   - classify the first literal dispatch as explore or staging;
   - stop after first classified staging or first classified explore dispatch outcome;
   - preserve `max_goals` or an equivalent first-dispatch/staging budget guard;
   - do not run full autonomous exploration.

Phase133 constraints:

- Phase133 must not run full autonomous exploration.
- Phase133 must not send a second exploration goal unless the accepted design explicitly chooses a bounded second-step contract smoke.
- Phase133 must preserve `max_goals` or an equivalent first-dispatch/staging budget guard.
- Phase133 must stop after first classified staging or first classified explore dispatch outcome.
- Phase133 must not directly disable staging.
- Phase133 must not tune Nav2 parameters.
- Phase133 must not interpret staging accepted/succeeded as exploration success or exit success.

## TDD evidence

Focused tests were written before the proposal/report and verified RED.

Initial RED command:

```bash
python3 -m pytest src/tugbot_maze/test/test_phase132_corridor_alignment_staging_contract_design.py -q
```

Initial RED output:

```text
6 failed, 1 passed in 0.03s
```

Expected failure reason: Phase132 proposal/report did not exist yet. The one passing test verified existing read-only source semantics in `maze_explorer.py`.

## Final verification

Focused tests:

```text
7 passed in 0.01s
```

Phase130/131/132 static bundle:

```text
22 passed in 0.09s
```

Doc path guard:

```text
phase132 docs present
```

Final guard artifact:

- `log/phase132_corridor_alignment_staging_contract_design/phase132_final_guard.txt`
- process guard: empty
- protected config diff guard: empty
- unexpected Phase132 runtime runner guard: `no runtime runner present`
- doc forbidden success/tuning claim guard: `forbidden success/tuning claims absent`
- pycache guard: clean after cleanup

## Stop condition

Phase132 is complete and stops here for human acceptance. Phase133 not entered.
