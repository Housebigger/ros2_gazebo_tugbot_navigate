# Phase135 second-step explore after staging design report

Status: PHASE135_SECOND_STEP_EXPLORE_AFTER_STAGING_DESIGN_COMPLETE_STOP_BEFORE_PHASE136

## Summary

Phase135 completed the Second-step explore dispatch after staging design review after Phase134. This phase is DESIGN_ONLY.

Phase134 established only that a first literal `goal_kind=corridor_alignment_staging` dispatch could be accepted and could return `result_status_label=SUCCEEDED` under a bounded staging-contract smoke. Phase134 intentionally stopped with `second_step_attempted=false` and `second_goal_dispatched=false`.

Phase135 defines how a future Phase136 may, after explicit human acceptance, allow exactly one bounded second-step `goal_kind=explore` dispatch after staging succeeded. It also defines the required artifact fields and classification vocabulary. Phase135 does not implement Phase136, does not start runtime, and does not send any goal.

## Deliverables

- Proposal: `doc/doc_proposal/phase135_second_step_explore_after_staging_design.md`
- Report: `doc/doc_report/phase135_second_step_explore_after_staging_design_report.md`
- Focused static tests: `src/tugbot_maze/test/test_phase135_second_step_explore_after_staging_design.py`

## Scope actually performed

Allowed work performed:

- Read Phase134 report and Phase132/133 staging-contract references.
- Read `maze_explorer.py` only to understand `pending_corridor_alignment_second_step` semantics.
- Added focused static tests.
- Wrote Phase135 proposal and this report.

Guardrails preserved:

- No Gazebo/RViz/Nav2 runtime was launched.
- No NavigateToPose goal was sent.
- No maze_explorer was started.
- No staging/explore/second-step goal was sent.
- No Nav2/MPPI/controller/goal checker/config tuning was performed.
- No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change was made.
- No direct staging disablement was performed.
- No autonomous exploration success or exit success is claimed.
- Phase136 not entered.

## Design outcome

Phase135 defines these required Phase136 preconditions before any second-step explore dispatch:

1. Staging accepted/succeeded and first literal dispatch is `goal_kind=corridor_alignment_staging`.
2. Fresh scan/local_costmap/TF evidence exists after staging.
3. `pending_corridor_alignment_second_step present` evidence exists and preserves the original explore target relationship.
4. `second_step_forward_goal valid` evidence exists.
5. `generated_after_fresh_evidence=true` and `selected_candidate_target` are present.
6. The outgoing second-step dispatch is explicitly `goal_kind=explore` and uses a recursion guard equivalent to `skip_two_step_staging=True`.

If any required precondition is missing or contradictory, Phase136 must fail closed before dispatch with:

`STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED`

## Phase136 boundary defined by this design

Future Phase136, if accepted, may only be a bounded second-step smoke:

- only after a bounded staging result succeeded;
- may send exactly one second-step `goal_kind=explore`;
- must stop after accepted/rejected/timeout/result according to a predeclared bounded wait policy;
- must preserve first staging evidence and second-step evidence separately;
- must not send a third goal;
- must not run full autonomous exploration;
- must not tune Nav2 parameters;
- must not change exploration strategy;
- must not disable staging;
- must not claim autonomous exploration success or exit success.

## Classification vocabulary

Phase135 defines the Phase136 classification vocabulary:

- `STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_ACCEPTED_STOP`
- `STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_REJECTED_DIAGNOSTIC_FAIL`
- `STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_TIMEOUT_DIAGNOSTIC_FAIL`
- `STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_RESULT_SUCCEEDED_STOP`
- `STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED`
- `SECOND_STEP_CONTRACT_AMBIGUOUS`

Boundary phrases preserved:

- second-step accepted is not autonomous exploration success.
- second-step succeeded is not exit success.
- second-step result must not be used to claim Phase127 timeout is fixed.

## TDD evidence

Focused tests were written before the design docs and verified RED.

Initial RED command:

```text
python3 -m pytest src/tugbot_maze/test/test_phase135_second_step_explore_after_staging_design.py -q
```

Initial RED output:

```text
7 failed, 1 passed in 0.03s
```

Expected RED reason: the Phase135 proposal/report documents did not exist yet.

## Static verification

Final focused tests:

```text
python3 -m pytest src/tugbot_maze/test/test_phase135_second_step_explore_after_staging_design.py -q
# 8 passed in 0.01s
```

Phase132-135 static design/staging bundle:

```text
python3 -m pytest \
  src/tugbot_maze/test/test_phase132_corridor_alignment_staging_contract_design.py \
  src/tugbot_maze/test/test_phase133_corridor_alignment_staging_contract_replay.py \
  src/tugbot_maze/test/test_phase134_bounded_corridor_alignment_staging_smoke.py \
  src/tugbot_maze/test/test_phase135_second_step_explore_after_staging_design.py -q
# 30 passed in 0.06s
```

No-runtime/protected guards recorded:

- no matching Gazebo/RViz/Nav2/SLAM/maze_explorer process;
- no Phase135/Phase136 runtime tool was added;
- protected Nav2/config diff guard was empty;
- forbidden runtime/success tokens were absent.

Guard artifact:

- `log/phase135_second_step_explore_after_staging_design/phase135_static_guard.txt`

## Non-claims

Phase135 does not claim:

- autonomous exploration success;
- exit success;
- second-step explore dispatch was sent;
- first exploration goal success;
- Phase127 timeout was fixed;
- Nav2/MPPI/controller/goal checker/config tuning was performed;
- exploration strategy changed.

## Stop condition

Phase135 is complete when the focused tests and no-runtime/protected guards pass. Stop here for human acceptance. Phase136 not entered.
