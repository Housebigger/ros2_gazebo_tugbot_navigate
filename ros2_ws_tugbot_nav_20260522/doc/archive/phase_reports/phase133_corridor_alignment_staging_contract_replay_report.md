# Phase133 corridor alignment staging contract replay report

Status: PHASE133_CORRIDOR_ALIGNMENT_STAGING_CONTRACT_REPLAY_COMPLETE_STOP_BEFORE_PHASE134

## Summary

Phase133 completed an offline artifact replay over existing Phase129 and Phase131 artifacts plus the existing Phase129 `maze_explorer` stderr log. It applied the Phase132 corridor-alignment staging contract to the Phase129 first literal dispatch.

Final classification:

`STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP`

The Phase129 first literal dispatch is `goal_kind=corridor_alignment_staging`. Under the Phase132 contract it is not the first exploration goal. It is a front-end staging dispatch before a possible second-step `goal_kind=explore` dispatch.

The artifact contains no later actual `goal_kind=explore` dispatch after the first staging dispatch. `second_goal_dispatched=false`. Therefore the first exploration goal was not attempted in this bounded replay artifact.

## Scope actually performed

Allowed work performed:

- Added offline analyzer: `tools/analyze_phase133_corridor_alignment_staging_contract_replay.py`.
- Added focused analyzer/static tests: `src/tugbot_maze/test/test_phase133_corridor_alignment_staging_contract_replay.py`.
- Read existing Phase129 artifacts and logs.
- Read existing Phase131 analyzer output.
- Read `maze_explorer.py` source only to preserve staging field semantics.
- Wrote analyzer outputs under `log/phase133_corridor_alignment_staging_contract_replay/`.
- Wrote this report.

Guardrails preserved:

- No Gazebo/RViz/Nav2 runtime was launched.
- No NavigateToPose goal was sent.
- No `maze_explorer` was started.
- No exploration/corridor/staging goal was sent.
- No Nav2/MPPI/controller/goal checker/config tuning was performed.
- No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change was made.
- No autonomous exploration success or exit success was claimed.
- Phase134 not entered.

## Inputs

Primary artifacts:

- `log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_instrumented_first_goal_timeout_diagnosis_rerun.json`
- `log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_instrumented_first_goal_timeout_diagnosis_rerun_maze_explorer_stderr.log`
- `log/phase131_first_dispatch_kind_artifact_replay/phase131_first_dispatch_kind_artifact_replay_analysis.json`

Contract reference:

- Phase132: `doc/doc_proposal/phase132_corridor_alignment_staging_contract_design.md`

## Replay results

Analyzer output:

- JSON: `log/phase133_corridor_alignment_staging_contract_replay/phase133_corridor_alignment_staging_contract_replay_analysis.json`
- Markdown summary: `log/phase133_corridor_alignment_staging_contract_replay/phase133_corridor_alignment_staging_contract_replay_summary.md`

Analyzer stdout:

```text
{"analysis": "log/phase133_corridor_alignment_staging_contract_replay/phase133_corridor_alignment_staging_contract_replay_analysis.json", "classification": "STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP", "summary": "log/phase133_corridor_alignment_staging_contract_replay/phase133_corridor_alignment_staging_contract_replay_summary.md", "valid": true}
```

## Evidence table

| Question | Phase133 answer |
| --- | --- |
| Phase129 first literal dispatch kind | `corridor_alignment_staging` |
| Is it the first exploration goal? | No |
| Was an actual later `goal_kind=explore` dispatch observed? | No |
| `second_goal_dispatched` | `false` |
| `staging_applied` | `true` |
| `two_step_stage_dispatch_requested` | `true` |
| `staging_reason` | `reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal` |
| `original_target` | `[1.7051882914469514, 1.0242420478499465]` |
| staging/refined target | `[1.7005222087522947, 0.07425350703890773]` |
| `pending_corridor_alignment_second_step` serialized evidence | missing |
| `second_step_forward_goal` evidence | missing |
| Phase129 legacy result label | `REJECTED_NON_EXPLORE_GOAL_KIND` |

Important interpretation:

The Phase129 legacy `REJECTED_NON_EXPLORE_GOAL_KIND` label came from the old first-explore replay contract guard. Phase133 does not treat that label as Nav2 rejection of a staging goal. It also does not use it to classify `FIRST_DISPATCH_STAGING_REJECTED_DIAGNOSTIC_FAIL`.

## Classification reasons

- Phase129 first literal dispatch is `corridor_alignment_staging`.
- Staging fields are coherent: original target, staging target, `staging_applied`, two-step request, and `staging_reason` are present.
- Existing stderr contains the send line for the staging dispatch: `maze explorer sending corridor_alignment_staging goal #1 seq=1`.
- The legacy Phase129 `REJECTED_NON_EXPLORE_GOAL_KIND` result is a first-explore replay guard, not Nav2 rejection of staging.
- No actual later `goal_kind=explore` dispatch appears after the first staging dispatch.
- `pending_corridor_alignment_second_step` is not serialized in the Phase129 artifact and remains explicit missing evidence.
- `second_step_forward_goal` is missing in the Phase129 dispatch event.

Therefore the correct Phase132 vocabulary classification for the existing Phase129 artifact is:

`STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP`

## Non-claims

Phase133 does not claim:

- exploration success;
- exit success;
- autonomous exploration success;
- first exploration goal acceptance;
- first exploration goal timeout;
- Nav2 staging rejection;
- staging timeout;
- Phase127 timeout-local-cost confirmation.

Explicit contract phrases:

- staging accepted/succeeded is not exploration success.
- staging accepted/succeeded is not exit success.
- the Phase129 first literal staging dispatch is not Phase127 FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED.

The Phase129 first literal staging dispatch is not Phase127 `FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED`. Phase133 does not mix the staging contract replay into Phase127 first explore-goal timeout-local-cost replay.

## TDD evidence

Focused tests were written before the analyzer and verified RED.

Initial RED command:

```bash
python3 -m pytest src/tugbot_maze/test/test_phase133_corridor_alignment_staging_contract_replay.py -q
```

Initial RED output:

```text
8 failed in 0.06s
```

Expected failure reason: the Phase133 analyzer and report did not exist yet.

## Final verification

Focused tests after report update:

```text
8 passed in 0.04s
```

Phase131/132/133 static bundle after report update:

```text
22 passed in 0.11s
```

Analyzer syntax check:

```text
py_compile ok
```

Analyzer rerun:

```text
{"analysis": "log/phase133_corridor_alignment_staging_contract_replay/phase133_corridor_alignment_staging_contract_replay_analysis.json", "classification": "STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP", "summary": "log/phase133_corridor_alignment_staging_contract_replay/phase133_corridor_alignment_staging_contract_replay_summary.md", "valid": true}
```

Final guard artifact:

- `log/phase133_corridor_alignment_staging_contract_replay/phase133_final_guard.txt`
- process guard: empty
- protected config diff guard: empty
- runtime runner guard: no runtime runner present
- forbidden runtime token guard: forbidden runtime tokens absent
- pycache guard: clean

## Phase134 boundary

If Phase134 is accepted and involves runtime, it may only be a bounded staging-contract smoke. It must be not full autonomous exploration. It must not run full autonomous exploration. It must not tune Nav2 parameters. It must not directly disable staging. It must not interpret staging accepted/succeeded as exploration success or exit success.

## Stop condition

Phase133 is complete and stops here for human acceptance. Phase134 not entered.
