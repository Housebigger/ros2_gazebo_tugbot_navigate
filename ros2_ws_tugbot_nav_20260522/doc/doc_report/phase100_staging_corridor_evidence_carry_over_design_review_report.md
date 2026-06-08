# Phase100 Staging corridor evidence carry-over design review report

Status: FINAL_DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_PHASE101_IMPLEMENTATION

Phase100 is design review only. No runtime implementation was performed. Phase101 not entered.

## Goal

Review and document a future design for Phase88 corridor evidence carry-over / evidence reuse for Phase92 staging eligibility, based on the Phase99 diagnosis that Goal1 staging candidates were too short / too near the dispatch-entrance window to observe two-side-wall evidence by themselves.

## Required reading completed

- `doc/doc_report/phase99_goal1_staging_evidence_path_diagnosis_report.md`
- `doc/doc_report/phase98_goal1_recovery_dominant_failure_root_cause_diagnosis_report.md`
- `doc/doc_report/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_report.md`
- `doc/doc_report/phase92_two_step_corridor_alignment_staging_goal_minimal_implementation_report.md`
- `doc/doc_report/phase88_safety_first_multi_candidate_forward_search_minimal_implementation_report.md`
- `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`
- top-level `doc/doc_proposal` design docs, including Phase91, Phase87, Phase83, Phase76, Phase27-alt-R5, and Phase27-alt near-exit fallback.

## Added Phase100 files

- `doc/doc_proposal/phase100_staging_corridor_evidence_carry_over_design_review.md`
- `doc/doc_report/phase100_staging_corridor_evidence_carry_over_design_review_report.md`
- `src/tugbot_maze/test/test_phase100_staging_corridor_evidence_carry_over_design_review.py`

## TDD/static test evidence

Focused static tests were written before the design/report existed.

Observed RED:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase100_staging_corridor_evidence_carry_over_design_review.py
FFFFF [100%]
5 failed in 0.05s
```

The failures were expected and specific: the Phase100 proposal/report files did not exist yet.

Observed GREEN/final verification:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase100_staging_corridor_evidence_carry_over_design_review.py
.....                                                                    [100%]
5 passed in 0.01s

PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile src/tugbot_maze/test/test_phase100_staging_corridor_evidence_carry_over_design_review.py
exit 0

nav2_config_diff=0
runtime_logic_diff=0
```

Final `__pycache__` cleanup was run and the final find output was empty before the Phase100 response.

## Design decision

Decision token:

```text
DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_PHASE101_IMPLEMENTATION
```

Phase100 recommends a narrow future design:

- Reuse only Phase88 corridor-level evidence for Phase92 staging eligibility when Phase92 staging window is too short/near-dispatch to see two-side-wall evidence by itself.
- Do not carry over safety or execution evidence.
- Require staging itself to recompute hard-safety / occupancy / clearance / footprint / front-wedge / fresh-evidence checks.
- Preserve branch scoring, exploration order, centerline gate, directional readiness, fallback/terminal acceptance, and Nav2 configuration.

## Allowed carry-over evidence

Only these corridor-level items are eligible for future carry-over:

- `same_corridor`
- `two_side_wall_evidence`
- `corridor_heading`
- `wall clearance context`
- `source_forward_window`

These are treated as corridor context only. They are not proof that a staging pose is safe.

## Forbidden carry-over evidence

These must never be carried over:

- `hard_safety_pass`
- `safety_floor_ok`
- `occupancy_free`
- `target_has_clearance`
- `footprint_lethal_not_increased`
- `front_wedge_lethal_not_increased`
- `fresh scan/local_costmap/TF`

Carry-over must never set or imply `hard_safety_pass=true`.

## Recomputed staging evidence requirement

Even if corridor-level carry-over applies, staging itself must recompute:

- hard safety
- safety floor
- occupancy
- target clearance
- footprint lethal regression
- front-wedge lethal regression
- local-cost target/path/footprint/front-wedge samples
- map occupancy / unknown policy checks
- scan consistency where available
- TF and local-costmap timestamp age
- bounded short-distance and forward/non-negative progress
- branch consistency

If recomputed staging evidence fails, staging must reject.

## Trigger design

Future carry-over evaluation may be considered only when:

1. Phase92 trigger bundle is satisfied.
2. The Phase92 staging window is missing two-side-wall evidence.
3. Phase88 forward window has same-corridor/two-side-wall evidence.
4. Robot/staging/forward target geometry is coherent with one selected corridor.
5. Source evidence is auditable: event sequence, frame, source window bounds, heading, and source corridor booleans are recorded.

## Reject design

Future carry-over must reject with explicit reason when any of these are true:

- `carry_over_source_stale`
- `frame_mismatch`
- `heading_mismatch`
- `forward_window_not_trustworthy`
- `staging_not_consistent_with_source_corridor`
- `source_missing_same_corridor_or_two_side_wall`
- `source_branch_or_goal_sequence_mismatch`
- `staging_safety_recompute_failed`
- `insufficient_carry_over_evidence`

Missing evidence must not be fabricated.

## Future goal_events fields

The design requires future diagnostics for:

- `corridor_evidence_carry_over`
- `carry_over_source`
- `carry_over_applied`
- `carry_over_reject_reason`
- `source_forward_window`
- `staging_window`
- `safety_evidence_recomputed=true`
- `branch_scoring_changed=false`
- `fallback_terminal_acceptance_used=false`

Important semantic distinction:

- `carry_over_applied=true` means only corridor-level evidence was reused.
- `staging_applied=true` still requires recomputed hard-safety success.

## Relationship to Phase99

Phase99 showed:

- Phase92 staging candidates were near dispatch/entrance: y≈0.024..0.124, max distance≈0.180m.
- Phase92 staging had `two_side_wall_true_count=0` and rejected with `missing_two_side_wall_evidence`.
- Phase88 forward window was y≈0.921..1.127, with all 63 candidates showing `same_corridor=true` and `two_side_wall_evidence=true`.
- The window gap was about 0.796m.

Phase100 does not reinterpret this as a request to loosen safety. It treats it as a corridor evidence path design gap.

## Guardrails

- No runtime code changed.
- runtime files changed: false
- No Phase88/92 logic changed.
- No maze_explorer strategy changed.
- No branch scoring changed.
- No exploration order changed.
- No centerline gate changed.
- No directional readiness changed.
- No fallback/terminal acceptance changed.
- No Nav2/MPPI/controller tuning.
- No inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No autonomous exploration success claim.
- No exit success claim.
- No simulation/runtime/long exploration run started.
- Phase101 not entered.

## Verification commands

Nav2 config diff guard and runtime logic diff guard are required for Phase100 because this phase is design-only.

Final verification to run after this report:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase100_staging_corridor_evidence_carry_over_design_review.py
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile src/tugbot_maze/test/test_phase100_staging_corridor_evidence_carry_over_design_review.py
git diff -- src/tugbot_navigation/config | wc -l
git diff -- src/tugbot_maze/tugbot_maze src/tugbot_bringup/launch src/tugbot_navigation/config | wc -l
find tools src/tugbot_maze/test -type d -name __pycache__ -prune -exec rm -rf {} +
```

## Stop condition

Phase100 stops after design document, report, focused static tests, guard checks, and reusable lesson update. Phase101 not entered.
