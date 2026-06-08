# Phase101 Staging corridor evidence carry-over minimal implementation

Status: IMPLEMENTED_STATIC_UNIT_VALIDATED_STOP_BEFORE_PHASE102

## Scope

Phase101 minimally implements the Phase100-approved corridor-level evidence carry-over path from Phase88 forward-window evidence into Phase92 staging eligibility.

The implementation is intentionally narrow:

- It only addresses the Phase99 evidence-path gap where the Phase92 staging window is too short / too near the entrance to observe two-side-wall evidence.
- It only carries over corridor-level context.
- It does not carry over safety or execution evidence.
- It does not lower any hard-safety boundary.
- It does not tune Nav2, MPPI, controller, inflation, robot_radius, clearance_radius_m, map thresholds, or local costmap thresholds.
- It does not claim autonomous exploration success or exit success.
- It does not run long exploration.

## Required reading completed

- `doc/doc_proposal/phase100_staging_corridor_evidence_carry_over_design_review.md`
- `doc/doc_report/phase100_staging_corridor_evidence_carry_over_design_review_report.md`
- `doc/doc_report/phase99_goal1_staging_evidence_path_diagnosis_report.md`
- `doc/doc_report/phase98_goal1_recovery_dominant_failure_root_cause_diagnosis_report.md`
- `doc/doc_report/phase92_two_step_corridor_alignment_staging_goal_minimal_implementation_report.md`
- `doc/doc_report/phase88_safety_first_multi_candidate_forward_search_minimal_implementation_report.md`

## Files changed / added

Changed runtime logic:

- `src/tugbot_maze/tugbot_maze/maze_perception.py`
- `src/tugbot_maze/tugbot_maze/maze_explorer.py`

Added focused tests:

- `src/tugbot_maze/test/test_phase101_staging_corridor_evidence_carry_over_minimal_implementation.py`

Added report:

- `doc/doc_report/phase101_staging_corridor_evidence_carry_over_minimal_implementation_report.md`

## Implementation summary

The Phase92 helper `plan_two_step_corridor_alignment_staging_goal(...)` now accepts optional carry-over audit inputs:

- `current_goal_sequence`
- `current_branch_context_id`
- `current_timestamp_sec`
- `staging_frame_id`
- `carry_over_source_max_age_sec`
- `carry_over_heading_tolerance_rad`

After the staging candidate family is generated, the helper computes:

- `source_forward_window`
- `staging_window`
- `corridor_evidence_carry_over`
- `carry_over_source`
- `carry_over_applied`
- `carry_over_reject_reason`
- `safety_evidence_recomputed=true`

Carry-over is evaluated only when the Phase92 staging window has candidates and its local two-side-wall count is zero.

If carry-over is eligible, only these corridor-level fields may influence staging eligibility:

- `same_corridor`
- `two_side_wall_evidence`
- `corridor_heading`
- `wall_clearance_context`
- `source_forward_window`

Forbidden evidence remains forbidden and is never carried over:

- `hard_safety_pass`
- `safety_floor_ok`
- `occupancy_free`
- `target_has_clearance`
- `footprint_lethal_not_increased`
- `front_wedge_lethal_not_increased`
- `fresh_scan_local_costmap_tf`

## Safety recomputation rule

`carry_over_applied=true does not mean staging_applied=true`.

Even when corridor evidence is carried over, the staging candidate must still recompute and pass its own safety/execution checks:

- `hard_safety_pass`
- `occupancy_free`
- `target_has_clearance`
- `safety_floor_ok`
- `footprint_lethal_not_increased`
- `front_wedge_lethal_not_increased`
- `forward_progress_ok`
- `bounded_short_distance`
- `lateral_residual_reduced`

The dispatch payload records `safety_evidence_recomputed=true`.

If corridor carry-over is available but recomputed staging safety fails, the result is:

- `carry_over_applied=true`
- `carry_over_reject_reason=staging_safety_recompute_failed`
- `staging_applied=false`
- `staging_reject_reason=staging_safety_recompute_failed`

## Reject conditions implemented

The carry-over evaluator records these reject reasons:

- `carry_over_source_stale`
- `frame_mismatch`
- `heading_mismatch`
- `forward_window_not_trustworthy`
- `staging_not_consistent_with_source_corridor`
- `source_missing_same_corridor_or_two_side_wall`
- `source_branch_or_goal_sequence_mismatch`
- `staging_safety_recompute_failed`
- `insufficient_carry_over_evidence`

Missing evidence is rejected; it is not fabricated.

## Goal events contract

`maze_explorer` now propagates these Phase101 fields into goal event context:

- `corridor_evidence_carry_over`
- `carry_over_source`
- `carry_over_applied`
- `carry_over_reject_reason`
- `source_forward_window`
- `staging_window`
- `safety_evidence_recomputed=true`
- `branch_scoring_changed=false`
- `fallback_terminal_acceptance_used=false`

Existing Phase88/92 fields remain present:

- `multi_candidate_forward_search`
- `hard_safety_pass_candidate_count`
- `selected_candidate_target`
- `selected_candidate_yaw`
- `two_step_staging_plan`
- `staging_goal_pose`
- `staging_executability_check`
- `staging_applied`
- `staging_reject_reason`

## Guardrails

- No branch scoring change.
- No exploration order change.
- No centerline gate change.
- No directional readiness change.
- No fallback/terminal acceptance change.
- No Nav2/MPPI/controller tuning.
- No inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No autonomous exploration success claimed.
- No exit success claimed.
- No long runtime exploration was started.
- Phase102 not entered.

## TDD evidence

Focused Phase101 tests were written before implementation.

Observed RED:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase101_staging_corridor_evidence_carry_over_minimal_implementation.py
FFFFF [100%]
5 failed
```

Expected RED causes included:

- `plan_two_step_corridor_alignment_staging_goal()` did not yet accept Phase101 carry-over audit parameters.
- Phase101 reject reason inventory and goal event fields were absent.
- Phase101 report did not yet exist.

Observed implementation validation before final report:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase101_staging_corridor_evidence_carry_over_minimal_implementation.py -k 'not report_records'
.... [100%]
4 passed, 1 deselected in 0.06s

PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase92_two_step_corridor_alignment_staging_goal_minimal_implementation.py src/tugbot_maze/test/test_phase88_safety_first_multi_candidate_forward_search_minimal_implementation.py src/tugbot_maze/test/test_phase84_corridor_aligned_intermediate_goal_refinement.py
................ [100%]
16 passed in 0.56s
```

Final verification after this report was written:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase101_staging_corridor_evidence_carry_over_minimal_implementation.py
..... [100%]
5 passed in 0.05s

PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase92_two_step_corridor_alignment_staging_goal_minimal_implementation.py src/tugbot_maze/test/test_phase88_safety_first_multi_candidate_forward_search_minimal_implementation.py src/tugbot_maze/test/test_phase84_corridor_aligned_intermediate_goal_refinement.py
................ [100%]
16 passed in 0.58s

PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile src/tugbot_maze/tugbot_maze/maze_perception.py src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_maze/test/test_phase101_staging_corridor_evidence_carry_over_minimal_implementation.py
exit 0

nav2_config_diff=0
pycache_after_cleanup=0
```

## Static / small replay validation

No long Gazebo/Nav2 exploration was run.

The focused Phase101 tests include a small deterministic unit replay by monkeypatching staging metrics to simulate:

1. Local staging window missing two-side-wall evidence while Phase88 source window has same-corridor/two-side-wall evidence.
2. Corridor carry-over applying only corridor evidence and then allowing staging only when recomputed staging safety passes.
3. Forbidden source safety fields being ignored, with recomputed staging safety failure causing `staging_safety_recompute_failed`.
4. Stale source, frame mismatch, heading mismatch, goal/branch mismatch, source-missing-corridor evidence, untrustworthy forward window, and geometry inconsistency rejects.
5. Goal event contract propagation in `maze_explorer`.

## Final verification commands

To be run after this report is written:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase101_staging_corridor_evidence_carry_over_minimal_implementation.py
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase92_two_step_corridor_alignment_staging_goal_minimal_implementation.py src/tugbot_maze/test/test_phase88_safety_first_multi_candidate_forward_search_minimal_implementation.py src/tugbot_maze/test/test_phase84_corridor_aligned_intermediate_goal_refinement.py
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile src/tugbot_maze/tugbot_maze/maze_perception.py src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_maze/test/test_phase101_staging_corridor_evidence_carry_over_minimal_implementation.py
git diff -- src/tugbot_navigation/config | wc -l
find src/tugbot_maze tools -type d -name __pycache__ -prune -exec rm -rf {} +
```

## Stop condition

Stop: do not enter Phase102.

Phase101 stops after minimal implementation, focused tests, compatibility tests, py_compile, Nav2 config guard, pycache cleanup, report, and reusable lesson update.
