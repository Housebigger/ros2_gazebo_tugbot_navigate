# Phase90: Rejected candidate failure landscape diagnosis report

Status: COMPLETE_DIAGNOSTIC_ONLY_STOP_BEFORE_PHASE91
Date: 2026-06-02
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## Scope

Phase90 diagnoses the Phase89 Goal2-equivalent rejected-candidate landscape after Phase88 safety-first multi-candidate forward search emitted candidate diagnostics but did not apply a refined target.

This phase is read-only with respect to navigation behavior:

- No Phase88 refinement changed.
- No maze_explorer strategy changed.
- No branch scoring changed.
- No exploration order changed.
- No centerline gate changed.
- No directional readiness changed.
- No fallback/terminal acceptance changed.
- No Nav2/MPPI/controller tuning.
- No inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No autonomous exploration success claimed.
- No exit success claimed.
- Phase91 not entered.

## Required cleanup summary

Phase90 first closed the Phase89 visible run scene and recorded:

`log/phase90_rejected_candidate_failure_landscape_diagnosis/phase90_process_cleanup_summary.md`

Cleanup was workspace-scoped to avoid unrelated process kills. The cleanup summary recorded the selected `tugbot_maze_slam_nav.launch.py` root and descendants, then confirmed the global named target snapshot after cleanup as `none` for the target names. A later independent verification also found `target_process_match_count=0` for Gazebo/RViz/SLAM/Nav2/maze_explorer/ros2-launch target patterns.

## Inputs read

Reports and workflow:

- `doc/doc_report/phase89_safety_first_refinement_bounded_goal2_validation_report.md`
- `doc/doc_report/phase88_safety_first_multi_candidate_forward_search_minimal_implementation_report.md`
- `doc/doc_report/phase86_lethal_cost_regression_reject_root_cause_report.md`
- `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`

Phase89 artifacts:

- `log/phase89_safety_first_refinement_bounded_goal2_validation/phase89_safety_first_refinement_bounded_goal2_validation_analysis.json`
- `log/phase89_safety_first_refinement_bounded_goal2_validation/phase89_safety_first_refinement_bounded_goal2_validation_goal_events.jsonl`
- `log/phase89_safety_first_refinement_bounded_goal2_validation/phase89_safety_first_refinement_bounded_goal2_validation_local_costmap_samples.jsonl`
- `log/phase89_safety_first_refinement_bounded_goal2_validation/phase89_safety_first_refinement_bounded_goal2_validation_raw_capture.json`
- `log/phase89_safety_first_refinement_bounded_goal2_validation/phase89_safety_first_refinement_bounded_goal2_validation_minimal_field_summary.md`
- `log/phase89_safety_first_refinement_bounded_goal2_validation/phase89_safety_first_refinement_bounded_goal2_validation_trigger_detected.json`

## New files

Analyzer and tests:

- `tools/analyze_phase90_rejected_candidate_failure_landscape.py`
- `src/tugbot_maze/test/test_phase90_rejected_candidate_failure_landscape_diagnosis.py`

Generated artifacts:

- `log/phase90_rejected_candidate_failure_landscape_diagnosis/phase90_rejected_candidate_failure_landscape_diagnosis.json`
- `log/phase90_rejected_candidate_failure_landscape_diagnosis/phase90_rejected_candidate_failure_landscape_diagnosis_minimal_summary.md`

This report:

- `doc/doc_report/phase90_rejected_candidate_failure_landscape_diagnosis_report.md`

## Analyzer result

Command:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 tools/analyze_phase90_rejected_candidate_failure_landscape.py \
  --phase89-artifact-dir log/phase89_safety_first_refinement_bounded_goal2_validation \
  --output-json log/phase90_rejected_candidate_failure_landscape_diagnosis/phase90_rejected_candidate_failure_landscape_diagnosis.json \
  --minimal-summary log/phase90_rejected_candidate_failure_landscape_diagnosis/phase90_rejected_candidate_failure_landscape_diagnosis_minimal_summary.md
```

Output:

```json
{
  "candidate_count": 63,
  "classification": "STAGING_ALIGNMENT_GOAL_NEEDED",
  "evidence_gaps": [],
  "hard_safety_pass_candidate_count": 0
}
```

Primary classification:

`STAGING_ALIGNMENT_GOAL_NEEDED`

Secondary supported classifications:

- `SAFETY_FLOOR_DOMINANT_BLOCKER`
- `CANDIDATE_FAMILY_TOO_LOCAL`

Classification reasons:

- all candidates failed hard safety
- all candidates failed safety floor
- some zero-lethal candidates still failed clearance floor
- execution-time front-wedge or footprint lethal evidence was present

## Source Phase89 context

- Phase89 classification: `PHASE88_REFINEMENT_STILL_REJECTED`
- candidate_count: 63
- hard_safety_pass_candidate_count: 0
- refinement_applied: false
- refinement_reject_reason: `lethal_cost_regression`
- original_target_preserved_on_reject: true
- branch_scoring_changed: false
- fallback_terminal_acceptance_used: false

## Candidate failure landscape

Failure false counts across 63 candidates:

- safety_floor: 63 / 63
- footprint_lethal_regression: 27 / 63
- front_wedge_lethal_regression: 35 / 63
- clearance_insufficient: 21 / 63
- forward_progress: 0 / 63
- occupancy: 0 / 63
- same_corridor: 0 / 63
- two_side_wall: 0 / 63

Interpretation:

The landscape is not a forward-progress, occupancy, same-corridor, or missing two-side-wall-evidence failure. All 63 candidates remained same-corridor, occupancy-free, two-side-wall supported, and forward-progress valid. The universal rejection dimension is safety floor / clearance margin, with front-wedge and footprint lethal regressions appearing as additional execution-risk signals rather than the sole blocker.

## Offset and side concentration

Offset grid coverage:

- lateral offsets: 7 buckets, 9 failed candidates each (`-0.45`, `-0.30`, `-0.15`, `0.0`, `0.15`, `0.30`, `0.45`)
- forward offsets: 3 buckets, 21 failed candidates each (`0.0`, `0.1`, `0.2`)
- heading offsets: 3 buckets, 21 failed candidates each (`-0.0872664626`, `0.0`, `0.0872664626`)

No single lateral, forward, or heading offset dominated the rejected landscape. The failure is distributed across the whole local candidate family.

Wall-side concentration:

- left wall nearer: 18
- right wall nearer: 24
- equal wall clearances: 21
- missing side data: 0
- dominant side: none

The rejected landscape is not concentrated on one wall side. Both side-near and centered/equal candidates are represented, and all still fail the safety floor.

Metric ranges:

- min_clearance_m: 0.0500000007 to 0.4031128934
- local_cost_max_radius: 54 to 99
- front_wedge_lethal_count: 0 to 90
- footprint_lethal_count: 0 to 51
- lateral_offset_m: -0.45 to 0.45
- forward_offset_m: 0.0 to 0.2
- heading_offset_rad: -0.0872664626 to 0.0872664626

The best observed candidate clearance stayed below the recorded safety floor of 0.45 m. Several least-risk candidates had zero footprint/front-wedge lethal counts but still failed `safety_floor_ok`, which supports the conclusion that the local candidate family cannot produce an executable single forward goal from this pose.

## Execution-time local-cost context

Phase89 local-cost samples for goal sequence 1:

- sample_count: 204
- front_wedge_lethal_count: min 0, max 142, last 66
- robot_footprint_lethal_count: min 0, max 52, last 52
- target_footprint_lethal_count: min 0, max 0, last 0
- target_radius_cost_max: min 54, max 63, last 63
- last_robot_pose: `[2.42368527111335, 1.0222857971263482, 1.5801517212345726]`

This supports Phase75/89 direction: the dispatch target itself is not the only problem; execution-time robot footprint/front-wedge cost at the final/near-final pose remains the active risk.

## Two-step intermediate goal assessment

Assessment:

`two_step_corridor_alignment_staging_supported_before_forward_exploration_goal`

Supporting reasons:

- no hard-safety-pass candidate in the local forward-search family
- zero-lethal candidate subset still fails safety floor
- execution-time front-wedge or footprint lethal evidence is present

Conservative interpretation:

Phase90 supports investigating a two-step flow in a later phase: first a corridor-alignment/staging pose, then a forward exploration goal. This is not an implementation decision in Phase90. It is a diagnosis-only handoff candidate because a single local forward candidate family appears too local/too constrained at the current pose.

## Artifacts to inspect before any future Phase91 decision

- Full JSON: `log/phase90_rejected_candidate_failure_landscape_diagnosis/phase90_rejected_candidate_failure_landscape_diagnosis.json`
- Minimal summary: `log/phase90_rejected_candidate_failure_landscape_diagnosis/phase90_rejected_candidate_failure_landscape_diagnosis_minimal_summary.md`
- Least-risk rejected candidates are recorded under `candidate_failure_landscape.least_risky_rejected_candidates` in the JSON.
- Offset bucket details are recorded under `candidate_failure_landscape.offset_distributions`.
- Wall-side details are recorded under `candidate_failure_landscape.wall_side_concentration`.

## Verification

Phase90 verification commands are recorded in the final response and include:

- RED focused tests first failed because the analyzer/artifacts/report did not yet exist.
- Analyzer ran successfully with `PYTHONDONTWRITEBYTECODE=1`.
- Focused tests passed after implementation/report generation.
- `py_compile` passed for analyzer and focused test.
- Guard diff confirmed no runtime/config strategy changes.
- `__pycache__` cleanup was performed.

## Final guardrail statement

Phase90 is a diagnostic-only phase. It does not change navigation strategy or tuning, does not claim autonomous exploration success, does not claim exit success, and stops before Phase91.
