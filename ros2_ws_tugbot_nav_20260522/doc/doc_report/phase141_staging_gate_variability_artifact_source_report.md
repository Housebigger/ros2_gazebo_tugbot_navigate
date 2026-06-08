# Phase141 staging gate variability artifact/source analysis

Status: `PHASE141_STAGING_GATE_VARIABILITY_ARTIFACT_SOURCE_COMPLETE_STOP_BEFORE_PHASE142`

## Scope

This analysis is artifact/source analyzer only. It read existing Phase134/136/139 artifacts and source anchors only.

- No Gazebo/RViz/Nav2 runtime was launched
- No NavigateToPose goal was sent
- No maze_explorer was started
- No staging/explore/third goal was sent
- No Nav2/MPPI/controller/goal checker/config tuning was performed
- No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change was made
- No staging was disabled
- No autonomous exploration success or exit success is claimed
- Phase142 not entered

## Normalized rows

### Phase134

- classification: `STAGING_GATE_TRIGGERED_CORRIDOR_ALIGNMENT`
- artifact_classification: `FIRST_DISPATCH_STAGING_ACCEPTED_STOP`
- goal_kind: `corridor_alignment_staging`
- staging_applied: `true`
- staging_reason: `reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal`
- staging_reject_reason: `None`
- lateral_residual_before_m: `0.175000002607703`
- lateral_residual_after_m: `7.719519468096792e-17`
- front_wedge_risk: max=99, mean=47.82014388489208, high_cost_count=40, lethal_count=9, sample_count=139
- source_single_step: candidate_count=63, hard_safety_pass_candidate_count=0, refinement_applied=false, original_target_preserved_on_reject=true
- trigger_conditions: `{"execution_time_footprint_front_wedge_risk": true, "near_goal_lateral_residual": true, "safety_floor_dominant_blocker": true, "single_step_forward_search_no_hard_safety_pass": true}`
- same_corridor: `true`
- two_side_wall_evidence: `true`
- target_local_cost: `0`
- target_local_cost_max_radius: `46`
- path_corridor_min_clearance_m: `0.850000012665987`
- target_clearance_m: `0.850000012665987`
- candidate_branch_count: `4`
- last_open_direction_count: `4`
- last_candidate_count: `4`
- branch_angle: `1.5654595565187426`
- selected_branch_geometry: `{"branch_angle": 1.5654595565187426, "chosen_branch_rank": 1, "exit_progress_delta_m": 0.5927159186471433, "target_exit_dist": 3.0758643073015954}`
- evidence_gaps: `[]`

### Phase136

- classification: `STAGING_GATE_TRIGGERED_CORRIDOR_ALIGNMENT`
- artifact_classification: `SECOND_STEP_CONTRACT_AMBIGUOUS`
- goal_kind: `corridor_alignment_staging`
- staging_applied: `true`
- staging_reason: `reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal`
- staging_reject_reason: `None`
- lateral_residual_before_m: `0.22500000335276146`
- lateral_residual_after_m: `1.1535911115245767e-16`
- front_wedge_risk: max=99, mean=48.518248175182485, high_cost_count=40, lethal_count=9, sample_count=137
- source_single_step: candidate_count=63, hard_safety_pass_candidate_count=0, refinement_applied=false, original_target_preserved_on_reject=true
- trigger_conditions: `{"execution_time_footprint_front_wedge_risk": true, "near_goal_lateral_residual": true, "safety_floor_dominant_blocker": true, "single_step_forward_search_no_hard_safety_pass": true}`
- same_corridor: `true`
- two_side_wall_evidence: `true`
- target_local_cost: `0`
- target_local_cost_max_radius: `46`
- path_corridor_min_clearance_m: `0.850000012665987`
- target_clearance_m: `0.850000012665987`
- candidate_branch_count: `4`
- last_open_direction_count: `4`
- last_candidate_count: `4`
- branch_angle: `1.564826936528279`
- selected_branch_geometry: `{"branch_angle": 1.564826936528279, "chosen_branch_rank": 1, "exit_progress_delta_m": 0.4561851271331032, "target_exit_dist": 3.208453264289823}`
- evidence_gaps: `[]`

### Phase139

- classification: `STAGING_GATE_DIRECT_EXPLORE_HARD_SAFE_CANDIDATE`
- artifact_classification: `SECOND_STEP_CONTRACT_STILL_AMBIGUOUS`
- goal_kind: `explore`
- staging_applied: `false`
- staging_reason: `None`
- staging_reject_reason: `single_step_forward_search_had_hard_safe_candidate`
- lateral_residual_before_m: `null`
- lateral_residual_after_m: `null`
- front_wedge_risk: max=99, mean=48.338235294117645, high_cost_count=39, lethal_count=9, sample_count=136
- source_single_step: candidate_count=63, hard_safety_pass_candidate_count=17, refinement_applied=true, original_target_preserved_on_reject=false
- trigger_conditions: `{"execution_time_footprint_front_wedge_risk": true, "near_goal_lateral_residual": true, "safety_floor_dominant_blocker": true, "single_step_forward_search_no_hard_safety_pass": false}`
- same_corridor: `false`
- two_side_wall_evidence: `false`
- target_local_cost: `null`
- target_local_cost_max_radius: `null`
- path_corridor_min_clearance_m: `null`
- target_clearance_m: `null`
- candidate_branch_count: `null`
- last_open_direction_count: `null`
- last_candidate_count: `null`
- branch_angle: `null`
- selected_branch_geometry: `{"branch_angle": null, "chosen_branch_rank": null, "exit_progress_delta_m": null, "target_exit_dist": null}`
- evidence_gaps: `["missing lateral_residual_before_m", "missing lateral_residual_after_m", "missing target_local_cost", "missing target_local_cost_max_radius", "missing path_corridor_min_clearance_m", "missing target_clearance_m", "missing candidate_branch_count", "missing last_open_direction_count", "missing last_candidate_count", "missing branch_angle"]`

## Cross-phase assessment

- classification: `STAGING_GATE_VARIABILITY_BY_COST_GEOMETRY`
- supported: `true`
- confidence: `supported_with_explicit_evidence_gaps`
- decision_boundary: `diagnostic_artifact_source_only_not_runtime_success`
- triggered_phases: `Phase134, Phase136`
- direct_explore_phases: `Phase139`

Supporting evidence:
- Phase134/Phase136 source single-step candidate_count=63 with hard_safety_pass_candidate_count=0
- Phase139 source single-step candidate_count=63 with hard_safety_pass_candidate_count=17
- Phase139 reject reason single_step_forward_search_had_hard_safe_candidate matches source trigger false condition
- all compared samples retain high front_wedge risk evidence
- same corridor and two-side wall evidence are present for staging-triggered samples
- staging-triggered samples include target/local cost and branch clearance geometry fields
- staging-triggered samples include selected branch geometry and branch_angle

Evidence gaps:
- Phase139 terminal branch geometry/local cost/clearance fields are absent from the Phase139 bounded artifact
- Phase139 lateral residual scalar is absent even though trigger_conditions.near_goal_lateral_residual is true
- Phase139: missing lateral_residual_before_m
- Phase139: missing lateral_residual_after_m
- Phase139: missing target_local_cost
- Phase139: missing target_local_cost_max_radius
- Phase139: missing path_corridor_min_clearance_m
- Phase139: missing target_clearance_m
- Phase139: missing candidate_branch_count
- Phase139: missing last_open_direction_count
- Phase139: missing last_candidate_count
- Phase139: missing branch_angle

Interpretation:

Cost/geometry/candidate evidence supports staging-gate variability: Phase134/136 had no hard-safe single-step forward candidates and therefore triggered corridor alignment staging, while Phase139 had 17 hard-safe candidates and rejected staging as not needed. The conclusion remains diagnostic because Phase139 lacks several terminal branch/local-cost/clearance scalars.

## Source review

- read_only_files: `src/tugbot_maze/tugbot_maze/maze_explorer.py, src/tugbot_maze/tugbot_maze/maze_perception.py`
- all_required_anchors_present: `true`
- present anchors:
  - `def _maybe_plan_corridor_alignment_staging`
  - `plan_two_step_corridor_alignment_staging_goal`
  - `def _two_step_staging_trigger_conditions`
  - `'near_goal_lateral_residual'`
  - `'single_step_forward_search_no_hard_safety_pass'`
  - `'safety_floor_dominant_blocker'`
  - `'execution_time_footprint_front_wedge_risk'`
  - `return 'single_step_forward_search_had_hard_safe_candidate'`

## Verification plan evidence

Focused tests were written before the analyzer implementation.

- RED command: `pytest -q src/tugbot_maze/test/test_phase141_staging_gate_variability_artifact_source.py`
- RED result: `5 failed`; expected reason was missing `tools/analyze_phase141_staging_gate_variability_artifact_source.py`
- GREEN command: `pytest -q src/tugbot_maze/test/test_phase141_staging_gate_variability_artifact_source.py`
- GREEN result: `5 passed in 0.30s`
- Static compile command: `python3 -m py_compile tools/analyze_phase141_staging_gate_variability_artifact_source.py src/tugbot_maze/test/test_phase141_staging_gate_variability_artifact_source.py`
- Artifact-only guard: no runtime processes are allowed during Phase141
- Stop condition: Phase142 not entered
