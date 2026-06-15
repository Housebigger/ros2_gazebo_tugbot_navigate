# Phase142 staging gate artifact completeness design

Status: DESIGN_ONLY

Phase142 is a doc-only/design-only review. It defines how a future implementation should complete staging-gate artifacts for both the direct-explore reject staging path and the staging-triggered path. Phase142 does not implement the serialization changes, does not run the robot stack, and does not authorize Phase143 runtime.

## Guardrails

- No Gazebo/RViz/Nav2 runtime may be launched.
- No NavigateToPose goal may be sent.
- No maze_explorer may be started.
- No staging/explore/third goal may be sent.
- No Nav2/MPPI/controller/goal checker/config tuning may be performed.
- No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change may be made.
- No staging may be disabled.
- No autonomous exploration success or exit success may be claimed.
- Phase143 not entered.

## Phase141 input carried forward

Phase141 conclusion: STAGING_GATE_VARIABILITY_BY_COST_GEOMETRY, supported=true. The Phase141 boundary remains diagnostic_artifact_source_only_not_runtime_success. This means the artifacts support a diagnostic explanation of staging-gate variability, but they do not prove autonomous exploration success, exit success, a Phase127 timeout fix, or a runtime success claim.

Phase134/136 triggered corridor_alignment_staging:

- candidate_count=63
- hard_safety_pass_candidate_count=0
- front_wedge_risk.max=99
- same_corridor=true
- two_side_wall_evidence=true
- staging_reason=reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal

Phase139 direct explore:

- staging_reject_reason=single_step_forward_search_had_hard_safe_candidate
- candidate_count=63
- hard_safety_pass_candidate_count=17
- trigger_conditions.single_step_forward_search_no_hard_safety_pass=false

Phase139 evidence gaps to close before future runtime interpretation:

- lateral residual scalar: lateral_residual_before_m and lateral_residual_after_m are absent.
- target/local cost: target_local_cost and target_local_cost_max_radius are absent.
- path_corridor_min_clearance is absent.
- target_clearance is absent.
- candidate_branch_count is absent.
- last_open_direction_count and last_candidate_count are absent from the gate-completeness row.
- branch_angle is absent.
- selected_branch_geometry is incomplete.
- hard-safe candidate summaries are absent for the direct-explore sample.

## Design objective

Future runtime artifacts must explain why the staging gate made one of two decisions:

1. direct-explore reject staging path: a normal explore goal is allowed because the gate found a hard-safe single-step candidate and staging was not needed.
2. staging-triggered path: a corridor_alignment_staging goal is dispatched because all trigger conditions are satisfied and the single-step search has no hard-safe candidate.

The artifact contract must be symmetric enough that Phase134/136 and Phase139-style samples can be compared without treating missing fields as evidence. Missing gate fields should produce INSUFFICIENT_STAGING_GATE_EVIDENCE or STAGING_GATE_CONTRACT_AMBIGUOUS, not a success claim.

## Required artifact completeness contract

Every first literal dispatch that evaluates the staging gate should include a top-level `staging_gate_artifact_completeness` payload or equivalent fields under `/maze/goal_events`. The design requirement is field availability, not behavior change.

### Shared required fields for both gate paths

- goal_sequence
- goal_kind
- requested_goal_kind
- dispatch_pose
- original_target
- refined_target
- branch_angle
- selected_branch_geometry
- candidate_branch_count
- last_open_direction_count
- last_candidate_count
- two_step_stage_dispatch_requested
- staging_applied
- staging_reason
- staging_reject_reason
- gate_classification_candidate
- gate_artifact_complete
- gate_artifact_missing_fields

### Lateral residual fields

Both paths must serialize:

- lateral_residual_before_m
- lateral_residual_after_m
- lateral_residual_source: one of source_single_step_selected_metrics, staging_goal_pose, or direct_explore_projected_target
- lateral_residual_available

For the staging-triggered path, lateral_residual_before_m and lateral_residual_after_m can come from `staging_goal_pose` or the selected staging candidate. For the direct-explore reject staging path, these must come from the direct explore selected source-single-step metrics or from the direct explore projected/refined target geometry. If the direct path rejected staging because a hard-safe candidate existed, the artifact must still show the lateral residual values that participated in the trigger_conditions.near_goal_lateral_residual decision.

### Local cost and clearance fields

Both paths must serialize:

- target_local_cost
- target_local_cost_max_radius
- target_local_cost_source
- target_in_local_costmap_bounds
- path_corridor_min_clearance_m
- target_clearance_m
- clearance_source
- front_wedge_risk.max
- front_wedge_risk.mean
- front_wedge_risk.high_cost_count
- front_wedge_risk.lethal_count
- front_wedge_risk.sample_count

For staging-triggered artifacts these can be sourced from staging candidate metrics and existing local-cost diagnostics. For direct-explore artifacts they must be copied from the selected single-step candidate metrics, centerline/refinement metrics, or the local-cost dispatch diagnostics for the final direct target. The goal is to explain whether direct explore was safe enough, not to re-score or re-target it.

### Branch geometry fields

Both paths must serialize:

- candidate_branch_count
- last_open_direction_count
- last_candidate_count
- branch_angle
- selected_branch_geometry
- selected_branch_geometry.branch_angle
- selected_branch_geometry.target_xy
- selected_branch_geometry.chosen_branch_rank
- selected_branch_geometry.exit_progress_delta_m
- selected_branch_geometry.target_exit_dist
- selected_branch_geometry.edge_id
- selected_branch_geometry.state

The selected_branch_geometry payload should be a compact read-only copy of the already selected branch context. It must not feed back into branch scoring, exploration order, fallback, centerline, terminal acceptance, or any navigation parameter.

### Source single-step fields

Both paths must serialize a normalized source_single_step section:

- source_single_step.candidate_count
- source_single_step.hard_safety_pass_candidate_count
- source_single_step.refinement_applied
- source_single_step.refinement_reject_reason
- source_single_step.original_target_preserved_on_reject
- source_single_step.selected_candidate_index
- source_single_step.selected_candidate_target
- source_single_step.selected_candidate_yaw
- source_single_step.hard_safe_candidate_summaries
- source_single_step.rejected_candidate_summaries
- source_single_step.selection_priority_trace

`source_single_step.hard_safe_candidate_summaries` should be bounded and compact. A future implementation can cap it to a small number of entries while also including total hard_safety_pass_candidate_count. Each summary should include candidate_index, target_xy, yaw, lateral_residual_after_m, target_local_cost, target_local_cost_max_radius, path_corridor_min_clearance_m, target_clearance_m, front_wedge_cost_max, footprint_lethal_count, front_wedge_lethal_count, same_corridor, two_side_wall_evidence, and hard_safety_pass.

### Trigger condition fields

Both paths must serialize the four trigger_conditions exactly and the final reason fields:

- trigger_conditions.near_goal_lateral_residual
- trigger_conditions.single_step_forward_search_no_hard_safety_pass
- trigger_conditions.safety_floor_dominant_blocker
- trigger_conditions.execution_time_footprint_front_wedge_risk
- staging_reason
- staging_reject_reason

The artifact should also record the scalar evidence backing each trigger condition:

- trigger_evidence.near_goal_lateral_residual.threshold_m
- trigger_evidence.near_goal_lateral_residual.value_m
- trigger_evidence.single_step_forward_search_no_hard_safety_pass.hard_safety_pass_candidate_count
- trigger_evidence.safety_floor_dominant_blocker.path_corridor_min_clearance_m
- trigger_evidence.safety_floor_dominant_blocker.target_clearance_m
- trigger_evidence.execution_time_footprint_front_wedge_risk.max
- trigger_evidence.execution_time_footprint_front_wedge_risk.lethal_count

### Gate-path classification fields

Future analyzers should distinguish at least these diagnostic states:

- staging_not_needed_direct_explore
- staging_expected_but_not_triggered
- staging_triggered_corridor_alignment
- staging_triggered_but_unsafe_or_unavailable
- insufficient_staging_gate_evidence

`staging_not_needed_direct_explore` requires all of the following:

- goal_kind=explore
- staging_applied=false
- staging_reject_reason=single_step_forward_search_had_hard_safe_candidate
- must have source_single_step.hard_safety_pass_candidate_count > 0
- must have trigger_conditions.single_step_forward_search_no_hard_safety_pass=false
- must have source_single_step.hard_safe_candidate_summaries with at least one hard_safety_pass=true summary, unless the bounded summary cap is zero and an explicit summary_omitted_reason is present
- must have lateral_residual_before_m/lateral_residual_after_m, target_local_cost/target_local_cost_max_radius, path_corridor_min_clearance_m/target_clearance_m, and branch geometry fields

`staging_expected_but_not_triggered` applies when the four trigger conditions indicate staging should have been considered/triggered, but the dispatch remains a direct explore without a valid direct-explore reject reason. This classification must remain diagnostic; it is not a runtime failure by itself until artifacts show a complete contradiction.

missing hard-safe candidate summaries means STAGING_GATE_CONTRACT_AMBIGUOUS when the direct-explore path claims single_step_forward_search_had_hard_safe_candidate. missing lateral/cost/clearance/branch geometry means INSUFFICIENT_STAGING_GATE_EVIDENCE because the decision cannot be independently explained.

## Phase143 minimal implementation scope

Phase143 minimal implementation scope, if later accepted by a human, should be artifact/diagnostic serialization only:

- Add serialization helpers or payload copying only.
- Fill the direct-explore reject staging path with lateral residual, target/local cost, clearance, branch geometry, source_single_step, hard-safe candidate summaries, trigger_conditions, staging_reason, and staging_reject_reason.
- Preserve the staging-triggered path fields already available in Phase134/136 and make the schema symmetric with direct explore.
- Add focused static/unit tests for serialization shape and diagnostic classifications.
- Do not launch runtime unless a later phase explicitly authorizes a bounded smoke.

Phase143 restrictions:

- No gate logic changes.
- No threshold tuning.
- No staging disablement.
- No exploration strategy change.
- No branch scoring change.
- No centerline/fallback/terminal acceptance change.
- do not convert timeout or direct explore into success.
- Phase143 runtime is not authorized by Phase142.

## Verification design for future runtime

A future runtime diagnosis, if separately authorized, should only verify artifact completeness for a bounded first literal dispatch. It should not send a second/third goal unless that later phase explicitly authorizes it. It should classify the gate decision using serialized fields, then stop.

Minimum future runtime acceptance criteria:

- A first literal dispatch contains the shared required fields.
- A direct explore sample contains hard-safe candidate summaries and the scalar evidence for the hard-safe direct path.
- A staging-triggered sample contains the same named lateral/cost/clearance/branch fields, not only staging-specific nested metrics.
- The analyzer can distinguish staging_not_needed_direct_explore from staging_expected_but_not_triggered.
- Missing required fields cause INSUFFICIENT_STAGING_GATE_EVIDENCE or STAGING_GATE_CONTRACT_AMBIGUOUS.
- The result remains diagnostic and must not claim autonomous exploration success, exit success, Phase127 timeout fixed, or Phase141 runtime success.

## Source review

Read-only files:

- src/tugbot_maze/tugbot_maze/maze_explorer.py
- src/tugbot_maze/tugbot_maze/maze_perception.py
- doc/doc_report/phase141_staging_gate_variability_artifact_source_report.md

Source anchors reviewed:

- def _maybe_plan_corridor_alignment_staging
- plan_two_step_corridor_alignment_staging_goal
- def _two_step_staging_trigger_conditions
- 'near_goal_lateral_residual'
- 'single_step_forward_search_no_hard_safety_pass'
- 'safety_floor_dominant_blocker'
- 'execution_time_footprint_front_wedge_risk'
- return 'single_step_forward_search_had_hard_safe_candidate'
- source_single_step
- staging_reject_reason

maze_explorer.py and maze_perception.py were only read. Phase142 is a design review and does not implement the serialization changes.

## Non-goals

- No Nav2/controller/MPPI/goal checker tuning.
- No branch scoring or exploration strategy changes.
- No centerline behavior changes.
- No fallback or terminal acceptance changes.
- No staging disablement.
- No runtime smoke.
- No autonomous exploration or exit-success claim.
