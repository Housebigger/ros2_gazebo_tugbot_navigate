# Phase140 staging gate variability design

Status: DESIGN_ONLY

## Purpose

Phase140 is a doc-only/design-only staging gate variability design review after Phase139.

Phase139 completed instrumented second-step contract runtime verification with:

- classification: SECOND_STEP_CONTRACT_STILL_AMBIGUOUS
- valid=false
- first literal dispatch: goal_kind=explore
- first_literal_staging_applied=false
- staging_reject_reason=single_step_forward_search_had_hard_safe_candidate
- no staging -> second-step chain was exercised

Phase140 explains how to diagnose why the current staging gate sometimes triggers corridor_alignment_staging and sometimes lets the first literal dispatch remain direct explore. It does not implement instrumentation, does not rerun runtime, and does not tune behavior.

## Scope and guardrails

This phase is doc-only/design-only.

Allowed:

- Read-only review of maze_explorer.py and maze_perception.py.
- Read-only review of Phase134, Phase136, and Phase139 artifacts.
- focused static tests.

Forbidden:

- No Gazebo/RViz/Nav2 runtime may be launched.
- No NavigateToPose goal may be sent.
- No maze_explorer may be started.
- No staging/explore/third goal may be sent.
- No Nav2/MPPI/controller/goal checker/config tuning may be performed.
- No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change may be made.
- No direct staging disablement is authorized.
- No autonomous exploration success or exit success may be claimed.
- No Phase127 timeout repair may be claimed.

Phase140 does not modify maze_explorer.py or maze_perception.py.
Phase141 not entered.

## Source gate model observed in read-only review

The staging gate is planned in maze_explorer.py through _maybe_plan_corridor_alignment_staging, which delegates to plan_two_step_corridor_alignment_staging_goal in maze_perception.py.

The source trigger bundle is _two_step_staging_trigger_conditions. It emits four boolean gates:

1. near_goal_lateral_residual
   - computed from lateral residual between dispatch pose and original target.
   - true only when the residual is at least the staging trigger threshold.

2. single_step_forward_search_no_hard_safety_pass
   - true only when candidate_count > 0, hard_safety_pass_candidate_count == 0, refinement_applied=false, and original_target_preserved_on_reject=true.
   - if hard_safety_pass_candidate_count > 0, _two_step_trigger_reject_reason returns single_step_forward_search_had_hard_safe_candidate.

3. safety_floor_dominant_blocker
   - true only when rejected candidates show safety floor as the dominant blocker while same_corridor and two_side_wall_evidence remain supported and occupancy/forward progress are not primary blockers.

4. execution_time_footprint_front_wedge_risk
   - true when rejected candidate evidence or runtime local-cost evidence indicates footprint/front-wedge risk.

All four gates must be true before two_step_stage_dispatch_requested can become true. Otherwise staging_reject_reason records why staging was not requested.

Important implication:

single_step_forward_search_had_hard_safe_candidate means direct explore can be a valid single-step path under the current gate. It means the forward candidate search found a hard-safe candidate and therefore the two-step staging path was not needed for that sample. It is not by itself a second-step contract failure.

## Artifact comparison: Phase134 / Phase136 / Phase139

### Phase134: staging gate triggered

Phase134 first literal: goal_kind=corridor_alignment_staging.

Observed first literal inputs/outcome:

- Phase134 lateral_residual_before_m=0.175000002607703
- Phase134 lateral_residual_after_m=7.719519468096792e-17
- Phase134 front_wedge_risk max=99 lethal_count=9
- Phase134 hard_safety_pass=true same_corridor=true two_side_wall_evidence=true
- staging_applied=true
- two_step_stage_dispatch_requested=true
- staging_reason=reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal
- staging_executability_check checked=true
- local_same_corridor=true
- local_two_side_wall_evidence=true
- local_cost_sample_count=112
- front_wedge_sample_count=255

Interpretation:

Phase134 is a STAGING_GATE_TRIGGERED_CORRIDOR_ALIGNMENT sample. It proves the gate can request corridor_alignment_staging when the trigger bundle and staging candidate hard-safety checks line up.

### Phase136: staging gate triggered, second-step contract remained ambiguous

Phase136 first literal: goal_kind=corridor_alignment_staging.

Observed first literal inputs/outcome:

- Phase136 lateral_residual_before_m=0.22500000335276146
- Phase136 lateral_residual_after_m=1.1535911115245767e-16
- Phase136 front_wedge_risk max=99 lethal_count=9
- Phase136 hard_safety_pass=true same_corridor=true two_side_wall_evidence=true
- staging_applied=true
- two_step_stage_dispatch_requested=true
- staging_reason=reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal
- staging_executability_check checked=true
- local_same_corridor=true
- local_two_side_wall_evidence=true
- local_cost_sample_count=111
- front_wedge_sample_count=255
- second_step_goal_count=1
- third_goal_dispatched=false
- Phase136 remains SECOND_STEP_CONTRACT_AMBIGUOUS because the older runtime artifact lacked the Phase138 serialization fields.

Interpretation:

Phase136 is also a STAGING_GATE_TRIGGERED_CORRIDOR_ALIGNMENT sample. It reached the staging -> second-step chain, but it does not verify the Phase138 serialization contract.

### Phase139: direct explore, no staging chain

Phase139 first literal: goal_kind=explore.

Observed first literal inputs/outcome:

- Phase139 first_literal_staging_applied=false
- Phase139 staging_reject_reason=single_step_forward_search_had_hard_safe_candidate
- Phase139 staging_executability_check checked=false
- first literal target=[2.0399239621407905, 1.0232057380741633]
- front_wedge_risk max=99 lethal_count=9
- two_step_stage_dispatch_requested=false
- second_step_goal_count=0
- third_goal_dispatched=false
- stop_reason=first_literal_not_staging_fail_closed_stop

Interpretation:

Phase139 is a STAGING_GATE_DIRECT_EXPLORE_HARD_SAFE_CANDIDATE sample. The first literal explore in Phase139 is not a second-step contract failure. The Phase138 serialization contract remains unverified but not invalidated, because the runtime did not exercise the required staging -> second-step path.

## Required Phase141 comparison fields

Phase141 must be artifact/source analyzer only. No Phase141 runtime is authorized.

A Phase141 analyzer should compare Phase134/136/139 artifacts plus source gate logic only, and should extract a normalized per-phase row with:

- phase label and artifact path
- classification and valid flag where present
- first literal goal_kind
- staging_applied
- two_step_stage_dispatch_requested
- staging_reject_reason
- staging_reason
- lateral_residual_before_m
- lateral_residual_after_m
- front_wedge_risk.max
- front_wedge_risk.lethal_count
- front_wedge_risk.high_cost_count
- staging_executability_check.checked
- staging_executability_check.hard_safety_pass
- staging_executability_check.same_corridor
- staging_executability_check.two_side_wall_evidence
- staging_executability_check.local_same_corridor
- staging_executability_check.local_two_side_wall_evidence
- staging_executability_check.target_has_clearance
- staging_executability_check.occupancy_free
- staging_executability_check.safety_floor_ok
- staging_executability_check.footprint_lethal_not_increased
- staging_executability_check.front_wedge_lethal_not_increased
- staging_executability_check.forward_progress_ok
- staging_executability_check.bounded_short_distance
- staging_executability_check.local_cost_sample_count
- staging_executability_check.front_wedge_sample_count
- candidate_count
- hard_safety_pass_candidate_count
- selected_candidate_index
- selected_candidate_target
- selection_priority_trace
- rejected_candidate_summaries
- target_local_cost_max_radius
- path_corridor_min_clearance
- dispatch_path_local_cost_max
- target_clearance_m
- branch geometry
- active_branch / start_node_id if available
- same corridor evidence source
- two-side wall evidence source

The analyzer should explicitly mark missing fields as evidence gaps. It must not infer missing candidate_count, target_local_cost_max_radius, path_corridor_min_clearance, or branch geometry from unrelated summary fields.

## Distinguishing the two main cases

### staging_not_needed_direct_explore

Use this interpretation when the artifact/source evidence shows:

- goal_kind=explore
- staging_applied=false
- two_step_stage_dispatch_requested=false
- staging_reject_reason=single_step_forward_search_had_hard_safe_candidate
- source forward search attempted candidate_count > 0
- hard_safety_pass_candidate_count > 0
- direct selected candidate has acceptable same_corridor, two_side_wall_evidence, target/local cost, target clearance, path_corridor_min_clearance, and branch geometry evidence
- no third goal is dispatched

In this case first literal explore is a valid single-step path for that sample. It should not be counted as a Phase138 second-step serialization failure. It should be classified as STAGING_GATE_DIRECT_EXPLORE_HARD_SAFE_CANDIDATE if the hard-safe-candidate evidence is complete, or STAGING_GATE_CONTRACT_AMBIGUOUS if only the reject reason is present but candidate evidence is missing.

### staging_expected_but_not_triggered

Use this interpretation when the artifact/source evidence shows:

- goal_kind=explore
- staging_applied=false
- lateral residual is above the near-goal threshold
- front_wedge_risk or footprint risk is high
- candidate_count > 0 but hard_safety_pass_candidate_count == 0, or rejected_candidate_summaries show no hard-safe direct explore candidate
- same_corridor and two_side_wall_evidence are present enough to support corridor context
- target/local cost, path_corridor_min_clearance, target_clearance_m, or branch geometry suggests direct explore is risky
- yet two_step_stage_dispatch_requested=false or staging_reject_reason does not explain the bypass

In this case the analyzer should classify as STAGING_GATE_CONTRACT_AMBIGUOUS or INSUFFICIENT_STAGING_GATE_EVIDENCE unless all gate inputs are complete enough to prove STAGING_GATE_VARIABILITY_BY_COST_GEOMETRY.

## Phase140 classification vocabulary

The allowed Phase140 / Phase141 staging-gate classifications are:

- STAGING_GATE_DIRECT_EXPLORE_HARD_SAFE_CANDIDATE
  - Direct explore was chosen because a hard-safe forward candidate existed.
  - The gate did not need staging for that sample.

- STAGING_GATE_TRIGGERED_CORRIDOR_ALIGNMENT
  - corridor_alignment_staging was dispatched as the first literal goal.
  - staging_applied=true and two_step_stage_dispatch_requested=true.

- STAGING_GATE_VARIABILITY_BY_COST_GEOMETRY
  - Cross-artifact comparison shows both direct explore and staging-triggered samples, and the difference is explained by geometry/cost/candidate inputs such as lateral residual, front_wedge_risk, hard safe candidate availability, same corridor, two-side wall evidence, candidate_count, target_local_cost_max_radius, path_corridor_min_clearance, or branch geometry.

- STAGING_GATE_CONTRACT_AMBIGUOUS
  - The artifact shows direct explore or staging but does not contain enough normalized gate-input evidence to decide whether this was expected.

- INSUFFICIENT_STAGING_GATE_EVIDENCE
  - Required artifact/source fields are absent or inconsistent enough that no meaningful gate classification should be made.

Current Phase140 design-review classification:

- Phase134: STAGING_GATE_TRIGGERED_CORRIDOR_ALIGNMENT
- Phase136: STAGING_GATE_TRIGGERED_CORRIDOR_ALIGNMENT
- Phase139: STAGING_GATE_DIRECT_EXPLORE_HARD_SAFE_CANDIDATE by reject reason, but candidate/cost/geometry completeness should be checked by Phase141 before upgrading cross-run interpretation.
- Cross-phase: STAGING_GATE_VARIABILITY_BY_COST_GEOMETRY is plausible but not fully proven until Phase141 extracts normalized candidate_count, hard_safety_pass_candidate_count, target/local cost, path corridor clearance, and branch geometry evidence from artifacts/source.

## Phase141 design boundary

Phase141 scope: artifact/source analyzer only.

Allowed in Phase141:

- Read Phase134/136/139 artifacts.
- Read maze_explorer.py and maze_perception.py.
- Build a static/source/artifact analyzer.
- Emit a classification matrix using the vocabulary above.
- Emit evidence gaps per phase.

Forbidden in Phase141:

- No Phase141 runtime.
- No Gazebo/RViz/Nav2.
- No NavigateToPose goal.
- No maze_explorer.
- No staging/explore/third goal.
- No Nav2/MPPI/controller/goal checker/config tuning.
- No exploration strategy, branch scoring, centerline, fallback, terminal acceptance, target selection, or goal timing change.
- No direct staging disablement.

Phase141 must compare Phase134/136/139 artifacts plus source gate logic only.

## Future runtime design options after Phase141

If later phases need runtime, choose one of these designs after Phase141 acceptance. Neither is authorized in Phase140 or Phase141.

### Option A: force observation

Force observation means choosing an initial condition, artifact replay, or bounded scenario likely to naturally enter the desired branch of the existing gate. It does not mean changing the gate.

Rules:

- force observation must not disable staging and must not tune thresholds.
- do not force staging by disabling hard-safe direct explore.
- do not alter branch scoring, centerline, fallback, terminal acceptance, Nav2 config, MPPI, controller, or goal checker settings.
- select only an observation scenario expected to exercise a gate branch naturally.
- still stop after the bounded observation target and do not run full exploration.

### Option B: classification matrix

classification matrix means accumulating naturally observed artifacts and classifying each artifact against the source gate model.

Rules:

- classification matrix must classify naturally observed artifacts without changing behavior.
- use normalized rows for lateral residual, front_wedge_risk, hard safe candidate, same corridor, two-side wall evidence, candidate_count, target/local cost, and branch geometry.
- do not change runtime behavior to fill missing cells.
- missing data should become evidence gaps, not inferred success.

Preferred next step:

Phase141 should start with classification matrix as artifact/source analyzer only. If evidence gaps remain, a future runtime phase may choose force observation, but only after acceptance and without disabling staging or tuning parameters.

## Non-claims

Phase140 does not claim autonomous exploration success.
Phase140 does not claim exit success.
Phase140 does not claim Phase127 timeout fixed.
Phase140 does not claim Phase138 contract verified.
Phase140 does not classify Phase136 as success; Phase136 remains SECOND_STEP_CONTRACT_AMBIGUOUS.
Phase140 does not enter Phase141.
