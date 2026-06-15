# Phase140 staging gate variability design report

Status: `PHASE140_STAGING_GATE_VARIABILITY_DESIGN_COMPLETE_STOP_BEFORE_PHASE141`

## Summary

Phase140 completed the Staging gate variability design review as DESIGN_ONLY work.

The review explains why Phase134/136 reached first-literal `corridor_alignment_staging` while Phase139 reached first-literal `explore` with `staging_reject_reason=single_step_forward_search_had_hard_safe_candidate`.

The design conclusion is diagnostic and bounded:

- Phase134: `STAGING_GATE_TRIGGERED_CORRIDOR_ALIGNMENT`
- Phase136: `STAGING_GATE_TRIGGERED_CORRIDOR_ALIGNMENT`
- Phase139: `STAGING_GATE_DIRECT_EXPLORE_HARD_SAFE_CANDIDATE` by observed reject reason, with Phase141 required to normalize candidate/cost/geometry evidence before any stronger cross-run conclusion
- Cross-phase variability: `STAGING_GATE_VARIABILITY_BY_COST_GEOMETRY` is plausible, but Phase140 does not claim it proven because candidate_count, hard_safety_pass_candidate_count, target/local cost, path_corridor_min_clearance, and branch geometry need normalized artifact/source extraction

Phase139 first literal explore is treated as a possible valid single-step path under the existing gate, not as a second-step contract failure. Phase138 serialization contract remains unverified by runtime, but not invalidated. Phase136 remains `SECOND_STEP_CONTRACT_AMBIGUOUS`.

## Scope actually used

Phase140 was doc-only/design-only.

Actions performed:

- Read-only review of `maze_explorer.py` staging delegation logic.
- Read-only review of `maze_perception.py` staging trigger/reject logic.
- Read-only comparison of Phase134, Phase136, and Phase139 artifacts.
- Added focused static tests.
- Wrote the Phase140 proposal and report.

Actions not performed:

- No Gazebo/RViz/Nav2 runtime was launched.
- No NavigateToPose goal was sent.
- No maze_explorer was started.
- No staging/explore/third goal was sent.
- No Nav2/MPPI/controller/goal checker/config tuning was performed.
- No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change was made.
- No direct staging disablement was performed.
- No autonomous exploration success or exit success is claimed.
- No Phase127 timeout repair is claimed.

Phase141 not entered.

## Delivered files

- `doc/doc_proposal/phase140_staging_gate_variability_design.md`
- `doc/doc_report/phase140_staging_gate_variability_design_report.md`
- `src/tugbot_maze/test/test_phase140_staging_gate_variability_design.py`

## RED-first and final focused test evidence

Focused static tests were added before writing the Phase140 proposal/report.

Initial RED run:

- Command: `pytest -q src/tugbot_maze/test/test_phase140_staging_gate_variability_design.py`
- Result: failed as expected
- Summary: `6 failed, 1 passed`
- Expected reason: missing Phase140 proposal/report artifacts

This satisfied the RED step for doc-only focused tests.

Final GREEN/static verification:

- Command: `pytest -q src/tugbot_maze/test/test_phase140_staging_gate_variability_design.py`
- Result: `7 passed in 0.08s`
- Command: `python3 -m py_compile src/tugbot_maze/test/test_phase140_staging_gate_variability_design.py`
- Result: passed with no output
- Bundle command: `pytest -q src/tugbot_maze/test/test_phase137_second_step_contract_serialization_design.py src/tugbot_maze/test/test_phase138_second_step_contract_serialization_minimal_implementation.py src/tugbot_maze/test/test_phase139_instrumented_second_step_contract_runtime_verification.py src/tugbot_maze/test/test_phase140_staging_gate_variability_design.py`
- Result: `27 passed in 0.10s`
- Runtime process guard: `PHASE140_DOC_ONLY_RUNTIME_PROCESS_GUARD`, `violations=0`
- Protected config/launch diff guard: `PHASE140_PROTECTED_CONFIG_LAUNCH_DIFF_GUARD`, `protected_changes=0`

## Source review anchors

Read-only source review found:

- `maze_explorer.py` delegates staging planning through `_maybe_plan_corridor_alignment_staging`.
- `_maybe_plan_corridor_alignment_staging` passes the current explore target and centerline refinement diagnostics to `plan_two_step_corridor_alignment_staging_goal`.
- `maze_perception.py` defines `_two_step_staging_trigger_conditions`.
- The trigger bundle includes:
  - `near_goal_lateral_residual`
  - `single_step_forward_search_no_hard_safety_pass`
  - `safety_floor_dominant_blocker`
  - `execution_time_footprint_front_wedge_risk`
- `_two_step_trigger_reject_reason` returns `single_step_forward_search_had_hard_safe_candidate` when hard-safe direct forward candidates exist.

This source model supports the Phase140 design distinction:

- `staging_not_needed_direct_explore`
- `staging_expected_but_not_triggered`

## Artifact comparison anchors

Phase134 artifact:

- Phase134 first literal: goal_kind=corridor_alignment_staging
- Phase134 lateral_residual_before_m=0.175000002607703
- Phase134 front_wedge_risk max=99 lethal_count=9
- Phase134 hard_safety_pass=true same_corridor=true two_side_wall_evidence=true
- staging_applied=true
- two_step_stage_dispatch_requested=true

Phase136 artifact:

- Phase136 first literal: goal_kind=corridor_alignment_staging
- Phase136 lateral_residual_before_m=0.22500000335276146
- Phase136 front_wedge_risk max=99 lethal_count=9
- Phase136 hard_safety_pass=true same_corridor=true two_side_wall_evidence=true
- staging_applied=true
- two_step_stage_dispatch_requested=true
- second_step_goal_count=1
- third_goal_dispatched=false
- Phase136 remains SECOND_STEP_CONTRACT_AMBIGUOUS

Phase139 artifact:

- Phase139 first literal: goal_kind=explore
- Phase139 first_literal_staging_applied=false
- Phase139 staging_reject_reason=single_step_forward_search_had_hard_safe_candidate
- Phase139 staging_executability_check checked=false
- second_step_goal_count=0
- third_goal_dispatched=false
- stop_reason=first_literal_not_staging_fail_closed_stop

## Phase141 design boundary

Phase141 is defined as artifact/source analyzer only.

Allowed:

- Read Phase134/136/139 artifacts.
- Read `maze_explorer.py` and `maze_perception.py`.
- Build an analyzer that emits normalized staging-gate rows and evidence gaps.
- Use the Phase140 classification vocabulary.

Forbidden:

- No Phase141 runtime.
- No Gazebo/RViz/Nav2.
- No NavigateToPose goal.
- No maze_explorer.
- No staging/explore/third goal.
- No Nav2/MPPI/controller/goal checker/config tuning.
- No strategy, branch scoring, centerline, fallback, terminal acceptance, target selection, or goal timing changes.
- No direct staging disablement.

Future runtime, if later authorized after Phase141, should choose between:

- force observation: select a scenario expected to naturally exercise a gate branch, without disabling staging or tuning thresholds
- classification matrix: classify naturally observed artifacts without changing behavior

## Final interpretation boundary

Phase140 is diagnostic design only.

No autonomous exploration success is claimed.
No exit success is claimed.
No Phase127 timeout repair is claimed.
No Phase138 runtime contract verification is claimed.
No Phase136 success reinterpretation is claimed; Phase136 remains `SECOND_STEP_CONTRACT_AMBIGUOUS`.
Phase141 not entered.
