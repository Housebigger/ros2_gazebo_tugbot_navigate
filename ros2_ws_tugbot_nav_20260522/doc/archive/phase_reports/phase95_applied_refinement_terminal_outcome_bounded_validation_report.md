# Phase95 Applied refinement terminal outcome bounded validation report

Status: FINAL_APPLIED_REFINEMENT_TERMINAL_SUCCEEDED

Scope: Phase95 reproduced the Phase94 applied Phase88 single-step refinement scene and extended the bounded visible window only until a Goal2-equivalent terminal outcome was captured. This phase was validation-only. It did not change maze_explorer strategy, Phase88/92 logic, branch scoring, exploration order, centerline gate, directional readiness, fallback/terminal acceptance, Nav2, MPPI, controller, inflation, robot radius, clearance radius, or map thresholds.

## Required cleanup

Initial cleanup was performed before reading/tooling/runtime work.

Initial cleanup artifact:

```text
log/phase95_applied_refinement_terminal_outcome_bounded_validation/phase95_initial_cleanup_summary.json
```

Observed initial cleanup summary:

```text
matching_processes_before_count=0
project_scoped_targets_before_count=0
matching_processes_after_count=0
project_scoped_targets_after_count=0
requested_components_confirmed_absent_project_scoped=true
```

Final cleanup was performed after the terminal success was captured and analyzed. Because the terminal outcome was success, the timeout screenshot-hold condition did not apply.

Final cleanup artifact:

```text
log/phase95_applied_refinement_terminal_outcome_bounded_validation/phase95_final_cleanup_summary.json
```

Observed final cleanup summary:

```text
matching_processes_after_count=0
requested_components_confirmed_absent_project_scoped=true
```

Final verification found no project-scoped `gz sim`, `rviz2`, `slam_toolbox`, `controller_server`, `bt_navigator`, `tugbot_maze_slam_nav.launch.py`, or `maze_explorer` processes still running.

Cleanup scope was limited to Phase95 wrapper descendants and processes containing the Tugbot workspace path / Phase95 run id / tugbot launch identity. Unrelated generic ROS/Gazebo processes without project identity were not targeted.

## Required reading completed

Read before Phase95 tooling/runtime validation:

1. `doc/doc_report/phase94_applied_single_step_refinement_execution_outcome_diagnosis_report.md`
2. `doc/doc_report/phase93_two_step_staging_bounded_goal2_validation_report.md`
3. `doc/doc_report/phase92_two_step_corridor_alignment_staging_goal_minimal_implementation_report.md`
4. `doc/doc_report/phase89_safety_first_refinement_bounded_goal2_validation_report.md`
5. `doc/doc_report/phase88_safety_first_multi_candidate_forward_search_minimal_implementation_report.md`
6. `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`

## Added Phase95 tooling

Tooling:

```text
tools/run_phase95_applied_refinement_terminal_outcome_bounded_validation.sh
tools/analyze_phase95_applied_refinement_terminal_outcome_bounded_validation.py
tools/record_phase95_terminal_evidence.py
```

Tests and docs:

```text
src/tugbot_maze/test/test_phase95_applied_refinement_terminal_outcome_bounded_validation.py
doc/doc_report/phase95_applied_refinement_terminal_outcome_bounded_validation_runbook.md
doc/doc_report/phase95_applied_refinement_terminal_outcome_bounded_validation_report.md
```

Analyzer correction during Phase95: the first runtime produced a legacy Goal2-like dispatch before the applied-refinement dispatch. The analyzer was corrected to prefer dispatches where Phase88 refinement was actually applied before falling back to old-target matching. This is analyzer-only evidence selection, not a maze_explorer strategy or Nav2 change.

## Runtime command shape

The successful bounded visible run used:

```text
PHASE95_SKIP_BUILD=0
PHASE95_MAX_GOALS=3
PHASE95_GOAL_TIMEOUT_SEC=180.0
PHASE95_TERMINAL_WINDOW_SEC=260
PHASE95_RUNTIME_RECORD_TIMEOUT_SEC=320
PHASE95_HOLD_AFTER_TRIGGER_SEC=0
PHASE95_CLEANUP_ON_EXIT=0
PYTHONDONTWRITEBYTECODE=1
tools/run_phase95_applied_refinement_terminal_outcome_bounded_validation.sh
```

The first wrapper attempt failed with `Permission denied`; this was corrected by setting the wrapper executable bit. The bounded runtime then completed evidence capture and held the scene until final cleanup.

## Runtime artifacts

Artifact directory:

```text
log/phase95_applied_refinement_terminal_outcome_bounded_validation
```

Key artifacts:

```text
phase95_applied_refinement_terminal_outcome_bounded_validation_goal_events.jsonl
phase95_applied_refinement_terminal_outcome_bounded_validation_nav2_feedback.jsonl
phase95_applied_refinement_terminal_outcome_bounded_validation_runtime_timeline.jsonl
phase95_applied_refinement_terminal_outcome_bounded_validation_local_costmap_samples.jsonl
phase95_applied_refinement_terminal_outcome_bounded_validation_controller_dynamics.jsonl
phase95_applied_refinement_terminal_outcome_bounded_validation_global_plan_samples.jsonl
phase95_applied_refinement_terminal_outcome_bounded_validation_explorer_state.jsonl
phase95_applied_refinement_terminal_outcome_bounded_validation_raw_capture.json
phase95_applied_refinement_terminal_outcome_bounded_validation_analysis.json
phase95_applied_refinement_terminal_outcome_bounded_validation_minimal_field_summary.md
```

Observed artifact line counts:

```text
goal_events: 6
nav2_feedback: 18558
runtime_timeline: 293
local_costmap_samples: 522
controller_dynamics: 16098
global_plan_samples: 170
explorer_state: 388
```

## Terminal classification

Analyzer output:

```text
classification=APPLIED_REFINEMENT_TERMINAL_SUCCEEDED
validation_goal_sequence=2
used_target_match_fallback=false
evidence_gaps=[]
```

This means the terminal outcome was captured for the applied-refinement Goal2-equivalent segment, not inferred from the earlier legacy/non-applied dispatch.

## Applied refinement fields

Selected applied-refinement dispatch fields:

```text
selected_candidate_target=[1.9584924394674026, 2.0203975839605524]
selected_candidate_yaw=1.56445284232015
hard_safety_pass_candidate_count=17
original_target=[2.107220733106594, 1.819450091636875]
nav2_goal_target=[1.9584924394674026, 2.0203975839605524]
target_shift_from_original_m=0.2499999999999998
selection_priority_trace=[hard safety pass, no footprint/front-wedge lethal regression, safety_floor_ok, forward_progress_ok, clearance better, balance error smaller]
branch_scoring_changed=false
fallback_terminal_acceptance_used=false
```

## Nav2 feedback and terminal outcome

Terminal fields:

```text
terminal_event=success
terminal_reason=succeeded
terminal_pose=[1.9435135497026927, 2.0250910022473696, 1.8317083150245441]
```

Distance remaining curve summary for the applied segment:

```text
distance_remaining_first_nonzero=1.2863069772720337
distance_remaining_last=0.056494489312171936
distance_remaining_progress_delta=1.2298124879598618
feedback_sample_count=854
recoveries_max=0
recoveries_timeline=[]
```

Interpretation: the applied-refinement segment reached a Nav2 terminal success with no recovery escalation in that segment.

## Terminal local-cost evidence

Terminal local-cost metrics for the applied segment:

```text
sample_count=20
terminal_sample_elapsed_sec=184.95563626289368
terminal_sample_pose=[1.9435135497026927, 2.0250910022473696, 1.8317083150245441]
front_wedge_clearance_min_m=0.15000000000000002
front_wedge_lethal_count_last=1.0
front_wedge_lethal_count_max=27.0
robot_footprint_lethal_count_last=0.0
robot_footprint_lethal_count_max=28.0
target_footprint_lethal_count_last=0.0
target_footprint_lethal_count_max=0.0
terminal_local_cost_blocked=true
```

Interpretation: the terminal target footprint was clean at success. There was residual front-wedge/footprint risk during the segment, including one front-wedge lethal sample at the terminal sample, so local-cost risk was not zero. However, the captured terminal outcome was success, not timeout/cancel/failure, and there was no recovery-dominant terminal failure in the applied segment.

## Phase85 / Phase89 / Phase94 comparison

Prior summaries loaded by the analyzer:

```text
Phase85 classification=REFINEMENT_REJECTED_OR_NOT_TRIGGERED
Phase85 recoveries_max=4
Phase85 distance_remaining_last=0.3802814185619354
Phase85 terminal_outcome=null
Phase85 local-cost max fields unavailable in analyzer summary

Phase89 classification=PHASE88_REFINEMENT_STILL_REJECTED
Phase89 recoveries_max=4
Phase89 distance_remaining_last=0.33783894777297974
Phase89 terminal_outcome=null
Phase89 local-cost max fields unavailable in analyzer summary

Phase94 classification=APPLIED_REFINEMENT_HELD_BEFORE_TERMINAL_OUTCOME
Phase94 refinement_applied=true
Phase94 hard_safety_pass_candidate_count=6
Phase94 recoveries_max=13
Phase94 distance_remaining_last=0.3290790319442749
Phase94 terminal_outcome=held_before_terminal
Phase94 front_wedge_lethal_count_max=129.0
Phase94 robot_footprint_lethal_count_max=52.0
```

Phase95 vs prior deltas:

```text
Phase95 terminal_outcome_current=succeeded
Phase95 hard_safety_pass_candidate_count=17
Phase95 recoveries_max=0
Phase95 distance_remaining_last=0.056494489312171936
Phase95 front_wedge_lethal_count_max=27.0
Phase95 robot_footprint_lethal_count_max=28.0

Phase95_vs_Phase85:
  distance_remaining_last_delta=-0.3237869292497635
  recoveries_max_delta=-4.0
  terminal_outcome_prior=null
  terminal_outcome_current=succeeded

Phase95_vs_Phase89:
  distance_remaining_last_delta=-0.2813444584608078
  recoveries_max_delta=-4.0
  terminal_outcome_prior=null
  terminal_outcome_current=succeeded

Phase95_vs_Phase94:
  hard_safety_pass_candidate_count_delta=11.0
  recoveries_max_delta=-13.0
  distance_remaining_last_delta=-0.27258454263210297
  front_wedge_lethal_count_max_delta=-102.0
  robot_footprint_lethal_count_max_delta=-24.0
  terminal_outcome_prior=held_before_terminal
  terminal_outcome_current=succeeded
  terminal_outcome_changed_from_held_to_terminal=true
```

Comparison interpretation:

- Compared with Phase85/89, Phase95 captured an applied-refinement terminal success with lower final distance_remaining and fewer recoveries. Phase85/89 prior analyzer summaries did not expose explicit `refinement_applied` booleans, but their classifications document rejected/not-triggered refinement states.
- Compared with Phase94, Phase95 converted the previously held-before-terminal applied-refinement scene into a captured terminal success.
- Phase95 also reduced observed applied-segment recovery count and local-cost max-risk metrics relative to Phase94.
- This is only a Goal2-equivalent applied-refinement segment validation result. It is not an autonomous exploration success claim and not an exit success claim.

## Guardrails

Observed guardrail state:

```text
No maze_explorer strategy changed.
No Phase88/92 logic changed.
No branch scoring changed.
No exploration order changed.
No centerline gate changed.
No directional readiness changed.
No fallback/terminal acceptance changed.
No Nav2/MPPI/controller tuning.
No inflation/robot_radius/clearance_radius_m/map threshold tuning.
Timeout was not treated as success.
No autonomous exploration success claimed.
No exit success claimed.
Phase96 not entered.
```

Config guard:

```text
git diff -- src/tugbot_navigation/config | wc -l
0
```

## TDD and verification evidence

Focused tests were added before implementation and produced expected RED first. After implementation and analyzer correction, focused tests pass:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase95_applied_refinement_terminal_outcome_bounded_validation.py
9 passed in 0.06s
```

Static checks performed during Phase95:

```text
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile tools/analyze_phase95_applied_refinement_terminal_outcome_bounded_validation.py tools/record_phase95_terminal_evidence.py src/tugbot_maze/test/test_phase95_applied_refinement_terminal_outcome_bounded_validation.py
bash -n tools/run_phase95_applied_refinement_terminal_outcome_bounded_validation.sh
git diff -- src/tugbot_navigation/config | wc -l
```

The final verification pass is recorded in the session transcript and should be rerun after any future edits.

## Final decision

Result category:

```text
APPLIED_REFINEMENT_TERMINAL_SUCCEEDED
```

Final cleanup decision:

```text
Scene cleaned up after success. Gazebo/RViz screenshot hold was not needed because terminal timeout/local-cost/recovery-dominant failure was not the outcome.
```

Stop condition satisfied: Phase95 is complete, and Phase96 was not entered.
