# Phase97 ingress-guided refinement chain bounded multi-goal smoke report

Status: FINAL_INGRESS_GUIDED_REFINEMENT_CHAIN_RECOVERY_DOMINANT

Goal
- Under the corrected startup flow, run a short visible ingress-guided bounded multi-goal smoke.
- First send an explicit inner-ingress Nav2 goal before maze_explorer.
- After ingress succeeds, run maze_explorer with max_goals:=2~3 and validate Phase88/92 refinement-chain behavior for the bounded goal set only.

Required prerequisite reading completed
- doc/doc_report/phase96_fix_ingress_guided_startup_correction_report.md
- doc/doc_report/phase95_applied_refinement_terminal_outcome_bounded_validation_report.md
- doc/doc_report/phase92_two_step_corridor_alignment_staging_goal_minimal_implementation_report.md
- doc/doc_report/phase88_safety_first_multi_candidate_forward_search_minimal_implementation_report.md
- doc/doc_proposal/obstacle_reproduction_handoff_workflow.md

Cleanup
- Initial cleanup artifact: log/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke/phase97_initial_cleanup_summary.json
- Initial cleanup target_count: 0
- Initial cleanup remaining_count: 0
- Initial cleanup scope: old project-scoped Gazebo/RViz/SLAM/Nav2/maze_explorer/recorders matching this Tugbot workspace/package/launch identity only; no unrelated generic processes targeted.
- Hold-scene cleanup artifact: log/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke/phase97_hold_scene_scoped_cleanup_summary.json
- Hold-scene stopped maze_explorer/Phase97 recorder targets: 5
- Post-cleanup remaining maze_explorer/Phase97 recorder count: 0
- Visible Gazebo/RViz/SLAM/Nav2 stack preserved for screenshots: 9

Files added/changed
- src/tugbot_maze/test/test_phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke.py
- tools/analyze_phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke.py
- tools/record_phase97_smoke_evidence.py
- tools/run_phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke.sh
- doc/doc_report/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_runbook.md
- doc/doc_report/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_report.md

TDD evidence
- RED: Phase97 focused tests were written before Phase97 tooling. Initial focused run failed with 6 failed because analyzer/wrapper/recorder/runbook/report were absent.
- GREEN: `PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke.py` -> 6 passed in 0.06s after tooling.

Static verification
- focused pytest: 6 passed in 0.06s.
- py_compile: passed for Phase97 analyzer, recorder, and focused test.
- bash -n: passed for Phase97 wrapper.
- Nav2 config diff guard: `git diff -- src/tugbot_navigation/config | wc -l` -> 0.
- __pycache__ cleanup: final find output empty.

Runtime notes
- First runtime attempt exited before ingress because the wrapper used unsupported `tf2_echo --once` for readiness. This was corrected in the wrapper readiness check only; no maze_explorer strategy or Nav2 config was changed.
- Successful runtime reused the visible stack from the first launch and ran the corrected readiness check, ingress, and bounded multi-goal smoke.

Runtime command
```bash
PHASE97_REUSE_VISIBLE_STACK=1 PHASE97_SKIP_BUILD=1 PHASE97_MAX_GOALS=3 \
PHASE97_INGRESS_TIMEOUT_SEC=90 PHASE97_GOAL_TIMEOUT_SEC=180.0 \
PHASE97_SMOKE_WINDOW_SEC=420 PHASE97_RUNTIME_RECORD_TIMEOUT_SEC=480 \
PHASE97_CLEANUP_ON_EXIT=0 PYTHONDONTWRITEBYTECODE=1 \
tools/run_phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke.sh
```

Runtime artifacts
- Artifact directory: log/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke
- controller_dynamics.jsonl: 17431 lines
- explorer_state.jsonl: 390 lines
- global_plan_samples.jsonl: 228 lines
- goal_events.jsonl: 5 lines
- local_costmap_samples.jsonl: 431 lines
- nav2_feedback.jsonl: 25636 lines
- runtime_timeline.jsonl: 243 lines

Readiness and ingress
- /navigate_to_pose ready: yes
- /map ready: yes
- /scan ready: yes
- /local_costmap/costmap ready: yes
- TF ready map->base_link: yes
- explicit inner-ingress success: True
- ingress status: succeeded
- ingress feedback samples: 459

Classification
- Final classification: INGRESS_GUIDED_REFINEMENT_CHAIN_RECOVERY_DOMINANT
- Trigger: {"bad_terminal_event": "failure", "dispatch_sequences": [1], "terminal_sequences": [1], "trigger": "goal_terminal_failure_observed"}
- Observed goal sequences: [1, 2]
- Evidence gaps: []
- Raw capture summary: {"available": true, "local_costmap_available": true, "map_available": true, "odom_available": true, "scan_available": true, "tf_available": true}

Per-goal evidence
## Goal 1
- dispatch_observed: True
- terminal_outcome: failure
- terminal_event: failure
- target: [2.017031729360098, 0.9240155526940924]
- selected_candidate_target: [2.017031729360098, 0.9240155526940924]
- selected_candidate_yaw: 1.563578938982321
- refinement_applied: False
- multi_candidate_forward_search.present: True
- multi_candidate_forward_search.candidate_count: 63
- hard_safety_pass_candidate_count: 0
- two_step_staging_plan.present: True
- two_step_staging_plan.enabled: True
- two_step_staging_plan.trigger_conditions: {"execution_time_footprint_front_wedge_risk": true, "near_goal_lateral_residual": true, "safety_floor_dominant_blocker": true, "single_step_forward_search_no_hard_safety_pass": true}
- staging_applied: False
- second_step_forward_goal: None
- recoveries_max: 15
- recoveries_timeline_head: [{"elapsed_sec": 16.288565635681152, "recoveries": 0}, {"elapsed_sec": 26.372405529022217, "recoveries": 1}, {"elapsed_sec": 48.29384446144104, "recoveries": 3}, {"elapsed_sec": 58.981948375701904, "recoveries": 4}, {"elapsed_sec": 69.15205407142639, "recoveries": 5}, {"elapsed_sec": 84.4117968082428, "recoveries": 6}, {"elapsed_sec": 94.74138927459717, "recoveries": 7}, {"elapsed_sec": 107.20291256904602, "recoveries": 8}]
- nav2_feedback.sample_count: 17009
- distance_remaining_first_nonzero: 0.9333145618438721
- distance_remaining_last: 0.34746986627578735
- distance_remaining_progress_delta: 0.5858446955680847
- local_cost.sample_count: 289
- front_wedge_lethal_count_max: 109.0
- robot_footprint_lethal_count_max: 39.0
- target_footprint_lethal_count_max: 0.0
- front_wedge_clearance_min_m: 0.1
- terminal_local_cost_blocked: True
- branch_scoring_changed: False
- fallback_terminal_acceptance_used: False

## Goal 2
- dispatch_observed: True
- terminal_outcome: None
- terminal_event: None
- target: [1.946004243102576, 1.926921642277237]
- selected_candidate_target: [1.946004243102576, 1.926921642277237]
- selected_candidate_yaw: 1.5626024267875263
- refinement_applied: True
- multi_candidate_forward_search.present: True
- multi_candidate_forward_search.candidate_count: 63
- hard_safety_pass_candidate_count: 18
- two_step_staging_plan.present: True
- two_step_staging_plan.enabled: False
- two_step_staging_plan.trigger_conditions: {"execution_time_footprint_front_wedge_risk": true, "near_goal_lateral_residual": true, "safety_floor_dominant_blocker": true, "single_step_forward_search_no_hard_safety_pass": false}
- staging_applied: False
- second_step_forward_goal: None
- recoveries_max: 0
- recoveries_timeline_head: [{"elapsed_sec": 189.67531299591064, "recoveries": 0}]
- nav2_feedback.sample_count: 472
- distance_remaining_first_nonzero: 1.1084363460540771
- distance_remaining_last: 0.41449692845344543
- distance_remaining_progress_delta: 0.6939394176006317
- local_cost.sample_count: 8
- front_wedge_lethal_count_max: 31.0
- robot_footprint_lethal_count_max: 34.0
- target_footprint_lethal_count_max: 0.0
- front_wedge_clearance_min_m: 0.15000000000000002
- terminal_local_cost_blocked: True
- branch_scoring_changed: False
- fallback_terminal_acceptance_used: False

Interpretation
- The ingress-guided startup path worked: readiness completed, ingress succeeded, and maze_explorer dispatched goals.
- The bounded chain did not pass. Goal 1 entered a recovery-dominant failure pattern: recoveries_max=15, terminal_outcome=failure, front_wedge_lethal_count_max=109, robot_footprint_lethal_count_max=39.
- Goal 2 dispatched with Phase88 refinement_applied=True and hard_safety_pass_candidate_count=18, but no terminal outcome was reached before the wrapper stopped on the Goal 1 failure trigger.
- Because failure/recovery-dominant behavior occurred, timeout/failure was not treated as success and no autonomous exploration or exit success is claimed.

Screenshot suggestions while scene is held
- Gazebo wide view: robot pose after Goal 1 failure / Goal 2 dispatch context and nearby corridor walls.
- RViz local costmap: robot footprint and front wedge around the held pose.
- RViz Nav2 trajectory/plan: show whether the path is pushing into inflated obstacle or oscillating.
- RViz goal marker/maze target: show Goal 1/Goal 2 target direction relative to the corridor.

Guardrails
- No maze_explorer strategy changed.
- No Phase88/92 refinement logic changed.
- No branch scoring/exploration order/centerline gate/directional readiness/fallback/terminal acceptance changed.
- No Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No autonomous exploration success claimed.
- No exit success claimed.
- Phase98 not entered.
