# Phase98 Goal1 recovery-dominant failure root-cause diagnosis report

Status: FINAL_GOAL1_STAGING_REJECT_ROOT_CAUSE

Scope
- Diagnosis only. Read Phase97 artifacts and decomposed Goal1 recovery-dominant terminal failure.
- No maze_explorer strategy, Phase88/92 logic, branch scoring, exploration order, centerline gate, directional readiness/fallback/terminal acceptance, or Nav2/config parameters were changed.
- No autonomous exploration success or exit success is claimed.

Required reading completed
- doc/doc_report/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_report.md
- doc/doc_report/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_runbook.md
- doc/doc_report/phase95_applied_refinement_terminal_outcome_bounded_validation_report.md
- doc/doc_report/phase92_two_step_corridor_alignment_staging_goal_minimal_implementation_report.md
- doc/doc_report/phase88_safety_first_multi_candidate_forward_search_minimal_implementation_report.md
- log/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke/*

Added Phase98 artifacts/tooling
- tools/analyze_phase98_goal1_recovery_dominant_failure_root_cause.py
- src/tugbot_maze/test/test_phase98_goal1_recovery_dominant_failure_root_cause_diagnosis.py
- log/phase98_goal1_recovery_dominant_failure_root_cause_diagnosis/phase98_goal1_recovery_dominant_failure_root_cause_analysis.json
- log/phase98_goal1_recovery_dominant_failure_root_cause_diagnosis/phase98_goal1_recovery_dominant_failure_root_cause_minimal_summary.md
- doc/doc_report/phase98_goal1_recovery_dominant_failure_root_cause_diagnosis_report.md

TDD evidence
- RED was verified before analyzer implementation: focused Phase98 test run failed with 5 failures because tools/analyze_phase98_goal1_recovery_dominant_failure_root_cause.py did not exist.
- GREEN after implementation: `PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase98_goal1_recovery_dominant_failure_root_cause_diagnosis.py` -> 5 passed in 0.05s; final static run -> 5 passed in 0.06s.

Static verification
- py_compile passed for Phase98 analyzer and test.
- Nav2 config diff guard: `git diff -- src/tugbot_navigation/config | wc -l` -> 0.
- __pycache__ cleanup final find output was empty after cleanup.

Source evidence
- Source artifact dir: log/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke
- Source Phase97 classification: INGRESS_GUIDED_REFINEMENT_CHAIN_RECOVERY_DOMINANT
- Stream counts: {"goal1_goal_event_rows": 2, "goal1_local_costmap_samples": 289, "goal1_nav2_feedback": 17468, "goal_events": 5, "local_costmap_samples": 431, "nav2_feedback": 25636}
- Raw capture summary: {"available": true, "local_costmap_available": true, "map_available": true, "odom_available": true, "scan_available": true, "tf_available": true}
- Evidence gaps: []

Final classification
- GOAL1_STAGING_REJECT_ROOT_CAUSE

Goal1 extracted fields
- dispatch_observed: True
- terminal_observed: True
- terminal_outcome: failure
- terminal_event: failure
- target: [2.017031729360098, 0.9240155526940924]
- selected_candidate_target: [2.017031729360098, 0.9240155526940924]
- selected_candidate_yaw: 1.563578938982321
- refinement_applied: False
- refinement_reject_reason: lethal_cost_regression
- multi_candidate_forward_search: {"candidate_count": 63, "present": true}
- hard_safety_pass_candidate_count: 0
- staging_trigger_bundle: {"execution_time_footprint_front_wedge_risk": true, "near_goal_lateral_residual": true, "safety_floor_dominant_blocker": true, "single_step_forward_search_no_hard_safety_pass": true}
- staging_applied: False
- staging_reject_reason: missing_two_side_wall_evidence
- second_step_forward_goal: None
- branch_scoring_changed: False
- fallback_terminal_acceptance_used: False

Staging executability check
- checked: True
- reason: missing_two_side_wall_evidence
- two_side_wall_evidence: False
- same_corridor: False
- hard_safety_pass: False
- safety_floor_ok: False
- occupancy_free: False
- target_has_clearance: False
- footprint_lethal_not_increased: False
- front_wedge_lethal_not_increased: False
- forward_progress_ok: False
- bounded_short_distance: False
- lateral_residual_reduced: False
- tf_stamp_age_sec: None
- local_costmap_stamp_age_sec: None

Single-step candidate failure landscape
- candidate_count: 63
- hard_safety_pass_candidate_count: 0
- reject_reason_counts: {"footprint_lethal_not_increased": 36, "front_wedge_lethal_not_increased": 41, "safety_floor_ok": 63, "target_has_clearance": 24}
- boolean_pass_counts: {"footprint_lethal_not_increased": {"false": 36, "true": 27}, "front_wedge_lethal_not_increased": {"false": 41, "true": 22}, "hard_safety_pass": {"false": 63}, "safety_floor_ok": {"false": 63}, "same_corridor": {"true": 63}, "target_has_clearance": {"false": 24, "true": 39}, "two_side_wall_evidence": {"true": 63}}
- metric_summary: {"balance_error_m": {"max": 0.9000000134110451, "mean": 0.23333333681027094, "min": 0.0}, "footprint_lethal_count": {"max": 38.0, "mean": 12.095238095238095, "min": 0.0}, "forward_progress_m": {"max": 1.1000000000000005, "mean": 1.0000000000000004, "min": 0.9000000000000002}, "front_wedge_lethal_count": {"max": 78.0, "mean": 16.873015873015873, "min": 0.0}, "local_cost_max_radius": {"max": 99.0, "mean": 82.52380952380952, "min": 46.0}, "min_clearance_m": {"max": 0.42720019363165534, "mean": 0.15038042331508442, "min": 0.05000000074505806}}
- best_by_clearance top 3:
  - {"candidate_index": 6, "candidate_reject_reasons": ["front_wedge_lethal_not_increased", "safety_floor_ok"], "footprint_lethal_not_increased": true, "front_wedge_lethal_not_increased": false, "hard_safety_pass": false, "min_clearance_m": 0.42720019363165534, "safety_floor_ok": false, "target": [2.167027822575544, 0.9229329539211488], "target_yaw": 1.4763124763826045}
  - {"candidate_index": 7, "candidate_reject_reasons": ["front_wedge_lethal_not_increased", "safety_floor_ok"], "footprint_lethal_not_increased": true, "front_wedge_lethal_not_increased": false, "hard_safety_pass": false, "min_clearance_m": 0.42720019363165534, "safety_floor_ok": false, "target": [2.167027822575544, 0.9229329539211488], "target_yaw": 1.563578938982321}
  - {"candidate_index": 8, "candidate_reject_reasons": ["safety_floor_ok"], "footprint_lethal_not_increased": true, "front_wedge_lethal_not_increased": true, "hard_safety_pass": false, "min_clearance_m": 0.42720019363165534, "safety_floor_ok": false, "target": [2.167027822575544, 0.9229329539211488], "target_yaw": 1.6508454015820373}
- lowest_front_wedge_lethal top 3:
  - {"candidate_index": 8, "candidate_reject_reasons": ["safety_floor_ok"], "footprint_lethal_not_increased": true, "front_wedge_lethal_count": 0, "front_wedge_lethal_not_increased": true, "hard_safety_pass": false, "safety_floor_ok": false, "target": [2.167027822575544, 0.9229329539211488], "target_yaw": 1.6508454015820373}
  - {"candidate_index": 9, "candidate_reject_reasons": ["safety_floor_ok"], "footprint_lethal_not_increased": true, "front_wedge_lethal_count": 0, "front_wedge_lethal_not_increased": true, "hard_safety_pass": false, "safety_floor_ok": false, "target": [2.017031729360098, 0.9240155526940924], "target_yaw": 1.4763124763826045}
  - {"candidate_index": 10, "candidate_reject_reasons": ["safety_floor_ok"], "footprint_lethal_not_increased": true, "front_wedge_lethal_count": 0, "front_wedge_lethal_not_increased": true, "hard_safety_pass": false, "safety_floor_ok": false, "target": [2.017031729360098, 0.9240155526940924], "target_yaw": 1.563578938982321}

Recovery timeline and distance plateau
- nav2 sample_count: 17468
- max_recoveries: 15
- distance_remaining_first_nonzero: 0.9333145618438721
- distance_remaining_last: 0.34746986627578735
- distance_remaining_min: 0.0
- distance_remaining_max: 1.9926457405090332
- distance_remaining_progress_delta: 0.5858446955680847
- plateau_tail_delta_last_100_samples: 0.0
- recovery_timeline:
  - {"distance_remaining": 0.0, "elapsed_sec": 4.652546405792236, "recoveries": 0}
  - {"distance_remaining": 0.9732714891433716, "elapsed_sec": 26.372405529022217, "recoveries": 1}
  - {"distance_remaining": 0.34746986627578735, "elapsed_sec": 48.29384446144104, "recoveries": 3}
  - {"distance_remaining": 0.34746986627578735, "elapsed_sec": 58.981948375701904, "recoveries": 4}
  - {"distance_remaining": 0.34746986627578735, "elapsed_sec": 69.15205407142639, "recoveries": 5}
  - {"distance_remaining": 0.34746986627578735, "elapsed_sec": 84.4117968082428, "recoveries": 6}
  - {"distance_remaining": 0.34746986627578735, "elapsed_sec": 94.74138927459717, "recoveries": 7}
  - {"distance_remaining": 0.34746986627578735, "elapsed_sec": 107.20291256904602, "recoveries": 8}
  - {"distance_remaining": 0.34746986627578735, "elapsed_sec": 117.32332730293274, "recoveries": 10}
  - {"distance_remaining": 0.34746986627578735, "elapsed_sec": 127.98233318328857, "recoveries": 11}
  - {"distance_remaining": 0.34746986627578735, "elapsed_sec": 138.1515052318573, "recoveries": 12}
  - {"distance_remaining": 0.34746986627578735, "elapsed_sec": 153.41262030601501, "recoveries": 13}
  - {"distance_remaining": 0.34746986627578735, "elapsed_sec": 163.74122619628906, "recoveries": 14}
  - {"distance_remaining": 0.34746986627578735, "elapsed_sec": 176.2034854888916, "recoveries": 15}

Terminal pose / local-cost / footprint / front-wedge risk
- sample_count: 289
- terminal_sample_elapsed_sec: 189.61298894882202
- terminal_pose: [2.352704174579082, 0.9277055164191437, 1.5870324267875269]
- front_wedge_lethal_count_max: 109.0
- robot_footprint_lethal_count_max: 39.0
- target_footprint_lethal_count_max: 0.0
- front_wedge_clearance_min_m: 0.1
- terminal_local_cost_blocked: True

Direct answers
1. Why Phase88 refinement was not applied: Phase88 did evaluate the multi-candidate family, but refinement_applied=false because refinement_reject_reason='lethal_cost_regression' and hard_safety_pass_candidate_count=0. The candidate landscape shows reject_reason_counts={'footprint_lethal_not_increased': 36, 'front_wedge_lethal_not_increased': 41, 'safety_floor_ok': 63, 'target_has_clearance': 24}.
2. Why hard_safety_pass_candidate_count=0: No candidate satisfied the full hard-safety bundle. The strongest common blockers were {'footprint_lethal_not_increased': 36, 'front_wedge_lethal_not_increased': 41, 'safety_floor_ok': 63, 'target_has_clearance': 24}; best observed min_clearance_m=0.42720019363165534, while safety_floor_ok remained false for the sampled candidate family.
3. Why staging enabled but staging_applied=false: The Phase92 trigger bundle was enabled, but staging_applied=false because staging_reject_reason='missing_two_side_wall_evidence'. The staging_executability_check reported reason='missing_two_side_wall_evidence', two_side_wall_evidence=False, same_corridor=False, hard_safety_pass=False.
4. What needs diagnosis next: Do not tune Nav2 or change exploration order from this evidence. The next root-cause focus should be the staging candidate/executability evidence path (missing_two_side_wall_evidence) plus the single-step no-hard-safe candidate landscape. Nav2 execution was recovery-dominant after the original unsafe target was used (recoveries_max=15, front_wedge_lethal_count_max=109.0, robot_footprint_lethal_count_max=39.0), but the upstream staging rejection explains why the designed escape path did not replace that target.

Root-cause interpretation
- classification=GOAL1_STAGING_REJECT_ROOT_CAUSE: staging was triggered, not applied, and rejected before a staging goal could be dispatched; therefore the robot executed the non-refined Goal1 target and then entered recovery-dominant/local-cost-blocked behavior.
- Nuance: this does not mean Nav2 execution/local-cost risk was absent. Nav2 became recovery-dominant after the original non-refined Goal1 target was executed; terminal local-cost metrics show footprint/front-wedge risk. The upstream diagnostic root cause is that the designed Phase92 staging escape path rejected before dispatch, while Phase88 had no hard-safe single-step candidate.

Guardrails
- algorithm_changed: False
- autonomous_exploration_success_claimed: False
- branch_scoring_or_order_changed: False
- exit_success_claimed: False
- nav2_config_changed: False
- phase88_92_logic_changed: False
- phase99_entered: False
- Phase99 not entered.
