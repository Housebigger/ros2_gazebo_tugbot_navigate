# Phase99 Goal1 staging evidence path diagnosis report

Status: FINAL_STAGING_CANDIDATE_TOO_SHORT_OR_TOO_NEAR_FOR_WALL_EVIDENCE

Scope
- Diagnosis only. Read Phase97/Phase98 artifacts and inspect Goal1 Phase92 staging candidate / executability evidence path.
- No maze_explorer strategy, Phase88/92 logic, branch scoring, exploration order, centerline gate, directional readiness/fallback/terminal acceptance, or Nav2/config parameters were changed.
- No autonomous exploration success or exit success is claimed.
- Stop condition: Phase99 stops here and does not enter Phase100.

Required reading completed
- doc/doc_report/phase98_goal1_recovery_dominant_failure_root_cause_diagnosis_report.md
- doc/doc_report/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_report.md
- doc/doc_report/phase92_two_step_corridor_alignment_staging_goal_minimal_implementation_report.md
- doc/doc_report/phase88_safety_first_multi_candidate_forward_search_minimal_implementation_report.md
- log/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke/*
- log/phase98_goal1_recovery_dominant_failure_root_cause_diagnosis/*
- doc/doc_proposal/obstacle_reproduction_handoff_workflow.md

Added Phase99 artifacts/tooling
- tools/analyze_phase99_goal1_staging_evidence_path.py
- src/tugbot_maze/test/test_phase99_goal1_staging_evidence_path_diagnosis.py
- log/phase99_goal1_staging_evidence_path_diagnosis/phase99_goal1_staging_evidence_path_analysis.json
- log/phase99_goal1_staging_evidence_path_diagnosis/phase99_goal1_staging_evidence_path_minimal_summary.md
- doc/doc_report/phase99_goal1_staging_evidence_path_diagnosis_report.md

TDD evidence
- RED was verified before analyzer implementation: `PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase99_goal1_staging_evidence_path_diagnosis.py` -> 5 failed because tools/analyze_phase99_goal1_staging_evidence_path.py did not exist.
- GREEN after implementation: focused Phase99 tests passed.
- Final focused run before report: 5 passed in 0.07s.

Static verification
- py_compile passed for Phase99 analyzer and focused test.
- Analyzer runtime command returned: {"classification": "STAGING_CANDIDATE_TOO_SHORT_OR_TOO_NEAR_FOR_WALL_EVIDENCE", "evidence_gaps": [] }.
- Nav2 config diff guard: `git diff -- src/tugbot_navigation/config | wc -l` -> 0.
- __pycache__ directories were observed after py_compile and are cleaned at final verification.

Source evidence
- Phase97 artifact dir: log/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke
- Phase98 artifact dir: log/phase98_goal1_recovery_dominant_failure_root_cause_diagnosis
- goal_events_rows: 5
- local_costmap_samples_rows: 431
- nav2_feedback_rows: 25636
- Phase97 classification: INGRESS_GUIDED_REFINEMENT_CHAIN_RECOVERY_DOMINANT
- Phase98 classification: GOAL1_STAGING_REJECT_ROOT_CAUSE
- Evidence gaps: []

Final classification
- STAGING_CANDIDATE_TOO_SHORT_OR_TOO_NEAR_FOR_WALL_EVIDENCE

Goal1 staging evidence path extracted fields
- dispatch_pose: [1.8605400412718738, 0.02512159219049059, -0.007217387812575548]
- robot_pose_from_raw_capture: {'map': [2.261372689619239, 1.5277892733191378, 1.9917979237464192], 'odom': [2.236548975118763, 1.5293640904328676, 2.0162279237464196]}
- target: [2.017031729360098, 0.9240155526940924]
- selected_candidate_target: None
- selected_candidate_yaw: None
- corridor_heading_yaw: 1.563578938982321
- staging_plan.enabled: True
- staging_plan.trigger_conditions: {"execution_time_footprint_front_wedge_risk": true, "near_goal_lateral_residual": true, "safety_floor_dominant_blocker": true, "single_step_forward_search_no_hard_safety_pass": true}
- staging_goal_pose: None
- staging_applied: False
- staging_reject_reason: missing_two_side_wall_evidence
- staging_executability_check.reason: missing_two_side_wall_evidence
- staging_executability_check.same_corridor: False
- staging_executability_check.two_side_wall_evidence: False
- staging_executability_check.hard_safety_pass: False

Phase92 staging candidate evidence summary
- candidate_count: 9
- same_corridor_true_count: 9
- same_corridor_false_count: 0
- two_side_wall_true_count: 0
- two_side_wall_false_count: 9
- hard_safety_pass_true_count: 0
- hard_safety_pass_false_count: 9
- safety_floor_ok_true_count: 9
- bounded_short_distance_true_count: 9
- forward_progress_ok_true_count: 9
- staging_distance_min_m: 0.10000000000000006
- staging_distance_max_m: 0.18027756563297714
- x_range: {'max': 2.011257869237732, 'min': 1.960537436748838}
- y_range: {'max': 0.12439725515215896, 'min': 0.024038993401414976}
- reject_reason_counts: {"missing_two_side_wall_evidence": 9}

Phase88 single-step evidence summary
- candidate_count: 63
- hard_safety_pass_candidate_count: 0
- two_side_wall_candidate_count: 63
- refinement_applied: False
- refinement_reject_reason: lethal_cost_regression
- corridor_heading_yaw: 1.563578938982321
- candidate_summary.same_corridor_true_count: 63
- candidate_summary.two_side_wall_true_count: 63
- candidate_summary.two_side_wall_false_count: 0
- candidate_summary.y_range: {'max': 1.1272581399668515, 'min': 0.9207677563752615}
- original_metrics: {"front_wedge_sample_count": 221, "left_wall_clearance_m": 0.6000000089406967, "left_wall_hit": true, "local_cost_sample_count": 113, "min_clearance_m": 0.3500000052154064, "occupancy_free": true, "right_wall_clearance_m": 0.6000000089406967, "right_wall_hit": true, "same_corridor": true, "target": [2.017031729360098, 0.9240155526940924], "target_has_clearance": true, "two_side_wall_evidence": true}
- forward_executability_check: {"checked": true, "hard_safety_pass": false, "reason": "lethal_cost_regression", "safety_floor_ok": false, "same_corridor": true, "target_has_clearance": true, "two_side_wall_evidence": true}

Phase88 vs Phase92 comparison
- phase88_candidate_count: 63
- phase88_same_corridor_true_count: 63
- phase88_two_side_wall_true_count: 63
- phase88_y_min: 0.9207677563752615
- phase88_y_max: 1.1272581399668515
- phase92_staging_candidate_count: 9
- phase92_same_corridor_true_count: 9
- phase92_two_side_wall_true_count: 0
- phase92_y_min: 0.024038993401414976
- phase92_y_max: 0.12439725515215896
- phase88_min_y_minus_phase92_max_y: 0.7963705012231026
- interpretation: Phase88 evidence was available at the forward Goal1 target window, while Phase92 staging candidates were generated in a much earlier/near-dispatch window with no two-side-wall evidence.

Root-cause evidence flags
- staging_near_dispatch_pose: True
- short_staging_candidates: True
- phase88_forward_window_has_two_side_wall: True
- phase92_staging_window_missing_two_side_wall: True
- phase92_candidate_same_corridor_present_despite_aggregate_check_false: True
- phase88_phase92_y_window_gap_m: 0.7963705012231026

Raw capture availability
- scan: {'available': True, 'frame_id': 'tugbot/scan_omni/scan_omni', 'ranges_count': 900, 'ranges_max_finite': 9.599980354309082, 'ranges_min': 0.5631214380264282, 'stamp_sec': 438.501}
- map: available=True, summary={'cell_count': 21384, 'free_count': 6820, 'high_cost_count': 356, 'known_count': 7176, 'lethal_count': 356, 'max': 100, 'occupied_count': 356, 'unknown_count': 14208}
- local_costmap: available=True, summary={'cell_count': 3600, 'free_count': 789, 'high_cost_count': 1740, 'known_count': 3600, 'lethal_count': 1533, 'max': 100, 'occupied_count': 2283, 'unknown_count': 0}
- odom: {'available': True, 'child_frame_id': 'base_link', 'frame_id': 'odom', 'pose': {'x': 2.239296253461071, 'y': 1.5235625199795824, 'yaw': 2.0098783487004903}, 'stamp_sec': 438.552}
- tf: {'available': True, 'available_transforms': ['map->base_link', 'map->odom', 'odom->base_link'], 'map_base_link': {'available': True, 'child': 'base_link', 'parent': 'map', 'stamp_sec': 438.588, 'translation': {'x': 2.261372689619239, 'y': 1.5277892733191378, 'z': 0.0}, 'yaw': 1.9917979237464192}}

Direct answers
1. Why same-corridor / two-side-wall evidence was missing in Phase92 staging:
- Goal1 staging candidates are short near-dispatch/entrance poses; their local evidence window lacks two-side-wall hits even though Phase88 forward Goal1 target evidence has them.
- Nuance: per-candidate same_corridor was true for all 9 staging candidates, but the selected/aggregate staging executability check reported same_corridor=false and two_side_wall_evidence=false. Phase99 does not invent the missing aggregate evidence; it records this mismatch as evidence-path diagnostic material.

2. Why Phase88 single-step candidates had evidence while Phase92 staging did not:
- Phase88 evidence was available at the forward Goal1 target window, while Phase92 staging candidates were generated in a much earlier/near-dispatch window with no two-side-wall evidence.
- Phase88 candidates were in the forward Goal1 target window y≈0.9207677563752615..1.1272581399668515 and all 63 had same_corridor=true and two_side_wall_evidence=true.
- Phase92 staging candidates were near the dispatch/entrance window y≈0.024038993401414976..0.12439725515215896 with max staging distance 0.18027756563297714 m, and all 9 had two_side_wall_evidence=false.

3. Candidate root cause decision:
- Chosen token: STAGING_CANDIDATE_TOO_SHORT_OR_TOO_NEAR_FOR_WALL_EVIDENCE.
- Not chosen as primary: STAGING_EVIDENCE_WINDOW_MISMATCH_WITH_PHASE88, because the artifacts more specifically show a very short/near-dispatch staging candidate family; the window mismatch is supporting evidence.
- Not chosen as primary: STAGING_FRAME_OR_POSE_PROJECTION_MISMATCH, because per-candidate same_corridor=true and spatial windows are coherent; no direct frame/projection error is proven.
- Not chosen as primary: ENTRANCE_GOAL1_SINGLE_SIDE_WALL_GEOMETRY, because Phase88 forward evidence had two-side-wall evidence for all candidates.
- Not chosen as primary: STAGING_CHECK_TOO_STRICT_REQUIRES_EVIDENCE_REUSE_DESIGN, because this is a future design implication rather than the immediate artifact-level cause.
- Not chosen: STAGING_EVIDENCE_PATH_INSUFFICIENT_DATA, because required Phase97/98 streams and fields were present and evidence_gaps is empty.

Future design assessment
- needs_phase88_corridor_evidence_carry_over_or_evidence_reuse_for_staging: True
- do_not_relax_safety_directly: True
- rationale: Phase88 forward target/candidate evidence contained same-corridor and two-side-wall wall hits, while Phase92 staging evaluated very short near-dispatch poses that lacked two-side-wall evidence. A future design should consider explicit Phase88 corridor evidence carry-over / evidence reuse for staging eligibility before any safety relaxation.

Interpretation
- Goal1 Phase92 staging did not fail because Phase88 lacked corridor evidence globally; Phase88 had same-corridor and two-side-wall evidence in the forward Goal1 target/candidate window.
- Phase92 staging candidate generation evaluated very short near-dispatch/entrance poses. Those poses preserved local safety booleans such as safety_floor_ok/forward_progress/footprint/front-wedge non-regression but lacked two-side-wall evidence.
- Therefore the immediate evidence-path root cause is that the staging candidate family is too short or too near the entrance/dispatch pose to observe the same two-side-wall corridor evidence that Phase88 saw forward at the target window.
- A future algorithm phase should consider an explicit evidence reuse/carry-over design from Phase88 corridor evidence into Phase92 staging eligibility. That is different from simply weakening same-corridor/two-side-wall/hard-safety requirements.

Guardrails
- algorithm_changed: False
- phase88_92_logic_changed: False
- branch_scoring_or_order_changed: False
- nav2_config_changed: False
- autonomous_exploration_success_claimed: False
- exit_success_claimed: False
- phase100_entered: False
