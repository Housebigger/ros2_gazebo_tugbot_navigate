# Phase96-fix ingress-guided startup correction report

Status: FINAL_INGRESS_GUIDED_DISPATCH_OBSERVED

Goal
- Correct the Phase96 bounded multi-goal smoke workflow so maze_explorer is started only after an explicit inner-ingress Nav2 goal succeeds.
- Validate that the corrected flow produces at least one maze_explorer dispatch, or classify the corrected flow as still blocked.

Background
- Phase96 first run stayed at the simulation start pose.
- /maze/goal_events=0.
- mode=WAIT_FOR_DISPATCH_ENTRY_READINESS.
- blocking_reasons=["map_sufficient"].
- Cause for this fix phase: the Phase96 smoke wrapper omitted the established inner-ingress handoff before maze_explorer.

Files added/changed
- src/tugbot_maze/test/test_phase96_fix_ingress_guided_startup_correction.py
- tools/analyze_phase96_fix_ingress_guided_startup_correction.py
- tools/record_phase96_fix_smoke_evidence.py
- tools/run_phase96_fix_ingress_guided_startup_correction.sh
- doc/doc_report/phase96_fix_ingress_guided_startup_correction_runbook.md
- doc/doc_report/phase96_fix_ingress_guided_startup_correction_report.md

TDD evidence
- RED: Phase96-fix focused tests were created before implementation and failed against missing tooling, command exit code 1.
- GREEN: `PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase96_fix_ingress_guided_startup_correction.py` -> 4 passed in 0.03s.

Static verification
- focused pytest: 4 passed in 0.03s.
- py_compile: passed for tools/analyze_phase96_fix_ingress_guided_startup_correction.py and tools/record_phase96_fix_smoke_evidence.py.
- bash -n: passed for tools/run_phase96_fix_ingress_guided_startup_correction.sh.
- Nav2 config diff guard: `git diff -- src/tugbot_navigation/config | wc -l` -> 0.
- __pycache__ cleanup: final find output empty.

Runtime command
- `PHASE96_FIX_REUSE_VISIBLE_STACK=1 PHASE96_FIX_SKIP_BUILD=1 PHASE96_FIX_MAX_GOALS=3 PHASE96_FIX_INGRESS_TIMEOUT_SEC=90 PHASE96_FIX_GOAL_TIMEOUT_SEC=180.0 PHASE96_FIX_SMOKE_WINDOW_SEC=240 PHASE96_FIX_RUNTIME_RECORD_TIMEOUT_SEC=300 PHASE96_FIX_POST_DISPATCH_OBSERVE_SEC=35 PHASE96_FIX_CLEANUP_ON_EXIT=1 PYTHONDONTWRITEBYTECODE=1 tools/run_phase96_fix_ingress_guided_startup_correction.sh`
- Runtime wrapper exit code: 0.

Runtime evidence
- Artifact directory: log/phase96_fix_ingress_guided_startup_correction
- Classification: INGRESS_GUIDED_DISPATCH_OBSERVED
- Ingress status: succeeded
- Ingress success: True
- Ingress action_server_available: True
- Ingress goal_accepted: True
- Ingress error_code: 0
- Ingress feedback samples: 455
- Dispatch observed: True
- Dispatch event count: 1
- Observed goal sequences: [1]
- Terminal event count: 0
- Trigger: maze_explorer_dispatch_observed
- Evidence gaps: []
- Last explorer mode: NAVIGATING
- Last blocking reasons: []

First maze_explorer dispatch fields
- goal_sequence: 1
- selected target: [2.0459234467755167, 1.0235984293041964]
- selected yaw / branch angle: 1.5627014531958294
- candidate_branch_count: 4
- selected target_local_cost: None
- selected target_local_cost_max_radius: None
- selected path_corridor_min_clearance_m: 0.7762087463794127
- refinement_applied: True
- refinement_reason: balance_first_applied
- centerline_refinement_candidate_count: 63
- hard_safety_pass_candidate_count: 16
- branch_scoring_changed: False
- two_step_staging_plan: {'enabled': False, 'source_single_step': {'candidate_count': 63, 'hard_safety_pass_candidate_count': 16, 'original_target_preserved_on_reject': False, 'refinement_applied': True, 'refinement_reject_reason': None}, 'trigger_conditions': {'execution_time_footprint_front_wedge_risk': True, 'near_goal_lateral_residual': True, 'safety_floor_dominant_blocker': True, 'single_step_forward_search_no_hard_safety_pass': False}, 'visual_handoff_mode': False}
- staging_applied: False

Captured stream counts
- goal_events: 1
- explorer_state: 70
- nav2_feedback: 4749
- local_costmap_samples: 71

Raw capture coverage
- scan_available: True
- map_available: True
- local_costmap_available: True
- odom_available: True
- tf_available: True
- terminal/raw robot pose in map frame: [2.422996459794258, 1.0162040417400473, 1.5620002113216946]
- raw map summary: {'cell_count': 21384, 'free_count': 6747, 'high_cost_count': 355, 'known_count': 7102, 'lethal_count': 355, 'max': 100, 'occupied_count': 355, 'unknown_count': 14282}
- raw local_costmap summary: {'cell_count': 3600, 'free_count': 1078, 'high_cost_count': 1511, 'known_count': 3600, 'lethal_count': 1328, 'max': 100, 'occupied_count': 2018, 'unknown_count': 0}

Classification set
- INGRESS_GUIDED_DISPATCH_OBSERVED
- INGRESS_GUIDED_DISPATCH_STILL_BLOCKED
- INGRESS_GUIDED_INGRESS_FAILED
- INGRESS_GUIDED_INSUFFICIENT_EVIDENCE

Final classification
- INGRESS_GUIDED_DISPATCH_OBSERVED

Interpretation
- The ingress-guided startup correction is validated for the immediate Phase96 blocker: after the explicit inner-ingress Nav2 goal succeeded, maze_explorer produced a dispatch event.
- This validates the wrapper/runbook startup sequence correction only.
- This does not prove multi-goal refinement-chain pass, autonomous exploration success, or exit success, because this bounded fix stopped after dispatch observation and terminal_event_count remained 0.

Cleanup decision
- Initial cleanup/inspection artifact: log/phase96_fix_ingress_guided_startup_correction/phase96_fix_initial_cleanup_summary.json
- Final scoped cleanup artifact: log/phase96_fix_ingress_guided_startup_correction/phase96_fix_final_workspace_scoped_cleanup_summary.json
- Final cleanup target_count: 22
- Final cleanup remaining_matching_processes: []
- Cleanup scope: Phase96-fix final workspace-scoped cleanup: exact tugbot maze launch tree plus Phase96-fix maze_explorer/recorders only; no unrelated generic processes targeted

Guardrails
- No maze_explorer strategy changed.
- No Phase88/92 refinement logic changed.
- No branch scoring/exploration order/centerline gate/directional readiness/fallback/terminal acceptance changed.
- No Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No autonomous exploration success claimed.
- No exit success claimed.
- Phase97 not entered.
