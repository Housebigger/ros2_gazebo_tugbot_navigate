from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
WRAPPER = ROOT / 'tools' / 'run_phase56_post_ingress_candidate_formation_diagnostics.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase56_post_ingress_candidate_formation_diagnostics.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase56_post_ingress_candidate_formation_diagnostics_report.md'
RUN_ID = 'phase56_post_ingress_candidate_formation_diagnostics'

ALLOWED_CLASSIFICATIONS = {
    'CANDIDATE_REJECTED_BY_MIN_DISTANCE',
    'CANDIDATE_REJECTED_BY_CLEARANCE',
    'CANDIDATE_REJECTED_AS_DUPLICATE_OR_EXHAUSTED',
    'CANDIDATE_REJECTED_BY_JUNCTION_OR_DEAD_END_POLICY',
    'CANDIDATE_FORMATION_REJECTION_CAUSE_IDENTIFIED',
    'OPEN_DIRECTION_TO_CANDIDATE_INCONCLUSIVE',
    'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
}


def test_phase56_maze_explorer_readonly_candidate_diagnostics_fields():
    text = MAZE_EXPLORER.read_text()
    assert 'phase56_open_direction_to_candidate_diagnostics' in text
    assert 'candidate_before_filter_count' in text
    assert 'candidate_after_filter_count' in text
    assert 'accepted_open_direction' in text
    required_fields = [
        'accepted_open_direction_angle_rad',
        'accepted_open_direction_vector',
        'sample_endpoint',
        'projection_distance_m',
        'lookahead_distance_m',
        'candidate_goal_point',
        'candidate_map_cell_state',
        'candidate_local_costmap_cell_state',
        'candidate_clearance_result',
        'min_travel_distance_check',
        'too_close_check',
        'duplicate_or_exhausted_filter',
        'junction_or_dead_end_policy_filter',
        'branch_candidate_rejection_reason',
    ]
    for field in required_fields:
        assert field in text
    assert 'Phase56 read-only' in text
    assert 'no strategy change' in text
    # Existing clearance_radius_m declaration is allowed; Phase56 must not add tuning
    # or wrapper overrides. The wrapper/report tests enforce that guardrail.


def test_phase56_wrapper_contract_and_guardrails():
    text = WRAPPER.read_text()
    assert f'RUN_ID="{RUN_ID}"' in text
    assert f'ARTIFACT_DIR="log/${{RUN_ID}}"' in text
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in text
    assert 'maze_20260528_scaled_instance.yaml' in text
    assert 'tugbot_maze_slam_nav.launch.py' in text
    assert 'tools/analyze_phase56_post_ingress_candidate_formation_diagnostics.py --record-runtime' in text
    assert 'tools/analyze_phase56_post_ingress_candidate_formation_diagnostics.py --send-goal' in text
    assert 'ros2 run tugbot_maze maze_explorer' in text
    assert 'maze_explorer --ros-args' in text
    assert '-p max_goals:=1' in text
    assert '-p near_exit_fallback_enabled:=false' in text
    assert '-p startup_warmup_no_dispatch:=false' in text
    assert '-p clearance_radius_m' not in text
    assert '-p dispatch_readiness_min_map_known_ratio' not in text
    assert '-p dispatch_readiness_min_map_free_ratio' not in text
    assert 'INGRESS_X="1.0"' in text
    assert 'INGRESS_Y="0.0"' in text
    assert 'ACCEPTANCE_RADIUS_M="${PHASE56_ACCEPTANCE_RADIUS_M:-0.35}"' in text
    assert 'RUN_TIMEOUT_SEC' in text and 'RUN_TIMEOUT_SEC > 360' in text
    assert 'open_direction_to_candidate_diagnostics.json' in text
    assert 'first_topology_sampling_snapshot.json' in text
    assert 'cleanup()' in text
    assert 'git diff -- src/tugbot_navigation/config' in text
    assert 'GUARDRAIL_VIOLATION_STRATEGY_CHANGED' in text
    assert 'autonomous exploration success' in text
    assert 'first dispatch is not exit success' in text


def test_phase56_analyzer_contract_and_classifications():
    text = ANALYZER.read_text()
    for classification in ALLOWED_CLASSIFICATIONS:
        assert classification in text
    assert 'PHASE = ' in text and 'Phase56 Post-Ingress Open Direction to Candidate Formation Diagnostics' in text
    assert 'RUN_ID = ' in text and RUN_ID in text
    assert 'INGRESS_WAYPOINT_MAP' in text and "'x_m': 1.0" in text
    assert 'ACCEPTANCE_RADIUS_M = 0.35' in text
    assert 'read_explorer_state_jsonl' in text
    assert 'read_goal_events_jsonl' in text
    assert 'extract_open_direction_to_candidate_diagnostics' in text
    assert 'first_topology_sampling_snapshot' in text
    assert 'open_direction_to_candidate_diagnostics' in text
    assert 'candidate formation restored' in text
    assert 'complete_autonomous_success_claimed' in text
    assert 'max_goals_limited_to_one' in text
    assert 'near_exit_fallback_disabled' in text


def test_phase56_artifact_contract():
    text = ANALYZER.read_text()
    required_artifacts = [
        'ingress action result',
        'explorer_state.jsonl',
        'goal_events.jsonl',
        'first topology sampling snapshot',
        'open_direction_to_candidate_diagnostics.json',
        'map/scan/TF/local/global costmap evidence',
        'summary/analysis JSON',
        'cleanup_processes_after.txt',
    ]
    for phrase in required_artifacts:
        assert phrase in text


def test_phase56_report_exists_and_preserves_scope():
    text = REPORT.read_text()
    assert '# Phase56: Post-Ingress Open Direction to Candidate Formation Diagnostics' in text
    assert any(classification in text for classification in ALLOWED_CLASSIFICATIONS)
    assert 'does not claim autonomous exploration success' in text
    assert 'first dispatch is not exit success' in text
    assert 'max_goals=1' in text
    assert 'near_exit_fallback_enabled=false' in text
    assert 'startup_warmup_no_dispatch=false' in text
    assert 'Nav2 config diff: empty' in text
