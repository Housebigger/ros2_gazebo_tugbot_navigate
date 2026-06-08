from pathlib import Path
import importlib.util
import json

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase75_goal2_timeout_after_directional_redispatch_diagnosis.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase75_goal2_timeout_after_directional_redispatch_diagnosis_report.md'
PHASE74_ARTIFACT = ROOT / 'log' / 'phase74_directional_local_costmap_readiness_gate_validation' / 'replay_02'
PHASE74_SUMMARY = ROOT / 'log' / 'phase74_directional_local_costmap_readiness_gate_validation' / 'phase74_directional_local_costmap_readiness_gate_validation.json'

ALLOWED_CLASSIFICATIONS = {
    'GOAL2_TIMEOUT_TARGET_TOO_CLOSE_TO_WALL',
    'GOAL2_TIMEOUT_NEAR_GOAL_TOLERANCE_ORIENTATION',
    'GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP',
    'GOAL2_TIMEOUT_PROGRESS_INSUFFICIENT',
    'INSUFFICIENT_EVIDENCE',
}


def _load_analyzer():
    spec = importlib.util.spec_from_file_location('phase75_analyzer', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_jsonl(path: Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps(row, sort_keys=True) for row in rows) + '\n', encoding='utf-8')


def _event(event, seq=2, wall_time=10.0, elapsed=10.0, **extra):
    payload = {
        'event': event,
        'goal_sequence': seq,
        'target': [2.0, 1.0],
        'original_target': [2.0, 1.0],
        'refined_target': [2.0, 1.0],
        'dispatch_pose': [2.3, 0.0, 0.0],
        'branch_angle': 1.57,
        'target_clearance_m': 0.85,
        'dispatch_target_local_cost': 0,
        'dispatch_target_local_cost_max_radius': 46,
        'dispatch_path_local_cost_max': 46,
        'dispatch_path_local_cost_mean': 17.0,
        'path_corridor_min_clearance_m': 0.65,
        'effective_timeout_sec': 45.0,
        'phase62_timestamp_consistency': {
            'all_available': True,
            'map_age_sec': 0.9,
            'scan_age_sec': 0.096,
            'local_costmap_age_sec': 0.207,
            'max_age_sec': 0.9,
        },
    }
    payload.update(extra)
    return {'state': payload, 'wall_time': wall_time, 'elapsed_sec': elapsed}


def _make_artifact(tmp_path: Path, *, target_clearance=0.85, recovery_max=3, final_distance=0.36, progress_failures=2):
    _write_jsonl(
        tmp_path / 'phase74_directional_local_costmap_readiness_gate_validation_replay_02_goal_events.jsonl',
        [
            _event('dispatch', wall_time=10.0, elapsed=10.0, target_clearance_m=target_clearance),
            _event(
                'timeout',
                wall_time=55.0,
                elapsed=55.0,
                target_clearance_m=target_clearance,
                result_reason='goal_timeout',
                timeout_front_wedge_clearance_m=0.05,
                timeout_front_wedge_cost_max=99,
                timeout_footprint_cost_max=99,
                timeout_footprint_lethal_cell_count=48,
                timeout_local_cost_sample_age_sec=0.01,
                timeout_robot_in_local_costmap_bounds=True,
            ),
        ],
    )
    _write_jsonl(
        tmp_path / 'phase74_directional_local_costmap_readiness_gate_validation_replay_02_explorer_state.jsonl',
        [
            {'state': {'mode': 'NAVIGATING', 'goal_count': 2, 'goal_success_count': 1, 'goal_failure_count': 0, 'active_goal_sequence_id': 2, 'active_goal_target': [2.0, 1.0], 'effective_goal_timeout_sec': 45.0}, 'wall_time': 10.0, 'elapsed_sec': 10.0},
            {'state': {'mode': 'SETTLING', 'goal_count': 2, 'goal_success_count': 1, 'goal_failure_count': 1, 'last_failure_reason': 'goal_timeout', 'last_nav2_status': 4}, 'wall_time': 56.0, 'elapsed_sec': 56.0},
        ],
    )
    _write_jsonl(
        tmp_path / 'phase74_controller_dynamics.jsonl',
        [
            {'source': 'odom', 'goal_sequence': 2, 'wall_time': 10.0, 'elapsed_sec': 10.0, 'x': 2.3, 'y': 0.0, 'yaw': 0.0},
            {'source': 'odom', 'goal_sequence': 2, 'wall_time': 45.0, 'elapsed_sec': 45.0, 'x': 2.36, 'y': 0.98, 'yaw': 1.56},
            {'source': 'odom', 'goal_sequence': 2, 'wall_time': 55.0, 'elapsed_sec': 55.0, 'x': 2.36, 'y': 0.98, 'yaw': 1.56},
            {'source': 'cmd_vel', 'goal_sequence': 2, 'wall_time': 10.0, 'elapsed_sec': 10.0, 'linear_x': 0.12, 'angular_z': 0.03},
            {'source': 'cmd_vel', 'goal_sequence': 2, 'wall_time': 45.0, 'elapsed_sec': 45.0, 'linear_x': 0.0, 'angular_z': 0.0},
            {'source': 'cmd_vel', 'goal_sequence': 2, 'wall_time': 55.0, 'elapsed_sec': 55.0, 'linear_x': 0.0, 'angular_z': 0.0},
        ],
    )
    _write_jsonl(
        tmp_path / 'phase74_nav2_feedback.jsonl',
        [
            {'event': 'nav2_feedback', 'goal_sequence': 2, 'wall_time': 10.0, 'elapsed_sec': 10.0, 'distance_remaining': 1.05, 'navigation_time_sec': 0.0, 'number_of_recoveries': 0},
            {'event': 'nav2_feedback', 'goal_sequence': 2, 'wall_time': 40.0, 'elapsed_sec': 40.0, 'distance_remaining': final_distance, 'navigation_time_sec': 30.0, 'number_of_recoveries': 1},
            {'event': 'nav2_feedback', 'goal_sequence': 2, 'wall_time': 55.0, 'elapsed_sec': 55.0, 'distance_remaining': final_distance, 'navigation_time_sec': 45.0, 'number_of_recoveries': recovery_max},
        ],
    )
    _write_jsonl(
        tmp_path / 'phase74_local_costmap_samples.jsonl',
        [
            {'event': 'local_costmap_sample', 'goal_sequence': 2, 'wall_time': 10.0, 'elapsed_sec': 10.0, 'dispatch_target': [2.0, 1.0], 'robot_pose': [2.3, 0.0, 0.0], 'front_wedge_clearance_m': 0.15, 'front_wedge_cost': {'max': 100, 'high_cost_count': 200, 'lethal_count': 100, 'sample_count': 272}, 'robot_footprint_cost': {'max': 99, 'high_cost_count': 40, 'lethal_count': 20, 'sample_count': 143}, 'target_footprint_cost': {'max': 56, 'high_cost_count': 0, 'lethal_count': 0, 'sample_count': 143}, 'local_costmap_target_evidence': {'available': True, 'target': [2.0, 1.0], 'value': 0, 'radius_cost_summary': {'max': 46, 'high_cost_count': 0, 'lethal_count': 0}}},
            {'event': 'local_costmap_sample', 'goal_sequence': 2, 'wall_time': 55.0, 'elapsed_sec': 55.0, 'dispatch_target': [2.0, 1.0], 'robot_pose': [2.36, 0.98, 1.56], 'front_wedge_clearance_m': 0.05, 'front_wedge_cost': {'max': 99, 'high_cost_count': 150, 'lethal_count': 60, 'sample_count': 272}, 'robot_footprint_cost': {'max': 99, 'high_cost_count': 78, 'lethal_count': 52, 'sample_count': 143}, 'target_footprint_cost': {'max': 63, 'high_cost_count': 0, 'lethal_count': 0, 'sample_count': 143}, 'local_costmap_target_evidence': {'available': True, 'target': [2.0, 1.0], 'value': 0, 'radius_cost_summary': {'max': 54, 'high_cost_count': 0, 'lethal_count': 0}}},
        ],
    )
    _write_jsonl(tmp_path / 'phase74_global_plan_samples.jsonl', [{'event': 'global_plan_update', 'goal_sequence': 2, 'wall_time': 10.0, 'elapsed_sec': 10.0, 'point_count': 42}, {'event': 'global_plan_update', 'goal_sequence': 2, 'wall_time': 55.0, 'elapsed_sec': 55.0, 'point_count': 13}])
    _write_jsonl(tmp_path / 'phase74_collision_monitor_state.jsonl', [])
    (tmp_path / 'phase74_directional_local_costmap_readiness_gate_validation_replay_02_launch.log').write_text('\n'.join(['[controller_server]: Failed to make progress'] * progress_failures + ['[controller_server]: [follow_path] [ActionServer] Aborting handle.', '[local_costmap.local_costmap]: Received request to clear entirely the local_costmap']), encoding='utf-8')
    return tmp_path


def test_phase75_analyzer_contract_and_scope_tokens():
    text = ANALYZER.read_text(encoding='utf-8')
    assert 'Phase75 Goal2 Timeout Diagnosis After Directional Re-dispatch' in text
    assert 'PHASE74_RUN_ID' in text and 'phase74_directional_local_costmap_readiness_gate_validation' in text
    for classification in ALLOWED_CLASSIFICATIONS:
        assert classification in text
    for required in [
        'goal2_original_target',
        'goal2_refined_target',
        'goal2_actual_target',
        'dispatch_pose',
        'trajectory_summary',
        'final_pose',
        'distance_remaining_summary',
        'goal_tolerance_band',
        'cmd_vel_summary',
        'nav2_recovery_summary',
        'nav2_result_summary',
        'local_cost_summary',
        'footprint_summary',
        'front_wedge_summary',
        'global_plan_summary',
        'collision_monitor_summary',
        'timestamp_age_summary',
    ]:
        assert required in text
    for forbidden in [
        'directional_local_costmap_readiness_override_enabled:=false',
        'clearance_radius_m:=',
        'robot_radius:=',
        'inflation_radius:=',
        'near_exit_fallback_enabled:=true',
    ]:
        assert forbidden not in text
    assert 'does not claim autonomous exploration success' in text
    assert 'does not claim exit success' in text


def test_phase75_synthetic_local_cost_recovery_loop_classification(tmp_path):
    analyzer = _load_analyzer()
    artifact = _make_artifact(tmp_path)
    summary = analyzer.analyze_artifact(artifact)
    assert summary['classification'] == 'GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP'
    assert summary['goal2_window']['outcome_event'] == 'timeout'
    assert summary['nav2_recovery_summary']['number_of_recoveries_max'] == 3
    assert summary['nav2_log_summary']['failed_to_make_progress_count'] == 2
    assert summary['front_wedge_summary']['max_lethal_count'] >= 60
    assert summary['footprint_summary']['max_lethal_count'] >= 52
    assert summary['target_wall_risk_summary']['target_too_close_to_wall'] is False
    assert summary['goal_tolerance_band']['xy_goal_tolerance_m'] == 0.25
    assert summary['complete_autonomous_success_claimed'] is False
    assert summary['exit_success_claimed'] is False


def test_phase75_target_too_close_has_priority_when_target_geometry_is_bad(tmp_path):
    analyzer = _load_analyzer()
    artifact = _make_artifact(tmp_path, target_clearance=0.12, recovery_max=0, progress_failures=0)
    summary = analyzer.analyze_artifact(artifact)
    assert summary['classification'] == 'GOAL2_TIMEOUT_TARGET_TOO_CLOSE_TO_WALL'
    assert summary['target_wall_risk_summary']['target_too_close_to_wall'] is True


def test_phase75_real_phase74_replay02_contract_if_artifacts_exist():
    if not PHASE74_ARTIFACT.exists():
        return
    analyzer = _load_analyzer()
    summary = analyzer.analyze_artifact(PHASE74_ARTIFACT, phase74_summary_path=PHASE74_SUMMARY)
    assert summary['source_phase74_classification'] == 'DIRECTIONAL_GATE_ENABLES_GOAL2_DISPATCH'
    assert summary['replay_id'] == 'replay_02'
    assert summary['goal2_window']['dispatch_observed'] is True
    assert summary['goal2_window']['outcome_event'] == 'timeout'
    assert summary['goal2_actual_target'] == summary['goal2_refined_target'] == summary['goal2_original_target']
    assert summary['target_wall_risk_summary']['target_too_close_to_wall'] is False
    assert summary['classification'] in ALLOWED_CLASSIFICATIONS


def test_phase75_report_exists_and_preserves_stop_scope():
    text = REPORT.read_text(encoding='utf-8')
    assert '# Phase75: Goal2 Timeout Diagnosis After Directional Re-dispatch' in text
    assert any(classification in text for classification in ALLOWED_CLASSIFICATIONS)
    assert 'Phase74 replay_02' in text
    assert 'does not claim autonomous exploration success' in text
    assert 'does not claim exit success' in text
    assert 'Phase76 not entered' in text
