from pathlib import Path
import importlib.util
import json

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase80_goal2_near_goal_lateral_residual.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase80_goal2_near_goal_forward_open_corridor_diagnostic_classification_report.md'
PHASE79_ARTIFACT = ROOT / 'log' / 'phase79_goal2_timeout_bounded_reproduction_handoff'

CLASSIFICATION = 'NEAR_GOAL_LATERAL_RESIDUAL_WITH_FORWARD_OPEN_CORRIDOR'
INSUFFICIENT = 'FORWARD_OPEN_EVIDENCE_INSUFFICIENT'


def _load_analyzer():
    spec = importlib.util.spec_from_file_location('phase80_analyzer', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_jsonl(path: Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps(row, sort_keys=True) for row in rows) + '\n', encoding='utf-8')


def _make_artifact(tmp_path: Path):
    target = [2.0, 1.0]
    final_pose = [2.36, 1.01, 1.57]
    _write_jsonl(
        tmp_path / 'phase79_goal2_timeout_bounded_reproduction_handoff_goal_events.jsonl',
        [
            {
                'event': 'dispatch',
                'goal_sequence': 2,
                'elapsed_sec': 9.4,
                'wall_time': 1009.4,
                'target': target,
                'original_target': target,
                'refined_target': target,
                'dispatch_pose': [2.3, 0.03, 0.02],
                'branch_angle': 1.59,
                'dispatch_target_local_cost': 0,
                'dispatch_target_local_cost_max_radius': 46,
                'path_corridor_min_clearance_m': 0.65,
                'phase62_timestamp_consistency': {
                    'all_available': True,
                    'map_age_sec': 1.2,
                    'scan_age_sec': 0.09,
                    'local_costmap_age_sec': 0.132,
                    'max_age_sec': 1.2,
                },
            },
            {
                'event': 'timeout',
                'goal_sequence': 2,
                'elapsed_sec': 55.4,
                'wall_time': 1055.4,
                'result_reason': 'goal_timeout',
                'target': target,
                'original_target': target,
                'refined_target': target,
                'dispatch_pose': [2.3, 0.03, 0.02],
                'branch_angle': 1.59,
                'timeout_front_wedge_cost_max': 99,
                'timeout_front_wedge_clearance_m': 0.04,
                'timeout_footprint_cost_max': 99,
                'timeout_footprint_lethal_cell_count': 42,
                'timeout_local_cost_sample_age_sec': 0.3,
                'timeout_robot_in_local_costmap_bounds': True,
                'phase62_timestamp_consistency': {
                    'all_available': True,
                    'map_age_sec': 1.2,
                    'scan_age_sec': 0.09,
                    'local_costmap_age_sec': 0.132,
                    'max_age_sec': 1.2,
                },
            },
        ],
    )
    _write_jsonl(
        tmp_path / 'phase79_goal2_timeout_bounded_reproduction_handoff_local_costmap_samples.jsonl',
        [
            {
                'event': 'local_costmap_sample',
                'goal_sequence': 2,
                'elapsed_sec': 55.2,
                'wall_time': 1055.2,
                'dispatch_target': target,
                'robot_pose': final_pose,
                'front_wedge_clearance_m': 0.05,
                'front_wedge_cost': {'max': 99, 'high_cost_count': 150, 'lethal_count': 60, 'sample_count': 272},
                'robot_footprint_cost': {'max': 99, 'high_cost_count': 78, 'lethal_count': 52, 'sample_count': 143},
                'target_footprint_cost': {'max': 54, 'high_cost_count': 0, 'lethal_count': 0, 'sample_count': 143},
                'local_costmap_target_evidence': {'available': True, 'value': 0, 'radius_cost_summary': {'max': 54, 'high_cost_count': 0, 'lethal_count': 0, 'sample_count': 111}},
            }
        ],
    )
    _write_jsonl(
        tmp_path / 'phase79_goal2_timeout_bounded_reproduction_handoff_nav2_feedback.jsonl',
        [
            {'event': 'nav2_feedback', 'goal_sequence': 2, 'elapsed_sec': 9.5, 'wall_time': 1009.5, 'distance_remaining': 1.0, 'number_of_recoveries': 0},
            {'event': 'nav2_feedback', 'goal_sequence': 2, 'elapsed_sec': 55.3, 'wall_time': 1055.3, 'distance_remaining': 0.36, 'number_of_recoveries': 3},
        ],
    )
    _write_jsonl(
        tmp_path / 'phase79_goal2_timeout_bounded_reproduction_handoff_controller_dynamics.jsonl',
        [
            {'source': 'odom', 'goal_sequence': 2, 'elapsed_sec': 9.2, 'wall_time': 1009.2, 'x': 2.3, 'y': 0.03, 'yaw': 0.02},
            {'source': 'odom', 'goal_sequence': 2, 'elapsed_sec': 55.5, 'wall_time': 1055.5, 'x': final_pose[0], 'y': final_pose[1], 'yaw': final_pose[2]},
        ],
    )
    return tmp_path


def test_phase80_analyzer_contract_and_guardrails():
    text = ANALYZER.read_text(encoding='utf-8')
    assert 'Phase80 Goal2 near-goal forward-open corridor diagnostic classification' in text
    assert CLASSIFICATION in text
    assert INSUFFICIENT in text
    for required in [
        'relative_target_body_frame',
        'forward_component_m',
        'lateral_component_m',
        'final_xy_error_m',
        'near_goal_outside_xy_tolerance',
        'recoveries_max',
        'front_wedge_cost_max',
        'forward_open_evidence_status',
        'no maze_explorer strategy change',
        'no branch scoring change',
        'no centerline gate change',
        'no directional readiness override change',
        'no fallback/terminal acceptance change',
        'No Nav2/MPPI/controller tuning',
        'does not claim autonomous exploration success',
        'does not claim exit success',
    ]:
        assert required in text
    for forbidden in [
        'inflation_radius:=',
        'robot_radius:=',
        'clearance_radius_m:=',
        'near_exit_fallback_enabled:=true',
        'directional_local_costmap_readiness_override_enabled:=false',
    ]:
        assert forbidden not in text


def test_phase80_synthetic_lateral_residual_geometry_with_insufficient_forward_open_evidence(tmp_path):
    analyzer = _load_analyzer()
    artifact = _make_artifact(tmp_path)
    summary = analyzer.analyze_artifact(artifact)
    assert summary['classification'] == CLASSIFICATION
    assert summary['goal_sequence'] == 2
    assert summary['target'] == [2.0, 1.0]
    assert summary['terminal_pose'] == [2.36, 1.01, 1.57]
    assert summary['final_xy_error_m'] > 0.35
    assert summary['near_goal_outside_xy_tolerance'] is True
    assert abs(summary['relative_target_body_frame']['forward_component_m']) < 0.05
    assert summary['relative_target_body_frame']['lateral_component_m'] > 0.30
    assert summary['relative_target_body_frame']['dominant_residual_axis'] == 'lateral'
    assert summary['recoveries_max'] == 3
    assert summary['front_wedge_cost_max'] == 99
    assert summary['forward_open_evidence_status'] == INSUFFICIENT
    assert 'no_scan_artifact' in summary['forward_open_evidence_gaps']
    assert summary['complete_autonomous_success_claimed'] is False
    assert summary['exit_success_claimed'] is False


def test_phase80_real_phase79_goal2_artifact_if_present():
    if not PHASE79_ARTIFACT.exists():
        return
    analyzer = _load_analyzer()
    summary = analyzer.analyze_artifact(PHASE79_ARTIFACT)
    assert summary['classification'] == CLASSIFICATION
    assert summary['goal_sequence'] == 2
    assert summary['trigger_event'] == 'timeout'
    assert 0.30 < summary['final_xy_error_m'] < 0.45
    assert summary['near_goal_outside_xy_tolerance'] is True
    assert abs(summary['relative_target_body_frame']['forward_component_m']) < 0.08
    assert abs(summary['relative_target_body_frame']['lateral_component_m']) > 0.30
    assert summary['recoveries_max'] == 3
    assert summary['front_wedge_cost_max'] >= 99
    assert summary['forward_open_evidence_status'] == INSUFFICIENT
    assert summary['source_artifact_dir'].endswith('phase79_goal2_timeout_bounded_reproduction_handoff')


def test_phase80_report_exists_and_stops_before_phase81():
    text = REPORT.read_text(encoding='utf-8')
    assert '# Phase80: Goal2 near-goal forward-open corridor diagnostic classification' in text
    assert CLASSIFICATION in text
    assert INSUFFICIENT in text
    assert 'near-goal lateral residual' in text
    assert 'not exit success' in text
    assert 'not fallback/terminal acceptance' in text
    assert 'not a Nav2 parameter final diagnosis' in text
    assert 'No maze_explorer strategy changed' in text
    assert 'No Nav2/MPPI/controller tuning' in text
    assert 'Phase81 not entered' in text
