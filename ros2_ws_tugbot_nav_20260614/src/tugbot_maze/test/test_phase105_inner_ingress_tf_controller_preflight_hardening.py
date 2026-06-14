from __future__ import annotations

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PREFLIGHT = ROOT / 'tools' / 'phase105_inner_ingress_tf_controller_preflight.py'
WRAPPER = ROOT / 'tools' / 'run_phase102_carry_over_bounded_goal1_staging_validation.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase102_carry_over_bounded_goal1_staging_validation.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase105_inner_ingress_tf_controller_preflight_hardening_minimal_implementation_report.md'


def _load_preflight():
    assert PREFLIGHT.exists(), f'missing {PREFLIGHT}'
    spec = importlib.util.spec_from_file_location('phase105_preflight', PREFLIGHT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_analyzer():
    spec = importlib.util.spec_from_file_location('phase102_analyzer_for_phase105', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _sample(module, **overrides):
    sample = module.make_synthetic_sample()
    sample.update(overrides)
    return sample


def test_phase105_reject_tokens_and_contract_are_complete():
    module = _load_preflight()
    assert module.REJECT_TOKENS == {
        'ingress_tf_unstable',
        'ingress_map_base_tf_missing',
        'ingress_map_odom_tf_stale',
        'ingress_odom_base_tf_stale',
        'ingress_scan_transform_unstable',
        'ingress_controller_robot_pose_unavailable',
        'ingress_goal_pose_transform_unavailable',
        'ingress_lifecycle_ambiguous',
        'ingress_first_scan_timeout',
        'ingress_tf_stable_window_not_met',
        'ingress_raw_snapshot_cross_check_failed',
        'ingress_preflight_timeout',
    }
    assert module.INNER_INGRESS_GOAL_POSE == {
        'frame_id': 'map',
        'x_m': 2.0,
        'y_m': 0.0,
        'yaw_rad': 0.0,
    }


def test_phase105_preflight_pass_preserves_inner_ingress_goal_and_artifact_contract(tmp_path):
    module = _load_preflight()
    result = module.evaluate_preflight_samples(
        [_sample(module, elapsed_sec=0.0), _sample(module, elapsed_sec=1.0), _sample(module, elapsed_sec=2.0)],
        module.PreflightConfig(tf_stability_window_sec=1.5, timeout_sec=5.0),
    )
    artifact = result.to_artifact()
    assert artifact['ingress_preflight']['passed'] is True
    assert artifact['ingress_preflight']['ingress_preflight_reject_reason'] is None
    assert artifact['ingress_preflight']['inner_ingress_goal_pose'] == module.INNER_INGRESS_GOAL_POSE
    assert artifact['ingress_preflight']['ingress_goal_sent'] is False  # wrapper flips only after sender is called
    for key in [
        'tf_stability_window_sec',
        'tf_jump_count',
        'scan_transform_check',
        'controller_pose_check',
        'goal_pose_transform_check',
        'map_base_tf_age_sec',
        'map_odom_tf_age_sec',
        'odom_base_tf_age_sec',
        'bounded_wait_elapsed_sec',
    ]:
        assert key in artifact['ingress_preflight']
    out = tmp_path / 'ingress_preflight.json'
    module.write_preflight_artifact(result, out)
    loaded = json.loads(out.read_text())
    assert loaded['ingress_preflight']['passed'] is True


def test_phase105_preflight_reject_cases_fail_closed_and_cover_tokens():
    module = _load_preflight()
    cases = {
        'ingress_map_base_tf_missing': {'map_base_tf_check': {'available': False, 'stable': False, 'sample_count': 1, 'latest_age_sec': None}},
        'ingress_tf_unstable': {'tf_jump_detected': True, 'tf_jump_count': 1},
        'ingress_map_odom_tf_stale': {'map_odom_tf_age_sec': 3.0},
        'ingress_odom_base_tf_stale': {'odom_base_tf_age_sec': 3.0},
        'ingress_scan_transform_unstable': {'scan_transform_check': {'scan_available': True, 'scan_frame_id': 'lidar', 'target_frame': 'map', 'transform_available': False, 'stable': False, 'cache_drop_detected': True, 'exception_count': 1}},
        'ingress_controller_robot_pose_unavailable': {'controller_pose_check': {'controller_server_active': True, 'bt_navigator_active': True, 'navigate_to_pose_action_ready': True, 'robot_pose_available': False, 'robot_pose_unavailable_log_count': 1}},
        'ingress_goal_pose_transform_unavailable': {'goal_pose_transform_check': {'goal_frame_id': 'map', 'global_costmap_frame': 'map', 'controller_frame': 'odom', 'transform_to_global_costmap_available': True, 'transform_to_controller_frame_available': False, 'exception_count': 1}},
        'ingress_preflight_timeout': {'force_timeout': True},
    }
    for expected, overrides in cases.items():
        samples = [_sample(module, elapsed_sec=0.0, **overrides)]
        result = module.evaluate_preflight_samples(samples, module.PreflightConfig(tf_stability_window_sec=1.0, timeout_sec=1.0))
        artifact = result.to_artifact()['ingress_preflight']
        assert artifact['passed'] is False, expected
        assert artifact['ingress_preflight_reject_reason'] == expected
        assert artifact['ingress_goal_sent'] is False
        assert artifact['maze_explorer_started'] is False
        assert expected in artifact['failed_gates']


def test_phase105_wrapper_runs_preflight_before_sender_and_blocks_explorer_on_reject():
    text = WRAPPER.read_text()
    assert 'INGRESS_PREFLIGHT_JSON=' in text
    assert 'phase105_inner_ingress_tf_controller_preflight.py' in text
    assert 'run_ingress_preflight' in text
    assert 'ingress_preflight_rejected_explorer_not_started' in text
    preflight_index = text.index('run_ingress_preflight')
    send_index = text.index('send_ingress_goal')
    assert preflight_index < send_index
    assert 'if run_ingress_preflight; then' in text
    guarded = text[text.index('if run_ingress_preflight; then'): text.index('run_analyzer >')]
    assert 'send_ingress_goal' in guarded
    assert 'start_explorer' in guarded
    assert 'mark_ingress_preflight_wrapper_state true false' in guarded
    assert 'mark_ingress_preflight_wrapper_state true true' in guarded
    reject_block = guarded.split('else', 1)[1]
    assert 'send_ingress_goal' not in reject_block
    assert 'start_explorer' not in reject_block
    forbidden_tuning = ['controller_server:', 'inflation_radius:', 'robot_radius:', 'clearance_radius_m:=']
    for token in forbidden_tuning:
        assert token not in text


def test_phase105_analyzer_surfaces_preflight_reject_as_insufficient_evidence(tmp_path):
    module = _load_preflight()
    analyzer = _load_analyzer()
    run_id = 'phase102_carry_over_bounded_goal1_staging_validation'
    artifact_dir = tmp_path / 'artifact'
    artifact_dir.mkdir()
    result = module.evaluate_preflight_samples(
        [_sample(module, elapsed_sec=0.0, map_base_tf_check={'available': False, 'stable': False, 'sample_count': 1, 'latest_age_sec': None})],
        module.PreflightConfig(tf_stability_window_sec=1.0, timeout_sec=1.0),
    )
    module.write_preflight_artifact(result, artifact_dir / f'{run_id}_ingress_preflight.json')
    (artifact_dir / f'{run_id}_ingress_result.json').write_text(json.dumps({
        'success': False,
        'status': 'preflight_rejected',
        'ingress_goal_sent': False,
        'maze_explorer_started': False,
        'ingress_preflight_reject_reason': 'ingress_map_base_tf_missing',
    }))
    (artifact_dir / f'{run_id}_trigger_detected.json').write_text(json.dumps({
        'trigger': 'ingress_preflight_rejected_explorer_not_started',
        'ingress_preflight_reject_reason': 'ingress_map_base_tf_missing',
    }))
    analysis = analyzer.analyze_artifact_dir(artifact_dir, run_id=run_id)
    assert analysis['classification'] == 'GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE'
    assert 'ingress_preflight_rejected_explorer_not_started' in analysis['evidence_gaps']
    assert analysis['ingress_preflight']['passed'] is False
    assert analysis['ingress_preflight']['ingress_preflight_reject_reason'] == 'ingress_map_base_tf_missing'
    assert analysis['ingress']['ingress_goal_sent'] is False


def test_phase105_report_and_guardrails_present():
    assert REPORT.exists(), f'missing {REPORT}'
    text = REPORT.read_text()
    for token in [
        'PHASE105_IMPLEMENTED_STATIC_UNIT_VALIDATED_STOP_BEFORE_PHASE106',
        'ingress_preflight.passed=false',
        'ingress_goal_sent=false',
        'maze_explorer_started=false',
        'map, x=2.0, y=0.0, yaw=0.0',
        'No Phase101 carry-over change',
        'No Phase88/92 logic change',
        'No maze_explorer strategy change',
        'No Nav2/MPPI/controller tuning',
        'Phase106 not entered',
    ]:
        assert token in text
