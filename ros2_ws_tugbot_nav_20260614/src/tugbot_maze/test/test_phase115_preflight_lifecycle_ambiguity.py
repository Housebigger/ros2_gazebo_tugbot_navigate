from __future__ import annotations

import importlib.util
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PREFLIGHT = ROOT / 'tools' / 'phase105_inner_ingress_tf_controller_preflight.py'
ANALYZER = ROOT / 'tools' / 'analyze_phase115_preflight_lifecycle_ambiguity.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase115_preflight_lifecycle_ambiguity_report.md'


def _load(path: Path, name: str):
    assert path.exists(), f'missing {path}'
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _timeout_query(node: str = '/controller_server') -> dict:
    return {
        'command': ['ros2', 'lifecycle', 'get', node],
        'timeout_sec': 0.25,
        'duration_sec': 0.251,
        'returncode': None,
        'state': None,
        'stdout': '',
        'stderr': 'timeout after 0.25 sec',
        'query_error': 'timeout after 0.25 sec',
        'timed_out': True,
    }


def _active_multisource(name: str) -> dict:
    return {
        'state': 'active_confirmed',
        'active_confirmed': True,
        'inactive_confirmed': False,
        'ambiguous': False,
        'query_error': True,
        'active_confirmed_by': 'multi_source_query_timeout_override',
        'ambiguous_detail': 'lifecycle subprocess timed out but graph/action/service/launch-active marker agree',
        'lifecycle_query': _timeout_query(f'/{name}'),
        'node_graph': {'present': True},
        'action_availability': {'available': True},
        'lifecycle_service': {'present': True, 'services': [f'/{name}/get_state']},
        'launch_active_marker': {'present': True, 'source': 'phase115_launch.log', 'token': 'Managed nodes are active'},
    }


def _phase115_artifact(*, passed: bool = True, active: bool = True, no_goal: bool = True) -> dict:
    lifecycle_sources = {
        'controller_server': _active_multisource('controller_server') if active else {
            **_active_multisource('controller_server'),
            'state': 'ambiguous',
            'active_confirmed': False,
            'ambiguous': True,
            'active_confirmed_by': None,
        },
        'bt_navigator': _active_multisource('bt_navigator') if active else {
            **_active_multisource('bt_navigator'),
            'state': 'ambiguous',
            'active_confirmed': False,
            'ambiguous': True,
            'active_confirmed_by': None,
        },
    }
    sample = {
        'elapsed_sec': 3.0,
        'phase': 'stable_window',
        'failed_gates': [] if passed else ['ingress_lifecycle_ambiguous'],
        'failure_reasons_by_gate': {} if passed else {'lifecycle': ['lifecycle_ambiguous']},
        'lifecycle_sources': lifecycle_sources,
        'stable_window_elapsed_sec': 2.2,
        'map_base_tf_check': {'available': True, 'stable': True, 'sample_count': 1, 'latest_age_sec': 0.02},
        'map_base_tf_age_sec': 0.02,
        'map_odom_tf_age_sec': 0.0,
        'odom_base_tf_age_sec': 0.02,
        'scan_transform_check': {
            'scan_available': True,
            'scan_frame_id': 'tugbot/scan_omni/scan_omni',
            'target_frame': 'map',
            'transform_available': True,
            'stable': True,
            'cache_drop_detected': False,
            'exception_count': 0,
        },
        'controller_pose_check': {
            'controller_server_active': active,
            'bt_navigator_active': active,
            'navigate_to_pose_action_ready': True,
            'robot_pose_available': True,
            'robot_pose_unavailable_log_count': 0,
            'lifecycle_sources': lifecycle_sources,
        },
        'goal_pose_transform_check': {
            'goal_frame_id': 'map',
            'global_costmap_frame': 'map',
            'controller_frame': 'odom',
            'transform_to_global_costmap_available': True,
            'transform_to_controller_frame_available': True,
            'exception_count': 0,
        },
        'tf_detector': {
            'tf_jump_count': 0,
            'cache_drop_count': 0,
            'robot_pose_unavailable_count': 0,
            'goal_pose_transform_failure_count': 0,
        },
        'internal_sampling_diagnostics': {
            'lifecycle': lifecycle_sources,
        },
    }
    return {
        'ingress_preflight': {
            'evaluated': True,
            'passed': passed,
            'ingress_preflight_reject_reason': None if passed else 'ingress_lifecycle_ambiguous',
            'failed_gates': [] if passed else ['ingress_lifecycle_ambiguous'],
            'ingress_goal_sent': not no_goal,
            'maze_explorer_started': False,
            'sample_count': 1,
            'sample_history': [sample],
        }
    }


def test_phase115_lifecycle_query_records_command_timeout_streams_returncode_and_duration(monkeypatch):
    preflight = _load(PREFLIGHT, 'phase115_preflight_query_diag')

    def fake_run(*args, **kwargs):
        raise subprocess.TimeoutExpired(cmd=args[0], timeout=kwargs.get('timeout'), output='partial out', stderr='partial err')

    monkeypatch.setattr(preflight.subprocess, 'run', fake_run)
    query = preflight._ros_lifecycle_query('/controller_server', timeout_sec=0.25)

    assert query['command'] == ['ros2', 'lifecycle', 'get', '/controller_server']
    assert query['timeout_sec'] == 0.25
    assert query['duration_sec'] >= 0.0
    assert query['returncode'] is None
    assert query['stdout'] == 'partial out'
    assert query['stderr'] == 'partial err'
    assert query['timed_out'] is True
    assert 'timeout' in query['query_error']


def test_phase115_multisource_timeout_override_requires_all_fail_closed_sources():
    preflight = _load(PREFLIGHT, 'phase115_preflight_multisource')
    query = _timeout_query('/controller_server')

    confirmed = preflight.derive_lifecycle_confirmation(
        lifecycle_query=query,
        node_graph_present=True,
        action_available=True,
        lifecycle_service_present=True,
        launch_active_marker_present=True,
        launch_active_marker_token='Managed nodes are active',
    )
    assert confirmed['state'] == 'active_confirmed'
    assert confirmed['active_confirmed'] is True
    assert confirmed['active_confirmed_by'] == 'multi_source_query_timeout_override'
    assert confirmed['query_error'] is True
    assert confirmed['lifecycle_query']['timed_out'] is True
    assert 'ambiguous_detail' in confirmed

    missing_marker = preflight.derive_lifecycle_confirmation(
        lifecycle_query=query,
        node_graph_present=True,
        action_available=True,
        lifecycle_service_present=True,
        launch_active_marker_present=False,
    )
    assert missing_marker['state'] == 'ambiguous'
    assert missing_marker['active_confirmed'] is False
    assert missing_marker['ambiguous'] is True


def test_phase115_multisource_active_confirmation_clears_lifecycle_gate_without_goal_dispatch():
    preflight = _load(PREFLIGHT, 'phase115_preflight_gate')
    lifecycle_sources = {
        'controller_server': _active_multisource('controller_server'),
        'bt_navigator': _active_multisource('bt_navigator'),
    }
    sample = preflight.make_synthetic_sample(
        elapsed_sec=3.0,
        controller_pose_check={
            'controller_server_active': False,
            'bt_navigator_active': False,
            'navigate_to_pose_action_ready': True,
            'robot_pose_available': True,
            'robot_pose_unavailable_log_count': 0,
            'lifecycle_sources': lifecycle_sources,
        },
    )

    result = preflight.evaluate_preflight_samples([sample], preflight.PreflightConfig(tf_stability_window_sec=1.0))
    artifact = result.to_artifact()['ingress_preflight']

    assert 'ingress_lifecycle_ambiguous' not in artifact['failed_gates']
    assert artifact['controller_pose_check']['controller_server_active'] is True
    assert artifact['controller_pose_check']['bt_navigator_active'] is True
    assert artifact['ingress_goal_sent'] is False
    assert artifact['maze_explorer_started'] is False


def test_phase115_analyzer_classifies_multisource_timeout_override_no_goal():
    analyzer = _load(ANALYZER, 'phase115_analyzer')
    analysis = analyzer.analyze_artifact(_phase115_artifact(passed=True, active=True, no_goal=True))

    assert analysis['classification'] == 'LIFECYCLE_AMBIGUITY_RESOLVED_PREFLIGHT_PASSED_NO_GOAL'
    assert analysis['lifecycle']['controller_server']['active_confirmed'] is True
    assert analysis['lifecycle']['bt_navigator']['active_confirmed'] is True
    assert analysis['lifecycle']['controller_server']['query_timed_out'] is True
    assert analysis['guardrails']['no_goal_dispatch_guard_valid'] is True


def test_phase115_analyzer_keeps_missing_multisource_evidence_fail_closed():
    analyzer = _load(ANALYZER, 'phase115_analyzer_missing_sources')
    analysis = analyzer.analyze_artifact(_phase115_artifact(passed=False, active=False, no_goal=True))

    assert analysis['classification'] == 'LIFECYCLE_QUERY_TIMEOUT_STILL_AMBIGUOUS_FAIL_CLOSED'
    assert analysis['preflight']['passed'] is False
    assert 'ingress_lifecycle_ambiguous' in analysis['preflight']['failed_gates']
    assert analysis['guardrails']['no_goal_dispatch_guard_valid'] is True


def test_phase115_cli_writes_analysis_and_summary(tmp_path):
    artifact_path = tmp_path / 'artifact.json'
    analysis_path = tmp_path / 'analysis.json'
    summary_path = tmp_path / 'summary.md'
    artifact_path.write_text(json.dumps(_phase115_artifact(passed=True, active=True), indent=2), encoding='utf-8')

    proc = subprocess.run([
        'python3', str(ANALYZER),
        '--artifact', str(artifact_path),
        '--output-json', str(analysis_path),
        '--minimal-summary-output', str(summary_path),
    ], cwd=ROOT, text=True, capture_output=True, check=False)

    assert proc.returncode == 0, proc.stderr
    analysis = json.loads(analysis_path.read_text(encoding='utf-8'))
    assert analysis['classification'] == 'LIFECYCLE_AMBIGUITY_RESOLVED_PREFLIGHT_PASSED_NO_GOAL'
    summary = summary_path.read_text(encoding='utf-8')
    assert 'Phase115 lifecycle ambiguity summary' in summary
    assert 'No NavigateToPose goal was sent' in summary
    assert 'Phase116 not entered' in summary


def test_phase115_report_guardrails_present_after_completion():
    assert REPORT.exists(), f'missing {REPORT}'
    text = REPORT.read_text(encoding='utf-8')
    for phrase in [
        'PHASE115_PREFLIGHT_LIFECYCLE_AMBIGUITY_COMPLETE_STOP_BEFORE_PHASE116',
        'No NavigateToPose goal was sent',
        'ingress_goal_sent=false',
        'maze_explorer_started=false',
        'No maze_explorer was started',
        'No Nav2/MPPI/controller/config tuning was performed',
        'does not prove autonomous exploration success',
        'Phase116 not entered',
    ]:
        assert phrase in text
