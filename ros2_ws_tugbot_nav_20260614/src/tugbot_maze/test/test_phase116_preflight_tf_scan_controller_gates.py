from __future__ import annotations

import importlib.util
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PREFLIGHT = ROOT / 'tools' / 'phase105_inner_ingress_tf_controller_preflight.py'
ANALYZER = ROOT / 'tools' / 'analyze_phase116_preflight_tf_scan_controller_gates.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase116_preflight_tf_scan_controller_gates_report.md'


def _load(path: Path, name: str):
    assert path.exists(), f'missing {path}'
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _tf_diag(*, can: bool, lookup: bool, age: float | None = None, exc: str | None = None) -> dict:
    return {
        'parent': 'map',
        'child': 'base_link',
        'can_transform': can,
        'lookup_success': lookup,
        'can_transform_exception': exc,
        'lookup_exception': exc,
        'age_sec': age,
        'finite': lookup,
        'failure_reason': None if lookup else 'lookup_exception',
        'sample_wall_time_sec': 100.0,
        'sample_ros_time_sec': 10.0,
        'can_transform_duration_sec': 0.01,
        'lookup_duration_sec': 0.01,
        'buffer_frames': 'Frame base_link exists with parent odom',
    }


def _sample(elapsed: float, *, ok: bool, force_timeout: bool = False) -> dict:
    frame = 'tugbot/scan_omni/scan_omni' if ok else None
    tf_buffer = {
        'map->base_link': _tf_diag(can=ok, lookup=ok, age=0.02 if ok else None, exc=None if ok else '"map" passed to lookupTransform argument target_frame does not exist.'),
        'map->odom': _tf_diag(can=ok, lookup=ok, age=0.0 if ok else None, exc=None if ok else '"map" passed to lookupTransform argument target_frame does not exist.'),
        'odom->base_link': _tf_diag(can=ok, lookup=ok, age=0.02 if ok else None, exc=None if ok else '"odom" passed to lookupTransform argument target_frame does not exist.'),
    }
    scan_diag = _tf_diag(can=ok, lookup=ok, age=0.02 if ok else None, exc=None if ok else 'no_scan_frame')
    scan_diag['child'] = frame
    tf_buffer['map->tugbot/scan_omni/scan_omni' if ok else 'map-><no_scan_frame>'] = scan_diag
    sample = {
        'elapsed_sec': elapsed,
        'force_timeout': force_timeout,
        'map_base_tf_check': {'available': ok, 'stable': ok, 'sample_count': 1, 'latest_age_sec': 0.02 if ok else None},
        'map_base_tf_age_sec': 0.02 if ok else None,
        'map_odom_tf_age_sec': 0.0 if ok else None,
        'odom_base_tf_age_sec': 0.02 if ok else None,
        'scan_transform_check': {
            'scan_available': ok,
            'scan_frame_id': frame,
            'target_frame': 'map',
            'transform_available': ok,
            'stable': ok,
            'cache_drop_detected': False,
            'exception_count': 0 if ok else 1,
            'age_sec': 0.01 if ok else None,
            'transform_age_sec': 0.02 if ok else None,
            'failure_reason': None if ok else 'no_scan_frame',
        },
        'controller_pose_check': {
            'controller_server_active': True,
            'bt_navigator_active': True,
            'navigate_to_pose_action_ready': True,
            'robot_pose_available': ok,
            'robot_pose_unavailable_log_count': 0 if ok else 1,
            'robot_pose_unavailable_reason': None if ok else 'map_base_tf_missing',
        },
        'goal_pose_transform_check': {
            'goal_frame_id': 'map',
            'global_costmap_frame': 'map',
            'controller_frame': 'odom',
            'transform_to_global_costmap_available': True,
            'transform_to_controller_frame_available': True,
            'exception_count': 0,
        },
        'tf_detector': {'tf_jump_count': 0, 'cache_drop_count': 0, 'robot_pose_unavailable_count': 0 if ok else 1, 'goal_pose_transform_failure_count': 0},
        'internal_sampling_diagnostics': {
            'tf_buffer': tf_buffer,
            'tf_buffer_frame_list': 'Frame base_link exists with parent odom' if ok else '',
            'scan_subscription': {
                'topic': '/scan',
                'publisher_count': 1,
                'callback_count_total': 10 if ok else 0,
                'latest_frame_id': frame,
                'latest_age_sec': 0.01 if ok else None,
                'latest_stamp_sec': 10.0 if ok else None,
            },
            'controller_direct_reason': None if ok else 'map_base_tf_missing',
        },
    }
    return sample


def _artifact() -> dict:
    return {
        'ingress_preflight': {
            'evaluated': True,
            'passed': False,
            'ingress_preflight_reject_reason': 'ingress_preflight_timeout',
            'failed_gates': ['ingress_map_base_tf_missing', 'ingress_scan_transform_unstable', 'ingress_controller_robot_pose_unavailable', 'ingress_preflight_timeout'],
            'ingress_goal_sent': False,
            'maze_explorer_started': False,
            'sample_count': 2,
            'sample_history': [_sample(0.0, ok=False), _sample(19.0, ok=True, force_timeout=True)],
        }
    }


def test_phase116_runtime_tf_diag_contract_records_timestamps_frames_and_failure_reasons():
    preflight = _load(PREFLIGHT, 'phase116_preflight_contract')
    text = PREFLIGHT.read_text(encoding='utf-8')
    for token in [
        'sample_wall_time_sec',
        'sample_ros_time_sec',
        'can_transform_duration_sec',
        'lookup_duration_sec',
        'failure_reason',
        'tf_buffer_frame_list',
        'robot_pose_unavailable_reason',
        'scan_transform_failure_reason',
    ]:
        assert token in text
    assert hasattr(preflight, 'derive_controller_pose_unavailable_reason')


def test_phase116_controller_direct_reason_distinguishes_tf_and_action_causes():
    preflight = _load(PREFLIGHT, 'phase116_preflight_direct_reason')
    assert preflight.derive_controller_pose_unavailable_reason(
        map_base_diag={'lookup_success': False, 'lookup_exception': '"map" passed to lookupTransform argument target_frame does not exist.'},
        map_base_age_sec=None,
        robot_pose_available=False,
        controller_active=True,
        bt_active=True,
        action_ready=True,
        tf_max_age_sec=1.5,
    ) == 'map_base_tf_missing'
    assert preflight.derive_controller_pose_unavailable_reason(
        map_base_diag={'lookup_success': True, 'finite': True},
        map_base_age_sec=3.0,
        robot_pose_available=False,
        controller_active=True,
        bt_active=True,
        action_ready=True,
        tf_max_age_sec=1.5,
    ) == 'map_base_tf_stale'
    assert preflight.derive_controller_pose_unavailable_reason(
        map_base_diag={'lookup_success': True, 'finite': True},
        map_base_age_sec=0.1,
        robot_pose_available=True,
        controller_active=False,
        bt_active=True,
        action_ready=False,
        tf_max_age_sec=1.5,
    ) == 'controller_action_or_state_insufficient'


def test_phase116_analyzer_classifies_startup_tf_buffer_fill_delay_no_goal():
    analyzer = _load(ANALYZER, 'phase116_analyzer')
    analysis = analyzer.analyze_artifact(_artifact(), external_timeline={
        'map->base_link': [{'elapsed_sec': 0.0, 'available': False}, {'elapsed_sec': 5.0, 'available': True}],
        'map->odom': [{'elapsed_sec': 0.0, 'available': False}, {'elapsed_sec': 5.0, 'available': True}],
        'odom->base_link': [{'elapsed_sec': 0.0, 'available': False}, {'elapsed_sec': 5.0, 'available': True}],
    })
    assert analysis['classification'] == 'TF_SCAN_CONTROLLER_RECOVERED_AFTER_STARTUP_DELAY_FAIL_CLOSED_TIMEOUT'
    assert 'STARTUP_TF_BUFFER_FILL_DELAY' in analysis['findings']
    assert analysis['scan']['frame_id'] == 'tugbot/scan_omni/scan_omni'
    assert analysis['controller']['direct_reason_sequence'][0] == 'map_base_tf_missing'
    assert analysis['guardrails']['no_goal_dispatch_guard_valid'] is True


def test_phase116_cli_writes_analysis_and_summary(tmp_path):
    artifact_path = tmp_path / 'artifact.json'
    timeline_path = tmp_path / 'timeline.json'
    analysis_path = tmp_path / 'analysis.json'
    summary_path = tmp_path / 'summary.md'
    artifact_path.write_text(json.dumps(_artifact(), indent=2), encoding='utf-8')
    timeline_path.write_text(json.dumps({'map->base_link': [{'elapsed_sec': 0.0, 'available': False}, {'elapsed_sec': 4.0, 'available': True}]}), encoding='utf-8')
    proc = subprocess.run([
        'python3', str(ANALYZER),
        '--artifact', str(artifact_path),
        '--external-tf-timeline-json', str(timeline_path),
        '--output-json', str(analysis_path),
        '--minimal-summary-output', str(summary_path),
    ], cwd=ROOT, text=True, capture_output=True, check=False)
    assert proc.returncode == 0, proc.stderr
    analysis = json.loads(analysis_path.read_text(encoding='utf-8'))
    assert analysis['classification'] == 'TF_SCAN_CONTROLLER_RECOVERED_AFTER_STARTUP_DELAY_FAIL_CLOSED_TIMEOUT'
    summary = summary_path.read_text(encoding='utf-8')
    assert 'Phase116 TF/scan/controller gate summary' in summary
    assert 'No NavigateToPose goal was sent' in summary
    assert 'Phase117 not entered' in summary


def test_phase116_report_guardrails_present_after_completion():
    assert REPORT.exists(), f'missing {REPORT}'
    text = REPORT.read_text(encoding='utf-8')
    for phrase in [
        'PHASE116_PREFLIGHT_TF_SCAN_CONTROLLER_GATES_COMPLETE_STOP_BEFORE_PHASE117',
        'No NavigateToPose goal was sent',
        'ingress_goal_sent=false',
        'maze_explorer_started=false',
        'No maze_explorer was started',
        'No Nav2/MPPI/controller/config tuning was performed',
        'No autonomous exploration success or exit success is claimed',
        'Phase117 not entered',
    ]:
        assert phrase in text
