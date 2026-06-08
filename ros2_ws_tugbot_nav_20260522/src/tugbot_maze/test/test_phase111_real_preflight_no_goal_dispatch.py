from __future__ import annotations

import importlib.util
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase111_real_preflight_no_goal_dispatch.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase111_real_preflight_no_goal_dispatch_report.md'


def _load_analyzer():
    assert ANALYZER.exists(), f'missing {ANALYZER}'
    spec = importlib.util.spec_from_file_location('phase111_real_preflight_analyzer', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _sample(*, elapsed_sec=0.0, failed_gates=None, failure_reasons_by_gate=None, lifecycle_state='active_confirmed'):
    failed_gates = [] if failed_gates is None else failed_gates
    failure_reasons_by_gate = {} if failure_reasons_by_gate is None else failure_reasons_by_gate
    lifecycle = {
        'state': lifecycle_state,
        'active_confirmed': lifecycle_state == 'active_confirmed',
        'inactive_confirmed': lifecycle_state == 'inactive_confirmed',
        'ambiguous': lifecycle_state == 'ambiguous',
        'query_error': lifecycle_state == 'query_error',
        'lifecycle_query': {'returncode': 0 if lifecycle_state in {'active_confirmed', 'inactive_confirmed'} else 1, 'state': 'active' if lifecycle_state == 'active_confirmed' else None},
        'node_graph': {'present': lifecycle_state in {'active_confirmed', 'ambiguous'}},
        'action_availability': {'available': lifecycle_state in {'active_confirmed', 'ambiguous'}},
    }
    return {
        'elapsed_sec': elapsed_sec,
        'phase': 'stable_window',
        'failed_gates': failed_gates,
        'failure_reasons_by_gate': failure_reasons_by_gate,
        'stable_window_elapsed_sec': elapsed_sec,
        'lifecycle_sources': {'controller_server': lifecycle, 'bt_navigator': lifecycle},
        'scan_transform_check': {'scan_available': True, 'scan_frame_id': 'lidar_link', 'target_frame': 'map', 'transform_available': True, 'stable': True},
        'map_base_tf_check': {'available': True, 'stable': True, 'sample_count': 1, 'latest_age_sec': 0.05},
        'map_base_tf_age_sec': 0.05,
        'map_odom_tf_age_sec': 0.05,
        'odom_base_tf_age_sec': 0.05,
    }


def _artifact(samples, *, passed=True, reject_reason=None, failed_gates=None):
    return {
        'ingress_preflight': {
            'evaluated': True,
            'passed': passed,
            'ingress_preflight_reject_reason': reject_reason,
            'last_specific_reject_reason': reject_reason,
            'failed_gates': [] if failed_gates is None else failed_gates,
            'ingress_goal_sent': False,
            'maze_explorer_started': False,
            'first_scan_seen': True,
            'first_scan_wait_elapsed_sec': 0.1,
            'raw_style_snapshot_cross_check': {'present': True, 'ambiguous': False, 'cross_check_contradictions': []},
            'controller_pose_check': {
                'lifecycle_sources': samples[-1].get('lifecycle_sources', {}) if samples else {},
                'navigate_to_pose_action_ready': True,
            },
            'sample_count': len(samples),
            'sample_history': samples,
        }
    }


def test_phase111_analyzer_rejects_goal_dispatch_or_maze_explorer(tmp_path):
    module = _load_analyzer()
    artifact = _artifact([_sample()])
    artifact['ingress_preflight']['ingress_goal_sent'] = True
    artifact_path = tmp_path / 'phase111_real_preflight_no_goal_dispatch_ingress_preflight.json'
    artifact_path.write_text(json.dumps(artifact), encoding='utf-8')

    analysis = module.analyze_artifact_dir(tmp_path, run_id='phase111_real_preflight_no_goal_dispatch')

    assert analysis['guardrails']['ingress_goal_sent'] is True
    assert analysis['guardrails']['no_goal_dispatch_guard_valid'] is False
    assert analysis['classification'] == 'PHASE111_GUARD_VIOLATION_GOAL_OR_EXPLORER_STARTED'


def test_phase111_analyzer_extracts_lifecycle_scan_tf_raw_snapshot_and_launch_consistency(tmp_path):
    module = _load_analyzer()
    samples = [
        _sample(elapsed_sec=0.0, failed_gates=['waiting_for_first_scan'], failure_reasons_by_gate={'scan': ['waiting_for_first_scan']}),
        _sample(elapsed_sec=0.6, failed_gates=['ingress_map_base_tf_missing'], failure_reasons_by_gate={'tf': ['map_base_tf_missing']}),
        _sample(elapsed_sec=1.8),
    ]
    artifact_path = tmp_path / 'phase111_real_preflight_no_goal_dispatch_ingress_preflight.json'
    artifact_path.write_text(json.dumps(_artifact(samples, passed=True)), encoding='utf-8')
    (tmp_path / 'phase111_real_preflight_no_goal_dispatch_launch.log').write_text(
        'controller_server lifecycle node launched\nManaged nodes are active\nbt_navigator lifecycle node launched\n',
        encoding='utf-8',
    )

    analysis = module.analyze_artifact_dir(tmp_path, run_id='phase111_real_preflight_no_goal_dispatch')

    assert analysis['classification'] == 'PHASE111_PREFLIGHT_PASS_NO_GOAL_DISPATCH'
    assert analysis['preflight']['passed'] is True
    assert analysis['lifecycle']['controller_server']['derived_state'] == 'active_confirmed'
    assert analysis['lifecycle']['bt_navigator']['derived_state'] == 'active_confirmed'
    assert analysis['scan']['waiting_for_first_scan_observed'] is True
    assert analysis['scan']['first_scan_seen'] is True
    assert analysis['tf']['early_tf_miss_recovered'] is True
    assert analysis['raw_style_snapshot_cross_check']['present'] is True
    assert analysis['launch_log']['managed_nav2_active_marker_seen'] is True
    assert analysis['launch_log']['artifact_consistent_with_active_marker'] is True
    assert analysis['sample_history_contract']['all_samples_have_failed_gates'] is True
    assert analysis['sample_history_contract']['all_samples_have_failure_reasons_by_gate'] is True


def test_phase111_analyzer_classifies_fail_closed_tokens(tmp_path):
    module = _load_analyzer()
    samples = [
        _sample(elapsed_sec=0.0, failed_gates=['waiting_for_first_scan'], failure_reasons_by_gate={'scan': ['waiting_for_first_scan']}),
        _sample(elapsed_sec=2.5, failed_gates=['ingress_first_scan_timeout'], failure_reasons_by_gate={'scan': ['first_scan_timeout']}),
    ]
    artifact = _artifact(samples, passed=False, reject_reason='ingress_first_scan_timeout', failed_gates=['ingress_first_scan_timeout'])
    artifact['ingress_preflight']['first_scan_seen'] = False
    artifact_path = tmp_path / 'phase111_real_preflight_no_goal_dispatch_ingress_preflight.json'
    artifact_path.write_text(json.dumps(artifact), encoding='utf-8')

    analysis = module.analyze_artifact_dir(tmp_path, run_id='phase111_real_preflight_no_goal_dispatch')

    assert analysis['classification'] == 'PHASE111_PREFLIGHT_REJECTED_NO_GOAL_DISPATCH'
    assert analysis['preflight']['reject_reason'] == 'ingress_first_scan_timeout'
    assert analysis['scan']['waiting_for_first_scan_observed'] is True
    assert analysis['scan']['first_scan_timeout'] is True


def test_phase111_cli_writes_json_and_human_readable_summary(tmp_path):
    artifact_path = tmp_path / 'phase111_real_preflight_no_goal_dispatch_ingress_preflight.json'
    artifact_path.write_text(json.dumps(_artifact([_sample()])), encoding='utf-8')
    output_json = tmp_path / 'analysis.json'
    summary = tmp_path / 'summary.md'
    proc = subprocess.run(
        [
            'python3', str(ANALYZER),
            '--artifact-dir', str(tmp_path),
            '--run-id', 'phase111_real_preflight_no_goal_dispatch',
            '--output-json', str(output_json),
            '--minimal-summary-output', str(summary),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr
    data = json.loads(output_json.read_text(encoding='utf-8'))
    text = summary.read_text(encoding='utf-8')
    assert data['guardrails']['no_goal_dispatch_guard_valid'] is True
    assert 'Phase111 real preflight no-goal summary' in text
    assert 'ingress_goal_sent: False' in text
    assert 'maze_explorer_started: False' in text
    assert 'classification:' in text


def test_phase111_report_guardrails_present_after_completion():
    assert REPORT.exists(), f'missing {REPORT}'
    text = REPORT.read_text(encoding='utf-8')
    for phrase in [
        'PHASE111_REAL_PREFLIGHT_NO_GOAL_DISPATCH_COMPLETE_STOP_BEFORE_PHASE112',
        'No NavigateToPose goal was sent',
        'ingress_goal_sent=false',
        'maze_explorer_started=false',
        'No maze_explorer was started',
        'No Nav2/MPPI/controller/config tuning was performed',
        'does not prove autonomous exploration success',
        'Phase112 not entered',
    ]:
        assert phrase in text
