from __future__ import annotations

import importlib.util
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase113_preflight_internal_sampling_discrepancy.py'
PREFLIGHT = ROOT / 'tools' / 'phase105_inner_ingress_tf_controller_preflight.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase113_preflight_internal_sampling_discrepancy_report.md'


def _load(path: Path, name: str):
    assert path.exists(), f'missing {path}'
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _artifact(*, callback_count: int = 0, map_base_available: bool = False) -> dict:
    samples = []
    for i in range(3):
        samples.append({
            'elapsed_sec': float(i),
            'phase': 'stable_window',
            'failed_gates': ['ingress_first_scan_timeout', 'ingress_map_base_tf_missing'],
            'failure_reasons_by_gate': {'scan': ['first_scan_timeout'], 'tf': ['map_base_tf_missing']},
            'lifecycle_sources': {},
            'scan_transform_check': {
                'scan_available': False,
                'scan_frame_id': 'tugbot/scan_omni/scan_omni' if callback_count else None,
                'target_frame': 'map',
                'transform_available': False,
                'stable': False,
            },
            'map_base_tf_check': {'available': map_base_available, 'stable': map_base_available, 'sample_count': 1, 'latest_age_sec': None},
            'map_base_tf_age_sec': None,
            'map_odom_tf_age_sec': None,
            'odom_base_tf_age_sec': None,
            'stable_window_elapsed_sec': 0.0,
            'internal_sampling_diagnostics': {
                'sample_index': i,
                'executor': {'spin_once_count_total': i + 1, 'last_spin_timeout_sec': 0.05},
                'clock': {'use_sim_time': True, 'ros_time_sec': 10.0 + i, 'wall_time_sec': 100.0 + i},
                'scan_subscription': {
                    'topic': '/scan',
                    'publisher_count': 1,
                    'subscriber_count': 1,
                    'callback_count_total': callback_count,
                    'callbacks_since_last_sample': 0 if callback_count == 0 else 1,
                    'latest_frame_id': 'tugbot/scan_omni/scan_omni' if callback_count else None,
                    'latest_stamp_sec': 9.5 if callback_count else None,
                    'latest_age_sec': 0.5 if callback_count else None,
                    'latest_received_wall_time_sec': 100.0 + i if callback_count else None,
                },
                'tf_buffer': {
                    'map->base_link': {
                        'can_transform': map_base_available,
                        'lookup_success': map_base_available,
                        'lookup_exception': None if map_base_available else 'Lookup would require extrapolation into the past',
                    },
                    'map->tugbot/scan_omni/scan_omni': {
                        'can_transform': False,
                        'lookup_success': False,
                        'lookup_exception': 'target frame does not exist',
                    },
                },
            },
        })
    return {
        'ingress_preflight': {
            'evaluated': True,
            'passed': False,
            'ingress_preflight_reject_reason': 'ingress_first_scan_timeout',
            'failed_gates': ['ingress_first_scan_timeout', 'ingress_map_base_tf_missing'],
            'ingress_goal_sent': False,
            'maze_explorer_started': False,
            'sample_count': len(samples),
            'sample_history': samples,
        }
    }


def test_phase113_analyzer_classifies_internal_scan_callback_starvation_with_external_scan_and_tf(tmp_path):
    analyzer = _load(ANALYZER, 'phase113_analyzer')
    artifact = _artifact(callback_count=0, map_base_available=False)
    snapshot = '''
--- tf map base_link ---
Translation: [1.0, 2.0, 0.0]
--- scan once ---
header:
  frame_id: tugbot/scan_omni/scan_omni
ranges:
- 1.0
'''

    analysis = analyzer.analyze_artifact(artifact, external_snapshot_text=snapshot, launch_log_text='Managed nodes are active')

    assert analysis['classification'] == 'INTERNAL_SCAN_CALLBACK_STARVED_WHILE_EXTERNAL_SCAN_AVAILABLE'
    assert 'INTERNAL_TF_BUFFER_MISSING_WHILE_EXTERNAL_TF_AVAILABLE' in analysis['contributing_findings']
    assert analysis['guardrails']['ingress_goal_sent'] is False
    assert analysis['guardrails']['maze_explorer_started'] is False
    assert analysis['scan_internal']['callback_count_max'] == 0
    assert analysis['external_snapshot']['scan_available'] is True
    assert analysis['external_snapshot']['map_base_tf_available'] is True


def test_phase113_analyzer_classifies_scan_received_but_rejected_by_internal_tf_or_age(tmp_path):
    analyzer = _load(ANALYZER, 'phase113_analyzer')
    analysis = analyzer.analyze_artifact(
        _artifact(callback_count=4, map_base_available=False),
        external_snapshot_text='frame_id: tugbot/scan_omni/scan_omni\nTranslation: [0, 0, 0]\n',
        launch_log_text='',
    )

    assert analysis['classification'] == 'INTERNAL_SCAN_RECEIVED_BUT_REJECTED_BY_AGE_OR_TF'
    assert analysis['scan_internal']['callback_count_max'] == 4
    assert analysis['scan_internal']['latest_frame_ids'] == ['tugbot/scan_omni/scan_omni']
    assert analysis['tf_internal']['map_base_all_missing'] is True


def test_phase113_analyzer_flags_sim_time_mismatch_when_scan_age_is_wall_time_sized():
    analyzer = _load(ANALYZER, 'phase113_analyzer')
    artifact = _artifact(callback_count=2, map_base_available=False)
    for sample in artifact['ingress_preflight']['sample_history']:
        diag = sample['internal_sampling_diagnostics']
        diag['clock']['use_sim_time'] = False
        diag['scan_subscription']['latest_age_sec'] = 1780498285.3

    analysis = analyzer.analyze_artifact(
        artifact,
        external_snapshot_text='frame_id: tugbot/scan_omni/scan_omni\nTranslation: [0, 0, 0]\n',
        launch_log_text='Managed nodes are active',
    )

    assert analysis['classification'] == 'INTERNAL_SCAN_RECEIVED_BUT_REJECTED_BY_AGE_OR_TF'
    assert 'INTERNAL_NODE_USE_SIM_TIME_FALSE_WITH_SIM_STAMPED_SCAN' in analysis['contributing_findings']
    assert analysis['executor_clock_internal']['use_sim_time_values'] == ['False']


def test_phase113_preflight_sample_contains_internal_diagnostics_without_changing_gate_logic():
    preflight = _load(PREFLIGHT, 'phase113_preflight')
    sample = preflight.make_synthetic_sample(
        internal_sampling_diagnostics={
            'executor': {'spin_once_count_total': 1},
            'scan_subscription': {'topic': '/scan', 'callback_count_total': 0},
            'tf_buffer': {'map->base_link': {'can_transform': False, 'lookup_success': False}},
            'clock': {'use_sim_time': True},
        },
        scan_transform_check={
            'scan_available': False,
            'scan_frame_id': None,
            'target_frame': 'map',
            'transform_available': False,
            'stable': False,
            'cache_drop_detected': False,
            'exception_count': 0,
        },
        elapsed_sec=3.0,
    )
    result = preflight.evaluate_preflight_samples([sample], preflight.PreflightConfig(startup_grace_sec=1.0))
    artifact = result.to_artifact()['ingress_preflight']

    assert artifact['passed'] is False
    assert artifact['ingress_goal_sent'] is False
    assert artifact['maze_explorer_started'] is False
    assert artifact['sample_history'][0]['internal_sampling_diagnostics']['scan_subscription']['topic'] == '/scan'
    assert 'ingress_first_scan_timeout' in artifact['failed_gates']


def test_phase113_cli_writes_analysis_and_summary(tmp_path):
    artifact_path = tmp_path / 'artifact.json'
    snapshot_path = tmp_path / 'snapshot.txt'
    analysis_path = tmp_path / 'analysis.json'
    summary_path = tmp_path / 'summary.md'
    artifact_path.write_text(json.dumps(_artifact(callback_count=0), indent=2), encoding='utf-8')
    snapshot_path.write_text('frame_id: tugbot/scan_omni/scan_omni\nTranslation: [0, 0, 0]\n', encoding='utf-8')

    proc = subprocess.run([
        'python3', str(ANALYZER),
        '--artifact', str(artifact_path),
        '--external-snapshot', str(snapshot_path),
        '--output-json', str(analysis_path),
        '--minimal-summary-output', str(summary_path),
    ], cwd=ROOT, text=True, capture_output=True, check=False)

    assert proc.returncode == 0, proc.stderr
    analysis = json.loads(analysis_path.read_text(encoding='utf-8'))
    assert analysis['classification'] == 'INTERNAL_SCAN_CALLBACK_STARVED_WHILE_EXTERNAL_SCAN_AVAILABLE'
    summary = summary_path.read_text(encoding='utf-8')
    assert 'Phase113 internal sampling discrepancy summary' in summary
    assert 'No NavigateToPose goal was sent' in summary


def test_phase113_report_guardrails_present_after_completion():
    assert REPORT.exists(), f'missing {REPORT}'
    text = REPORT.read_text(encoding='utf-8')
    for phrase in [
        'PHASE113_PREFLIGHT_INTERNAL_SAMPLING_DISCREPANCY_DIAGNOSIS_COMPLETE_STOP_BEFORE_PHASE114',
        'No NavigateToPose goal was sent',
        'ingress_goal_sent=false',
        'maze_explorer_started=false',
        'No maze_explorer was started',
        'No Nav2/MPPI/controller/config tuning was performed',
        'diagnosis-only',
        'does not prove autonomous exploration success',
        'Phase114 not entered',
    ]:
        assert phrase in text
