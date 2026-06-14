from __future__ import annotations

import importlib.util
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PREFLIGHT = ROOT / 'tools' / 'phase105_inner_ingress_tf_controller_preflight.py'
ANALYZER = ROOT / 'tools' / 'analyze_phase114_preflight_sim_time_alignment_fix.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase114_preflight_sim_time_alignment_fix_report.md'


def _load(path: Path, name: str):
    assert path.exists(), f'missing {path}'
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _phase114_artifact(*, use_sim_time: bool, scan_age: float | None, reject_reason: str | None = 'ingress_lifecycle_ambiguous', map_tf_ok: bool = True) -> dict:
    sample = {
        'elapsed_sec': 4.0,
        'phase': 'stable_window',
        'failed_gates': [] if reject_reason is None else [reject_reason],
        'failure_reasons_by_gate': {},
        'lifecycle_sources': {},
        'stable_window_elapsed_sec': 0.0,
        'map_base_tf_check': {'available': map_tf_ok, 'stable': map_tf_ok, 'sample_count': 1, 'latest_age_sec': 0.1 if map_tf_ok else None},
        'map_base_tf_age_sec': 0.1 if map_tf_ok else None,
        'map_odom_tf_age_sec': 0.1 if map_tf_ok else None,
        'odom_base_tf_age_sec': 0.1,
        'scan_transform_check': {
            'scan_available': scan_age is not None and scan_age <= 1.5,
            'scan_frame_id': 'tugbot/scan_omni/scan_omni',
            'target_frame': 'map',
            'transform_available': map_tf_ok,
            'stable': map_tf_ok,
            'cache_drop_detected': False,
            'exception_count': 0 if map_tf_ok else 1,
        },
        'internal_sampling_diagnostics': {
            'executor': {'spin_once_count_total': 3, 'last_spin_timeout_sec': 0.05},
            'clock': {'use_sim_time': use_sim_time, 'ros_time_sec': 45.0, 'wall_time_sec': 1780499000.0},
            'scan_subscription': {
                'topic': '/scan',
                'publisher_count': 1,
                'subscriber_count': 1,
                'callback_count_total': 3,
                'callbacks_since_last_sample': 1,
                'latest_frame_id': 'tugbot/scan_omni/scan_omni',
                'latest_stamp_sec': 44.8,
                'latest_age_sec': scan_age,
                'latest_received_wall_time_sec': 1780499000.0,
            },
            'tf_buffer': {
                'map->base_link': {'can_transform': map_tf_ok, 'lookup_success': map_tf_ok, 'age_sec': 0.1 if map_tf_ok else None, 'lookup_exception': None if map_tf_ok else 'map missing'},
                'map->odom': {'can_transform': map_tf_ok, 'lookup_success': map_tf_ok, 'age_sec': 0.1 if map_tf_ok else None, 'lookup_exception': None if map_tf_ok else 'map missing'},
                'odom->base_link': {'can_transform': True, 'lookup_success': True, 'age_sec': 0.1, 'lookup_exception': None},
                'map->tugbot/scan_omni/scan_omni': {'can_transform': map_tf_ok, 'lookup_success': map_tf_ok, 'age_sec': 0.1 if map_tf_ok else None, 'lookup_exception': None if map_tf_ok else 'map missing'},
            },
        },
    }
    return {
        'ingress_preflight': {
            'evaluated': True,
            'passed': reject_reason is None,
            'use_sim_time': use_sim_time,
            'ingress_preflight_reject_reason': reject_reason,
            'failed_gates': [] if reject_reason is None else [reject_reason],
            'ingress_goal_sent': False,
            'maze_explorer_started': False,
            'sample_count': 1,
            'sample_history': [sample],
        }
    }


def test_phase114_config_defaults_to_sim_time_and_artifact_records_it():
    preflight = _load(PREFLIGHT, 'phase114_preflight')
    config = preflight.PreflightConfig()
    assert config.use_sim_time is True

    sample = preflight.make_synthetic_sample(elapsed_sec=0.0)
    result = preflight.evaluate_preflight_samples([sample], config)
    artifact = result.to_artifact()['ingress_preflight']

    assert artifact['use_sim_time'] is True
    assert artifact['ingress_goal_sent'] is False
    assert artifact['maze_explorer_started'] is False


def test_phase114_cli_exposes_use_sim_time_switches_and_synthetic_artifact(tmp_path):
    output = tmp_path / 'preflight.json'
    help_proc = subprocess.run(['python3', str(PREFLIGHT), '--help'], cwd=ROOT, text=True, capture_output=True, check=False)
    assert help_proc.returncode == 0
    assert '--use-sim-time' in help_proc.stdout
    assert '--no-use-sim-time' in help_proc.stdout

    proc = subprocess.run([
        'python3', str(PREFLIGHT),
        '--output', str(output),
        '--synthetic-pass',
        '--use-sim-time',
    ], cwd=ROOT, text=True, capture_output=True, check=False)
    assert proc.returncode == 0, proc.stderr
    artifact = json.loads(output.read_text(encoding='utf-8'))['ingress_preflight']
    assert artifact['use_sim_time'] is True


def test_phase114_runtime_sampler_source_sets_node_use_sim_time_before_sampling():
    text = PREFLIGHT.read_text(encoding='utf-8')
    assert "'use_sim_time'" in text
    assert 'parameter_overrides' in text or 'set_parameters' in text
    assert 'config.use_sim_time' in text


def test_phase114_runtime_sampler_uses_sensor_qos_and_background_executor_for_sim_time_callbacks():
    text = PREFLIGHT.read_text(encoding='utf-8')
    assert 'ReliabilityPolicy.BEST_EFFORT' in text
    assert '_spin_callbacks' in text
    assert 'spin_warmup_sec' in text
    assert 'SingleThreadedExecutor' in text
    assert 'executor_thread' in text
    assert 'background_executor_thread' in text


def test_phase114_evaluate_does_not_report_first_scan_timeout_after_a_scan_was_received():
    preflight = _load(PREFLIGHT, 'phase114_preflight_scan_timeout_rewrite')
    config = preflight.PreflightConfig(startup_grace_sec=1.0, use_sim_time=True)
    first = preflight.make_synthetic_sample(elapsed_sec=0.0)
    stale_after_first = preflight.make_synthetic_sample(
        elapsed_sec=2.0,
        scan_transform_check={
            'scan_available': False,
            'scan_frame_id': 'tugbot/scan_omni/scan_omni',
            'target_frame': 'map',
            'transform_available': False,
            'stable': False,
            'cache_drop_detected': False,
            'exception_count': 0,
        },
        internal_sampling_diagnostics={
            'clock': {'use_sim_time': True},
            'scan_subscription': {'topic': '/scan', 'callback_count_total': 1, 'latest_frame_id': 'tugbot/scan_omni/scan_omni', 'latest_age_sec': 9.4},
            'tf_buffer': {},
        },
    )

    result = preflight.evaluate_preflight_samples([first, stale_after_first], config)
    artifact = result.to_artifact()['ingress_preflight']

    assert artifact['first_scan_seen'] is True
    assert artifact['ingress_preflight_reject_reason'] != 'ingress_first_scan_timeout'
    assert 'ingress_scan_transform_unstable' in artifact['failed_gates']
    assert 'ingress_first_scan_timeout' not in artifact['sample_history'][-1]['failed_gates']


def test_phase114_analyzer_accepts_recovered_scan_age_and_tf_even_if_fail_closed_for_new_reason():
    analyzer = _load(ANALYZER, 'phase114_analyzer')
    analysis = analyzer.analyze_artifact(_phase114_artifact(use_sim_time=True, scan_age=0.2, reject_reason='ingress_lifecycle_ambiguous', map_tf_ok=True))

    assert analysis['classification'] == 'SIM_TIME_ALIGNMENT_RECOVERED_FAIL_CLOSED_NEW_REASON'
    assert analysis['sim_time']['artifact_use_sim_time'] is True
    assert analysis['scan']['wall_time_sized_age_detected'] is False
    assert analysis['scan']['max_scan_age_sec'] == 0.2
    assert analysis['tf']['map_base_any_can_transform'] is True
    assert analysis['preflight']['reject_reason'] == 'ingress_lifecycle_ambiguous'
    assert analysis['guardrails']['no_goal_dispatch_guard_valid'] is True


def test_phase114_analyzer_rejects_old_first_scan_timeout_signature():
    analyzer = _load(ANALYZER, 'phase114_analyzer')
    analysis = analyzer.analyze_artifact(_phase114_artifact(use_sim_time=False, scan_age=1780498285.0, reject_reason='ingress_first_scan_timeout', map_tf_ok=False))

    assert analysis['classification'] == 'SIM_TIME_ALIGNMENT_NOT_EFFECTIVE'
    assert 'PRELIGHT_USE_SIM_TIME_FALSE' in analysis['findings']
    assert 'WALL_TIME_SIZED_SCAN_AGE_STILL_PRESENT' in analysis['findings']
    assert 'REJECT_REASON_STILL_INGRESS_FIRST_SCAN_TIMEOUT' in analysis['findings']


def test_phase114_report_guardrails_present_after_completion():
    assert REPORT.exists(), f'missing {REPORT}'
    text = REPORT.read_text(encoding='utf-8')
    for phrase in [
        'PHASE114_PREFLIGHT_SIM_TIME_ALIGNMENT_FIX_COMPLETE_STOP_BEFORE_PHASE115',
        'No NavigateToPose goal was sent',
        'ingress_goal_sent=false',
        'maze_explorer_started=false',
        'No maze_explorer was started',
        'No Nav2/MPPI/controller/config tuning was performed',
        'does not prove autonomous exploration success',
        'Phase115 not entered',
    ]:
        assert phrase in text
