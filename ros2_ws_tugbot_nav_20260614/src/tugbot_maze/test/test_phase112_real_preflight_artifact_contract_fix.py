from __future__ import annotations

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PREFLIGHT = ROOT / 'tools' / 'phase105_inner_ingress_tf_controller_preflight.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase112_real_preflight_artifact_contract_fix_report.md'

REQUIRED_SAMPLE_FIELDS = {
    'phase',
    'failed_gates',
    'failure_reasons_by_gate',
    'lifecycle_sources',
    'scan_transform_check',
    'map_base_tf_check',
    'map_base_tf_age_sec',
    'map_odom_tf_age_sec',
    'odom_base_tf_age_sec',
    'stable_window_elapsed_sec',
}


def _load_preflight():
    assert PREFLIGHT.exists(), f'missing {PREFLIGHT}'
    spec = importlib.util.spec_from_file_location('phase105_preflight_phase112', PREFLIGHT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _sample(module, **overrides):
    sample = module.make_synthetic_sample()
    sample.update(overrides)
    return sample


def _missing_scan():
    return {
        'scan_available': False,
        'scan_frame_id': None,
        'target_frame': 'map',
        'transform_available': False,
        'stable': False,
        'cache_drop_detected': False,
        'exception_count': 0,
    }


def _missing_tf():
    return {'available': False, 'stable': False, 'sample_count': 1, 'latest_age_sec': None}


def assert_complete_sample_contract(artifact):
    history = artifact['sample_history']
    assert artifact['sample_count'] == len(history)
    assert len(history) >= 1
    for index, sample in enumerate(history):
        missing = REQUIRED_SAMPLE_FIELDS - set(sample)
        assert not missing, f'sample {index} missing {sorted(missing)}: {sample}'
        assert isinstance(sample['failed_gates'], list), f'sample {index} failed_gates not list'
        assert isinstance(sample['failure_reasons_by_gate'], dict), f'sample {index} reasons not dict'
        assert isinstance(sample['lifecycle_sources'], dict), f'sample {index} lifecycle_sources not dict'
        assert 'controller_server' in sample['lifecycle_sources']
        assert 'bt_navigator' in sample['lifecycle_sources']
        assert isinstance(sample['scan_transform_check'], dict), f'sample {index} scan evidence not dict'
        assert isinstance(sample['map_base_tf_check'], dict), f'sample {index} TF evidence not dict'
        assert sample['phase'] in {'startup_grace', 'stable_window'}


def test_phase112_final_timeout_path_annotates_samples_after_first_scan_timeout():
    module = _load_preflight()
    config = module.PreflightConfig(
        startup_grace_sec=1.0,
        tf_stability_window_sec=2.0,
        timeout_sec=5.0,
        sample_period_sec=0.5,
    )
    samples = [
        _sample(module, elapsed_sec=0.0, scan_transform_check=_missing_scan()),
        _sample(module, elapsed_sec=1.5, scan_transform_check=_missing_scan()),
        _sample(module, elapsed_sec=3.0, scan_transform_check=_missing_scan(), map_base_tf_check=_missing_tf()),
        _sample(module, elapsed_sec=5.0, scan_transform_check=_missing_scan(), map_base_tf_check=_missing_tf(), force_timeout=True),
    ]

    result = module.evaluate_preflight_samples(samples, config)
    artifact = result.to_artifact()['ingress_preflight']

    assert artifact['passed'] is False
    assert artifact['ingress_goal_sent'] is False
    assert artifact['maze_explorer_started'] is False
    assert artifact['ingress_preflight_reject_reason'] == 'ingress_first_scan_timeout'
    assert_complete_sample_contract(artifact)
    assert artifact['sample_history'][2]['failed_gates']
    assert artifact['sample_history'][3]['failure_reasons_by_gate']


def test_phase112_pass_path_keeps_complete_contract_for_prior_failed_and_recovered_samples():
    module = _load_preflight()
    config = module.PreflightConfig(
        startup_grace_sec=0.0,
        tf_stability_window_sec=1.0,
        timeout_sec=4.0,
        sample_period_sec=0.5,
    )
    samples = [
        _sample(module, elapsed_sec=0.0, map_base_tf_check=_missing_tf()),
        _sample(module, elapsed_sec=0.5),
        _sample(module, elapsed_sec=1.6),
    ]

    result = module.evaluate_preflight_samples(samples, config)
    artifact = result.to_artifact()['ingress_preflight']

    assert artifact['passed'] is True
    assert artifact['ingress_preflight_reject_reason'] is None
    assert_complete_sample_contract(artifact)
    assert artifact['sample_history'][0]['failed_gates'] == ['ingress_map_base_tf_missing']
    assert artifact['sample_history'][1]['stable_window_elapsed_sec'] == 0.0
    assert artifact['sample_history'][2]['stable_window_elapsed_sec'] >= 1.0


def test_phase112_fail_closed_states_are_preserved_while_contract_is_completed():
    module = _load_preflight()
    config = module.PreflightConfig(startup_grace_sec=0.0, tf_stability_window_sec=1.0, timeout_sec=2.0)
    ambiguous_lifecycle = {
        'state': 'ambiguous',
        'active_confirmed': False,
        'inactive_confirmed': False,
        'ambiguous': True,
        'query_error': False,
        'lifecycle_query': {'returncode': 1, 'state': None, 'query_error': 'timeout'},
        'node_graph': {'present': True},
        'action_availability': {'available': True},
    }
    sample = _sample(
        module,
        elapsed_sec=0.0,
        controller_pose_check={
            'controller_server_active': False,
            'bt_navigator_active': False,
            'navigate_to_pose_action_ready': True,
            'robot_pose_available': True,
            'robot_pose_unavailable_log_count': 0,
            'lifecycle_sources': {
                'controller_server': ambiguous_lifecycle,
                'bt_navigator': ambiguous_lifecycle,
            },
        },
        raw_style_snapshot_override={
            'tf_pairs_available': {'map->base_link': True},
            'navigate_to_pose_action_ready': True,
        },
    )

    result = module.evaluate_preflight_samples([sample], config)
    artifact = result.to_artifact()['ingress_preflight']

    assert artifact['passed'] is False
    assert artifact['ingress_goal_sent'] is False
    assert artifact['maze_explorer_started'] is False
    assert 'ingress_lifecycle_ambiguous' in artifact['failed_gates']
    assert artifact['raw_style_snapshot_cross_check']['ambiguous'] is True
    assert 'ingress_raw_snapshot_cross_check_failed' in artifact['failed_gates']
    assert_complete_sample_contract(artifact)


def test_phase112_report_guardrails_present_after_completion():
    assert REPORT.exists(), f'missing {REPORT}'
    text = REPORT.read_text(encoding='utf-8')
    for phrase in [
        'PHASE112_REAL_PREFLIGHT_ARTIFACT_CONTRACT_FIX_COMPLETE_STOP_BEFORE_PHASE113',
        'No NavigateToPose goal was sent',
        'ingress_goal_sent=false',
        'maze_explorer_started=false',
        'No maze_explorer was started',
        'No Nav2/MPPI/controller/config tuning was performed',
        'does not prove autonomous exploration success',
        'Phase113 not entered',
    ]:
        assert phrase in text
