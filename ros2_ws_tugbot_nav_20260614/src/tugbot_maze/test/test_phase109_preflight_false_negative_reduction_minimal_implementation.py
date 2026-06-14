from __future__ import annotations

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PREFLIGHT = ROOT / 'tools' / 'phase105_inner_ingress_tf_controller_preflight.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase109_preflight_false_negative_reduction_minimal_implementation_report.md'


def _load_preflight():
    assert PREFLIGHT.exists(), f'missing {PREFLIGHT}'
    spec = importlib.util.spec_from_file_location('phase105_preflight_phase109', PREFLIGHT)
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


def test_phase109_lifecycle_multi_source_confirmation_states():
    module = _load_preflight()

    active = module.derive_lifecycle_confirmation(
        lifecycle_query={'returncode': 0, 'state': 'active'},
        node_graph_present=True,
        action_available=True,
    )
    assert active['state'] == 'active_confirmed'
    assert active['active_confirmed'] is True
    assert active['lifecycle_query']['state'] == 'active'
    assert active['node_graph']['present'] is True
    assert active['action_availability']['available'] is True

    inactive = module.derive_lifecycle_confirmation(
        lifecycle_query={'returncode': 0, 'state': 'inactive'},
        node_graph_present=False,
        action_available=False,
    )
    assert inactive['state'] == 'inactive_confirmed'
    assert inactive['inactive_confirmed'] is True

    ambiguous = module.derive_lifecycle_confirmation(
        lifecycle_query={'returncode': 1, 'state': None, 'query_error': 'timeout'},
        node_graph_present=True,
        action_available=True,
    )
    assert ambiguous['state'] == 'ambiguous'
    assert ambiguous['ambiguous'] is True
    assert ambiguous['active_confirmed'] is False
    assert ambiguous['inactive_confirmed'] is False

    query_error = module.derive_lifecycle_confirmation(
        lifecycle_query={'returncode': 1, 'state': None, 'query_error': 'timeout'},
        node_graph_present=False,
        action_available=False,
    )
    assert query_error['state'] == 'query_error'
    assert query_error['query_error'] is True


def test_phase109_first_scan_waits_during_startup_grace_then_times_out():
    module = _load_preflight()
    config = module.PreflightConfig(
        startup_grace_sec=2.0,
        tf_stability_window_sec=1.0,
        timeout_sec=3.0,
        sample_period_sec=0.5,
    )
    samples = [
        _sample(module, elapsed_sec=0.0, scan_transform_check=_missing_scan()),
        _sample(module, elapsed_sec=1.0, scan_transform_check=_missing_scan()),
        _sample(module, elapsed_sec=2.5, scan_transform_check=_missing_scan()),
    ]

    result = module.evaluate_preflight_samples(samples, config)
    artifact = result.to_artifact()['ingress_preflight']

    assert artifact['passed'] is False
    assert artifact['ingress_preflight_reject_reason'] == 'ingress_first_scan_timeout'
    assert 'ingress_first_scan_timeout' in artifact['failed_gates']
    assert artifact['first_scan_seen'] is False
    assert artifact['first_scan_wait_elapsed_sec'] == 2.5
    assert artifact['sample_history'][0]['phase'] == 'startup_grace'
    assert artifact['sample_history'][0]['failed_gates'] == ['waiting_for_first_scan']
    assert artifact['sample_history'][0]['failure_reasons_by_gate']['scan'] == ['waiting_for_first_scan']
    assert artifact['sample_history'][-1]['failure_reasons_by_gate']['scan'] == ['first_scan_timeout']


def test_phase109_single_early_tf_miss_does_not_become_final_failure_if_stable_window_later_passes():
    module = _load_preflight()
    config = module.PreflightConfig(
        startup_grace_sec=0.0,
        tf_stability_window_sec=1.0,
        timeout_sec=3.0,
        sample_period_sec=0.5,
    )
    samples = [
        _sample(module, elapsed_sec=0.0, map_base_tf_check={'available': False, 'stable': False, 'sample_count': 1, 'latest_age_sec': None}),
        _sample(module, elapsed_sec=0.5),
        _sample(module, elapsed_sec=1.6),
    ]

    result = module.evaluate_preflight_samples(samples, config)
    artifact = result.to_artifact()['ingress_preflight']

    assert artifact['passed'] is True
    assert artifact['ingress_preflight_reject_reason'] is None
    assert artifact['sample_history'][0]['failed_gates'] == ['ingress_map_base_tf_missing']
    assert artifact['sample_history'][1]['failed_gates'] == []
    assert artifact['sample_history'][2]['stable_window_elapsed_sec'] >= 1.0


def test_phase109_no_complete_stable_window_reports_stable_window_not_met_not_early_miss():
    module = _load_preflight()
    config = module.PreflightConfig(
        startup_grace_sec=0.0,
        tf_stability_window_sec=1.0,
        timeout_sec=3.0,
        sample_period_sec=0.5,
    )
    samples = [
        _sample(module, elapsed_sec=0.0, map_base_tf_check={'available': False, 'stable': False, 'sample_count': 1, 'latest_age_sec': None}),
        _sample(module, elapsed_sec=0.5),
        _sample(module, elapsed_sec=0.9),
    ]

    result = module.evaluate_preflight_samples(samples, config)
    artifact = result.to_artifact()['ingress_preflight']

    assert artifact['passed'] is False
    assert artifact['ingress_preflight_reject_reason'] == 'ingress_tf_stable_window_not_met'
    assert 'ingress_tf_stable_window_not_met' in artifact['failed_gates']
    assert artifact['last_specific_reject_reason'] == 'ingress_tf_stable_window_not_met'
    assert artifact['sample_history'][0]['failed_gates'] == ['ingress_map_base_tf_missing']
    assert artifact['sample_history'][-1]['stable_window_elapsed_sec'] < 1.0


def test_phase109_sample_history_and_raw_snapshot_contradiction_are_recorded_fail_closed():
    module = _load_preflight()
    config = module.PreflightConfig(startup_grace_sec=0.0, tf_stability_window_sec=1.0, timeout_sec=1.0)
    sample = _sample(
        module,
        elapsed_sec=0.0,
        map_base_tf_check={'available': False, 'stable': False, 'sample_count': 1, 'latest_age_sec': None},
        raw_style_snapshot_override={
            'present': True,
            'scan_available': True,
            'map_available': True,
            'local_costmap_available': True,
            'odom_available': True,
            'tf_pairs_available': {'map->base_link': True, 'map->odom': True, 'odom->base_link': True},
            'navigate_to_pose_action_ready': True,
            'lifecycle_node_graph_present': {'controller_server': True, 'bt_navigator': True},
        },
    )

    result = module.evaluate_preflight_samples([sample], config)
    artifact = result.to_artifact()['ingress_preflight']

    assert artifact['passed'] is False
    assert artifact['ingress_goal_sent'] is False
    assert artifact['maze_explorer_started'] is False
    assert 'sample_history' in artifact
    assert artifact['sample_history'][0]['failure_reasons_by_gate']['tf'] == ['map_base_tf_missing']
    snapshot = artifact['raw_style_snapshot_cross_check']
    assert snapshot['present'] is True
    assert snapshot['ambiguous'] is True
    assert 'map_base_tf_missing_but_raw_snapshot_has_map_base_tf' in snapshot['cross_check_contradictions']
    assert 'ingress_raw_snapshot_cross_check_failed' in artifact['failed_gates']


def test_phase109_report_guardrails_present_after_implementation():
    assert REPORT.exists(), f'missing {REPORT}'
    text = REPORT.read_text(encoding='utf-8')
    for phrase in [
        'PHASE109_IMPLEMENTED_STATIC_UNIT_VALIDATED_STOP_BEFORE_PHASE110',
        'No Phase106 rerun was performed',
        'No ingress goal was sent',
        'No Gazebo/RViz/SLAM/Nav2/maze_explorer was started',
        'No Nav2/MPPI/controller/config tuning was performed',
        'does not prove it is safe to send an ingress goal',
        'Phase110 not entered',
    ]:
        assert phrase in text
