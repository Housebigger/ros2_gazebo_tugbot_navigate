from pathlib import Path
import importlib.util
import json

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase73_local_costmap_sufficiency_gate_diagnosis.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase73_local_costmap_sufficiency_gate_diagnosis_report.md'
RUN_ID = 'phase73_local_costmap_sufficiency_gate_diagnosis'
PHASE72_RUN_ID = 'phase72_multigoal_bounded_rerun_from_inner_ingress'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def _load_analyzer():
    spec = importlib.util.spec_from_file_location('phase73_analyzer', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_jsonl(path: Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps(row, sort_keys=True) for row in rows) + '\n', encoding='utf-8')


def _blocked_gate(free_ratio=0.43, occupied_ratio=0.57, unknown_ratio=0.0):
    sample_count = 1000
    free_count = int(round(sample_count * free_ratio))
    unknown_count = int(round(sample_count * unknown_ratio))
    occupied_count = sample_count - free_count - unknown_count
    return {
        'passed': False,
        'blocking_reasons': ['local_costmap_sufficient'],
        'checks': {
            'map_sufficient': True,
            'scan_sufficient': True,
            'tf_sufficient': True,
            'local_costmap_sufficient': False,
            'nav2_lifecycle_active': True,
            'navigate_to_pose_action_ready': True,
            'goal_pose_subscriber_ready': True,
        },
        'map': {'known_ratio': 1.0, 'free_ratio': 0.95, 'sample_age_sec': 0.10},
        'scan': {'finite_count': 600, 'sample_age_sec': 0.05},
        'tf': {'sample_age_sec': 0.02},
        'local_costmap': {
            'sufficient': False,
            'reason': 'local_costmap_ratio_or_bounds_insufficient',
            'known_ratio': 1.0,
            'free_ratio': free_ratio,
            'occupied_ratio': occupied_ratio,
            'unknown_ratio': unknown_ratio,
            'free_count': free_count,
            'occupied_count': occupied_count,
            'unknown_count': unknown_count,
            'known_count': free_count + occupied_count,
            'sample_count': sample_count,
            'in_bounds_count': sample_count,
            'out_of_bounds_count': 0,
            'min_free_ratio': 0.5,
            'min_known_ratio': 0.95,
            'radius_m': 1.0,
            'sample_age_sec': 0.15,
            'grid_resolution': 0.05,
            'grid_width': 60,
            'grid_height': 60,
            'robot_in_bounds': True,
            'sample_window': {'min_x': 10, 'max_x': 50, 'min_y': 9, 'max_y': 49},
        },
    }


def _replay_scaffold(tmp_path: Path, replay_id: str, *, direction_traversable: bool):
    replay_dir = tmp_path / replay_id
    replay_dir.mkdir(parents=True, exist_ok=True)
    branch_good = {
        'rank': 2,
        'target': [1.7, 0.95],
        'branch_angle': 1.57,
        'is_reverse_candidate': False,
        'target_local_cost': 52,
        'target_local_cost_max_radius': 72,
        'dispatch_path_local_cost_max': 52,
        'dispatch_path_local_cost_mean': 12.0,
        'path_corridor_min_clearance_m': 0.46,
        'rejection_reason': 'lower_rank_not_selected',
    }
    branch_bad = {
        'rank': 1,
        'target': [2.6, 0.2],
        'branch_angle': 0.0,
        'is_reverse_candidate': False,
        'target_local_cost': 99,
        'target_local_cost_max_radius': 99,
        'dispatch_path_local_cost_max': 99,
        'dispatch_path_local_cost_mean': 80.0,
        'path_corridor_min_clearance_m': 0.35,
        'rejection_reason': None,
    }
    branches = [branch_bad, branch_good] if direction_traversable else [branch_bad, dict(branch_bad, rank=2, rejection_reason='lower_rank_not_selected')]
    topo_samples = (
        [
            {'local_costmap': {'costmap_lethal_or_unknown_result': 'lethal_or_obstacle', 'sampled_endpoint_cost': 99, 'sampled_endpoint_max_radius_cost': 99}},
            {'local_costmap': {'costmap_lethal_or_unknown_result': 'inflated', 'sampled_endpoint_cost': 0, 'sampled_endpoint_max_radius_cost': 72}},
            {'local_costmap': {'costmap_lethal_or_unknown_result': 'clear', 'sampled_endpoint_cost': 0, 'sampled_endpoint_max_radius_cost': 0}},
        ]
        if direction_traversable
        else [
            {'local_costmap': {'costmap_lethal_or_unknown_result': 'lethal_or_obstacle', 'sampled_endpoint_cost': 99, 'sampled_endpoint_max_radius_cost': 99}},
            {'local_costmap': {'costmap_lethal_or_unknown_result': 'lethal_or_obstacle', 'sampled_endpoint_cost': 99, 'sampled_endpoint_max_radius_cost': 99}},
        ]
    )
    state = {
        'mode': 'WAIT_FOR_DISPATCH_ENTRY_READINESS',
        'goal_count': 1,
        'goal_success_count': 1,
        'goal_active': False,
        'dispatch_readiness_gate': _blocked_gate(),
        'dispatch_readiness_gate_passed': False,
        'dispatch_readiness_blocking_reasons': ['local_costmap_sufficient'],
        'last_candidate_count': 4,
        'last_open_direction_count': 4,
        'last_local_topology_kind': 'junction',
        'last_topology_sampling_diagnostics': {
            'raw_open_direction_count': 4,
            'filtered_open_direction_count': 4,
            'candidate_branch_count': 4,
            'candidate_after_filter_count': 4,
            'samples': topo_samples,
        },
    }
    local_row = {
        'event': 'local_costmap_sample',
        'goal_sequence': 1,
        'elapsed_sec': 100.0,
        'robot_pose': [2.3, 0.03, 0.0],
        'robot_footprint_cost': {'sample_count': 143, 'in_bounds_sample_count': 143, 'max': 40 if direction_traversable else 99, 'mean': 10.0 if direction_traversable else 75.0, 'high_cost_count': 0 if direction_traversable else 80, 'lethal_count': 0 if direction_traversable else 45},
        'front_wedge_cost': {'sample_count': 272, 'in_bounds_sample_count': 272, 'max': 55 if direction_traversable else 100, 'mean': 20.0 if direction_traversable else 90.0, 'high_cost_count': 0 if direction_traversable else 220, 'lethal_count': 0 if direction_traversable else 180},
        'target_footprint_cost': {'sample_count': 143, 'in_bounds_sample_count': 143, 'max': 50 if direction_traversable else 99, 'mean': 20.0 if direction_traversable else 85.0, 'high_cost_count': 0 if direction_traversable else 100, 'lethal_count': 0 if direction_traversable else 80},
        'local_costmap_target_evidence': {'available': True, 'in_bounds': True, 'value': 52 if direction_traversable else 99, 'radius_cost_summary': {'sample_count': 111, 'max': 72 if direction_traversable else 99, 'high_cost_count': 2 if direction_traversable else 90, 'lethal_count': 0 if direction_traversable else 70}},
    }
    _write_jsonl(replay_dir / f'{PHASE72_RUN_ID}_{replay_id}_goal_events.jsonl', [
        {'elapsed_sec': 1.0, 'state': {'event': 'dispatch', 'goal_sequence': 1, 'candidate_branch_count': 4, 'candidate_branches': branches}},
        {'elapsed_sec': 2.0, 'state': {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded'}},
    ])
    _write_jsonl(replay_dir / f'{PHASE72_RUN_ID}_{replay_id}_explorer_state.jsonl', [{'elapsed_sec': 3.0, 'state': state}])
    _write_jsonl(replay_dir / 'phase72_local_costmap_samples.jsonl', [local_row])
    return replay_dir


def test_phase73_analyzer_contract_mentions_required_semantic_slices_and_labels():
    source = _read(ANALYZER)
    assert f"RUN_ID = '{RUN_ID}'" in source
    for label in [
        'FULL_WINDOW_FREE_RATIO_TOO_STRICT',
        'LOCAL_COSTMAP_TRUE_BLOCKED',
        'LOCAL_DIRECTION_TRAVERSABLE_GATE_SHOULD_REPLACE_GLOBAL_RATIO',
        'INSUFFICIENT_EVIDENCE',
    ]:
        assert label in source
    for field in [
        'full_window_distribution',
        'robot_footprint_nearby_distribution',
        'front_wedge_distribution',
        'candidate_direction_corridor_slices',
        'inflated_count_estimate',
        'post_success_local_costmap_sufficiency_gate',
        'free_ratio_margin_to_threshold',
        'direction_traversable_count',
        'target_footprint_front_wedge_local_cost',
    ]:
        assert field in source
    for guardrail in [
        'no Nav2/MPPI/controller tuning',
        'no inflation/robot_radius/clearance_radius_m/map threshold tuning',
        'no branch scoring change',
        'no centerline gate change',
        'no fallback/terminal acceptance change',
        'no autonomous exploration success claim',
        'no exit success claim',
    ]:
        assert guardrail in source


def test_phase73_classifies_global_ratio_block_with_direction_traversable_as_semantic_replacement_candidate(tmp_path):
    module = _load_analyzer()
    _replay_scaffold(tmp_path, 'replay_01', direction_traversable=True)
    result = module.analyze_phase73(tmp_path, output=None)
    replay = result['replays'][0]
    assert result['classification'] == 'LOCAL_DIRECTION_TRAVERSABLE_GATE_SHOULD_REPLACE_GLOBAL_RATIO'
    assert replay['post_success_local_costmap_sufficiency_gate']['blocked_by_local_costmap_sufficient'] is True
    assert replay['full_window_distribution']['free_ratio'] == 0.43
    assert replay['full_window_distribution']['free_ratio_margin_to_threshold'] < 0
    assert replay['candidate_direction_corridor_slices']['direction_traversable_count'] >= 1
    assert replay['robot_footprint_nearby_distribution']['lethal_count'] == 0
    assert replay['front_wedge_distribution']['lethal_count'] == 0
    assert result['threshold_semantics']['replace_global_ratio_with_directional_gate_candidate'] is True


def test_phase73_classifies_low_full_ratio_plus_blocked_footprint_wedge_and_candidates_as_true_blocked(tmp_path):
    module = _load_analyzer()
    _replay_scaffold(tmp_path, 'replay_01', direction_traversable=False)
    result = module.analyze_phase73(tmp_path, output=None)
    replay = result['replays'][0]
    assert result['classification'] == 'LOCAL_COSTMAP_TRUE_BLOCKED'
    assert replay['candidate_direction_corridor_slices']['direction_traversable_count'] == 0
    assert replay['robot_footprint_nearby_distribution']['lethal_count'] > 0
    assert replay['front_wedge_distribution']['lethal_count'] > 0


def test_phase73_report_contract_records_artifact_reuse_guardrails_and_stop_condition():
    report = _read(REPORT)
    assert 'Phase73' in report
    assert RUN_ID in report
    assert 'Phase72' in report
    assert 'MULTIGOAL_NO_REDISPATCH_AFTER_SUCCESS' in report
    assert 'POST_SUCCESS_REDISPATCH_BLOCKED_BY_LOCAL_COSTMAP_SUFFICIENCY' in report
    assert 'free_ratio' in report
    assert 'candidate' in report.lower()
    assert 'front wedge' in report.lower() or 'front_wedge' in report
    assert '不进入 Phase74' in report or 'Do not enter Phase74' in report
    assert '不宣称 autonomous exploration success' in report or 'no autonomous exploration success claim' in report
    assert '不宣称 exit success' in report or 'no exit success claim' in report
