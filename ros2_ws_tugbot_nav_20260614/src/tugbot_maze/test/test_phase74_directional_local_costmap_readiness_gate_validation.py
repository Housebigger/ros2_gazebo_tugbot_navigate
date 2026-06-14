from pathlib import Path
import importlib.util
import json

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase74_directional_local_costmap_readiness_gate_validation.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase74_directional_local_costmap_readiness_gate_validation.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase74_directional_local_costmap_readiness_gate_validation_report.md'
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
RUN_ID = 'phase74_directional_local_costmap_readiness_gate_validation'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def _load_analyzer():
    spec = importlib.util.spec_from_file_location('phase74_analyzer', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_jsonl(path: Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps(row, sort_keys=True) for row in rows), encoding='utf-8')


def _artifact_scaffold(tmp_path: Path, max_goals: int = 3):
    (tmp_path / f'{RUN_ID}_preflight.txt').write_text(
        '\n'.join([
            f'RUN_ID={RUN_ID}',
            f'MAX_GOALS={max_goals}',
            'REPLAY_COUNT=2',
            'GOAL_TIMEOUT_SEC=100',
            'phase73_classification=LOCAL_DIRECTION_TRAVERSABLE_GATE_SHOULD_REPLACE_GLOBAL_RATIO',
            'phase72_classification=MULTIGOAL_NO_REDISPATCH_AFTER_SUCCESS',
            'phase65_inner_ingress_waypoint_map=x=2.0,y=0.0,yaw=0.0',
            'directional_local_costmap_readiness_override_enabled=true',
            'near_exit_fallback_enabled=false',
            'centerline_target_refinement_enabled=true',
            'centerline_target_refinement_gate_mode=balance_first',
            'no centerline gate runtime behavior change',
            'no branch scoring change',
        ]) + '\n',
        encoding='utf-8',
    )
    (tmp_path / f'{RUN_ID}_nav2_config_diff.txt').write_text('', encoding='utf-8')
    (tmp_path / f'{RUN_ID}_cleanup_processes_after.txt').write_text('', encoding='utf-8')


def _make_replay(tmp_path: Path, replay_id: str, events, states):
    replay_dir = tmp_path / replay_id
    replay_dir.mkdir(parents=True, exist_ok=True)
    (replay_dir / f'{RUN_ID}_{replay_id}_inner_ingress_navigate_to_pose_action_result.json').write_text(
        '{"success": true, "goal_sent": true, "result_received": true}', encoding='utf-8'
    )
    _write_jsonl(replay_dir / f'{RUN_ID}_{replay_id}_goal_events.jsonl', [{'state': row, 'elapsed_sec': i + 1.0} for i, row in enumerate(events)])
    _write_jsonl(replay_dir / f'{RUN_ID}_{replay_id}_explorer_state.jsonl', [{'state': row, 'elapsed_sec': i + 1.0} for i, row in enumerate(states)])
    _write_jsonl(replay_dir / 'phase74_runtime_timeline.jsonl', [])
    _write_jsonl(replay_dir / 'phase74_controller_dynamics.jsonl', [])
    _write_jsonl(replay_dir / 'phase74_nav2_feedback.jsonl', [])
    _write_jsonl(replay_dir / 'phase74_local_costmap_samples.jsonl', [])
    return replay_dir


def _directional_override(applied=True):
    return {
        'phase74_directional_local_costmap_readiness_override': True,
        'enabled': True,
        'override_considered': True,
        'override_applied': applied,
        'reason': 'directional_corridor_traversable' if applied else 'no_directional_corridor_traversable',
        'full_window_gate_failed': True,
        'fresh': True,
        'knownness_ok': True,
        'robot_in_bounds': True,
        'selected_candidate_direction': {
            'candidate_direction_angle_rad': 0.0,
            'candidate_direction_angle_deg': 0.0,
            'target': [3.1, 0.0],
            'path_cost': {'max': 0, 'mean': 0.0, 'sample_count': 21, 'coverage_ratio': 1.0},
            'min_clearance_m': 0.55,
            'target_risk': {'target_local_cost': 0, 'target_local_cost_max_radius': 0},
            'footprint_risk': {'max': 0, 'lethal_count': 0},
            'front_wedge_risk': {'max': 40, 'high_cost_count': 0, 'lethal_count': 0},
            'direction_corridor_traversable': applied,
        } if applied else None,
        'candidate_directions': [],
        'direction_traversable_count': 1 if applied else 0,
        'non_reverse_direction_traversable_count': 1 if applied else 0,
    }


def _gate(applied=True):
    override = _directional_override(applied)
    return {
        'passed': bool(applied),
        'blocking_reasons': [] if applied else ['local_costmap_sufficient'],
        'checks': {
            'map_sufficient': True,
            'scan_sufficient': True,
            'tf_sufficient': True,
            'local_costmap_sufficient': bool(applied),
            'local_costmap_full_window_sufficient': False,
            'local_costmap_directional_override_applied': bool(applied),
            'nav2_lifecycle_active': True,
            'navigate_to_pose_action_ready': True,
            'goal_pose_subscriber_ready': True,
        },
        'local_costmap_full_window': {
            'sufficient': False,
            'reason': 'local_costmap_ratio_or_bounds_insufficient',
            'known_ratio': 1.0,
            'free_ratio': 0.43,
            'min_known_ratio': 0.95,
            'min_free_ratio': 0.50,
            'robot_in_bounds': True,
            'sample_age_sec': 0.25,
        },
        'local_costmap': {
            'sufficient': bool(applied),
            'effective_sufficient': bool(applied),
            'full_window_sufficient': False,
            'full_window_reason': 'local_costmap_ratio_or_bounds_insufficient',
            'reason': 'directional_override_sufficient' if applied else 'local_costmap_ratio_or_bounds_insufficient',
            'known_ratio': 1.0,
            'free_ratio': 0.43,
            'min_known_ratio': 0.95,
            'min_free_ratio': 0.50,
            'robot_in_bounds': True,
            'sample_age_sec': 0.25,
            'directional_override': override,
        },
        'local_costmap_directional_override': override,
    }


def _dispatch(seq, target, applied=True):
    return {
        'event': 'dispatch',
        'goal_sequence': seq,
        'target': target,
        'dispatch_pose': [2.0 + seq * 0.2, 0.0, 0.0],
        'candidate_branch_count': 4,
        'candidate_after_filter_count': 4,
        'raw_open_direction_count': 4,
        'filtered_open_direction_count': 4,
        'last_local_topology_kind': 'junction',
        'dispatch_readiness_gate': _gate(applied),
        'directional_local_costmap_readiness_override_applied': applied,
        'centerline_refinement_applied': False,
        'centerline_refinement_reason': 'balance_first_gate_no_apply',
        'branch_scoring_changed': False,
        'dispatch_path_local_cost_max': 0,
        'dispatch_path_local_cost_mean': 0.0,
        'target_local_cost': 0,
        'target_local_cost_max_radius': 0,
        'path_corridor_min_clearance_m': 0.55,
    }


def test_phase74_maze_explorer_has_directional_override_contract_without_forbidden_tuning():
    source = _read(MAZE_EXPLORER)
    for token in [
        'directional_local_costmap_readiness_override_enabled',
        '_directional_local_costmap_readiness_override',
        'phase74_directional_local_costmap_readiness_override',
        'local_costmap_full_window',
        'local_costmap_directional_override',
        'override_applied',
        'selected_candidate_direction',
        'candidate_direction_angle_rad',
        'path_cost',
        'min_clearance_m',
        'target_risk',
        'footprint_risk',
        'front_wedge_risk',
        'local_costmap_full_window_sufficient',
        'local_costmap_directional_override_applied',
    ]:
        assert token in source
    for forbidden in [
        "declare_parameter('clearance_radius_m', 0.30)",
        "declare_parameter('dispatch_readiness_min_local_costmap_free_ratio', 0.40)",
        'score_for_exit(node.xy, (self.exit_x, self.exit_y), self.exit_bias_weight) +',
        'near_exit_terminal_acceptance_radius_m:=',
    ]:
        assert forbidden not in source


def test_phase74_wrapper_contract_uses_phase65_inner_ingress_replay2_maxgoals3_and_guardrails():
    wrapper = _read(WRAPPER)
    assert f'RUN_ID="{RUN_ID}"' in wrapper
    assert 'PHASE74_REPLAY_COUNT:-2' in wrapper
    assert 'PHASE74_MAX_GOALS:-3' in wrapper
    assert 'PHASE74_MAX_GOALS must be exactly 3' in wrapper
    assert 'INNER_INGRESS_X="2.0"' in wrapper
    assert 'INNER_INGRESS_Y="0.0"' in wrapper
    assert 'send-inner-ingress-goal' in wrapper
    assert '-p max_goals:="$MAX_GOALS"' in wrapper
    assert '-p directional_local_costmap_readiness_override_enabled:=true' in wrapper
    for artifact in [
        'phase74_runtime_timeline.jsonl',
        'phase74_controller_dynamics.jsonl',
        'phase74_nav2_feedback.jsonl',
        'phase74_local_costmap_samples.jsonl',
        'phase74_global_plan_samples.jsonl',
        'phase74_collision_monitor_state.jsonl',
    ]:
        assert artifact in wrapper
    for guardrail in [
        'no Nav2/MPPI/controller tuning',
        'no inflation/robot_radius/clearance_radius_m/map threshold tuning',
        'no branch scoring change',
        'no centerline gate runtime behavior change',
        'no fallback/terminal acceptance change',
        'no autonomous exploration success claim',
        'no exit success claim',
    ]:
        assert guardrail in wrapper


def test_phase74_analyzer_contract_mentions_dual_gate_records_and_classifications():
    source = _read(ANALYZER)
    assert f"RUN_ID = '{RUN_ID}'" in source
    for classification in [
        'DIRECTIONAL_GATE_ENABLES_GOAL2_DISPATCH',
        'DIRECTIONAL_GATE_APPLIED_TIMEOUT_REMAINS',
        'DIRECTIONAL_GATE_NO_APPLY',
        'DIRECTIONAL_GATE_REGRESSION',
        'INSUFFICIENT_EVIDENCE',
    ]:
        assert classification in source
    for field in [
        'local_costmap_full_window',
        'local_costmap_directional_override',
        'override_applied',
        'selected_candidate_direction',
        'path_cost',
        'min_clearance_m',
        'target_risk',
        'footprint_risk',
        'front_wedge_risk',
        'goal2_dispatch_observed',
        'dispatch_after_goal1_success_count',
        'complete_autonomous_success_claimed',
        'exit_success_claimed',
    ]:
        assert field in source


def test_phase74_classifies_directional_override_goal2_dispatch_as_enabled(tmp_path):
    module = _load_analyzer()
    _artifact_scaffold(tmp_path)
    states = [
        {'mode': 'WAIT_FOR_DISPATCH_ENTRY_READINESS', 'goal_count': 1, 'goal_success_count': 1, 'goal_active': False, 'dispatch_readiness_gate': _gate(False)},
        {'mode': 'AT_NODE_ANALYZE', 'goal_count': 1, 'goal_success_count': 1, 'goal_active': False, 'dispatch_readiness_gate': _gate(True)},
        {'mode': 'NAVIGATING', 'goal_count': 2, 'goal_success_count': 1, 'goal_active': True, 'dispatch_readiness_gate': _gate(True)},
    ]
    events = [
        _dispatch(1, [2.6, 0.2], applied=True),
        {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded', 'result_status': 4},
        _dispatch(2, [3.2, 0.0], applied=True),
    ]
    _make_replay(tmp_path, 'replay_01', events, states)
    result = module.analyze_phase74(tmp_path)
    replay = result['replays'][0]
    assert result['classification'] == 'DIRECTIONAL_GATE_ENABLES_GOAL2_DISPATCH'
    assert replay['goal1_success_observed'] is True
    assert replay['goal2_dispatch_observed'] is True
    assert replay['directional_override_summary']['override_applied_observed'] is True
    assert replay['directional_override_summary']['selected_candidate_direction']['path_cost']['max'] == 0
    assert replay['directional_override_summary']['selected_candidate_direction']['min_clearance_m'] == 0.55
    assert result['metrics']['goal2_dispatch_replay_count'] == 1
    assert result['complete_autonomous_success_claimed'] is False
    assert result['exit_success_claimed'] is False


def test_phase74_classifies_no_override_applied_when_goal2_still_not_dispatched(tmp_path):
    module = _load_analyzer()
    _artifact_scaffold(tmp_path)
    events = [
        _dispatch(1, [2.6, 0.2], applied=False),
        {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded', 'result_status': 4},
    ]
    states = [
        {'mode': 'WAIT_FOR_DISPATCH_ENTRY_READINESS', 'goal_count': 1, 'goal_success_count': 1, 'goal_active': False, 'dispatch_readiness_gate': _gate(False)},
    ]
    _make_replay(tmp_path, 'replay_01', events, states)
    result = module.analyze_phase74(tmp_path)
    assert result['classification'] == 'DIRECTIONAL_GATE_NO_APPLY'
    assert result['metrics']['directional_override_applied_replay_count'] == 0
    assert result['metrics']['goal2_dispatch_replay_count'] == 0


def test_phase74_report_exists_and_stops_before_phase75():
    report = _read(REPORT)
    assert 'Phase74 Directional Local Costmap Readiness Gate / Bounded Runtime Validation' in report
    assert 'DIRECTIONAL_GATE_' in report
    assert 'full-window local costmap gate' in report
    assert 'directional override' in report
    assert '不进入 Phase75' in report or 'Do not enter Phase75' in report
