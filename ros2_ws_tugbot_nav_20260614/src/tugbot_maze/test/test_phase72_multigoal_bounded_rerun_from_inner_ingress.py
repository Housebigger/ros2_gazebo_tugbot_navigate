from pathlib import Path
import importlib.util
import json

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase72_multigoal_bounded_rerun_from_inner_ingress.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase72_multigoal_bounded_rerun_from_inner_ingress.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase72_multigoal_bounded_rerun_from_inner_ingress_report.md'
RUN_ID = 'phase72_multigoal_bounded_rerun_from_inner_ingress'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def _load_analyzer():
    spec = importlib.util.spec_from_file_location('phase72_analyzer', ANALYZER)
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
            'phase71_classification=POST_SUCCESS_EXHAUSTED_PREMATURELY',
            'phase70_classification=BALANCE_FIRST_GATE_NO_APPLY',
            'phase65_inner_ingress_waypoint_map=x=2.0,y=0.0,yaw=0.0',
            'near_exit_fallback_enabled=false',
            'centerline_target_refinement_enabled=true',
            'centerline_target_refinement_gate_mode=balance_first',
            'no additional centerline gate relaxation',
        ]) + '\n',
        encoding='utf-8',
    )
    (tmp_path / f'{RUN_ID}_nav2_config_diff.txt').write_text('', encoding='utf-8')
    (tmp_path / f'{RUN_ID}_cleanup_processes_after.txt').write_text('', encoding='utf-8')


def _make_replay(tmp_path: Path, replay_id: str, events, states, timeline=None, controller=None, feedback=None, local=None):
    replay_dir = tmp_path / replay_id
    replay_dir.mkdir(parents=True, exist_ok=True)
    (replay_dir / f'{RUN_ID}_{replay_id}_inner_ingress_navigate_to_pose_action_result.json').write_text(
        '{"success": true, "goal_sent": true, "result_received": true}', encoding='utf-8'
    )
    _write_jsonl(replay_dir / f'{RUN_ID}_{replay_id}_goal_events.jsonl', [{'state': row, 'elapsed_sec': i + 1.0} for i, row in enumerate(events)])
    _write_jsonl(replay_dir / f'{RUN_ID}_{replay_id}_explorer_state.jsonl', [{'state': row, 'elapsed_sec': i + 1.0} for i, row in enumerate(states)])
    _write_jsonl(replay_dir / 'phase72_runtime_timeline.jsonl', timeline or [])
    _write_jsonl(replay_dir / 'phase72_controller_dynamics.jsonl', controller or [])
    _write_jsonl(replay_dir / 'phase72_nav2_feedback.jsonl', feedback or [])
    _write_jsonl(replay_dir / 'phase72_local_costmap_samples.jsonl', local or [])
    return replay_dir


def _ready_gate():
    return {
        'passed': True,
        'blocking_reasons': [],
        'checks': {
            'map_sufficient': True,
            'scan_sufficient': True,
            'tf_sufficient': True,
            'local_costmap_sufficient': True,
            'nav2_lifecycle_active': True,
            'navigate_to_pose_action_ready': True,
            'goal_pose_subscriber_ready': True,
        },
        'map': {'sample_age_sec': 0.25, 'known_ratio': 0.97, 'free_ratio': 0.91},
        'scan': {'sample_age_sec': 0.10, 'finite_count': 630},
        'tf': {'sample_age_sec': 0.03},
        'local_costmap': {'sample_age_sec': 0.20, 'known_ratio': 1.0, 'free_ratio': 0.58},
    }


def _dispatch(seq, target):
    return {
        'event': 'dispatch',
        'goal_sequence': seq,
        'target': target,
        'dispatch_pose': [1.9 + seq * 0.1, 0.0, 0.0],
        'candidate_branch_count': 4,
        'candidate_after_filter_count': 4,
        'raw_open_direction_count': 4,
        'filtered_open_direction_count': 4,
        'last_local_topology_kind': 'junction',
        'dispatch_readiness_gate': _ready_gate(),
        'centerline_refinement_applied': False,
        'centerline_refinement_reason': 'balance_first_gate_no_apply',
        'branch_scoring_changed': False,
        'dispatch_target_local_cost': 0,
        'phase62_target_footprint_cost': {'summary': {'max': 0, 'lethal_count': 0}},
        'phase62_front_wedge_cost': {'max': 40, 'high_cost_count': 0},
    }


def test_phase72_wrapper_contract_uses_multigoal_inner_ingress_without_strategy_tuning():
    wrapper = _read(WRAPPER)
    assert f'RUN_ID="{RUN_ID}"' in wrapper
    assert 'PHASE72_REPLAY_COUNT:-2' in wrapper
    assert 'PHASE72_MAX_GOALS:-3' in wrapper
    assert 'PHASE72_MAX_GOALS must be 3 or 4' in wrapper
    assert 'INNER_INGRESS_X="2.0"' in wrapper
    assert 'INNER_INGRESS_Y="0.0"' in wrapper
    assert 'send-inner-ingress-goal' in wrapper
    assert '-p max_goals:="$MAX_GOALS"' in wrapper
    assert 'record_explorer_state_series.py --topic /maze/goal_events' in wrapper
    assert 'record_explorer_state_series.py --topic /maze/explorer_state' in wrapper
    assert 'analyze_phase72_multigoal_bounded_rerun_from_inner_ingress.py --record-runtime' in wrapper
    for artifact in [
        'phase72_runtime_timeline.jsonl',
        'phase72_controller_dynamics.jsonl',
        'phase72_nav2_feedback.jsonl',
        'phase72_local_costmap_samples.jsonl',
        'phase72_global_plan_samples.jsonl',
        'phase72_collision_monitor_state.jsonl',
    ]:
        assert artifact in wrapper
    for guardrail in [
        'no Nav2/MPPI/controller parameter edits',
        'no inflation/robot_radius/clearance_radius_m/map threshold tuning',
        'no branch scoring change',
        'no additional centerline gate relaxation',
        'no corridor-following cmd_vel control',
        'no fallback/terminal acceptance change',
        'no autonomous exploration success claim',
        'no exit success claim',
    ]:
        assert guardrail in wrapper


def test_phase72_analyzer_contract_mentions_required_multigoal_diagnostics():
    source = _read(ANALYZER)
    assert f"RUN_ID = '{RUN_ID}'" in source
    for classification in [
        'MULTIGOAL_REDISPATCH_WORKS',
        'MULTIGOAL_TIMEOUT_REMAINS',
        'MULTIGOAL_NO_REDISPATCH_AFTER_SUCCESS',
        'MULTIGOAL_LOCAL_COST_RISK_REMAINS',
        'INSUFFICIENT_EVIDENCE',
    ]:
        assert classification in source
    for field in [
        'dispatch_after_goal1_success_count',
        'goal2_dispatch_observed',
        'per_goal_summaries',
        'mode_transitions',
        'candidate_open_direction_summary',
        'readiness_summary',
        'nav2_feedback_summary',
        'cmd_vel_summary',
        'robot_progress_summary',
        'distance_remaining',
        'local_cost_footprint_front_wedge_summary',
        'goal_tolerance_proximity_summary',
        'complete_autonomous_success_claimed',
        'exit_success_claimed',
    ]:
        assert field in source


def test_phase72_classifies_goal1_success_goal2_dispatch_then_timeout_as_timeout_remains(tmp_path):
    module = _load_analyzer()
    _artifact_scaffold(tmp_path, max_goals=3)
    _make_replay(
        tmp_path,
        'replay_01',
        events=[
            _dispatch(1, [2.6, 0.2]),
            {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded', 'result_status': 4},
            _dispatch(2, [2.7, 1.0]),
            {'event': 'timeout', 'goal_sequence': 2, 'result_reason': 'goal_timeout'},
            {'event': 'timeout_cancel_result', 'goal_sequence': 2, 'result_status': 5, 'result_reason': 'goal_canceled_after_timeout'},
        ],
        states=[
            {'mode': 'NAVIGATING', 'goal_count': 1, 'goal_success_count': 0, 'dispatch_readiness_gate': _ready_gate()},
            {'mode': 'WAIT_FOR_DISPATCH_ENTRY_READINESS', 'goal_count': 1, 'goal_success_count': 1, 'last_completed_goal_sequence_id': 1, 'dispatch_readiness_gate': _ready_gate()},
            {'mode': 'NAVIGATING', 'goal_count': 2, 'goal_success_count': 1, 'active_goal_sequence_id': 2, 'last_candidate_count': 4, 'last_open_direction_count': 4, 'last_local_topology_kind': 'junction', 'dispatch_readiness_gate': _ready_gate()},
            {'mode': 'FAILED_EXHAUSTED', 'goal_count': 2, 'goal_success_count': 1, 'last_failure_reason': 'goal_canceled_after_timeout', 'dispatch_readiness_gate': _ready_gate()},
        ],
        feedback=[
            {'event': 'nav2_feedback', 'goal_sequence': 2, 'distance_remaining': 0.7, 'number_of_recoveries': 0},
            {'event': 'nav2_feedback', 'goal_sequence': 2, 'distance_remaining': 0.21, 'number_of_recoveries': 2},
        ],
        controller=[
            {'source': 'cmd_vel', 'goal_sequence': 2, 'linear_x': 0.1, 'angular_z': 0.0},
            {'source': 'odom', 'goal_sequence': 2, 'x': 2.1, 'y': 0.2, 'yaw': 0.0},
            {'source': 'odom', 'goal_sequence': 2, 'x': 2.55, 'y': 0.9, 'yaw': 0.0},
        ],
        local=[
            {'event': 'local_costmap_sample', 'goal_sequence': 2, 'local_costmap_target_evidence': {'value': 0, 'radius_cost_summary': {'max': 10}}, 'target_footprint_cost': {'max': 0, 'lethal_count': 0}, 'front_wedge_cost': {'max': 20, 'high_cost_count': 0}},
        ],
    )
    result = module.analyze_phase72(tmp_path, output=None)
    replay = result['replays'][0]
    assert result['classification'] == 'MULTIGOAL_TIMEOUT_REMAINS'
    assert result['metrics']['redispatch_after_goal1_success_observed'] is True
    assert replay['dispatch_after_goal1_success_count'] == 1
    assert replay['goal2_dispatch_observed'] is True
    assert replay['timeout_count'] >= 1
    goal2 = next(g for g in replay['per_goal_summaries'] if g['goal_sequence'] == 2)
    assert goal2['outcome_event'] == 'timeout'
    assert goal2['goal_tolerance_proximity_summary']['final_distance_remaining_m'] == 0.21
    assert goal2['goal_tolerance_proximity_summary']['near_goal_tolerance_band_observed'] is True


def test_phase72_classifies_goal1_success_without_goal2_dispatch_as_no_redispatch(tmp_path):
    module = _load_analyzer()
    _artifact_scaffold(tmp_path, max_goals=3)
    _make_replay(
        tmp_path,
        'replay_01',
        events=[_dispatch(1, [2.6, 0.2]), {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded'}],
        states=[
            {'mode': 'NAVIGATING', 'goal_count': 1, 'goal_success_count': 0, 'dispatch_readiness_gate': _ready_gate()},
            {'mode': 'WAIT_FOR_DISPATCH_ENTRY_READINESS', 'goal_count': 1, 'goal_success_count': 1, 'last_completed_goal_sequence_id': 1, 'dispatch_readiness_gate': _ready_gate(), 'last_candidate_count': 4, 'last_open_direction_count': 4, 'last_local_topology_kind': 'junction'},
        ],
    )
    result = module.analyze_phase72(tmp_path, output=None)
    assert result['classification'] == 'MULTIGOAL_NO_REDISPATCH_AFTER_SUCCESS'
    replay = result['replays'][0]
    assert replay['goal1_success_observed'] is True
    assert replay['goal2_dispatch_observed'] is False
    assert replay['dispatch_after_goal1_success_count'] == 0


def test_phase72_report_contract_stop_condition_and_no_phase73():
    report = _read(REPORT)
    assert 'Phase72' in report
    assert RUN_ID in report
    assert 'POST_SUCCESS_EXHAUSTED_PREMATURELY' in report
    assert 'max_goals=3' in report or 'max_goals=4' in report
    assert 'Goal2' in report
    assert '不进入 Phase73' in report or 'Do not enter Phase73' in report
    assert '不宣称 autonomous exploration success' in report or 'no autonomous exploration success claim' in report
    assert '不宣称 exit success' in report or 'no exit success claim' in report
