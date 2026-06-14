import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / 'tools' / 'run_phase134_bounded_corridor_alignment_staging_smoke.py'
ANALYZER = ROOT / 'tools' / 'analyze_phase134_bounded_corridor_alignment_staging_smoke.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase134_bounded_corridor_alignment_staging_smoke_report.md'


def _load(path: Path, module_name: str):
    assert path.exists(), f'missing module: {path}'
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _staging_dispatch(seq=1):
    return {
        'event': 'dispatch',
        'goal_sequence': seq,
        'goal_kind': 'corridor_alignment_staging',
        'target': {'x': 2.08, 'y': 0.12, 'yaw': 0.0, 'frame_id': 'map'},
        'original_target': {'x': 2.08, 'y': 1.02},
        'refined_target': {'x': 2.08, 'y': 1.02},
        'staging_goal_pose': {
            'x': 2.08,
            'y': 0.12,
            'yaw': 0.0,
            'frame_id': 'map',
            'lateral_residual_before_m': 0.52,
            'lateral_residual_after_m': 0.06,
            'staging_distance_m': 0.28,
        },
        'staging_applied': True,
        'two_step_stage_dispatch_requested': True,
        'staging_reason': 'reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal',
        'staging_executability_check': {
            'checked': True,
            'hard_safety_pass': True,
            'lateral_residual_reduced': True,
        },
        'second_step_forward_goal': {
            'x': 2.08,
            'y': 1.02,
            'yaw': 1.57,
            'frame_id': 'map',
        },
        'source_forward_window': {'sample_count': 9},
        'staging_window': {'sample_count': 7},
        'phase62_front_wedge_cost': {'max': 42, 'mean': 11.0, 'high_cost_count': 0},
        'dispatch_target_local_cost_max_radius': 40,
        'path_corridor_min_clearance_m': 0.61,
        'received_wall_time_sec': 10.0,
    }


def _explore_dispatch(seq=1):
    return {
        'event': 'dispatch',
        'goal_sequence': seq,
        'goal_kind': 'explore',
        'target': {'x': 2.08, 'y': 1.02, 'yaw': 1.57, 'frame_id': 'map'},
        'original_target': {'x': 2.08, 'y': 1.02},
        'received_wall_time_sec': 10.0,
    }


def _terminal_event(event='success', seq=1, status=4, reason='succeeded'):
    return {
        'event': event,
        'goal_sequence': seq,
        'goal_kind': 'corridor_alignment_staging',
        'result_status': status,
        'result_reason': reason,
        'received_wall_time_sec': 12.0,
    }


def _artifact(runner, events, *, observation_timed_out=False, status_label=None, rejected=False):
    record = runner.initial_bounded_staging_record(max_goals=1)
    record.update({
        'maze_explorer_start_allowed': True,
        'maze_explorer_started': True,
        'goal_events': events,
        'goal_event_count': len(events),
        'dispatch_event_count': sum(1 for event in events if event.get('event') == 'dispatch'),
        'second_goal_dispatched': sum(1 for event in events if event.get('event') == 'dispatch') > 1,
        'second_step_attempted': any(event.get('goal_kind') == 'explore' for event in events[1:]),
        'observation_timed_out': observation_timed_out,
    })
    record = runner.finalize_record_from_events(record)
    if status_label is not None:
        record['first_literal_dispatch']['result_status_label'] = status_label
    if rejected:
        record['first_literal_dispatch']['rejected'] = True
        record['first_literal_dispatch']['accepted'] = False
    return runner.build_phase134_artifact(
        run_id='unit',
        phase120_artifact={'classification': 'INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER'},
        handoff_artifact={'preconditions': {'inner_ingress_result_succeeded': True}},
        bounded_record=record,
    )


def test_phase134_runner_and_analyzer_contract_constants():
    runner = _load(RUNNER, 'phase134_runner_contract')
    analyzer = _load(ANALYZER, 'phase134_analyzer_contract')
    assert runner.PHASE == 'Phase134'
    assert runner.MODE == 'bounded_corridor_alignment_staging_smoke'
    assert runner.LOCKED_GOAL == {'frame_id': 'map', 'x': 2.0, 'y': 0.0, 'yaw': 0.0}
    assert runner.CLASSIFICATIONS == [
        'FIRST_DISPATCH_EXPLORE_ACCEPTED_STOP',
        'FIRST_DISPATCH_STAGING_ACCEPTED_STOP',
        'FIRST_DISPATCH_STAGING_REJECTED_DIAGNOSTIC_FAIL',
        'FIRST_DISPATCH_STAGING_TIMEOUT_DIAGNOSTIC_FAIL',
        'STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP',
        'DISPATCH_KIND_CONTRACT_AMBIGUOUS',
    ]
    assert analyzer.CLASSIFICATIONS == runner.CLASSIFICATIONS
    assert runner.FORBIDDEN_ACTIONS == {
        'second_step_goal_kind_explore': False,
        'second_exploration_goal': False,
        'manual_goal1_carry_over_staging_branch_centerline_fallback_terminal_exit_goal': False,
        'nav2_mppi_controller_goal_checker_config_tuning': False,
        'exploration_strategy_branch_scoring_centerline_fallback_terminal_acceptance_change': False,
        'direct_staging_disablement': False,
        'autonomous_or_exit_success_claim': False,
    }


def test_phase134_initial_record_contains_required_staging_contract_fields():
    runner = _load(RUNNER, 'phase134_runner_schema')
    record = runner.initial_bounded_staging_record(max_goals=1)
    for key in runner.REQUIRED_RESULT_FIELDS:
        assert key in record, key
    first = record['first_literal_dispatch']
    for key in [
        'goal_kind',
        'original_target',
        'refined_target',
        'staging_target',
        'staging_applied',
        'two_step_stage_dispatch_requested',
        'staging_reason',
        'lateral_residual_before_m',
        'lateral_residual_after_m',
        'front_wedge_risk',
        'staging_executability_check',
        'pending_corridor_alignment_second_step',
        'second_step_forward_goal',
        'accepted',
        'rejected',
        'result_status_label',
        'abort_text',
        'timeout',
        'second_step_attempted',
        'second_goal_dispatched',
    ]:
        assert key in first, key
    assert record['maze_explorer_max_goals'] == 1
    assert record['second_step_attempted'] is False
    assert record['second_goal_dispatched'] is False


def test_phase134_classifies_staging_success_as_bounded_staging_stop_only():
    runner = _load(RUNNER, 'phase134_runner_staging_success')
    analyzer = _load(ANALYZER, 'phase134_analyzer_staging_success')
    artifact = _artifact(runner, [_staging_dispatch(), _terminal_event('success')])
    assert artifact['classification'] == 'FIRST_DISPATCH_STAGING_ACCEPTED_STOP'
    assert artifact['first_literal_dispatch']['goal_kind'] == 'corridor_alignment_staging'
    assert artifact['first_literal_dispatch']['accepted'] is True
    assert artifact['first_literal_dispatch']['result_status_label'] == 'SUCCEEDED'
    assert artifact['second_step_attempted'] is False
    assert artifact['second_goal_dispatched'] is False
    assert artifact['guardrails']['max_goals_one'] is True
    assert artifact['guardrails']['second_step_attempted_false'] is True
    assert artifact['guardrails']['no_autonomous_success_claim'] is True
    assert artifact['guardrails']['no_exit_success_claim'] is True
    analysis = analyzer.analyze_artifact(artifact)
    assert analysis['valid'] is True
    assert analysis['classification'] == 'FIRST_DISPATCH_STAGING_ACCEPTED_STOP'


def test_phase134_classifies_staging_timeout_and_rejection_diagnostic_failures():
    runner = _load(RUNNER, 'phase134_runner_staging_failures')
    timeout_artifact = _artifact(runner, [_staging_dispatch(), _terminal_event('timeout', status=5, reason='goal_timeout')], observation_timed_out=True)
    assert timeout_artifact['classification'] == 'FIRST_DISPATCH_STAGING_TIMEOUT_DIAGNOSTIC_FAIL'
    reject_artifact = _artifact(runner, [_staging_dispatch()], status_label='REJECTED', rejected=True)
    assert reject_artifact['classification'] == 'FIRST_DISPATCH_STAGING_REJECTED_DIAGNOSTIC_FAIL'


def test_phase134_classifies_explore_first_dispatch_as_accepted_stop_not_second_step():
    runner = _load(RUNNER, 'phase134_runner_explore_first')
    artifact = _artifact(runner, [_explore_dispatch()], status_label='ACCEPTED')
    assert artifact['classification'] == 'FIRST_DISPATCH_EXPLORE_ACCEPTED_STOP'
    assert artifact['first_literal_dispatch']['goal_kind'] == 'explore'
    assert artifact['first_literal_dispatch']['is_first_exploration_goal'] is True
    assert artifact['second_step_attempted'] is False
    assert artifact['second_goal_dispatched'] is False


def test_phase134_analyzer_rejects_second_dispatch_or_missing_staging_fields():
    runner = _load(RUNNER, 'phase134_runner_guard_violations')
    analyzer = _load(ANALYZER, 'phase134_analyzer_guard_violations')
    second = _explore_dispatch(seq=2)
    artifact = _artifact(runner, [_staging_dispatch(), second])
    analysis = analyzer.analyze_artifact(artifact)
    assert analysis['valid'] is False
    assert analysis['guardrails']['second_goal_dispatched_false'] is False
    assert analysis['guardrails']['second_step_attempted_false'] is False
    bad = _artifact(runner, [{**_staging_dispatch(), 'staging_goal_pose': None}])
    bad_analysis = analyzer.analyze_artifact(bad)
    assert bad_analysis['classification'] == 'DISPATCH_KIND_CONTRACT_AMBIGUOUS'
    assert bad_analysis['valid'] is False


def test_phase134_report_records_runtime_boundaries_after_completion():
    if not REPORT.exists():
        return
    text = REPORT.read_text(encoding='utf-8')
    required = [
        'PHASE134_BOUNDED_CORRIDOR_ALIGNMENT_STAGING_SMOKE_COMPLETE_STOP_BEFORE_PHASE135',
        'visible-stack',
        'explicit inner-ingress goal: map,x=2.0,y=0.0,yaw=0.0',
        'max_goals=1',
        'first_literal_dispatch.goal_kind',
        'staging_applied',
        'two_step_stage_dispatch_requested',
        'pending_corridor_alignment_second_step',
        'second_step_attempted=false',
        'second_goal_dispatched=false',
        'No second-step goal_kind=explore',
        'No Nav2/MPPI/controller/goal checker/config tuning',
        'No autonomous exploration success or exit success is claimed',
        'Phase135 not entered',
    ]
    for phrase in required:
        assert phrase in text
