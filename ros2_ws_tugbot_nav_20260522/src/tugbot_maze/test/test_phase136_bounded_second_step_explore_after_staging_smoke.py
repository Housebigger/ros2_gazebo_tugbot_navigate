import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / 'tools' / 'run_phase136_bounded_second_step_explore_after_staging_smoke.py'
ANALYZER = ROOT / 'tools' / 'analyze_phase136_bounded_second_step_explore_after_staging_smoke.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase136_bounded_second_step_explore_after_staging_smoke_report.md'


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
        'target': {'x': 1.68, 'y': 0.02, 'yaw': 0.0, 'frame_id': 'map'},
        'original_target': {'x': 1.68, 'y': 0.97},
        'refined_target': {'x': 1.68, 'y': 0.97},
        'staging_goal_pose': {
            'x': 1.68,
            'y': 0.02,
            'yaw': 0.0,
            'frame_id': 'map',
            'lateral_residual_before_m': 0.175,
            'lateral_residual_after_m': 0.0,
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
            'x': 1.68,
            'y': 0.97,
            'yaw': 1.57,
            'frame_id': 'map',
        },
        'phase62_front_wedge_cost': {'max': 99, 'mean': 47.8, 'high_cost_count': 40, 'lethal_count': 9},
        'received_wall_time_sec': 10.0,
    }


def _staging_success(seq=1):
    return {
        'event': 'success',
        'goal_sequence': seq,
        'goal_kind': 'corridor_alignment_staging',
        'result_status': 4,
        'result_reason': 'succeeded',
        'received_wall_time_sec': 12.0,
    }


def _second_step_dispatch(seq=2, *, skip_guard=True, generated=True, selected=True):
    second = {
        'generated_after_fresh_evidence': generated,
        'fresh_scan_received': True,
        'fresh_local_costmap_received': True,
        'fresh_tf_received': True,
        'selected_candidate_target': [1.68, 0.97] if selected else None,
        'selected_candidate_yaw': 1.57,
        'hard_safety_pass_candidate_count': 1,
        'front_wedge_risk_after_staging': {'max': 12, 'mean': 3.2, 'high_cost_count': 0},
        'forward_refinement_result': {
            'refinement_applied': generated,
            'selected_candidate_target': [1.68, 0.97] if selected else None,
            'selected_candidate_yaw': 1.57,
        },
    }
    return {
        'event': 'dispatch',
        'goal_sequence': seq,
        'goal_kind': 'explore',
        'target': {'x': 1.68, 'y': 0.97, 'yaw': 1.57, 'frame_id': 'map'},
        'original_target': {'x': 1.68, 'y': 0.97},
        'selected_due_to_context': 'phase92_corridor_alignment_staging_second_step',
        'second_step_forward_goal': second,
        'staging_goal_pose': _staging_dispatch()['staging_goal_pose'],
        'staging_applied': True,
        'two_step_stage_dispatch_requested': False,
        'skip_two_step_staging': skip_guard,
        'phase136_recursion_guard': skip_guard,
        'phase62_front_wedge_cost': {'max': 12, 'mean': 3.2, 'high_cost_count': 0},
        'received_wall_time_sec': 13.0,
    }


def _second_terminal(event='success', seq=2, status=4, reason='succeeded'):
    return {
        'event': event,
        'goal_sequence': seq,
        'goal_kind': 'explore',
        'result_status': status,
        'result_reason': reason,
        'received_wall_time_sec': 15.0,
    }


def _record(runner, events, *, observation_timed_out=False):
    record = runner.initial_bounded_second_step_record(max_goals=2)
    record.update({
        'maze_explorer_start_allowed': True,
        'maze_explorer_started': True,
        'goal_events': events,
        'observation_timed_out': observation_timed_out,
        'freshness_after_staging': {
            'fresh_scan_received': True,
            'fresh_local_costmap_received': True,
            'fresh_tf_received': True,
            'fresh_scan_sample_time_sec': 12.3,
            'fresh_local_costmap_sample_time_sec': 12.4,
            'fresh_tf_sample_time_sec': 12.5,
        },
        'pending_corridor_alignment_second_step': {
            'available': True,
            'exists': True,
            'original_goal_kind': 'explore',
            'original_target': {'x': 1.68, 'y': 0.97},
        },
        'front_wedge_risk_after_staging': {'max': 12, 'mean': 3.2, 'high_cost_count': 0},
        'lateral_residual_after': 0.0,
    })
    return runner.finalize_record_from_events(record)


def _artifact(runner, events, *, observation_timed_out=False):
    record = _record(runner, events, observation_timed_out=observation_timed_out)
    return runner.build_phase136_artifact(
        run_id='unit',
        phase120_artifact={'classification': 'INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER'},
        handoff_artifact={'preconditions': {'inner_ingress_result_succeeded': True}},
        bounded_record=record,
    )


def test_phase136_runner_and_analyzer_contract_constants():
    runner = _load(RUNNER, 'phase136_runner_contract')
    analyzer = _load(ANALYZER, 'phase136_analyzer_contract')
    assert runner.PHASE == 'Phase136'
    assert runner.MODE == 'bounded_second_step_explore_after_staging_smoke'
    assert runner.LOCKED_GOAL == {'frame_id': 'map', 'x': 2.0, 'y': 0.0, 'yaw': 0.0}
    assert runner.CLASSIFICATIONS == [
        'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_ACCEPTED_STOP',
        'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_REJECTED_DIAGNOSTIC_FAIL',
        'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_TIMEOUT_DIAGNOSTIC_FAIL',
        'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_RESULT_SUCCEEDED_STOP',
        'STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED',
        'SECOND_STEP_CONTRACT_AMBIGUOUS',
    ]
    assert analyzer.CLASSIFICATIONS == runner.CLASSIFICATIONS
    assert runner.FORBIDDEN_ACTIONS == {
        'third_goal': False,
        'full_exploration': False,
        'manual_goal1_carry_over_branch_centerline_fallback_terminal_exit_goal': False,
        'nav2_mppi_controller_goal_checker_config_tuning': False,
        'exploration_strategy_branch_scoring_centerline_fallback_terminal_acceptance_change': False,
        'direct_staging_disablement': False,
        'autonomous_or_exit_success_claim': False,
        'phase127_timeout_fixed_claim': False,
    }


def test_phase136_initial_record_contains_required_schema_and_max_goals_two():
    runner = _load(RUNNER, 'phase136_runner_schema')
    record = runner.initial_bounded_second_step_record(max_goals=2)
    for key in runner.REQUIRED_RESULT_FIELDS:
        assert key in record, key
    assert record['maze_explorer_max_goals'] == 2
    assert record['second_step_attempted'] is False
    assert record['third_goal_dispatched'] is False
    first = record['first_literal_dispatch']
    second = record['second_step_dispatch']
    for key in ['original_target', 'staging_target', 'result_status_label', 'pending_corridor_alignment_second_step', 'second_step_forward_goal']:
        assert key in first, key
    for key in [
        'available', 'goal_kind', 'second_step_forward_goal', 'freshness_after_staging',
        'front_wedge_risk_after_staging', 'lateral_residual_after',
        'skip_two_step_staging', 'recursion_guard', 'accepted', 'rejected',
        'result_status_label', 'abort_text', 'timeout',
    ]:
        assert key in second, key


def test_phase136_classifies_second_step_success_result_and_preserves_separate_staging_evidence():
    runner = _load(RUNNER, 'phase136_runner_success')
    analyzer = _load(ANALYZER, 'phase136_analyzer_success')
    artifact = _artifact(runner, [_staging_dispatch(), _staging_success(), _second_step_dispatch(), _second_terminal('success')])
    assert artifact['classification'] == 'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_RESULT_SUCCEEDED_STOP'
    assert artifact['first_literal_dispatch']['goal_kind'] == 'corridor_alignment_staging'
    assert artifact['first_literal_dispatch']['result_status_label'] == 'SUCCEEDED'
    assert artifact['second_step_dispatch']['goal_kind'] == 'explore'
    assert artifact['second_step_dispatch']['skip_two_step_staging'] is True
    assert artifact['second_step_dispatch']['second_step_forward_goal']['generated_after_fresh_evidence'] is True
    assert artifact['second_step_attempted'] is True
    assert artifact['second_step_goal_count'] == 1
    assert artifact['third_goal_dispatched'] is False
    assert artifact['guardrails']['one_second_step_goal_only'] is True
    assert artifact['guardrails']['third_goal_dispatched_false'] is True
    analysis = analyzer.analyze_artifact(artifact)
    assert analysis['valid'] is True
    assert analysis['classification'] == 'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_RESULT_SUCCEEDED_STOP'
    assert analysis['claims']['autonomous_exploration_success'] is False
    assert analysis['claims']['exit_success'] is False
    assert analysis['claims']['phase127_timeout_fixed'] is False


def test_phase136_classifies_second_step_accepted_stop_without_terminal_result():
    runner = _load(RUNNER, 'phase136_runner_accepted')
    artifact = _artifact(runner, [_staging_dispatch(), _staging_success(), _second_step_dispatch()])
    assert artifact['classification'] == 'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_ACCEPTED_STOP'
    assert artifact['second_step_dispatch']['accepted'] is True
    assert artifact['second_step_dispatch']['result_status_label'] == 'ACCEPTED'
    assert artifact['stop_reason'] == 'second_step_explore_accepted_bounded_stop'


def test_phase136_classifies_rejected_timeout_and_not_ready_fail_closed():
    runner = _load(RUNNER, 'phase136_runner_failures')
    rejected = _artifact(runner, [_staging_dispatch(), _staging_success(), _second_step_dispatch(), _second_terminal('failure', status=6, reason='blocked_nav2')])
    assert rejected['classification'] == 'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_REJECTED_DIAGNOSTIC_FAIL'
    timed_out = _artifact(runner, [_staging_dispatch(), _staging_success(), _second_step_dispatch(), _second_terminal('timeout', status=5, reason='goal_timeout')], observation_timed_out=True)
    assert timed_out['classification'] == 'STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_TIMEOUT_DIAGNOSTIC_FAIL'
    not_ready = _artifact(runner, [_staging_dispatch(), _staging_success()])
    assert not_ready['classification'] == 'STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED'
    assert not_ready['second_step_attempted'] is False
    assert not_ready['third_goal_dispatched'] is False


def test_phase136_rejects_missing_recursion_guard_or_missing_fresh_evidence_as_ambiguous():
    runner = _load(RUNNER, 'phase136_runner_guard_fail')
    analyzer = _load(ANALYZER, 'phase136_analyzer_guard_fail')
    no_guard = _artifact(runner, [_staging_dispatch(), _staging_success(), _second_step_dispatch(skip_guard=False)])
    no_guard_analysis = analyzer.analyze_artifact(no_guard)
    assert no_guard_analysis['valid'] is False
    assert no_guard_analysis['classification'] == 'SECOND_STEP_CONTRACT_AMBIGUOUS'
    assert no_guard_analysis['guardrails']['second_step_recursion_guard_present'] is False

    stale = _artifact(runner, [_staging_dispatch(), _staging_success(), _second_step_dispatch(generated=False)])
    stale_analysis = analyzer.analyze_artifact(stale)
    assert stale_analysis['valid'] is False
    assert stale_analysis['classification'] == 'SECOND_STEP_CONTRACT_AMBIGUOUS'
    assert stale_analysis['guardrails']['second_step_generated_after_fresh_evidence'] is False


def test_phase136_rejects_third_goal_dispatch_guard_violation():
    runner = _load(RUNNER, 'phase136_runner_third_goal')
    analyzer = _load(ANALYZER, 'phase136_analyzer_third_goal')
    third = {
        'event': 'dispatch',
        'goal_sequence': 3,
        'goal_kind': 'explore',
        'target': {'x': 2.0, 'y': 2.0, 'yaw': 0.0, 'frame_id': 'map'},
        'received_wall_time_sec': 16.0,
    }
    artifact = _artifact(runner, [_staging_dispatch(), _staging_success(), _second_step_dispatch(), third])
    assert artifact['third_goal_dispatched'] is True
    analysis = analyzer.analyze_artifact(artifact)
    assert analysis['valid'] is False
    assert analysis['classification'] == 'SECOND_STEP_CONTRACT_AMBIGUOUS'
    assert analysis['guardrails']['third_goal_dispatched_false'] is False


def test_phase136_report_records_completion_boundaries_after_runtime():
    if not REPORT.exists():
        return
    text = REPORT.read_text(encoding='utf-8')
    required = [
        'PHASE136_BOUNDED_SECOND_STEP_EXPLORE_AFTER_STAGING_SMOKE_COMPLETE_STOP_BEFORE_PHASE137',
        'visible-stack',
        'explicit inner-ingress goal: map,x=2.0,y=0.0,yaw=0.0',
        'max_goals=2',
        'first literal corridor_alignment_staging',
        'second_step goal_kind=explore',
        'skip_two_step_staging',
        'third_goal_dispatched=false',
        'No Nav2/MPPI/controller/goal checker/config tuning',
        'No autonomous exploration success or exit success is claimed',
        'Phase127 timeout fixed claim=false',
        'Phase137 not entered',
    ]
    for phrase in required:
        assert phrase in text
