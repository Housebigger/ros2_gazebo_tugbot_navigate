import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / 'tools' / 'run_phase125_first_exploration_goal_execution_result_smoke.py'
ANALYZER = ROOT / 'tools' / 'analyze_phase125_first_exploration_goal_execution_result_smoke.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase125_first_exploration_goal_execution_result_smoke_report.md'


def _load(path: Path, module_name: str):
    assert path.exists(), f'missing module: {path}'
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _ready_handoff():
    return {
        'ingress_goal_result': {
            'accepted': True,
            'result_received': True,
            'result_status_label': 'SUCCEEDED',
            'locked_explicit_inner_ingress_goal': True,
        },
        'robot_pose_after_ingress': {'pose_available': True, 'x': 2.02, 'y': 0.01, 'yaw': 0.0, 'frame_id': 'map'},
        'distance_to_ingress_goal': {'meters': 0.03, 'within_tolerance': True},
        'orientation_error': {'radians': 0.0, 'within_tolerance': True},
        'costmap_freshness': {'fresh': True, 'global_costmap_available': True, 'local_costmap_available': True},
        'scan_freshness': {'fresh': True, 'scan_seen': True},
        'tf_freshness': {'fresh': True, 'map_base_available': True, 'odom_base_available': True, 'map_odom_available': True},
        'nav2_action_idle_state': {'idle': True, 'active_goal_count': 0, 'pending_goal_count': 0, 'executing_goal_count': 0, 'canceling_goal_count': 0},
        'preconditions': {
            'same_run_readiness_wait_passed': True,
            'preflight_passed': True,
            'inner_ingress_result_succeeded': True,
            'robot_pose_near_ingress_goal': True,
            'nav2_action_idle': True,
            'costmap_fresh': True,
            'scan_fresh': True,
            'tf_fresh': True,
        },
        'failure_reasons': [],
    }


def _dispatch(goal_kind='explore', sequence=1):
    return {
        'event': 'dispatch',
        'goal_sequence': sequence,
        'goal_kind': goal_kind,
        'target': [2.7, 0.0],
        'refined_target': [2.7, 0.0],
        'original_target': [2.7, 0.0],
        'branch_angle': 0.0,
        'last_candidate_count': 1,
        'last_open_direction_count': 1,
        'candidate_branch_count': 1,
        'candidate_branches': [{'rank': 1, 'target': [2.7, 0.0]}],
        'chosen_branch_rank': 1,
        'chosen_branch_score_components': {'score': -1.0},
        'dispatch_readiness_gate': {'ready': True},
        'local_topology': 'junction',
        'near_exit': False,
        'received_wall_time_sec': 100.0,
    }


def _result(event='success', status=None, reason='unit result'):
    return {
        'event': event,
        'goal_sequence': 1,
        'goal_kind': 'explore',
        'result_status': status,
        'result_reason': reason,
        'received_wall_time_sec': 130.0,
    }


def _evidence():
    return {
        'scan': {'seen': True, 'fresh': True, 'received_wall_time_sec': 130.0},
        'map': {'seen': True, 'received_wall_time_sec': 130.0},
        'odom': {'seen': True, 'received_wall_time_sec': 130.0},
        'tf': {'fresh': True, 'received_wall_time_sec': 130.0, 'translation': {'x': 2.65, 'y': 0.02, 'z': 0.0}},
        'local_costmap_samples': [{'seen': True, 'fresh': True, 'received_wall_time_sec': 130.0}],
        'nav2_feedback': [
            {'received_wall_time_sec': 110.0, 'distance_remaining': 0.8, 'number_of_recoveries': 0},
            {'received_wall_time_sec': 125.0, 'distance_remaining': 0.1, 'number_of_recoveries': 0},
        ],
        'action_status_samples': [{'statuses': [4], 'received_wall_time_sec': 130.0}],
    }


def _artifact_for(runner, events):
    record = runner.first_goal_result_from_observation(
        goal_events=events,
        explorer_states=[{'goal_count': 1, 'goal_active': False}],
        action_status_samples=_evidence()['action_status_samples'],
        evidence_samples=_evidence(),
        process_record={'returncode': None},
        dispatch_started_wall_time_sec=100.0,
        observation_timed_out=False,
    )
    return runner.build_phase125_artifact(
        run_id='unit',
        phase120_artifact={'classification': 'CONTROLLED_INGRESS_SUCCEEDED'},
        handoff_artifact=_ready_handoff(),
        first_goal_result_record=record,
        classification=runner.classify_phase125(_ready_handoff(), record),
    )


def test_phase125_runner_and_analyzer_modules_exist_with_contract_constants():
    runner = _load(RUNNER, 'phase125_runner_contract')
    analyzer = _load(ANALYZER, 'phase125_analyzer_contract')
    assert runner.PHASE == 'Phase125'
    assert runner.MODE == 'first_exploration_goal_execution_result_smoke'
    assert set(runner.CLASSIFICATIONS) == {
        'FIRST_EXPLORATION_GOAL_RESULT_SUCCEEDED_STOP',
        'FIRST_EXPLORATION_GOAL_RESULT_ABORTED_DIAGNOSTIC_FAIL',
        'FIRST_EXPLORATION_GOAL_RESULT_CANCELED_DIAGNOSTIC_FAIL',
        'FIRST_EXPLORATION_GOAL_RESULT_TIMEOUT_DIAGNOSTIC_FAIL',
        'FIRST_EXPLORATION_GOAL_REJECTED_DIAGNOSTIC_FAIL',
        'NO_VALID_FIRST_GOAL_FAIL_CLOSED',
        'HANDOFF_NOT_READY_NO_EXPLORATION',
    }
    assert analyzer.REQUIRED_RESULT_FIELDS == runner.REQUIRED_RESULT_FIELDS


def test_phase125_initial_schema_records_result_specific_required_fields():
    runner = _load(RUNNER, 'phase125_runner_schema')
    record = runner.initial_first_goal_result_record(max_goals=1)
    for key in runner.REQUIRED_RESULT_FIELDS:
        assert key in record, key
    first = record['first_goal']
    for key in ['candidate', 'pose', 'frame_id', 'goal_kind', 'selection_reason', 'accepted', 'rejected', 'nav2_feedback_timeline', 'result_status_label', 'abort_text', 'timeout', 'robot_pose_at_result', 'distance_to_first_goal']:
        assert key in first, key
    assert record['second_goal_dispatched'] is False
    assert record['dispatch_event_count'] == 0
    assert record['goal_event_count'] == 0


def test_phase125_classifies_success_aborted_canceled_timeout_rejected_distinctly():
    runner = _load(RUNNER, 'phase125_runner_classify')
    cases = [
        ([_dispatch(), _result('success', 4)], 'FIRST_EXPLORATION_GOAL_RESULT_SUCCEEDED_STOP', 'SUCCEEDED'),
        ([_dispatch(), _result('failure', 6, 'aborted')], 'FIRST_EXPLORATION_GOAL_RESULT_ABORTED_DIAGNOSTIC_FAIL', 'ABORTED'),
        ([_dispatch(), _result('failure', 5, 'canceled')], 'FIRST_EXPLORATION_GOAL_RESULT_CANCELED_DIAGNOSTIC_FAIL', 'CANCELED'),
        ([_dispatch(), _result('timeout', None, 'bounded timeout')], 'FIRST_EXPLORATION_GOAL_RESULT_TIMEOUT_DIAGNOSTIC_FAIL', 'TIMEOUT'),
        ([_dispatch(goal_kind='terminal')], 'FIRST_EXPLORATION_GOAL_REJECTED_DIAGNOSTIC_FAIL', 'REJECTED_NON_EXPLORE_GOAL_KIND'),
    ]
    for events, expected, status_label in cases:
        artifact = _artifact_for(runner, events)
        assert artifact['classification'] == expected
        assert artifact['first_goal']['result_status_label'] == status_label
        assert artifact['maze_explorer_max_goals'] == 1
        assert artifact['second_goal_dispatched'] is False


def test_phase125_no_valid_goal_and_handoff_not_ready_are_fail_closed():
    runner = _load(RUNNER, 'phase125_runner_fail_closed')
    no_goal = runner.first_goal_result_from_observation(
        goal_events=[],
        explorer_states=[{'mode': 'FAILED_EXHAUSTED', 'goal_count': 0}],
        action_status_samples=[],
        evidence_samples={},
        process_record={'returncode': 1},
        dispatch_started_wall_time_sec=None,
        observation_timed_out=False,
    )
    assert runner.classify_phase125(_ready_handoff(), no_goal) == 'NO_VALID_FIRST_GOAL_FAIL_CLOSED'
    handoff = _ready_handoff()
    handoff['preconditions']['tf_fresh'] = False
    handoff['tf_freshness']['fresh'] = False
    assert runner.classify_phase125(handoff, runner.initial_first_goal_result_record(max_goals=1)) == 'HANDOFF_NOT_READY_NO_EXPLORATION'


def test_phase125_analyzer_validates_second_goal_guardrail_and_result_evidence():
    runner = _load(RUNNER, 'phase125_runner_analyzer_sample')
    analyzer = _load(ANALYZER, 'phase125_analyzer_validation')
    artifact = _artifact_for(runner, [_dispatch(), _result('success', 4)])
    analysis = analyzer.analyze_artifact(artifact)
    assert analysis['valid'] is True
    assert analysis['classification_matches_evidence'] is True
    assert analysis['guardrails']['second_goal_dispatched_false'] is True
    assert analysis['evidence_summary']['nav2_feedback_count'] == 2
    assert analysis['evidence_summary']['robot_pose_at_result_available'] is True
    artifact['first_goal_result_artifact']['goal_events'].append(_dispatch(sequence=2))
    artifact['first_goal_result_artifact']['dispatch_event_count'] = 2
    artifact['first_goal_result_artifact']['second_goal_dispatched'] = True
    bad = analyzer.analyze_artifact(artifact)
    assert bad['valid'] is False
    assert bad['guardrails']['second_goal_dispatched_false'] is False
    assert bad['guardrails']['dispatch_event_count_one_or_less'] is False


def test_phase125_report_records_runtime_boundaries_and_no_success_overclaim():
    assert REPORT.exists(), f'missing Phase125 report: {REPORT}'
    text = REPORT.read_text(encoding='utf-8')
    required = [
        'PHASE125',
        'First exploration goal execution result smoke',
        'max_goals=1',
        'goal_kind=explore',
        'second_goal_dispatched=false',
        'robot_pose_at_result',
        'distance_to_first_goal',
        'not autonomous exploration success',
        'not exit success',
        'Phase126 not entered',
        'Nav2 config diff guard',
        'process guard',
    ]
    for phrase in required:
        assert phrase in text
