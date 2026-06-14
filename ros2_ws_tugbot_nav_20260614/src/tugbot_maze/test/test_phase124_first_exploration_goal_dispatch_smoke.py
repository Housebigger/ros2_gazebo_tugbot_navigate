import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / 'tools' / 'run_phase124_first_exploration_goal_dispatch_smoke.py'
ANALYZER = ROOT / 'tools' / 'analyze_phase124_first_exploration_goal_dispatch_smoke.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase124_first_exploration_goal_dispatch_smoke_report.md'


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


def _sample_dispatch(goal_kind='explore', event='dispatch'):
    return {
        'event': event,
        'goal_sequence': 1,
        'goal_kind': goal_kind,
        # maze_explorer emits target/refined/original points as JSON lists in real /maze/goal_events.
        'target': [2.7, 0.0],
        'refined_target': [2.7, 0.0],
        'original_target': [2.7, 0.0],
        'dispatch_pose': {'frame_id': 'map', 'x': 2.02, 'y': 0.01, 'yaw': 0.0},
        'branch_angle': 0.0,
        'last_candidate_count': 1,
        'last_open_direction_count': 1,
        'candidate_branch_count': 1,
        'candidate_branches': [{'rank': 0, 'angle': 0.0}],
        'chosen_branch_rank': 0,
        'chosen_branch_score_components': {'distance': 1.0},
        'dispatch_readiness_gate': {'ready': True},
        'local_topology': 'corridor',
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
        'near_exit': False,
    }


def _accepted_artifact(runner):
    first_goal = runner.first_goal_from_observation(
        goal_events=[_sample_dispatch()],
        explorer_states=[{'goal_active': True, 'goal_count': 1}],
        action_status_samples=[{'active_goal_count': 1, 'executing_goal_count': 1, 'pending_goal_count': 0, 'canceling_goal_count': 0}],
        evidence_samples={'scan': {'seen': True}, 'map': {'seen': True}, 'odom': {'seen': True}, 'tf': {'fresh': True}, 'local_costmap_samples': [{'seen': True}]},
        process_record={'returncode': None},
        dispatch_started_wall_time_sec=123.0,
        observation_timed_out=False,
    )
    return runner.build_phase124_artifact(
        run_id='unit',
        phase120_artifact={'classification': 'CONTROLLED_INGRESS_SUCCEEDED'},
        handoff_artifact=_ready_handoff(),
        first_goal_record=first_goal,
        classification=runner.classify_phase124(_ready_handoff(), first_goal),
    )


def test_phase124_runner_and_analyzer_modules_exist_with_contract_constants():
    runner = _load(RUNNER, 'phase124_runner_contract')
    analyzer = _load(ANALYZER, 'phase124_analyzer_contract')
    assert runner.PHASE == 'Phase124'
    assert runner.MODE == 'first_exploration_goal_dispatch_smoke'
    assert set(runner.CLASSIFICATIONS) == {
        'FIRST_EXPLORATION_GOAL_DISPATCHED_ACCEPTED_STOP',
        'FIRST_EXPLORATION_GOAL_REJECTED_DIAGNOSTIC_FAIL',
        'FIRST_EXPLORATION_GOAL_TIMEOUT_DIAGNOSTIC_FAIL',
        'NO_VALID_FIRST_GOAL_FAIL_CLOSED',
        'HANDOFF_NOT_READY_NO_EXPLORATION',
    }
    assert analyzer.REQUIRED_FIRST_GOAL_FIELDS == runner.REQUIRED_FIRST_GOAL_FIELDS


def test_phase124_initial_first_goal_schema_records_required_evidence_fields():
    runner = _load(RUNNER, 'phase124_runner_schema')
    first = runner.initial_first_goal_record(max_goals=1)
    for key in runner.REQUIRED_FIRST_GOAL_FIELDS:
        assert key in first, key
    assert first['maze_explorer_max_goals'] == 1
    assert first['first_goal']['frame_id'] is None
    assert first['first_goal']['candidate'] == {}
    assert first['first_goal']['pose'] == {}
    assert first['first_goal']['goal_kind'] is None
    assert first['goal_event_count'] == 0
    assert first['dispatch_event_count'] == 0
    assert first['stop_reason'] is None


def test_phase124_handoff_not_ready_is_fail_closed_and_does_not_start_explorer():
    runner = _load(RUNNER, 'phase124_runner_handoff')
    handoff = _ready_handoff()
    handoff['preconditions']['scan_fresh'] = False
    handoff['scan_freshness']['fresh'] = False
    first = runner.initial_first_goal_record(max_goals=1)
    classification = runner.classify_phase124(handoff, first)
    assert classification == 'HANDOFF_NOT_READY_NO_EXPLORATION'
    artifact = runner.build_phase124_artifact(
        run_id='unit',
        phase120_artifact={},
        handoff_artifact=handoff,
        first_goal_record=first,
        classification=classification,
    )
    assert artifact['maze_explorer_started'] is False
    assert artifact['exploration_goal_dispatched'] is False
    assert artifact['guardrails']['handoff_ready_required'] is True


def test_phase124_accepts_exactly_one_current_algorithm_explore_goal_then_stop():
    runner = _load(RUNNER, 'phase124_runner_accept')
    artifact = _accepted_artifact(runner)
    first = artifact['first_goal_artifact']
    assert artifact['classification'] == 'FIRST_EXPLORATION_GOAL_DISPATCHED_ACCEPTED_STOP'
    assert artifact['maze_explorer_max_goals'] == 1
    assert artifact['goal_event_count'] == 1
    assert artifact['dispatch_event_count'] == 1
    assert artifact['first_goal']['goal_kind'] == 'explore'
    assert artifact['first_goal']['accepted'] is True
    assert artifact['first_goal']['result_status_label'] == 'ACCEPTED'
    assert artifact['stop_reason'] == 'first_explore_goal_accepted_stop'
    assert artifact['guardrails']['manual_goal1_forbidden'] is True
    assert artifact['guardrails']['fallback_terminal_exit_goal_forbidden_as_success'] is True
    assert artifact['guardrails']['autonomous_success_claimed'] is False
    assert artifact['guardrails']['exit_success_claimed'] is False


def test_phase124_rejects_non_explore_or_fallback_terminal_exit_as_success():
    runner = _load(RUNNER, 'phase124_runner_reject_non_explore')
    for goal_kind in ['backtrack', 'near_exit_micro_goal', 'terminal', 'corridor_alignment_staging']:
        first = runner.first_goal_from_observation(
            goal_events=[_sample_dispatch(goal_kind=goal_kind)],
            explorer_states=[],
            action_status_samples=[{'active_goal_count': 1, 'executing_goal_count': 1}],
            evidence_samples={},
            process_record={},
            dispatch_started_wall_time_sec=0.0,
            observation_timed_out=False,
        )
        assert first['first_goal']['goal_kind'] == goal_kind
        assert runner.classify_phase124(_ready_handoff(), first) == 'FIRST_EXPLORATION_GOAL_REJECTED_DIAGNOSTIC_FAIL'
        assert first['first_goal']['accepted'] is False


def test_phase124_timeout_and_no_valid_goal_classifications_are_distinct():
    runner = _load(RUNNER, 'phase124_runner_timeout')
    timeout_event = dict(_sample_dispatch(), event='timeout')
    first_timeout = runner.first_goal_from_observation(
        goal_events=[_sample_dispatch(), timeout_event],
        explorer_states=[],
        action_status_samples=[{'active_goal_count': 1, 'executing_goal_count': 1}],
        evidence_samples={},
        process_record={},
        dispatch_started_wall_time_sec=0.0,
        observation_timed_out=True,
    )
    assert runner.classify_phase124(_ready_handoff(), first_timeout) == 'FIRST_EXPLORATION_GOAL_TIMEOUT_DIAGNOSTIC_FAIL'
    first_none = runner.first_goal_from_observation(
        goal_events=[],
        explorer_states=[{'mode': 'FAILED_EXHAUSTED', 'goal_count': 0}],
        action_status_samples=[],
        evidence_samples={},
        process_record={'returncode': 1},
        dispatch_started_wall_time_sec=0.0,
        observation_timed_out=False,
    )
    assert runner.classify_phase124(_ready_handoff(), first_none) == 'NO_VALID_FIRST_GOAL_FAIL_CLOSED'


def test_phase124_analyzer_validates_guardrails_and_rejects_second_dispatch():
    runner = _load(RUNNER, 'phase124_runner_analyzer_sample')
    analyzer = _load(ANALYZER, 'phase124_analyzer_validation')
    artifact = _accepted_artifact(runner)
    analysis = analyzer.analyze_artifact(artifact)
    assert analysis['valid'] is True
    assert analysis['classification_matches_evidence'] is True
    assert analysis['guardrails']['exactly_one_dispatch_event'] is True
    artifact['first_goal_artifact']['goal_events'].append(dict(_sample_dispatch(), goal_sequence=2))
    artifact['first_goal_artifact']['dispatch_event_count'] = 2
    bad = analyzer.analyze_artifact(artifact)
    assert bad['valid'] is False
    assert bad['guardrails']['exactly_one_dispatch_event'] is False


def test_phase124_report_records_runtime_boundaries_and_no_success_overclaim():
    assert REPORT.exists(), f'missing Phase124 report: {REPORT}'
    text = REPORT.read_text(encoding='utf-8')
    required = [
        'PHASE124',
        'First exploration goal dispatch smoke',
        'max_goals=1',
        'goal_kind=explore',
        'not autonomous exploration success',
        'not exit success',
        'Phase125 not entered',
        'Nav2 config diff guard',
        'process guard',
    ]
    for phrase in required:
        assert phrase in text
