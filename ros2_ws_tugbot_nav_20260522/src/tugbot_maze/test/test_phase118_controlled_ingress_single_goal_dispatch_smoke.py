from __future__ import annotations

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / 'tools' / 'run_phase118_controlled_ingress_single_goal_dispatch_smoke.py'
ANALYZER = ROOT / 'tools' / 'analyze_phase118_controlled_ingress_single_goal_dispatch_smoke.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase118_controlled_ingress_single_goal_dispatch_smoke_report.md'


def _load(path: Path, name: str):
    assert path.exists(), f'missing required Phase118 file: {path}'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _text(path: Path) -> str:
    assert path.exists(), f'missing required Phase118 file: {path}'
    return path.read_text(encoding='utf-8')


def test_phase118_runner_locks_dispatch_preconditions_and_goal_identity():
    mod = _load(RUNNER, 'phase118_runner')
    passing_preflight = {
        'evaluated': True,
        'passed': True,
        'failed_gates': [],
        'ingress_preflight_reject_reason': None,
    }
    decision = mod.evaluate_dispatch_preconditions(passing_preflight, action_server_ready=True)
    assert decision['allowed'] is True
    assert decision['reasons'] == []

    failing_cases = [
        ({'evaluated': True, 'passed': False, 'failed_gates': [], 'ingress_preflight_reject_reason': None}, True),
        ({'evaluated': True, 'passed': True, 'failed_gates': ['ingress_map_base_tf_missing'], 'ingress_preflight_reject_reason': None}, True),
        ({'evaluated': True, 'passed': True, 'failed_gates': [], 'ingress_preflight_reject_reason': 'ingress_preflight_timeout'}, True),
        (passing_preflight, False),
    ]
    for preflight, action_ready in failing_cases:
        decision = mod.evaluate_dispatch_preconditions(preflight, action_server_ready=action_ready)
        assert decision['allowed'] is False
        assert decision['classification'] == 'PREFLIGHT_FAILED_NO_DISPATCH'

    goal = mod.locked_goal_pose(stamp={'sec': 12, 'nanosec': 34})
    assert goal['frame_id'] == 'map'
    assert goal['x'] == 2.0
    assert goal['y'] == 0.0
    assert goal['yaw'] == 0.0
    assert goal['stamp'] == {'sec': 12, 'nanosec': 34}


def test_phase118_runner_state_transition_and_classification_contract():
    mod = _load(RUNNER, 'phase118_runner_state')
    base = mod.initial_dispatch_record(bounded_goal_result_wait_sec=30.0)
    assert base['ingress_goal_sent'] is False
    assert base['maze_explorer_started'] is False
    assert base['goal_pose']['frame_id'] == 'map'

    rejected = dict(base, dispatch_attempted=True, accepted=False, rejected=True)
    assert mod.classify_dispatch_record(rejected) == 'INGRESS_DISPATCH_REJECTED_DIAGNOSTIC_FAIL'
    assert rejected['ingress_goal_sent'] is False

    timeout = dict(base, dispatch_attempted=True, accepted=True, rejected=False, ingress_goal_sent=True, result_received=False)
    assert mod.classify_dispatch_record(timeout) == 'INGRESS_RESULT_TIMEOUT_DIAGNOSTIC_FAIL'

    succeeded = dict(base, dispatch_attempted=True, accepted=True, rejected=False, ingress_goal_sent=True, result_received=True, result_status_label='SUCCEEDED')
    assert mod.classify_dispatch_record(succeeded) == 'INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER'

    aborted = dict(base, dispatch_attempted=True, accepted=True, rejected=False, ingress_goal_sent=True, result_received=True, result_status_label='ABORTED')
    assert mod.classify_dispatch_record(aborted) == 'INGRESS_RESULT_ABORTED_DIAGNOSTIC_FAIL'

    canceled = dict(base, dispatch_attempted=True, accepted=True, rejected=False, ingress_goal_sent=True, result_received=True, result_status_label='CANCELED')
    assert mod.classify_dispatch_record(canceled) == 'INGRESS_RESULT_CANCELED_DIAGNOSTIC_FAIL'


def test_phase118_analyzer_validates_required_artifact_schema_and_guardrails(tmp_path: Path):
    runner = _load(RUNNER, 'phase118_runner_artifact')
    analyzer = _load(ANALYZER, 'phase118_analyzer')
    artifact = runner.build_artifact(
        run_id='unit',
        preflight={'evaluated': True, 'passed': True, 'failed_gates': [], 'ingress_preflight_reject_reason': None},
        dispatch=dict(
            runner.initial_dispatch_record(bounded_goal_result_wait_sec=1.0),
            dispatch_attempted=True,
            action_server_ready=True,
            accepted=True,
            rejected=False,
            ingress_goal_sent=True,
            result_received=True,
            result_status=4,
            result_status_label='SUCCEEDED',
        ),
        classification='INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER',
    )
    path = tmp_path / 'artifact.json'
    path.write_text(json.dumps(artifact), encoding='utf-8')
    analysis = analyzer.analyze_artifact(artifact)
    assert analysis['classification'] == 'INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER'
    assert analysis['schema']['required_dispatch_fields_present'] is True
    assert analysis['goal_identity']['locked_explicit_inner_ingress_goal'] is True
    assert analysis['guardrails']['maze_explorer_started'] is False
    assert analysis['guardrails']['no_autonomous_success_claim'] is True
    assert analysis['guardrails']['no_exit_success_claim'] is True


def test_phase118_source_guardrails_exclude_forbidden_goal_and_maze_explorer_start():
    text = _text(RUNNER)
    required = [
        'single_goal_dispatch_smoke_only',
        'PREFLIGHT_FAILED_NO_DISPATCH',
        'INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER',
        'timeout/abort/cancel are diagnostic fail',
        'ingress_goal_sent=true only after accepted',
        'frame_id',
        'x',
        'y',
        'yaw',
    ]
    for phrase in required:
        assert phrase in text
    forbidden = [
        'maze_explorer_started=True',
        'max_goals=1',
        'Goal1',
        'carry_over',
        'staging goal',
        'exit goal',
    ]
    for phrase in forbidden:
        assert phrase not in text


def test_phase118_report_records_runtime_result_and_no_success_overclaim():
    text = _text(REPORT)
    required = [
        'PHASE118_CONTROLLED_INGRESS_SINGLE_GOAL_DISPATCH_SMOKE_COMPLETE_STOP_BEFORE_PHASE119',
        'preflight artifact',
        'goal_pose',
        'action_server_ready',
        'accepted',
        'result_status_label',
        'bounded_goal_result_wait_sec',
        'ingress_goal_sent',
        'maze_explorer_started=false',
        'No maze_explorer was started',
        'No Goal1/carry-over/staging/branch/centerline/fallback/terminal/exit goal was sent',
        'No autonomous exploration success or exit success is claimed',
        'Phase119 not entered',
    ]
    for phrase in required:
        assert phrase in text
