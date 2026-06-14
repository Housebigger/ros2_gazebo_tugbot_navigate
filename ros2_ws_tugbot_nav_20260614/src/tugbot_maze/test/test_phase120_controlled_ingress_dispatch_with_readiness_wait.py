from __future__ import annotations

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / 'tools' / 'run_phase118_controlled_ingress_single_goal_dispatch_smoke.py'
ANALYZER = ROOT / 'tools' / 'analyze_phase118_controlled_ingress_single_goal_dispatch_smoke.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase120_controlled_ingress_dispatch_with_readiness_wait_report.md'


def _load(path: Path, name: str):
    assert path.exists(), f'missing required file: {path}'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _text(path: Path) -> str:
    assert path.exists(), f'missing required Phase120 report: {path}'
    return path.read_text(encoding='utf-8')


def test_phase120_runner_readiness_wait_timeout_blocks_preflight_and_dispatch(tmp_path: Path):
    mod = _load(RUNNER, 'phase120_runner_readiness')
    wait = mod.initial_readiness_wait_record(timeout_sec=3.0)
    wait['enabled'] = True
    wait['timed_out'] = True
    wait['marker_found'] = False
    wait['multi_source_ready'] = False

    dispatch = mod.initial_dispatch_record(bounded_goal_result_wait_sec=5.0)
    preflight = {'evaluated': False, 'passed': False, 'failed_gates': [], 'ingress_preflight_reject_reason': None}
    artifact = mod.build_artifact(
        run_id='unit_phase120_timeout',
        preflight=preflight,
        dispatch=dispatch,
        classification=mod.READY_WAIT_TIMEOUT_NO_DISPATCH,
        phase='Phase120',
        mode='controlled_ingress_dispatch_with_managed_active_readiness_wait',
        readiness_wait=wait,
    )

    assert artifact['classification'] == 'READY_WAIT_TIMEOUT_NO_DISPATCH'
    assert artifact['readiness_wait']['readiness_wait_start_wall_time_sec'] is None
    assert artifact['readiness_wait']['readiness_wait_timeout_sec'] == 3.0
    assert artifact['readiness_wait']['marker_found'] is False
    assert artifact['dispatch']['dispatch_attempted'] is False
    assert artifact['dispatch']['ingress_goal_sent'] is False
    assert artifact['guardrails']['maze_explorer_started'] is False


def test_phase120_runner_marker_or_multisource_readiness_schema_and_precondition():
    mod = _load(RUNNER, 'phase120_runner_schema')
    marker_wait = mod.initial_readiness_wait_record(timeout_sec=10.0)
    marker_wait.update({'enabled': True, 'marker_found': True, 'timed_out': False})
    assert mod.readiness_wait_passed(marker_wait) is True

    multisource_wait = mod.initial_readiness_wait_record(timeout_sec=10.0)
    multisource_wait.update({
        'enabled': True,
        'marker_found': False,
        'multi_source_ready': True,
        'action_server_ready': True,
        'lifecycle_states': {
            'controller_server': 'active [3]',
            'bt_navigator': 'active [3]',
            'planner_server': 'active [3]',
            'behavior_server': 'active [3]',
        },
        'timed_out': False,
    })
    assert mod.readiness_wait_passed(multisource_wait) is True

    failed_wait = mod.initial_readiness_wait_record(timeout_sec=10.0)
    failed_wait.update({'enabled': True, 'marker_found': False, 'multi_source_ready': False, 'timed_out': True})
    assert mod.readiness_wait_passed(failed_wait) is False

    decision = mod.evaluate_readiness_wait_decision(failed_wait)
    assert decision['allowed'] is False
    assert decision['classification'] == 'READY_WAIT_TIMEOUT_NO_DISPATCH'


def test_phase120_analyzer_validates_readiness_wait_and_ready_timeout_classification():
    runner = _load(RUNNER, 'phase120_runner_artifact')
    analyzer = _load(ANALYZER, 'phase120_analyzer')
    wait = runner.initial_readiness_wait_record(timeout_sec=7.0)
    wait.update({'enabled': True, 'timed_out': True, 'marker_found': False, 'multi_source_ready': False})
    artifact = runner.build_artifact(
        run_id='unit_phase120_analyzer',
        preflight={'evaluated': False, 'passed': False, 'failed_gates': [], 'ingress_preflight_reject_reason': None},
        dispatch=runner.initial_dispatch_record(bounded_goal_result_wait_sec=1.0),
        classification='READY_WAIT_TIMEOUT_NO_DISPATCH',
        phase='Phase120',
        mode='controlled_ingress_dispatch_with_managed_active_readiness_wait',
        readiness_wait=wait,
    )
    analysis = analyzer.analyze_artifact(artifact)
    assert analysis['phase'] == 'Phase120'
    assert analysis['mode'] == 'controlled_ingress_dispatch_with_managed_active_readiness_wait'
    assert analysis['classification'] == 'READY_WAIT_TIMEOUT_NO_DISPATCH'
    assert analysis['readiness_wait']['enabled'] is True
    assert analysis['readiness_wait']['timed_out'] is True
    assert analysis['readiness_wait']['schema_valid'] is True
    assert analysis['dispatch']['dispatch_attempted'] is False
    assert analysis['dispatch']['ingress_goal_sent'] is False
    assert analysis['valid'] is True


def test_phase120_source_guardrails_keep_single_goal_only_and_no_maze_explorer():
    text = RUNNER.read_text(encoding='utf-8')
    required = [
        'READY_WAIT_TIMEOUT_NO_DISPATCH',
        'controlled_ingress_dispatch_with_managed_active_readiness_wait',
        'Managed nodes are active',
        'readiness_wait_start_wall_time_sec',
        'readiness_wait_end_wall_time_sec',
        'marker_found',
        'multi_source_ready',
        'ingress_goal_sent=true only after accepted',
    ]
    for phrase in required:
        assert phrase in text
    forbidden = [
        'maze_explorer_started=True',
        'max_goals=1',
        'carry_over',
        'staging goal',
        'exit goal',
    ]
    for phrase in forbidden:
        assert phrase not in text


def test_phase120_report_records_runtime_and_manual_cleanup_context():
    text = _text(REPORT)
    required = [
        'PHASE120_CONTROLLED_INGRESS_DISPATCH_WITH_READINESS_WAIT',
        'READY_WAIT_TIMEOUT_NO_DISPATCH',
        'PREFLIGHT_FAILED_NO_DISPATCH',
        'INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER',
        'readiness_wait_start',
        'readiness_wait_end',
        'marker_found',
        'action_server_ready',
        'ingress_goal_sent',
        'maze_explorer_started=false',
        'No maze_explorer was started',
        'No autonomous exploration success or exit success is claimed',
        'Phase121 not entered',
    ]
    for phrase in required:
        assert phrase in text
