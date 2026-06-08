#!/usr/bin/env python3
"""Analyze Phase118 controlled ingress single-goal dispatch smoke artifacts."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

PHASE = 'Phase118'
MODE = 'single_goal_dispatch_smoke_only'
LOCKED_GOAL = {'frame_id': 'map', 'x': 2.0, 'y': 0.0, 'yaw': 0.0}

PREFLIGHT_FAILED_NO_DISPATCH = 'PREFLIGHT_FAILED_NO_DISPATCH'
READY_WAIT_TIMEOUT_NO_DISPATCH = 'READY_WAIT_TIMEOUT_NO_DISPATCH'
INGRESS_DISPATCH_REJECTED_DIAGNOSTIC_FAIL = 'INGRESS_DISPATCH_REJECTED_DIAGNOSTIC_FAIL'
INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER = 'INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER'
INGRESS_RESULT_ABORTED_DIAGNOSTIC_FAIL = 'INGRESS_RESULT_ABORTED_DIAGNOSTIC_FAIL'
INGRESS_RESULT_CANCELED_DIAGNOSTIC_FAIL = 'INGRESS_RESULT_CANCELED_DIAGNOSTIC_FAIL'
INGRESS_RESULT_TIMEOUT_DIAGNOSTIC_FAIL = 'INGRESS_RESULT_TIMEOUT_DIAGNOSTIC_FAIL'
INGRESS_DISPATCH_INSUFFICIENT_EVIDENCE = 'INGRESS_DISPATCH_INSUFFICIENT_EVIDENCE'

REQUIRED_DISPATCH_FIELDS = [
    'dispatch_attempted',
    'preflight_pass_required',
    'preflight_failed_gates_required_empty',
    'preflight_reject_reason_required_absent',
    'goal_pose',
    'frame_id',
    'stamp',
    'x',
    'y',
    'yaw',
    'action_name',
    'action_server_ready',
    'action_server_ready_checked_at_wall_time',
    'send_time',
    'send_wall_time_sec',
    'send_ros_time_sec',
    'goal_response_wall_time_sec',
    'accepted',
    'rejected',
    'goal_handle_available',
    'result_wait_started_wall_time_sec',
    'bounded_goal_result_wait_sec',
    'result_received',
    'result_status',
    'result_status_label',
    'abort_text',
    'cancel_requested',
    'cancel_result',
    'exception_text',
    'traceback_tail',
    'ingress_goal_sent',
    'maze_explorer_started',
]
REQUIRED_READINESS_WAIT_FIELDS = [
    'enabled',
    'readiness_wait_start_wall_time_sec',
    'readiness_wait_end_wall_time_sec',
    'readiness_wait_elapsed_sec',
    'readiness_wait_timeout_sec',
    'marker_found',
    'marker_found_wall_time_sec',
    'multi_source_ready',
    'timed_out',
    'lifecycle_states',
    'action_server_ready',
    'samples',
]


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y'}
    return bool(value)


def _goal_identity(dispatch: dict[str, Any]) -> dict[str, Any]:
    goal = _dict(dispatch.get('goal_pose'))
    locked = (
        goal.get('frame_id') == LOCKED_GOAL['frame_id']
        and float(goal.get('x', 9999)) == LOCKED_GOAL['x']
        and float(goal.get('y', 9999)) == LOCKED_GOAL['y']
        and float(goal.get('yaw', 9999)) == LOCKED_GOAL['yaw']
        and dispatch.get('frame_id') == LOCKED_GOAL['frame_id']
        and float(dispatch.get('x', 9999)) == LOCKED_GOAL['x']
        and float(dispatch.get('y', 9999)) == LOCKED_GOAL['y']
        and float(dispatch.get('yaw', 9999)) == LOCKED_GOAL['yaw']
    )
    return {
        'locked_explicit_inner_ingress_goal': bool(locked),
        'goal_pose': goal,
        'expected': dict(LOCKED_GOAL),
    }


def _schema(dispatch: dict[str, Any]) -> dict[str, Any]:
    missing = [field for field in REQUIRED_DISPATCH_FIELDS if field not in dispatch]
    return {
        'required_dispatch_fields_present': not missing,
        'missing_dispatch_fields': missing,
        'required_dispatch_fields': list(REQUIRED_DISPATCH_FIELDS),
    }


def _classify_from_dispatch(preflight: dict[str, Any], dispatch: dict[str, Any], readiness_wait: dict[str, Any] | None = None) -> str:
    readiness_wait = _dict(readiness_wait)
    if _bool(readiness_wait.get('enabled')) and _bool(readiness_wait.get('timed_out')):
        return READY_WAIT_TIMEOUT_NO_DISPATCH
    if not _bool(dispatch.get('dispatch_attempted')):
        return PREFLIGHT_FAILED_NO_DISPATCH
    if _bool(dispatch.get('rejected')):
        return INGRESS_DISPATCH_REJECTED_DIAGNOSTIC_FAIL
    if not _bool(dispatch.get('accepted')):
        return INGRESS_DISPATCH_INSUFFICIENT_EVIDENCE
    if not _bool(dispatch.get('result_received')):
        return INGRESS_RESULT_TIMEOUT_DIAGNOSTIC_FAIL
    label = dispatch.get('result_status_label')
    if label == 'SUCCEEDED':
        return INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER
    if label == 'ABORTED':
        return INGRESS_RESULT_ABORTED_DIAGNOSTIC_FAIL
    if label == 'CANCELED':
        return INGRESS_RESULT_CANCELED_DIAGNOSTIC_FAIL
    return INGRESS_DISPATCH_INSUFFICIENT_EVIDENCE


def _readiness_wait_schema(readiness_wait: dict[str, Any]) -> dict[str, Any]:
    missing = [field for field in REQUIRED_READINESS_WAIT_FIELDS if field not in readiness_wait]
    return {'schema_valid': not missing, 'missing_fields': missing, 'required_fields': list(REQUIRED_READINESS_WAIT_FIELDS)}


def analyze_artifact(artifact: dict[str, Any]) -> dict[str, Any]:
    preflight = _dict(artifact.get('preflight'))
    dispatch = _dict(artifact.get('dispatch'))
    readiness_wait = _dict(artifact.get('readiness_wait'))
    guardrails = _dict(artifact.get('guardrails'))
    classification = artifact.get('classification') or _classify_from_dispatch(preflight, dispatch, readiness_wait)
    expected = _classify_from_dispatch(preflight, dispatch, readiness_wait)
    schema = _schema(dispatch)
    readiness_schema = _readiness_wait_schema(readiness_wait) if readiness_wait else {'schema_valid': True, 'missing_fields': [], 'required_fields': list(REQUIRED_READINESS_WAIT_FIELDS)}
    goal_identity = _goal_identity(dispatch)
    accepted = _bool(dispatch.get('accepted'))
    ingress_goal_sent = _bool(dispatch.get('ingress_goal_sent'))
    state_transition_valid = (ingress_goal_sent == accepted) if _bool(dispatch.get('dispatch_attempted')) else not ingress_goal_sent
    result_label = dispatch.get('result_status_label')
    diagnostic_fail = classification in {
        PREFLIGHT_FAILED_NO_DISPATCH,
        READY_WAIT_TIMEOUT_NO_DISPATCH,
        INGRESS_DISPATCH_REJECTED_DIAGNOSTIC_FAIL,
        INGRESS_RESULT_ABORTED_DIAGNOSTIC_FAIL,
        INGRESS_RESULT_CANCELED_DIAGNOSTIC_FAIL,
        INGRESS_RESULT_TIMEOUT_DIAGNOSTIC_FAIL,
        INGRESS_DISPATCH_INSUFFICIENT_EVIDENCE,
    }
    return {
        'phase': artifact.get('phase') or PHASE,
        'mode': artifact.get('mode') or MODE,
        'classification': classification,
        'expected_classification_from_dispatch': expected,
        'classification_matches_dispatch': classification == expected,
        'preflight': {
            'passed': _bool(preflight.get('passed')),
            'failed_gates': _list(preflight.get('failed_gates')),
            'reject_reason': preflight.get('ingress_preflight_reject_reason'),
            'evaluated': _bool(preflight.get('evaluated')),
        },
        'dispatch': {
            'dispatch_attempted': _bool(dispatch.get('dispatch_attempted')),
            'action_server_ready': _bool(dispatch.get('action_server_ready')),
            'accepted': accepted,
            'rejected': _bool(dispatch.get('rejected')),
            'result_received': _bool(dispatch.get('result_received')),
            'result_status': dispatch.get('result_status'),
            'result_status_label': result_label,
            'abort_text': dispatch.get('abort_text'),
            'bounded_goal_result_wait_sec': dispatch.get('bounded_goal_result_wait_sec'),
            'cancel_requested': _bool(dispatch.get('cancel_requested')),
            'cancel_result': dispatch.get('cancel_result'),
            'ingress_goal_sent': ingress_goal_sent,
            'maze_explorer_started': _bool(dispatch.get('maze_explorer_started')),
            'state_transition_valid': state_transition_valid,
        },
        'readiness_wait': {
            'enabled': _bool(readiness_wait.get('enabled')),
            'marker_found': _bool(readiness_wait.get('marker_found')),
            'multi_source_ready': _bool(readiness_wait.get('multi_source_ready')),
            'timed_out': _bool(readiness_wait.get('timed_out')),
            'action_server_ready': _bool(readiness_wait.get('action_server_ready')),
            'readiness_wait_start_wall_time_sec': readiness_wait.get('readiness_wait_start_wall_time_sec'),
            'readiness_wait_end_wall_time_sec': readiness_wait.get('readiness_wait_end_wall_time_sec'),
            'readiness_wait_elapsed_sec': readiness_wait.get('readiness_wait_elapsed_sec'),
            'readiness_wait_timeout_sec': readiness_wait.get('readiness_wait_timeout_sec'),
            'lifecycle_states': _dict(readiness_wait.get('lifecycle_states')),
            'schema_valid': readiness_schema['schema_valid'],
            'missing_fields': readiness_schema['missing_fields'],
        },
        'schema': schema,
        'goal_identity': goal_identity,
        'guardrails': {
            'ingress_goal_sent': ingress_goal_sent,
            'maze_explorer_started': _bool(dispatch.get('maze_explorer_started')) or _bool(guardrails.get('maze_explorer_started')),
            'no_maze_explorer_auto_start_guard_valid': not (_bool(dispatch.get('maze_explorer_started')) or _bool(guardrails.get('maze_explorer_started'))),
            'nav2_config_tuned': _bool(guardrails.get('nav2_config_tuned')),
            'exploration_strategy_changed': _bool(guardrails.get('exploration_strategy_changed')),
            'preflight_removed': _bool(guardrails.get('preflight_removed')),
            'no_autonomous_success_claim': not _bool(guardrails.get('autonomous_success_claimed')),
            'no_exit_success_claim': not _bool(guardrails.get('exit_success_claimed')),
            'diagnostic_fail_if_not_succeeded': diagnostic_fail or classification == INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER,
        },
        'valid': bool(
            schema['required_dispatch_fields_present']
            and readiness_schema['schema_valid']
            and goal_identity['locked_explicit_inner_ingress_goal']
            and state_transition_valid
            and not (_bool(dispatch.get('maze_explorer_started')) or _bool(guardrails.get('maze_explorer_started')))
            and classification == expected
        ),
        'notes': [
            'timeout/abort/cancel/rejected are diagnostic fail',
            'succeeded means single ingress goal only; no autonomous exploration or exit success claimed',
        ],
    }


def write_summary(analysis: dict[str, Any], path: Path) -> None:
    title = '# Phase120 controlled ingress dispatch with managed-active readiness wait summary' if analysis['phase'] == 'Phase120' else '# Phase118 controlled ingress single-goal dispatch smoke summary'
    lines = [
        title,
        '',
        f"classification: {analysis['classification']}",
        f"valid: {analysis['valid']}",
        f"preflight_passed: {analysis['preflight']['passed']}",
        f"preflight_failed_gates: {analysis['preflight']['failed_gates']}",
        f"preflight_reject_reason: {analysis['preflight']['reject_reason']}",
        f"dispatch_attempted: {analysis['dispatch']['dispatch_attempted']}",
        f"action_server_ready: {analysis['dispatch']['action_server_ready']}",
        f"accepted: {analysis['dispatch']['accepted']}",
        f"rejected: {analysis['dispatch']['rejected']}",
        f"result_status_label: {analysis['dispatch']['result_status_label']}",
        f"abort_text: {analysis['dispatch']['abort_text']}",
        f"bounded_goal_result_wait_sec: {analysis['dispatch']['bounded_goal_result_wait_sec']}",
        f"cancel_requested: {analysis['dispatch']['cancel_requested']}",
        f"cancel_result: {analysis['dispatch']['cancel_result']}",
        f"ingress_goal_sent: {analysis['dispatch']['ingress_goal_sent']}",
        f"maze_explorer_started: {analysis['dispatch']['maze_explorer_started']}",
        '',
        '## Guardrails',
        f"- no_maze_explorer_auto_start_guard_valid: {analysis['guardrails']['no_maze_explorer_auto_start_guard_valid']}",
        f"- no_autonomous_success_claim: {analysis['guardrails']['no_autonomous_success_claim']}",
        f"- no_exit_success_claim: {analysis['guardrails']['no_exit_success_claim']}",
        '- No Goal1/carry-over/staging/branch/centerline/fallback/terminal/exit goal was sent.',
        '- No maze_explorer was started.',
        '- No Nav2/MPPI/controller/config tuning was performed.',
        f"- {'Phase121' if analysis['phase'] == 'Phase120' else 'Phase119'} not entered.",
        '',
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines), encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact', type=Path, required=True)
    parser.add_argument('--output-json', type=Path, required=True)
    parser.add_argument('--minimal-summary-output', type=Path, required=True)
    args = parser.parse_args()
    artifact = json.loads(args.artifact.read_text(encoding='utf-8'))
    analysis = analyze_artifact(artifact)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(analysis, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    write_summary(analysis, args.minimal_summary_output)
    print(json.dumps({
        'classification': analysis['classification'],
        'valid': analysis['valid'],
        'ingress_goal_sent': analysis['dispatch']['ingress_goal_sent'],
        'maze_explorer_started': analysis['dispatch']['maze_explorer_started'],
        'analysis': str(args.output_json),
        'summary': str(args.minimal_summary_output),
    }, sort_keys=True))
    return 0 if analysis['valid'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
