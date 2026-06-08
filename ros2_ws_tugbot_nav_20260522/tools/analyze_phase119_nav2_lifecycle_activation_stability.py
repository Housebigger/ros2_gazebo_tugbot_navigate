#!/usr/bin/env python3
"""Phase119 analyzer: Nav2 lifecycle activation stability diagnosis.

Diagnosis-only: analyze lifecycle/readiness timelines and preflight evidence. This
script never sends NavigateToPose goals and never starts maze_explorer.
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

PHASE = 'Phase119'
MODE = 'nav2_lifecycle_activation_stability_diagnosis_no_goal'

REQUIRED_ACTIVE_NODES = [
    'controller_server',
    'bt_navigator',
    'planner_server',
    'behavior_server',
]
OBSERVED_NODES = [
    'lifecycle_manager_navigation',
    'controller_server',
    'bt_navigator',
    'planner_server',
    'behavior_server',
    'map_server',
    'slam_toolbox',
]


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y'}
    return bool(value)


def _float(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _state(sample: dict[str, Any], node: str) -> str | None:
    nodes = _dict(sample.get('nodes'))
    data = _dict(nodes.get(node))
    state = data.get('state')
    return str(state).strip().lower() if state is not None else None


def _is_active_state(state: str | None) -> bool:
    return state in {'active', 'active [3]', 'active_confirmed'}


def _is_definitely_not_active_state(state: str | None) -> bool:
    return state in {'inactive', 'inactive [2]', 'unconfigured', 'unconfigured [1]', 'finalized', 'query_failed'}


def _is_query_timeout_state(state: str | None) -> bool:
    return state == 'query_timeout'


def _sample_all_required_active(sample: dict[str, Any]) -> bool:
    return all(_is_active_state(_state(sample, node)) for node in REQUIRED_ACTIVE_NODES)


def _first_sample(samples: list[dict[str, Any]], predicate) -> dict[str, Any] | None:
    for sample in samples:
        if predicate(sample):
            return sample
    return None


def _last_sample(samples: list[dict[str, Any]]) -> dict[str, Any]:
    return samples[-1] if samples else {}


def parse_launch_log_events(log_text: str, *, launch_start_wall_time_sec: float | None = None) -> list[dict[str, Any]]:
    """Extract lifecycle transition/marker/failure events from a launch log."""
    events: list[dict[str, Any]] = []
    interesting = [
        'Managed nodes are active',
        'Activating ',
        'Configuring ',
        'Deactivating ',
        'Cleaning up',
        'Shutting down',
        'Failed to bring up',
        'failed to bring up',
        'bond timeout',
        'Bond timer timeout',
        'restarting lifecycle',
        'transition',
    ]
    ts_re = re.compile(r'\[(\d+(?:\.\d+)?)\]')
    for line_no, line in enumerate(log_text.splitlines(), start=1):
        if not any(token in line for token in interesting):
            continue
        # Prefer the final [number] field as ROS log timestamp when present.
        matches = ts_re.findall(line)
        ros_time = _float(matches[-1]) if matches else None
        event = None
        if 'bond timeout' in line and 'restarting lifecycle' in line:
            event = 'bond timeout, restarting lifecycle'
        else:
            for token in interesting:
                if token in line:
                    if token.endswith(' '):
                        idx = line.find(token)
                        tail = line[idx:].split(':')[-1].strip() if ':' in line[idx:] else line[idx:].strip()
                        event = tail or token.strip()
                    else:
                        event = token
                    break
        if event is None:
            event = line.strip()
        elapsed = None
        if launch_start_wall_time_sec is not None and ros_time is not None:
            # In sim logs ROS time is not wall time; keep this as diagnostic only.
            elapsed = ros_time - launch_start_wall_time_sec
        events.append({
            'line_no': line_no,
            'ros_time_sec': ros_time,
            'elapsed_sec': elapsed,
            'event': event,
            'line': line.strip(),
        })
    return events


def _marker_timeline(artifact: dict[str, Any], samples: list[dict[str, Any]]) -> dict[str, Any]:
    events = _list(artifact.get('launch_log_events'))
    marker_events = [event for event in events if 'Managed nodes are active' in str(_dict(event).get('event') or _dict(event).get('line') or '')]
    sample_markers = [sample for sample in samples if _bool(sample.get('launch_active_marker_present'))]
    first_marker_elapsed = None
    if marker_events:
        first_marker_elapsed = _float(_dict(marker_events[0]).get('elapsed_sec'))
        if first_marker_elapsed is None:
            first_marker_elapsed = _float(_dict(marker_events[0]).get('ros_time_sec'))
    elif sample_markers:
        first_marker_elapsed = _float(sample_markers[0].get('elapsed_sec'))
    return {
        'managed_nodes_active_seen': bool(marker_events or sample_markers),
        'first_marker_elapsed_sec': first_marker_elapsed,
        'marker_event_count': len(marker_events),
        'sample_marker_count': len(sample_markers),
        'events': marker_events,
    }


def _stable_active_window(samples: list[dict[str, Any]]) -> dict[str, Any]:
    first = _first_sample(samples, _sample_all_required_active)
    last = _last_sample(samples)
    per_node_first: dict[str, float | None] = {}
    per_node_last_state: dict[str, str | None] = {}
    for node in OBSERVED_NODES:
        found = _first_sample(samples, lambda s, node=node: _is_active_state(_state(s, node)))
        per_node_first[node] = _float(found.get('elapsed_sec')) if found else None
        per_node_last_state[node] = _state(last, node)
    return {
        'all_required_active_observed': first is not None,
        'first_all_active_elapsed_sec': _float(first.get('elapsed_sec')) if first else None,
        'first_all_active_wall_time_sec': _float(first.get('wall_time_sec')) if first else None,
        'per_node_first_active_elapsed_sec': per_node_first,
        'per_node_last_state': per_node_last_state,
    }


def _contradiction_windows(samples: list[dict[str, Any]]) -> dict[str, Any]:
    action_present_bt_not_active = []
    action_present_any_required_not_active = []
    action_present_cli_query_timeout = []
    marker_missing_required_not_active = []
    for sample in samples:
        action = _bool(sample.get('action_present'))
        marker = _bool(sample.get('launch_active_marker_present'))
        bt_state = _state(sample, 'bt_navigator')
        bt_active = _is_active_state(bt_state)
        bt_definitely_not_active = _is_definitely_not_active_state(bt_state)
        required_active = _sample_all_required_active(sample)
        required_states = [_state(sample, node) for node in REQUIRED_ACTIVE_NODES]
        any_required_definitely_not_active = any(_is_definitely_not_active_state(state) for state in required_states)
        all_required_query_timeout = all(_is_query_timeout_state(state) for state in required_states)
        if action and bt_definitely_not_active:
            action_present_bt_not_active.append(sample)
        if action and any_required_definitely_not_active:
            action_present_any_required_not_active.append(sample)
        if action and all_required_query_timeout:
            action_present_cli_query_timeout.append(sample)
        if not marker and not required_active:
            marker_missing_required_not_active.append(sample)
    def compact(sample: dict[str, Any]) -> dict[str, Any]:
        return {
            'elapsed_sec': sample.get('elapsed_sec'),
            'wall_time_sec': sample.get('wall_time_sec'),
            'action_present': _bool(sample.get('action_present')),
            'launch_active_marker_present': _bool(sample.get('launch_active_marker_present')),
            'bt_navigator_state': _state(sample, 'bt_navigator'),
            'controller_server_state': _state(sample, 'controller_server'),
            'planner_server_state': _state(sample, 'planner_server'),
            'behavior_server_state': _state(sample, 'behavior_server'),
        }
    return {
        'action_present_bt_not_active_count': len(action_present_bt_not_active),
        'action_present_any_required_not_active_count': len(action_present_any_required_not_active),
        'action_present_cli_query_timeout_count': len(action_present_cli_query_timeout),
        'marker_missing_required_not_active_count': len(marker_missing_required_not_active),
        'action_present_bt_not_active_samples': [compact(s) for s in action_present_bt_not_active],
        'action_present_any_required_not_active_samples': [compact(s) for s in action_present_any_required_not_active],
        'action_present_cli_query_timeout_samples': [compact(s) for s in action_present_cli_query_timeout],
    }


def _preflight_window(artifact: dict[str, Any]) -> dict[str, Any]:
    preflight = _dict(artifact.get('preflight'))
    if 'ingress_preflight' in artifact:
        preflight = _dict(artifact.get('ingress_preflight'))
    return {
        'start_wall_time_sec': _float(preflight.get('start_wall_time_sec') or preflight.get('start_time_sec')),
        'end_wall_time_sec': _float(preflight.get('end_wall_time_sec') or preflight.get('end_time_sec')),
        'passed': _bool(preflight.get('passed')),
        'failed_gates': _list(preflight.get('failed_gates')),
        'reject_reason': preflight.get('reject_reason') if 'reject_reason' in preflight else preflight.get('ingress_preflight_reject_reason'),
        'ingress_goal_sent': _bool(preflight.get('ingress_goal_sent')),
        'maze_explorer_started': _bool(preflight.get('maze_explorer_started')),
    }


def _classify(*, preflight: dict[str, Any], stable: dict[str, Any], marker: dict[str, Any], contradictions: dict[str, Any], samples: list[dict[str, Any]]) -> tuple[str, list[str]]:
    findings: list[str] = []
    if contradictions['action_present_bt_not_active_count'] > 0:
        findings.append('ACTION_PRESENT_WHILE_BT_NOT_ACTIVE')
    if contradictions['action_present_any_required_not_active_count'] > 0:
        findings.append('ACTION_PRESENT_WHILE_REQUIRED_NAV2_NODE_NOT_ACTIVE')
    if contradictions.get('action_present_cli_query_timeout_count', 0) > 0:
        findings.append('LIFECYCLE_CLI_QUERY_TIMEOUT_WITH_ACTION_PRESENT')
    if not marker['managed_nodes_active_seen']:
        findings.append('LAUNCH_ACTIVE_MARKER_MISSING_BEFORE_PREFLIGHT')
    if stable['all_required_active_observed']:
        first_wall = _float(stable.get('first_all_active_wall_time_sec'))
        pre_start = _float(preflight.get('start_wall_time_sec'))
        if pre_start is not None and first_wall is not None and first_wall > pre_start:
            findings.append('LATE_MANAGED_NODES_ACTIVE_AFTER_PREFLIGHT_START')
    else:
        findings.append('MANAGED_NODES_NEVER_ALL_ACTIVE_IN_CAPTURE')

    if preflight['passed'] and marker['managed_nodes_active_seen'] and contradictions['action_present_bt_not_active_count'] == 0:
        if contradictions.get('action_present_cli_query_timeout_count', 0) > 0:
            findings.append('QUERY_RACE_OR_CLI_TIMEOUT_DESPITE_MULTI_SOURCE_PREFLIGHT_PASS')
            return 'NAV2_LIFECYCLE_ACTIVE_MARKER_PREFLIGHT_PASS_QUERY_TIMEOUTS', findings
        if stable['all_required_active_observed']:
            return 'NAV2_LIFECYCLE_ACTIVATION_STABLE_PREFLIGHT_PASS', findings
    if stable['all_required_active_observed']:
        first_wall = _float(stable.get('first_all_active_wall_time_sec'))
        pre_start = _float(preflight.get('start_wall_time_sec'))
        if pre_start is not None and first_wall is not None and first_wall > pre_start:
            return 'NAV2_ACTIVATION_DELAY_OR_PREFLIGHT_TOO_EARLY', findings
    if contradictions['action_present_bt_not_active_count'] > 0:
        return 'ACTION_PRESENT_WITH_INACTIVE_LIFECYCLE_WINDOW', findings
    if not stable['all_required_active_observed']:
        return 'MANAGED_NODES_NOT_ACTIVE_IN_CAPTURE', findings
    return 'NAV2_LIFECYCLE_STABILITY_INSUFFICIENT_EVIDENCE', findings


def analyze_artifact(artifact: dict[str, Any]) -> dict[str, Any]:
    samples = [dict(sample) for sample in _list(artifact.get('timeline_samples'))]
    preflight = _preflight_window(artifact)
    marker = _marker_timeline(artifact, samples)
    stable = _stable_active_window(samples)
    contradictions = _contradiction_windows(samples)
    classification, findings = _classify(
        preflight=preflight,
        stable=stable,
        marker=marker,
        contradictions=contradictions,
        samples=samples,
    )
    guardrails = _dict(artifact.get('guardrails'))
    no_goal = not (_bool(guardrails.get('ingress_goal_sent')) or _bool(preflight.get('ingress_goal_sent')))
    no_maze = not (_bool(guardrails.get('maze_explorer_started')) or _bool(preflight.get('maze_explorer_started')))
    launch_args = _dict(artifact.get('launch_args'))
    return {
        'phase': PHASE,
        'mode': MODE,
        'classification': classification,
        'findings': findings,
        'launch_args': {
            'autostart': launch_args.get('autostart'),
            'use_sim_time': launch_args.get('use_sim_time'),
            'use_composition': launch_args.get('use_composition'),
            'use_respawn': launch_args.get('use_respawn'),
            'headless': launch_args.get('headless'),
            'use_rviz': launch_args.get('use_rviz'),
        },
        'preflight_window': preflight,
        'marker_timeline': marker,
        'stable_active_window': stable,
        'contradiction_windows': contradictions,
        'timeline_sample_count': len(samples),
        'nodes_observed': OBSERVED_NODES,
        'required_active_nodes': REQUIRED_ACTIVE_NODES,
        'guardrails': {
            'ingress_goal_sent': not no_goal,
            'maze_explorer_started': not no_maze,
            'no_goal_guard_valid': no_goal,
            'no_maze_explorer_guard_valid': no_maze,
            'nav2_config_tuned': _bool(guardrails.get('nav2_config_tuned')),
            'exploration_strategy_changed': _bool(guardrails.get('exploration_strategy_changed')),
            'autonomous_success_claimed': _bool(guardrails.get('autonomous_success_claimed')),
            'exit_success_claimed': _bool(guardrails.get('exit_success_claimed')),
        },
        'valid': bool(no_goal and no_maze and len(samples) > 0),
    }


def write_summary(analysis: dict[str, Any], path: Path) -> None:
    lines = [
        '# Phase119 Nav2 lifecycle activation stability summary',
        '',
        f"classification: {analysis['classification']}",
        f"valid: {analysis['valid']}",
        f"findings: {analysis['findings']}",
        f"timeline_sample_count: {analysis['timeline_sample_count']}",
        '',
        '## Launch args',
        f"autostart: {analysis['launch_args'].get('autostart')}",
        f"use_sim_time: {analysis['launch_args'].get('use_sim_time')}",
        f"use_composition: {analysis['launch_args'].get('use_composition')}",
        f"use_respawn: {analysis['launch_args'].get('use_respawn')}",
        '',
        '## Preflight window',
        f"passed: {analysis['preflight_window']['passed']}",
        f"failed_gates: {analysis['preflight_window']['failed_gates']}",
        f"reject_reason: {analysis['preflight_window']['reject_reason']}",
        '',
        '## Marker/stability',
        f"managed_nodes_active_seen: {analysis['marker_timeline']['managed_nodes_active_seen']}",
        f"first_marker_elapsed_sec: {analysis['marker_timeline']['first_marker_elapsed_sec']}",
        f"all_required_active_observed: {analysis['stable_active_window']['all_required_active_observed']}",
        f"first_all_active_elapsed_sec: {analysis['stable_active_window']['first_all_active_elapsed_sec']}",
        f"per_node_last_state: {analysis['stable_active_window']['per_node_last_state']}",
        '',
        '## Contradictions',
        f"action_present_bt_not_active_count: {analysis['contradiction_windows']['action_present_bt_not_active_count']}",
        f"action_present_any_required_not_active_count: {analysis['contradiction_windows']['action_present_any_required_not_active_count']}",
        f"action_present_cli_query_timeout_count: {analysis['contradiction_windows'].get('action_present_cli_query_timeout_count')}",
        '',
        '## Guardrails',
        f"ingress_goal_sent: {analysis['guardrails']['ingress_goal_sent']}",
        f"maze_explorer_started: {analysis['guardrails']['maze_explorer_started']}",
        f"no_goal_guard_valid: {analysis['guardrails']['no_goal_guard_valid']}",
        f"no_maze_explorer_guard_valid: {analysis['guardrails']['no_maze_explorer_guard_valid']}",
        '- No NavigateToPose goal was sent.',
        '- No maze_explorer was started.',
        '- No autonomous exploration success or exit success is claimed.',
        '- Phase120 not entered.',
        '',
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines), encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact', type=Path, required=True)
    parser.add_argument('--output-json', type=Path, required=True)
    parser.add_argument('--minimal-summary-output', type=Path, required=True)
    parser.add_argument('--launch-log', type=Path, help='optional launch log to parse when artifact lacks launch_log_events')
    args = parser.parse_args()
    artifact = json.loads(args.artifact.read_text(encoding='utf-8'))
    if args.launch_log and args.launch_log.exists() and not artifact.get('launch_log_events'):
        artifact['launch_log_events'] = parse_launch_log_events(args.launch_log.read_text(encoding='utf-8'))
    analysis = analyze_artifact(artifact)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(analysis, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    write_summary(analysis, args.minimal_summary_output)
    print(json.dumps({
        'classification': analysis['classification'],
        'valid': analysis['valid'],
        'no_goal_guard_valid': analysis['guardrails']['no_goal_guard_valid'],
        'no_maze_explorer_guard_valid': analysis['guardrails']['no_maze_explorer_guard_valid'],
        'analysis': str(args.output_json),
        'summary': str(args.minimal_summary_output),
    }, sort_keys=True))
    return 0 if analysis['valid'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
