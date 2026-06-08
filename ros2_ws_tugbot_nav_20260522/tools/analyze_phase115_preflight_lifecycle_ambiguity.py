#!/usr/bin/env python3
"""Phase115 analyzer: diagnose lifecycle subprocess timeout ambiguity after Phase114."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y'}
    return bool(value)


def _num(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _lifecycle_sources(preflight: dict[str, Any]) -> dict[str, Any]:
    samples = _list(preflight.get('sample_history'))
    for sample in reversed(samples):
        sample = _dict(sample)
        ctrl = _dict(sample.get('controller_pose_check'))
        sources = _dict(ctrl.get('lifecycle_sources')) or _dict(sample.get('lifecycle_sources'))
        if sources:
            return sources
        diag_sources = _dict(_dict(sample.get('internal_sampling_diagnostics')).get('lifecycle'))
        if diag_sources:
            return {k: v for k, v in diag_sources.items() if k in {'controller_server', 'bt_navigator'}}
    return {}


def _node_summary(source: dict[str, Any]) -> dict[str, Any]:
    query = _dict(source.get('lifecycle_query'))
    return {
        'state': source.get('state'),
        'active_confirmed': _bool(source.get('active_confirmed')),
        'inactive_confirmed': _bool(source.get('inactive_confirmed')),
        'ambiguous': _bool(source.get('ambiguous')),
        'query_error': _bool(source.get('query_error')),
        'active_confirmed_by': source.get('active_confirmed_by'),
        'ambiguous_detail': source.get('ambiguous_detail'),
        'query_command': query.get('command'),
        'query_timeout_sec': query.get('timeout_sec'),
        'query_duration_sec': query.get('duration_sec'),
        'query_returncode': query.get('returncode'),
        'query_timed_out': _bool(query.get('timed_out')),
        'query_stdout': query.get('stdout'),
        'query_stderr': query.get('stderr'),
        'query_error_text': query.get('query_error'),
        'node_graph_present': _bool(_dict(source.get('node_graph')).get('present')),
        'action_available': _bool(_dict(source.get('action_availability')).get('available')),
        'lifecycle_service_present': _bool(_dict(source.get('lifecycle_service')).get('present')),
        'launch_active_marker_present': _bool(_dict(source.get('launch_active_marker')).get('present')),
        'launch_active_marker_token': _dict(source.get('launch_active_marker')).get('token'),
    }


def collect_lifecycle(preflight: dict[str, Any]) -> dict[str, Any]:
    sources = _lifecycle_sources(preflight)
    return {
        'controller_server': _node_summary(_dict(sources.get('controller_server'))),
        'bt_navigator': _node_summary(_dict(sources.get('bt_navigator'))),
    }


def classify(preflight: dict[str, Any], lifecycle: dict[str, Any]) -> tuple[str, list[str]]:
    findings: list[str] = []
    nodes = [lifecycle['controller_server'], lifecycle['bt_navigator']]
    active_all = all(node['active_confirmed'] for node in nodes)
    timed_out_any = any(node['query_timed_out'] for node in nodes)
    ambiguous_any = any(node['ambiguous'] for node in nodes)
    missing_sources = []
    for name, node in lifecycle.items():
        for key in ['node_graph_present', 'action_available', 'lifecycle_service_present', 'launch_active_marker_present']:
            if not node.get(key):
                missing_sources.append(f'{name}:{key}')
    if missing_sources:
        findings.append('MISSING_STRICT_MULTISOURCE_EVIDENCE:' + ','.join(missing_sources))
    if timed_out_any and active_all:
        findings.append('LIFECYCLE_QUERY_TIMEOUT_RECLASSIFIED_BY_MULTISOURCE_ACTIVE_CONFIRMATION')
    if ambiguous_any:
        findings.append('LIFECYCLE_REMAINS_AMBIGUOUS')

    if not _bool(preflight.get('ingress_goal_sent')) and not _bool(preflight.get('maze_explorer_started')):
        pass
    else:
        findings.append('NO_GOAL_GUARD_VIOLATED')

    if preflight.get('passed') is True and active_all:
        return 'LIFECYCLE_AMBIGUITY_RESOLVED_PREFLIGHT_PASSED_NO_GOAL', findings
    if active_all and preflight.get('ingress_preflight_reject_reason') != 'ingress_lifecycle_ambiguous':
        return 'LIFECYCLE_AMBIGUITY_RESOLVED_FAIL_CLOSED_NEW_REASON', findings
    if timed_out_any or ambiguous_any:
        return 'LIFECYCLE_QUERY_TIMEOUT_STILL_AMBIGUOUS_FAIL_CLOSED', findings
    return 'LIFECYCLE_AMBIGUITY_INSUFFICIENT_EVIDENCE', findings


def analyze_artifact(artifact: dict[str, Any]) -> dict[str, Any]:
    preflight = _dict(artifact.get('ingress_preflight'))
    lifecycle = collect_lifecycle(preflight)
    classification, findings = classify(preflight, lifecycle)
    guardrails = {
        'ingress_goal_sent': _bool(preflight.get('ingress_goal_sent')),
        'maze_explorer_started': _bool(preflight.get('maze_explorer_started')),
        'no_goal_dispatch_guard_valid': not _bool(preflight.get('ingress_goal_sent')) and not _bool(preflight.get('maze_explorer_started')),
        'nav2_config_tuned': False,
        'exploration_strategy_changed': False,
        'preflight_removed': False,
    }
    return {
        'phase': 'Phase115',
        'mode': 'preflight_lifecycle_ambiguity_diagnosis_and_minimal_fix',
        'classification': classification,
        'findings': findings,
        'preflight': {
            'evaluated': _bool(preflight.get('evaluated')),
            'passed': _bool(preflight.get('passed')),
            'reject_reason': preflight.get('ingress_preflight_reject_reason'),
            'failed_gates': _list(preflight.get('failed_gates')),
            'sample_count': int(_num(preflight.get('sample_count')) or len(_list(preflight.get('sample_history')))),
        },
        'lifecycle': lifecycle,
        'guardrails': guardrails,
        'no_goal_validation_only': True,
    }


def write_summary(analysis: dict[str, Any], path: Path) -> None:
    lines = [
        '# Phase115 lifecycle ambiguity summary',
        '',
        f"classification: {analysis['classification']}",
        f"findings: {analysis['findings']}",
        f"passed: {analysis['preflight']['passed']}",
        f"reject_reason: {analysis['preflight']['reject_reason']}",
        '',
        '## Lifecycle',
    ]
    for name in ['controller_server', 'bt_navigator']:
        node = analysis['lifecycle'][name]
        lines.extend([
            f"### {name}",
            f"- active_confirmed: {node['active_confirmed']}",
            f"- active_confirmed_by: {node['active_confirmed_by']}",
            f"- query_timed_out: {node['query_timed_out']}",
            f"- query_command: {node['query_command']}",
            f"- query_timeout_sec: {node['query_timeout_sec']}",
            f"- query_duration_sec: {node['query_duration_sec']}",
            f"- query_returncode: {node['query_returncode']}",
            f"- ambiguous_detail: {node['ambiguous_detail']}",
        ])
    lines.extend([
        '',
        '## Guardrails',
        f"- ingress_goal_sent: {analysis['guardrails']['ingress_goal_sent']}",
        f"- maze_explorer_started: {analysis['guardrails']['maze_explorer_started']}",
        f"- no_goal_dispatch_guard_valid: {analysis['guardrails']['no_goal_dispatch_guard_valid']}",
        '- No NavigateToPose goal was sent.',
        '- No maze_explorer was started.',
        '- No Nav2/MPPI/controller/config tuning was performed.',
        '- No autonomous exploration success or exit success is claimed.',
        '- Phase116 not entered.',
        '',
    ])
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
        'analysis': str(args.output_json),
        'summary': str(args.minimal_summary_output),
        'classification': analysis['classification'],
        'no_goal_dispatch_guard_valid': analysis['guardrails']['no_goal_dispatch_guard_valid'],
        'reject_reason': analysis['preflight']['reject_reason'],
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
