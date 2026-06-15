#!/usr/bin/env python3
"""Phase111 analyzer for real ingress preflight rerun without goal dispatch.

Consumes the Phase105/109 ingress preflight artifact produced in a real
ROS/Gazebo/Nav2 bringup and writes a JSON analysis plus human-readable minimal
summary. Analyzer is read-only and must not send NavigateToPose goals or start
maze_explorer.
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

RUN_ID = 'phase111_real_preflight_no_goal_dispatch'
ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ARTIFACT_DIR = ROOT / 'log' / RUN_ID


def _read_json(path: Path) -> dict[str, Any]:
    if not path.exists() or not path.stat().st_size:
        return {}
    try:
        value = json.loads(path.read_text(encoding='utf-8', errors='replace'))
        return value if isinstance(value, dict) else {}
    except Exception:
        return {}


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def _artifact_path(artifact_dir: Path, run_id: str) -> Path:
    preferred = artifact_dir / f'{run_id}_ingress_preflight.json'
    if preferred.exists():
        return preferred
    matches = sorted(artifact_dir.glob('*ingress_preflight*.json'))
    return matches[0] if matches else preferred


def _launch_log_path(artifact_dir: Path, run_id: str) -> Path:
    preferred = artifact_dir / f'{run_id}_launch.log'
    if preferred.exists():
        return preferred
    for pattern in [f'{run_id}_visible_launch.log', '*visible_launch.log', '*launch.log']:
        matches = sorted(artifact_dir.glob(pattern))
        if matches:
            return matches[0]
    return preferred


def _preflight(data: dict[str, Any]) -> dict[str, Any]:
    preflight = data.get('ingress_preflight')
    return preflight if isinstance(preflight, dict) else {}


def _sample_history(preflight: dict[str, Any]) -> list[dict[str, Any]]:
    rows = preflight.get('sample_history')
    return [row for row in rows if isinstance(row, dict)] if isinstance(rows, list) else []


def _lifecycle_sources(preflight: dict[str, Any]) -> dict[str, dict[str, Any]]:
    samples = _sample_history(preflight)
    source = None
    if samples:
        source = samples[-1].get('lifecycle_sources')
    if not isinstance(source, dict):
        controller_raw = preflight.get('controller_pose_check')
        controller = controller_raw if isinstance(controller_raw, dict) else {}
        lifecycle_raw = controller.get('lifecycle_sources')
        source = lifecycle_raw if isinstance(lifecycle_raw, dict) else {}
    out: dict[str, dict[str, Any]] = {}
    for node in ['controller_server', 'bt_navigator']:
        item = source.get(node) if isinstance(source, dict) else None
        item = item if isinstance(item, dict) else {}
        out[node] = {
            'derived_state': item.get('state'),
            'active_confirmed': bool(item.get('active_confirmed')),
            'inactive_confirmed': bool(item.get('inactive_confirmed')),
            'ambiguous': bool(item.get('ambiguous')),
            'query_error': bool(item.get('query_error')),
            'lifecycle_query': item.get('lifecycle_query') if isinstance(item.get('lifecycle_query'), dict) else {},
            'node_graph': item.get('node_graph') if isinstance(item.get('node_graph'), dict) else {},
            'action_availability': item.get('action_availability') if isinstance(item.get('action_availability'), dict) else {},
        }
    return out


def _scan_summary(preflight: dict[str, Any]) -> dict[str, Any]:
    samples = _sample_history(preflight)
    tokens = [token for sample in samples for token in sample.get('failed_gates', []) if isinstance(sample.get('failed_gates'), list)]
    reasons = [reason for sample in samples for values in (sample.get('failure_reasons_by_gate') or {}).values() for reason in (values if isinstance(values, list) else [])]
    latest = samples[-1].get('scan_transform_check') if samples and isinstance(samples[-1].get('scan_transform_check'), dict) else {}
    return {
        'first_scan_seen': bool(preflight.get('first_scan_seen')),
        'first_scan_wait_elapsed_sec': preflight.get('first_scan_wait_elapsed_sec'),
        'waiting_for_first_scan_observed': 'waiting_for_first_scan' in tokens or 'waiting_for_first_scan' in reasons,
        'first_scan_timeout': 'ingress_first_scan_timeout' in (preflight.get('failed_gates') or []) or 'first_scan_timeout' in reasons,
        'latest_scan_available': bool(latest.get('scan_available')),
        'latest_scan_frame_id': latest.get('scan_frame_id'),
        'latest_target_frame': latest.get('target_frame'),
        'latest_transform_available': bool(latest.get('transform_available')),
    }


def _tf_summary(preflight: dict[str, Any]) -> dict[str, Any]:
    samples = _sample_history(preflight)
    sample_tokens = [list(sample.get('failed_gates') or []) for sample in samples]
    early_missing = any('ingress_map_base_tf_missing' in tokens for tokens in sample_tokens[:-1])
    latest_tokens = sample_tokens[-1] if sample_tokens else []
    final_missing = 'ingress_map_base_tf_missing' in latest_tokens or preflight.get('ingress_preflight_reject_reason') == 'ingress_map_base_tf_missing'
    latest = samples[-1] if samples else {}
    return {
        'sample_count': len(samples),
        'early_tf_miss_observed': early_missing,
        'early_tf_miss_recovered': bool(early_missing and not final_missing),
        'stable_window_not_met': preflight.get('ingress_preflight_reject_reason') == 'ingress_tf_stable_window_not_met' or 'ingress_tf_stable_window_not_met' in (preflight.get('failed_gates') or []),
        'latest_stable_window_elapsed_sec': latest.get('stable_window_elapsed_sec'),
        'latest_map_base_tf_check': latest.get('map_base_tf_check') if isinstance(latest.get('map_base_tf_check'), dict) else {},
        'latest_map_odom_tf_age_sec': latest.get('map_odom_tf_age_sec'),
        'latest_odom_base_tf_age_sec': latest.get('odom_base_tf_age_sec'),
        'tf_sample_tokens': sample_tokens,
    }


def _sample_history_contract(preflight: dict[str, Any]) -> dict[str, Any]:
    missing: list[dict[str, Any]] = []
    for idx, sample in enumerate(_sample_history(preflight)):
        for field in ['failed_gates', 'failure_reasons_by_gate']:
            if field not in sample:
                missing.append({'sample_index': idx, 'field': field})
    return {
        'sample_count': len(_sample_history(preflight)),
        'all_samples_have_failed_gates': not any(item['field'] == 'failed_gates' for item in missing),
        'all_samples_have_failure_reasons_by_gate': not any(item['field'] == 'failure_reasons_by_gate' for item in missing),
        'missing_fields': missing,
    }


def _launch_summary(path: Path, lifecycle: dict[str, dict[str, Any]]) -> dict[str, Any]:
    text = path.read_text(encoding='utf-8', errors='replace') if path.exists() else ''
    lowered = text.lower()
    active_marker = bool(re.search(r'managed nodes.*active|nodes.*active|activated.*controller_server|activated.*bt_navigator|successfully activated', lowered))
    artifact_active = all(lifecycle.get(node, {}).get('derived_state') == 'active_confirmed' for node in ['controller_server', 'bt_navigator'])
    artifact_ambiguous = any(lifecycle.get(node, {}).get('ambiguous') or lifecycle.get(node, {}).get('query_error') for node in ['controller_server', 'bt_navigator'])
    return {
        'path': str(path),
        'exists': path.exists(),
        'managed_nav2_active_marker_seen': active_marker,
        'artifact_consistent_with_active_marker': bool((active_marker and artifact_active) or (not active_marker) or artifact_ambiguous),
        'artifact_active_confirmed': artifact_active,
        'artifact_lifecycle_ambiguous_or_query_error': artifact_ambiguous,
    }


def _classification(preflight: dict[str, Any], guardrails: dict[str, Any]) -> str:
    if guardrails['ingress_goal_sent'] or guardrails['maze_explorer_started']:
        return 'PHASE111_GUARD_VIOLATION_GOAL_OR_EXPLORER_STARTED'
    if not preflight:
        return 'PHASE111_PREFLIGHT_ARTIFACT_MISSING'
    if not guardrails['artifact_contract_valid']:
        return 'PHASE111_PREFLIGHT_ARTIFACT_CONTRACT_INVALID'
    if preflight.get('passed') is True:
        return 'PHASE111_PREFLIGHT_PASS_NO_GOAL_DISPATCH'
    return 'PHASE111_PREFLIGHT_REJECTED_NO_GOAL_DISPATCH'


def analyze_artifact_dir(artifact_dir: Path, *, run_id: str = RUN_ID) -> dict[str, Any]:
    artifact_path = _artifact_path(artifact_dir, run_id)
    data = _read_json(artifact_path)
    preflight = _preflight(data)
    lifecycle = _lifecycle_sources(preflight)
    contract = _sample_history_contract(preflight)
    raw = preflight.get('raw_style_snapshot_cross_check') if isinstance(preflight.get('raw_style_snapshot_cross_check'), dict) else {}
    guardrails = {
        'ingress_goal_sent': bool(preflight.get('ingress_goal_sent')),
        'maze_explorer_started': bool(preflight.get('maze_explorer_started')),
        'no_goal_dispatch_guard_valid': not bool(preflight.get('ingress_goal_sent')) and not bool(preflight.get('maze_explorer_started')),
        'nav2_config_tuned': False,
        'exploration_strategy_changed': False,
        'preflight_removed': False,
        'artifact_contract_valid': bool(preflight) and contract['all_samples_have_failed_gates'] and contract['all_samples_have_failure_reasons_by_gate'],
    }
    launch = _launch_summary(_launch_log_path(artifact_dir, run_id), lifecycle)
    analysis = {
        'run_id': run_id,
        'phase': 'Phase111',
        'mode': 'real_preflight_only_no_goal_dispatch',
        'artifact_path': str(artifact_path),
        'classification': None,
        'preflight': {
            'evaluated': bool(preflight.get('evaluated')),
            'passed': bool(preflight.get('passed')),
            'reject_reason': preflight.get('ingress_preflight_reject_reason'),
            'last_specific_reject_reason': preflight.get('last_specific_reject_reason'),
            'failed_gates': list(preflight.get('failed_gates') or []),
            'passed_gates': list(preflight.get('passed_gates') or []),
            'bounded_wait_elapsed_sec': preflight.get('bounded_wait_elapsed_sec'),
            'startup_grace_sec': preflight.get('startup_grace_sec'),
            'stable_window_sec': preflight.get('stable_window_sec') or preflight.get('tf_stability_window_sec'),
            'sample_period_sec': preflight.get('sample_period_sec'),
            'sample_count': preflight.get('sample_count') or len(_sample_history(preflight)),
        },
        'lifecycle': lifecycle,
        'scan': _scan_summary(preflight),
        'tf': _tf_summary(preflight),
        'sample_history_contract': contract,
        'raw_style_snapshot_cross_check': raw,
        'launch_log': launch,
        'guardrails': guardrails,
    }
    analysis['classification'] = _classification(preflight, guardrails)
    return analysis


def render_minimal_summary(analysis: dict[str, Any]) -> str:
    preflight = analysis['preflight']
    guardrails = analysis['guardrails']
    scan = analysis['scan']
    tf = analysis['tf']
    raw = analysis.get('raw_style_snapshot_cross_check') or {}
    lifecycle = analysis.get('lifecycle') or {}
    launch = analysis.get('launch_log') or {}
    lines = [
        '# Phase111 real preflight no-goal summary',
        '',
        f"classification: {analysis['classification']}",
        f"passed: {preflight['passed']}",
        f"reject_reason: {preflight['reject_reason']}",
        f"failed_gates: {preflight['failed_gates']}",
        f"ingress_goal_sent: {guardrails['ingress_goal_sent']}",
        f"maze_explorer_started: {guardrails['maze_explorer_started']}",
        f"no_goal_dispatch_guard_valid: {guardrails['no_goal_dispatch_guard_valid']}",
        '',
        '## Lifecycle',
    ]
    for node in ['controller_server', 'bt_navigator']:
        src = lifecycle.get(node, {})
        lines.append(f"- {node}: {src.get('derived_state')} query={src.get('lifecycle_query')} node_graph={src.get('node_graph')} action={src.get('action_availability')}")
    lines.extend([
        '',
        '## Scan',
        f"- waiting_for_first_scan_observed: {scan['waiting_for_first_scan_observed']}",
        f"- first_scan_seen: {scan['first_scan_seen']}",
        f"- first_scan_timeout: {scan['first_scan_timeout']}",
        f"- latest_scan_frame_id: {scan['latest_scan_frame_id']}",
        '',
        '## TF',
        f"- sample_count: {tf['sample_count']}",
        f"- early_tf_miss_observed: {tf['early_tf_miss_observed']}",
        f"- early_tf_miss_recovered: {tf['early_tf_miss_recovered']}",
        f"- stable_window_not_met: {tf['stable_window_not_met']}",
        f"- latest_stable_window_elapsed_sec: {tf['latest_stable_window_elapsed_sec']}",
        '',
        '## Raw snapshot cross-check',
        f"- present: {bool(raw.get('present'))}",
        f"- ambiguous: {bool(raw.get('ambiguous'))}",
        f"- contradictions: {raw.get('cross_check_contradictions')}",
        '',
        '## Launch log consistency',
        f"- managed_nav2_active_marker_seen: {launch.get('managed_nav2_active_marker_seen')}",
        f"- artifact_consistent_with_active_marker: {launch.get('artifact_consistent_with_active_marker')}",
        '',
        '## Contract',
        f"- all_samples_have_failed_gates: {analysis['sample_history_contract']['all_samples_have_failed_gates']}",
        f"- all_samples_have_failure_reasons_by_gate: {analysis['sample_history_contract']['all_samples_have_failure_reasons_by_gate']}",
        '',
        '## Guardrails',
        '- No NavigateToPose goal was sent by Phase111 tooling.',
        '- No maze_explorer was started by Phase111 tooling.',
        '- No Nav2/MPPI/controller/config tuning was performed.',
        '- No autonomous exploration success or exit success is claimed.',
        '- Phase112 not entered.',
    ])
    return '\n'.join(lines) + '\n'


def write_outputs(artifact_dir: Path, *, run_id: str, output_json: Path, minimal_summary_output: Path) -> dict[str, Path]:
    analysis = analyze_artifact_dir(artifact_dir, run_id=run_id)
    _write_json(output_json, analysis)
    minimal_summary_output.parent.mkdir(parents=True, exist_ok=True)
    minimal_summary_output.write_text(render_minimal_summary(analysis), encoding='utf-8')
    return {'analysis': output_json, 'summary': minimal_summary_output}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact-dir', type=Path, default=DEFAULT_ARTIFACT_DIR)
    parser.add_argument('--run-id', default=RUN_ID)
    parser.add_argument('--output-json', type=Path)
    parser.add_argument('--minimal-summary-output', type=Path)
    args = parser.parse_args()
    output_json = args.output_json or args.artifact_dir / f'{args.run_id}_analysis.json'
    summary = args.minimal_summary_output or args.artifact_dir / f'{args.run_id}_minimal_summary.md'
    paths = write_outputs(args.artifact_dir, run_id=args.run_id, output_json=output_json, minimal_summary_output=summary)
    analysis = _read_json(paths['analysis'])
    print(json.dumps({
        'classification': analysis.get('classification'),
        'passed': (analysis.get('preflight') or {}).get('passed'),
        'reject_reason': (analysis.get('preflight') or {}).get('reject_reason'),
        'no_goal_dispatch_guard_valid': (analysis.get('guardrails') or {}).get('no_goal_dispatch_guard_valid'),
        'analysis': str(paths['analysis']),
        'summary': str(paths['summary']),
    }, sort_keys=True))
    return 0 if (analysis.get('guardrails') or {}).get('no_goal_dispatch_guard_valid') else 2


if __name__ == '__main__':
    raise SystemExit(main())
