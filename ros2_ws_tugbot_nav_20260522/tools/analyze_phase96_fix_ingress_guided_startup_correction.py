#!/usr/bin/env python3
"""Phase96-fix ingress-guided startup correction analyzer.

Validation-only analyzer for the Phase96 correction: an explicit inner-ingress
Nav2 goal must complete before starting maze_explorer. The primary check is
whether maze_explorer dispatches at least one explore goal after ingress.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

RUN_ID = 'phase96_fix_ingress_guided_startup_correction'
ALLOWED_CLASSIFICATIONS = {
    'INGRESS_GUIDED_DISPATCH_OBSERVED',
    'INGRESS_GUIDED_DISPATCH_STILL_BLOCKED',
    'INGRESS_GUIDED_INGRESS_FAILED',
    'INGRESS_GUIDED_INSUFFICIENT_EVIDENCE',
}

JSONDict = dict[str, Any]


def _read_json(path: Path | None, default: Any = None) -> Any:
    if default is None:
        default = {}
    if not path or not path.exists() or not path.stat().st_size:
        return default
    try:
        return json.loads(path.read_text(encoding='utf-8', errors='replace'))
    except Exception:
        return default


def _read_jsonl(path: Path | None) -> list[JSONDict]:
    rows: list[JSONDict] = []
    if not path or not path.exists() or not path.stat().st_size:
        return rows
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        try:
            raw = json.loads(line)
        except Exception:
            continue
        payload = raw.get('state') if isinstance(raw, dict) and isinstance(raw.get('state'), dict) else raw
        if isinstance(payload, dict):
            row = dict(payload)
            if isinstance(raw, dict):
                row.setdefault('_recorder_elapsed_sec', raw.get('elapsed_sec'))
                row.setdefault('_recorder_wall_time', raw.get('wall_time'))
                row.setdefault('_recorder_seq', raw.get('seq'))
            rows.append(row)
    return rows


def _glob_first(artifact_dir: Path, suffix: str, run_id: str = RUN_ID) -> Path | None:
    candidates = [artifact_dir / f'{run_id}_{suffix}', artifact_dir / suffix]
    candidates.extend(sorted(artifact_dir.glob(f'*{suffix}')))
    for path in candidates:
        if path.exists():
            return path
    return None


def _goal_seq(row: JSONDict) -> int | None:
    value = row.get('goal_sequence')
    try:
        return int(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def _ingress_status(ingress: JSONDict) -> str:
    status = str(ingress.get('status') or ingress.get('status_text') or '').lower()
    if ingress.get('success') is True or status in {'succeeded', 'status_succeeded'} or status.endswith('succeeded'):
        return 'succeeded'
    if ingress.get('goal_sent') is False or ingress.get('action_server_available') is False:
        return 'not_sent'
    if ingress.get('goal_accepted') is False:
        return 'rejected'
    if ingress.get('result_received') is False or 'timeout' in str(ingress.get('reason') or '').lower():
        return 'timeout'
    if ingress:
        return 'failed'
    return 'missing'


def _last_state_summary(states: list[JSONDict]) -> JSONDict:
    if not states:
        return {}
    last = states[-1]
    gate_raw = last.get('dispatch_readiness_gate')
    gate: JSONDict = gate_raw if isinstance(gate_raw, dict) else {}
    blocking_reasons = last.get('dispatch_readiness_blocking_reasons') or gate.get('blocking_reasons')
    checks = gate.get('checks')
    return {
        'mode': last.get('mode'),
        'goal_count': last.get('goal_count'),
        'goal_success_count': last.get('goal_success_count'),
        'goal_failure_count': last.get('goal_failure_count'),
        'goal_sequence_id': last.get('goal_sequence_id'),
        'active_goal_sequence_id': last.get('active_goal_sequence_id'),
        'blocking_reasons': blocking_reasons,
        'dispatch_readiness_checks': checks,
        'dispatch_readiness_gate_passed': last.get('dispatch_readiness_gate_passed'),
        'blocked_branch_count': last.get('blocked_branch_count'),
        'blacklisted_goal_count': last.get('blacklisted_goal_count'),
    }


def _raw_capture_gaps(raw: JSONDict) -> list[str]:
    gaps: list[str] = []
    if not raw:
        return ['raw_capture_missing']
    for key in ['scan', 'map', 'local_costmap', 'odom', 'tf']:
        if not raw.get(key):
            gaps.append(f'{key}_capture_missing')
    return gaps


def _terminal_events(events: list[JSONDict]) -> list[JSONDict]:
    terminals = {'success', 'succeeded', 'timeout', 'failure', 'failed', 'cancel', 'canceled', 'cancelled'}
    return [row for row in events if str(row.get('event') or '').lower() in terminals]


def analyze_artifacts(artifact_dir: str | Path, *, run_id: str = RUN_ID, max_goals: int = 3) -> JSONDict:
    artifact = Path(artifact_dir)
    ingress = _read_json(_glob_first(artifact, 'ingress_result.json', run_id), {})
    events = _read_jsonl(_glob_first(artifact, 'goal_events.jsonl', run_id))
    states = _read_jsonl(_glob_first(artifact, 'explorer_state.jsonl', run_id))
    nav2_feedback = _read_jsonl(_glob_first(artifact, 'nav2_feedback.jsonl', run_id))
    local_samples = _read_jsonl(_glob_first(artifact, 'local_costmap_samples.jsonl', run_id))
    raw_capture = _read_json(_glob_first(artifact, 'raw_capture.json', run_id), {})

    ingress_state = _ingress_status(ingress)
    dispatch_events = [row for row in events if str(row.get('event') or '').lower() == 'dispatch']
    observed_sequences = sorted({seq for seq in (_goal_seq(row) for row in events) if seq is not None})[:max_goals]
    terminals = _terminal_events(events)
    raw_gaps = _raw_capture_gaps(raw_capture if isinstance(raw_capture, dict) else {})
    evidence_gaps: list[str] = []
    if not ingress:
        evidence_gaps.append('ingress_result_missing')
    if not states and not dispatch_events:
        evidence_gaps.append('explorer_state_missing_without_dispatch')
    evidence_gaps.extend(raw_gaps)

    if ingress_state != 'succeeded':
        classification = 'INGRESS_GUIDED_INGRESS_FAILED' if ingress else 'INGRESS_GUIDED_INSUFFICIENT_EVIDENCE'
    elif dispatch_events:
        classification = 'INGRESS_GUIDED_DISPATCH_OBSERVED'
    elif not evidence_gaps or evidence_gaps == ['explorer_state_missing_without_dispatch']:
        classification = 'INGRESS_GUIDED_DISPATCH_STILL_BLOCKED'
    elif states:
        classification = 'INGRESS_GUIDED_DISPATCH_STILL_BLOCKED'
    else:
        classification = 'INGRESS_GUIDED_INSUFFICIENT_EVIDENCE'

    return {
        'run_id': run_id,
        'artifact_dir': str(artifact),
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'classification': classification,
        'max_goals_configured': int(max_goals),
        'ingress': {
            'status': ingress_state,
            'success': ingress_state == 'succeeded',
            'target': ingress.get('target') or ingress.get('goal_pose') or ingress.get('inner_ingress_waypoint_map'),
            'distance_to_goal_m': ingress.get('distance_to_goal_m') or ingress.get('final_distance_to_ingress_m'),
            'raw': ingress,
        },
        'dispatch_observed': bool(dispatch_events),
        'dispatch_event_count': len(dispatch_events),
        'first_dispatch_event': dispatch_events[0] if dispatch_events else None,
        'observed_goal_sequences': observed_sequences,
        'observed_goal_count': len(observed_sequences),
        'terminal_events': terminals,
        'terminal_event_count': len(terminals),
        'last_explorer_state': _last_state_summary(states),
        'evidence_gaps': evidence_gaps if classification != 'INGRESS_GUIDED_DISPATCH_OBSERVED' else raw_gaps,
        'stream_counts': {
            'goal_events': len(events),
            'explorer_state': len(states),
            'nav2_feedback': len(nav2_feedback),
            'local_costmap_samples': len(local_samples),
        },
        'raw_capture_summary': {
            'available': bool(raw_capture),
            'scan_available': bool(raw_capture.get('scan')) if isinstance(raw_capture, dict) else False,
            'map_available': bool(raw_capture.get('map')) if isinstance(raw_capture, dict) else False,
            'local_costmap_available': bool(raw_capture.get('local_costmap')) if isinstance(raw_capture, dict) else False,
            'odom_available': bool(raw_capture.get('odom')) if isinstance(raw_capture, dict) else False,
            'tf_available': bool(raw_capture.get('tf')) if isinstance(raw_capture, dict) else False,
        },
        'guardrails': {
            'algorithm_changed': False,
            'phase88_92_refinement_logic_changed': False,
            'branch_scoring_or_exploration_order_changed': False,
            'centerline_gate_or_directional_readiness_changed': False,
            'fallback_or_terminal_acceptance_changed': False,
            'nav2_config_changed': False,
            'nav2_mppi_controller_tuned': False,
            'inflation_robot_radius_clearance_map_threshold_tuned': False,
            'no_autonomous_exploration_success_claimed': True,
            'no_exit_success_claimed': True,
            'phase97_not_entered': True,
        },
    }


def _write_minimal_summary(result: JSONDict, path: Path) -> None:
    lines = [
        '# Phase96-fix minimal field summary',
        '',
        f"Classification: {result.get('classification')}",
        f"Run/artifacts: {result.get('artifact_dir')}",
        f"Ingress status: {result.get('ingress', {}).get('status')}",
        f"Dispatch observed: {result.get('dispatch_observed')}",
        f"Observed goal sequences: {result.get('observed_goal_sequences')}",
        f"Terminal event count: {result.get('terminal_event_count')}",
        f"Evidence gaps: {result.get('evidence_gaps')}",
        f"Last explorer mode: {result.get('last_explorer_state', {}).get('mode')}",
        f"Last blocking reasons: {result.get('last_explorer_state', {}).get('blocking_reasons')}",
        '',
        'No autonomous exploration success claimed. No exit success claimed. Phase97 not entered.',
        '',
    ]
    path.write_text('\n'.join(lines), encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact-dir', type=Path, required=True)
    parser.add_argument('--run-id', default=RUN_ID)
    parser.add_argument('--max-goals', type=int, default=3)
    parser.add_argument('--output-json', type=Path)
    parser.add_argument('--minimal-summary-output', type=Path)
    args = parser.parse_args()
    result = analyze_artifacts(args.artifact_dir, run_id=args.run_id, max_goals=args.max_goals)
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    if args.minimal_summary_output:
        args.minimal_summary_output.parent.mkdir(parents=True, exist_ok=True)
        _write_minimal_summary(result, args.minimal_summary_output)
    print(json.dumps({'classification': result['classification'], 'dispatch_observed': result['dispatch_observed'], 'evidence_gaps': result['evidence_gaps']}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
