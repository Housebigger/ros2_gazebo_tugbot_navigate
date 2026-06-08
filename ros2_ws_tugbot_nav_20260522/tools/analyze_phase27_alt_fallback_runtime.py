#!/usr/bin/env python3
"""Analyze Phase27-alt-R1 near-exit fallback runtime smoke artifacts.

Diagnostics-only: consumes /maze/goal_events JSONL, optionally explorer_state
JSONL, and reports event coverage, fallback gate behavior, topology
non-pollution, final mode / exit distance, and explicit guardrails. It does not
modify Nav2, MPPI, controller parameters, or branch selection.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

REQUIRED_FALLBACK_FIELDS = [
    'near_exit_fallback_triggered',
    'fallback_reason',
    'robot_exit_dist',
    'cmd_near_zero_duration',
    'last_nav2_result',
    'robot_to_path_distance',
    'action',
]
TRIGGER_REQUIRED_FIELDS = [
    'action',
    'fallback_reason',
    'robot_exit_dist',
    'near_exit_fallback_attempts',
]
MICRO_GOAL_DIAGNOSTIC_FIELDS = [
    'micro_goal_geometry',
    'micro_goal_local_cost',
]


def load_jsonl(path: Path | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if path is None or not path.exists() or path.stat().st_size == 0:
        return rows
    for line_no, line in enumerate(path.read_text(encoding='utf-8').splitlines(), start=1):
        if not line.strip():
            continue
        try:
            raw = json.loads(line)
        except json.JSONDecodeError as exc:
            rows.append({'_parse_error': str(exc), '_line_no': line_no, '_raw': line})
            continue
        payload = raw.get('state', raw) if isinstance(raw, dict) else raw
        if isinstance(payload, dict):
            if isinstance(raw, dict) and 'wall_time' in raw and 'wall_time' not in payload:
                payload = dict(payload)
                payload['wall_time'] = raw.get('wall_time')
            rows.append(payload)
    return rows


def number(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def integer(value: Any) -> int | None:
    num = number(value)
    if num is None:
        return None
    return int(num)


def is_fallback_field_row(row: dict[str, Any]) -> bool:
    return any(field in row for field in REQUIRED_FALLBACK_FIELDS) or row.get('event') == 'near_exit_fallback'


def is_fallback_event(row: dict[str, Any]) -> bool:
    return row.get('event') == 'near_exit_fallback'


def compact_event(row: dict[str, Any]) -> dict[str, Any]:
    keys = [
        'event',
        'goal_sequence',
        'goal_kind',
        'near_exit_fallback_triggered',
        'fallback_reason',
        'robot_exit_dist',
        'cmd_near_zero_duration',
        'last_nav2_result',
        'robot_to_path_distance',
        'action',
        'near_exit_fallback_attempts',
        'near_exit_fallback_max_attempts',
        'target',
        'micro_goal_geometry',
        'micro_goal_local_cost',
        'blocked_branch_count',
        'blacklisted_goal_count',
    ]
    return {key: row.get(key) for key in keys if key in row}


def analyze_schema(goal_rows: list[dict[str, Any]]) -> dict[str, Any]:
    field_rows = [row for row in goal_rows if is_fallback_field_row(row)]
    present = {field for row in field_rows for field in REQUIRED_FALLBACK_FIELDS if field in row}
    missing = [field for field in REQUIRED_FALLBACK_FIELDS if field not in present]
    triggered = [row for row in goal_rows if row.get('near_exit_fallback_triggered') is True]
    trigger_missing: list[dict[str, Any]] = []
    for row in triggered:
        missing_for_row = [field for field in TRIGGER_REQUIRED_FIELDS if field not in row]
        if row.get('action') == 'micro_goal':
            missing_for_row.extend(field for field in MICRO_GOAL_DIAGNOSTIC_FIELDS if field not in row)
        if missing_for_row:
            trigger_missing.append({
                'goal_sequence': row.get('goal_sequence'),
                'event': row.get('event'),
                'missing_fields': sorted(set(missing_for_row)),
            })
    return {
        'field_row_count': len(field_rows),
        'required_fields': REQUIRED_FALLBACK_FIELDS,
        'present_required_fields': sorted(present),
        'missing_required_fields': missing,
        'required_fields_present': not missing,
        'triggered_event_required_field_violations': trigger_missing,
        'triggered_event_required_fields_present': not trigger_missing,
    }


def min_exit_distance(rows: list[dict[str, Any]]) -> float | None:
    values = [number(row.get('robot_exit_dist')) for row in rows]
    values = [value for value in values if value is not None]
    if not values:
        values = [number(row.get('exit_distance_m')) for row in rows]
        values = [value for value in values if value is not None]
    return min(values) if values else None


def analyze_fallback(goal_rows: list[dict[str, Any]]) -> dict[str, Any]:
    fallback_events = [row for row in goal_rows if is_fallback_event(row)]
    triggered = [row for row in fallback_events if row.get('near_exit_fallback_triggered') is True]
    no_action = [row for row in fallback_events if row.get('near_exit_fallback_triggered') is not True]
    actions: dict[str, int] = {}
    reasons: dict[str, int] = {}
    no_trigger_reasons: dict[str, int] = {}
    for row in fallback_events:
        action = str(row.get('action'))
        actions[action] = actions.get(action, 0) + 1
        reason = str(row.get('fallback_reason'))
        reasons[reason] = reasons.get(reason, 0) + 1
        if row.get('near_exit_fallback_triggered') is not True:
            no_trigger_reasons[reason] = no_trigger_reasons.get(reason, 0) + 1
    no_trigger_reason = None
    if not triggered:
        if no_trigger_reasons:
            no_trigger_reason = sorted(no_trigger_reasons.items(), key=lambda item: (-item[1], item[0]))[0][0]
        elif not fallback_events:
            no_trigger_reason = 'no_near_exit_fallback_events_observed'
        else:
            no_trigger_reason = 'fallback_events_present_but_none_triggered'
    return {
        'event_count': len(fallback_events),
        'triggered_count': len(triggered),
        'no_action_event_count': len(no_action),
        'actions': actions,
        'fallback_reasons': reasons,
        'no_trigger_reasons': no_trigger_reasons,
        'no_trigger_reason': no_trigger_reason,
        'min_robot_exit_dist_observed': min_exit_distance(goal_rows),
        'triggered_events': [compact_event(row) for row in triggered[:20]],
        'no_action_events': [compact_event(row) for row in no_action[:20]],
    }


def analyze_topology(goal_rows: list[dict[str, Any]], state_rows: list[dict[str, Any]]) -> dict[str, Any]:
    violations: list[dict[str, Any]] = []
    micro_goal_active = False
    micro_goal_start_blocked: int | None = None
    micro_goal_start_blacklisted: int | None = None
    micro_goal_seq: Any = None

    for row in goal_rows:
        goal_kind = row.get('goal_kind')
        event = row.get('event')
        if goal_kind == 'near_exit_micro_goal' and event in {'dispatch', 'near_exit_fallback'}:
            micro_goal_active = True
            micro_goal_start_blocked = integer(row.get('blocked_branch_count'))
            micro_goal_start_blacklisted = integer(row.get('blacklisted_goal_count'))
            micro_goal_seq = row.get('goal_sequence')
        elif goal_kind == 'near_exit_micro_goal' and event in {'failure', 'timeout'}:
            blocked = integer(row.get('blocked_branch_count'))
            blacklisted = integer(row.get('blacklisted_goal_count'))
            if micro_goal_start_blocked is not None and blocked is not None and blocked > micro_goal_start_blocked:
                violations.append({
                    'goal_sequence': row.get('goal_sequence'),
                    'event': event,
                    'counter': 'blocked_branch_count',
                    'before': micro_goal_start_blocked,
                    'after': blocked,
                })
            if micro_goal_start_blacklisted is not None and blacklisted is not None and blacklisted > micro_goal_start_blacklisted:
                violations.append({
                    'goal_sequence': row.get('goal_sequence'),
                    'event': event,
                    'counter': 'blacklisted_goal_count',
                    'before': micro_goal_start_blacklisted,
                    'after': blacklisted,
                })
            micro_goal_active = False
        elif micro_goal_active and row.get('goal_sequence') == micro_goal_seq:
            blocked = integer(row.get('blocked_branch_count'))
            blacklisted = integer(row.get('blacklisted_goal_count'))
            if micro_goal_start_blocked is not None and blocked is not None and blocked > micro_goal_start_blocked:
                violations.append({'goal_sequence': row.get('goal_sequence'), 'event': event, 'counter': 'blocked_branch_count', 'before': micro_goal_start_blocked, 'after': blocked})
            if micro_goal_start_blacklisted is not None and blacklisted is not None and blacklisted > micro_goal_start_blacklisted:
                violations.append({'goal_sequence': row.get('goal_sequence'), 'event': event, 'counter': 'blacklisted_goal_count', 'before': micro_goal_start_blacklisted, 'after': blacklisted})

    state_counts = {
        'first_blocked_branch_count': None,
        'last_blocked_branch_count': None,
        'first_blacklisted_goal_count': None,
        'last_blacklisted_goal_count': None,
    }
    if state_rows:
        first = state_rows[0]
        last = state_rows[-1]
        state_counts = {
            'first_blocked_branch_count': integer(first.get('blocked_branch_count')),
            'last_blocked_branch_count': integer(last.get('blocked_branch_count')),
            'first_blacklisted_goal_count': integer(first.get('blacklisted_goal_count')),
            'last_blacklisted_goal_count': integer(last.get('blacklisted_goal_count')),
        }
    return {
        'passed': not violations,
        'violations': violations,
        **state_counts,
    }


def final_state_summary(state_rows: list[dict[str, Any]], goal_rows: list[dict[str, Any]]) -> dict[str, Any]:
    last = state_rows[-1] if state_rows else {}
    last_goal = goal_rows[-1] if goal_rows else {}
    return {
        'final_mode': last.get('mode'),
        'final_exit_distance_m': number(last.get('exit_distance_m')),
        'goal_count': integer(last.get('goal_count')),
        'goal_success_count': integer(last.get('goal_success_count')),
        'goal_failure_count': integer(last.get('goal_failure_count')),
        'blocked_branch_count': integer(last.get('blocked_branch_count')),
        'blacklisted_goal_count': integer(last.get('blacklisted_goal_count')),
        'last_goal_event': last_goal.get('event'),
        'last_goal_sequence': last_goal.get('goal_sequence'),
        'last_goal_robot_exit_dist': number(last_goal.get('robot_exit_dist')),
    }


def conclude(schema: dict[str, Any], fallback: dict[str, Any], topology: dict[str, Any]) -> str:
    if not topology.get('passed'):
        return 'runtime_topology_pollution_detected'
    if not schema.get('required_fields_present'):
        return 'runtime_fallback_event_schema_incomplete'
    if not schema.get('triggered_event_required_fields_present'):
        return 'runtime_triggered_fallback_event_schema_incomplete'
    if int(fallback.get('triggered_count') or 0) > 0:
        return 'runtime_fallback_triggered_without_topology_pollution'
    return 'runtime_no_fallback_trigger_observed'


def analyze(goal_events: Path, explorer_state: Path | None) -> dict[str, Any]:
    goal_rows = load_jsonl(goal_events)
    state_rows = load_jsonl(explorer_state)
    schema = analyze_schema(goal_rows)
    fallback = analyze_fallback(goal_rows)
    topology = analyze_topology(goal_rows, state_rows)
    final_state = final_state_summary(state_rows, goal_rows)
    return {
        'phase': 'Phase27-alt-R1',
        'inputs': {
            'goal_events': str(goal_events),
            'explorer_state': str(explorer_state) if explorer_state else None,
        },
        'counts': {
            'goal_event_rows': len(goal_rows),
            'explorer_state_rows': len(state_rows),
        },
        'schema': schema,
        'fallback': fallback,
        'topology_non_pollution': topology,
        'final_state': final_state,
        'guardrails': {
            'diagnostics_only_runtime_validation': True,
            'nav2_mppi_controller_params_modified': False,
            'branch_selection_strategy_modified': False,
            'costcritic_275_promoted_or_rejected': False,
            'mppi_source_overlay_used': False,
        },
        'mppi_root_cause_claim': 'not_evaluated_by_phase27_alt_r1',
        'conclusion': conclude(schema, fallback, topology),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal-events', required=True, type=Path)
    parser.add_argument('--explorer-state', type=Path)
    parser.add_argument('--output-json', required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = analyze(args.goal_events, args.explorer_state)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({
        'output': str(args.output_json),
        'conclusion': result['conclusion'],
        'fallback_triggered_count': result['fallback']['triggered_count'],
        'topology_non_pollution_passed': result['topology_non_pollution']['passed'],
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
