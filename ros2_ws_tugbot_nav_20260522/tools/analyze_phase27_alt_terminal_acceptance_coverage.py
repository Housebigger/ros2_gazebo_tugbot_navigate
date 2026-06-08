#!/usr/bin/env python3
"""Analyze Phase27-alt-R2 terminal acceptance branch coverage.

Replay/static diagnostics-only analyzer. It reads Phase27-alt runtime artifacts
(`/maze/goal_events` JSONL and `/maze/explorer_state` JSONL) and determines
whether the terminal_acceptance branch itself was covered: a near_exit_fallback
event with action=terminal_acceptance, robot_exit_dist <= terminal radius,
recent failure/cancel evidence, followed by EXIT_REACHED.

It does not run Gazebo/Nav2, change branch selection, change Nav2/MPPI/controller
parameters, relax local-cost gates, or explain MPPI selected-control root cause.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

TERMINAL_RADIUS_M = 0.6
RECENT_FAILURE_RESULTS = {
    'goal_timeout',
    'goal_canceled_after_timeout',
    'blocked_nav2',
    'goal_rejected',
    'GOAL_TIMEOUT',
    'GOAL_CANCELED_AFTER_TIMEOUT',
    'BLOCKED_NAV2',
    'GOAL_REJECTED',
}
RECENT_FAILURE_EVENTS = {'timeout', 'timeout_cancel_result', 'failure'}


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists() or path.stat().st_size == 0:
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
            row = dict(payload)
            for key in ('wall_time', 'elapsed_sec', 'seq'):
                if isinstance(raw, dict) and key in raw and key not in row:
                    row[key] = raw.get(key)
            rows.append(row)
    return rows


def number(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def event_time(row: dict[str, Any]) -> float | None:
    return number(row.get('elapsed_sec')) or number(row.get('wall_time'))


def compact(row: dict[str, Any]) -> dict[str, Any]:
    keys = [
        'event',
        'mode',
        'goal_sequence',
        'goal_kind',
        'elapsed_sec',
        'near_exit_fallback_triggered',
        'fallback_reason',
        'action',
        'robot_exit_dist',
        'last_nav2_result',
        'near_exit_fallback_attempts',
        'near_exit_fallback_max_attempts',
        'exit_distance_m',
        'blocked_branch_count',
        'blacklisted_goal_count',
    ]
    return {key: row.get(key) for key in keys if key in row}


def is_recent_failure_result(value: Any) -> bool:
    if value is None:
        return False
    return str(value) in RECENT_FAILURE_RESULTS


def nearest_prior_failure(goal_rows: list[dict[str, Any]], timestamp: float | None) -> dict[str, Any] | None:
    candidates = []
    for row in goal_rows:
        if row.get('event') not in RECENT_FAILURE_EVENTS:
            continue
        row_time = event_time(row)
        if timestamp is not None and row_time is not None and row_time > timestamp:
            continue
        candidates.append(row)
    if not candidates:
        return None
    candidates.sort(key=lambda row: event_time(row) or -1.0)
    return candidates[-1]


def final_state(state_rows: list[dict[str, Any]], goal_rows: list[dict[str, Any]]) -> dict[str, Any]:
    last_state = state_rows[-1] if state_rows else {}
    last_goal = goal_rows[-1] if goal_rows else {}
    return {
        'final_mode': last_state.get('mode'),
        'final_exit_distance_m': number(last_state.get('exit_distance_m')),
        'blocked_branch_count': last_state.get('blocked_branch_count'),
        'blacklisted_goal_count': last_state.get('blacklisted_goal_count'),
        'last_goal_event': last_goal.get('event'),
        'last_goal_mode': last_goal.get('mode'),
    }


def exit_radius_window(state_rows: list[dict[str, Any]]) -> dict[str, Any]:
    values = [(row, number(row.get('exit_distance_m'))) for row in state_rows]
    values = [(row, value) for row, value in values if value is not None]
    if not values:
        return {
            'state_count_at_or_below_terminal_radius': 0,
            'min_exit_distance_m': None,
            'first_state_at_or_below_terminal_radius': None,
            'last_state_at_or_below_terminal_radius': None,
        }
    at_or_below = [(row, value) for row, value in values if value <= TERMINAL_RADIUS_M]
    min_value = min(value for _, value in values)
    return {
        'state_count_at_or_below_terminal_radius': len(at_or_below),
        'min_exit_distance_m': min_value,
        'first_state_at_or_below_terminal_radius': compact(at_or_below[0][0]) if at_or_below else None,
        'last_state_at_or_below_terminal_radius': compact(at_or_below[-1][0]) if at_or_below else None,
    }


def terminal_acceptance_events(goal_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    events = []
    for row in goal_rows:
        if row.get('event') != 'near_exit_fallback':
            continue
        if row.get('action') != 'terminal_acceptance':
            continue
        if row.get('fallback_reason') != 'terminal_acceptance_radius':
            continue
        events.append(row)
    return events


def annotate_terminal_acceptance_event(row: dict[str, Any], goal_rows: list[dict[str, Any]], state_rows: list[dict[str, Any]]) -> dict[str, Any]:
    timestamp = event_time(row)
    prior_failure = nearest_prior_failure(goal_rows, timestamp)
    state_after = None
    for state in state_rows:
        state_time = event_time(state)
        if timestamp is not None and state_time is not None and state_time < timestamp:
            continue
        if state.get('mode') == 'EXIT_REACHED':
            state_after = state
            break
    last_result_ok = is_recent_failure_result(row.get('last_nav2_result'))
    prior_failure_ok = prior_failure is not None
    robot_exit_dist = number(row.get('robot_exit_dist'))
    return {
        **compact(row),
        'robot_exit_dist_at_or_below_terminal_radius': robot_exit_dist is not None and robot_exit_dist <= TERMINAL_RADIUS_M,
        'last_nav2_result_is_recent_failure_evidence': last_result_ok,
        'prior_failure_event': compact(prior_failure) if prior_failure else None,
        'prior_failure_event_present': prior_failure_ok,
        'exit_reached_after_event': state_after is not None,
        'first_exit_reached_state_after_event': compact(state_after) if state_after else None,
    }


def no_covered_reason(goal_rows: list[dict[str, Any]], state_rows: list[dict[str, Any]], window: dict[str, Any]) -> str:
    if terminal_acceptance_events(goal_rows):
        return 'terminal_acceptance_event_present_but_invalid'
    if window.get('state_count_at_or_below_terminal_radius', 0) > 0:
        return 'exit_radius_state_seen_but_no_terminal_acceptance_event'
    if any(row.get('event') == 'near_exit_fallback' for row in goal_rows):
        return 'near_exit_fallback_events_present_but_no_terminal_acceptance_action'
    return 'no_terminal_radius_or_terminal_acceptance_evidence'


def analyze(goal_events_path: Path, explorer_state_path: Path) -> dict[str, Any]:
    goal_rows = load_jsonl(goal_events_path)
    state_rows = load_jsonl(explorer_state_path)
    term_events = terminal_acceptance_events(goal_rows)
    annotated = [annotate_terminal_acceptance_event(row, goal_rows, state_rows) for row in term_events]
    valid = [
        row for row in annotated
        if row.get('near_exit_fallback_triggered') is True
        and row.get('robot_exit_dist_at_or_below_terminal_radius') is True
        and row.get('last_nav2_result_is_recent_failure_evidence') is True
        and row.get('exit_reached_after_event') is True
    ]
    invalid_missing_failure = [
        row for row in annotated
        if row.get('near_exit_fallback_triggered') is True
        and row.get('robot_exit_dist_at_or_below_terminal_radius') is True
        and row.get('last_nav2_result_is_recent_failure_evidence') is False
    ]
    window = exit_radius_window(state_rows)
    fstate = final_state(state_rows, goal_rows)
    if valid:
        coverage_status = 'covered'
        conclusion = 'PASS_TERMINAL_ACCEPTANCE_BRANCH_VALIDATED'
        not_covered_reason = None
    elif invalid_missing_failure:
        coverage_status = 'invalid_terminal_acceptance_evidence'
        conclusion = 'TERMINAL_ACCEPTANCE_EVENT_MISSING_RECENT_FAILURE_EVIDENCE'
        not_covered_reason = None
    else:
        coverage_status = 'not_covered'
        conclusion = 'NOT_COVERED_TERMINAL_ACCEPTANCE_BRANCH'
        not_covered_reason = no_covered_reason(goal_rows, state_rows, window)
    return {
        'phase': 'Phase27-alt-R2',
        'inputs': {
            'goal_events': str(goal_events_path),
            'explorer_state': str(explorer_state_path),
        },
        'counts': {
            'goal_event_rows': len(goal_rows),
            'explorer_state_rows': len(state_rows),
            'terminal_acceptance_event_count': len(term_events),
        },
        'terminal_radius_m': TERMINAL_RADIUS_M,
        'coverage_status': coverage_status,
        'conclusion': conclusion,
        'not_covered_reason': not_covered_reason,
        'terminal_acceptance_event': valid[0] if valid else (annotated[0] if annotated else None),
        'terminal_acceptance_events': annotated[:20],
        'nearest_exit_radius_state_window': window,
        'final_state': fstate,
        'bounded_runtime_smoke_recommendation': {
            'recommended': coverage_status != 'covered',
            'reason': 'R1 replay/static logs do not cover the terminal_acceptance branch' if coverage_status != 'covered' else None,
            'keep_thresholds_default': True,
            'do_not_relax_local_cost_gate': True,
            'do_not_change_branch_selection': True,
            'do_not_change_nav2_mppi_controller_params': True,
        },
        'guardrails': {
            'diagnostics_only_replay_static_validation': True,
            'nav2_mppi_controller_source_fetched': False,
            'nav2_mppi_controller_params_modified': False,
            'costcritic_275_promoted_or_rejected': False,
            'branch_selection_strategy_modified': False,
            'local_cost_gate_relaxed': False,
        },
        'mppi_root_cause_claim': 'not_evaluated_by_phase27_alt_r2',
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal-events', required=True, type=Path)
    parser.add_argument('--explorer-state', required=True, type=Path)
    parser.add_argument('--output-json', required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = analyze(args.goal_events, args.explorer_state)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({
        'output': str(args.output_json),
        'coverage_status': result['coverage_status'],
        'conclusion': result['conclusion'],
        'not_covered_reason': result['not_covered_reason'],
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
