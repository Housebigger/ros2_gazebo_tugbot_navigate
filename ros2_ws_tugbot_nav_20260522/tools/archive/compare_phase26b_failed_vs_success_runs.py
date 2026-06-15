#!/usr/bin/env python3
"""Align Phase 26B failed-vs-success runs to identify variance triggers.

This is post-run analysis only. It consumes existing Phase26B artifacts and does
not start ROS/Gazebo or propose parameter tuning.
"""

from __future__ import annotations

import argparse
import json
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

TERMINAL_EVENTS = {'success', 'timeout', 'terminal_cancel', 'blocked_nav2'}


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists() or path.stat().st_size == 0:
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def load_jsonl_payloads(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists() or path.stat().st_size == 0:
        return rows
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        payload = row.get('state', row)
        if isinstance(payload, dict):
            rows.append(payload)
    return rows


def last_state(log_dir: Path, run_id: str) -> dict[str, Any]:
    rows = load_jsonl_payloads(log_dir / f'{run_id}_explorer_state.jsonl')
    return rows[-1] if rows else {}


def terminal_goal_events(log_dir: Path, run_id: str) -> list[dict[str, Any]]:
    rows = load_jsonl_payloads(log_dir / f'{run_id}_goal_events.jsonl')
    terminal: list[dict[str, Any]] = []
    seen: set[tuple[int, str]] = set()
    for row in rows:
        event = row.get('event')
        seq = row.get('goal_sequence')
        if event not in TERMINAL_EVENTS or seq is None:
            continue
        key = (int(seq), str(event))
        if key in seen:
            continue
        seen.add(key)
        terminal.append(row)
    terminal.sort(key=lambda row: int(row.get('goal_sequence', 0)))
    return terminal


def latest_by_sequence(rows: list[dict[str, Any]]) -> dict[int, dict[str, Any]]:
    result: dict[int, dict[str, Any]] = {}
    for row in rows:
        seq = row.get('goal_sequence')
        if seq is not None:
            result[int(seq)] = row
    return result


def index_subtypes(log_dir: Path, run_id: str) -> dict[int, dict[str, Any]]:
    data = load_json(log_dir / f'{run_id}_timeout_subtypes.json')
    return latest_by_sequence(data.get('timeout_subtypes', []))


def index_failure_windows(log_dir: Path, run_id: str) -> dict[int, dict[str, Any]]:
    data = load_json(log_dir / f'{run_id}_failure_windows.json')
    return latest_by_sequence(data.get('failure_windows', []))


def grouped_recoveries(log_dir: Path, run_id: str) -> dict[int, list[dict[str, Any]]]:
    data = load_json(log_dir / f'{run_id}_post_recovery_enriched.json')
    grouped: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for row in data.get('enriched_recovery_snapshots', []):
        seq = row.get('goal_sequence')
        if seq is not None:
            grouped[int(seq)].append(row)
    return dict(grouped)


def number(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def target_tuple(row: dict[str, Any]) -> list[float] | None:
    target = row.get('target')
    if isinstance(target, list) and len(target) >= 2:
        x = number(target[0])
        y = number(target[1])
        if x is not None and y is not None:
            return [round(x, 6), round(y, 6)]
    return None


def event_summary(row: dict[str, Any], subtype: dict[str, Any] | None = None, failure: dict[str, Any] | None = None, recoveries: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    recoveries = recoveries or []
    path_updates_after_recovery = [number(r.get('path_update_count_after_recovery')) for r in recoveries]
    near_zero_path_cost = [number(r.get('near_zero_path_ahead_1_0m_cost_max')) for r in recoveries]
    near_zero_path_dist = [number(r.get('near_zero_robot_to_path_distance_m')) for r in recoveries]
    return {
        'goal_sequence': row.get('goal_sequence'),
        'event': row.get('event'),
        'result_reason': row.get('result_reason'),
        'near_exit': bool(row.get('near_exit')),
        'robot_exit_dist_at_dispatch': number(row.get('robot_exit_dist_at_dispatch')),
        'target_exit_dist': number(row.get('target_exit_dist')),
        'target': target_tuple(row),
        'branch_angle': number(row.get('branch_angle')),
        'timeout_footprint_cost_max': number(row.get('timeout_footprint_cost_max')),
        'timeout_path_ahead_1_0m_cost_max': number(row.get('timeout_path_ahead_1_0m_cost_max')),
        'controller_subtype': (subtype or {}).get('controller_subtype'),
        'timing_subtype': (subtype or {}).get('timing_subtype'),
        'classification': (subtype or {}).get('classification') or (failure or {}).get('classification'),
        'near_zero_cmd_duration_sec': number((subtype or {}).get('near_zero_cmd_duration_sec') or (failure or {}).get('near_zero_cmd_duration_sec')),
        'failure_window_path_ahead_1_0m_cost_max': number((failure or {}).get('timeout_path_ahead_1_0m_cost_max')),
        'failure_window_footprint_cost_max': number((failure or {}).get('timeout_footprint_cost_max')),
        'post_recovery_count': len(recoveries),
        'post_recovery_path_update_count_max': max((v for v in path_updates_after_recovery if v is not None), default=None),
        'near_zero_path_ahead_1_0m_cost_max_max': max((v for v in near_zero_path_cost if v is not None), default=None),
        'near_zero_robot_to_path_distance_m_min': min((v for v in near_zero_path_dist if v is not None), default=None),
        'controller_received_path_but_cmd_near_zero_any': any(r.get('controller_received_path_but_cmd_near_zero') is True for r in recoveries),
    }


def read_run(log_dir: Path, run_id: str) -> dict[str, Any]:
    state = last_state(log_dir, run_id)
    events = terminal_goal_events(log_dir, run_id)
    subtypes = index_subtypes(log_dir, run_id)
    failures = index_failure_windows(log_dir, run_id)
    recoveries = grouped_recoveries(log_dir, run_id)
    event_rows = [
        event_summary(row, subtypes.get(int(row['goal_sequence'])), failures.get(int(row['goal_sequence'])), recoveries.get(int(row['goal_sequence']), []))
        for row in events
        if row.get('goal_sequence') is not None
    ]
    timeout_rows = [row for row in event_rows if row.get('event') == 'timeout']
    success_rows = [row for row in event_rows if row.get('event') == 'success']
    terminal_rows = [row for row in event_rows if row.get('event') == 'terminal_cancel']
    return {
        'run_id': run_id,
        'final_mode': state.get('mode'),
        'exit_distance_m': number(state.get('exit_distance_m')),
        'goal_count': state.get('goal_count'),
        'goal_success_count': state.get('goal_success_count'),
        'goal_failure_count': state.get('goal_failure_count'),
        'timeout_cancel_count': state.get('timeout_cancel_count'),
        'blocked_branch_count': state.get('blocked_branch_count'),
        'blacklisted_goal_count': state.get('blacklisted_goal_count'),
        'event_rows': event_rows,
        'timeout_rows': timeout_rows,
        'success_rows': success_rows,
        'terminal_rows': terminal_rows,
        'last_events': event_rows[-6:],
        'severe_late_silent_sequences': [int(row['goal_sequence']) for row in timeout_rows if row.get('controller_subtype') == 'footprint_path_blocked_late_silent'],
        'late_successes_moving_away_from_exit': [
            row for row in success_rows[-4:]
            if row.get('robot_exit_dist_at_dispatch') is not None
            and row.get('target_exit_dist') is not None
            and row['target_exit_dist'] > row['robot_exit_dist_at_dispatch'] + 0.25
        ],
    }


def median(values: list[Any]) -> float | None:
    clean = [v for v in (number(value) for value in values) if v is not None]
    return round(statistics.median(clean), 6) if clean else None


def alignment_summary(runs: list[dict[str, Any]]) -> dict[str, Any]:
    failed = [run for run in runs if run.get('final_mode') == 'FAILED_EXHAUSTED']
    reached = [run for run in runs if run.get('final_mode') == 'EXIT_REACHED']
    subtype_counts = Counter()
    failed_timeout_sequences = []
    reached_timeout_sequences = []
    for run in failed:
        for row in run.get('timeout_rows', []):
            subtype_counts[str(row.get('controller_subtype'))] += 1
            failed_timeout_sequences.append(row.get('goal_sequence'))
    for run in reached:
        for row in run.get('timeout_rows', []):
            reached_timeout_sequences.append(row.get('goal_sequence'))
    return {
        'run_count': len(runs),
        'failed_run_count': len(failed),
        'exit_reached_run_count': len(reached),
        'failed_run_ids': [run['run_id'] for run in failed],
        'exit_reached_run_ids': [run['run_id'] for run in reached],
        'failed_timeout_sequences': failed_timeout_sequences,
        'exit_reached_timeout_sequences': reached_timeout_sequences,
        'failed_timeout_controller_subtype_counts': dict(sorted(subtype_counts.items())),
        'failed_exit_distance_median': median([run.get('exit_distance_m') for run in failed]),
        'exit_reached_distance_median': median([run.get('exit_distance_m') for run in reached]),
    }


def focus_analysis(run: dict[str, Any], all_runs: list[dict[str, Any]]) -> dict[str, Any]:
    reached = [r for r in all_runs if r.get('final_mode') == 'EXIT_REACHED']
    reached_timeout_median = median([number(r.get('timeout_cancel_count')) for r in reached])
    high_exit_distance = run.get('exit_distance_m') is not None and reached and run['exit_distance_m'] > max(r.get('exit_distance_m') or 0 for r in reached) + 0.25
    timeout_count = number(run.get('timeout_cancel_count'))
    high_exit_without_high_timeout = bool(high_exit_distance and reached_timeout_median is not None and timeout_count is not None and timeout_count <= reached_timeout_median + 1)
    severe_sequences = run.get('severe_late_silent_sequences', [])
    moving_away = run.get('late_successes_moving_away_from_exit', [])
    timeout_rows = run.get('timeout_rows', [])
    post_rows = [row for row in timeout_rows if row.get('post_recovery_count', 0) > 0]
    has_path_refresh = any((row.get('post_recovery_path_update_count_max') or 0) > 0 for row in post_rows)
    persistent_cost = any((row.get('near_zero_path_ahead_1_0m_cost_max_max') or 0) >= 90 for row in post_rows)
    close_to_path = any((row.get('near_zero_robot_to_path_distance_m_min') or 999) <= 0.12 for row in post_rows)

    if high_exit_without_high_timeout and severe_sequences and moving_away and has_path_refresh and persistent_cost:
        explanation = 'route_divergence_after_near_exit_timeout_with_persistent_local_cost_pressure'
    elif severe_sequences and has_path_refresh and persistent_cost:
        explanation = 'persistent_local_cost_pressure_after_recovery'
    elif high_exit_without_high_timeout and moving_away:
        explanation = 'route_divergence_after_timeout'
    else:
        explanation = 'variance_trigger_uncertain'

    return {
        'run_id': run['run_id'],
        'final_mode': run.get('final_mode'),
        'exit_distance_m': run.get('exit_distance_m'),
        'timeout_cancel_count': run.get('timeout_cancel_count'),
        'reached_timeout_median': reached_timeout_median,
        'high_exit_distance_without_high_timeout_count': high_exit_without_high_timeout,
        'severe_late_silent_sequences': severe_sequences,
        'late_successes_moving_away_from_exit_count': len(moving_away),
        'late_successes_moving_away_from_exit': moving_away,
        'post_recovery_path_refresh_absent': not has_path_refresh,
        'persistent_near_zero_path_cost': persistent_cost,
        'robot_remained_close_to_path_during_near_zero': close_to_path,
        'timeout_rows': timeout_rows,
        'last_events': run.get('last_events', []),
        'primary_explanation': explanation,
    }


def analyze(log_dir: Path, run_ids: list[str], focus_run: str | None) -> dict[str, Any]:
    runs = [read_run(log_dir, run_id) for run_id in run_ids]
    focus = None
    if focus_run:
        matches = [run for run in runs if run['run_id'] == focus_run]
        if not matches:
            raise ValueError(f'focus run not found: {focus_run}')
        focus = focus_analysis(matches[0], runs)
    return {
        'phase': '26C',
        'purpose': 'variance trigger alignment from existing Phase26B artifacts only',
        'run_ids': run_ids,
        'alignment_summary': alignment_summary(runs),
        'runs': runs,
        'focus_run_analysis': focus,
        'decision_guardrails': [
            'No new tuning run was started for this analysis.',
            'Do not promote cost_weight=2.75 from variance-trigger evidence alone.',
            'Treat path-refresh-present + near-zero-cmd evidence as controller/local-cost pressure, not missing path publication.',
        ],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, default=Path('log'))
    parser.add_argument('--runs', required=True)
    parser.add_argument('--focus-run')
    parser.add_argument('--output-json', type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_ids = [item.strip() for item in args.runs.split(',') if item.strip()]
    payload = analyze(args.log_dir, run_ids, args.focus_run)
    text = json.dumps(payload, indent=2, sort_keys=True) + '\n'
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(text, encoding='utf-8')
    print(text)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
