#!/usr/bin/env python3
"""Analyze Phase 26D branch-choice snapshots from existing goal events.

Post-run only: this script consumes Phase26B/26C artifacts and does not start
ROS/Gazebo. Its success criterion follows the project requirement that reaching
the configured exit coordinates is success; full-map exploration is not required.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

TERMINAL_EVENTS = {'dispatch', 'success', 'timeout', 'terminal_cancel'}
EXIT_RADIUS_M = 0.6


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


def exit_progress_delta(row: dict[str, Any]) -> float | None:
    robot = number(row.get('robot_exit_dist_at_dispatch'))
    target = number(row.get('target_exit_dist'))
    if robot is None or target is None:
        return None
    return round(robot - target, 6)


def classify_choice(row: dict[str, Any]) -> str:
    event = row.get('event')
    if event == 'terminal_cancel' and row.get('result_reason') == 'exit_reached':
        return 'exit_reached_terminal_success'
    delta = exit_progress_delta(row)
    target_exit = number(row.get('target_exit_dist'))
    if target_exit is not None and target_exit <= EXIT_RADIUS_M:
        return 'target_inside_exit_radius'
    if delta is None:
        return 'unknown_exit_progress'
    if delta > 0.25:
        return 'exit_progress_branch'
    if delta < -0.25:
        return 'successful_branch_moved_away_from_exit' if event == 'success' else 'branch_moves_away_from_exit'
    return 'exit_neutral_branch'


def event_row(row: dict[str, Any]) -> dict[str, Any]:
    delta = exit_progress_delta(row)
    return {
        'goal_sequence': row.get('goal_sequence'),
        'event': row.get('event'),
        'result_reason': row.get('result_reason'),
        'current_node_id': row.get('current_node_id'),
        'start_node_id': row.get('start_node_id'),
        'local_topology': row.get('local_topology'),
        'last_candidate_count': row.get('last_candidate_count'),
        'last_open_direction_count': row.get('last_open_direction_count'),
        'robot_exit_dist_at_dispatch': number(row.get('robot_exit_dist_at_dispatch')),
        'target_exit_dist': number(row.get('target_exit_dist')),
        'exit_progress_delta_m': delta,
        'target': target_tuple(row),
        'branch_angle': number(row.get('branch_angle')),
        'near_exit': bool(row.get('near_exit')),
        'dispatch_path_local_cost_max': number(row.get('dispatch_path_local_cost_max')),
        'dispatch_path_local_cost_mean': number(row.get('dispatch_path_local_cost_mean')),
        'dispatch_target_local_cost': number(row.get('dispatch_target_local_cost')),
        'dispatch_target_local_cost_max_radius': number(row.get('dispatch_target_local_cost_max_radius')),
        'target_clearance_m': number(row.get('target_clearance_m')),
        'path_corridor_min_clearance_m': number(row.get('path_corridor_min_clearance_m')),
        'line_of_sight_min_clearance_m': number(row.get('line_of_sight_min_clearance_m')),
        'target_crosses_narrow_passage': row.get('target_crosses_narrow_passage'),
        'timeout_path_ahead_1_0m_cost_max': number(row.get('timeout_path_ahead_1_0m_cost_max')),
        'classification': classify_choice(row),
        'artifact_gaps': {
            'chosen_branch_rank': 'chosen_branch_rank' not in row,
            'rejected_branch_summary': 'rejected_branch_summary' not in row,
            'candidate_branch_target_exit_distances': 'candidate_branch_target_exit_distances' not in row,
            'candidate_branch_local_path_costs': 'candidate_branch_local_path_costs' not in row,
            'reverse_backtrack_context': 'reverse_backtrack_context' not in row,
        },
    }


def read_run(log_dir: Path, run_id: str) -> dict[str, Any]:
    payloads = load_jsonl_payloads(log_dir / f'{run_id}_goal_events.jsonl')
    events = [event_row(row) for row in payloads if row.get('event') in TERMINAL_EVENTS and row.get('goal_sequence') is not None]
    # Prefer terminal outcome rows over dispatch rows when multiple rows exist.
    outcome_by_seq: dict[int, dict[str, Any]] = {}
    dispatch_by_seq: dict[int, dict[str, Any]] = {}
    for row in events:
        seq = row.get('goal_sequence')
        if seq is None:
            continue
        seq = int(seq)
        if row.get('event') == 'dispatch':
            dispatch_by_seq[seq] = row
        else:
            outcome_by_seq[seq] = row
    outcomes = [outcome_by_seq[k] for k in sorted(outcome_by_seq)]
    final_mode = 'EXIT_REACHED' if any(row.get('classification') == 'exit_reached_terminal_success' for row in outcomes) else 'FAILED_OR_INCOMPLETE'
    return {
        'run_id': run_id,
        'final_mode_from_goal_events': final_mode,
        'goal_outcomes': outcomes,
        'dispatch_rows': [dispatch_by_seq[k] for k in sorted(dispatch_by_seq)],
        'exit_reached': final_mode == 'EXIT_REACHED',
        'last_outcomes': outcomes[-6:],
        'away_successes': [row for row in outcomes if row.get('event') == 'success' and row.get('classification') == 'successful_branch_moved_away_from_exit'],
        'exit_progress_successes': [row for row in outcomes if row.get('event') == 'success' and row.get('classification') in ('exit_progress_branch', 'target_inside_exit_radius')],
    }


def find_by_seq(rows: list[dict[str, Any]], seq: int) -> dict[str, Any] | None:
    for row in rows:
        if int(row.get('goal_sequence', -1)) == seq:
            return row
    return None


def aggregate_artifact_gaps(rows: list[dict[str, Any]]) -> dict[str, bool]:
    keys = [
        'chosen_branch_rank',
        'rejected_branch_summary',
        'candidate_branch_target_exit_distances',
        'candidate_branch_local_path_costs',
        'reverse_backtrack_context',
    ]
    return {key: any(row.get('artifact_gaps', {}).get(key) is True for row in rows) for key in keys}


def focus_analysis(runs: list[dict[str, Any]], focus_run: str, focus_timeout_seq: int) -> dict[str, Any]:
    focus_matches = [run for run in runs if run['run_id'] == focus_run]
    if not focus_matches:
        raise ValueError(f'focus run not found: {focus_run}')
    run = focus_matches[0]
    outcomes = run['goal_outcomes']
    timeout_row = find_by_seq(outcomes, focus_timeout_seq)
    pre_row = find_by_seq(outcomes, focus_timeout_seq - 1)
    post_row = find_by_seq(outcomes, focus_timeout_seq + 1)
    success_runs = [item for item in runs if item.get('exit_reached')]
    success_after_away = sum(
        1
        for item in success_runs
        if any((row.get('exit_progress_delta_m') or 0) < -0.25 for row in item.get('last_outcomes', []))
    )
    related_rows = [row for row in (pre_row, timeout_row, post_row) if row is not None]
    gaps = aggregate_artifact_gaps(related_rows)
    post_moved_away = post_row is not None and post_row.get('classification') == 'successful_branch_moved_away_from_exit'
    if post_moved_away and (gaps.get('chosen_branch_rank') or gaps.get('rejected_branch_summary')):
        diagnosis = 'post_timeout_branch_choice_moved_away_from_exit_but_candidate_ranking_missing'
    elif post_moved_away:
        diagnosis = 'post_timeout_branch_choice_moved_away_from_exit'
    else:
        diagnosis = 'post_timeout_branch_choice_not_explained_by_current_artifacts'
    return {
        'run_id': focus_run,
        'focus_timeout_seq': focus_timeout_seq,
        'pre_timeout_choice': pre_row,
        'timeout_choice': timeout_row,
        'post_timeout_choice': post_row,
        'comparison_to_success_runs': {
            'success_run_count': len(success_runs),
            'success_runs_reached_exit_after_away_branch_count': success_after_away,
            'success_run_ids': [item['run_id'] for item in success_runs],
        },
        'artifact_gaps': gaps,
        'diagnosis': diagnosis,
    }


def analyze(log_dir: Path, run_ids: list[str], focus_run: str | None, focus_timeout_seq: int | None) -> dict[str, Any]:
    runs = [read_run(log_dir, run_id) for run_id in run_ids]
    focus = None
    if focus_run and focus_timeout_seq is not None:
        focus = focus_analysis(runs, focus_run, focus_timeout_seq)
    return {
        'phase': '26D',
        'purpose': 'branch-choice snapshot alignment from existing goal_events artifacts',
        'success_definition': {
            'success_condition': 'robot_reaches_exit_coordinates',
            'full_map_exploration_required': False,
            'exit_radius_m': EXIT_RADIUS_M,
        },
        'run_ids': run_ids,
        'runs': runs,
        'focus_run_analysis': focus,
        'decision_guardrails': [
            'Do not judge success by full-maze exploration completeness.',
            'Do not add branch scoring until candidate branch ranking/rejection diagnostics exist.',
            'Use EXIT_REACHED / terminal_cancel result_reason=exit_reached as the success signal.',
        ],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, default=Path('log'))
    parser.add_argument('--runs', required=True)
    parser.add_argument('--focus-run')
    parser.add_argument('--focus-timeout-seq', type=int)
    parser.add_argument('--output-json', type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_ids = [item.strip() for item in args.runs.split(',') if item.strip()]
    payload = analyze(args.log_dir, run_ids, args.focus_run, args.focus_timeout_seq)
    text = json.dumps(payload, indent=2, sort_keys=True) + '\n'
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(text, encoding='utf-8')
    print(text)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
