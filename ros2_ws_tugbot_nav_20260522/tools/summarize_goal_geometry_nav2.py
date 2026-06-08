#!/usr/bin/env python3
"""Join Nav2/controller per-goal analysis with Phase 17 goal geometry fields.

Inputs:
- `analyze_goal_events_with_nav2_log.py --output-json` output.
- `/maze/goal_events` JSONL containing dispatch geometry fields.

Output:
- Per-goal joined rows preserving Nav2 counts and geometry fields.
- Success/timeout/terminal/failure geometry summaries for fast post-smoke comparison.
"""

from __future__ import annotations
import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any

NUMERIC_FIELDS = [
    'target_clearance_m',
    'line_of_sight_min_clearance_m',
    'path_corridor_min_clearance_m',
    'line_of_sight_occupied_count',
    'line_of_sight_unknown_count',
    'target_exit_dist',
    'elapsed_sec',
    'progress_failure_count',
    'recovery_count',
    'controller_abort_count',
]

BOOLEAN_FIELDS = [
    'target_near_wall',
    'target_crosses_wall_corner',
    'target_crosses_narrow_passage',
    'target_yaw_corridor_conflict',
    'caused_branch_failure',
    'caused_blacklist',
]

GEOMETRY_FIELDS = [
    'goal_kind',
    'local_topology',
    'branch_angle',
    'dispatch_pose',
    'target',
    'target_cell_occupancy',
    *NUMERIC_FIELDS[:6],
    *BOOLEAN_FIELDS[:4],
]


def load_dispatch_geometry(goal_events_path: Path) -> dict[int, dict[str, Any]]:
    geometry_by_seq: dict[int, dict[str, Any]] = {}
    geometry_keys = set(GEOMETRY_FIELDS)
    for line in goal_events_path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        payload = row.get('state', row)
        seq = payload.get('goal_sequence')
        if seq is None:
            continue
        seq = int(seq)
        event = payload.get('event')
        has_geometry = any(key in payload for key in geometry_keys)
        if event == 'dispatch' or (has_geometry and seq not in geometry_by_seq):
            geometry_by_seq[seq] = payload
    return geometry_by_seq


def load_nav2_goals(nav2_analysis_path: Path) -> list[dict[str, Any]]:
    data = json.loads(nav2_analysis_path.read_text(encoding='utf-8'))
    return list(data.get('goals', []))


def join_goals(nav2_goals: list[dict[str, Any]], dispatch_by_seq: dict[int, dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for goal in nav2_goals:
        seq = int(goal['goal_sequence'])
        dispatch = dispatch_by_seq.get(seq, {})
        row = dict(goal)
        for field in GEOMETRY_FIELDS:
            if field in dispatch:
                row[field] = dispatch.get(field)
        rows.append(row)
    return rows


def _numeric_summary(values: list[float]) -> dict[str, Any]:
    if not values:
        return {'count': 0, 'min': None, 'mean': None, 'max': None, 'sum': 0}
    return {
        'count': len(values),
        'min': min(values),
        'mean': sum(values) / len(values),
        'max': max(values),
        'sum': sum(values),
    }


def summarize_group(rows: list[dict[str, Any]]) -> dict[str, Any]:
    summary: dict[str, Any] = {'count': len(rows)}
    for field in NUMERIC_FIELDS:
        values = [float(row[field]) for row in rows if isinstance(row.get(field), (int, float))]
        summary[field] = _numeric_summary(values)
    for field in BOOLEAN_FIELDS:
        values = [row.get(field) for row in rows if isinstance(row.get(field), bool)]
        summary[field] = {'count': len(values), 'true_count': sum(1 for value in values if value), 'false_count': sum(1 for value in values if not value)}
    return summary


def summarize_by_outcome(rows: list[dict[str, Any]]) -> dict[str, Any]:
    outcomes = ['success', 'timeout', 'terminal_cancel', 'failure']
    summary = {outcome: summarize_group([row for row in rows if row.get('outcome') == outcome]) for outcome in outcomes}
    other_rows = [row for row in rows if row.get('outcome') not in outcomes]
    if other_rows:
        summary['other'] = summarize_group(other_rows)
    return summary


def print_summary_csv(summary: dict[str, Any]) -> None:
    fieldnames = [
        'outcome',
        'count',
        'target_clearance_mean',
        'path_corridor_min_clearance_mean',
        'line_of_sight_occupied_sum',
        'line_of_sight_unknown_sum',
        'progress_failure_sum',
        'recovery_sum',
        'controller_abort_sum',
        'narrow_passage_true_count',
        'wall_corner_true_count',
    ]
    writer = csv.DictWriter(sys.stdout, fieldnames=fieldnames)
    writer.writeheader()
    for outcome, group in summary.items():
        writer.writerow({
            'outcome': outcome,
            'count': group['count'],
            'target_clearance_mean': group['target_clearance_m']['mean'],
            'path_corridor_min_clearance_mean': group['path_corridor_min_clearance_m']['mean'],
            'line_of_sight_occupied_sum': group['line_of_sight_occupied_count']['sum'],
            'line_of_sight_unknown_sum': group['line_of_sight_unknown_count']['sum'],
            'progress_failure_sum': group['progress_failure_count']['sum'],
            'recovery_sum': group['recovery_count']['sum'],
            'controller_abort_sum': group['controller_abort_count']['sum'],
            'narrow_passage_true_count': group['target_crosses_narrow_passage']['true_count'],
            'wall_corner_true_count': group['target_crosses_wall_corner']['true_count'],
        })


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal-events', required=True, type=Path)
    parser.add_argument('--nav2-analysis', required=True, type=Path)
    parser.add_argument('--output-json', type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    joined = join_goals(load_nav2_goals(args.nav2_analysis), load_dispatch_geometry(args.goal_events))
    summary = summarize_by_outcome(joined)
    data = {'summary': summary, 'joined_goals': joined}
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print_summary_csv(summary)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
