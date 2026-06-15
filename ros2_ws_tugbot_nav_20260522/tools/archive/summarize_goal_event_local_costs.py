#!/usr/bin/env python3
"""Summarize local-cost fields embedded in `/maze/goal_events` JSONL."""

from __future__ import annotations
import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any

COST_FIELDS = [
    'dispatch_local_cost_sample_age_sec',
    'dispatch_target_in_local_costmap_bounds',
    'dispatch_robot_in_local_costmap_bounds',
    'dispatch_path_local_cost_sample_count',
    'dispatch_path_local_cost_in_bounds_sample_count',
    'dispatch_local_cost_sample_coverage_ratio',
    'dispatch_target_local_cost',
    'dispatch_target_local_cost_max_radius',
    'dispatch_path_local_cost_max',
    'dispatch_path_local_cost_mean',
    'timeout_local_cost_sample_age_sec',
    'timeout_robot_in_local_costmap_bounds',
    'timeout_robot_local_cost_max',
    'timeout_robot_local_cost_mean',
    'timeout_robot_obstacle_cluster_count',
    'footprint_corridor_inflation_squeezed',
    'timeout_footprint_cost_max',
    'timeout_footprint_cost_mean',
    'timeout_footprint_cost_p95',
    'timeout_footprint_inflated_cell_count',
    'timeout_footprint_lethal_cell_count',
    'timeout_front_wedge_cost_max',
    'timeout_front_wedge_cost_mean',
    'timeout_front_wedge_clearance_m',
    'timeout_left_side_cost_max',
    'timeout_left_side_clearance_m',
    'timeout_right_side_cost_max',
    'timeout_right_side_clearance_m',
    'timeout_path_ahead_0_5m_cost_max',
    'timeout_path_ahead_0_5m_cost_mean',
    'timeout_path_ahead_1_0m_cost_max',
    'timeout_path_ahead_1_0m_cost_mean',
]


def load_goal_cost_rows(path: Path) -> list[dict[str, Any]]:
    by_seq: dict[int, dict[str, Any]] = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        payload = row.get('state', row)
        seq = payload.get('goal_sequence')
        if seq is None:
            continue
        rec = by_seq.setdefault(int(seq), {'goal_sequence': int(seq), 'events': []})
        event = payload.get('event')
        rec['events'].append(event)
        if event == 'dispatch' or rec.get('dispatch_event_seen') is None:
            for key in COST_FIELDS:
                if key.startswith('dispatch_') and key in payload:
                    rec[key] = payload.get(key)
            rec['dispatch_event_seen'] = event == 'dispatch'
        if event in ('success', 'timeout', 'failure', 'terminal_cancel'):
            rec['outcome'] = event
            for key in COST_FIELDS:
                if key in payload:
                    rec[key] = payload.get(key)
    return [rec for _, rec in sorted(by_seq.items())]


def _numbers(rows: list[dict[str, Any]], key: str) -> list[float]:
    return [float(row[key]) for row in rows if isinstance(row.get(key), (int, float))]


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    by_outcome: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        by_outcome.setdefault(str(row.get('outcome', 'unknown')), []).append(row)
    outcome_summary = {}
    for outcome, outcome_rows in sorted(by_outcome.items()):
        coverage = _numbers(outcome_rows, 'dispatch_local_cost_sample_coverage_ratio')
        dispatch_age = _numbers(outcome_rows, 'dispatch_local_cost_sample_age_sec')
        timeout_age = _numbers(outcome_rows, 'timeout_local_cost_sample_age_sec')
        outcome_summary[outcome] = {
            'count': len(outcome_rows),
            'dispatch_coverage_mean': _mean(coverage),
            'dispatch_coverage_min': min(coverage) if coverage else None,
            'dispatch_sample_age_mean_sec': _mean(dispatch_age),
            'timeout_sample_age_mean_sec': _mean(timeout_age),
            'target_in_bounds_count': sum(1 for row in outcome_rows if row.get('dispatch_target_in_local_costmap_bounds') is True),
            'robot_in_bounds_count': sum(1 for row in outcome_rows if row.get('dispatch_robot_in_local_costmap_bounds') is True),
            'timeout_robot_in_bounds_count': sum(1 for row in outcome_rows if row.get('timeout_robot_in_local_costmap_bounds') is True),
            'dispatch_target_cost_mean': _mean(_numbers(outcome_rows, 'dispatch_target_local_cost')),
            'dispatch_path_cost_max_mean': _mean(_numbers(outcome_rows, 'dispatch_path_local_cost_max')),
            'timeout_robot_cost_max_mean': _mean(_numbers(outcome_rows, 'timeout_robot_local_cost_max')),
            'timeout_footprint_cost_max_mean': _mean(_numbers(outcome_rows, 'timeout_footprint_cost_max')),
            'timeout_front_wedge_cost_max_mean': _mean(_numbers(outcome_rows, 'timeout_front_wedge_cost_max')),
            'timeout_path_ahead_0_5m_cost_max_mean': _mean(_numbers(outcome_rows, 'timeout_path_ahead_0_5m_cost_max')),
            'timeout_path_ahead_1_0m_cost_max_mean': _mean(_numbers(outcome_rows, 'timeout_path_ahead_1_0m_cost_max')),
            'timeout_footprint_inflated_cell_sum': sum(int(row.get('timeout_footprint_inflated_cell_count') or 0) for row in outcome_rows),
            'timeout_footprint_lethal_cell_sum': sum(int(row.get('timeout_footprint_lethal_cell_count') or 0) for row in outcome_rows),
            'timeout_robot_obstacle_cluster_sum': sum(int(row.get('timeout_robot_obstacle_cluster_count') or 0) for row in outcome_rows),
            'squeezed_count': sum(1 for row in outcome_rows if row.get('footprint_corridor_inflation_squeezed') is True),
        }
    return {'goal_count': len(rows), 'outcomes': outcome_summary}


def write_csv(rows: list[dict[str, Any]]) -> None:
    fieldnames = ['goal_sequence', 'outcome'] + COST_FIELDS
    writer = csv.DictWriter(sys.stdout, fieldnames=fieldnames, extrasaction='ignore')
    writer.writeheader()
    for row in rows:
        writer.writerow(row)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal-events', type=Path, required=True)
    parser.add_argument('--output-json', type=Path)
    args = parser.parse_args()
    rows = load_goal_cost_rows(args.goal_events)
    data = {'summary': summarize(rows), 'goals': rows}
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    write_csv(rows)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
