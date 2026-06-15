#!/usr/bin/env python3
"""Classify timeout failure windows into Phase 24A subtypes.

This is analysis-only. It consumes Phase 23B failure-window JSON and standardizes
late-silent timeout subtypes before any navigation intervention.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any

FOOTPRINT_BLOCKED_COST = 90
FOOTPRINT_INFLATED_COST = 70
PATH_BLOCKED_COST = 90
FRONT_BLOCKED_COST = 90
SIDE_HIGH_COST = 90
ROBOT_HIGH_COST = 90


def load_failure_windows(path: Path) -> list[dict[str, Any]]:
    data = json.loads(path.read_text(encoding='utf-8'))
    rows = data.get('failure_windows', data if isinstance(data, list) else [])
    if not isinstance(rows, list):
        raise ValueError('failure-window input must contain a failure_windows list')
    return [row for row in rows if isinstance(row, dict)]


def number(row: dict[str, Any], key: str, default: float = 0.0) -> float:
    value = row.get(key)
    if isinstance(value, bool):
        return float(int(value))
    if isinstance(value, (int, float)):
        return float(value)
    return float(default)


def is_timeout_late_silent(row: dict[str, Any]) -> bool:
    if row.get('outcome') != 'timeout':
        return False
    return row.get('classification') == 'healthy_motion_but_late_stall' or bool(row.get('late_controller_silent'))


def footprint_path_blocked(row: dict[str, Any]) -> bool:
    footprint_max = number(row, 'timeout_footprint_cost_max')
    footprint_p95 = number(row, 'timeout_footprint_cost_p95')
    footprint_lethal = number(row, 'timeout_footprint_lethal_cell_count')
    footprint_inflated = number(row, 'timeout_footprint_inflated_cell_count')
    front_max = number(row, 'timeout_front_wedge_cost_max')
    path_05 = number(row, 'timeout_path_ahead_0_5m_cost_max')
    path_10 = number(row, 'timeout_path_ahead_1_0m_cost_max')

    footprint_severe = (
        footprint_max >= FOOTPRINT_BLOCKED_COST
        and (footprint_lethal > 0 or footprint_p95 >= FOOTPRINT_BLOCKED_COST or footprint_inflated >= 10)
    )
    path_severe = path_05 >= PATH_BLOCKED_COST or path_10 >= PATH_BLOCKED_COST
    front_severe = front_max >= FRONT_BLOCKED_COST
    return bool(footprint_severe and (path_severe or front_severe))


def side_or_timing(row: dict[str, Any]) -> bool:
    robot_high = number(row, 'timeout_robot_local_cost_max') >= ROBOT_HIGH_COST
    left_high = number(row, 'timeout_left_side_cost_max') >= SIDE_HIGH_COST
    right_high = number(row, 'timeout_right_side_cost_max') >= SIDE_HIGH_COST
    squeezed = bool(row.get('footprint_corridor_inflation_squeezed'))
    return bool(robot_high or left_high or right_high or squeezed)


def classify_controller_subtype(row: dict[str, Any]) -> str:
    if footprint_path_blocked(row):
        return 'footprint_path_blocked_late_silent'
    if side_or_timing(row):
        return 'side_cost_or_timing_late_silent'
    return 'unclassified_late_silent'


def classify_timing_subtype(row: dict[str, Any]) -> str:
    delta = row.get('near_zero_cmd_to_first_progress_failure_sec')
    if isinstance(delta, (int, float)):
        if float(delta) < 0.0:
            return 'cmd_silent_before_progress_failure'
        return 'cmd_silent_after_recovery_abort'
    near_zero = row.get('near_zero_cmd_start_time')
    first_progress = row.get('first_progress_failure_time')
    if isinstance(near_zero, (int, float)) and isinstance(first_progress, (int, float)):
        if float(near_zero) < float(first_progress):
            return 'cmd_silent_before_progress_failure'
        return 'cmd_silent_after_recovery_abort'
    return 'cmd_silent_timing_unknown'


def classify_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    classified: list[dict[str, Any]] = []
    for row in rows:
        if not is_timeout_late_silent(row):
            continue
        controller_subtype = classify_controller_subtype(row)
        timing_subtype = classify_timing_subtype(row)
        reasons = {
            'footprint_path_blocked': footprint_path_blocked(row),
            'side_or_timing_signal': side_or_timing(row),
            'footprint_cost_max': row.get('timeout_footprint_cost_max'),
            'footprint_lethal_cell_count': row.get('timeout_footprint_lethal_cell_count'),
            'front_wedge_cost_max': row.get('timeout_front_wedge_cost_max'),
            'path_ahead_0_5m_cost_max': row.get('timeout_path_ahead_0_5m_cost_max'),
            'path_ahead_1_0m_cost_max': row.get('timeout_path_ahead_1_0m_cost_max'),
            'left_side_cost_max': row.get('timeout_left_side_cost_max'),
            'right_side_cost_max': row.get('timeout_right_side_cost_max'),
            'near_zero_cmd_to_first_progress_failure_sec': row.get('near_zero_cmd_to_first_progress_failure_sec'),
        }
        classified.append({
            'goal_sequence': row.get('goal_sequence'),
            'outcome': row.get('outcome'),
            'classification': row.get('classification'),
            'controller_subtype': controller_subtype,
            'timing_subtype': timing_subtype,
            'combined_subtype': f'{controller_subtype}+{timing_subtype}',
            'late_controller_silent': bool(row.get('late_controller_silent')),
            'cmd_linear_abs_mean': row.get('cmd_linear_abs_mean'),
            'cmd_linear_abs_max': row.get('cmd_linear_abs_max'),
            'near_zero_cmd_duration_sec': row.get('near_zero_cmd_duration_sec'),
            'reasons': reasons,
        })
    return classified


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    timeout_count = len(rows)
    controller_counts = Counter(str(row['controller_subtype']) for row in rows)
    timing_counts = Counter(str(row['timing_subtype']) for row in rows)
    combined_counts = Counter(str(row['combined_subtype']) for row in rows)
    severe = controller_counts.get('footprint_path_blocked_late_silent', 0)
    return {
        'timeout_count': timeout_count,
        'controller_subtype_counts': dict(sorted(controller_counts.items())),
        'timing_subtype_counts': dict(sorted(timing_counts.items())),
        'combined_subtype_counts': dict(sorted(combined_counts.items())),
        'severe_footprint_path_count': severe,
        'severe_footprint_path_ratio': round(severe / timeout_count, 6) if timeout_count else None,
        'side_or_timing_count': controller_counts.get('side_cost_or_timing_late_silent', 0),
        'unclassified_count': controller_counts.get('unclassified_late_silent', 0),
    }


def write_csv(rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        'goal_sequence',
        'outcome',
        'controller_subtype',
        'timing_subtype',
        'combined_subtype',
        'classification',
        'cmd_linear_abs_mean',
        'cmd_linear_abs_max',
        'near_zero_cmd_duration_sec',
    ]
    writer = csv.DictWriter(sys.stdout, fieldnames=fieldnames, extrasaction='ignore')
    writer.writeheader()
    for row in rows:
        writer.writerow(row)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--failure-windows', type=Path, required=True)
    parser.add_argument('--output-json', type=Path)
    args = parser.parse_args()

    classified = classify_rows(load_failure_windows(args.failure_windows))
    data = {'summary': summarize(classified), 'timeout_subtypes': classified}
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    write_csv(classified)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
