#!/usr/bin/env python3
"""Analyze controller/local-cost details in failure windows.

Phase 23A analysis-only tool. It consumes existing Phase 21/22 artifacts and
summarizes the last-window cmd_vel distribution, local-cost outcome fields,
and Nav2 timing around timeout/failure goals.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

NEAR_ZERO_CMD_LINEAR = 0.03
NEAR_ZERO_CMD_ANGULAR = 0.05
FAILURE_WINDOW_NEAR_ZERO_CMD_LINEAR = 0.01
FAILURE_WINDOW_NEAR_ZERO_CMD_ANGULAR = 0.05
HIGH_TIMEOUT_ROBOT_COST = 90
LAST_WINDOW_SEC = 10.0


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        payload = row.get('state', row)
        if 'wall_time' not in payload and 'wall_time' in row:
            payload = dict(payload)
            payload['wall_time'] = row['wall_time']
        rows.append(payload)
    return rows


def by_sequence(rows: list[dict[str, Any]]) -> dict[int, dict[str, Any]]:
    return {int(row['goal_sequence']): row for row in rows if row.get('goal_sequence') is not None}


def load_goal_rows(path: Path) -> list[dict[str, Any]]:
    data = read_json(path)
    return data.get('goals', [])


def split_cmd_samples(samples: list[dict[str, Any]]) -> list[dict[str, Any]]:
    cmd: list[dict[str, Any]] = []
    for sample in samples:
        source = sample.get('source')
        if source in ('cmd_vel', 'cmd_vel_nav', 'cmd') or 'linear_x' in sample or 'angular_z' in sample:
            cmd.append(sample)
    return sorted(cmd, key=lambda row: float(row.get('wall_time', 0.0)))


def samples_between(samples: list[dict[str, Any]], start: float, end: float) -> list[dict[str, Any]]:
    return [row for row in samples if start <= float(row.get('wall_time', -math.inf)) <= end]


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    sorted_values = sorted(values)
    if len(sorted_values) == 1:
        return sorted_values[0]
    # nearest-rank keeps p95 at max for small diagnostic windows, matching
    # operator intuition and the Phase 23 contract.
    rank = max(1, math.ceil(q * len(sorted_values)))
    return sorted_values[min(rank - 1, len(sorted_values) - 1)]


def rounded(value: float | None) -> float | None:
    return round(value, 6) if value is not None else None


def cmd_distribution(cmd_samples: list[dict[str, Any]]) -> dict[str, Any]:
    linear = [abs(float(row.get('linear_x', 0.0))) for row in cmd_samples]
    angular = [abs(float(row.get('angular_z', 0.0))) for row in cmd_samples]
    if not cmd_samples:
        return {
            'cmd_sample_count': 0,
            'cmd_linear_abs_min': None,
            'cmd_linear_abs_mean': None,
            'cmd_linear_abs_max': None,
            'cmd_linear_abs_p95': None,
            'cmd_angular_abs_min': None,
            'cmd_angular_abs_mean': None,
            'cmd_angular_abs_max': None,
            'cmd_angular_abs_p95': None,
            'last_nonzero_cmd_time': None,
            'near_zero_cmd_start_time': None,
            'near_zero_cmd_duration_sec': None,
        }
    nonzero_times = [
        float(row.get('wall_time', 0.0)) for row in cmd_samples
        if abs(float(row.get('linear_x', 0.0))) > FAILURE_WINDOW_NEAR_ZERO_CMD_LINEAR
        or abs(float(row.get('angular_z', 0.0))) > FAILURE_WINDOW_NEAR_ZERO_CMD_ANGULAR
    ]
    near_zero_start = None
    # Find the start of the final contiguous near-zero segment.
    for row in reversed(cmd_samples):
        is_near_zero = (
            abs(float(row.get('linear_x', 0.0))) <= FAILURE_WINDOW_NEAR_ZERO_CMD_LINEAR
            and abs(float(row.get('angular_z', 0.0))) <= FAILURE_WINDOW_NEAR_ZERO_CMD_ANGULAR
        )
        if is_near_zero:
            near_zero_start = float(row.get('wall_time', 0.0))
        else:
            break
    window_end = float(cmd_samples[-1].get('wall_time', 0.0))
    return {
        'cmd_sample_count': len(cmd_samples),
        'cmd_linear_abs_min': rounded(min(linear)),
        'cmd_linear_abs_mean': rounded(sum(linear) / len(linear)),
        'cmd_linear_abs_max': rounded(max(linear)),
        'cmd_linear_abs_p95': rounded(percentile(linear, 0.95)),
        'cmd_angular_abs_min': rounded(min(angular)),
        'cmd_angular_abs_mean': rounded(sum(angular) / len(angular)),
        'cmd_angular_abs_max': rounded(max(angular)),
        'cmd_angular_abs_p95': rounded(percentile(angular, 0.95)),
        'last_nonzero_cmd_time': rounded(max(nonzero_times) if nonzero_times else None),
        'near_zero_cmd_start_time': rounded(near_zero_start),
        'near_zero_cmd_duration_sec': rounded(window_end - near_zero_start if near_zero_start is not None else None),
    }


def first(values: list[Any]) -> Any:
    return values[0] if values else None


def last(values: list[Any]) -> Any:
    return values[-1] if values else None


def timing_fields(nav_goal: dict[str, Any], near_zero_start: float | None) -> dict[str, Any]:
    progress_times = [float(v) for v in nav_goal.get('progress_failure_times', [])]
    clear_times = [float(v) for v in nav_goal.get('clear_costmap_times', [])]
    abort_times = [float(v) for v in nav_goal.get('controller_abort_times', [])]
    first_progress = first(progress_times)
    return {
        'progress_failure_count': nav_goal.get('progress_failure_count', 0),
        'recovery_count': nav_goal.get('recovery_count', 0),
        'controller_abort_count': nav_goal.get('controller_abort_count', 0),
        'first_progress_failure_time': rounded(first_progress),
        'last_progress_failure_time': rounded(last(progress_times)),
        'first_clear_costmap_time': rounded(first(clear_times)),
        'last_clear_costmap_time': rounded(last(clear_times)),
        'first_controller_abort_time': rounded(first(abort_times)),
        'last_controller_abort_time': rounded(last(abort_times)),
        'near_zero_cmd_to_first_progress_failure_sec': rounded(near_zero_start - first_progress if first_progress is not None and near_zero_start is not None else None),
    }


def analyze_failure_windows(controller_rows: list[dict[str, Any]], cmd_samples: list[dict[str, Any]], local_cost: dict[int, dict[str, Any]], nav2: dict[int, dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for goal in controller_rows:
        if goal.get('outcome') not in ('timeout', 'failure'):
            continue
        seq = int(goal['goal_sequence'])
        end = float(goal.get('end_time'))
        start = max(float(goal.get('start_time', end - LAST_WINDOW_SEC)), end - LAST_WINDOW_SEC)
        window_cmd = samples_between(cmd_samples, start, end)
        cmd_stats = cmd_distribution(window_cmd)
        cost = local_cost.get(seq, {})
        nav_goal = nav2.get(seq, {})
        row = {
            'goal_sequence': seq,
            'outcome': goal.get('outcome'),
            'classification': goal.get('classification'),
            'late_controller_silent': bool(goal.get('late_controller_silent')),
            'healthy_motion_but_late_stall': bool(goal.get('healthy_motion_but_late_stall')),
            'failure_window_start_time': rounded(start),
            'failure_window_end_time': rounded(end),
            'last_10s_odom_distance_m': goal.get('last_10s_odom_distance_m'),
            'last_10s_cmd_linear_abs_mean': goal.get('last_10s_cmd_linear_abs_mean'),
            'last_10s_cmd_angular_abs_mean': goal.get('last_10s_cmd_angular_abs_mean'),
            **cmd_stats,
            'dispatch_path_local_cost_max': cost.get('dispatch_path_local_cost_max'),
            'timeout_robot_local_cost_max': cost.get('timeout_robot_local_cost_max'),
            'timeout_robot_local_cost_mean': cost.get('timeout_robot_local_cost_mean'),
            'timeout_robot_obstacle_cluster_count': cost.get('timeout_robot_obstacle_cluster_count'),
            'footprint_corridor_inflation_squeezed': bool(cost.get('footprint_corridor_inflation_squeezed')),
            'timeout_robot_in_local_costmap_bounds': cost.get('timeout_robot_in_local_costmap_bounds'),
            'timeout_footprint_cost_max': cost.get('timeout_footprint_cost_max'),
            'timeout_footprint_cost_mean': cost.get('timeout_footprint_cost_mean'),
            'timeout_footprint_cost_p95': cost.get('timeout_footprint_cost_p95'),
            'timeout_footprint_inflated_cell_count': cost.get('timeout_footprint_inflated_cell_count'),
            'timeout_footprint_lethal_cell_count': cost.get('timeout_footprint_lethal_cell_count'),
            'timeout_front_wedge_cost_max': cost.get('timeout_front_wedge_cost_max'),
            'timeout_front_wedge_cost_mean': cost.get('timeout_front_wedge_cost_mean'),
            'timeout_front_wedge_clearance_m': cost.get('timeout_front_wedge_clearance_m'),
            'timeout_left_side_cost_max': cost.get('timeout_left_side_cost_max'),
            'timeout_left_side_clearance_m': cost.get('timeout_left_side_clearance_m'),
            'timeout_right_side_cost_max': cost.get('timeout_right_side_cost_max'),
            'timeout_right_side_clearance_m': cost.get('timeout_right_side_clearance_m'),
            'timeout_path_ahead_0_5m_cost_max': cost.get('timeout_path_ahead_0_5m_cost_max'),
            'timeout_path_ahead_0_5m_cost_mean': cost.get('timeout_path_ahead_0_5m_cost_mean'),
            'timeout_path_ahead_1_0m_cost_max': cost.get('timeout_path_ahead_1_0m_cost_max'),
            'timeout_path_ahead_1_0m_cost_mean': cost.get('timeout_path_ahead_1_0m_cost_mean'),
            'high_timeout_robot_cost': bool((cost.get('timeout_robot_local_cost_max') or 0) >= HIGH_TIMEOUT_ROBOT_COST),
            'available_diagnostics': {
                'footprint_cost_stats': all(k in cost for k in ('timeout_footprint_cost_max', 'timeout_footprint_cost_mean')),
                'front_side_clearance': all(k in cost for k in ('timeout_front_wedge_clearance_m', 'timeout_left_side_clearance_m', 'timeout_right_side_clearance_m')),
                'path_ahead_cost': all(k in cost for k in ('timeout_path_ahead_0_5m_cost_max', 'timeout_path_ahead_1_0m_cost_max')),
                'nav2_event_times': any(k in nav_goal for k in ('progress_failure_times', 'clear_costmap_times', 'controller_abort_times')),
            },
        }
        row.update(timing_fields(nav_goal, row.get('near_zero_cmd_start_time')))
        rows.append(row)
    return rows


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        'failure_window_count': len(rows),
        'late_controller_silent_count': sum(1 for row in rows if row.get('late_controller_silent')),
        'healthy_motion_but_late_stall_count': sum(1 for row in rows if row.get('healthy_motion_but_late_stall')),
        'squeezed_count': sum(1 for row in rows if row.get('footprint_corridor_inflation_squeezed')),
        'high_timeout_robot_cost_count': sum(1 for row in rows if row.get('high_timeout_robot_cost')),
        'with_nav2_event_times_count': sum(1 for row in rows if row.get('available_diagnostics', {}).get('nav2_event_times')),
        'with_footprint_cost_stats_count': sum(1 for row in rows if row.get('available_diagnostics', {}).get('footprint_cost_stats')),
        'with_path_ahead_cost_count': sum(1 for row in rows if row.get('available_diagnostics', {}).get('path_ahead_cost')),
    }


def write_csv(rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        'goal_sequence', 'classification', 'failure_window_start_time', 'failure_window_end_time',
        'cmd_linear_abs_min', 'cmd_linear_abs_mean', 'cmd_linear_abs_max', 'cmd_linear_abs_p95',
        'last_nonzero_cmd_time', 'near_zero_cmd_start_time', 'near_zero_cmd_duration_sec',
        'timeout_robot_local_cost_max', 'footprint_corridor_inflation_squeezed',
        'first_progress_failure_time', 'last_controller_abort_time',
    ]
    writer = csv.DictWriter(sys.stdout, fieldnames=fieldnames, extrasaction='ignore')
    writer.writeheader()
    for row in rows:
        writer.writerow(row)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller-analysis', type=Path, required=True)
    parser.add_argument('--controller-dynamics', type=Path, required=True)
    parser.add_argument('--local-cost-summary', type=Path, required=True)
    parser.add_argument('--nav2-analysis', type=Path, required=True)
    parser.add_argument('--output-json', type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    controller_rows = load_goal_rows(args.controller_analysis)
    cmd_samples = split_cmd_samples(read_jsonl(args.controller_dynamics))
    local_cost = by_sequence(load_goal_rows(args.local_cost_summary))
    nav2 = by_sequence(load_goal_rows(args.nav2_analysis))
    rows = analyze_failure_windows(controller_rows, cmd_samples, local_cost, nav2)
    payload = {'summary': summarize(rows), 'failure_windows': rows}
    write_csv(rows)
    if args.output_json:
        args.output_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    sys.exit(main())
