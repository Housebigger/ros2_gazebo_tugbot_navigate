#!/usr/bin/env python3
"""Analyze post-recovery path/local-cost alignment for late-silent failures.

Phase 24B analysis-only tool. It combines existing launch-log path update lines,
Nav2 recovery/abort timestamps, controller dynamics cmd samples, and Phase 23B
local-cost timeout snapshots.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from pathlib import Path
from typing import Any

PATH_UPDATE_PATTERNS = ('Passing new path to controller',)
LOG_TIME_RE = re.compile(r'\[(\d+(?:\.\d+)?)\]')
NEAR_ZERO_CMD_LINEAR = 0.01
NEAR_ZERO_CMD_ANGULAR = 0.05


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists():
        return rows
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def parse_log_time(line: str) -> float | None:
    matches = LOG_TIME_RE.findall(line)
    if not matches:
        return None
    return float(matches[0])


def load_path_update_times(path: Path) -> list[float]:
    if not path.exists():
        return []
    times: list[float] = []
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if any(pattern in line for pattern in PATH_UPDATE_PATTERNS):
            timestamp = parse_log_time(line)
            if timestamp is not None:
                times.append(timestamp)
    return sorted(times)


def cmd_samples(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    samples = []
    for row in rows:
        if row.get('source') in ('cmd_vel', 'cmd_vel_nav', 'cmd') or 'linear_x' in row or 'angular_z' in row:
            samples.append(row)
    return sorted(samples, key=lambda row: float(row.get('wall_time', 0.0)))


def is_near_zero_cmd(row: dict[str, Any]) -> bool:
    return (
        abs(float(row.get('linear_x', 0.0))) <= NEAR_ZERO_CMD_LINEAR
        and abs(float(row.get('angular_z', 0.0))) <= NEAR_ZERO_CMD_ANGULAR
    )


def rows_between(rows: list[dict[str, Any]], start: float | None, end: float | None) -> list[dict[str, Any]]:
    if start is None or end is None:
        return []
    return [row for row in rows if start <= float(row.get('wall_time', 0.0)) <= end]


def times_between(times: list[float], start: float | None, end: float | None) -> list[float]:
    if start is None or end is None:
        return []
    return [time for time in times if start <= time <= end]


def by_goal_sequence(rows: list[dict[str, Any]]) -> dict[int, dict[str, Any]]:
    out: dict[int, dict[str, Any]] = {}
    for row in rows:
        seq = row.get('goal_sequence')
        if seq is not None:
            out[int(seq)] = row
    return out


def rounded(value: float | None) -> float | None:
    return round(value, 6) if isinstance(value, (int, float)) else None


def analyze(
    failure_windows: list[dict[str, Any]],
    nav2_goals: dict[int, dict[str, Any]],
    cmd: list[dict[str, Any]],
    path_update_times: list[float],
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for fw in failure_windows:
        seq_raw = fw.get('goal_sequence')
        if seq_raw is None:
            continue
        seq = int(seq_raw)
        nav = nav2_goals.get(seq, {})
        clear_times = [float(v) for v in nav.get('clear_costmap_times', []) if isinstance(v, (int, float))]
        recovery_time = max(clear_times) if clear_times else None
        near_zero_start = fw.get('near_zero_cmd_start_time')
        if isinstance(near_zero_start, (int, float)):
            near_zero_start_f = float(near_zero_start)
        else:
            near_zero_start_f = None
        window_start = float(fw.get('failure_window_start_time', near_zero_start_f or 0.0))
        window_end = float(fw.get('failure_window_end_time', near_zero_start_f or window_start))
        post_recovery_end = near_zero_start_f if near_zero_start_f is not None else window_end
        updates_after_recovery = times_between(path_update_times, recovery_time, window_end)
        updates_before_near_zero = times_between(path_update_times, recovery_time, post_recovery_end)
        updates_while_near_zero = times_between(path_update_times, near_zero_start_f, window_end)
        cmd_after_recovery = rows_between(cmd, recovery_time, window_end)
        near_zero_after_recovery = [sample for sample in cmd_after_recovery if is_near_zero_cmd(sample)]
        path_update_to_near_zero = None
        if updates_before_near_zero and near_zero_start_f is not None:
            path_update_to_near_zero = near_zero_start_f - updates_before_near_zero[-1]
        controller_received_path_but_cmd_near_zero = bool(updates_while_near_zero and near_zero_after_recovery)
        rows.append({
            'goal_sequence': seq,
            'outcome': fw.get('outcome'),
            'classification': fw.get('classification'),
            'last_recovery_time': recovery_time,
            'near_zero_cmd_start_time': near_zero_start_f,
            'failure_window_start_time': fw.get('failure_window_start_time'),
            'failure_window_end_time': fw.get('failure_window_end_time'),
            'near_zero_after_last_recovery_sec': rounded(near_zero_start_f - recovery_time if recovery_time is not None and near_zero_start_f is not None else None),
            'path_updates_after_recovery': len(updates_after_recovery),
            'path_updates_after_recovery_before_near_zero': len(updates_before_near_zero),
            'path_updates_while_cmd_near_zero': len(updates_while_near_zero),
            'first_path_update_after_recovery_time': updates_after_recovery[0] if updates_after_recovery else None,
            'last_path_update_after_recovery_time': updates_after_recovery[-1] if updates_after_recovery else None,
            'last_path_update_before_near_zero_time': updates_before_near_zero[-1] if updates_before_near_zero else None,
            'path_update_to_near_zero_sec': rounded(path_update_to_near_zero),
            'cmd_samples_after_recovery': len(cmd_after_recovery),
            'near_zero_cmd_samples_after_recovery': len(near_zero_after_recovery),
            'controller_received_path_but_cmd_near_zero': controller_received_path_but_cmd_near_zero,
            'timeout_footprint_cost_max': fw.get('timeout_footprint_cost_max'),
            'timeout_front_wedge_cost_max': fw.get('timeout_front_wedge_cost_max'),
            'timeout_left_side_cost_max': fw.get('timeout_left_side_cost_max'),
            'timeout_right_side_cost_max': fw.get('timeout_right_side_cost_max'),
            'timeout_path_ahead_0_5m_cost_max': fw.get('timeout_path_ahead_0_5m_cost_max'),
            'timeout_path_ahead_1_0m_cost_max': fw.get('timeout_path_ahead_1_0m_cost_max'),
            'available_diagnostics': {
                'path_update_times': bool(path_update_times),
                'recovery_times': bool(clear_times),
                'cmd_samples': bool(cmd),
                'local_cost_timeout_snapshot': any(
                    fw.get(key) is not None
                    for key in ('timeout_footprint_cost_max', 'timeout_path_ahead_0_5m_cost_max', 'timeout_path_ahead_1_0m_cost_max')
                ),
                'pre_post_recovery_local_cost_snapshots': False,
                'robot_to_active_path_distance': False,
            },
        })
    return rows


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        'row_count': len(rows),
        'with_recovery_time_count': sum(1 for row in rows if row.get('last_recovery_time') is not None),
        'with_path_updates_after_recovery_count': sum(1 for row in rows if int(row.get('path_updates_after_recovery') or 0) > 0),
        'with_path_updates_before_near_zero_count': sum(1 for row in rows if int(row.get('path_updates_after_recovery_before_near_zero') or 0) > 0),
        'controller_received_path_but_cmd_near_zero_count': sum(1 for row in rows if row.get('controller_received_path_but_cmd_near_zero') is True),
        'with_local_cost_timeout_snapshot_count': sum(1 for row in rows if row.get('available_diagnostics', {}).get('local_cost_timeout_snapshot') is True),
        'with_pre_post_recovery_local_cost_snapshots_count': sum(1 for row in rows if row.get('available_diagnostics', {}).get('pre_post_recovery_local_cost_snapshots') is True),
        'with_robot_to_active_path_distance_count': sum(1 for row in rows if row.get('available_diagnostics', {}).get('robot_to_active_path_distance') is True),
    }


def write_csv(rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        'goal_sequence',
        'outcome',
        'last_recovery_time',
        'path_updates_after_recovery',
        'path_updates_after_recovery_before_near_zero',
        'path_updates_while_cmd_near_zero',
        'near_zero_after_last_recovery_sec',
        'path_update_to_near_zero_sec',
        'near_zero_cmd_samples_after_recovery',
        'controller_received_path_but_cmd_near_zero',
    ]
    writer = csv.DictWriter(sys.stdout, fieldnames=fieldnames, extrasaction='ignore')
    writer.writeheader()
    for row in rows:
        writer.writerow(row)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--failure-windows', type=Path, required=True)
    parser.add_argument('--nav2-analysis', type=Path, required=True)
    parser.add_argument('--controller-dynamics', type=Path, required=True)
    parser.add_argument('--launch-log', type=Path, required=True)
    parser.add_argument('--output-json', type=Path)
    args = parser.parse_args()

    failure_data = load_json(args.failure_windows)
    nav2_data = load_json(args.nav2_analysis)
    failure_windows = failure_data.get('failure_windows', [])
    nav2_goals = by_goal_sequence(nav2_data.get('goals', []))
    cmd = cmd_samples(load_jsonl(args.controller_dynamics))
    path_update_times = load_path_update_times(args.launch_log)
    rows = analyze(failure_windows, nav2_goals, cmd, path_update_times)
    data = {'summary': summarize(rows), 'post_recovery_alignment': rows}
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    write_csv(rows)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
