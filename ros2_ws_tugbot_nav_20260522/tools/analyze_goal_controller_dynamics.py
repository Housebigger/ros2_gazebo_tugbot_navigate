#!/usr/bin/env python3
"""Analyze controller dynamics over maze goal intervals.

Phase 21A analysis-only tool. It joins `/maze/goal_events` intervals with
controller dynamics samples captured from odom and cmd_vel/cmd_vel_nav and
classifies each goal as healthy_motion, stuck_with_cmd, oscillation_candidate,
controller_silent, slow_progress, or insufficient_data.
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
NEAR_ZERO_ODOM_STEP = 0.02
STUCK_ODOM_DISTANCE_M = 0.10
HEALTHY_ODOM_DISTANCE_M = 0.50
MEAN_CMD_ACTIVE = 0.10
OSCILLATION_YAW_ABS_SUM_RAD = 1.50
OSCILLATION_ANGULAR_MEAN = 0.40
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


def normalize_angle_delta(delta: float) -> float:
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


def goal_intervals(goal_events: list[dict[str, Any]]) -> dict[int, dict[str, Any]]:
    intervals: dict[int, dict[str, Any]] = {}
    for event in goal_events:
        seq = event.get('goal_sequence')
        if seq is None:
            continue
        seq_i = int(seq)
        rec = intervals.setdefault(seq_i, {'goal_sequence': seq_i})
        name = event.get('event')
        wall_time = event.get('wall_time')
        if name == 'dispatch':
            rec['start_time'] = wall_time
        elif name in ('success', 'timeout', 'failure', 'terminal_cancel'):
            rec['end_time'] = wall_time
            rec['outcome'] = name
            rec['result_reason'] = event.get('result_reason')
        for key, value in event.items():
            if value is not None and key not in ('event',):
                rec[key] = value
    return intervals


def nav2_by_sequence(nav2_analysis: dict[str, Any]) -> dict[int, dict[str, Any]]:
    return {int(goal['goal_sequence']): goal for goal in nav2_analysis.get('goals', []) if goal.get('goal_sequence') is not None}


def split_samples(samples: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    odom: list[dict[str, Any]] = []
    cmd: list[dict[str, Any]] = []
    for sample in samples:
        source = sample.get('source')
        if source == 'odom' or {'x', 'y'} <= set(sample.keys()):
            odom.append(sample)
        if source in ('cmd_vel', 'cmd_vel_nav', 'cmd') or 'linear_x' in sample or 'angular_z' in sample:
            cmd.append(sample)
    return sorted(odom, key=lambda row: float(row.get('wall_time', 0.0))), sorted(cmd, key=lambda row: float(row.get('wall_time', 0.0)))


def samples_between(samples: list[dict[str, Any]], start: float, end: float) -> list[dict[str, Any]]:
    return [row for row in samples if start <= float(row.get('wall_time', -math.inf)) <= end]


def mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def distance(a: dict[str, Any], b: dict[str, Any]) -> float:
    return math.hypot(float(b.get('x', 0.0)) - float(a.get('x', 0.0)), float(b.get('y', 0.0)) - float(a.get('y', 0.0)))


def odom_metrics(samples: list[dict[str, Any]]) -> dict[str, Any]:
    if len(samples) < 2:
        return {
            'odom_sample_count': len(samples),
            'odom_distance_m': None,
            'odom_yaw_delta_abs_sum_rad': None,
            'near_zero_odom_ratio': None,
        }
    total = 0.0
    near_zero_steps = 0
    yaw_sum = 0.0
    for prev, cur in zip(samples, samples[1:]):
        step = distance(prev, cur)
        total += step
        if step <= NEAR_ZERO_ODOM_STEP:
            near_zero_steps += 1
        yaw_sum += abs(normalize_angle_delta(float(cur.get('yaw', 0.0)) - float(prev.get('yaw', 0.0))))
    step_count = max(1, len(samples) - 1)
    return {
        'odom_sample_count': len(samples),
        'odom_distance_m': round(total, 6),
        'odom_yaw_delta_abs_sum_rad': round(yaw_sum, 6),
        'near_zero_odom_ratio': round(near_zero_steps / step_count, 6),
    }


def cmd_metrics(samples: list[dict[str, Any]]) -> dict[str, Any]:
    linear_abs = [abs(float(row.get('linear_x', 0.0))) for row in samples]
    angular_abs = [abs(float(row.get('angular_z', 0.0))) for row in samples]
    near_zero = [
        1 for row in samples
        if abs(float(row.get('linear_x', 0.0))) <= NEAR_ZERO_CMD_LINEAR
        and abs(float(row.get('angular_z', 0.0))) <= NEAR_ZERO_CMD_ANGULAR
    ]
    return {
        'cmd_vel_sample_count': len(samples),
        'cmd_linear_abs_mean': round(mean(linear_abs) or 0.0, 6) if samples else None,
        'cmd_angular_abs_mean': round(mean(angular_abs) or 0.0, 6) if samples else None,
        'near_zero_cmd_ratio': round(len(near_zero) / len(samples), 6) if samples else None,
    }


def _base_flags() -> dict[str, bool]:
    return {
        'healthy_motion': False,
        'stuck_with_cmd': False,
        'oscillation_candidate': False,
        'controller_silent': False,
        'slow_progress': False,
        'late_controller_silent': False,
        'late_stuck_with_cmd': False,
        'late_oscillation': False,
        'healthy_motion_but_late_stall': False,
        'healthy_motion_but_timed_out': False,
        'timeout_or_failure_late_stall': False,
        'terminal_cancel_after_exit': False,
    }


def classify(metrics: dict[str, Any], outcome: str | None = None, result_reason: str | None = None) -> tuple[str, dict[str, bool]]:
    odom_distance = metrics.get('odom_distance_m')
    yaw_sum = metrics.get('odom_yaw_delta_abs_sum_rad')
    cmd_linear = metrics.get('cmd_linear_abs_mean')
    cmd_angular = metrics.get('cmd_angular_abs_mean')
    near_zero_cmd_ratio = metrics.get('near_zero_cmd_ratio')
    flags = _base_flags()
    flags['terminal_cancel_after_exit'] = bool(outcome == 'terminal_cancel' and result_reason == 'exit_reached')
    if odom_distance is None or cmd_linear is None or cmd_angular is None:
        return 'insufficient_data', flags

    controller_silent = bool(near_zero_cmd_ratio is not None and near_zero_cmd_ratio >= 0.9 and odom_distance <= STUCK_ODOM_DISTANCE_M)
    stuck_with_cmd = bool(odom_distance <= STUCK_ODOM_DISTANCE_M and cmd_linear >= MEAN_CMD_ACTIVE and not controller_silent)
    oscillation_candidate = bool(odom_distance <= STUCK_ODOM_DISTANCE_M and (yaw_sum or 0.0) >= OSCILLATION_YAW_ABS_SUM_RAD and cmd_angular >= OSCILLATION_ANGULAR_MEAN)
    healthy_motion = bool(odom_distance >= HEALTHY_ODOM_DISTANCE_M and not controller_silent)
    slow_progress = bool(not healthy_motion and not stuck_with_cmd and not oscillation_candidate and not controller_silent and odom_distance > STUCK_ODOM_DISTANCE_M)

    flags.update({
        'healthy_motion': healthy_motion,
        'stuck_with_cmd': stuck_with_cmd,
        'oscillation_candidate': oscillation_candidate,
        'controller_silent': controller_silent,
        'slow_progress': slow_progress,
    })

    last_distance = metrics.get('last_10s_odom_distance_m')
    last_yaw_sum = metrics.get('last_10s_yaw_delta_abs_sum_rad')
    last_cmd_linear = metrics.get('last_10s_cmd_linear_abs_mean')
    last_cmd_angular = metrics.get('last_10s_cmd_angular_abs_mean')
    is_timeout_or_failure = outcome in ('timeout', 'failure')
    last_odom_stalled = isinstance(last_distance, (int, float)) and float(last_distance) <= STUCK_ODOM_DISTANCE_M
    last_cmd_silent = (
        isinstance(last_cmd_linear, (int, float))
        and isinstance(last_cmd_angular, (int, float))
        and float(last_cmd_linear) <= NEAR_ZERO_CMD_LINEAR
        and float(last_cmd_angular) <= NEAR_ZERO_CMD_ANGULAR
    )
    last_cmd_active = isinstance(last_cmd_linear, (int, float)) and float(last_cmd_linear) >= MEAN_CMD_ACTIVE
    late_oscillation = bool(
        last_odom_stalled
        and isinstance(last_yaw_sum, (int, float))
        and isinstance(last_cmd_angular, (int, float))
        and float(last_yaw_sum) >= OSCILLATION_YAW_ABS_SUM_RAD
        and float(last_cmd_angular) >= OSCILLATION_ANGULAR_MEAN
    )
    late_controller_silent = bool(last_odom_stalled and last_cmd_silent)
    late_stuck_with_cmd = bool(last_odom_stalled and last_cmd_active and not late_controller_silent)
    timeout_or_failure_late_stall = bool(is_timeout_or_failure and (late_controller_silent or late_stuck_with_cmd or late_oscillation))
    healthy_motion_but_late_stall = bool(healthy_motion and timeout_or_failure_late_stall)
    healthy_motion_but_timed_out = bool(outcome == 'timeout' and healthy_motion and not timeout_or_failure_late_stall)

    flags.update({
        'late_controller_silent': late_controller_silent,
        'late_stuck_with_cmd': late_stuck_with_cmd,
        'late_oscillation': late_oscillation,
        'healthy_motion_but_late_stall': healthy_motion_but_late_stall,
        'healthy_motion_but_timed_out': healthy_motion_but_timed_out,
        'timeout_or_failure_late_stall': timeout_or_failure_late_stall,
    })

    if healthy_motion_but_late_stall:
        classification = 'healthy_motion_but_late_stall'
    elif healthy_motion_but_timed_out:
        classification = 'healthy_motion_but_timed_out'
    elif oscillation_candidate:
        classification = 'oscillation_candidate'
    elif stuck_with_cmd:
        classification = 'stuck_with_cmd'
    elif controller_silent:
        classification = 'controller_silent'
    elif late_oscillation and is_timeout_or_failure:
        classification = 'late_oscillation'
    elif late_stuck_with_cmd and is_timeout_or_failure:
        classification = 'late_stuck_with_cmd'
    elif late_controller_silent and is_timeout_or_failure:
        classification = 'late_controller_silent'
    elif healthy_motion:
        classification = 'healthy_motion'
    elif slow_progress:
        classification = 'slow_progress'
    else:
        classification = 'insufficient_data'
    return classification, flags


def analyze_goal(interval: dict[str, Any], nav_goal: dict[str, Any], odom: list[dict[str, Any]], cmd: list[dict[str, Any]]) -> dict[str, Any]:
    start = float(interval.get('start_time'))
    end = float(interval.get('end_time'))
    odom_samples = samples_between(odom, start, end)
    cmd_samples = samples_between(cmd, start, end)
    metrics = {
        **odom_metrics(odom_samples),
        **cmd_metrics(cmd_samples),
    }
    last_start = max(start, end - LAST_WINDOW_SEC)
    last_odom = odom_metrics(samples_between(odom, last_start, end))
    last_cmd = cmd_metrics(samples_between(cmd, last_start, end))
    metrics_with_window = {
        **metrics,
        'last_10s_odom_distance_m': last_odom.get('odom_distance_m'),
        'last_10s_yaw_delta_abs_sum_rad': last_odom.get('odom_yaw_delta_abs_sum_rad'),
        'last_10s_cmd_linear_abs_mean': last_cmd.get('cmd_linear_abs_mean'),
        'last_10s_cmd_angular_abs_mean': last_cmd.get('cmd_angular_abs_mean'),
    }
    classification, flags = classify(
        metrics_with_window,
        nav_goal.get('outcome') or interval.get('outcome'),
        nav_goal.get('result_reason') or interval.get('result_reason'),
    )
    row = {
        'goal_sequence': int(interval['goal_sequence']),
        'outcome': nav_goal.get('outcome') or interval.get('outcome'),
        'result_reason': nav_goal.get('result_reason') or interval.get('result_reason'),
        'start_time': start,
        'end_time': end,
        'elapsed_sec': round(end - start, 6),
        'progress_failure_count': nav_goal.get('progress_failure_count', 0),
        'recovery_count': nav_goal.get('recovery_count', 0),
        'controller_abort_count': nav_goal.get('controller_abort_count', 0),
        **metrics,
        'last_10s_odom_distance_m': last_odom.get('odom_distance_m'),
        'last_10s_yaw_delta_abs_sum_rad': last_odom.get('odom_yaw_delta_abs_sum_rad'),
        'last_10s_cmd_linear_abs_mean': last_cmd.get('cmd_linear_abs_mean'),
        'last_10s_cmd_angular_abs_mean': last_cmd.get('cmd_angular_abs_mean'),
        'classification': classification,
        **flags,
    }
    return row


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    summary = {
        'goal_count': len(rows),
        'healthy_motion_count': 0,
        'stuck_with_cmd_count': 0,
        'oscillation_candidate_count': 0,
        'controller_silent_count': 0,
        'slow_progress_count': 0,
        'healthy_motion_but_late_stall_count': 0,
        'healthy_motion_but_timed_out_count': 0,
        'late_controller_silent_count': 0,
        'late_stuck_with_cmd_count': 0,
        'late_oscillation_count': 0,
        'timeout_or_failure_late_stall_count': 0,
        'terminal_cancel_after_exit_count': 0,
        'insufficient_data_count': 0,
    }
    for row in rows:
        key = f"{row['classification']}_count"
        if key in summary:
            summary[key] += 1
        for flag in (
            'late_controller_silent',
            'late_stuck_with_cmd',
            'late_oscillation',
            'timeout_or_failure_late_stall',
            'terminal_cancel_after_exit',
        ):
            if row.get(flag):
                summary[f'{flag}_count'] += 1
    return summary


def write_csv(rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        'goal_sequence', 'outcome', 'classification', 'odom_distance_m', 'odom_yaw_delta_abs_sum_rad',
        'cmd_linear_abs_mean', 'cmd_angular_abs_mean', 'near_zero_cmd_ratio',
        'last_10s_odom_distance_m', 'last_10s_cmd_linear_abs_mean',
        'progress_failure_count', 'recovery_count', 'controller_abort_count',
    ]
    writer = csv.DictWriter(sys.stdout, fieldnames=fieldnames, extrasaction='ignore')
    writer.writeheader()
    for row in rows:
        writer.writerow(row)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal-events', type=Path, required=True)
    parser.add_argument('--controller-dynamics', type=Path, required=True)
    parser.add_argument('--nav2-analysis', type=Path, required=True)
    parser.add_argument('--output-json', type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    intervals = goal_intervals(read_jsonl(args.goal_events))
    nav = nav2_by_sequence(read_json(args.nav2_analysis))
    odom, cmd = split_samples(read_jsonl(args.controller_dynamics))
    rows: list[dict[str, Any]] = []
    for seq in sorted(intervals):
        interval = intervals[seq]
        if 'start_time' not in interval or 'end_time' not in interval:
            continue
        rows.append(analyze_goal(interval, nav.get(seq, {'goal_sequence': seq}), odom, cmd))
    data = {
        'thresholds': {
            'near_zero_cmd_linear': NEAR_ZERO_CMD_LINEAR,
            'near_zero_cmd_angular': NEAR_ZERO_CMD_ANGULAR,
            'near_zero_odom_step': NEAR_ZERO_ODOM_STEP,
            'stuck_odom_distance_m': STUCK_ODOM_DISTANCE_M,
            'healthy_odom_distance_m': HEALTHY_ODOM_DISTANCE_M,
            'mean_cmd_active': MEAN_CMD_ACTIVE,
            'oscillation_yaw_abs_sum_rad': OSCILLATION_YAW_ABS_SUM_RAD,
            'oscillation_angular_mean': OSCILLATION_ANGULAR_MEAN,
            'last_window_sec': LAST_WINDOW_SEC,
        },
        'summary': summarize(rows),
        'goals': rows,
    }
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    write_csv(rows)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
