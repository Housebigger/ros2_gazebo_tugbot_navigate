#!/usr/bin/env python3
"""Analyze Phase26P MPPI evidence around first cmd near-zero windows.

Analysis-only: joins Phase26N timeline windows with diagnostics-only MPPI evidence
recorded from critics_stats / optimal trajectory / trajectory topics. It can form
a condition hypothesis only when real evidence is present; otherwise intervention
remains blocked.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

COSTCRITIC_HIGH_COST = 100000.0
NEAR_ZERO_LINEAR = 0.03
NEAR_ZERO_ANGULAR = 0.05


def number(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists() or path.stat().st_size == 0:
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    if not path.exists() or path.stat().st_size == 0:
        return rows
    for line in path.open('r', encoding='utf-8'):
        if line.strip():
            rows.append(json.loads(line))
    return rows


def iter_critic_entries(data: Any) -> list[dict[str, Any]]:
    if not isinstance(data, dict):
        return []
    critics = data.get('critics') or data.get('critic_stats') or data.get('critics_stats') or []
    entries = []
    if isinstance(critics, list):
        for item in critics:
            if isinstance(item, dict):
                name = item.get('name') or item.get('critic_name')
                cost = number(item.get('cost'))
                if cost is None:
                    cost = number(item.get('cost_sum'))
                if cost is None:
                    cost = number(item.get('sum_cost'))
                changed = item.get('changed')
                if changed is None:
                    changed = item.get('changed_cost')
                entries.append({'name': name, 'cost': cost, 'changed': changed})
    return entries


def velocities_from_value(value: Any) -> list[dict[str, float]]:
    velocities: list[dict[str, float]] = []
    if isinstance(value, dict):
        if 'velocity' in value:
            velocity = value.get('velocity') or {}
            linear = velocity.get('linear') if isinstance(velocity, dict) else None
            angular = velocity.get('angular') if isinstance(velocity, dict) else None
            vx = number((linear or {}).get('x')) if isinstance(linear, dict) else None
            wz = number((angular or {}).get('z')) if isinstance(angular, dict) else None
            if vx is not None or wz is not None:
                velocities.append({'linear_x': vx or 0.0, 'angular_z': wz or 0.0})
        if 'velocities' in value and isinstance(value['velocities'], list):
            for item in value['velocities']:
                velocities.extend(velocities_from_value(item))
        for key in ('trajectory', 'poses', 'points', 'samples'):
            if isinstance(value.get(key), list):
                for item in value[key]:
                    velocities.extend(velocities_from_value(item))
        if 'linear' in value or 'angular' in value:
            linear = value.get('linear') if isinstance(value.get('linear'), dict) else {}
            angular = value.get('angular') if isinstance(value.get('angular'), dict) else {}
            vx = number(linear.get('x'))
            wz = number(angular.get('z'))
            if vx is not None or wz is not None:
                velocities.append({'linear_x': vx or 0.0, 'angular_z': wz or 0.0})
    elif isinstance(value, list):
        for item in value:
            velocities.extend(velocities_from_value(item))
    return velocities


def path_points_from_value(value: Any) -> list[dict[str, float]]:
    points: list[dict[str, float]] = []
    if isinstance(value, dict):
        pose = value.get('pose') if isinstance(value.get('pose'), dict) else None
        position = None
        if pose and isinstance(pose.get('position'), dict):
            position = pose.get('position')
        elif isinstance(value.get('position'), dict):
            position = value.get('position')
        if isinstance(position, dict):
            x = number(position.get('x'))
            y = number(position.get('y'))
            if x is not None and y is not None:
                points.append({'x': x, 'y': y})
        for key in ('poses', 'points', 'trajectory', 'samples'):
            if isinstance(value.get(key), list):
                for item in value[key]:
                    points.extend(path_points_from_value(item))
    elif isinstance(value, list):
        for item in value:
            points.extend(path_points_from_value(item))
    return points


def path_displacement(points: list[dict[str, float]]) -> float | None:
    if len(points) < 2:
        return None
    first = points[0]
    last = points[-1]
    dx = last['x'] - first['x']
    dy = last['y'] - first['y']
    return (dx * dx + dy * dy) ** 0.5


def near_zero_velocity(item: dict[str, float]) -> bool:
    return abs(item.get('linear_x', 0.0)) <= NEAR_ZERO_LINEAR and abs(item.get('angular_z', 0.0)) <= NEAR_ZERO_ANGULAR


def rows_in_window(rows: list[dict[str, Any]], start: float, end: float) -> list[dict[str, Any]]:
    return [row for row in rows if (number(row.get('wall_time')) is not None and start <= float(row['wall_time']) <= end)]


def analyze_case(case: dict[str, Any], evidence_rows: list[dict[str, Any]]) -> dict[str, Any]:
    run_id = case.get('run_id')
    seq = int(case.get('goal_sequence'))
    relation = case.get('cmd_near_zero_relation', {})
    first_cmd = number(relation.get('first_cmd_near_zero_time'))
    start = (first_cmd - 1.0) if first_cmd is not None else 0.0
    end = (first_cmd + 1.0) if first_cmd is not None else 0.0
    window_rows = rows_in_window(evidence_rows, start, end)

    critic_samples = []
    top = {'name': None, 'max_cost': None, 'changed_count': 0}
    for row in window_rows:
        topic = str(row.get('topic', ''))
        if 'critics_stats' not in topic:
            continue
        entries = iter_critic_entries(row.get('data'))
        if entries:
            critic_samples.append({'wall_time': row.get('wall_time'), 'entries': entries})
        for entry in entries:
            cost = number(entry.get('cost'))
            if cost is not None and (top['max_cost'] is None or cost > float(top['max_cost'])):
                top = {'name': entry.get('name'), 'max_cost': cost, 'changed_count': int(bool(entry.get('changed')))}
            elif entry.get('name') == top.get('name') and entry.get('changed'):
                top['changed_count'] = int(top.get('changed_count', 0)) + 1

    trajectory_rows = [row for row in window_rows if any(token in str(row.get('topic', '')) for token in ('optimal_trajectory', 'trajectory'))]
    trajectory_summary_rows = [row for row in window_rows if (row.get('data_summary') or {}).get('summary_kind') == 'marker_array_trajectory_summary']
    marker_count_values = []
    point_count_values = []
    degenerate_values = []
    representative_lengths = []
    for row in trajectory_summary_rows:
        summary = row.get('data_summary') or {}
        for values, key in (
            (marker_count_values, 'marker_count'),
            (point_count_values, 'point_count'),
            (degenerate_values, 'degenerate_trajectory_count'),
            (representative_lengths, 'representative_path_length'),
        ):
            value = number(summary.get(key))
            if value is not None:
                values.append(value)
    velocity_samples = []
    path_sample_count = 0
    zero_displacement_path_sample_count = 0
    path_displacements = []
    for row in trajectory_rows:
        data = row.get('data') if 'data' in row else row.get('data_summary')
        velocity_samples.extend(velocities_from_value(data))
        points = path_points_from_value(data)
        displacement = path_displacement(points)
        if displacement is None and isinstance(data, dict) and data.get('summary_kind') == 'path_summary':
            displacement = number(data.get('path_displacement'))
        if displacement is not None:
            path_sample_count += 1
            path_displacements.append(displacement)
            if displacement <= 0.03:
                zero_displacement_path_sample_count += 1
    near_zero_count = sum(1 for item in velocity_samples if near_zero_velocity(item))

    validator_rows = [row for row in window_rows if 'validator' in str(row.get('topic', '')).lower()]
    local_cost = case.get('local_cost_windows', {})
    high_before = False
    high_delta = number(local_cost.get('first_high_cost_window_after_recovery_sec'))
    cmd_delta = number(relation.get('first_cmd_near_zero_after_recovery_sec'))
    if high_delta is not None and cmd_delta is not None:
        high_before = high_delta <= cmd_delta

    if top.get('name') == 'CostCritic' and number(top.get('max_cost')) is not None and float(top['max_cost']) >= COSTCRITIC_HIGH_COST and (near_zero_count > 0 or zero_displacement_path_sample_count > 0):
        if zero_displacement_path_sample_count > 0:
            hypothesis = 'costcritic_high_cost_with_degenerate_optimal_path'
        else:
            hypothesis = 'costcritic_high_cost_with_near_zero_optimal_trajectory'
    elif critic_samples and (near_zero_count > 0 or zero_displacement_path_sample_count > 0):
        hypothesis = 'critic_stats_present_with_low_motion_trajectory_unclassified'
    elif critic_samples:
        hypothesis = 'critic_stats_present_without_optimal_velocity_evidence'
    elif degenerate_values and max(degenerate_values) > 0:
        hypothesis = 'trajectory_summary_degenerate_without_critic_stats'
    elif trajectory_rows:
        hypothesis = 'trajectory_evidence_present_without_critic_stats'
    else:
        hypothesis = 'insufficient_mppi_evidence'

    return {
        'run_id': run_id,
        'goal_sequence': seq,
        'join_window': {
            'start_time': start,
            'end_time': end,
            'start_offset_sec': -1.0,
            'end_offset_sec': 1.0,
            'evidence_row_count': len(window_rows),
        },
        'critic_stats': {
            'sample_count': len(critic_samples),
            'top_critic_by_max_cost': top,
        },
        'optimal_trajectory': {
            'sample_count': len(trajectory_rows),
            'velocity_sample_count': len(velocity_samples),
            'near_zero_velocity_sample_count': near_zero_count,
            'path_sample_count': path_sample_count,
            'zero_displacement_path_sample_count': zero_displacement_path_sample_count,
            'path_displacement_min': min(path_displacements) if path_displacements else None,
        },
        'trajectory_summary': {
            'sample_count': len(trajectory_summary_rows),
            'marker_count_max': max(marker_count_values) if marker_count_values else None,
            'point_count_max': max(point_count_values) if point_count_values else None,
            'degenerate_trajectory_count_max': max(degenerate_values) if degenerate_values else 0,
            'representative_path_length_max': max(representative_lengths) if representative_lengths else None,
        },
        'trajectory_validator': {
            'sample_count': len(validator_rows),
            'rows': validator_rows[:5],
        },
        'local_cost_vs_cmd_timing': {
            'high_cost_window_count_after_recovery': local_cost.get('high_cost_window_count_after_recovery'),
            'first_high_cost_window_after_recovery_sec': high_delta,
            'first_cmd_near_zero_after_recovery_sec': cmd_delta,
            'high_cost_before_first_cmd_near_zero': high_before,
        },
        'condition_hypothesis': hypothesis,
    }


def build_report(timeline_json: Path, mppi_evidence: Path) -> dict[str, Any]:
    timeline = load_json(timeline_json)
    evidence_rows = [row for row in load_jsonl(mppi_evidence) if row.get('event', 'message') in ('message', None)]
    cases = [analyze_case(case, evidence_rows) for case in timeline.get('cases', [])]
    hypothesis_counts: dict[str, int] = {}
    for case in cases:
        hypothesis_counts[case['condition_hypothesis']] = hypothesis_counts.get(case['condition_hypothesis'], 0) + 1
    real_evidence_count = sum(
        1 for case in cases
        if case['critic_stats']['sample_count'] > 0 or case['optimal_trajectory']['sample_count'] > 0
    )
    synthetic_only = bool(cases) and all(str(case.get('run_id', '')).startswith('synthetic') for case in cases)
    intervention_signal = (
        'blocked_until_real_run_evidence'
        if real_evidence_count == 0 or synthetic_only
        else 'requires_human_review_before_intervention'
    )
    return {
        'phase': '26Q' if any(case.get('trajectory_summary', {}).get('sample_count', 0) > 0 for case in cases) else '26P',
        'analysis_only': True,
        'source_timeline_json': str(timeline_json),
        'source_mppi_evidence': str(mppi_evidence),
        'cases': cases,
        'summary': {
            'case_count': len(cases),
            'real_mppi_evidence_case_count': real_evidence_count,
            'condition_hypothesis_counts': hypothesis_counts,
        },
        'decision': {
            'phase27_candidate_signal': 'not_supported',
            'intervention_signal': intervention_signal,
            'guardrails': [
                'do_not_change_branch_selection',
                'do_not_tune_mppi_cost_velocity_semantics_in_phase26p',
                'diagnostics_only_profile_must_preserve_semantic_params',
            ],
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeline-json', type=Path, required=True)
    parser.add_argument('--mppi-evidence', type=Path, required=True)
    parser.add_argument('--output-json', type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = build_report(args.timeline_json, args.mppi_evidence)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({'summary': report['summary'], 'decision': report['decision']}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
