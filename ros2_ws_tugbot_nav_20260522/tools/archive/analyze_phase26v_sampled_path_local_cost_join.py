#!/usr/bin/env python3
"""Phase26V sampled path / local-cost spatial join.

Diagnostics-only. Joins bounded sampled `/optimal_trajectory` and
`/transformed_global_plan` path poses with local high-cost points near first
cmd_vel_nav near-zero. It never authorizes intervention.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import Counter
from pathlib import Path
from typing import Any

HIGH_COST = 90
INTERSECTION_DISTANCE_M = 0.10


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists() or path.stat().st_size == 0:
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def iter_jsonl(path: Path):
    if not path.exists():
        return
    with path.open(encoding='utf-8') as f:
        for line in f:
            if line.strip():
                try:
                    yield json.loads(line)
                except json.JSONDecodeError:
                    continue


def norm_point(value: Any) -> tuple[float, float] | None:
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        try:
            return float(value[0]), float(value[1])
        except (TypeError, ValueError):
            return None
    if isinstance(value, dict):
        x = value.get('x')
        y = value.get('y')
        try:
            return float(x), float(y)
        except (TypeError, ValueError):
            return None
    return None


def distance_2d(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def path_length(points: list[tuple[float, float]]) -> float:
    return sum(distance_2d(a, b) for a, b in zip(points, points[1:]))


def evidence_rows(log_dir: Path, run_id: str, start: float, end: float, topic: str) -> list[dict[str, Any]]:
    rows = []
    for row in iter_jsonl(log_dir / f'{run_id}_mppi_evidence_summary.jsonl') or []:
        if row.get('topic') != topic:
            continue
        t = row.get('wall_time')
        if isinstance(t, (int, float)) and start <= t <= end:
            rows.append(row)
    return rows


def timeline_cases(log_dir: Path, run_id: str) -> list[dict[str, Any]]:
    return load_json(log_dir / f'{run_id}_phase26p_single_goal_timeline.json').get('cases', [])


def nearest_snapshot(log_dir: Path, run_id: str, seq: int, first_cmd: float | None) -> dict[str, Any]:
    enriched = load_json(log_dir / f'{run_id}_post_recovery_enriched.json')
    best = None
    best_age = None
    for row in enriched.get('enriched_recovery_snapshots', []):
        if int(row.get('goal_sequence') or -1) != int(seq):
            continue
        snap = row.get('near_zero_snapshot') or {}
        t = snap.get('wall_time')
        if isinstance(first_cmd, (int, float)) and isinstance(t, (int, float)):
            age = abs(t - first_cmd)
        else:
            age = abs(float(row.get('near_zero_age_sec') or 9999))
        if best is None or age < (best_age or 999999):
            best = row
            best_age = age
    return best or {}


def collect_sampled_points(rows: list[dict[str, Any]]) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for row in rows:
        summary = row.get('data_summary') or {}
        for raw in summary.get('sampled_points') or []:
            point = norm_point(raw)
            if point is not None:
                points.append(point)
    return points


def metric_for_path(rows: list[dict[str, Any]], high_points: list[tuple[float, float]]) -> dict[str, Any]:
    sampled = collect_sampled_points(rows)
    min_distance = None
    if sampled and high_points:
        min_distance = min(distance_2d(a, b) for a in sampled for b in high_points)
    lengths = []
    disps = []
    point_counts = []
    for row in rows:
        summary = row.get('data_summary') or {}
        if summary.get('path_length') is not None:
            lengths.append(float(summary['path_length']))
        if summary.get('path_displacement') is not None:
            disps.append(float(summary['path_displacement']))
        if summary.get('sampled_point_count') is not None:
            point_counts.append(int(summary['sampled_point_count']))
    return {
        'sample_count': len(rows),
        'sampled_point_count': len(sampled),
        'sampled_point_count_max_per_message': max(point_counts) if point_counts else None,
        'path_length_max': max(lengths) if lengths else None,
        'path_displacement_min': min(disps) if disps else None,
        'path_displacement_max': max(disps) if disps else None,
        'min_distance_to_high_cost': min_distance,
        'intersects_high_cost': bool(min_distance is not None and min_distance <= INTERSECTION_DISTANCE_M),
    }


def analyze_case(log_dir: Path, run_id: str, case: dict[str, Any]) -> dict[str, Any]:
    seq = int(case.get('goal_sequence'))
    rel = case.get('cmd_near_zero_relation') or {}
    first_cmd = rel.get('first_cmd_near_zero_time')
    start = float(first_cmd) - 1.0 if isinstance(first_cmd, (int, float)) else 0.0
    end = float(first_cmd) + 1.0 if isinstance(first_cmd, (int, float)) else 0.0
    snap_row = nearest_snapshot(log_dir, run_id, seq, first_cmd)
    snap = snap_row.get('near_zero_snapshot') or {}
    high_points = [p for p in (norm_point(v) for v in (snap.get('path_ahead_1_0m_high_cost_points') or [])) if p is not None]
    cost_max = snap.get('path_ahead_1_0m_cost_max', snap_row.get('near_zero_path_ahead_1_0m_cost_max'))
    optimal_rows = evidence_rows(log_dir, run_id, start, end, '/optimal_trajectory')
    transformed_rows = evidence_rows(log_dir, run_id, start, end, '/transformed_global_plan')
    optimal = metric_for_path(optimal_rows, high_points)
    transformed = metric_for_path(transformed_rows, high_points)
    if optimal['intersects_high_cost']:
        condition = 'sampled_optimal_path_intersects_high_cost_choke'
    elif transformed['intersects_high_cost']:
        condition = 'sampled_transformed_plan_intersects_high_cost_choke'
    elif high_points and (optimal['sampled_point_count'] or transformed['sampled_point_count']):
        condition = 'sampled_paths_do_not_intersect_high_cost_choke'
    else:
        condition = 'insufficient_sampled_geometry_for_join'
    return {
        'run_id': run_id,
        'goal_sequence': seq,
        'window': {'start_time': start, 'end_time': end, 'first_cmd_near_zero_time': first_cmd},
        'local_high_cost': {
            'path_ahead_1_0m_cost_max': cost_max,
            'high_cost_point_count': len(high_points),
            'robot_to_path_distance_m': snap.get('robot_to_path_distance_m', snap_row.get('near_zero_robot_to_path_distance_m')),
        },
        'optimal_path_intersects_high_cost': optimal['intersects_high_cost'],
        'transformed_plan_intersects_high_cost': transformed['intersects_high_cost'],
        'optimal_path_min_distance_to_high_cost': optimal['min_distance_to_high_cost'],
        'transformed_plan_min_distance_to_high_cost': transformed['min_distance_to_high_cost'],
        'optimal_path_summary': optimal,
        'transformed_plan_summary': transformed,
        'condition_hypothesis': condition,
    }


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--log-dir', type=Path, default=Path('log'))
    p.add_argument('--run-ids', nargs='+', required=True)
    p.add_argument('--output-json', type=Path, required=True)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    cases = []
    for run_id in args.run_ids:
        for case in timeline_cases(args.log_dir, run_id):
            cases.append(analyze_case(args.log_dir, run_id, case))
    counts = Counter(c['condition_hypothesis'] for c in cases)
    intersection_count = sum(1 for c in cases if c['optimal_path_intersects_high_cost'] or c['transformed_plan_intersects_high_cost'])
    report = {
        'phase': '26V',
        'analysis_only': True,
        'source_run_ids': args.run_ids,
        'cases': cases,
        'summary': {
            'case_count': len(cases),
            'condition_hypothesis_counts': dict(counts),
            'sampled_geometry_case_count': sum(1 for c in cases if c['optimal_path_summary']['sampled_point_count'] or c['transformed_plan_summary']['sampled_point_count']),
            'intersection_case_count': intersection_count,
        },
        'decision': {
            'phase27_candidate_signal': 'review_only' if intersection_count else 'not_supported',
            'intervention_allowed': False,
            'guardrails': ['diagnostics_only', 'do_not_change_branch_selection', 'do_not_tune_nav2_controller_params', 'do_not_promote_or_reject_candidate'],
        },
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({'summary': report['summary'], 'decision': report['decision']}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
