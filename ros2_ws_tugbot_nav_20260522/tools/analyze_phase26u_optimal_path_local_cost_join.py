#!/usr/bin/env python3
"""Phase26U offline optimal path / local-cost join.

Diagnostics-only. Uses existing compact MPPI summaries plus post-recovery local
cost snapshots. It reports whether a real spatial join is possible and whether
near-zero coincides with local high-cost choke evidence.
"""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from typing import Any

HIGH_COST = 90
ZERO_DISPLACEMENT_EPS = 0.01


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


def analysis_case(log_dir: Path, run_id: str, seq: int) -> dict[str, Any]:
    for case in load_json(log_dir / f'{run_id}_phase26p_mppi_evidence_analysis.json').get('cases', []):
        if int(case.get('goal_sequence') or -1) == int(seq):
            return case
    return {}


def near_zero_snapshot(log_dir: Path, run_id: str, seq: int, first_cmd: float | None) -> dict[str, Any]:
    enriched = load_json(log_dir / f'{run_id}_post_recovery_enriched.json')
    best = None
    best_abs_age = None
    for row in enriched.get('enriched_recovery_snapshots', []):
        if int(row.get('goal_sequence') or -1) != int(seq):
            continue
        snap = row.get('near_zero_snapshot') or {}
        t = snap.get('wall_time')
        if isinstance(first_cmd, (int, float)) and isinstance(t, (int, float)):
            age = abs(t - first_cmd)
        else:
            age = abs(float(row.get('near_zero_age_sec') or 9999))
        if best is None or age < (best_abs_age or 999999):
            best = row
            best_abs_age = age
    return best or {}


def summarize_path_rows(rows: list[dict[str, Any]]) -> dict[str, Any]:
    disps = []
    lengths = []
    points = []
    raw_pose_rows = 0
    for row in rows:
        summary = row.get('data_summary') or {}
        if summary.get('path_displacement') is not None:
            disps.append(float(summary['path_displacement']))
        if summary.get('path_length') is not None:
            lengths.append(float(summary['path_length']))
        if summary.get('point_count') is not None:
            points.append(int(summary['point_count']))
        data = row.get('data')
        if isinstance(data, dict) and data.get('poses'):
            raw_pose_rows += 1
    return {
        'sample_count': len(rows),
        'point_count_max': max(points) if points else None,
        'path_displacement_min': min(disps) if disps else None,
        'path_displacement_max': max(disps) if disps else None,
        'path_length_max': max(lengths) if lengths else None,
        'zero_displacement_sample_count': sum(1 for d in disps if d <= ZERO_DISPLACEMENT_EPS),
        'raw_pose_rows': raw_pose_rows,
    }


def analyze_case(log_dir: Path, run_id: str, case: dict[str, Any]) -> dict[str, Any]:
    seq = int(case.get('goal_sequence'))
    rel = case.get('cmd_near_zero_relation') or {}
    first_cmd = rel.get('first_cmd_near_zero_time')
    start = float(first_cmd) - 1.0 if isinstance(first_cmd, (int, float)) else 0.0
    end = float(first_cmd) + 1.0 if isinstance(first_cmd, (int, float)) else 0.0
    optimal_rows = evidence_rows(log_dir, run_id, start, end, '/optimal_trajectory')
    transformed_rows = evidence_rows(log_dir, run_id, start, end, '/transformed_global_plan')
    optimal_summary = summarize_path_rows(optimal_rows)
    transformed_summary = summarize_path_rows(transformed_rows)
    mppi_case = analysis_case(log_dir, run_id, seq)
    if mppi_case.get('optimal_trajectory'):
        optimal_summary.update({k: v for k, v in mppi_case['optimal_trajectory'].items() if k not in optimal_summary or optimal_summary[k] in (None, 0)})
    enriched = near_zero_snapshot(log_dir, run_id, seq, first_cmd)
    snap = enriched.get('near_zero_snapshot') or {}
    cost_max = snap.get('path_ahead_1_0m_cost_max', enriched.get('near_zero_path_ahead_1_0m_cost_max'))
    high_count = snap.get('path_ahead_1_0m_high_cost_count')
    robot_to_path = snap.get('robot_to_path_distance_m', enriched.get('near_zero_robot_to_path_distance_m'))
    high_cost = isinstance(cost_max, (int, float)) and cost_max >= HIGH_COST
    geometry_available = optimal_summary.get('raw_pose_rows', 0) > 0
    spatial_join_possible = geometry_available and bool(snap.get('path_ahead_1_0m_high_cost_points'))
    if high_cost and not spatial_join_possible:
        condition = 'near_zero_with_local_path_high_cost_but_optimal_path_geometry_missing'
    elif high_cost and spatial_join_possible:
        condition = 'near_zero_with_spatially_joined_high_cost_choke'
    else:
        condition = 'insufficient_local_cost_choke_evidence'
    return {
        'run_id': run_id,
        'goal_sequence': seq,
        'window': {'start_time': start, 'end_time': end, 'first_cmd_near_zero_time': first_cmd},
        'optimal_trajectory_summary': optimal_summary,
        'transformed_global_plan_summary': transformed_summary,
        'local_cost_choke_evidence': {
            'near_zero_path_ahead_1_0m_cost_max': cost_max,
            'near_zero_high_cost_count': high_count,
            'near_zero_robot_to_path_distance_m': robot_to_path,
            'high_cost_choke_near_zero': high_cost,
        },
        'spatial_join': {
            'optimal_path_geometry_available': geometry_available,
            'high_cost_points_available': bool(snap.get('path_ahead_1_0m_high_cost_points')),
            'spatial_join_possible': spatial_join_possible,
        },
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
    possible_count = sum(1 for c in cases if c['spatial_join']['spatial_join_possible'])
    report = {
        'phase': '26U',
        'analysis_only': True,
        'source_run_ids': args.run_ids,
        'cases': cases,
        'summary': {
            'case_count': len(cases),
            'condition_hypothesis_counts': dict(counts),
            'spatial_join_possible_case_count': possible_count,
            'high_cost_choke_case_count': sum(1 for c in cases if c['local_cost_choke_evidence']['high_cost_choke_near_zero']),
        },
        'decision': {
            'phase27_candidate_signal': 'review_only' if possible_count and any('spatially_joined' in k for k in counts) else 'not_supported',
            'intervention_allowed': False,
            'missing_evidence': ['raw_or_sampled_path_points_needed'] if possible_count < len(cases) else [],
            'guardrails': ['diagnostics_only', 'do_not_change_branch_selection', 'do_not_tune_nav2_controller_params', 'do_not_promote_or_reject_candidate'],
        },
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({'summary': report['summary'], 'decision': report['decision']}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
