#!/usr/bin/env python3
"""Post-run enrich Phase 24C runtime snapshots with Nav2 recovery times."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any

DENSITY_THRESHOLD_SEC = 2.5


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def by_goal(rows: list[dict[str, Any]]) -> dict[int, list[dict[str, Any]]]:
    grouped: dict[int, list[dict[str, Any]]] = {}
    for row in rows:
        seq = row.get('goal_sequence')
        if seq is not None:
            grouped.setdefault(int(seq), []).append(row)
    for goal_rows in grouped.values():
        goal_rows.sort(key=lambda row: float(row.get('wall_time', 0.0)))
    return grouped


def usable_snapshot(row: dict[str, Any]) -> bool:
    if row.get('event') != 'snapshot':
        return False
    return row.get('path_ahead_1_0m_cost_max') is not None or row.get('robot_to_path_distance_m') is not None


def nearest_before(rows: list[dict[str, Any]], timestamp: float) -> dict[str, Any] | None:
    candidates = [row for row in rows if usable_snapshot(row) and float(row.get('wall_time', 0.0)) <= timestamp]
    return candidates[-1] if candidates else None


def nearest_after(rows: list[dict[str, Any]], timestamp: float) -> dict[str, Any] | None:
    for row in rows:
        if usable_snapshot(row) and float(row.get('wall_time', 0.0)) >= timestamp:
            return row
    return None


def nearest_near_zero(rows: list[dict[str, Any]], timestamp: float) -> dict[str, Any] | None:
    near_zero = [row for row in rows if row.get('event') == 'snapshot' and row.get('snapshot_type') == 'near_zero_onset']
    if not near_zero:
        return None
    return min(near_zero, key=lambda row: abs(float(row.get('wall_time', 0.0)) - timestamp))


def path_updates_after(rows: list[dict[str, Any]], timestamp: float) -> int:
    return sum(1 for row in rows if row.get('event') == 'path_update' and float(row.get('wall_time', 0.0)) >= timestamp)


def age_before(snapshot: dict[str, Any] | None, timestamp: float) -> float | None:
    if snapshot is None:
        return None
    return round(timestamp - float(snapshot.get('wall_time')), 6)


def age_after(snapshot: dict[str, Any] | None, timestamp: float) -> float | None:
    if snapshot is None:
        return None
    return round(float(snapshot.get('wall_time')) - timestamp, 6)


def field(snapshot: dict[str, Any] | None, key: str) -> Any:
    return snapshot.get(key) if snapshot is not None else None


def enrich_recovery(goal_seq: int, outcome: str | None, recovery_time: float, rows: list[dict[str, Any]]) -> dict[str, Any]:
    pre = nearest_before(rows, recovery_time)
    post = nearest_after(rows, recovery_time)
    near_zero = nearest_near_zero(rows, recovery_time)
    pre_age = age_before(pre, recovery_time)
    post_age = age_after(post, recovery_time)
    near_zero_age = age_after(near_zero, recovery_time)
    density_ok = (
        pre_age is not None
        and post_age is not None
        and pre_age <= DENSITY_THRESHOLD_SEC
        and post_age <= DENSITY_THRESHOLD_SEC
    )
    return {
        'goal_sequence': goal_seq,
        'outcome': outcome,
        'recovery_time': recovery_time,
        'pre_recovery_snapshot': pre,
        'post_recovery_snapshot': post,
        'near_zero_snapshot': near_zero,
        'pre_recovery_age_sec': pre_age,
        'post_recovery_age_sec': post_age,
        'near_zero_age_sec': near_zero_age,
        'snapshot_density_sufficient': bool(density_ok),
        'path_update_count_after_recovery': path_updates_after(rows, recovery_time),
        'pre_recovery_path_ahead_1_0m_cost_max': field(pre, 'path_ahead_1_0m_cost_max'),
        'post_recovery_path_ahead_1_0m_cost_max': field(post, 'path_ahead_1_0m_cost_max'),
        'near_zero_path_ahead_1_0m_cost_max': field(near_zero, 'path_ahead_1_0m_cost_max'),
        'pre_recovery_robot_to_path_distance_m': field(pre, 'robot_to_path_distance_m'),
        'post_recovery_robot_to_path_distance_m': field(post, 'robot_to_path_distance_m'),
        'near_zero_robot_to_path_distance_m': field(near_zero, 'robot_to_path_distance_m'),
        'controller_received_path_but_cmd_near_zero': field(near_zero, 'controller_received_path_but_cmd_near_zero'),
    }


def enrich(snapshots: list[dict[str, Any]], nav2: dict[str, Any]) -> dict[str, Any]:
    grouped = by_goal(snapshots)
    rows: list[dict[str, Any]] = []
    for goal in nav2.get('goals', []):
        seq = goal.get('goal_sequence')
        if seq is None:
            continue
        seq = int(seq)
        goal_rows = grouped.get(seq, [])
        for recovery_time in goal.get('clear_costmap_times', []) or []:
            if isinstance(recovery_time, (int, float)):
                rows.append(enrich_recovery(seq, goal.get('outcome'), float(recovery_time), goal_rows))
    summary = {
        'recovery_count': len(rows),
        'with_pre_recovery_count': sum(1 for row in rows if row.get('pre_recovery_snapshot') is not None),
        'with_post_recovery_count': sum(1 for row in rows if row.get('post_recovery_snapshot') is not None),
        'with_near_zero_count': sum(1 for row in rows if row.get('near_zero_snapshot') is not None),
        'sufficient_density_count': sum(1 for row in rows if row.get('snapshot_density_sufficient') is True),
        'needs_periodic_snapshot_count': sum(1 for row in rows if row.get('snapshot_density_sufficient') is not True),
        'controller_received_path_but_cmd_near_zero_count': sum(1 for row in rows if row.get('controller_received_path_but_cmd_near_zero') is True),
    }
    return {'summary': summary, 'enriched_recovery_snapshots': rows}


def write_csv(rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        'goal_sequence',
        'recovery_time',
        'pre_recovery_age_sec',
        'post_recovery_age_sec',
        'near_zero_age_sec',
        'snapshot_density_sufficient',
        'pre_recovery_path_ahead_1_0m_cost_max',
        'post_recovery_path_ahead_1_0m_cost_max',
        'near_zero_path_ahead_1_0m_cost_max',
        'near_zero_robot_to_path_distance_m',
        'path_update_count_after_recovery',
    ]
    writer = csv.DictWriter(sys.stdout, fieldnames=fieldnames, extrasaction='ignore')
    writer.writeheader()
    for row in rows:
        writer.writerow(row)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--snapshots', type=Path, required=True)
    parser.add_argument('--nav2-analysis', type=Path, required=True)
    parser.add_argument('--output-json', type=Path)
    args = parser.parse_args()
    data = enrich(load_jsonl(args.snapshots), load_json(args.nav2_analysis))
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    write_csv(data['enriched_recovery_snapshots'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
