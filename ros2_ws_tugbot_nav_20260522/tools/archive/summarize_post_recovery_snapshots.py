#!/usr/bin/env python3
"""Summarize Phase 24C post-recovery snapshot JSONL."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any

SNAPSHOT_TYPES = ('pre_recovery', 'post_recovery', 'near_zero_onset')
FIELDS = (
    'path_ahead_0_5m_cost_max',
    'path_ahead_1_0m_cost_max',
    'robot_to_path_distance_m',
)


def load_rows(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def summarize_goal(rows: list[dict[str, Any]]) -> dict[str, Any]:
    seq = int(rows[0].get('goal_sequence'))
    out: dict[str, Any] = {'goal_sequence': seq}
    for snapshot_type in SNAPSHOT_TYPES:
        candidates = [row for row in rows if row.get('event') == 'snapshot' and row.get('snapshot_type') == snapshot_type]
        if not candidates:
            continue
        row = candidates[-1]
        out_prefix = 'near_zero' if snapshot_type == 'near_zero_onset' else snapshot_type
        out[f'{out_prefix}_time'] = row.get('wall_time')
        for field in FIELDS:
            out[f'{out_prefix}_{field}'] = row.get(field)
        if snapshot_type == 'near_zero_onset':
            out['controller_received_path_but_cmd_near_zero'] = row.get('controller_received_path_but_cmd_near_zero')
            out['post_recovery_path_update_count'] = row.get('post_recovery_path_update_count')
    return out


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    by_seq: dict[int, list[dict[str, Any]]] = {}
    for row in rows:
        seq = row.get('goal_sequence')
        if seq is not None:
            by_seq.setdefault(int(seq), []).append(row)
    goals = [summarize_goal(goal_rows) for _, goal_rows in sorted(by_seq.items())]
    summary = {
        'goal_count': len(goals),
        'with_pre_recovery_count': sum(1 for row in goals if 'pre_recovery_time' in row),
        'with_post_recovery_count': sum(1 for row in goals if 'post_recovery_time' in row),
        'with_near_zero_onset_count': sum(1 for row in goals if 'near_zero_time' in row),
        'controller_received_path_but_cmd_near_zero_count': sum(1 for row in goals if row.get('controller_received_path_but_cmd_near_zero') is True),
        'with_robot_to_path_distance_count': sum(1 for row in goals if row.get('near_zero_robot_to_path_distance_m') is not None),
    }
    return {'summary': summary, 'goals': goals}


def write_csv(goals: list[dict[str, Any]]) -> None:
    fieldnames = [
        'goal_sequence',
        'pre_recovery_path_ahead_1_0m_cost_max',
        'post_recovery_path_ahead_1_0m_cost_max',
        'near_zero_path_ahead_1_0m_cost_max',
        'near_zero_robot_to_path_distance_m',
        'controller_received_path_but_cmd_near_zero',
        'post_recovery_path_update_count',
    ]
    writer = csv.DictWriter(sys.stdout, fieldnames=fieldnames, extrasaction='ignore')
    writer.writeheader()
    for row in goals:
        writer.writerow(row)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--snapshots', type=Path, required=True)
    parser.add_argument('--output-json', type=Path)
    args = parser.parse_args()
    data = summarize(load_rows(args.snapshots))
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    write_csv(data['goals'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
