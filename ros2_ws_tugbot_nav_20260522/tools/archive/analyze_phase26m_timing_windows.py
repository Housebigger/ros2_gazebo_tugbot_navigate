#!/usr/bin/env python3
"""Phase26M controller/local-cost timing window refinement.

Analysis-only: joins existing Phase26L artifacts to compare matched timeout pairs.
It does not launch ROS/Gazebo/Nav2 and does not change branch selection or
controller parameters.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
from typing import Any


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists() or path.stat().st_size == 0:
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists() or path.stat().st_size == 0:
        return rows
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        payload = row.get('state', row)
        if isinstance(payload, dict):
            rows.append(payload)
    return rows


def number(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def goal_by_sequence(items: list[dict[str, Any]]) -> dict[int, dict[str, Any]]:
    result: dict[int, dict[str, Any]] = {}
    for item in items:
        seq = number(item.get('goal_sequence'))
        if seq is not None:
            result[int(seq)] = item
    return result


def rows_for_goal(rows: list[dict[str, Any]], seq: int) -> list[dict[str, Any]]:
    selected = [row for row in rows if int(number(row.get('goal_sequence')) or -1) == seq]
    selected.sort(key=lambda row: float(number(row.get('wall_time')) or 0.0))
    return selected


def usable_snapshot(row: dict[str, Any]) -> bool:
    return row.get('event') == 'snapshot' and row.get('path_ahead_1_0m_cost_max') is not None


def nearest_before(rows: list[dict[str, Any]], timestamp: float) -> dict[str, Any] | None:
    candidates = [row for row in rows if usable_snapshot(row) and (number(row.get('wall_time')) or 0.0) <= timestamp]
    return candidates[-1] if candidates else None


def nearest_after(rows: list[dict[str, Any]], timestamp: float) -> dict[str, Any] | None:
    for row in rows:
        if usable_snapshot(row) and (number(row.get('wall_time')) or 0.0) >= timestamp:
            return row
    return None


def near_zero_after_recovery(rows: list[dict[str, Any]], recovery_time: float) -> dict[str, Any] | None:
    near_zero = [
        row for row in rows
        if row.get('event') == 'snapshot'
        and row.get('snapshot_type') == 'near_zero_onset'
        and (number(row.get('wall_time')) or 0.0) >= recovery_time
    ]
    return near_zero[0] if near_zero else None


def count_path_updates_after(rows: list[dict[str, Any]], timestamp: float) -> int:
    return sum(1 for row in rows if row.get('event') == 'path_update' and (number(row.get('wall_time')) or 0.0) >= timestamp)


def cmd_rows_after(log_dir: Path, run_id: str, timestamp: float, seq: int | None = None) -> list[dict[str, Any]]:
    rows = [
        row for row in load_jsonl(log_dir / f'{run_id}_controller_dynamics.jsonl')
        if row.get('source') in ('cmd_vel', 'cmd_vel_nav', 'cmd') and number(row.get('wall_time')) is not None and float(row['wall_time']) >= timestamp
    ]
    if seq is not None and any(number(row.get('goal_sequence')) is not None for row in rows):
        rows = [row for row in rows if int(number(row.get('goal_sequence')) or -1) == seq]
    rows.sort(key=lambda row: float(row.get('wall_time', 0.0)))
    return rows


def is_cmd_near_zero(row: dict[str, Any]) -> bool:
    linear = abs(number(row.get('linear_x')) or 0.0)
    angular = abs(number(row.get('angular_z')) or 0.0)
    return linear <= 0.03 and angular <= 0.05


def first_cmd_near_zero_after(log_dir: Path, run_id: str, timestamp: float, seq: int | None = None) -> dict[str, Any] | None:
    for row in cmd_rows_after(log_dir, run_id, timestamp, seq):
        if is_cmd_near_zero(row):
            return row
    return None


def cmd_near_zero_ratio_after(log_dir: Path, run_id: str, timestamp: float, window_sec: float, seq: int | None = None) -> float | None:
    rows = [row for row in cmd_rows_after(log_dir, run_id, timestamp, seq) if float(row.get('wall_time', 0.0)) <= timestamp + window_sec]
    if not rows:
        return None
    return round(sum(1 for row in rows if is_cmd_near_zero(row)) / len(rows), 6)


def max_or_none(values: list[float | None]) -> float | None:
    cleaned = [value for value in values if value is not None]
    return max(cleaned) if cleaned else None


def extract_timeout_subtype(log_dir: Path, run_id: str, seq: int) -> dict[str, Any]:
    data = load_json(log_dir / f'{run_id}_timeout_subtypes.json')
    rows = data.get('timeouts') or data.get('timeout_subtypes') or []
    for row in rows:
        if int(number(row.get('goal_sequence')) or -1) == seq:
            return row
    return {}


def extract_nav_goal(log_dir: Path, run_id: str, seq: int) -> dict[str, Any]:
    data = load_json(log_dir / f'{run_id}_goal_nav2_analysis.json')
    return goal_by_sequence(data.get('goals', [])) .get(seq, {})


def extract_controller_goal(log_dir: Path, run_id: str, seq: int) -> dict[str, Any]:
    data = load_json(log_dir / f'{run_id}_goal_controller_dynamics.json')
    return goal_by_sequence(data.get('goals', [])) .get(seq, {})


def analyze_case(log_dir: Path, run_id: str, seq: int) -> dict[str, Any]:
    rows = rows_for_goal(load_jsonl(log_dir / f'{run_id}_post_recovery_snapshots.jsonl'), seq)
    nav_goal = extract_nav_goal(log_dir, run_id, seq)
    controller_goal = extract_controller_goal(log_dir, run_id, seq)
    subtype = extract_timeout_subtype(log_dir, run_id, seq)
    recovery_times = [float(value) for value in nav_goal.get('clear_costmap_times', []) if number(value) is not None]
    recovery_time = recovery_times[-1] if recovery_times else None
    if recovery_time is None:
        snapshots = [row for row in rows if usable_snapshot(row)]
        recovery_time = number(snapshots[0].get('wall_time')) if snapshots else None
    pre = nearest_before(rows, recovery_time) if recovery_time is not None else None
    post = nearest_after(rows, recovery_time) if recovery_time is not None else None
    near_zero = near_zero_after_recovery(rows, recovery_time) if recovery_time is not None else None
    if near_zero is None:
        near_zero_candidates = [row for row in rows if row.get('event') == 'snapshot' and row.get('snapshot_type') == 'near_zero_onset']
        near_zero = near_zero_candidates[0] if near_zero_candidates else None
    path_updates = count_path_updates_after(rows, recovery_time) if recovery_time is not None else 0
    near_zero_time = number(near_zero.get('wall_time')) if near_zero else None
    cmd_near_zero = first_cmd_near_zero_after(log_dir, run_id, recovery_time, seq) if recovery_time is not None else None
    cmd_near_zero_time = number(cmd_near_zero.get('wall_time')) if cmd_near_zero else None
    post_time = number(post.get('wall_time')) if post else None
    pre_time = number(pre.get('wall_time')) if pre else None
    near_zero_delay = round(near_zero_time - recovery_time, 6) if near_zero_time is not None and recovery_time is not None else None
    abort_times = [number(value) for value in nav_goal.get('controller_abort_times', [])]
    progress_times = [number(value) for value in nav_goal.get('progress_failure_times', [])]
    return {
        'run_id': run_id,
        'goal_sequence': seq,
        'outcome': nav_goal.get('outcome') or subtype.get('outcome'),
        'combined_subtype': subtype.get('combined_subtype'),
        'controller_subtype': subtype.get('controller_subtype'),
        'timing_subtype': subtype.get('timing_subtype'),
        'controller_classification': controller_goal.get('classification'),
        'last_window_classification': controller_goal.get('last_window_classification'),
        'recovery_time': recovery_time,
        'near_zero_time': near_zero_time,
        'near_zero_after_recovery_sec': near_zero_delay,
        'cmd_near_zero_time': cmd_near_zero_time,
        'cmd_near_zero_after_recovery_sec': round(cmd_near_zero_time - recovery_time, 6) if cmd_near_zero_time is not None and recovery_time is not None else None,
        'cmd_near_zero_ratio_5s_after_recovery': cmd_near_zero_ratio_after(log_dir, run_id, recovery_time, 5.0, seq) if recovery_time is not None else None,
        'pre_recovery_age_sec': round(recovery_time - pre_time, 6) if recovery_time is not None and pre_time is not None else None,
        'post_recovery_age_sec': round(post_time - recovery_time, 6) if recovery_time is not None and post_time is not None else None,
        'path_updates_after_recovery': path_updates,
        'controller_abort_after_recovery_sec': round((max_or_none(abort_times) or 0.0) - recovery_time, 6) if recovery_time is not None and max_or_none(abort_times) is not None else None,
        'progress_failure_after_recovery_sec': round((max_or_none(progress_times) or 0.0) - recovery_time, 6) if recovery_time is not None and max_or_none(progress_times) is not None else None,
        'pre_path_ahead_1_0m_cost_max': pre.get('path_ahead_1_0m_cost_max') if pre else None,
        'post_path_ahead_1_0m_cost_max': post.get('path_ahead_1_0m_cost_max') if post else None,
        'near_zero_path_ahead_1_0m_cost_max': near_zero.get('path_ahead_1_0m_cost_max') if near_zero else None,
        'near_zero_first_high_cost_distance_m': near_zero.get('path_ahead_1_0m_first_high_cost_distance_m') if near_zero else None,
        'near_zero_high_cost_count': near_zero.get('path_ahead_1_0m_high_cost_count') if near_zero else None,
        'near_zero_robot_to_path_distance_m': near_zero.get('robot_to_path_distance_m') if near_zero else None,
        'chosen_route_corridor_cost_max': near_zero.get('chosen_route_corridor_cost_max') if near_zero else None,
        'chosen_route_first_high_cost_distance_m': near_zero.get('chosen_route_first_high_cost_distance_m') if near_zero else None,
        'explored_candidate_corridor_cost_max': near_zero.get('explored_candidate_corridor_cost_max') if near_zero else None,
        'explored_candidate_first_high_cost_distance_m': near_zero.get('explored_candidate_first_high_cost_distance_m') if near_zero else None,
        'controller_received_path_but_cmd_near_zero': near_zero.get('controller_received_path_but_cmd_near_zero') if near_zero else None,
    }


def classify_pair(label: str, left: dict[str, Any], right: dict[str, Any]) -> tuple[str, str]:
    same_subtype = left.get('combined_subtype') == right.get('combined_subtype') and left.get('combined_subtype') is not None
    both_path_updates = (left.get('path_updates_after_recovery') or 0) > 0 and (right.get('path_updates_after_recovery') or 0) > 0
    both_late_silent = left.get('timing_subtype') == 'cmd_silent_after_recovery_abort' and right.get('timing_subtype') == 'cmd_silent_after_recovery_abort'
    if label == 'near_exit':
        return 'non_divergent_or_shared_near_exit_timing_window', 'near_exit_pair_is_not_phase27_branch_evidence'
    if same_subtype and both_path_updates and both_late_silent:
        return 'persistent_cmd_silent_after_recovery_with_path_updates', 'timing_or_controller_silence_not_branch_selection'
    return 'mixed_or_insufficient_timing_window', 'continue_diagnostics'


def parse_pair(value: str) -> tuple[str, str, int, str, int]:
    parts = value.split(':')
    if len(parts) != 5:
        raise argparse.ArgumentTypeError('pair must be label:left_run:left_seq:right_run:right_seq')
    return parts[0], parts[1], int(parts[2]), parts[3], int(parts[4])


def analyze_pairs(log_dir: Path, pair_specs: list[tuple[str, str, int, str, int]]) -> dict[str, Any]:
    pairs: dict[str, Any] = {}
    subtype_counts: Counter[str] = Counter()
    for label, left_run, left_seq, right_run, right_seq in pair_specs:
        left = analyze_case(log_dir, left_run, left_seq)
        right = analyze_case(log_dir, right_run, right_seq)
        if left.get('combined_subtype'):
            subtype_counts[str(left['combined_subtype'])] += 1
        if right.get('combined_subtype'):
            subtype_counts[str(right['combined_subtype'])] += 1
        matched_pattern, interpretation = classify_pair(label, left, right)
        pairs[label] = {
            'label': label,
            'baseline': left,
            'candidate': right,
            'matched_pattern': matched_pattern,
            'interpretation': interpretation,
            'delta': {
                'near_zero_after_recovery_sec': (
                    round((right.get('near_zero_after_recovery_sec') or 0.0) - (left.get('near_zero_after_recovery_sec') or 0.0), 6)
                    if left.get('near_zero_after_recovery_sec') is not None and right.get('near_zero_after_recovery_sec') is not None else None
                ),
                'path_updates_after_recovery': (right.get('path_updates_after_recovery') or 0) - (left.get('path_updates_after_recovery') or 0),
                'robot_to_path_distance_m': (
                    round((right.get('near_zero_robot_to_path_distance_m') or 0.0) - (left.get('near_zero_robot_to_path_distance_m') or 0.0), 6)
                    if left.get('near_zero_robot_to_path_distance_m') is not None and right.get('near_zero_robot_to_path_distance_m') is not None else None
                ),
            },
        }
    common = subtype_counts.most_common(1)[0][0] if subtype_counts else None
    return {
        'phase': '26M',
        'analysis_only': True,
        'pairs': pairs,
        'summary': {
            'pair_count': len(pairs),
            'common_combined_subtype': common,
            'combined_subtype_counts': dict(sorted(subtype_counts.items())),
        },
        'decision': {
            'phase27_candidate_signal': 'not_supported',
            'next_recommendation': 'continue_controller_local_cost_timing_diagnostics',
            'guardrails': [
                'do_not_change_branch_selection',
                'do_not_tune_nav2_or_controller_from_phase26m_alone',
                'require_timing_root_cause_before_intervention',
            ],
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, required=True)
    parser.add_argument('--pair', action='append', type=parse_pair, required=True, help='label:left_run:left_seq:right_run:right_seq')
    parser.add_argument('--output-json', type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = analyze_pairs(args.log_dir, args.pair)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps(report['decision'], sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
