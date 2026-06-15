#!/usr/bin/env python3
"""Phase26N per-goal controller/local-cost timeline analyzer.

Analysis-only: reads existing Phase26L/26M artifacts and orders dispatch,
recovery clear-costmap, progress failure, controller abort, first cmd near-zero,
path updates, and local-cost high windows. It does not run ROS/Gazebo/Nav2 or
change branch selection / controller parameters.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

HIGH_COST_THRESHOLD = 70.0
NEAR_ZERO_LINEAR = 0.03
NEAR_ZERO_ANGULAR = 0.05


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


def seq_eq(row: dict[str, Any], seq: int) -> bool:
    return int(number(row.get('goal_sequence')) or -1) == seq


def nav_goal(log_dir: Path, run_id: str, seq: int) -> dict[str, Any]:
    for goal in load_json(log_dir / f'{run_id}_goal_nav2_analysis.json').get('goals', []):
        if seq_eq(goal, seq):
            return goal
    return {}


def subtype_goal(log_dir: Path, run_id: str, seq: int) -> dict[str, Any]:
    data = load_json(log_dir / f'{run_id}_timeout_subtypes.json')
    for row in data.get('timeouts') or data.get('timeout_subtypes') or []:
        if seq_eq(row, seq):
            return row
    return {}


def goal_events(log_dir: Path, run_id: str, seq: int) -> list[dict[str, Any]]:
    return [row for row in load_jsonl(log_dir / f'{run_id}_goal_events.jsonl') if seq_eq(row, seq)]


def snapshot_rows(log_dir: Path, run_id: str, seq: int) -> list[dict[str, Any]]:
    rows = [row for row in load_jsonl(log_dir / f'{run_id}_post_recovery_snapshots.jsonl') if seq_eq(row, seq)]
    rows.sort(key=lambda row: number(row.get('wall_time')) or 0.0)
    return rows


def cmd_rows(log_dir: Path, run_id: str, seq: int) -> list[dict[str, Any]]:
    rows = [
        row for row in load_jsonl(log_dir / f'{run_id}_controller_dynamics.jsonl')
        if row.get('source') in ('cmd_vel', 'cmd_vel_nav', 'cmd') and number(row.get('wall_time')) is not None
    ]
    if any(number(row.get('goal_sequence')) is not None for row in rows):
        rows = [row for row in rows if seq_eq(row, seq)]
    rows.sort(key=lambda row: number(row.get('wall_time')) or 0.0)
    return rows


def is_cmd_near_zero(row: dict[str, Any]) -> bool:
    return abs(number(row.get('linear_x')) or 0.0) <= NEAR_ZERO_LINEAR and abs(number(row.get('angular_z')) or 0.0) <= NEAR_ZERO_ANGULAR


def first_cmd_near_zero_after(rows: list[dict[str, Any]], timestamp: float | None) -> dict[str, Any] | None:
    if timestamp is None:
        return None
    for row in rows:
        wall_time = number(row.get('wall_time'))
        if wall_time is not None and wall_time >= timestamp and is_cmd_near_zero(row):
            return row
    return None


def event_time(row: dict[str, Any] | None) -> float | None:
    return number(row.get('wall_time')) if row else None


def add_event(events: list[dict[str, Any]], label: str, timestamp: float | None, **extra: Any) -> None:
    if timestamp is None:
        return
    row = {'label': label, 'time': round(timestamp, 6)}
    row.update({key: value for key, value in extra.items() if value is not None})
    events.append(row)


def goal_event_time(rows: list[dict[str, Any]], event_name: str) -> float | None:
    for row in rows:
        if row.get('event') == event_name:
            return number(row.get('wall_time'))
    return None


def local_cost_high_snapshots(rows: list[dict[str, Any]], after: float | None = None) -> list[dict[str, Any]]:
    result = []
    for row in rows:
        if row.get('event') != 'snapshot':
            continue
        wall_time = number(row.get('wall_time'))
        if wall_time is None or (after is not None and wall_time < after):
            continue
        max_cost = number(row.get('path_ahead_1_0m_cost_max'))
        high_count = number(row.get('path_ahead_1_0m_high_cost_count'))
        if (max_cost is not None and max_cost >= HIGH_COST_THRESHOLD) or (high_count is not None and high_count > 0):
            result.append(row)
    return result


def path_update_rows(rows: list[dict[str, Any]], after: float | None = None) -> list[dict[str, Any]]:
    result = []
    for row in rows:
        wall_time = number(row.get('wall_time'))
        if row.get('event') == 'path_update' and wall_time is not None and (after is None or wall_time >= after):
            result.append(row)
    return result


def classify(recovery: float | None, progress_failure: float | None, controller_abort: float | None, first_cmd: float | None, path_updates_after: int) -> str:
    if recovery is None or first_cmd is None:
        return 'insufficient_timeline_data'
    before_progress = progress_failure is not None and first_cmd < progress_failure
    before_abort = controller_abort is not None and first_cmd < controller_abort
    after_recovery = first_cmd >= recovery
    if after_recovery and before_progress and before_abort and path_updates_after > 0:
        return 'cmd_near_zero_starts_before_progress_failure_and_abort_while_paths_continue'
    if after_recovery and path_updates_after > 0:
        return 'cmd_near_zero_after_recovery_with_paths_continuing'
    return 'mixed_or_insufficient_timeline'


def analyze_case(log_dir: Path, run_id: str, seq: int) -> dict[str, Any]:
    nav = nav_goal(log_dir, run_id, seq)
    subtype = subtype_goal(log_dir, run_id, seq)
    gevents = goal_events(log_dir, run_id, seq)
    snapshots = snapshot_rows(log_dir, run_id, seq)
    cmds = cmd_rows(log_dir, run_id, seq)

    dispatch = goal_event_time(gevents, 'dispatch')
    timeout = goal_event_time(gevents, 'timeout') or goal_event_time(gevents, 'failure')
    recovery_times = [float(value) for value in nav.get('clear_costmap_times', []) if number(value) is not None]
    progress_times = [float(value) for value in nav.get('progress_failure_times', []) if number(value) is not None]
    abort_times = [float(value) for value in nav.get('controller_abort_times', []) if number(value) is not None]
    recovery = recovery_times[-1] if recovery_times else None
    progress_failure = progress_times[-1] if progress_times else None
    controller_abort = abort_times[-1] if abort_times else None
    first_cmd = first_cmd_near_zero_after(cmds, recovery)
    first_cmd_time = event_time(first_cmd)
    path_updates = path_update_rows(snapshots, recovery)
    high_windows = local_cost_high_snapshots(snapshots, recovery)

    timeline: list[dict[str, Any]] = []
    add_event(timeline, 'dispatch', dispatch)
    for item in recovery_times:
        add_event(timeline, 'recovery_clear_costmap', item)
    add_event(timeline, 'first_cmd_near_zero_after_recovery', first_cmd_time)
    for row in path_updates:
        add_event(timeline, 'path_update', number(row.get('wall_time')))
    for row in snapshots:
        if row.get('event') == 'snapshot' and row.get('snapshot_type') == 'near_zero_onset':
            add_event(timeline, 'near_zero_snapshot', number(row.get('wall_time')), path_ahead_1_0m_cost_max=row.get('path_ahead_1_0m_cost_max'))
    for row in high_windows:
        add_event(
            timeline,
            'local_cost_high_window',
            number(row.get('wall_time')),
            snapshot_type=row.get('snapshot_type'),
            path_ahead_1_0m_cost_max=row.get('path_ahead_1_0m_cost_max'),
            first_high_cost_distance_m=row.get('path_ahead_1_0m_first_high_cost_distance_m'),
            robot_to_path_distance_m=row.get('robot_to_path_distance_m'),
        )
    for item in progress_times:
        add_event(timeline, 'progress_failure', item)
    for item in abort_times:
        add_event(timeline, 'controller_abort', item)
    add_event(timeline, 'timeout', timeout)
    order = {
        'dispatch': 0,
        'recovery_clear_costmap': 1,
        'first_cmd_near_zero_after_recovery': 2,
        'path_update': 3,
        'near_zero_snapshot': 4,
        'local_cost_high_window': 5,
        'progress_failure': 6,
        'controller_abort': 7,
        'timeout': 8,
    }
    timeline.sort(key=lambda event: (float(event['time']), order.get(str(event['label']), 99)))

    return {
        'run_id': run_id,
        'goal_sequence': seq,
        'combined_subtype': subtype.get('combined_subtype'),
        'timing_subtype': subtype.get('timing_subtype'),
        'classification': classify(recovery, progress_failure, controller_abort, first_cmd_time, len(path_updates)),
        'cmd_near_zero_relation': {
            'recovery_time': recovery,
            'progress_failure_time': progress_failure,
            'controller_abort_time': controller_abort,
            'first_cmd_near_zero_time': first_cmd_time,
            'first_cmd_near_zero_after_recovery_sec': round(first_cmd_time - recovery, 6) if first_cmd_time is not None and recovery is not None else None,
            'cmd_near_zero_before_progress_failure': bool(first_cmd_time is not None and progress_failure is not None and first_cmd_time < progress_failure),
            'cmd_near_zero_before_controller_abort': bool(first_cmd_time is not None and controller_abort is not None and first_cmd_time < controller_abort),
            'progress_failure_after_cmd_near_zero_sec': round(progress_failure - first_cmd_time, 6) if first_cmd_time is not None and progress_failure is not None else None,
            'controller_abort_after_cmd_near_zero_sec': round(controller_abort - first_cmd_time, 6) if first_cmd_time is not None and controller_abort is not None else None,
        },
        'path_update_cadence': {
            'path_update_count_after_recovery': len(path_updates),
            'first_path_update_after_recovery_sec': round((number(path_updates[0].get('wall_time')) or 0.0) - recovery, 6) if path_updates and recovery is not None else None,
            'last_path_update_after_recovery_sec': round((number(path_updates[-1].get('wall_time')) or 0.0) - recovery, 6) if path_updates and recovery is not None else None,
        },
        'local_cost_windows': {
            'high_cost_window_count_after_recovery': len(high_windows),
            'first_high_cost_window_after_recovery_sec': round((number(high_windows[0].get('wall_time')) or 0.0) - recovery, 6) if high_windows and recovery is not None else None,
            'max_path_ahead_1_0m_cost': max([number(row.get('path_ahead_1_0m_cost_max')) or 0.0 for row in high_windows], default=None),
        },
        'timeline': timeline,
    }


def parse_case(value: str) -> tuple[str, int]:
    if ':' not in value:
        raise argparse.ArgumentTypeError('case must be run_id:goal_sequence')
    run_id, seq = value.rsplit(':', 1)
    return run_id, int(seq)


def build_report(log_dir: Path, case_specs: list[tuple[str, int]]) -> dict[str, Any]:
    cases = [analyze_case(log_dir, run_id, seq) for run_id, seq in case_specs]
    classifications: dict[str, int] = {}
    for case in cases:
        classifications[case['classification']] = classifications.get(case['classification'], 0) + 1
    return {
        'phase': '26N',
        'analysis_only': True,
        'cases': cases,
        'summary': {
            'case_count': len(cases),
            'classification_counts': classifications,
            'cmd_near_zero_before_progress_and_abort_count': sum(
                1 for case in cases
                if case['cmd_near_zero_relation']['cmd_near_zero_before_progress_failure']
                and case['cmd_near_zero_relation']['cmd_near_zero_before_controller_abort']
            ),
        },
        'decision': {
            'phase27_candidate_signal': 'not_supported',
            'next_recommendation': 'inspect_controller_mppi_critic_logs_before_intervention',
            'guardrails': [
                'do_not_change_branch_selection',
                'do_not_tune_controller_without_mppi_critic_evidence',
                'timeline_evidence_only_not_intervention',
            ],
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, required=True)
    parser.add_argument('--case', action='append', type=parse_case, required=True)
    parser.add_argument('--output-json', type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = build_report(args.log_dir, args.case)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps(report['decision'], sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
