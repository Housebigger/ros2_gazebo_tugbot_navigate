#!/usr/bin/env python3
"""Correlate /maze/goal_events JSONL with Nav2/controller launch logs.

The goal-events stream gives goal sequence, dispatch/outcome wall time, target,
branch angle, topology, and elapsed time. The launch log gives controller-side
symptoms such as progress checker failures, recovery/costmap clears, and action
aborts. This script assigns each controller log event to the active goal interval
[dispatch_wall_time, outcome_wall_time] and prints a per-goal diagnostic table.
"""

from __future__ import annotations
import argparse
import csv
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

PROGRESS_PATTERNS = ('Failed to make progress',)
RECOVERY_PATTERNS = (
    'Received request to clear entirely the local_costmap',
    'Received request to clear entirely the global_costmap',
    'local_costmap]: Received request to clear',
    'global_costmap]: Received request to clear',
    'clear entirely the local_costmap',
    'clear entirely the global_costmap',
)
ABORT_PATTERNS = ('Aborting handle', 'Goal failed', 'Goal aborted')
LOG_TIME_RE = re.compile(r'\[(\d+(?:\.\d+)?)\]')


@dataclass
class GoalRecord:
    goal_sequence: int
    dispatch_wall_time: Optional[float] = None
    outcome_wall_time: Optional[float] = None
    outcome: Optional[str] = None
    dispatch: dict[str, Any] = field(default_factory=dict)
    outcome_event: dict[str, Any] = field(default_factory=dict)
    progress_failure_count: int = 0
    recovery_count: int = 0
    controller_abort_count: int = 0
    controller_events: list[dict[str, Any]] = field(default_factory=list)

    def interval_end(self, fallback: float) -> float:
        if self.outcome_wall_time is not None:
            return self.outcome_wall_time
        return fallback


def load_goal_records(path: Path) -> dict[int, GoalRecord]:
    records: dict[int, GoalRecord] = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        payload = row.get('state', row)
        seq = payload.get('goal_sequence')
        if seq is None:
            continue
        seq = int(seq)
        rec = records.setdefault(seq, GoalRecord(goal_sequence=seq))
        event = payload.get('event')
        wall_time = row.get('wall_time')
        if event == 'dispatch':
            rec.dispatch = payload
            rec.dispatch_wall_time = float(wall_time) if wall_time is not None else None
        elif event in ('success', 'timeout', 'failure', 'terminal_cancel', 'stale_result'):
            # Keep first terminal/outcome-class event. Cancel-result events are extra status detail.
            if rec.outcome is None:
                rec.outcome = event
                rec.outcome_event = payload
                rec.outcome_wall_time = float(wall_time) if wall_time is not None else None
        elif event in ('timeout_cancel_result', 'terminal_cancel_result'):
            if rec.outcome_event:
                rec.outcome_event.setdefault('cancel_result_status', payload.get('result_status'))
                rec.outcome_event.setdefault('cancel_result_reason', payload.get('result_reason'))
            else:
                rec.outcome_event = payload
    return records


def parse_log_time(line: str) -> Optional[float]:
    matches = LOG_TIME_RE.findall(line)
    if not matches:
        return None
    # ROS log lines include several bracketed fields; the numeric timestamp is the first numeric bracket.
    return float(matches[0])


def classify_log_line(line: str) -> list[str]:
    categories: list[str] = []
    if any(pattern in line for pattern in PROGRESS_PATTERNS):
        categories.append('progress_failure')
    if any(pattern in line for pattern in RECOVERY_PATTERNS):
        categories.append('recovery')
    if any(pattern in line for pattern in ABORT_PATTERNS):
        categories.append('controller_abort')
    return categories


def assign_controller_events(records: dict[int, GoalRecord], log_path: Path) -> None:
    if not records or not log_path.exists():
        return
    ordered = sorted(records.values(), key=lambda rec: (rec.dispatch_wall_time is None, rec.dispatch_wall_time or 0.0, rec.goal_sequence))
    known_times = [t for rec in ordered for t in (rec.dispatch_wall_time, rec.outcome_wall_time) if t is not None]
    fallback_end = max(known_times) + 1e9 if known_times else float('inf')

    for line in log_path.read_text(encoding='utf-8', errors='replace').splitlines():
        categories = classify_log_line(line)
        if not categories:
            continue
        timestamp = parse_log_time(line)
        if timestamp is None:
            continue
        rec = find_goal_for_time(ordered, timestamp, fallback_end=fallback_end)
        if rec is None:
            continue
        event = {'timestamp': timestamp, 'categories': categories, 'line': line}
        rec.controller_events.append(event)
        if 'progress_failure' in categories:
            rec.progress_failure_count += 1
        if 'recovery' in categories:
            rec.recovery_count += 1
        if 'controller_abort' in categories:
            rec.controller_abort_count += 1


def find_goal_for_time(ordered: list[GoalRecord], timestamp: float, fallback_end: float) -> Optional[GoalRecord]:
    candidate = None
    for idx, rec in enumerate(ordered):
        if rec.dispatch_wall_time is None:
            continue
        next_dispatch = None
        for later in ordered[idx + 1:]:
            if later.dispatch_wall_time is not None:
                next_dispatch = later.dispatch_wall_time
                break
        end = rec.interval_end(next_dispatch if next_dispatch is not None else fallback_end)
        if rec.dispatch_wall_time <= timestamp <= end:
            candidate = rec
            break
    return candidate


def row_for_record(rec: GoalRecord) -> dict[str, Any]:
    dispatch = rec.dispatch
    outcome_event = rec.outcome_event
    elapsed = outcome_event.get('elapsed_sec')
    if elapsed is None and rec.dispatch_wall_time is not None and rec.outcome_wall_time is not None:
        elapsed = rec.outcome_wall_time - rec.dispatch_wall_time
    progress_failure_times = [event['timestamp'] for event in rec.controller_events if 'progress_failure' in event.get('categories', [])]
    clear_costmap_times = [event['timestamp'] for event in rec.controller_events if 'recovery' in event.get('categories', [])]
    controller_abort_times = [event['timestamp'] for event in rec.controller_events if 'controller_abort' in event.get('categories', [])]
    return {
        'goal_sequence': rec.goal_sequence,
        'outcome': rec.outcome,
        'goal_kind': dispatch.get('goal_kind'),
        'local_topology': dispatch.get('local_topology'),
        'branch_angle': dispatch.get('branch_angle'),
        'dispatch_pose': dispatch.get('dispatch_pose'),
        'target': dispatch.get('target'),
        'target_exit_dist': dispatch.get('target_exit_dist'),
        'robot_exit_dist_at_dispatch': dispatch.get('robot_exit_dist_at_dispatch'),
        'near_exit': dispatch.get('near_exit'),
        'effective_timeout_sec': dispatch.get('effective_timeout_sec'),
        'elapsed_sec': elapsed,
        'result_status': outcome_event.get('result_status'),
        'result_reason': outcome_event.get('result_reason'),
        'branch_failure_state': outcome_event.get('branch_failure_state'),
        'caused_branch_failure': outcome_event.get('caused_branch_failure'),
        'caused_blacklist': outcome_event.get('caused_blacklist'),
        'progress_failure_count': rec.progress_failure_count,
        'recovery_count': rec.recovery_count,
        'controller_abort_count': rec.controller_abort_count,
        'progress_failure_times': progress_failure_times,
        'clear_costmap_times': clear_costmap_times,
        'controller_abort_times': controller_abort_times,
        'first_progress_failure_time': progress_failure_times[0] if progress_failure_times else None,
        'last_progress_failure_time': progress_failure_times[-1] if progress_failure_times else None,
        'first_clear_costmap_time': clear_costmap_times[0] if clear_costmap_times else None,
        'last_clear_costmap_time': clear_costmap_times[-1] if clear_costmap_times else None,
        'first_controller_abort_time': controller_abort_times[0] if controller_abort_times else None,
        'last_controller_abort_time': controller_abort_times[-1] if controller_abort_times else None,
    }


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    timeout_rows = [row for row in rows if row.get('outcome') == 'timeout']
    success_rows = [row for row in rows if row.get('outcome') == 'success']
    return {
        'goal_count': len(rows),
        'success_count': len(success_rows),
        'timeout_count': len(timeout_rows),
        'timeout_with_progress_failure_count': sum(1 for row in timeout_rows if row.get('progress_failure_count', 0) > 0),
        'success_with_progress_failure_count': sum(1 for row in success_rows if row.get('progress_failure_count', 0) > 0),
        'timeout_with_recovery_count': sum(1 for row in timeout_rows if row.get('recovery_count', 0) > 0),
        'success_with_recovery_count': sum(1 for row in success_rows if row.get('recovery_count', 0) > 0),
        'timeout_with_controller_abort_count': sum(1 for row in timeout_rows if row.get('controller_abort_count', 0) > 0),
        'success_with_controller_abort_count': sum(1 for row in success_rows if row.get('controller_abort_count', 0) > 0),
    }


def print_table(rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        'goal_sequence',
        'outcome',
        'goal_kind',
        'local_topology',
        'branch_angle',
        'target_exit_dist',
        'robot_exit_dist_at_dispatch',
        'near_exit',
        'effective_timeout_sec',
        'elapsed_sec',
        'progress_failure_count',
        'recovery_count',
        'controller_abort_count',
        'result_reason',
        'branch_failure_state',
        'caused_branch_failure',
        'caused_blacklist',
    ]
    writer = csv.DictWriter(sys.stdout, fieldnames=fieldnames, extrasaction='ignore')
    writer.writeheader()
    for row in rows:
        writer.writerow(row)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal-events', required=True, type=Path)
    parser.add_argument('--log', required=True, type=Path)
    parser.add_argument('--output-json', type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    records = load_goal_records(args.goal_events)
    assign_controller_events(records, args.log)
    rows = [row_for_record(records[seq]) for seq in sorted(records)]
    summary = summarize(rows)
    data = {'summary': summary, 'goals': rows}
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print_table(rows)
    print(json.dumps({'summary': summary}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
