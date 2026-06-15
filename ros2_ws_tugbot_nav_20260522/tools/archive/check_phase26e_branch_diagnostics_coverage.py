#!/usr/bin/env python3
"""Validate Phase26E branch-choice diagnostic coverage in goal_events JSONL."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

TOP_FIELDS = [
    'chosen_branch_rank',
    'chosen_branch_score_components',
    'candidate_branch_count',
    'candidate_branches',
    'selected_due_to_context',
]

CANDIDATE_FIELDS = [
    'branch_angle',
    'target',
    'target_exit_dist',
    'exit_progress_delta_m',
    'target_clearance_m',
    'path_corridor_min_clearance_m',
    'dispatch_path_local_cost_max',
    'dispatch_path_local_cost_mean',
    'target_local_cost',
    'target_local_cost_max_radius',
    'is_reverse_candidate',
    'is_backtrack_context',
    'is_near_exit_candidate',
    'rejection_reason',
]


def load_events(path: Path) -> list[dict]:
    events: list[dict] = []
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        events.append(row.get('state', row))
    return events


def summarize(path: Path) -> dict:
    events = load_events(path)
    dispatch = [row for row in events if row.get('event') == 'dispatch']
    top_missing = {field: 0 for field in TOP_FIELDS}
    candidate_missing = {field: 0 for field in CANDIDATE_FIELDS}
    candidate_rows = 0
    nonempty_candidate_dispatch = 0

    for row in dispatch:
        for field in TOP_FIELDS:
            if field not in row:
                top_missing[field] += 1
        candidates = row.get('candidate_branches') or []
        if candidates:
            nonempty_candidate_dispatch += 1
        for candidate in candidates:
            candidate_rows += 1
            for field in CANDIDATE_FIELDS:
                if field not in candidate:
                    candidate_missing[field] += 1

    pass_coverage = bool(dispatch) and all(count == 0 for count in top_missing.values())
    pass_coverage = pass_coverage and candidate_rows > 0 and all(count == 0 for count in candidate_missing.values())

    return {
        'goal_events': str(path),
        'total_events': len(events),
        'dispatch_events': len(dispatch),
        'dispatch_events_with_nonempty_candidate_branches': nonempty_candidate_dispatch,
        'candidate_branch_rows': candidate_rows,
        'chosen_branch_ranks': [row.get('chosen_branch_rank') for row in dispatch],
        'candidate_branch_counts': [row.get('candidate_branch_count') for row in dispatch],
        'selected_due_to_context_values': sorted({str(row.get('selected_due_to_context')) for row in dispatch}),
        'top_field_missing_counts': top_missing,
        'candidate_field_missing_counts': candidate_missing,
        'pass_coverage': pass_coverage,
        'diagnostics_only_note': 'coverage check only; do not use this smoke to promote or reject Nav2/controller/branch parameters',
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal-events', required=True, type=Path)
    parser.add_argument('--output-json', required=True, type=Path)
    args = parser.parse_args()

    summary = summarize(args.goal_events)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps(summary, indent=2, sort_keys=True))
    if not summary['pass_coverage']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
