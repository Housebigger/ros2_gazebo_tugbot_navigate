#!/usr/bin/env python3
"""Compare maze goal diagnostics across multiple smoke runs.

This is an analysis-only Phase 20 tool: it labels repeated timeout/success
signals, but does not recommend or apply navigation-control changes.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any

HIGH_COST_THRESHOLD = 70
HIGH_TIMEOUT_ROBOT_COST_THRESHOLD = 90
LOW_CORRIDOR_CLEARANCE_M = 0.55
NEAR_EXIT_DISTANCE_M = 1.0


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def _read_jsonl_payloads(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists():
        return rows
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        rows.append(row.get('state', row))
    return rows


def _last_state(path: Path) -> dict[str, Any]:
    rows = _read_jsonl_payloads(path)
    return rows[-1] if rows else {}


def _goal_event_context(goal_events_path: Path) -> dict[int, dict[str, Any]]:
    by_seq: dict[int, dict[str, Any]] = {}
    for payload in _read_jsonl_payloads(goal_events_path):
        seq = payload.get('goal_sequence')
        if seq is None:
            continue
        rec = by_seq.setdefault(int(seq), {'goal_sequence': int(seq), 'events': []})
        rec['events'].append(payload.get('event'))
        # Keep last non-null value for every diagnostic field. Outcome events can
        # carry timeout-specific local-cost fields not present on dispatch.
        for key, value in payload.items():
            if key in ('event',):
                continue
            if value is not None:
                rec[key] = value
        event = payload.get('event')
        if event in ('success', 'timeout', 'failure', 'terminal_cancel'):
            rec['outcome'] = event
    return by_seq


def _nav2_goals(nav2_path: Path) -> list[dict[str, Any]]:
    data = _read_json(nav2_path)
    return list(data.get('goals', []))


def _nav2_summary(nav2_path: Path) -> dict[str, Any]:
    data = _read_json(nav2_path)
    return dict(data.get('summary', {}))


def _bool(value: Any) -> bool:
    return bool(value) if value is not None else False


def _num(value: Any, default: float = 0.0) -> float:
    return float(value) if isinstance(value, (int, float)) else default


def _taxonomy_tags(goal: dict[str, Any]) -> dict[str, bool]:
    path_clearance = goal.get('path_corridor_min_clearance_m')
    target_exit_dist = goal.get('target_exit_dist')
    timeout_robot_cost = goal.get('timeout_robot_local_cost_max')
    return {
        'map_narrow_passage': _bool(goal.get('target_crosses_narrow_passage')),
        'low_path_corridor_clearance': isinstance(path_clearance, (int, float)) and float(path_clearance) < LOW_CORRIDOR_CLEARANCE_M,
        'high_dispatch_target_cost': _num(goal.get('dispatch_target_local_cost')) >= HIGH_COST_THRESHOLD,
        'high_dispatch_path_cost': _num(goal.get('dispatch_path_local_cost_max')) >= HIGH_COST_THRESHOLD,
        'high_timeout_robot_cost': isinstance(timeout_robot_cost, (int, float)) and float(timeout_robot_cost) >= HIGH_TIMEOUT_ROBOT_COST_THRESHOLD,
        'timeout_squeezed': _bool(goal.get('footprint_corridor_inflation_squeezed')),
        'nav2_progress_cluster': _num(goal.get('progress_failure_count')) > 0 or _num(goal.get('recovery_count')) > 0 or _num(goal.get('controller_abort_count')) > 0,
        'near_exit': isinstance(target_exit_dist, (int, float)) and float(target_exit_dist) <= NEAR_EXIT_DISTANCE_M,
        'caused_blocked_or_blacklist': _bool(goal.get('caused_branch_failure')) or _bool(goal.get('caused_blacklist')),
    }


def _empty_counts() -> dict[str, int]:
    return {key: 0 for key in _taxonomy_tags({}).keys()}


def _merge_goal(nav_goal: dict[str, Any], event_by_seq: dict[int, dict[str, Any]]) -> dict[str, Any]:
    seq = int(nav_goal.get('goal_sequence'))
    merged = dict(event_by_seq.get(seq, {}))
    merged.update(nav_goal)
    # Preserve embedded local-cost fields from goal events if nav2 goal rows do not know them.
    for key, value in event_by_seq.get(seq, {}).items():
        if key not in merged or merged.get(key) is None:
            merged[key] = value
    merged['goal_sequence'] = seq
    return merged


def analyze_run(run: dict[str, Path]) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    run_id = str(run['run_id'])
    final_state = _last_state(run['state'])
    nav2_summary = _nav2_summary(run['nav2'])
    event_by_seq = _goal_event_context(run['events'])
    merged_goals = [_merge_goal(goal, event_by_seq) for goal in _nav2_goals(run['nav2']) if goal.get('goal_sequence') is not None]

    timeout_rows: list[dict[str, Any]] = []
    success_rows: list[dict[str, Any]] = []
    timeout_counts = _empty_counts()
    success_counts = _empty_counts()

    for goal in merged_goals:
        tags = _taxonomy_tags(goal)
        row = {
            'run_id': run_id,
            'goal_sequence': goal.get('goal_sequence'),
            'outcome': goal.get('outcome'),
            'target_exit_dist': goal.get('target_exit_dist'),
            'target_clearance_m': goal.get('target_clearance_m'),
            'path_corridor_min_clearance_m': goal.get('path_corridor_min_clearance_m'),
            'dispatch_target_local_cost': goal.get('dispatch_target_local_cost'),
            'dispatch_path_local_cost_max': goal.get('dispatch_path_local_cost_max'),
            'dispatch_local_cost_sample_coverage_ratio': goal.get('dispatch_local_cost_sample_coverage_ratio'),
            'timeout_robot_local_cost_max': goal.get('timeout_robot_local_cost_max'),
            'timeout_robot_obstacle_cluster_count': goal.get('timeout_robot_obstacle_cluster_count'),
            'progress_failure_count': goal.get('progress_failure_count'),
            'recovery_count': goal.get('recovery_count'),
            'controller_abort_count': goal.get('controller_abort_count'),
            'caused_branch_failure': goal.get('caused_branch_failure'),
            'caused_blacklist': goal.get('caused_blacklist'),
            'tags': tags,
            'taxonomy_labels': [key for key, value in tags.items() if value],
        }
        if goal.get('outcome') == 'timeout':
            timeout_rows.append(row)
            for key, value in tags.items():
                timeout_counts[key] += int(value)
        elif goal.get('outcome') == 'success':
            success_rows.append(row)
            for key, value in tags.items():
                success_counts[key] += int(value)

    run_summary = {
        'run_id': run_id,
        'final_mode': final_state.get('mode'),
        'exit_distance_m': final_state.get('exit_distance_m'),
        'goal_count': final_state.get('goal_count', nav2_summary.get('goal_count')),
        'success_count': nav2_summary.get('success_count', final_state.get('goal_success_count')),
        'timeout_count': nav2_summary.get('timeout_count', len(timeout_rows)),
        'timeout_with_progress_failure_count': nav2_summary.get('timeout_with_progress_failure_count'),
        'timeout_with_recovery_count': nav2_summary.get('timeout_with_recovery_count'),
        'blocked_branch_count': final_state.get('blocked_branch_count'),
        'blacklisted_goal_count': final_state.get('blacklisted_goal_count'),
        'timeout_taxonomy_counts': timeout_counts,
        'success_taxonomy_counts': success_counts,
        'timeout_goal_sequences': [row['goal_sequence'] for row in timeout_rows],
    }
    return run_summary, timeout_rows, success_rows


def _taxonomy_by_outcome(timeout_goals: list[dict[str, Any]], success_goals: list[dict[str, Any]]) -> dict[str, dict[str, dict[str, int]]]:
    taxonomy: dict[str, dict[str, dict[str, int]]] = {}
    for outcome, rows in (('timeout', timeout_goals), ('success', success_goals)):
        taxonomy[outcome] = {key: {'count': 0} for key in _empty_counts().keys()}
        for row in rows:
            for key, value in row.get('tags', {}).items():
                taxonomy[outcome][key]['count'] += int(bool(value))
    return taxonomy


def summarize_all(runs: list[dict[str, Any]], timeout_goals: list[dict[str, Any]]) -> dict[str, Any]:
    timeout_values = [int(run.get('timeout_count') or 0) for run in runs]
    exit_reached_count = sum(1 for run in runs if run.get('final_mode') == 'EXIT_REACHED')
    all_blocked_blacklist_zero = all((run.get('blocked_branch_count') or 0) == 0 and (run.get('blacklisted_goal_count') or 0) == 0 for run in runs)
    timeout_tag_totals = _empty_counts()
    for row in timeout_goals:
        for key, value in row['tags'].items():
            timeout_tag_totals[key] += int(value)

    run_count = len(runs)
    timeout_total = sum(timeout_values)
    branch_selection_ready = bool(
        run_count >= 3
        and exit_reached_count == run_count
        and all_blocked_blacklist_zero
        and timeout_total > 0
        and timeout_tag_totals['high_dispatch_path_cost'] == timeout_total
        and timeout_tag_totals['high_timeout_robot_cost'] == timeout_total
        and timeout_tag_totals['caused_blocked_or_blacklist'] > 0
    )

    if branch_selection_ready:
        recommended = 'branch_scoring_candidate'
    elif timeout_total and timeout_tag_totals['nav2_progress_cluster'] == timeout_total and timeout_tag_totals['high_timeout_robot_cost'] < timeout_total:
        recommended = 'controller_diagnostics_candidate'
    elif timeout_total and timeout_tag_totals['near_exit'] >= max(1, timeout_total // 2):
        recommended = 'near_exit_approach_candidate'
    else:
        recommended = 'repeat_sampling'

    return {
        'run_count': run_count,
        'exit_reached_count': exit_reached_count,
        'timeout_count_total': timeout_total,
        'blocked_or_blacklist_run_count': sum(
            1 for run in runs if (run.get('blocked_branch_count') or 0) > 0 or (run.get('blacklisted_goal_count') or 0) > 0
        ),
        'all_blocked_blacklist_zero': all_blocked_blacklist_zero,
        'timeout_count_values': timeout_values,
        'timeout_count_min': min(timeout_values) if timeout_values else None,
        'timeout_count_max': max(timeout_values) if timeout_values else None,
        'timeout_count_stable': len(set(timeout_values)) <= 1 if timeout_values else False,
        'timeout_taxonomy_totals': timeout_tag_totals,
        'branch_selection_ready': branch_selection_ready,
        'recommended_next_phase': recommended,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--run',
        nargs='+',
        action='append',
        metavar='RUN_ARG',
        help='Run tuple: RUN_ID state.jsonl goal_events.jsonl nav2_analysis.json [geometry_summary.json] [cost_summary.json]',
        required=True,
    )
    parser.add_argument('--state', action='append', type=Path, help='State JSONL for the preceding --run RUN_ID form.')
    parser.add_argument('--goal-events', action='append', type=Path, help='Goal events JSONL for the preceding --run RUN_ID form.')
    parser.add_argument('--nav2-analysis', action='append', type=Path, help='Nav2 analysis JSON for the preceding --run RUN_ID form.')
    parser.add_argument('--geometry-summary', action='append', type=Path, help='Optional geometry summary JSON for the preceding --run RUN_ID form.')
    parser.add_argument('--local-cost-summary', action='append', type=Path, help='Optional local-cost summary JSON for the preceding --run RUN_ID form.')
    parser.add_argument('--output-json', type=Path)
    return parser.parse_args()


def _parse_runs(args: argparse.Namespace) -> list[dict[str, Path]]:
    raw_runs = args.run
    if raw_runs and all(len(raw) == 1 for raw in raw_runs) and args.state and args.goal_events and args.nav2_analysis:
        run_count = len(raw_runs)
        if len(args.state) != run_count or len(args.goal_events) != run_count or len(args.nav2_analysis) != run_count:
            raise SystemExit('flag-style --run requires one --state, --goal-events, and --nav2-analysis per run')
        runs = []
        for index, raw in enumerate(raw_runs):
            run: dict[str, Path | str] = {
                'run_id': raw[0],
                'state': args.state[index],
                'events': args.goal_events[index],
                'nav2': args.nav2_analysis[index],
            }
            if args.geometry_summary and index < len(args.geometry_summary):
                run['geometry'] = args.geometry_summary[index]
            if args.local_cost_summary and index < len(args.local_cost_summary):
                run['cost'] = args.local_cost_summary[index]
            runs.append(run)  # type: ignore[arg-type]
        return runs  # type: ignore[return-value]

    runs = []
    for raw in raw_runs:
        if len(raw) < 4:
            raise SystemExit('--run requires at least RUN_ID STATE_JSONL GOAL_EVENTS_JSONL NAV2_JSON')
        run: dict[str, Path | str] = {
            'run_id': raw[0],
            'state': Path(raw[1]),
            'events': Path(raw[2]),
            'nav2': Path(raw[3]),
        }
        if len(raw) >= 5:
            run['geometry'] = Path(raw[4])
        if len(raw) >= 6:
            run['cost'] = Path(raw[5])
        runs.append(run)  # type: ignore[arg-type]
    return runs  # type: ignore[return-value]


def write_run_csv(runs: list[dict[str, Any]]) -> None:
    fieldnames = [
        'run_id',
        'final_mode',
        'timeout_count',
        'blocked_branch_count',
        'blacklisted_goal_count',
        'success_count',
        'goal_count',
        'exit_distance_m',
        'timeout_with_progress_failure_count',
        'timeout_with_recovery_count',
    ]
    writer = csv.DictWriter(sys.stdout, fieldnames=fieldnames, extrasaction='ignore')
    writer.writeheader()
    for row in runs:
        writer.writerow(row)


def main() -> int:
    args = parse_args()
    run_specs = _parse_runs(args)
    runs: list[dict[str, Any]] = []
    timeout_goals: list[dict[str, Any]] = []
    success_goals: list[dict[str, Any]] = []
    for run_spec in run_specs:
        run_summary, timeout_rows, success_rows = analyze_run(run_spec)
        runs.append(run_summary)
        timeout_goals.extend(timeout_rows)
        success_goals.extend(success_rows)

    data = {
        'thresholds': {
            'high_cost_threshold': HIGH_COST_THRESHOLD,
            'high_timeout_robot_cost_threshold': HIGH_TIMEOUT_ROBOT_COST_THRESHOLD,
            'low_corridor_clearance_m': LOW_CORRIDOR_CLEARANCE_M,
            'near_exit_distance_m': NEAR_EXIT_DISTANCE_M,
        },
        'summary': summarize_all(runs, timeout_goals),
        'taxonomy': _taxonomy_by_outcome(timeout_goals, success_goals),
        'runs': runs,
        'timeout_goals': timeout_goals,
        'success_goals': success_goals,
    }
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    write_run_csv(runs)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
