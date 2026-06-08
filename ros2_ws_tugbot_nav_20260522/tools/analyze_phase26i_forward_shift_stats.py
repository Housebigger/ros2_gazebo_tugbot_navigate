#!/usr/bin/env python3
"""Phase26I forward-shift statistics for same-start explored branch candidates.

Analysis-only: consumes existing Phase26E/G/H goal_events artifacts. It does not
start ROS, Gazebo, Nav2, recorders, or change branch-selection/controller params.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
from statistics import median
from typing import Any

LOCAL_COST_CONSTRAINED_THRESHOLD = 70.0
ANGLE_MATCH_THRESHOLD_RAD = math.radians(20.0)
FORWARD_SHIFT_EPS_M = 0.05


def load_jsonl_payloads(path: Path) -> list[dict[str, Any]]:
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


def integer(value: Any) -> int | None:
    numeric = number(value)
    return int(numeric) if numeric is not None else None


def target_payload(value: Any) -> list[float] | None:
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        x = number(value[0])
        y = number(value[1])
        if x is not None and y is not None:
            return [round(x, 6), round(y, 6)]
    return None


def distance(a: list[float] | None, b: list[float] | None) -> float | None:
    if a is None or b is None:
        return None
    return math.hypot(a[0] - b[0], a[1] - b[1])


def angle_delta(a: float | None, b: float | None) -> float | None:
    if a is None or b is None:
        return None
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def local_cost_constrained(candidate: dict[str, Any], dispatch: dict[str, Any] | None = None) -> bool:
    values = [
        number(candidate.get('dispatch_path_local_cost_max')),
        number(candidate.get('target_local_cost')),
        number(candidate.get('target_local_cost_max_radius')),
    ]
    if dispatch is not None:
        values.extend([
            number(dispatch.get('dispatch_path_local_cost_max')),
            number(dispatch.get('dispatch_target_local_cost')),
            number(dispatch.get('dispatch_target_local_cost_max_radius')),
        ])
    return any(value is not None and value >= LOCAL_COST_CONSTRAINED_THRESHOLD for value in values)


def cost_classification(candidate: dict[str, Any], dispatch: dict[str, Any]) -> str:
    return 'high_local_cost' if local_cost_constrained(candidate, dispatch) else 'clean_low_cost'


def normalize_candidate(candidate: dict[str, Any]) -> dict[str, Any]:
    return {
        'branch_angle': number(candidate.get('branch_angle')),
        'target': target_payload(candidate.get('target')),
        'target_exit_dist': number(candidate.get('target_exit_dist')),
        'dispatch_path_local_cost_max': number(candidate.get('dispatch_path_local_cost_max')),
        'dispatch_path_local_cost_mean': number(candidate.get('dispatch_path_local_cost_mean')),
        'target_local_cost': number(candidate.get('target_local_cost')),
        'target_local_cost_max_radius': number(candidate.get('target_local_cost_max_radius')),
        'is_near_exit_candidate': bool(candidate.get('is_near_exit_candidate')),
        'is_reverse_candidate': bool(candidate.get('is_reverse_candidate')),
        'is_backtrack_context': bool(candidate.get('is_backtrack_context')),
        'rejection_reason': candidate.get('rejection_reason'),
        'rank': integer(candidate.get('rank')),
        'state': candidate.get('state'),
    }


def normalize_dispatch(row: dict[str, Any]) -> dict[str, Any]:
    return {
        'event': row.get('event'),
        'goal_sequence': integer(row.get('goal_sequence')),
        'goal_kind': row.get('goal_kind'),
        'start_node_id': integer(row.get('start_node_id')),
        'current_node_id': integer(row.get('current_node_id')),
        'branch_angle': number(row.get('branch_angle')),
        'target': target_payload(row.get('target')),
        'target_exit_dist': number(row.get('target_exit_dist')),
        'robot_exit_dist_at_dispatch': number(row.get('robot_exit_dist_at_dispatch')),
        'dispatch_path_local_cost_max': number(row.get('dispatch_path_local_cost_max')),
        'dispatch_target_local_cost': number(row.get('dispatch_target_local_cost')),
        'dispatch_target_local_cost_max_radius': number(row.get('dispatch_target_local_cost_max_radius')),
        'candidate_branches': [normalize_candidate(candidate) for candidate in row.get('candidate_branches') or [] if isinstance(candidate, dict)],
    }


def outcome_by_sequence(payloads: list[dict[str, Any]]) -> dict[int, dict[str, Any]]:
    outcomes: dict[int, dict[str, Any]] = {}
    for row in payloads:
        if row.get('event') in ('success', 'timeout', 'failure', 'terminal_cancel', 'terminal_cancel_result'):
            seq = integer(row.get('goal_sequence'))
            if seq is not None:
                outcomes[seq] = row
    return outcomes


def read_final_mode(log_dir: Path, run_id: str, payloads: list[dict[str, Any]]) -> str:
    state_path = log_dir / f'{run_id}_explorer_state.jsonl'
    final = None
    for row in load_jsonl_payloads(state_path):
        if row.get('mode'):
            final = row.get('mode')
    if final:
        return str(final)
    if any(row.get('event') == 'terminal_cancel' and row.get('result_reason') == 'exit_reached' for row in payloads):
        return 'EXIT_REACHED'
    return 'UNKNOWN'


def forward_shift_case(source: dict[str, Any], later: dict[str, Any], candidate: dict[str, Any], run_final_mode: str, outcomes: dict[int, dict[str, Any]]) -> dict[str, Any] | None:
    source_start = source.get('start_node_id')
    later_start = later.get('start_node_id')
    if source_start is None or later_start is None or source_start != later_start:
        return None
    d_angle = angle_delta(source.get('branch_angle'), candidate.get('branch_angle'))
    if d_angle is None or d_angle > ANGLE_MATCH_THRESHOLD_RAD:
        return None
    source_target = source.get('target')
    candidate_target = candidate.get('target')
    shift_distance = distance(source_target, candidate_target)
    source_exit = source.get('target_exit_dist')
    candidate_exit = candidate.get('target_exit_dist')
    if source_exit is None or candidate_exit is None:
        return None
    exit_delta = candidate_exit - source_exit
    closer = exit_delta < -FORWARD_SHIFT_EPS_M
    source_seq = source.get('goal_sequence')
    later_seq = later.get('goal_sequence')
    source_outcome = outcomes.get(source_seq or -1, {})
    later_outcome = outcomes.get(later_seq or -1, {})
    classification = cost_classification(candidate, later)
    return {
        'run_final_mode': run_final_mode,
        'start_node_id': source_start,
        'source_goal_sequence': source_seq,
        'later_goal_sequence': later_seq,
        'source_outcome': source_outcome.get('event'),
        'later_outcome': later_outcome.get('event'),
        'source_target': source_target,
        'later_explored_candidate_target': candidate_target,
        'source_target_exit_dist': round(source_exit, 6),
        'later_explored_candidate_target_exit_dist': round(candidate_exit, 6),
        'target_exit_dist_delta_m': round(exit_delta, 6),
        'closer_to_exit': closer,
        'target_forward_shift_distance_m': round(shift_distance, 6) if shift_distance is not None else None,
        'source_branch_angle': source.get('branch_angle'),
        'later_explored_candidate_branch_angle': candidate.get('branch_angle'),
        'angle_delta_rad': round(d_angle, 6),
        'cost_classification': classification,
        'dispatch_path_local_cost_max': candidate.get('dispatch_path_local_cost_max'),
        'target_local_cost': candidate.get('target_local_cost'),
        'target_local_cost_max_radius': candidate.get('target_local_cost_max_radius'),
        'later_dispatch_path_local_cost_max': later.get('dispatch_path_local_cost_max'),
        'later_dispatch_target_local_cost': later.get('dispatch_target_local_cost'),
        'rejection_reason': candidate.get('rejection_reason'),
        'candidate_state': candidate.get('state'),
    }


def find_forward_shift_cases(dispatches: list[dict[str, Any]], outcomes: dict[int, dict[str, Any]], run_final_mode: str) -> list[dict[str, Any]]:
    cases: list[dict[str, Any]] = []
    for later in dispatches:
        later_seq = later.get('goal_sequence')
        if later_seq is None:
            continue
        for candidate in later.get('candidate_branches', []):
            if candidate.get('rejection_reason') != 'explored':
                continue
            for source in dispatches:
                source_seq = source.get('goal_sequence')
                if source_seq is None or source_seq >= later_seq:
                    continue
                case = forward_shift_case(source, later, candidate, run_final_mode, outcomes)
                if case is not None:
                    cases.append(case)
    cases.sort(key=lambda row: (row['later_goal_sequence'], row['source_goal_sequence']))
    return cases


def numeric_summary(values: list[float | None]) -> dict[str, float | int | None]:
    numeric = [float(value) for value in values if value is not None]
    if not numeric:
        return {'count': 0, 'min': None, 'median': None, 'max': None}
    return {
        'count': len(numeric),
        'min': round(min(numeric), 6),
        'median': round(median(numeric), 6),
        'max': round(max(numeric), 6),
    }


def run_summary(cases: list[dict[str, Any]], dispatches: list[dict[str, Any]]) -> dict[str, Any]:
    closer = [case for case in cases if case.get('closer_to_exit')]
    clean_closer = [case for case in closer if case.get('cost_classification') == 'clean_low_cost']
    high_closer = [case for case in closer if case.get('cost_classification') == 'high_local_cost']
    return {
        'dispatch_count': len(dispatches),
        'same_start_explored_forward_shift_count': len(cases),
        'explored_closer_to_exit_forward_shift_count': len(closer),
        'clean_low_cost_explored_closer_count': len(clean_closer),
        'high_local_cost_explored_closer_count': len(high_closer),
        'target_forward_shift_distance_m': numeric_summary([case.get('target_forward_shift_distance_m') for case in cases]),
        'target_exit_dist_delta_m': numeric_summary([case.get('target_exit_dist_delta_m') for case in cases]),
        'angle_delta_rad': numeric_summary([case.get('angle_delta_rad') for case in cases]),
    }


def read_run(log_dir: Path, run_id: str) -> dict[str, Any]:
    payloads = load_jsonl_payloads(log_dir / f'{run_id}_goal_events.jsonl')
    dispatches = [normalize_dispatch(row) for row in payloads if row.get('event') == 'dispatch']
    outcomes = outcome_by_sequence(payloads)
    final_mode = read_final_mode(log_dir, run_id, payloads)
    cases = find_forward_shift_cases(dispatches, outcomes, final_mode)
    return {
        'run_id': run_id,
        'final_mode': final_mode,
        'goal_events_path': str(log_dir / f'{run_id}_goal_events.jsonl'),
        'forward_shift_cases': cases,
        'summary': run_summary(cases, dispatches),
    }


def aggregate_group(run_ids: list[str], runs: dict[str, dict[str, Any]]) -> dict[str, Any]:
    selected = [runs[run_id] for run_id in run_ids if run_id in runs]
    cases = [case for run in selected for case in run.get('forward_shift_cases', [])]
    closer = [case for case in cases if case.get('closer_to_exit')]
    clean_closer = [case for case in closer if case.get('cost_classification') == 'clean_low_cost']
    high_closer = [case for case in closer if case.get('cost_classification') == 'high_local_cost']
    return {
        'run_ids': run_ids,
        'run_count': len(selected),
        'final_modes': {run['run_id']: run.get('final_mode') for run in selected},
        'same_start_explored_forward_shift_count': len(cases),
        'explored_closer_to_exit_forward_shift_count': len(closer),
        'clean_low_cost_explored_closer_count': len(clean_closer),
        'high_local_cost_explored_closer_count': len(high_closer),
        'target_forward_shift_distance_m': numeric_summary([case.get('target_forward_shift_distance_m') for case in cases]),
        'target_exit_dist_delta_m': numeric_summary([case.get('target_exit_dist_delta_m') for case in cases]),
        'angle_delta_rad': numeric_summary([case.get('angle_delta_rad') for case in cases]),
    }


def aggregate_by_final_mode(runs: dict[str, dict[str, Any]]) -> dict[str, Any]:
    modes: dict[str, list[str]] = {}
    for run_id, run in runs.items():
        modes.setdefault(str(run.get('final_mode')), []).append(run_id)
    return {mode: aggregate_group(run_ids, runs) for mode, run_ids in sorted(modes.items())}


def split_runs(value: str | None) -> list[str]:
    if not value:
        return []
    return [item.strip() for item in value.split(',') if item.strip()]


def build_decision(comparison: dict[str, Any], failed_ids: list[str], success_ids: list[str]) -> dict[str, Any]:
    failed_groups = ['FAILED_EXHAUSTED'] if not failed_ids else []
    success_groups = ['EXIT_REACHED'] if not success_ids else []
    failed_clean = 0
    success_clean = 0
    if failed_ids:
        failed_clean = comparison.get('failed_runs', {}).get('clean_low_cost_explored_closer_count', 0)
    else:
        failed_clean = sum(comparison.get(mode, {}).get('clean_low_cost_explored_closer_count', 0) for mode in failed_groups)
    if success_ids:
        success_clean = comparison.get('success_runs', {}).get('clean_low_cost_explored_closer_count', 0)
    else:
        success_clean = sum(comparison.get(mode, {}).get('clean_low_cost_explored_closer_count', 0) for mode in success_groups)
    stable_signal = failed_clean > 0 and success_clean == 0
    return {
        'stable_clean_low_cost_failed_only_signal': stable_signal,
        'phase27_candidate_signal': 'possible_but_requires_real_run_repeats' if stable_signal else 'not_supported',
        'recommendation': 'analysis_only_no_runtime_change' if not stable_signal else 'analysis_only_review_repeat_stability_before_phase27',
        'guardrails': [
            'do_not_enter_phase27_from_phase26i_alone',
            'do_not_change_branch_selection',
            'do_not_tune_nav2_or_controller_params_from_phase26i',
            'require_failed_vs_exit_reached_repeat_stability_before_intervention',
        ],
    }


def analyze(log_dir: Path, run_ids: list[str], failed_ids: list[str], success_ids: list[str]) -> dict[str, Any]:
    all_ids = []
    for run_id in [*run_ids, *failed_ids, *success_ids]:
        if run_id not in all_ids:
            all_ids.append(run_id)
    runs = {run_id: read_run(log_dir, run_id) for run_id in all_ids}
    comparison = aggregate_by_final_mode(runs)
    if failed_ids:
        comparison['failed_runs'] = aggregate_group(failed_ids, runs)
    if success_ids:
        comparison['success_runs'] = aggregate_group(success_ids, runs)
    return {
        'phase': '26I',
        'purpose': 'same-start-node explored branch forward-shift statistics and failed-vs-exit comparison',
        'analysis_only': True,
        'runs': runs,
        'comparison': comparison,
        'decision': build_decision(comparison, failed_ids, success_ids),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, default=Path('log'))
    parser.add_argument('--runs', default='', help='Comma-separated run IDs to analyze')
    parser.add_argument('--failed-runs', default='', help='Comma-separated FAILED_EXHAUSTED run IDs for explicit comparison')
    parser.add_argument('--success-runs', default='', help='Comma-separated EXIT_REACHED run IDs for explicit comparison')
    parser.add_argument('--output-json', type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    payload = analyze(
        args.log_dir,
        split_runs(args.runs),
        split_runs(args.failed_runs),
        split_runs(args.success_runs),
    )
    text = json.dumps(payload, indent=2, sort_keys=True) + '\n'
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(text, encoding='utf-8')
    print(text)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
