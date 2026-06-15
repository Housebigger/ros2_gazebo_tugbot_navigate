#!/usr/bin/env python3
"""Phase26J joins same-start explored forward-shift cases with outcome,
timeout subtype, and post-recovery enriched artifacts.

Analysis-only: consumes existing goal_events / timeout_subtypes /
post_recovery_enriched artifacts. It does not start ROS, Gazebo, Nav2, recorders,
or change branch-selection/controller params.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from statistics import median
from typing import Any

LOCAL_COST_CONSTRAINED_THRESHOLD = 70.0
ANGLE_MATCH_THRESHOLD_RAD = math.radians(20.0)
FORWARD_SHIFT_EPS_M = 0.05
HIGH_COST_THRESHOLD = 70.0


def load_json(path: Path, default: Any) -> Any:
    if not path.exists() or path.stat().st_size == 0:
        return default
    return json.loads(path.read_text(encoding='utf-8'))


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


def compact_outcome(row: dict[str, Any] | None) -> dict[str, Any]:
    if not row:
        return {'event': None, 'result_reason': None}
    return {
        'event': row.get('event'),
        'result_reason': row.get('result_reason'),
        'status': row.get('status'),
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


def load_timeout_subtypes(log_dir: Path, run_id: str) -> dict[int, dict[str, Any]]:
    data = load_json(log_dir / f'{run_id}_timeout_subtypes.json', {})
    rows = data.get('timeout_subtypes', []) if isinstance(data, dict) else []
    result: dict[int, dict[str, Any]] = {}
    for row in rows:
        seq = integer(row.get('goal_sequence'))
        if seq is not None:
            result[seq] = row
    return result


def compact_timeout_subtype(row: dict[str, Any] | None) -> dict[str, Any] | None:
    if not row:
        return None
    return {
        'goal_sequence': integer(row.get('goal_sequence')),
        'classification': row.get('classification'),
        'controller_subtype': row.get('controller_subtype'),
        'timing_subtype': row.get('timing_subtype'),
        'combined_subtype': row.get('combined_subtype'),
        'reasons': row.get('reasons', {}),
    }


def load_post_recovery(log_dir: Path, run_id: str) -> dict[int, dict[str, Any]]:
    data = load_json(log_dir / f'{run_id}_post_recovery_enriched.json', {})
    rows = data.get('enriched_recovery_snapshots', []) if isinstance(data, dict) else []
    by_seq: dict[int, list[dict[str, Any]]] = {}
    for row in rows:
        seq = integer(row.get('goal_sequence'))
        if seq is not None:
            by_seq.setdefault(seq, []).append(row)
    result: dict[int, dict[str, Any]] = {}
    for seq, items in by_seq.items():
        result[seq] = max(items, key=lambda row: number(row.get('path_update_count_after_recovery')) or 0)
    return result


def compact_post_recovery(row: dict[str, Any] | None) -> dict[str, Any] | None:
    if not row:
        return None
    return {
        'goal_sequence': integer(row.get('goal_sequence')),
        'snapshot_density_sufficient': bool(row.get('snapshot_density_sufficient')),
        'pre_recovery_path_ahead_1_0m_cost_max': number(row.get('pre_recovery_path_ahead_1_0m_cost_max')),
        'post_recovery_path_ahead_1_0m_cost_max': number(row.get('post_recovery_path_ahead_1_0m_cost_max')),
        'near_zero_path_ahead_1_0m_cost_max': number(row.get('near_zero_path_ahead_1_0m_cost_max')),
        'near_zero_robot_to_path_distance_m': number(row.get('near_zero_robot_to_path_distance_m')),
        'path_update_count_after_recovery': integer(row.get('path_update_count_after_recovery')),
    }


def chosen_exit_progress_delta(dispatch: dict[str, Any]) -> float | None:
    robot = number(dispatch.get('robot_exit_dist_at_dispatch'))
    target = number(dispatch.get('target_exit_dist'))
    if robot is None or target is None:
        return None
    return round(robot - target, 6)


def route_context(dispatch: dict[str, Any], candidate_exit_delta: float | None) -> str:
    chosen_delta = chosen_exit_progress_delta(dispatch)
    if chosen_delta is not None and chosen_delta < -FORWARD_SHIFT_EPS_M:
        if candidate_exit_delta is not None and candidate_exit_delta < -FORWARD_SHIFT_EPS_M:
            return 'route_divergence_chosen_moves_away_rejected_moves_toward_exit'
        return 'chosen_moves_away_without_closer_explored_candidate'
    if chosen_delta is not None and chosen_delta > FORWARD_SHIFT_EPS_M:
        return 'non_divergent_chosen_also_toward_exit'
    return 'non_divergent_or_unknown_chosen_progress'


def failure_window_flags(timeout_subtype: dict[str, Any] | None, post_recovery: dict[str, Any] | None) -> dict[str, bool]:
    reasons = timeout_subtype.get('reasons', {}) if timeout_subtype else {}
    controller = str(timeout_subtype.get('controller_subtype', '')) if timeout_subtype else ''
    timing = str(timeout_subtype.get('timing_subtype', '')) if timeout_subtype else ''
    classification = str(timeout_subtype.get('classification', '')) if timeout_subtype else ''
    near_zero_cost = number(post_recovery.get('near_zero_path_ahead_1_0m_cost_max')) if post_recovery else None
    post_cost = number(post_recovery.get('post_recovery_path_ahead_1_0m_cost_max')) if post_recovery else None
    pre_cost = number(post_recovery.get('pre_recovery_path_ahead_1_0m_cost_max')) if post_recovery else None
    path_updates = integer(post_recovery.get('path_update_count_after_recovery')) if post_recovery else None
    return {
        'footprint_path_blocked': bool(reasons.get('footprint_path_blocked')) or 'footprint_path_blocked' in controller,
        'late_silent': 'late_silent' in controller or 'cmd_silent_after_recovery_abort' in timing or classification == 'healthy_motion_but_late_stall',
        'near_zero_path_ahead_high_cost': near_zero_cost is not None and near_zero_cost >= HIGH_COST_THRESHOLD,
        'post_recovery_path_ahead_high_cost': post_cost is not None and post_cost >= HIGH_COST_THRESHOLD,
        'pre_recovery_path_ahead_high_cost': pre_cost is not None and pre_cost >= HIGH_COST_THRESHOLD,
        'path_updates_after_recovery': path_updates is not None and path_updates > 0,
    }


def evidence_class(cost_class: str, route_ctx: str, outcome: dict[str, Any], flags: dict[str, bool]) -> str:
    divergent = route_ctx == 'route_divergence_chosen_moves_away_rejected_moves_toward_exit'
    timeout = outcome.get('event') == 'timeout'
    failure_window = any(flags.get(key) for key in ('footprint_path_blocked', 'late_silent', 'near_zero_path_ahead_high_cost'))
    if divergent and cost_class == 'clean_low_cost':
        return 'clean_route_divergence_candidate'
    if divergent and cost_class == 'high_local_cost' and timeout and failure_window:
        return 'route_divergence_high_cost_timeout_failure_window'
    if divergent and cost_class == 'high_local_cost':
        return 'route_divergence_high_cost_without_timeout_failure_window'
    if cost_class == 'high_local_cost':
        return 'harmless_or_non_divergent_high_cost'
    return 'harmless_or_non_divergent_clean'


def base_forward_shift_case(source: dict[str, Any], later: dict[str, Any], candidate: dict[str, Any], run_final_mode: str) -> dict[str, Any] | None:
    source_start = source.get('start_node_id')
    later_start = later.get('start_node_id')
    if source_start is None or later_start is None or source_start != later_start:
        return None
    d_angle = angle_delta(source.get('branch_angle'), candidate.get('branch_angle'))
    if d_angle is None or d_angle > ANGLE_MATCH_THRESHOLD_RAD:
        return None
    source_exit = source.get('target_exit_dist')
    candidate_exit = candidate.get('target_exit_dist')
    if source_exit is None or candidate_exit is None:
        return None
    shift_distance = distance(source.get('target'), candidate.get('target'))
    exit_delta = candidate_exit - source_exit
    return {
        'run_final_mode': run_final_mode,
        'start_node_id': source_start,
        'source_goal_sequence': source.get('goal_sequence'),
        'later_goal_sequence': later.get('goal_sequence'),
        'source_target': source.get('target'),
        'later_explored_candidate_target': candidate.get('target'),
        'source_target_exit_dist': round(source_exit, 6),
        'later_explored_candidate_target_exit_dist': round(candidate_exit, 6),
        'target_exit_dist_delta_m': round(exit_delta, 6),
        'closer_to_exit': exit_delta < -FORWARD_SHIFT_EPS_M,
        'target_forward_shift_distance_m': round(shift_distance, 6) if shift_distance is not None else None,
        'source_branch_angle': source.get('branch_angle'),
        'later_explored_candidate_branch_angle': candidate.get('branch_angle'),
        'angle_delta_rad': round(d_angle, 6),
        'cost_classification': cost_classification(candidate, later),
        'dispatch_path_local_cost_max': candidate.get('dispatch_path_local_cost_max'),
        'target_local_cost': candidate.get('target_local_cost'),
        'target_local_cost_max_radius': candidate.get('target_local_cost_max_radius'),
        'later_dispatch_path_local_cost_max': later.get('dispatch_path_local_cost_max'),
        'later_dispatch_target_local_cost': later.get('dispatch_target_local_cost'),
        'rejection_reason': candidate.get('rejection_reason'),
        'candidate_state': candidate.get('state'),
    }


def joined_case(
    source: dict[str, Any],
    later: dict[str, Any],
    candidate: dict[str, Any],
    run_final_mode: str,
    outcomes: dict[int, dict[str, Any]],
    timeout_subtypes: dict[int, dict[str, Any]],
    post_recovery: dict[int, dict[str, Any]],
) -> dict[str, Any] | None:
    base = base_forward_shift_case(source, later, candidate, run_final_mode)
    if base is None:
        return None
    later_seq = base.get('later_goal_sequence')
    later_outcome = compact_outcome(outcomes.get(later_seq or -1))
    timeout = compact_timeout_subtype(timeout_subtypes.get(later_seq or -1))
    recovery = compact_post_recovery(post_recovery.get(later_seq or -1))
    flags = failure_window_flags(timeout, recovery)
    ctx = route_context(later, base.get('target_exit_dist_delta_m'))
    base.update({
        'later_outcome': later_outcome,
        'timeout_subtype': timeout,
        'post_recovery': recovery,
        'failure_window_flags': flags,
        'chosen_exit_progress_delta_m': chosen_exit_progress_delta(later),
        'route_context': ctx,
        'evidence_class': evidence_class(base['cost_classification'], ctx, later_outcome, flags),
    })
    return base


def find_joined_cases(
    dispatches: list[dict[str, Any]],
    run_final_mode: str,
    outcomes: dict[int, dict[str, Any]],
    timeout_subtypes: dict[int, dict[str, Any]],
    post_recovery: dict[int, dict[str, Any]],
) -> list[dict[str, Any]]:
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
                case = joined_case(source, later, candidate, run_final_mode, outcomes, timeout_subtypes, post_recovery)
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


def summarize_cases(cases: list[dict[str, Any]], dispatch_count: int | None = None) -> dict[str, Any]:
    closer = [case for case in cases if case.get('closer_to_exit')]
    high_closer = [case for case in closer if case.get('cost_classification') == 'high_local_cost']
    clean_closer = [case for case in closer if case.get('cost_classification') == 'clean_low_cost']
    evidence_counts: dict[str, int] = {}
    for case in cases:
        evidence_counts[case.get('evidence_class', 'unknown')] = evidence_counts.get(case.get('evidence_class', 'unknown'), 0) + 1
    summary = {
        'same_start_explored_forward_shift_count': len(cases),
        'explored_closer_to_exit_forward_shift_count': len(closer),
        'clean_low_cost_explored_closer_count': len(clean_closer),
        'high_local_cost_explored_closer_count': len(high_closer),
        'route_divergence_case_count': sum(1 for case in cases if case.get('route_context') == 'route_divergence_chosen_moves_away_rejected_moves_toward_exit'),
        'clean_route_divergence_candidate_count': evidence_counts.get('clean_route_divergence_candidate', 0),
        'route_divergence_high_cost_timeout_failure_window_count': evidence_counts.get('route_divergence_high_cost_timeout_failure_window', 0),
        'harmless_or_non_divergent_high_cost_count': evidence_counts.get('harmless_or_non_divergent_high_cost', 0),
        'high_cost_closer_with_footprint_path_blocked_count': sum(1 for case in high_closer if case.get('failure_window_flags', {}).get('footprint_path_blocked')),
        'high_cost_closer_with_late_silent_count': sum(1 for case in high_closer if case.get('failure_window_flags', {}).get('late_silent')),
        'high_cost_closer_with_near_zero_path_high_cost_count': sum(1 for case in high_closer if case.get('failure_window_flags', {}).get('near_zero_path_ahead_high_cost')),
        'target_forward_shift_distance_m': numeric_summary([case.get('target_forward_shift_distance_m') for case in cases]),
        'target_exit_dist_delta_m': numeric_summary([case.get('target_exit_dist_delta_m') for case in cases]),
        'angle_delta_rad': numeric_summary([case.get('angle_delta_rad') for case in cases]),
        'evidence_class_counts': dict(sorted(evidence_counts.items())),
    }
    if dispatch_count is not None:
        summary['dispatch_count'] = dispatch_count
    return summary


def read_run(log_dir: Path, run_id: str) -> dict[str, Any]:
    payloads = load_jsonl_payloads(log_dir / f'{run_id}_goal_events.jsonl')
    dispatches = [normalize_dispatch(row) for row in payloads if row.get('event') == 'dispatch']
    outcomes = outcome_by_sequence(payloads)
    timeout_subtypes = load_timeout_subtypes(log_dir, run_id)
    post_recovery = load_post_recovery(log_dir, run_id)
    final_mode = read_final_mode(log_dir, run_id, payloads)
    cases = find_joined_cases(dispatches, final_mode, outcomes, timeout_subtypes, post_recovery)
    return {
        'run_id': run_id,
        'final_mode': final_mode,
        'goal_events_path': str(log_dir / f'{run_id}_goal_events.jsonl'),
        'timeout_subtypes_path': str(log_dir / f'{run_id}_timeout_subtypes.json'),
        'post_recovery_enriched_path': str(log_dir / f'{run_id}_post_recovery_enriched.json'),
        'joined_forward_shift_cases': cases,
        'summary': summarize_cases(cases, len(dispatches)),
    }


def aggregate_group(run_ids: list[str], runs: dict[str, dict[str, Any]]) -> dict[str, Any]:
    selected = [runs[run_id] for run_id in run_ids if run_id in runs]
    cases = [case for run in selected for case in run.get('joined_forward_shift_cases', [])]
    summary = summarize_cases(cases)
    summary.update({
        'run_ids': run_ids,
        'run_count': len(selected),
        'final_modes': {run['run_id']: run.get('final_mode') for run in selected},
    })
    return summary


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
    failed_key = 'failed_runs' if failed_ids else 'FAILED_EXHAUSTED'
    success_key = 'success_runs' if success_ids else 'EXIT_REACHED'
    failed = comparison.get(failed_key, {})
    success = comparison.get(success_key, {})
    clean_route = int(failed.get('clean_route_divergence_candidate_count', 0) or 0)
    success_clean_route = int(success.get('clean_route_divergence_candidate_count', 0) or 0)
    high_failure = int(failed.get('route_divergence_high_cost_timeout_failure_window_count', 0) or 0)
    high_overlap = high_failure > 0 or int(failed.get('high_cost_closer_with_footprint_path_blocked_count', 0) or 0) > 0 or int(failed.get('high_cost_closer_with_near_zero_path_high_cost_count', 0) or 0) > 0
    possible_clean_signal = clean_route > 0 and success_clean_route == 0
    return {
        'phase27_candidate_signal': 'possible_only_after_matched_repeat_validation' if possible_clean_signal else 'not_supported',
        'high_cost_failure_window_overlap_in_failed_runs': high_overlap,
        'clean_route_divergence_failed_only_signal': possible_clean_signal,
        'recommendation': 'analysis_only_join_more_or_collect_matched_repeats' if possible_clean_signal else 'analysis_only_no_phase27_no_runtime_change',
        'guardrails': [
            'do_not_enter_phase27',
            'do_not_change_branch_selection',
            'do_not_tune_nav2_or_controller_params_from_phase26j',
            'require_clean_low_cost_failed_only_route_divergence_before_phase27',
        ],
    }


def analyze(log_dir: Path, run_ids: list[str], failed_ids: list[str], success_ids: list[str]) -> dict[str, Any]:
    all_ids: list[str] = []
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
        'phase': '26J',
        'purpose': 'join same-start explored forward-shift cases with outcome, timeout subtype, post-recovery, and route-divergence context',
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
