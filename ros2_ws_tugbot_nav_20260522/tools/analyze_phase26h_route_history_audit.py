#!/usr/bin/env python3
"""Phase26H route-history audit for explored near-exit branch candidates.

Analysis-only: consumes existing Phase26E/F/G artifacts. It does not start ROS,
Gazebo, Nav2, or recorders; it does not edit params or branch-selection logic.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
from typing import Any

LOCAL_COST_CONSTRAINED_THRESHOLD = 70.0
DIVERGENCE_EPS_M = 0.05
TARGET_MATCH_RADIUS_M = 0.65


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


def exit_progress_delta(row: dict[str, Any]) -> float | None:
    direct = number(row.get('exit_progress_delta_m'))
    if direct is not None:
        return round(direct, 6)
    robot = number(row.get('robot_exit_dist_at_dispatch'))
    target = number(row.get('target_exit_dist'))
    if robot is None or target is None:
        return None
    return round(robot - target, 6)


def candidate_exit_progress(candidate: dict[str, Any], dispatch_row: dict[str, Any]) -> float | None:
    direct = number(candidate.get('exit_progress_delta_m'))
    if direct is not None:
        return round(direct, 6)
    robot = number(dispatch_row.get('robot_exit_dist_at_dispatch'))
    target = number(candidate.get('target_exit_dist'))
    if robot is None or target is None:
        return None
    return round(robot - target, 6)


def candidate_summary(candidate: dict[str, Any], dispatch_row: dict[str, Any]) -> dict[str, Any]:
    return {
        'branch_angle': number(candidate.get('branch_angle')),
        'target': target_payload(candidate.get('target')),
        'target_exit_dist': number(candidate.get('target_exit_dist')),
        'exit_progress_delta_m': candidate_exit_progress(candidate, dispatch_row),
        'target_clearance_m': number(candidate.get('target_clearance_m')),
        'path_corridor_min_clearance_m': number(candidate.get('path_corridor_min_clearance_m')),
        'dispatch_path_local_cost_max': number(candidate.get('dispatch_path_local_cost_max')),
        'dispatch_path_local_cost_mean': number(candidate.get('dispatch_path_local_cost_mean')),
        'target_local_cost': number(candidate.get('target_local_cost')),
        'target_local_cost_max_radius': number(candidate.get('target_local_cost_max_radius')),
        'is_reverse_candidate': bool(candidate.get('is_reverse_candidate')),
        'is_backtrack_context': bool(candidate.get('is_backtrack_context')),
        'is_near_exit_candidate': bool(candidate.get('is_near_exit_candidate')),
        'rejection_reason': candidate.get('rejection_reason'),
        'rank': integer(candidate.get('rank')),
        'state': candidate.get('state'),
    }


def local_cost_constrained(candidate: dict[str, Any], dispatch_row: dict[str, Any] | None = None) -> bool:
    values = [
        number(candidate.get('dispatch_path_local_cost_max')),
        number(candidate.get('target_local_cost')),
        number(candidate.get('target_local_cost_max_radius')),
    ]
    if dispatch_row is not None:
        values.extend([
            number(dispatch_row.get('dispatch_path_local_cost_max')),
            number(dispatch_row.get('dispatch_target_local_cost')),
            number(dispatch_row.get('dispatch_target_local_cost_max_radius')),
        ])
    return any(value is not None and value >= LOCAL_COST_CONSTRAINED_THRESHOLD for value in values)


def best_rejected_candidate(candidates: list[dict[str, Any]]) -> dict[str, Any] | None:
    rejected = [candidate for candidate in candidates if candidate.get('rejection_reason') not in (None, '', 'selected')]
    if not rejected:
        return None
    def key(candidate: dict[str, Any]) -> float:
        value = number(candidate.get('exit_progress_delta_m'))
        return value if value is not None else float('-inf')
    return max(rejected, key=key)


def classify_dispatch(chosen_delta: float | None, best_rejected: dict[str, Any] | None) -> str:
    best_rejected_delta = best_rejected.get('exit_progress_delta_m') if best_rejected else None
    if chosen_delta is not None and chosen_delta < -DIVERGENCE_EPS_M:
        if best_rejected_delta is not None and best_rejected_delta > DIVERGENCE_EPS_M:
            return 'chosen_moves_away_rejected_moves_toward_exit'
        return 'chosen_moves_away_no_better_rejected_candidate'
    if chosen_delta is not None and chosen_delta > DIVERGENCE_EPS_M:
        return 'chosen_moves_toward_exit'
    return 'chosen_exit_neutral_or_unknown'


def dispatch_summary(row: dict[str, Any]) -> dict[str, Any]:
    candidates = [candidate_summary(candidate, row) for candidate in row.get('candidate_branches') or [] if isinstance(candidate, dict)]
    chosen_delta = exit_progress_delta(row)
    best_rejected = best_rejected_candidate(candidates)
    return {
        'goal_sequence': row.get('goal_sequence'),
        'goal_kind': row.get('goal_kind'),
        'start_node_id': integer(row.get('start_node_id')),
        'current_node_id': integer(row.get('current_node_id')),
        'dispatch_pose': target_payload(row.get('dispatch_pose')),
        'target': target_payload(row.get('target')),
        'branch_angle': number(row.get('branch_angle')),
        'target_exit_dist': number(row.get('target_exit_dist')),
        'robot_exit_dist_at_dispatch': number(row.get('robot_exit_dist_at_dispatch')),
        'chosen_branch_rank': integer(row.get('chosen_branch_rank')),
        'candidate_branch_count': integer(row.get('candidate_branch_count')) if row.get('candidate_branch_count') is not None else len(candidates),
        'candidate_branches': candidates,
        'selected_due_to_context': row.get('selected_due_to_context'),
        'chosen_exit_progress_delta_m': chosen_delta,
        'classification': classify_dispatch(chosen_delta, best_rejected),
        'best_rejected_rejection_reason': best_rejected.get('rejection_reason') if best_rejected else None,
        'best_rejected_target_exit_dist': best_rejected.get('target_exit_dist') if best_rejected else None,
        'best_rejected_exit_progress_delta_m': best_rejected.get('exit_progress_delta_m') if best_rejected else None,
        'best_rejected_target': best_rejected.get('target') if best_rejected else None,
        'near_exit_local_cost_explored_candidates': [
            candidate for candidate in candidates
            if candidate.get('rejection_reason') == 'explored'
            and candidate.get('is_near_exit_candidate')
            and local_cost_constrained(candidate, row)
        ],
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


def load_post_recovery(log_dir: Path, run_id: str) -> dict[int, dict[str, Any]]:
    data = load_json(log_dir / f'{run_id}_post_recovery_enriched.json', {})
    rows = data.get('enriched_recovery_snapshots', []) if isinstance(data, dict) else []
    by_seq: dict[int, list[dict[str, Any]]] = {}
    for row in rows:
        seq = integer(row.get('goal_sequence'))
        if seq is not None:
            by_seq.setdefault(seq, []).append(row)
    result = {}
    for seq, items in by_seq.items():
        result[seq] = max(items, key=lambda row: number(row.get('path_update_count_after_recovery')) or 0)
    return result


def provenance_for_candidate(candidate: dict[str, Any], dispatches: list[dict[str, Any]], outcomes: dict[int, dict[str, Any]], current_seq: int | None) -> dict[str, Any]:
    target = candidate.get('target')
    best: tuple[float, dict[str, Any]] | None = None
    for prior in dispatches:
        seq = integer(prior.get('goal_sequence'))
        if current_seq is not None and seq is not None and seq >= current_seq:
            continue
        prior_target = prior.get('target')
        d = distance(target, prior_target)
        if d is None or d > TARGET_MATCH_RADIUS_M:
            continue
        if best is None or d < best[0]:
            best = (d, prior)
    if best is None:
        current_dispatch = next((row for row in dispatches if integer(row.get('goal_sequence')) == current_seq), None)
        current_start_node_id = integer(current_dispatch.get('start_node_id')) if current_dispatch else None
        candidate_angle = number(candidate.get('branch_angle'))
        angle_best: tuple[float, dict[str, Any]] | None = None
        if current_start_node_id is not None and candidate_angle is not None:
            for prior in dispatches:
                seq = integer(prior.get('goal_sequence'))
                if current_seq is not None and seq is not None and seq >= current_seq:
                    continue
                if integer(prior.get('start_node_id')) != current_start_node_id:
                    continue
                prior_angle = number(prior.get('branch_angle'))
                if prior_angle is None:
                    continue
                delta = abs(math.atan2(math.sin(candidate_angle - prior_angle), math.cos(candidate_angle - prior_angle)))
                if delta <= math.radians(20.0) and (angle_best is None or delta < angle_best[0]):
                    angle_best = (delta, prior)
        if angle_best is None:
            return {
                'classification': 'unknown_no_prior_matching_dispatch',
                'source_goal_sequence': None,
                'source_outcome': None,
                'target_match_distance_m': None,
            }
        angle_delta, prior = angle_best
        source_seq = integer(prior.get('goal_sequence'))
        outcome: dict[str, Any] = {}
        if source_seq is not None:
            outcome = outcomes.get(source_seq, {})
        event = outcome.get('event')
        if event == 'success':
            classification = 'prior_success_same_start_node_angle_match'
        else:
            classification = f"prior_{event or 'dispatch'}_same_start_node_angle_match"
        return {
            'classification': classification,
            'source_goal_sequence': source_seq,
            'source_outcome': event,
            'source_result_reason': outcome.get('result_reason'),
            'source_goal_kind': prior.get('goal_kind'),
            'source_target': prior.get('target'),
            'source_target_exit_dist': prior.get('target_exit_dist'),
            'target_match_distance_m': distance(target, prior.get('target')),
            'angle_delta_rad': round(angle_delta, 6),
            'match_basis': 'same_start_node_branch_angle',
        }
    match_distance, prior = best
    source_seq = integer(prior.get('goal_sequence'))
    outcome: dict[str, Any] = {}
    if source_seq is not None:
        outcome = outcomes.get(source_seq, {})
    event = outcome.get('event')
    result_reason = outcome.get('result_reason')
    goal_kind = prior.get('goal_kind')
    if event == 'success' and goal_kind == 'explore':
        classification = 'prior_success_near_same_target'
    elif event == 'success' and goal_kind == 'backtrack':
        classification = 'prior_backtrack_success_near_same_target'
    elif event == 'timeout':
        classification = 'prior_timeout_near_same_target'
    elif event in ('terminal_cancel', 'terminal_cancel_result'):
        classification = 'prior_terminal_cancel_near_same_target'
    elif event is None:
        classification = 'prior_dispatch_without_recorded_outcome'
    else:
        classification = f'prior_{event}_near_same_target'
    return {
        'classification': classification,
        'source_goal_sequence': source_seq,
        'source_outcome': event,
        'source_result_reason': result_reason,
        'source_goal_kind': goal_kind,
        'source_target': prior.get('target'),
        'source_target_exit_dist': prior.get('target_exit_dist'),
        'target_match_distance_m': round(match_distance, 6),
    }


def build_explored_case(
    dispatch: dict[str, Any],
    dispatches: list[dict[str, Any]],
    outcomes: dict[int, dict[str, Any]],
    timeout_subtypes: dict[int, dict[str, Any]],
    post_recovery: dict[int, dict[str, Any]],
) -> dict[str, Any] | None:
    explored = [candidate for candidate in dispatch.get('candidate_branches', []) if candidate.get('rejection_reason') == 'explored']
    if not explored:
        return None
    seq = integer(dispatch.get('goal_sequence'))
    enriched = []
    for candidate in explored:
        enriched.append({
            **candidate,
            'near_exit_local_cost_constrained': bool(candidate.get('is_near_exit_candidate') and local_cost_constrained(candidate)),
            'provenance': provenance_for_candidate(candidate, dispatches, outcomes, seq),
        })
    return {
        'goal_sequence': seq,
        'route_divergence': dispatch.get('classification') == 'chosen_moves_away_rejected_moves_toward_exit',
        'route_divergence_case': dispatch,
        'explored_rejected_candidates': enriched,
        'joined_timeout_subtype': timeout_subtypes.get(seq),
        'joined_post_recovery': post_recovery.get(seq),
    }


def read_run(log_dir: Path, run_id: str) -> dict[str, Any]:
    payloads = load_jsonl_payloads(log_dir / f'{run_id}_goal_events.jsonl')
    dispatches = [dispatch_summary(row) for row in payloads if row.get('event') == 'dispatch']
    outcomes = outcome_by_sequence(payloads)
    timeout_subtypes = load_timeout_subtypes(log_dir, run_id)
    post_recovery = load_post_recovery(log_dir, run_id)
    rejection_counts: Counter[str] = Counter()
    for dispatch in dispatches:
        for candidate in dispatch.get('candidate_branches', []):
            reason = candidate.get('rejection_reason')
            if reason not in (None, '', 'selected'):
                rejection_counts[str(reason)] += 1
    explored_cases = []
    for dispatch in dispatches:
        case = build_explored_case(dispatch, dispatches, outcomes, timeout_subtypes, post_recovery)
        if case is not None:
            explored_cases.append(case)
    near_exit_local_cost_count = sum(
        1
        for dispatch in dispatches
        for candidate in dispatch.get('candidate_branches', [])
        if candidate.get('rejection_reason') == 'explored'
        and candidate.get('is_near_exit_candidate')
        and local_cost_constrained(candidate)
    )
    unknown_provenance_count = sum(
        1
        for case in explored_cases
        for candidate in case.get('explored_rejected_candidates', [])
        if candidate.get('provenance', {}).get('classification') == 'unknown_no_prior_matching_dispatch'
    )
    normalized_rejection_counts = dict(sorted(rejection_counts.items()))
    for expected_reason in ('lower_rank_not_selected', 'explored', 'blacklisted'):
        normalized_rejection_counts.setdefault(expected_reason, 0)
    return {
        'run_id': run_id,
        'final_mode': read_final_mode(log_dir, run_id, payloads),
        'dispatch_summaries': dispatches,
        'explored_route_history_cases': explored_cases,
        'summary': {
            'dispatch_count': len(dispatches),
            'rejection_reason_counts': normalized_rejection_counts,
            'explored_rejected_candidate_count': rejection_counts.get('explored', 0),
            'lower_rank_not_selected_count': rejection_counts.get('lower_rank_not_selected', 0),
            'blacklisted_rejected_candidate_count': sum(count for reason, count in rejection_counts.items() if 'blacklist' in reason.lower()),
            'near_exit_local_cost_explored_candidate_count': near_exit_local_cost_count,
            'route_divergence_explored_case_count': sum(1 for case in explored_cases if case.get('route_divergence')),
            'unknown_explored_provenance_count': unknown_provenance_count,
        },
    }


def split_runs(value: str | None) -> list[str]:
    if not value:
        return []
    return [item.strip() for item in value.split(',') if item.strip()]


def focus_audit(runs: dict[str, dict[str, Any]], focus_run: str | None, focus_goal_sequence: int | None) -> dict[str, Any] | None:
    if not focus_run or focus_goal_sequence is None or focus_run not in runs:
        return None
    run = runs[focus_run]
    for case in run.get('explored_route_history_cases', []):
        if case.get('goal_sequence') == focus_goal_sequence:
            return {
                'run_id': focus_run,
                'goal_sequence': focus_goal_sequence,
                'route_divergence_case': case.get('route_divergence_case'),
                'explored_rejected_candidates': case.get('explored_rejected_candidates', []),
                'joined_timeout_subtype': case.get('joined_timeout_subtype'),
                'joined_post_recovery': case.get('joined_post_recovery'),
                'recommendation': 'do_not_intervene_audit_more_route_history',
            }
    return {
        'run_id': focus_run,
        'goal_sequence': focus_goal_sequence,
        'error': 'focus_goal_sequence_not_found_or_no_explored_rejected_candidate',
        'recommendation': 'do_not_intervene_audit_more_route_history',
    }


def analyze(log_dir: Path, run_ids: list[str], focus_run: str | None, focus_goal_sequence: int | None) -> dict[str, Any]:
    runs = {run_id: read_run(log_dir, run_id) for run_id in run_ids}
    total_explored = sum(run['summary']['explored_rejected_candidate_count'] for run in runs.values())
    total_near_exit_local_cost = sum(run['summary']['near_exit_local_cost_explored_candidate_count'] for run in runs.values())
    total_route_divergence_explored = sum(run['summary']['route_divergence_explored_case_count'] for run in runs.values())
    return {
        'phase': '26H',
        'purpose': 'route-history audit for explored near-exit rejected candidates with timeout/post-recovery joins',
        'analysis_only': True,
        'runs': runs,
        'focus_audit': focus_audit(runs, focus_run, focus_goal_sequence),
        'summary': {
            'run_count': len(runs),
            'explored_rejected_candidate_count': total_explored,
            'near_exit_local_cost_explored_candidate_count': total_near_exit_local_cost,
            'route_divergence_explored_case_count': total_route_divergence_explored,
        },
        'decision': {
            'stable_route_history_evidence_for_phase27': False,
            'recommendation': 'analysis_only_route_history_audit_before_intervention',
            'guardrails': [
                'do_not_enter_phase27',
                'do_not_change_branch_selection',
                'do_not_tune_nav2_or_controller_params_from_phase26h',
                'audit_explored_state_provenance_before_runtime_intervention',
            ],
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, default=Path('log'))
    parser.add_argument('--runs', required=True, help='Comma-separated run IDs to audit')
    parser.add_argument('--focus-run', default=None)
    parser.add_argument('--focus-goal-sequence', type=int, default=None)
    parser.add_argument('--output-json', type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    payload = analyze(args.log_dir, split_runs(args.runs), args.focus_run, args.focus_goal_sequence)
    text = json.dumps(payload, indent=2, sort_keys=True) + '\n'
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(text, encoding='utf-8')
    print(text)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
