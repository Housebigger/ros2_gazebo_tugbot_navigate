#!/usr/bin/env python3
"""Phase26F post-run branch-choice diagnostics analyzer.

Analysis-only: consumes goal_events artifacts with Phase26E candidate branch
payloads. It does not start ROS/Gazebo, does not edit Nav2 params, and does not
make branch-selection changes.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

LOCAL_COST_CONSTRAINED_THRESHOLD = 70.0
DIVERGENCE_EPS_M = 0.05


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


def target_payload(value: Any) -> list[float] | None:
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        x = number(value[0])
        y = number(value[1])
        if x is not None and y is not None:
            return [round(x, 6), round(y, 6)]
    return None


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


def is_rejected_candidate(candidate: dict[str, Any]) -> bool:
    reason = candidate.get('rejection_reason')
    if reason not in (None, '', 'selected'):
        return True
    rank = integer(candidate.get('rank'))
    chosen_rank = integer(candidate.get('chosen_branch_rank'))
    if rank is not None and chosen_rank is not None:
        return rank != chosen_rank
    return False


def best_rejected_candidate(dispatch_row: dict[str, Any], candidates: list[dict[str, Any]]) -> dict[str, Any] | None:
    rejected = [candidate for candidate in candidates if candidate.get('rejection_reason') not in (None, '', 'selected')]
    if not rejected:
        return None
    def key(candidate: dict[str, Any]) -> float:
        delta = candidate.get('exit_progress_delta_m')
        return float(delta) if delta is not None else float('-inf')
    return max(rejected, key=key)


def context_flags(row: dict[str, Any], candidates: list[dict[str, Any]]) -> dict[str, bool]:
    costs = [
        number(row.get('dispatch_path_local_cost_max')),
        number(row.get('dispatch_target_local_cost')),
        number(row.get('dispatch_target_local_cost_max_radius')),
    ]
    for candidate in candidates:
        costs.extend([
            number(candidate.get('dispatch_path_local_cost_max')),
            number(candidate.get('target_local_cost')),
            number(candidate.get('target_local_cost_max_radius')),
        ])
    rejection_reasons = [str(candidate.get('rejection_reason')) for candidate in candidates if candidate.get('rejection_reason') not in (None, '')]
    return {
        'reverse': any(bool(candidate.get('is_reverse_candidate')) for candidate in candidates),
        'backtrack': bool(row.get('goal_kind') == 'backtrack') or any(bool(candidate.get('is_backtrack_context')) for candidate in candidates),
        'near_exit': bool(row.get('near_exit')) or any(bool(candidate.get('is_near_exit_candidate')) for candidate in candidates),
        'local_cost_constrained': any(value is not None and value >= LOCAL_COST_CONSTRAINED_THRESHOLD for value in costs),
        'blacklisted_or_rejected': any(reason not in ('None', '') for reason in rejection_reasons),
        'blacklisted': any('blacklist' in reason.lower() for reason in rejection_reasons),
    }


def context_labels(flags: dict[str, bool]) -> list[str]:
    labels = []
    if flags.get('reverse'):
        labels.append('reverse')
    if flags.get('backtrack'):
        labels.append('backtrack')
    if flags.get('near_exit'):
        labels.append('near_exit')
    if flags.get('local_cost_constrained'):
        labels.append('local_cost_constrained')
    else:
        labels.append('local_cost_clear')
    if flags.get('blacklisted'):
        labels.append('blacklisted')
    elif flags.get('blacklisted_or_rejected'):
        labels.append('rejected_candidate_context')
    return labels


def classify_dispatch(row: dict[str, Any], chosen_delta: float | None, best_rejected: dict[str, Any] | None) -> str:
    best_rejected_delta = best_rejected.get('exit_progress_delta_m') if best_rejected else None
    if chosen_delta is not None and chosen_delta < -DIVERGENCE_EPS_M:
        if best_rejected_delta is not None and best_rejected_delta > DIVERGENCE_EPS_M:
            return 'chosen_moves_away_rejected_moves_toward_exit'
        return 'chosen_moves_away_no_better_rejected_candidate'
    if chosen_delta is not None and chosen_delta > DIVERGENCE_EPS_M:
        return 'chosen_moves_toward_exit'
    return 'chosen_exit_neutral_or_unknown'


def dispatch_summary(row: dict[str, Any]) -> dict[str, Any]:
    raw_candidates = row.get('candidate_branches') or []
    candidates = [candidate_summary(candidate, row) for candidate in raw_candidates if isinstance(candidate, dict)]
    chosen_delta = exit_progress_delta(row)
    best_rejected = best_rejected_candidate(row, candidates)
    flags = context_flags(row, candidates)
    classification = classify_dispatch(row, chosen_delta, best_rejected)
    return {
        'goal_sequence': row.get('goal_sequence'),
        'goal_kind': row.get('goal_kind'),
        'chosen_branch_rank': integer(row.get('chosen_branch_rank')),
        'candidate_branch_count': integer(row.get('candidate_branch_count')) if row.get('candidate_branch_count') is not None else len(candidates),
        'chosen_exit_progress_delta_m': chosen_delta,
        'robot_exit_dist_at_dispatch': number(row.get('robot_exit_dist_at_dispatch')),
        'chosen_target_exit_dist': number(row.get('target_exit_dist')),
        'chosen_target': target_payload(row.get('target')),
        'chosen_branch_angle': number(row.get('branch_angle')),
        'selected_due_to_context': row.get('selected_due_to_context'),
        'best_rejected_exit_progress_delta_m': best_rejected.get('exit_progress_delta_m') if best_rejected else None,
        'best_rejected_target_exit_dist': best_rejected.get('target_exit_dist') if best_rejected else None,
        'best_rejected_rejection_reason': best_rejected.get('rejection_reason') if best_rejected else None,
        'best_rejected_branch_angle': best_rejected.get('branch_angle') if best_rejected else None,
        'context_flags': flags,
        'contexts': context_labels(flags),
        'classification': classification,
        'candidate_branches': candidates,
    }


def read_final_mode(log_dir: Path, run_id: str, payloads: list[dict[str, Any]]) -> str:
    state_path = log_dir / f'{run_id}_explorer_state.jsonl'
    if state_path.exists() and state_path.stat().st_size:
        final = None
        for row in load_jsonl_payloads(state_path):
            if 'mode' in row:
                final = row.get('mode')
        if final:
            return str(final)
    if any(row.get('event') == 'terminal_cancel' and row.get('result_reason') == 'exit_reached' for row in payloads):
        return 'EXIT_REACHED'
    return 'UNKNOWN'


def read_run(log_dir: Path, run_id: str) -> dict[str, Any]:
    path = log_dir / f'{run_id}_goal_events.jsonl'
    payloads = load_jsonl_payloads(path)
    dispatches = [dispatch_summary(row) for row in payloads if row.get('event') == 'dispatch']
    route_cases = [row for row in dispatches if row.get('classification') == 'chosen_moves_away_rejected_moves_toward_exit']
    missing_candidate_diag = [row.get('goal_sequence') for row in payloads if row.get('event') == 'dispatch' and 'candidate_branches' not in row]
    final_mode = read_final_mode(log_dir, run_id, payloads)
    return {
        'run_id': run_id,
        'goal_events_path': str(path),
        'final_mode': final_mode,
        'dispatch_summaries': dispatches,
        'route_divergence_cases': route_cases,
        'summary': {
            'dispatch_count': len(dispatches),
            'route_divergence_case_count': len(route_cases),
            'chosen_moves_away_count': sum(1 for row in dispatches if row.get('classification', '').startswith('chosen_moves_away')),
            'chosen_moves_toward_exit_count': sum(1 for row in dispatches if row.get('classification') == 'chosen_moves_toward_exit'),
            'near_exit_dispatch_count': sum(1 for row in dispatches if row.get('context_flags', {}).get('near_exit')),
            'reverse_context_count': sum(1 for row in dispatches if row.get('context_flags', {}).get('reverse')),
            'backtrack_context_count': sum(1 for row in dispatches if row.get('context_flags', {}).get('backtrack')),
            'local_cost_constrained_count': sum(1 for row in dispatches if row.get('context_flags', {}).get('local_cost_constrained')),
            'blacklisted_or_rejected_context_count': sum(1 for row in dispatches if row.get('context_flags', {}).get('blacklisted_or_rejected')),
            'missing_candidate_diagnostics_count': len(missing_candidate_diag),
        },
        'artifact_gaps': {
            'dispatch_sequences_missing_candidate_branches': missing_candidate_diag,
        },
    }


def group_summary(run_ids: list[str], runs: dict[str, dict[str, Any]]) -> dict[str, Any]:
    selected = [runs[run_id] for run_id in run_ids if run_id in runs]
    return {
        'run_ids': run_ids,
        'run_count': len(selected),
        'dispatch_count': sum(run['summary']['dispatch_count'] for run in selected),
        'route_divergence_case_count': sum(run['summary']['route_divergence_case_count'] for run in selected),
        'chosen_moves_away_count': sum(run['summary']['chosen_moves_away_count'] for run in selected),
        'chosen_moves_toward_exit_count': sum(run['summary']['chosen_moves_toward_exit_count'] for run in selected),
        'final_modes': {run['run_id']: run.get('final_mode') for run in selected},
    }


def split_runs(value: str | None) -> list[str]:
    if not value:
        return []
    return [item.strip() for item in value.split(',') if item.strip()]


def analyze(log_dir: Path, baseline_runs: list[str], candidate_runs: list[str], extra_runs: list[str]) -> dict[str, Any]:
    all_run_ids = []
    for run_id in baseline_runs + candidate_runs + extra_runs:
        if run_id not in all_run_ids:
            all_run_ids.append(run_id)
    runs = {run_id: read_run(log_dir, run_id) for run_id in all_run_ids}
    baseline = group_summary(baseline_runs, runs)
    candidate = group_summary(candidate_runs, runs)
    stable_evidence = False
    if baseline['run_count'] >= 2 and candidate['run_count'] >= 2:
        stable_evidence = False
    recommendation = 'analysis_only_collect_more_or_repeat'
    return {
        'phase': '26F',
        'purpose': 'post-run branch-choice diagnostics analysis from Phase26E candidate branch payloads',
        'analysis_only': True,
        'matched_groups': {
            'baseline': baseline,
            'candidate': candidate,
            'extra': group_summary(extra_runs, runs),
        },
        'matched_comparison': {
            'baseline': baseline,
            'candidate': candidate,
        },
        'runs': runs,
        'decision': {
            'stable_route_divergence_evidence': stable_evidence,
            'recommendation': recommendation,
            'guardrails': [
                'do_not_enter_phase27',
                'do_not_change_branch_selection',
                'do_not_tune_nav2_or_controller_params_from_phase26f',
                'require_matched_repeats_before_intervention',
            ],
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, default=Path('log'))
    parser.add_argument('--baseline-runs', default='')
    parser.add_argument('--candidate-runs', default='')
    parser.add_argument('--extra-runs', default='')
    parser.add_argument('--output-json', type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    payload = analyze(
        args.log_dir,
        baseline_runs=split_runs(args.baseline_runs),
        candidate_runs=split_runs(args.candidate_runs),
        extra_runs=split_runs(args.extra_runs),
    )
    text = json.dumps(payload, indent=2, sort_keys=True) + '\n'
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(text, encoding='utf-8')
    print(text)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
