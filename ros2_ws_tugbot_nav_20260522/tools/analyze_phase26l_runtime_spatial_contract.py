#!/usr/bin/env python3
"""Phase26L runtime spatial diagnostics contract analyzer.

Analysis-only: consumes future-style goal_events and post_recovery_snapshots
artifacts. It does not start ROS/Gazebo/Nav2 and does not change navigation
behavior. Its purpose is to verify whether the Phase26L recorder contract is
complete enough to support later branch-selection discussion.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
from typing import Any

HIGH_COST_THRESHOLD = 70.0
CLEAN_CORRIDOR_MAX_COST = 60.0


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


def target_xy(value: Any) -> list[float] | None:
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        x = number(value[0])
        y = number(value[1])
        if x is not None and y is not None:
            return [round(x, 6), round(y, 6)]
    return None


def selected_explored_candidate(dispatch: dict[str, Any]) -> dict[str, Any] | None:
    candidates = [
        candidate for candidate in dispatch.get('candidate_branches') or []
        if isinstance(candidate, dict) and candidate.get('rejection_reason') == 'explored'
    ]
    if not candidates:
        return None

    def key(candidate: dict[str, Any]) -> float:
        value = number(candidate.get('target_exit_dist'))
        return value if value is not None else float('inf')

    return min(candidates, key=key)


def route_context(dispatch: dict[str, Any], explored: dict[str, Any] | None) -> str:
    chosen = number(dispatch.get('target_exit_dist'))
    robot = number(dispatch.get('robot_exit_dist_at_dispatch'))
    explored_dist = number(explored.get('target_exit_dist')) if explored else None
    if chosen is None or robot is None or explored_dist is None:
        return 'unknown'
    if chosen > robot and explored_dist < robot:
        return 'route_divergence_chosen_moves_away_rejected_moves_toward_exit'
    if chosen <= robot:
        return 'non_divergent_chosen_also_toward_exit'
    return 'route_divergence_without_cleaner_explored_candidate'


def load_dispatches(log_dir: Path, run_id: str) -> dict[int, dict[str, Any]]:
    dispatches: dict[int, dict[str, Any]] = {}
    for row in load_jsonl_payloads(log_dir / f'{run_id}_goal_events.jsonl'):
        if row.get('event') == 'dispatch':
            seq = number(row.get('goal_sequence'))
            if seq is not None:
                dispatches[int(seq)] = row
    return dispatches


def load_outcomes(log_dir: Path, run_id: str) -> dict[int, dict[str, Any]]:
    outcomes: dict[int, dict[str, Any]] = {}
    for row in load_jsonl_payloads(log_dir / f'{run_id}_goal_events.jsonl'):
        if row.get('event') in ('success', 'timeout', 'failure', 'terminal_cancel', 'terminal_cancel_result'):
            seq = number(row.get('goal_sequence'))
            if seq is not None:
                outcomes[int(seq)] = {'event': row.get('event'), 'result_reason': row.get('result_reason')}
    return outcomes


def load_snapshots(log_dir: Path, run_id: str) -> dict[int, list[dict[str, Any]]]:
    by_seq: dict[int, list[dict[str, Any]]] = {}
    for row in load_jsonl_payloads(log_dir / f'{run_id}_post_recovery_snapshots.jsonl'):
        if row.get('event') != 'snapshot':
            continue
        seq = number(row.get('goal_sequence'))
        if seq is None:
            continue
        by_seq.setdefault(int(seq), []).append(row)
    return by_seq


def best_spatial_snapshot(rows: list[dict[str, Any]]) -> dict[str, Any] | None:
    preferred = [row for row in rows if row.get('snapshot_type') == 'near_zero_onset']
    candidates = preferred or rows
    if not candidates:
        return None

    def key(row: dict[str, Any]) -> tuple[int, float]:
        has_cells = 1 if int(number(row.get('path_ahead_1_0m_high_cost_count')) or 0) > 0 else 0
        cost = number(row.get('path_ahead_1_0m_cost_max')) or 0.0
        return (has_cells, cost)

    return max(candidates, key=key)


def has_point(value: Any) -> bool:
    return target_xy(value) is not None


def classify_case(dispatch: dict[str, Any], outcome: dict[str, Any] | None, snapshot: dict[str, Any] | None) -> dict[str, Any]:
    explored = selected_explored_candidate(dispatch)
    context = route_context(dispatch, explored)
    snapshot = snapshot or {}
    high_count = int(number(snapshot.get('path_ahead_1_0m_high_cost_count')) or 0)
    chosen_max = number(snapshot.get('chosen_route_corridor_cost_max'))
    explored_max = number(snapshot.get('explored_candidate_corridor_cost_max'))
    chosen_first = number(snapshot.get('chosen_route_first_high_cost_distance_m'))
    explored_first = number(snapshot.get('explored_candidate_first_high_cost_distance_m'))
    required_spatial_fields = [
        'path_ahead_1_0m_high_cost_count',
        'path_ahead_1_0m_nearest_high_cost_point',
        'path_ahead_1_0m_high_cost_centroid',
        'path_ahead_1_0m_first_high_cost_distance_m',
        'chosen_route_corridor_cost_max',
        'chosen_route_corridor_cost_mean',
        'chosen_route_first_high_cost_distance_m',
        'explored_candidate_corridor_cost_max',
        'explored_candidate_corridor_cost_mean',
    ]
    spatial_complete = all(field in snapshot for field in required_spatial_fields) and has_point(snapshot.get('path_ahead_1_0m_nearest_high_cost_point')) and has_point(snapshot.get('path_ahead_1_0m_high_cost_centroid'))
    path_cells_present = high_count > 0 and has_point(snapshot.get('path_ahead_1_0m_nearest_high_cost_point'))
    chosen_high = chosen_max is not None and chosen_max >= HIGH_COST_THRESHOLD and chosen_first is not None
    explored_clean = explored_max is not None and explored_max < CLEAN_CORRIDOR_MAX_COST and explored_first is None
    outcome_event = outcome.get('event') if outcome else None
    failed_like = outcome_event in ('timeout', 'failure')
    if spatial_complete and failed_like and context == 'route_divergence_chosen_moves_away_rejected_moves_toward_exit' and path_cells_present and chosen_high and explored_clean:
        classification = 'chosen_high_explored_clean_contract_signal'
    else:
        classification = 'insufficient_or_dirty_explored_corridor'
    return {
        'goal_sequence': int(number(dispatch.get('goal_sequence')) or -1),
        'outcome': outcome or {'event': None, 'result_reason': None},
        'dispatch_xy': target_xy(dispatch.get('dispatch_pose')),
        'chosen_target': target_xy(dispatch.get('target')),
        'explored_candidate_target': target_xy(explored.get('target')) if explored else None,
        'chosen_target_exit_dist': number(dispatch.get('target_exit_dist')),
        'explored_candidate_target_exit_dist': number(explored.get('target_exit_dist')) if explored else None,
        'route_context': context,
        'snapshot_type': snapshot.get('snapshot_type'),
        'spatial_contract_complete': spatial_complete,
        'path_ahead_high_cost_cells_present': path_cells_present,
        'path_ahead_high_cost_count': high_count,
        'path_ahead_nearest_high_cost_point': target_xy(snapshot.get('path_ahead_1_0m_nearest_high_cost_point')),
        'path_ahead_high_cost_centroid': target_xy(snapshot.get('path_ahead_1_0m_high_cost_centroid')),
        'path_ahead_first_high_cost_distance_m': number(snapshot.get('path_ahead_1_0m_first_high_cost_distance_m')),
        'chosen_route_corridor_cost_max': chosen_max,
        'chosen_route_corridor_cost_mean': number(snapshot.get('chosen_route_corridor_cost_mean')),
        'chosen_route_first_high_cost_distance_m': chosen_first,
        'explored_candidate_corridor_cost_max': explored_max,
        'explored_candidate_corridor_cost_mean': number(snapshot.get('explored_candidate_corridor_cost_mean')),
        'explored_candidate_first_high_cost_distance_m': explored_first,
        'chosen_route_high_cost': chosen_high,
        'explored_candidate_corridor_clean': explored_clean,
        'classification': classification,
    }


def analyze_run(log_dir: Path, run_id: str, failed_only: bool) -> dict[str, Any]:
    dispatches = load_dispatches(log_dir, run_id)
    outcomes = load_outcomes(log_dir, run_id)
    snapshots = load_snapshots(log_dir, run_id)
    cases = []
    for seq, dispatch in sorted(dispatches.items()):
        outcome = outcomes.get(seq, {'event': None, 'result_reason': None})
        if failed_only and outcome.get('event') not in ('timeout', 'failure'):
            continue
        case = classify_case(dispatch, outcome, best_spatial_snapshot(snapshots.get(seq, [])))
        if case['route_context'].startswith('route_divergence') or case['path_ahead_high_cost_cells_present'] or case['chosen_route_corridor_cost_max'] is not None:
            cases.append(case)
    counts = Counter(case['classification'] for case in cases)
    return {
        'run_id': run_id,
        'failed_only': failed_only,
        'case_count': len(cases),
        'classification_counts': dict(sorted(counts.items())),
        'cases': cases,
    }


def decision_for(failed_runs: dict[str, Any]) -> dict[str, Any]:
    failed_cases = [case for run in failed_runs.values() for case in run['cases'] if case['outcome'].get('event') in ('timeout', 'failure')]
    signal_count = sum(1 for case in failed_cases if case['classification'] == 'chosen_high_explored_clean_contract_signal')
    all_signal_complete = signal_count > 0 and all(case['spatial_contract_complete'] for case in failed_cases if case['classification'] == 'chosen_high_explored_clean_contract_signal')
    if signal_count > 0 and all_signal_complete:
        phase27_signal = 'possible_only_after_real_matched_repeat_validation'
        recommendation = 'collect_real_phase26l_artifacts_before_any_phase27'
    else:
        phase27_signal = 'not_supported'
        recommendation = 'complete_runtime_spatial_contract_and_repeat_failed_only_matches'
    return {
        'phase27_candidate_signal': phase27_signal,
        'next_recommendation': recommendation,
        'guardrails': [
            'do_not_enter_phase27_from_contract_only',
            'require_real_artifacts_not_synthetic_contracts',
            'require_failed_only_matched_repeats_stable',
            'require_chosen_away_route_high_cost_and_explored_candidate_corridor_clean',
        ],
    }


def build_report(log_dir: Path, failed_runs: list[str], compare_runs: list[str]) -> dict[str, Any]:
    failed_run_reports: dict[str, Any] = {}
    compare_run_reports: dict[str, Any] = {}
    for run_id in failed_runs:
        failed_run_reports[run_id] = analyze_run(log_dir, run_id, failed_only=True)
    for run_id in compare_runs:
        compare_run_reports[run_id] = analyze_run(log_dir, run_id, failed_only=False)
    failed_signal_count = sum(
        1
        for run_id in failed_runs
        for case in failed_run_reports[run_id]['cases']
        if case['classification'] == 'chosen_high_explored_clean_contract_signal'
    )
    failed_complete_count = sum(
        1
        for run_id in failed_runs
        for case in failed_run_reports[run_id]['cases']
        if case['spatial_contract_complete']
    )
    return {
        'phase': '26L',
        'analysis_only': True,
        'inputs': {
            'log_dir': str(log_dir),
            'failed_runs': failed_runs,
            'compare_runs': compare_runs,
        },
        'failed_runs': failed_run_reports,
        'compare_runs': compare_run_reports,
        'runs': {**failed_run_reports, **{f'compare::{key}': value for key, value in compare_run_reports.items()}},
        'comparison': {
            'failed_runs': {
                'run_count': len(failed_runs),
                'spatial_contract_complete_case_count': failed_complete_count,
                'chosen_high_explored_clean_contract_signal_count': failed_signal_count,
            }
        },
        'decision': decision_for(failed_run_reports),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', required=True)
    parser.add_argument('--failed-runs', required=True, help='Comma-separated run ids to inspect as failed-only evidence.')
    parser.add_argument('--compare-runs', default='', help='Optional comma-separated comparison run ids.')
    parser.add_argument('--output-json', required=True)
    return parser.parse_args()


def split_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(',') if item.strip()]


def main() -> int:
    args = parse_args()
    log_dir = Path(args.log_dir)
    output = Path(args.output_json)
    report = build_report(log_dir, split_csv(args.failed_runs), split_csv(args.compare_runs))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps(report['decision'], sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
