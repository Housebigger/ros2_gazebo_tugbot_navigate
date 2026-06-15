#!/usr/bin/env python3
"""Phase26S matched-repeat aggregator for compact MPPI summary smokes.

Diagnostics-only. Aggregates Phase26R coverage and MPPI trajectory summary
metrics across repeat runs. It can mark a stable specific condition as review-only,
but never authorizes Phase27 intervention.
"""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from statistics import mean
from typing import Any

SPECIFIC_CONDITIONS = {
    'trajectory_summary_degenerate_without_critic_stats',
    'costcritic_high_cost_with_degenerate_optimal_path',
    'costcritic_high_cost_with_near_zero_optimal_trajectory',
    'critic_stats_present_with_low_motion_trajectory_unclassified',
}


def number(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists() or path.stat().st_size == 0:
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def run_profile(run_id: str) -> str:
    if 'candidate' in run_id:
        return 'candidate'
    return 'baseline'


def expected_cost_weight(profile: str) -> float:
    return 2.75 if profile == 'candidate' else 3.81


def extract_runtime_cost_weight(log_dir: Path, run_id: str) -> float | None:
    summary = load_json(log_dir / f'{run_id}_runtime_params' / 'controller_server_summary.json')
    return number(
        summary.get('controller_server', {})
        .get('FollowPath', {})
        .get('CostCritic', {})
        .get('cost_weight')
    )


def condition_from_coverage_or_analysis(coverage: dict[str, Any], analysis: dict[str, Any]) -> str | None:
    for case in coverage.get('cases', []):
        if case.get('condition_hypothesis'):
            return str(case.get('condition_hypothesis'))
    counts = analysis.get('summary', {}).get('condition_hypothesis_counts', {})
    if counts:
        return Counter(counts).most_common(1)[0][0]
    for case in analysis.get('cases', []):
        if case.get('condition_hypothesis'):
            return str(case.get('condition_hypothesis'))
    return None


def collect_case_metrics(coverage: dict[str, Any]) -> list[dict[str, Any]]:
    rows = []
    for case in coverage.get('cases', []):
        metrics = case.get('trajectory_summary_metrics', {})
        required_topics = case.get('required_topics', {})
        rows.append({
            'run_id': case.get('run_id'),
            'goal_sequence': case.get('goal_sequence'),
            'all_required_topics_covered': bool(case.get('all_required_topics_covered')),
            'condition_hypothesis': case.get('condition_hypothesis'),
            'trajectory_summary_metrics': metrics,
            'required_topic_sample_counts': {
                topic: (details or {}).get('sample_count')
                for topic, details in required_topics.items()
            },
        })
    return rows


def summarize_numbers(rows: list[dict[str, Any]], metric_name: str) -> dict[str, Any]:
    values = []
    for row in rows:
        value = number((row.get('trajectory_summary_metrics') or {}).get(metric_name))
        if value is not None:
            values.append(value)
    return {
        'count': len(values),
        'min': min(values) if values else None,
        'max': max(values) if values else None,
        'mean': mean(values) if values else None,
    }


def load_run(log_dir: Path, run_id: str) -> dict[str, Any]:
    coverage_path = log_dir / f'{run_id}_phase26r_summary_evidence_coverage.json'
    analysis_path = log_dir / f'{run_id}_phase26p_mppi_evidence_analysis.json'
    evidence_path = log_dir / f'{run_id}_mppi_evidence_summary.jsonl'
    coverage = load_json(coverage_path)
    analysis = load_json(analysis_path)
    profile = run_profile(run_id)
    runtime_weight = extract_runtime_cost_weight(log_dir, run_id)
    expected_weight = expected_cost_weight(profile)
    semantic_ok = runtime_weight is not None and abs(runtime_weight - expected_weight) <= 1e-6
    case_rows = collect_case_metrics(coverage)
    condition = condition_from_coverage_or_analysis(coverage, analysis)
    return {
        'run_id': run_id,
        'profile': profile,
        'coverage_path': str(coverage_path),
        'analysis_path': str(analysis_path),
        'evidence_path': str(evidence_path),
        'coverage_exists': coverage_path.exists(),
        'analysis_exists': analysis_path.exists(),
        'pass_coverage': bool(coverage.get('pass_coverage')),
        'case_count': int(coverage.get('summary', {}).get('case_count') or 0),
        'evidence_size_bytes': int(coverage.get('summary', {}).get('evidence_size_bytes') or (evidence_path.stat().st_size if evidence_path.exists() else 0)),
        'condition_hypothesis': condition,
        'runtime_cost_weight': runtime_weight,
        'expected_cost_weight': expected_weight,
        'runtime_semantics_ok': semantic_ok,
        'cases': case_rows,
    }


def build_candidate_comparison(runs: list[dict[str, Any]]) -> dict[str, Any]:
    baseline_runs = [run for run in runs if run.get('profile') == 'baseline']
    candidate_runs = [run for run in runs if run.get('profile') == 'candidate']
    baseline_conditions = Counter(run.get('condition_hypothesis') for run in baseline_runs if run.get('condition_hypothesis'))
    candidate_conditions = Counter(run.get('condition_hypothesis') for run in candidate_runs if run.get('condition_hypothesis'))
    baseline_top = baseline_conditions.most_common(1)[0][0] if baseline_conditions else None
    candidate_top = candidate_conditions.most_common(1)[0][0] if candidate_conditions else None
    candidate_specific = bool(candidate_top in SPECIFIC_CONDITIONS)
    candidate_differs = bool(candidate_top and baseline_top and candidate_top != baseline_top)
    return {
        'mode': 'evidence_only_not_promotion',
        'baseline_condition_counts': dict(baseline_conditions),
        'candidate_condition_counts': dict(candidate_conditions),
        'baseline_top_condition': baseline_top,
        'candidate_top_condition': candidate_top,
        'candidate_has_specific_condition': candidate_specific,
        'candidate_condition_differs_from_baseline': candidate_differs,
    }


def build_report(log_dir: Path, run_ids: list[str], phase: str = '26S') -> dict[str, Any]:
    runs = [load_run(log_dir, run_id) for run_id in run_ids]
    case_rows = [case for run in runs for case in run.get('cases', [])]
    condition_counts = Counter(run.get('condition_hypothesis') for run in runs if run.get('condition_hypothesis'))
    stable_condition, stable_count = (None, 0)
    if condition_counts:
        stable_condition, stable_count = condition_counts.most_common(1)[0]
    all_coverage = bool(runs) and all(run['pass_coverage'] for run in runs)
    all_semantic = bool(runs) and all(run['runtime_semantics_ok'] for run in runs)
    baseline_count = sum(1 for run in runs if run['profile'] == 'baseline')
    candidate_count = sum(1 for run in runs if run['profile'] == 'candidate')
    matched_repeat_count = len(runs) >= 2
    stable_specific = bool(stable_condition in SPECIFIC_CONDITIONS and stable_count >= 2)
    if not all_coverage or not all_semantic:
        signal = 'not_supported'
        reason = 'coverage_or_semantic_guard_failed'
    elif not matched_repeat_count:
        signal = 'not_supported'
        reason = 'insufficient_matched_repeats'
    elif stable_specific:
        signal = 'review_only'
        reason = 'stable_specific_condition_requires_human_review_before_phase27'
    else:
        signal = 'not_supported'
        reason = 'no_stable_specific_condition'
    weights_by_profile: dict[str, list[float]] = {'baseline': [], 'candidate': []}
    for run in runs:
        weight = run.get('runtime_cost_weight')
        if weight is not None:
            weights_by_profile[run['profile']].append(weight)
    candidate_comparison = build_candidate_comparison(runs)
    if phase == '26T' and candidate_count > 0 and all_coverage and all_semantic:
        if candidate_comparison['candidate_has_specific_condition'] and candidate_comparison['candidate_condition_differs_from_baseline']:
            signal = 'review_only'
            reason = 'candidate_specific_difference_requires_human_review_before_phase27'
        else:
            signal = 'not_supported'
            reason = 'candidate_condition_not_specific_or_not_different'
    guardrails = [
        'diagnostics_only_matched_repeat_aggregation',
        'do_not_change_branch_selection',
        'do_not_tune_nav2_controller_params_from_phase26s',
        'do_not_promote_or_reject_candidate_from_phase26s',
        'human_review_before_phase27',
    ]
    if phase == '26T':
        guardrails.append('do_not_promote_or_reject_candidate_from_phase26t')
    return {
        'phase': phase,
        'analysis_only': True,
        'log_dir': str(log_dir),
        'run_ids': run_ids,
        'runs': runs,
        'summary': {
            'run_count': len(runs),
            'baseline_run_count': baseline_count,
            'candidate_run_count': candidate_count,
            'all_runs_passed_coverage': all_coverage,
            'all_runtime_semantics_ok': all_semantic,
            'condition_hypothesis_counts': dict(condition_counts),
            'stable_condition': stable_condition,
            'stable_condition_count': stable_count,
            'stable_condition_is_specific': stable_specific,
            'runtime_cost_weights_by_profile': weights_by_profile,
            'candidate_comparison': candidate_comparison,
            'trajectory_summary_metric_ranges': {
                'sample_count': summarize_numbers(case_rows, 'sample_count'),
                'marker_count_max': summarize_numbers(case_rows, 'marker_count_max'),
                'degenerate_trajectory_count_max': summarize_numbers(case_rows, 'degenerate_trajectory_count_max'),
                'representative_path_length_max': summarize_numbers(case_rows, 'representative_path_length_max'),
            },
        },
        'decision': {
            'phase27_candidate_signal': signal,
            'intervention_allowed': False,
            'candidate_promotion_allowed': False,
            'candidate_rejection_allowed': False,
            'reason': reason,
            'guardrails': guardrails,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, default=Path('log'))
    parser.add_argument('--run-ids', nargs='+', required=True)
    parser.add_argument('--phase', choices=['26S', '26T'], default='26S')
    parser.add_argument('--output-json', type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = build_report(args.log_dir, args.run_ids, phase=args.phase)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({'summary': report['summary'], 'decision': report['decision']}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
