#!/usr/bin/env python3
"""Phase26R coverage checker for compact MPPI summary evidence.

Analysis-only. Verifies that the compact MPPI recorder covers required summary
topics in first-cmd-near-zero ±1s windows and that analyzer trajectory summary
metrics are present. It never authorizes intervention.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

REQUIRED_TOPICS = ('/trajectories', '/optimal_trajectory', '/transformed_global_plan')


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


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    rows = []
    for line in path.open('r', encoding='utf-8'):
        if line.strip():
            rows.append(json.loads(line))
    return rows


def case_key(row: dict[str, Any]) -> tuple[str, int]:
    return (str(row.get('run_id')), int(row.get('goal_sequence')))


def rows_in_window(rows: list[dict[str, Any]], start: float, end: float) -> list[dict[str, Any]]:
    selected = []
    for row in rows:
        t = number(row.get('wall_time'))
        if t is not None and start <= t <= end:
            selected.append(row)
    return selected


def analyze_case(case: dict[str, Any], evidence_rows: list[dict[str, Any]], analysis_cases: dict[tuple[str, int], dict[str, Any]]) -> dict[str, Any]:
    relation = case.get('cmd_near_zero_relation', {})
    first_cmd = number(relation.get('first_cmd_near_zero_time'))
    if first_cmd is None:
        # No first near-zero window in this case; report as not coverable rather than guessing.
        start = end = None
        window_rows: list[dict[str, Any]] = []
    else:
        start = first_cmd - 1.0
        end = first_cmd + 1.0
        window_rows = rows_in_window(evidence_rows, start, end)

    required = {}
    for topic in REQUIRED_TOPICS:
        topic_rows = [row for row in window_rows if row.get('topic') == topic]
        required[topic] = {
            'covered': bool(topic_rows),
            'sample_count': len(topic_rows),
            'summary_kinds': sorted({(row.get('data_summary') or {}).get('summary_kind') for row in topic_rows if row.get('data_summary')}),
        }

    key = case_key(case)
    analysis_case = analysis_cases.get(key, {})
    trajectory_summary = analysis_case.get('trajectory_summary', {})
    all_required = all(item['covered'] for item in required.values())
    metrics_present = number(trajectory_summary.get('sample_count')) is not None and float(trajectory_summary.get('sample_count') or 0) > 0
    return {
        'run_id': key[0],
        'goal_sequence': key[1],
        'window': {
            'first_cmd_near_zero_time': first_cmd,
            'start_time': start,
            'end_time': end,
            'start_offset_sec': -1.0,
            'end_offset_sec': 1.0,
            'evidence_row_count': len(window_rows),
        },
        'required_topics': required,
        'all_required_topics_covered': all_required,
        'trajectory_summary_metrics': trajectory_summary,
        'trajectory_summary_metrics_present': metrics_present,
        'condition_hypothesis': analysis_case.get('condition_hypothesis'),
    }


def build_report(timeline_json: Path, mppi_evidence: Path, analysis_json: Path, max_evidence_bytes: int) -> dict[str, Any]:
    timeline = load_json(timeline_json)
    analysis = load_json(analysis_json)
    evidence_rows = [row for row in load_jsonl(mppi_evidence) if row.get('event', 'message') in ('message', None)]
    analysis_cases = {case_key(case): case for case in analysis.get('cases', [])}
    cases = [analyze_case(case, evidence_rows, analysis_cases) for case in timeline.get('cases', [])]
    evidence_size = mppi_evidence.stat().st_size if mppi_evidence.exists() else 0
    all_required_count = sum(1 for case in cases if case['all_required_topics_covered'])
    metrics_count = sum(1 for case in cases if case['trajectory_summary_metrics_present'])
    pass_coverage = bool(cases) and evidence_size <= max_evidence_bytes and all_required_count == len(cases) and metrics_count == len(cases)
    return {
        'phase': '26R',
        'analysis_only': True,
        'source_timeline_json': str(timeline_json),
        'source_mppi_evidence': str(mppi_evidence),
        'source_analysis_json': str(analysis_json),
        'pass_coverage': pass_coverage,
        'cases': cases,
        'summary': {
            'case_count': len(cases),
            'evidence_size_bytes': evidence_size,
            'max_evidence_bytes': max_evidence_bytes,
            'all_required_topics_covered_case_count': all_required_count,
            'trajectory_summary_metrics_present_case_count': metrics_count,
        },
        'decision': {
            'phase27_candidate_signal': 'not_supported',
            'intervention_allowed': False,
            'guardrails': [
                'summary_evidence_smoke_only',
                'do_not_change_branch_selection',
                'do_not_tune_nav2_controller_params_from_phase26r',
                'require_matched_repeats_with_stable_specific_condition_before_phase27',
            ],
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeline-json', type=Path, required=True)
    parser.add_argument('--mppi-evidence', type=Path, required=True)
    parser.add_argument('--analysis-json', type=Path, required=True)
    parser.add_argument('--output-json', type=Path, required=True)
    parser.add_argument('--max-evidence-bytes', type=int, default=5_000_000)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = build_report(args.timeline_json, args.mppi_evidence, args.analysis_json, args.max_evidence_bytes)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({'pass_coverage': report['pass_coverage'], 'summary': report['summary'], 'decision': report['decision']}, sort_keys=True))
    return 0 if report['pass_coverage'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
