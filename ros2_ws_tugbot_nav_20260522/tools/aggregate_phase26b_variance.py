#!/usr/bin/env python3
"""Aggregate Phase 26B repeat-run variance artifacts.

This tool is intentionally descriptive rather than a promotion gate. It groups
canonical baseline and CostCritic 2.75 candidate runs, verifies source/runtime
cost-weight evidence, and summarizes run-to-run variance metrics.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any

METRIC_KEYS = [
    'goal_count',
    'goal_success_count',
    'goal_failure_count',
    'timeout_cancel_count',
    'blocked_branch_count',
    'blacklisted_goal_count',
    'exit_distance_m',
    'footprint_path_blocked_late_silent_count',
    'side_or_timing_late_silent_count',
    'unclassified_late_silent_count',
]


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists() or path.stat().st_size == 0:
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def load_last_state(path: Path) -> dict[str, Any]:
    last: dict[str, Any] = {}
    if not path.exists() or path.stat().st_size == 0:
        return last
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        payload = row.get('state', row)
        if isinstance(payload, dict):
            last = payload
    return last


def get_nested(data: dict[str, Any], *keys: str, default: Any = None) -> Any:
    cur: Any = data
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def as_float(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(number):
        return None
    return number


def unique_sorted(values: list[Any]) -> list[Any]:
    clean: list[Any] = []
    for value in values:
        if value is None:
            continue
        if value not in clean:
            clean.append(value)
    return sorted(clean)


def metric_summary(values: list[Any]) -> dict[str, Any]:
    nums = [v for v in (as_float(value) for value in values) if v is not None]
    if not nums:
        return {'count': 0, 'min': None, 'max': None, 'mean': None, 'median': None, 'stdev': None, 'values': []}
    return {
        'count': len(nums),
        'min': round(min(nums), 6),
        'max': round(max(nums), 6),
        'mean': round(statistics.fmean(nums), 6),
        'median': round(statistics.median(nums), 6),
        'stdev': round(statistics.stdev(nums), 6) if len(nums) >= 2 else 0.0,
        'values': [round(v, 6) for v in nums],
    }


def classify_run_id(run_id: str, fingerprint: dict[str, Any]) -> str:
    selected = fingerprint.get('selected_profile')
    if selected:
        return str(selected)
    if run_id.startswith('phase26b_candidate'):
        return 'candidate_costcritic_275'
    return 'canonical_baseline'


def read_run(log_dir: Path, run_id: str) -> dict[str, Any]:
    fingerprint = load_json(log_dir / f'{run_id}_params_fingerprint.json')
    runtime = load_json(log_dir / f'{run_id}_runtime_params' / 'controller_server_summary.json')
    state = load_last_state(log_dir / f'{run_id}_explorer_state.jsonl')
    subtypes = load_json(log_dir / f'{run_id}_timeout_subtypes.json')
    post_recovery = load_json(log_dir / f'{run_id}_post_recovery_enriched.json')

    controller_counts = get_nested(subtypes, 'summary', 'controller_subtype_counts', default={}) or {}
    run = {
        'run_id': run_id,
        'profile': classify_run_id(run_id, fingerprint),
        'final_mode': state.get('mode'),
        'goal_count': state.get('goal_count'),
        'goal_success_count': state.get('goal_success_count'),
        'goal_failure_count': state.get('goal_failure_count'),
        'timeout_cancel_count': state.get('timeout_cancel_count'),
        'blocked_branch_count': state.get('blocked_branch_count'),
        'blacklisted_goal_count': state.get('blacklisted_goal_count'),
        'exit_distance_m': state.get('exit_distance_m'),
        'source_cost_weight': get_nested(fingerprint, 'params_file', 'costcritic_cost_weight'),
        'runtime_cost_weight': get_nested(runtime, 'controller_server', 'FollowPath', 'CostCritic', 'cost_weight'),
        'footprint_path_blocked_late_silent_count': controller_counts.get('footprint_path_blocked_late_silent', 0),
        'side_or_timing_late_silent_count': controller_counts.get('side_cost_or_timing_late_silent', subtypes.get('summary', {}).get('side_or_timing_count', 0) if isinstance(subtypes.get('summary'), dict) else 0),
        'unclassified_late_silent_count': controller_counts.get('unclassified_late_silent', subtypes.get('summary', {}).get('unclassified_count', 0) if isinstance(subtypes.get('summary'), dict) else 0),
        'post_recovery_summary': post_recovery.get('summary', {}),
        'artifact_presence': {
            'params_fingerprint': bool(fingerprint),
            'runtime_params_summary': bool(runtime),
            'explorer_state': bool(state),
            'timeout_subtypes': bool(subtypes),
            'post_recovery_enriched': bool(post_recovery),
        },
    }
    return run


def summarize_group(profile: str, runs: list[dict[str, Any]]) -> dict[str, Any]:
    summary: dict[str, Any] = {
        'profile': profile,
        'run_count': len(runs),
        'run_ids': [run['run_id'] for run in runs],
        'exit_reached_count': sum(1 for run in runs if run.get('final_mode') == 'EXIT_REACHED'),
        'failed_exhausted_count': sum(1 for run in runs if run.get('final_mode') == 'FAILED_EXHAUSTED'),
        'final_mode_values': [run.get('final_mode') for run in runs],
        'source_cost_weight_values': unique_sorted([run.get('source_cost_weight') for run in runs]),
        'runtime_cost_weight_values': unique_sorted([run.get('runtime_cost_weight') for run in runs]),
        'missing_artifacts': {
            run['run_id']: [name for name, ok in run.get('artifact_presence', {}).items() if not ok]
            for run in runs
            if any(not ok for ok in run.get('artifact_presence', {}).values())
        },
    }
    for key in METRIC_KEYS:
        summary[key] = metric_summary([run.get(key) for run in runs])
    return summary


def recommendation(groups: dict[str, dict[str, Any]]) -> str:
    baseline = groups.get('canonical_baseline')
    candidate = groups.get('candidate_costcritic_275')
    if not baseline or not candidate or baseline.get('run_count', 0) < 3 or candidate.get('run_count', 0) < 3:
        return 'insufficient_repeat_data'
    if baseline.get('missing_artifacts') or candidate.get('missing_artifacts'):
        return 'artifact_gaps_require_rerun_before_decision'
    b_modes = baseline.get('final_mode_values', [])
    c_modes = candidate.get('final_mode_values', [])
    b_var = len(set(str(v) for v in b_modes)) > 1
    c_var = len(set(str(v) for v in c_modes)) > 1
    c_timeouts = candidate['timeout_cancel_count']['median']
    b_timeouts = baseline['timeout_cancel_count']['median']
    c_fp = candidate['footprint_path_blocked_late_silent_count']['median']
    b_fp = baseline['footprint_path_blocked_late_silent_count']['median']
    if b_var or c_var:
        return 'candidate_variance_characterized_not_promotion_ready'
    if c_timeouts is not None and b_timeouts is not None and c_fp is not None and b_fp is not None and c_timeouts <= b_timeouts and c_fp <= b_fp:
        return 'candidate_repeat_metrics_favorable_but_requires_review'
    return 'candidate_repeat_metrics_not_stable_enough_for_promotion'


def aggregate(log_dir: Path, run_ids: list[str]) -> dict[str, Any]:
    runs = [read_run(log_dir, run_id) for run_id in run_ids]
    grouped_runs: dict[str, list[dict[str, Any]]] = {}
    for run in runs:
        grouped_runs.setdefault(str(run['profile']), []).append(run)
    groups = {profile: summarize_group(profile, items) for profile, items in sorted(grouped_runs.items())}
    return {
        'phase': '26B',
        'purpose': 'repeat-run variance characterization for canonical baseline vs CostCritic 2.75 candidate',
        'run_ids': run_ids,
        'runs': runs,
        'groups': groups,
        'recommendation': recommendation(groups),
        'decision_guardrails': [
            'Do not promote cost_weight=2.75 from a single smoke or mixed artifact evidence.',
            'Use source fingerprint plus runtime /controller_server dump for every run.',
            'Interpret candidate differences as variance unless repeat medians and final-mode stability agree.',
        ],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, default=Path('log'))
    parser.add_argument('--runs', required=True, help='Comma-separated run ids')
    parser.add_argument('--output-json', type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_ids = [item.strip() for item in args.runs.split(',') if item.strip()]
    if not run_ids:
        print('no run ids provided', file=sys.stderr)
        return 2
    payload = aggregate(args.log_dir, run_ids)
    text = json.dumps(payload, indent=2, sort_keys=True) + '\n'
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(text, encoding='utf-8')
    print(text)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
