#!/usr/bin/env python3
"""Compare Phase 25A local-cost relief experiment against baseline metrics."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

EXIT_RANK = {
    'FAILED_EXHAUSTED': 0,
    'SETTLING': 1,
    'EXIT_REACHED': 2,
}


def load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def get_nested(data: dict[str, Any], *keys: str, default: Any = None) -> Any:
    cur: Any = data
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def footprint_count(data: dict[str, Any]) -> int:
    return int(get_nested(data, 'timeout_subtypes', 'summary', 'controller_subtype_counts', default={}).get('footprint_path_blocked_late_silent', 0))


def run_value(data: dict[str, Any], key: str, default: Any = 0) -> Any:
    return get_nested(data, 'run_summary', key, default=default)


def check(name: str, baseline: Any, experiment: Any, passed: bool, criterion: str) -> dict[str, Any]:
    return {
        'name': name,
        'baseline': baseline,
        'experiment': experiment,
        'criterion': criterion,
        'passed': bool(passed),
    }


def compare(baseline: dict[str, Any], experiment: dict[str, Any]) -> dict[str, Any]:
    base_fp = footprint_count(baseline)
    exp_fp = footprint_count(experiment)
    base_timeouts = int(run_value(baseline, 'timeout_cancel_count'))
    exp_timeouts = int(run_value(experiment, 'timeout_cancel_count'))
    base_blocked = int(run_value(baseline, 'blocked_branch_count'))
    exp_blocked = int(run_value(experiment, 'blocked_branch_count'))
    base_blacklist = int(run_value(baseline, 'blacklisted_goal_count'))
    exp_blacklist = int(run_value(experiment, 'blacklisted_goal_count'))
    base_success = int(run_value(baseline, 'goal_success_count'))
    exp_success = int(run_value(experiment, 'goal_success_count'))
    base_mode = str(run_value(baseline, 'final_mode', 'FAILED_EXHAUSTED'))
    exp_mode = str(run_value(experiment, 'final_mode', 'FAILED_EXHAUSTED'))

    checks = {
        'footprint_path_blocked_reduced': check('footprint_path_blocked_reduced', base_fp, exp_fp, exp_fp < base_fp, 'experiment < baseline'),
        'timeouts_reduced': check('timeouts_reduced', base_timeouts, exp_timeouts, exp_timeouts < base_timeouts, 'experiment < baseline'),
        'blocked_branch_no_regression': check('blocked_branch_no_regression', base_blocked, exp_blocked, exp_blocked <= base_blocked, 'experiment <= baseline'),
        'blacklisted_goal_no_regression': check('blacklisted_goal_no_regression', base_blacklist, exp_blacklist, exp_blacklist <= base_blacklist, 'experiment <= baseline'),
        'success_no_regression': check('success_no_regression', base_success, exp_success, exp_success >= base_success, 'experiment >= baseline'),
        'exit_behavior_preserved_or_improved': check(
            'exit_behavior_preserved_or_improved',
            base_mode,
            exp_mode,
            EXIT_RANK.get(exp_mode, -1) >= EXIT_RANK.get(base_mode, -1),
            'experiment rank >= baseline rank',
        ),
    }
    accepted = all(item['passed'] for item in checks.values())
    return {
        'accepted': accepted,
        'checks': checks,
        'baseline_summary': {
            'footprint_path_blocked_late_silent': base_fp,
            'timeout_cancel_count': base_timeouts,
            'goal_success_count': base_success,
            'blocked_branch_count': base_blocked,
            'blacklisted_goal_count': base_blacklist,
            'final_mode': base_mode,
        },
        'experiment_summary': {
            'footprint_path_blocked_late_silent': exp_fp,
            'timeout_cancel_count': exp_timeouts,
            'goal_success_count': exp_success,
            'blocked_branch_count': exp_blocked,
            'blacklisted_goal_count': exp_blacklist,
            'final_mode': exp_mode,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--experiment', type=Path, required=True)
    parser.add_argument('--output-json', type=Path)
    args = parser.parse_args()
    result = compare(load(args.baseline), load(args.experiment))
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
