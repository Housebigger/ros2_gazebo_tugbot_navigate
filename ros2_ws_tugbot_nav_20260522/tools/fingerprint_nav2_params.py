#!/usr/bin/env python3
"""Fingerprint and semantically compare Nav2 params files for maze smoke runs."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import yaml


COSTCRITIC_PATH = (
    'controller_server',
    'ros__parameters',
    'FollowPath',
    'CostCritic',
)
LOCAL_INFLATION_PATH = (
    'local_costmap',
    'local_costmap',
    'ros__parameters',
    'inflation_layer',
)
GLOBAL_INFLATION_PATH = (
    'global_costmap',
    'global_costmap',
    'ros__parameters',
    'inflation_layer',
)
PROGRESS_CHECKER_PATH = (
    'controller_server',
    'ros__parameters',
    'progress_checker',
)
GOAL_CHECKER_PATH = (
    'controller_server',
    'ros__parameters',
    'goal_checker',
)


def _load_yaml(path: Path) -> Any:
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _get(data: Any, path: tuple[str, ...]) -> Any:
    current = data
    for key in path:
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


def _cost_weight(data: Any) -> float | None:
    costcritic = _get(data, COSTCRITIC_PATH)
    if not isinstance(costcritic, dict):
        return None
    value = costcritic.get('cost_weight')
    return float(value) if isinstance(value, (int, float)) else value


def _summary(data: Any) -> dict[str, Any]:
    costcritic = _get(data, COSTCRITIC_PATH)
    local_inflation = _get(data, LOCAL_INFLATION_PATH)
    global_inflation = _get(data, GLOBAL_INFLATION_PATH)
    progress_checker = _get(data, PROGRESS_CHECKER_PATH)
    goal_checker = _get(data, GOAL_CHECKER_PATH)
    return {
        'costcritic': costcritic,
        'costcritic_cost_weight': _cost_weight(data),
        'local_inflation': local_inflation,
        'global_inflation': global_inflation,
        'progress_checker': progress_checker,
        'goal_checker': goal_checker,
    }


def _semantic_differences(left: Any, right: Any) -> list[dict[str, Any]]:
    diffs: list[dict[str, Any]] = []
    for label, path in [
        ('CostCritic', COSTCRITIC_PATH),
        ('local_inflation', LOCAL_INFLATION_PATH),
        ('global_inflation', GLOBAL_INFLATION_PATH),
        ('progress_checker', PROGRESS_CHECKER_PATH),
        ('goal_checker', GOAL_CHECKER_PATH),
    ]:
        left_value = _get(left, path)
        right_value = _get(right, path)
        if left_value != right_value:
            diffs.append({
                'section': label,
                'path': '.'.join(path),
                'left': left_value,
                'right': right_value,
            })
    if left != right:
        # Preserve a compact whole-file signal without dumping large YAML content.
        diffs.append({
            'section': 'whole_file',
            'path': '<root>',
            'left_equals_right': False,
        })
    return diffs


def build_fingerprint(
    params_file: Path,
    run_id: str,
    selected_profile: str,
    baseline_params_file: Path | None = None,
    compare_params_file: Path | None = None,
) -> dict[str, Any]:
    params_file = params_file.resolve()
    data = _load_yaml(params_file)
    result: dict[str, Any] = {
        'run_id': run_id,
        'selected_profile': selected_profile,
        'params_file': {
            'path': str(params_file),
            'sha256': _sha256(params_file),
            **_summary(data),
        },
    }

    if baseline_params_file is not None:
        baseline_params_file = baseline_params_file.resolve()
        baseline_data = _load_yaml(baseline_params_file)
        baseline_weight = _cost_weight(baseline_data)
        params_weight = _cost_weight(data)
        delta = None
        if isinstance(baseline_weight, (int, float)) and isinstance(params_weight, (int, float)):
            delta = round(params_weight - baseline_weight, 6)
        result['baseline'] = {
            'path': str(baseline_params_file),
            'sha256': _sha256(baseline_params_file),
            **_summary(baseline_data),
        }
        result['baseline_delta'] = {
            'costcritic_cost_weight': delta,
        }

    if compare_params_file is not None:
        compare_params_file = compare_params_file.resolve()
        compare_data = _load_yaml(compare_params_file)
        diffs = _semantic_differences(data, compare_data)
        result['compare'] = {
            'path': str(compare_params_file),
            'sha256': _sha256(compare_params_file),
            'equivalent': data == compare_data,
            'semantic_differences': diffs,
        }

    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--params-file', required=True, type=Path)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--selected-profile', default='unknown')
    parser.add_argument('--baseline-params-file', type=Path)
    parser.add_argument('--compare-params-file', type=Path)
    parser.add_argument('--output-json', type=Path)
    args = parser.parse_args()

    result = build_fingerprint(
        params_file=args.params_file,
        run_id=args.run_id,
        selected_profile=args.selected_profile,
        baseline_params_file=args.baseline_params_file,
        compare_params_file=args.compare_params_file,
    )
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(text + '\n', encoding='utf-8')
    else:
        print(text)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
