#!/usr/bin/env python3
"""Retry dumping Nav2 controller runtime params and validate CostCritic weight."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import yaml


COST_WEIGHT_PATH = (
    'controller_server',
    'ros__parameters',
    'FollowPath',
    'CostCritic',
    'cost_weight',
)


def _load_yaml(path: Path) -> Any:
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _get(data: Any, path: tuple[str, ...]) -> Any:
    current = data
    for key in path:
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


def extract_cost_weight(path: Path) -> dict[str, Any]:
    data = _load_yaml(path)
    value = _get(data, COST_WEIGHT_PATH)
    if not isinstance(value, (int, float)):
        value = _get(data, ('/controller_server', *COST_WEIGHT_PATH[1:]))
    if not isinstance(value, (int, float)):
        raise ValueError(f'missing FollowPath.CostCritic.cost_weight in {path}')
    return {
        'controller_server': {
            'FollowPath': {
                'CostCritic': {
                    'cost_weight': float(value),
                }
            }
        }
    }


def _run_ros2_param_dump(node: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ['ros2', 'param', 'dump', node],
        text=True,
        capture_output=True,
        check=False,
    )


def dump_with_retry(
    node: str,
    output: Path,
    summary_json: Path | None,
    expected_cost_weight: float | None,
    timeout_sec: float,
    interval_sec: float,
) -> int:
    deadline = time.monotonic() + timeout_sec
    output.parent.mkdir(parents=True, exist_ok=True)
    tmp = output.with_suffix(output.suffix + '.tmp')
    attempts = 0
    last_error = ''
    while time.monotonic() <= deadline:
        attempts += 1
        result = _run_ros2_param_dump(node)
        if result.returncode == 0 and result.stdout.strip():
            tmp.write_text(result.stdout, encoding='utf-8')
            try:
                summary = extract_cost_weight(tmp)
                cost_weight = summary['controller_server']['FollowPath']['CostCritic']['cost_weight']
                if expected_cost_weight is not None and abs(cost_weight - expected_cost_weight) > 1e-9:
                    last_error = (
                        f'FollowPath.CostCritic.cost_weight expected '
                        f'{expected_cost_weight} but got {cost_weight}'
                    )
                else:
                    tmp.replace(output)
                    summary.update({
                        'node': node,
                        'output': str(output),
                        'attempts': attempts,
                        'expected_cost_weight': expected_cost_weight,
                    })
                    if summary_json is not None:
                        summary_json.parent.mkdir(parents=True, exist_ok=True)
                        summary_json.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n', encoding='utf-8')
                    print(json.dumps(summary, indent=2, sort_keys=True))
                    return 0
            except Exception as exc:  # noqa: BLE001 - diagnostics script reports and retries.
                last_error = str(exc)
        else:
            last_error = (result.stderr or result.stdout or f'ros2 param dump {node} failed').strip()
        time.sleep(interval_sec)
    if tmp.exists():
        tmp.unlink()
    print(f'failed to dump valid runtime params for {node} after {attempts} attempts: {last_error}', file=sys.stderr)
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--extract-cost-weight', type=Path)
    parser.add_argument('--node', default='/controller_server')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--summary-json', type=Path)
    parser.add_argument('--expected-cost-weight', type=float)
    parser.add_argument('--timeout-sec', type=float, default=120.0)
    parser.add_argument('--interval-sec', type=float, default=1.0)
    args = parser.parse_args()

    if args.extract_cost_weight is not None:
        try:
            summary = extract_cost_weight(args.extract_cost_weight)
        except Exception as exc:  # noqa: BLE001 - CLI should surface readable validation errors.
            print(str(exc), file=sys.stderr)
            return 1
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0

    if args.output is None:
        parser.error('--output is required unless --extract-cost-weight is used')
    return dump_with_retry(
        node=args.node,
        output=args.output,
        summary_json=args.summary_json,
        expected_cost_weight=args.expected_cost_weight,
        timeout_sec=args.timeout_sec,
        interval_sec=args.interval_sec,
    )


if __name__ == '__main__':
    raise SystemExit(main())
