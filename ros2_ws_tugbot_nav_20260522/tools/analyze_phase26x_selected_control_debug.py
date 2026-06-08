#!/usr/bin/env python3
"""Analyze Phase26X MPPI selected-control debug JSONL.

Diagnostics-only. This analyzer consumes bounded JSONL rows emitted by a proposed
Nav2 MPPI source overlay around Optimizer / MPPIController selected-control
points. It never recommends branch-selection changes, Nav2/controller parameter
changes, MPPI behavior changes, or candidate promotion/rejection.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

NEAR_ZERO_LINEAR = 0.03
NEAR_ZERO_ANGULAR = 0.05


def number(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists() or path.stat().st_size == 0:
        return rows
    for line_no, line in enumerate(path.read_text(encoding='utf-8').splitlines(), start=1):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f'{path}:{line_no}: invalid JSONL row: {exc}') from exc
        if isinstance(row, dict):
            rows.append(row)
    return rows


def pick_nested(data: dict[str, Any], *keys: str) -> Any:
    current: Any = data
    for key in keys:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    return current


def velocity_from(value: Any) -> dict[str, float | None]:
    if not isinstance(value, dict):
        return {'vx': None, 'vy': None, 'wz': None}
    # Accept both overlay-friendly vx/vy/wz and ROS Twist-ish shapes.
    vx = number(value.get('vx'))
    vy = number(value.get('vy'))
    wz = number(value.get('wz'))
    if vx is None:
        vx = number(value.get('linear_x'))
    if vy is None:
        vy = number(value.get('linear_y'))
    if wz is None:
        wz = number(value.get('angular_z'))
    linear_value = value.get('linear')
    angular_value = value.get('angular')
    linear = linear_value if isinstance(linear_value, dict) else {}
    angular = angular_value if isinstance(angular_value, dict) else {}
    if vx is None:
        vx = number(linear.get('x'))
    if vy is None:
        vy = number(linear.get('y'))
    if wz is None:
        wz = number(angular.get('z'))
    return {'vx': vx, 'vy': vy, 'wz': wz}


def near_zero_velocity(value: Any) -> bool | None:
    vel = velocity_from(value)
    vals = [vel.get('vx'), vel.get('vy'), vel.get('wz')]
    if all(v is None for v in vals):
        return None
    vx = vel.get('vx') or 0.0
    vy = vel.get('vy') or 0.0
    wz = vel.get('wz') or 0.0
    return abs(vx) <= NEAR_ZERO_LINEAR and abs(vy) <= NEAR_ZERO_LINEAR and abs(wz) <= NEAR_ZERO_ANGULAR


def nonzero_velocity(value: Any) -> bool | None:
    nz = near_zero_velocity(value)
    if nz is None:
        return None
    return not nz


def first_control(row: dict[str, Any]) -> dict[str, Any] | None:
    for key in ('control_sequence_head', 'control_sequence_first', 'control_sequence'):
        value = row.get(key)
        if isinstance(value, list) and value:
            return value[0] if isinstance(value[0], dict) else None
        if isinstance(value, dict):
            return value
    return None


def fallback_used(row: dict[str, Any]) -> bool | None:
    if isinstance(row.get('fallback_used'), bool):
        return bool(row['fallback_used'])
    fallback = row.get('fallback')
    if isinstance(fallback, dict):
        for key in ('used', 'fallback_used', 'called'):
            if isinstance(fallback.get(key), bool):
                return bool(fallback[key])
    if isinstance(row.get('eval_control'), dict):
        return fallback_used(row['eval_control'])
    return None


def exception_or_failure(row: dict[str, Any]) -> dict[str, Any]:
    fallback_value = row.get('fallback')
    eval_control_value = row.get('eval_control')
    fallback = fallback_value if isinstance(fallback_value, dict) else {}
    eval_control = eval_control_value if isinstance(eval_control_value, dict) else {}
    exception = row.get('exception') or eval_control.get('exception')
    failure = row.get('failure')
    if failure is None:
        failure = row.get('failed')
    if failure is None:
        failure = fallback.get('failed')
    if failure is None:
        failure = eval_control.get('failure') or eval_control.get('failed')
    retry_count = row.get('retry_count')
    if retry_count is None:
        retry_count = eval_control.get('retry_count')
    return {
        'exception_present': bool(exception),
        'exception': exception,
        'failure_present': bool(failure),
        'failure': failure,
        'retry_count': retry_count,
    }


def optimizer_update_collapsed(row: dict[str, Any]) -> bool | None:
    update = row.get('update_summary') or row.get('control_update') or row.get('optimizer_update')
    if not isinstance(update, dict):
        return None
    before = update.get('first_before') or update.get('before')
    after = update.get('first_after') or update.get('after')
    before_nonzero = nonzero_velocity(before)
    after_near_zero = near_zero_velocity(after)
    if before_nonzero is None or after_near_zero is None:
        return None
    return bool(before_nonzero and after_near_zero)


def analyze_cycle(row: dict[str, Any], index: int) -> dict[str, Any]:
    returned = row.get('returned_twist') or row.get('twist') or pick_nested(row, 'compute_velocity_commands', 'returned_twist') or {}
    first = first_control(row)
    returned_nz = near_zero_velocity(returned)
    first_nz = near_zero_velocity(first)
    fb_used = fallback_used(row)
    update_collapsed = optimizer_update_collapsed(row)
    control_nonzero_but_returned_zero = False
    if first_nz is False and returned_nz is True:
        control_nonzero_but_returned_zero = True
    exc = exception_or_failure(row)
    return {
        'cycle_index': row.get('cycle_index', row.get('cycle', index)),
        'wall_time': row.get('wall_time'),
        'event': row.get('event'),
        'returned_twist': velocity_from(returned),
        'control_sequence_first': velocity_from(first),
        'control_sequence_first_is_near_zero': first_nz,
        'returned_twist_is_near_zero': returned_nz,
        'fallback_used': fb_used,
        'control_nonzero_but_returned_zero': control_nonzero_but_returned_zero,
        'optimizer_update_collapsed_control': update_collapsed,
        'exception_present': exc['exception_present'],
        'failure_present': exc['failure_present'],
        'retry_count': exc['retry_count'],
    }


def count_true(rows: list[dict[str, Any]], key: str) -> int:
    return sum(1 for row in rows if row.get(key) is True)


def choose_conclusion(cycles: list[dict[str, Any]]) -> dict[str, Any]:
    if not cycles:
        return {
            'near_zero_reason': 'insufficient_evidence',
            'evidence_complete': False,
            'explanation': 'No Phase26X selected-control debug cycles were available.',
        }

    near_zero_cycles = [c for c in cycles if c.get('returned_twist_is_near_zero') is True]
    if not near_zero_cycles:
        return {
            'near_zero_reason': 'no_returned_near_zero_cycles_in_debug_log',
            'evidence_complete': True,
            'explanation': 'The debug log contains cycles, but none with a near-zero returned twist under Phase26X thresholds.',
        }

    fallback_count = count_true(near_zero_cycles, 'fallback_used')
    first_near_zero_count = count_true(near_zero_cycles, 'control_sequence_first_is_near_zero')
    conversion_count = count_true(near_zero_cycles, 'control_nonzero_but_returned_zero')
    update_collapsed_count = count_true(near_zero_cycles, 'optimizer_update_collapsed_control')

    # Prefer the most local / explanatory signal, while preserving uncertainty if
    # coverage is mixed or fields are missing.
    if fallback_count == len(near_zero_cycles):
        reason = 'fallback_used'
        complete = True
        explanation = 'All returned-near-zero cycles report fallback usage.'
    elif conversion_count > 0:
        reason = 'conversion_or_post_sequence_clamp_after_nonzero_control'
        complete = True
        explanation = 'At least one cycle has a nonzero first control sequence entry but a near-zero returned twist.'
    elif update_collapsed_count > 0:
        reason = 'optimizer_update_collapsed_control_sequence'
        complete = True
        explanation = 'At least one cycle shows updateControlSequence changing first control from nonzero to near-zero.'
    elif first_near_zero_count == len(near_zero_cycles):
        reason = 'control_sequence_itself_near_zero'
        complete = True
        explanation = 'All returned-near-zero cycles with evidence have a near-zero first control sequence entry.'
    else:
        reason = 'insufficient_or_mixed_evidence'
        complete = False
        explanation = 'Phase26X fields are missing or mixed; do not infer a tuning direction.'

    return {
        'near_zero_reason': reason,
        'evidence_complete': complete,
        'explanation': explanation,
    }


def build_report(rows: list[dict[str, Any]]) -> dict[str, Any]:
    cycles = [analyze_cycle(row, idx) for idx, row in enumerate(rows, start=1)]
    near_zero_count = count_true(cycles, 'returned_twist_is_near_zero')
    return {
        'phase': '26X',
        'analysis_only': True,
        'thresholds': {
            'near_zero_linear_abs_max': NEAR_ZERO_LINEAR,
            'near_zero_angular_abs_max': NEAR_ZERO_ANGULAR,
        },
        'summary': {
            'cycle_count': len(cycles),
            'near_zero_cycle_count': near_zero_count,
            'control_sequence_first_near_zero_count': count_true(cycles, 'control_sequence_first_is_near_zero'),
            'returned_twist_near_zero_count': near_zero_count,
            'fallback_used_count': count_true(cycles, 'fallback_used'),
            'control_nonzero_but_returned_zero_count': count_true(cycles, 'control_nonzero_but_returned_zero'),
            'optimizer_update_collapsed_control_count': count_true(cycles, 'optimizer_update_collapsed_control'),
            'exception_present_count': count_true(cycles, 'exception_present'),
            'failure_present_count': count_true(cycles, 'failure_present'),
        },
        'cycles': cycles,
        'conclusion': choose_conclusion(cycles),
        'decision': {
            'phase27_allowed': False,
            'candidate_promotion_or_rejection_allowed': False,
            'intervention_allowed': False,
            'guardrails': [
                'do_not_enter_phase27_from_phase26x',
                'do_not_change_branch_selection',
                'do_not_tune_nav2_controller_params_from_phase26x',
                'do_not_modify_mppi_control_behavior',
                'do_not_promote_or_reject_candidate_from_phase26x',
            ],
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-jsonl', required=True, type=Path)
    parser.add_argument('--output-json', required=True, type=Path)
    args = parser.parse_args()

    rows = load_jsonl(args.input_jsonl)
    report = build_report(rows)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
