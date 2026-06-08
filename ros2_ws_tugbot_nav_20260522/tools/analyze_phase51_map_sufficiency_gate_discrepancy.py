#!/usr/bin/env python3
"""Phase51 map sufficiency gate discrepancy diagnostics.

Offline analyzer comparing maze_explorer gate map ratios against runtime recorder
map snapshots. It is diagnostics-only: no Nav2, MPPI, controller, clearance, or
exploration strategy changes.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

CLASSIFICATIONS = {
    'RECORDER_IN_BOUNDS_DENOMINATOR_MISMATCH',
    'GATE_RUNTIME_SAMPLING_ALIGNED_MAP_INSUFFICIENT',
    'TIMESTAMP_OR_SNAPSHOT_MISMATCH',
    'GATE_PAYLOAD_INSUFFICIENT_DIAGNOSTICS',
    'INSUFFICIENT_EVIDENCE',
}


def _safe_json_loads(text: str) -> Any | None:
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return None


def _read_json(path: Path | None) -> dict[str, Any]:
    if not path or not path.exists() or not path.stat().st_size:
        return {}
    data = _safe_json_loads(path.read_text(encoding='utf-8', errors='replace'))
    return data if isinstance(data, dict) else {}


def _read_jsonl_payloads(path: Path | None) -> list[dict[str, Any]]:
    if not path or not path.exists() or not path.stat().st_size:
        return []
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        row = _safe_json_loads(line)
        if not isinstance(row, dict):
            continue
        payload = row.get('state') if isinstance(row.get('state'), dict) else row
        if isinstance(payload, dict):
            payload['_recorder_elapsed_sec'] = row.get('elapsed_sec')
            payload['_recorder_seq'] = row.get('seq')
            rows.append(payload)
    return rows


def _last_gate(states: list[dict[str, Any]]) -> dict[str, Any] | None:
    for state in reversed(states):
        gate = state.get('dispatch_readiness_gate')
        if isinstance(gate, dict):
            return gate
    return None


def _last_runtime_snapshot(runtime: dict[str, Any]) -> dict[str, Any] | None:
    snapshots = runtime.get('snapshots')
    if isinstance(snapshots, list) and snapshots:
        return snapshots[-1] if isinstance(snapshots[-1], dict) else None
    return None


def _close(a: Any, b: Any, eps: float = 1e-6) -> bool:
    try:
        return abs(float(a) - float(b)) <= eps
    except Exception:
        return False


def _ratio(obj: dict[str, Any] | None, key: str) -> float | None:
    if not isinstance(obj, dict):
        return None
    value = obj.get(key)
    return float(value) if isinstance(value, (int, float)) else None


def _classify(gate_map: dict[str, Any] | None, runtime_map: dict[str, Any] | None) -> tuple[str, dict[str, Any], str]:
    if not isinstance(gate_map, dict) or not isinstance(runtime_map, dict):
        return 'INSUFFICIENT_EVIDENCE', {}, 'collect gate and runtime map payloads with Phase51 diagnostics'

    inclusive = runtime_map.get('inclusive_near_robot') if isinstance(runtime_map.get('inclusive_near_robot'), dict) else None
    legacy = runtime_map.get('near_robot') if isinstance(runtime_map.get('near_robot'), dict) else None
    in_bounds = runtime_map.get('in_bounds_near_robot') if isinstance(runtime_map.get('in_bounds_near_robot'), dict) else legacy

    required_gate_fields = ['sample_count', 'robot_cell', 'sample_window', 'in_bounds_count', 'out_of_bounds_count']
    missing_gate_fields = [field for field in required_gate_fields if field not in gate_map]
    if missing_gate_fields:
        return 'GATE_PAYLOAD_INSUFFICIENT_DIAGNOSTICS', {'missing_gate_fields': missing_gate_fields}, 'rerun after Phase51 gate payload instrumentation'

    comparisons: dict[str, Any] = {
        'gate_known_ratio': gate_map.get('known_ratio'),
        'gate_free_ratio': gate_map.get('free_ratio'),
        'gate_sample_count': gate_map.get('sample_count'),
        'gate_robot_cell': gate_map.get('robot_cell'),
        'gate_sample_window': gate_map.get('sample_window'),
        'runtime_inclusive_known_ratio': _ratio(inclusive, 'known_ratio'),
        'runtime_inclusive_free_ratio': _ratio(inclusive, 'free_ratio'),
        'runtime_inclusive_sample_count': inclusive.get('sample_count') if inclusive else None,
        'runtime_legacy_known_ratio': _ratio(legacy, 'known_ratio'),
        'runtime_legacy_free_ratio': _ratio(legacy, 'free_ratio'),
        'runtime_legacy_total': legacy.get('total') if legacy else None,
        'runtime_in_bounds_known_ratio': _ratio(in_bounds, 'known_ratio'),
        'runtime_in_bounds_free_ratio': _ratio(in_bounds, 'free_ratio'),
    }

    comparisons['sample_count_match'] = bool(inclusive and int(gate_map.get('sample_count', -1)) == int(inclusive.get('sample_count', -2)))
    comparisons['robot_cell_match'] = bool(inclusive and gate_map.get('robot_cell') == inclusive.get('robot_cell'))
    comparisons['sample_window_match'] = bool(inclusive and gate_map.get('sample_window') == inclusive.get('sample_window'))
    comparisons['gate_matches_runtime_inclusive'] = bool(
        inclusive
        and _close(gate_map.get('known_ratio'), inclusive.get('known_ratio'))
        and _close(gate_map.get('free_ratio'), inclusive.get('free_ratio'))
    )
    comparisons['legacy_recorder_excluded_out_of_bounds'] = bool(
        legacy
        and inclusive
        and int(legacy.get('total', legacy.get('sample_count', -1))) == int(inclusive.get('in_bounds_count', -2))
        and int(inclusive.get('out_of_bounds_count', 0)) > 0
    )

    if comparisons['gate_matches_runtime_inclusive'] and comparisons['legacy_recorder_excluded_out_of_bounds']:
        return (
            'RECORDER_IN_BOUNDS_DENOMINATOR_MISMATCH',
            comparisons,
            'keep_gate_waiting_logic; use inclusive diagnostics before considering threshold_or_rotation',
        )
    if comparisons['gate_matches_runtime_inclusive']:
        return 'GATE_RUNTIME_SAMPLING_ALIGNED_MAP_INSUFFICIENT', comparisons, 'map is genuinely insufficient in the gate window; discuss longer wait or initial scan motion later'
    if not inclusive:
        return 'GATE_PAYLOAD_INSUFFICIENT_DIAGNOSTICS', comparisons, 'rerun runtime recorder with inclusive_near_robot diagnostics'
    return 'TIMESTAMP_OR_SNAPSHOT_MISMATCH', comparisons, 'align gate map_stamp/robot_cell/sample_window with nearest runtime snapshot before deciding'


def analyze(args: argparse.Namespace) -> dict[str, Any]:
    runtime = _read_json(Path(args.runtime_evidence) if args.runtime_evidence else None)
    states = _read_jsonl_payloads(Path(args.explorer_state) if args.explorer_state else None)
    gate = _last_gate(states) or {}
    gate_map = gate.get('map') if isinstance(gate.get('map'), dict) else None
    snapshot = _last_runtime_snapshot(runtime) or {}
    runtime_map = snapshot.get('map') if isinstance(snapshot.get('map'), dict) else None
    classification, comparison, next_step = _classify(gate_map, runtime_map)
    assert classification in CLASSIFICATIONS
    summary = {
        'phase': 'Phase51 Map Sufficiency Gate Discrepancy Diagnostics',
        'classification': classification,
        'guardrails': {
            'no_nav2_mppi_controller_tuning': True,
            'no_clearance_strategy_change': True,
            'no_threshold_change': True,
            'no_autonomous_success_claim': True,
        },
        'final_gate_map': gate_map,
        'runtime_last_map': runtime_map,
        'gate_vs_runtime': comparison,
        'next_step': next_step,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--runtime-evidence', required=True)
    parser.add_argument('--explorer-state', required=True)
    parser.add_argument('--output', required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary = analyze(args)
    print(json.dumps({'output': args.output, 'classification': summary['classification']}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
