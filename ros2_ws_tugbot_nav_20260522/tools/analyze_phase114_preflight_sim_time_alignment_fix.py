#!/usr/bin/env python3
"""Phase114 analyzer: verify preflight sim-time alignment minimal fix."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

WALL_TIME_SIZED_AGE_SEC = 1000.0


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y'}
    return bool(value)


def _num(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def sample_diags(preflight: dict[str, Any]) -> list[dict[str, Any]]:
    out = []
    for sample in _list(preflight.get('sample_history')):
        diag = _dict(_dict(sample).get('internal_sampling_diagnostics'))
        if diag:
            out.append(diag)
    return out


def collect_scan(diags: list[dict[str, Any]]) -> dict[str, Any]:
    ages: list[float] = []
    frames: set[str] = set()
    callback_max = 0
    publisher_max = 0
    for diag in diags:
        scan = _dict(diag.get('scan_subscription'))
        age = _num(scan.get('latest_age_sec'))
        if age is not None:
            ages.append(age)
        if scan.get('latest_frame_id'):
            frames.add(str(scan.get('latest_frame_id')))
        callback_max = max(callback_max, int(_num(scan.get('callback_count_total')) or 0))
        publisher_max = max(publisher_max, int(_num(scan.get('publisher_count')) or 0))
    max_age = max(ages) if ages else None
    return {
        'callback_count_max': callback_max,
        'publisher_count_max': publisher_max,
        'latest_frame_ids': sorted(frames),
        'latest_age_sec_values': ages,
        'max_scan_age_sec': max_age,
        'wall_time_sized_age_detected': bool(max_age is not None and max_age > WALL_TIME_SIZED_AGE_SEC),
    }


def _tf_pair(diags: list[dict[str, Any]], key: str) -> dict[str, Any]:
    can = False
    lookup = False
    ages: list[float] = []
    exceptions: list[str] = []
    for diag in diags:
        pair = _dict(_dict(diag.get('tf_buffer')).get(key))
        can = can or _bool(pair.get('can_transform'))
        lookup = lookup or _bool(pair.get('lookup_success'))
        age = _num(pair.get('age_sec'))
        if age is not None:
            ages.append(age)
        for field in ['lookup_exception', 'can_transform_exception']:
            if pair.get(field):
                exceptions.append(str(pair.get(field)))
    return {
        'any_can_transform': can,
        'any_lookup_success': lookup,
        'age_sec_values': ages,
        'latest_exception': exceptions[-1] if exceptions else None,
    }


def collect_tf(diags: list[dict[str, Any]]) -> dict[str, Any]:
    pairs = {
        'map->base_link': _tf_pair(diags, 'map->base_link'),
        'map->odom': _tf_pair(diags, 'map->odom'),
        'odom->base_link': _tf_pair(diags, 'odom->base_link'),
    }
    return {
        'pairs': pairs,
        'map_base_any_can_transform': pairs['map->base_link']['any_can_transform'],
        'map_odom_any_can_transform': pairs['map->odom']['any_can_transform'],
        'odom_base_any_can_transform': pairs['odom->base_link']['any_can_transform'],
        'all_core_tf_can_transform': all(pair['any_can_transform'] for pair in pairs.values()),
    }


def collect_sim_time(preflight: dict[str, Any], diags: list[dict[str, Any]]) -> dict[str, Any]:
    values = []
    for diag in diags:
        clock = _dict(diag.get('clock'))
        if 'use_sim_time' in clock:
            values.append(_bool(clock.get('use_sim_time')))
    return {
        'artifact_use_sim_time': _bool(preflight.get('use_sim_time')),
        'sample_use_sim_time_values': sorted(set(values)),
        'all_samples_use_sim_time': bool(values) and all(values),
    }


def classify(preflight: dict[str, Any], sim_time: dict[str, Any], scan: dict[str, Any], tf: dict[str, Any]) -> tuple[str, list[str]]:
    findings: list[str] = []
    reject = preflight.get('ingress_preflight_reject_reason')
    if not sim_time.get('artifact_use_sim_time'):
        findings.append('PRELIGHT_USE_SIM_TIME_FALSE')
    if sim_time.get('sample_use_sim_time_values') and not sim_time.get('all_samples_use_sim_time'):
        findings.append('SAMPLE_CLOCK_NOT_ALL_SIM_TIME')
    if scan.get('wall_time_sized_age_detected'):
        findings.append('WALL_TIME_SIZED_SCAN_AGE_STILL_PRESENT')
    if reject == 'ingress_first_scan_timeout':
        findings.append('REJECT_REASON_STILL_INGRESS_FIRST_SCAN_TIMEOUT')
    if scan.get('callback_count_max', 0) <= 0:
        findings.append('NO_SCAN_CALLBACK_OBSERVED')
    if not tf.get('map_base_any_can_transform'):
        findings.append('MAP_BASE_TF_STILL_MISSING_INTERNALLY')

    old_signature = any(f in findings for f in ['PRELIGHT_USE_SIM_TIME_FALSE', 'WALL_TIME_SIZED_SCAN_AGE_STILL_PRESENT', 'REJECT_REASON_STILL_INGRESS_FIRST_SCAN_TIMEOUT'])
    if old_signature:
        return 'SIM_TIME_ALIGNMENT_NOT_EFFECTIVE', findings
    if preflight.get('passed') is True:
        return 'SIM_TIME_ALIGNMENT_RECOVERED_PREFLIGHT_PASSED_NO_GOAL', findings
    if sim_time.get('artifact_use_sim_time') and not scan.get('wall_time_sized_age_detected') and reject != 'ingress_first_scan_timeout':
        return 'SIM_TIME_ALIGNMENT_RECOVERED_FAIL_CLOSED_NEW_REASON', findings
    return 'SIM_TIME_ALIGNMENT_INSUFFICIENT_EVIDENCE', findings


def analyze_artifact(artifact: dict[str, Any]) -> dict[str, Any]:
    preflight = _dict(artifact.get('ingress_preflight'))
    diags = sample_diags(preflight)
    scan = collect_scan(diags)
    tf = collect_tf(diags)
    sim_time = collect_sim_time(preflight, diags)
    classification, findings = classify(preflight, sim_time, scan, tf)
    guardrails = {
        'ingress_goal_sent': _bool(preflight.get('ingress_goal_sent')),
        'maze_explorer_started': _bool(preflight.get('maze_explorer_started')),
        'no_goal_dispatch_guard_valid': not _bool(preflight.get('ingress_goal_sent')) and not _bool(preflight.get('maze_explorer_started')),
        'nav2_config_tuned': False,
        'exploration_strategy_changed': False,
        'preflight_removed': False,
    }
    return {
        'phase': 'Phase114',
        'mode': 'preflight_sim_time_alignment_fix_validation',
        'classification': classification,
        'findings': findings,
        'preflight': {
            'evaluated': _bool(preflight.get('evaluated')),
            'passed': _bool(preflight.get('passed')),
            'reject_reason': preflight.get('ingress_preflight_reject_reason'),
            'failed_gates': _list(preflight.get('failed_gates')),
            'sample_count': int(_num(preflight.get('sample_count')) or len(_list(preflight.get('sample_history')))),
        },
        'sim_time': sim_time,
        'scan': scan,
        'tf': tf,
        'guardrails': guardrails,
        'diagnosis_only': False,
        'no_goal_validation_only': True,
    }


def write_summary(analysis: dict[str, Any], path: Path) -> None:
    lines = [
        '# Phase114 preflight sim-time alignment summary',
        '',
        f"classification: {analysis['classification']}",
        f"findings: {analysis['findings']}",
        f"passed: {analysis['preflight']['passed']}",
        f"reject_reason: {analysis['preflight']['reject_reason']}",
        '',
        '## Sim time',
        f"- artifact_use_sim_time: {analysis['sim_time']['artifact_use_sim_time']}",
        f"- all_samples_use_sim_time: {analysis['sim_time']['all_samples_use_sim_time']}",
        '',
        '## Scan',
        f"- callback_count_max: {analysis['scan']['callback_count_max']}",
        f"- max_scan_age_sec: {analysis['scan']['max_scan_age_sec']}",
        f"- wall_time_sized_age_detected: {analysis['scan']['wall_time_sized_age_detected']}",
        '',
        '## TF',
        f"- map_base_any_can_transform: {analysis['tf']['map_base_any_can_transform']}",
        f"- map_odom_any_can_transform: {analysis['tf']['map_odom_any_can_transform']}",
        f"- odom_base_any_can_transform: {analysis['tf']['odom_base_any_can_transform']}",
        '',
        '## Guardrails',
        f"- ingress_goal_sent: {analysis['guardrails']['ingress_goal_sent']}",
        f"- maze_explorer_started: {analysis['guardrails']['maze_explorer_started']}",
        f"- no_goal_dispatch_guard_valid: {analysis['guardrails']['no_goal_dispatch_guard_valid']}",
        '- No NavigateToPose goal was sent.',
        '- No maze_explorer was started.',
        '- No Nav2/MPPI/controller/config tuning was performed.',
        '- No autonomous exploration success or exit success is claimed.',
        '- Phase115 not entered.',
        '',
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines), encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact', type=Path, required=True)
    parser.add_argument('--output-json', type=Path, required=True)
    parser.add_argument('--minimal-summary-output', type=Path, required=True)
    args = parser.parse_args()
    artifact = json.loads(args.artifact.read_text(encoding='utf-8'))
    analysis = analyze_artifact(artifact)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(analysis, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    write_summary(analysis, args.minimal_summary_output)
    print(json.dumps({
        'analysis': str(args.output_json),
        'summary': str(args.minimal_summary_output),
        'classification': analysis['classification'],
        'no_goal_dispatch_guard_valid': analysis['guardrails']['no_goal_dispatch_guard_valid'],
        'reject_reason': analysis['preflight']['reject_reason'],
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
