#!/usr/bin/env python3
"""Phase113 analyzer: diagnose preflight internal sampling discrepancy.

This is diagnosis-only tooling. It consumes a real Phase105/109/112 preflight
artifact plus optional external ROS snapshot/launch logs and classifies why the
preflight could still reject with ingress_first_scan_timeout / TF-missing while
external tools can see /scan and map->base_link. It does not send goals, start
maze_explorer, tune Nav2, or alter preflight safety policy.
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y'}
    return bool(value)


def _read_text(path: Path | None) -> str:
    if path is None or not path.exists():
        return ''
    return path.read_text(encoding='utf-8', errors='replace')


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding='utf-8', errors='replace'))
    return value if isinstance(value, dict) else {}


def parse_external_snapshot(text: str) -> dict[str, Any]:
    lower = text.lower()
    scan_available = bool(
        re.search(r'frame_id:\s*[\w/.-]+', text)
        or 'ranges:' in lower
        or '/scan ready' in lower
        or 'scan once' in lower and 'frame_id' in lower
    )
    frames = sorted(set(re.findall(r'frame_id:\s*([\w/.-]+)', text)))
    map_base_tf_available = bool(
        'translation:' in lower
        or 'transform:' in lower and ('base_link' in lower or 'child_frame_id' in lower)
        or 'tf ready: map->base_link' in lower
    )
    lifecycle_active = 'active [3]' in lower or 'managed nodes are active' in lower
    navigate_ready = '/navigate_to_pose ready' in lower or '/navigate_to_pose' in lower
    return {
        'scan_available': scan_available,
        'scan_frame_ids': frames,
        'map_base_tf_available': map_base_tf_available,
        'lifecycle_active_marker_seen': lifecycle_active,
        'navigate_to_pose_action_seen': navigate_ready,
    }


def _samples(preflight: dict[str, Any]) -> list[dict[str, Any]]:
    return [sample for sample in _list(preflight.get('sample_history')) if isinstance(sample, dict)]


def _sample_diag(sample: dict[str, Any]) -> dict[str, Any]:
    return _dict(sample.get('internal_sampling_diagnostics'))


def _max_int(values: list[Any]) -> int:
    out = 0
    for value in values:
        try:
            out = max(out, int(value))
        except (TypeError, ValueError):
            pass
    return out


def collect_internal_scan(samples: list[dict[str, Any]]) -> dict[str, Any]:
    diags = [_sample_diag(sample) for sample in samples]
    scan_diags = [_dict(diag.get('scan_subscription')) for diag in diags]
    scan_checks = [_dict(sample.get('scan_transform_check')) for sample in samples]
    callback_count_max = _max_int([diag.get('callback_count_total') for diag in scan_diags])
    publisher_count_max = _max_int([diag.get('publisher_count') for diag in scan_diags])
    callbacks_since_sample_max = _max_int([diag.get('callbacks_since_last_sample') for diag in scan_diags])
    latest_frame_ids = sorted({str(v) for v in [diag.get('latest_frame_id') for diag in scan_diags] + [check.get('scan_frame_id') for check in scan_checks] if v})
    latest_ages = [diag.get('latest_age_sec') for diag in scan_diags if diag.get('latest_age_sec') is not None]
    any_scan_available = any(_bool(check.get('scan_available')) for check in scan_checks)
    any_transform_available = any(_bool(check.get('transform_available')) for check in scan_checks)
    topics = sorted({str(diag.get('topic')) for diag in scan_diags if diag.get('topic')})
    return {
        'instrumented_samples': sum(1 for diag in scan_diags if diag),
        'topic_names': topics,
        'publisher_count_max': publisher_count_max,
        'callback_count_max': callback_count_max,
        'callbacks_since_sample_max': callbacks_since_sample_max,
        'latest_frame_ids': latest_frame_ids,
        'latest_age_sec_values': latest_ages,
        'any_scan_available_gate': any_scan_available,
        'any_scan_transform_available_gate': any_transform_available,
        'all_scan_unavailable_gate': bool(scan_checks) and not any_scan_available,
    }


def _tf_pair_from_diag(diag: dict[str, Any], pair: str) -> dict[str, Any]:
    tf = _dict(diag.get('tf_buffer'))
    return _dict(tf.get(pair))


def collect_internal_tf(samples: list[dict[str, Any]]) -> dict[str, Any]:
    diags = [_sample_diag(sample) for sample in samples]
    pair_names = ['map->base_link', 'map->odom', 'odom->base_link', 'map->tugbot/scan_omni/scan_omni']
    pairs: dict[str, dict[str, Any]] = {}
    for pair in pair_names:
        values = [_tf_pair_from_diag(diag, pair) for diag in diags]
        values = [value for value in values if value]
        pairs[pair] = {
            'instrumented_samples': len(values),
            'any_can_transform': any(_bool(value.get('can_transform')) for value in values),
            'any_lookup_success': any(_bool(value.get('lookup_success')) for value in values),
            'latest_exception': next((value.get('lookup_exception') or value.get('can_transform_exception') for value in reversed(values) if value.get('lookup_exception') or value.get('can_transform_exception')), None),
            'exceptions': [value.get('lookup_exception') or value.get('can_transform_exception') for value in values if value.get('lookup_exception') or value.get('can_transform_exception')],
        }
    map_base_checks = [_dict(sample.get('map_base_tf_check')) for sample in samples]
    return {
        'pairs': pairs,
        'map_base_any_available_gate': any(_bool(check.get('available')) for check in map_base_checks),
        'map_base_all_missing': bool(map_base_checks) and not any(_bool(check.get('available')) for check in map_base_checks),
        'map_base_latest_age_values': [sample.get('map_base_tf_age_sec') for sample in samples if sample.get('map_base_tf_age_sec') is not None],
    }


def collect_executor_clock(samples: list[dict[str, Any]]) -> dict[str, Any]:
    diags = [_sample_diag(sample) for sample in samples]
    executors = [_dict(diag.get('executor')) for diag in diags]
    clocks = [_dict(diag.get('clock')) for diag in diags]
    spin_max = _max_int([executor.get('spin_once_count_total') for executor in executors])
    use_sim_values = sorted({str(clock.get('use_sim_time')) for clock in clocks if 'use_sim_time' in clock})
    ros_times = [clock.get('ros_time_sec') for clock in clocks if clock.get('ros_time_sec') is not None]
    wall_times = [clock.get('wall_time_sec') for clock in clocks if clock.get('wall_time_sec') is not None]
    return {
        'instrumented_samples': sum(1 for diag in diags if diag),
        'spin_once_count_max': spin_max,
        'use_sim_time_values': use_sim_values,
        'ros_time_first_last': [ros_times[0], ros_times[-1]] if ros_times else [],
        'wall_time_first_last': [wall_times[0], wall_times[-1]] if wall_times else [],
    }


def classify(preflight: dict[str, Any], external: dict[str, Any], scan: dict[str, Any], tf: dict[str, Any], executor: dict[str, Any]) -> tuple[str, list[str]]:
    failed = set(_list(preflight.get('failed_gates')))
    reject = preflight.get('ingress_preflight_reject_reason')
    contributing: list[str] = []
    external_scan = bool(external.get('scan_available'))
    external_tf = bool(external.get('map_base_tf_available'))
    callback_max = int(scan.get('callback_count_max') or 0)
    if external_tf and tf.get('map_base_all_missing'):
        contributing.append('INTERNAL_TF_BUFFER_MISSING_WHILE_EXTERNAL_TF_AVAILABLE')
    use_sim_values = set(str(v) for v in executor.get('use_sim_time_values') or [])
    age_values = [float(v) for v in scan.get('latest_age_sec_values') or [] if isinstance(v, (int, float))]
    if 'False' in use_sim_values and age_values and max(age_values) > 1000.0:
        contributing.append('INTERNAL_NODE_USE_SIM_TIME_FALSE_WITH_SIM_STAMPED_SCAN')
    if int(executor.get('spin_once_count_max') or 0) == 0:
        contributing.append('INTERNAL_EXECUTOR_SPIN_NOT_OBSERVED')
    if reject == 'ingress_first_scan_timeout' or 'ingress_first_scan_timeout' in failed:
        if external_scan and callback_max == 0:
            return 'INTERNAL_SCAN_CALLBACK_STARVED_WHILE_EXTERNAL_SCAN_AVAILABLE', contributing
        if callback_max > 0 and not scan.get('any_scan_available_gate'):
            contributing.append('SCAN_CALLBACK_RECEIVED_BUT_SCAN_GATE_UNAVAILABLE')
            return 'INTERNAL_SCAN_RECEIVED_BUT_REJECTED_BY_AGE_OR_TF', contributing
        if callback_max > 0 and not scan.get('any_scan_transform_available_gate'):
            contributing.append('SCAN_CALLBACK_RECEIVED_BUT_SCAN_TF_UNAVAILABLE')
            return 'INTERNAL_SCAN_RECEIVED_BUT_REJECTED_BY_AGE_OR_TF', contributing
        return 'FIRST_SCAN_TIMEOUT_WITH_INSUFFICIENT_INTERNAL_DIAGNOSTICS', contributing
    if external_tf and tf.get('map_base_all_missing'):
        return 'INTERNAL_TF_BUFFER_MISSING_WHILE_EXTERNAL_TF_AVAILABLE', contributing
    return 'NO_INTERNAL_SAMPLING_DISCREPANCY_CLASSIFIED', contributing


def analyze_artifact(artifact: dict[str, Any], *, external_snapshot_text: str = '', launch_log_text: str = '') -> dict[str, Any]:
    preflight = _dict(artifact.get('ingress_preflight'))
    samples = _samples(preflight)
    external = parse_external_snapshot('\n'.join([external_snapshot_text, launch_log_text]))
    scan = collect_internal_scan(samples)
    tf = collect_internal_tf(samples)
    executor = collect_executor_clock(samples)
    classification, contributing = classify(preflight, external, scan, tf, executor)
    guardrails = {
        'ingress_goal_sent': _bool(preflight.get('ingress_goal_sent')),
        'maze_explorer_started': _bool(preflight.get('maze_explorer_started')),
        'no_goal_dispatch_guard_valid': (not _bool(preflight.get('ingress_goal_sent')) and not _bool(preflight.get('maze_explorer_started'))),
        'nav2_config_tuned': False,
        'exploration_strategy_changed': False,
        'preflight_removed': False,
    }
    return {
        'phase': 'Phase113',
        'mode': 'real_preflight_internal_sampling_discrepancy_diagnosis',
        'classification': classification,
        'contributing_findings': contributing,
        'preflight': {
            'evaluated': _bool(preflight.get('evaluated')),
            'passed': _bool(preflight.get('passed')),
            'reject_reason': preflight.get('ingress_preflight_reject_reason'),
            'failed_gates': _list(preflight.get('failed_gates')),
            'sample_count': len(samples),
        },
        'guardrails': guardrails,
        'external_snapshot': external,
        'scan_internal': scan,
        'tf_internal': tf,
        'executor_clock_internal': executor,
        'launch_log': {
            'managed_nav2_active_marker_seen': 'managed nodes are active' in launch_log_text.lower() or 'all lifecycle nodes reached active' in launch_log_text.lower(),
        },
        'diagnosis_only': True,
    }


def write_summary(analysis: dict[str, Any], path: Path) -> None:
    pre = _dict(analysis.get('preflight'))
    guard = _dict(analysis.get('guardrails'))
    scan = _dict(analysis.get('scan_internal'))
    tf = _dict(analysis.get('tf_internal'))
    ext = _dict(analysis.get('external_snapshot'))
    lines = [
        '# Phase113 internal sampling discrepancy summary',
        '',
        f"classification: {analysis.get('classification')}",
        f"contributing_findings: {analysis.get('contributing_findings')}",
        f"passed: {pre.get('passed')}",
        f"reject_reason: {pre.get('reject_reason')}",
        f"failed_gates: {pre.get('failed_gates')}",
        '',
        '## External snapshot',
        f"- scan_available: {ext.get('scan_available')}",
        f"- scan_frame_ids: {ext.get('scan_frame_ids')}",
        f"- map_base_tf_available: {ext.get('map_base_tf_available')}",
        '',
        '## Internal scan sampling',
        f"- topic_names: {scan.get('topic_names')}",
        f"- publisher_count_max: {scan.get('publisher_count_max')}",
        f"- callback_count_max: {scan.get('callback_count_max')}",
        f"- latest_frame_ids: {scan.get('latest_frame_ids')}",
        f"- any_scan_available_gate: {scan.get('any_scan_available_gate')}",
        f"- any_scan_transform_available_gate: {scan.get('any_scan_transform_available_gate')}",
        '',
        '## Internal TF sampling',
        f"- map_base_all_missing: {tf.get('map_base_all_missing')}",
        f"- map_base_any_available_gate: {tf.get('map_base_any_available_gate')}",
        f"- pairs: {tf.get('pairs')}",
        '',
        '## Guardrails',
        f"- ingress_goal_sent: {guard.get('ingress_goal_sent')}",
        f"- maze_explorer_started: {guard.get('maze_explorer_started')}",
        f"- no_goal_dispatch_guard_valid: {guard.get('no_goal_dispatch_guard_valid')}",
        '- No NavigateToPose goal was sent.',
        '- No maze_explorer was started.',
        '- No Nav2/MPPI/controller/config tuning was performed.',
        '- No autonomous exploration success or exit success is claimed.',
        '- Phase114 not entered.',
        '',
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines), encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact', type=Path, required=True)
    parser.add_argument('--external-snapshot', type=Path)
    parser.add_argument('--launch-log', type=Path)
    parser.add_argument('--output-json', type=Path, required=True)
    parser.add_argument('--minimal-summary-output', type=Path, required=True)
    args = parser.parse_args()

    artifact = _read_json(args.artifact)
    analysis = analyze_artifact(
        artifact,
        external_snapshot_text=_read_text(args.external_snapshot),
        launch_log_text=_read_text(args.launch_log),
    )
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(analysis, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    write_summary(analysis, args.minimal_summary_output)
    print(json.dumps({
        'analysis': str(args.output_json),
        'summary': str(args.minimal_summary_output),
        'classification': analysis.get('classification'),
        'no_goal_dispatch_guard_valid': analysis.get('guardrails', {}).get('no_goal_dispatch_guard_valid'),
        'reject_reason': analysis.get('preflight', {}).get('reject_reason'),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
