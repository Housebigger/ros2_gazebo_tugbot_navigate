#!/usr/bin/env python3
"""Phase116 analyzer: diagnose preflight TF/scan/controller gate failures."""
from __future__ import annotations

import argparse
import json
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


def _num(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _load_json(path: Path | None) -> dict[str, Any]:
    if not path:
        return {}
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        return value if isinstance(value, dict) else {}
    except Exception:
        return {}


def _samples(preflight: dict[str, Any]) -> list[dict[str, Any]]:
    return [_dict(s) for s in _list(preflight.get('sample_history'))]


def _latest_good_sample(samples: list[dict[str, Any]]) -> dict[str, Any] | None:
    for sample in reversed(samples):
        if not _bool(sample.get('force_timeout')) and not _list(sample.get('failed_gates')):
            return sample
        if _bool(_dict(sample.get('map_base_tf_check')).get('available')) and _bool(_dict(sample.get('scan_transform_check')).get('transform_available')) and _bool(_dict(sample.get('controller_pose_check')).get('robot_pose_available')):
            return sample
    return None


def _tf_entry(sample: dict[str, Any], pair: str) -> dict[str, Any]:
    return _dict(_dict(sample.get('internal_sampling_diagnostics')).get('tf_buffer')).get(pair, {})


def collect_tf_timeline(preflight: dict[str, Any], external_timeline: dict[str, Any] | None = None) -> dict[str, Any]:
    pairs = ['map->base_link', 'map->odom', 'odom->base_link']
    samples = _samples(preflight)
    result: dict[str, Any] = {}
    for pair in pairs:
        points = []
        for sample in samples:
            entry = _dict(_tf_entry(sample, pair))
            if not entry and pair == 'map->base_link':
                mb = _dict(sample.get('map_base_tf_check'))
                entry = {'can_transform': mb.get('available'), 'lookup_success': mb.get('available'), 'age_sec': sample.get('map_base_tf_age_sec')}
            elif not entry and pair == 'map->odom':
                entry = {'age_sec': sample.get('map_odom_tf_age_sec'), 'can_transform': sample.get('map_odom_tf_age_sec') is not None, 'lookup_success': sample.get('map_odom_tf_age_sec') is not None}
            elif not entry and pair == 'odom->base_link':
                entry = {'age_sec': sample.get('odom_base_tf_age_sec'), 'can_transform': sample.get('odom_base_tf_age_sec') is not None, 'lookup_success': sample.get('odom_base_tf_age_sec') is not None}
            points.append({
                'elapsed_sec': sample.get('elapsed_sec'),
                'available': _bool(entry.get('lookup_success')) or _bool(entry.get('can_transform')),
                'can_transform': _bool(entry.get('can_transform')),
                'lookup_success': _bool(entry.get('lookup_success')),
                'age_sec': entry.get('age_sec'),
                'failure_reason': entry.get('failure_reason'),
                'lookup_exception': entry.get('lookup_exception'),
                'can_transform_exception': entry.get('can_transform_exception'),
                'sample_wall_time_sec': entry.get('sample_wall_time_sec') or sample.get('wall_time_sec'),
                'sample_ros_time_sec': entry.get('sample_ros_time_sec'),
                'buffer_frames_present': bool(entry.get('buffer_frames') or _dict(sample.get('internal_sampling_diagnostics')).get('tf_buffer_frame_list')),
            })
        ext_points = _list((external_timeline or {}).get(pair))
        recovered_internal = any(p['available'] for p in points) and any(not p['available'] for p in points)
        result[pair] = {
            'internal': points,
            'external': ext_points,
            'ever_available_internal': any(p['available'] for p in points),
            'initial_missing_internal': bool(points and not points[0]['available']),
            'recovered_internal': recovered_internal,
            'ever_available_external': any(_bool(_dict(p).get('available')) for p in ext_points),
        }
    return result


def collect_scan(preflight: dict[str, Any]) -> dict[str, Any]:
    samples = _samples(preflight)
    sequence = []
    for sample in samples:
        scan = _dict(sample.get('scan_transform_check'))
        diag = _dict(sample.get('internal_sampling_diagnostics'))
        sub = _dict(diag.get('scan_subscription'))
        tf_buffer = _dict(diag.get('tf_buffer'))
        scan_frame = scan.get('scan_frame_id') or sub.get('latest_frame_id')
        scan_pair = f'map->{scan_frame}' if scan_frame else 'map-><no_scan_frame>'
        tf_diag = _dict(tf_buffer.get(scan_pair))
        sequence.append({
            'elapsed_sec': sample.get('elapsed_sec'),
            'frame_id': scan_frame,
            'scan_age_sec': scan.get('age_sec') if scan.get('age_sec') is not None else sub.get('latest_age_sec'),
            'scan_stamp_sec': sub.get('latest_stamp_sec'),
            'callback_count_total': sub.get('callback_count_total'),
            'transform_available': _bool(scan.get('transform_available')),
            'transform_age_sec': scan.get('transform_age_sec') if scan.get('transform_age_sec') is not None else tf_diag.get('age_sec'),
            'failure_reason': scan.get('scan_transform_failure_reason') or scan.get('failure_reason') or tf_diag.get('failure_reason'),
            'lookup_exception': tf_diag.get('lookup_exception'),
        })
    latest = next((item for item in reversed(sequence) if item.get('frame_id')), sequence[-1] if sequence else {})
    return {
        'frame_id': latest.get('frame_id'),
        'sequence': sequence,
        'first_frame_elapsed_sec': next((item.get('elapsed_sec') for item in sequence if item.get('frame_id')), None),
        'ever_transform_available': any(item['transform_available'] for item in sequence),
        'initial_missing_or_unavailable': bool(sequence and not sequence[0]['transform_available']),
        'latest_scan_age_sec': latest.get('scan_age_sec'),
        'latest_transform_age_sec': latest.get('transform_age_sec'),
    }


def collect_controller(preflight: dict[str, Any]) -> dict[str, Any]:
    sequence = []
    for sample in _samples(preflight):
        ctrl = _dict(sample.get('controller_pose_check'))
        diag = _dict(sample.get('internal_sampling_diagnostics'))
        direct = ctrl.get('robot_pose_unavailable_reason') or diag.get('controller_direct_reason')
        if direct is None and not _bool(ctrl.get('robot_pose_available')):
            direct = 'map_base_tf_missing' if 'ingress_map_base_tf_missing' in _list(sample.get('failed_gates')) else 'robot_pose_unavailable_unknown'
        sequence.append({
            'elapsed_sec': sample.get('elapsed_sec'),
            'robot_pose_available': _bool(ctrl.get('robot_pose_available')),
            'direct_reason': direct,
            'controller_server_active': _bool(ctrl.get('controller_server_active')),
            'bt_navigator_active': _bool(ctrl.get('bt_navigator_active')),
            'action_ready': _bool(ctrl.get('navigate_to_pose_action_ready')),
        })
    return {
        'sequence': sequence,
        'direct_reason_sequence': [item['direct_reason'] for item in sequence],
        'ever_robot_pose_available': any(item['robot_pose_available'] for item in sequence),
        'initial_unavailable': bool(sequence and not sequence[0]['robot_pose_available']),
    }


def classify(preflight: dict[str, Any], tf: dict[str, Any], scan: dict[str, Any], controller: dict[str, Any]) -> tuple[str, list[str]]:
    findings: list[str] = []
    failed = set(_list(preflight.get('failed_gates')))
    internal_recovered = all(_dict(tf[p]).get('ever_available_internal') for p in ['map->base_link', 'map->odom', 'odom->base_link'])
    initial_missing = any(_dict(tf[p]).get('initial_missing_internal') for p in ['map->base_link', 'map->odom', 'odom->base_link'])
    if initial_missing and internal_recovered:
        findings.append('STARTUP_TF_BUFFER_FILL_DELAY')
    if scan.get('initial_missing_or_unavailable') and scan.get('ever_transform_available'):
        findings.append('SCAN_TRANSFORM_RECOVERED_AFTER_TF_BUFFER_FILL')
    if controller.get('initial_unavailable') and controller.get('ever_robot_pose_available'):
        findings.append('CONTROLLER_POSE_RECOVERED_AFTER_MAP_BASE_TF_AVAILABLE')
    if scan.get('frame_id') and '/' in str(scan.get('frame_id')):
        findings.append('SCAN_FRAME_NAMESPACED_BUT_MATCHED_BY_TF_WHEN_BUFFER_FILLED')
    if preflight.get('passed') is True:
        return 'TF_SCAN_CONTROLLER_GATES_PASSED_NO_GOAL', findings
    if internal_recovered and scan.get('ever_transform_available') and controller.get('ever_robot_pose_available') and 'ingress_preflight_timeout' in failed:
        return 'TF_SCAN_CONTROLLER_RECOVERED_AFTER_STARTUP_DELAY_FAIL_CLOSED_TIMEOUT', findings
    if not internal_recovered:
        return 'TF_BUFFER_NEVER_FILLED_FAIL_CLOSED', findings
    if not scan.get('ever_transform_available'):
        return 'SCAN_TRANSFORM_NEVER_AVAILABLE_FAIL_CLOSED', findings
    if not controller.get('ever_robot_pose_available'):
        return 'CONTROLLER_POSE_NEVER_AVAILABLE_FAIL_CLOSED', findings
    return 'TF_SCAN_CONTROLLER_GATE_DIAGNOSIS_INSUFFICIENT_EVIDENCE', findings


def analyze_artifact(artifact: dict[str, Any], external_timeline: dict[str, Any] | None = None) -> dict[str, Any]:
    preflight = _dict(artifact.get('ingress_preflight'))
    tf = collect_tf_timeline(preflight, external_timeline or {})
    scan = collect_scan(preflight)
    controller = collect_controller(preflight)
    classification, findings = classify(preflight, tf, scan, controller)
    guardrails = {
        'ingress_goal_sent': _bool(preflight.get('ingress_goal_sent')),
        'maze_explorer_started': _bool(preflight.get('maze_explorer_started')),
        'no_goal_dispatch_guard_valid': not _bool(preflight.get('ingress_goal_sent')) and not _bool(preflight.get('maze_explorer_started')),
        'nav2_config_tuned': False,
        'exploration_strategy_changed': False,
        'preflight_removed': False,
    }
    return {
        'phase': 'Phase116',
        'mode': 'preflight_tf_scan_controller_gate_diagnosis_only',
        'classification': classification,
        'findings': findings,
        'preflight': {
            'evaluated': _bool(preflight.get('evaluated')),
            'passed': _bool(preflight.get('passed')),
            'reject_reason': preflight.get('ingress_preflight_reject_reason'),
            'failed_gates': _list(preflight.get('failed_gates')),
            'sample_count': int(_num(preflight.get('sample_count')) or len(_samples(preflight))),
        },
        'tf': tf,
        'scan': scan,
        'controller': controller,
        'guardrails': guardrails,
        'no_goal_validation_only': True,
    }


def write_summary(analysis: dict[str, Any], path: Path) -> None:
    lines = [
        '# Phase116 TF/scan/controller gate summary',
        '',
        f"classification: {analysis['classification']}",
        f"findings: {analysis['findings']}",
        f"passed: {analysis['preflight']['passed']}",
        f"reject_reason: {analysis['preflight']['reject_reason']}",
        f"failed_gates: {analysis['preflight']['failed_gates']}",
        '',
        '## TF',
    ]
    for pair, info in analysis['tf'].items():
        lines.append(f"- {pair}: initial_missing={info['initial_missing_internal']}, recovered_internal={info['recovered_internal']}, ever_available_internal={info['ever_available_internal']}, ever_available_external={info['ever_available_external']}")
    lines.extend([
        '',
        '## Scan',
        f"- frame_id: {analysis['scan']['frame_id']}",
        f"- first_frame_elapsed_sec: {analysis['scan']['first_frame_elapsed_sec']}",
        f"- ever_transform_available: {analysis['scan']['ever_transform_available']}",
        '',
        '## Controller',
        f"- direct_reason_sequence: {analysis['controller']['direct_reason_sequence']}",
        f"- ever_robot_pose_available: {analysis['controller']['ever_robot_pose_available']}",
        '',
        '## Guardrails',
        f"- ingress_goal_sent: {analysis['guardrails']['ingress_goal_sent']}",
        f"- maze_explorer_started: {analysis['guardrails']['maze_explorer_started']}",
        f"- no_goal_dispatch_guard_valid: {analysis['guardrails']['no_goal_dispatch_guard_valid']}",
        '- No NavigateToPose goal was sent.',
        '- No maze_explorer was started.',
        '- No Nav2/MPPI/controller/config tuning was performed.',
        '- No autonomous exploration success or exit success is claimed.',
        '- Phase117 not entered.',
        '',
    ])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines), encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact', type=Path, required=True)
    parser.add_argument('--external-tf-timeline-json', type=Path)
    parser.add_argument('--output-json', type=Path, required=True)
    parser.add_argument('--minimal-summary-output', type=Path, required=True)
    args = parser.parse_args()
    artifact = json.loads(args.artifact.read_text(encoding='utf-8'))
    external = _load_json(args.external_tf_timeline_json)
    analysis = analyze_artifact(artifact, external_timeline=external)
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
