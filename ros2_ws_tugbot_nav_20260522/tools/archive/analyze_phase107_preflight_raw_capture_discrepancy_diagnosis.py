#!/usr/bin/env python3
"""Phase107 Phase105 preflight vs raw-capture discrepancy diagnosis.

Diagnosis-only analyzer. It reads Phase106 artifacts and compares the Phase105
preflight evidence with later raw-capture evidence. It does not call ROS, send
goals, edit preflight/tooling/runtime logic, or tune Nav2 parameters.
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

RUN_ID = 'phase107_preflight_raw_capture_discrepancy_diagnosis'
PHASE106_RUN_ID = 'phase106_preflighted_carry_over_bounded_goal1_staging_validation'

ALLOWED_CLASSIFICATIONS = {
    'PREFLIGHT_SAMPLING_TOO_EARLY',
    'PREFLIGHT_STABILITY_WINDOW_TOO_SHORT',
    'PREFLIGHT_FRAME_QUERY_MISMATCH',
    'PREFLIGHT_TOPIC_QUERY_MISMATCH',
    'PREFLIGHT_LIFECYCLE_READY_ORDERING_ISSUE',
    'PREFLIGHT_TOOL_FALSE_NEGATIVE',
    'RAW_CAPTURE_LATER_THAN_PREFLIGHT',
    'PREFLIGHT_RAW_CAPTURE_DISCREPANCY_INSUFFICIENT_EVIDENCE',
}

JSONDict = dict[str, Any]


def _safe_json(path: Path | None, default: Any = None) -> Any:
    if default is None:
        default = {}
    if path is None or not path.exists() or not path.stat().st_size:
        return default
    try:
        return json.loads(path.read_text(encoding='utf-8', errors='replace'))
    except Exception:
        return default


def _read_text(path: Path | None) -> str:
    if path is None or not path.exists():
        return ''
    return path.read_text(encoding='utf-8', errors='replace')


def _jsonl(path: Path | None) -> list[JSONDict]:
    rows: list[JSONDict] = []
    if path is None or not path.exists():
        return rows
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except Exception:
            continue
        if isinstance(value, dict):
            rows.append(value)
    return rows


def _dict(value: Any) -> JSONDict:
    return value if isinstance(value, dict) else {}


def _float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def _bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y', 'active', 'ready'}
    return bool(value)


def _first_existing(artifact_dir: Path, run_id: str, suffix: str) -> Path | None:
    exact = artifact_dir / f'{run_id}_{suffix}'
    if exact.exists():
        return exact
    matches = sorted(artifact_dir.glob(f'*_{suffix}'))
    return matches[0] if matches else None


def _load_bundle(artifact_dir: Path, phase106_run_id: str = PHASE106_RUN_ID) -> JSONDict:
    artifact_dir = Path(artifact_dir)
    paths = {
        'ingress_preflight': _first_existing(artifact_dir, phase106_run_id, 'ingress_preflight.json'),
        'raw_capture': _first_existing(artifact_dir, phase106_run_id, 'raw_capture.json'),
        'ingress_result': _first_existing(artifact_dir, phase106_run_id, 'ingress_result.json'),
        'analysis': _first_existing(artifact_dir, phase106_run_id, 'analysis.json'),
        'runtime_timeline': _first_existing(artifact_dir, phase106_run_id, 'runtime_timeline.jsonl'),
        'preflight_text': _first_existing(artifact_dir, phase106_run_id, 'preflight.txt'),
        'preflight_stdout': _first_existing(artifact_dir, phase106_run_id, 'ingress_preflight_stdout.json'),
        'preflight_stderr': _first_existing(artifact_dir, phase106_run_id, 'ingress_preflight_stderr.log'),
        'initial_cleanup': _first_existing(artifact_dir, phase106_run_id, 'initial_cleanup_summary.json'),
        'post_cleanup': _first_existing(artifact_dir, phase106_run_id, 'post_cleanup_summary.json'),
        'ros_graph_ready': _first_existing(artifact_dir, phase106_run_id, 'ros_graph_ready.txt'),
        'ros_graph_final': _first_existing(artifact_dir, phase106_run_id, 'ros_graph_final.txt'),
        'visible_launch': _first_existing(artifact_dir, phase106_run_id, 'visible_launch.log'),
        'runtime_recorder_stderr': _first_existing(artifact_dir, phase106_run_id, 'runtime_recorder_stderr.log'),
    }
    preflight_doc = _safe_json(paths['ingress_preflight'], {})
    return {
        'artifact_dir': str(artifact_dir),
        'phase106_run_id': phase106_run_id,
        'paths': {key: str(path) if path else None for key, path in paths.items()},
        'ingress_preflight_doc': preflight_doc,
        'ingress_preflight': _dict(preflight_doc.get('ingress_preflight')),
        'raw_capture': _safe_json(paths['raw_capture'], {}),
        'ingress_result': _safe_json(paths['ingress_result'], {}),
        'analysis': _safe_json(paths['analysis'], {}),
        'runtime_timeline_rows': _jsonl(paths['runtime_timeline']),
        'preflight_text': _read_text(paths['preflight_text']),
        'preflight_stdout_text': _read_text(paths['preflight_stdout']),
        'preflight_stderr_text': _read_text(paths['preflight_stderr']),
        'initial_cleanup': _safe_json(paths['initial_cleanup'], {}),
        'post_cleanup': _safe_json(paths['post_cleanup'], {}),
        'ros_graph_ready_text': _read_text(paths['ros_graph_ready']),
        'ros_graph_final_text': _read_text(paths['ros_graph_final']),
        'visible_launch_text': _read_text(paths['visible_launch']),
        'runtime_recorder_stderr_text': _read_text(paths['runtime_recorder_stderr']),
    }


def _raw_available(raw: JSONDict, key: str) -> bool:
    value = raw.get(key)
    if key == 'tf':
        tf = _dict(value)
        return any(_dict(v).get('available') is True for v in tf.values())
    return isinstance(value, dict) and bool(value)


def _sample_times(preflight: JSONDict) -> list[float]:
    times = []
    for sample in preflight.get('sample_tail') or []:
        ts = _float(_dict(sample).get('wall_time_sec'))
        if ts is not None:
            times.append(ts)
    return times


def timestamp_comparison(preflight: JSONDict, raw: JSONDict) -> JSONDict:
    start = _float(preflight.get('start_time_sec'))
    end = _float(preflight.get('end_time_sec'))
    timeout = _float(preflight.get('ingress_preflight_timeout_sec'))
    bounded = _float(preflight.get('bounded_wait_elapsed_sec'))
    raw_end = _float(raw.get('capture_wall_time'))
    raw_elapsed = _float(raw.get('elapsed_sec'))
    raw_start = raw_end - raw_elapsed if raw_end is not None and raw_elapsed is not None else None
    timeout_deadline = start + timeout if start is not None and timeout is not None else end
    sample_times = _sample_times(preflight)
    return {
        'preflight_start_wall_time_sec': start,
        'preflight_end_wall_time_sec': end,
        'preflight_timeout_sec': timeout,
        'preflight_timeout_deadline_wall_time_sec': timeout_deadline,
        'preflight_bounded_wait_elapsed_sec': bounded,
        'preflight_sample_wall_times_sec': sample_times,
        'preflight_sample_count': preflight.get('sample_count') or len(sample_times),
        'raw_capture_start_wall_time_sec': raw_start,
        'raw_capture_end_wall_time_sec': raw_end,
        'raw_capture_elapsed_sec': raw_elapsed,
        'raw_capture_started_after_preflight_end': bool(raw_start is not None and end is not None and raw_start > end),
        'raw_capture_started_after_preflight_timeout_deadline': bool(raw_start is not None and timeout_deadline is not None and raw_start > timeout_deadline),
        'raw_capture_end_after_preflight_end': bool(raw_end is not None and end is not None and raw_end > end),
        'seconds_between_preflight_end_and_raw_capture_start': (raw_start - end) if raw_start is not None and end is not None else None,
    }


def _latest_sample(preflight: JSONDict) -> JSONDict:
    samples = preflight.get('sample_tail') if isinstance(preflight.get('sample_tail'), list) else []
    return _dict(samples[-1]) if samples else {}


def topic_comparison(preflight: JSONDict, raw: JSONDict, preflight_text: str) -> JSONDict:
    # Phase105 subscribes to /scan in source; Phase106 preflight artifact may only
    # expose the observed scan frame, not the topic. The run preflight text can
    # carry audit notes in tests/future artifacts. Runtime source audit: /scan.
    preflight_scan_topic = '/scan'
    match = re.search(r'ingress_preflight_topic=([^\s]+)', preflight_text)
    if match:
        preflight_scan_topic = match.group(1).strip()
    raw_scan = _dict(raw.get('scan'))
    raw_map = _dict(raw.get('map'))
    raw_local = _dict(raw.get('local_costmap'))
    raw_odom = _dict(raw.get('odom'))
    return {
        'preflight_scan_topic': preflight_scan_topic,
        'raw_scan_topic': raw_scan.get('topic'),
        'scan_topic_same': bool(raw_scan.get('topic') in (None, preflight_scan_topic)),
        'raw_map_topic': raw_map.get('topic'),
        'raw_local_costmap_topic': raw_local.get('topic'),
        'raw_odom_topic': raw_odom.get('topic'),
        'preflight_expected_topics': ['/scan', '/navigate_to_pose', '/controller_server', '/bt_navigator', '/global_costmap/global_costmap', '/local_costmap/local_costmap'],
        'raw_capture_topics': [topic for topic in [raw_scan.get('topic'), raw_map.get('topic'), raw_local.get('topic'), raw_odom.get('topic')] if topic],
    }


def frame_comparison(preflight: JSONDict, raw: JSONDict) -> JSONDict:
    latest = _latest_sample(preflight)
    preflight_scan = _dict(preflight.get('scan_transform_check')) or _dict(latest.get('scan_transform_check'))
    preflight_goal = _dict(preflight.get('goal_pose_transform_check')) or _dict(latest.get('goal_pose_transform_check'))
    raw_scan = _dict(raw.get('scan'))
    raw_map = _dict(raw.get('map'))
    raw_local = _dict(raw.get('local_costmap'))
    raw_odom = _dict(raw.get('odom'))
    tf = _dict(raw.get('tf'))
    preflight_scan_frame = preflight_scan.get('scan_frame_id')
    raw_scan_frame = raw_scan.get('frame_id')
    if preflight_scan_frame is None:
        scan_frame_same: bool | None = None
    else:
        scan_frame_same = preflight_scan_frame == raw_scan_frame
    core_tf_frames = ['map->base_link', 'map->odom', 'odom->base_link']
    return {
        'preflight_scan_frame_id': preflight_scan_frame,
        'raw_scan_frame_id': raw_scan_frame,
        'scan_frame_same': scan_frame_same,
        'preflight_scan_target_frame': preflight_scan.get('target_frame'),
        'preflight_goal_frame_id': preflight_goal.get('goal_frame_id'),
        'preflight_global_costmap_frame': preflight_goal.get('global_costmap_frame'),
        'preflight_controller_frame': preflight_goal.get('controller_frame'),
        'raw_map_frame_id': raw_map.get('frame_id'),
        'raw_local_costmap_frame_id': raw_local.get('frame_id'),
        'raw_odom_frame_id': raw_odom.get('frame_id'),
        'raw_odom_child_frame_id': raw_odom.get('child_frame_id'),
        'raw_tf_core_available': {key: bool(_dict(tf.get(key)).get('available')) for key in core_tf_frames},
        'core_tf_frames_same': bool(raw_map.get('frame_id') in (None, 'map') and raw_local.get('frame_id') in (None, 'odom') and raw_odom.get('child_frame_id') in (None, 'base_link')),
        'raw_scan_static_tf_errors': {key: _dict(value).get('error') for key, value in tf.items() if isinstance(value, dict) and ('scan' in key or 'base_scan' in key) and value.get('available') is False},
    }


def _extract_epoch_times(text: str, containing: str | None = None) -> list[float]:
    times: list[float] = []
    for line in text.splitlines():
        if containing and containing not in line:
            continue
        # Runtime launch logs use epoch-like stamps such as [1780476297.266].
        # Focused tests use compact synthetic stamps such as [125.0].  Parse
        # bracketed numeric values with decimals; process labels such as
        # [lifecycle_manager-17] and [INFO] are naturally ignored.
        for match in re.finditer(r'\[(\d+(?:\.\d+)?)\]', line):
            ts = _float(match.group(1))
            if ts is not None:
                times.append(ts)
    return times


def lifecycle_comparison(preflight: JSONDict, ready_text: str, launch_text: str, final_text: str) -> JSONDict:
    controller = _dict(preflight.get('controller_pose_check')) or _dict(_latest_sample(preflight).get('controller_pose_check'))
    start = _float(preflight.get('start_time_sec'))
    end = _float(preflight.get('end_time_sec'))
    managed_times = _extract_epoch_times(launch_text, 'Managed nodes are active')
    controller_bond_times = _extract_epoch_times(launch_text, 'Server controller_server connected with bond')
    bt_bond_times = _extract_epoch_times(launch_text, 'Server bt_navigator connected with bond')
    ready_times = _extract_epoch_times(ready_text)
    managed_time = min(managed_times) if managed_times else None
    controller_bond_time = min(controller_bond_times) if controller_bond_times else None
    bt_bond_time = min(bt_bond_times) if bt_bond_times else None
    ready_time = min(ready_times) if ready_times else None
    final_has_nodes = '/controller_server' in final_text and '/bt_navigator' in final_text
    before_start = bool(managed_time is not None and start is not None and managed_time < start)
    after_end = bool(managed_time is not None and end is not None and managed_time > end)
    during = bool(managed_time is not None and start is not None and end is not None and start <= managed_time <= end)
    return {
        'preflight_controller_server_active': _bool(controller.get('controller_server_active')),
        'preflight_bt_navigator_active': _bool(controller.get('bt_navigator_active')),
        'preflight_navigate_to_pose_action_ready': _bool(controller.get('navigate_to_pose_action_ready')),
        'preflight_robot_pose_available': _bool(controller.get('robot_pose_available')),
        'ros_graph_ready_marker_before_preflight': bool(ready_time is not None and start is not None and ready_time < start),
        'ros_graph_ready_first_epoch_sec': ready_time,
        'managed_nodes_active_time_sec': managed_time,
        'controller_server_bond_time_sec': controller_bond_time,
        'bt_navigator_bond_time_sec': bt_bond_time,
        'managed_nodes_active_before_preflight_start': before_start,
        'managed_nodes_active_during_preflight': during,
        'managed_nodes_active_after_preflight_end': after_end,
        'ros_graph_final_has_controller_and_bt': final_has_nodes,
    }


def query_method_comparison(preflight: JSONDict, raw: JSONDict) -> JSONDict:
    raw_tf = _dict(raw.get('tf'))
    raw_available_keys = [key for key, value in raw_tf.items() if isinstance(value, dict) and value.get('available') is True]
    return {
        'preflight_tf_query_method': 'tf2 Buffer.lookup_transform(parent, child, Time()) plus can_transform for scan/goal frames',
        'preflight_scan_query_method': 'subscribe /scan, then require scan freshness and can_transform(map, scan_frame)',
        'preflight_lifecycle_query_method': 'ros2 lifecycle get /controller_server and /bt_navigator subprocess checks plus ros2 action list',
        'raw_capture_query_method': 'read-only subscriptions for /scan,/map,/local_costmap/costmap,/odom plus tf2 Buffer.lookup_transform snapshot after Phase106 analyzer',
        'raw_capture_tf_pairs': sorted(raw_available_keys),
        'query_method_same': False,
    }


def _available_summary(raw: JSONDict) -> JSONDict:
    return {
        'raw_capture_present': bool(raw),
        'scan_available': _raw_available(raw, 'scan'),
        'map_available': _raw_available(raw, 'map'),
        'local_costmap_available': _raw_available(raw, 'local_costmap'),
        'odom_available': _raw_available(raw, 'odom'),
        'tf_available': _raw_available(raw, 'tf'),
    }


def _evidence_gaps(bundle: JSONDict, raw_summary: JSONDict, timing: JSONDict) -> list[str]:
    gaps: list[str] = []
    if not bundle.get('ingress_preflight'):
        gaps.append('missing_ingress_preflight')
    if not bundle.get('raw_capture'):
        gaps.append('missing_raw_capture')
    if not bundle.get('runtime_timeline_rows'):
        gaps.append('missing_runtime_timeline_samples')
    if not bundle.get('preflight_stdout_text'):
        gaps.append('missing_or_empty_preflight_stdout')
    if timing.get('preflight_start_wall_time_sec') is None or timing.get('preflight_end_wall_time_sec') is None:
        gaps.append('missing_preflight_wall_timestamps')
    if raw_summary.get('raw_capture_present') and timing.get('raw_capture_start_wall_time_sec') is None:
        gaps.append('missing_raw_capture_wall_timestamps')
    return gaps


def _classify(
    raw_summary: JSONDict,
    timing: JSONDict,
    topics: JSONDict,
    frames: JSONDict,
    lifecycle: JSONDict,
    preflight: JSONDict,
    gaps: list[str],
) -> tuple[str, list[str]]:
    if 'missing_ingress_preflight' in gaps or 'missing_raw_capture' in gaps or not raw_summary.get('raw_capture_present'):
        return 'PREFLIGHT_RAW_CAPTURE_DISCREPANCY_INSUFFICIENT_EVIDENCE', []

    contributing: list[str] = []
    if timing.get('raw_capture_started_after_preflight_timeout_deadline') or timing.get('raw_capture_started_after_preflight_end'):
        contributing.append('RAW_CAPTURE_LATER_THAN_PREFLIGHT')

    if topics.get('scan_topic_same') is False:
        return 'PREFLIGHT_TOPIC_QUERY_MISMATCH', contributing
    if frames.get('scan_frame_same') is False:
        return 'PREFLIGHT_FRAME_QUERY_MISMATCH', contributing

    controller_false = not lifecycle.get('preflight_controller_server_active') or not lifecycle.get('preflight_bt_navigator_active')
    if controller_false and lifecycle.get('managed_nodes_active_after_preflight_end'):
        contributing.append('PREFLIGHT_LIFECYCLE_READY_ORDERING_ISSUE')
        return 'PREFLIGHT_LIFECYCLE_READY_ORDERING_ISSUE', contributing

    if controller_false and (lifecycle.get('managed_nodes_active_before_preflight_start') or lifecycle.get('ros_graph_ready_marker_before_preflight')):
        contributing.append('PREFLIGHT_TOOL_FALSE_NEGATIVE')
        return 'PREFLIGHT_TOOL_FALSE_NEGATIVE', contributing

    sample_count = int(preflight.get('sample_count') or len(preflight.get('sample_tail') or []))
    if sample_count <= 1 and raw_summary.get('tf_available'):
        contributing.append('PREFLIGHT_SAMPLING_TOO_EARLY')
        return 'PREFLIGHT_SAMPLING_TOO_EARLY', contributing

    stable_window = _float(preflight.get('tf_stability_window_sec'))
    bounded = _float(preflight.get('bounded_wait_elapsed_sec'))
    if stable_window is not None and bounded is not None and stable_window >= bounded and raw_summary.get('tf_available'):
        contributing.append('PREFLIGHT_STABILITY_WINDOW_TOO_SHORT')
        return 'PREFLIGHT_STABILITY_WINDOW_TOO_SHORT', contributing

    if contributing:
        return contributing[0], contributing
    return 'PREFLIGHT_RAW_CAPTURE_DISCREPANCY_INSUFFICIENT_EVIDENCE', contributing


def analyze_phase106_artifacts(artifact_dir: Path, phase106_run_id: str = PHASE106_RUN_ID) -> JSONDict:
    bundle = _load_bundle(Path(artifact_dir), phase106_run_id=phase106_run_id)
    preflight = bundle['ingress_preflight']
    raw = bundle['raw_capture']
    timing = timestamp_comparison(preflight, raw)
    topics = topic_comparison(preflight, raw, bundle.get('preflight_text') or '')
    frames = frame_comparison(preflight, raw)
    lifecycle = lifecycle_comparison(
        preflight,
        bundle.get('ros_graph_ready_text') or '',
        bundle.get('visible_launch_text') or '',
        bundle.get('ros_graph_final_text') or '',
    )
    methods = query_method_comparison(preflight, raw)
    raw_summary = _available_summary(raw)
    gaps = _evidence_gaps(bundle, raw_summary, timing)
    classification, contributing = _classify(raw_summary, timing, topics, frames, lifecycle, preflight, gaps)
    # Keep unique ordered contributing classifications and include primary.
    ordered_contributing: list[str] = []
    for token in [classification] + contributing:
        if token in ALLOWED_CLASSIFICATIONS and token not in ordered_contributing:
            ordered_contributing.append(token)

    raw_after_timeout = bool(timing.get('raw_capture_started_after_preflight_timeout_deadline') or timing.get('raw_capture_started_after_preflight_end'))
    frame_or_topic_diff = bool(topics.get('scan_topic_same') is False or frames.get('scan_frame_same') is False)
    startup_ordering = bool(lifecycle.get('managed_nodes_active_after_preflight_end'))

    result: JSONDict = {
        'run_id': RUN_ID,
        'source_phase106_run_id': phase106_run_id,
        'artifact_dir': str(Path(artifact_dir)),
        'classification': classification,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'contributing_classifications': ordered_contributing,
        'evidence_gaps': gaps,
        'raw_capture_summary': raw_summary,
        'timestamp_comparison': timing,
        'topic_comparison': topics,
        'frame_comparison': frames,
        'query_method_comparison': methods,
        'lifecycle_comparison': lifecycle,
        'preflight_reject': {
            'passed': preflight.get('passed'),
            'reject_reason': preflight.get('ingress_preflight_reject_reason'),
            'last_specific_reject_reason': preflight.get('last_specific_reject_reason'),
            'failed_gates': preflight.get('failed_gates'),
            'ingress_goal_sent': preflight.get('ingress_goal_sent'),
            'maze_explorer_started': preflight.get('maze_explorer_started'),
        },
        'stdout_stderr': {
            'preflight_stdout': bundle.get('preflight_stdout_text'),
            'preflight_stderr_nonempty': bool(bundle.get('preflight_stderr_text')),
            'runtime_recorder_stderr_nonempty': bool(bundle.get('runtime_recorder_stderr_text')),
        },
        'cleanup_artifacts': {
            'initial_cleanup': bundle.get('initial_cleanup'),
            'post_cleanup': bundle.get('post_cleanup'),
        },
        'direct_answers': {
            'raw_capture_available_after_preflight_timeout': raw_after_timeout,
            'preflight_used_different_frame_or_topic_than_raw_capture': frame_or_topic_diff,
            'preflight_scan_topic': topics.get('preflight_scan_topic'),
            'raw_scan_topic': topics.get('raw_scan_topic'),
            'preflight_scan_frame_id': frames.get('preflight_scan_frame_id'),
            'raw_scan_frame_id': frames.get('raw_scan_frame_id'),
            'bt_controller_inactive_is_startup_ordering_issue': startup_ordering,
            'bt_controller_inactive_is_tool_false_negative_candidate': bool(
                (not lifecycle.get('preflight_controller_server_active') or not lifecycle.get('preflight_bt_navigator_active'))
                and (lifecycle.get('managed_nodes_active_before_preflight_start') or lifecycle.get('ros_graph_ready_marker_before_preflight'))
            ),
        },
        'guardrails': {
            'diagnosis_only': True,
            'phase105_preflight_changed': False,
            'phase101_carry_over_changed': False,
            'phase88_92_logic_changed': False,
            'maze_explorer_strategy_changed': False,
            'nav2_config_changed': False,
            'nav2_controller_tuning_changed': False,
            'no_success_claimed': True,
            'no_exit_success_claimed': True,
            'phase108_entered': False,
        },
    }
    return result


def write_minimal_summary(result: JSONDict, path: Path) -> None:
    direct = _dict(result.get('direct_answers'))
    preflight = _dict(result.get('preflight_reject'))
    timing = _dict(result.get('timestamp_comparison'))
    lifecycle = _dict(result.get('lifecycle_comparison'))
    topics = _dict(result.get('topic_comparison'))
    frames = _dict(result.get('frame_comparison'))
    lines = [
        '# Phase107 minimal discrepancy summary',
        '',
        f"Classification: {result.get('classification')}",
        f"Contributing classifications: {result.get('contributing_classifications')}",
        f"Evidence gaps: {result.get('evidence_gaps')}",
        '',
        'Preflight reject:',
        f"- passed: {preflight.get('passed')}",
        f"- reject_reason: {preflight.get('reject_reason')}",
        f"- last_specific_reject_reason: {preflight.get('last_specific_reject_reason')}",
        f"- ingress_goal_sent: {preflight.get('ingress_goal_sent')}",
        f"- maze_explorer_started: {preflight.get('maze_explorer_started')}",
        '',
        'Timestamp comparison:',
        f"- preflight_start_wall_time_sec: {timing.get('preflight_start_wall_time_sec')}",
        f"- preflight_end_wall_time_sec: {timing.get('preflight_end_wall_time_sec')}",
        f"- raw_capture_start_wall_time_sec: {timing.get('raw_capture_start_wall_time_sec')}",
        f"- raw_capture_end_wall_time_sec: {timing.get('raw_capture_end_wall_time_sec')}",
        f"- raw_capture_started_after_preflight_timeout_deadline: {timing.get('raw_capture_started_after_preflight_timeout_deadline')}",
        '',
        'Frame/topic comparison:',
        f"- preflight_scan_topic: {topics.get('preflight_scan_topic')}",
        f"- raw_scan_topic: {topics.get('raw_scan_topic')}",
        f"- scan_topic_same: {topics.get('scan_topic_same')}",
        f"- preflight_scan_frame_id: {frames.get('preflight_scan_frame_id')}",
        f"- raw_scan_frame_id: {frames.get('raw_scan_frame_id')}",
        f"- scan_frame_same: {frames.get('scan_frame_same')}",
        '',
        'Lifecycle comparison:',
        f"- preflight_controller_server_active: {lifecycle.get('preflight_controller_server_active')}",
        f"- preflight_bt_navigator_active: {lifecycle.get('preflight_bt_navigator_active')}",
        f"- managed_nodes_active_time_sec: {lifecycle.get('managed_nodes_active_time_sec')}",
        f"- managed_nodes_active_before_preflight_start: {lifecycle.get('managed_nodes_active_before_preflight_start')}",
        f"- managed_nodes_active_after_preflight_end: {lifecycle.get('managed_nodes_active_after_preflight_end')}",
        '',
        'Direct answers:',
        f"- raw_capture_available_after_preflight_timeout: {direct.get('raw_capture_available_after_preflight_timeout')}",
        f"- preflight_used_different_frame_or_topic_than_raw_capture: {direct.get('preflight_used_different_frame_or_topic_than_raw_capture')}",
        f"- bt_controller_inactive_is_startup_ordering_issue: {direct.get('bt_controller_inactive_is_startup_ordering_issue')}",
        f"- bt_controller_inactive_is_tool_false_negative_candidate: {direct.get('bt_controller_inactive_is_tool_false_negative_candidate')}",
        '',
        'Guardrails: diagnosis-only; no preflight/tooling/algorithm/Nav2 tuning changed; no autonomous/exit success claim; Phase108 not entered.',
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines) + '\n')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase106-artifact-dir', '--artifact-dir', dest='artifact_dir', type=Path, required=True)
    parser.add_argument('--phase106-run-id', default=PHASE106_RUN_ID)
    parser.add_argument('--output-json', type=Path)
    parser.add_argument('--minimal-summary-output', type=Path)
    args = parser.parse_args()
    result = analyze_phase106_artifacts(args.artifact_dir, phase106_run_id=args.phase106_run_id)
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    if args.minimal_summary_output:
        write_minimal_summary(result, args.minimal_summary_output)
    print(json.dumps({'classification': result['classification'], 'contributing_classifications': result['contributing_classifications'], 'evidence_gaps': result['evidence_gaps']}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
