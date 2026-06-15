#!/usr/bin/env python3
"""Phase103 inner-ingress goal failure diagnosis analyzer.

Validation/diagnosis-only tool. It reads Phase102 ingress failure artifacts and
compares them with Phase96-fix / Phase97 ingress-success artifacts. It does not
call ROS, send goals, edit runtime logic, or tune Nav2 parameters.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

RUN_ID = 'phase103_inner_ingress_goal_failure_diagnosis'
PHASE102_RUN_ID = 'phase102_carry_over_bounded_goal1_staging_validation'

CLASSIFICATIONS = [
    'INNER_INGRESS_GOAL_POSE_BLOCKED',
    'INNER_INGRESS_START_POSE_OR_FRAME_MISMATCH',
    'INNER_INGRESS_NAV2_PLANNING_FAILED',
    'INNER_INGRESS_CONTROLLER_EXECUTION_FAILED',
    'INNER_INGRESS_REUSE_VISIBLE_STACK_STATE_DIRTY',
    'INNER_INGRESS_DIAGNOSIS_INSUFFICIENT_EVIDENCE',
]

REQUIRED_DIAGNOSTIC_FIELDS = [
    'ingress_goal_pose',
    'start_pose',
    'map_frame',
    'tf_available',
    'global_plan',
    'local_costmap',
    'nav2_lifecycle',
    'planner_controller_readiness',
    'costmap_obstruction',
    'reuse_visible_stack',
    'phase96_fix_comparison',
    'phase97_comparison',
]

TERMINAL_ABORT_STATUSES = {'6', 'aborted', 'status_aborted'}
SUCCESS_STATUSES = {'4', 'succeeded', 'status_succeeded'}


def _safe_json(path: Path, default: Any = None) -> Any:
    if default is None:
        default = {}
    try:
        return json.loads(path.read_text())
    except Exception:
        return default


def _jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists():
        return rows
    for line in path.read_text(errors='ignore').splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            raw = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(raw, dict):
            rows.append(raw)
    return rows


def _selected_log_text(path: Path | None, max_lines: int = 800) -> str:
    if not path or not path.exists():
        return ''
    critical_tokens = (
        'Timed out waiting for transform',
        'error_code',
        'TF_ERROR',
        'Goal failed',
        'Aborting',
        'Received a goal, begin computing control effort',
        'Unable to transform goal pose into costmap frame',
        'Robot pose is not available',
        'Failed to get "tugbot/scan_omni/scan_omni"->"base_link" frame transform',
        'Extrapolation Error looking up target frame',
    )
    noisy_tokens = (
        'Detected jump back in time',
        'Resetting RViz',
        'timestamp on the message is earlier',
        'queue is full',
        'Managed nodes are active',
    )
    critical: list[str] = []
    noisy: list[str] = []
    try:
        with path.open(errors='ignore') as fh:
            for line in fh:
                line = line.rstrip()
                if any(token in line for token in critical_tokens):
                    critical.append(line)
                elif any(token in line for token in noisy_tokens):
                    noisy.append(line)
                    if len(noisy) > max_lines:
                        noisy = noisy[-max_lines:]
    except Exception:
        return ''
    # Preserve rare goal/abort/error lines even when noisy TF-reset lines dominate the log.
    return '\n'.join(critical + noisy)


def _first_existing(artifact_dir: Path, patterns: list[str]) -> Path | None:
    for pattern in patterns:
        matches = sorted(artifact_dir.glob(pattern))
        if matches:
            return matches[0]
    return None


def _load_artifact_bundle(artifact_dir: Path, preferred_run_id: str | None = None) -> dict[str, Any]:
    artifact_dir = Path(artifact_dir)
    rid = preferred_run_id or artifact_dir.name
    ingress_path = _first_existing(artifact_dir, [f'{rid}_ingress_result.json', '*_ingress_result.json'])
    raw_path = _first_existing(artifact_dir, [f'{rid}_raw_capture.json', '*_raw_capture.json'])
    analysis_path = _first_existing(artifact_dir, [f'{rid}_analysis.json', '*_analysis.json'])
    feedback_path = _first_existing(artifact_dir, [f'{rid}_nav2_feedback.jsonl', '*_nav2_feedback.jsonl'])
    timeline_path = _first_existing(artifact_dir, [f'{rid}_runtime_timeline.jsonl', '*_runtime_timeline.jsonl'])
    local_path = _first_existing(artifact_dir, [f'{rid}_local_costmap_samples.jsonl', '*_local_costmap_samples.jsonl'])
    plan_path = _first_existing(artifact_dir, [f'{rid}_global_plan_samples.jsonl', '*_global_plan_samples.jsonl'])
    preflight_path = _first_existing(artifact_dir, [f'{rid}_preflight.txt', '*_preflight.txt'])
    ready_path = _first_existing(artifact_dir, [f'{rid}_ros_graph_ready.txt', '*_ros_graph_ready.txt'])
    launch_log_path = _first_existing(artifact_dir, [f'{rid}_visible_launch.log', '*_visible_launch.log'])
    return {
        'artifact_dir': str(artifact_dir),
        'run_id': rid,
        'ingress_path': str(ingress_path) if ingress_path else None,
        'raw_capture_path': str(raw_path) if raw_path else None,
        'analysis_path': str(analysis_path) if analysis_path else None,
        'feedback_path': str(feedback_path) if feedback_path else None,
        'runtime_timeline_path': str(timeline_path) if timeline_path else None,
        'local_costmap_samples_path': str(local_path) if local_path else None,
        'global_plan_samples_path': str(plan_path) if plan_path else None,
        'preflight_path': str(preflight_path) if preflight_path else None,
        'ros_graph_ready_path': str(ready_path) if ready_path else None,
        'launch_log_path': str(launch_log_path) if launch_log_path else None,
        'ingress': _safe_json(ingress_path, {}) if ingress_path else {},
        'raw_capture': _safe_json(raw_path, {}) if raw_path else {},
        'analysis': _safe_json(analysis_path, {}) if analysis_path else {},
        'feedback_rows': _jsonl(feedback_path) if feedback_path else [],
        'timeline_rows': _jsonl(timeline_path) if timeline_path else [],
        'local_costmap_rows': _jsonl(local_path) if local_path else [],
        'global_plan_rows': _jsonl(plan_path) if plan_path else [],
        'preflight_text': preflight_path.read_text(errors='ignore') if preflight_path and preflight_path.exists() else '',
        'ros_graph_ready_text': ready_path.read_text(errors='ignore') if ready_path and ready_path.exists() else '',
        'launch_log_text': _selected_log_text(launch_log_path),
    }


def _status_text(ingress: dict[str, Any]) -> str:
    return str(ingress.get('status') if ingress.get('status') is not None else ingress.get('status_text') or '').lower()


def _ingress_success(ingress: dict[str, Any]) -> bool:
    status = _status_text(ingress)
    return bool(ingress.get('success') is True or status in SUCCESS_STATUSES or status.endswith('succeeded'))


def _ingress_failed(ingress: dict[str, Any]) -> bool:
    if not ingress:
        return False
    status = _status_text(ingress)
    return bool(ingress.get('success') is False or status in TERMINAL_ABORT_STATUSES or ingress.get('error_code') not in (None, 0))


def _goal_pose(ingress: dict[str, Any]) -> dict[str, Any]:
    pose = ingress.get('goal_pose') if isinstance(ingress.get('goal_pose'), dict) else {}
    if not pose and isinstance(ingress.get('target'), list) and len(ingress['target']) >= 2:
        pose = {'frame_id': 'map', 'x_m': ingress['target'][0], 'y_m': ingress['target'][1], 'yaw_rad': ingress['target'][2] if len(ingress['target']) > 2 else None}
    return {
        'frame_id': pose.get('frame_id'),
        'x_m': pose.get('x_m'),
        'y_m': pose.get('y_m'),
        'yaw_rad': pose.get('yaw_rad'),
    }


def _robot_pose(raw: dict[str, Any]) -> Any:
    if 'robot_pose_map' in raw:
        return raw.get('robot_pose_map')
    if 'raw_robot_pose_map' in raw:
        return raw.get('raw_robot_pose_map')
    by_frame = raw.get('robot_pose_by_frame') if isinstance(raw.get('robot_pose_by_frame'), dict) else {}
    for key in ('map', 'map->base_link', 'base_link_in_map'):
        if key in by_frame:
            return by_frame[key]
    # Some artifacts store by frame pair.
    for value in by_frame.values():
        if isinstance(value, (list, dict)):
            return value
    odom = raw.get('odom') if isinstance(raw.get('odom'), dict) else {}
    return odom.get('pose') or odom.get('robot_pose')


def _available(raw: dict[str, Any], key: str) -> bool:
    direct = raw.get(f'{key}_available')
    if direct is not None:
        return bool(direct)
    value = raw.get(key)
    if isinstance(value, dict):
        if 'available' in value:
            return bool(value.get('available'))
        if key == 'tf':
            for candidate in ('map->base_link', 'map->odom', 'odom->base_link'):
                item = value.get(candidate)
                if isinstance(item, dict) and item.get('available'):
                    return True
            return bool(value.get('map_to_base_link') or value.get('map_base_link_available') or value.get('available'))
        return bool(value)
    return bool(value)


def _tf_diagnosis(raw: dict[str, Any], ready_text: str = '') -> dict[str, Any]:
    tf = raw.get('tf') if isinstance(raw.get('tf'), dict) else {}
    def ok(name: str) -> bool:
        item = tf.get(name)
        return bool(isinstance(item, dict) and item.get('available'))
    map_base = ok('map->base_link')
    map_odom = ok('map->odom')
    odom_base = ok('odom->base_link')
    ready_marker = 'TF ready' in ready_text or 'Translation:' in ready_text
    errors = {k: v.get('error') for k, v in tf.items() if isinstance(v, dict) and v.get('available') is False and v.get('error')}
    return {
        'available': bool(map_base or (map_odom and odom_base) or ready_marker),
        'map_to_base_link_available': map_base,
        'map_to_odom_available': map_odom,
        'odom_to_base_link_available': odom_base,
        'ready_marker_present': ready_marker,
        'frame_errors': errors,
    }


def _scan_frame_diagnosis(raw: dict[str, Any]) -> dict[str, Any]:
    scan = raw.get('scan') if isinstance(raw.get('scan'), dict) else {}
    frame = scan.get('frame_id')
    tf = raw.get('tf') if isinstance(raw.get('tf'), dict) else {}
    scan_tf_available = False
    scan_tf_errors: dict[str, Any] = {}
    for key, value in tf.items():
        if isinstance(value, dict) and ('scan' in key or 'base_scan' in key or 'scan_omni' in key):
            if value.get('available'):
                scan_tf_available = True
            elif value.get('error'):
                scan_tf_errors[key] = value.get('error')
    return {
        'frame_id': frame,
        'scan_available': bool(scan),
        'scan_to_base_tf_available': scan_tf_available,
        'scan_tf_errors': scan_tf_errors,
        'namespaced_scan_frame': bool(isinstance(frame, str) and '/' in frame),
    }


def _map_summary(raw: dict[str, Any]) -> dict[str, Any]:
    if isinstance(raw.get('map_summary'), dict):
        return raw['map_summary']
    m = raw.get('map') if isinstance(raw.get('map'), dict) else {}
    if isinstance(m.get('summary'), dict):
        return m['summary']
    return {k: m.get(k) for k in ('cell_count', 'known_count', 'free_count', 'occupied_count', 'unknown_count', 'lethal_count') if k in m}


def _local_summary(raw: dict[str, Any]) -> dict[str, Any]:
    if isinstance(raw.get('local_costmap_summary'), dict):
        return raw['local_costmap_summary']
    lc = raw.get('local_costmap') if isinstance(raw.get('local_costmap'), dict) else {}
    if isinstance(lc.get('summary'), dict):
        return lc['summary']
    return {k: lc.get(k) for k in ('cell_count', 'known_count', 'free_count', 'occupied_count', 'lethal_count', 'max') if k in lc}


def _nav2_lifecycle(bundle: dict[str, Any]) -> dict[str, Any]:
    rows = bundle.get('timeline_rows') or []
    ready_text = bundle.get('ros_graph_ready_text') or ''
    latest = rows[-1] if rows else {}
    ready = bool(
        latest.get('nav2_lifecycle_active')
        or (latest.get('planner_server_active') and latest.get('controller_server_active') and latest.get('bt_navigator_active'))
        or ('/bt_navigator' in ready_text and '/controller_server' in ready_text)
        or ('/navigate_to_pose ready' in ready_text and '/local_costmap/costmap ready' in ready_text)
    )
    return {
        'ready': ready,
        'runtime_timeline_sample_count': len(rows),
        'ros_graph_ready_available': bool(ready_text),
        'latest_runtime_sample': latest,
        'ready_text_markers': {
            'navigate_to_pose_ready': '/navigate_to_pose ready' in ready_text or '^/navigate_to_pose$' in ready_text,
            'map_ready': '/map ready' in ready_text or '^/map$' in ready_text,
            'scan_ready': '/scan ready' in ready_text or '^/scan$' in ready_text,
            'local_costmap_ready': '/local_costmap/costmap ready' in ready_text or '^/local_costmap/costmap$' in ready_text,
            'tf_ready': 'TF ready' in ready_text or 'Translation:' in ready_text,
        },
    }


def _planner_controller_readiness(bundle: dict[str, Any]) -> dict[str, Any]:
    text = bundle.get('ros_graph_ready_text') or ''
    latest = (bundle.get('timeline_rows') or [{}])[-1]
    return {
        'navigate_to_pose_action_ready': bool(latest.get('navigate_to_pose_action_ready') or '/navigate_to_pose ready' in text or bundle.get('ingress', {}).get('action_server_available')),
        'planner_server_active': bool(latest.get('planner_server_active') or '/planner_server' in text),
        'controller_server_active': bool(latest.get('controller_server_active') or '/controller_server' in text),
        'bt_navigator_active': bool(latest.get('bt_navigator_active') or '/bt_navigator' in text),
        'action_server_available': bool(bundle.get('ingress', {}).get('action_server_available')),
    }


def _local_costmap_diagnosis(bundle: dict[str, Any], raw: dict[str, Any]) -> dict[str, Any]:
    rows = bundle.get('local_costmap_rows') or []
    summary = _local_summary(raw)
    target_lethal_values: list[float] = []
    robot_lethal_values: list[float] = []
    front_values: list[float] = []
    target_in_bounds = []
    for row in rows:
        for key, dest in [
            ('target_footprint_lethal_count', target_lethal_values),
            ('target_footprint_lethal_count_max', target_lethal_values),
            ('robot_footprint_lethal_count', robot_lethal_values),
            ('robot_footprint_lethal_count_max', robot_lethal_values),
            ('front_wedge_lethal_count', front_values),
            ('front_wedge_lethal_count_max', front_values),
        ]:
            value = row.get(key)
            if isinstance(value, (int, float)):
                dest.append(float(value))
        if 'target_in_local_costmap_bounds' in row:
            target_in_bounds.append(bool(row.get('target_in_local_costmap_bounds')))
    target_max = max(target_lethal_values) if target_lethal_values else None
    robot_max = max(robot_lethal_values) if robot_lethal_values else None
    front_max = max(front_values) if front_values else None
    return {
        'available': _available(raw, 'local_costmap'),
        'sample_count': len(rows),
        'summary': summary,
        'target_footprint_lethal_count_max': target_max,
        'robot_footprint_lethal_count_max': robot_max,
        'front_wedge_lethal_count_max': front_max,
        'target_in_bounds_observed': any(target_in_bounds) if target_in_bounds else None,
    }


def _costmap_obstruction(local: dict[str, Any], raw: dict[str, Any]) -> dict[str, Any]:
    target_max = local.get('target_footprint_lethal_count_max')
    robot_max = local.get('robot_footprint_lethal_count_max')
    target_blocked = bool(isinstance(target_max, (int, float)) and target_max > 0)
    start_blocked = bool(isinstance(robot_max, (int, float)) and robot_max > 0)
    return {
        'target_pose_blocked': target_blocked,
        'start_pose_blocked': start_blocked,
        'sample_backed': local.get('sample_count', 0) > 0,
        'local_costmap_available': local.get('available'),
        'raw_local_costmap_summary': local.get('summary'),
    }


def _log_diagnosis(bundle: dict[str, Any]) -> dict[str, Any]:
    text = bundle.get('launch_log_text') or ''
    markers = {
        'tf_jump_back_count': text.count('Detected jump back in time'),
        'rviz_reset_count': text.count('Resetting RViz'),
        'global_costmap_transform_timeout': 'Timed out waiting for transform from base_link to map' in text,
        'planner_transform_extrapolation': '[planner_server' in text and 'Extrapolation Error' in text,
        'controller_goal_received': 'Received a goal, begin computing control effort' in text,
        'controller_unable_to_transform_goal_pose': 'Unable to transform goal pose into costmap frame' in text,
        'follow_path_abort': '[follow_path] [ActionServer] Aborting handle' in text,
        'bt_robot_pose_unavailable': 'Robot pose is not available' in text,
        'bt_goal_failed': 'Goal failed' in text,
        'scan_transform_cache_drop': 'timestamp on the message is earlier than all the data in the transform cache' in text,
        'scan_frame_transform_error': 'Failed to get "tugbot/scan_omni/scan_omni"->"base_link" frame transform' in text,
        'slam_queue_full_drop': 'queue is full' in text,
        'managed_nodes_active': 'Managed nodes are active' in text,
    }
    lines = []
    for line in text.splitlines():
        if any(token in line for token in ('Timed out waiting for transform', 'Detected jump back in time', 'Resetting RViz', 'timestamp on the message is earlier', 'Managed nodes are active')):
            lines.append(line)
        if len(lines) >= 20:
            break
    return {'available': bool(text), **markers, 'selected_lines': lines}


def _action_error_diagnosis(ingress: dict[str, Any], log_diag: dict[str, Any], scan_diag: dict[str, Any]) -> dict[str, Any]:
    code = ingress.get('error_code')
    meaning = None
    likely_source = None
    if code == 102:
        # NavigateToPose only defines NONE=0; in practice BT sub-action error codes can surface here.
        # 102 matches FollowPath.TF_ERROR, and Phase102 logs also show repeated TF buffer resets.
        meaning = 'TF_ERROR-compatible error code surfaced through NavigateToPose result'
        likely_source = 'controller_or_bt_subaction_tf_error'
    return {
        'error_code': code,
        'error_msg': ingress.get('error_msg'),
        'interpreted_meaning': meaning,
        'likely_source': likely_source,
        'tf_jump_back_count': log_diag.get('tf_jump_back_count'),
        'scan_tf_missing': bool(scan_diag.get('scan_tf_errors')),
    }


def _global_plan(bundle: dict[str, Any]) -> dict[str, Any]:
    rows = bundle.get('global_plan_rows') or []
    lengths = []
    for row in rows:
        for key in ('path_length_m', 'global_plan_length_m', 'plan_length_m'):
            if isinstance(row.get(key), (int, float)):
                lengths.append(float(row[key]))
    return {
        'sample_count': len(rows),
        'available': bool(rows),
        'length_m_last': lengths[-1] if lengths else None,
        'length_m_max': max(lengths) if lengths else None,
    }


def _feedback_diagnosis(bundle: dict[str, Any]) -> dict[str, Any]:
    ingress_feedback = bundle.get('ingress', {}).get('feedback') if isinstance(bundle.get('ingress', {}).get('feedback'), list) else []
    rows = bundle.get('feedback_rows') or []
    all_rows = rows or [r for r in ingress_feedback if isinstance(r, dict)]
    distances = [float(r.get('distance_remaining')) for r in all_rows if isinstance(r.get('distance_remaining'), (int, float))]
    recoveries = [int(r.get('number_of_recoveries')) for r in all_rows if isinstance(r.get('number_of_recoveries'), int)]
    nav_times = [float(r.get('navigation_time_sec')) for r in all_rows if isinstance(r.get('navigation_time_sec'), (int, float))]
    return {
        'sample_count': len(all_rows),
        'distance_remaining_first_nonzero': next((d for d in distances if d > 0.001), None),
        'distance_remaining_last': distances[-1] if distances else None,
        'distance_remaining_min': min(distances) if distances else None,
        'distance_remaining_max': max(distances) if distances else None,
        'recoveries_max': max(recoveries) if recoveries else 0,
        'navigation_time_last_sec': nav_times[-1] if nav_times else None,
        'immediate_abort_like': bool(nav_times and nav_times[-1] < 2.0 and (max(recoveries) if recoveries else 0) == 0),
    }


def _reuse_visible_stack(bundle: dict[str, Any]) -> dict[str, Any]:
    text = bundle.get('preflight_text') or ''
    artifact_dir = Path(bundle.get('artifact_dir', ''))
    attempt_dir = artifact_dir / 'attempt1_ingress_failed_before_rerun'
    reuse = 'reuse_visible_stack=1' in text or 'reuse_visible_stack=true' in text.lower()
    attempt_failed = False
    attempt_reuse: bool | None = None
    attempt_reuse_line = None
    if attempt_dir.exists():
        attempt_bundle = _load_artifact_bundle(attempt_dir, PHASE102_RUN_ID)
        attempt_failed = _ingress_failed(attempt_bundle.get('ingress') or {})
        attempt_text = attempt_bundle.get('preflight_text') or ''
        attempt_reuse_line = next((line for line in attempt_text.splitlines() if 'reuse_visible_stack' in line), None)
        if 'reuse_visible_stack=0' in attempt_text or 'reuse_visible_stack=false' in attempt_text.lower():
            attempt_reuse = False
        elif 'reuse_visible_stack=1' in attempt_text or 'reuse_visible_stack=true' in attempt_text.lower():
            attempt_reuse = True
    clean_attempt_failed = bool(attempt_failed and attempt_reuse is False)
    return {
        'reuse_visible_stack': reuse,
        'attempt1_failure_artifact_present': attempt_dir.exists(),
        'attempt1_ingress_failed': attempt_failed,
        'attempt1_reuse_visible_stack': attempt_reuse,
        'attempt1_preflight_reuse_line': attempt_reuse_line,
        'clean_attempt_also_failed': clean_attempt_failed,
        'dirty_reuse_detected': bool(reuse and attempt_failed and not clean_attempt_failed),
        'dirty_reuse_present_but_not_primary': bool(reuse and clean_attempt_failed),
        'preflight_reuse_line': next((line for line in text.splitlines() if 'reuse_visible_stack' in line), None),
    }


def _comparison(bundle: dict[str, Any], label: str) -> dict[str, Any]:
    ingress = bundle.get('ingress') or {}
    raw = bundle.get('raw_capture') or {}
    return {
        'label': label,
        'artifact_dir': bundle.get('artifact_dir'),
        'ingress_success': _ingress_success(ingress),
        'ingress_status': ingress.get('status'),
        'ingress_status_text': ingress.get('status_text'),
        'error_code': ingress.get('error_code'),
        'goal_pose': _goal_pose(ingress),
        'feedback_sample_count': len(ingress.get('feedback') or []),
        'raw_capture_available': bool(raw),
        'scan_available': _available(raw, 'scan'),
        'map_available': _available(raw, 'map'),
        'local_costmap_available': _available(raw, 'local_costmap'),
        'odom_available': _available(raw, 'odom'),
        'tf_available': _available(raw, 'tf'),
        'robot_pose': _robot_pose(raw),
    }


def _same_goal_pose(a: dict[str, Any], b: dict[str, Any]) -> bool:
    try:
        return (
            a.get('frame_id') == b.get('frame_id')
            and math.isclose(float(a.get('x_m')), float(b.get('x_m')), abs_tol=1e-6)
            and math.isclose(float(a.get('y_m')), float(b.get('y_m')), abs_tol=1e-6)
        )
    except Exception:
        return False


def _classify(diag: dict[str, Any], p96: dict[str, Any], p97: dict[str, Any]) -> str:
    ingress = diag['phase102']['ingress']
    if not ingress or not diag['phase102'].get('raw_capture'):
        return 'INNER_INGRESS_DIAGNOSIS_INSUFFICIENT_EVIDENCE'
    if diag['map_frame']['goal_frame_id'] != 'map' or not diag['tf_available'] or not diag['start_pose'].get('available'):
        return 'INNER_INGRESS_START_POSE_OR_FRAME_MISMATCH'
    if diag['costmap_obstruction']['target_pose_blocked']:
        return 'INNER_INGRESS_GOAL_POSE_BLOCKED'
    if diag['reuse_visible_stack']['dirty_reuse_detected'] and p96.get('ingress_success') and p97.get('ingress_success'):
        return 'INNER_INGRESS_REUSE_VISIBLE_STACK_STATE_DIRTY'
    log_diag = diag.get('log_diagnosis') or {}
    # Controller-side TF/pose failures are more specific than missing recorder plan samples:
    # Phase102 logs show controller_server accepted FollowPath then could not transform the goal pose.
    if _ingress_failed(ingress) and (
        log_diag.get('controller_goal_received')
        or log_diag.get('controller_unable_to_transform_goal_pose')
        or log_diag.get('follow_path_abort')
        or (diag.get('action_error_diagnosis') or {}).get('likely_source') == 'controller_or_bt_subaction_tf_error'
    ):
        return 'INNER_INGRESS_CONTROLLER_EXECUTION_FAILED'
    if _ingress_failed(ingress) and not diag['global_plan']['available'] and diag['nav2_feedback']['immediate_abort_like']:
        return 'INNER_INGRESS_NAV2_PLANNING_FAILED'
    if _ingress_failed(ingress) and diag['global_plan']['available']:
        return 'INNER_INGRESS_CONTROLLER_EXECUTION_FAILED'
    return 'INNER_INGRESS_DIAGNOSIS_INSUFFICIENT_EVIDENCE'


def analyze_artifacts(phase102_artifact_dir: Path | str, phase96_fix_artifact_dir: Path | str, phase97_artifact_dir: Path | str) -> dict[str, Any]:
    phase102 = _load_artifact_bundle(Path(phase102_artifact_dir), PHASE102_RUN_ID)
    phase96 = _load_artifact_bundle(Path(phase96_fix_artifact_dir))
    phase97 = _load_artifact_bundle(Path(phase97_artifact_dir))
    ingress = phase102['ingress']
    raw = phase102['raw_capture']
    goal_pose = _goal_pose(ingress)
    robot_pose = _robot_pose(raw)
    local = _local_costmap_diagnosis(phase102, raw)
    nav2 = _nav2_lifecycle(phase102)
    planner_controller = _planner_controller_readiness(phase102)
    reuse = _reuse_visible_stack(phase102)
    log_diag = _log_diagnosis(phase102)
    scan_diag = {'available': _available(raw, 'scan'), 'scan_tf_errors': bool(log_diag.get('scan_frame_transform_error') or log_diag.get('scan_transform_cache_drop'))}
    action_error = _action_error_diagnosis(ingress, log_diag, scan_diag)
    p96_cmp = _comparison(phase96, 'Phase96-fix')
    p97_cmp = _comparison(phase97, 'Phase97')
    diag = {
        'ingress_goal_pose': goal_pose,
        'start_pose': {'available': robot_pose is not None, 'robot_pose_map': robot_pose},
        'map_frame': {'goal_frame_id': goal_pose.get('frame_id'), 'expected_frame_id': 'map', 'frame_ok': goal_pose.get('frame_id') == 'map'},
        'tf_available': _available(raw, 'tf'),
        'global_plan': _global_plan(phase102),
        'local_costmap': local,
        'nav2_lifecycle': nav2,
        'planner_controller_readiness': planner_controller,
        'costmap_obstruction': _costmap_obstruction(local, raw),
        'reuse_visible_stack': reuse,
        'log_diagnosis': log_diag,
        'action_error_diagnosis': action_error,
        'phase96_fix_comparison': p96_cmp,
        'phase97_comparison': p97_cmp,
        'nav2_feedback': _feedback_diagnosis(phase102),
        'map': {'available': _available(raw, 'map'), 'summary': _map_summary(raw)},
        'scan': scan_diag,
        'odom': {'available': _available(raw, 'odom')},
    }
    phase102_summary = {
        'artifact_dir': phase102.get('artifact_dir'),
        'ingress': {
            'goal_sent': ingress.get('goal_sent'),
            'goal_accepted': ingress.get('goal_accepted'),
            'result_received': ingress.get('result_received'),
            'success': ingress.get('success'),
            'status': ingress.get('status'),
            'status_text': ingress.get('status_text'),
            'error_code': ingress.get('error_code'),
            'feedback_count': len(ingress.get('feedback') or []),
            'failed': _ingress_failed(ingress),
        },
        'raw_capture': {
            'available': bool(raw),
            'scan_available': _available(raw, 'scan'),
            'map_available': _available(raw, 'map'),
            'local_costmap_available': _available(raw, 'local_costmap'),
            'odom_available': _available(raw, 'odom'),
            'tf_available': _available(raw, 'tf'),
        },
        'artifact_paths': {k: phase102.get(k) for k in ('ingress_path','raw_capture_path','feedback_path','runtime_timeline_path','local_costmap_samples_path','global_plan_samples_path','preflight_path','ros_graph_ready_path')},
    }
    tmp = {'phase102': {'ingress': ingress, 'raw_capture': raw}, **diag}
    classification = _classify(tmp, p96_cmp, p97_cmp)
    evidence: list[str] = []
    if _same_goal_pose(goal_pose, p96_cmp.get('goal_pose') or {}) and _same_goal_pose(goal_pose, p97_cmp.get('goal_pose') or {}):
        evidence.append('same_inner_ingress_goal_pose_as_successful_phase96_fix_and_phase97')
    if p96_cmp.get('ingress_success') and p97_cmp.get('ingress_success'):
        evidence.append('phase96_fix_and_phase97_ingress_success_reference_available')
    if reuse.get('dirty_reuse_detected'):
        evidence.append('phase102_reused_visible_stack_after_failed_attempt')
    if nav2.get('ready') and planner_controller.get('navigate_to_pose_action_ready'):
        evidence.append('nav2_action_and_readiness_markers_present')
    if diag['nav2_feedback'].get('immediate_abort_like'):
        evidence.append('phase102_immediate_abort_like_feedback_no_recoveries')
    if not diag['global_plan'].get('available'):
        evidence.append('no_global_plan_samples_available_for_phase102_ingress')
    if log_diag.get('controller_unable_to_transform_goal_pose'):
        evidence.append('controller_aborted_after_unable_to_transform_goal_pose_into_costmap_frame')
    if log_diag.get('bt_robot_pose_unavailable'):
        evidence.append('bt_navigator_reported_robot_pose_unavailable')
    if log_diag.get('tf_jump_back_count'):
        evidence.append('tf_buffer_jump_back_resets_observed_during_ingress_failure')
    return {
        'run_id': RUN_ID,
        'classification': classification,
        'allowed_classifications': CLASSIFICATIONS,
        'required_diagnostic_fields': REQUIRED_DIAGNOSTIC_FIELDS,
        'phase102': phase102_summary,
        'diagnosis': diag,
        'comparison': {'phase96_fix': p96_cmp, 'phase97': p97_cmp},
        'evidence': evidence,
        'guardrails': {
            'phase101_carry_over_changed': False,
            'phase88_92_logic_changed': False,
            'maze_explorer_strategy_changed': False,
            'nav2_params_changed': False,
            'inflation_robot_radius_clearance_map_threshold_tuned': False,
            'phase104_entered': False,
        },
    }


def write_summary(result: dict[str, Any], path: Path) -> None:
    diag = result.get('diagnosis', {})
    phase102 = result.get('phase102', {})
    ingress = phase102.get('ingress', {})
    lines = [
        '# Phase103 minimal summary',
        '',
        f"Classification: {result.get('classification')}",
        f"Phase102 ingress status: {ingress.get('status')} / success={ingress.get('success')} / error_code={ingress.get('error_code')}",
        f"Ingress goal pose: {diag.get('ingress_goal_pose')}",
        f"Start pose: {diag.get('start_pose')}",
        f"TF available: {diag.get('tf_available')}",
        f"Nav2 lifecycle ready: {(diag.get('nav2_lifecycle') or {}).get('ready')}",
        f"Planner/controller readiness: {diag.get('planner_controller_readiness')}",
        f"Global plan: {diag.get('global_plan')}",
        f"Local costmap: {diag.get('local_costmap')}",
        f"Costmap obstruction: {diag.get('costmap_obstruction')}",
        f"Reuse visible stack: {diag.get('reuse_visible_stack')}",
        '',
        f"Phase96-fix comparison: {result.get('comparison', {}).get('phase96_fix')}",
        f"Phase97 comparison: {result.get('comparison', {}).get('phase97')}",
        '',
        f"Evidence: {result.get('evidence')}",
        '',
        'No Phase101 carry-over or Nav2 parameter changes were made.',
        'No inflation/robot_radius/clearance_radius_m/map threshold tuning was made.',
        'Phase104 not entered.',
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines) + '\n')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Phase103 inner-ingress goal failure diagnosis analyzer')
    parser.add_argument('--phase102-artifact-dir', default='log/phase102_carry_over_bounded_goal1_staging_validation')
    parser.add_argument('--phase96-fix-artifact-dir', default='log/phase96_fix_ingress_guided_startup_correction')
    parser.add_argument('--phase97-artifact-dir', default='log/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke')
    parser.add_argument('--output-json', default='log/phase103_inner_ingress_goal_failure_diagnosis/phase103_inner_ingress_goal_failure_diagnosis_analysis.json')
    parser.add_argument('--minimal-summary-output', default='log/phase103_inner_ingress_goal_failure_diagnosis/phase103_inner_ingress_goal_failure_diagnosis_minimal_summary.md')
    args = parser.parse_args(argv)
    result = analyze_artifacts(Path(args.phase102_artifact_dir), Path(args.phase96_fix_artifact_dir), Path(args.phase97_artifact_dir))
    out = Path(args.output_json)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    write_summary(result, Path(args.minimal_summary_output))
    print(json.dumps({'classification': result['classification'], 'output_json': str(out), 'minimal_summary': args.minimal_summary_output}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
