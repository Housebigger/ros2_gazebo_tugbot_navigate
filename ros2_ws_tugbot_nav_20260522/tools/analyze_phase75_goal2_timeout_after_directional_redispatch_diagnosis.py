#!/usr/bin/env python3
"""Phase75 Goal2 Timeout Diagnosis After Directional Re-dispatch.

Scope: offline diagnosis only.  This analyzer reads Phase74 replay_02 artifacts
and reconstructs Goal2 from dispatch to timeout.  It does not tune Nav2/MPPI/
controller parameters, does not tune inflation / robot radius / clearance radius /
map threshold parameters, does not change branch scoring, does not change the
centerline gate, does not change the directional readiness override, does not
change fallback or terminal acceptance, does not claim autonomous exploration
success, and does not claim exit success.
"""
from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from typing import Any

PHASE = 'Phase75 Goal2 Timeout Diagnosis After Directional Re-dispatch'
RUN_ID = 'phase75_goal2_timeout_after_directional_redispatch_diagnosis'
PHASE74_RUN_ID = 'phase74_directional_local_costmap_readiness_gate_validation'
PHASE74_EXPECTED_CLASSIFICATION = 'DIRECTIONAL_GATE_ENABLES_GOAL2_DISPATCH'
REPLAY_ID = 'replay_02'
GOAL_SEQUENCE = 2
XY_GOAL_TOLERANCE_M = 0.25
YAW_GOAL_TOLERANCE_RAD = 0.25
HIGH_COST_THRESHOLD = 70
LETHAL_COST_THRESHOLD = 99
NEAR_ZERO_LINEAR_MPS = 0.03
NEAR_ZERO_ANGULAR_RADPS = 0.05

ALLOWED_CLASSIFICATIONS = [
    'GOAL2_TIMEOUT_TARGET_TOO_CLOSE_TO_WALL',
    'GOAL2_TIMEOUT_NEAR_GOAL_TOLERANCE_ORIENTATION',
    'GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP',
    'GOAL2_TIMEOUT_PROGRESS_INSUFFICIENT',
    'INSUFFICIENT_EVIDENCE',
]

GUARDRAILS = [
    'offline diagnosis only',
    'reuse Phase74 replay_02 artifacts',
    'no Nav2/MPPI/controller tuning',
    'no inflation/robot_radius/clearance_radius_m/map threshold tuning',
    'no branch scoring change',
    'no centerline gate change',
    'no directional readiness override change',
    'no fallback/terminal acceptance change',
    'does not claim autonomous exploration success',
    'does not claim exit success',
    'Phase76 not entered',
]


def _safe_json_load(path: Path | None, default: Any) -> Any:
    if not path or not path.exists() or not path.stat().st_size:
        return default
    try:
        return json.loads(path.read_text(encoding='utf-8', errors='replace'))
    except Exception:
        return default


def _safe_json_loads(line: str) -> Any | None:
    try:
        return json.loads(line)
    except Exception:
        return None


def _read_text(path: Path | None) -> str:
    if not path or not path.exists():
        return ''
    return path.read_text(encoding='utf-8', errors='replace')


def _read_jsonl(path: Path | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path or not path.exists() or not path.stat().st_size:
        return rows
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        row = _safe_json_loads(line)
        if not isinstance(row, dict):
            continue
        payload = row.get('state') if isinstance(row.get('state'), dict) else row
        if isinstance(payload, dict):
            payload = dict(payload)
            payload.setdefault('_recorder_elapsed_sec', row.get('elapsed_sec'))
            payload.setdefault('_recorder_wall_time', row.get('wall_time'))
            payload.setdefault('_recorder_seq', row.get('seq'))
            rows.append(payload)
    return rows


def _number(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _angle_wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _stats(values: list[float]) -> dict[str, Any]:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    if not clean:
        return {'count': 0, 'min': None, 'max': None, 'mean': None, 'first': None, 'final': None}
    return {
        'count': len(clean),
        'min': min(clean),
        'max': max(clean),
        'mean': sum(clean) / len(clean),
        'first': clean[0],
        'final': clean[-1],
    }


def _artifact_file(artifact_dir: Path, suffix: str) -> Path | None:
    candidates = [artifact_dir / suffix]
    candidates.extend(sorted(artifact_dir.glob(f'*{suffix}')))
    for path in candidates:
        if path.exists():
            return path
    return None


def _goal_events_path(artifact_dir: Path) -> Path | None:
    return _artifact_file(artifact_dir, f'{PHASE74_RUN_ID}_{REPLAY_ID}_goal_events.jsonl')


def _explorer_state_path(artifact_dir: Path) -> Path | None:
    return _artifact_file(artifact_dir, f'{PHASE74_RUN_ID}_{REPLAY_ID}_explorer_state.jsonl')


def _find_goal_event(events: list[dict[str, Any]], event: str, seq: int = GOAL_SEQUENCE) -> dict[str, Any] | None:
    return next((row for row in events if int(row.get('goal_sequence') or -1) == seq and row.get('event') == event), None)


def _goal_outcome(events: list[dict[str, Any]], seq: int = GOAL_SEQUENCE) -> dict[str, Any] | None:
    return next((row for row in events if int(row.get('goal_sequence') or -1) == seq and row.get('event') in {'success', 'failure', 'timeout'}), None)


def _wall_time(row: dict[str, Any] | None) -> float | None:
    if not row:
        return None
    return _number(row.get('_recorder_wall_time') or row.get('wall_time'))


def _elapsed(row: dict[str, Any] | None) -> float | None:
    if not row:
        return None
    return _number(row.get('_recorder_elapsed_sec') or row.get('elapsed_sec'))


def _rows_for_goal(rows: list[dict[str, Any]], seq: int = GOAL_SEQUENCE) -> list[dict[str, Any]]:
    return [row for row in rows if int(row.get('goal_sequence') or -999) == seq]


def _rows_in_window(rows: list[dict[str, Any]], start_wall: float | None, end_wall: float | None, margin_sec: float = 1.0) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for row in rows:
        wall = _number(row.get('_recorder_wall_time') or row.get('wall_time'))
        if wall is None:
            out.append(row)
            continue
        if start_wall is not None and wall < start_wall - margin_sec:
            continue
        if end_wall is not None and wall > end_wall + margin_sec:
            continue
        out.append(row)
    return out


def _phase74_classification(path: Path | None) -> str | None:
    data = _safe_json_load(path, {}) if path else {}
    if not isinstance(data, dict):
        return None
    for key in ('classification', 'final_classification', 'phase74_classification'):
        value = data.get(key)
        if isinstance(value, str):
            return value
    summary = data.get('summary')
    if isinstance(summary, dict):
        for key in ('classification', 'final_classification', 'phase74_classification'):
            value = summary.get(key)
            if isinstance(value, str):
                return value
    return None


def _target_field(event: dict[str, Any] | None, name: str) -> list[float] | None:
    value = (event or {}).get(name)
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        x = _number(value[0])
        y = _number(value[1])
        if x is not None and y is not None:
            return [x, y]
    return None


def _pose_field(event: dict[str, Any] | None, name: str) -> list[float] | None:
    value = (event or {}).get(name)
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        x = _number(value[0])
        y = _number(value[1])
        yaw = _number(value[2]) if len(value) >= 3 else None
        if x is not None and y is not None:
            return [x, y] if yaw is None else [x, y, yaw]
    return None


def _trajectory_summary(controller_rows: list[dict[str, Any]], target: list[float] | None) -> dict[str, Any]:
    odom = [r for r in controller_rows if r.get('source') == 'odom' and _number(r.get('x')) is not None and _number(r.get('y')) is not None]
    poses: list[list[float]] = []
    distances: list[float] = []
    for row in odom:
        x = float(row.get('x'))
        y = float(row.get('y'))
        yaw = _number(row.get('yaw')) or 0.0
        poses.append([x, y, yaw])
        if target is not None:
            distances.append(math.hypot(x - target[0], y - target[1]))
    path_length = 0.0
    for a, b in zip(poses, poses[1:]):
        path_length += math.hypot(b[0] - a[0], b[1] - a[1])
    final_pose = poses[-1] if poses else None
    straight = math.hypot(poses[-1][0] - poses[0][0], poses[-1][1] - poses[0][1]) if len(poses) >= 2 else None
    last_10 = []
    if odom:
        end_wall = max([_number(r.get('wall_time')) for r in odom if _number(r.get('wall_time')) is not None] or [None])
        if end_wall is not None:
            last_10 = [r for r in odom if (_number(r.get('wall_time')) or -1e9) >= end_wall - 10.0]
    last_10_motion = 0.0
    last_10_poses = [[float(r.get('x')), float(r.get('y')), _number(r.get('yaw')) or 0.0] for r in last_10]
    for a, b in zip(last_10_poses, last_10_poses[1:]):
        last_10_motion += math.hypot(b[0] - a[0], b[1] - a[1])
    return {
        'pose_sample_count': len(odom),
        'first_pose': poses[0] if poses else None,
        'final_pose': final_pose,
        'path_length_m': path_length,
        'straight_line_displacement_m': straight,
        'distance_to_target': _stats(distances),
        'distance_to_target_improvement_m': (distances[0] - distances[-1]) if distances else None,
        'last_10s_motion_m': last_10_motion,
        'last_10s_stalled': bool(len(last_10_poses) > 2 and last_10_motion < 0.05),
    }


def _cmd_vel_summary(controller_rows: list[dict[str, Any]]) -> dict[str, Any]:
    cmd = [r for r in controller_rows if r.get('source') in {'cmd_vel', 'cmd_vel_smoothed', 'cmd_vel_nav'} or 'linear_x' in r]
    by_source: dict[str, int] = {}
    for row in cmd:
        source = str(row.get('source', 'cmd_vel'))
        by_source[source] = by_source.get(source, 0) + 1
    linear_abs = [abs(_number(r.get('linear_x')) or 0.0) for r in cmd]
    angular_abs = [abs(_number(r.get('angular_z')) or 0.0) for r in cmd]
    moving_flags = [l > NEAR_ZERO_LINEAR_MPS or a > NEAR_ZERO_ANGULAR_RADPS for l, a in zip(linear_abs, angular_abs)]
    last10 = []
    if cmd:
        end_wall = max([_number(r.get('wall_time')) for r in cmd if _number(r.get('wall_time')) is not None] or [None])
        if end_wall is not None:
            last10 = [r for r in cmd if (_number(r.get('wall_time')) or -1e9) >= end_wall - 10.0]
    last10_flags = [abs(_number(r.get('linear_x')) or 0.0) > NEAR_ZERO_LINEAR_MPS or abs(_number(r.get('angular_z')) or 0.0) > NEAR_ZERO_ANGULAR_RADPS for r in last10]
    return {
        'cmd_vel_sample_count': len(cmd),
        'samples_by_source': by_source,
        'linear_x_abs': _stats(linear_abs),
        'angular_z_abs': _stats(angular_abs),
        'nonzero_command_count': sum(1 for flag in moving_flags if flag),
        'nonzero_command_ratio': (sum(1 for flag in moving_flags if flag) / len(moving_flags)) if moving_flags else None,
        'near_zero_command_ratio': (sum(1 for flag in moving_flags if not flag) / len(moving_flags)) if moving_flags else None,
        'last_10s_sample_count': len(last10),
        'last_10s_nonzero_command_ratio': (sum(1 for flag in last10_flags if flag) / len(last10_flags)) if last10_flags else None,
        'last_10s_all_near_zero': bool(last10_flags and not any(last10_flags)),
        'cmd_vel_available': bool(cmd),
    }


def _distance_remaining_summary(feedback_rows: list[dict[str, Any]]) -> dict[str, Any]:
    distances = [_number(r.get('distance_remaining')) for r in feedback_rows if r.get('event') == 'nav2_feedback' or 'distance_remaining' in r]
    distances_clean = [v for v in distances if v is not None]
    recoveries = [int(r.get('number_of_recoveries') or 0) for r in feedback_rows if r.get('event') == 'nav2_feedback' or 'number_of_recoveries' in r]
    # ROS action feedback may emit an initial zero before the path is available; keep raw stats but add nonzero improvement.
    nonzero = [v for v in distances_clean if v > 1e-6]
    return {
        'feedback_sample_count': len([r for r in feedback_rows if r.get('event') == 'nav2_feedback' or 'distance_remaining' in r]),
        'distance_remaining': _stats(distances_clean),
        'distance_remaining_nonzero': _stats(nonzero),
        'distance_remaining_improvement_m': (distances_clean[0] - distances_clean[-1]) if distances_clean else None,
        'distance_remaining_nonzero_improvement_m': (nonzero[0] - nonzero[-1]) if nonzero else None,
        'number_of_recoveries_max': max(recoveries, default=0),
        'number_of_recoveries_values': sorted(set(recoveries)),
    }


def _nav2_recovery_summary(feedback_rows: list[dict[str, Any]], launch_text: str) -> dict[str, Any]:
    dist = _distance_remaining_summary(feedback_rows)
    return {
        'number_of_recoveries_max': dist['number_of_recoveries_max'],
        'number_of_recoveries_values': dist['number_of_recoveries_values'],
        'failed_to_make_progress_count': launch_text.count('Failed to make progress'),
        'follow_path_abort_count': len(re.findall(r'Aborting handle', launch_text, flags=re.IGNORECASE)),
        'costmap_clear_count': len(re.findall(r'Received request to clear entirely.*costmap', launch_text, flags=re.IGNORECASE)),
        'goal_canceled_count': len(re.findall(r'Goal canceled|Goal was canceled|Client requested to cancel', launch_text, flags=re.IGNORECASE)),
    }


def _cost_summary(samples: list[dict[str, Any]], key: str) -> dict[str, Any]:
    dicts = [row.get(key) for row in samples if isinstance(row.get(key), dict)]
    maxes = [_number(d.get('max')) for d in dicts]
    lethal = [_number(d.get('lethal_count')) for d in dicts]
    high = [_number(d.get('high_cost_count')) for d in dicts]
    means = [_number(d.get('mean')) for d in dicts]
    return {
        'sample_count': len(dicts),
        'max_cost': _stats([v for v in maxes if v is not None]),
        'mean_cost': _stats([v for v in means if v is not None]),
        'max_lethal_count': max([int(v) for v in lethal if v is not None], default=0),
        'max_high_cost_count': max([int(v) for v in high if v is not None], default=0),
        'latest': dicts[-1] if dicts else None,
    }


def _local_cost_summary(samples: list[dict[str, Any]]) -> dict[str, Any]:
    target_rows = [row.get('local_costmap_target_evidence') for row in samples if isinstance(row.get('local_costmap_target_evidence'), dict)]
    target_values = [_number(row.get('value')) for row in target_rows]
    target_radius_maxes = [_number(((row.get('radius_cost_summary') or {}).get('max'))) for row in target_rows]
    return {
        'sample_count': len(samples),
        'target_evidence_sample_count': len(target_rows),
        'target_cell_value': _stats([v for v in target_values if v is not None]),
        'target_radius_cost_max': _stats([v for v in target_radius_maxes if v is not None]),
        'latest_target_evidence': target_rows[-1] if target_rows else None,
    }


def _front_wedge_summary(samples: list[dict[str, Any]], outcome: dict[str, Any] | None) -> dict[str, Any]:
    base = _cost_summary(samples, 'front_wedge_cost')
    clearances = [_number(row.get('front_wedge_clearance_m')) for row in samples]
    out_clearance = _number((outcome or {}).get('timeout_front_wedge_clearance_m'))
    out_cost = _number((outcome or {}).get('timeout_front_wedge_cost_max'))
    base.update({
        'clearance_m': _stats([v for v in clearances if v is not None]),
        'outcome_timeout_front_wedge_clearance_m': out_clearance,
        'outcome_timeout_front_wedge_cost_max': out_cost,
        'front_wedge_blocked': bool((out_clearance is not None and out_clearance < 0.15) or (out_cost is not None and out_cost >= LETHAL_COST_THRESHOLD) or base['max_lethal_count'] > 0),
    })
    return base


def _footprint_summary(samples: list[dict[str, Any]], outcome: dict[str, Any] | None) -> dict[str, Any]:
    base = _cost_summary(samples, 'robot_footprint_cost')
    out_max = _number((outcome or {}).get('timeout_footprint_cost_max'))
    out_lethal = _number((outcome or {}).get('timeout_footprint_lethal_cell_count'))
    base.update({
        'outcome_timeout_footprint_cost_max': out_max,
        'outcome_timeout_footprint_lethal_cell_count': out_lethal,
        'footprint_high_risk': bool((out_max is not None and out_max >= LETHAL_COST_THRESHOLD) or (out_lethal is not None and out_lethal > 0) or base['max_lethal_count'] > 0),
    })
    return base


def _target_footprint_summary(samples: list[dict[str, Any]]) -> dict[str, Any]:
    return _cost_summary(samples, 'target_footprint_cost')


def _global_plan_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    counts = [int(row.get('point_count') or 0) for row in rows if 'point_count' in row]
    return {'sample_count': len(rows), 'point_count': _stats([float(v) for v in counts]), 'latest': rows[-1] if rows else None}


def _collision_monitor_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    actions: dict[str, int] = {}
    for row in rows:
        action = str(row.get('action_type') or row.get('action') or row.get('state') or 'unknown')
        actions[action] = actions.get(action, 0) + 1
    return {'sample_count': len(rows), 'action_type_counts': actions, 'latest': rows[-1] if rows else None}


def _timestamp_age_summary(dispatch: dict[str, Any] | None, outcome: dict[str, Any] | None) -> dict[str, Any]:
    consistency = (dispatch or {}).get('phase62_timestamp_consistency')
    if not isinstance(consistency, dict):
        consistency = {}
    return {
        'dispatch_phase62_timestamp_consistency': consistency or None,
        'dispatch_map_age_sec': _number(consistency.get('map_age_sec')),
        'dispatch_scan_age_sec': _number(consistency.get('scan_age_sec')),
        'dispatch_local_costmap_age_sec': _number(consistency.get('local_costmap_age_sec') or (dispatch or {}).get('dispatch_local_cost_sample_age_sec')),
        'dispatch_tf_age_sec': _number(consistency.get('tf_age_sec')),
        'dispatch_all_available': consistency.get('all_available'),
        'timeout_local_cost_sample_age_sec': _number((outcome or {}).get('timeout_local_cost_sample_age_sec')),
        'timeout_robot_in_local_costmap_bounds': (outcome or {}).get('timeout_robot_in_local_costmap_bounds'),
    }


def _goal_tolerance_band(target: list[float] | None, final_pose: list[float] | None, branch_angle: float | None) -> dict[str, Any]:
    xy_error = None
    yaw_error = None
    if target is not None and final_pose is not None and len(final_pose) >= 2:
        xy_error = math.hypot(final_pose[0] - target[0], final_pose[1] - target[1])
    if final_pose is not None and len(final_pose) >= 3 and branch_angle is not None:
        yaw_error = abs(_angle_wrap(final_pose[2] - branch_angle))
    return {
        'xy_goal_tolerance_m': XY_GOAL_TOLERANCE_M,
        'yaw_goal_tolerance_rad': YAW_GOAL_TOLERANCE_RAD,
        'final_xy_error_m': xy_error,
        'final_yaw_error_to_branch_rad': yaw_error,
        'inside_xy_tolerance': bool(xy_error is not None and xy_error <= XY_GOAL_TOLERANCE_M),
        'inside_yaw_tolerance_to_branch': bool(yaw_error is not None and yaw_error <= YAW_GOAL_TOLERANCE_RAD),
        'near_goal_but_outside_xy_tolerance': bool(xy_error is not None and XY_GOAL_TOLERANCE_M < xy_error <= XY_GOAL_TOLERANCE_M + 0.20),
    }


def _target_wall_risk_summary(dispatch: dict[str, Any] | None, samples: list[dict[str, Any]]) -> dict[str, Any]:
    target_clearance = _number((dispatch or {}).get('target_clearance_m'))
    dispatch_target_cost = _first_number((dispatch or {}).get('dispatch_target_local_cost'), (dispatch or {}).get('target_local_cost'))
    dispatch_target_radius = _first_number((dispatch or {}).get('dispatch_target_local_cost_max_radius'), (dispatch or {}).get('target_local_cost_max_radius'))
    target_footprint = _target_footprint_summary(samples)
    latest_target = _local_cost_summary(samples)
    target_too_close = bool(
        (target_clearance is not None and target_clearance < 0.25)
        or (dispatch_target_cost is not None and dispatch_target_cost >= HIGH_COST_THRESHOLD)
        or (dispatch_target_radius is not None and dispatch_target_radius >= LETHAL_COST_THRESHOLD)
        or target_footprint.get('max_lethal_count', 0) > 0
    )
    return {
        'target_clearance_m': target_clearance,
        'dispatch_target_local_cost': dispatch_target_cost,
        'dispatch_target_local_cost_max_radius': dispatch_target_radius,
        'target_footprint_summary': target_footprint,
        'latest_runtime_target_cost_summary': latest_target,
        'target_too_close_to_wall': target_too_close,
    }


def _event_summary(event: dict[str, Any] | None) -> dict[str, Any] | None:
    if not event:
        return None
    keys = [
        'event',
        'goal_sequence',
        'target',
        'original_target',
        'refined_target',
        'dispatch_pose',
        'branch_angle',
        'target_clearance_m',
        'dispatch_target_local_cost',
        'dispatch_target_local_cost_max_radius',
        'dispatch_path_local_cost_max',
        'dispatch_path_local_cost_mean',
        'path_corridor_min_clearance_m',
        'result_reason',
        'result_status',
        'effective_timeout_sec',
        'timeout_front_wedge_clearance_m',
        'timeout_front_wedge_cost_max',
        'timeout_footprint_cost_max',
        'timeout_footprint_lethal_cell_count',
        'timeout_local_cost_sample_age_sec',
        'timeout_robot_in_local_costmap_bounds',
        'phase62_timestamp_consistency',
        '_recorder_elapsed_sec',
        '_recorder_wall_time',
    ]
    return {key: event.get(key) for key in keys if key in event}


def _first_number(*values: Any) -> float | None:
    for value in values:
        number = _number(value)
        if number is not None:
            return number
    return None


def _nav2_result_summary(outcome: dict[str, Any] | None, cancel: dict[str, Any] | None) -> dict[str, Any]:
    result = (outcome or {}).get('phase62_nav2_result_summary')
    if not isinstance(result, dict):
        result = None
    return {
        'outcome_event': (outcome or {}).get('event'),
        'outcome_result_reason': (outcome or {}).get('result_reason'),
        'outcome_result_status': (outcome or {}).get('result_status'),
        'last_nav2_result': (outcome or {}).get('last_nav2_result'),
        'phase62_nav2_result_summary': result,
        'cancel_event': cancel.get('event') if cancel else None,
        'cancel_result_reason': cancel.get('result_reason') if cancel else None,
    }


def _nav2_log_summary(launch_text: str) -> dict[str, Any]:
    return {
        'failed_to_make_progress_count': launch_text.count('Failed to make progress'),
        'goal_succeeded_count': launch_text.count('Goal succeeded'),
        'reached_goal_count': launch_text.count('Reached the goal!'),
        'follow_path_abort_count': len(re.findall(r'Aborting handle', launch_text, flags=re.IGNORECASE)),
        'costmap_clear_count': len(re.findall(r'Received request to clear entirely.*costmap', launch_text, flags=re.IGNORECASE)),
        'bt_goal_canceled_count': len(re.findall(r'Goal canceled|Client requested to cancel|Goal was canceled', launch_text, flags=re.IGNORECASE)),
        'tail_key_lines': [line for line in launch_text.splitlines() if any(token in line for token in ['Failed to make progress', 'Aborting handle', 'Received request to clear entirely', 'Goal canceled', 'Client requested to cancel'])][-20:],
    }


def classify_goal2_timeout(summary: dict[str, Any]) -> str:
    window = summary.get('goal2_window') or {}
    if not window.get('dispatch_observed') or window.get('outcome_event') != 'timeout':
        return 'INSUFFICIENT_EVIDENCE'
    target = summary.get('target_wall_risk_summary') or {}
    tolerance = summary.get('goal_tolerance_band') or {}
    recovery = summary.get('nav2_recovery_summary') or {}
    front = summary.get('front_wedge_summary') or {}
    footprint = summary.get('footprint_summary') or {}
    trajectory = summary.get('trajectory_summary') or {}
    distance = summary.get('distance_remaining_summary') or {}
    cmd = summary.get('cmd_vel_summary') or {}
    if target.get('target_too_close_to_wall'):
        return 'GOAL2_TIMEOUT_TARGET_TOO_CLOSE_TO_WALL'
    # Prefer the recovery/local-cost subtype over the near-goal subtype when
    # Nav2 explicitly entered a progress-failure / recovery loop and runtime
    # footprint or front-wedge evidence is high-risk.  The robot may also be
    # near the XY tolerance band, but Phase75 is diagnosing why it could not
    # complete; the recovery loop is the stronger causal evidence.
    if (
        int(recovery.get('number_of_recoveries_max') or 0) > 0
        and (
            int(recovery.get('failed_to_make_progress_count') or 0) > 0
            or bool(front.get('front_wedge_blocked'))
            or bool(footprint.get('footprint_high_risk'))
        )
    ):
        return 'GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP'
    if tolerance.get('near_goal_but_outside_xy_tolerance') and (cmd.get('last_10s_all_near_zero') or trajectory.get('last_10s_stalled')):
        return 'GOAL2_TIMEOUT_NEAR_GOAL_TOLERANCE_ORIENTATION'
    nonzero_improvement = _number((distance.get('distance_remaining_nonzero') or {}).get('first'))
    nonzero_final = _number((distance.get('distance_remaining_nonzero') or {}).get('final'))
    if trajectory.get('last_10s_stalled') or (nonzero_improvement is not None and nonzero_final is not None and nonzero_improvement - nonzero_final < 0.10):
        return 'GOAL2_TIMEOUT_PROGRESS_INSUFFICIENT'
    return 'INSUFFICIENT_EVIDENCE'


def analyze_artifact(artifact_dir: Path | str, *, phase74_summary_path: Path | str | None = None, output: Path | str | None = None) -> dict[str, Any]:
    artifact_dir = Path(artifact_dir)
    if phase74_summary_path is not None:
        phase74_summary_path = Path(phase74_summary_path)
    else:
        candidate = artifact_dir.parent / f'{PHASE74_RUN_ID}.json'
        phase74_summary_path = candidate if candidate.exists() else None

    events = _read_jsonl(_goal_events_path(artifact_dir))
    states = _read_jsonl(_explorer_state_path(artifact_dir))
    controller = _rows_for_goal(_read_jsonl(artifact_dir / 'phase74_controller_dynamics.jsonl'))
    feedback = _rows_for_goal(_read_jsonl(artifact_dir / 'phase74_nav2_feedback.jsonl'))
    local_cost = _rows_for_goal(_read_jsonl(artifact_dir / 'phase74_local_costmap_samples.jsonl'))
    global_plan = _rows_for_goal(_read_jsonl(artifact_dir / 'phase74_global_plan_samples.jsonl'))
    collision = _rows_for_goal(_read_jsonl(artifact_dir / 'phase74_collision_monitor_state.jsonl'))
    launch_text = _read_text(artifact_dir / f'{PHASE74_RUN_ID}_{REPLAY_ID}_launch.log')

    dispatch = _find_goal_event(events, 'dispatch')
    outcome = _goal_outcome(events)
    cancel = _find_goal_event(events, 'timeout_cancel_result')
    start_wall = _wall_time(dispatch)
    end_wall = _wall_time(outcome)
    controller_window = _rows_in_window(controller, start_wall, end_wall)
    feedback_window = _rows_in_window(feedback, start_wall, end_wall)
    local_cost_window = _rows_in_window(local_cost, start_wall, end_wall)
    global_plan_window = _rows_in_window(global_plan, start_wall, end_wall)
    collision_window = _rows_in_window(collision, start_wall, end_wall)

    goal2_actual_target = _target_field(dispatch, 'target') or _target_field(outcome, 'target')
    goal2_original_target = _target_field(dispatch, 'original_target') or goal2_actual_target
    goal2_refined_target = _target_field(dispatch, 'refined_target') or goal2_actual_target
    dispatch_pose = _pose_field(dispatch, 'dispatch_pose')
    branch_angle = _number((dispatch or {}).get('branch_angle'))
    trajectory = _trajectory_summary(controller_window, goal2_actual_target)
    final_pose = trajectory.get('final_pose')

    distance_remaining_summary = _distance_remaining_summary(feedback_window)
    cmd_vel_summary = _cmd_vel_summary(controller_window)
    nav2_recovery_summary = _nav2_recovery_summary(feedback_window, launch_text)
    nav2_result_summary = _nav2_result_summary(outcome, cancel)
    local_cost_summary = _local_cost_summary(local_cost_window)
    footprint_summary = _footprint_summary(local_cost_window, outcome)
    front_wedge_summary = _front_wedge_summary(local_cost_window, outcome)
    global_plan_summary = _global_plan_summary(global_plan_window)
    collision_monitor_summary = _collision_monitor_summary(collision_window)
    timestamp_age_summary = _timestamp_age_summary(dispatch, outcome)
    goal_tolerance_band = _goal_tolerance_band(goal2_actual_target, final_pose, branch_angle)
    target_wall_risk_summary = _target_wall_risk_summary(dispatch, local_cost_window)
    nav2_log_summary = _nav2_log_summary(launch_text)

    mode_transitions: list[dict[str, Any]] = []
    prev = object()
    for row in states:
        mode = row.get('mode')
        if mode != prev:
            mode_transitions.append({
                'elapsed_sec': _elapsed(row),
                'wall_time': _wall_time(row),
                'mode': mode,
                'goal_count': row.get('goal_count'),
                'goal_success_count': row.get('goal_success_count'),
                'goal_failure_count': row.get('goal_failure_count'),
                'active_goal_sequence_id': row.get('active_goal_sequence_id'),
                'last_failure_reason': row.get('last_failure_reason'),
                'last_nav2_status': row.get('last_nav2_status'),
            })
            prev = mode

    goal2_window = {
        'dispatch_observed': dispatch is not None,
        'outcome_event': (outcome or {}).get('event'),
        'dispatch_elapsed_sec': _elapsed(dispatch),
        'outcome_elapsed_sec': _elapsed(outcome),
        'duration_sec': (_wall_time(outcome) - _wall_time(dispatch)) if _wall_time(dispatch) is not None and _wall_time(outcome) is not None else None,
        'effective_timeout_sec': _number((dispatch or {}).get('effective_timeout_sec') or (outcome or {}).get('effective_timeout_sec')),
        'mode_transitions': mode_transitions,
    }

    summary: dict[str, Any] = {
        'phase': PHASE,
        'run_id': RUN_ID,
        'source_phase74_run_id': PHASE74_RUN_ID,
        'source_phase74_classification': _phase74_classification(Path(phase74_summary_path) if phase74_summary_path else None),
        'artifact_dir': str(artifact_dir),
        'replay_id': REPLAY_ID,
        'goal_sequence': GOAL_SEQUENCE,
        'goal2_window': goal2_window,
        'goal2_dispatch_event': _event_summary(dispatch),
        'goal2_outcome_event': _event_summary(outcome),
        'goal2_cancel_event': _event_summary(cancel),
        'goal2_original_target': goal2_original_target,
        'goal2_refined_target': goal2_refined_target,
        'goal2_actual_target': goal2_actual_target,
        'dispatch_pose': dispatch_pose,
        'trajectory_summary': trajectory,
        'final_pose': final_pose,
        'distance_remaining_summary': distance_remaining_summary,
        'goal_tolerance_band': goal_tolerance_band,
        'cmd_vel_summary': cmd_vel_summary,
        'nav2_recovery_summary': nav2_recovery_summary,
        'nav2_result_summary': nav2_result_summary,
        'nav2_log_summary': nav2_log_summary,
        'local_cost_summary': local_cost_summary,
        'footprint_summary': footprint_summary,
        'front_wedge_summary': front_wedge_summary,
        'target_wall_risk_summary': target_wall_risk_summary,
        'global_plan_summary': global_plan_summary,
        'collision_monitor_summary': collision_monitor_summary,
        'timestamp_age_summary': timestamp_age_summary,
        'classification': 'INSUFFICIENT_EVIDENCE',
        'classification_reason': None,
        'guardrails': GUARDRAILS,
        'complete_autonomous_success_claimed': False,
        'exit_success_claimed': False,
    }
    summary['classification'] = classify_goal2_timeout(summary)
    reason_map = {
        'GOAL2_TIMEOUT_TARGET_TOO_CLOSE_TO_WALL': 'Goal2 target geometry/runtime target footprint is too close to high/lethal local cost or wall clearance is below target safety band.',
        'GOAL2_TIMEOUT_NEAR_GOAL_TOLERANCE_ORIENTATION': 'Robot reached near the XY tolerance band but stalled outside goal tolerance/orientation acceptance.',
        'GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP': 'Nav2 feedback/logs show recoveries or Failed to make progress while runtime footprint/front wedge local cost is high/lethal.',
        'GOAL2_TIMEOUT_PROGRESS_INSUFFICIENT': 'Robot/feedback progress is insufficient or stalls without enough local-cost/recovery evidence for a stronger subtype.',
        'INSUFFICIENT_EVIDENCE': 'Required dispatch, timeout, trajectory, feedback, or local-cost evidence is missing or contradictory.',
    }
    summary['classification_reason'] = reason_map.get(summary['classification'])

    if output is not None:
        Path(output).parent.mkdir(parents=True, exist_ok=True)
        Path(output).write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    return summary


def _default_artifact_dir() -> Path:
    return Path('log') / PHASE74_RUN_ID / REPLAY_ID


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=PHASE)
    parser.add_argument('--artifact-dir', default=str(_default_artifact_dir()), help='Phase74 replay_02 artifact directory')
    parser.add_argument('--phase74-summary', default=None, help='Phase74 top-level summary JSON')
    parser.add_argument('--output', default=str(Path('log') / RUN_ID / f'{RUN_ID}.json'))
    parser.add_argument('--print-summary', action='store_true')
    args = parser.parse_args(argv)
    summary = analyze_artifact(args.artifact_dir, phase74_summary_path=args.phase74_summary, output=args.output)
    if args.print_summary:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        print(f"classification={summary['classification']}")
        print(f"artifact={args.output}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
