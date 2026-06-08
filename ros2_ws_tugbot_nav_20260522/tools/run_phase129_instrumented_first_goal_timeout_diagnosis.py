#!/usr/bin/env python3
"""Phase129 instrumented first-goal timeout runtime diagnosis.

Scope guardrails:
- Repeat the Phase125 bounded first-goal result smoke under the visible stack.
- Send only the explicit inner-ingress goal: map,x=2.0,y=0.0,yaw=0.0 through the inherited Phase120 chain.
- Start maze_explorer with max_goals:=1 only after Phase122-style handoff readiness.
- Allow only the first current-algorithm goal_kind=explore.
- Stop after the first terminal result: succeeded/timeout/abort/cancel/reject/no-goal.
- Do not send a second exploration goal or any manual Goal1/carry-over/staging/branch/centerline/fallback/terminal/exit goal.
- Do not tune Nav2/MPPI/controller/goal checker/config or change exploration logic.
- Do not claim autonomous exploration success or exit success.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import signal
import subprocess
import sys
import time
import traceback
from pathlib import Path
from typing import Any

PHASE = 'Phase129'
MODE = 'instrumented_first_goal_timeout_diagnosis'
ACTION_NAME = '/navigate_to_pose'
GOAL_EVENTS_TOPIC = '/maze/goal_events'
EXPLORER_STATE_TOPIC = '/maze/explorer_state'
LOCKED_GOAL = {'frame_id': 'map', 'x': 2.0, 'y': 0.0, 'yaw': 0.0}

FIRST_GOAL_RESULT_SUCCEEDED_WITH_INSTRUMENTATION_STOP = 'FIRST_GOAL_RESULT_SUCCEEDED_WITH_INSTRUMENTATION_STOP'
FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED = 'FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED'
FIRST_GOAL_TIMEOUT_CONTROLLER_STALL = 'FIRST_GOAL_TIMEOUT_CONTROLLER_STALL'
FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE = 'FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE'
FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY = 'FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY'
INSUFFICIENT_TIMEOUT_EVIDENCE = 'INSUFFICIENT_TIMEOUT_EVIDENCE'
CLASSIFICATIONS = [
    FIRST_GOAL_RESULT_SUCCEEDED_WITH_INSTRUMENTATION_STOP,
    FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED,
    FIRST_GOAL_TIMEOUT_CONTROLLER_STALL,
    FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE,
    FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY,
    INSUFFICIENT_TIMEOUT_EVIDENCE,
]

FORBIDDEN_ACTIONS = {
    'second_exploration_goal': False,
    'manual_goal1_carry_over_staging_branch_centerline_fallback_terminal_exit_goal': False,
    'nav2_mppi_controller_goal_checker_config_tuning': False,
    'exploration_strategy_branch_scoring_centerline_fallback_terminal_acceptance_change': False,
    'autonomous_or_exit_success_claim': False,
}

REQUIRED_RESULT_FIELDS = [
    'maze_explorer_start_allowed',
    'maze_explorer_started',
    'maze_explorer_max_goals',
    'exploration_goal_dispatched',
    'first_goal',
    'goal_event_count',
    'dispatch_event_count',
    'second_goal_dispatched',
    'goal_events',
    'explorer_states',
    'nav2_feedback',
    'action_status_samples',
    'local_costmap_samples',
    'global_costmap_samples',
    'scan',
    'map',
    'odom',
    'tf',
    'costmap_freshness',
    'scan_freshness',
    'tf_freshness',
    'instrumentation',
    'stop_reason',
    'process',
]

INSTRUMENTATION_FIELDS = [
    'cmd_vel_timeline',
    'odom_velocity_timeline',
    'commanded_vs_measured_velocity',
    'robot_pose_trace',
    'distance_error_curve',
    'yaw_error_curve',
    'goal_checker_state',
    'local_costmap_windows',
    'global_costmap_windows',
    'footprint_front_path_cost_snapshots',
    'planned_path_snapshots',
    'bt_recovery_events',
    'controller_server_logs',
    'bt_navigator_logs',
    'first_goal_candidate_risk',
    'evidence_coverage',
    'evidence_gaps',
]


def _load_phase125_runner() -> Any:
    path = Path(__file__).resolve().parent / 'run_phase125_first_exploration_goal_execution_result_smoke.py'
    spec = importlib.util.spec_from_file_location('phase125_runner_for_phase129', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot import Phase125 runner from {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_p125 = _load_phase125_runner()
_phase122 = _p125._phase122


def _now_wall() -> float:
    return time.time()


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'y'}
    return bool(value)


def _float(value: Any, default: float | None = None) -> float | None:
    try:
        if value is None:
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def _int(value: Any, default: int = 0) -> int:
    try:
        if value is None:
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def _stamp_dict(stamp: Any) -> dict[str, int] | None:
    return _p125._stamp_dict(stamp)


def _duration_msg_to_sec(msg: Any) -> float | None:
    return _p125._duration_msg_to_sec(msg)


def _goal_pose_from_event(event: dict[str, Any]) -> dict[str, Any]:
    return _p125._goal_pose_from_event(event)


def _candidate_from_event(event: dict[str, Any]) -> dict[str, Any]:
    return _p125._candidate_from_event(event)


def _frontier_evidence_from_event(event: dict[str, Any]) -> dict[str, Any]:
    return _p125._frontier_evidence_from_event(event)


def _topology_evidence_from_event(event: dict[str, Any]) -> dict[str, Any]:
    return _p125._topology_evidence_from_event(event)


def _goal_event_matches_first(dispatch: dict[str, Any], event: dict[str, Any]) -> bool:
    return _p125._goal_event_matches_first(dispatch, event)


def _status_label(status: Any) -> str | None:
    return _p125._status_label(status)


def _result_status_from_event(event: dict[str, Any]) -> str | None:
    return _p125._result_status_from_event(event)


def _latest_result_event(dispatch: dict[str, Any], goal_events: list[dict[str, Any]]) -> dict[str, Any] | None:
    return _p125._latest_result_event(dispatch, goal_events)


def _quat_to_yaw(q: Any) -> float:
    x = float(getattr(q, 'x', 0.0))
    y = float(getattr(q, 'y', 0.0))
    z = float(getattr(q, 'z', 0.0))
    w = float(getattr(q, 'w', 1.0))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _angle_diff(a: float | None, b: float | None) -> float | None:
    if a is None or b is None:
        return None
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _runtime_tf_pose(buffer: Any, node: Any, target_frame: str = 'map', source_frame: str = 'base_link') -> dict[str, Any]:
    try:
        import rclpy
        from rclpy.time import Time
        trans = buffer.lookup_transform(target_frame, source_frame, Time(), timeout=rclpy.duration.Duration(seconds=0.05))
        yaw = _quat_to_yaw(trans.transform.rotation)
        return {
            'fresh': True,
            'frame_id': target_frame,
            'child_frame_id': source_frame,
            'stamp': _stamp_dict(trans.header.stamp),
            'received_wall_time_sec': _now_wall(),
            'x': float(trans.transform.translation.x),
            'y': float(trans.transform.translation.y),
            'z': float(trans.transform.translation.z),
            'yaw': yaw,
            'translation': {
                'x': float(trans.transform.translation.x),
                'y': float(trans.transform.translation.y),
                'z': float(trans.transform.translation.z),
            },
            'rotation_yaw': yaw,
        }
    except Exception as exc:  # noqa: BLE001 - runtime TF failures are evidence gaps.
        return {'fresh': False, 'frame_id': target_frame, 'child_frame_id': source_frame, 'exception_text': str(exc), 'received_wall_time_sec': _now_wall()}


def _transform_map_point_to_frame(buffer: Any, node: Any, point: dict[str, Any], target_frame: str) -> tuple[float | None, float | None]:
    x = _float(point.get('x'))
    y = _float(point.get('y'))
    if x is None or y is None:
        return None, None
    if target_frame in ('map', '', None):
        return x, y
    try:
        import rclpy
        from rclpy.time import Time
        trans = buffer.lookup_transform(target_frame, 'map', Time(), timeout=rclpy.duration.Duration(seconds=0.05))
        tx = float(trans.transform.translation.x)
        ty = float(trans.transform.translation.y)
        yaw = _quat_to_yaw(trans.transform.rotation)
        c = math.cos(yaw)
        s = math.sin(yaw)
        return c * x - s * y + tx, s * x + c * y + ty
    except Exception:
        return None, None


def _project_to_grid(msg: Any, x: float | None, y: float | None) -> tuple[int | None, int | None, bool]:
    if x is None or y is None:
        return None, None, False
    res = float(msg.info.resolution)
    origin = msg.info.origin.position
    gx = int(math.floor((x - float(origin.x)) / res))
    gy = int(math.floor((y - float(origin.y)) / res))
    in_bounds = 0 <= gx < int(msg.info.width) and 0 <= gy < int(msg.info.height)
    return gx, gy, in_bounds


def _cell_cost(data: Any, width: int, height: int, gx: int, gy: int) -> int | None:
    if not (0 <= gx < width and 0 <= gy < height):
        return None
    idx = gy * width + gx
    try:
        return int(data[idx])
    except Exception:
        return None


def _window_stats(msg: Any, center_x: int | None, center_y: int | None, radius_cells: int = 6) -> dict[str, Any]:
    width = int(msg.info.width)
    height = int(msg.info.height)
    if center_x is None or center_y is None or not (0 <= center_x < width and 0 <= center_y < height):
        return {'available': False, 'in_bounds': False}
    costs: list[int] = []
    for gy in range(max(0, center_y - radius_cells), min(height, center_y + radius_cells + 1)):
        for gx in range(max(0, center_x - radius_cells), min(width, center_x + radius_cells + 1)):
            c = _cell_cost(msg.data, width, height, gx, gy)
            if c is not None:
                costs.append(c)
    known = [c for c in costs if c >= 0]
    return {
        'available': True,
        'in_bounds': True,
        'center_grid': {'x': center_x, 'y': center_y},
        'radius_cells': radius_cells,
        'sample_count': len(costs),
        'known_count': len(known),
        'unknown_count': sum(1 for c in costs if c < 0),
        'lethal_count': sum(1 for c in known if c >= 99),
        'inflated_count': sum(1 for c in known if 50 <= c < 99),
        'max': max(known) if known else None,
        'mean': (sum(known) / len(known)) if known else None,
    }


def _sample_forward(msg: Any, robot_x: float | None, robot_y: float | None, yaw: float | None, distance: float, lateral_offsets: list[float]) -> list[int]:
    if robot_x is None or robot_y is None or yaw is None:
        return []
    width = int(msg.info.width)
    height = int(msg.info.height)
    costs: list[int] = []
    for lat in lateral_offsets:
        x = robot_x + math.cos(yaw) * distance - math.sin(yaw) * lat
        y = robot_y + math.sin(yaw) * distance + math.cos(yaw) * lat
        gx, gy, ok = _project_to_grid(msg, x, y)
        if ok and gx is not None and gy is not None:
            c = _cell_cost(msg.data, width, height, gx, gy)
            if c is not None:
                costs.append(c)
    return costs


def _cost_values_summary(values: list[int]) -> dict[str, Any]:
    known = [v for v in values if v >= 0]
    return {
        'sample_count': len(values),
        'known_count': len(known),
        'unknown_count': sum(1 for v in values if v < 0),
        'max': max(known) if known else None,
        'mean': (sum(known) / len(known)) if known else None,
        'lethal_count': sum(1 for v in known if v >= 99),
        'inflated_count': sum(1 for v in known if 50 <= v < 99),
    }


def _cost_snapshot_from_local_costmap(msg: Any, *, robot_frame_pose: dict[str, Any], goal_frame_xy: tuple[float | None, float | None], latest_path: dict[str, Any] | None = None) -> dict[str, Any]:
    frame = msg.header.frame_id
    rx = _float(robot_frame_pose.get('x'))
    ry = _float(robot_frame_pose.get('y'))
    yaw = _float(robot_frame_pose.get('yaw'))
    gx, gy, robot_in_bounds = _project_to_grid(msg, rx, ry)
    tx, ty = goal_frame_xy
    target_gx, target_gy, target_in_bounds = _project_to_grid(msg, tx, ty)
    footprint = _window_stats(msg, gx, gy, radius_cells=5)
    front_vals = []
    for dist in (0.2, 0.35, 0.5, 0.65):
        front_vals.extend(_sample_forward(msg, rx, ry, yaw, dist, [-0.22, -0.11, 0.0, 0.11, 0.22]))
    p05_vals = _sample_forward(msg, rx, ry, yaw, 0.5, [-0.18, -0.09, 0.0, 0.09, 0.18])
    p10_vals = _sample_forward(msg, rx, ry, yaw, 1.0, [-0.18, -0.09, 0.0, 0.09, 0.18])
    front = _cost_values_summary(front_vals)
    p05 = _cost_values_summary(p05_vals)
    p10 = _cost_values_summary(p10_vals)
    target_window = _window_stats(msg, target_gx, target_gy, radius_cells=6)
    return {
        'sample_wall_time_sec': _now_wall(),
        'frame_id': frame,
        'stamp': _stamp_dict(msg.header.stamp),
        'resolution': float(msg.info.resolution),
        'width': int(msg.info.width),
        'height': int(msg.info.height),
        'robot_grid': {'x': gx, 'y': gy, 'in_bounds': robot_in_bounds},
        'target_grid': {'x': target_gx, 'y': target_gy, 'in_bounds': target_in_bounds},
        'in_bounds': bool(robot_in_bounds),
        'footprint': footprint,
        'front_wedge': front,
        'path_0_5m': p05,
        'path_1_0m': p10,
        'target_window': target_window,
        'footprint_max': footprint.get('max'),
        'front_max': front.get('max'),
        'path_0_5m_max': p05.get('max'),
        'path_1_0m_max': p10.get('max'),
    }


def _freshness_from_sample(sample: dict[str, Any], *, max_age_sec: float, required_seen_key: str | None = None) -> dict[str, Any]:
    now = _now_wall()
    received = _float(sample.get('received_wall_time_sec'))
    age = None if received is None else max(0.0, now - received)
    seen = bool(sample.get(required_seen_key)) if required_seen_key else bool(sample)
    fresh = bool(seen and age is not None and age <= max_age_sec)
    return {'fresh': fresh, 'seen': seen, 'age_sec': age, 'max_allowed_age_sec': max_age_sec}


def _costmap_freshness(local_costmap_samples: list[dict[str, Any]], global_costmap_samples: list[dict[str, Any]], max_age_sec: float = 5.0) -> dict[str, Any]:
    latest_local = local_costmap_samples[-1] if local_costmap_samples else {}
    latest_global = global_costmap_samples[-1] if global_costmap_samples else {}
    now = _now_wall()
    local_received = _float(latest_local.get('received_wall_time_sec'))
    global_received = _float(latest_global.get('received_wall_time_sec'))
    local_age = None if local_received is None else max(0.0, now - local_received)
    global_age = None if global_received is None else max(0.0, now - global_received)
    return {
        'fresh': bool(latest_local and local_age is not None and local_age <= max_age_sec),
        'local_costmap_available': bool(latest_local),
        'global_costmap_available': bool(latest_global),
        'local_costmap_age_sec': local_age,
        'global_costmap_age_sec': global_age,
        'max_allowed_age_sec': max_age_sec,
    }


def initial_instrumented_first_goal_record(*, max_goals: int = 1) -> dict[str, Any]:
    return {
        'maze_explorer_start_allowed': False,
        'maze_explorer_started': False,
        'maze_explorer_max_goals': int(max_goals),
        'exploration_goal_dispatched': False,
        'first_goal': {
            'candidate': {},
            'pose': {},
            'frame_id': None,
            'goal_kind': None,
            'selection_reason': None,
            'frontier_evidence': {},
            'topology_evidence': {},
            'dispatch_wall_time_sec': None,
            'accepted': False,
            'rejected': False,
            'nav2_feedback_timeline': [],
            'result_status_label': None,
            'abort_text': None,
            'timeout': False,
            'robot_pose_at_result': {'pose_available': False},
            'distance_to_first_goal': {'available': False, 'meters': None},
        },
        'goal_event_count': 0,
        'dispatch_event_count': 0,
        'second_goal_dispatched': False,
        'goal_events': [],
        'explorer_states': [],
        'nav2_feedback': [],
        'action_status_samples': [],
        'local_costmap_samples': [],
        'global_costmap_samples': [],
        'scan': {},
        'map': {},
        'odom': {},
        'tf': {},
        'costmap_freshness': {'fresh': False},
        'scan_freshness': {'fresh': False},
        'tf_freshness': {'fresh': False},
        'instrumentation': {
            **{field: [] for field in INSTRUMENTATION_FIELDS},
            'goal_checker_state': {'available': False},
            'commanded_vs_measured_velocity': {'available': False},
            'first_goal_candidate_risk': {'available': False},
            'evidence_coverage': {},
            'evidence_gaps': [],
        },
        'observation_timed_out': False,
        'stop_reason': None,
        'process': {
            'command': ['ros2', 'run', 'tugbot_maze', 'maze_explorer', '--ros-args', '-p', 'use_sim_time:=true', '-p', 'max_goals:=1'],
            'pid': None,
            'returncode': None,
            'stdout_path': None,
            'stderr_path': None,
            'exception_text': None,
        },
    }


def _instrumentation_coverage(instrumentation: dict[str, Any]) -> dict[str, dict[str, Any]]:
    coverage: dict[str, dict[str, Any]] = {}
    for field in INSTRUMENTATION_FIELDS:
        value = instrumentation.get(field)
        if isinstance(value, list):
            present = len(value) > 0
            count = len(value)
        elif isinstance(value, dict):
            present = bool(value.get('available', bool(value)))
            count = 1 if present else 0
        else:
            present = bool(value)
            count = 1 if present else 0
        coverage[field] = {'present': present, 'count': count}
    return coverage


def _evidence_gaps(instrumentation: dict[str, Any]) -> list[str]:
    coverage = _instrumentation_coverage(instrumentation)
    gaps = [field for field, info in coverage.items() if field not in {'evidence_gaps', 'evidence_coverage'} and not info['present']]
    return gaps


def _build_candidate_risk(first: dict[str, Any]) -> dict[str, Any]:
    frontier = _dict(first.get('frontier_evidence'))
    branches = _list(frontier.get('candidate_branches'))
    chosen_rank = _int(frontier.get('chosen_branch_rank'), 1)
    selected = None
    for branch in branches:
        if _int(_dict(branch).get('rank'), -1) == chosen_rank:
            selected = _dict(branch)
            break
    if selected is None and branches:
        selected = _dict(branches[0])
    selected = selected or {}
    clearance = _float(selected.get('target_clearance_m'), _float(selected.get('clearance')))
    corridor = _float(selected.get('path_corridor_min_clearance_m'), _float(selected.get('path_corridor_min_clearance')))
    max_radius = _float(selected.get('target_local_cost_max_radius'))
    target_cost = _float(selected.get('target_local_cost'))
    safer_alternative = False
    for branch in branches:
        b = _dict(branch)
        if _int(b.get('rank'), -1) == chosen_rank:
            continue
        b_clear = _float(b.get('target_clearance_m'), _float(b.get('clearance')))
        b_corr = _float(b.get('path_corridor_min_clearance_m'), _float(b.get('path_corridor_min_clearance')))
        b_cost = _float(b.get('target_local_cost_max_radius'), _float(b.get('target_local_cost')))
        if ((b_clear is not None and clearance is not None and b_clear > clearance + 0.20)
                or (b_corr is not None and corridor is not None and b_corr > corridor + 0.20)
                or (b_cost is not None and max_radius is not None and b_cost + 30 < max_radius)):
            safer_alternative = True
    return {
        'available': bool(selected or first.get('candidate')),
        'candidate_family': _dict(first.get('candidate')).get('candidate_family'),
        'candidate_rank': _dict(first.get('candidate')).get('candidate_rank'),
        'clearance': clearance,
        'target_clearance_m': clearance,
        'target_local_cost': target_cost,
        'target_local_cost_max_radius': max_radius,
        'path_corridor_min_clearance_m': corridor,
        'branch_geometry': {
            'branch_angle': _dict(first.get('candidate')).get('branch_angle'),
            'candidate_branch_count': frontier.get('candidate_branch_count'),
            'chosen_branch_rank': chosen_rank,
        },
        'selected_branch_vs_lower_ranked_branches': branches,
        'selected_branch_riskier_than_alternatives': safer_alternative,
        'near_wall': bool(clearance is not None and clearance < 0.30),
        'near_high_cost_band': bool((max_radius is not None and max_radius >= 90) or (target_cost is not None and target_cost >= 90)),
    }


def _robot_pose_from_tf(tf: dict[str, Any]) -> dict[str, Any]:
    x = _float(tf.get('x'), _float(_dict(tf.get('translation')).get('x')))
    y = _float(tf.get('y'), _float(_dict(tf.get('translation')).get('y')))
    return {
        'pose_available': x is not None and y is not None,
        'frame_id': tf.get('frame_id', 'map'),
        'x': x,
        'y': y,
        'yaw': _float(tf.get('yaw'), _float(tf.get('rotation_yaw'))),
        'sample_wall_time_sec': tf.get('received_wall_time_sec'),
        'source': 'tf_map_base_link',
    }


def _distance_to_goal(robot_pose: dict[str, Any], goal_pose: dict[str, Any]) -> dict[str, Any]:
    rx = _float(robot_pose.get('x'))
    ry = _float(robot_pose.get('y'))
    gx = _float(goal_pose.get('x'))
    gy = _float(goal_pose.get('y'))
    if rx is None or ry is None or gx is None or gy is None:
        return {'available': False, 'meters': None}
    return {'available': True, 'meters': math.hypot(rx - gx, ry - gy)}


def _finalize_record_from_observation(*, goal_events: list[dict[str, Any]], explorer_states: list[dict[str, Any]], action_status_samples: list[dict[str, Any]], evidence_samples: dict[str, Any], instrumentation: dict[str, Any], process_record: dict[str, Any], dispatch_started_wall_time_sec: float | None, observation_timed_out: bool) -> dict[str, Any]:
    record = initial_instrumented_first_goal_record(max_goals=1)
    record['maze_explorer_start_allowed'] = True
    record['maze_explorer_started'] = True
    record['process'].update(process_record)
    record['goal_events'] = goal_events[-120:]
    record['explorer_states'] = explorer_states[-120:]
    record['goal_event_count'] = len(goal_events)
    dispatch_events = [event for event in goal_events if str(event.get('event', '')).lower() == 'dispatch']
    record['dispatch_event_count'] = len(dispatch_events)
    record['second_goal_dispatched'] = len(dispatch_events) > 1
    record['exploration_goal_dispatched'] = bool(dispatch_events)
    record['observation_timed_out'] = bool(observation_timed_out)
    record['local_costmap_samples'] = _list(evidence_samples.get('local_costmap_samples'))[-40:]
    record['global_costmap_samples'] = _list(evidence_samples.get('global_costmap_samples'))[-40:]
    record['scan'] = _dict(evidence_samples.get('scan'))
    record['map'] = _dict(evidence_samples.get('map'))
    record['odom'] = _dict(evidence_samples.get('odom'))
    record['tf'] = _dict(evidence_samples.get('tf'))
    record['nav2_feedback'] = _list(evidence_samples.get('nav2_feedback'))[-160:]
    record['action_status_samples'] = action_status_samples[-160:]
    record['costmap_freshness'] = _costmap_freshness(record['local_costmap_samples'], record['global_costmap_samples'])
    record['scan_freshness'] = _freshness_from_sample(record['scan'], max_age_sec=2.0, required_seen_key='seen')
    record['tf_freshness'] = {'fresh': _bool(record['tf'].get('fresh')), 'received_wall_time_sec': record['tf'].get('received_wall_time_sec')}

    if not dispatch_events:
        record['instrumentation'] = instrumentation
        record['instrumentation']['evidence_coverage'] = _instrumentation_coverage(instrumentation)
        record['instrumentation']['evidence_gaps'] = _evidence_gaps(instrumentation)
        record['stop_reason'] = 'no_valid_first_goal_fail_closed'
        return record

    dispatch = dispatch_events[0]
    goal_kind = dispatch.get('goal_kind')
    pose = _goal_pose_from_event(dispatch)
    dispatch_wall = dispatch.get('received_wall_time_sec', dispatch_started_wall_time_sec)
    result_event = _latest_result_event(dispatch, goal_events)
    non_explore = goal_kind != 'explore'
    accepted = False
    rejected = False
    timeout = False
    status_label: str | None = None
    abort_text: str | None = None
    stop_reason: str | None = None

    if non_explore:
        rejected = True
        status_label = 'REJECTED_NON_EXPLORE_GOAL_KIND'
        abort_text = f'first dispatch goal_kind={goal_kind!r} is not explore'
        stop_reason = 'first_dispatch_not_explore_diagnostic_stop'
    elif record['second_goal_dispatched']:
        rejected = True
        status_label = 'REJECTED_SECOND_GOAL_DISPATCHED'
        abort_text = 'second exploration goal dispatch observed inside Phase129 bounded first-goal diagnosis'
        stop_reason = 'second_goal_dispatched_diagnostic_stop'
    elif result_event is not None:
        raw_event = str(result_event.get('event', '')).lower()
        status_label = _result_status_from_event(result_event)
        abort_text = result_event.get('result_reason')
        if raw_event == 'success' or status_label == 'SUCCEEDED':
            accepted = True
            status_label = 'SUCCEEDED'
            stop_reason = 'first_explore_goal_result_succeeded_with_instrumentation_stop'
        elif raw_event == 'timeout' or status_label == 'TIMEOUT':
            accepted = True
            timeout = True
            status_label = 'TIMEOUT'
            abort_text = abort_text or 'bounded_observation_timeout_after_first_dispatch'
            stop_reason = 'first_explore_goal_timeout_instrumented_stop'
        elif status_label == 'CANCELED':
            accepted = True
            rejected = True
            stop_reason = 'first_explore_goal_result_canceled_stop'
        elif status_label == 'ABORTED':
            accepted = True
            rejected = True
            stop_reason = 'first_explore_goal_result_aborted_stop'
        else:
            accepted = True
            rejected = True
            status_label = status_label or 'REJECTED_OR_FAILED'
            abort_text = abort_text or 'first explore goal failed or was rejected'
            stop_reason = 'first_explore_goal_rejected_or_failed_stop'
    elif observation_timed_out:
        accepted = True
        timeout = True
        status_label = 'TIMEOUT'
        abort_text = 'bounded_observation_timeout_waiting_for_first_goal_result'
        stop_reason = 'first_explore_goal_timeout_instrumented_stop'
    else:
        rejected = True
        status_label = 'REJECTED_OR_NOT_CONFIRMED'
        abort_text = 'first explore dispatch observed but no terminal result was confirmed inside bounded window'
        stop_reason = 'first_explore_goal_result_not_confirmed_stop'

    robot_pose = _robot_pose_from_tf(record['tf'])
    distance = _distance_to_goal(robot_pose, pose)
    first = {
        'candidate': _candidate_from_event(dispatch),
        'pose': pose,
        'frame_id': pose.get('frame_id'),
        'goal_kind': goal_kind,
        'selection_reason': dispatch.get('selected_due_to_context') or dispatch.get('centerline_refinement_reason') or dispatch.get('local_topology') or 'maze_explorer_current_algorithm_first_dispatch',
        'frontier_evidence': _frontier_evidence_from_event(dispatch),
        'topology_evidence': _topology_evidence_from_event(dispatch),
        'dispatch_wall_time_sec': dispatch_wall,
        'accepted': bool(accepted),
        'rejected': bool(rejected),
        'nav2_feedback_timeline': record['nav2_feedback'][-120:],
        'result_status_label': status_label,
        'abort_text': abort_text,
        'timeout': bool(timeout),
        'robot_pose_at_result': robot_pose,
        'distance_to_first_goal': distance,
    }
    record['first_goal'] = first
    instrumentation['first_goal_candidate_risk'] = _build_candidate_risk(first)
    instrumentation['evidence_coverage'] = _instrumentation_coverage(instrumentation)
    instrumentation['evidence_gaps'] = _evidence_gaps(instrumentation)
    record['instrumentation'] = instrumentation
    record['stop_reason'] = stop_reason
    return record


def _velocity_summary(cmd: list[dict[str, Any]], odom_vel: list[dict[str, Any]]) -> dict[str, Any]:
    cmd_nonzero = sum(1 for s in cmd if abs(_float(s.get('linear_x'), 0.0) or 0.0) > 0.03 or abs(_float(s.get('angular_z'), 0.0) or 0.0) > 0.08)
    odom_zero = sum(1 for s in odom_vel if abs(_float(s.get('linear_x'), 0.0) or 0.0) < 0.025 and abs(_float(s.get('angular_z'), 0.0) or 0.0) < 0.08)
    mismatch = min(cmd_nonzero, odom_zero) if cmd and odom_vel else 0
    oscillation = 0
    last_sign = 0
    for s in cmd:
        value = (_float(s.get('angular_z'), 0.0) or 0.0) + (_float(s.get('linear_x'), 0.0) or 0.0)
        sign = 1 if value > 0.05 else (-1 if value < -0.05 else 0)
        if sign and last_sign and sign != last_sign:
            oscillation += 1
        if sign:
            last_sign = sign
    return {
        'available': bool(cmd or odom_vel),
        'cmd_sample_count': len(cmd),
        'odom_velocity_sample_count': len(odom_vel),
        'cmd_nonzero_count': cmd_nonzero,
        'odom_near_zero_count': odom_zero,
        'command_actual_mismatch_count': mismatch,
        'oscillation_count': oscillation,
    }


def _line_has_recovery(text: str) -> bool:
    lower = text.lower()
    return 'recover' in lower or 'recovery' in lower or 'spin' in lower or 'backup' in lower or 'wait' in lower


def run_maze_explorer_instrumented_first_goal(*, duration_sec: float, stdout_path: Path, stderr_path: Path) -> dict[str, Any]:  # pragma: no cover - ROS runtime
    import rclpy
    import tf2_ros
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
    from rcl_interfaces.msg import Log
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    try:
        from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
    except Exception:  # noqa: BLE001
        NavigateToPose_FeedbackMessage = None

    command = [
        'ros2', 'run', 'tugbot_maze', 'maze_explorer',
        '--ros-args', '-p', 'use_sim_time:=true', '-p', 'max_goals:=1', '-p', 'exploration_rate_hz:=1.0'
    ]
    record = initial_instrumented_first_goal_record(max_goals=1)
    record['maze_explorer_start_allowed'] = True
    record['process']['command'] = command
    stdout_path.parent.mkdir(parents=True, exist_ok=True)
    stderr_path.parent.mkdir(parents=True, exist_ok=True)

    events: list[dict[str, Any]] = []
    states: list[dict[str, Any]] = []
    action_status_samples: list[dict[str, Any]] = []
    nav2_feedback: list[dict[str, Any]] = []
    local_costmap_samples: list[dict[str, Any]] = []
    global_costmap_samples: list[dict[str, Any]] = []
    evidence: dict[str, Any] = {'scan': {}, 'map': {}, 'odom': {}, 'tf': {}}
    instrumentation: dict[str, Any] = {field: [] for field in INSTRUMENTATION_FIELDS}
    instrumentation['goal_checker_state'] = {'available': False, 'reason': 'goal checker runtime state topic/parameter API not directly available in Phase129 runner'}
    instrumentation['commanded_vs_measured_velocity'] = {'available': False}
    instrumentation['first_goal_candidate_risk'] = {'available': False}
    instrumentation['evidence_coverage'] = {}
    instrumentation['evidence_gaps'] = []
    latest_goal_pose: dict[str, Any] = {}
    latest_path: dict[str, Any] | None = None
    proc: subprocess.Popen[str] | None = None
    first_dispatch_wall: float | None = None
    terminal_seen = False
    second_dispatch_seen = False
    last_recovery_count = 0

    def _event_cb(msg: String) -> None:
        nonlocal first_dispatch_wall, terminal_seen, second_dispatch_seen, latest_goal_pose
        try:
            parsed = json.loads(msg.data)
            payload = parsed if isinstance(parsed, dict) else {'raw': msg.data}
        except Exception:
            payload = {'raw': msg.data}
        payload['received_wall_time_sec'] = _now_wall()
        events.append(payload)
        if str(payload.get('event', '')).lower() == 'dispatch':
            if first_dispatch_wall is None:
                first_dispatch_wall = payload['received_wall_time_sec']
                latest_goal_pose = _goal_pose_from_event(payload)
            else:
                second_dispatch_seen = True
        if first_dispatch_wall is not None and str(payload.get('event', '')).lower() in {'success', 'failure', 'timeout', 'stale_result', 'terminal_cancel_result', 'timeout_cancel_result'}:
            terminal_seen = True

    def _state_cb(msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
            payload = parsed if isinstance(parsed, dict) else {'raw': msg.data}
        except Exception:
            payload = {'raw': msg.data}
        payload['received_wall_time_sec'] = _now_wall()
        states.append(payload)
        del states[:-160]

    def _scan_cb(msg: LaserScan) -> None:
        evidence['scan'] = {
            'seen': True,
            'frame_id': msg.header.frame_id,
            'stamp': _stamp_dict(msg.header.stamp),
            'received_wall_time_sec': _now_wall(),
            'finite_count': sum(1 for value in msg.ranges if math.isfinite(value)),
        }

    def _map_cb(msg: OccupancyGrid) -> None:
        evidence['map'] = {'seen': True, 'frame_id': msg.header.frame_id, 'stamp': _stamp_dict(msg.header.stamp), 'received_wall_time_sec': _now_wall(), 'width': int(msg.info.width), 'height': int(msg.info.height)}

    def _local_cb(msg: OccupancyGrid) -> None:
        sample = {'seen': True, 'frame_id': msg.header.frame_id, 'stamp': _stamp_dict(msg.header.stamp), 'received_wall_time_sec': _now_wall(), 'width': int(msg.info.width), 'height': int(msg.info.height), 'resolution': float(msg.info.resolution)}
        local_costmap_samples.append(sample)
        del local_costmap_samples[:-80]
        local_window = {'sample_wall_time_sec': sample['received_wall_time_sec'], 'frame_id': msg.header.frame_id, 'width': int(msg.info.width), 'height': int(msg.info.height), 'resolution': float(msg.info.resolution)}
        local_pose = _runtime_tf_pose(tf_buffer, node, target_frame=msg.header.frame_id, source_frame='base_link')
        goal_xy = _transform_map_point_to_frame(tf_buffer, node, latest_goal_pose, msg.header.frame_id) if latest_goal_pose else (None, None)
        if local_pose.get('fresh'):
            snap = _cost_snapshot_from_local_costmap(msg, robot_frame_pose=local_pose, goal_frame_xy=goal_xy, latest_path=latest_path)
            instrumentation['footprint_front_path_cost_snapshots'].append(snap)
            del instrumentation['footprint_front_path_cost_snapshots'][:-80]
            local_window.update({'robot_grid': snap.get('robot_grid'), 'target_grid': snap.get('target_grid'), 'footprint': snap.get('footprint'), 'target_window': snap.get('target_window')})
        instrumentation['local_costmap_windows'].append(local_window)
        del instrumentation['local_costmap_windows'][:-80]

    def _global_cb(msg: OccupancyGrid) -> None:
        sample = {'seen': True, 'frame_id': msg.header.frame_id, 'stamp': _stamp_dict(msg.header.stamp), 'received_wall_time_sec': _now_wall(), 'width': int(msg.info.width), 'height': int(msg.info.height), 'resolution': float(msg.info.resolution)}
        global_costmap_samples.append(sample)
        del global_costmap_samples[:-80]
        global_window = {'sample_wall_time_sec': sample['received_wall_time_sec'], 'frame_id': msg.header.frame_id, 'width': int(msg.info.width), 'height': int(msg.info.height), 'resolution': float(msg.info.resolution)}
        pose = _runtime_tf_pose(tf_buffer, node, target_frame=msg.header.frame_id, source_frame='base_link')
        goal_xy = _transform_map_point_to_frame(tf_buffer, node, latest_goal_pose, msg.header.frame_id) if latest_goal_pose else (None, None)
        if pose.get('fresh'):
            rgx, rgy, rok = _project_to_grid(msg, _float(pose.get('x')), _float(pose.get('y')))
            tgx, tgy, tok = _project_to_grid(msg, goal_xy[0], goal_xy[1])
            global_window.update({'robot_grid': {'x': rgx, 'y': rgy, 'in_bounds': rok}, 'target_grid': {'x': tgx, 'y': tgy, 'in_bounds': tok}, 'robot_window': _window_stats(msg, rgx, rgy), 'target_window': _window_stats(msg, tgx, tgy)})
        instrumentation['global_costmap_windows'].append(global_window)
        del instrumentation['global_costmap_windows'][:-80]

    def _odom_cb(msg: Odometry) -> None:
        yaw = _quat_to_yaw(msg.pose.pose.orientation)
        evidence['odom'] = {
            'seen': True,
            'frame_id': msg.header.frame_id,
            'child_frame_id': msg.child_frame_id,
            'stamp': _stamp_dict(msg.header.stamp),
            'received_wall_time_sec': _now_wall(),
            'pose': {'x': float(msg.pose.pose.position.x), 'y': float(msg.pose.pose.position.y), 'z': float(msg.pose.pose.position.z), 'yaw': yaw},
        }
        instrumentation['odom_velocity_timeline'].append({
            'sample_wall_time_sec': _now_wall(),
            'frame_id': msg.header.frame_id,
            'child_frame_id': msg.child_frame_id,
            'stamp': _stamp_dict(msg.header.stamp),
            'linear_x': float(msg.twist.twist.linear.x),
            'linear_y': float(msg.twist.twist.linear.y),
            'angular_z': float(msg.twist.twist.angular.z),
            'speed': math.hypot(float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y)),
        })
        del instrumentation['odom_velocity_timeline'][:-240]

    def _cmd_cb(msg: Twist) -> None:
        instrumentation['cmd_vel_timeline'].append({
            'sample_wall_time_sec': _now_wall(),
            'linear_x': float(msg.linear.x),
            'linear_y': float(msg.linear.y),
            'angular_z': float(msg.angular.z),
            'command_magnitude': abs(float(msg.linear.x)) + abs(float(msg.angular.z)),
        })
        del instrumentation['cmd_vel_timeline'][:-240]

    def _status_cb(msg: GoalStatusArray) -> None:
        statuses = [int(status.status) for status in msg.status_list]
        sample = {
            'received_wall_time_sec': _now_wall(),
            'statuses': statuses,
            'active_goal_count': sum(1 for status in statuses if status in (1, 2, 3)),
            'pending_goal_count': sum(1 for status in statuses if status == 1),
            'executing_goal_count': sum(1 for status in statuses if status == 2),
            'canceling_goal_count': sum(1 for status in statuses if status == 3),
            'succeeded_goal_count': sum(1 for status in statuses if status == 4),
            'canceled_goal_count': sum(1 for status in statuses if status == 5),
            'aborted_goal_count': sum(1 for status in statuses if status == 6),
        }
        action_status_samples.append(sample)
        del action_status_samples[:-160]

    def _feedback_cb(msg: Any) -> None:
        nonlocal last_recovery_count
        feedback = getattr(msg, 'feedback', None)
        if feedback is None:
            return
        recoveries = _int(getattr(feedback, 'number_of_recoveries', 0))
        sample = {
            'received_wall_time_sec': _now_wall(),
            'navigation_time_sec': _duration_msg_to_sec(getattr(feedback, 'navigation_time', None)),
            'estimated_time_remaining_sec': _duration_msg_to_sec(getattr(feedback, 'estimated_time_remaining', None)),
            'distance_remaining': _float(getattr(feedback, 'distance_remaining', None)),
            'number_of_recoveries': recoveries,
        }
        nav2_feedback.append(sample)
        del nav2_feedback[:-240]
        if recoveries > last_recovery_count:
            instrumentation['bt_recovery_events'].append({'sample_wall_time_sec': sample['received_wall_time_sec'], 'source': 'nav2_feedback_number_of_recoveries_increment', 'from': last_recovery_count, 'to': recoveries})
            del instrumentation['bt_recovery_events'][:-80]
            last_recovery_count = recoveries

    def _path_cb(msg: NavPath) -> None:
        nonlocal latest_path
        points = [{'x': float(p.pose.position.x), 'y': float(p.pose.position.y)} for p in msg.poses[:80]]
        length = 0.0
        for a, b in zip(points, points[1:]):
            length += math.hypot(b['x'] - a['x'], b['y'] - a['y'])
        latest_path = {'sample_wall_time_sec': _now_wall(), 'frame_id': msg.header.frame_id, 'stamp': _stamp_dict(msg.header.stamp), 'point_count': len(msg.poses), 'sampled_points': points[:20], 'sampled_length_m': length}
        instrumentation['planned_path_snapshots'].append(latest_path)
        del instrumentation['planned_path_snapshots'][:-80]

    def _rosout_cb(msg: Log) -> None:
        line = {'sample_wall_time_sec': _now_wall(), 'stamp': _stamp_dict(msg.stamp), 'level': int(msg.level), 'name': msg.name, 'msg': msg.msg}
        lower_name = msg.name.lower()
        lower_msg = msg.msg.lower()
        if 'controller_server' in lower_name or 'controller' in lower_msg or 'progress checker' in lower_msg or 'goal checker' in lower_msg:
            instrumentation['controller_server_logs'].append(line)
            del instrumentation['controller_server_logs'][:-120]
        if 'bt_navigator' in lower_name or 'bt navigator' in lower_msg or 'behavior tree' in lower_msg or 'recovery' in lower_msg:
            instrumentation['bt_navigator_logs'].append(line)
            del instrumentation['bt_navigator_logs'][:-120]
        if _line_has_recovery(msg.msg):
            instrumentation['bt_recovery_events'].append({'sample_wall_time_sec': line['sample_wall_time_sec'], 'source': 'rosout', 'node': msg.name, 'msg': msg.msg})
            del instrumentation['bt_recovery_events'][:-120]

    rclpy.init(args=None)
    node = rclpy.create_node('phase129_instrumented_first_goal_diagnosis_monitor')
    node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    costmap_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
    subs: list[Any] = [
        node.create_subscription(String, GOAL_EVENTS_TOPIC, _event_cb, 10),
        node.create_subscription(String, EXPLORER_STATE_TOPIC, _state_cb, 10),
        node.create_subscription(GoalStatusArray, f'{ACTION_NAME}/_action/status', _status_cb, costmap_qos),
        node.create_subscription(OccupancyGrid, '/map', _map_cb, costmap_qos),
        node.create_subscription(OccupancyGrid, '/local_costmap/costmap', _local_cb, costmap_qos),
        node.create_subscription(OccupancyGrid, '/global_costmap/costmap', _global_cb, costmap_qos),
        node.create_subscription(LaserScan, '/scan', _scan_cb, qos_profile_sensor_data),
        node.create_subscription(Odometry, '/odom', _odom_cb, qos_profile_sensor_data),
        node.create_subscription(Twist, '/cmd_vel', _cmd_cb, 10),
        node.create_subscription(NavPath, '/plan', _path_cb, 10),
        node.create_subscription(Log, '/rosout', _rosout_cb, 100),
    ]
    if NavigateToPose_FeedbackMessage is not None:
        subs.append(node.create_subscription(NavigateToPose_FeedbackMessage, f'{ACTION_NAME}/_action/feedback', _feedback_cb, 10))

    start = _now_wall()
    observation_timed_out = False
    try:
        proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=os.environ.copy(), start_new_session=True)
        record['maze_explorer_started'] = True
        record['process']['pid'] = proc.pid
        deadline = start + float(duration_sec)
        while _now_wall() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            tf_map = _runtime_tf_pose(tf_buffer, node, target_frame='map', source_frame='base_link')
            evidence['tf'] = tf_map
            if tf_map.get('fresh'):
                pose_sample = {'sample_wall_time_sec': tf_map.get('received_wall_time_sec'), 'frame_id': 'map', 'x': tf_map.get('x'), 'y': tf_map.get('y'), 'yaw': tf_map.get('yaw')}
                instrumentation['robot_pose_trace'].append(pose_sample)
                del instrumentation['robot_pose_trace'][:-240]
                if latest_goal_pose:
                    dist = _distance_to_goal({'x': tf_map.get('x'), 'y': tf_map.get('y')}, latest_goal_pose)
                    instrumentation['distance_error_curve'].append({'sample_wall_time_sec': tf_map.get('received_wall_time_sec'), 'distance_error_m': dist.get('meters')})
                    yaw_error = _angle_diff(_float(latest_goal_pose.get('yaw')), _float(tf_map.get('yaw')))
                    instrumentation['yaw_error_curve'].append({'sample_wall_time_sec': tf_map.get('received_wall_time_sec'), 'yaw_error_rad': yaw_error})
                    del instrumentation['distance_error_curve'][:-240]
                    del instrumentation['yaw_error_curve'][:-240]
            instrumentation['commanded_vs_measured_velocity'] = _velocity_summary(_list(instrumentation.get('cmd_vel_timeline')), _list(instrumentation.get('odom_velocity_timeline')))
            dispatch_events = [event for event in events if str(event.get('event', '')).lower() == 'dispatch']
            first_event = dispatch_events[0] if dispatch_events else None
            if second_dispatch_seen or len(dispatch_events) > 1:
                break
            if first_event is not None and first_event.get('goal_kind') != 'explore':
                break
            if terminal_seen:
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.1)
                    evidence['tf'] = _runtime_tf_pose(tf_buffer, node, target_frame='map', source_frame='base_link')
                break
            if proc.poll() is not None:
                for _ in range(5):
                    rclpy.spin_once(node, timeout_sec=0.1)
                break
        else:
            observation_timed_out = True
    except Exception as exc:  # noqa: BLE001
        record['process']['exception_text'] = str(exc)
        record['process']['traceback_tail'] = traceback.format_exc()[-2000:]
    finally:
        try:
            for sub in subs:
                node.destroy_subscription(sub)
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        stdout = ''
        stderr = ''
        if proc is not None:
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except Exception:
                    proc.terminate()
                try:
                    stdout, stderr = proc.communicate(timeout=5.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except Exception:
                        proc.kill()
                    stdout, stderr = proc.communicate(timeout=5.0)
            else:
                stdout, stderr = proc.communicate(timeout=5.0)
            record['process']['returncode'] = proc.returncode
        stdout_path.write_text(stdout or '', encoding='utf-8')
        stderr_path.write_text(stderr or '', encoding='utf-8')
        record['process']['stdout_path'] = str(stdout_path)
        record['process']['stderr_path'] = str(stderr_path)

    evidence['local_costmap_samples'] = local_costmap_samples
    evidence['global_costmap_samples'] = global_costmap_samples
    evidence['nav2_feedback'] = nav2_feedback
    evidence['action_status_samples'] = action_status_samples
    instrumentation['commanded_vs_measured_velocity'] = _velocity_summary(_list(instrumentation.get('cmd_vel_timeline')), _list(instrumentation.get('odom_velocity_timeline')))
    return _finalize_record_from_observation(
        goal_events=events,
        explorer_states=states,
        action_status_samples=action_status_samples,
        evidence_samples=evidence,
        instrumentation=instrumentation,
        process_record=_dict(record.get('process')),
        dispatch_started_wall_time_sec=first_dispatch_wall,
        observation_timed_out=observation_timed_out,
    )


def _distance_plateau(first: dict[str, Any], instr: dict[str, Any]) -> bool:
    curve = _list(instr.get('distance_error_curve'))
    values = [_float(s.get('distance_error_m')) for s in curve[-10:] if _float(s.get('distance_error_m')) is not None]
    if len(values) >= 2 and max(values) - min(values) < 0.05:
        return True
    feedback = _list(first.get('nav2_feedback_timeline'))
    d = [_float(s.get('distance_remaining')) for s in feedback[-10:] if _float(s.get('distance_remaining')) is not None]
    return bool(len(d) >= 2 and max(d) - min(d) < 0.05)


def _local_cost_support(instr: dict[str, Any], first: dict[str, Any]) -> dict[str, Any]:
    snapshots = _list(instr.get('footprint_front_path_cost_snapshots'))
    high = []
    for s in snapshots:
        footprint = _float(_dict(s).get('footprint_max'))
        front = _float(_dict(s).get('front_max'))
        p05 = _float(_dict(s).get('path_0_5m_max'))
        p10 = _float(_dict(s).get('path_1_0m_max'))
        if ((footprint is not None and footprint >= 90) or (front is not None and front >= 90)) and ((p05 is not None and p05 >= 70) or (p10 is not None and p10 >= 70)):
            high.append(s)
    plateau = _distance_plateau(first, instr)
    return {
        'supports': bool(high and plateau),
        'high_cost_sample_count': len(high),
        'snapshot_count': len(snapshots),
        'distance_plateau': plateau,
    }


def _controller_support(instr: dict[str, Any], local_supports: bool, first: dict[str, Any]) -> dict[str, Any]:
    vel = _dict(instr.get('commanded_vs_measured_velocity'))
    mismatch = _int(vel.get('command_actual_mismatch_count'))
    oscillation = _int(vel.get('oscillation_count'))
    cmd_nonzero = _int(vel.get('cmd_nonzero_count'))
    plateau = _distance_plateau(first, instr)
    supports = bool((mismatch >= 2 or oscillation >= 2 or cmd_nonzero >= 3) and plateau and not local_supports)
    return {'supports': supports, 'mismatch_count': mismatch, 'oscillation_count': oscillation, 'cmd_nonzero_count': cmd_nonzero, 'distance_plateau': plateau, 'suppressed_by_local_cost': bool(local_supports)}


def _tolerance_support(instr: dict[str, Any], local_supports: bool, controller_supports: bool) -> dict[str, Any]:
    state = _dict(instr.get('goal_checker_state'))
    curve = _list(instr.get('distance_error_curve'))
    yaw_curve = _list(instr.get('yaw_error_curve'))
    distance = _float(state.get('distance_error_m'))
    xy_tol = _float(state.get('xy_tolerance_m'))
    yaw_err = _float(state.get('yaw_error_rad'))
    yaw_tol = _float(state.get('yaw_tolerance_rad'))
    near_xy = bool(distance is not None and xy_tol is not None and distance <= xy_tol + 0.15)
    yaw_out = bool(yaw_err is not None and yaw_tol is not None and abs(yaw_err) > yaw_tol)
    supports = bool(state.get('available') and (near_xy or yaw_out) and curve and yaw_curve and not local_supports and not controller_supports)
    return {'supports': supports, 'goal_checker_available': bool(state.get('available')), 'near_xy_tolerance': near_xy, 'yaw_outside_tolerance': yaw_out, 'pose_trace_count': len(_list(instr.get('robot_pose_trace')))}


def _candidate_support(instr: dict[str, Any], local_supports: bool, controller_supports: bool, tolerance_supports: bool) -> dict[str, Any]:
    risk = _dict(instr.get('first_goal_candidate_risk'))
    clearance = _float(risk.get('target_clearance_m'), _float(risk.get('clearance')))
    max_radius = _float(risk.get('target_local_cost_max_radius'))
    corridor = _float(risk.get('path_corridor_min_clearance_m'))
    risky = bool((clearance is not None and clearance < 0.30) or (max_radius is not None and max_radius >= 90) or (corridor is not None and corridor < 0.30) or risk.get('selected_branch_riskier_than_alternatives') or risk.get('near_wall') or risk.get('near_high_cost_band'))
    supports = bool(risk.get('available') and risky and not local_supports and not controller_supports and not tolerance_supports)
    return {'supports': supports, 'available': bool(risk.get('available')), 'risky': risky, 'target_clearance_m': clearance, 'target_local_cost_max_radius': max_radius, 'path_corridor_min_clearance_m': corridor}


def analyze_instrumented_evidence(artifact: dict[str, Any]) -> dict[str, Any]:
    first = _dict(artifact.get('first_goal')) or _dict(_dict(artifact.get('first_goal_result_artifact')).get('first_goal'))
    instr = _dict(artifact.get('instrumentation')) or _dict(_dict(artifact.get('first_goal_result_artifact')).get('instrumentation'))
    status = first.get('result_status_label')
    timeout = _bool(first.get('timeout')) or status == 'TIMEOUT'
    if status == 'SUCCEEDED' and _bool(first.get('accepted')) and not _bool(first.get('rejected')):
        classification = FIRST_GOAL_RESULT_SUCCEEDED_WITH_INSTRUMENTATION_STOP
    elif timeout:
        local = _local_cost_support(instr, first)
        controller = _controller_support(instr, local['supports'], first)
        tolerance = _tolerance_support(instr, local['supports'], controller['supports'])
        candidate = _candidate_support(instr, local['supports'], controller['supports'], tolerance['supports'])
        if local['supports']:
            classification = FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED
        elif controller['supports']:
            classification = FIRST_GOAL_TIMEOUT_CONTROLLER_STALL
        elif tolerance['supports']:
            classification = FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE
        elif candidate['supports']:
            classification = FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY
        else:
            classification = INSUFFICIENT_TIMEOUT_EVIDENCE
        return {
            'classification': classification,
            'evidence': {
                'local_cost_blocked': local,
                'controller_stall': controller,
                'goal_tolerance_edge': tolerance,
                'candidate_too_risky': candidate,
            },
            'coverage': _instrumentation_coverage(instr),
            'evidence_gaps': _evidence_gaps(instr),
        }
    else:
        classification = INSUFFICIENT_TIMEOUT_EVIDENCE
    return {
        'classification': classification,
        'evidence': {
            'local_cost_blocked': _local_cost_support(instr, first),
            'controller_stall': _controller_support(instr, False, first),
            'goal_tolerance_edge': _tolerance_support(instr, False, False),
            'candidate_too_risky': _candidate_support(instr, False, False, False),
        },
        'coverage': _instrumentation_coverage(instr),
        'evidence_gaps': _evidence_gaps(instr),
    }


def classify_phase129(first_goal_record: dict[str, Any]) -> str:
    first = _dict(first_goal_record.get('first_goal'))
    if first.get('result_status_label') == 'SUCCEEDED' and _bool(first.get('accepted')) and not _bool(first.get('rejected')):
        return FIRST_GOAL_RESULT_SUCCEEDED_WITH_INSTRUMENTATION_STOP
    if _bool(first.get('timeout')) or first.get('result_status_label') == 'TIMEOUT':
        return analyze_instrumented_evidence({'first_goal': first, 'instrumentation': _dict(first_goal_record.get('instrumentation'))})['classification']
    return INSUFFICIENT_TIMEOUT_EVIDENCE


def build_phase129_artifact(*, run_id: str, phase120_artifact: dict[str, Any], handoff_artifact: dict[str, Any], first_goal_record: dict[str, Any], classification: str) -> dict[str, Any]:
    dispatch_count = _int(first_goal_record.get('dispatch_event_count'))
    first = _dict(first_goal_record.get('first_goal'))
    instrumentation = _dict(first_goal_record.get('instrumentation'))
    derived = analyze_instrumented_evidence({'first_goal': first, 'instrumentation': instrumentation})
    final_classification = classification if classification == FIRST_GOAL_RESULT_SUCCEEDED_WITH_INSTRUMENTATION_STOP else derived['classification']
    return {
        'phase': PHASE,
        'run_id': run_id,
        'mode': MODE,
        'created_wall_time_sec': _now_wall(),
        'phase120_source_artifact': phase120_artifact.get('artifact_path') or phase120_artifact.get('run_id'),
        'phase120_ingress_artifact': phase120_artifact,
        'handoff_artifact': handoff_artifact,
        'handoff_allowed': bool(_p125.handoff_ready(handoff_artifact)),
        'first_goal_result_artifact': first_goal_record,
        'classification': final_classification,
        'raw_terminal_classification': classification,
        'maze_explorer_start_allowed': _bool(first_goal_record.get('maze_explorer_start_allowed')),
        'maze_explorer_started': _bool(first_goal_record.get('maze_explorer_started')),
        'maze_explorer_max_goals': _int(first_goal_record.get('maze_explorer_max_goals'), -1),
        'exploration_goal_dispatched': _bool(first_goal_record.get('exploration_goal_dispatched')),
        'goal_event_count': _int(first_goal_record.get('goal_event_count')),
        'dispatch_event_count': dispatch_count,
        'second_goal_dispatched': _bool(first_goal_record.get('second_goal_dispatched')),
        'first_goal': first,
        'instrumentation': instrumentation,
        'diagnostic_analysis': derived,
        'stop_reason': first_goal_record.get('stop_reason'),
        'guardrails': {
            'handoff_ready_required': True,
            'same_run_phase120_ingress_required': True,
            'only_explicit_inner_ingress_goal_sent': _phase122.phase120_ingress_success(phase120_artifact) if phase120_artifact else True,
            'max_goals_one': _int(first_goal_record.get('maze_explorer_max_goals'), -1) == 1,
            'first_exploration_goal_only': dispatch_count <= 1,
            'second_goal_dispatched_false': not _bool(first_goal_record.get('second_goal_dispatched')),
            'manual_goal1_forbidden': True,
            'no_goal1_carry_over_staging_branch_centerline_fallback_terminal_exit_manual_goal': True,
            'branch_scoring_changed': False,
            'centerline_gate_changed': False,
            'fallback_changed': False,
            'terminal_acceptance_changed': False,
            'nav2_config_changed': False,
            'autonomous_success_claimed': False,
            'exit_success_claimed': False,
            'no_autonomous_success_claim': True,
            'no_exit_success_claim': True,
        },
        'forbidden_actions': dict(FORBIDDEN_ACTIONS),
        'notes': [
            'Phase129 is instrumentation-only first-goal diagnosis.',
            'A succeeded first-goal result is not autonomous exploration success and not exit success.',
            'Timeout classifications remain diagnostic and do not authorize tuning or repair.',
        ],
    }


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def run_phase120_ingress_chain(args: argparse.Namespace) -> dict[str, Any]:
    phase120_artifact = _p125.run_phase120_ingress_chain(args)
    if args.phase120_output:
        phase120_artifact['artifact_path'] = str(args.phase120_output)
    return phase120_artifact


def run_phase129(args: argparse.Namespace) -> dict[str, Any]:
    phase120_artifact = run_phase120_ingress_chain(args)
    handoff = _phase122.collect_handoff_runtime_evidence(
        phase120_artifact,
        collection_sec=args.handoff_collection_sec,
        pose_tolerance_m=args.pose_tolerance_m,
        yaw_tolerance_rad=args.yaw_tolerance_rad,
        max_costmap_age_sec=args.max_costmap_age_sec,
        max_scan_age_sec=args.max_scan_age_sec,
        max_tf_age_sec=args.max_tf_age_sec,
    )
    result_record = initial_instrumented_first_goal_record(max_goals=1)
    if _p125.handoff_ready(handoff):
        result_record = run_maze_explorer_instrumented_first_goal(
            duration_sec=args.first_goal_result_observation_sec,
            stdout_path=args.output.with_name(args.output.stem + '_maze_explorer_stdout.log'),
            stderr_path=args.output.with_name(args.output.stem + '_maze_explorer_stderr.log'),
        )
    classification = classify_phase129(result_record)
    artifact = build_phase129_artifact(
        run_id=args.run_id,
        phase120_artifact=phase120_artifact,
        handoff_artifact=handoff,
        first_goal_record=result_record,
        classification=classification,
    )
    write_json(args.output, artifact)
    return artifact


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--phase120-output', type=Path, required=True)
    parser.add_argument('--preflight-output', type=Path, required=True)
    parser.add_argument('--phase120-artifact', type=Path, help='test/debug only: reuse existing Phase120 artifact instead of dispatching')
    parser.add_argument('--run-id', default='phase129_instrumented_first_goal_timeout_diagnosis')
    parser.add_argument('--launch-log-path')
    parser.add_argument('--preflight-timeout-sec', type=float, default=35.0)
    parser.add_argument('--tf-stability-window-sec', type=float, default=2.0)
    parser.add_argument('--sample-period-sec', type=float, default=0.5)
    parser.add_argument('--startup-grace-sec', type=float, default=2.0)
    parser.add_argument('--action-server-wait-sec', type=float, default=10.0)
    parser.add_argument('--bounded-goal-result-wait-sec', type=float, default=45.0)
    parser.add_argument('--readiness-wait-timeout-sec', type=float, default=60.0)
    parser.add_argument('--readiness-wait-sample-period-sec', type=float, default=1.0)
    parser.add_argument('--handoff-collection-sec', type=float, default=4.0)
    parser.add_argument('--pose-tolerance-m', type=float, default=0.35)
    parser.add_argument('--yaw-tolerance-rad', type=float, default=0.35)
    parser.add_argument('--max-costmap-age-sec', type=float, default=5.0)
    parser.add_argument('--max-scan-age-sec', type=float, default=2.0)
    parser.add_argument('--max-tf-age-sec', type=float, default=1.5)
    parser.add_argument('--first-goal-result-observation-sec', type=float, default=75.0)
    args = parser.parse_args()
    artifact = run_phase129(args)
    first = _dict(artifact.get('first_goal'))
    instr = _dict(artifact.get('instrumentation'))
    print(json.dumps({
        'classification': artifact.get('classification'),
        'handoff_allowed': artifact.get('handoff_allowed'),
        'maze_explorer_started': artifact.get('maze_explorer_started'),
        'maze_explorer_max_goals': artifact.get('maze_explorer_max_goals'),
        'goal_kind': first.get('goal_kind'),
        'accepted': first.get('accepted'),
        'rejected': first.get('rejected'),
        'timeout': first.get('timeout'),
        'result_status_label': first.get('result_status_label'),
        'dispatch_event_count': artifact.get('dispatch_event_count'),
        'second_goal_dispatched': artifact.get('second_goal_dispatched'),
        'stop_reason': artifact.get('stop_reason'),
        'cmd_vel_samples': len(_list(instr.get('cmd_vel_timeline'))),
        'odom_velocity_samples': len(_list(instr.get('odom_velocity_timeline'))),
        'local_cost_snapshots': len(_list(instr.get('footprint_front_path_cost_snapshots'))),
        'artifact': str(args.output),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
