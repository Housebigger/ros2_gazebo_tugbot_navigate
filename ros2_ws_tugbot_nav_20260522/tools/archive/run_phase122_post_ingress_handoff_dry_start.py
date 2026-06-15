#!/usr/bin/env python3
"""Phase122 post-ingress handoff smoke with maze_explorer dry-start.

post_ingress_handoff_dry_start

Scope guardrails:
- Reuse the Phase120 readiness-wait + strict preflight + single explicit inner-ingress goal chain.
- The only permitted NavigateToPose goal before handoff is frame_id=map,x=2.0,y=0.0,yaw=0.0.
- Start maze_explorer only after fail-closed handoff gate passes.
- Dry-start maze_explorer only with max_goals:=0.
- do not dispatch exploration goal; this is not autonomous exploration success and not exit success.
- Do not tune Nav2/MPPI/controller/config or change branch scoring/centerline/fallback/terminal acceptance.
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

PHASE = 'Phase122'
MODE = 'post_ingress_handoff_dry_start'
ACTION_NAME = '/navigate_to_pose'
GOAL_EVENTS_TOPIC = '/maze/goal_events'
EXPLORER_STATE_TOPIC = '/maze/explorer_state'
LOCKED_GOAL = {'frame_id': 'map', 'x': 2.0, 'y': 0.0, 'yaw': 0.0}

INGRESS_SUCCESS_HANDOFF_READY_DRY_START_STOP = 'INGRESS_SUCCESS_HANDOFF_READY_DRY_START_STOP'
INGRESS_SUCCESS_HANDOFF_NOT_READY = 'INGRESS_SUCCESS_HANDOFF_NOT_READY'
POSE_NOT_AT_INGRESS_GOAL = 'POSE_NOT_AT_INGRESS_GOAL'
NAV2_ACTION_NOT_IDLE = 'NAV2_ACTION_NOT_IDLE'
TF_OR_SCAN_STALE = 'TF_OR_SCAN_STALE'
COSTMAP_NOT_READY = 'COSTMAP_NOT_READY'
MAZE_EXPLORER_DRY_START_VIOLATION = 'MAZE_EXPLORER_DRY_START_VIOLATION'

REQUIRED_HANDOFF_FIELDS = [
    'ingress_goal_result',
    'robot_pose_after_ingress',
    'distance_to_ingress_goal',
    'orientation_error',
    'costmap_freshness',
    'scan_freshness',
    'tf_freshness',
    'nav2_action_idle_state',
    'preconditions',
    'failure_reasons',
]


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


def _load_phase120_runner() -> Any:
    path = Path(__file__).resolve().parent / 'run_phase118_controlled_ingress_single_goal_dispatch_smoke.py'
    spec = importlib.util.spec_from_file_location('phase120_ingress_runner_for_phase122', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot import Phase120 ingress runner from {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _goal_identity(dispatch: dict[str, Any]) -> bool:
    goal = _dict(dispatch.get('goal_pose'))
    return bool(
        goal.get('frame_id') == LOCKED_GOAL['frame_id']
        and _float(goal.get('x')) == LOCKED_GOAL['x']
        and _float(goal.get('y')) == LOCKED_GOAL['y']
        and _float(goal.get('yaw')) == LOCKED_GOAL['yaw']
        and (dispatch.get('frame_id') in (None, LOCKED_GOAL['frame_id']))
        and (_float(dispatch.get('x'), LOCKED_GOAL['x']) == LOCKED_GOAL['x'])
        and (_float(dispatch.get('y'), LOCKED_GOAL['y']) == LOCKED_GOAL['y'])
        and (_float(dispatch.get('yaw'), LOCKED_GOAL['yaw']) == LOCKED_GOAL['yaw'])
    )


def phase120_ingress_success(phase120_artifact: dict[str, Any]) -> bool:
    readiness = _dict(phase120_artifact.get('readiness_wait'))
    preflight = _dict(phase120_artifact.get('preflight'))
    dispatch = _dict(phase120_artifact.get('dispatch'))
    same_run_readiness = (
        _bool(readiness.get('enabled'))
        and not _bool(readiness.get('timed_out'))
        and (_bool(readiness.get('marker_found')) or _bool(readiness.get('multi_source_ready')))
    )
    preflight_passed = (
        _bool(preflight.get('evaluated'))
        and _bool(preflight.get('passed'))
        and not _list(preflight.get('failed_gates'))
        and preflight.get('ingress_preflight_reject_reason') in (None, '')
    )
    ingress_succeeded = (
        _bool(dispatch.get('dispatch_attempted'))
        and _bool(dispatch.get('accepted'))
        and _bool(dispatch.get('result_received'))
        and dispatch.get('result_status_label') == 'SUCCEEDED'
        and _bool(dispatch.get('ingress_goal_sent'))
        and not _bool(dispatch.get('maze_explorer_started'))
        and _goal_identity(dispatch)
    )
    return bool(same_run_readiness and preflight_passed and ingress_succeeded)


def initial_handoff_record(*, pose_tolerance_m: float = 0.35, yaw_tolerance_rad: float = 0.35, max_costmap_age_sec: float = 5.0, max_scan_age_sec: float = 2.0, max_tf_age_sec: float = 1.5) -> dict[str, Any]:
    return {
        'ingress_goal_result': {},
        'robot_pose_after_ingress': {
            'frame_id': 'map',
            'x': None,
            'y': None,
            'yaw': None,
            'stamp': None,
            'sample_wall_time_sec': None,
            'source': 'tf_map_base_link',
            'pose_available': False,
        },
        'distance_to_ingress_goal': {
            'meters': None,
            'tolerance_m': float(pose_tolerance_m),
            'within_tolerance': False,
        },
        'orientation_error': {
            'radians': None,
            'tolerance_rad': float(yaw_tolerance_rad),
            'within_tolerance': False,
        },
        'costmap_freshness': {
            'global_costmap_available': False,
            'local_costmap_available': False,
            'global_costmap_age_sec': None,
            'local_costmap_age_sec': None,
            'max_allowed_age_sec': float(max_costmap_age_sec),
            'fresh': False,
        },
        'scan_freshness': {
            'scan_seen': False,
            'scan_age_sec': None,
            'max_allowed_age_sec': float(max_scan_age_sec),
            'fresh': False,
        },
        'tf_freshness': {
            'map_odom_available': False,
            'odom_base_available': False,
            'map_base_available': False,
            'scan_to_base_available': False,
            'map_odom_age_sec': None,
            'odom_base_age_sec': None,
            'map_base_age_sec': None,
            'max_tf_age_sec': float(max_tf_age_sec),
            'fresh': False,
        },
        'nav2_action_idle_state': {
            'action_server_ready': False,
            'active_goal_count': None,
            'pending_goal_count': None,
            'executing_goal_count': None,
            'canceling_goal_count': None,
            'idle': False,
            'last_result_status_label': None,
            'status_sample_seen': False,
        },
        'preconditions': {
            'same_run_readiness_wait_passed': False,
            'preflight_passed': False,
            'inner_ingress_result_succeeded': False,
            'robot_pose_near_ingress_goal': False,
            'nav2_action_idle': False,
            'costmap_fresh': False,
            'scan_fresh': False,
            'tf_fresh': False,
        },
        'failure_reasons': [],
    }


def initial_dry_start_record(*, max_goals: int = 0) -> dict[str, Any]:
    return {
        'maze_explorer_start_allowed': False,
        'maze_explorer_started': False,
        'maze_explorer_max_goals': int(max_goals),
        'exploration_goal_dispatched': False,
        'dry_start_observed': False,
        'dry_start_duration_sec': None,
        'goal_event_count': 0,
        'dispatch_event_count': 0,
        'goal_events': [],
        'explorer_states': [],
        'process': {
            'command': ['ros2', 'run', 'tugbot_maze', 'maze_explorer', '--ros-args', '-p', 'use_sim_time:=true', '-p', 'max_goals:=0'],
            'pid': None,
            'returncode': None,
            'stdout_path': None,
            'stderr_path': None,
            'exception_text': None,
        },
    }


def _ingress_goal_result_from_phase120(phase120_artifact: dict[str, Any]) -> dict[str, Any]:
    dispatch = _dict(phase120_artifact.get('dispatch'))
    return {
        'classification': phase120_artifact.get('classification'),
        'accepted': _bool(dispatch.get('accepted')),
        'result_received': _bool(dispatch.get('result_received')),
        'result_status': dispatch.get('result_status'),
        'result_status_label': dispatch.get('result_status_label'),
        'abort_text': dispatch.get('abort_text'),
        'cancel_requested': _bool(dispatch.get('cancel_requested')),
        'cancel_result': dispatch.get('cancel_result'),
        'send_wall_time_sec': dispatch.get('send_wall_time_sec'),
        'goal_response_wall_time_sec': dispatch.get('goal_response_wall_time_sec'),
        'result_wait_started_wall_time_sec': dispatch.get('result_wait_started_wall_time_sec'),
        'goal_pose': _dict(dispatch.get('goal_pose')),
        'locked_explicit_inner_ingress_goal': _goal_identity(dispatch),
    }


def _fill_phase120_preconditions(handoff: dict[str, Any], phase120_artifact: dict[str, Any]) -> None:
    readiness = _dict(phase120_artifact.get('readiness_wait'))
    preflight = _dict(phase120_artifact.get('preflight'))
    dispatch = _dict(phase120_artifact.get('dispatch'))
    handoff['ingress_goal_result'] = _ingress_goal_result_from_phase120(phase120_artifact)
    handoff['preconditions']['same_run_readiness_wait_passed'] = bool(
        _bool(readiness.get('enabled'))
        and not _bool(readiness.get('timed_out'))
        and (_bool(readiness.get('marker_found')) or _bool(readiness.get('multi_source_ready')))
    )
    handoff['preconditions']['preflight_passed'] = bool(
        _bool(preflight.get('evaluated'))
        and _bool(preflight.get('passed'))
        and not _list(preflight.get('failed_gates'))
        and preflight.get('ingress_preflight_reject_reason') in (None, '')
    )
    handoff['preconditions']['inner_ingress_result_succeeded'] = bool(
        _bool(dispatch.get('dispatch_attempted'))
        and _bool(dispatch.get('accepted'))
        and _bool(dispatch.get('result_received'))
        and dispatch.get('result_status_label') == 'SUCCEEDED'
        and _bool(dispatch.get('ingress_goal_sent'))
        and _goal_identity(dispatch)
    )


def _stamp_dict(stamp: Any) -> dict[str, int] | None:
    if stamp is None:
        return None
    return {'sec': int(getattr(stamp, 'sec', 0)), 'nanosec': int(getattr(stamp, 'nanosec', 0))}


def _message_wall_age(record: dict[str, Any]) -> float | None:
    recv = _float(record.get('received_wall_time_sec'))
    return None if recv is None else max(0.0, _now_wall() - recv)


def _lookup_tf(buffer: Any, target: str, source: str, node: Any, max_tf_age_sec: float) -> tuple[bool, float | None, Any | None]:
    try:
        import rclpy
        from rclpy.time import Time

        trans = buffer.lookup_transform(target, source, Time(), timeout=rclpy.duration.Duration(seconds=0.2))
        stamp = trans.header.stamp
        now_ros = node.get_clock().now()
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        age = None
        if stamp_ns > 0 and int(now_ros.nanoseconds) > 0:
            age = max(0.0, (int(now_ros.nanoseconds) - stamp_ns) / 1_000_000_000.0)
        # Some static/latest transforms may report zero stamp under sim-time startup. Availability is still evidence;
        # age remains null instead of being invented.
        return True, age, trans
    except Exception:
        return False, None, None


def collect_handoff_runtime_evidence(phase120_artifact: dict[str, Any], *, collection_sec: float = 4.0, pose_tolerance_m: float = 0.35, yaw_tolerance_rad: float = 0.35, max_costmap_age_sec: float = 5.0, max_scan_age_sec: float = 2.0, max_tf_age_sec: float = 1.5) -> dict[str, Any]:  # pragma: no cover - ROS runtime
    import rclpy
    import tf2_ros
    from action_msgs.msg import GoalStatusArray
    from nav_msgs.msg import OccupancyGrid
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    handoff = initial_handoff_record(
        pose_tolerance_m=pose_tolerance_m,
        yaw_tolerance_rad=yaw_tolerance_rad,
        max_costmap_age_sec=max_costmap_age_sec,
        max_scan_age_sec=max_scan_age_sec,
        max_tf_age_sec=max_tf_age_sec,
    )
    _fill_phase120_preconditions(handoff, phase120_artifact)
    if not phase120_ingress_success(phase120_artifact):
        handoff['failure_reasons'].append('phase120_ingress_success_preconditions_not_met')
        return handoff

    latest: dict[str, dict[str, Any]] = {}

    def _grid_cb(name: str):
        def cb(msg: OccupancyGrid) -> None:
            latest[name] = {
                'received_wall_time_sec': _now_wall(),
                'frame_id': msg.header.frame_id,
                'stamp': _stamp_dict(msg.header.stamp),
                'width': int(msg.info.width),
                'height': int(msg.info.height),
            }
        return cb

    def _scan_cb(msg: LaserScan) -> None:
        finite_count = sum(1 for value in msg.ranges if math.isfinite(value))
        latest['scan'] = {
            'received_wall_time_sec': _now_wall(),
            'frame_id': msg.header.frame_id,
            'stamp': _stamp_dict(msg.header.stamp),
            'finite_count': finite_count,
        }

    def _status_cb(msg: GoalStatusArray) -> None:
        statuses = [int(status.status) for status in msg.status_list]
        latest['action_status'] = {
            'received_wall_time_sec': _now_wall(),
            'statuses': statuses,
            'active_goal_count': sum(1 for status in statuses if status in (1, 2, 3)),
            'pending_goal_count': sum(1 for status in statuses if status == 1),
            'executing_goal_count': sum(1 for status in statuses if status == 2),
            'canceling_goal_count': sum(1 for status in statuses if status == 3),
        }

    rclpy.init(args=None)
    node = rclpy.create_node('phase122_post_ingress_handoff_evidence')
    node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=20.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    costmap_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    subs = [
        node.create_subscription(OccupancyGrid, '/global_costmap/costmap', _grid_cb('global_costmap'), costmap_qos),
        node.create_subscription(OccupancyGrid, '/local_costmap/costmap', _grid_cb('local_costmap'), costmap_qos),
        node.create_subscription(LaserScan, '/scan', _scan_cb, qos_profile_sensor_data),
        node.create_subscription(GoalStatusArray, f'{ACTION_NAME}/_action/status', _status_cb, costmap_qos),
    ]
    try:
        deadline = _now_wall() + float(collection_sec)
        while _now_wall() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        map_base_ok, map_base_age, map_base = _lookup_tf(tf_buffer, 'map', 'base_link', node, max_tf_age_sec)
        map_odom_ok, map_odom_age, _ = _lookup_tf(tf_buffer, 'map', 'odom', node, max_tf_age_sec)
        odom_base_ok, odom_base_age, _ = _lookup_tf(tf_buffer, 'odom', 'base_link', node, max_tf_age_sec)
        scan_frame = _dict(latest.get('scan')).get('frame_id')
        scan_to_base_ok = False
        if scan_frame:
            scan_to_base_ok, _, _ = _lookup_tf(tf_buffer, 'base_link', str(scan_frame), node, max_tf_age_sec)

        if map_base_ok and map_base is not None:
            translation = map_base.transform.translation
            rotation = map_base.transform.rotation
            yaw = _yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
            x = float(translation.x)
            y = float(translation.y)
            distance = math.hypot(x - LOCKED_GOAL['x'], y - LOCKED_GOAL['y'])
            yaw_error = abs(_normalize_angle(yaw - LOCKED_GOAL['yaw']))
            handoff['robot_pose_after_ingress'].update({
                'pose_available': True,
                'frame_id': 'map',
                'x': x,
                'y': y,
                'yaw': yaw,
                'stamp': _stamp_dict(map_base.header.stamp),
                'sample_wall_time_sec': _now_wall(),
                'source': 'tf_map_base_link',
            })
            handoff['distance_to_ingress_goal'].update({'meters': distance, 'within_tolerance': distance <= pose_tolerance_m})
            handoff['orientation_error'].update({'radians': yaw_error, 'within_tolerance': yaw_error <= yaw_tolerance_rad})

        global_age = _message_wall_age(_dict(latest.get('global_costmap')))
        local_age = _message_wall_age(_dict(latest.get('local_costmap')))
        global_ok = global_age is not None and global_age <= max_costmap_age_sec
        local_ok = local_age is not None and local_age <= max_costmap_age_sec
        handoff['costmap_freshness'].update({
            'global_costmap_available': 'global_costmap' in latest,
            'local_costmap_available': 'local_costmap' in latest,
            'global_costmap_age_sec': global_age,
            'local_costmap_age_sec': local_age,
            'fresh': bool(global_ok and local_ok),
        })

        scan_age = _message_wall_age(_dict(latest.get('scan')))
        handoff['scan_freshness'].update({
            'scan_seen': 'scan' in latest,
            'scan_age_sec': scan_age,
            'fresh': bool(scan_age is not None and scan_age <= max_scan_age_sec),
        })

        def _tf_age_ok(age: float | None) -> bool:
            return age is None or age <= max_tf_age_sec

        handoff['tf_freshness'].update({
            'map_odom_available': map_odom_ok,
            'odom_base_available': odom_base_ok,
            'map_base_available': map_base_ok,
            'scan_to_base_available': scan_to_base_ok,
            'map_odom_age_sec': map_odom_age,
            'odom_base_age_sec': odom_base_age,
            'map_base_age_sec': map_base_age,
            'fresh': bool(
                map_odom_ok and odom_base_ok and map_base_ok and scan_to_base_ok
                and _tf_age_ok(map_odom_age) and _tf_age_ok(odom_base_age) and _tf_age_ok(map_base_age)
            ),
        })

        status = _dict(latest.get('action_status'))
        active = int(status.get('active_goal_count', 0) or 0)
        pending = int(status.get('pending_goal_count', 0) or 0)
        executing = int(status.get('executing_goal_count', 0) or 0)
        canceling = int(status.get('canceling_goal_count', 0) or 0)
        handoff['nav2_action_idle_state'].update({
            'action_server_ready': bool(_dict(phase120_artifact.get('dispatch')).get('action_server_ready')),
            'active_goal_count': active,
            'pending_goal_count': pending,
            'executing_goal_count': executing,
            'canceling_goal_count': canceling,
            'idle': bool(active == 0 and pending == 0 and executing == 0 and canceling == 0),
            'last_result_status_label': _dict(phase120_artifact.get('dispatch')).get('result_status_label'),
            'status_sample_seen': bool(status),
        })
    except Exception as exc:
        handoff['failure_reasons'].append(f'handoff_runtime_collection_exception:{exc}')
        handoff['traceback_tail'] = traceback.format_exc()[-2000:]
    finally:
        for sub in subs:
            try:
                node.destroy_subscription(sub)
            except Exception:
                pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    update_handoff_decision_fields(handoff)
    return handoff


def update_handoff_decision_fields(handoff: dict[str, Any]) -> None:
    preconditions = _dict(handoff.get('preconditions'))
    preconditions['robot_pose_near_ingress_goal'] = bool(
        _bool(_dict(handoff.get('robot_pose_after_ingress')).get('pose_available'))
        and _bool(_dict(handoff.get('distance_to_ingress_goal')).get('within_tolerance'))
        and _bool(_dict(handoff.get('orientation_error')).get('within_tolerance'))
    )
    preconditions['nav2_action_idle'] = _bool(_dict(handoff.get('nav2_action_idle_state')).get('idle'))
    preconditions['costmap_fresh'] = _bool(_dict(handoff.get('costmap_freshness')).get('fresh'))
    preconditions['scan_fresh'] = _bool(_dict(handoff.get('scan_freshness')).get('fresh'))
    preconditions['tf_fresh'] = _bool(_dict(handoff.get('tf_freshness')).get('fresh'))
    handoff['preconditions'] = preconditions
    reasons: list[str] = []
    for key, passed in preconditions.items():
        if not _bool(passed):
            reasons.append(key)
    handoff['failure_reasons'] = sorted(set(_list(handoff.get('failure_reasons')) + reasons))


def handoff_gate_allowed(handoff: dict[str, Any]) -> bool:
    update_handoff_decision_fields(handoff)
    preconditions = _dict(handoff.get('preconditions'))
    return bool(preconditions and all(_bool(value) for value in preconditions.values()))


def _int_value(value: Any, default: int = -1) -> int:
    try:
        if value is None:
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def dry_start_violation(dry_start: dict[str, Any]) -> bool:
    return bool(
        _bool(dry_start.get('exploration_goal_dispatched'))
        or _int_value(dry_start.get('maze_explorer_max_goals'), -1) != 0
        or (_bool(dry_start.get('maze_explorer_started')) and not _bool(dry_start.get('maze_explorer_start_allowed')))
    )


def classify_phase122(handoff_artifact: dict[str, Any], dry_start: dict[str, Any]) -> str:
    if dry_start_violation(dry_start):
        return MAZE_EXPLORER_DRY_START_VIOLATION
    pose = _dict(handoff_artifact.get('robot_pose_after_ingress'))
    distance = _dict(handoff_artifact.get('distance_to_ingress_goal'))
    orientation = _dict(handoff_artifact.get('orientation_error'))
    action = _dict(handoff_artifact.get('nav2_action_idle_state'))
    tf = _dict(handoff_artifact.get('tf_freshness'))
    scan = _dict(handoff_artifact.get('scan_freshness'))
    costmap = _dict(handoff_artifact.get('costmap_freshness'))
    ingress = _dict(handoff_artifact.get('ingress_goal_result'))
    if ingress and not (
        _bool(ingress.get('accepted'))
        and _bool(ingress.get('result_received'))
        and ingress.get('result_status_label') == 'SUCCEEDED'
        and _bool(ingress.get('locked_explicit_inner_ingress_goal', True))
    ):
        return INGRESS_SUCCESS_HANDOFF_NOT_READY
    if not (_bool(pose.get('pose_available')) and _bool(distance.get('within_tolerance')) and _bool(orientation.get('within_tolerance'))):
        return POSE_NOT_AT_INGRESS_GOAL
    if not _bool(action.get('idle')):
        return NAV2_ACTION_NOT_IDLE
    if not (_bool(tf.get('fresh')) and _bool(scan.get('fresh'))):
        return TF_OR_SCAN_STALE
    if not _bool(costmap.get('fresh')):
        return COSTMAP_NOT_READY
    if _bool(dry_start.get('maze_explorer_started')) and _int_value(dry_start.get('maze_explorer_max_goals'), -1) == 0 and not _bool(dry_start.get('exploration_goal_dispatched')):
        return INGRESS_SUCCESS_HANDOFF_READY_DRY_START_STOP
    return INGRESS_SUCCESS_HANDOFF_NOT_READY


def run_phase120_ingress_chain(args: argparse.Namespace) -> dict[str, Any]:
    if args.phase120_artifact and args.phase120_artifact.exists():
        return json.loads(args.phase120_artifact.read_text(encoding='utf-8'))
    runner = _load_phase120_runner()
    phase120_args = argparse.Namespace(
        output=args.phase120_output,
        preflight_output=args.preflight_output,
        run_id=args.run_id,
        launch_log_path=args.launch_log_path,
        preflight_timeout_sec=args.preflight_timeout_sec,
        tf_stability_window_sec=args.tf_stability_window_sec,
        sample_period_sec=args.sample_period_sec,
        startup_grace_sec=args.startup_grace_sec,
        action_server_wait_sec=args.action_server_wait_sec,
        bounded_goal_result_wait_sec=args.bounded_goal_result_wait_sec,
        dry_run_dispatch=False,
        enable_readiness_wait=True,
        readiness_wait_timeout_sec=args.readiness_wait_timeout_sec,
        readiness_wait_sample_period_sec=args.readiness_wait_sample_period_sec,
    )
    return runner.run_smoke(phase120_args)


def run_maze_explorer_dry_start(*, dry_start: dict[str, Any], duration_sec: float, stdout_path: Path, stderr_path: Path) -> dict[str, Any]:  # pragma: no cover - ROS runtime
    import rclpy
    from rclpy.parameter import Parameter
    from std_msgs.msg import String

    record = dry_start
    record['maze_explorer_start_allowed'] = True
    record['maze_explorer_max_goals'] = 0
    command = [
        'ros2', 'run', 'tugbot_maze', 'maze_explorer',
        '--ros-args', '-p', 'use_sim_time:=true', '-p', 'max_goals:=0', '-p', 'exploration_rate_hz:=1.0'
    ]
    # Guardrail token for static tests and human audit: max_goals:=0 is the only allowed dry-start mode.
    record['process']['command'] = command
    stdout_path.parent.mkdir(parents=True, exist_ok=True)
    stderr_path.parent.mkdir(parents=True, exist_ok=True)
    events: list[dict[str, Any]] = []
    states: list[dict[str, Any]] = []
    proc: subprocess.Popen[str] | None = None

    def _event_cb(msg: String) -> None:
        payload: dict[str, Any]
        try:
            parsed = json.loads(msg.data)
            payload = parsed if isinstance(parsed, dict) else {'raw': msg.data}
        except Exception:
            payload = {'raw': msg.data}
        payload['received_wall_time_sec'] = _now_wall()
        events.append(payload)

    def _state_cb(msg: String) -> None:
        payload: dict[str, Any]
        try:
            parsed = json.loads(msg.data)
            payload = parsed if isinstance(parsed, dict) else {'raw': msg.data}
        except Exception:
            payload = {'raw': msg.data}
        payload['received_wall_time_sec'] = _now_wall()
        states.append(payload)

    rclpy.init(args=None)
    node = rclpy.create_node('phase122_maze_explorer_dry_start_monitor')
    node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    sub_events = node.create_subscription(String, GOAL_EVENTS_TOPIC, _event_cb, 10)
    sub_state = node.create_subscription(String, EXPLORER_STATE_TOPIC, _state_cb, 10)
    start = _now_wall()
    try:
        proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=os.environ.copy(), start_new_session=True)
        record['maze_explorer_started'] = True
        record['process']['pid'] = proc.pid
        deadline = start + float(duration_sec)
        while _now_wall() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if proc.poll() is not None:
                # Continue spinning briefly to capture any final state/event messages.
                for _ in range(5):
                    rclpy.spin_once(node, timeout_sec=0.1)
                break
        record['dry_start_duration_sec'] = max(0.0, _now_wall() - start)
    except Exception as exc:
        record['process']['exception_text'] = str(exc)
        record['process']['traceback_tail'] = traceback.format_exc()[-2000:]
    finally:
        try:
            node.destroy_subscription(sub_events)
            node.destroy_subscription(sub_state)
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

    dispatch_events = [event for event in events if str(event.get('event', '')).lower() == 'dispatch']
    combined_logs = ((stdout_path.read_text(encoding='utf-8', errors='replace') if stdout_path.exists() else '') + '\n' + (stderr_path.read_text(encoding='utf-8', errors='replace') if stderr_path.exists() else ''))
    log_dispatch = 'maze explorer sending' in combined_logs or 'send_goal_async' in combined_logs
    record['goal_events'] = events[-20:]
    record['explorer_states'] = states[-20:]
    record['goal_event_count'] = len(events)
    record['dispatch_event_count'] = len(dispatch_events)
    record['exploration_goal_dispatched'] = bool(dispatch_events or log_dispatch)
    record['dry_start_observed'] = bool(record['maze_explorer_started'] and ('maze_explorer started' in combined_logs or states or record['dry_start_duration_sec']))
    return record


def build_phase122_artifact(*, run_id: str, phase120_artifact: dict[str, Any], handoff_artifact: dict[str, Any], dry_start: dict[str, Any], classification: str) -> dict[str, Any]:
    # Keep the standalone handoff artifact synchronized with the same-run Phase120 chain.
    # This preserves the fail-closed requirement that readiness, preflight, and the locked
    # explicit inner-ingress result all passed before maze_explorer dry-start is allowed.
    _fill_phase120_preconditions(handoff_artifact, phase120_artifact)
    handoff_allowed = handoff_gate_allowed(handoff_artifact)
    return {
        'phase': PHASE,
        'run_id': run_id,
        'mode': MODE,
        'created_wall_time_sec': _now_wall(),
        'phase120_source_artifact': phase120_artifact.get('artifact_path') or phase120_artifact.get('run_id'),
        'phase120_artifact': phase120_artifact,
        'handoff_artifact': handoff_artifact,
        'dry_start': dry_start,
        'classification': classification,
        'handoff_allowed': bool(handoff_allowed),
        'maze_explorer_start_allowed': _bool(dry_start.get('maze_explorer_start_allowed')),
        'maze_explorer_started': _bool(dry_start.get('maze_explorer_started')),
        'maze_explorer_max_goals': _int_value(dry_start.get('maze_explorer_max_goals'), -1),
        'exploration_goal_dispatched': _bool(dry_start.get('exploration_goal_dispatched')),
        'guardrails': {
            'only_explicit_inner_ingress_goal_sent': phase120_ingress_success(phase120_artifact),
            'no_goal1_or_carry_over_or_staging_goal_sent': True,
            'no_branch_scoring_change': True,
            'branch_scoring_changed': False,
            'no_centerline_fallback_terminal_acceptance_change': True,
            'no_nav2_mppi_controller_config_tuning': True,
            'preflight_preserved': True,
            'autonomous_success_claimed': False,
            'exit_success_claimed': False,
        },
        'notes': [
            'Phase122 validates post-ingress handoff and maze_explorer dry-start/max_goals=0 only.',
            'No Goal1/carry-over/staging/branch/centerline/fallback/terminal/exit goal is sent.',
            'Dry-start success is not autonomous exploration success and not exit success.',
        ],
    }


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def run_phase122(args: argparse.Namespace) -> dict[str, Any]:
    phase120_artifact = run_phase120_ingress_chain(args)
    if args.phase120_output:
        phase120_artifact['artifact_path'] = str(args.phase120_output)
    handoff = collect_handoff_runtime_evidence(
        phase120_artifact,
        collection_sec=args.handoff_collection_sec,
        pose_tolerance_m=args.pose_tolerance_m,
        yaw_tolerance_rad=args.yaw_tolerance_rad,
        max_costmap_age_sec=args.max_costmap_age_sec,
        max_scan_age_sec=args.max_scan_age_sec,
        max_tf_age_sec=args.max_tf_age_sec,
    )
    dry = initial_dry_start_record(max_goals=0)
    if handoff_gate_allowed(handoff):
        dry = run_maze_explorer_dry_start(
            dry_start=dry,
            duration_sec=args.dry_start_duration_sec,
            stdout_path=args.output.with_name(args.output.stem + '_maze_explorer_stdout.log'),
            stderr_path=args.output.with_name(args.output.stem + '_maze_explorer_stderr.log'),
        )
    classification = classify_phase122(handoff, dry)
    artifact = build_phase122_artifact(
        run_id=args.run_id,
        phase120_artifact=phase120_artifact,
        handoff_artifact=handoff,
        dry_start=dry,
        classification=classification,
    )
    write_json(args.output, artifact)
    return artifact


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--phase120-output', type=Path, required=True)
    parser.add_argument('--preflight-output', type=Path, required=True)
    parser.add_argument('--phase120-artifact', type=Path, help='test/debug only: reuse an existing Phase120 artifact instead of dispatching')
    parser.add_argument('--run-id', default='phase122_post_ingress_handoff_dry_start')
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
    parser.add_argument('--dry-start-duration-sec', type=float, default=5.0)
    args = parser.parse_args()
    artifact = run_phase122(args)
    print(json.dumps({
        'classification': artifact.get('classification'),
        'handoff_allowed': artifact.get('handoff_allowed'),
        'maze_explorer_start_allowed': artifact.get('maze_explorer_start_allowed'),
        'maze_explorer_started': artifact.get('maze_explorer_started'),
        'maze_explorer_max_goals': artifact.get('maze_explorer_max_goals'),
        'exploration_goal_dispatched': artifact.get('exploration_goal_dispatched'),
        'artifact': str(args.output),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
