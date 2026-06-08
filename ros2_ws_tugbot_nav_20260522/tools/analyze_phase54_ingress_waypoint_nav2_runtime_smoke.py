#!/usr/bin/env python3
"""Phase54 ingress waypoint Nav2 runtime smoke recorder/analyzer.

This tool is diagnostics-only for the active scaled2x maze world. It can:

* record runtime map/scan/TF/local-costmap/plan evidence;
* send one Nav2 NavigateToPose goal to ingress_waypoint_map=(1.0, 0.0, 0.0);
* analyze bounded-runtime artifacts.

It does not start maze_explorer and it never claims autonomous exploration success.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

PHASE = 'Phase54 Ingress Waypoint Nav2 Runtime Smoke'
RUN_ID = 'phase54_ingress_waypoint_nav2_runtime_smoke'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
ACTIVE_METADATA = 'maze_20260528_scaled_instance.yaml'
INGRESS_WAYPOINT_MAP = {'x_m': 1.0, 'y_m': 0.0, 'yaw_rad': 0.0}
ACCEPTANCE_RADIUS_M = 0.35
MAP_EXIT = {'x_m': 21.072562, 'y_m': 18.083566, 'radius_m': 1.2}
TF_LOOKUP_PAIRS = ('map->base_link', 'odom->base_link', 'map->odom')
ALLOWED_CLASSIFICATIONS = {
    'INGRESS_NAV2_GOAL_REACHED',
    'INGRESS_NAV2_GOAL_FAILED',
    'INGRESS_NAV2_INCONCLUSIVE_RUNTIME_DATA_GAP',
    'GUARDRAIL_VIOLATION_EXPLORER_STARTED',
}

try:  # Runtime imports are optional for offline unit tests.
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
    from sensor_msgs.msg import LaserScan
    import tf2_ros
except Exception:  # pragma: no cover
    rclpy = None
    ActionClient = None
    Duration = None
    Node = object
    QoSProfile = None
    ReliabilityPolicy = None
    DurabilityPolicy = None
    HistoryPolicy = None
    GoalStatus = None
    PoseStamped = None
    NavigateToPose = None
    OccupancyGrid = None
    Odometry = None
    NavPath = None
    LaserScan = None
    tf2_ros = None


def _safe_json_load(path: Path | None, default: Any) -> Any:
    if not path or not path.exists() or not path.stat().st_size:
        return default
    try:
        return json.loads(path.read_text(encoding='utf-8', errors='replace'))
    except Exception:
        return default


def _read_text(path: Path | None) -> str:
    if not path or not path.exists():
        return ''
    return path.read_text(encoding='utf-8', errors='replace')


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _quat_yaw(q: Any) -> float:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _transform_to_dict(transform_stamped: Any) -> dict[str, Any]:
    t = transform_stamped.transform.translation
    r = transform_stamped.transform.rotation
    return {
        'available': True,
        'parent': str(transform_stamped.header.frame_id),
        'child': str(transform_stamped.child_frame_id),
        'translation': {'x': float(t.x), 'y': float(t.y), 'z': float(t.z)},
        'rotation': {'x': float(r.x), 'y': float(r.y), 'z': float(r.z), 'w': float(r.w)},
        'yaw_rad': _quat_yaw(r),
    }


def _lookup_transform(buffer: Any, parent: str, child: str) -> dict[str, Any]:
    try:
        return _transform_to_dict(buffer.lookup_transform(parent, child, rclpy.time.Time()))
    except Exception as exc:
        return {'available': False, 'parent': parent, 'child': child, 'error': str(exc)}


def _odom_summary(msg: Any | None) -> dict[str, Any] | None:
    if msg is None:
        return None
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return {
        'frame_id': str(msg.header.frame_id),
        'child_frame_id': str(msg.child_frame_id),
        'pose': {'x': float(p.x), 'y': float(p.y), 'z': float(p.z), 'yaw_rad': _quat_yaw(q)},
    }


def _robot_pose_from_tf(tf_lookups: dict[str, Any], odom: dict[str, Any] | None = None) -> list[float] | None:
    tf = tf_lookups.get('map->base_link') or {}
    if tf.get('available'):
        t = tf.get('translation') or {}
        return [float(t.get('x', 0.0)), float(t.get('y', 0.0)), float(tf.get('yaw_rad', 0.0))]
    if odom:
        pose = odom.get('pose') or {}
        return [float(pose.get('x', 0.0)), float(pose.get('y', 0.0)), float(pose.get('yaw_rad', 0.0))]
    return None


def _grid_near_robot_window(msg: Any, robot_pose: list[float], radius_m: float = 1.0, *, include_out_of_bounds: bool) -> dict[str, Any]:
    width = int(msg.info.width)
    height = int(msg.info.height)
    resolution = float(msg.info.resolution)
    origin_x = float(msg.info.origin.position.x)
    origin_y = float(msg.info.origin.position.y)
    data = [int(v) for v in msg.data]
    cx = int(math.floor((float(robot_pose[0]) - origin_x) / resolution))
    cy = int(math.floor((float(robot_pose[1]) - origin_y) / resolution))
    radius_cells = int(math.ceil(radius_m / resolution))
    counts = {'sample_count': 0, 'in_bounds_count': 0, 'out_of_bounds_count': 0, 'unknown_count': 0, 'free_count': 0, 'occupied_count': 0}
    min_x = cx - radius_cells
    max_x = cx + radius_cells
    min_y = cy - radius_cells
    max_y = cy + radius_cells
    for yy in range(min_y, max_y + 1):
        for xx in range(min_x, max_x + 1):
            wx = origin_x + (xx + 0.5) * resolution
            wy = origin_y + (yy + 0.5) * resolution
            if math.hypot(wx - float(robot_pose[0]), wy - float(robot_pose[1])) > radius_m + 1e-9:
                continue
            in_bounds = 0 <= xx < width and 0 <= yy < height
            if not in_bounds:
                counts['out_of_bounds_count'] += 1
                if include_out_of_bounds:
                    counts['sample_count'] += 1
                    counts['unknown_count'] += 1
                continue
            counts['in_bounds_count'] += 1
            counts['sample_count'] += 1
            value = data[yy * width + xx]
            if value < 0:
                counts['unknown_count'] += 1
            elif value >= 65:
                counts['occupied_count'] += 1
            else:
                counts['free_count'] += 1
    total = counts['sample_count']
    known = counts['free_count'] + counts['occupied_count']
    return {
        **counts,
        'total': total,
        'known_count': known,
        'known_ratio': known / total if total else 0.0,
        'free_ratio': counts['free_count'] / total if total else 0.0,
        'occupied_ratio': counts['occupied_count'] / total if total else 0.0,
        'unknown_ratio': counts['unknown_count'] / total if total else 0.0,
        'center_cell': [cx, cy],
        'robot_cell': [cx, cy],
        'robot_in_bounds': bool(0 <= cx < width and 0 <= cy < height),
        'radius_m': radius_m,
        'radius_cells': radius_cells,
        'sample_window': {'min_x': min_x, 'max_x': max_x, 'min_y': min_y, 'max_y': max_y},
        'out_of_bounds_as_unknown': bool(include_out_of_bounds),
    }


def _grid_summary(msg: Any | None, robot_pose: list[float] | None = None, radius_m: float = 1.0) -> dict[str, Any]:
    if msg is None:
        return {'available': False}
    width = int(msg.info.width)
    height = int(msg.info.height)
    resolution = float(msg.info.resolution)
    origin_x = float(msg.info.origin.position.x)
    origin_y = float(msg.info.origin.position.y)
    out: dict[str, Any] = {
        'available': True,
        'frame_id': str(msg.header.frame_id),
        'width': width,
        'height': height,
        'resolution': resolution,
        'origin': {'x': origin_x, 'y': origin_y},
        'map_boundary': {
            'min_x': origin_x,
            'max_x': origin_x + width * resolution,
            'min_y': origin_y,
            'max_y': origin_y + height * resolution,
            'width_m': width * resolution,
            'height_m': height * resolution,
        },
    }
    if robot_pose and width > 0 and height > 0 and resolution > 0:
        out['inclusive_near_robot'] = _grid_near_robot_window(msg, robot_pose, radius_m, include_out_of_bounds=True)
        out['in_bounds_near_robot'] = _grid_near_robot_window(msg, robot_pose, radius_m, include_out_of_bounds=False)
    return out


def _scan_summary(msg: Any | None) -> dict[str, Any]:
    if msg is None:
        return {'available': False, 'finite_count': 0, 'nearest_obstacle_m': None}
    finite = [float(v) for v in msg.ranges if math.isfinite(float(v))]
    return {
        'available': True,
        'frame_id': str(msg.header.frame_id),
        'sample_count': len(msg.ranges),
        'finite_count': len(finite),
        'nearest_obstacle_m': min(finite) if finite else None,
        'range_min': float(msg.range_min),
        'range_max': float(msg.range_max),
    }


def _path_summary(msg: Any | None) -> dict[str, Any]:
    if msg is None:
        return {'available': False, 'pose_count': 0}
    poses = list(msg.poses)
    summary: dict[str, Any] = {'available': True, 'frame_id': str(msg.header.frame_id), 'pose_count': len(poses)}
    if poses:
        first = poses[0].pose.position
        last = poses[-1].pose.position
        summary['first_pose'] = {'x': float(first.x), 'y': float(first.y)}
        summary['last_pose'] = {'x': float(last.x), 'y': float(last.y)}
        length = 0.0
        prev = poses[0].pose.position
        for pose in poses[1:]:
            cur = pose.pose.position
            length += math.hypot(float(cur.x - prev.x), float(cur.y - prev.y))
            prev = cur
        summary['path_length_m'] = length
    return summary


class RuntimeRecorder(Node):  # pragma: no cover - requires ROS graph
    def __init__(self, duration_sec: float, snapshot_period_sec: float, output: Path):
        super().__init__('phase54_ingress_waypoint_runtime_recorder')
        self.duration_sec = float(duration_sec)
        self.snapshot_period_sec = float(snapshot_period_sec)
        self.output = output
        self.start_time = time.monotonic()
        self.samples: list[dict[str, Any]] = []
        self.map_msg = None
        self.local_costmap_msg = None
        self.global_costmap_msg = None
        self.scan_msg = None
        self.odom_msg = None
        self.plan_msg = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        live_qos = QoSProfile(depth=10)
        self.create_subscription(OccupancyGrid, '/map', self._set_map, map_qos)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._set_local, live_qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self._set_global, live_qos)
        self.create_subscription(LaserScan, '/scan', self._set_scan, live_qos)
        self.create_subscription(Odometry, '/odom', self._set_odom, live_qos)
        self.create_subscription(NavPath, '/plan', self._set_plan, live_qos)
        self.create_timer(self.snapshot_period_sec, self._snapshot)

    def _set_map(self, msg: Any) -> None:
        self.map_msg = msg

    def _set_local(self, msg: Any) -> None:
        self.local_costmap_msg = msg

    def _set_global(self, msg: Any) -> None:
        self.global_costmap_msg = msg

    def _set_scan(self, msg: Any) -> None:
        self.scan_msg = msg

    def _set_odom(self, msg: Any) -> None:
        self.odom_msg = msg

    def _set_plan(self, msg: Any) -> None:
        self.plan_msg = msg

    def _snapshot(self) -> None:
        elapsed = time.monotonic() - self.start_time
        odom = _odom_summary(self.odom_msg)
        tf_lookups = {pair: _lookup_transform(self.tf_buffer, *pair.split('->')) for pair in TF_LOOKUP_PAIRS}
        robot_pose = _robot_pose_from_tf(tf_lookups, odom)
        distance = None
        if robot_pose:
            distance = math.hypot(robot_pose[0] - INGRESS_WAYPOINT_MAP['x_m'], robot_pose[1] - INGRESS_WAYPOINT_MAP['y_m'])
        self.samples.append({
            'elapsed_sec': elapsed,
            'robot_pose_map': robot_pose,
            'distance_to_ingress_m': distance,
            'odom': odom,
            'tf_lookups': tf_lookups,
            'map': _grid_summary(self.map_msg, robot_pose, 1.0),
            'local_costmap': _grid_summary(self.local_costmap_msg, robot_pose, 1.0),
            'global_costmap': _grid_summary(self.global_costmap_msg, robot_pose, 1.0),
            'scan': _scan_summary(self.scan_msg),
            'plan_evidence': _path_summary(self.plan_msg),
        })
        if elapsed >= self.duration_sec:
            raise KeyboardInterrupt


def record_runtime(args: argparse.Namespace) -> int:  # pragma: no cover - requires ROS graph
    if rclpy is None:
        raise SystemExit('rclpy is required for --record-runtime')
    rclpy.init()
    node = RuntimeRecorder(args.record_runtime, args.snapshot_period_sec, Path(args.output))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if exc.__class__.__name__ != 'ExternalShutdownException':
            raise
    finally:
        data = {
            'phase': PHASE,
            'run_id': RUN_ID,
            'ingress_waypoint_map': INGRESS_WAYPOINT_MAP,
            'acceptance_radius_m': ACCEPTANCE_RADIUS_M,
            'samples': node.samples,
        }
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        Path(args.output).write_text(json.dumps(data, indent=2, sort_keys=True), encoding='utf-8')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


class IngressGoalClient(Node):  # pragma: no cover - requires ROS graph
    def __init__(self, output: Path, goal_timeout_sec: float):
        super().__init__('phase54_ingress_waypoint_goal_client')
        self.output = output
        self.goal_timeout_sec = float(goal_timeout_sec)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.result: dict[str, Any] = {
            'phase': PHASE,
            'run_id': RUN_ID,
            'ingress_waypoint_map': INGRESS_WAYPOINT_MAP,
            'goal_sent': False,
            'goal_accepted': False,
            'result_received': False,
            'success': False,
            'feedback': [],
        }

    def run(self) -> dict[str, Any]:
        self.result['action_server_available'] = bool(self.client.wait_for_server(timeout_sec=20.0))
        if not self.result['action_server_available']:
            self.result['reason'] = 'navigate_to_pose_action_server_unavailable'
            return self.result
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(INGRESS_WAYPOINT_MAP['x_m'])
        goal_msg.pose.pose.position.y = float(INGRESS_WAYPOINT_MAP['y_m'])
        qx, qy, qz, qw = _quat_from_yaw(float(INGRESS_WAYPOINT_MAP['yaw_rad']))
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        self.result['goal_sent'] = True
        self.result['goal_pose'] = {'frame_id': 'map', **INGRESS_WAYPOINT_MAP}
        send_future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result() if send_future.done() else None
        if goal_handle is None:
            self.result['reason'] = 'goal_send_future_timeout'
            return self.result
        self.result['goal_accepted'] = bool(goal_handle.accepted)
        if not goal_handle.accepted:
            self.result['reason'] = 'goal_rejected'
            return self.result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.goal_timeout_sec)
        if not result_future.done():
            self.result['reason'] = 'goal_result_timeout'
            try:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                self.result['cancel_requested'] = True
            except Exception as exc:
                self.result['cancel_error'] = str(exc)
            return self.result
        result_wrapper = result_future.result()
        status = int(getattr(result_wrapper, 'status', -1))
        nav_result = getattr(result_wrapper, 'result', None)
        error_code = int(getattr(nav_result, 'error_code', -1)) if nav_result is not None else -1
        error_msg = str(getattr(nav_result, 'error_msg', '')) if nav_result is not None else ''
        self.result.update({
            'result_received': True,
            'status': status,
            'status_text': 'STATUS_SUCCEEDED' if status == int(GoalStatus.STATUS_SUCCEEDED) else str(status),
            'error_code': error_code,
            'error_msg': error_msg,
            'success': bool(status == int(GoalStatus.STATUS_SUCCEEDED) and error_code == 0),
        })
        return self.result

    def _feedback(self, feedback_msg: Any) -> None:
        feedback = feedback_msg.feedback
        self.result['feedback'].append({
            'navigation_time_sec': float(getattr(getattr(feedback, 'navigation_time', None), 'sec', 0)) + float(getattr(getattr(feedback, 'navigation_time', None), 'nanosec', 0)) / 1e9,
            'distance_remaining': float(getattr(feedback, 'distance_remaining', math.nan)),
            'number_of_recoveries': int(getattr(feedback, 'number_of_recoveries', 0)),
        })


def send_goal(args: argparse.Namespace) -> int:  # pragma: no cover - requires ROS graph
    if rclpy is None:
        raise SystemExit('rclpy is required for --send-goal')
    rclpy.init()
    node = IngressGoalClient(Path(args.output), args.goal_timeout_sec)
    try:
        result = node.run()
    finally:
        node.output.parent.mkdir(parents=True, exist_ok=True)
        node.output.write_text(json.dumps(node.result, indent=2, sort_keys=True), encoding='utf-8')
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, sort_keys=True))
    return 0


def _distance_to_ingress(pose: list[float] | None) -> float | None:
    if not pose:
        return None
    return math.hypot(float(pose[0]) - INGRESS_WAYPOINT_MAP['x_m'], float(pose[1]) - INGRESS_WAYPOINT_MAP['y_m'])


def _runtime_stats(runtime: dict[str, Any]) -> dict[str, Any]:
    samples = runtime.get('samples') if isinstance(runtime.get('samples'), list) else []
    pose_samples = [s for s in samples if isinstance(s, dict) and s.get('robot_pose_map')]
    first_pose = pose_samples[0].get('robot_pose_map') if pose_samples else None
    final_pose = pose_samples[-1].get('robot_pose_map') if pose_samples else None
    max_move = 0.0
    if first_pose:
        for sample in pose_samples:
            pose = sample.get('robot_pose_map')
            max_move = max(max_move, math.hypot(float(pose[0]) - float(first_pose[0]), float(pose[1]) - float(first_pose[1])))
    final_distance = _distance_to_ingress(final_pose)
    plan_samples = [s.get('plan_evidence') for s in samples if isinstance(s, dict) and isinstance(s.get('plan_evidence'), dict)]
    plan_generated = any(bool(p.get('available')) and int(p.get('pose_count') or 0) > 0 for p in plan_samples)
    return {
        'sample_count': len(samples),
        'pose_sample_count': len(pose_samples),
        'first_pose_map': first_pose,
        'final_pose_map': final_pose,
        'final_distance_to_ingress_m': final_distance,
        'robot_moved_distance_m': max_move,
        'robot_moved': bool(max_move >= 0.10),
        'plan_generated': bool(plan_generated),
        'plan_evidence': next((p for p in reversed(plan_samples) if bool(p.get('available')) and int(p.get('pose_count') or 0) > 0), {'available': False, 'pose_count': 0}),
        'first_sample': samples[0] if samples else None,
        'final_sample': samples[-1] if samples else None,
    }


def _last_lifecycle_state(lifecycle: str, node: str) -> str | None:
    current_node = None
    latest: str | None = None
    states = {'active [3]', 'inactive [2]', 'unconfigured [1]', 'finalized [4]'}
    for raw in lifecycle.splitlines():
        line = raw.strip()
        if line.startswith('/'):
            current_node = line[1:]
            continue
        if current_node == node and line in states:
            latest = line
    return latest


def _readiness_summary(lifecycle: str, action_info: str, goal_pose_info: str) -> dict[str, Any]:
    controller_state = _last_lifecycle_state(lifecycle, 'controller_server')
    planner_state = _last_lifecycle_state(lifecycle, 'planner_server')
    bt_state = _last_lifecycle_state(lifecycle, 'bt_navigator')
    return {
        'controller_server_state': controller_state,
        'planner_server_state': planner_state,
        'bt_navigator_state': bt_state,
        'controller_server_active': controller_state == 'active [3]',
        'planner_server_active': planner_state == 'active [3]',
        'bt_navigator_active': bt_state == 'active [3]',
        'navigate_to_pose_action_servers_1': 'Action servers: 1' in action_info,
        'goal_pose_subscription_count_1': 'Subscription count: 1' in goal_pose_info,
        'readiness_ready': 'readiness_ready=1' in lifecycle,
    }


def analyze(args: argparse.Namespace) -> int:
    artifact_dir = Path(args.artifact_dir)
    runtime = _safe_json_load(Path(args.runtime_evidence) if args.runtime_evidence else None, {})
    action = _safe_json_load(Path(args.action_result) if args.action_result else None, {})
    cleanup_text = _read_text(Path(args.cleanup_processes_after) if args.cleanup_processes_after else None)
    lifecycle_text = _read_text(Path(args.lifecycle_readiness) if args.lifecycle_readiness else None)
    action_info_text = _read_text(Path(args.action_info) if args.action_info else None)
    goal_pose_info_text = _read_text(Path(args.goal_pose_info) if args.goal_pose_info else None)
    graph_text = _read_text(Path(args.ros_graph_snapshot) if args.ros_graph_snapshot else None) + '\n' + _read_text(Path(args.ros_graph_final) if args.ros_graph_final else None)
    launch_text = _read_text(Path(args.launch_log) if args.launch_log else None)

    explorer_violation = any(token in graph_text or token in launch_text for token in ('/maze_explorer', 'maze_goal_monitor', 'frontier_explorer'))
    stats = _runtime_stats(runtime if isinstance(runtime, dict) else {})
    final_sample = stats.get('final_sample') if isinstance(stats.get('final_sample'), dict) else {}
    action_success = bool(action.get('success'))
    action_result_received = bool(action.get('result_received'))
    goal_reached = bool(stats.get('final_distance_to_ingress_m') is not None and float(stats['final_distance_to_ingress_m']) <= ACCEPTANCE_RADIUS_M)
    data_gap = not bool(stats['sample_count']) or not action or stats.get('final_pose_map') is None or not action_result_received

    if explorer_violation:
        classification = 'GUARDRAIL_VIOLATION_EXPLORER_STARTED'
    elif data_gap:
        classification = 'INGRESS_NAV2_INCONCLUSIVE_RUNTIME_DATA_GAP'
    elif action_success and goal_reached and bool(stats.get('robot_moved')):
        classification = 'INGRESS_NAV2_GOAL_REACHED'
    else:
        classification = 'INGRESS_NAV2_GOAL_FAILED'

    summary = {
        'phase': PHASE,
        'run_id': RUN_ID,
        'classification': classification,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'active_world': ACTIVE_WORLD,
        'active_metadata': ACTIVE_METADATA,
        'ingress_waypoint_map': INGRESS_WAYPOINT_MAP,
        'acceptance_radius_m': ACCEPTANCE_RADIUS_M,
        'readiness': _readiness_summary(lifecycle_text, action_info_text, goal_pose_info_text),
        'action_result': action,
        'goal_reached_within_acceptance_radius': goal_reached,
        'distance_to_ingress_m': stats.get('final_distance_to_ingress_m'),
        'robot_pose_timeline_sample_count': stats.get('pose_sample_count'),
        'robot_moved_distance_m': stats.get('robot_moved_distance_m'),
        'robot_moved': stats.get('robot_moved'),
        'plan_generated': stats.get('plan_generated'),
        'plan_evidence': stats.get('plan_evidence'),
        'first_robot_pose_map': stats.get('first_pose_map'),
        'final_robot_pose_map': stats.get('final_pose_map'),
        'before_ingress_evidence': stats.get('first_sample'),
        'after_ingress_evidence': final_sample,
        'after_ingress_map_inclusive_near_robot': (final_sample.get('map') or {}).get('inclusive_near_robot') if isinstance(final_sample, dict) else None,
        'after_ingress_local_costmap_inclusive_near_robot': (final_sample.get('local_costmap') or {}).get('inclusive_near_robot') if isinstance(final_sample, dict) else None,
        'after_ingress_scan': final_sample.get('scan') if isinstance(final_sample, dict) else None,
        'after_ingress_tf_lookups': final_sample.get('tf_lookups') if isinstance(final_sample, dict) else None,
        'guardrail_violation_explorer_started': explorer_violation,
        'cleanup_empty': cleanup_text.strip() == '',
        'cleanup_processes_after': cleanup_text.splitlines(),
        'complete_autonomous_success_claimed': False,
        'notes': [
            'Phase54 validates only ingress waypoint Nav2 runtime reachability.',
            'It does not start maze_explorer and does not claim autonomous exploration success.',
            'A later explicitly authorized phase is required before gated maze_explorer smoke after ingress.',
        ],
        'artifact_dir': str(artifact_dir),
    }
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    print(json.dumps(summary, sort_keys=True))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=PHASE)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--record-runtime', type=float, help='record runtime evidence for N seconds')
    mode.add_argument('--send-goal', action='store_true', help='send the single ingress NavigateToPose goal')
    mode.add_argument('--analyze', action='store_true', help='analyze collected artifacts')
    parser.add_argument('--snapshot-period-sec', type=float, default=1.0)
    parser.add_argument('--goal-timeout-sec', type=float, default=90.0)
    parser.add_argument('--artifact-dir', default='log/phase54_ingress_waypoint_nav2_runtime_smoke')
    parser.add_argument('--output', required=True)
    parser.add_argument('--runtime-evidence')
    parser.add_argument('--action-result')
    parser.add_argument('--lifecycle-readiness')
    parser.add_argument('--action-info')
    parser.add_argument('--goal-pose-info')
    parser.add_argument('--ros-graph-snapshot')
    parser.add_argument('--ros-graph-final')
    parser.add_argument('--launch-log')
    parser.add_argument('--cleanup-processes-after')
    args = parser.parse_args(argv)
    if args.record_runtime is not None:
        return record_runtime(args)
    if args.send_goal:
        return send_goal(args)
    return analyze(args)


if __name__ == '__main__':
    raise SystemExit(main())
