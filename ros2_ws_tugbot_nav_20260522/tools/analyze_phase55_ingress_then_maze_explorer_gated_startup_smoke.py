#!/usr/bin/env python3
"""Phase55 ingress-then-maze_explorer gated startup smoke recorder/analyzer.

Artifact contract documented in this tool:
* launch.log
* ingress action result
* lifecycle/action/topic readiness
* robot pose timeline
* explorer_state.jsonl
* goal_events.jsonl
* gate payload before/after ingress
* first topology sampling diagnostics
* map/scan/TF/local/global costmap evidence
* summary/analysis JSON
* cleanup_processes_after.txt

Phase55 only validates that ingress guidance can hand off into the Phase49
maze_explorer dispatch-entry readiness gate. It never claims autonomous
exploration success and does not require reaching the exit.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

PHASE = 'Phase55 Ingress-then-maze_explorer Gated Startup Smoke'
RUN_ID = 'phase55_ingress_then_maze_explorer_gated_startup_smoke'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
ACTIVE_METADATA = 'maze_20260528_scaled_instance.yaml'
INGRESS_WAYPOINT_MAP = {'x_m': 1.0, 'y_m': 0.0, 'yaw_rad': 0.0}
ACCEPTANCE_RADIUS_M = 0.35
MAP_EXIT = {'x_m': 21.072562, 'y_m': 18.083566, 'radius_m': 1.2}
TF_LOOKUP_PAIRS = ('map->base_link', 'odom->base_link', 'map->odom')
ALLOWED_CLASSIFICATIONS = {
    'INGRESS_THEN_GATE_READY_FIRST_DISPATCH',
    'INGRESS_THEN_GATE_READY_NO_DISPATCH_TOPOLOGY_REJECTED',
    'INGRESS_REACHED_GATE_STILL_NOT_READY',
    'INGRESS_GOAL_FAILED',
    'GUARDRAIL_VIOLATION_UNBOUNDED_OR_SUCCESS_CLAIM',
}

try:  # Runtime imports are optional for static unit tests.
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
    from std_msgs.msg import String
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
    String = None
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


def _safe_json_loads(text: str) -> Any | None:
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return None


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
        super().__init__('phase55_ingress_then_explorer_runtime_recorder')
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
        super().__init__('phase55_ingress_goal_client')
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


def read_explorer_state_jsonl(path: Path | None) -> list[dict[str, Any]]:
    return _read_jsonl_payloads(path, payload_key='state')


def read_goal_events_jsonl(path: Path | None) -> list[dict[str, Any]]:
    return _read_jsonl_payloads(path, payload_key=None)


def _read_jsonl_payloads(path: Path | None, *, payload_key: str | None) -> list[dict[str, Any]]:
    if not path or not path.exists() or not path.stat().st_size:
        return []
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        row = _safe_json_loads(line)
        if not isinstance(row, dict):
            continue
        payload = row.get(payload_key) if payload_key and isinstance(row.get(payload_key), dict) else row
        if payload_key is None and isinstance(row.get('state'), dict):
            payload = row.get('state')
        if isinstance(payload, dict):
            payload = dict(payload)
            payload['_recorder_elapsed_sec'] = row.get('elapsed_sec')
            payload['_recorder_wall_time'] = row.get('wall_time')
            payload['_recorder_seq'] = row.get('seq')
            rows.append(payload)
    return rows


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
    return {
        'sample_count': len(samples),
        'pose_sample_count': len(pose_samples),
        'first_pose_map': first_pose,
        'final_pose_map': final_pose,
        'final_distance_to_ingress_m': final_distance,
        'robot_moved_distance_m': max_move,
        'robot_moved': bool(max_move >= 0.10),
        'first_sample': samples[0] if samples else None,
        'final_sample': samples[-1] if samples else None,
        'samples': samples,
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


def _first_state_with(states: list[dict[str, Any]], predicate) -> dict[str, Any] | None:
    return next((row for row in states if predicate(row)), None)


def _topology_rejection_summary(topology: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(topology, dict):
        return {'available': False, 'reason_counts': {}}
    counts = topology.get('reject_reason_counts') if isinstance(topology.get('reject_reason_counts'), dict) else {}
    return {
        'available': True,
        'sampled_direction_count': topology.get('sampled_direction_count'),
        'raw_open_direction_count': topology.get('raw_open_direction_count'),
        'filtered_open_direction_count': topology.get('filtered_open_direction_count'),
        'candidate_branch_count': topology.get('candidate_branch_count'),
        'reason_counts': counts,
    }


def analyze(args: argparse.Namespace) -> int:
    artifact_dir = Path(args.artifact_dir)
    runtime = _safe_json_load(Path(args.runtime_evidence) if args.runtime_evidence else None, {})
    action = _safe_json_load(Path(args.action_result) if args.action_result else None, {})
    lifecycle_text = _read_text(Path(args.lifecycle_readiness) if args.lifecycle_readiness else None)
    action_info_text = _read_text(Path(args.action_info) if args.action_info else None)
    goal_pose_info_text = _read_text(Path(args.goal_pose_info) if args.goal_pose_info else None)
    cleanup_text = _read_text(Path(args.cleanup_processes_after) if args.cleanup_processes_after else None)
    launch_text = _read_text(Path(args.launch_log) if args.launch_log else None)
    preflight_text = _read_text(Path(args.preflight) if args.preflight else None)
    graph_text = _read_text(Path(args.ros_graph_snapshot) if args.ros_graph_snapshot else None) + '\n' + _read_text(Path(args.ros_graph_final) if args.ros_graph_final else None)
    explorer_states = read_explorer_state_jsonl(Path(args.explorer_state) if args.explorer_state else None)
    goal_events = read_goal_events_jsonl(Path(args.goal_events) if args.goal_events else None)

    stats = _runtime_stats(runtime if isinstance(runtime, dict) else {})
    action_success = bool(action.get('success'))
    action_result_received = bool(action.get('result_received'))
    goal_reached = bool(stats.get('final_distance_to_ingress_m') is not None and float(stats['final_distance_to_ingress_m']) <= ACCEPTANCE_RADIUS_M)
    ingress_reached = bool(action_success and action_result_received and goal_reached)

    first_gate_ready_state = _first_state_with(explorer_states, lambda row: bool(row.get('dispatch_readiness_gate_passed')))
    first_gate_not_ready_state = _first_state_with(explorer_states, lambda row: bool(row.get('dispatch_readiness_gate')) and not bool(row.get('dispatch_readiness_gate_passed')))
    first_topology_state = _first_state_with(explorer_states, lambda row: bool(row.get('last_topology_sampling_diagnostics')))
    first_dispatch_event = _first_state_with(goal_events, lambda row: row.get('event') == 'dispatch')
    outcome_events = [row for row in goal_events if row.get('event') in {'success', 'failure', 'timeout'}]
    first_outcome_event = outcome_events[0] if outcome_events else None
    topology_sampling_diagnostics = first_topology_state.get('last_topology_sampling_diagnostics') if isinstance(first_topology_state, dict) else None
    gate_payload_after_ingress = (first_gate_ready_state or first_gate_not_ready_state or {}).get('dispatch_readiness_gate')
    gate_payload_before_ingress = None
    dispatch_events = [row for row in goal_events if row.get('event') == 'dispatch']
    goal_count_violation = any(int(row.get('goal_sequence') or 0) > 1 for row in dispatch_events) or len(dispatch_events) > 1
    max_goals_limited_to_one = 'max_goals=1' in preflight_text or 'max_goals:=1' in preflight_text or '-p max_goals:=1' in preflight_text
    near_exit_fallback_disabled = 'near_exit_fallback_enabled=false' in preflight_text or '-p near_exit_fallback_enabled:=false' in preflight_text
    startup_warmup_no_dispatch_false = 'startup_warmup_no_dispatch=false' in preflight_text or '-p startup_warmup_no_dispatch:=false' in preflight_text
    forbidden_unbounded = 'GUARDRAIL_VIOLATION_UNBOUNDED_OR_SUCCESS_CLAIM' in launch_text or 'complete autonomous success' in launch_text.lower()
    cleanup_empty = cleanup_text.strip() == ''

    if forbidden_unbounded or goal_count_violation or not max_goals_limited_to_one or not near_exit_fallback_disabled:
        classification = 'GUARDRAIL_VIOLATION_UNBOUNDED_OR_SUCCESS_CLAIM'
    elif not ingress_reached:
        classification = 'INGRESS_GOAL_FAILED'
    elif first_dispatch_event is not None:
        classification = 'INGRESS_THEN_GATE_READY_FIRST_DISPATCH'
    elif first_gate_ready_state is not None and first_topology_state is not None:
        classification = 'INGRESS_THEN_GATE_READY_NO_DISPATCH_TOPOLOGY_REJECTED'
    elif first_gate_ready_state is None:
        classification = 'INGRESS_REACHED_GATE_STILL_NOT_READY'
    else:
        classification = 'INGRESS_THEN_GATE_READY_NO_DISPATCH_TOPOLOGY_REJECTED'

    summary = {
        'phase': PHASE,
        'run_id': RUN_ID,
        'classification': classification,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'active_world': ACTIVE_WORLD,
        'active_metadata': ACTIVE_METADATA,
        'ingress_waypoint_map': INGRESS_WAYPOINT_MAP,
        'acceptance_radius_m': ACCEPTANCE_RADIUS_M,
        'max_goals_limited_to_one': bool(max_goals_limited_to_one),
        'near_exit_fallback_disabled': bool(near_exit_fallback_disabled),
        'startup_warmup_no_dispatch_false': bool(startup_warmup_no_dispatch_false),
        'readiness': _readiness_summary(lifecycle_text, action_info_text, goal_pose_info_text),
        'ingress_action_result': action,
        'ingress_reached': bool(ingress_reached),
        'goal_reached_within_acceptance_radius': bool(goal_reached),
        'distance_to_ingress_m': stats.get('final_distance_to_ingress_m'),
        'robot_pose_timeline_sample_count': stats.get('pose_sample_count'),
        'robot_moved_distance_m': stats.get('robot_moved_distance_m'),
        'first_robot_pose_map': stats.get('first_pose_map'),
        'final_robot_pose_map': stats.get('final_pose_map'),
        'before_ingress_evidence': stats.get('first_sample'),
        'after_ingress_evidence': stats.get('final_sample'),
        'gate_payload_before_ingress': gate_payload_before_ingress,
        'gate_payload_after_ingress': gate_payload_after_ingress,
        'first_gate_ready_state': first_gate_ready_state,
        'first_gate_not_ready_state': first_gate_not_ready_state,
        'first_topology_state': first_topology_state,
        'topology_sampling_diagnostics': topology_sampling_diagnostics,
        'topology_rejection_summary': _topology_rejection_summary(topology_sampling_diagnostics),
        'first_dispatch_event': first_dispatch_event,
        'dispatch_events': dispatch_events,
        'dispatch_event_count': len(dispatch_events),
        'first_outcome_event': first_outcome_event,
        'outcome_events': outcome_events,
        'explorer_state_sample_count': len(explorer_states),
        'goal_events_sample_count': len(goal_events),
        'goal_count_violation': bool(goal_count_violation),
        'automatic_dispatch_entry_restored': bool(first_dispatch_event is not None),
        'complete_autonomous_success_claimed': False,
        'cleanup_empty': bool(cleanup_empty),
        'cleanup_processes_after': cleanup_text.splitlines(),
        'artifact_dir': str(artifact_dir),
        'report_scope': [
            'Phase55 only validates ingress handoff into maze_explorer dispatch-entry readiness.',
            'It does not claim autonomous exploration success.',
            'It does not require reaching the exit.',
            'If a first dispatch appears, it means automatic dispatch entry restored only.',
        ],
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
    parser.add_argument('--artifact-dir', default='log/phase55_ingress_then_maze_explorer_gated_startup_smoke')
    parser.add_argument('--output', required=True)
    parser.add_argument('--runtime-evidence')
    parser.add_argument('--action-result')
    parser.add_argument('--lifecycle-readiness')
    parser.add_argument('--action-info')
    parser.add_argument('--goal-pose-info')
    parser.add_argument('--explorer-state')
    parser.add_argument('--goal-events')
    parser.add_argument('--ros-graph-snapshot')
    parser.add_argument('--ros-graph-final')
    parser.add_argument('--launch-log')
    parser.add_argument('--preflight')
    parser.add_argument('--cleanup-processes-after')
    args = parser.parse_args(argv)
    if args.record_runtime is not None:
        return record_runtime(args)
    if args.send_goal:
        return send_goal(args)
    return analyze(args)


if __name__ == '__main__':
    raise SystemExit(main())
