#!/usr/bin/env python3
"""Phase57 First Dispatch Navigation Timeout Diagnostics recorder/analyzer.

Artifact contract documented in this tool:
* first_dispatch_execution.jsonl
* controller_dynamics.jsonl
* nav2_feedback.jsonl
* local_costmap_samples.jsonl
* global_plan_samples.jsonl
* collision_monitor_state.jsonl
* first_dispatch_execution_summary.json
* explorer_state.jsonl
* goal_events.jsonl
* controller_server/bt_navigator logs
* summary/analysis JSON
* cleanup_processes_after.txt

Phase57 diagnoses only first-dispatch Nav2/local traversability execution. It
reuses the Phase56 ingress -> maze_explorer max_goals=1 chain, does not tune
Nav2/MPPI/controller parameters, does not change maze_explorer strategy, and does
not claim autonomous exploration success; first dispatch is not exit success.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from pathlib import Path
from typing import Any

PHASE = 'Phase57 First Dispatch Navigation Timeout Diagnostics'
RUN_ID = 'phase57_first_dispatch_navigation_timeout_diagnostics'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
ACTIVE_METADATA = 'maze_20260528_scaled_instance.yaml'
INGRESS_WAYPOINT_MAP = {'x_m': 1.0, 'y_m': 0.0, 'yaw_rad': 0.0}
ACCEPTANCE_RADIUS_M = 0.35
HIGH_COST_THRESHOLD = 70
LETHAL_COST_THRESHOLD = 99
NEAR_ZERO_LINEAR = 0.01
NEAR_ZERO_ANGULAR = 0.05
ALLOWED_CLASSIFICATIONS = {
    'TIMEOUT_TARGET_IN_HIGH_LOCAL_COST',
    'TIMEOUT_FRONT_WEDGE_BLOCKED',
    'TIMEOUT_CONTROLLER_NO_CMD_VEL',
    'TIMEOUT_ROBOT_STUCK_NEAR_WALL',
    'TIMEOUT_GOAL_TOO_CLOSE_TO_OBSTACLE',
    'TIMEOUT_NAV2_EXECUTION_CAUSE_IDENTIFIED',
    'TIMEOUT_INCONCLUSIVE_DATA_GAP',
    'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
    'FIRST_DISPATCH_NAVIGATION_SUCCEEDED_BOUNDED',
}

try:  # ROS imports are optional for static tests and offline analysis.
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.msg import CollisionMonitorState
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
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
    Twist = None
    NavigateToPose = None
    CollisionMonitorState = None
    OccupancyGrid = None
    Odometry = None
    NavPath = None
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


def _read_jsonl(path: Path | None) -> list[dict[str, Any]]:
    if not path or not path.exists() or not path.stat().st_size:
        return []
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        row = _safe_json_loads(line)
        if isinstance(row, dict):
            rows.append(row)
    return rows


def read_goal_events_jsonl(path: Path | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for row in _read_jsonl(path):
        payload = row.get('state') if isinstance(row.get('state'), dict) else row
        if isinstance(payload, dict):
            payload = dict(payload)
            payload.setdefault('_recorder_elapsed_sec', row.get('elapsed_sec'))
            payload.setdefault('_recorder_wall_time', row.get('wall_time'))
            payload.setdefault('_recorder_seq', row.get('seq'))
            rows.append(payload)
    return rows


def read_explorer_state_jsonl(path: Path | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for row in _read_jsonl(path):
        payload = row.get('state') if isinstance(row.get('state'), dict) else row
        if isinstance(payload, dict):
            payload = dict(payload)
            payload.setdefault('_recorder_elapsed_sec', row.get('elapsed_sec'))
            payload.setdefault('_recorder_wall_time', row.get('wall_time'))
            payload.setdefault('_recorder_seq', row.get('seq'))
            rows.append(payload)
    return rows


def read_execution_jsonl(path: Path | None) -> list[dict[str, Any]]:
    return _read_jsonl(path)


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _quat_yaw(q: Any) -> float:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _duration_seconds(value: Any) -> float:
    if value is None:
        return 0.0
    return float(getattr(value, 'sec', 0)) + float(getattr(value, 'nanosec', 0)) / 1e9


def _number(value: Any) -> float | None:
    if value is None:
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _stats(values: list[float]) -> dict[str, Any]:
    values = [float(v) for v in values if math.isfinite(float(v))]
    if not values:
        return {'count': 0, 'min': None, 'max': None, 'mean': None, 'final': None}
    return {
        'count': len(values),
        'min': min(values),
        'max': max(values),
        'mean': float(sum(values) / len(values)),
        'final': values[-1],
    }


class CostmapView:
    def __init__(self, msg: Any) -> None:
        self.msg = msg
        self.width = int(msg.info.width)
        self.height = int(msg.info.height)
        self.resolution = float(msg.info.resolution)
        self.origin_x = float(msg.info.origin.position.x)
        self.origin_y = float(msg.info.origin.position.y)
        self.data = [int(v) for v in msg.data]
        self.stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
        self.frame_id = str(msg.header.frame_id)

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (int(math.floor((x - self.origin_x) / self.resolution)), int(math.floor((y - self.origin_y) / self.resolution)))

    def in_bounds(self, cell: tuple[int, int]) -> bool:
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def cell_value(self, cell: tuple[int, int]) -> int | None:
        if not self.in_bounds(cell):
            return None
        return int(self.data[cell[1] * self.width + cell[0]])

    def value_at(self, x: float, y: float) -> int | None:
        return self.cell_value(self.world_to_cell(x, y))

    def radius_values(self, xy: tuple[float, float], radius_m: float) -> list[int]:
        cx, cy = self.world_to_cell(xy[0], xy[1])
        radius_cells = int(math.ceil(radius_m / max(self.resolution, 1e-9)))
        values: list[int] = []
        for yy in range(cy - radius_cells, cy + radius_cells + 1):
            for xx in range(cx - radius_cells, cx + radius_cells + 1):
                wx = self.origin_x + (xx + 0.5) * self.resolution
                wy = self.origin_y + (yy + 0.5) * self.resolution
                if math.hypot(wx - xy[0], wy - xy[1]) > radius_m + 1e-9:
                    continue
                value = self.cell_value((xx, yy))
                if value is not None:
                    values.append(value)
        return values

    def line_samples(self, start: tuple[float, float], end: tuple[float, float]) -> list[dict[str, Any]]:
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        step = max(self.resolution * 0.5, 0.05)
        steps = max(1, int(math.ceil(distance / step)))
        samples: list[dict[str, Any]] = []
        for index in range(steps + 1):
            ratio = index / steps
            x = start[0] + (end[0] - start[0]) * ratio
            y = start[1] + (end[1] - start[1]) * ratio
            cell = self.world_to_cell(x, y)
            value = self.cell_value(cell)
            samples.append({
                'point': [float(x), float(y)],
                'cell': list(cell),
                'in_bounds': bool(self.in_bounds(cell)),
                'value': value,
                'distance_m': float(distance * ratio),
            })
        return samples


def _cost_values_summary(values: list[int | None]) -> dict[str, Any]:
    clean = [int(v) for v in values if v is not None]
    if not clean:
        return {'sample_count': len(values), 'in_bounds_sample_count': 0, 'max': None, 'mean': None, 'high_cost_count': 0, 'lethal_count': 0}
    return {
        'sample_count': len(values),
        'in_bounds_sample_count': len(clean),
        'max': max(clean),
        'mean': float(sum(clean) / len(clean)),
        'high_cost_count': sum(1 for v in clean if v >= HIGH_COST_THRESHOLD),
        'lethal_count': sum(1 for v in clean if v >= LETHAL_COST_THRESHOLD),
    }


def _path_summary_from_points(points: list[tuple[float, float]]) -> dict[str, Any]:
    if not points:
        return {'available': False, 'pose_count': 0, 'path_length_m': None}
    length = 0.0
    for a, b in zip(points, points[1:]):
        length += math.hypot(b[0] - a[0], b[1] - a[1])
    return {
        'available': True,
        'pose_count': len(points),
        'path_length_m': float(length),
        'first_pose': list(points[0]),
        'last_pose': list(points[-1]),
    }


def _target_evidence(costmap: CostmapView | None, target: tuple[float, float] | None) -> dict[str, Any]:
    if costmap is None or target is None:
        return {'available': False}
    cell = costmap.world_to_cell(target[0], target[1])
    radius_values = costmap.radius_values(target, 0.25)
    return {
        'available': True,
        'target': list(target),
        'cell': list(cell),
        'in_bounds': bool(costmap.in_bounds(cell)),
        'value': costmap.cell_value(cell),
        'radius_m': 0.25,
        'radius_cost_summary': _cost_values_summary(radius_values),
    }


def _path_cost_evidence(costmap: CostmapView | None, start: tuple[float, float] | None, target: tuple[float, float] | None) -> dict[str, Any]:
    if costmap is None or start is None or target is None:
        return {'available': False}
    samples = costmap.line_samples(start, target)
    values = [row.get('value') for row in samples]
    first_high = next((row for row in samples if row.get('value') is not None and int(row['value']) >= HIGH_COST_THRESHOLD), None)
    return {
        'available': True,
        'start': list(start),
        'target': list(target),
        'cost_summary': _cost_values_summary(values),
        'first_high_cost_distance_m': first_high.get('distance_m') if first_high else None,
        'first_high_cost_point': first_high.get('point') if first_high else None,
    }


def _footprint_values(costmap: CostmapView | None, pose: tuple[float, float, float] | None, length_m: float = 0.60, width_m: float = 0.50) -> list[int]:
    if costmap is None or pose is None:
        return []
    x, y, yaw = pose
    values: list[int] = []
    step = max(costmap.resolution, 0.05)
    half_l = length_m / 2.0
    half_w = width_m / 2.0
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    nx = int(math.ceil(length_m / step))
    ny = int(math.ceil(width_m / step))
    for i in range(-nx, nx + 1):
        local_x = half_l * i / max(nx, 1)
        for j in range(-ny, ny + 1):
            local_y = half_w * j / max(ny, 1)
            wx = x + local_x * cos_y - local_y * sin_y
            wy = y + local_x * sin_y + local_y * cos_y
            value = costmap.value_at(wx, wy)
            if value is not None:
                values.append(value)
    return values


def _front_wedge_values(costmap: CostmapView | None, pose: tuple[float, float, float] | None, radius_m: float = 0.75, half_angle_rad: float = math.radians(35.0)) -> tuple[list[int], float | None]:
    if costmap is None or pose is None:
        return [], None
    x, y, yaw = pose
    values: list[int] = []
    nearest_high: float | None = None
    cell_radius = int(math.ceil(radius_m / max(costmap.resolution, 1e-9)))
    cx, cy = costmap.world_to_cell(x, y)
    for yy in range(cy - cell_radius, cy + cell_radius + 1):
        for xx in range(cx - cell_radius, cx + cell_radius + 1):
            value = costmap.cell_value((xx, yy))
            if value is None:
                continue
            wx = costmap.origin_x + (xx + 0.5) * costmap.resolution
            wy = costmap.origin_y + (yy + 0.5) * costmap.resolution
            dx = wx - x
            dy = wy - y
            dist = math.hypot(dx, dy)
            if dist > radius_m or dist <= 1e-9:
                continue
            angle = math.atan2(dy, dx)
            delta = math.atan2(math.sin(angle - yaw), math.cos(angle - yaw))
            if abs(delta) <= half_angle_rad:
                values.append(value)
                if value >= HIGH_COST_THRESHOLD:
                    nearest_high = dist if nearest_high is None else min(nearest_high, dist)
    return values, nearest_high


def _pose_from_odom_msg(msg: Any) -> tuple[float, float, float]:
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    return (float(pos.x), float(pos.y), _quat_yaw(ori))


class FirstDispatchExecutionRecorder(Node):  # pragma: no cover - requires ROS graph
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('phase57_first_dispatch_execution_recorder')
        self.args = args
        self.started_at = time.time()
        self.done = False
        self.active_sequence: int | None = None
        self.dispatch_event: dict[str, Any] | None = None
        self.outcome_event: dict[str, Any] | None = None
        self.latest_local_costmap: CostmapView | None = None
        self.latest_global_costmap: CostmapView | None = None
        self.latest_pose: tuple[float, float, float] | None = None
        self.latest_plan_points: list[tuple[float, float]] = []
        self.latest_cmd: dict[str, Any] | None = None
        self.last_periodic_snapshot = 0.0
        self.paths = {
            'execution': Path(args.execution_output),
            'controller': Path(args.controller_dynamics_output),
            'feedback': Path(args.nav2_feedback_output),
            'local_costmap': Path(args.local_costmap_samples_output),
            'global_plan': Path(args.global_plan_samples_output),
            'collision': Path(args.collision_monitor_output),
        }
        for path in self.paths.values():
            path.parent.mkdir(parents=True, exist_ok=True)
        self.fps = {name: path.open('w', encoding='utf-8') for name, path in self.paths.items()}
        live_qos = QoSProfile(depth=50)
        self.create_subscription(String, args.goal_events_topic, self._on_goal_event, live_qos)
        self.create_subscription(OccupancyGrid, args.local_costmap_topic, self._on_local_costmap, live_qos)
        self.create_subscription(OccupancyGrid, args.global_costmap_topic, self._on_global_costmap, live_qos)
        self.create_subscription(NavPath, args.path_topic, self._on_plan, live_qos)
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, live_qos)
        self.create_subscription(Twist, args.cmd_topic, self._on_cmd, live_qos)
        self.create_subscription(Twist, args.cmd_smoothed_topic, self._on_cmd_smoothed, live_qos)
        self.create_subscription(CollisionMonitorState, args.collision_monitor_topic, self._on_collision_state, live_qos)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage, args.nav2_feedback_topic, self._on_nav2_feedback, live_qos)
        self.create_timer(0.2, self._on_timer)
        self.get_logger().info('Phase57 read-only first dispatch execution recorder started')

    def _write(self, stream: str, row: dict[str, Any]) -> None:
        row.setdefault('wall_time', time.time())
        row['elapsed_sec'] = row['wall_time'] - self.started_at
        self.fps[stream].write(json.dumps(row, sort_keys=True) + '\n')
        self.fps[stream].flush()

    def _on_goal_event(self, msg: Any) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        event = payload.get('event')
        seq = payload.get('goal_sequence')
        if seq is not None:
            seq = int(seq)
        if event == 'dispatch' and self.active_sequence is None and seq is not None:
            self.active_sequence = seq
            self.dispatch_event = payload
            self._write('execution', {'event': 'goal_event', 'snapshot_type': 'dispatch', 'goal_sequence': seq, 'payload': payload})
            self._emit_snapshot('dispatch_snapshot')
        elif event in {'success', 'failure', 'timeout'} and seq is not None and seq == self.active_sequence:
            self.outcome_event = payload
            self._write('execution', {'event': 'goal_event', 'snapshot_type': event, 'goal_sequence': seq, 'payload': payload})
            self._emit_snapshot(f'{event}_snapshot')
            self.done = True

    def _on_local_costmap(self, msg: Any) -> None:
        self.latest_local_costmap = CostmapView(msg)
        if self.active_sequence is not None:
            target = self._target()
            pose_xy = (self.latest_pose[0], self.latest_pose[1]) if self.latest_pose else None
            self._write('local_costmap', {
                'event': 'local_costmap_sample',
                'goal_sequence': self.active_sequence,
                'dispatch_target': list(target) if target else None,
                'robot_pose': list(self.latest_pose) if self.latest_pose else None,
                'target_evidence': _target_evidence(self.latest_local_costmap, target),
                'path_evidence': _path_cost_evidence(self.latest_local_costmap, pose_xy, target),
                'footprint_cost': _cost_values_summary(_footprint_values(self.latest_local_costmap, self.latest_pose)),
                'front_wedge_cost': _cost_values_summary(_front_wedge_values(self.latest_local_costmap, self.latest_pose)[0]),
                'front_wedge_clearance_m': _front_wedge_values(self.latest_local_costmap, self.latest_pose)[1],
            })

    def _on_global_costmap(self, msg: Any) -> None:
        self.latest_global_costmap = CostmapView(msg)

    def _on_plan(self, msg: Any) -> None:
        self.latest_plan_points = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
        if self.active_sequence is not None:
            row = {'event': 'global_plan_update', 'goal_sequence': self.active_sequence, 'global_plan_summary': _path_summary_from_points(self.latest_plan_points)}
            self._write('execution', row)
            self._write('global_plan', row)

    def _on_odom(self, msg: Any) -> None:
        self.latest_pose = _pose_from_odom_msg(msg)
        pos = msg.pose.pose.position
        tw = msg.twist.twist
        self._write('controller', {
            'source': 'odom',
            'x': float(pos.x),
            'y': float(pos.y),
            'yaw': self.latest_pose[2],
            'odom_linear_x': float(tw.linear.x),
            'odom_linear_y': float(tw.linear.y),
            'odom_angular_z': float(tw.angular.z),
            'goal_sequence': self.active_sequence,
        })

    def _on_cmd(self, msg: Any) -> None:
        self._write_cmd('cmd_vel', msg)

    def _on_cmd_smoothed(self, msg: Any) -> None:
        self._write_cmd('cmd_vel_smoothed', msg)

    def _write_cmd(self, source: str, msg: Any) -> None:
        row = {
            'source': source,
            'linear_x': float(msg.linear.x),
            'linear_y': float(msg.linear.y),
            'linear_z': float(msg.linear.z),
            'angular_x': float(msg.angular.x),
            'angular_y': float(msg.angular.y),
            'angular_z': float(msg.angular.z),
            'goal_sequence': self.active_sequence,
        }
        self.latest_cmd = row
        self._write('controller', row)

    def _on_collision_state(self, msg: Any) -> None:
        row = {
            'event': 'collision_monitor_state',
            'goal_sequence': self.active_sequence,
            'action_type': int(getattr(msg, 'action_type', -1)),
            'polygon_name': str(getattr(msg, 'polygon_name', '')),
        }
        self._write('collision', row)

    def _on_nav2_feedback(self, msg: Any) -> None:
        feedback = msg.feedback
        row = {
            'event': 'nav2_feedback',
            'goal_sequence': self.active_sequence,
            'distance_remaining': float(getattr(feedback, 'distance_remaining', math.nan)),
            'navigation_time_sec': _duration_seconds(getattr(feedback, 'navigation_time', None)),
            'estimated_time_remaining_sec': _duration_seconds(getattr(feedback, 'estimated_time_remaining', None)),
            'number_of_recoveries': int(getattr(feedback, 'number_of_recoveries', 0)),
        }
        self._write('feedback', row)

    def _on_timer(self) -> None:
        now = time.time()
        if self.active_sequence is not None and now - self.last_periodic_snapshot >= self.args.periodic_snapshot_sec:
            self.last_periodic_snapshot = now
            self._emit_snapshot('periodic_active_goal')
        if now - self.started_at >= self.args.timeout_sec:
            self._emit_snapshot('recorder_timeout_snapshot')
            self.done = True

    def _target(self) -> tuple[float, float] | None:
        if not self.dispatch_event:
            return None
        target = self.dispatch_event.get('target')
        if isinstance(target, (list, tuple)) and len(target) >= 2:
            return (float(target[0]), float(target[1]))
        return None

    def _emit_snapshot(self, snapshot_type: str) -> None:
        target = self._target()
        pose = self.latest_pose
        pose_xy = (pose[0], pose[1]) if pose else None
        footprint_values = _footprint_values(self.latest_local_costmap, pose)
        wedge_values, wedge_clearance = _front_wedge_values(self.latest_local_costmap, pose)
        row = {
            'event': 'snapshot',
            'snapshot_type': snapshot_type,
            'goal_sequence': self.active_sequence,
            'dispatch_target': list(target) if target else None,
            'robot_pose': list(pose) if pose else None,
            'global_plan_summary': _path_summary_from_points(self.latest_plan_points),
            'local_costmap_target_evidence': _target_evidence(self.latest_local_costmap, target),
            'local_costmap_path_evidence': _path_cost_evidence(self.latest_local_costmap, pose_xy, target),
            'global_costmap_target_evidence': _target_evidence(self.latest_global_costmap, target),
            'footprint_cost': _cost_values_summary(footprint_values),
            'front_wedge_cost': _cost_values_summary(wedge_values),
            'front_wedge_clearance_m': wedge_clearance,
            'last_cmd': self.latest_cmd,
        }
        self._write('execution', row)

    def close(self) -> None:
        for fp in self.fps.values():
            fp.close()


def record_execution(args: argparse.Namespace) -> int:  # pragma: no cover - requires ROS graph
    if rclpy is None:
        raise SystemExit('rclpy is required for --record-execution')
    rclpy.init()
    node = FirstDispatchExecutionRecorder(args)
    try:
        while rclpy.ok() and not node.done:
            try:
                rclpy.spin_once(node, timeout_sec=0.2)
            except rclpy.executors.ExternalShutdownException:
                break
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


class IngressGoalClient(Node):  # pragma: no cover - requires ROS graph
    def __init__(self, output: Path, goal_timeout_sec: float):
        super().__init__('phase57_ingress_goal_client')
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
            'navigation_time_sec': _duration_seconds(getattr(feedback, 'navigation_time', None)),
            'distance_remaining': float(getattr(feedback, 'distance_remaining', math.nan)),
            'number_of_recoveries': int(getattr(feedback, 'number_of_recoveries', 0)),
        })


def send_ingress_goal(args: argparse.Namespace) -> int:  # pragma: no cover - requires ROS graph
    if rclpy is None:
        raise SystemExit('rclpy is required for --send-ingress-goal')
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


def _first_goal_event(goal_events: list[dict[str, Any]], event: str) -> dict[str, Any] | None:
    return next((row for row in goal_events if row.get('event') == event), None)


def _execution_snapshots(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [row for row in rows if row.get('event') == 'snapshot']


def _goal_window_times(goal_events: list[dict[str, Any]]) -> tuple[float | None, float | None]:
    dispatch = _first_goal_event(goal_events, 'dispatch')
    outcomes = [row for row in goal_events if row.get('event') in {'success', 'failure', 'timeout'}]
    start = _number((dispatch or {}).get('_recorder_wall_time') or (dispatch or {}).get('wall_time'))
    end = _number((outcomes[0] if outcomes else {}).get('_recorder_wall_time') or (outcomes[0] if outcomes else {}).get('wall_time')) if outcomes else None
    return start, end


def _rows_in_goal_window(rows: list[dict[str, Any]], start: float | None, end: float | None) -> list[dict[str, Any]]:
    out = []
    for row in rows:
        wall = _number(row.get('wall_time'))
        if wall is None:
            out.append(row)
            continue
        if start is not None and wall < start - 1.0:
            continue
        if end is not None and wall > end + 1.0:
            continue
        out.append(row)
    return out


def _cmd_vel_timeline_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    cmd_rows = [row for row in rows if str(row.get('source')) in {'cmd_vel', 'cmd_vel_smoothed', 'cmd_vel_nav'} or 'linear_x' in row]
    linear = [_number(row.get('linear_x')) or 0.0 for row in cmd_rows]
    angular = [_number(row.get('angular_z')) or 0.0 for row in cmd_rows]
    moving = [abs(l) > NEAR_ZERO_LINEAR or abs(a) > NEAR_ZERO_ANGULAR for l, a in zip(linear, angular)]
    by_source: dict[str, int] = {}
    for row in cmd_rows:
        by_source[str(row.get('source', 'cmd_vel'))] = by_source.get(str(row.get('source', 'cmd_vel')), 0) + 1
    return {
        'cmd_vel_sample_count': len(cmd_rows),
        'samples_by_source': by_source,
        'linear_x_abs': _stats([abs(v) for v in linear]),
        'angular_z_abs': _stats([abs(v) for v in angular]),
        'nonzero_command_count': sum(1 for flag in moving if flag),
        'near_zero_command_ratio': (sum(1 for flag in moving if not flag) / len(moving)) if moving else None,
        'controller_no_cmd_vel': bool(len(cmd_rows) == 0 or sum(1 for flag in moving if flag) == 0),
    }


def _pose_progress_summary(rows: list[dict[str, Any]], dispatch: dict[str, Any] | None, outcome: dict[str, Any] | None) -> dict[str, Any]:
    odom_rows = [row for row in rows if row.get('source') == 'odom' and _number(row.get('x')) is not None and _number(row.get('y')) is not None]
    target = (dispatch or {}).get('target')
    if not (isinstance(target, (list, tuple)) and len(target) >= 2):
        target = None
    distances: list[float] = []
    poses: list[list[float]] = []
    for row in odom_rows:
        x = float(row.get('x'))
        y = float(row.get('y'))
        poses.append([x, y, float(row.get('yaw', 0.0))])
        if target:
            distances.append(math.hypot(x - float(target[0]), y - float(target[1])))
    total_motion = 0.0
    for a, b in zip(poses, poses[1:]):
        total_motion += math.hypot(b[0] - a[0], b[1] - a[1])
    improvement = None
    if distances:
        improvement = distances[0] - distances[-1]
    timeout_wedge = _number((outcome or {}).get('timeout_front_wedge_clearance_m'))
    return {
        'pose_sample_count': len(odom_rows),
        'first_pose': poses[0] if poses else None,
        'final_pose': poses[-1] if poses else None,
        'total_motion_m': total_motion,
        'distance_to_target': _stats(distances),
        'distance_to_target_improvement_m': improvement,
        'robot_stuck': bool(len(odom_rows) > 5 and total_motion < 0.10),
        'stuck_near_wall_hint': bool(len(odom_rows) > 5 and total_motion < 0.10 and timeout_wedge is not None and timeout_wedge < 0.20),
    }


def _nav2_feedback_distance_remaining_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    feedback_rows = [row for row in rows if row.get('event') == 'nav2_feedback']
    distances = [_number(row.get('distance_remaining')) for row in feedback_rows]
    distances_clean = [v for v in distances if v is not None]
    return {
        'feedback_sample_count': len(feedback_rows),
        'distance_remaining': _stats(distances_clean),
        'distance_remaining_improvement_m': (distances_clean[0] - distances_clean[-1]) if distances_clean else None,
        'number_of_recoveries_max': max([int(row.get('number_of_recoveries') or 0) for row in feedback_rows], default=0),
    }


def _collision_monitor_status_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    states = [row for row in rows if row.get('event') == 'collision_monitor_state']
    action_counts: dict[str, int] = {}
    polygon_counts: dict[str, int] = {}
    for row in states:
        action = str(row.get('action_type'))
        polygon = str(row.get('polygon_name') or '')
        action_counts[action] = action_counts.get(action, 0) + 1
        if polygon:
            polygon_counts[polygon] = polygon_counts.get(polygon, 0) + 1
    return {
        'sample_count': len(states),
        'action_type_counts': action_counts,
        'polygon_name_counts': polygon_counts,
        'latest': states[-1] if states else None,
    }


def _timeline_from_snapshots(snapshots: list[dict[str, Any]], field: str) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for row in snapshots:
        value = row.get(field)
        if value is not None:
            out.append({'elapsed_sec': row.get('elapsed_sec'), 'snapshot_type': row.get('snapshot_type'), field: value})
    return out


def _front_wedge_summary(snapshots: list[dict[str, Any]], outcome: dict[str, Any] | None) -> dict[str, Any]:
    clearances = [_number(row.get('front_wedge_clearance_m')) for row in snapshots]
    clearances = [v for v in clearances if v is not None]
    max_values = [_number((row.get('front_wedge_cost') or {}).get('max')) for row in snapshots if isinstance(row.get('front_wedge_cost'), dict)]
    max_values = [v for v in max_values if v is not None]
    outcome_clearance = _number((outcome or {}).get('timeout_front_wedge_clearance_m'))
    outcome_cost = _number((outcome or {}).get('timeout_front_wedge_cost_max'))
    return {
        'sample_count': len(snapshots),
        'clearance_m': _stats(clearances),
        'cost_max': _stats(max_values),
        'outcome_timeout_front_wedge_clearance_m': outcome_clearance,
        'outcome_timeout_front_wedge_cost_max': outcome_cost,
        'front_wedge_blocked': bool((outcome_clearance is not None and outcome_clearance < 0.15) or (outcome_cost is not None and outcome_cost >= LETHAL_COST_THRESHOLD)),
    }


def _footprint_summary(snapshots: list[dict[str, Any]], outcome: dict[str, Any] | None) -> dict[str, Any]:
    max_values = [_number((row.get('footprint_cost') or {}).get('max')) for row in snapshots if isinstance(row.get('footprint_cost'), dict)]
    max_values = [v for v in max_values if v is not None]
    outcome_max = _number((outcome or {}).get('timeout_footprint_cost_max'))
    outcome_lethal = _number((outcome or {}).get('timeout_footprint_lethal_cell_count'))
    return {
        'sample_count': len(snapshots),
        'cost_max': _stats(max_values),
        'outcome_timeout_footprint_cost_max': outcome_max,
        'outcome_timeout_footprint_lethal_cell_count': outcome_lethal,
        'footprint_high_cost': bool((outcome_max is not None and outcome_max >= LETHAL_COST_THRESHOLD) or (outcome_lethal is not None and outcome_lethal > 0)),
    }


def _local_costmap_evidence_from_snapshots(snapshots: list[dict[str, Any]]) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    target_rows = [row.get('local_costmap_target_evidence') for row in snapshots if isinstance(row.get('local_costmap_target_evidence'), dict) and row['local_costmap_target_evidence'].get('available')]
    path_rows = [row.get('local_costmap_path_evidence') for row in snapshots if isinstance(row.get('local_costmap_path_evidence'), dict) and row['local_costmap_path_evidence'].get('available')]
    plan_rows = [row.get('global_plan_summary') for row in snapshots if isinstance(row.get('global_plan_summary'), dict) and row['global_plan_summary'].get('available')]
    target_maxes = [_number(((row.get('radius_cost_summary') or {}).get('max'))) for row in target_rows]
    path_maxes = [_number(((row.get('cost_summary') or {}).get('max'))) for row in path_rows]
    return (
        {
            'available_sample_count': len(target_rows),
            'latest': target_rows[-1] if target_rows else None,
            'target_radius_cost_max': _stats([v for v in target_maxes if v is not None]),
        },
        {
            'available_sample_count': len(path_rows),
            'latest': path_rows[-1] if path_rows else None,
            'path_cost_max': _stats([v for v in path_maxes if v is not None]),
        },
        {'available_sample_count': len(plan_rows), 'latest': plan_rows[-1] if plan_rows else None},
    )


def _log_summary(text: str, component: str) -> dict[str, Any]:
    lines = [line for line in text.splitlines() if component in line or component.replace('_', ' ') in line.lower()]
    patterns = {
        'failed_to_make_progress': 'Failed to make progress',
        'goal_failed': 'failed',
        'abort': 'abort',
        'collision': 'collision',
        'costmap': 'costmap',
    }
    lower_lines = [line.lower() for line in lines]
    counts = {name: sum(1 for line in lower_lines if pattern.lower() in line) for name, pattern in patterns.items()}
    return {'component': component, 'line_count': len(lines), 'pattern_counts': counts, 'tail': lines[-20:]}


def classify_first_dispatch_execution(summary: dict[str, Any]) -> str:
    if summary.get('guardrail_violation'):
        return 'GUARDRAIL_VIOLATION_STRATEGY_CHANGED'
    outcome = summary.get('goal_outcome') or {}
    if outcome.get('event') == 'success':
        return 'FIRST_DISPATCH_NAVIGATION_SUCCEEDED_BOUNDED'
    if outcome.get('event') != 'timeout':
        return 'TIMEOUT_INCONCLUSIVE_DATA_GAP'
    dispatch = summary.get('first_dispatch_event') or {}
    target_cost = _number(dispatch.get('dispatch_target_local_cost'))
    target_radius = _number(dispatch.get('dispatch_target_local_cost_max_radius'))
    target_clearance = _number(dispatch.get('target_clearance_m'))
    front = summary.get('front_wedge_clearance_timeline') or {}
    footprint = summary.get('footprint_cost_timeline') or {}
    cmd = summary.get('cmd_vel_timeline_summary') or {}
    pose = summary.get('pose_progress_summary') or {}
    if target_radius is not None and target_radius >= LETHAL_COST_THRESHOLD:
        return 'TIMEOUT_TARGET_IN_HIGH_LOCAL_COST'
    if target_cost is not None and target_cost >= HIGH_COST_THRESHOLD:
        return 'TIMEOUT_TARGET_IN_HIGH_LOCAL_COST'
    if target_clearance is not None and target_clearance < 0.25:
        return 'TIMEOUT_GOAL_TOO_CLOSE_TO_OBSTACLE'
    if front.get('front_wedge_blocked'):
        return 'TIMEOUT_FRONT_WEDGE_BLOCKED'
    if cmd.get('controller_no_cmd_vel'):
        return 'TIMEOUT_CONTROLLER_NO_CMD_VEL'
    if pose.get('stuck_near_wall_hint') or (pose.get('robot_stuck') and footprint.get('footprint_high_cost')):
        return 'TIMEOUT_ROBOT_STUCK_NEAR_WALL'
    if footprint.get('footprint_high_cost') or front.get('front_wedge_blocked'):
        return 'TIMEOUT_NAV2_EXECUTION_CAUSE_IDENTIFIED'
    return 'TIMEOUT_INCONCLUSIVE_DATA_GAP'


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
    explorer_states = read_explorer_state_jsonl(Path(args.explorer_state) if args.explorer_state else None)
    goal_events = read_goal_events_jsonl(Path(args.goal_events) if args.goal_events else None)
    execution_rows = read_execution_jsonl(Path(args.first_dispatch_execution) if args.first_dispatch_execution else None)
    controller_rows = _read_jsonl(Path(args.controller_dynamics) if args.controller_dynamics else None)
    nav2_feedback_rows = _read_jsonl(Path(args.nav2_feedback) if args.nav2_feedback else None)
    collision_rows = _read_jsonl(Path(args.collision_monitor) if args.collision_monitor else None)

    start_wall, end_wall = _goal_window_times(goal_events)
    controller_window = _rows_in_goal_window(controller_rows, start_wall, end_wall)
    feedback_window = _rows_in_goal_window(nav2_feedback_rows, start_wall, end_wall)
    collision_window = _rows_in_goal_window(collision_rows, start_wall, end_wall)
    snapshots = _execution_snapshots(execution_rows)

    first_dispatch_event = _first_goal_event(goal_events, 'dispatch')
    outcome_events = [row for row in goal_events if row.get('event') in {'success', 'failure', 'timeout'}]
    first_outcome_event = outcome_events[0] if outcome_events else None
    first_topology_state = next((row for row in explorer_states if row.get('last_topology_sampling_diagnostics')), None)
    first_execution_summary = None
    target_evidence, path_evidence, global_plan_summary = _local_costmap_evidence_from_snapshots(snapshots)
    cmd_vel_summary = _cmd_vel_timeline_summary(controller_window)
    pose_summary = _pose_progress_summary(controller_window, first_dispatch_event, first_outcome_event)
    feedback_summary = _nav2_feedback_distance_remaining_summary(feedback_window)
    collision_summary = _collision_monitor_status_summary(collision_window)
    footprint_timeline = _footprint_summary(snapshots, first_outcome_event)
    front_wedge_timeline = _front_wedge_summary(snapshots, first_outcome_event)

    max_goals_limited_to_one = 'max_goals=1' in preflight_text or 'max_goals:=1' in preflight_text or '-p max_goals:=1' in preflight_text
    near_exit_fallback_disabled = 'near_exit_fallback_enabled=false' in preflight_text or '-p near_exit_fallback_enabled:=false' in preflight_text
    startup_warmup_no_dispatch_false = 'startup_warmup_no_dispatch=false' in preflight_text or '-p startup_warmup_no_dispatch:=false' in preflight_text
    goal_count_violation = len([row for row in goal_events if row.get('event') == 'dispatch']) > 1
    forbidden_unbounded = 'complete autonomous success' in launch_text.lower() or 'PHASE58' in launch_text
    cleanup_empty = cleanup_text.strip() == ''
    nav2_config_diff_empty = True
    if args.nav2_config_diff:
        nav2_config_diff_empty = _read_text(Path(args.nav2_config_diff)).strip() == ''
    guardrail_violation = bool(
        goal_count_violation
        or not max_goals_limited_to_one
        or not near_exit_fallback_disabled
        or not startup_warmup_no_dispatch_false
        or forbidden_unbounded
        or not nav2_config_diff_empty
    )

    summary: dict[str, Any] = {
        'phase': PHASE,
        'run_id': RUN_ID,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'active_world': ACTIVE_WORLD,
        'active_metadata': ACTIVE_METADATA,
        'ingress_waypoint_map': INGRESS_WAYPOINT_MAP,
        'acceptance_radius_m': ACCEPTANCE_RADIUS_M,
        'readiness': _readiness_summary(lifecycle_text, action_info_text, goal_pose_info_text),
        'ingress_action_result': action,
        'ingress_reached': bool(action.get('success') and action.get('result_received') and (action.get('status_text') == 'STATUS_SUCCEEDED' or int(action.get('error_code') or -1) == 0)),
        'max_goals_limited_to_one': bool(max_goals_limited_to_one),
        'near_exit_fallback_disabled': bool(near_exit_fallback_disabled),
        'startup_warmup_no_dispatch_false': bool(startup_warmup_no_dispatch_false),
        'first_topology_state': first_topology_state,
        'first_dispatch_event': first_dispatch_event,
        'dispatch_target': (first_dispatch_event or {}).get('target'),
        'dispatch_target_local_cost': (first_dispatch_event or {}).get('dispatch_target_local_cost'),
        'dispatch_target_local_cost_max_radius': (first_dispatch_event or {}).get('dispatch_target_local_cost_max_radius'),
        'goal_outcome': first_outcome_event,
        'timeout_reason': (first_outcome_event or {}).get('result_reason'),
        'outcome_events': outcome_events,
        'goal_events_sample_count': len(goal_events),
        'explorer_state_sample_count': len(explorer_states),
        'first_dispatch_execution_sample_count': len(execution_rows),
        'goal_window_wall_time': {'start': start_wall, 'end': end_wall},
        'global_plan_summary': global_plan_summary,
        'local_costmap_target_evidence': target_evidence,
        'local_costmap_path_evidence': path_evidence,
        'footprint_cost_timeline': footprint_timeline,
        'front_wedge_clearance_timeline': front_wedge_timeline,
        'cmd_vel_timeline_summary': cmd_vel_summary,
        'pose_progress_summary': pose_summary,
        'nav2_feedback_distance_remaining_summary': feedback_summary,
        'collision_monitor_status_summary': collision_summary,
        'controller_server_log_summary': _log_summary(launch_text, 'controller_server'),
        'bt_navigator_log_summary': _log_summary(launch_text, 'bt_navigator'),
        'guardrail_violation': bool(guardrail_violation),
        'goal_count_violation': bool(goal_count_violation),
        'nav2_config_diff_empty': bool(nav2_config_diff_empty),
        'cleanup_empty': bool(cleanup_empty),
        'cleanup_processes_after': cleanup_text.splitlines(),
        'artifact_dir': str(artifact_dir),
        'complete_autonomous_success_claimed': False,
        'report_scope': [
            'Phase57 diagnoses first-dispatch Nav2/local traversability execution only.',
            'It performs no Nav2/MPPI/controller parameter edits.',
            'It does not change maze_explorer strategy.',
            'It does not claim autonomous exploration success; first dispatch is not exit success.',
        ],
    }
    summary['classification'] = classify_first_dispatch_execution(summary)
    first_execution_summary = {
        'classification': summary['classification'],
        'dispatch_target': summary['dispatch_target'],
        'goal_outcome': summary['goal_outcome'],
        'timeout_reason': summary['timeout_reason'],
        'local_costmap_target_evidence': summary['local_costmap_target_evidence'],
        'local_costmap_path_evidence': summary['local_costmap_path_evidence'],
        'footprint_cost_timeline': summary['footprint_cost_timeline'],
        'front_wedge_clearance_timeline': summary['front_wedge_clearance_timeline'],
        'cmd_vel_timeline_summary': summary['cmd_vel_timeline_summary'],
        'pose_progress_summary': summary['pose_progress_summary'],
        'nav2_feedback_distance_remaining_summary': summary['nav2_feedback_distance_remaining_summary'],
        'collision_monitor_status_summary': summary['collision_monitor_status_summary'],
    }
    if args.first_dispatch_execution_summary:
        path = Path(args.first_dispatch_execution_summary)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(first_execution_summary, indent=2, sort_keys=True), encoding='utf-8')
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    print(json.dumps(summary, sort_keys=True))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=PHASE)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--record-execution', action='store_true', help='record first-dispatch execution diagnostics')
    mode.add_argument('--send-ingress-goal', action='store_true', help='send the single ingress NavigateToPose goal')
    mode.add_argument('--analyze', action='store_true', help='analyze collected artifacts')
    parser.add_argument('--timeout-sec', type=float, default=140.0)
    parser.add_argument('--periodic-snapshot-sec', type=float, default=1.0)
    parser.add_argument('--goal-timeout-sec', type=float, default=90.0)
    parser.add_argument('--artifact-dir', default='log/phase57_first_dispatch_navigation_timeout_diagnostics')
    parser.add_argument('--output', required=True)
    parser.add_argument('--execution-output', default='log/phase57_first_dispatch_navigation_timeout_diagnostics/first_dispatch_execution.jsonl')
    parser.add_argument('--controller-dynamics-output', default='log/phase57_first_dispatch_navigation_timeout_diagnostics/controller_dynamics.jsonl')
    parser.add_argument('--nav2-feedback-output', default='log/phase57_first_dispatch_navigation_timeout_diagnostics/nav2_feedback.jsonl')
    parser.add_argument('--local-costmap-samples-output', default='log/phase57_first_dispatch_navigation_timeout_diagnostics/local_costmap_samples.jsonl')
    parser.add_argument('--global-plan-samples-output', default='log/phase57_first_dispatch_navigation_timeout_diagnostics/global_plan_samples.jsonl')
    parser.add_argument('--collision-monitor-output', default='log/phase57_first_dispatch_navigation_timeout_diagnostics/collision_monitor_state.jsonl')
    parser.add_argument('--local-costmap-topic', default='/local_costmap/costmap')
    parser.add_argument('--global-costmap-topic', default='/global_costmap/costmap')
    parser.add_argument('--path-topic', default='/plan')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--cmd-topic', default='/cmd_vel')
    parser.add_argument('--cmd-smoothed-topic', default='/cmd_vel_smoothed')
    parser.add_argument('--goal-events-topic', default='/maze/goal_events')
    parser.add_argument('--collision-monitor-topic', default='/collision_monitor_state')
    parser.add_argument('--nav2-feedback-topic', default='/navigate_to_pose/_action/feedback')
    parser.add_argument('--runtime-evidence')
    parser.add_argument('--action-result')
    parser.add_argument('--lifecycle-readiness')
    parser.add_argument('--action-info')
    parser.add_argument('--goal-pose-info')
    parser.add_argument('--explorer-state')
    parser.add_argument('--goal-events')
    parser.add_argument('--first-dispatch-execution')
    parser.add_argument('--controller-dynamics')
    parser.add_argument('--nav2-feedback')
    parser.add_argument('--collision-monitor')
    parser.add_argument('--launch-log')
    parser.add_argument('--preflight')
    parser.add_argument('--cleanup-processes-after')
    parser.add_argument('--nav2-config-diff')
    parser.add_argument('--first-dispatch-execution-summary')
    args = parser.parse_args(argv)
    if args.record_execution:
        return record_execution(args)
    if args.send_ingress_goal:
        return send_ingress_goal(args)
    return analyze(args)


if __name__ == '__main__':
    raise SystemExit(main())
