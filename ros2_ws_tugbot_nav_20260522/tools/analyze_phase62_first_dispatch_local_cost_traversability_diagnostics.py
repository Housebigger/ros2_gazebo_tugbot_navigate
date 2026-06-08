#!/usr/bin/env python3
"""Phase62 First Dispatch Local Cost / Traversability Diagnostics.

This analyzer/recorder is read-only. It diagnoses the first dispatch target after
Phase61 restored dispatch. It does not tune Nav2/MPPI/controller parameters,
clearance radius, map sufficiency thresholds, branch scoring, fallback, or
terminal acceptance. First dispatch is not autonomous exploration success and not
exit success.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import statistics
import time
from pathlib import Path
from typing import Any

PHASE = 'Phase62 First Dispatch Local Cost / Traversability Diagnostics'
RUN_ID = 'phase62_first_dispatch_local_cost_traversability_diagnostics'
PHASE61_RUN_ID = 'phase61_post_ingress_single_open_candidate_exception'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
ACTIVE_METADATA = 'maze_20260528_scaled_instance.yaml'
PHASE61_CLASSIFICATION = 'SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED'
HIGH_COST_THRESHOLD = 70
LETHAL_COST_THRESHOLD = 99
NEAR_ZERO_LINEAR = 0.01
NEAR_ZERO_ANGULAR = 0.05
ALLOWED_CLASSIFICATIONS = {
    'TARGET_TOO_CLOSE_TO_WALL',
    'LOCAL_COSTMAP_INFLATION_DOMINANT',
    'CORRIDOR_TOO_NARROW',
    'SENSOR_COSTMAP_MISALIGNMENT',
    'INSUFFICIENT_EVIDENCE',
    'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
}
GUARDRAILS = [
    'bounded runtime diagnostics only',
    'max_goals=1',
    'no Nav2/MPPI/controller parameter edits',
    'no clearance_radius_m tuning',
    'no map sufficiency threshold tuning',
    'no branch selection scoring change',
    'no fallback/terminal acceptance',
    'no autonomous exploration success claim',
    'first dispatch is not exit success',
]

try:  # ROS imports are optional for offline tests/analyze.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.msg import CollisionMonitorState
    from std_msgs.msg import String
except Exception:  # pragma: no cover
    rclpy = None
    Node = object
    QoSProfile = None
    Twist = None
    OccupancyGrid = None
    Odometry = None
    NavPath = None
    NavigateToPose = None
    CollisionMonitorState = None
    String = None


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
    except Exception:
        return None


def _read_jsonl(path: Path | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path or not path.exists() or not path.stat().st_size:
        return rows
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        row = _safe_json_loads(line)
        if isinstance(row, dict):
            payload = row.get('state') if isinstance(row.get('state'), dict) else row
            if isinstance(payload, dict):
                payload = dict(payload)
                payload.setdefault('_recorder_elapsed_sec', row.get('elapsed_sec'))
                payload.setdefault('_recorder_wall_time', row.get('wall_time'))
                rows.append(payload)
    return rows


def _phase61_module() -> Any:  # pragma: no cover - requires ROS runtime.
    path = Path(__file__).resolve().parent / 'analyze_phase61_post_ingress_single_open_candidate_exception.py'
    spec = importlib.util.spec_from_file_location('phase61_runtime_helpers', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot load Phase61 runtime helper module: {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    setattr(module, 'RUN_ID', RUN_ID)
    setattr(module, 'PHASE', PHASE)
    return module


def send_ingress_goal(args: argparse.Namespace) -> int:  # pragma: no cover
    return int(_phase61_module().send_ingress_goal(args))


def _number(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _stats(values: list[float]) -> dict[str, Any]:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    if not clean:
        return {'count': 0, 'min': None, 'max': None, 'mean': None, 'final': None}
    return {'count': len(clean), 'min': min(clean), 'max': max(clean), 'mean': sum(clean) / len(clean), 'final': clean[-1]}


def _duration_seconds(value: Any) -> float | None:
    if value is None:
        return None
    return float(getattr(value, 'sec', 0)) + float(getattr(value, 'nanosec', 0)) / 1e9


def _yaw_from_quat(q: Any) -> float:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class CostmapView:
    def __init__(self, msg: Any) -> None:
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

    def patch(self, xy: tuple[float, float], radius_cells: int = 4) -> dict[str, Any]:
        cx, cy = self.world_to_cell(xy[0], xy[1])
        values: list[int] = []
        rows = []
        for yy in range(cy - radius_cells, cy + radius_cells + 1):
            row_values: list[int | None] = []
            for xx in range(cx - radius_cells, cx + radius_cells + 1):
                value = self.cell_value((xx, yy))
                row_values.append(value)
                if value is not None:
                    values.append(value)
            rows.append({'cell_y': yy, 'values': row_values})
        return {'center_cell': [cx, cy], 'radius_cells': radius_cells, 'rows': rows, 'summary': _cost_values_summary(values)}

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
            samples.append({'point': [x, y], 'cell': list(cell), 'in_bounds': self.in_bounds(cell), 'value': self.cell_value(cell), 'distance_m': distance * ratio})
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


def _target_evidence(costmap: CostmapView | None, target: tuple[float, float] | None) -> dict[str, Any]:
    if costmap is None or target is None:
        return {'available': False}
    cell = costmap.world_to_cell(target[0], target[1])
    radius_values = costmap.radius_values(target, 0.30)
    return {
        'available': True,
        'target': list(target),
        'cell': list(cell),
        'in_bounds': costmap.in_bounds(cell),
        'value': costmap.cell_value(cell),
        'radius_m': 0.30,
        'radius_cost_summary': _cost_values_summary(radius_values),
        'patch': costmap.patch(target, 4),
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
            delta = math.atan2(math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw))
            if abs(delta) <= half_angle_rad:
                values.append(value)
                if value >= HIGH_COST_THRESHOLD:
                    nearest_high = dist if nearest_high is None else min(nearest_high, dist)
    return values, nearest_high


def _path_cost_evidence(costmap: CostmapView | None, start: tuple[float, float] | None, target: tuple[float, float] | None) -> dict[str, Any]:
    if costmap is None or start is None or target is None:
        return {'available': False}
    samples = costmap.line_samples(start, target)
    values = [row.get('value') for row in samples]
    first_high = next((row for row in samples if row.get('value') is not None and int(row['value']) >= HIGH_COST_THRESHOLD), None)
    return {'available': True, 'start': list(start), 'target': list(target), 'cost_summary': _cost_values_summary(values), 'first_high_cost_distance_m': first_high.get('distance_m') if first_high else None, 'first_high_cost_point': first_high.get('point') if first_high else None}


def _pose_from_odom_msg(msg: Any) -> tuple[float, float, float]:
    pos = msg.pose.pose.position
    return (float(pos.x), float(pos.y), _yaw_from_quat(msg.pose.pose.orientation))


def _path_summary_from_points(points: list[tuple[float, float]]) -> dict[str, Any]:
    if not points:
        return {'available': False, 'pose_count': 0, 'path_length_m': None}
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))
    return {'available': True, 'pose_count': len(points), 'path_length_m': length, 'first_pose': list(points[0]), 'last_pose': list(points[-1])}


class FirstDispatchTraversabilityRecorder(Node):  # pragma: no cover - requires ROS graph
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('phase62_first_dispatch_traversability_recorder')
        self.args = args
        self.started_at = time.time()
        self.done = False
        self.active_sequence: int | None = None
        self.dispatch_event: dict[str, Any] | None = None
        self.outcome_event: dict[str, Any] | None = None
        self.latest_local_costmap: CostmapView | None = None
        self.latest_pose: tuple[float, float, float] | None = None
        self.latest_plan_points: list[tuple[float, float]] = []
        self.latest_cmd: dict[str, Any] | None = None
        self.latest_scan_stamp: float | None = None
        self.latest_costmap_stamp: float | None = None
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
        qos = QoSProfile(depth=50)
        self.create_subscription(String, args.goal_events_topic, self._on_goal_event, qos)
        self.create_subscription(OccupancyGrid, args.local_costmap_topic, self._on_local_costmap, qos)
        self.create_subscription(NavPath, args.path_topic, self._on_plan, qos)
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, qos)
        self.create_subscription(Twist, args.cmd_topic, self._on_cmd, qos)
        self.create_subscription(Twist, args.cmd_smoothed_topic, self._on_cmd_smoothed, qos)
        self.create_subscription(CollisionMonitorState, args.collision_monitor_topic, self._on_collision_state, qos)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage, args.nav2_feedback_topic, self._on_nav2_feedback, qos)
        self.create_timer(0.2, self._on_timer)

    def _write(self, stream: str, row: dict[str, Any]) -> None:
        row.setdefault('wall_time', time.time())
        row['elapsed_sec'] = row['wall_time'] - self.started_at
        self.fps[stream].write(json.dumps(row, sort_keys=True) + '\n')
        self.fps[stream].flush()

    def _target(self) -> tuple[float, float] | None:
        target = (self.dispatch_event or {}).get('target')
        if isinstance(target, (list, tuple)) and len(target) >= 2:
            return (float(target[0]), float(target[1]))
        return None

    def _on_goal_event(self, msg: Any) -> None:
        payload = _safe_json_loads(msg.data)
        if not isinstance(payload, dict):
            return
        event = payload.get('event')
        seq = payload.get('goal_sequence')
        seq = int(seq) if seq is not None else None
        if event == 'dispatch' and self.active_sequence is None and seq is not None:
            self.active_sequence = seq
            self.dispatch_event = payload
            self._write('execution', {'event': 'goal_event', 'snapshot_type': 'dispatch', 'goal_sequence': seq, 'payload': payload})
            self._emit_snapshot('dispatch_snapshot')
        elif event in {'success', 'failure', 'timeout', 'timeout_cancel_result'} and seq == self.active_sequence:
            self.outcome_event = payload
            self._write('execution', {'event': 'goal_event', 'snapshot_type': event, 'goal_sequence': seq, 'payload': payload})
            self._emit_snapshot(f'{event}_snapshot')
            self.done = True

    def _on_local_costmap(self, msg: Any) -> None:
        self.latest_local_costmap = CostmapView(msg)
        self.latest_costmap_stamp = self.latest_local_costmap.stamp_sec
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
                'target_footprint_cost': _cost_values_summary(_footprint_values(self.latest_local_costmap, (target[0], target[1], 0.0)) if target else []),
                'robot_footprint_cost': _cost_values_summary(_footprint_values(self.latest_local_costmap, self.latest_pose)),
                'front_wedge_cost': _cost_values_summary(_front_wedge_values(self.latest_local_costmap, self.latest_pose)[0]),
                'front_wedge_clearance_m': _front_wedge_values(self.latest_local_costmap, self.latest_pose)[1],
                'timestamp_consistency': self._timestamp_consistency(),
            })

    def _on_plan(self, msg: Any) -> None:
        self.latest_plan_points = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
        if self.active_sequence is not None:
            row = {'event': 'global_plan_update', 'goal_sequence': self.active_sequence, 'global_plan_summary': _path_summary_from_points(self.latest_plan_points)}
            self._write('execution', row)
            self._write('global_plan', row)

    def _on_odom(self, msg: Any) -> None:
        self.latest_pose = _pose_from_odom_msg(msg)
        tw = msg.twist.twist
        self._write('controller', {'source': 'odom', 'x': self.latest_pose[0], 'y': self.latest_pose[1], 'yaw': self.latest_pose[2], 'odom_linear_x': float(tw.linear.x), 'odom_linear_y': float(tw.linear.y), 'odom_angular_z': float(tw.angular.z), 'goal_sequence': self.active_sequence})

    def _on_cmd(self, msg: Any) -> None:
        self._write_cmd('cmd_vel', msg)

    def _on_cmd_smoothed(self, msg: Any) -> None:
        self._write_cmd('cmd_vel_smoothed', msg)

    def _write_cmd(self, source: str, msg: Any) -> None:
        row = {'source': source, 'linear_x': float(msg.linear.x), 'linear_y': float(msg.linear.y), 'linear_z': float(msg.linear.z), 'angular_x': float(msg.angular.x), 'angular_y': float(msg.angular.y), 'angular_z': float(msg.angular.z), 'goal_sequence': self.active_sequence}
        self.latest_cmd = row
        self._write('controller', row)

    def _on_collision_state(self, msg: Any) -> None:
        self._write('collision', {'event': 'collision_monitor_state', 'goal_sequence': self.active_sequence, 'action_type': int(getattr(msg, 'action_type', -1)), 'polygon_name': str(getattr(msg, 'polygon_name', ''))})

    def _on_nav2_feedback(self, msg: Any) -> None:
        feedback = msg.feedback
        self._write('feedback', {'event': 'nav2_feedback', 'goal_sequence': self.active_sequence, 'distance_remaining': float(getattr(feedback, 'distance_remaining', math.nan)), 'navigation_time_sec': _duration_seconds(getattr(feedback, 'navigation_time', None)), 'estimated_time_remaining_sec': _duration_seconds(getattr(feedback, 'estimated_time_remaining', None)), 'number_of_recoveries': int(getattr(feedback, 'number_of_recoveries', 0))})

    def _timestamp_consistency(self) -> dict[str, Any]:
        now = time.time()
        return {'wall_time': now, 'local_costmap_stamp_sec': self.latest_costmap_stamp, 'scan_stamp_sec': self.latest_scan_stamp, 'costmap_wall_delta_sec': (now - self.latest_costmap_stamp) if self.latest_costmap_stamp else None}

    def _emit_snapshot(self, snapshot_type: str) -> None:
        target = self._target()
        pose_xy = (self.latest_pose[0], self.latest_pose[1]) if self.latest_pose else None
        wedge_values, wedge_clearance = _front_wedge_values(self.latest_local_costmap, self.latest_pose)
        row = {
            'event': 'snapshot',
            'snapshot_type': snapshot_type,
            'goal_sequence': self.active_sequence,
            'dispatch_target': list(target) if target else None,
            'robot_pose': list(self.latest_pose) if self.latest_pose else None,
            'local_costmap_target_evidence': _target_evidence(self.latest_local_costmap, target),
            'local_costmap_path_evidence': _path_cost_evidence(self.latest_local_costmap, pose_xy, target),
            'target_footprint_cost': _cost_values_summary(_footprint_values(self.latest_local_costmap, (target[0], target[1], 0.0)) if target else []),
            'robot_footprint_cost': _cost_values_summary(_footprint_values(self.latest_local_costmap, self.latest_pose)),
            'front_wedge_cost': _cost_values_summary(wedge_values),
            'front_wedge_clearance_m': wedge_clearance,
            'global_plan_summary': _path_summary_from_points(self.latest_plan_points),
            'last_cmd': self.latest_cmd,
            'timestamp_consistency': self._timestamp_consistency(),
        }
        self._write('execution', row)

    def _on_timer(self) -> None:
        now = time.time()
        if self.active_sequence is not None and now - self.last_periodic_snapshot >= self.args.periodic_snapshot_sec:
            self.last_periodic_snapshot = now
            self._emit_snapshot('periodic_active_goal')
        if now - self.started_at >= self.args.timeout_sec:
            self._emit_snapshot('recorder_timeout_snapshot')
            self.done = True

    def close(self) -> None:
        for fp in self.fps.values():
            fp.close()


def record_execution(args: argparse.Namespace) -> int:  # pragma: no cover
    if rclpy is None:
        raise SystemExit('rclpy is required for --record-execution')
    rclpy.init()
    node = FirstDispatchTraversabilityRecorder(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def _first_goal_event(goal_events: list[dict[str, Any]], event: str) -> dict[str, Any] | None:
    return next((row for row in goal_events if row.get('event') == event), None)


def _execution_snapshots(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [row for row in rows if row.get('event') == 'snapshot']


def _goal_window_times(goal_events: list[dict[str, Any]]) -> tuple[float | None, float | None]:
    dispatch = _first_goal_event(goal_events, 'dispatch')
    outcomes = [row for row in goal_events if row.get('event') in {'success', 'failure', 'timeout', 'timeout_cancel_result'}]
    start = _number((dispatch or {}).get('_recorder_wall_time') or (dispatch or {}).get('wall_time'))
    end = _number((outcomes[0] if outcomes else {}).get('_recorder_wall_time') or (outcomes[0] if outcomes else {}).get('wall_time')) if outcomes else None
    return start, end


def _rows_in_goal_window(rows: list[dict[str, Any]], start: float | None, end: float | None) -> list[dict[str, Any]]:
    out = []
    for row in rows:
        wall = _number(row.get('wall_time'))
        if wall is None or ((start is None or wall >= start - 1.0) and (end is None or wall <= end + 1.0)):
            out.append(row)
    return out


def _cmd_vel_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    cmd_rows = [row for row in rows if str(row.get('source')) in {'cmd_vel', 'cmd_vel_smoothed'} or 'linear_x' in row]
    linear = [_number(row.get('linear_x')) or 0.0 for row in cmd_rows]
    angular = [_number(row.get('angular_z')) or 0.0 for row in cmd_rows]
    moving = [abs(l) > NEAR_ZERO_LINEAR or abs(a) > NEAR_ZERO_ANGULAR for l, a in zip(linear, angular)]
    return {'cmd_vel_sample_count': len(cmd_rows), 'linear_x_abs': _stats([abs(v) for v in linear]), 'angular_z_abs': _stats([abs(v) for v in angular]), 'nonzero_command_count': sum(1 for flag in moving if flag), 'near_zero_command_ratio': (sum(1 for flag in moving if not flag) / len(moving)) if moving else None, 'controller_no_cmd_vel': bool(len(cmd_rows) == 0 or sum(1 for flag in moving if flag) == 0)}


def _robot_to_target_progress_summary(rows: list[dict[str, Any]], dispatch: dict[str, Any] | None) -> dict[str, Any]:
    odom_rows = [row for row in rows if row.get('source') == 'odom' and _number(row.get('x')) is not None and _number(row.get('y')) is not None]
    target = (dispatch or {}).get('target')
    if not (isinstance(target, (list, tuple)) and len(target) >= 2):
        target = None
    poses = [[float(row.get('x')), float(row.get('y')), float(row.get('yaw', 0.0))] for row in odom_rows]
    distances = [math.hypot(p[0] - float(target[0]), p[1] - float(target[1])) for p in poses] if target else []
    total_motion = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poses, poses[1:]))
    return {'pose_sample_count': len(odom_rows), 'first_pose': poses[0] if poses else None, 'final_pose': poses[-1] if poses else None, 'total_motion_m': total_motion, 'distance_to_target': _stats(distances), 'distance_improvement_m': (distances[0] - distances[-1]) if distances else None, 'robot_stuck': bool(len(odom_rows) > 5 and total_motion < 0.10)}


def _nav2_feedback_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    feedback_rows = [row for row in rows if row.get('event') == 'nav2_feedback']
    distances = [_number(row.get('distance_remaining')) for row in feedback_rows]
    clean = [v for v in distances if v is not None]
    return {'feedback_sample_count': len(feedback_rows), 'distance_remaining': _stats(clean), 'distance_remaining_improvement_m': (clean[0] - clean[-1]) if clean else None, 'number_of_recoveries_max': max([int(row.get('number_of_recoveries') or 0) for row in feedback_rows], default=0)}


def _timeline_from_snapshots(snapshots: list[dict[str, Any]], key: str) -> dict[str, Any]:
    maxes = [_number((row.get(key) or {}).get('max')) for row in snapshots if isinstance(row.get(key), dict)]
    return {'sample_count': len(snapshots), 'max': _stats([v for v in maxes if v is not None]), 'latest': snapshots[-1].get(key) if snapshots else None}


def _local_costmap_evidence_from_snapshots(snapshots: list[dict[str, Any]]) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    targets = [row.get('local_costmap_target_evidence') for row in snapshots if isinstance(row.get('local_costmap_target_evidence'), dict) and row['local_costmap_target_evidence'].get('available')]
    paths = [row.get('local_costmap_path_evidence') for row in snapshots if isinstance(row.get('local_costmap_path_evidence'), dict) and row['local_costmap_path_evidence'].get('available')]
    plans = [row.get('global_plan_summary') for row in snapshots if isinstance(row.get('global_plan_summary'), dict) and row['global_plan_summary'].get('available')]
    return ({'available_sample_count': len(targets), 'latest': targets[-1] if targets else None, 'target_radius_cost_max': _stats([_number((r.get('radius_cost_summary') or {}).get('max')) for r in targets if _number((r.get('radius_cost_summary') or {}).get('max')) is not None])}, {'available_sample_count': len(paths), 'latest': paths[-1] if paths else None, 'path_cost_max': _stats([_number((r.get('cost_summary') or {}).get('max')) for r in paths if _number((r.get('cost_summary') or {}).get('max')) is not None])}, {'available_sample_count': len(plans), 'latest': plans[-1] if plans else None})


def _timestamp_consistency_summary(snapshots: list[dict[str, Any]], local_rows: list[dict[str, Any]]) -> dict[str, Any]:
    rows = [row.get('timestamp_consistency') for row in snapshots + local_rows if isinstance(row.get('timestamp_consistency'), dict)]
    deltas = [_number(row.get('costmap_wall_delta_sec')) for row in rows]
    return {'sample_count': len(rows), 'costmap_wall_delta_sec': _stats([v for v in deltas if v is not None]), 'latest': rows[-1] if rows else None}


def _log_summary(text: str) -> dict[str, Any]:
    patterns = ['Failed to make progress', 'Goal failed', 'Controller patience exceeded', 'No valid', 'collision', 'costmap']
    return {'pattern_counts': {p: text.count(p) for p in patterns}, 'tail': text.splitlines()[-80:]}


def classify_phase62(summary: dict[str, Any]) -> str:
    dispatch_observed = bool(summary.get('first_dispatch_event'))
    if summary.get('guardrail_violation'):
        return 'GUARDRAIL_VIOLATION_STRATEGY_CHANGED'
    if not dispatch_observed or not summary.get('runtime_evidence_required'):
        return 'INSUFFICIENT_EVIDENCE'
    dispatch = summary.get('first_dispatch_event') or {}
    target_clearance = _number(dispatch.get('target_clearance_m'))
    path_clearance = _number(dispatch.get('path_corridor_min_clearance_m'))
    target_evidence = summary.get('local_costmap_target_evidence') or {}
    latest_target = target_evidence.get('latest') or {}
    radius_max = _number(((latest_target.get('radius_cost_summary') or {}).get('max')))
    target_value = _number(latest_target.get('value'))
    footprint = summary.get('target_footprint_cost_summary') or {}
    footprint_max = _number(((footprint.get('max') or {}).get('max')) or ((footprint.get('latest') or {}).get('max')))
    timestamp = summary.get('timestamp_consistency_summary') or {}
    ts_delta_max = _number(((timestamp.get('costmap_wall_delta_sec') or {}).get('max')))
    if target_clearance is not None and target_clearance < 0.35:
        return 'TARGET_TOO_CLOSE_TO_WALL'
    if path_clearance is not None and path_clearance < 0.50:
        return 'CORRIDOR_TOO_NARROW'
    if (radius_max is not None and radius_max >= HIGH_COST_THRESHOLD) or (target_value is not None and target_value >= HIGH_COST_THRESHOLD) or (footprint_max is not None and footprint_max >= HIGH_COST_THRESHOLD):
        return 'LOCAL_COSTMAP_INFLATION_DOMINANT'
    if ts_delta_max is not None and ts_delta_max > 2.0:
        return 'SENSOR_COSTMAP_MISALIGNMENT'
    return 'INSUFFICIENT_EVIDENCE'


def summarize_replay(replay_dir: Path, run_id: str) -> dict[str, Any]:
    events = _read_jsonl(replay_dir / f'{run_id}_{replay_dir.name}_goal_events.jsonl')
    states = _read_jsonl(replay_dir / f'{run_id}_{replay_dir.name}_explorer_state.jsonl')
    execution_rows = _read_jsonl(replay_dir / 'first_dispatch_execution.jsonl')
    controller_rows = _read_jsonl(replay_dir / 'controller_dynamics.jsonl')
    feedback_rows = _read_jsonl(replay_dir / 'nav2_feedback.jsonl')
    local_rows = _read_jsonl(replay_dir / 'local_costmap_samples.jsonl')
    start, end = _goal_window_times(events)
    controller_window = _rows_in_goal_window(controller_rows, start, end)
    feedback_window = _rows_in_goal_window(feedback_rows, start, end)
    snapshots = _execution_snapshots(execution_rows)
    first_dispatch = _first_goal_event(events, 'dispatch')
    outcome = next((row for row in events if row.get('event') in {'success', 'failure', 'timeout', 'timeout_cancel_result'}), None)
    target_ev, path_ev, plan_ev = _local_costmap_evidence_from_snapshots(snapshots)
    embedded_phase62 = first_dispatch if first_dispatch and first_dispatch.get('phase62_first_dispatch_traversability') else None
    return {
        'replay_id': replay_dir.name,
        'artifact_dir': str(replay_dir),
        'state_sample_count': len(states),
        'goal_event_count': len(events),
        'first_dispatch_event': first_dispatch,
        'goal_outcome': outcome,
        'dispatch_observed': bool(first_dispatch),
        'embedded_phase62_dispatch_diagnostics': embedded_phase62,
        'local_costmap_patch_summary': (embedded_phase62 or {}).get('phase62_local_costmap_patch') or (target_ev.get('latest') or {}).get('patch'),
        'target_footprint_cost_summary': {'latest': (embedded_phase62 or {}).get('phase62_target_footprint_cost'), 'max': _timeline_from_snapshots(snapshots, 'target_footprint_cost').get('max')},
        'front_wedge_clearance_summary': {'embedded': (embedded_phase62 or {}).get('phase62_front_wedge_clearance_m'), 'timeline': _timeline_from_snapshots(snapshots, 'front_wedge_cost')},
        'robot_to_target_progress_summary': _robot_to_target_progress_summary(controller_window, first_dispatch),
        'cmd_vel_summary': _cmd_vel_summary(controller_window),
        'nav2_feedback_summary': _nav2_feedback_summary(feedback_window),
        'nav2_result_summary': (embedded_phase62 or {}).get('phase62_nav2_result_summary') or outcome,
        'local_costmap_target_evidence': target_ev,
        'local_costmap_path_evidence': path_ev,
        'global_plan_summary': plan_ev,
        'timestamp_consistency_summary': _timestamp_consistency_summary(snapshots, local_rows),
        'runtime_evidence_required': bool(first_dispatch and (snapshots or local_rows or controller_rows or feedback_rows)),
    }


def _cleanup_empty(path: Path) -> bool:
    return _read_text(path).strip() == ''


def analyze(args: argparse.Namespace) -> int:
    artifact_dir = Path(args.artifact_dir)
    replay_dirs = sorted(p for p in artifact_dir.glob('replay_*') if p.is_dir())
    replays = [summarize_replay(path, RUN_ID) for path in replay_dirs]
    nav2_config_diff_empty = not _read_text(artifact_dir / f'{RUN_ID}_nav2_config_diff.txt').strip()
    cleanup_empty = _cleanup_empty(artifact_dir / f'{RUN_ID}_cleanup_processes_after.txt')
    launch_text = '\n'.join(_read_text(p) for p in artifact_dir.glob('replay_*/*launch.log'))
    preflight_text = _read_text(artifact_dir / f'{RUN_ID}_preflight.txt')
    goal_count_violation = any(len([e for e in _read_jsonl(Path(r['artifact_dir']) / f'{RUN_ID}_{r["replay_id"]}_goal_events.jsonl') if e.get('event') == 'dispatch']) > 1 for r in replays)
    guardrail_violation = bool(goal_count_violation or not nav2_config_diff_empty or 'max_goals=1' not in preflight_text or 'near_exit_fallback_enabled=false' not in preflight_text)
    combined = {
        'guardrail_violation': guardrail_violation,
        'runtime_evidence_required': any(r['runtime_evidence_required'] for r in replays),
        'first_dispatch_event': next((r['first_dispatch_event'] for r in replays if r['first_dispatch_event']), None),
        'local_costmap_target_evidence': next((r['local_costmap_target_evidence'] for r in replays if r['local_costmap_target_evidence'].get('available_sample_count')), {}),
        'target_footprint_cost_summary': next((r['target_footprint_cost_summary'] for r in replays if r['target_footprint_cost_summary']), {}),
        'timestamp_consistency_summary': next((r['timestamp_consistency_summary'] for r in replays if r['timestamp_consistency_summary'].get('sample_count')), {}),
    }
    classification = classify_phase62(combined)
    summary = {
        'phase': PHASE,
        'run_id': RUN_ID,
        'classification': classification,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'active_world': ACTIVE_WORLD,
        'active_metadata': ACTIVE_METADATA,
        'phase61_classification_preserved': PHASE61_CLASSIFICATION,
        'artifact_dir': str(artifact_dir),
        'replay_count': len(replays),
        'replays': replays,
        'metrics': {
            'dispatch_observed': any(r['dispatch_observed'] for r in replays),
            'runtime_evidence_required': any(r['runtime_evidence_required'] for r in replays),
            'local_costmap_patch_summary': [r['local_costmap_patch_summary'] for r in replays],
            'target_footprint_cost_summary': [r['target_footprint_cost_summary'] for r in replays],
            'front_wedge_clearance_summary': [r['front_wedge_clearance_summary'] for r in replays],
            'robot_to_target_progress_summary': [r['robot_to_target_progress_summary'] for r in replays],
            'cmd_vel_summary': [r['cmd_vel_summary'] for r in replays],
            'nav2_feedback_summary': [r['nav2_feedback_summary'] for r in replays],
            'nav2_result_summary': [r['nav2_result_summary'] for r in replays],
            'planner_controller_log_correlation': _log_summary(launch_text),
            'timestamp_consistency_summary': [r['timestamp_consistency_summary'] for r in replays],
        },
        'guardrails': GUARDRAILS,
        'guardrail_violation': guardrail_violation,
        'goal_count_violation': goal_count_violation,
        'nav2_config_diff_empty': nav2_config_diff_empty,
        'cleanup_empty': cleanup_empty,
        'complete_autonomous_success_claimed': False,
        'first_dispatch_is_not_exit_success': True,
        'recommendations': ['Interpret as first-dispatch local traversability diagnostics only; do not claim autonomous exploration or exit success.'],
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=PHASE)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--send-ingress-goal', action='store_true')
    mode.add_argument('--record-execution', action='store_true')
    mode.add_argument('--analyze', action='store_true')
    parser.add_argument('--artifact-dir', default=str(Path('log') / RUN_ID))
    parser.add_argument('--output', required=True)
    parser.add_argument('--goal-timeout-sec', type=float, default=100.0)
    parser.add_argument('--timeout-sec', type=float, default=140.0)
    parser.add_argument('--periodic-snapshot-sec', type=float, default=1.0)
    parser.add_argument('--execution-output', default=str(Path('log') / RUN_ID / 'first_dispatch_execution.jsonl'))
    parser.add_argument('--controller-dynamics-output', default=str(Path('log') / RUN_ID / 'controller_dynamics.jsonl'))
    parser.add_argument('--nav2-feedback-output', default=str(Path('log') / RUN_ID / 'nav2_feedback.jsonl'))
    parser.add_argument('--local-costmap-samples-output', default=str(Path('log') / RUN_ID / 'local_costmap_samples.jsonl'))
    parser.add_argument('--global-plan-samples-output', default=str(Path('log') / RUN_ID / 'global_plan_samples.jsonl'))
    parser.add_argument('--collision-monitor-output', default=str(Path('log') / RUN_ID / 'collision_monitor_state.jsonl'))
    parser.add_argument('--local-costmap-topic', default='/local_costmap/costmap')
    parser.add_argument('--path-topic', default='/plan')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--cmd-topic', default='/cmd_vel')
    parser.add_argument('--cmd-smoothed-topic', default='/cmd_vel_smoothed')
    parser.add_argument('--goal-events-topic', default='/maze/goal_events')
    parser.add_argument('--collision-monitor-topic', default='/collision_monitor_state')
    parser.add_argument('--nav2-feedback-topic', default='/navigate_to_pose/_action/feedback')
    args = parser.parse_args(argv)
    if args.send_ingress_goal:
        return send_ingress_goal(args)
    if args.record_execution:
        return record_execution(args)
    return analyze(args)


if __name__ == '__main__':
    raise SystemExit(main())
