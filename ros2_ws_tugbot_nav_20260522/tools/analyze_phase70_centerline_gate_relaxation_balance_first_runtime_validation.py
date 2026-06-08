#!/usr/bin/env python3
"""Phase70 Centerline Gate Relaxation / Balance-First Runtime Validation.

Guardrails: bounded runtime only, max_goals=1, no Nav2/MPPI/controller
parameter edits, no inflation/robot_radius/clearance_radius_m/map threshold
tuning, no branch scoring change, no corridor-following cmd_vel control, no
fallback/terminal acceptance change, no old scaffold world/map, no autonomous
exploration success claim, no exit success claim.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import time
from pathlib import Path
from typing import Any

PHASE = 'Phase70 Centerline Gate Relaxation / Balance-First Runtime Validation'
RUN_ID = 'phase70_centerline_gate_relaxation_balance_first_runtime_validation'
PHASE69_CLASSIFICATION = 'CENTERLINE_GATE_NO_APPLY'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
ACTIVE_METADATA = 'maze_20260528_scaled_instance.yaml'
INNER_INGRESS_WAYPOINT_MAP = {'x_m': 2.0, 'y_m': 0.0, 'yaw_rad': 0.0}
HIGH_COST_THRESHOLD = 70
LETHAL_COST_THRESHOLD = 99
NEAR_ZERO_LINEAR = 0.01
NEAR_ZERO_ANGULAR = 0.05
ALLOWED_CLASSIFICATIONS = [
    'BALANCE_FIRST_CENTERLINE_APPLIED_AND_IMPROVES',
    'BALANCE_FIRST_APPLIED_TIMEOUT_REMAINS',
    'BALANCE_FIRST_GATE_NO_APPLY',
    'BALANCE_FIRST_REGRESSION',
    'INSUFFICIENT_EVIDENCE',
    'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
]
GUARDRAILS = [
    'bounded runtime only',
    'max_goals=1',
    'no Nav2/MPPI/controller parameter edits',
    'no inflation/robot_radius/clearance_radius_m/map threshold tuning',
    'no branch scoring change',
    'no corridor-following cmd_vel control',
    'no fallback/terminal acceptance change',
    'no old scaffold world/map',
    'no autonomous exploration success claim',
    'no exit success claim',
]

try:  # pragma: no cover - ROS graph only.
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


def _safe_json_loads(text: str) -> Any | None:
    try:
        return json.loads(text)
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
        if isinstance(row, dict):
            payload = row.get('state') if isinstance(row.get('state'), dict) else row
            if isinstance(payload, dict):
                payload = dict(payload)
                payload.setdefault('_recorder_elapsed_sec', row.get('elapsed_sec'))
                payload.setdefault('_recorder_wall_time', row.get('wall_time'))
                rows.append(payload)
    return rows


def _number(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _stats(values: list[float | None]) -> dict[str, Any]:
    clean = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    if not clean:
        return {'count': 0, 'min': None, 'max': None, 'mean': None, 'final': None}
    return {'count': len(clean), 'min': min(clean), 'max': max(clean), 'mean': sum(clean) / len(clean), 'final': clean[-1]}


def _yaw_from_quat(q: Any) -> float:
    return math.atan2(2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)), 1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2))


def _duration_seconds(value: Any) -> float | None:
    if value is None:
        return None
    return float(getattr(value, 'sec', 0)) + float(getattr(value, 'nanosec', 0)) / 1e9


class CostmapView:
    def __init__(self, msg: Any) -> None:
        self.width = int(msg.info.width)
        self.height = int(msg.info.height)
        self.resolution = float(msg.info.resolution)
        self.origin_x = float(msg.info.origin.position.x)
        self.origin_y = float(msg.info.origin.position.y)
        self.data = [int(v) for v in msg.data]

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (int(math.floor((x - self.origin_x) / max(self.resolution, 1e-9))), int(math.floor((y - self.origin_y) / max(self.resolution, 1e-9))))

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
                if math.hypot(wx - xy[0], wy - xy[1]) <= radius_m + 1e-9:
                    value = self.cell_value((xx, yy))
                    if value is not None:
                        values.append(value)
        return values


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


def _pose_from_odom_msg(msg: Any) -> tuple[float, float, float]:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return (float(p.x), float(p.y), _yaw_from_quat(q))


def _footprint_values(costmap: CostmapView | None, pose: tuple[float, float, float] | None, length_m: float = 0.60, width_m: float = 0.50) -> list[int]:
    if costmap is None or pose is None:
        return []
    x, y, yaw = pose
    values: list[int] = []
    step = max(costmap.resolution, 0.05)
    half_l, half_w = length_m / 2.0, width_m / 2.0
    c, s = math.cos(yaw), math.sin(yaw)
    nx = max(1, int(math.ceil(length_m / step)))
    ny = max(1, int(math.ceil(width_m / step)))
    for ix in range(nx + 1):
        lx = -half_l + ix * length_m / nx
        for iy in range(ny + 1):
            ly = -half_w + iy * width_m / ny
            value = costmap.value_at(x + c * lx - s * ly, y + s * lx + c * ly)
            if value is not None:
                values.append(value)
    return values


def _front_wedge_values(costmap: CostmapView | None, pose: tuple[float, float, float] | None, length_m: float = 0.80, half_angle_rad: float = 0.45) -> tuple[list[int], float | None]:
    if costmap is None or pose is None:
        return [], None
    x, y, yaw = pose
    values: list[int] = []
    first_high: float | None = None
    radial_steps = max(2, int(math.ceil(length_m / max(costmap.resolution, 0.05))))
    for ir in range(1, radial_steps + 1):
        r = length_m * ir / radial_steps
        for ia in range(-8, 9):
            a = yaw + half_angle_rad * ia / 8
            value = costmap.value_at(x + r * math.cos(a), y + r * math.sin(a))
            if value is not None:
                values.append(value)
                if value >= HIGH_COST_THRESHOLD and first_high is None:
                    first_high = r
    return values, first_high


def _target_evidence(costmap: CostmapView | None, target: tuple[float, float] | None) -> dict[str, Any]:
    if costmap is None or target is None:
        return {'available': False}
    cell = costmap.world_to_cell(target[0], target[1])
    radius_values = costmap.radius_values(target, 0.30)
    return {'available': True, 'target': list(target), 'cell': list(cell), 'in_bounds': costmap.in_bounds(cell), 'value': costmap.cell_value(cell), 'radius_cost_summary': _cost_values_summary(radius_values)}


def _phase65_module() -> Any:  # pragma: no cover
    path = Path(__file__).resolve().parent / 'analyze_phase65_ingress_goal_depth_adjustment_inner_staging.py'
    spec = importlib.util.spec_from_file_location('phase65_inner_ingress_runtime_helpers', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot load Phase65 helper module: {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    setattr(module, 'RUN_ID', RUN_ID)
    setattr(module, 'PHASE', PHASE)
    return module


def send_inner_ingress_goal(args: argparse.Namespace) -> int:  # pragma: no cover
    return int(_phase65_module().send_inner_ingress_goal(args))


class Phase70RuntimeRecorder(Node):  # pragma: no cover - requires ROS graph.
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('phase70_centerline_runtime_recorder')
        self.args = args
        self.started_at = time.time()
        self.done = False
        self.latest_local_costmap: CostmapView | None = None
        self.latest_pose: tuple[float, float, float] | None = None
        self.latest_plan_points: list[tuple[float, float]] = []
        self.active_targets: dict[int, tuple[float, float]] = {}
        self.active_sequence: int | None = None
        self.last_periodic_snapshot = 0.0
        self.paths = {
            'timeline': Path(args.timeline_output),
            'controller': Path(args.controller_dynamics_output),
            'feedback': Path(args.nav2_feedback_output),
            'local_costmap': Path(args.local_costmap_samples_output),
            'global_plan': Path(args.global_plan_samples_output),
            'collision': Path(args.collision_monitor_output),
        }
        for path in self.paths.values():
            path.parent.mkdir(parents=True, exist_ok=True)
        self.fps = {name: path.open('w', encoding='utf-8') for name, path in self.paths.items()}
        qos = QoSProfile(depth=80)
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

    def _snapshot(self, snapshot_type: str, seq: int | None = None) -> dict[str, Any]:
        target = self.active_targets.get(seq if seq is not None else self.active_sequence)
        wedge_values, wedge_clearance = _front_wedge_values(self.latest_local_costmap, self.latest_pose)
        target_pose = (target[0], target[1], 0.0) if target else None
        return {
            'event': 'snapshot',
            'snapshot_type': snapshot_type,
            'goal_sequence': seq if seq is not None else self.active_sequence,
            'dispatch_target': list(target) if target else None,
            'robot_pose': list(self.latest_pose) if self.latest_pose else None,
            'local_costmap_target_evidence': _target_evidence(self.latest_local_costmap, target),
            'target_footprint_cost': _cost_values_summary(_footprint_values(self.latest_local_costmap, target_pose)),
            'robot_footprint_cost': _cost_values_summary(_footprint_values(self.latest_local_costmap, self.latest_pose)),
            'front_wedge_cost': _cost_values_summary(wedge_values),
            'front_wedge_clearance_m': wedge_clearance,
            'global_plan_point_count': len(self.latest_plan_points),
        }

    def _on_goal_event(self, msg: Any) -> None:
        payload = _safe_json_loads(msg.data)
        if not isinstance(payload, dict):
            return
        event = str(payload.get('event'))
        seq_raw = payload.get('goal_sequence')
        seq = int(seq_raw) if seq_raw is not None else None
        target = payload.get('target')
        if event == 'dispatch' and seq is not None and isinstance(target, (list, tuple)) and len(target) >= 2:
            self.active_sequence = seq
            self.active_targets[seq] = (float(target[0]), float(target[1]))
        self._write('timeline', {'event': 'goal_event', 'goal_sequence': seq, 'payload': payload})
        if event == 'dispatch':
            self._write('timeline', self._snapshot('dispatch_snapshot', seq))
        if event in {'success', 'failure', 'timeout', 'timeout_cancel_result'}:
            self._write('timeline', self._snapshot(f'{event}_snapshot', seq))

    def _on_local_costmap(self, msg: Any) -> None:
        self.latest_local_costmap = CostmapView(msg)
        if self.active_sequence is not None:
            row = self._snapshot('local_costmap_sample')
            row['event'] = 'local_costmap_sample'
            self._write('local_costmap', row)

    def _on_plan(self, msg: Any) -> None:
        self.latest_plan_points = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
        if self.active_sequence is not None:
            self._write('global_plan', {'event': 'global_plan_update', 'goal_sequence': self.active_sequence, 'point_count': len(self.latest_plan_points)})

    def _on_odom(self, msg: Any) -> None:
        pose = _pose_from_odom_msg(msg)
        self.latest_pose = pose
        tw = msg.twist.twist
        self._write('controller', {'source': 'odom', 'goal_sequence': self.active_sequence, 'x': pose[0], 'y': pose[1], 'yaw': pose[2], 'odom_linear_x': float(tw.linear.x), 'odom_linear_y': float(tw.linear.y), 'odom_angular_z': float(tw.angular.z)})

    def _on_cmd(self, msg: Any) -> None:
        self._write_cmd('cmd_vel', msg)

    def _on_cmd_smoothed(self, msg: Any) -> None:
        self._write_cmd('cmd_vel_smoothed', msg)

    def _write_cmd(self, source: str, msg: Any) -> None:
        self._write('controller', {'source': source, 'goal_sequence': self.active_sequence, 'linear_x': float(msg.linear.x), 'linear_y': float(msg.linear.y), 'linear_z': float(msg.linear.z), 'angular_x': float(msg.angular.x), 'angular_y': float(msg.angular.y), 'angular_z': float(msg.angular.z)})

    def _on_collision_state(self, msg: Any) -> None:
        self._write('collision', {'event': 'collision_monitor_state', 'goal_sequence': self.active_sequence, 'action_type': int(getattr(msg, 'action_type', -1)), 'polygon_name': str(getattr(msg, 'polygon_name', ''))})

    def _on_nav2_feedback(self, msg: Any) -> None:
        feedback = msg.feedback
        self._write('feedback', {'event': 'nav2_feedback', 'goal_sequence': self.active_sequence, 'distance_remaining': float(getattr(feedback, 'distance_remaining', math.nan)), 'navigation_time_sec': _duration_seconds(getattr(feedback, 'navigation_time', None)), 'estimated_time_remaining_sec': _duration_seconds(getattr(feedback, 'estimated_time_remaining', None)), 'number_of_recoveries': int(getattr(feedback, 'number_of_recoveries', 0))})

    def _on_timer(self) -> None:
        now = time.time()
        if self.active_sequence is not None and now - self.last_periodic_snapshot >= self.args.periodic_snapshot_sec:
            self.last_periodic_snapshot = now
            self._write('timeline', self._snapshot('periodic_active_goal'))
        if now - self.started_at >= self.args.timeout_sec:
            self._write('timeline', self._snapshot('recorder_timeout_snapshot'))
            self.done = True

    def close(self) -> None:
        for fp in self.fps.values():
            fp.close()


def record_runtime(args: argparse.Namespace) -> int:  # pragma: no cover
    if rclpy is None:
        raise SystemExit('rclpy is required for --record-runtime')
    rclpy.init()
    node = Phase70RuntimeRecorder(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def _goal_events_by_sequence(events: list[dict[str, Any]]) -> dict[int, list[dict[str, Any]]]:
    grouped: dict[int, list[dict[str, Any]]] = {}
    for event in events:
        seq = event.get('goal_sequence')
        if seq is None:
            continue
        try:
            grouped.setdefault(int(seq), []).append(event)
        except (TypeError, ValueError):
            continue
    return grouped


def _rows_for_sequence(rows: list[dict[str, Any]], seq: int) -> list[dict[str, Any]]:
    return [row for row in rows if row.get('goal_sequence') == seq]


def _cmd_vel_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    cmd_rows = [row for row in rows if str(row.get('source')) in {'cmd_vel', 'cmd_vel_smoothed'} or 'linear_x' in row]
    linear = [_number(row.get('linear_x')) or 0.0 for row in cmd_rows]
    angular = [_number(row.get('angular_z')) or 0.0 for row in cmd_rows]
    moving = [abs(l) > NEAR_ZERO_LINEAR or abs(a) > NEAR_ZERO_ANGULAR for l, a in zip(linear, angular)]
    return {'cmd_vel_sample_count': len(cmd_rows), 'linear_x_abs': _stats([abs(v) for v in linear]), 'angular_z_abs': _stats([abs(v) for v in angular]), 'nonzero_command_count': sum(1 for flag in moving if flag), 'near_zero_command_ratio': (sum(1 for flag in moving if not flag) / len(moving)) if moving else None}


def _nav2_feedback_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    feedback_rows = [row for row in rows if row.get('event') == 'nav2_feedback']
    clean = [_number(row.get('distance_remaining')) for row in feedback_rows]
    clean = [v for v in clean if v is not None]
    return {'feedback_sample_count': len(feedback_rows), 'distance_remaining': _stats(clean), 'distance_remaining_improvement_m': (clean[0] - clean[-1]) if clean else None, 'number_of_recoveries_max': max([int(row.get('number_of_recoveries') or 0) for row in feedback_rows], default=0)}


def _robot_progress_summary(rows: list[dict[str, Any]], dispatch: dict[str, Any] | None) -> dict[str, Any]:
    odom_rows = [row for row in rows if row.get('source') == 'odom' and _number(row.get('x')) is not None and _number(row.get('y')) is not None]
    poses = [[float(row['x']), float(row['y']), float(row.get('yaw', 0.0))] for row in odom_rows]
    target = (dispatch or {}).get('target')
    distances = [math.hypot(p[0] - float(target[0]), p[1] - float(target[1])) for p in poses] if isinstance(target, list) and len(target) >= 2 else []
    total_motion = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poses, poses[1:]))
    return {'pose_sample_count': len(poses), 'first_pose': poses[0] if poses else None, 'final_pose': poses[-1] if poses else None, 'total_motion_m': total_motion, 'distance_to_target': _stats(distances), 'distance_improvement_m': (distances[0] - distances[-1]) if distances else None}


def _local_cost_footprint_front_wedge_summary(rows: list[dict[str, Any]], dispatch: dict[str, Any] | None) -> dict[str, Any]:
    candidates: list[dict[str, Any]] = []
    if dispatch:
        candidates.append(dispatch)
    candidates.extend(rows)
    target_values: list[float | None] = []
    footprint_max: list[float | None] = []
    footprint_lethal: list[float | None] = []
    wedge_max: list[float | None] = []
    wedge_high: list[float | None] = []
    for row in candidates:
        target_values.append(_number(row.get('dispatch_target_local_cost')) if row.get('dispatch_target_local_cost') is not None else None)
        embedded_foot = row.get('phase62_target_footprint_cost') if isinstance(row.get('phase62_target_footprint_cost'), dict) else None
        embedded_wedge = row.get('phase62_front_wedge_cost') if isinstance(row.get('phase62_front_wedge_cost'), dict) else None
        local_target = row.get('local_costmap_target_evidence') if isinstance(row.get('local_costmap_target_evidence'), dict) else None
        target_foot = row.get('target_footprint_cost') if isinstance(row.get('target_footprint_cost'), dict) else None
        wedge = row.get('front_wedge_cost') if isinstance(row.get('front_wedge_cost'), dict) else None
        if local_target:
            target_values.append(_number(local_target.get('value')))
            summary = local_target.get('radius_cost_summary') if isinstance(local_target.get('radius_cost_summary'), dict) else {}
            target_values.append(_number(summary.get('max')))
        for source in [embedded_foot, target_foot]:
            if source:
                summary = source.get('summary') if isinstance(source.get('summary'), dict) else source
                footprint_max.append(_number(summary.get('max')))
                footprint_lethal.append(_number(summary.get('lethal_count')))
        for source in [embedded_wedge, wedge]:
            if source:
                wedge_max.append(_number(source.get('max')))
                wedge_high.append(_number(source.get('high_cost_count')))
    risk = any((v or 0) >= HIGH_COST_THRESHOLD for v in target_values + footprint_max + wedge_max) or any((v or 0) >= 1 for v in footprint_lethal)
    return {'sample_count': len(candidates), 'target_cost_max': _stats(target_values), 'target_footprint_max': _stats(footprint_max), 'target_footprint_lethal_count': _stats(footprint_lethal), 'front_wedge_cost_max': _stats(wedge_max), 'front_wedge_high_cost_count': _stats(wedge_high), 'local_cost_risk_observed': bool(risk)}


def summarize_replay(replay_dir: Path, run_id: str = RUN_ID) -> dict[str, Any]:
    replay_id = replay_dir.name
    ingress_action = _safe_json_load(replay_dir / f'{run_id}_{replay_id}_inner_ingress_navigate_to_pose_action_result.json', {})
    events = _read_jsonl(replay_dir / f'{run_id}_{replay_id}_goal_events.jsonl')
    states = _read_jsonl(replay_dir / f'{run_id}_{replay_id}_explorer_state.jsonl')
    timeline = _read_jsonl(replay_dir / 'phase70_runtime_timeline.jsonl')
    controller_rows = _read_jsonl(replay_dir / 'phase70_controller_dynamics.jsonl')
    feedback_rows = _read_jsonl(replay_dir / 'phase70_nav2_feedback.jsonl')
    local_rows = _read_jsonl(replay_dir / 'phase70_local_costmap_samples.jsonl')
    grouped = _goal_events_by_sequence(events)
    per_goal: list[dict[str, Any]] = []
    for seq in sorted(grouped):
        seq_events = grouped[seq]
        dispatch = next((e for e in seq_events if e.get('event') == 'dispatch'), None)
        outcome = next((e for e in seq_events if e.get('event') in {'success', 'failure', 'timeout', 'timeout_cancel_result'}), None)
        seq_timeline = _rows_for_sequence(timeline, seq)
        seq_controller = _rows_for_sequence(controller_rows, seq)
        seq_feedback = _rows_for_sequence(feedback_rows, seq)
        seq_local = _rows_for_sequence(local_rows, seq)
        local_summary = _local_cost_footprint_front_wedge_summary(seq_timeline + seq_local, dispatch)
        refinement = (dispatch or {}).get('centerline_target_refinement') if isinstance((dispatch or {}).get('centerline_target_refinement'), dict) else {}
        per_goal.append({
            'goal_sequence': seq,
            'dispatch_observed': dispatch is not None,
            'outcome_event': outcome.get('event') if outcome else None,
            'result_status': (outcome or dispatch or {}).get('result_status'),
            'result_reason': (outcome or dispatch or {}).get('result_reason'),
            'timeout': bool((outcome or {}).get('event') in {'timeout', 'timeout_cancel_result'}),
            'target': (dispatch or {}).get('target'),
            'original_target': (dispatch or {}).get('original_target'),
            'refined_target': (dispatch or {}).get('refined_target'),
            'centerline_refinement_applied': bool((dispatch or {}).get('centerline_refinement_applied')),
            'centerline_refinement_reason': (dispatch or {}).get('centerline_refinement_reason'),
            'centerline_target_refinement': refinement,
            'branch_scoring_changed': bool((dispatch or {}).get('branch_scoring_changed', False)),
            'cmd_vel_summary': _cmd_vel_summary(seq_controller),
            'nav2_feedback_summary': _nav2_feedback_summary(seq_feedback),
            'robot_progress_summary': _robot_progress_summary(seq_controller, dispatch),
            'local_cost_footprint_front_wedge_summary': local_summary,
            'local_cost_risk_observed': local_summary.get('local_cost_risk_observed'),
        })
    dispatch_count = sum(1 for e in events if e.get('event') == 'dispatch')
    outcome_count = sum(1 for e in events if e.get('event') in {'success', 'failure', 'timeout', 'timeout_cancel_result'})
    timeout_count = sum(1 for e in events if e.get('event') in {'timeout', 'timeout_cancel_result'})
    applied_count = sum(1 for g in per_goal if g.get('centerline_refinement_applied'))
    gate_no_apply_count = sum(1 for g in per_goal if g.get('dispatch_observed') and not g.get('centerline_refinement_applied'))
    regression = any(bool(g.get('branch_scoring_changed')) for g in per_goal)
    final_state = states[-1] if states else {}
    return {
        'replay_id': replay_id,
        'artifact_dir': str(replay_dir),
        'inner_ingress_goal_success': bool(ingress_action.get('success')),
        'inner_ingress_action_result': ingress_action,
        'state_sample_count': len(states),
        'goal_event_count': len(events),
        'runtime_timeline_sample_count': len(timeline),
        'dispatch_count': dispatch_count,
        'outcome_count': outcome_count,
        'timeout_count': timeout_count,
        'balance_first_applied_count': applied_count,
        'balance_first_gate_no_apply_count': gate_no_apply_count,
        'branch_scoring_changed': regression,
        'per_goal_summaries': per_goal,
        'cmd_vel_summary': _cmd_vel_summary(controller_rows),
        'nav2_feedback_summary': _nav2_feedback_summary(feedback_rows),
        'robot_progress_summary': _robot_progress_summary(controller_rows, next((e for e in events if e.get('event') == 'dispatch'), None)),
        'local_cost_footprint_front_wedge_summary': _local_cost_footprint_front_wedge_summary(timeline + local_rows, next((e for e in events if e.get('event') == 'dispatch'), None)),
        'final_mode': final_state.get('mode'),
        'final_goal_count': final_state.get('goal_count'),
        'complete_autonomous_success_claimed': False,
        'exit_success_claimed': False,
    }


def classify_phase70(replays: list[dict[str, Any]], guardrail_violation: bool = False) -> str:
    if guardrail_violation:
        return 'GUARDRAIL_VIOLATION_STRATEGY_CHANGED'
    if not replays or not any(r.get('inner_ingress_goal_success') for r in replays):
        return 'INSUFFICIENT_EVIDENCE'
    if any(r.get('branch_scoring_changed') for r in replays):
        return 'GUARDRAIL_VIOLATION_STRATEGY_CHANGED'
    if any((r.get('balance_first_applied_count') or 0) > 0 and (r.get('timeout_count') or 0) > 0 for r in replays):
        return 'BALANCE_FIRST_APPLIED_TIMEOUT_REMAINS'
    if any((r.get('balance_first_applied_count') or 0) > 0 and (r.get('dispatch_count') or 0) > 0 for r in replays):
        return 'BALANCE_FIRST_CENTERLINE_APPLIED_AND_IMPROVES'
    if any((r.get('balance_first_gate_no_apply_count') or 0) > 0 for r in replays):
        return 'BALANCE_FIRST_GATE_NO_APPLY'
    return 'INSUFFICIENT_EVIDENCE'


def analyze_phase70(artifact_dir: Path | str, output: Path | str | None = None) -> dict[str, Any]:
    artifact_path = Path(artifact_dir)
    replay_dirs = sorted(p for p in artifact_path.glob('replay_*') if p.is_dir())
    replays = [summarize_replay(path, RUN_ID) for path in replay_dirs]
    preflight_text = _read_text(artifact_path / f'{RUN_ID}_preflight.txt')
    nav2_config_diff_empty = not _read_text(artifact_path / f'{RUN_ID}_nav2_config_diff.txt').strip()
    cleanup_empty = not _read_text(artifact_path / f'{RUN_ID}_cleanup_processes_after.txt').strip()
    max_goals = _number(next((line.split('=', 1)[1] for line in preflight_text.splitlines() if line.startswith('MAX_GOALS=')), None))
    max_goals_ok = max_goals == 1
    guardrail_violation = bool((not nav2_config_diff_empty) or (not max_goals_ok) or any(r.get('branch_scoring_changed') for r in replays))
    classification = classify_phase70(replays, guardrail_violation)
    summary = {
        'phase': PHASE,
        'run_id': RUN_ID,
        'classification': classification,
        'allowed_classifications': ALLOWED_CLASSIFICATIONS,
        'active_world': ACTIVE_WORLD,
        'active_metadata': ACTIVE_METADATA,
        'phase69_classification_preserved': PHASE69_CLASSIFICATION,
        'inner_ingress_waypoint_map': INNER_INGRESS_WAYPOINT_MAP,
        'artifact_dir': str(artifact_path),
        'max_goals': int(max_goals) if max_goals is not None else None,
        'replay_count': len(replays),
        'replays': replays,
        'balance_first_applied_count': sum(int(r.get('balance_first_applied_count') or 0) for r in replays),
        'balance_first_gate_no_apply_count': sum(int(r.get('balance_first_gate_no_apply_count') or 0) for r in replays),
        'metrics': {
            'inner_ingress_goal_success': any(r.get('inner_ingress_goal_success') for r in replays),
            'dispatch_count_total': sum(int(r.get('dispatch_count') or 0) for r in replays),
            'outcome_count_total': sum(int(r.get('outcome_count') or 0) for r in replays),
            'timeout_count_total': sum(int(r.get('timeout_count') or 0) for r in replays),
            'balance_first_applied_count': sum(int(r.get('balance_first_applied_count') or 0) for r in replays),
            'balance_first_gate_no_apply_count': sum(int(r.get('balance_first_gate_no_apply_count') or 0) for r in replays),
        },
        'guardrails': GUARDRAILS,
        'guardrail_violation': guardrail_violation,
        'nav2_config_diff_empty': nav2_config_diff_empty,
        'cleanup_empty': cleanup_empty,
        'branch_scoring_changed': any(r.get('branch_scoring_changed') for r in replays),
        'complete_autonomous_success_claimed': False,
        'exit_success_claimed': False,
        'recommendations': ['Use Phase70 as bounded Goal1 evidence only; stop for human acceptance and do not enter Phase71.'],
    }
    if output is not None:
        out_path = Path(output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    return summary


def analyze(args: argparse.Namespace) -> int:
    summary = analyze_phase70(Path(args.artifact_dir), output=Path(args.output))
    print(json.dumps({'output': str(args.output), 'classification': summary['classification'], 'replay_count': summary['replay_count']}, sort_keys=True))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=PHASE)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--send-inner-ingress-goal', action='store_true')
    mode.add_argument('--record-runtime', action='store_true')
    mode.add_argument('--analyze', action='store_true')
    parser.add_argument('--artifact-dir', default=str(Path('log') / RUN_ID))
    parser.add_argument('--output', required=True)
    parser.add_argument('--goal-timeout-sec', type=float, default=100.0)
    parser.add_argument('--timeout-sec', type=float, default=180.0)
    parser.add_argument('--periodic-snapshot-sec', type=float, default=1.0)
    parser.add_argument('--timeline-output', default=str(Path('log') / RUN_ID / 'phase70_runtime_timeline.jsonl'))
    parser.add_argument('--controller-dynamics-output', default=str(Path('log') / RUN_ID / 'phase70_controller_dynamics.jsonl'))
    parser.add_argument('--nav2-feedback-output', default=str(Path('log') / RUN_ID / 'phase70_nav2_feedback.jsonl'))
    parser.add_argument('--local-costmap-samples-output', default=str(Path('log') / RUN_ID / 'phase70_local_costmap_samples.jsonl'))
    parser.add_argument('--global-plan-samples-output', default=str(Path('log') / RUN_ID / 'phase70_global_plan_samples.jsonl'))
    parser.add_argument('--collision-monitor-output', default=str(Path('log') / RUN_ID / 'phase70_collision_monitor_state.jsonl'))
    parser.add_argument('--local-costmap-topic', default='/local_costmap/costmap')
    parser.add_argument('--path-topic', default='/plan')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--cmd-topic', default='/cmd_vel')
    parser.add_argument('--cmd-smoothed-topic', default='/cmd_vel_smoothed')
    parser.add_argument('--goal-events-topic', default='/maze/goal_events')
    parser.add_argument('--collision-monitor-topic', default='/collision_monitor_state')
    parser.add_argument('--nav2-feedback-topic', default='/navigate_to_pose/_action/feedback')
    args = parser.parse_args(argv)
    if args.send_inner_ingress_goal:
        return send_inner_ingress_goal(args)
    if args.record_runtime:
        return record_runtime(args)
    return analyze(args)


if __name__ == '__main__':
    raise SystemExit(main())
