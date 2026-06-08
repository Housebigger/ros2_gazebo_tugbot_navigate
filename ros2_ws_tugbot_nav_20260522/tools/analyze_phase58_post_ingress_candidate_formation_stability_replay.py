#!/usr/bin/env python3
"""Phase58 Post-Ingress Candidate Formation Stability Replay analyzer/recorder.

Phase58 diagnoses candidate-formation stability across bounded post-ingress
replays. It is read-only diagnostics: no Nav2/MPPI/controller parameter edits,
no clearance_radius_m tuning, no map sufficiency threshold tuning, no
maze_explorer strategy change, no fallback/terminal acceptance, no old scaffold
world/map, bounded replay only, max_goals=1, no autonomous exploration success
claim, and first dispatch is not exit success. It also does not attribute the
Phase57 first-dispatch timeout; Phase57 remains TIMEOUT_INCONCLUSIVE_DATA_GAP.

Artifact contract includes per-replay ingress action result, explorer_state.jsonl,
goal_events.jsonl, stability_evidence.jsonl with map/scan/TF/local/global costmap
snapshots, and an aggregate summary JSON.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from pathlib import Path
from typing import Any, Callable

PHASE = 'Phase58 Post-Ingress Candidate Formation Stability Replay'
RUN_ID = 'phase58_post_ingress_candidate_formation_stability_replay'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
ACTIVE_METADATA = 'maze_20260528_scaled_instance.yaml'
INGRESS_WAYPOINT_MAP = {'x_m': 1.0, 'y_m': 0.0, 'yaw_rad': 0.0}
ACCEPTANCE_RADIUS_M = 0.35
ALLOWED_CLASSIFICATIONS = {
    'CANDIDATE_FORMATION_STABLE_DISPATCH_REPRODUCED',
    'CANDIDATE_FORMATION_STABLE_NO_DISPATCH',
    'CANDIDATE_FORMATION_UNSTABLE_POSE_OR_TIMING_SENSITIVE',
    'CANDIDATE_FORMATION_UNSTABLE_COSTMAP_SENSITIVE',
    'CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE',
    'CANDIDATE_FORMATION_STABILITY_INCONCLUSIVE',
    'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
}

try:  # ROS imports are optional for offline unit tests and analysis.
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan
    import tf2_ros
except Exception:  # pragma: no cover
    rclpy = None
    ActionClient = None
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
    LaserScan = None
    tf2_ros = None


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
            rows.append(row)
    return rows


def read_payload_jsonl(path: Path | None) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for row in _read_jsonl(path):
        payload = row.get('state') if isinstance(row.get('state'), dict) else row
        if isinstance(payload, dict):
            payload = dict(payload)
            payload.setdefault('_recorder_elapsed_sec', row.get('elapsed_sec'))
            payload.setdefault('_recorder_wall_time', row.get('wall_time'))
            payload.setdefault('_recorder_seq', row.get('seq'))
            out.append(payload)
    return out


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _quat_yaw(q: Any) -> float:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _status_text(status: int) -> str:
    mapping = {
        0: 'STATUS_UNKNOWN', 1: 'STATUS_ACCEPTED', 2: 'STATUS_EXECUTING',
        3: 'STATUS_CANCELING', 4: 'STATUS_SUCCEEDED', 5: 'STATUS_CANCELED',
        6: 'STATUS_ABORTED',
    }
    return mapping.get(status, str(status))


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


def _robot_pose_from_tf(tf_lookups: dict[str, Any], odom: dict[str, Any] | None = None) -> list[float] | None:
    tf = tf_lookups.get('map->base_link') or {}
    if tf.get('available'):
        t = tf.get('translation') or {}
        return [float(t.get('x', 0.0)), float(t.get('y', 0.0)), float(tf.get('yaw_rad', 0.0))]
    if odom:
        pose = odom.get('pose') or {}
        return [float(pose.get('x', 0.0)), float(pose.get('y', 0.0)), float(pose.get('yaw_rad', 0.0))]
    return None


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


def _grid_near_robot_window(msg: Any, robot_pose: list[float], radius_m: float, *, include_out_of_bounds: bool) -> dict[str, Any]:
    width = int(msg.info.width)
    height = int(msg.info.height)
    resolution = float(msg.info.resolution)
    origin_x = float(msg.info.origin.position.x)
    origin_y = float(msg.info.origin.position.y)
    data = [int(v) for v in msg.data]
    cx = int(math.floor((float(robot_pose[0]) - origin_x) / resolution))
    cy = int(math.floor((float(robot_pose[1]) - origin_y) / resolution))
    radius_cells = int(math.ceil(radius_m / max(resolution, 1e-9)))
    counts = {'sample_count': 0, 'in_bounds_count': 0, 'out_of_bounds_count': 0, 'unknown_count': 0, 'free_count': 0, 'occupied_count': 0}
    for yy in range(cy - radius_cells, cy + radius_cells + 1):
        for xx in range(cx - radius_cells, cx + radius_cells + 1):
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
        'known_count': known,
        'known_ratio': known / total if total else 0.0,
        'free_ratio': counts['free_count'] / total if total else 0.0,
        'occupied_ratio': counts['occupied_count'] / total if total else 0.0,
        'unknown_ratio': counts['unknown_count'] / total if total else 0.0,
        'center_cell': [cx, cy],
        'robot_cell': [cx, cy],
        'robot_in_bounds': bool(0 <= cx < width and 0 <= cy < height),
        'radius_m': radius_m,
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
            'min_x': origin_x, 'max_x': origin_x + width * resolution,
            'min_y': origin_y, 'max_y': origin_y + height * resolution,
            'width_m': width * resolution, 'height_m': height * resolution,
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


class IngressGoalClient(Node):  # pragma: no cover - requires ROS graph
    def __init__(self, output: Path, goal_timeout_sec: float):
        super().__init__('phase58_ingress_goal_client')
        self.output = output
        self.goal_timeout_sec = float(goal_timeout_sec)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.result: dict[str, Any] = {
            'phase': PHASE, 'run_id': RUN_ID, 'ingress_waypoint_map': INGRESS_WAYPOINT_MAP,
            'goal_sent': False, 'goal_accepted': False, 'result_received': False,
            'success': False, 'feedback': [], 'wall_time_start': time.time(),
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
            'wall_time_result': time.time(),
            'result_received': True,
            'status': status,
            'status_text': _status_text(status),
            'error_code': error_code,
            'error_msg': error_msg,
            'success': bool(GoalStatus is not None and status == int(GoalStatus.STATUS_SUCCEEDED) and error_code == 0),
        })
        return self.result

    def _feedback(self, feedback_msg: Any) -> None:
        feedback = feedback_msg.feedback
        nav_time = getattr(feedback, 'navigation_time', None)
        self.result['feedback'].append({
            'wall_time': time.time(),
            'navigation_time_sec': float(getattr(nav_time, 'sec', 0)) + float(getattr(nav_time, 'nanosec', 0)) / 1e9,
            'distance_remaining': float(getattr(feedback, 'distance_remaining', math.nan)),
            'number_of_recoveries': int(getattr(feedback, 'number_of_recoveries', 0)),
        })


class StabilityEvidenceRecorder(Node):  # pragma: no cover - requires ROS graph
    def __init__(self, args: argparse.Namespace):
        super().__init__('phase58_stability_evidence_recorder')
        self.output = Path(args.output)
        self.timeout_sec = float(args.timeout_sec)
        self.periodic_snapshot_sec = float(args.periodic_snapshot_sec)
        self.started = time.monotonic()
        self.seq = 0
        self.map_msg = None
        self.local_costmap_msg = None
        self.global_costmap_msg = None
        self.scan_msg = None
        self.odom_msg = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        default_qos = QoSProfile(depth=10)
        self.create_subscription(OccupancyGrid, '/map', lambda msg: setattr(self, 'map_msg', msg), map_qos)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda msg: setattr(self, 'local_costmap_msg', msg), map_qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda msg: setattr(self, 'global_costmap_msg', msg), map_qos)
        self.create_subscription(LaserScan, '/scan', lambda msg: setattr(self, 'scan_msg', msg), sensor_qos)
        self.create_subscription(Odometry, '/odom', lambda msg: setattr(self, 'odom_msg', msg), default_qos)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output_handle = self.output.open('a', encoding='utf-8')
        self.timer = self.create_timer(self.periodic_snapshot_sec, self._snapshot)

    def close(self) -> None:
        try:
            self.output_handle.close()
        except Exception:
            pass

    def _snapshot(self) -> None:
        elapsed = time.monotonic() - self.started
        if elapsed > self.timeout_sec:
            rclpy.shutdown()
            return
        tf_lookups = {
            'map->base_link': _lookup_transform(self.tf_buffer, 'map', 'base_link'),
            'odom->base_link': _lookup_transform(self.tf_buffer, 'odom', 'base_link'),
            'map->odom': _lookup_transform(self.tf_buffer, 'map', 'odom'),
        }
        odom = _odom_summary(self.odom_msg)
        robot_pose = _robot_pose_from_tf(tf_lookups, odom)
        row = {
            'seq': self.seq,
            'elapsed_sec': elapsed,
            'wall_time': time.time(),
            'robot_pose_map': robot_pose,
            'ingress_final_pose_map': robot_pose,
            'ingress_final_yaw_rad': robot_pose[2] if robot_pose else None,
            'distance_to_ingress_m': math.hypot(robot_pose[0] - 1.0, robot_pose[1] - 0.0) if robot_pose else None,
            'odom': odom,
            'tf_evidence_at_topology': tf_lookups,
            'tf_lookups': tf_lookups,
            'map_evidence_at_topology': _grid_summary(self.map_msg, robot_pose),
            'map': _grid_summary(self.map_msg, robot_pose),
            'local_costmap_evidence_at_topology': _grid_summary(self.local_costmap_msg, robot_pose),
            'local_costmap': _grid_summary(self.local_costmap_msg, robot_pose),
            'global_costmap_evidence_at_topology': _grid_summary(self.global_costmap_msg, robot_pose),
            'global_costmap': _grid_summary(self.global_costmap_msg, robot_pose),
            'scan_evidence_at_topology': _scan_summary(self.scan_msg),
            'scan': _scan_summary(self.scan_msg),
        }
        self.output_handle.write(json.dumps(row, sort_keys=True) + '\n')
        self.output_handle.flush()
        self.seq += 1


def send_ingress_goal(args: argparse.Namespace) -> int:  # pragma: no cover
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


def record_stability(args: argparse.Namespace) -> int:  # pragma: no cover
    if rclpy is None:
        raise SystemExit('rclpy is required for --record-stability')
    rclpy.init()
    node = StabilityEvidenceRecorder(args)
    try:
        while rclpy.ok() and (time.monotonic() - node.started) <= node.timeout_sec + 1.0:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps({'output': str(args.output), 'sample_count': node.seq}, sort_keys=True))
    return 0


def _first_state_with(rows: list[dict[str, Any]], pred: Callable[[dict[str, Any]], bool]) -> dict[str, Any] | None:
    for row in rows:
        try:
            if pred(row):
                return row
        except Exception:
            pass
    return None


def _nearest_by_wall_time(rows: list[dict[str, Any]], wall_time: float | None) -> dict[str, Any] | None:
    if wall_time is None or not rows:
        return rows[-1] if rows else None
    return min(rows, key=lambda r: abs(float(r.get('wall_time') or 0.0) - wall_time))


def _distance(a: list[float] | None, b: list[float] | None) -> float | None:
    if not a or not b:
        return None
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _number(value: Any) -> float | None:
    try:
        out = float(value)
        return out if math.isfinite(out) else None
    except Exception:
        return None


def _extract_phase56_payload(first_topology_state: dict[str, Any] | None) -> tuple[dict[str, Any], dict[str, Any]]:
    if not isinstance(first_topology_state, dict):
        return {}, {}
    topology = first_topology_state.get('last_topology_sampling_diagnostics')
    if not isinstance(topology, dict):
        topology = {}
    phase56 = first_topology_state.get('phase56_open_direction_to_candidate_diagnostics')
    if not isinstance(phase56, dict):
        phase56 = topology.get('phase56_open_direction_to_candidate_diagnostics')
    if not isinstance(phase56, dict):
        phase56 = {}
    return topology, phase56


def summarize_replay(replay_dir: Path, aggregate_dir: Path) -> dict[str, Any]:
    replay_id = replay_dir.name
    action = _safe_json_load(replay_dir / f'{RUN_ID}_{replay_id}_ingress_navigate_to_pose_action_result.json', {})
    if not action:
        action = _safe_json_load(replay_dir / 'ingress_navigate_to_pose_action_result.json', {})
    explorer_states = read_payload_jsonl(replay_dir / f'{RUN_ID}_{replay_id}_explorer_state.jsonl')
    goal_events = read_payload_jsonl(replay_dir / f'{RUN_ID}_{replay_id}_goal_events.jsonl')
    evidence_rows = _read_jsonl(replay_dir / f'{RUN_ID}_{replay_id}_stability_evidence.jsonl')
    preflight = _read_text(replay_dir / f'{RUN_ID}_{replay_id}_preflight.txt') + '\n' + _read_text(aggregate_dir / f'{RUN_ID}_preflight.txt')
    cleanup_text = _read_text(aggregate_dir / f'{RUN_ID}_cleanup_processes_after.txt')

    first_gate_ready_state = _first_state_with(explorer_states, lambda row: bool(row.get('dispatch_readiness_gate_passed')))
    first_topology_state = _first_state_with(explorer_states, lambda row: bool(row.get('last_topology_sampling_diagnostics')))
    first_dispatch_event = _first_state_with(goal_events, lambda row: row.get('event') == 'dispatch')
    topology, phase56 = _extract_phase56_payload(first_topology_state)
    topology_wall = _number(first_topology_state.get('_recorder_wall_time')) if first_topology_state else None
    topology_evidence = _nearest_by_wall_time(evidence_rows, topology_wall)
    final_evidence = evidence_rows[-1] if evidence_rows else {}

    ingress_final_pose_map = final_evidence.get('robot_pose_map') if isinstance(final_evidence, dict) else None
    ingress_final_yaw_rad = ingress_final_pose_map[2] if isinstance(ingress_final_pose_map, list) and len(ingress_final_pose_map) > 2 else None
    topology_pose = topology.get('robot_pose_map') or (first_topology_state or {}).get('robot_pose_map')
    gate_wall = _number(first_gate_ready_state.get('_recorder_wall_time')) if first_gate_ready_state else None
    explorer_start_wall = None
    if explorer_states:
        explorer_start_wall = _number(explorer_states[0].get('_recorder_wall_time'))
    ingress_success_wall = _number(action.get('wall_time_result'))
    dispatch_observed = first_dispatch_event is not None
    candidate_local_costmap_cell_state = phase56.get('candidate_local_costmap_cell_state') if isinstance(phase56, dict) else None

    return {
        'replay_id': replay_id,
        'artifact_dir': str(replay_dir),
        'ingress_success': bool(action.get('success')),
        'ingress_action_status_text': action.get('status_text'),
        'ingress_final_pose_map': ingress_final_pose_map,
        'ingress_final_yaw_rad': ingress_final_yaw_rad,
        'time_from_ingress_success_to_explorer_start_sec': (explorer_start_wall - ingress_success_wall) if explorer_start_wall and ingress_success_wall else None,
        'time_from_explorer_start_to_gate_ready_sec': (gate_wall - explorer_start_wall) if gate_wall and explorer_start_wall else None,
        'time_from_explorer_start_to_first_topology_sec': (topology_wall - explorer_start_wall) if topology_wall and explorer_start_wall else None,
        'robot_pose_at_first_topology_sampling': topology_pose,
        'pose_delta_ingress_final_to_topology_m': _distance(ingress_final_pose_map, topology_pose),
        'dispatch_observed': bool(dispatch_observed),
        'goal_events_sample_count': len(goal_events),
        'explorer_state_sample_count': len(explorer_states),
        'stability_evidence_sample_count': len(evidence_rows),
        'first_dispatch_event': first_dispatch_event,
        'first_topology_state': first_topology_state,
        'topology_sampling_diagnostics': topology,
        'map_evidence_at_topology': (topology_evidence or {}).get('map_evidence_at_topology') or (topology_evidence or {}).get('map'),
        'scan_evidence_at_topology': (topology_evidence or {}).get('scan_evidence_at_topology') or (topology_evidence or {}).get('scan'),
        'tf_evidence_at_topology': (topology_evidence or {}).get('tf_evidence_at_topology') or (topology_evidence or {}).get('tf_lookups'),
        'local_costmap_evidence_at_topology': (topology_evidence or {}).get('local_costmap_evidence_at_topology') or (topology_evidence or {}).get('local_costmap'),
        'global_costmap_evidence_at_topology': (topology_evidence or {}).get('global_costmap_evidence_at_topology') or (topology_evidence or {}).get('global_costmap'),
        'raw_open_direction_count': topology.get('raw_open_direction_count') or phase56.get('raw_open_direction_count'),
        'filtered_open_direction_count': topology.get('filtered_open_direction_count') or phase56.get('filtered_open_direction_count'),
        'accepted_open_direction_angle_rad': phase56.get('accepted_open_direction_angle_rad'),
        'accepted_open_direction_angle_deg': phase56.get('accepted_open_direction_angle_deg'),
        'accepted_open_direction_vector': phase56.get('accepted_open_direction_vector'),
        'candidate_before_filter_count': phase56.get('candidate_before_filter_count') if 'candidate_before_filter_count' in phase56 else topology.get('candidate_before_filter_count'),
        'candidate_after_filter_count': phase56.get('candidate_after_filter_count') if 'candidate_after_filter_count' in phase56 else topology.get('candidate_after_filter_count'),
        'candidate_branch_count': topology.get('candidate_branch_count'),
        'branch_candidate_rejection_reason': phase56.get('branch_candidate_rejection_reason'),
        'junction_or_dead_end_policy_filter': phase56.get('junction_or_dead_end_policy_filter'),
        'duplicate_or_exhausted_filter': phase56.get('duplicate_or_exhausted_filter'),
        'candidate_branch_states': phase56.get('candidate_branch_states'),
        'candidate_local_costmap_cell_state': candidate_local_costmap_cell_state,
        'candidate_local_cost_value': (candidate_local_costmap_cell_state or {}).get('value') if isinstance(candidate_local_costmap_cell_state, dict) else None,
        'max_radius_cost': (candidate_local_costmap_cell_state or {}).get('max_radius_cost') if isinstance(candidate_local_costmap_cell_state, dict) else None,
        'candidate_map_cell_state': phase56.get('candidate_map_cell_state'),
        'candidate_goal_point': phase56.get('candidate_goal_point'),
        'chosen_candidate_goal_point': phase56.get('chosen_candidate_goal_point'),
        'reject_reason_counts': topology.get('reject_reason_counts'),
        'mode_at_first_topology': (first_topology_state or {}).get('mode'),
        'goal_count_at_first_topology': (first_topology_state or {}).get('goal_count'),
        'guardrail_preflight_ok': all(s in preflight for s in ['max_goals=1', 'near_exit_fallback_enabled=false', 'startup_warmup_no_dispatch=false']),
        'cleanup_empty': cleanup_text.strip() == '',
    }


def _spread(values: list[float]) -> float | None:
    values = [v for v in values if v is not None and math.isfinite(v)]
    if not values:
        return None
    return max(values) - min(values)


def compare_replays(replays: list[dict[str, Any]]) -> dict[str, Any]:
    dispatch = [r for r in replays if r.get('dispatch_observed')]
    no_dispatch = [r for r in replays if not r.get('dispatch_observed')]
    poses = [r.get('robot_pose_at_first_topology_sampling') for r in replays if isinstance(r.get('robot_pose_at_first_topology_sampling'), list)]
    pose_spread = 0.0
    if len(poses) >= 2:
        pose_spread = max(_distance(a, b) or 0.0 for a in poses for b in poses)
    timing_values = [_number(r.get('time_from_explorer_start_to_first_topology_sec')) for r in replays]
    timing_spread = _spread([v for v in timing_values if v is not None])
    local_costs = [_number(r.get('candidate_local_cost_value')) for r in replays]
    max_radius = [_number(r.get('max_radius_cost')) for r in replays]
    local_cost_spread = _spread([v for v in local_costs if v is not None])
    max_radius_spread = _spread([v for v in max_radius if v is not None])
    dead_end_policy_values = sorted({str(r.get('junction_or_dead_end_policy_filter')) for r in replays})
    rejection_values = sorted({str(r.get('branch_candidate_rejection_reason')) for r in replays})
    candidate_counts = [int(r.get('candidate_after_filter_count') or 0) for r in replays]
    raw_counts = [int(r.get('raw_open_direction_count') or 0) for r in replays]
    filtered_counts = [int(r.get('filtered_open_direction_count') or 0) for r in replays]
    difference_hypothesis = 'not_enough_replays_or_no_topology_evidence'
    if dispatch and no_dispatch:
        if len(dead_end_policy_values) > 1 or 'dead_end_policy_before_candidate_generation' in dead_end_policy_values:
            difference_hypothesis = 'dead_end_policy_sensitive'
        elif pose_spread and pose_spread > 0.05 or (timing_spread is not None and timing_spread > 2.0):
            difference_hypothesis = 'pose_or_timing_sensitive'
        elif (local_cost_spread is not None and local_cost_spread >= 10.0) or (max_radius_spread is not None and max_radius_spread >= 10.0):
            difference_hypothesis = 'costmap_sensitive'
        else:
            difference_hypothesis = 'unknown_mixed_dispatch_outcome'
    elif dispatch and not no_dispatch:
        difference_hypothesis = 'stable_dispatch_reproduced'
    elif no_dispatch and not dispatch and replays:
        if any('dead_end_policy' in str(r.get('branch_candidate_rejection_reason')) or 'dead_end_policy' in str(r.get('junction_or_dead_end_policy_filter')) for r in replays):
            difference_hypothesis = 'stable_no_dispatch_dead_end_policy'
        else:
            difference_hypothesis = 'stable_no_dispatch_unknown'
    return {
        'compare_replays': True,
        'replay_count': len(replays),
        'dispatch_count': len(dispatch),
        'no_dispatch_count': len(no_dispatch),
        'mixed_dispatch_and_no_dispatch': bool(dispatch and no_dispatch),
        'pose_spread_at_topology_m': pose_spread,
        'timing_spread_explorer_start_to_topology_sec': timing_spread,
        'candidate_local_cost_value_spread': local_cost_spread,
        'candidate_local_cost_max_radius_spread': max_radius_spread,
        'raw_open_direction_counts': raw_counts,
        'filtered_open_direction_counts': filtered_counts,
        'candidate_after_filter_counts': candidate_counts,
        'dead_end_junction_policy_states': dead_end_policy_values,
        'rejection_reasons': rejection_values,
        'difference_hypothesis': difference_hypothesis,
    }


def classify(replays: list[dict[str, Any]], comparison: dict[str, Any], guardrail_violation: bool) -> str:
    if guardrail_violation:
        return 'GUARDRAIL_VIOLATION_STRATEGY_CHANGED'
    if len(replays) < 2:
        return 'CANDIDATE_FORMATION_STABILITY_INCONCLUSIVE'
    if comparison.get('dispatch_count') == len(replays):
        return 'CANDIDATE_FORMATION_STABLE_DISPATCH_REPRODUCED'
    if comparison.get('no_dispatch_count') == len(replays):
        return 'CANDIDATE_FORMATION_STABLE_NO_DISPATCH'
    if comparison.get('mixed_dispatch_and_no_dispatch'):
        hyp = comparison.get('difference_hypothesis')
        if hyp == 'dead_end_policy_sensitive':
            return 'CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE'
        if hyp == 'costmap_sensitive':
            return 'CANDIDATE_FORMATION_UNSTABLE_COSTMAP_SENSITIVE'
        if hyp == 'pose_or_timing_sensitive':
            return 'CANDIDATE_FORMATION_UNSTABLE_POSE_OR_TIMING_SENSITIVE'
        return 'CANDIDATE_FORMATION_STABILITY_INCONCLUSIVE'
    return 'CANDIDATE_FORMATION_STABILITY_INCONCLUSIVE'


def analyze(args: argparse.Namespace) -> int:
    artifact_dir = Path(args.artifact_dir)
    replay_dirs = sorted(p for p in artifact_dir.glob('replay_*') if p.is_dir())
    replays = [summarize_replay(p, artifact_dir) for p in replay_dirs]
    preflight_text = _read_text(artifact_dir / f'{RUN_ID}_preflight.txt')
    nav2_diff_text = _read_text(artifact_dir / f'{RUN_ID}_nav2_config_diff.txt')
    cleanup_text = _read_text(artifact_dir / f'{RUN_ID}_cleanup_processes_after.txt')
    guardrail_violation = bool(
        nav2_diff_text.strip()
        or 'complete autonomous success' in preflight_text.lower()
        or 'max_goals=1' not in preflight_text
        or 'near_exit_fallback_enabled=false' not in preflight_text
        or 'startup_warmup_no_dispatch=false' not in preflight_text
    )
    comparison = compare_replays(replays)
    classification = classify(replays, comparison, guardrail_violation)
    recommendations = []
    if comparison.get('mixed_dispatch_and_no_dispatch'):
        recommendations = [
            'post-ingress stable wait before topology sampling',
            'pose alignment check at explorer start and first topology sampling',
            'multi-frame topology consistency check before dead_end_policy terminalization',
        ]
    elif classification == 'CANDIDATE_FORMATION_STABLE_NO_DISPATCH':
        recommendations = ['investigate dead_end/junction policy classification at ingress pose before changing parameters']
    else:
        recommendations = ['preserve bounded replay diagnostics; do not tune Nav2/MPPI/controller or exploration strategy without new evidence']
    summary = {
        'phase': PHASE,
        'run_id': RUN_ID,
        'classification': classification,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'artifact_dir': str(artifact_dir),
        'replay_count': len(replays),
        'replays': replays,
        'comparison': comparison,
        'guardrail_violation': guardrail_violation,
        'nav2_config_diff_empty': nav2_diff_text.strip() == '',
        'cleanup_empty': cleanup_text.strip() == '',
        'complete_autonomous_success_claimed': False,
        'first_dispatch_timeout_attributed': False,
        'phase57_conclusion_preserved': 'TIMEOUT_INCONCLUSIVE_DATA_GAP',
        'recommendations': recommendations,
        'guardrails': [
            'active scaled2x world only',
            'bounded replay only',
            'max_goals=1',
            'no Nav2/MPPI/controller parameter edits',
            'no clearance_radius_m tuning',
            'no map sufficiency threshold tuning',
            'no maze_explorer strategy change',
            'no fallback/terminal acceptance',
            'no autonomous exploration success claim',
            'first dispatch is not exit success',
            'does not attribute first-dispatch timeout',
        ],
    }
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    print(json.dumps(summary, sort_keys=True))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=PHASE)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--send-ingress-goal', action='store_true')
    mode.add_argument('--record-stability', action='store_true')
    mode.add_argument('--analyze', action='store_true')
    parser.add_argument('--output', required=True)
    parser.add_argument('--artifact-dir', default='log/phase58_post_ingress_candidate_formation_stability_replay/')
    parser.add_argument('--goal-timeout-sec', type=float, default=100.0)
    parser.add_argument('--timeout-sec', type=float, default=110.0)
    parser.add_argument('--periodic-snapshot-sec', type=float, default=1.0)
    args = parser.parse_args(argv)
    if args.send_ingress_goal:
        return send_ingress_goal(args)
    if args.record_stability:
        return record_stability(args)
    return analyze(args)


if __name__ == '__main__':
    raise SystemExit(main())
