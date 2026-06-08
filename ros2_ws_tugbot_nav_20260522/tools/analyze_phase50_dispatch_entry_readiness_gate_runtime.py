#!/usr/bin/env python3
"""Phase50 dispatch-entry readiness gate bounded runtime recorder/analyzer.

This phase validates the Phase49 maze_explorer readiness gate in a bounded
active scaled2x runtime. It does not tune clearance, Nav2, MPPI, or controller
parameters and it does not claim autonomous exploration success.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

PHASE = 'Phase50 Dispatch Entry Readiness Gate Bounded Runtime Validation'
RUN_ID = 'phase50_dispatch_entry_readiness_gate_bounded_runtime'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
ACTIVE_METADATA = 'maze_20260528_scaled_instance.yaml'
MAP_ENTRANCE = {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
MAP_EXIT = {'x_m': 21.072562, 'y_m': 18.083566, 'radius_m': 1.2}
TF_LOOKUP_PAIRS = ('map->base_link', 'odom->base_link', 'map->odom')
PHASE50_ALLOWED_CLASSIFICATIONS = {
    'GATE_VALIDATED_TOPOLOGY_SAMPLING_AFTER_READY',
    'GATE_VALIDATED_CLEARANCE_GEOMETRY_NEXT_LAYER',
    'GATE_WAITING_FOR_READINESS_DATA',
    'GATE_BYPASS_OR_EARLY_TOPOLOGY_BUG',
    'INSUFFICIENT_RUNTIME_EVIDENCE',
}

try:  # Runtime imports are optional for offline analyzer tests.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan
    import tf2_ros
except Exception:  # pragma: no cover
    rclpy = None
    Node = object
    QoSProfile = None
    ReliabilityPolicy = None
    DurabilityPolicy = None
    HistoryPolicy = None
    OccupancyGrid = None
    Odometry = None
    LaserScan = None
    tf2_ros = None


def _safe_json_loads(text: str) -> Any | None:
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return None


def _read_jsonl_payloads(path: Path | None) -> list[dict[str, Any]]:
    if not path or not path.exists() or not path.stat().st_size:
        return []
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        row = _safe_json_loads(line)
        if not isinstance(row, dict):
            continue
        payload = row.get('state') if isinstance(row.get('state'), dict) else row
        payload['_recorder_elapsed_sec'] = row.get('elapsed_sec')
        payload['_recorder_wall_time'] = row.get('wall_time')
        payload['_recorder_seq'] = row.get('seq')
        rows.append(payload)
    return rows


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
    sample_count = 0
    in_bounds_count = 0
    out_of_bounds_count = 0
    unknown = 0
    occupied = 0
    free = 0
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
                out_of_bounds_count += 1
                if include_out_of_bounds:
                    sample_count += 1
                    unknown += 1
                continue
            in_bounds_count += 1
            sample_count += 1
            value = int(data[yy * width + xx])
            if value < 0:
                unknown += 1
            elif value >= 65:
                occupied += 1
            else:
                free += 1
    known = free + occupied
    total = sample_count
    return {
        'total': total,
        'sample_count': total,
        'unknown': unknown,
        'free': free,
        'occupied': occupied,
        'known_count': known,
        'free_count': free,
        'occupied_count': occupied,
        'unknown_count': unknown,
        'in_bounds_count': in_bounds_count,
        'out_of_bounds_count': out_of_bounds_count,
        'out_of_bounds_as_unknown': bool(include_out_of_bounds),
        'known_ratio': float(known / total) if total else 0.0,
        'free_ratio': float(free / total) if total else 0.0,
        'occupied_ratio': float(occupied / total) if total else 0.0,
        'unknown_ratio': float(unknown / total) if total else 0.0,
        'center_cell': [cx, cy],
        'robot_cell': [cx, cy],
        'center_in_bounds': bool(0 <= cx < width and 0 <= cy < height),
        'radius_m': radius_m,
        'radius_cells': radius_cells,
        'sample_window': {'min_x': min_x, 'max_x': max_x, 'min_y': min_y, 'max_y': max_y},
        'bbox_world': {
            'min_x': origin_x + min_x * resolution,
            'max_x': origin_x + (max_x + 1) * resolution,
            'min_y': origin_y + min_y * resolution,
            'max_y': origin_y + (max_y + 1) * resolution,
        },
    }


def _grid_summary(msg: Any | None, robot_pose: list[float] | None = None, radius_m: float = 1.0) -> dict[str, Any]:
    out = {
        'available': msg is not None,
        'frame_id': None,
        'width': 0,
        'height': 0,
        'resolution': None,
        'origin': None,
        'map_stamp': None,
        'full': {'total': 0, 'unknown': 0, 'free': 0, 'occupied': 0, 'known_ratio': 0.0, 'free_ratio': 0.0, 'occupied_ratio': 0.0, 'unknown_ratio': 0.0},
        'near_robot': None,
        'inclusive_near_robot': None,
        'in_bounds_near_robot': None,
    }
    if msg is None:
        return out
    width = int(msg.info.width)
    height = int(msg.info.height)
    resolution = float(msg.info.resolution)
    origin_x = float(msg.info.origin.position.x)
    origin_y = float(msg.info.origin.position.y)
    data = [int(v) for v in msg.data]

    def ratios(values: list[int]) -> dict[str, Any]:
        total = len(values)
        unknown = sum(1 for v in values if int(v) < 0)
        occupied = sum(1 for v in values if int(v) >= 65)
        free = max(0, total - unknown - occupied)
        return {
            'total': total,
            'unknown': unknown,
            'free': free,
            'occupied': occupied,
            'known_ratio': float((total - unknown) / total) if total else 0.0,
            'free_ratio': float(free / total) if total else 0.0,
            'occupied_ratio': float(occupied / total) if total else 0.0,
            'unknown_ratio': float(unknown / total) if total else 0.0,
        }

    out.update({'frame_id': str(msg.header.frame_id), 'width': width, 'height': height, 'resolution': resolution, 'origin': {'x': origin_x, 'y': origin_y}, 'map_stamp': float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9})
    out['full'] = ratios(data)
    if robot_pose and width > 0 and height > 0 and resolution > 0:
        inclusive = _grid_near_robot_window(msg, robot_pose, radius_m, include_out_of_bounds=True)
        in_bounds_only = _grid_near_robot_window(msg, robot_pose, radius_m, include_out_of_bounds=False)
        out['inclusive_near_robot'] = inclusive
        out['in_bounds_near_robot'] = in_bounds_only
        # Preserve the Phase50 legacy field name for report compatibility. It
        # intentionally excluded out-of-bounds cells; Phase51 compares it
        # against inclusive_near_robot, which matches the maze_explorer gate.
        out['near_robot'] = in_bounds_only
    return out


def _scan_summary(msg: Any | None) -> dict[str, Any]:
    out = {'available': msg is not None, 'frame_id': None, 'sample_count': 0, 'finite_count': 0, 'nearest_obstacle_m': None, 'range_min': None, 'range_max': None}
    if msg is None:
        return out
    finite = [float(v) for v in msg.ranges if math.isfinite(float(v))]
    out.update({
        'frame_id': str(msg.header.frame_id),
        'sample_count': len(msg.ranges),
        'finite_count': len(finite),
        'nearest_obstacle_m': min(finite) if finite else None,
        'range_min': float(msg.range_min),
        'range_max': float(msg.range_max),
    })
    return out


class Phase50RuntimeRecorder(Node):  # type: ignore[misc]
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('phase50_dispatch_entry_readiness_runtime_recorder')
        self.args = args
        self.started_at = time.time()
        self.latest_map = None
        self.latest_local_costmap = None
        self.latest_global_costmap = None
        self.latest_scan = None
        self.latest_odom = None
        self.snapshots: list[dict[str, Any]] = []
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, '/map', lambda msg: setattr(self, 'latest_map', msg), map_qos)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda msg: setattr(self, 'latest_local_costmap', msg), 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda msg: setattr(self, 'latest_global_costmap', msg), 10)
        self.create_subscription(LaserScan, '/scan', lambda msg: setattr(self, 'latest_scan', msg), sensor_qos)
        self.create_subscription(Odometry, '/odom', lambda msg: setattr(self, 'latest_odom', msg), 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(max(float(args.snapshot_period_sec), 1.0), self.capture_snapshot)

    def capture_snapshot(self) -> None:
        elapsed = time.time() - self.started_at
        tf_lookups = {}
        for pair in TF_LOOKUP_PAIRS:
            parent, child = pair.split('->', 1)
            tf_lookups[pair] = _lookup_transform(self.tf_buffer, parent, child)
        odom = _odom_summary(self.latest_odom)
        robot_pose = _robot_pose_from_tf(tf_lookups, odom)
        self.snapshots.append({
            'elapsed_sec': elapsed,
            'tf_lookups': tf_lookups,
            'odom': odom,
            'robot_pose_in_map': robot_pose,
            'robot_to_active_entrance_m': math.hypot(robot_pose[0] - MAP_ENTRANCE['x_m'], robot_pose[1] - MAP_ENTRANCE['y_m']) if robot_pose else None,
            'map': _grid_summary(self.latest_map, robot_pose),
            'local_costmap': _grid_summary(self.latest_local_costmap, robot_pose),
            'global_costmap': _grid_summary(self.latest_global_costmap, robot_pose),
            'scan': _scan_summary(self.latest_scan),
        })

    def done(self) -> bool:
        return time.time() - self.started_at >= float(self.args.record_runtime)


def run_runtime_record(args: argparse.Namespace) -> dict[str, Any]:
    if rclpy is None:
        raise RuntimeError('rclpy unavailable; cannot record runtime evidence')
    rclpy.init()
    node = Phase50RuntimeRecorder(args)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.2)
        node.capture_snapshot()
        summary = {'run_id': RUN_ID, 'snapshots': node.snapshots, 'runtime_record_duration_sec': float(args.record_runtime)}
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    return summary


def _load_runtime(path: Path | None) -> dict[str, Any]:
    if not path or not path.exists() or not path.stat().st_size:
        return {'snapshots': []}
    data = _safe_json_loads(path.read_text(encoding='utf-8', errors='replace'))
    return data if isinstance(data, dict) else {'snapshots': []}


def _nav2_action_available(path: Path | None) -> bool:
    if not path or not path.exists():
        return False
    text = path.read_text(encoding='utf-8', errors='replace').lower()
    return '/navigate_to_pose' in text and ('action servers: 1' in text or 'server count: 1' in text or 'servers: 1' in text)


def _goal_pose_subscriber_available(path: Path | None) -> bool:
    if not path or not path.exists():
        return False
    text = path.read_text(encoding='utf-8', errors='replace').lower()
    return 'subscription count: 1' in text or 'subscriptions: 1' in text or 'subscriber count: 1' in text


def _lifecycle_active(path: Path | None) -> bool:
    if not path or not path.exists():
        return False
    text = path.read_text(encoding='utf-8', errors='replace').lower()
    return all(node in text for node in ['/controller_server', '/planner_server', '/bt_navigator']) and text.count('active [3]') >= 3


def _cleanup_empty(path: Path | None) -> bool:
    if not path or not path.exists():
        return False
    return not any(line.strip() for line in path.read_text(encoding='utf-8', errors='replace').splitlines())


def _first_gate_ready_state(states: list[dict[str, Any]]) -> dict[str, Any] | None:
    for state in states:
        if state.get('dispatch_readiness_gate_passed') is True or (isinstance(state.get('dispatch_readiness_gate'), dict) and state['dispatch_readiness_gate'].get('passed') is True):
            return state
    return None


def _first_waiting_state(states: list[dict[str, Any]]) -> dict[str, Any] | None:
    for state in states:
        if state.get('mode') == 'WAIT_FOR_DISPATCH_ENTRY_READINESS':
            return state
    return None


def _first_topology_state(states: list[dict[str, Any]]) -> dict[str, Any] | None:
    for state in states:
        topo = state.get('last_topology_sampling_diagnostics')
        if isinstance(topo, dict) and topo.get('sampled_direction_count') is not None:
            return state
    return None


def _topology_has_clearance_blocked(state: dict[str, Any] | None) -> bool:
    topo = (state or {}).get('last_topology_sampling_diagnostics') or {}
    reasons = topo.get('reject_reason_counts') or {}
    if 'clearance_radius_blocked' in reasons:
        return True
    return any(sample.get('reject_reason') == 'clearance_radius_blocked' for sample in (topo.get('samples') or []))


def _elapsed(state: dict[str, Any] | None) -> float | None:
    if not state:
        return None
    value = state.get('_recorder_elapsed_sec')
    return float(value) if value is not None else None


def _classify(summary: dict[str, Any]) -> str:
    if summary.get('explorer_state_samples', 0) == 0:
        return 'INSUFFICIENT_RUNTIME_EVIDENCE'
    if summary.get('first_topology_state') and not summary.get('first_topology_after_gate_ready'):
        return 'GATE_BYPASS_OR_EARLY_TOPOLOGY_BUG'
    # Phase49 state payloads expose dispatch_readiness_blocking_reasons while waiting.
    if summary.get('first_gate_ready_state') is None:
        return 'GATE_WAITING_FOR_READINESS_DATA'
    if summary.get('first_topology_state') is None:
        return 'GATE_WAITING_FOR_READINESS_DATA'
    if summary.get('clearance_radius_blocked_after_gate_ready'):
        return 'GATE_VALIDATED_CLEARANCE_GEOMETRY_NEXT_LAYER'
    if summary.get('first_topology_after_gate_ready'):
        return 'GATE_VALIDATED_TOPOLOGY_SAMPLING_AFTER_READY'
    return 'INSUFFICIENT_RUNTIME_EVIDENCE'


def analyze(args: argparse.Namespace) -> dict[str, Any]:
    artifact_dir = Path(args.artifact_dir)
    states = _read_jsonl_payloads(Path(args.explorer_state) if args.explorer_state else None)
    events = _read_jsonl_payloads(Path(args.goal_events) if args.goal_events else None)
    runtime = _load_runtime(Path(args.runtime_evidence) if args.runtime_evidence else None)
    first_waiting = _first_waiting_state(states)
    first_gate_ready = _first_gate_ready_state(states)
    first_topology = _first_topology_state(states)
    wait_elapsed = _elapsed(first_waiting)
    ready_elapsed = _elapsed(first_gate_ready)
    topo_elapsed = _elapsed(first_topology)
    first_topology_after_gate_ready = bool(first_topology and first_gate_ready and ready_elapsed is not None and topo_elapsed is not None and topo_elapsed >= ready_elapsed)
    waited_before_ready = bool(first_waiting and first_gate_ready and wait_elapsed is not None and ready_elapsed is not None and wait_elapsed <= ready_elapsed)
    final_state = states[-1] if states else {}
    summary: dict[str, Any] = {
        'phase': PHASE,
        'run_id': RUN_ID,
        'artifact_dir': str(artifact_dir),
        'active_world': ACTIVE_WORLD,
        'active_metadata': ACTIVE_METADATA,
        'active_truth_frame': 'map',
        'entrance_map': MAP_ENTRANCE,
        'exit_map': MAP_EXIT,
        'bounded_runtime_validation': True,
        'explorer_state_samples': len(states),
        'goal_events_samples': len(events),
        'dispatch_events': sum(1 for e in events if e.get('event') == 'dispatch'),
        'outcome_events': sum(1 for e in events if e.get('event') in {'success', 'failure', 'timeout', 'result'}),
        'final_state': final_state,
        'first_waiting_state': first_waiting,
        'first_gate_ready_state': first_gate_ready,
        'first_topology_state': first_topology,
        'waited_before_ready': waited_before_ready,
        'first_topology_after_gate_ready': first_topology_after_gate_ready,
        'clearance_radius_blocked_after_gate_ready': bool(first_topology_after_gate_ready and _topology_has_clearance_blocked(first_topology)),
        'gate_never_bypassed_before_ready': not bool(first_topology and not first_topology_after_gate_ready),
        'runtime_evidence': runtime,
        'nav2_action_server_available': _nav2_action_available(Path(args.action_info) if args.action_info else None),
        'goal_pose_subscriber_available': _goal_pose_subscriber_available(Path(args.goal_pose_info) if args.goal_pose_info else None),
        'nav2_lifecycle_active': _lifecycle_active(Path(args.lifecycle_states) if args.lifecycle_states else None),
        'cleanup_empty': _cleanup_empty(Path(args.cleanup_processes_after) if args.cleanup_processes_after else None),
        'complete_autonomous_success_claimed': False,
        'phase49_conclusion_preserved': 'DISPATCH_ENTRY_READINESS_GATE_IMPLEMENTED',
        'phase47_manual_baseline_preserved': 'MANUAL_NAV2_GOAL_BASELINE_OK',
    }
    summary['classification'] = _classify(summary)
    assert summary['classification'] in PHASE50_ALLOWED_CLASSIFICATIONS
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--record-runtime', type=float, default=0.0, help='Record full runtime map/scan/odom/TF/costmap evidence for N seconds and exit.')
    parser.add_argument('--snapshot-period-sec', type=float, default=5.0)
    parser.add_argument('--analyze', action='store_true')
    parser.add_argument('--artifact-dir', default='log/phase50_dispatch_entry_readiness_gate_bounded_runtime')
    parser.add_argument('--output', required=True)
    parser.add_argument('--explorer-state')
    parser.add_argument('--goal-events')
    parser.add_argument('--runtime-evidence')
    parser.add_argument('--action-info')
    parser.add_argument('--goal-pose-info')
    parser.add_argument('--lifecycle-states')
    parser.add_argument('--cleanup-processes-after')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.record_runtime > 0:
        run_runtime_record(args)
        return 0
    if args.analyze:
        summary = analyze(args)
        print(json.dumps({'output': args.output, 'classification': summary.get('classification')}, sort_keys=True))
        return 0
    raise SystemExit('Use --record-runtime or --analyze')


if __name__ == '__main__':
    sys.exit(main())
