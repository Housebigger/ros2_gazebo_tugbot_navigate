#!/usr/bin/env python3
"""Phase52 startup map-boundary warmup recorder/analyzer.

Diagnostics-only validation that startup warmup can make the maze_explorer
 dispatch-entry /map gate naturally become sufficient without threshold,
clearance, Nav2, MPPI, or controller tuning. It never claims autonomous
exploration success.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

PHASE = 'Phase52 Startup Map Boundary Warmup Bounded Runtime Validation'
RUN_ID = 'phase52_startup_map_boundary_warmup'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
ACTIVE_METADATA = 'maze_20260528_scaled_instance.yaml'
MAP_ENTRANCE = {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
MAP_EXIT = {'x_m': 21.072562, 'y_m': 18.083566, 'radius_m': 1.2}
TF_LOOKUP_PAIRS = ('map->base_link', 'odom->base_link', 'map->odom')
MIN_MAP_KNOWN_RATIO = 0.70
MIN_MAP_FREE_RATIO = 0.50
ALLOWED_CLASSIFICATIONS = {
    'MAP_SUFFICIENCY_NATURALLY_READY_AFTER_WARMUP',
    'MAP_BOUNDARY_STILL_BLOCKING_AFTER_WARMUP',
    'WARMUP_INCONCLUSIVE_RUNTIME_DATA_GAP',
    'GUARDRAIL_VIOLATION_DISPATCH_OCCURRED',
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
        payload = dict(payload)
        payload['_recorder_elapsed_sec'] = row.get('elapsed_sec', payload.get('elapsed_sec'))
        payload['_recorder_wall_time'] = row.get('wall_time')
        payload['_recorder_seq'] = row.get('seq')
        rows.append(payload)
    return rows


def _read_goal_events(path: Path | None) -> list[dict[str, Any]]:
    if not path or not path.exists() or not path.stat().st_size:
        return []
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        row = _safe_json_loads(line)
        if isinstance(row, dict):
            payload = row.get('state') if isinstance(row.get('state'), dict) else row
            rows.append(payload if isinstance(payload, dict) else row)
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


def _ratios(counts: dict[str, int]) -> dict[str, Any]:
    total = counts.get('sample_count', 0)
    known = counts.get('free_count', 0) + counts.get('occupied_count', 0)
    out = dict(counts)
    out.update({
        'known_count': known,
        'unknown_count': counts.get('unknown_count', 0),
        'known_ratio': known / total if total else 0.0,
        'free_ratio': counts.get('free_count', 0) / total if total else 0.0,
        'occupied_ratio': counts.get('occupied_count', 0) / total if total else 0.0,
        'unknown_ratio': counts.get('unknown_count', 0) / total if total else 0.0,
    })
    return out


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
            value = int(data[yy * width + xx])
            if value < 0:
                counts['unknown_count'] += 1
            elif value >= 65:
                counts['occupied_count'] += 1
            else:
                counts['free_count'] += 1
    result = _ratios(counts)
    result.update({
        'total': counts['sample_count'],
        'center_cell': [cx, cy],
        'robot_cell': [cx, cy],
        'radius_m': radius_m,
        'radius_cells': radius_cells,
        'sample_window': {'min_x': min_x, 'max_x': max_x, 'min_y': min_y, 'max_y': max_y},
        'bbox_world': {
            'min_x': origin_x + min_x * resolution,
            'max_x': origin_x + (max_x + 1) * resolution,
            'min_y': origin_y + min_y * resolution,
            'max_y': origin_y + (max_y + 1) * resolution,
        },
        'robot_in_bounds': 0 <= cx < width and 0 <= cy < height,
        'out_of_bounds_as_unknown': bool(include_out_of_bounds),
    })
    return result


def _grid_summary(msg: Any | None, robot_pose: list[float] | None = None, radius_m: float = 1.0) -> dict[str, Any]:
    empty = {
        'available': False,
        'frame_id': None,
        'width': 0,
        'height': 0,
        'resolution': 0.0,
        'origin': None,
        'map_stamp': None,
        'inclusive_near_robot': None,
        'in_bounds_near_robot': None,
        'map_boundary': None,
    }
    if msg is None:
        return empty
    width = int(msg.info.width)
    height = int(msg.info.height)
    resolution = float(msg.info.resolution)
    origin_x = float(msg.info.origin.position.x)
    origin_y = float(msg.info.origin.position.y)
    out = dict(empty)
    out.update({
        'available': True,
        'frame_id': str(msg.header.frame_id),
        'width': width,
        'height': height,
        'resolution': resolution,
        'origin': {'x': origin_x, 'y': origin_y},
        'map_stamp': float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9,
        'map_boundary': {
            'min_x': origin_x,
            'max_x': origin_x + width * resolution,
            'min_y': origin_y,
            'max_y': origin_y + height * resolution,
            'width_m': width * resolution,
            'height_m': height * resolution,
        },
    })
    if robot_pose:
        out['inclusive_near_robot'] = _grid_near_robot_window(msg, robot_pose, radius_m, include_out_of_bounds=True)
        out['in_bounds_near_robot'] = _grid_near_robot_window(msg, robot_pose, radius_m, include_out_of_bounds=False)
    return out


def _scan_summary(msg: Any | None) -> dict[str, Any]:
    if msg is None:
        return {'available': False, 'finite_count': 0, 'nearest_obstacle_m': None}
    finite = [float(v) for v in msg.ranges if math.isfinite(float(v))]
    return {'available': True, 'finite_count': len(finite), 'nearest_obstacle_m': min(finite) if finite else None}


class Phase52RuntimeRecorder(Node):  # type: ignore[misc]
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('phase52_startup_map_boundary_warmup_runtime_recorder')
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
    node = Phase52RuntimeRecorder(args)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.2)
        node.capture_snapshot()
        summary = {'run_id': RUN_ID, 'phase': PHASE, 'snapshots': node.snapshots, 'runtime_record_duration_sec': float(args.record_runtime)}
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


def _file_contains(path: Path | None, *needles: str) -> bool:
    if not path or not path.exists():
        return False
    text = path.read_text(encoding='utf-8', errors='replace').lower()
    return all(needle.lower() in text for needle in needles)


def _nav2_action_available(path: Path | None) -> bool:
    return _file_contains(path, '/navigate_to_pose') and (
        _file_contains(path, 'action servers: 1') or _file_contains(path, 'server count: 1') or _file_contains(path, 'servers: 1')
    )


def _goal_pose_subscriber_available(path: Path | None) -> bool:
    return _file_contains(path, 'subscription count: 1') or _file_contains(path, 'subscriptions: 1') or _file_contains(path, 'subscriber count: 1')


def _lifecycle_active(path: Path | None) -> bool:
    if not path or not path.exists():
        return False
    text = path.read_text(encoding='utf-8', errors='replace').lower()
    return all(node in text for node in ['/controller_server', '/planner_server', '/bt_navigator']) and text.count('active [3]') >= 3


def _cleanup_empty(path: Path | None) -> bool:
    if not path or not path.exists():
        return False
    return not any(line.strip() for line in path.read_text(encoding='utf-8', errors='replace').splitlines())


def _elapsed(state: dict[str, Any] | None) -> float | None:
    if not state:
        return None
    value = state.get('_recorder_elapsed_sec', state.get('elapsed_sec'))
    return float(value) if value is not None else None


def _gate(state: dict[str, Any]) -> dict[str, Any]:
    gate = state.get('dispatch_readiness_gate')
    return gate if isinstance(gate, dict) else {}


def _gate_map(state: dict[str, Any]) -> dict[str, Any]:
    gate = _gate(state)
    value = gate.get('map') or gate.get('map_sufficient') or {}
    return value if isinstance(value, dict) else {}


def _first_gate_ready_state(states: list[dict[str, Any]]) -> dict[str, Any] | None:
    for state in states:
        gate = _gate(state)
        if state.get('dispatch_readiness_gate_passed') is True or gate.get('passed') is True:
            return state
    return None


def _first_waiting_map_blocked_state(states: list[dict[str, Any]]) -> dict[str, Any] | None:
    for state in states:
        gate = _gate(state)
        reasons = gate.get('blocking_reasons') or state.get('dispatch_readiness_blocking_reasons') or []
        if state.get('mode') == 'WAIT_FOR_DISPATCH_ENTRY_READINESS' and 'map_sufficient' in reasons:
            return state
    return None


def _first_topology_state(states: list[dict[str, Any]]) -> dict[str, Any] | None:
    for state in states:
        topo = state.get('last_topology_sampling_diagnostics')
        if isinstance(topo, dict) and topo.get('sampled_direction_count') is not None:
            return state
    return None


def _dispatch_events(events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out = []
    for event in events:
        kind = str(event.get('event') or event.get('event_type') or event.get('type') or '').lower()
        if kind == 'dispatch' or 'dispatch' in kind:
            out.append(event)
    return out


def _goal_count_violation(states: list[dict[str, Any]]) -> bool:
    return any(int(state.get('goal_count') or 0) > 0 for state in states)


def _map_sufficient_from_snapshot(snapshot: dict[str, Any]) -> bool:
    near = ((snapshot.get('map') or {}).get('inclusive_near_robot') or {})
    return float(near.get('known_ratio', 0.0)) >= MIN_MAP_KNOWN_RATIO and float(near.get('free_ratio', 0.0)) >= MIN_MAP_FREE_RATIO


def _map_sufficient_from_ratios(ratios: dict[str, Any]) -> bool:
    return float(ratios.get('known_ratio', 0.0)) >= MIN_MAP_KNOWN_RATIO and float(ratios.get('free_ratio', 0.0)) >= MIN_MAP_FREE_RATIO


def _map_timeline(snapshots: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for snap in snapshots:
        map_summary = snap.get('map') or {}
        near = map_summary.get('inclusive_near_robot') or {}
        rows.append({
            'elapsed_sec': snap.get('elapsed_sec'),
            'map_stamp': map_summary.get('map_stamp'),
            'known_ratio': near.get('known_ratio'),
            'free_ratio': near.get('free_ratio'),
            'unknown_ratio': near.get('unknown_ratio'),
            'sample_count': near.get('sample_count'),
            'in_bounds_count': near.get('in_bounds_count'),
            'out_of_bounds_count': near.get('out_of_bounds_count'),
            'robot_cell': near.get('robot_cell') or near.get('center_cell'),
            'sample_window': near.get('sample_window'),
            'bbox_world': near.get('bbox_world'),
            'map_origin': map_summary.get('origin'),
            'map_resolution': map_summary.get('resolution'),
            'map_width': map_summary.get('width'),
            'map_height': map_summary.get('height'),
            'map_boundary': map_summary.get('map_boundary'),
            'sufficient_by_threshold': _map_sufficient_from_snapshot(snap),
        })
    return rows


def _classification(summary: dict[str, Any]) -> str:
    if summary.get('dispatch_events') or summary.get('goal_count_violation') or summary.get('topology_sampling_occurred'):
        return 'GUARDRAIL_VIOLATION_DISPATCH_OCCURRED'
    if not summary.get('explorer_state_samples') or not summary.get('runtime_snapshot_count'):
        return 'WARMUP_INCONCLUSIVE_RUNTIME_DATA_GAP'
    if summary.get('first_gate_ready_state') is not None:
        return 'MAP_SUFFICIENCY_NATURALLY_READY_AFTER_WARMUP'
    final_map = summary.get('final_gate_map') or summary.get('final_runtime_map') or {}
    final_map_blocked = bool(final_map) and not _map_sufficient_from_ratios(final_map)
    saw_map_blocker = summary.get('first_waiting_map_blocked_elapsed_sec') is not None
    runtime_map_samples = summary.get('runtime_snapshot_count', 0) >= 2
    if final_map_blocked and (saw_map_blocker or runtime_map_samples):
        return 'MAP_BOUNDARY_STILL_BLOCKING_AFTER_WARMUP'
    if not (summary.get('nav2_action_server_available') and summary.get('goal_pose_subscriber_available') and summary.get('nav2_lifecycle_active')):
        return 'WARMUP_INCONCLUSIVE_RUNTIME_DATA_GAP'
    return 'WARMUP_INCONCLUSIVE_RUNTIME_DATA_GAP'


def analyze(args: argparse.Namespace) -> dict[str, Any]:
    artifact_dir = Path(args.artifact_dir)
    states = _read_jsonl_payloads(Path(args.explorer_state) if args.explorer_state else None)
    events = _read_goal_events(Path(args.goal_events) if args.goal_events else None)
    runtime = _load_runtime(Path(args.runtime_evidence) if args.runtime_evidence else None)
    snapshots = runtime.get('snapshots') if isinstance(runtime.get('snapshots'), list) else []
    first_waiting = _first_waiting_map_blocked_state(states)
    first_gate_ready = _first_gate_ready_state(states)
    final_state = states[-1] if states else {}
    final_gate_map = _gate_map(final_state)
    final_runtime_map = (((snapshots[-1] if snapshots else {}).get('map') or {}).get('inclusive_near_robot') or {})
    dispatches = _dispatch_events(events)
    topology = _first_topology_state(states)
    timeline = _map_timeline(snapshots)
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
        'warmup_only': True,
        'max_goals': 0,
        'startup_warmup_no_dispatch': True,
        'explorer_state_samples': len(states),
        'goal_events_samples': len(events),
        'dispatch_events': len(dispatches),
        'goal_count_violation': _goal_count_violation(states),
        'topology_sampling_occurred': topology is not None,
        'first_waiting_map_blocked_elapsed_sec': _elapsed(first_waiting),
        'first_gate_ready_elapsed_sec': _elapsed(first_gate_ready),
        'first_gate_ready_state': first_gate_ready,
        'final_state': final_state,
        'final_gate_map': final_gate_map,
        'runtime_snapshot_count': len(snapshots),
        'first_runtime_map': ((snapshots[0] if snapshots else {}).get('map') or {}).get('inclusive_near_robot'),
        'final_runtime_map': final_runtime_map,
        'map_boundary_timeline': timeline,
        'map_sufficiency_thresholds': {'min_known_ratio': MIN_MAP_KNOWN_RATIO, 'min_free_ratio': MIN_MAP_FREE_RATIO},
        'nav2_action_server_available': _nav2_action_available(Path(args.action_info) if args.action_info else None),
        'goal_pose_subscriber_available': _goal_pose_subscriber_available(Path(args.goal_pose_info) if args.goal_pose_info else None),
        'nav2_lifecycle_active': _lifecycle_active(Path(args.lifecycle_states) if args.lifecycle_states else None),
        'cleanup_empty': _cleanup_empty(Path(args.cleanup_processes_after) if args.cleanup_processes_after else None),
        'complete_autonomous_success_claimed': False,
        'guardrails': {
            'no_dispatch_goal': len(dispatches) == 0 and not _goal_count_violation(states),
            'no_topology_sampling': topology is None,
            'no_autonomous_success_claim': True,
            'no_threshold_change': True,
            'no_clearance_strategy_change': True,
            'no_nav2_mppi_controller_tuning': True,
        },
    }
    summary['classification'] = _classification(summary)
    if summary['classification'] not in ALLOWED_CLASSIFICATIONS:
        raise RuntimeError(f'unexpected classification: {summary["classification"]}')
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=PHASE)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--record-runtime', type=float)
    mode.add_argument('--analyze', action='store_true')
    parser.add_argument('--snapshot-period-sec', type=float, default=10.0)
    parser.add_argument('--output', required=True)
    parser.add_argument('--artifact-dir', default='log/phase52_startup_map_boundary_warmup')
    parser.add_argument('--explorer-state')
    parser.add_argument('--goal-events')
    parser.add_argument('--runtime-evidence')
    parser.add_argument('--action-info')
    parser.add_argument('--goal-pose-info')
    parser.add_argument('--lifecycle-states')
    parser.add_argument('--cleanup-processes-after')
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.record_runtime is not None:
        summary = run_runtime_record(args)
    else:
        summary = analyze(args)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
