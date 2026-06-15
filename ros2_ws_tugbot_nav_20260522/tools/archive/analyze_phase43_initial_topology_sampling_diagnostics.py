#!/usr/bin/env python3
"""Phase43 initial topology sampling diagnostics analyzer/recorder.

Offline analyzer plus optional short runtime recorder for complete map/scan/odom/TF/
costmap evidence. It intentionally does not dispatch goals, change Nav2 params, or
change maze_explorer strategy.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

PHASE = 'Phase43 Initial Topology Sampling Diagnostics'
RUN_ID = 'phase43_initial_topology_sampling_diagnostics'
PHASE43_ALLOWED_CLASSIFICATIONS = {
    'INSUFFICIENT_INITIAL_MAP_OR_SCAN',
    'FRAME_MISMATCH',
    'ROBOT_NOT_AT_ENTRANCE',
    'COSTMAP_BLOCKED_OR_UNKNOWN',
    'LASER_SCAN_EMPTY_OR_INVALID',
    'TOPOLOGY_REJECTION_CAUSE_IDENTIFIED',
    'INCONCLUSIVE_NEEDS_TARGETED_RUNTIME_CAPTURE',
}
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
ACTIVE_METADATA = 'maze_20260528_scaled_instance.yaml'
MAP_ENTRANCE = {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
MAP_EXIT = {'x_m': 21.072562, 'y_m': 18.083566, 'radius_m': 1.2}
TF_LOOKUP_PAIRS = ('map->base_link', 'odom->base_link', 'map->odom')

try:  # Runtime imports are optional so pytest/offline analysis works without ROS.
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

try:
    from PIL import Image, ImageDraw
except Exception:  # pragma: no cover
    Image = None
    ImageDraw = None


def _safe_json_loads(text: str) -> Any | None:
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return None


def _read_jsonl_payloads(path: Path) -> list[dict[str, Any]]:
    if not path.exists() or not path.stat().st_size:
        return []
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        row = _safe_json_loads(line)
        if isinstance(row, dict):
            payload = row.get('state') if isinstance(row.get('state'), dict) else row
            payload['_recorder_elapsed_sec'] = row.get('elapsed_sec')
            payload['_recorder_wall_time'] = row.get('wall_time')
            rows.append(payload)
    return rows


def _grid_summary(msg: Any | None, robot_pose: list[float] | None = None, radius_m: float = 1.5) -> dict[str, Any]:
    out = {
        'available': msg is not None,
        'frame_id': None,
        'width': 0,
        'height': 0,
        'resolution': None,
        'origin': None,
        'full': {'total': 0, 'unknown': 0, 'free': 0, 'occupied': 0, 'known_ratio': 0.0, 'free_ratio': 0.0, 'occupied_ratio': 0.0, 'unknown_ratio': 0.0},
        'near_robot': None,
    }
    if msg is None:
        return out
    width = int(msg.info.width)
    height = int(msg.info.height)
    resolution = float(msg.info.resolution)
    origin_x = float(msg.info.origin.position.x)
    origin_y = float(msg.info.origin.position.y)
    data = list(msg.data)
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
    out.update({'frame_id': str(msg.header.frame_id), 'width': width, 'height': height, 'resolution': resolution, 'origin': {'x': origin_x, 'y': origin_y}})
    out['full'] = ratios(data)
    if robot_pose and width > 0 and height > 0 and resolution > 0:
        cx = int(math.floor((float(robot_pose[0]) - origin_x) / resolution))
        cy = int(math.floor((float(robot_pose[1]) - origin_y) / resolution))
        radius_cells = int(math.ceil(radius_m / resolution))
        vals = []
        in_bounds_center = 0 <= cx < width and 0 <= cy < height
        for yy in range(cy - radius_cells, cy + radius_cells + 1):
            for xx in range(cx - radius_cells, cx + radius_cells + 1):
                if 0 <= xx < width and 0 <= yy < height:
                    wx = origin_x + (xx + 0.5) * resolution
                    wy = origin_y + (yy + 0.5) * resolution
                    if math.hypot(wx - float(robot_pose[0]), wy - float(robot_pose[1])) <= radius_m:
                        vals.append(int(data[yy * width + xx]))
        near = ratios(vals)
        near['center_cell'] = [cx, cy]
        near['center_in_bounds'] = bool(in_bounds_center)
        near['radius_m'] = radius_m
        out['near_robot'] = near
    return out


def _scan_summary(msg: Any | None) -> dict[str, Any]:
    out = {'available': msg is not None, 'frame_id': None, 'sample_count': 0, 'finite_count': 0, 'nearest_obstacle_m': None, 'range_min': None, 'range_max': None}
    if msg is None:
        return out
    finite = [float(v) for v in msg.ranges if math.isfinite(float(v))]
    nearest = min(finite) if finite else None
    out.update({
        'frame_id': str(msg.header.frame_id),
        'sample_count': len(msg.ranges),
        'finite_count': len(finite),
        'nearest_obstacle_m': nearest,
        'range_min': float(msg.range_min),
        'range_max': float(msg.range_max),
    })
    return out


def _quat_yaw(q: Any) -> float:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _odom_summary(msg: Any | None) -> dict[str, Any] | None:
    if msg is None:
        return None
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return {'frame_id': str(msg.header.frame_id), 'child_frame_id': str(msg.child_frame_id), 'pose': {'x': float(p.x), 'y': float(p.y), 'z': float(p.z), 'yaw_rad': _quat_yaw(q)}}


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


class Phase43RuntimeRecorder(Node):  # type: ignore[misc]
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('phase43_initial_topology_runtime_recorder')
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
        row = {
            'elapsed_sec': elapsed,
            'tf_lookups': tf_lookups,
            'odom': odom,
            'robot_pose_in_map': robot_pose,
            'robot_to_active_entrance_m': math.hypot(robot_pose[0] - MAP_ENTRANCE['x_m'], robot_pose[1] - MAP_ENTRANCE['y_m']) if robot_pose else None,
            'map': _grid_summary(self.latest_map, robot_pose),
            'local_costmap': _grid_summary(self.latest_local_costmap, robot_pose),
            'global_costmap': _grid_summary(self.latest_global_costmap, robot_pose),
            'scan': _scan_summary(self.latest_scan),
        }
        self.snapshots.append(row)

    def done(self) -> bool:
        return time.time() - self.started_at >= float(self.args.record_runtime)


def run_runtime_record(args: argparse.Namespace) -> dict[str, Any]:
    if rclpy is None:
        raise RuntimeError('rclpy unavailable; cannot record runtime evidence')
    rclpy.init()
    node = Phase43RuntimeRecorder(args)
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


def _first_topology_state(states: list[dict[str, Any]]) -> dict[str, Any] | None:
    for state in states:
        topo = state.get('last_topology_sampling_diagnostics')
        if isinstance(topo, dict) and topo.get('sampled_direction_count') is not None:
            return state
    return None


def _sufficiency_before_after(runtime: dict[str, Any], first_elapsed: float | None) -> dict[str, Any]:
    snapshots = list(runtime.get('snapshots') or [])
    if first_elapsed is None:
        return {'first_topology_elapsed_sec': None, 'before': None, 'after': None}
    before = None
    after = None
    for snap in snapshots:
        elapsed = float(snap.get('elapsed_sec', 0.0))
        if elapsed <= first_elapsed:
            before = snap
        if elapsed >= first_elapsed and after is None:
            after = snap
    return {'first_topology_elapsed_sec': first_elapsed, 'before': before, 'after': after}


def _nav2_action_available(path: Path | None) -> bool:
    if not path or not path.exists():
        return False
    text = path.read_text(encoding='utf-8', errors='replace').lower()
    return '/navigate_to_pose' in text and ('action clients' in text or 'action servers' in text or 'server count' in text or 'servers:' in text)


def _cleanup_empty(path: Path | None) -> bool:
    if not path or not path.exists():
        return False
    interesting = [line for line in path.read_text(encoding='utf-8', errors='replace').splitlines() if line.strip()]
    return len(interesting) == 0


def _classify(summary: dict[str, Any]) -> str:
    first_state = summary.get('first_topology_state') or {}
    topo = first_state.get('last_topology_sampling_diagnostics') or {}
    runtime = summary.get('runtime_evidence') or {}
    latest = (runtime.get('snapshots') or [None])[-1] if runtime.get('snapshots') else None
    scan = (latest or {}).get('scan') or {}
    robot_dist = summary.get('robot_to_active_entrance_m_at_first_topology')
    reason_counts = topo.get('reject_reason_counts') or {}
    cost_reasons = {'unknown', 'lethal_or_obstacle', 'inflated', 'out_of_bounds'}
    cost_bad = any((sample.get('costmap_lethal_or_unknown_result') in cost_reasons) for sample in (topo.get('samples') or []))
    if robot_dist is not None and float(robot_dist) > 1.5:
        return 'ROBOT_NOT_AT_ENTRANCE'
    if scan.get('available') and int(scan.get('finite_count') or 0) == 0:
        return 'LASER_SCAN_EMPTY_OR_INVALID'
    if any(str(k).startswith('unknown_cell') for k in reason_counts) or reason_counts.get('map_unavailable'):
        return 'INSUFFICIENT_INITIAL_MAP_OR_SCAN'
    if cost_bad:
        return 'COSTMAP_BLOCKED_OR_UNKNOWN'
    if reason_counts:
        return 'TOPOLOGY_REJECTION_CAUSE_IDENTIFIED'
    if not summary.get('active_map_frame_truth_used'):
        return 'FRAME_MISMATCH'
    return 'INCONCLUSIVE_NEEDS_TARGETED_RUNTIME_CAPTURE'


def _draw_overlays(summary: dict[str, Any], artifact_dir: Path) -> None:
    if Image is None or ImageDraw is None:
        return
    artifact_dir.mkdir(parents=True, exist_ok=True)
    # Overlay 1: robot vs entrance/exit.
    img = Image.new('RGB', (900, 650), 'white')
    draw = ImageDraw.Draw(img)
    points: list[tuple[float, float, str, str]] = [(0.0, 0.0, 'entrance_map', 'green'), (MAP_EXIT['x_m'], MAP_EXIT['y_m'], 'exit_map', 'blue')]
    robot_pose = summary.get('robot_pose_in_map_at_first_topology') or summary.get('latest_robot_pose_in_map')
    if robot_pose:
        points.append((float(robot_pose[0]), float(robot_pose[1]), 'robot', 'red'))
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    span = max(max(xs) - min(xs), max(ys) - min(ys), 2.0)
    cx, cy = (max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0
    scale = min(760 / span, 520 / span)
    def px(x: float, y: float) -> tuple[int, int]:
        return int(450 + (x - cx) * scale), int(335 - (y - cy) * scale)
    draw.rectangle((70, 70, 830, 590), outline='gray')
    draw.line([px(0.0, 0.0), px(MAP_EXIT['x_m'], MAP_EXIT['y_m'])], fill='blue', width=2)
    for x, y, label, color in points:
        sx, sy = px(x, y)
        draw.ellipse((sx - 8, sy - 8, sx + 8, sy + 8), fill=color, outline='black')
        draw.text((sx + 10, sy - 8), f'{label}: ({x:.2f},{y:.2f})', fill='black')
    draw.text((20, 20), PHASE, fill='black')
    draw.text((20, 42), f"classification: {summary.get('classification')}", fill='black')
    img.save(artifact_dir / 'phase43_robot_vs_entrance_overlay.png')
    # Overlay 2: topology reason bars.
    img2 = Image.new('RGB', (900, 500), 'white')
    draw2 = ImageDraw.Draw(img2)
    reasons = ((summary.get('first_topology_sampling_snapshot') or {}).get('reject_reason_counts') or {})
    draw2.text((20, 20), 'Phase43 topology rejection reason counts', fill='black')
    max_count = max([1] + [int(v) for v in reasons.values()])
    y = 60
    for reason, count in sorted(reasons.items(), key=lambda kv: str(kv[0])):
        bar = int(650 * int(count) / max_count)
        draw2.rectangle((220, y, 220 + bar, y + 18), fill='orange')
        draw2.text((20, y), str(reason)[:32], fill='black')
        draw2.text((230 + bar, y), str(count), fill='black')
        y += 30
    img2.save(artifact_dir / 'phase43_initial_topology_rejection_overlay.png')


def analyze(args: argparse.Namespace) -> dict[str, Any]:
    artifact_dir = Path(args.artifact_dir)
    states = _read_jsonl_payloads(Path(args.explorer_state)) if args.explorer_state else []
    events = _read_jsonl_payloads(Path(args.goal_events)) if args.goal_events else []
    runtime = _load_runtime(Path(args.runtime_evidence) if args.runtime_evidence else None)
    first_state = _first_topology_state(states)
    final_state = states[-1] if states else {}
    first_elapsed = first_state.get('_recorder_elapsed_sec') if first_state else None
    topo = (first_state or {}).get('last_topology_sampling_diagnostics') or {}
    first_pose = topo.get('robot_pose_map') or (first_state or {}).get('robot_pose')
    latest_pose = None
    if runtime.get('snapshots'):
        latest_pose = (runtime['snapshots'][-1] or {}).get('robot_pose_in_map')
    robot_dist = math.hypot(float(first_pose[0]), float(first_pose[1])) if isinstance(first_pose, list) and len(first_pose) >= 2 else None
    summary: dict[str, Any] = {
        'phase': PHASE,
        'run_id': RUN_ID,
        'artifact_dir': str(artifact_dir),
        'active_world': ACTIVE_WORLD,
        'active_metadata': ACTIVE_METADATA,
        'active_truth_frame': 'map',
        'active_map_frame_truth_used': True,
        'entrance_map': MAP_ENTRANCE,
        'exit_map': MAP_EXIT,
        'explorer_state_samples': len(states),
        'goal_events_samples': len(events),
        'dispatch_events': sum(1 for e in events if e.get('event') == 'dispatch'),
        'outcome_events': sum(1 for e in events if e.get('event') in {'success', 'failure', 'timeout', 'result'}),
        'final_state': final_state,
        'first_topology_state': first_state,
        'first_topology_sampling_snapshot': topo,
        'first_topology_sampling_evidence_captured': bool(topo),
        'rejection_reason_present_or_explained': bool((topo.get('reject_reason_counts') or {}) or (not topo)),
        'runtime_evidence': runtime,
        'map_sufficiency_before_after_first_topology': _sufficiency_before_after(runtime, first_elapsed),
        'robot_pose_in_map_at_first_topology': first_pose,
        'latest_robot_pose_in_map': latest_pose,
        'robot_to_active_entrance_m_at_first_topology': robot_dist,
        'nav2_action_server_available': _nav2_action_available(Path(args.action_info) if args.action_info else None),
        'cleanup_empty': _cleanup_empty(Path(args.cleanup_processes_after) if args.cleanup_processes_after else None),
        'complete_autonomous_success_claimed': False,
        'phase42_conclusion_preserved': 'BOUNDED_SMOKE_NO_DISPATCH_TOPOLOGY_SAMPLING',
        'phase41_conclusion_preserved': 'MAP_FRAME_TRUTH_ALIGNED_READY_FOR_PHASE37_RERUN',
    }
    summary['entered_failed_exhausted_before_map_sufficient'] = bool(final_state.get('mode') == 'FAILED_EXHAUSTED' and not topo.get('raw_open_direction_count'))
    summary['classification'] = _classify(summary)
    assert summary['classification'] in PHASE43_ALLOWED_CLASSIFICATIONS
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    snapshot_path = artifact_dir / 'first_topology_sampling_snapshot.json'
    snapshot_path.write_text(json.dumps(topo, indent=2, sort_keys=True), encoding='utf-8')
    _draw_overlays(summary, artifact_dir)
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--record-runtime', type=float, default=0.0, help='Record full runtime map/scan/odom/TF/costmap evidence for N seconds and exit.')
    parser.add_argument('--snapshot-period-sec', type=float, default=5.0)
    parser.add_argument('--analyze', action='store_true')
    parser.add_argument('--artifact-dir', default='log/phase43_initial_topology_sampling_diagnostics')
    parser.add_argument('--output', required=True)
    parser.add_argument('--explorer-state')
    parser.add_argument('--goal-events')
    parser.add_argument('--runtime-evidence')
    parser.add_argument('--action-info')
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
