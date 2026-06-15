#!/usr/bin/env python3
"""Phase40 bounded start-pose / entrance-frame alignment recorder.

Runtime mode records pose-only evidence for the active scaled2x world startup:
/odom and tf2 lookups map->base_link, odom->base_link, and map->odom. It does
not subscribe to maze_explorer topics, does not send goals, and does not mutate
Nav2/MPPI/controller or maze_explorer strategy.

Offline mode (--analyze-existing) analyzes a serialized pose evidence JSON for
fast contract tests and report regeneration without ROS runtime.
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml
from PIL import Image, ImageDraw

try:  # Runtime-only imports; offline analysis must work without a sourced ROS shell.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    import tf2_ros
except Exception:  # pragma: no cover
    rclpy = None
    Node = object
    QoSProfile = None
    ReliabilityPolicy = None
    DurabilityPolicy = None
    HistoryPolicy = None
    Odometry = None
    tf2_ros = None

PHASE40_ALLOWED_CONCLUSIONS = {
    'START_POSE_ALIGNED_READY_FOR_TOPOLOGY_RERUN',
    'SPAWN_WIRING_FIXED_PENDING_RUNTIME_CHECK',
    'METADATA_WORLD_FRAME_CONVENTION_CONFLICT',
    'INSUFFICIENT_EVIDENCE',
}
TF_LOOKUP_PAIRS = ('map->base_link', 'odom->base_link', 'map->odom')
DEFAULT_ENTRANCE = {'x_m': -11.011281, 'y_m': -9.025070, 'yaw_rad': 0.0}
DEFAULT_EXIT = {'x_m': 10.061281, 'y_m': 9.058496, 'radius_m': 1.2}
PHASE = 'Phase40 Start Pose / Entrance Frame Reconciliation'


def _stamp_dict(stamp: Any) -> dict[str, int]:
    return {'sec': int(getattr(stamp, 'sec', 0)), 'nanosec': int(getattr(stamp, 'nanosec', 0))}


def _header_dict(header: Any) -> dict[str, Any]:
    return {'stamp': _stamp_dict(header.stamp), 'frame_id': str(header.frame_id)}


def _position_dict(p: Any) -> dict[str, float]:
    return {'x': float(p.x), 'y': float(p.y), 'z': float(p.z)}


def _quaternion_dict(q: Any) -> dict[str, float]:
    return {'x': float(q.x), 'y': float(q.y), 'z': float(q.z), 'w': float(q.w)}


def _quat_yaw(q: dict[str, float]) -> float:
    return math.atan2(
        2.0 * (q.get('w', 1.0) * q.get('z', 0.0) + q.get('x', 0.0) * q.get('y', 0.0)),
        1.0 - 2.0 * (q.get('y', 0.0) ** 2 + q.get('z', 0.0) ** 2),
    )


def _transform_to_dict(transform_stamped: Any) -> dict[str, Any]:
    t = transform_stamped.transform.translation
    r = transform_stamped.transform.rotation
    q = _quaternion_dict(r)
    return {
        'header': _header_dict(transform_stamped.header),
        'child_frame_id': str(transform_stamped.child_frame_id),
        'translation': _position_dict(t),
        'rotation': q,
        'yaw_rad': _quat_yaw(q),
    }


def _lookup_transform_dict(buffer: Any, parent: str, child: str) -> dict[str, Any]:
    if buffer is None or rclpy is None:
        return {'available': False, 'error': 'tf2 unavailable', 'parent': parent, 'child': child}
    try:
        # tf2_ros.Buffer lookup: map->base_link, odom->base_link, map->odom.
        msg = buffer.lookup_transform(parent, child, rclpy.time.Time())
        row = _transform_to_dict(msg)
        row['available'] = True
        return row
    except Exception as exc:
        return {'available': False, 'error': str(exc), 'parent': parent, 'child': child}


def _odom_msg_to_dict(msg: Any | None) -> dict[str, Any] | None:
    if msg is None:
        return None
    pose = msg.pose.pose
    twist = msg.twist.twist
    q = _quaternion_dict(pose.orientation)
    return {
        'header': _header_dict(msg.header),
        'child_frame_id': str(msg.child_frame_id),
        'pose': {
            'position': _position_dict(pose.position),
            'orientation': q,
            'yaw_rad': _quat_yaw(q),
        },
        'twist': {
            'linear': _position_dict(twist.linear),
            'angular': _position_dict(twist.angular),
        },
    }


def _load_metadata(path: Path | None) -> dict[str, Any]:
    if path and path.exists():
        return yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    return {'entrance': dict(DEFAULT_ENTRANCE), 'exit': dict(DEFAULT_EXIT)}


def _floats(text: str) -> list[float]:
    return [float(v) for v in text.split()]


def _extract_sdf_pose(world_path: Path | None) -> dict[str, Any]:
    if not world_path or not world_path.exists():
        return {'available': False, 'error': 'world path missing', 'pose': None}
    tree = ET.parse(world_path)
    world = tree.getroot().find('.//world')
    if world is None:
        return {'available': False, 'error': 'missing world element', 'pose': None}
    for inc in world.findall('include'):
        if (inc.findtext('uri') or '').strip() == 'model://tugbot':
            vals = _floats(inc.findtext('pose') or '')
            return {
                'available': len(vals) >= 6,
                'name': (inc.findtext('name') or '').strip(),
                'uri': 'model://tugbot',
                'pose': vals,
            }
    return {'available': False, 'error': 'missing model://tugbot include', 'pose': None}


def _extract_slam_map_start_pose(params_path: Path | None) -> dict[str, Any]:
    if not params_path or not params_path.exists():
        return {'available': False, 'error': 'slam params missing', 'pose': None}
    data = yaml.safe_load(params_path.read_text(encoding='utf-8')) or {}
    params = ((data.get('slam_toolbox') or {}).get('ros__parameters') or {})
    pose = params.get('map_start_pose')
    if pose is None:
        return {'available': False, 'pose': None}
    return {'available': True, 'pose': [float(v) for v in pose]}


def _pose_distance(a: dict[str, float] | list[float] | None, b: dict[str, float]) -> float | None:
    if a is None:
        return None
    if isinstance(a, list):
        if len(a) < 2:
            return None
        ax, ay = float(a[0]), float(a[1])
    else:
        ax, ay = float(a.get('x_m', a.get('x', 0.0))), float(a.get('y_m', a.get('y', 0.0)))
    return math.hypot(ax - float(b['x_m']), ay - float(b['y_m']))


def _tf_robot_pose(snapshot: dict[str, Any]) -> dict[str, float] | None:
    tf = ((snapshot.get('tf_lookups') or {}).get('map->base_link') or {})
    if tf.get('available'):
        t = tf.get('translation') or {}
        return {'x': float(t.get('x', 0.0)), 'y': float(t.get('y', 0.0)), 'yaw_rad': float(tf.get('yaw_rad', 0.0))}
    tf = ((snapshot.get('tf_lookups') or {}).get('odom->base_link') or {})
    if tf.get('available'):
        t = tf.get('translation') or {}
        return {'x': float(t.get('x', 0.0)), 'y': float(t.get('y', 0.0)), 'yaw_rad': float(tf.get('yaw_rad', 0.0))}
    odom = snapshot.get('odom') or {}
    pose = (odom.get('pose') or {})
    pos = pose.get('position') or {}
    if pos:
        return {'x': float(pos.get('x', 0.0)), 'y': float(pos.get('y', 0.0)), 'yaw_rad': float(pose.get('yaw_rad', 0.0))}
    return None


def _draw_alignment_overlay(summary: dict[str, Any], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    w, h = 900, 700
    img = Image.new('RGB', (w, h), 'white')
    draw = ImageDraw.Draw(img)
    margin = 70
    points: list[tuple[float, float, str, str]] = []
    ent = summary['active_truth']['entrance']
    points.append((float(ent['x_m']), float(ent['y_m']), 'entrance', 'green'))
    sdf = summary.get('sdf_tugbot_spawn_pose') or {}
    if sdf.get('pose'):
        points.append((float(sdf['pose'][0]), float(sdf['pose'][1]), 'sdf spawn', 'blue'))
    robot = ((summary.get('frame_alignment') or {}).get('latest_robot_pose_in_map') or {})
    if robot:
        points.append((float(robot['x']), float(robot['y']), 'runtime robot', 'red'))
    if not points:
        draw.text((20, 20), 'No pose evidence available', fill='black')
        img.save(output_dir / 'start_pose_entrance_alignment_overlay.png')
        return
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    span = max(max(xs) - min(xs), max(ys) - min(ys), 2.0)
    cx = (max(xs) + min(xs)) / 2.0
    cy = (max(ys) + min(ys)) / 2.0
    scale = min((w - 2 * margin) / span, (h - 2 * margin) / span)

    def px(x: float, y: float) -> tuple[int, int]:
        return (int(w / 2 + (x - cx) * scale), int(h / 2 - (y - cy) * scale))

    draw.rectangle((margin, margin, w - margin, h - margin), outline='gray')
    if len(points) >= 2:
        p0 = px(points[0][0], points[0][1])
        for p in points[1:]:
            draw.line([p0, px(p[0], p[1])], fill='orange', width=2)
    for x, y, label, color in points:
        sx, sy = px(x, y)
        r = 8
        draw.ellipse((sx - r, sy - r, sx + r, sy + r), fill=color, outline='black')
        draw.text((sx + 10, sy - 8), f'{label}: ({x:.3f}, {y:.3f})', fill='black')
    dist = (summary.get('frame_alignment') or {}).get('robot_to_active_entrance_distance_m')
    draw.text((20, 20), PHASE, fill='black')
    draw.text((20, 42), f'conclusion: {summary.get("conclusion")}', fill='black')
    draw.text((20, 64), f'robot_to_entrance_distance_m: {dist}', fill='black')
    img.save(output_dir / 'start_pose_entrance_alignment_overlay.png')


def _analyze(payload: dict[str, Any], output_dir: Path) -> dict[str, Any]:
    metadata = payload.get('metadata') or {}
    active_truth = payload.get('active_truth') or {}
    entrance = active_truth.get('entrance') or dict(DEFAULT_ENTRANCE)
    snapshots = payload.get('snapshots') or []
    latest = snapshots[-1] if snapshots else {}
    latest_robot_pose = _tf_robot_pose(latest)
    robot_distance = None
    if latest_robot_pose:
        robot_distance = math.hypot(float(latest_robot_pose['x']) - float(entrance['x_m']), float(latest_robot_pose['y']) - float(entrance['y_m']))
    sdf_pose = payload.get('sdf_tugbot_spawn_pose') or {}
    slam_pose = payload.get('slam_map_start_pose') or {}
    sdf_distance = _pose_distance(sdf_pose.get('pose'), entrance)
    slam_distance = _pose_distance(slam_pose.get('pose'), entrance)

    timeline = []
    for snap in snapshots:
        pose = _tf_robot_pose(snap)
        dist = None
        if pose:
            dist = math.hypot(float(pose['x']) - float(entrance['x_m']), float(pose['y']) - float(entrance['y_m']))
        timeline.append({
            'elapsed_sec': snap.get('elapsed_sec'),
            'robot_pose_in_map': pose,
            'robot_to_active_entrance_distance_m': dist,
            'tf_available': {k: bool(((snap.get('tf_lookups') or {}).get(k) or {}).get('available')) for k in TF_LOOKUP_PAIRS},
            'odom_available': snap.get('odom') is not None,
        })

    reasons = []
    runtime_alignment_ok = robot_distance is not None and robot_distance < 0.75
    sdf_aligned = sdf_distance is not None and sdf_distance < 0.01
    slam_aligned = slam_distance is not None and slam_distance < 0.01
    if sdf_aligned:
        reasons.append('active SDF Tugbot spawn equals metadata entrance')
    else:
        reasons.append(f'active SDF Tugbot spawn distance to entrance is {sdf_distance}')
    if slam_aligned:
        reasons.append('slam_toolbox map_start_pose equals metadata entrance')
    else:
        reasons.append('slam_toolbox map_start_pose missing or not aligned to entrance')
    if robot_distance is None:
        conclusion = 'SPAWN_WIRING_FIXED_PENDING_RUNTIME_CHECK' if sdf_aligned and slam_aligned else 'INSUFFICIENT_EVIDENCE'
        reasons.append('runtime TF/odom pose evidence unavailable')
    elif runtime_alignment_ok and sdf_aligned and slam_aligned:
        conclusion = 'START_POSE_ALIGNED_READY_FOR_TOPOLOGY_RERUN'
        reasons.append(f'robot pose is {robot_distance:.3f} m from active entrance')
    elif sdf_aligned and slam_aligned:
        conclusion = 'METADATA_WORLD_FRAME_CONVENTION_CONFLICT'
        reasons.append(f'runtime robot pose remains {robot_distance:.3f} m from active entrance despite aligned SDF/SLAM start pose')
    else:
        conclusion = 'INSUFFICIENT_EVIDENCE'

    guardrails = metadata.get('guardrails') or {}
    summary = {
        'phase': PHASE,
        'allowed_conclusions': sorted(PHASE40_ALLOWED_CONCLUSIONS),
        'conclusion': conclusion,
        'classification_reasons': reasons,
        'active_truth': active_truth,
        'sdf_tugbot_spawn_pose': sdf_pose,
        'slam_map_start_pose': slam_pose,
        'frame_alignment': {
            'latest_robot_pose_in_map': latest_robot_pose,
            'robot_to_active_entrance_distance_m': robot_distance,
            'alignment_ok_threshold_m': 0.75,
            'sdf_spawn_to_active_entrance_distance_m': sdf_distance,
            'slam_map_start_pose_to_active_entrance_distance_m': slam_distance,
            'map_base_link_lookup_samples': [((s.get('tf_lookups') or {}).get('map->base_link') or {}) for s in snapshots],
            'odom_base_link_lookup_samples': [((s.get('tf_lookups') or {}).get('odom->base_link') or {}) for s in snapshots],
            'map_odom_lookup_samples': [((s.get('tf_lookups') or {}).get('map->odom') or {}) for s in snapshots],
        },
        'timeline': timeline,
        'snapshot_count': len(snapshots),
        'guardrails': {
            'bounded_startup_pose_only_runtime_check': bool(guardrails.get('bounded_startup_pose_only_runtime_check', True)),
            'nav2_mppi_controller_params_modified': bool(guardrails.get('nav2_mppi_controller_params_modified', False)),
            'maze_explorer_strategy_modified': bool(guardrails.get('maze_explorer_strategy_modified', False)),
            'maze_explorer_started': bool(guardrails.get('maze_explorer_started', False)),
            'goals_dispatched': bool(guardrails.get('goals_dispatched', False)),
            'old_scaffold_world_used': bool(guardrails.get('old_scaffold_world_used', False)),
            'autonomous_success_claimed': bool(guardrails.get('autonomous_success_claimed', False)),
        },
        'phase37_conclusion': 'BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH',
        'phase39_conclusion': 'FRAME_ALIGNMENT_ISSUE_CONFIRMED',
        'autonomous_success_claimed': False,
        'artifacts': {
            'summary_json': str(output_dir / 'phase40_start_pose_entrance_frame_reconciliation.json'),
            'full_data_json': str(output_dir / 'phase40_start_pose_full_data.json'),
            'start_pose_entrance_alignment_overlay': str(output_dir / 'start_pose_entrance_alignment_overlay.png'),
        },
    }
    _draw_alignment_overlay(summary, output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'phase40_start_pose_entrance_frame_reconciliation.json').write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    return summary


class Phase40Recorder(Node):  # type: ignore[misc]
    def __init__(self, *, output_dir: Path, metadata_path: Path, world_path: Path, slam_params_path: Path, duration: float, snapshots: list[float]):
        super().__init__('phase40_start_pose_alignment_recorder')
        self.output_dir = output_dir
        self.metadata_path = metadata_path
        self.world_path = world_path
        self.slam_params_path = slam_params_path
        self.duration = duration
        self.snapshot_targets = sorted(snapshots)
        self.start_wall = time.time()
        self.done = False
        self.latest_odom = None
        self.snapshots: list[dict[str, Any]] = []
        self._captured_targets: set[float] = set()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos)
        self.create_timer(0.2, self._tick)

    def _elapsed(self) -> float:
        return time.time() - self.start_wall

    def _on_odom(self, msg: Any) -> None:
        self.latest_odom = msg

    def _snapshot(self, target: float | str) -> None:
        elapsed = self._elapsed()
        snap = {
            'target': target,
            'elapsed_sec': elapsed,
            'wall_time': time.time(),
            'odom': _odom_msg_to_dict(self.latest_odom),
            'tf_lookups': {
                'map->base_link': _lookup_transform_dict(self.tf_buffer, 'map', 'base_link'),
                'odom->base_link': _lookup_transform_dict(self.tf_buffer, 'odom', 'base_link'),
                'map->odom': _lookup_transform_dict(self.tf_buffer, 'map', 'odom'),
            },
        }
        self.snapshots.append(snap)
        print(f'captured Phase40 pose snapshot target={target} actual={elapsed:.3f}s', file=sys.stderr, flush=True)

    def _tick(self) -> None:
        elapsed = self._elapsed()
        for target in self.snapshot_targets:
            if target not in self._captured_targets and elapsed >= target:
                self._captured_targets.add(target)
                self._snapshot(target)
        if elapsed >= self.duration and not self.done:
            self.done = True

    def payload(self) -> dict[str, Any]:
        meta = _load_metadata(self.metadata_path)
        return {
            'metadata': {
                'created_wall_time': time.time(),
                'active_world': str(self.world_path),
                'active_metadata': str(self.metadata_path),
                'slam_params_file': str(self.slam_params_path),
                'guardrails': {
                    'bounded_startup_pose_only_runtime_check': True,
                    'nav2_mppi_controller_params_modified': False,
                    'maze_explorer_strategy_modified': False,
                    'maze_explorer_started': False,
                    'goals_dispatched': False,
                    'old_scaffold_world_used': False,
                    'autonomous_success_claimed': False,
                },
            },
            'active_truth': {
                'entrance': {
                    'x_m': float((meta.get('entrance') or DEFAULT_ENTRANCE).get('x_m', DEFAULT_ENTRANCE['x_m'])),
                    'y_m': float((meta.get('entrance') or DEFAULT_ENTRANCE).get('y_m', DEFAULT_ENTRANCE['y_m'])),
                    'yaw_rad': float((meta.get('entrance') or DEFAULT_ENTRANCE).get('yaw_rad', DEFAULT_ENTRANCE['yaw_rad'])),
                },
                'exit': {
                    'x_m': float((meta.get('exit') or DEFAULT_EXIT).get('x_m', DEFAULT_EXIT['x_m'])),
                    'y_m': float((meta.get('exit') or DEFAULT_EXIT).get('y_m', DEFAULT_EXIT['y_m'])),
                    'radius_m': float((meta.get('exit') or DEFAULT_EXIT).get('radius_m', DEFAULT_EXIT['radius_m'])),
                },
            },
            'sdf_tugbot_spawn_pose': _extract_sdf_pose(self.world_path),
            'slam_map_start_pose': _extract_slam_map_start_pose(self.slam_params_path),
            'snapshots': self.snapshots,
        }


def _parse_snapshots(text: str) -> list[float]:
    return [float(part.strip()) for part in text.split(',') if part.strip()]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=Path('log/phase40_start_pose_entrance_frame_reconciliation'))
    parser.add_argument('--metadata', type=Path, default=Path('src/tugbot_maze/config/maze_20260528_scaled_instance.yaml'))
    parser.add_argument('--world', type=Path, default=Path('src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf'))
    parser.add_argument('--slam-params', type=Path, default=Path('src/tugbot_navigation/config/slam_toolbox_params.yaml'))
    parser.add_argument('--duration', type=float, default=45.0)
    parser.add_argument('--snapshots', default='15,30,45')
    parser.add_argument('--analyze-existing', type=Path)
    args = parser.parse_args(argv)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    if args.analyze_existing:
        payload = json.loads(args.analyze_existing.read_text(encoding='utf-8'))
        summary = _analyze(payload, args.output_dir)
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0

    if rclpy is None:
        raise RuntimeError('rclpy/tf2 runtime imports unavailable; source ROS before runtime recording')

    rclpy.init()
    node = Phase40Recorder(
        output_dir=args.output_dir,
        metadata_path=args.metadata,
        world_path=args.world,
        slam_params_path=args.slam_params,
        duration=float(args.duration),
        snapshots=_parse_snapshots(args.snapshots),
    )

    stop_requested = False

    def _stop(_signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        stop_requested = True
        node.done = True

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    try:
        while rclpy.ok() and not node.done and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
        payload = node.payload()
        full_path = args.output_dir / 'phase40_start_pose_full_data.json'
        full_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding='utf-8')
        summary = _analyze(payload, args.output_dir)
        print(json.dumps(summary, indent=2, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
