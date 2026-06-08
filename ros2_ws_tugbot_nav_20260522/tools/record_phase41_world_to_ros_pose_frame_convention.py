#!/usr/bin/env python3
"""Phase41 bounded world-to-ROS pose frame convention recorder.

Convention A evidence:
- Gazebo/SDF world_frame_truth remains in active scaled2x world coordinates.
- ROS runtime map_frame_truth is offset so entrance_map=(0, 0, 0) and
  exit_map=exit_world-entrance_world.
- Runtime checks only /odom and TF map->base_link, odom->base_link, map->odom.
- No maze_explorer, no goals, no Nav2/MPPI/controller tuning, no strategy edits.
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

try:  # Runtime-only imports; offline analysis must work in plain pytest.
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

PHASE = 'Phase41 World-to-ROS Pose Frame Convention Fix'
RUN_ID = 'phase41_world_to_ros_pose_frame_convention'
ALLOWED_CONCLUSIONS = {
    'MAP_FRAME_TRUTH_ALIGNED_READY_FOR_PHASE37_RERUN',
    'EXPLICIT_WORLD_TO_MAP_TRANSFORM_REQUIRED',
    'FRAME_CONVENTION_UNRESOLVED',
}
TF_LOOKUP_PAIRS = ('map->base_link', 'odom->base_link', 'map->odom')
DEFAULT_WORLD_ENTRANCE = {'x_m': -11.011281, 'y_m': -9.025070, 'yaw_rad': 0.0}
DEFAULT_WORLD_EXIT = {'x_m': 10.061281, 'y_m': 9.058496, 'radius_m': 1.2}
DEFAULT_MAP_ENTRANCE = {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
DEFAULT_MAP_EXIT = {'x_m': 21.072562, 'y_m': 18.083566, 'radius_m': 1.2}


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
        'twist': {'linear': _position_dict(twist.linear), 'angular': _position_dict(twist.angular)},
    }


def _load_metadata(path: Path | None) -> dict[str, Any]:
    if path and path.exists():
        return yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    return {
        'world_frame_truth': {'entrance': dict(DEFAULT_WORLD_ENTRANCE), 'exit': dict(DEFAULT_WORLD_EXIT)},
        'map_frame_truth': {'entrance': dict(DEFAULT_MAP_ENTRANCE), 'exit': dict(DEFAULT_MAP_EXIT)},
    }


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
            return {'available': len(vals) >= 6, 'name': (inc.findtext('name') or '').strip(), 'uri': 'model://tugbot', 'pose': vals}
    return {'available': False, 'error': 'missing model://tugbot include', 'pose': None}


def _pose_distance_xy(a: dict[str, float] | list[float] | None, b: dict[str, float] | None) -> float | None:
    if a is None or b is None:
        return None
    if isinstance(a, list):
        if len(a) < 2:
            return None
        ax, ay = float(a[0]), float(a[1])
    else:
        ax, ay = float(a.get('x_m', a.get('x', 0.0))), float(a.get('y_m', a.get('y', 0.0)))
    return math.hypot(ax - float(b.get('x_m', b.get('x', 0.0))), ay - float(b.get('y_m', b.get('y', 0.0))))


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
    pos = ((odom.get('pose') or {}).get('position') or {})
    if pos:
        return {'x': float(pos.get('x', 0.0)), 'y': float(pos.get('y', 0.0)), 'yaw_rad': float((odom.get('pose') or {}).get('yaw_rad', 0.0))}
    return None


def _metadata_truth(meta: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    world_truth = meta.get('world_frame_truth') or {'entrance': meta.get('entrance', DEFAULT_WORLD_ENTRANCE), 'exit': meta.get('exit', DEFAULT_WORLD_EXIT)}
    map_truth = meta.get('map_frame_truth') or {'entrance': DEFAULT_MAP_ENTRANCE, 'exit': DEFAULT_MAP_EXIT}
    return world_truth, map_truth


def _draw_overlay(summary: dict[str, Any], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    img = Image.new('RGB', (900, 650), 'white')
    draw = ImageDraw.Draw(img)
    margin = 70
    points: list[tuple[float, float, str, str]] = []
    map_ent = summary['map_frame_truth']['entrance']
    map_exit = summary['map_frame_truth']['exit']
    points.append((float(map_ent['x_m']), float(map_ent['y_m']), 'entrance_map', 'green'))
    points.append((float(map_exit['x_m']), float(map_exit['y_m']), 'exit_map', 'blue'))
    robot = (summary.get('frame_alignment') or {}).get('latest_robot_pose_in_map')
    if robot:
        points.append((float(robot['x']), float(robot['y']), 'runtime robot', 'red'))
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    span = max(max(xs) - min(xs), max(ys) - min(ys), 2.0)
    cx, cy = (max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0
    scale = min((900 - 2 * margin) / span, (650 - 2 * margin) / span)

    def px(x: float, y: float) -> tuple[int, int]:
        return int(450 + (x - cx) * scale), int(325 - (y - cy) * scale)

    draw.rectangle((margin, margin, 900 - margin, 650 - margin), outline='gray')
    draw.line([px(float(map_ent['x_m']), float(map_ent['y_m'])), px(float(map_exit['x_m']), float(map_exit['y_m']))], fill='blue', width=2)
    for x, y, label, color in points:
        sx, sy = px(x, y)
        draw.ellipse((sx - 8, sy - 8, sx + 8, sy + 8), fill=color, outline='black')
        draw.text((sx + 10, sy - 8), f'{label}: ({x:.3f}, {y:.3f})', fill='black')
    draw.text((20, 20), PHASE, fill='black')
    draw.text((20, 42), f'conclusion: {summary.get("conclusion")}', fill='black')
    draw.text((20, 64), f'robot_to_map_entrance_m: {(summary.get("frame_alignment") or {}).get("robot_to_map_entrance_distance_m")}', fill='black')
    draw.text((20, 86), f'exit_map_distance_m: {summary.get("exit_map_distance_m")}', fill='black')
    img.save(output_dir / 'world_to_ros_map_frame_truth_overlay.png')


def _analyze(payload: dict[str, Any], output_dir: Path) -> dict[str, Any]:
    meta = payload.get('metadata_yaml') or {}
    world_truth, map_truth = payload.get('world_frame_truth'), payload.get('map_frame_truth')
    if not world_truth or not map_truth:
        world_truth, map_truth = _metadata_truth(meta)
    map_entrance = map_truth['entrance']
    map_exit = map_truth['exit']
    world_entrance = world_truth['entrance']
    world_exit = world_truth['exit']
    snapshots = payload.get('snapshots') or []
    latest = snapshots[-1] if snapshots else {}
    latest_pose = _tf_robot_pose(latest)
    robot_to_map_entrance = _pose_distance_xy(latest_pose, map_entrance) if latest_pose else None
    exit_map_distance = math.hypot(float(map_exit['x_m']) - float(map_entrance['x_m']), float(map_exit['y_m']) - float(map_entrance['y_m']))
    transformed_exit_x = float(world_exit['x_m']) - float(world_entrance['x_m'])
    transformed_exit_y = float(world_exit['y_m']) - float(world_entrance['y_m'])
    transform_matches = math.isclose(float(map_exit['x_m']), transformed_exit_x, abs_tol=1e-6) and math.isclose(float(map_exit['y_m']), transformed_exit_y, abs_tol=1e-6)
    map_entrance_zero = abs(float(map_entrance['x_m'])) < 1e-6 and abs(float(map_entrance['y_m'])) < 1e-6
    runtime_alignment_ok = robot_to_map_entrance is not None and robot_to_map_entrance < 0.75
    exit_reasonable = 20.0 <= exit_map_distance <= 32.0
    tf_available = {k: bool(((latest.get('tf_lookups') or {}).get(k) or {}).get('available')) for k in TF_LOOKUP_PAIRS}

    reasons = []
    if transform_matches:
        reasons.append('map_frame_truth exit equals world exit minus world entrance')
    else:
        reasons.append('map_frame_truth exit does not match world-frame offset')
    if map_entrance_zero:
        reasons.append('map_frame_truth entrance is at ROS map origin')
    if runtime_alignment_ok:
        reasons.append(f'runtime map->base_link is {robot_to_map_entrance:.3f} m from map_frame entrance')
    elif robot_to_map_entrance is None:
        reasons.append('runtime TF/odom pose evidence unavailable')
    else:
        reasons.append(f'runtime map->base_link is {robot_to_map_entrance:.3f} m from map_frame entrance')
    if exit_reasonable:
        reasons.append(f'exit_map distance is reasonable at {exit_map_distance:.3f} m')
    else:
        reasons.append(f'exit_map distance is outside expected bounds: {exit_map_distance:.3f} m')

    if transform_matches and map_entrance_zero and runtime_alignment_ok and exit_reasonable:
        conclusion = 'MAP_FRAME_TRUTH_ALIGNED_READY_FOR_PHASE37_RERUN'
    elif transform_matches and map_entrance_zero and robot_to_map_entrance is not None:
        conclusion = 'EXPLICIT_WORLD_TO_MAP_TRANSFORM_REQUIRED'
    else:
        conclusion = 'FRAME_CONVENTION_UNRESOLVED'

    timeline = []
    for snap in snapshots:
        pose = _tf_robot_pose(snap)
        timeline.append({
            'elapsed_sec': snap.get('elapsed_sec'),
            'robot_pose_in_map': pose,
            'robot_to_map_entrance_distance_m': _pose_distance_xy(pose, map_entrance) if pose else None,
            'tf_available': {k: bool(((snap.get('tf_lookups') or {}).get(k) or {}).get('available')) for k in TF_LOOKUP_PAIRS},
            'odom_available': snap.get('odom') is not None,
        })

    guardrails = (payload.get('metadata') or {}).get('guardrails') or {}
    summary = {
        'phase': PHASE,
        'allowed_conclusions': sorted(ALLOWED_CONCLUSIONS),
        'conclusion': conclusion,
        'classification_reasons': reasons,
        'selected_convention': 'A_map_frame_follows_slam_startup_pose',
        'world_frame_truth': world_truth,
        'map_frame_truth': map_truth,
        'sdf_tugbot_spawn_pose': payload.get('sdf_tugbot_spawn_pose'),
        'frame_alignment': {
            'latest_robot_pose_in_map': latest_pose,
            'robot_to_map_entrance_distance_m': robot_to_map_entrance,
            'alignment_ok_threshold_m': 0.75,
            'tf_available_latest': tf_available,
            'map_base_link_lookup_samples': [((s.get('tf_lookups') or {}).get('map->base_link') or {}) for s in snapshots],
            'odom_base_link_lookup_samples': [((s.get('tf_lookups') or {}).get('odom->base_link') or {}) for s in snapshots],
            'map_odom_lookup_samples': [((s.get('tf_lookups') or {}).get('map->odom') or {}) for s in snapshots],
        },
        'exit_map_distance_m': exit_map_distance,
        'world_to_map_transform': {
            'type': 'translation_offset_by_world_entrance',
            'map_x': 'world_x - world_entrance_x',
            'map_y': 'world_y - world_entrance_y',
            'transformed_exit_x_m': transformed_exit_x,
            'transformed_exit_y_m': transformed_exit_y,
            'transform_matches_metadata': transform_matches,
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
            'phase37_rerun_started': bool(guardrails.get('phase37_rerun_started', False)),
        },
        'phase37_conclusion': 'BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH',
        'phase40_conclusion': 'METADATA_WORLD_FRAME_CONVENTION_CONFLICT',
        'autonomous_success_claimed': False,
        'artifacts': {
            'summary_json': str(output_dir / 'phase41_world_to_ros_pose_frame_convention.json'),
            'full_data_json': str(output_dir / 'phase41_world_to_ros_pose_frame_convention_full_data.json'),
            'overlay_png': str(output_dir / 'world_to_ros_map_frame_truth_overlay.png'),
        },
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    _draw_overlay(summary, output_dir)
    (output_dir / 'phase41_world_to_ros_pose_frame_convention.json').write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    return summary


class Phase41Recorder(Node):  # type: ignore[misc]
    def __init__(self, *, output_dir: Path, metadata_path: Path, world_path: Path, duration: float, snapshots: list[float]):
        super().__init__('phase41_world_to_ros_pose_frame_convention_recorder')
        self.output_dir = output_dir
        self.metadata_path = metadata_path
        self.world_path = world_path
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
        self.snapshots.append({
            'target': target,
            'elapsed_sec': elapsed,
            'wall_time': time.time(),
            'odom': _odom_msg_to_dict(self.latest_odom),
            'tf_lookups': {
                'map->base_link': _lookup_transform_dict(self.tf_buffer, 'map', 'base_link'),
                'odom->base_link': _lookup_transform_dict(self.tf_buffer, 'odom', 'base_link'),
                'map->odom': _lookup_transform_dict(self.tf_buffer, 'map', 'odom'),
            },
        })
        print(f'captured Phase41 pose snapshot target={target} actual={elapsed:.3f}s', file=sys.stderr, flush=True)

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
        world_truth, map_truth = _metadata_truth(meta)
        return {
            'metadata': {
                'created_wall_time': time.time(),
                'active_world': str(self.world_path),
                'active_metadata': str(self.metadata_path),
                'guardrails': {
                    'bounded_startup_pose_only_runtime_check': True,
                    'nav2_mppi_controller_params_modified': False,
                    'maze_explorer_strategy_modified': False,
                    'maze_explorer_started': False,
                    'goals_dispatched': False,
                    'old_scaffold_world_used': False,
                    'autonomous_success_claimed': False,
                    'phase37_rerun_started': False,
                },
            },
            'metadata_yaml': meta,
            'world_frame_truth': world_truth,
            'map_frame_truth': map_truth,
            'sdf_tugbot_spawn_pose': _extract_sdf_pose(self.world_path),
            'snapshots': self.snapshots,
        }


def _parse_snapshots(text: str) -> list[float]:
    return [float(part.strip()) for part in text.split(',') if part.strip()]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=Path('log/phase41_world_to_ros_pose_frame_convention'))
    parser.add_argument('--metadata', type=Path, default=Path('src/tugbot_maze/config/maze_20260528_scaled_instance.yaml'))
    parser.add_argument('--world', type=Path, default=Path('src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf'))
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
    node = Phase41Recorder(output_dir=args.output_dir, metadata_path=args.metadata, world_path=args.world, duration=float(args.duration), snapshots=_parse_snapshots(args.snapshots))
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
        full_path = args.output_dir / 'phase41_world_to_ros_pose_frame_convention_full_data.json'
        full_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding='utf-8')
        summary = _analyze(payload, args.output_dir)
        print(json.dumps(summary, indent=2, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
