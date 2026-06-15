#!/usr/bin/env python3
"""Phase81 raw evidence recorder for the held Goal2 timeout scene.

Captures raw /scan, /local_costmap/costmap, /local_costmap/published_footprint,
/odom, and TF snapshots.  It only observes topics; it sends no goals and changes
no navigation parameters.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PolygonStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

RUN_ID = 'phase81_goal2_forward_open_machine_evidence_capture'
FRAME_PAIRS = [
    ('map', 'base_link'),
    ('odom', 'base_link'),
    ('map', 'odom'),
    ('base_link', 'scan_omni'),
    ('base_link', 'base_scan'),
]


def quat_to_yaw(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def stamp_to_sec(msg: Any) -> float:
    stamp = msg.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def summarize_transform(msg: TransformStamped) -> dict[str, Any]:
    t = msg.transform.translation
    r = msg.transform.rotation
    return {
        'available': True,
        'parent': msg.header.frame_id,
        'child': msg.child_frame_id,
        'stamp_sec': stamp_to_sec(msg),
        'translation': {'x': float(t.x), 'y': float(t.y), 'z': float(t.z)},
        'yaw': quat_to_yaw(r),
    }


def transform_pose_xy_yaw(msg: TransformStamped) -> list[float]:
    t = msg.transform.translation
    return [float(t.x), float(t.y), quat_to_yaw(msg.transform.rotation)]


def scan_to_dict(msg: LaserScan | None) -> dict[str, Any] | None:
    if msg is None:
        return None
    return {
        'topic': '/scan',
        'frame_id': msg.header.frame_id,
        'stamp_sec': stamp_to_sec(msg),
        'angle_min': float(msg.angle_min),
        'angle_max': float(msg.angle_max),
        'angle_increment': float(msg.angle_increment),
        'time_increment': float(msg.time_increment),
        'scan_time': float(msg.scan_time),
        'range_min': float(msg.range_min),
        'range_max': float(msg.range_max),
        'ranges': [float(v) for v in msg.ranges],
        'intensities': [float(v) for v in msg.intensities],
    }


def costmap_to_dict(msg: OccupancyGrid | None) -> dict[str, Any] | None:
    if msg is None:
        return None
    origin = msg.info.origin
    return {
        'topic': '/local_costmap/costmap',
        'frame_id': msg.header.frame_id,
        'stamp_sec': stamp_to_sec(msg),
        'info': {
            'resolution': float(msg.info.resolution),
            'width': int(msg.info.width),
            'height': int(msg.info.height),
            'origin': {
                'position': {
                    'x': float(origin.position.x),
                    'y': float(origin.position.y),
                    'z': float(origin.position.z),
                },
                'orientation': {
                    'x': float(origin.orientation.x),
                    'y': float(origin.orientation.y),
                    'z': float(origin.orientation.z),
                    'w': float(origin.orientation.w),
                },
                'orientation_yaw': quat_to_yaw(origin.orientation),
            },
        },
        'data': [int(v) for v in msg.data],
    }


def footprint_to_dict(msg: PolygonStamped | None) -> dict[str, Any] | None:
    if msg is None:
        return None
    return {
        'topic': '/local_costmap/published_footprint',
        'frame_id': msg.header.frame_id,
        'stamp_sec': stamp_to_sec(msg),
        'points': [[float(p.x), float(p.y), float(p.z)] for p in msg.polygon.points],
    }


def odom_to_dict(msg: Odometry | None) -> dict[str, Any] | None:
    if msg is None:
        return None
    pose = msg.pose.pose
    twist = msg.twist.twist
    return {
        'topic': '/odom',
        'frame_id': msg.header.frame_id,
        'child_frame_id': msg.child_frame_id,
        'stamp_sec': stamp_to_sec(msg),
        'pose': {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'z': float(pose.position.z),
            'yaw': quat_to_yaw(pose.orientation),
        },
        'twist': {
            'linear_x': float(twist.linear.x),
            'linear_y': float(twist.linear.y),
            'angular_z': float(twist.angular.z),
        },
    }


class Phase81Recorder(Node):
    def __init__(self, duration: float, target: list[float] | None, source_phase80: dict[str, Any]) -> None:
        super().__init__('phase81_goal2_forward_open_machine_evidence_recorder')
        self.duration = duration
        self.started = time.time()
        self.target = target
        self.source_phase80 = source_phase80
        self.scan_msg: LaserScan | None = None
        self.costmap_msg: OccupancyGrid | None = None
        self.footprint_msg: PolygonStamped | None = None
        self.odom_msg: Odometry | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._on_costmap, 10)
        self.create_subscription(PolygonStamped, '/local_costmap/published_footprint', self._on_footprint, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan_msg = msg

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self.costmap_msg = msg

    def _on_footprint(self, msg: PolygonStamped) -> None:
        self.footprint_msg = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.odom_msg = msg

    def snapshot(self) -> dict[str, Any]:
        transforms: dict[str, Any] = {}
        robot_pose_by_frame: dict[str, list[float]] = {}
        frame_pairs = list(FRAME_PAIRS)
        if self.scan_msg is not None and self.scan_msg.header.frame_id:
            scan_frame = self.scan_msg.header.frame_id.lstrip('/')
            if ('base_link', scan_frame) not in frame_pairs:
                frame_pairs.append(('base_link', scan_frame))
        for parent, child in frame_pairs:
            key = f'{parent}->{child}'
            try:
                transform = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                transforms[key] = summarize_transform(transform)
                if child == 'base_link':
                    robot_pose_by_frame[parent] = transform_pose_xy_yaw(transform)
            except TransformException as exc:
                transforms[key] = {'available': False, 'error': str(exc)}
        odom = odom_to_dict(self.odom_msg)
        if odom:
            robot_pose_by_frame.setdefault('odom', [odom['pose']['x'], odom['pose']['y'], odom['pose']['yaw']])
        map_pose = robot_pose_by_frame.get('map')
        terminal_pose = map_pose or robot_pose_by_frame.get('odom')
        return {
            'run_id': RUN_ID,
            'capture_wall_time': time.time(),
            'elapsed_sec': time.time() - self.started,
            'goal_sequence': 2,
            'target': self.target,
            'terminal_pose': terminal_pose,
            'robot_pose_by_frame': robot_pose_by_frame,
            'scan': scan_to_dict(self.scan_msg),
            'local_costmap': costmap_to_dict(self.costmap_msg),
            'footprint': footprint_to_dict(self.footprint_msg),
            'odom': odom,
            'tf': transforms,
            'source_phase80_classification': self.source_phase80.get('classification'),
            'source_phase80_forward_open_status': self.source_phase80.get('forward_open_evidence_status'),
            'source_phase80_json': self.source_phase80.get('_path'),
            'guardrails': [
                'recording-only; no goals sent',
                'No maze_explorer strategy changed',
                'No Nav2/MPPI/controller tuning',
                'No autonomous exploration success claimed',
                'No exit success claimed',
                'Phase82 not entered',
            ],
        }


def load_phase80(path: Path | None) -> dict[str, Any]:
    if path and path.exists() and path.stat().st_size:
        data = json.loads(path.read_text(encoding='utf-8', errors='replace'))
        data['_path'] = str(path)
        return data
    return {}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-sec', type=float, default=5.0)
    parser.add_argument('--phase80-json', type=Path, default=Path('log/phase80_goal2_near_goal_forward_open_corridor_diagnostic_classification/phase80_goal2_near_goal_forward_open_corridor_diagnostic_classification.json'))
    parser.add_argument('--output-json', type=Path, required=True)
    args = parser.parse_args()

    phase80 = load_phase80(args.phase80_json)
    target = phase80.get('target') if isinstance(phase80.get('target'), list) else None
    rclpy.init()
    node = Phase81Recorder(args.duration_sec, target, phase80)
    deadline = time.time() + args.duration_sec
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    evidence = node.snapshot()
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(evidence, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({
        'output_json': str(args.output_json),
        'scan_available': evidence.get('scan') is not None,
        'local_costmap_available': evidence.get('local_costmap') is not None,
        'footprint_available': evidence.get('footprint') is not None,
        'odom_available': evidence.get('odom') is not None,
        'robot_pose_frames': sorted(evidence.get('robot_pose_by_frame', {}).keys()),
    }, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
