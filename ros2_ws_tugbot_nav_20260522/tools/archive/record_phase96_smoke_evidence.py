#!/usr/bin/env python3
"""Phase96 smoke evidence recorder.

Captures a short read-only snapshot for the Phase88/92 refinement-chain bounded
multi-goal smoke: latest /maze/goal_events, /scan, /local_costmap/costmap,
/local_costmap/published_footprint, /odom and TF. It changes no parameters and
creates no action clients.
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
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

RUN_ID = 'phase96_refinement_chain_bounded_multi_goal_smoke'
FRAME_PAIRS = [('map', 'base_link'), ('odom', 'base_link'), ('map', 'odom'), ('base_link', 'base_scan'), ('base_link', 'scan_omni')]


def quat_to_yaw(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def stamp_to_sec(msg: Any) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9


def summarize_transform(msg: TransformStamped) -> dict[str, Any]:
    t = msg.transform.translation
    return {
        'available': True,
        'parent': msg.header.frame_id,
        'child': msg.child_frame_id,
        'stamp_sec': stamp_to_sec(msg),
        'translation': {'x': float(t.x), 'y': float(t.y), 'z': float(t.z)},
        'yaw': quat_to_yaw(msg.transform.rotation),
    }


def pose_from_transform(msg: TransformStamped) -> list[float]:
    t = msg.transform.translation
    return [float(t.x), float(t.y), quat_to_yaw(msg.transform.rotation)]


def scan_to_dict(msg: LaserScan | None) -> dict[str, Any] | None:
    if msg is None:
        return None
    finite = [float(v) for v in msg.ranges if math.isfinite(float(v))]
    return {
        'topic': '/scan',
        'frame_id': msg.header.frame_id,
        'stamp_sec': stamp_to_sec(msg),
        'angle_min': float(msg.angle_min),
        'angle_max': float(msg.angle_max),
        'angle_increment': float(msg.angle_increment),
        'range_min': float(msg.range_min),
        'range_max': float(msg.range_max),
        'ranges_count': len(msg.ranges),
        'ranges_min': min(finite) if finite else None,
        'ranges_max_finite': max(finite) if finite else None,
    }


def costmap_to_dict(msg: OccupancyGrid | None) -> dict[str, Any] | None:
    if msg is None:
        return None
    origin = msg.info.origin
    values = [int(v) for v in msg.data]
    return {
        'topic': '/local_costmap/costmap',
        'frame_id': msg.header.frame_id,
        'stamp_sec': stamp_to_sec(msg),
        'info': {
            'resolution': float(msg.info.resolution),
            'width': int(msg.info.width),
            'height': int(msg.info.height),
            'origin': {'x': float(origin.position.x), 'y': float(origin.position.y), 'yaw': quat_to_yaw(origin.orientation)},
        },
        'summary': {
            'cell_count': len(values),
            'lethal_count': sum(1 for v in values if v >= 99),
            'high_cost_count': sum(1 for v in values if v >= 75),
            'max': max(values) if values else None,
        },
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
        'pose': {'x': float(pose.position.x), 'y': float(pose.position.y), 'yaw': quat_to_yaw(pose.orientation)},
        'twist': {'linear_x': float(twist.linear.x), 'linear_y': float(twist.linear.y), 'angular_z': float(twist.angular.z)},
    }


class Phase96Recorder(Node):
    def __init__(self, duration: float, source_analysis: dict[str, Any]) -> None:
        super().__init__('phase96_smoke_evidence_recorder')
        self.duration = duration
        self.started = time.time()
        self.source_analysis = source_analysis
        self.goal_event_msg: String | None = None
        self.scan_msg: LaserScan | None = None
        self.costmap_msg: OccupancyGrid | None = None
        self.footprint_msg: PolygonStamped | None = None
        self.odom_msg: Odometry | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(String, '/maze/goal_events', self._on_goal_event, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._on_costmap, 10)
        self.create_subscription(PolygonStamped, '/local_costmap/published_footprint', self._on_footprint, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

    def _on_goal_event(self, msg: String) -> None:
        self.goal_event_msg = msg

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
        for parent, child in FRAME_PAIRS:
            key = f'{parent}->{child}'
            try:
                transform = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                transforms[key] = summarize_transform(transform)
                if child == 'base_link':
                    robot_pose_by_frame[parent] = pose_from_transform(transform)
            except TransformException as exc:
                transforms[key] = {'available': False, 'error': str(exc)}
        odom = odom_to_dict(self.odom_msg)
        if odom:
            robot_pose_by_frame.setdefault('odom', [odom['pose']['x'], odom['pose']['y'], odom['pose']['yaw']])
        goal_payload = None
        if self.goal_event_msg is not None:
            try:
                goal_payload = json.loads(self.goal_event_msg.data)
            except Exception:
                goal_payload = {'raw': self.goal_event_msg.data}
        return {
            'run_id': RUN_ID,
            'capture_wall_time': time.time(),
            'elapsed_sec': time.time() - self.started,
            'goal_event': {'topic': '/maze/goal_events', 'payload': goal_payload} if goal_payload is not None else None,
            'robot_pose_by_frame': robot_pose_by_frame,
            'scan': scan_to_dict(self.scan_msg),
            'local_costmap': costmap_to_dict(self.costmap_msg),
            'footprint': footprint_to_dict(self.footprint_msg),
            'odom': odom,
            'tf': transforms,
            'source_analysis_classification': self.source_analysis.get('classification'),
            'guardrails': [
                'recording-only; no goals are created',
                'No maze_explorer strategy changed',
                'No Phase88/92 logic changed',
                'No branch scoring changed',
                'No exploration order changed',
                'No centerline gate changed',
                'No directional readiness changed',
                'No fallback/terminal acceptance changed',
                'No Nav2/MPPI/controller tuning',
                'No inflation/robot_radius/clearance_radius_m/map threshold tuning',
                'No autonomous exploration success claimed',
                'No exit success claimed',
                'Phase97 not entered',
            ],
        }


def load_json(path: Path | None) -> dict[str, Any]:
    if path and path.exists() and path.stat().st_size:
        try:
            data = json.loads(path.read_text(encoding='utf-8', errors='replace'))
            if isinstance(data, dict):
                data['_path'] = str(path)
                return data
        except Exception:
            return {}
    return {}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-sec', type=float, default=5.0)
    parser.add_argument('--analysis-json', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Phase96Recorder(args.duration_sec, load_json(args.analysis_json))
    try:
        deadline = time.time() + max(0.1, args.duration_sec)
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        snapshot = node.snapshot()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(snapshot, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        print(json.dumps({'output': str(args.output), 'scan': snapshot['scan'] is not None, 'odom': snapshot['odom'] is not None, 'tf_keys': sorted(snapshot['tf'])}, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
