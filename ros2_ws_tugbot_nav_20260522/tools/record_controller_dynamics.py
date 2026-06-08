#!/usr/bin/env python3
"""Record odom and cmd_vel/cmd_vel_nav samples for Phase 21 controller diagnostics."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ControllerDynamicsRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('controller_dynamics_recorder')
        self.args = args
        self.output_path = Path(args.output).expanduser().resolve()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.fp = self.output_path.open('w', encoding='utf-8')
        self.started_at = time.time()
        self.count = 0
        self.odom_count = 0
        self.cmd_count = 0
        self.done = False
        self.odom_sub = self.create_subscription(Odometry, args.odom_topic, self._on_odom, 20)
        self.cmd_sub = self.create_subscription(Twist, args.cmd_topic, self._on_cmd, 20)
        self.timer = self.create_timer(0.2, self._on_timer)
        self.get_logger().info(
            f'recording odom={args.odom_topic} cmd_vel={args.cmd_topic} to {self.output_path}'
        )

    def _write_row(self, row: dict) -> None:
        self.count += 1
        row.setdefault('wall_time', time.time())
        row['elapsed_sec'] = row['wall_time'] - self.started_at
        row['seq'] = self.count
        self.fp.write(json.dumps(row, sort_keys=True) + '\n')
        self.fp.flush()
        if self.count >= self.args.max_samples:
            self.done = True

    def _on_odom(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        twist = msg.twist.twist
        self.odom_count += 1
        self._write_row({
            'source': 'odom',
            'ros_stamp_sec': float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
            'frame_id': msg.header.frame_id,
            'child_frame_id': msg.child_frame_id,
            'x': float(pos.x),
            'y': float(pos.y),
            'z': float(pos.z),
            'yaw': yaw_from_quaternion(float(ori.x), float(ori.y), float(ori.z), float(ori.w)),
            'odom_linear_x': float(twist.linear.x),
            'odom_linear_y': float(twist.linear.y),
            'odom_angular_z': float(twist.angular.z),
        })

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd_count += 1
        self._write_row({
            'source': 'cmd_vel',
            'linear_x': float(msg.linear.x),
            'linear_y': float(msg.linear.y),
            'linear_z': float(msg.linear.z),
            'angular_x': float(msg.angular.x),
            'angular_y': float(msg.angular.y),
            'angular_z': float(msg.angular.z),
        })

    def _on_timer(self) -> None:
        if time.time() - self.started_at >= self.args.timeout_sec:
            self.done = True

    def close(self) -> None:
        self.fp.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--cmd-topic', default='cmd_vel_nav')
    parser.add_argument('--max-samples', type=int, default=20000)
    parser.add_argument('--timeout-sec', type=float, default=360.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = ControllerDynamicsRecorder(args)
    interrupted_by_external_shutdown = False
    try:
        while rclpy.ok() and not node.done:
            try:
                rclpy.spin_once(node, timeout_sec=0.2)
            except rclpy.executors.ExternalShutdownException:
                interrupted_by_external_shutdown = True
                break
    finally:
        out = node.output_path
        count = node.count
        odom_count = node.odom_count
        cmd_count = node.cmd_count
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps({
        'output': str(out),
        'samples': count,
        'odom_samples': odom_count,
        'cmd_vel_samples': cmd_count,
        'external_shutdown': interrupted_by_external_shutdown,
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
