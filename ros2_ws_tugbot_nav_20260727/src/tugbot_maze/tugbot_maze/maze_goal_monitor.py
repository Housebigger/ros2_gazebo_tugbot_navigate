#!/usr/bin/env python3
"""Exit-region monitor for the Tugbot maze workspace.

This node is intentionally small in the 20260522 framework stage. It watches the
robot pose from TF and publishes /maze/exit_reached once the configured exit
region is reached. The exact success log token is part of the contract:
MAZE_EXIT_REACHED.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import tf2_ros


class MazeGoalMonitor(Node):
    def __init__(self) -> None:
        super().__init__('maze_goal_monitor')
        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link').value)
        self.exit_x = float(self.declare_parameter('exit_x', 4.0).value)
        self.exit_y = float(self.declare_parameter('exit_y', 3.0).value)
        self.exit_radius = float(self.declare_parameter('exit_radius', 0.6).value)
        self.check_rate_hz = float(self.declare_parameter('check_rate_hz', 5.0).value)
        self.success_topic = str(self.declare_parameter('success_topic', '/maze/exit_reached').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Bool, self.success_topic, 1)
        self.exit_reached = False
        self.timer = self.create_timer(1.0 / max(self.check_rate_hz, 0.1), self._tick)

        self.get_logger().info(
            'maze_goal_monitor started: map_frame=%s base_frame=%s exit=(%.3f, %.3f) radius=%.3f topic=%s'
            % (self.map_frame, self.base_frame, self.exit_x, self.exit_y, self.exit_radius, self.success_topic)
        )

    def _lookup_pose_xy(self) -> Optional[tuple[float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001 - ROS TF exceptions vary by distro/version.
            self.get_logger().debug('waiting for TF %s -> %s: %s' % (self.map_frame, self.base_frame, exc))
            return None
        translation = transform.transform.translation
        return float(translation.x), float(translation.y)

    def _tick(self) -> None:
        pose_xy = self._lookup_pose_xy()
        if pose_xy is None:
            return
        x, y = pose_xy
        distance = math.hypot(x - self.exit_x, y - self.exit_y)
        reached = distance <= self.exit_radius
        self.publisher.publish(Bool(data=reached))
        if reached and not self.exit_reached:
            self.exit_reached = True
            self.get_logger().info(
                'MAZE_EXIT_REACHED pose=(%.3f, %.3f) exit=(%.3f, %.3f) distance=%.3f radius=%.3f'
                % (x, y, self.exit_x, self.exit_y, distance, self.exit_radius)
            )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MazeGoalMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
