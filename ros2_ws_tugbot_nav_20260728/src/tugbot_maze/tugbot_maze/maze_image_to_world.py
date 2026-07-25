#!/usr/bin/env python3
"""Placeholder maze image-to-world utility for the 20260522 framework stage.

The source JPEG is a 3D/shadowed reference image, not a clean occupancy map. This
script currently records that policy and prints the intended inputs. A later
phase should replace this placeholder with a cleaned/manual wall extraction that
writes tugbot_maze_world.sdf.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node


class MazeImageToWorld(Node):
    def __init__(self) -> None:
        super().__init__('maze_image_to_world')
        self.config_file = str(self.declare_parameter('config_file', '').value)
        self.output_world = str(self.declare_parameter('output_world', 'tugbot_maze_world.sdf').value)
        self.get_logger().info(
            'maze_image_to_world placeholder: config_file=%s output_world=%s policy=manual_simplified_first_pass'
            % (self.config_file or '<unset>', self.output_world)
        )
        if self.config_file and not Path(self.config_file).exists():
            self.get_logger().warn('config_file does not exist yet: %s' % self.config_file)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MazeImageToWorld()
    try:
        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
