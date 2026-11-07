"""The validated maze solver with timing adapted to the Go2 gait."""
import math

import rclpy
from tugbot_maze.flood_fill_solver import FloodFillSolver
from tugbot_maze.footprint import front_gap
from tugbot_maze.go2_exit_approach import exit_approach_command


def configure_go2_motion(motion):
    motion.center_timeout_s = 12.0
    motion.align_timeout_s = 8.0
    motion.turn_timeout_s = 15.0
    motion.backout_v = 0.18
    motion.backout_timeout_s = 25.0
    motion.settle_s = 0.7
    motion.no_progress_s = 180.0
    motion.no_progress_fast_s = 60.0
    motion._no_progress_win = motion.no_progress_s


class Go2MazeSolver(FloodFillSolver):
    def __init__(self):
        super().__init__()
        configure_go2_motion(self.motion)
        self._exit_started = None
        self._exit_settled_since = None

    def _maze_completed(self):
        # The inherited discrete-cell tolerance can stop outside the formal
        # exit circle. Do not publish EXIT_REACHED until the fine approach.
        self.phase = 'exit_approach'
        self._exit_started = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info('EXIT_CELL_REACHED: starting low-speed final approach')

    def _control_tick(self):
        if self.phase not in ('exit_approach', 'exit_failed'):
            return super()._control_tick()
        now = self.get_clock().now().nanoseconds / 1e9
        pose = self._lookup_pose()
        self._publish_map_to_odom()
        if self.phase == 'exit_failed' or now-self._exit_started > 30.0:
            if self.phase != 'exit_failed':
                self.get_logger().error('EXIT_APPROACH_FAILED: timeout, holding stand')
            self.phase = 'exit_failed'
            self._publish_cmd(0.0, 0.0)
            return
        clearance = math.nan
        if self.scan_msg is not None:
            scan = self.scan_msg
            stamp = scan.header.stamp.sec + scan.header.stamp.nanosec/1e9
            if 0.0 <= now-stamp <= 0.5:
                clearance = front_gap(scan.ranges, scan.angle_min, scan.angle_increment)
        if pose is None:
            self._publish_cmd(0.0, 0.0)
            self._exit_settled_since = None
            return
        v, w, reached = exit_approach_command(
            pose, (self.exit_x, self.exit_y), self.exit_radius-0.35, clearance)
        self._publish_cmd(v, w)
        if not reached:
            self._exit_settled_since = None
        elif self._exit_settled_since is None:
            self._exit_settled_since = now
        elif now-self._exit_settled_since >= 2.0:
            super()._maze_completed()


def main(args=None):
    rclpy.init(args=args)
    node = Go2MazeSolver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
