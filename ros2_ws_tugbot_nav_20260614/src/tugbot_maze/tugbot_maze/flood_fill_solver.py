"""Thin ROS 2 node: cell-grid flood-fill (micromouse) maze solver.

Subscribes /scan (LaserScan) and uses map->base_link TF for pose. Each control tick:
localize to a maze cell, sense its 4 edges from the live LIDAR (cell_walls), ask the
FloodFillBrain for the next cell, and drive there with hop_command. Owns the entry
drive, a per-hop stall watchdog, and the exit self-check. No LIDAR entrance seal is
needed here: the brain only ever targets interior cells (cx 1..10), so the robot is
confined to the interior and cannot reach the entrance opening or the exterior (a
stronger guarantee than the wall-follower's seal). ROS plumbing mirrors
wall_follow_solver.py.
"""
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf2_ros

from tugbot_maze.flood_fill_brain import (
    FloodFillBrain, ENTRANCE_CELL, EXIT_CELL, DIRS, in_grid, cell_center, pose_to_cell)
from tugbot_maze.cell_walls import sense_cell_walls
from tugbot_maze.hop_controller import hop_command
from tugbot_maze.pose_tracking import compose_2d, quat_to_yaw
from tugbot_maze.wall_follow_control import exit_reached, entering_done


class FloodFillSolver(Node):
    def __init__(self):
        super().__init__('flood_fill_solver')
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        # pose_source: 'slam' (live map->base_link) | 'odom_locked' (freeze map->odom
        # once at startup, then track on wheel odometry only -- avoids slam_toolbox
        # pose degradation in the narrow repetitive corridors).
        self.pose_source = self.declare_parameter('pose_source', 'slam').value
        self.goal_events_topic = self.declare_parameter('goal_events_topic', '/maze/goal_events').value
        self.exit_x = float(self.declare_parameter('exit_x', 21.072562).value)
        self.exit_y = float(self.declare_parameter('exit_y', 18.083566).value)
        self.exit_radius = float(self.declare_parameter('exit_radius', 1.2).value)
        self.entry_direct_distance_m = float(self.declare_parameter('entry_direct_distance_m', 2.0).value)
        self.startup_delay_sec = float(self.declare_parameter('startup_delay_sec', 3.0).value)
        self.hop_timeout_s = float(self.declare_parameter('hop_timeout_s', 25.0).value)
        self.backup_s = float(self.declare_parameter('backup_s', 1.0).value)
        self.backup_v = float(self.declare_parameter('backup_v', -0.15).value)
        self.cruise_v = float(self.declare_parameter('cruise_v', 0.3).value)

        self.brain = FloodFillBrain(exit_cell=EXIT_CELL)
        self.scan_msg: Optional[LaserScan] = None
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.goal_events_pub = self.create_publisher(String, self.goal_events_topic, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.phase = 'startup'        # startup | entering | hop | backup | done
        self.start_xy = None
        self.map_to_odom_locked = None   # frozen map->odom for pose_source='odom_locked'
        self.target_xy = None
        self.hop_deadline = 0.0
        self.backup_until = 0.0
        self.create_timer(0.1, self._control_tick)
        self.create_timer(5.0, self._diag_tick)
        self.start_time = self.get_clock().now()
        self.get_logger().info('flood_fill_solver started (pose_source=%s).' % self.pose_source)

    def _scan_cb(self, msg):
        self.scan_msg = msg

    def _lookup_tf(self, parent, child):
        try:
            t = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                quat_to_yaw(q.x, q.y, q.z, q.w))

    def _lookup_pose(self):
        if self.pose_source != 'odom_locked':
            return self._lookup_tf(self.map_frame, self.base_frame)
        # odom_locked: pose = (frozen map->odom) o (live odom->base_link).
        odom_base = self._lookup_tf(self.odom_frame, self.base_frame)
        if odom_base is None:
            return None
        if self.map_to_odom_locked is None:
            # During startup, let SLAM settle; report its live pose but don't freeze yet.
            if self.phase == 'startup':
                return self._lookup_tf(self.map_frame, self.base_frame)
            mo = self._lookup_tf(self.map_frame, self.odom_frame)
            if mo is None:                                  # SLAM not up yet -> fall back
                return self._lookup_tf(self.map_frame, self.base_frame)
            self.map_to_odom_locked = mo
            self.get_logger().info(
                'POSE_SOURCE=odom_locked: froze map->odom=(%.3f, %.3f, %.3f rad); '
                'tracking on wheel odometry only from here.' % mo)
        return compose_2d(self.map_to_odom_locked, odom_base)

    def _publish_cmd(self, v, w):
        m = Twist(); m.linear.x = float(v); m.angular.z = float(w); self.cmd_vel_pub.publish(m)

    def _sense(self, cur, yaw):
        if self.scan_msg is None:
            return
        s = self.scan_msg
        walls = sense_cell_walls(s.ranges, s.angle_min, s.angle_increment, yaw)
        for d, is_wall in walls.items():
            self.brain.mark(cur, d, is_wall)

    def _control_tick(self):
        now = self.get_clock().now(); t = now.nanoseconds / 1e9
        pose = self._lookup_pose()
        if pose is not None and exit_reached(pose, (self.exit_x, self.exit_y), self.exit_radius):
            if self.phase != 'done':
                self.phase = 'done'
                self.get_logger().info('EXIT_REACHED (flood_fill_solver)')
                self.goal_events_pub.publish(String(data='EXIT_REACHED'))
                self._publish_cmd(0.0, 0.0)
            return
        if self.phase == 'done':
            return
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if self.phase == 'startup':
            if elapsed >= self.startup_delay_sec and pose is not None:
                self.start_xy = (pose[0], pose[1]); self.phase = 'entering'
            return
        if pose is None:
            return
        if self.phase == 'entering':
            if entering_done(self.start_xy, (pose[0], pose[1]), self.entry_direct_distance_m):
                self.phase = 'hop'; self.target_xy = None
            else:
                self._publish_cmd(self.cruise_v, 0.0)
            return
        if self.phase == 'backup':
            if t >= self.backup_until:
                self.phase = 'hop'; self.target_xy = None
            else:
                self._publish_cmd(self.backup_v, 0.0)
            return
        # phase == 'hop'
        cur = pose_to_cell(pose[0], pose[1])
        # Clear the target once arrived, so we re-sense + choose the next cell and
        # drive toward it in THIS same tick (continuous motion, no stale-command coast).
        if self.target_xy is not None and hop_command(pose, self.target_xy)[2]:
            self.target_xy = None
        if self.target_xy is None:
            self._sense(cur, pose[2])
            nxt = self.brain.next_cell(cur)
            if nxt is None:                             # connected maze -> only at exit (caught above)
                self._publish_cmd(0.0, 0.0); return
            self.brain.mark_traversal(cur, nxt)         # Trémaux: count this edge traversal
            self.target_xy = cell_center(nxt)
            self.hop_deadline = t + self.hop_timeout_s
        if t >= self.hop_deadline:                      # wedged -> back up and retry
            self.phase = 'backup'; self.backup_until = t + self.backup_s
            self.goal_events_pub.publish(String(data='HOP_BACKUP')); return
        v, w, _ = hop_command(pose, self.target_xy)
        self._publish_cmd(v, w)

    def _diag_tick(self):
        pose = self._lookup_pose()
        if pose is None:
            return
        cur = pose_to_cell(pose[0], pose[1])
        dist = math.hypot(pose[0] - self.exit_x, pose[1] - self.exit_y)
        self.get_logger().info('DIAG pose=(%.2f, %.2f) cell=%s dist_to_exit=%.2f phase=%s'
                               % (pose[0], pose[1], cur, dist, self.phase))


def main(args=None):
    rclpy.init(args=args)
    node = FloodFillSolver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
