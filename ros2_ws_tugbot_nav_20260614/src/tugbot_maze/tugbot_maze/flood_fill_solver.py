"""Thin ROS 2 node: cell-grid flood-fill (micromouse) maze solver.

Subscribes /scan (LaserScan) and uses map->base_link TF for pose. Delegates all
motion control (centering, turning, driving cell-to-cell) to MazeMotion, which
internally uses FloodFillBrain for routing. This node owns only ROS plumbing
(params, TF, pub/sub, timers) and the startup / entering phases.
"""
import math
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf2_ros

from tugbot_maze.flood_fill_brain import (
    FloodFillBrain, ENTRANCE_CELL, EXIT_CELL, pose_to_cell)
from tugbot_maze.map_memory import MapMemory
from tugbot_maze.maze_motion import MazeMotion
from tugbot_maze.junction_log import JunctionLog, update_junctions
from tugbot_maze.pose_tracking import compose_2d, quat_to_yaw
from tugbot_maze.wall_follow_control import entering_done


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
        # sense_debug: dump full LIDAR scan at startup for footprint characterization.
        self.sense_debug = bool(self.declare_parameter('sense_debug', False).value)
        self.goal_events_topic = self.declare_parameter('goal_events_topic', '/maze/goal_events').value
        self.exit_x = float(self.declare_parameter('exit_x', 21.072562).value)
        self.exit_y = float(self.declare_parameter('exit_y', 18.083566).value)
        self.exit_radius = float(self.declare_parameter('exit_radius', 1.2).value)
        self.entry_direct_distance_m = float(self.declare_parameter('entry_direct_distance_m', 2.0).value)
        self.startup_delay_sec = float(self.declare_parameter('startup_delay_sec', 3.0).value)
        self.hop_timeout_s = float(self.declare_parameter('hop_timeout_s', 25.0).value)
        self.cruise_v = float(self.declare_parameter('cruise_v', 0.3).value)
        # Sense a cell only when centered within this radius and settled (clean geometry).
        self.center_tol_m = float(self.declare_parameter('center_tol_m', 0.12).value)
        self.yaw_tol_rad = float(self.declare_parameter('yaw_tol_rad', 0.10).value)
        # A hop counts as arrived once dead-reckoned distance reaches CELL_SIZE - slack.
        self.hop_arrive_slack_m = float(self.declare_parameter('hop_arrive_slack_m', 0.05).value)
        self.front_block_m = float(self.declare_parameter('front_block_m', 0.7).value)
        self.junction_log_dir = str(self.declare_parameter('junction_log_dir', '').value)
        self.junction_log_path = os.path.join(self.junction_log_dir or 'log', 'junctions.json')

        self.brain = FloodFillBrain(exit_cell=EXIT_CELL)
        self.mem = MapMemory(self.brain)
        self.motion = MazeMotion(self.brain, cruise_v=self.cruise_v,
                                 center_tol_m=self.center_tol_m,
                                 yaw_tol_rad=self.yaw_tol_rad,
                                 hop_arrive_slack_m=self.hop_arrive_slack_m,
                                 front_block_m=self.front_block_m,
                                 hop_timeout_s=self.hop_timeout_s,
                                 mem=self.mem)
        self.junctions = JunctionLog()
        self._prev_motion_cell = None
        self.scan_msg: Optional[LaserScan] = None
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.goal_events_pub = self.create_publisher(String, self.goal_events_topic, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.phase = 'startup'        # startup | entering | driving | done
        self.start_xy = None
        self.map_to_odom_locked = None   # frozen map->odom for pose_source='odom_locked'
        self._startup_scan_dumped = False   # one-time full-scan dump for footprint characterization
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

    def _flush_junctions(self):
        try:
            self.junctions.flush(self.junction_log_path)
        except Exception as e:                       # never let artifact IO kill the solver
            self.get_logger().warning('junction log flush failed: %r' % e)

    def _control_tick(self):
        now = self.get_clock().now(); t = now.nanoseconds / 1e9
        pose = self._lookup_pose()
        if self.phase == 'done':
            return
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if self.phase == 'startup':
            if self.sense_debug and self.scan_msg is not None and not self._startup_scan_dumped:
                s = self.scan_msg
                full = ['%.3f' % min(float(v), 99.0) if (v is not None and math.isfinite(v) and v > 0)
                        else 'inf' for v in s.ranges]
                self.get_logger().info(
                    'FULLSCAN pose=%s amin=%.5f ainc=%.6f n=%d ranges=%s'
                    % (pose, s.angle_min, s.angle_increment, len(s.ranges), ' '.join(full)))
                self._startup_scan_dumped = True
            if elapsed >= self.startup_delay_sec and pose is not None:
                self.start_xy = (pose[0], pose[1]); self.phase = 'entering'
            return
        if pose is None:
            return
        if self.phase == 'entering':
            if entering_done(self.start_xy, (pose[0], pose[1]), self.entry_direct_distance_m):
                self.motion.cell = ENTRANCE_CELL
                self.phase = 'driving'
            else:
                self._publish_cmd(self.cruise_v, 0.0)
            return
        if self.phase == 'driving':
            if self.scan_msg is None:
                return
            s = self.scan_msg
            v, w, done = self.motion.step(pose, (s.ranges, s.angle_min, s.angle_increment), t)
            self._publish_cmd(v, w)
            if self.motion.events:                            # DIAG: drain structured stall/escape events
                for ev in self.motion.events:
                    self.get_logger().info(ev)
                self.motion.events.clear()
            self._prev_motion_cell, j = update_junctions(
                self.junctions, self.brain, self.motion.cell, self._prev_motion_cell,
                self.motion.sensed, t)
            if j is not None:
                self.get_logger().info('JUNCTION cell=%s exits=%s order=%d visits=%d'
                                       % (tuple(j['cell']), j['exits'],
                                          j['discovery_index'], j['visits']))
            if done and self.phase != 'done':
                self.phase = 'done'
                self.get_logger().info('EXIT_REACHED (flood_fill_solver)')
                self.goal_events_pub.publish(String(data='EXIT_REACHED'))
                self._flush_junctions()
            return

    def _diag_tick(self):
        self._flush_junctions()
        pose = self._lookup_pose()
        if pose is None:
            return
        odom_cell = pose_to_cell(pose[0], pose[1])
        dist = math.hypot(pose[0] - self.exit_x, pose[1] - self.exit_y)
        self.get_logger().info('DIAG pose=(%.2f, %.2f, %.2f) dcell=%s odomcell=%s dist_to_exit=%.2f phase=%s/%s mem_supp=%d mem_rec=%d'
                               % (pose[0], pose[1], pose[2], self.motion.cell, odom_cell, dist,
                                  self.phase, self.motion.phase,
                                  self.motion.mem.suppressed, self.motion.mem.reconciles))
        if self.sense_debug and self.motion.dbg:
            d = self.motion.dbg
            self.get_logger().info('SENSE cell=%s pose=%s off=%s good=%s committed=%s corrob=%s walls=%s'
                                   % (d.get('cell'), d.get('pose'), d.get('off'), d.get('good'),
                                      d.get('committed'), d.get('corrob'), d.get('walls')))


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
