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
    FloodFillBrain, ENTRANCE_CELL, EXIT_CELL, DIRS, CELL_SIZE_M, in_grid, cell_center, pose_to_cell)
from tugbot_maze.cell_walls import sense_cell_walls, cell_wall_min_ranges
from tugbot_maze.hop_controller import hop_command
from tugbot_maze.pose_tracking import compose_2d, quat_to_yaw
from tugbot_maze.wall_follow_control import exit_reached, entering_done
from tugbot_maze.wall_localize import cell_center_offset, heading_snap


def _dir_name(d):
    return {(1, 0): 'E', (-1, 0): 'W', (0, 1): 'N', (0, -1): 'S'}[d]


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
        # sense_debug: log per-edge sensed walls + min LIDAR ranges per cell (diagnostics).
        self.sense_debug = bool(self.declare_parameter('sense_debug', False).value)
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
        # Sense a cell only when centered within this radius and settled (clean geometry).
        self.center_tol_m = float(self.declare_parameter('center_tol_m', 0.12).value)
        self.settle_s = float(self.declare_parameter('settle_s', 0.4).value)

        self.yaw_tol_rad = float(self.declare_parameter('yaw_tol_rad', 0.10).value)

        self.brain = FloodFillBrain(exit_cell=EXIT_CELL)
        self.scan_msg: Optional[LaserScan] = None
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.goal_events_pub = self.create_publisher(String, self.goal_events_topic, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.phase = 'startup'        # startup | entering | hop | center | backup | done
        self.start_xy = None
        self.map_to_odom_locked = None   # frozen map->odom for pose_source='odom_locked'
        self.target_xy = None
        self.hop_deadline = 0.0
        self.backup_until = 0.0
        self.sensed = set()              # cells sensed once already (sticky map; static maze)
        self.settle_until = 0.0
        self._startup_scan_dumped = False   # one-time full-scan dump for footprint characterization
        self.cell = ENTRANCE_CELL        # discrete cell tracker (drift-immune)
        self.hop_target = None           # target cell for current hop
        self.hop_dir = None              # direction tuple for current hop
        self.hop_start = None            # pose xy at start of current hop
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

    def _sense(self, cur, pose):
        if self.scan_msg is None:
            return
        s = self.scan_msg
        yaw = pose[2]
        walls = sense_cell_walls(s.ranges, s.angle_min, s.angle_increment, yaw)
        for d, is_wall in walls.items():
            self.brain.mark(cur, d, is_wall)
        if self.sense_debug:
            r = cell_wall_min_ranges(s.ranges, s.angle_min, s.angle_increment, yaw)
            self.get_logger().info(
                'SENSE cell=%s pose=(%.2f,%.2f) yaw=%.2f walls=N%dE%dS%dW%d '
                'minr=N%.2f E%.2f S%.2f W%.2f'
                % (cur, pose[0], pose[1], yaw,
                   walls['N'], walls['E'], walls['S'], walls['W'],
                   r['N'], r['E'], r['S'], r['W']))
            # Raw downsampled scan for offline real-vs-SDF comparison (drift vs lidar-gap).
            step = max(1, len(s.ranges) // 90)
            ds = ['%.2f' % min(float(v), 12.0) if (v is not None and math.isfinite(v) and v > 0) else 'inf'
                  for v in s.ranges[::step]]
            self.get_logger().info(
                'SCANDUMP cell=%s pose=(%.3f,%.3f) yaw=%.4f amin=%.4f ainc=%.6f step=%d ranges=%s'
                % (cur, pose[0], pose[1], yaw, s.angle_min, s.angle_increment, step, ' '.join(ds)))

    def _control_tick(self):
        now = self.get_clock().now(); t = now.nanoseconds / 1e9
        pose = self._lookup_pose()
        if self.phase not in ('startup',) and self.cell == EXIT_CELL:
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
                self.cell = ENTRANCE_CELL
                self.phase = 'center'
                self.settle_until = t + self.settle_s
                self.hop_deadline = t + self.hop_timeout_s
            else:
                self._publish_cmd(self.cruise_v, 0.0)
            return
        if self.phase == 'backup':
            if t >= self.backup_until:
                self.phase = 'center'
                self.settle_until = t + self.settle_s
                self.hop_deadline = t + self.hop_timeout_s
            else:
                self._publish_cmd(self.backup_v, 0.0)
            return
        # phase == 'center': wall-referenced centering — use LIDAR offsets to find the true
        # cell centre, snap yaw to cardinal, then sense and plan next hop.  Drift-immune:
        # uses relative LIDAR displacement, not absolute odom.
        if self.phase == 'center':
            if self.scan_msg is None:
                return
            s = self.scan_msg
            ox, oy = cell_center_offset(s.ranges, s.angle_min, s.angle_increment, pose[2])
            snap_yaw, dyaw = heading_snap(pose[2])
            if abs(dyaw) > self.yaw_tol_rad and t < self.hop_deadline:   # rotate to cardinal first
                self._publish_cmd(0.0, max(-0.5, min(0.5, 1.5 * dyaw)))
                self.settle_until = t + self.settle_s
                return
            need = (ox is not None and abs(ox) > self.center_tol_m) or \
                   (oy is not None and abs(oy) > self.center_tol_m)
            if need and t < self.hop_deadline:                          # relative move to centre
                tx = pose[0] - (ox or 0.0)
                ty = pose[1] - (oy or 0.0)
                v, w, _ = hop_command(pose, (tx, ty), arrive_m=0.06)
                self._publish_cmd(v, w)
                self.settle_until = t + self.settle_s
                return
            self._publish_cmd(0.0, 0.0)
            if t < self.settle_until:
                return
            if self.cell not in self.sensed:                            # sense the cell ONCE (sticky)
                self._sense(self.cell, pose)
                self.sensed.add(self.cell)
            nxt = self.brain.next_cell(self.cell)
            if nxt is None:
                self._publish_cmd(0.0, 0.0); return
            self.brain.mark_traversal(self.cell, nxt)
            self.hop_target = nxt
            self.hop_dir = (nxt[0] - self.cell[0], nxt[1] - self.cell[1])
            self.hop_start = (pose[0], pose[1])
            self.hop_deadline = t + self.hop_timeout_s
            self.phase = 'hop'
            return
        # phase == 'hop': drive one cell in hop_dir using dead-reckoning distance from
        # hop_start.  Drift-immune: arrival decided by distance moved, not absolute pose.
        if self.phase == 'hop':
            target_yaw = math.atan2(self.hop_dir[1], self.hop_dir[0])
            dyaw = math.atan2(math.sin(target_yaw - pose[2]), math.cos(target_yaw - pose[2]))
            if abs(dyaw) > self.yaw_tol_rad:                            # face the hop cardinal
                self._publish_cmd(0.0, max(-0.5, min(0.5, 1.5 * dyaw))); return
            moved = math.hypot(pose[0] - self.hop_start[0], pose[1] - self.hop_start[1])
            if moved >= CELL_SIZE_M - 0.10:                             # arrived in next cell
                self.cell = self.hop_target                             # DISCRETE advance
                self.phase = 'center'
                self.settle_until = t + self.settle_s
                self.hop_deadline = t + self.hop_timeout_s
                return
            if t >= self.hop_deadline:                                  # blocked -> mark wall, back up
                self.brain.mark(self.cell, _dir_name(self.hop_dir), is_wall=True)
                self.phase = 'backup'; self.backup_until = t + self.backup_s
                self.goal_events_pub.publish(String(data='HOP_BACKUP')); return
            self._publish_cmd(self.cruise_v, 0.0)
            return

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
