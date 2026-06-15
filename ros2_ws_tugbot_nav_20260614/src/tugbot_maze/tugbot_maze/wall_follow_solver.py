"""Thin ROS 2 node: reactive wall-following autonomous maze solver.

All control is delegated to the ROS-free `WallFollower` policy; this node owns
only ROS plumbing (scan subscription, TF pose, a 10 Hz control timer, a 5 Hz DIAG
timer) plus entry, stall recovery, and the exit self-check. The control loop needs
only /scan; SLAM (map->base_link TF) is used solely for the exit distance check.

ROS plumbing mirrors maze_solver.py: /cmd_vel_nav publisher, goal_events String
publisher ('EXIT_REACHED'), and the map->base_link yaw extraction.
"""
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf2_ros

from tugbot_maze.wall_follower import WallFollower, State, sectorize
from tugbot_maze.wall_follow_control import exit_reached, entering_done, StallWatchdog


class WallFollowSolver(Node):
    def __init__(self):
        super().__init__('wall_follow_solver')
        # --- ROS / frames ---
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.goal_events_topic = self.declare_parameter('goal_events_topic', '/maze/goal_events').value
        # --- mission geometry ---
        self.exit_x = float(self.declare_parameter('exit_x', 21.072562).value)
        self.exit_y = float(self.declare_parameter('exit_y', 18.083566).value)
        self.exit_radius = float(self.declare_parameter('exit_radius', 1.2).value)
        self.entry_direct_distance_m = float(self.declare_parameter('entry_direct_distance_m', 2.0).value)
        self.startup_delay_sec = float(self.declare_parameter('startup_delay_sec', 3.0).value)
        # --- wall-follower tuning (defaults == robust WallFollower defaults from the Task 5 proof) ---
        self.follow_side = self.declare_parameter('follow_side', 'left').value
        target_wall_m = float(self.declare_parameter('target_wall_m', 0.6).value)
        front_block_m = float(self.declare_parameter('front_block_m', 0.7).value)
        wall_lost_m = float(self.declare_parameter('wall_lost_m', 1.2).value)
        engage_m = float(self.declare_parameter('engage_m', 1.0).value)
        cruise_v = float(self.declare_parameter('cruise_v', 0.3).value)
        corner_v = float(self.declare_parameter('corner_v', 0.45).value)
        corner_w = float(self.declare_parameter('corner_w', 0.6).value)
        turn_w = float(self.declare_parameter('turn_w', 1.0).value)
        w_max = float(self.declare_parameter('w_max', 1.0).value)
        kp = float(self.declare_parameter('kp', 1.5).value)
        kd = float(self.declare_parameter('kd', 0.4).value)
        self.max_range_m = float(self.declare_parameter('max_range_m', 12.0).value)
        # --- stall watchdog ---
        stall_s = float(self.declare_parameter('stall_s', 4.0).value)
        progress_eps_m = float(self.declare_parameter('progress_eps_m', 0.2).value)
        self.backup_s = float(self.declare_parameter('backup_s', 1.0).value)
        self.backup_v = float(self.declare_parameter('backup_v', -0.15).value)
        self.cruise_v = cruise_v

        self.follower = WallFollower(
            target_wall_m=target_wall_m, front_block_m=front_block_m,
            wall_lost_m=wall_lost_m, engage_m=engage_m, cruise_v=cruise_v,
            corner_v=corner_v, corner_w=corner_w, turn_w=turn_w, w_max=w_max,
            kp=kp, kd=kd, follow_side=self.follow_side)
        self.watchdog = StallWatchdog(stall_s=stall_s, progress_eps_m=progress_eps_m)

        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.goal_events_pub = self.create_publisher(String, self.goal_events_topic, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.scan_msg: Optional[LaserScan] = None
        self.phase = 'startup'      # startup | entering | follow | backup | done
        self.start_xy = None
        self.backup_until = 0.0

        self.create_timer(0.1, self._control_tick)   # 10 Hz control
        self.create_timer(5.0, self._diag_tick)
        self.start_time = self.get_clock().now()
        self.get_logger().info('wall_follow_solver started (follow_side=%s).' % self.follow_side)

    # --- ROS callbacks / helpers (mirror maze_solver.py) ---

    def _scan_cb(self, msg):
        self.scan_msg = msg

    def _lookup_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def _publish_event(self, text):
        self.goal_events_pub.publish(String(data=text))

    def _publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_vel_pub.publish(msg)

    def _sectors(self):
        s = self.scan_msg
        return sectorize(s.ranges, s.angle_min, s.angle_increment, self.follow_side,
                         max_range=self.max_range_m)

    # --- control loop ---

    def _control_tick(self):
        now = self.get_clock().now()
        t = now.nanoseconds / 1e9
        pose = self._lookup_pose()

        # Exit self-check (needs pose; control loop otherwise needs only /scan).
        if pose is not None and exit_reached(pose, (self.exit_x, self.exit_y), self.exit_radius):
            if self.phase != 'done':
                self.phase = 'done'
                self.get_logger().info('EXIT_REACHED (wall_follow_solver)')
                self._publish_event('EXIT_REACHED')
                self._publish_cmd(0.0, 0.0)
            return
        if self.phase == 'done':
            return

        elapsed = (now - self.start_time).nanoseconds / 1e9
        if self.phase == 'startup':
            if elapsed >= self.startup_delay_sec and pose is not None:
                self.start_xy = (pose[0], pose[1])
                self.phase = 'entering'
                self.get_logger().info('ENTRY_DIRECT: driving %.2f m inward' % self.entry_direct_distance_m)
            return

        if self.phase == 'entering':
            # Drive straight in until ~entry_direct_distance_m, but hand off early if
            # a wall looms ahead (don't ram it — let the follower turn away). This
            # tolerates a spawn heading that is not perfectly inward.
            done = pose is not None and entering_done(
                self.start_xy, (pose[0], pose[1]), self.entry_direct_distance_m)
            blocked = self.scan_msg is not None and \
                self._sectors().front < self.follower.front_block_m
            if done or blocked:
                self.phase = 'follow'
                self.follower.state = State.FIND_WALL
                if pose is not None:
                    self.watchdog.reset(t, pose[0], pose[1])
                self.get_logger().info('engaging wall-follower (entered=%s blocked=%s)'
                                       % (done, blocked))
            else:
                self._publish_cmd(self.cruise_v, 0.0)   # straight in
            return

        # follow / backup need a scan
        if self.scan_msg is None:
            return

        if self.phase == 'backup':
            if t >= self.backup_until:
                self.phase = 'follow'
                self.follower.state = State.FIND_WALL
                if pose is not None:
                    self.watchdog.reset(t, pose[0], pose[1])
            else:
                self._publish_cmd(self.backup_v, 0.0)
            return

        # phase == 'follow'
        if pose is not None and self.watchdog.update(t, pose[0], pose[1]):
            self.phase = 'backup'
            self.backup_until = t + self.backup_s
            self._publish_event('STALL_BACKUP')
            self.get_logger().warn('stall detected — backing up %.1f s' % self.backup_s)
            self._publish_cmd(self.backup_v, 0.0)
            return

        cmd = self.follower.update(self._sectors())
        self._publish_cmd(cmd.v, cmd.w)

    def _diag_tick(self):
        pose = self._lookup_pose()
        if pose is None:
            return
        dist = math.hypot(pose[0] - self.exit_x, pose[1] - self.exit_y)
        self.get_logger().info(
            'DIAG pose=(%.2f, %.2f) yaw=%.2f dist_to_exit=%.2f phase=%s state=%s'
            % (pose[0], pose[1], pose[2], dist, self.phase, self.follower.state.value))


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowSolver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
