"""Thin ROS 2 node: Trémaux brain + reactive pilot autonomous maze solver.

The control loop here is authoritative; the building blocks (Trémaux brain,
reactive pilot, perception, dead-end classifier) are pure-Python and unit
tested. The node only owns ROS plumbing: subscriptions/QoS, the action client,
TF pose lookup, and a two-rate timer pair (10 Hz pilot tick, 2 Hz brain tick).

ROS plumbing mirrors the proven maze_explorer.py:
  * /map is published latched (TRANSIENT_LOCAL); a plain depth-10 subscription
    would miss the single latched sample, so we replicate _map_qos_profile().
  * OccupancyGridView is built from nav_msgs/OccupancyGrid the same way as
    maze_explorer._grid_view_from_msg (no from_msg classmethod exists).
  * yaw extraction matches maze_explorer._yaw_from_quaternion.
"""
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
import tf2_ros

from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView
from tugbot_maze import maze_perception as perception
from tugbot_maze.tremaux_solver import (
    TremauxSolver, EXPLORE, REROUTE, BACK_OUT, DONE,
    OUT_WALL, OUT_WEDGED, OUT_SUCCESS,
)
from tugbot_maze.reactive_pilot import ReactivePilot, WALL_AHEAD, WEDGED
from tugbot_maze.dead_end_classifier import is_true_dead_end


class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver')
        self.map_topic = self.declare_parameter('map_topic', '/map').value
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.action_name = self.declare_parameter('action_name', '/navigate_to_pose').value
        self.goal_events_topic = self.declare_parameter('goal_events_topic', '/maze/goal_events').value
        self.exit_x = float(self.declare_parameter('exit_x', 21.07).value)
        self.exit_y = float(self.declare_parameter('exit_y', 18.08).value)
        self.exit_radius = float(self.declare_parameter('exit_radius', 1.2).value)
        self.entrance_yaw = float(self.declare_parameter('entrance_yaw', 0.0).value)
        self.entry_direct_distance_m = float(self.declare_parameter('entry_direct_distance_m', 2.0).value)
        self.clearance_radius_m = float(self.declare_parameter('clearance_radius_m', 0.35).value)
        self.lookahead_m = float(self.declare_parameter('open_direction_lookahead_m', 1.5).value)
        self.min_open_distance_m = float(self.declare_parameter('min_open_distance_m', 0.45).value)
        self.goal_step_m = float(self.declare_parameter('branch_goal_step_m', 1.5).value)
        self.goal_timeout_sec = float(self.declare_parameter('goal_timeout_sec', 18.0).value)
        self.max_goals = int(self.declare_parameter('max_goals', 400).value)
        self.startup_delay_sec = float(self.declare_parameter('startup_delay_sec', 3.0).value)

        self.action_client = ActionClient(self, NavigateToPose, self.action_name)
        # /map is latched by slam_toolbox/map_server (TRANSIENT_LOCAL); a plain
        # depth-10 volatile subscription can miss the single latched sample and
        # never populate map_view. Match maze_explorer._map_qos_profile().
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, self._map_qos_profile())
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.goal_events_pub = self.create_publisher(String, self.goal_events_topic, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map_view: Optional[OccupancyGridView] = None
        self.scan_msg: Optional[LaserScan] = None
        self.brain = TremauxSolver(exit_xy=(self.exit_x, self.exit_y))
        self.pilot = ReactivePilot(self, self.action_client, nav_timeout_sec=self.goal_timeout_sec)
        self.goal_count = 0
        self.phase = 'startup'          # startup | entering | deciding | busy | done
        self.pending_action = None
        self.max_false_dead_end_resamples = int(self.declare_parameter('max_false_dead_end_resamples', 4).value)
        self._false_dead_end_count = 0

        self._publish_maze_boundary_map()

        self.create_timer(0.1, self._reactive_tick)   # 10 Hz pilot
        self.create_timer(0.5, self._control_tick)     # 2 Hz brain
        self.create_timer(5.0, self._diag_tick)
        self.start_time = self.get_clock().now()
        self.get_logger().info('maze_solver started (Trémaux autonomous).')

    def _publish_maze_boundary_map(self) -> None:
        """Publish a rectangular exterior boundary as a static OccupancyGrid on
        /maze_boundary_map (transient-local). The Nav2 global_costmap's
        maze_boundary_layer subscribes to this; without a publisher the costmap
        never updates and every plan aborts ("no map received"). Exterior cells
        are occupied (100), interior is unknown (-1) so SLAM's static_layer
        provides the real walls; a bottom-left entrance gap stays open.
        """
        import numpy as np
        resolution = 24.0 / 359.0
        width, height = 360, 360
        origin_x, origin_y = -0.989, -2.975
        maze_x_min, maze_x_max = 0.0, 22.0
        maze_y_min, maze_y_max = -1.5, 22.0
        entrance_x_min, entrance_x_max = -0.5, 3.0
        entrance_y_min, entrance_y_max = origin_y, -0.5

        xs = np.arange(width) * resolution + origin_x
        ys = np.arange(height) * resolution + origin_y
        XX, YY = np.meshgrid(xs, ys)
        outside = ((XX < maze_x_min) | (XX > maze_x_max) |
                   (YY < maze_y_min) | (YY > maze_y_max))
        in_entrance = ((XX >= entrance_x_min) & (XX <= entrance_x_max) &
                       (YY >= entrance_y_min) & (YY <= entrance_y_max))
        grid = np.full((height, width), -1, dtype=np.int8)
        grid[outside & ~in_entrance] = 100

        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        grid_msg.info.resolution = resolution
        grid_msg.info.width = width
        grid_msg.info.height = height
        grid_msg.info.origin.position.x = origin_x
        grid_msg.info.origin.position.y = origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = grid.flatten().tolist()

        self._boundary_map_pub = self.create_publisher(
            OccupancyGrid, '/maze_boundary_map',
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._boundary_map_pub.publish(grid_msg)
        blocked = int((grid == 100).sum())
        self.get_logger().info(
            'published maze boundary map (%dx%d, %d exterior cells blocked)'
            % (width, height, blocked))

    @staticmethod
    def _map_qos_profile() -> QoSProfile:
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    def _map_cb(self, msg):
        # OccupancyGridView has no from_msg classmethod; build it the same way
        # maze_explorer._grid_view_from_msg does.
        self.map_view = OccupancyGridView(
            info=OccupancyGridInfo(
                width=int(msg.info.width),
                height=int(msg.info.height),
                resolution=float(msg.info.resolution),
                origin_x=float(msg.info.origin.position.x),
                origin_y=float(msg.info.origin.position.y),
            ),
            data=msg.data,
        )

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

    def _forward_min_range(self):
        if self.scan_msg is None:
            return None
        s = self.scan_msg
        best = None
        for i, r in enumerate(s.ranges):
            ang = s.angle_min + i * s.angle_increment
            if abs(math.atan2(math.sin(ang), math.cos(ang))) <= math.radians(20):
                if r and r == r and r > 0.05 and (best is None or r < best):
                    best = r
        return best

    def _publish_event(self, text):
        self.goal_events_pub.publish(String(data=text))

    def _reactive_tick(self):
        if not self.pilot.is_active():
            return
        # Pass pose even if None — the pilot runs its elapsed-watchdog regardless,
        # so a TF dropout during a drive can't pin us active forever.
        self.pilot.tick(self._lookup_pose())

    def _control_tick(self):
        pose = self._lookup_pose()
        if pose is None or self.map_view is None or self.scan_msg is None:
            return
        if math.hypot(pose[0] - self.exit_x, pose[1] - self.exit_y) <= self.exit_radius:
            if self.phase != 'done':
                self.phase = 'done'
                self.get_logger().info('EXIT_REACHED (maze_solver)')
                self._publish_event('EXIT_REACHED')
                self.cmd_vel_pub.publish(Twist())   # stop the robot at the exit
            return
        if self.phase == 'done':
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if self.phase == 'startup':
            if elapsed >= self.startup_delay_sec:
                self.phase = 'entering'
            return
        if self.goal_count >= self.max_goals:
            self.get_logger().warn('FAILED_EXHAUSTED: max_goals reached')
            self._publish_event('FAILED_EXHAUSTED'); self.phase = 'done'; return

        if self.pilot.is_active():
            return

        if self.phase == 'entering':
            if self.pilot.result is None:
                self.pilot.reactive_drive(self.entrance_yaw, self.entry_direct_distance_m)
                return
            self.pilot.result = None
            self.phase = 'deciding'
            return

        if self.pilot.result is not None:
            self._handle_result()
            return

        if self.phase == 'deciding':
            self._decide(pose)

    def _decide(self, pose):
        local = perception.classify_local_topology(
            self.map_view, pose, lookahead_m=self.lookahead_m,
            clearance_radius_m=self.clearance_radius_m,
            min_open_distance_m=self.min_open_distance_m,
        )
        if local.kind == perception.DEAD_END and not is_true_dead_end(
                local.kind, self._forward_min_range()):
            # Map says DEAD_END (corridor ahead still UNKNOWN in SLAM) but the live
            # scan reads open ahead. Re-sample a few times in case the map fills in;
            # if it persists, the robot is frozen in a corridor — nudge forward to
            # force SLAM to map the ahead-region and break the tie. The scan has
            # already confirmed >= ~1.36 m clearance, so a short reactive push is safe.
            self._false_dead_end_count += 1
            if self._false_dead_end_count < self.max_false_dead_end_resamples:
                return
            self.get_logger().warn(
                'false DEAD_END x%d (map unknown ahead, scan open) — nudging forward'
                % self._false_dead_end_count)
            self._false_dead_end_count = 0
            self.phase = 'busy'
            self.pilot.reactive_drive(pose[2], 0.6)
            return
        self._false_dead_end_count = 0   # reset on any real decision
        action = self.brain.update((pose[0], pose[1]), pose[2], local)
        self.goal_count += 1
        self.pending_action = action
        self._publish_event('explore goal #%d kind=%s' % (self.goal_count, action.kind))
        self.get_logger().info('explore goal #%d kind=%s reason=%s'
                               % (self.goal_count, action.kind, action.reason))
        if action.kind == DONE:
            self.get_logger().warn('FAILED_EXHAUSTED: brain reports full coverage w/o exit')
            self._publish_event('FAILED_EXHAUSTED'); self.phase = 'done'; return
        self.phase = 'busy'
        if action.kind == BACK_OUT:
            # Fixed reverse with laser safety; intentionally ignores action.target_xy
            # (the pilot reverses a set distance rather than planning to prev_xy).
            self.pilot.back_out(self.goal_step_m, pose)
        elif action.kind == REROUTE:
            self.pilot.follow_path(action.path_xy[1:])   # skip the current node
        else:  # EXPLORE
            self.pilot.drive_to(action.target_xy)

    def _handle_result(self):
        res = self.pilot.result
        self.pilot.result = None
        act = self.pending_action
        self.pending_action = None
        if act is not None and act.kind == EXPLORE:
            if res == WALL_AHEAD:
                self.brain.report_outcome(OUT_WALL)
            elif res == WEDGED:
                self.brain.report_outcome(OUT_WEDGED)
            else:  # SUCCESS: branch driven to target → mark explored so brain advances
                self.brain.report_outcome(OUT_SUCCESS)
        self.phase = 'deciding'


    def _diag_tick(self):
        pose = self._lookup_pose()
        if pose is None:
            return
        dist = math.hypot(pose[0] - self.exit_x, pose[1] - self.exit_y)
        self.get_logger().info(
            'DIAG pose=(%.2f, %.2f) yaw=%.2f dist_to_exit=%.2f goal=%d phase=%s'
            % (pose[0], pose[1], pose[2], dist, self.goal_count, self.phase))


def main(args=None):
    rclpy.init(args=args)
    node = MazeSolver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
