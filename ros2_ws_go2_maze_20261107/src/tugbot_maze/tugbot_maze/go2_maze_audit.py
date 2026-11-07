"""Independent evaluation of native Gazebo poses; never publishes control commands."""
import csv
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RobotPath
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from tugbot_maze.maze_sim import MazeSim, load_segments, outer_boundary_box
from tugbot_maze.pose_tracking import quat_to_yaw
from tugbot_maze.footprint import FOOTPRINT_MAX_TILT_RAD

ENTRANCE_OFFSET = (11.011281, 9.025070)
EXIT = (21.072562, 18.083566)
EXIT_RADIUS = 1.2


class Go2MazeAudit(Node):
    def __init__(self):
        super().__init__('go2_maze_audit')
        self.run_dir = Path(self.declare_parameter('run_dir', '/tmp/go2-maze-run').value)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.oracle = MazeSim(load_segments(), (0.0, 0.0), 0.0)
        self.bounds = outer_boundary_box()
        self.odom = None
        self.previous = None
        self.first_t = None
        self.stopped_since = None
        self.solver_exit = False
        self.last_path_t = -math.inf
        self.path = RobotPath()
        self.path.header.frame_id = 'map'
        self.path_pub = self.create_publisher(RobotPath, '/maze/go2_actual_path', 1)
        self.metrics = dict(
            success=False, samples=0, swept_samples=0, collision_samples=0,
            exterior_samples=0, pose_pair_samples=0, max_pose_pair_error=0.0,
            min_height=1e9, max_tilt_deg=0.0, route_length_m=0.0,
            elapsed_sim_s=0.0, exit_distance_m=1e9, solver_exit=False,
        )
        self.csv_file = (self.run_dir / 'ground_truth.csv').open('w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['sim_time', 'map_x', 'map_y', 'yaw', 'height', 'tilt', 'collision'])
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(TFMessage, '/gt/dynamic_pose', self.on_truth, 20)
        self.create_subscription(String, '/maze/goal_events', self.on_event, 10)
        self.create_timer(1.0, self.write_status)

    def on_odom(self, message):
        self.odom = message

    def on_event(self, message):
        if message.data == 'EXIT_REACHED':
            self.solver_exit = True

    def on_truth(self, message):
        if not message.transforms or self.odom is None:
            return
        # SceneBroadcaster has only one dynamic top-level model in this world.
        # The bridge drops its names and stamps. Validate its first transform
        # against the separately published odometry before trusting the audit.
        truth = next((tr for tr in message.transforms if tr.child_frame_id == 'go2'),
                     message.transforms[0])
        position = truth.transform.translation
        orientation = truth.transform.rotation
        actual = self.odom.pose.pose.position
        pair_error = math.hypot(position.x - actual.x, position.y - actual.y)
        self.metrics['pose_pair_samples'] += 1
        self.metrics['max_pose_pair_error'] = max(self.metrics['max_pose_pair_error'], pair_error)
        now = self.get_clock().now().nanoseconds / 1e9
        if self.first_t is None:
            self.first_t = now
        x = position.x + ENTRANCE_OFFSET[0]
        y = position.y + ENTRANCE_OFFSET[1]
        yaw = quat_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        tilt = math.acos(max(-1.0, min(1.0, 1 - 2*(orientation.x**2 + orientation.y**2))))
        collision = False
        count = 1
        if self.previous is not None:
            px, py, previous_yaw = self.previous
            turn = math.remainder(yaw - previous_yaw, math.tau)
            count = max(1, math.ceil(math.hypot(x-px, y-py)/0.01),
                        math.ceil(abs(turn)/0.02))
            for index in range(1, count+1):
                fraction = index/count
                collision |= self.oracle.collides(
                    px + fraction*(x-px), py + fraction*(y-py),
                    previous_yaw + fraction*turn,
                )
            self.metrics['route_length_m'] += math.hypot(x-px, y-py)
        else:
            collision = self.oracle.collides(x, y, yaw)
        self.previous = x, y, yaw
        self.metrics['samples'] += 1
        self.metrics['swept_samples'] += count
        self.metrics['collision_samples'] += int(collision)
        self.metrics['min_height'] = min(self.metrics['min_height'], position.z)
        self.metrics['max_tilt_deg'] = max(self.metrics['max_tilt_deg'], math.degrees(tilt))
        self.metrics['elapsed_sim_s'] = now - self.first_t
        self.metrics['exit_distance_m'] = math.hypot(x-EXIT[0], y-EXIT[1])
        self.metrics['pose'] = [x, y, yaw]
        xmin, xmax, ymin, ymax = self.bounds
        outside = not (xmin <= x <= xmax and ymin <= y <= ymax)
        entrance_lane = -0.5 <= x <= 2.0 and abs(y) <= 1.0
        exit_lane = self.metrics['exit_distance_m'] <= EXIT_RADIUS
        self.metrics['exterior_samples'] += int(outside and not entrance_lane and not exit_lane)
        linear = self.odom.twist.twist.linear
        angular = self.odom.twist.twist.angular
        speed = math.hypot(linear.x, linear.y, linear.z)
        turning = math.hypot(angular.x, angular.y, angular.z)
        qualified = (self.solver_exit and self.metrics['exit_distance_m'] <= EXIT_RADIUS
                     and speed < 0.03 and turning < 0.08)
        if qualified:
            if self.stopped_since is None:
                self.stopped_since = now
        else:
            self.stopped_since = None
        self.metrics['solver_exit'] = self.solver_exit
        self.metrics['speed_m_s'] = speed
        self.metrics['success'] = bool(
            self.stopped_since is not None and now-self.stopped_since >= 2.0
            and self.metrics['samples'] >= 100
            and self.metrics['collision_samples'] == 0
            and self.metrics['exterior_samples'] == 0
            and self.metrics['max_pose_pair_error'] < 0.05
            and self.metrics['min_height'] > 0.22
            and self.metrics['max_tilt_deg'] <= math.degrees(FOOTPRINT_MAX_TILT_RAD)
        )
        self.csv.writerow([now, x, y, yaw, position.z, tilt, int(collision)])
        if now-self.last_path_t >= 0.2:
            point = PoseStamped()
            point.header.frame_id = 'map'
            point.header.stamp = self.get_clock().now().to_msg()
            point.pose.position.x, point.pose.position.y = x, y
            point.pose.position.z = position.z
            point.pose.orientation = self.odom.pose.pose.orientation
            self.path.header = point.header
            self.path.poses.append(point)
            self.path_pub.publish(self.path)
            self.last_path_t = now

    def write_status(self):
        self.csv_file.flush()
        temporary = self.run_dir / 'status.json.tmp'
        temporary.write_text(json.dumps(self.metrics, indent=2) + '\n')
        temporary.replace(self.run_dir / 'status.json')

    def destroy_node(self):
        self.write_status()
        self.csv_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Go2MazeAudit()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
