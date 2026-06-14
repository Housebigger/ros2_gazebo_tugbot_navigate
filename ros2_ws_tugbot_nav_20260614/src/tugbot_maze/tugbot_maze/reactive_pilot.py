"""Reliable locomotion for the Trémaux solver.

Owns end-to-end motion: drive_to/follow_path try a short Nav2 NavigateToPose
goal and fall back to a 10 Hz reactive /cmd_vel_nav state machine (rotate ->
drive) with watchdog + unwedge; back_out reverses out of a dead end. Extracted
from the proven GCN reactive drive in maze_explorer.py. Mode-free.
"""

import math

from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose

from tugbot_maze.maze_perception import normalize_angle

SUCCESS = 'success'
WALL_AHEAD = 'wall_ahead'
WEDGED = 'wedged'

# Proven reactive thresholds ported verbatim from maze_explorer.py.
_ROT_TOLERANCE = math.radians(8)        # rotation "complete" / jam threshold
_FWD_CONE_HALF = math.radians(20)       # forward (and rear) laser cone half-angle
_FWD_MIN_RANGE = 0.5                    # forward clearance required while driving (m)
_REAR_MIN_RANGE = 0.35                  # rear clearance required while backing (m)
_PROGRESS_EPS = 0.1                     # min movement counted as progress (m)
_UNWEDGE_BACKUP_M = 0.4                 # reverse distance for a jam unwedge (m)


class ReactivePilot:
    """Nav2-hop-with-reactive-fallback locomotion for the Trémaux solver.

    Caller must poll is_active() and only issue a new drive_to/follow_path/back_out
    when not active.
    """

    def __init__(self, node, action_client, *, nav_timeout_sec=18.0,
                 max_seconds=12.0, no_progress_sec=3.0, no_rot_sec=3.0,
                 forward_speed=0.2, turn_speed=0.5, reverse_speed=0.15):
        self.node = node
        self.action_client = action_client
        self.nav_timeout_sec = nav_timeout_sec
        self.max_seconds = max_seconds
        self.no_progress_sec = no_progress_sec
        self.no_rot_sec = no_rot_sec
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.reverse_speed = reverse_speed
        self._reset()

    def _reset(self):
        self.state = 'idle'        # idle | nav | rotating | driving | backup
        self.queue = []            # remaining waypoints for follow_path
        self.result = None         # SUCCESS | WALL_AHEAD | WEDGED on finish
        self.target_xy = None
        self.target_yaw = 0.0
        self.target_distance = 0.0
        self.start_xy = None
        self.start_time = None
        self.progress_ref_xy = None
        self.progress_ref_time = None
        self.backup_ref_xy = None
        self.rot_ref_yaw = None
        self.rot_ref_time = None
        self.nav_done = True
        self.nav_succeeded = False
        self.nav_deadline = None
        self._nav_goal_handle = None
        self._nav_seq = 0          # monotonic goal token; stale callbacks ignored
        # True => current 'backup' is an intentional back_out (reverse the full
        # requested distance, finish SUCCESS).  False => a mid-rotate/drive jam
        # unwedge (reverse a short safety distance, finish WEDGED).
        self._reverse_is_backout = False

    def is_active(self):
        return self.state != 'idle'

    # ------------------------------------------------------------------ API
    def drive_to(self, target_xy):
        self.queue = [(float(target_xy[0]), float(target_xy[1]))]
        self.result = None
        self._begin_next()

    def follow_path(self, waypoints):
        self.queue = [(float(x), float(y)) for (x, y) in waypoints]
        self.result = None
        self._begin_next()

    def back_out(self, distance, robot_pose):
        self.queue = []
        self.result = None
        self._begin_reactive(robot_pose[2], distance, reverse=True)

    def reactive_drive(self, angle, distance):
        """Forced-reactive forward push (used for maze entry, no Nav2)."""
        self.queue = []
        self.result = None
        self._begin_reactive(angle, distance)

    def _begin_next(self):
        if not self.queue:
            self._finish(SUCCESS)
            return
        self.target_xy = self.queue[0]
        self._dispatch_nav(self.target_xy)

    # ------------------------------------------------------------- Nav2 hop
    def _dispatch_nav(self, target_xy):
        # Non-blocking readiness gate: a blocking wait_for_server() would freeze
        # the single-threaded executor (no cmd_vel/scan) during the 10 Hz loop.
        if not self.action_client.server_is_ready():
            self._fallback_reactive(); return
        pose = self.node._lookup_pose()
        yaw = math.atan2(target_xy[1] - pose[1], target_xy[0] - pose[0]) if pose else 0.0
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.node.map_frame
        goal.pose.pose.position.x = float(target_xy[0])
        goal.pose.pose.position.y = float(target_xy[1])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.nav_done = False
        self.nav_succeeded = False
        self.nav_deadline = self.node.get_clock().now().nanoseconds / 1e9 + self.nav_timeout_sec
        self.state = 'nav'
        self._nav_seq += 1
        seq = self._nav_seq
        fut = self.action_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._on_goal_response(f, seq))

    # Nav2 callbacks. Assumes rclpy's default single-threaded executor (these
    # callbacks and tick() serialize); the seq-token also makes it safe under a
    # MultiThreadedExecutor — a stale callback from a cancelled/superseded goal
    # (seq != self._nav_seq) is ignored.
    def _on_goal_response(self, fut, seq):
        gh = fut.result()
        if not gh.accepted:
            if seq == self._nav_seq:
                self.nav_done = True
                self.nav_succeeded = False
            return
        self._nav_goal_handle = gh
        gh.get_result_async().add_done_callback(
            lambda f: self._on_nav_result(f, seq))

    def _on_nav_result(self, fut, seq):
        if seq != self._nav_seq:
            return  # stale result from a cancelled/superseded goal
        self.nav_succeeded = (fut.result().status == 4)
        self.nav_done = True

    def _fallback_reactive(self):
        pose = self.node._lookup_pose()
        if pose is None or self.target_xy is None:
            self._finish(WEDGED); return
        ang = math.atan2(self.target_xy[1] - pose[1], self.target_xy[0] - pose[0])
        dist = max(0.3, math.hypot(self.target_xy[0] - pose[0], self.target_xy[1] - pose[1]))
        self._begin_reactive(ang, dist)

    # -------------------------------------------------- reactive state setup
    def _begin_reactive(self, angle, distance, reverse=False):
        self.target_yaw = angle
        self.target_distance = distance
        self.start_xy = None
        self.start_time = self.node.get_clock().now()
        self.progress_ref_xy = None; self.progress_ref_time = None
        self.backup_ref_xy = None
        self.rot_ref_yaw = None; self.rot_ref_time = None
        self._reverse_is_backout = reverse
        self.state = 'backup' if reverse else 'rotating'

    # ------------------------------------------------------------- per tick
    def tick(self, robot_pose):
        if self.state == 'idle':
            return

        if self.state == 'nav':
            now = self.node.get_clock().now().nanoseconds / 1e9
            if self.nav_done:
                if self.nav_succeeded:
                    self._on_hop_reached()
                else:
                    self._fallback_reactive()
            elif now > self.nav_deadline:
                self._cancel_nav(); self._fallback_reactive()
            return

        if robot_pose is None:
            return

        # ---- Ported reactive state machine (maze_explorer.py:4429-4580) ----
        robot_x, robot_y, robot_yaw = robot_pose
        now = self.node.get_clock().now()

        # Watchdog: hard timeout for the whole reactive drive. An intentional
        # back_out can legitimately take longer (reverse the full target_distance),
        # so scale the limit by the expected reverse time in that case.
        if self.start_time is not None:
            elapsed = (now - self.start_time).nanoseconds / 1e9
            if self._reverse_is_backout:
                effective_max = max(self.max_seconds,
                                    self.target_distance / self.reverse_speed + 4.0)
            else:
                effective_max = self.max_seconds
            if elapsed > effective_max:
                self.node.get_logger().warn(
                    'PILOT: watchdog timeout after %.1fs (state=%s, limit=%.1fs) — aborting'
                    % (elapsed, self.state, effective_max))
                self._finish(WEDGED)
                return

        if self.state == 'backup':
            # Reverse to recover. Two uses, selected by _reverse_is_backout:
            #   intentional back_out -> reverse target_distance, finish SUCCESS
            #   jam unwedge          -> reverse _UNWEDGE_BACKUP_M, finish WEDGED
            if self.backup_ref_xy is None:
                self.backup_ref_xy = (robot_x, robot_y)
            bx, by = self.backup_ref_xy
            backed = math.hypot(robot_x - bx, robot_y - by)
            target = self.target_distance if self._reverse_is_backout else _UNWEDGE_BACKUP_M
            if backed >= target:
                self.node.get_logger().info(
                    'PILOT: backed up %.2fm / %.2fm (%s) — finishing'
                    % (backed, target, 'back_out' if self._reverse_is_backout else 'unwedge'))
                self._finish(SUCCESS if self._reverse_is_backout else WEDGED)
                return
            # Rear-cone safety: stop reversing if a wall is close behind.
            if not self._is_laser_clear_in_direction(
                    math.pi, min_range=_REAR_MIN_RANGE, cone_half_angle=_FWD_CONE_HALF):
                self.node.get_logger().warn(
                    'PILOT: obstacle behind after backing %.2fm — stopping' % backed)
                self._finish(WEDGED)
                return
            twist = Twist()
            twist.linear.x = -self.reverse_speed
            self.node.cmd_vel_pub.publish(twist)
            return

        if self.state == 'rotating':
            angle_diff = normalize_angle(self.target_yaw - robot_yaw)

            if abs(angle_diff) < _ROT_TOLERANCE:
                # Rotation complete → start driving.
                self.node.cmd_vel_pub.publish(Twist())
                self.state = 'driving'
                self.start_xy = (robot_x, robot_y)
                self.node.get_logger().info('PILOT: rotation complete, driving forward')
                return

            # No-rotation (jammed) detection: if the robot is physically caught
            # against a wall it won't actually turn even while commanded. If yaw
            # barely changes for a few seconds, back up to free it.
            if self.rot_ref_yaw is None:
                self.rot_ref_yaw = robot_yaw
                self.rot_ref_time = now
            else:
                yaw_moved = abs(normalize_angle(robot_yaw - self.rot_ref_yaw))
                if yaw_moved > _ROT_TOLERANCE:
                    self.rot_ref_yaw = robot_yaw
                    self.rot_ref_time = now
                elif self.rot_ref_time is not None:
                    stuck = (now - self.rot_ref_time).nanoseconds / 1e9
                    if stuck > self.no_rot_sec:
                        self.node.get_logger().warn(
                            'PILOT: cannot rotate (yaw moved %.0f deg in %.1fs) '
                            '— jammed, backing up' % (math.degrees(yaw_moved), stuck))
                        self.state = 'backup'
                        self.backup_ref_xy = None
                        self._reverse_is_backout = False
                        return

            # Publish rotation (turn_speed = velocity_smoother max for snappier turns).
            twist = Twist()
            twist.angular.z = self.turn_speed if angle_diff > 0 else -self.turn_speed
            self.node.cmd_vel_pub.publish(twist)
            return

        if self.state == 'driving':
            if self.start_xy is not None:
                dx = robot_x - self.start_xy[0]
                dy = robot_y - self.start_xy[1]
                distance_traveled = math.hypot(dx, dy)
            else:
                distance_traveled = 0.0

            if distance_traveled >= self.target_distance:
                # Target reached.
                self.node.get_logger().info(
                    'PILOT: completed %.2fm / %.1fm'
                    % (distance_traveled, self.target_distance))
                self._on_hop_reached()
                return

            # Safety: check laser ahead.
            if not self._is_laser_clear_ahead(min_range=_FWD_MIN_RANGE, cone_half_angle=_FWD_CONE_HALF):
                self.node.get_logger().warn(
                    'PILOT: obstacle ahead after %.2fm, stopping' % distance_traveled)
                self._finish(WALL_AHEAD)
                return

            # No-progress (wedge) detection: the forward laser cone can stay
            # clear while the robot is physically caught on a corner wall and
            # odom barely advances. If we don't actually move for a few seconds,
            # treat it as wedged and back up to unstick.
            if self.progress_ref_xy is None:
                self.progress_ref_xy = (robot_x, robot_y)
                self.progress_ref_time = now
            else:
                rx0, ry0 = self.progress_ref_xy
                moved = math.hypot(robot_x - rx0, robot_y - ry0)
                if moved > _PROGRESS_EPS:
                    self.progress_ref_xy = (robot_x, robot_y)
                    self.progress_ref_time = now
                elif self.progress_ref_time is not None:
                    stuck = (now - self.progress_ref_time).nanoseconds / 1e9
                    if stuck > self.no_progress_sec:
                        self.node.get_logger().warn(
                            'PILOT: no progress for %.1fs (moved %.2fm) after %.2fm '
                            '— wedged, backing up' % (stuck, moved, distance_traveled))
                        self.state = 'backup'
                        self.backup_ref_xy = None
                        self._reverse_is_backout = False
                        return

            # Drive forward.
            twist = Twist()
            twist.linear.x = self.forward_speed
            self.node.cmd_vel_pub.publish(twist)
            return

        # Unknown state → finish defensively.
        self._finish(WEDGED)

    def _on_hop_reached(self):
        self.node.cmd_vel_pub.publish(Twist())
        if self.queue:
            self.queue.pop(0)
        if self.queue:
            self._begin_next()
        else:
            self._finish(SUCCESS)

    def _cancel_nav(self):
        try:
            if self._nav_goal_handle is not None:
                self._nav_goal_handle.cancel_goal_async()
        except Exception:
            pass
        self._nav_goal_handle = None
        self.nav_done = True; self.nav_succeeded = False

    def _finish(self, result):
        self.node.cmd_vel_pub.publish(Twist())
        self._reset()
        self.result = result

    # ----------------------------------------------------- laser helpers
    def _is_laser_clear_in_direction(self, direction, min_range, cone_half_angle):
        """Check laser scan for clear path in direction (robot frame)."""
        scan = self.node.scan_msg
        if scan is None:
            return False
        for i in range(len(scan.ranges)):
            r = scan.ranges[i]
            if not math.isfinite(r):
                continue
            scan_angle = scan.angle_min + i * scan.angle_increment
            if abs(normalize_angle(scan_angle - direction)) <= cone_half_angle:
                if r < min_range:
                    return False
        return True

    def _is_laser_clear_ahead(self, min_range, cone_half_angle):
        """Check laser scan for clear path directly ahead (0° robot frame)."""
        return self._is_laser_clear_in_direction(0.0, min_range, cone_half_angle)
