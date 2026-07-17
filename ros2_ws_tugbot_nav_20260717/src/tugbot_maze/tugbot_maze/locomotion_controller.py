"""Real legged locomotion: /cmd_vel -> trot FSM -> 12 joint position targets.

Replaces the 20260716 gait_animator (which was a visual-only animation over a
magic-force base). This node IS the base motion now: if it stops stepping the
dog stops. Still guarded per-tick — an exception holds the last targets and
logs, it never crashes the node.

Fall detection lives here too (odom is dimensions=3 now): |roll| or |pitch|
> fall_attitude_rad, or base z < fall_z_m, sustained fall_hold_s, logs one
FALL_DETECTED line (the run wrapper greps it) and freezes the gait in STAND.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

from tugbot_maze.legged.kinematics import JOINTS
from tugbot_maze.legged.trot import LocomotionFSM


def _roll_pitch(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


class LocomotionController(Node):
    def __init__(self):
        super().__init__('locomotion_controller')
        rate_hz = self.declare_parameter('rate_hz', 100.0).value
        model = self.declare_parameter('model_name', 'anymal_c').value
        self.declare_parameter('force_trot', False)
        self._fall_att = self.declare_parameter('fall_attitude_rad', 0.6).value
        self._fall_z = self.declare_parameter('fall_z_m', 0.25).value
        self._fall_hold = self.declare_parameter('fall_hold_s', 1.0).value
        # ROS-side topics have no /0 joint-index token (illegal in ROS names);
        # the bridge maps them onto gz .../joint/<J>/0/cmd_pos.
        self._pubs = {
            j: self.create_publisher(Float64, f'/model/{model}/joint/{j}/cmd_pos', 10)
            for j in JOINTS
        }
        self._fsm = LocomotionFSM()
        self._cmd = (0.0, 0.0)
        self._att = (0.0, 0.0)
        self._z = None
        self._odom_t = None          # sim time (s) of last odom
        self._viol_since = None
        self._fallen = False
        self._last_out = None
        self._dt = 1.0 / max(float(rate_hz), 1.0)
        self._diag_count = 0
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'locomotion_controller started (model={model}, rate={1.0 / self._dt:.0f} Hz).')

    def _on_cmd(self, msg):
        self._cmd = (msg.linear.x, msg.angular.z)

    def _on_odom(self, msg):
        self._att = _roll_pitch(msg.pose.pose.orientation)
        self._z = msg.pose.pose.position.z
        self._odom_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._check_fall()

    def _check_fall(self):
        if self._fallen or self._odom_t is None:
            return
        roll, pitch = self._att
        bad = abs(roll) > self._fall_att or abs(pitch) > self._fall_att or self._z < self._fall_z
        if not bad:
            self._viol_since = None
            return
        if self._viol_since is None:
            self._viol_since = self._odom_t
        elif self._odom_t - self._viol_since >= self._fall_hold:
            self._fallen = True
            self.get_logger().error(
                f'FALL_DETECTED roll={roll:.2f} pitch={pitch:.2f} z={self._z:.2f} '
                f't={self._odom_t:.1f}')

    def _tick(self):
        try:
            if self._fallen or self._fsm.mode == LocomotionFSM.INIT:
                # Zero during INIT too: a cmd held through the settle would
                # slew the FSM's internal vx to max and make the first TROT
                # entry apply a full stride in one tick (Task 4 review note).
                vx, wz = 0.0, 0.0
            else:
                vx, wz = self._cmd
            force_trot = bool(self.get_parameter('force_trot').value)
            roll, pitch = self._att
            out = self._fsm.step(self._dt, vx, wz, roll, pitch,
                                 force_trot=force_trot and not self._fallen)
            self._last_out = out
        except Exception as exc:   # never crash the control loop
            self.get_logger().warning(f'locomotion tick failed: {exc}')
            out = self._last_out
        if out is None:
            return
        for joint, angle in out.items():
            msg = Float64()
            msg.data = float(angle)
            self._pubs[joint].publish(msg)
        self._diag_count += 1
        if self._diag_count >= int(1.0 / self._dt):   # ~1 Hz
            self._diag_count = 0
            roll, pitch = self._att
            self.get_logger().info(
                f'LOCO mode={self._fsm.mode} phase={self._fsm.phase:.2f} '
                f'vx={self._fsm.vx:.2f} wz={self._fsm.wz:.2f} '
                f'roll={roll:.3f} pitch={pitch:.3f} z={self._z if self._z is None else round(self._z, 3)}')


def main(args=None):
    rclpy.init(args=args)
    node = LocomotionController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
