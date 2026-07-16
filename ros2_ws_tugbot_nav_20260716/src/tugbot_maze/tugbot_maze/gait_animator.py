"""Drives the dog's 12 leg joints with an open-loop trot synced to /odom.

Visual only, zero nav coupling by construction: reads /odom, writes joint
position targets bridged to the model's JointPositionControllers. If this
node dies the dog just stops swinging its legs and glides; navigation is
untouched.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

from tugbot_maze.gait import JOINTS, stride_frequency, trot_pose


class GaitAnimator(Node):
    def __init__(self):
        super().__init__('gait_animator')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('model_name', 'anymal_c')
        model = self.get_parameter('model_name').value
        # Real gz JointPositionController topic layout includes the joint
        # index segment: /model/<model>/joint/<J>/0/cmd_pos (verified).
        self._pubs = {
            j: self.create_publisher(Float64, f'/model/{model}/joint/{j}/0/cmd_pos', 10)
            for j in JOINTS
        }
        self._v = 0.0
        self._omega = 0.0
        self._phase = 0.0
        self._dt = 1.0 / float(self.get_parameter('rate_hz').value)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_timer(self._dt, self._tick)

    def _on_odom(self, msg):
        self._v = msg.twist.twist.linear.x
        self._omega = msg.twist.twist.angular.z

    def _tick(self):
        try:
            f = stride_frequency(self._v, self._omega)
            self._phase = (self._phase + 2.0 * math.pi * f * self._dt) % (2.0 * math.pi)
            for joint, angle in trot_pose(self._phase, self._v, self._omega).items():
                msg = Float64()
                msg.data = float(angle)
                self._pubs[joint].publish(msg)
        except Exception as exc:  # never crash the animation loop
            self.get_logger().warning(f'gait tick failed: {exc}')


def main():
    rclpy.init()
    node = GaitAnimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
