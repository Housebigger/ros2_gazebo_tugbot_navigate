#!/usr/bin/env python3
"""World-anchored-odom smoke check (tugbot-return Task 5). Drives a gentle arc
via /cmd_vel for DRIVE_S seconds, then compares the /odom displacement against
the ground-truth displacement from /gt/dynamic_pose (EVAL-ONLY readout — red
line: gt never feeds control; here it only grades the odom plugin).

PASS = displacement mismatch < 0.05 m AND both sources actually moved > 0.3 m
(catches both a drifting wheel-integrated odom and a silent/dead plugin).
Exit code 0 on PASS, 1 on FAIL, 2 on missing data."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

DRIVE_S = 6.0
SETTLE_S = 2.0
V, W = 0.25, 0.10        # near-straight arc: the maze entry corridor is 2 m wide,
                         # lateral excursion at R=2.5 m over 6 s is ~0.44 m — safe
TOL_M = 0.05
MIN_TRAVEL_M = 0.3


class AnchorCheck(Node):
    def __init__(self):
        super().__init__('tugbot_odom_anchor_check')
        self.odom_xy = None
        self.gt_xy = None
        self.odom0 = None
        self.gt0 = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(TFMessage, '/gt/dynamic_pose', self._gt, 10)

    def _odom(self, msg):
        p = msg.pose.pose.position
        self.odom_xy = (p.x, p.y)

    def _gt(self, msg):
        # ros_gz_bridge's built-in gz.msgs.Pose_V -> tf2_msgs/TFMessage
        # conversion does NOT carry the gz Pose "name" field into
        # header.frame_id / child_frame_id -- both come through empty
        # (confirmed via raw ros2/gz dumps). tugbot is the maze world's only
        # non-static top-level model, so gz's SceneBroadcaster consistently
        # emits its own world-anchored pose as transforms[0], ahead of its
        # child links (base_link, imu_link, ...) -- index 0 is used instead.
        if msg.transforms:
            tr = msg.transforms[0].transform.translation
            self.gt_xy = (tr.x, tr.y)


def main():
    rclpy.init()
    node = AnchorCheck()
    # 40s: the maze world's SceneBroadcaster (50+ static wall models) needs
    # real margin to enumerate the scene and start publishing dynamic_pose/info
    # before the lazy gt bridge has anything to forward -- measured ~40s to
    # first message on this machine, well past a 20s deadline.
    deadline = time.monotonic() + 40.0
    while (node.odom_xy is None or node.gt_xy is None) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.odom_xy is None or node.gt_xy is None:
        print(f'ANCHOR_CHECK missing data: odom={node.odom_xy} gt={node.gt_xy}')
        return 2
    node.odom0, node.gt0 = node.odom_xy, node.gt_xy

    cmd = Twist()
    cmd.linear.x, cmd.angular.z = V, W
    t_end = time.monotonic() + DRIVE_S
    while time.monotonic() < t_end:
        node.pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.05)
    node.pub.publish(Twist())                      # stop
    t_end = time.monotonic() + SETTLE_S
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    d_odom = (node.odom_xy[0] - node.odom0[0], node.odom_xy[1] - node.odom0[1])
    d_gt = (node.gt_xy[0] - node.gt0[0], node.gt_xy[1] - node.gt0[1])
    err = math.hypot(d_odom[0] - d_gt[0], d_odom[1] - d_gt[1])
    trav = math.hypot(*d_gt)
    print(f'ANCHOR_CHECK d_odom={d_odom} d_gt={d_gt} err={err:.4f} travel={trav:.3f}')
    ok = err < TOL_M and trav > MIN_TRAVEL_M and math.hypot(*d_odom) > MIN_TRAVEL_M
    print(f'ANCHOR_CHECK {"PASS" if ok else "FAIL"}')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
