#!/usr/bin/env python3
"""One-shot capture of the latched production map (/maze/cloud_map_3d) to an
.npy voxel dump. MUST run while the stack is alive (the latched publisher
dies with the accumulator node). Exit 0 on success, 2 on timeout."""
import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--topic', default='/maze/cloud_map_3d')
    ap.add_argument('--timeout', type=float, default=20.0)
    args = ap.parse_args(argv)

    rclpy.init()
    node = Node('cloud_map_capture')
    got = {}
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL   # receive the latch
    node.create_subscription(PointCloud2, args.topic,
                             lambda m: got.setdefault('msg', m), qos)
    deadline = time.monotonic() + args.timeout
    while 'msg' not in got and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    rc = 2
    if 'msg' in got:
        pts = point_cloud2.read_points_numpy(
            got['msg'], field_names=('x', 'y', 'z'), skip_nans=False)
        np.save(args.out, pts.astype(np.float32))
        print('CAPTURE %s voxels=%d -> %s' % (args.topic, len(pts), args.out))
        rc = 0
    else:
        print('CAPTURE timeout on %s' % args.topic)
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
