#!/usr/bin/env python3
"""One-shot: capture the legacy 2D /scan message's field contract to JSON.
Run while the (still 2D) sim is up. The projector must reproduce these
fields verbatim so the nav chain can't tell the sensor changed."""
import json
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Capture(Node):
    def __init__(self):
        super().__init__('scan_baseline_capture')
        self.msg = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

    def on_scan(self, msg):
        self.msg = msg


def main():
    out_path = sys.argv[1]
    rclpy.init()
    node = Capture()
    import time
    t0 = time.monotonic()
    while node.msg is None and time.monotonic() - t0 < 20.0:
        rclpy.spin_once(node, timeout_sec=0.5)
    m = node.msg
    assert m is not None, 'no /scan within 20s'
    n = len(m.ranges)
    inf_count = sum(1 for r in m.ranges if math.isinf(r))
    nan_count = sum(1 for r in m.ranges if math.isnan(r))
    contract = {
        'n_bins': n,
        'angle_min': m.angle_min,
        'angle_max': m.angle_max,
        'angle_increment': m.angle_increment,
        'time_increment': m.time_increment,
        'scan_time': m.scan_time,
        'range_min': m.range_min,
        'range_max': m.range_max,
        'frame_id': m.header.frame_id,
        'observed_inf_bins': inf_count,
        'observed_nan_bins': nan_count,
    }
    with open(out_path, 'w') as f:
        json.dump(contract, f, indent=2, sort_keys=True)
    print(json.dumps(contract, indent=2, sort_keys=True))
    rclpy.shutdown()
    print('SCAN BASELINE CAPTURED')


if __name__ == '__main__':
    main()
