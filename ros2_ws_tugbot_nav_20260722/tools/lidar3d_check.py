#!/usr/bin/env python3
"""Live gate: /lidar/points must stream 3D clouds and the projected /scan
must reproduce the legacy field contract exactly (the nav chain must not
be able to tell the sensor changed)."""
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2


class Check(Node):
    def __init__(self):
        super().__init__('lidar3d_check')
        self.clouds = []
        self.scans = []
        self.create_subscription(PointCloud2, '/lidar/points', self.on_cloud, 5)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

    def on_cloud(self, m):
        self.clouds.append((time.monotonic(), m))

    def on_scan(self, m):
        self.scans.append((time.monotonic(), m))


def main():
    contract = json.loads((Path(__file__).resolve().parents[1] /
                           'src/tugbot_maze/test/scan_omni_baseline.json').read_text())
    rclpy.init()
    node = Check()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8.0:   # ~80 clouds/scans at 10Hz nominal
        rclpy.spin_once(node, timeout_sec=0.5)
    assert len(node.clouds) >= 5, f'only {len(node.clouds)} clouds in 8s'
    assert len(node.scans) >= 5, f'only {len(node.scans)} scans in 8s'
    span = node.scans[-1][0] - node.scans[0][0]
    hz = (len(node.scans) - 1) / span if span > 0 else 0.0
    cloud = node.clouds[-1][1]
    n_pts = cloud.width * cloud.height
    scan = node.scans[-1][1]
    print(f'clouds={len(node.clouds)} scans={len(node.scans)} scan_hz={hz:.1f} '
          f'cloud_pts={n_pts} cloud_frame={cloud.header.frame_id} scan_frame={scan.header.frame_id}')
    assert 8000 <= n_pts <= 28800, f'cloud size {n_pts}: expected O(1800x16)'
    assert 6.0 <= hz <= 14.0, f'scan rate {hz:.1f}Hz outside 6-14 (nominal 10)'
    assert len(scan.ranges) == contract['n_bins'], f'{len(scan.ranges)} bins'
    for key, got in (('angle_min', scan.angle_min), ('angle_max', scan.angle_max),
                     ('angle_increment', scan.angle_increment),
                     ('time_increment', scan.time_increment),
                     ('scan_time', scan.scan_time),
                     ('range_min', scan.range_min), ('range_max', scan.range_max)):
        assert math.isclose(got, contract[key], abs_tol=1e-7), f'{key}: {got} != {contract[key]}'
    finite = [r for r in scan.ranges if math.isfinite(r)]
    assert len(finite) > 0.2 * len(scan.ranges), f'only {len(finite)} finite bins - projector starving?'
    assert min(finite) < 5.0, 'no nearby wall return - geometry suspicious'
    rclpy.shutdown()
    print('LIDAR3D CHECK PASS')


if __name__ == '__main__':
    main()
