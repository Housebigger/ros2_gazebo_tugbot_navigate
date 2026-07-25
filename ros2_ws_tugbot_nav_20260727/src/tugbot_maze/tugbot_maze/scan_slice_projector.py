#!/usr/bin/env python3
"""Subscribes /lidar/points (PointCloud2 from the 16-beam lidar) and
republishes the legacy /scan (LaserScan) via the pure slice_to_scan core,
so the 2D-native nav chain runs unchanged on the 3D sensor. One scan out
per cloud in (10Hz). Field values come from SCAN_CONTRACT (captured from
the last 2D boot); header stamp/frame are passed through from the cloud."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

from tugbot_maze.slice_to_scan import SCAN_CONTRACT, fold_to_ranges


class ScanSliceProjector(Node):
    def __init__(self):
        super().__init__('scan_slice_projector')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(PointCloud2, '/lidar/points', self.on_cloud, 5)

    def on_cloud(self, cloud):
        try:
            pts = point_cloud2.read_points_numpy(
                cloud, field_names=('x', 'y', 'z'), skip_nans=False)
            c = SCAN_CONTRACT
            msg = LaserScan()
            msg.header = cloud.header
            msg.angle_min = c['angle_min']
            msg.angle_max = c['angle_max']
            msg.angle_increment = c['angle_increment']
            msg.time_increment = c['time_increment']
            msg.scan_time = c['scan_time']
            msg.range_min = c['range_min']
            msg.range_max = c['range_max']
            msg.ranges = fold_to_ranges(
                pts, c['n_bins'], c['angle_min'], c['angle_increment'],
                c['range_min'], c['range_max'])
            self.pub.publish(msg)
        except Exception as e:   # never crash: /scan feeds the whole nav chain
            self.get_logger().error(f'projector cloud dropped: {e}',
                                    throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = ScanSliceProjector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
