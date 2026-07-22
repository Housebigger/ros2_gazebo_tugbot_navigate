#!/usr/bin/env python3
"""Accumulated 3D scatter map for RViz: subscribes the 16-beam lidar cloud
(/lidar/points, sensor frame), transforms each frame to the map frame via one
TF lookup (map->odom = the solver's ICP-corrected pose; odom->base_link = gz's
full-3D odometry incl. walking roll/pitch; base_link->anymal_c/base/lidar_3d =
the launch static mount TF), voxel-dedups into CloudMap3D and republishes the
accumulation (latched) on /maze/cloud_map_3d. Pure viz in its own process --
consumes only TF + the cloud, never touches navigation state. Frames without a
resolvable TF are dropped silently (normal until the solver seeds map->odom).
Launch-gated to flood_fill + online_slam."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros

from tugbot_maze.cloud_map_3d import CloudMap3D, should_publish, transform_to_matrix


class CloudMapAccumulator(Node):
    def __init__(self):
        super().__init__('cloud_map_accumulator')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.publish_period_s = float(self.declare_parameter('publish_period_s', 1.0).value)
        self._map = CloudMap3D()
        self._last_pub_s = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL   # latch for late RViz joins
        self.pub = self.create_publisher(PointCloud2, '/maze/cloud_map_3d', qos)
        self.create_subscription(PointCloud2, '/lidar/points', self._cloud_cb, 5)
        self.get_logger().info('cloud_map_accumulator started.')

    def _cloud_cb(self, cloud):
        try:
            try:
                t = self.tf_buffer.lookup_transform(          # latest available
                    self.map_frame, cloud.header.frame_id, Time())
            except tf2_ros.TransformException:
                return
            tr, q = t.transform.translation, t.transform.rotation
            T = transform_to_matrix(tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)
            pts = point_cloud2.read_points_numpy(
                cloud, field_names=('x', 'y', 'z'), skip_nans=False)
            added = self._map.add_cloud(pts, T)
            now = self.get_clock().now()
            now_s = now.nanoseconds / 1e9
            if should_publish(self._last_pub_s, now_s, added, self.publish_period_s):
                self.pub.publish(self._map.to_pointcloud2(
                    frame_id=self.map_frame, stamp=now.to_msg()))
                self._last_pub_s = now_s
                self.get_logger().info('CLOUDMAP voxels=%d' % len(self._map))
        except Exception as e:              # viz must never crash or spam
            self.get_logger().error('cloud frame dropped: %r' % e,
                                    throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = CloudMapAccumulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
