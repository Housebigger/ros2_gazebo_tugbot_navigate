#!/usr/bin/env python3
"""Accumulated 3D scatter map for RViz: subscribes the 16-beam lidar cloud
(/lidar/points, sensor frame), transforms each frame to the map frame via one
TF lookup (map->odom = the solver's ICP-corrected pose; odom->base_link = gz's
full-3D odometry incl. walking roll/pitch; base_link->anymal_c/base/lidar_3d =
the launch static mount TF), voxel-dedups into CloudMap3D and republishes the
accumulation (latched) on /maze/cloud_map_3d. Pure viz in its own process --
consumes only TF + the cloud, never touches navigation state. Frames without a
resolvable TF are dropped silently (normal until the solver seeds map->odom).
Launch-gated to flood_fill + online_slam.

TF lookup is done at the CLOUD's capture stamp (cloud.header.stamp), not the
latest available transform: the accumulator's own export blocks the callback
~300-450ms at late-run map sizes, so clouds can queue up to ~0.5s old, and
pairing a backlogged cloud with the latest (newer) pose paints 0.1-0.2m ghost
shells -- a positive feedback loop (bigger map -> slower export -> more
backlog -> more ghosts). This was quantified in the 2026-07-23 GUI ghost
forensics: the export-block backlog x latest-TF pairing doubled late-run
voxel growth relative to headless. Looking up the transform at the cloud's
own stamp (the tf2 buffer keeps history) means a backlog frame can never be
paired with a newer pose. Frames older than 0.5s are dropped outright before
the lookup -- they are near-duplicates of their neighbors, and dropping them
loses no coverage while cutting the post-block processing spike."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.duration import Duration
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
        self._stale_dropped = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL   # latch for late RViz joins
        self.pub = self.create_publisher(PointCloud2, '/maze/cloud_map_3d', qos)
        self.create_subscription(PointCloud2, '/lidar/points', self._cloud_cb, 5)
        self.get_logger().info('cloud_map_accumulator started.')

    def _cloud_cb(self, cloud):
        try:
            age = (self.get_clock().now()
                   - Time.from_msg(cloud.header.stamp)).nanoseconds / 1e9
            if age > 0.5:
                self._stale_dropped += 1
                return
            try:
                t = self.tf_buffer.lookup_transform(          # at capture stamp
                    self.map_frame, cloud.header.frame_id,
                    Time.from_msg(cloud.header.stamp),
                    timeout=Duration(seconds=0.05))
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
                self.get_logger().info(
                    'CLOUDMAP voxels=%d stale_dropped=%d'
                    % (len(self._map), self._stale_dropped))
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
