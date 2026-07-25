#!/usr/bin/env python3
"""GT parallel accumulator (center-twist-root-cause, EVAL-ONLY, default OFF).

Accumulates the SAME /lidar/points clouds as the production
cloud_map_accumulator, but transformed by the GROUND-TRUTH pose from
/gt/dynamic_pose instead of the believed map->base TF. One run yields two
same-scale maps: if the gt map is clean while the production map twists, the
twist is 100% believed-pose error; if both twist, localization is exonerated
and the cause sits in the sensing/timing layer.

RED LINE: ground truth enters ONLY this instrument. It publishes nothing any
control or navigation node consumes (/maze/gt_cloud_map_3d is latched for
optional RViz inspection, at most one publish per 30 s -- Heisenberg guard:
the ghost-forensics lesson is that instrument load alters the phenomenon).

A/B fairness rules (the comparison is void without them):
- Same CloudMap3D parameters as production (0.05 m voxel, range <= 8 m,
  z-clip [-1, 3]) -- CloudMap3D() defaults, byte-shared code path.
- Same staleness discipline: frames older than 0.5 s are dropped (mirrors the
  production accumulator's stale-drop).
- Pairing window: nearest gt sample to the CLOUD stamp, |dt| <= 0.05 s -- the
  production TF lookup timeout. gt arrives at ~58 Hz vs 10 Hz clouds, so the
  typical pairing distance is <= ~9 ms.
- /gt/dynamic_pose frame ids are EMPTY (ros_gz_bridge drops gz names --
  tugbot-return finding): transforms[0] is the tugbot (the maze world's only
  non-static top-level model). Caveat: a second top-level dynamic model would
  break the fixed-index assumption.

The final voxel set is dumped as an .npy (N x 3 float32, map frame) into
dump_dir at shutdown for offline comparison by tools/map_twist_metric.py."""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage

from tugbot_maze.cloud_map_3d import CloudMap3D, should_publish, transform_to_matrix

# Coordinate anchors (maze_20260528_scaled_instance.yaml): map = world + offset.
WORLD_TO_MAP_X = 11.011281
WORLD_TO_MAP_Y = 9.02507
LIDAR_MOUNT_Z = 0.55          # tugbot contract: lidar_3d at base_link +0.55 (0 0 0.55)
MAX_PAIR_DT_S = 0.05          # == the production accumulator's TF lookup timeout
STALE_S = 0.5                 # == the production accumulator's stale-drop bound
GT_BUFFER_S = 2.0             # rolling gt sample window
TOPIC_PUB_MIN_PERIOD_S = 30.0  # Heisenberg guard on the latched publish


def nearest_gt_sample(buffer, t_s):
    """Nearest-stamp sample from [(t_s, payload), ...]; None if empty or the
    nearest is farther than MAX_PAIR_DT_S (fairness: no looser than the
    production TF pairing window).
    Nearest (not interpolated) -- production tf2 interpolates to the exact
    stamp; at ~58 Hz gt the difference is sub-voxel (~mm), accepted asymmetry."""
    if not buffer:
        return None
    best = min(buffer, key=lambda item: abs(item[0] - t_s))
    if abs(best[0] - t_s) > MAX_PAIR_DT_S:
        return None
    return best


def trim_gt_buffer(buf, newest_t_s):
    """Drop samples older than newest_t_s - GT_BUFFER_S. Trim keys off the
    NEWEST stamp; a backward sim-time jump would leave future-stamped
    samples lingering (acceptable in sim -- documented, not guarded)."""
    cutoff = newest_t_s - GT_BUFFER_S
    while buf and buf[0][0] < cutoff:
        buf.pop(0)
    return buf


def compose_map_sensor(x, y, z, qx, qy, qz, qw):
    """T_map_sensor = Trans(world->map) @ T_world_base(gt) @ T_base_sensor.
    The gz model pose IS the base_link pose (model root); the sensor mount is
    a pure +z translation per the tugbot contract (lidar_3d pose 0 0 0.55)."""
    T_offset = transform_to_matrix(WORLD_TO_MAP_X, WORLD_TO_MAP_Y, 0.0,
                                   0.0, 0.0, 0.0, 1.0)
    T_world_base = transform_to_matrix(x, y, z, qx, qy, qz, qw)
    T_base_sensor = transform_to_matrix(0.0, 0.0, LIDAR_MOUNT_Z,
                                        0.0, 0.0, 0.0, 1.0)
    return T_offset @ T_world_base @ T_base_sensor


class GtCloudAccumulator(Node):
    def __init__(self):
        super().__init__('gt_cloud_accumulator')
        self.dump_dir = self.declare_parameter('dump_dir', '').value
        self._map = CloudMap3D()               # SAME defaults as production
        self._gt_buf = []                      # [(t_s, (x,y,z,qx,qy,qz,qw))]
        self._last_log_s = None
        self._last_topic_pub_s = None
        self._stale_dropped = 0
        self._unpaired = 0
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(PointCloud2, '/maze/gt_cloud_map_3d', qos)
        self.create_subscription(TFMessage, '/gt/dynamic_pose', self._gt_cb, 50)
        self.create_subscription(PointCloud2, '/lidar/points', self._cloud_cb, 5)
        self.create_timer(10.0, self._heartbeat)
        self.get_logger().info('gt_cloud_accumulator started (EVAL-ONLY).')

    def _heartbeat(self):
        # Unconditional -- a fully-starved gt path (no pairing ever succeeds)
        # would otherwise be SILENT, since the should_publish-gated log below
        # never fires. This guarantees a voided A/B is loudly visible in
        # launch.log even when nothing accumulates.
        self.get_logger().info(
            'GTCLOUD voxels=%d stale_dropped=%d unpaired=%d gt_buf=%d'
            % (len(self._map), self._stale_dropped, self._unpaired, len(self._gt_buf)))

    def _gt_cb(self, msg):
        # transforms[0] = the tugbot model root (bridge drops gz names; index
        # convention per the tugbot-return record). Stamp from the message
        # header of the first transform.
        if not msg.transforms:
            return
        t0 = msg.transforms[0]
        t_s = Time.from_msg(t0.header.stamp).nanoseconds / 1e9
        tr, q = t0.transform.translation, t0.transform.rotation
        self._gt_buf.append((t_s, (tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)))
        trim_gt_buffer(self._gt_buf, t_s)

    def _cloud_cb(self, cloud):
        try:
            now = self.get_clock().now()
            t_cloud = Time.from_msg(cloud.header.stamp).nanoseconds / 1e9
            if (now.nanoseconds / 1e9) - t_cloud > STALE_S:
                self._stale_dropped += 1
                return
            paired = nearest_gt_sample(self._gt_buf, t_cloud)
            if paired is None:
                self._unpaired += 1
                return
            T = compose_map_sensor(*paired[1])
            pts = point_cloud2.read_points_numpy(
                cloud, field_names=('x', 'y', 'z'), skip_nans=False)
            added = self._map.add_cloud(pts, T)
            now_s = now.nanoseconds / 1e9
            if should_publish(self._last_log_s, now_s, added, 1.0):
                self._last_log_s = now_s
                self.get_logger().info(
                    'GTCLOUD voxels=%d stale_dropped=%d unpaired=%d'
                    % (len(self._map), self._stale_dropped, self._unpaired))
                if (self._last_topic_pub_s is None
                        or now_s - self._last_topic_pub_s >= TOPIC_PUB_MIN_PERIOD_S):
                    self.pub.publish(self._map.to_pointcloud2(
                        frame_id='map', stamp=now.to_msg()))
                    self._last_topic_pub_s = now_s
        except Exception as e:                 # eval must never crash or spam
            self.get_logger().error('gt cloud frame dropped: %r' % e,
                                    throttle_duration_sec=5.0)

    def dump(self):
        if not self.dump_dir:
            return
        try:
            pc = self._map.to_pointcloud2(frame_id='map')
            pts = point_cloud2.read_points_numpy(
                pc, field_names=('x', 'y', 'z'), skip_nans=False)
            path = os.path.join(self.dump_dir, 'gt_cloud_map.npy')
            np.save(path, pts.astype(np.float32))
            self.get_logger().info('GTCLOUD dumped %d voxels -> %s'
                                   % (len(pts), path))
        except Exception as e:
            self.get_logger().error('GTCLOUD dump failed: %r' % e)


def main(args=None):
    rclpy.init(args=args)
    node = GtCloudAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.dump()                            # SIGINT teardown path saves the map
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
