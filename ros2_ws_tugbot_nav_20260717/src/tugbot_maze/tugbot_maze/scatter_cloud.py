"""Accumulated LIDAR scatter cloud for the online self-built map: project each valid
scan return into the map frame (using the exact same projection the ScanMatchLocalizer
uses -- SCAN_OFFSET_X plus a pose rotate/translate), voxel-dedup the map points into an
in-memory set that grows over the run, and export the accumulation as a PointCloud2.
Pure NumPy + std msgs -- no ROS node state, no rclpy.init needed."""
from __future__ import annotations
import math

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from tugbot_maze.footprint import SCAN_OFFSET_X   # DRY: the one true LIDAR mount offset


class ScatterCloud:
    def __init__(self, voxel_m: float = 0.05,
                 scan_offset_x: float = SCAN_OFFSET_X,
                 usable_range_m: float = 8.0) -> None:
        self.voxel_m = float(voxel_m)
        self.scan_offset_x = float(scan_offset_x)
        self.usable_range_m = float(usable_range_m)
        self._voxels: set[tuple[int, int]] = set()

    def add_scan(self, pose, ranges, angle_min, angle_inc) -> int:
        """Project one scan to the map frame and merge its voxel-snapped endpoints into
        the accumulated set. `pose` is the map->base_link (x, y, yaw). Returns the number
        of NEW voxels added by this scan. Mirrors ScanMatchLocalizer._beams_to_points."""
        r = np.asarray(ranges, dtype=float)
        if r.size == 0:
            return 0
        ang = angle_min + np.arange(r.shape[0], dtype=float) * angle_inc   # sensor==base_link axes
        valid = np.isfinite(r) & (r > 0.0) & (r <= self.usable_range_m)
        r, ang = r[valid], ang[valid]
        if r.size == 0:
            return 0
        bx = self.scan_offset_x + r * np.cos(ang)                          # endpoint in base_link
        by = r * np.sin(ang)
        x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
        c, s = math.cos(th), math.sin(th)
        px = x + c * bx - s * by                                           # base_link -> map
        py = y + s * bx + c * by
        kx = np.round(px / self.voxel_m).astype(np.int64)
        ky = np.round(py / self.voxel_m).astype(np.int64)
        before = len(self._voxels)
        self._voxels.update(zip(kx.tolist(), ky.tolist()))
        return len(self._voxels) - before

    def to_pointcloud2(self, frame_id: str = 'map', stamp=None) -> PointCloud2:
        """Export the accumulated voxel set as an xyz-float32 PointCloud2 (z=0).
        Point coords are the voxel keys mapped back to map metres (kx*voxel_m, ky*voxel_m),
        the strict inverse of the round(coord/voxel_m) snapping in add_scan."""
        msg = PointCloud2()
        msg.header.frame_id = frame_id
        if stamp is not None:
            msg.header.stamp = stamp
        msg.height = 1
        msg.width = len(self._voxels)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.is_dense = True
        if self._voxels:
            keys = np.array(sorted(self._voxels), dtype=np.float64)        # (N,2) deterministic order
            xyz = np.zeros((keys.shape[0], 3), dtype='<f4')
            xyz[:, 0] = keys[:, 0] * self.voxel_m
            xyz[:, 1] = keys[:, 1] * self.voxel_m
            msg.data = xyz.tobytes()
        else:
            msg.data = b''
        return msg

    def __len__(self) -> int:
        return len(self._voxels)
