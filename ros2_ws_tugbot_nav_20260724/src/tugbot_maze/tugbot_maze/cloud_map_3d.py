"""Map-frame 3D voxel accumulation for the cloud-map-3d viz: filter a sensor-frame
lidar cloud (finite, range<=usable), transform by a 4x4 map<-sensor matrix, clip
to a sane map-frame z band (the odom frame is anchored at the spawn pose z=0.62,
so the GROUND sits near map z=-0.62 -- z_min must stay below it), and dedup into
an integer voxel set exported as an xyz-float32 PointCloud2. Pure NumPy + msg
construction -- no ROS node state, no rclpy.init needed (mirrors scatter_cloud's
old contract, generalized to 3D)."""
from __future__ import annotations

import itertools

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def transform_to_matrix(tx: float, ty: float, tz: float,
                        qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """4x4 homogeneous map<-sensor matrix from a TF translation + quaternion."""
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
                 [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
                 [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]]
    T[:3, 3] = (tx, ty, tz)
    return T


def should_publish(last_pub_s, now_s: float, added: int, period_s: float) -> bool:
    """First frame always publishes; afterwards only when the map grew AND the
    throttle period elapsed (the old scatter-cloud cadence contract: a growth
    inside the throttle window is published by the NEXT growth's full-set
    publish, so nothing is lost, cadence just lags)."""
    if last_pub_s is None:
        return True
    return added > 0 and (now_s - last_pub_s) >= period_s


class CloudMap3D:
    def __init__(self, voxel_m: float = 0.05, usable_range_m: float = 8.0,
                 z_min: float = -1.0, z_max: float = 3.0) -> None:
        self.voxel_m = float(voxel_m)
        self.usable_range_m = float(usable_range_m)
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self._voxels: set[tuple[int, int, int]] = set()

    def add_cloud(self, points_xyz, T_map_sensor) -> int:
        """Merge one sensor-frame cloud into the accumulated map-frame voxel set.
        Returns the number of NEW voxels added."""
        p = np.asarray(points_xyz, dtype=float).reshape(-1, 3)
        if p.size == 0:
            return 0
        p = p[np.isfinite(p).all(axis=1)]
        if p.size == 0:
            return 0
        rng = np.linalg.norm(p, axis=1)
        p = p[(rng > 0.0) & (rng <= self.usable_range_m)]
        if p.size == 0:
            return 0
        T = np.asarray(T_map_sensor, dtype=float)
        m = p @ T[:3, :3].T + T[:3, 3]
        m = m[(m[:, 2] >= self.z_min) & (m[:, 2] <= self.z_max)]
        if m.size == 0:
            return 0
        keys = np.round(m / self.voxel_m).astype(np.int64)
        before = len(self._voxels)
        self._voxels.update(map(tuple, keys.tolist()))
        return len(self._voxels) - before

    def to_pointcloud2(self, frame_id: str = 'map', stamp=None) -> PointCloud2:
        """Export the voxel set as xyz-float32 (keys * voxel_m, deterministic order)."""
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
            arr = np.fromiter(itertools.chain.from_iterable(self._voxels),
                              dtype=np.int64, count=3 * len(self._voxels)).reshape(-1, 3)
            keys = arr[np.lexsort((arr[:, 2], arr[:, 1], arr[:, 0]))].astype(np.float64)
            msg.data = (keys * self.voxel_m).astype('<f4').tobytes()
        else:
            msg.data = b''
        return msg

    def __len__(self) -> int:
        return len(self._voxels)
