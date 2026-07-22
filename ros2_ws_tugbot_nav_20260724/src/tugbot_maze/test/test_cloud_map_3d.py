"""CloudMap3D: map-frame 3D voxel accumulation. Pure NumPy + msg construction,
pure-msg test conventions (no rclpy.init needed; style inherited from the
removed 2D scatter tests)."""
import math
import struct

import numpy as np

from tugbot_maze.cloud_map_3d import CloudMap3D, should_publish, transform_to_matrix


def _decode_xyz(msg):
    data = bytes(msg.data)
    out = []
    for i in range(msg.width):
        out.append(struct.unpack_from('<fff', data, i * msg.point_step))
    return out


def _quat_yaw(t):
    return (0.0, 0.0, math.sin(t / 2.0), math.cos(t / 2.0))


def _quat_pitch(t):
    return (0.0, math.sin(t / 2.0), 0.0, math.cos(t / 2.0))


def test_transform_identity_translation():
    T = transform_to_matrix(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    p = T @ np.array([0.5, 0.0, 0.0, 1.0])
    assert np.allclose(p[:3], (1.5, 2.0, 3.0))


def test_transform_yaw_90():
    qx, qy, qz, qw = _quat_yaw(math.pi / 2.0)
    T = transform_to_matrix(0.0, 0.0, 0.0, qx, qy, qz, qw)
    p = T @ np.array([1.0, 0.0, 0.0, 1.0])
    assert np.allclose(p[:3], (0.0, 1.0, 0.0), atol=1e-9)


def test_transform_pitch_moves_forward_point_down():
    # +pitch (about +y) rotates a forward point toward -z: the trot pitch-compensation path.
    qx, qy, qz, qw = _quat_pitch(math.radians(10.0))
    T = transform_to_matrix(0.0, 0.0, 0.0, qx, qy, qz, qw)
    p = T @ np.array([1.0, 0.0, 0.0, 1.0])
    assert np.allclose(p[:3], (math.cos(math.radians(10)), 0.0, -math.sin(math.radians(10))), atol=1e-9)


def test_add_cloud_dedup_same_cloud_twice():
    m = CloudMap3D(voxel_m=0.05)
    pts = np.array([[1.0, 0.0, 0.0], [2.0, 1.0, 0.5]])
    T = np.eye(4)
    a1 = m.add_cloud(pts, T)
    a2 = m.add_cloud(pts, T)
    assert a1 == 2 and a2 == 0 and len(m) == 2


def test_add_cloud_range_filter():
    m = CloudMap3D(voxel_m=0.05, usable_range_m=8.0)
    pts = np.array([[7.0, 0.0, 0.0], [9.0, 0.0, 0.0]])   # sensor-frame norm 7 / 9
    assert m.add_cloud(pts, np.eye(4)) == 1


def test_add_cloud_drops_nonfinite():
    m = CloudMap3D(voxel_m=0.05)
    pts = np.array([[np.inf, 0.0, 0.0], [np.nan, 1.0, 0.0], [1.0, 1.0, 0.0]])
    assert m.add_cloud(pts, np.eye(4)) == 1 and len(m) == 1


def test_add_cloud_empty_input():
    m = CloudMap3D()
    assert m.add_cloud(np.zeros((0, 3)), np.eye(4)) == 0 and len(m) == 0


def test_add_cloud_z_clip_in_map_frame():
    # Ground sits near map z ~ -0.62 (odom anchored at spawn z=0.62): must be KEPT;
    # z < -1.0 and z > 3.0 are clipped.
    m = CloudMap3D(voxel_m=0.05, z_min=-1.0, z_max=3.0)
    pts = np.array([[1.0, 0.0, -0.62], [1.0, 1.0, -1.5], [1.0, 2.0, 3.5]])
    assert m.add_cloud(pts, np.eye(4)) == 1


def test_to_pointcloud2_roundtrip():
    m = CloudMap3D(voxel_m=0.05)
    pts = np.array([[1.02, -0.51, 0.26], [3.99, 2.0, -0.6]])
    m.add_cloud(pts, np.eye(4))
    out = _decode_xyz(m.to_pointcloud2(frame_id='map'))
    assert len(out) == 2
    for orig in pts:
        assert min(max(abs(a - b) for a, b in zip(orig, got)) for got in out) <= 0.05 / 2 + 1e-6


def test_to_pointcloud2_empty():
    msg = CloudMap3D().to_pointcloud2()
    assert msg.width == 0 and bytes(msg.data) == b'' and msg.header.frame_id == 'map'


def test_should_publish_first_frame_always():
    assert should_publish(None, 100.0, 0, 1.0) is True


def test_should_publish_growth_and_period():
    assert should_publish(100.0, 101.5, 5, 1.0) is True     # growth AND period elapsed
    assert should_publish(100.0, 101.5, 0, 1.0) is False    # no growth
    assert should_publish(100.0, 100.4, 5, 1.0) is False    # growth but inside throttle window


def test_add_cloud_frame_contract_with_rotated_translated_T():
    # Range filter is SENSOR-frame (pre-transform); z clip is MAP-frame (post-transform).
    qx, qy, qz, qw = _quat_yaw(math.pi / 2.0)
    T = transform_to_matrix(10.0, 0.0, -0.5, qx, qy, qz, qw)
    m = CloudMap3D(voxel_m=0.05, usable_range_m=8.0, z_min=-1.0, z_max=3.0)
    pts = np.array([
        [7.9, 0.0, 0.0],    # sensor norm 7.9 <= 8 kept, even though map x = 10 (far): sensor-frame filter
        [1.0, 0.0, 0.4],    # map z = 0.4 - 0.5 = -0.1 in [-1, 3]: kept
        [1.0, 0.0, -0.6],   # map z = -0.6 - 0.5 = -1.1 < -1.0: clipped (a pre-transform clip would keep it)
    ])
    assert m.add_cloud(pts, T) == 2
