import math
import struct

from sensor_msgs.msg import PointCloud2, PointField
from tugbot_maze.footprint import SCAN_OFFSET_X
from tugbot_maze.scatter_cloud import ScatterCloud


def _decode_xy(msg):
    """Unpack the (x, y) of every point from a PointCloud2 (xyz float32 layout)."""
    data = bytes(msg.data)
    out = []
    for i in range(msg.width):
        x, y, _z = struct.unpack_from('<fff', data, i * msg.point_step)
        out.append((x, y))
    return out


def test_forward_beam_endpoint_includes_scan_offset():
    # Pose at origin facing +x; one beam straight ahead (ang=0), range 3.0 m.
    sc = ScatterCloud(voxel_m=0.05)
    sc.add_scan((0.0, 0.0, 0.0), [3.0], angle_min=0.0, angle_inc=0.1)
    pts = _decode_xy(sc.to_pointcloud2())
    assert len(pts) == 1
    px, py = pts[0]
    assert abs(px - (SCAN_OFFSET_X + 3.0)) <= sc.voxel_m
    assert abs(py - 0.0) <= sc.voxel_m


def test_projection_applies_pose_rotation_and_translation():
    # Pose (10, 5) facing +y (yaw=pi/2); beam straight ahead range 2.0 m.
    # base endpoint (SCAN_OFFSET_X+2.0, 0) rotated +90 and translated -> (10, 5 + SCAN_OFFSET_X + 2.0)
    sc = ScatterCloud(voxel_m=0.05)
    sc.add_scan((10.0, 5.0, math.pi / 2), [2.0], angle_min=0.0, angle_inc=0.1)
    (px, py), = _decode_xy(sc.to_pointcloud2())
    assert abs(px - 10.0) <= sc.voxel_m
    assert abs(py - (5.0 + SCAN_OFFSET_X + 2.0)) <= sc.voxel_m


def test_two_close_points_collapse_to_one_voxel():
    # Two beams at the same angle, endpoints ~1 cm apart -> one 5 cm voxel.
    sc = ScatterCloud(voxel_m=0.05)
    sc.add_scan((0.0, 0.0, 0.0), [1.000, 1.010], angle_min=0.0, angle_inc=0.0)
    assert len(sc) == 1


def test_accumulation_across_frames_is_union():
    sc = ScatterCloud(voxel_m=0.05)
    a = sc.add_scan((0.0, 0.0, 0.0), [1.0], angle_min=0.0, angle_inc=0.0)
    b = sc.add_scan((0.0, 0.0, 0.0), [5.0], angle_min=0.0, angle_inc=0.0)
    assert a == 1 and b == 1
    assert len(sc) == 2
    c = sc.add_scan((0.0, 0.0, 0.0), [1.0], angle_min=0.0, angle_inc=0.0)  # repeat -> nothing new
    assert c == 0
    assert len(sc) == 2


def test_invalid_beams_produce_no_points():
    sc = ScatterCloud(voxel_m=0.05, usable_range_m=8.0)
    added = sc.add_scan(
        (0.0, 0.0, 0.0),
        [float('inf'), float('nan'), -1.0, 0.0, 100.0],  # inf, nan, negative, zero, beyond usable range
        angle_min=0.0, angle_inc=0.1)
    assert added == 0
    assert len(sc) == 0


def test_pointcloud2_message_is_wellformed():
    sc = ScatterCloud(voxel_m=0.05)
    sc.add_scan((0.0, 0.0, 0.0), [1.0, 5.0], angle_min=0.0, angle_inc=0.5)
    msg = sc.to_pointcloud2(frame_id='map')
    assert isinstance(msg, PointCloud2)
    assert msg.header.frame_id == 'map'
    assert msg.height == 1
    assert msg.width == len(sc)
    assert msg.point_step == 12
    assert msg.row_step == 12 * msg.width
    assert msg.is_dense is True
    assert msg.is_bigendian is False
    assert [f.name for f in msg.fields] == ['x', 'y', 'z']
    assert all(f.datatype == PointField.FLOAT32 for f in msg.fields)
    assert len(bytes(msg.data)) == 12 * msg.width
    assert len(_decode_xy(msg)) == msg.width


def test_empty_cloud_is_valid():
    sc = ScatterCloud(voxel_m=0.05)
    msg = sc.to_pointcloud2(frame_id='map')
    assert msg.width == 0
    assert bytes(msg.data) == b''
    assert msg.row_step == 0
