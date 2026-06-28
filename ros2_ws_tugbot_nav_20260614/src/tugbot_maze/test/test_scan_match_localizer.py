import math
import numpy as np
from tugbot_maze.scan_match_localizer import ScanMatchLocalizer


def test_init_builds_unit_normals_perpendicular_to_edges():
    # one horizontal wall (along x), one vertical wall (along y)
    segs = [(-1.0, 2.0, 1.0, 2.0), (3.0, -1.0, 3.0, 1.0)]
    loc = ScanMatchLocalizer(segs, scan_offset_x=0.0)
    assert loc._n.shape == (2, 2)
    # unit length
    assert np.allclose(np.hypot(loc._n[:, 0], loc._n[:, 1]), 1.0)
    # perpendicular to edges (dot with edge == 0)
    assert np.allclose(np.sum(loc._n * loc._e, axis=1), 0.0, atol=1e-9)


def test_beams_to_points_places_endpoint_in_map_frame():
    loc = ScanMatchLocalizer([(0.0, 0.0, 1.0, 0.0)], scan_offset_x=0.0, usable_range_m=10.0)
    # robot at origin facing +x; two beams at angles 0 and pi/2, range 2.0
    pts = loc._beams_to_points((0.0, 0.0, 0.0), [2.0, 2.0], 0.0, math.pi / 2)
    # beam 0 at angle 0 -> (2,0); beam 1 at angle pi/2 -> (0,2)
    assert np.allclose(pts[0], [2.0, 0.0], atol=1e-9)
    assert np.allclose(pts[1], [0.0, 2.0], atol=1e-9)


def test_beams_to_points_applies_scan_offset_and_pose():
    loc = ScanMatchLocalizer([(0.0, 0.0, 1.0, 0.0)], scan_offset_x=-0.1855, usable_range_m=10.0)
    # robot at (5, 5) facing +y (yaw=pi/2); sensor sits 0.1855 behind center -> at (5, 4.8145)
    # a beam at angle 0 (sensor x = body forward = +y in map), range 1.0 -> endpoint (5, 5.8145)
    pts = loc._beams_to_points((5.0, 5.0, math.pi / 2), [1.0], 0.0, math.pi / 2)
    assert np.allclose(pts[0], [5.0, 4.8145 + 1.0], atol=1e-6)


def test_associate_finds_nearest_segment_and_robot_facing_normal():
    loc = ScanMatchLocalizer([(-5.0, 2.0, 5.0, 2.0)], scan_offset_x=0.0)  # wall at y=2
    pts = np.array([[0.0, 1.8]])         # point just below the wall (robot side y<2)
    foot, n, dist = loc._associate(pts)
    assert np.allclose(foot[0], [0.0, 2.0], atol=1e-9)
    assert abs(dist[0] - 0.2) < 1e-9
    # normal must point toward the robot point (negative y here)
    assert n[0, 1] < 0


def test_beams_to_points_applies_beam_stride():
    loc = ScanMatchLocalizer([(0.0, 0.0, 1.0, 0.0)], scan_offset_x=0.0,
                             usable_range_m=10.0, beam_stride=2)
    # 4 beams at angles 0, pi/2, pi, 3pi/2; stride 2 keeps indices [0, 2]
    pts = loc._beams_to_points((0.0, 0.0, 0.0), [2.0, 2.0, 2.0, 2.0],
                               0.0, math.pi / 2)
    assert pts.shape == (2, 2)
    assert np.allclose(pts[0], [2.0, 0.0], atol=1e-9)   # index 0, angle 0
    assert np.allclose(pts[1], [-2.0, 0.0], atol=1e-9)  # index 2, angle pi


def test_associate_empty_returns_distinct_unaliased_arrays():
    loc = ScanMatchLocalizer([(0.0, 0.0, 1.0, 0.0)], scan_offset_x=0.0)
    foot, n, dist = loc._associate(np.empty((0, 2)))
    assert foot.shape == (0, 2) and n.shape == (0, 2) and dist.shape == (0,)
    # no segments but points present -> dist all inf, foot/normal must be
    # distinct arrays (not aliased) so in-place ICP centering cannot corrupt
    loc2 = ScanMatchLocalizer([], scan_offset_x=0.0)
    foot2, n2, dist2 = loc2._associate(np.array([[1.0, 2.0]]))
    assert foot2 is not n2
    assert np.all(np.isinf(dist2))
