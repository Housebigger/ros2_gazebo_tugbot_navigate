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


import pytest
from tugbot_maze.maze_sim import MazeSim, load_segments
from tugbot_maze.scan_match_localizer import _wrap

_SCAN_KW = dict(n_beams=360, fov_rad=2 * math.pi)


def _scan_at(segs, pose):
    sim = MazeSim(segs, (pose[0], pose[1]), pose[2])
    return sim.scan(**_SCAN_KW)


# Corridor poses (cell centers) with clear side walls and good observability.
_RECOVERY_POSES = [(2.0, 2.0, math.pi / 2), (2.0, 6.0, math.pi / 2),
                   (8.0, 2.0, 0.0), (4.0, 8.0, 0.0)]


@pytest.mark.parametrize("true_pose", _RECOVERY_POSES)
def test_recovers_true_pose_under_prior_drift(true_pose):
    segs = load_segments()
    loc = ScanMatchLocalizer(segs, scan_offset_x=0.0)
    ranges, amin, ainc = _scan_at(segs, true_pose)
    prior = (true_pose[0] + 0.35, true_pose[1] - 0.25, true_pose[2] + math.radians(7))
    est, info = loc.correct(prior, ranges, amin, ainc)
    assert not info["rejected"], f"unexpectedly rejected at {true_pose}: {info}"
    assert math.hypot(est[0] - true_pose[0], est[1] - true_pose[1]) < 0.05, info
    assert abs(_wrap(est[2] - true_pose[2])) < math.radians(2), info


def test_no_half_thickness_bias_at_true_pose():
    segs = load_segments()
    loc = ScanMatchLocalizer(segs, scan_offset_x=0.0)
    true_pose = (2.0, 2.0, math.pi / 2)
    ranges, amin, ainc = _scan_at(segs, true_pose)
    est, _ = loc.correct(true_pose, ranges, amin, ainc)   # prior == truth
    assert math.hypot(est[0] - true_pose[0], est[1] - true_pose[1]) < 0.02


def test_open_axis_falls_back_to_prior():
    # infinite-feeling N-S corridor: walls only on the x axis -> y unobservable
    walls = [(-1.0, -5.0, -1.0, 5.0), (1.0, -5.0, 1.0, 5.0)]
    loc = ScanMatchLocalizer(walls, scan_offset_x=0.0, usable_range_m=8.0)
    ranges, amin, ainc = _scan_at(walls, (0.0, 0.0, math.pi / 2))
    prior = (0.3, 0.4, math.pi / 2)                  # x off (constrained), y off (free)
    est, info = loc.correct(prior, ranges, amin, ainc)
    assert "y" in info["fell_back"], info
    assert abs(est[0] - 0.0) < 0.05, f"x not corrected: {est}"
    assert abs(est[1] - 0.4) < 0.05, f"y should stay at prior: {est}"


def test_rejects_correction_beyond_clamp():
    segs = load_segments()
    loc = ScanMatchLocalizer(segs, scan_offset_x=0.0, trans_clamp_m=0.5)
    true_pose = (2.0, 2.0, math.pi / 2)
    ranges, amin, ainc = _scan_at(segs, true_pose)
    prior = (true_pose[0] + 1.2, true_pose[1], true_pose[2])   # 1.2 m > clamp
    est, info = loc.correct(prior, ranges, amin, ainc)
    assert info["rejected"]
    assert est == prior                                       # no wild jump


def test_robust_to_outlier_returns():
    segs = load_segments()
    loc = ScanMatchLocalizer(segs, scan_offset_x=0.0)
    true_pose = (8.0, 2.0, 0.0)
    ranges, amin, ainc = _scan_at(segs, true_pose)
    ranges = list(ranges)
    for k in range(0, len(ranges), 11):                       # ~9% spurious short returns
        ranges[k] = 0.4
    prior = (true_pose[0] + 0.3, true_pose[1] - 0.2, true_pose[2] + math.radians(5))
    est, info = loc.correct(prior, ranges, amin, ainc)
    assert math.hypot(est[0] - true_pose[0], est[1] - true_pose[1]) < 0.08, info
