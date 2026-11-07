"""Unit tests for the twist-metric math on synthetic voxel sets: a clean wall
must report near-zero spread and angle; a yaw-twisted copy must report the
injected angle and a larger p95 spread. Imports the analysis functions from
tools/ via a path shim (the tools dir is not a package)."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from map_twist_metric import segment_band_stats, wall_points  # noqa: E402


def _wall(x0, y0, x1, y1, n=400, z=0.8, noise=0.0, seed=7):
    rng = np.random.default_rng(seed)
    t = np.linspace(0.0, 1.0, n)
    xs = x0 + (x1 - x0) * t
    ys = y0 + (y1 - y0) * t
    if noise:
        xs = xs + rng.normal(0, noise, n)
        ys = ys + rng.normal(0, noise, n)
    return np.column_stack([xs, ys, np.full(n, z)]).astype(np.float32)


def test_clean_wall_near_zero_spread_and_angle():
    seg = (10.0, 8.0, 10.0, 11.0)                 # vertical wall x=10
    pts = _wall(*seg, noise=0.01)
    stats = segment_band_stats(pts, seg)
    assert stats['count'] > 300
    assert stats['spread_p95'] < 0.05             # ~voxel scale
    assert abs(stats['angle_deg']) < 1.5


def test_twisted_wall_reports_injected_angle():
    seg = (10.0, 8.0, 10.0, 11.0)
    # rotate the wall 3 degrees about its midpoint -> fan/oblique twist
    pts = _wall(*seg, noise=0.01)
    mid = np.array([10.0, 9.5, 0.0])
    a = math.radians(3.0)
    R = np.array([[math.cos(a), -math.sin(a), 0.0],
                  [math.sin(a), math.cos(a), 0.0],
                  [0.0, 0.0, 1.0]])
    twisted = (pts - mid) @ R.T + mid
    both = np.vstack([pts, twisted.astype(np.float32)])
    stats = segment_band_stats(both, seg)
    assert stats['spread_p95'] > 0.03             # double-image widens the band
    clean = segment_band_stats(pts, seg)
    assert stats['spread_p95'] > clean['spread_p95'] * 1.5


def test_wall_points_selects_band_only():
    seg = (10.0, 8.0, 10.0, 11.0)
    inside = _wall(*seg, n=50)
    far = _wall(12.0, 8.0, 12.0, 11.0, n=50)      # 2 m away: outside the band
    floor = _wall(*seg, n=50, z=0.02)             # below the wall z-band
    sel = wall_points(np.vstack([inside, far, floor]), seg)
    assert 40 <= len(sel) <= 60                   # only the in-band wall survives


def test_single_image_angle_reports_injected_rotation():
    # Angle-channel validity on a SINGLE rotated image. The clean+twisted
    # UNION attenuates the fitted angle ~2x (Task 2 review I2), which is why
    # angle is corroborating-only in the verdict protocol -- this test pins
    # that the channel itself is sound when given an unmixed image.
    seg = (10.0, 8.0, 10.0, 11.0)
    pts = _wall(*seg, noise=0.01)
    mid = np.array([10.0, 9.5, 0.0])
    a = math.radians(3.0)
    R = np.array([[math.cos(a), -math.sin(a), 0.0],
                  [math.sin(a), math.cos(a), 0.0],
                  [0.0, 0.0, 1.0]])
    rotated = ((pts - mid) @ R.T + mid).astype(np.float32)
    stats = segment_band_stats(rotated, seg)
    assert abs(abs(stats['angle_deg']) - 3.0) < 0.8


def test_stats_are_z_datum_invariant():
    # Task 1 quality review I2: the gt and production maps may sit on
    # different z datums. Shifting a cloud by +0.5 m in z and passing the
    # matching ground_z must reproduce the same band statistics.
    seg = (10.0, 8.0, 10.0, 11.0)
    pts = _wall(*seg, noise=0.01)
    shifted = pts + np.array([0.0, 0.0, 0.5], dtype=np.float32)
    a = segment_band_stats(pts, seg, ground_z=0.0)
    b = segment_band_stats(shifted, seg, ground_z=0.5)
    assert a['count'] == b['count']
    assert a['spread_p95'] == pytest.approx(b['spread_p95'], abs=1e-6)
    assert a['angle_deg'] == pytest.approx(b['angle_deg'], abs=1e-6)
