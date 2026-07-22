"""Pure projection core tests: synthetic 3D points -> 900-bin ranges.
Channel selection is by per-point vertical angle (no cloud-organization
assumption); the 16-ring +/-15deg lidar has rings at +/-1deg, +/-3deg, ...
so RING_BAND_RAD must accept exactly the two +/-1deg rings."""
import json
import math
from pathlib import Path

import pytest

from tugbot_maze.slice_to_scan import RING_BAND_RAD, SCAN_CONTRACT, fold_to_ranges


def _contract_kwargs():
    c = SCAN_CONTRACT
    return dict(n_bins=c['n_bins'], angle_min=c['angle_min'],
                angle_increment=c['angle_increment'],
                range_min=c['range_min'], range_max=c['range_max'])


def test_contract_matches_captured_baseline():
    fixture = json.loads((Path(__file__).parent / 'scan_omni_baseline.json').read_text())
    for key in ('n_bins', 'angle_min', 'angle_max', 'angle_increment',
                'time_increment', 'scan_time', 'range_min', 'range_max'):
        assert SCAN_CONTRACT[key] == pytest.approx(fixture[key], abs=1e-9), key


def test_ring_band_accepts_only_the_two_horizontal_rings():
    ring_angles = [math.radians(-15 + 2 * k) for k in range(16)]
    inside = [a for a in ring_angles if abs(a) <= RING_BAND_RAD]
    assert len(inside) == 2
    assert inside == pytest.approx([math.radians(-1), math.radians(1)])


def test_wall_point_lands_in_correct_bin_with_min_fold():
    kw = _contract_kwargs()
    az = 0.5
    z1 = 3.0 * math.tan(math.radians(1))   # on the +1deg ring at 3m horizontal
    pts = [(3.0 * math.cos(az), 3.0 * math.sin(az), z1),
           (5.0 * math.cos(az), 5.0 * math.sin(az), 5.0 * math.tan(math.radians(-1)))]
    ranges = fold_to_ranges(pts, **kw)
    idx = round((az - kw['angle_min']) / kw['angle_increment']) % kw['n_bins']
    assert ranges[idx] == pytest.approx(math.hypot(3.0, z1), abs=1e-6)  # min of the two
    assert sum(1 for r in ranges if math.isfinite(r)) == 1


def test_out_of_band_and_out_of_range_points_are_dropped():
    kw = _contract_kwargs()
    pts = [
        (2.0, 0.0, 2.0 * math.tan(math.radians(5))),    # 5deg ring: outside band
        (2.0, 0.0, -2.0 * math.tan(math.radians(15))),  # floor-seeking ring
        (0.05, 0.0, 0.0),                               # below range_min
        (150.0, 0.0, 0.0),                              # beyond range_max
        (0.0, 0.0, 1.0),                                # degenerate: zero horizontal
        (float('inf'), 0.0, 0.0),                       # non-finite x
        (2.0, 0.0, float('nan')),                       # non-finite z
    ]
    ranges = fold_to_ranges(pts, **kw)
    assert all(math.isinf(r) for r in ranges)


def test_empty_bins_are_positive_inf():
    kw = _contract_kwargs()
    ranges = fold_to_ranges([], **kw)
    assert len(ranges) == kw['n_bins']
    assert all(math.isinf(r) and r > 0 for r in ranges)


def test_pi_boundary_wraps_to_a_valid_bin():
    kw = _contract_kwargs()
    pts = [(-4.0, -1e-9, 0.0)]   # azimuth ~ -pi
    ranges = fold_to_ranges(pts, **kw)
    assert sum(1 for r in ranges if math.isfinite(r)) == 1


def test_fractional_bin_offset_rounds_to_nearest_not_floor():
    kw = _contract_kwargs()
    az = kw['angle_min'] + 10.6 * kw['angle_increment']   # rint -> 11, floor -> 10
    pts = [(3.0 * math.cos(az), 3.0 * math.sin(az), 0.0)]
    ranges = fold_to_ranges(pts, **kw)
    idx = round((az - kw['angle_min']) / kw['angle_increment'])
    assert idx == 11
    assert ranges[idx] == pytest.approx(3.0, abs=1e-6)
    assert sum(1 for r in ranges if math.isfinite(r)) == 1


def test_pi_azimuth_wraps_via_modulo_in_synthetic_geometry():
    # NOT the captured contract: with a float64 -pi angle_min and a 2*pi/8
    # increment, an azimuth just below +pi yields raw index 8 (rint of
    # ~7.9999997), which only the % n_bins wrap folds into bin 0. The
    # captured float32 contract angles are slightly wider than +/-pi, so
    # they never exercise the wrap.
    n_bins = 8
    ranges = fold_to_ranges([(-5.0, 1e-6, 0.0)], n_bins=n_bins,
                            angle_min=-math.pi,
                            angle_increment=2 * math.pi / n_bins,
                            range_min=0.1, range_max=50.0)
    assert len(ranges) == n_bins
    assert math.isfinite(ranges[0])
    assert ranges[0] == pytest.approx(5.0, abs=1e-6)
    assert sum(1 for r in ranges if math.isfinite(r)) == 1
