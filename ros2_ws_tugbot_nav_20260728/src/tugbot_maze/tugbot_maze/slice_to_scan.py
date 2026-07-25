"""Pure projection core: 3D lidar cloud -> the legacy 2D /scan geometry.

The 16-ring +/-15deg lidar has rings every 2deg (+/-1, +/-3, ... +/-15);
we keep only points whose per-point vertical angle is within the two
+/-1deg rings (RING_BAND_RAD) and min-fold them into the legacy 900
azimuth bins. Per-point vertical angle (atan2(z, hypot(x,y))) avoids any
assumption about the cloud's row/column organization. Empty bins are
+inf, matching the observed legacy encoding (scan_omni_baseline.json:
564 inf / 0 nan). SCAN_CONTRACT values are copied verbatim from that
captured fixture; test_contract_matches_captured_baseline pins them.
"""
import math

import numpy as np

RING_BAND_RAD = math.radians(1.1)   # accepts exactly the +/-1deg rings, rejects +/-3deg

SCAN_CONTRACT = {
    'n_bins': 900,
    'angle_min': -3.1415927410125732,
    'angle_max': 3.1415927410125732,
    'angle_increment': 0.006989082787185907,
    'time_increment': 0.0,
    'scan_time': 0.0,
    'range_min': 0.20000000298023224,
    'range_max': 100.0,
}


def fold_to_ranges(points_xyz, n_bins, angle_min, angle_increment,
                   range_min, range_max, band_rad=RING_BAND_RAD):
    """points_xyz: (N,3) array-like in the sensor frame. Returns a list of
    n_bins ranges: min-folded per azimuth bin, +inf where no return."""
    xyz = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
    ranges = np.full(n_bins, np.inf)
    if xyz.size:
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if xyz.size:
        r_h = np.hypot(xyz[:, 0], xyz[:, 1])
        keep = r_h > 0.0
        xyz, r_h = xyz[keep], r_h[keep]
        keep = np.abs(np.arctan2(xyz[:, 2], r_h)) <= band_rad
        xyz, r_h = xyz[keep], r_h[keep]
        r = np.hypot(r_h, xyz[:, 2])
        keep = (r >= range_min) & (r <= range_max)
        xyz, r = xyz[keep], r[keep]
        if r.size:
            az = np.arctan2(xyz[:, 1], xyz[:, 0])
            idx = np.rint((az - angle_min) / angle_increment).astype(np.int64) % n_bins
            np.minimum.at(ranges, idx, r)
    return ranges.tolist()
