"""Scan-to-known-map localization: point-to-line ICP against the maze wall map.

Given a prior pose (from odom propagation) and a LIDAR scan, returns an absolute
map-frame pose by aligning the scan endpoints to the known wall centerline
segments. Pure NumPy, ROS-free, no external deps.
See docs/superpowers/specs/2026-06-28-scan-match-localization-design.md.
"""
from __future__ import annotations
import math
from typing import Dict, Tuple

import numpy as np

from tugbot_maze.footprint import SCAN_OFFSET_X

Pose2D = Tuple[float, float, float]


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class ScanMatchLocalizer:
    def __init__(self, segments, *,
                 scan_offset_x: float = SCAN_OFFSET_X,
                 wall_half_thickness_m: float = 0.12,
                 usable_range_m: float = 8.0,
                 beam_stride: int = 1,
                 max_corr_dist_m: float = 0.30,
                 max_iters: int = 6,
                 tol_m: float = 0.002,
                 trans_clamp_m: float = 0.5,
                 yaw_clamp_rad: float = math.radians(10.0),
                 min_inliers: int = 20,
                 min_eig: float = 5.0,
                 min_yaw_info: float = 3.0,
                 lm_lambda: float = 1e-3):
        segs = np.asarray(segments, dtype=float).reshape(-1, 4)
        self._a = segs[:, 0:2]                                  # (S,2) endpoint A
        self._e = segs[:, 2:4] - segs[:, 0:2]                   # (S,2) edge A->B
        self._len2 = np.maximum(np.sum(self._e * self._e, axis=1), 1e-12)  # (S,)
        nx, ny = -self._e[:, 1], self._e[:, 0]                  # perpendicular to edge
        nlen = np.maximum(np.hypot(nx, ny), 1e-12)
        self._n = np.stack([nx / nlen, ny / nlen], axis=1)     # (S,2) unit normal
        self.scan_offset_x = float(scan_offset_x)
        self.wall_half_thickness_m = float(wall_half_thickness_m)
        self.usable_range_m = float(usable_range_m)
        self.beam_stride = max(1, int(beam_stride))
        self.max_corr_dist_m = float(max_corr_dist_m)
        self.max_iters = int(max_iters)
        self.tol_m = float(tol_m)
        self.trans_clamp_m = float(trans_clamp_m)
        self.yaw_clamp_rad = float(yaw_clamp_rad)
        self.min_inliers = int(min_inliers)
        self.min_eig = float(min_eig)
        self.min_yaw_info = float(min_yaw_info)
        self.lm_lambda = float(lm_lambda)

    def _beams_to_points(self, pose: Pose2D, ranges, angle_min, angle_inc) -> np.ndarray:
        """Return (N,2) map-frame endpoints of valid beams (finite, range in usable window)."""
        r = np.asarray(ranges, dtype=float)
        idx = np.arange(0, r.shape[0], self.beam_stride)
        r = r[idx]
        ang = angle_min + idx.astype(float) * angle_inc        # sensor==base_link axes
        valid = np.isfinite(r) & (r > 0.0) & (r <= self.usable_range_m)
        r, ang = r[valid], ang[valid]
        bx = self.scan_offset_x + r * np.cos(ang)              # endpoint in base_link
        by = r * np.sin(ang)
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        px = x + c * bx - s * by                              # base_link -> map
        py = y + s * bx + c * by
        return np.stack([px, py], axis=1)

    def _associate(self, pts: np.ndarray):
        """Nearest segment per point. Return (foot (N,2), normal (N,2) robot-facing, dist (N,))."""
        n_pts = pts.shape[0]
        if self._a.shape[0] == 0 or n_pts == 0:
            return (np.zeros((n_pts, 2)), np.zeros((n_pts, 2)),
                    np.full(n_pts, np.inf))
        ap = pts[:, None, :] - self._a[None, :, :]            # (N,S,2)
        e = self._e[None, :, :]                               # (1,S,2)
        t = np.clip(np.sum(ap * e, axis=2) / self._len2[None, :], 0.0, 1.0)  # (N,S)
        foot = self._a[None, :, :] + t[:, :, None] * e        # (N,S,2)
        diff = pts[:, None, :] - foot                         # (N,S,2)
        d2 = np.sum(diff * diff, axis=2)                      # (N,S)
        j = np.argmin(d2, axis=1)                            # (N,)
        rows = np.arange(n_pts)
        foot_j = foot[rows, j]                                # (N,2)
        dist = np.sqrt(d2[rows, j])                           # (N,)
        n_j = self._n[j]                                      # (N,2)
        sign = np.sign(np.sum((pts - foot_j) * n_j, axis=1))  # orient toward robot point
        sign[sign == 0] = 1.0
        return foot_j, n_j * sign[:, None], dist
