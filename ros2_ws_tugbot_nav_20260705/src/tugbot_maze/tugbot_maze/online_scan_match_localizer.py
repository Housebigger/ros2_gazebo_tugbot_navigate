"""Online self-built-map localization: scan-match against a GROWING set of confirmed
walls (known perimeter + the flood-fill brain's committed interior walls) instead of a
prior full map. Because the grid is known, a confirmed wall's centerline is grid-snapped
and exact, so the reused ICP still yields an absolute, drift-free pose.
See docs/superpowers/specs/2026-07-05-online-slam-maze-design.md.

Design notes (offline validation, Task 6):
  Two ICP pathologies were diagnosed when the interior reference is sparse:

  1. Perimeter-only false minimum: with zero interior walls, iteration-0 matches
     interior-hitting scan points to the far perimeter, biasing the Jacobian and
     converging to a false minimum.  Gate: skip when interior is empty
     (min_interior_segs=1 default).

  2. Locality drift: even with some committed walls, the ICP drifts when the robot
     moves into new (uncommitted) territory.  Committed walls are from already-visited
     cells; as the robot explores forward the reference walls are increasingly behind it.
     The unfiltered first iteration uses stale far-wall correspondences, producing a
     small per-step bias that compounds.  Gate: skip when no committed interior segment
     has a point within local_radius of the prior (= robot is in unmapped territory →
     use pure odom until the cell is revisited and committed walls surround the robot).

  3. Iteration-0 far-beam bias (residual): even when local walls exist, beams that hit
     unmodeled walls are matched to the nearest reference in iteration 0.  Pre-mask
     such beams to inf before calling ICP so that iteration 0 uses only locally-grounded
     correspondences.  If fewer than min_inliers pre-masked beams remain, ICP rejects
     with under_inliers → returns the odom prior unchanged.
"""
from __future__ import annotations
import math
from typing import Iterable, List, Tuple

import numpy as np

from tugbot_maze.flood_fill_brain import CELL_SIZE_M, DIRS
from tugbot_maze.scan_match_localizer import ScanMatchLocalizer

Segment = Tuple[float, float, float, float]
Cell = Tuple[int, int]

# ICP is grounded when at least one committed wall endpoint/midpoint is within this
# distance of the prior pose.  CELL_SIZE_M/2 = 1.0 m means the robot must be inside
# (or at the exact boundary of) a committed cell's coverage.  Any farther and the
# robot is in new territory -- fall back to the odom prior.
_LOCAL_RADIUS: float = CELL_SIZE_M / 2.0   # = 1.0 m


def _edge_segment(cell: Cell, d: str) -> Segment:
    """Grid-snapped centerline of the wall on edge (cell, d). Cell centre = (2c, 2r);
    each wall lies half a cell (1.0 m) from centre and spans the cell width."""
    c, r = cell
    cx, cy = CELL_SIZE_M * c, CELL_SIZE_M * r
    h = CELL_SIZE_M / 2.0
    if d == 'N':
        return (cx - h, cy + h, cx + h, cy + h)
    if d == 'S':
        return (cx - h, cy - h, cx + h, cy - h)
    if d == 'E':
        return (cx + h, cy - h, cx + h, cy + h)
    return (cx - h, cy - h, cx - h, cy + h)              # 'W'


def _canonical(seg: Segment) -> Segment:
    a, b = (seg[0], seg[1]), (seg[2], seg[3])
    return (a[0], a[1], b[0], b[1]) if a <= b else (b[0], b[1], a[0], a[1])


def confirmed_wall_segments(brain, committed_cells: Iterable[Cell]) -> List[Segment]:
    """Segments for every WALL edge of a committed cell, de-duplicated (a wall is shared by
    two cells). Only committed cells contribute, so a poorly-sensed wall never enters the
    localization reference."""
    seen = set()
    for cell in committed_cells:
        for d in DIRS:
            if brain.is_wall(cell, d):
                seen.add(_canonical(_edge_segment(cell, d)))
    return [tuple(s) for s in seen]


class OnlineScanMatchLocalizer:
    """Scan-match against `perimeter + confirmed-interior-walls`, rebuilding the inner
    ScanMatchLocalizer only when the interior set changes. The ICP itself is reused
    unchanged; only the segment source is dynamic.

    Three layered gates prevent the ICP from running in situations where it produces
    biased corrections (see module docstring for the full diagnosis)."""

    def __init__(self, perimeter_segments, *,
                 min_interior_segs: int = 1,
                 local_radius: float = _LOCAL_RADIUS,
                 **icp_kwargs):
        self._perimeter = [tuple(s) for s in perimeter_segments]
        self._icp_kwargs = dict(icp_kwargs)
        self._min_interior = int(min_interior_segs)
        self._local_radius = float(local_radius)
        self._sig = frozenset()
        self._rebuild([])

    def _rebuild(self, interior_segments) -> None:
        segs = self._perimeter + [tuple(s) for s in interior_segments]
        self._icp = ScanMatchLocalizer(segs, **self._icp_kwargs)

    def _has_local_interior(self, prior_pose, interior_segments) -> bool:
        """Return True iff at least one committed interior segment has an endpoint or
        midpoint within local_radius of the prior pose.

        When the robot is in uncommitted territory the reference walls are all from
        already-visited cells; they are increasingly far away, and the ICP Jacobian is
        dominated by stale correspondences.  This check gates the ICP so that it runs
        only when the robot is genuinely inside (or at the immediate boundary of) the
        committed map, where the reference provides reliable local constraints."""
        px, py = float(prior_pose[0]), float(prior_pose[1])
        r2 = self._local_radius * self._local_radius
        for seg in interior_segments:
            x0, y0, x1, y1 = seg
            mx, my = (x0 + x1) * 0.5, (y0 + y1) * 0.5
            for tx, ty in ((x0, y0), (x1, y1), (mx, my)):
                if (tx - px) * (tx - px) + (ty - py) * (ty - py) <= r2:
                    return True
        return False

    def _mask_far_beams(self, prior_pose, ranges, angle_min, angle_inc):
        """Return a modified ranges list where beams whose map-frame endpoints (computed
        from the prior pose) are farther than (max_corr_dist_m + wall_half_thickness_m)
        from every reference segment are set to inf.

        ScanMatchLocalizer's first ICP iteration is unfiltered (max_corr=None); when the
        interior reference is sparse this lets beams hitting unmodeled walls form wrong
        correspondences, biasing the Jacobian toward a false minimum.  Pre-masking
        enforces the distance gate from iteration 0 onward.  If fewer than min_inliers
        beams survive, the ICP rejects (under_inliers) and returns the odom prior."""
        if self._icp._a.shape[0] == 0:
            return ranges
        r = np.asarray(ranges, dtype=float)
        x, y, th = float(prior_pose[0]), float(prior_pose[1]), float(prior_pose[2])
        c, s = math.cos(th), math.sin(th)
        ang = angle_min + np.arange(len(r), dtype=float) * angle_inc
        bx = self._icp.scan_offset_x + r * np.cos(ang)
        by = r * np.sin(ang)
        px = x + c * bx - s * by
        py = y + s * bx + c * by
        pts = np.stack([px, py], axis=1)                              # (N, 2)
        ap = pts[:, None, :] - self._icp._a[None, :, :]              # (N, S, 2)
        t = np.clip(
            np.sum(ap * self._icp._e[None, :, :], axis=2) / self._icp._len2[None, :],
            0.0, 1.0)
        foot = self._icp._a[None, :, :] + t[:, :, None] * self._icp._e
        dist = np.sqrt(np.sum((pts[:, None, :] - foot) ** 2, axis=2)).min(axis=1)  # (N,)
        threshold = self._icp.max_corr_dist_m + self._icp.wall_half_thickness_m
        far = np.isfinite(r) & (r > 0.0) & (dist > threshold)
        if not np.any(far):
            return ranges                                             # fast path: nothing masked
        out = r.copy()
        out[far] = np.inf
        return out.tolist()

    def correct(self, prior_pose, ranges, angle_min, angle_inc, interior_segments):
        """Return (corrected_pose, info).

        Gates (in order):
          1. Sparse-interior: skip if fewer than min_interior_segs interior walls.
          2. Locality: skip if no interior segment is within local_radius of the prior.
          3. Beam premasking: remove beams too far from any reference wall.
          4. ICP (rejects if under min_inliers after premasking)."""
        sig = frozenset(tuple(s) for s in interior_segments)
        if sig != self._sig:
            self._rebuild(interior_segments)
            self._sig = sig
        # Gate 1: sparse-interior
        if len(interior_segments) < self._min_interior:
            return prior_pose, {'rejected': True, 'n_inliers': 0,
                                'reason': 'sparse_interior_gate'}
        # Gate 2: locality — robot must be near at least one committed wall
        if not self._has_local_interior(prior_pose, interior_segments):
            return prior_pose, {'rejected': True, 'n_inliers': 0,
                                'reason': 'no_local_interior_walls'}
        # Gate 3 + ICP: pre-mask far beams then run the ICP
        masked = self._mask_far_beams(prior_pose, ranges, angle_min, angle_inc)
        return self._icp.correct(prior_pose, masked, angle_min, angle_inc)
