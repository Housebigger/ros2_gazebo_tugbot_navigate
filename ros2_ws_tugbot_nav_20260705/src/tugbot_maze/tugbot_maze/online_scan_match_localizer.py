"""Online self-built-map localization: scan-match against a GROWING set of confirmed
walls (known perimeter + the flood-fill brain's committed interior walls) instead of a
prior full map. Because the grid is known, a confirmed wall's centerline is grid-snapped
and exact, so the reused ICP still yields an absolute, drift-free pose.
See docs/superpowers/specs/2026-07-05-online-slam-maze-design.md.
"""
from __future__ import annotations
from typing import Iterable, List, Tuple

from tugbot_maze.flood_fill_brain import CELL_SIZE_M, DIRS
from tugbot_maze.scan_match_localizer import ScanMatchLocalizer

Segment = Tuple[float, float, float, float]
Cell = Tuple[int, int]


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
    unchanged; only the segment source is dynamic."""

    def __init__(self, perimeter_segments, **icp_kwargs):
        self._perimeter = [tuple(s) for s in perimeter_segments]
        self._icp_kwargs = dict(icp_kwargs)
        self._sig = frozenset()
        self._rebuild([])

    def _rebuild(self, interior_segments) -> None:
        segs = self._perimeter + [tuple(s) for s in interior_segments]
        self._icp = ScanMatchLocalizer(segs, **self._icp_kwargs)

    def correct(self, prior_pose, ranges, angle_min, angle_inc, interior_segments):
        sig = frozenset(tuple(s) for s in interior_segments)
        if sig != self._sig:
            self._rebuild(interior_segments)
            self._sig = sig
        return self._icp.correct(prior_pose, ranges, angle_min, angle_inc)
