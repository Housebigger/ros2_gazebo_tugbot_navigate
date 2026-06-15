"""Sense a maze cell's N/E/S/W edges from a SLAM OccupancyGridView.

Connector-based: for each neighbor, sample the straight center->neighbor-center line
(2 m) on the occupancy grid. Return True (WALL) if any sampled cell is occupied,
None (UNKNOWN) if too much of the line is unmapped (off-grid or unknown -> re-sense
later), else False (OPEN). This mirrors maze_sim.ground_truth_edge_open (which the
offline guarantee uses), so live SLAM sensing matches the offline proof's notion of
"can the robot traverse center-to-center" -- and a real wall (which blocks the whole
traversal) is detected by ANY occupied sample, not a fraction of an edge line. The
maze cell grid is 2 m: cell (cx,cy) is centered at (CELL_SIZE_M*cx, CELL_SIZE_M*cy).
"""
from __future__ import annotations
from typing import Dict, Optional

from tugbot_maze.flood_fill_brain import CELL_SIZE_M, DIRS


def sense_cell_walls(grid_view, cell, *, cell_size: float = CELL_SIZE_M,
                     n_samples: int = 9, unknown_frac: float = 0.5) -> Dict[str, Optional[bool]]:
    if n_samples < 1:
        raise ValueError("n_samples must be >= 1")
    cx, cy = cell
    ccx, ccy = cell_size * cx, cell_size * cy
    out: Dict[str, Optional[bool]] = {}
    for d, (dx, dy) in DIRS.items():
        nbx, nby = ccx + dx * cell_size, ccy + dy * cell_size   # neighbor center
        occ = unk = 0
        for i in range(n_samples + 1):
            t = i / n_samples
            g = grid_view.world_to_cell(ccx + (nbx - ccx) * t, ccy + (nby - ccy) * t)
            if (not grid_view.in_bounds(g)) or grid_view.is_unknown(g):
                unk += 1                       # unmapped: don't commit to OPEN yet
            elif grid_view.is_occupied(g):
                occ += 1                       # a real wall blocks the whole traversal
        if occ > 0:
            out[d] = True
        elif unk > unknown_frac * (n_samples + 1):
            out[d] = None
        else:
            out[d] = False
    return out
