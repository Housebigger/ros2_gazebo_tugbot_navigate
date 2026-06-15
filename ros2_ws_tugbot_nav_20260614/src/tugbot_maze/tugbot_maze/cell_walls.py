"""Sense a maze cell's N/E/S/W edges from a SLAM OccupancyGridView.

The maze cell grid is 2 m (cell (cx,cy) centered at (2*cx, 2*cy)). Each edge is
the boundary line ~1 m from the center toward a neighbor. We sample occupancy at
points across that edge: mostly-occupied -> WALL (True), clearly free -> OPEN
(False), too-unknown -> None (leave UNKNOWN, re-sense later).
"""
from __future__ import annotations
from typing import Dict, Optional

from tugbot_maze.flood_fill_brain import CELL_SIZE_M, DIRS


def sense_cell_walls(grid_view, cell, *, cell_size: float = CELL_SIZE_M,
                     n_samples: int = 5, occupied_frac: float = 0.34,
                     unknown_frac: float = 0.6) -> Dict[str, Optional[bool]]:
    if n_samples < 2:
        raise ValueError("n_samples must be >= 2 (need both edge endpoints)")
    cx, cy = cell
    ccx, ccy = cell_size * cx, cell_size * cy
    half = cell_size / 2.0
    out: Dict[str, Optional[bool]] = {}
    for d, (dx, dy) in DIRS.items():
        # edge midpoint, and the in-plane axis to sample along
        emx, emy = ccx + dx * half, ccy + dy * half
        ax, ay = (0.0, 1.0) if dx != 0 else (1.0, 0.0)   # sample perpendicular to travel
        occ = unk = 0
        for i in range(n_samples):
            t = (i / (n_samples - 1) - 0.5) * cell_size    # -half..+half along the edge
            px, py = emx + ax * t, emy + ay * t
            gcell = grid_view.world_to_cell(px, py)
            if grid_view.is_unknown(gcell):
                unk += 1
            elif grid_view.is_occupied(gcell):
                occ += 1
        if unk / n_samples >= unknown_frac:
            out[d] = None
        else:
            out[d] = (occ / n_samples) >= occupied_frac
    return out
