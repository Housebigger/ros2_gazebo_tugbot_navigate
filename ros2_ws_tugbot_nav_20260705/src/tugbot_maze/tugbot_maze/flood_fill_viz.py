"""RViz visualization for the online self-built map: turn confirmed wall segments into a
MarkerArray so RViz shows the map the robot actually built (perimeter + confirmed interior
walls), rather than slam_toolbox's occupancy grid, and into a classic-SLAM-semantics
OccupancyGrid (unknown/free/occupied). Pure builder — no ROS node state."""
from __future__ import annotations
import math
from typing import Iterable, Tuple

import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

Segment = Tuple[float, float, float, float]


def self_built_wall_markerarray(segments: Iterable[Segment], frame_id: str = 'map',
                                stamp=None, ns: str = 'self_built_walls') -> MarkerArray:
    """A single green LINE_LIST marker with two points per wall segment, in `frame_id`."""
    m = Marker()
    m.header.frame_id = frame_id
    if stamp is not None:
        m.header.stamp = stamp
    m.ns = ns
    m.id = 0
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.scale.x = 0.05
    m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
    m.pose.orientation.w = 1.0
    for x0, y0, x1, y1 in segments:
        m.points.append(Point(x=float(x0), y=float(y0), z=0.0))
        m.points.append(Point(x=float(x1), y=float(y1), z=0.0))
    return MarkerArray(markers=[m])


def self_built_occupancy_grid(sensed_cells, wall_segments, perimeter_segments,
                              resolution: float = 0.05, margin_m: float = 0.5,
                              wall_half_thickness_m: float = 0.12,
                              frame_id: str = 'map', stamp=None) -> OccupancyGrid:
    """Render the self-built map as a classic-SLAM-semantics OccupancyGrid:
    unknown (-1) everywhere, sensed cells' 2x2m interiors free (0), and wall
    bands (perimeter + confirmed interior walls) occupied (100, overwrites free).
    Bounds come from the perimeter bbox + margin. Occupancy is decided at each
    grid cell's CENTER (inside a cell interior -> free; within
    wall_half_thickness_m of a wall centerline -> occupied)."""
    xs = [s[i] for s in perimeter_segments for i in (0, 2)]
    ys = [s[i] for s in perimeter_segments for i in (1, 3)]
    x0, y0 = min(xs) - margin_m, min(ys) - margin_m
    width = int(math.ceil((max(xs) + margin_m - x0) / resolution))
    height = int(math.ceil((max(ys) + margin_m - y0) / resolution))
    grid = np.full((height, width), -1, dtype=np.int8)

    # 1) sensed cell interiors -> free. Cell (c, r) interior spans
    #    [2c-1, 2c+1] x [2r-1, 2r+1] metres (cell centre = (2c, 2r)).
    for c, r in sensed_cells:
        ix0 = max(int((2 * c - 1 - x0) / resolution), 0)
        ix1 = min(int((2 * c + 1 - x0) / resolution), width)
        iy0 = max(int((2 * r - 1 - y0) / resolution), 0)
        iy1 = min(int((2 * r + 1 - y0) / resolution), height)
        grid[iy0:iy1, ix0:ix1] = 0

    # 2) wall bands -> occupied (overwrites free). Per segment, only the local
    #    window is evaluated: grid-cell centres within wall_half_thickness_m
    #    of the segment are set to 100.
    ht = wall_half_thickness_m
    cxs = x0 + (np.arange(width) + 0.5) * resolution
    cys = y0 + (np.arange(height) + 0.5) * resolution
    for sx0, sy0, sx1, sy1 in list(perimeter_segments) + list(wall_segments):
        wx0 = max(int((min(sx0, sx1) - ht - x0) / resolution), 0)
        wx1 = min(int(math.ceil((max(sx0, sx1) + ht - x0) / resolution)), width)
        wy0 = max(int((min(sy0, sy1) - ht - y0) / resolution), 0)
        wy1 = min(int(math.ceil((max(sy0, sy1) + ht - y0) / resolution)), height)
        if wx0 >= wx1 or wy0 >= wy1:
            continue
        X, Y = np.meshgrid(cxs[wx0:wx1], cys[wy0:wy1])
        ex, ey = sx1 - sx0, sy1 - sy0
        len2 = ex * ex + ey * ey
        if len2 == 0.0:
            t = np.zeros_like(X)
        else:
            t = np.clip(((X - sx0) * ex + (Y - sy0) * ey) / len2, 0.0, 1.0)
        d2 = (X - (sx0 + t * ex)) ** 2 + (Y - (sy0 + t * ey)) ** 2
        sub = grid[wy0:wy1, wx0:wx1]
        sub[d2 <= ht * ht] = 100

    msg = OccupancyGrid()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp
    msg.info.resolution = float(resolution)
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.position.x = float(x0)
    msg.info.origin.position.y = float(y0)
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.flatten().tolist()          # row-major, row 0 at origin.y
    return msg
