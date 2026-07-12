"""Radar-accumulated occupancy grid for the online self-built map: integrate each LIDAR
scan by ray-casting from the sensor into a fixed log-odds grid -- cells along each beam's
free path decrease (free), the endpoint cell increases (occupied) -- using the same
projection convention as ScanMatchLocalizer (SCAN_OFFSET_X + pose rotate/translate) driven
by the drift-free ICP pose. Threshold to a classic -1/0/100 OccupancyGrid, so the explored
area grows in a radar-scatter style rather than as grid-snapped cell blocks. Pure NumPy +
std msgs -- no ROS node state, no rclpy.init needed."""
from __future__ import annotations
import math

import numpy as np
from nav_msgs.msg import OccupancyGrid

from tugbot_maze.footprint import SCAN_OFFSET_X


class RadarOccupancyGrid:
    def __init__(self, perimeter_segments, resolution: float = 0.05,
                 margin_m: float = 0.5, scan_offset_x: float = SCAN_OFFSET_X,
                 usable_range_m: float = 8.0, l_occ: float = 0.85,
                 l_free: float = 0.4, l_clamp: float = 5.0,
                 occ_thresh: float = 0.4, free_thresh: float = 0.4) -> None:
        segs = list(perimeter_segments)
        xs = [s[i] for s in segs for i in (0, 2)]
        ys = [s[i] for s in segs for i in (1, 3)]
        self.resolution = float(resolution)
        self.x0 = min(xs) - margin_m
        self.y0 = min(ys) - margin_m
        self.width = int(math.ceil((max(xs) + margin_m - self.x0) / resolution))
        self.height = int(math.ceil((max(ys) + margin_m - self.y0) / resolution))
        self.scan_offset_x = float(scan_offset_x)
        self.usable_range_m = float(usable_range_m)
        self.l_occ = float(l_occ)
        self.l_free = float(l_free)
        self.l_clamp = float(l_clamp)
        self.occ_thresh = float(occ_thresh)
        self.free_thresh = float(free_thresh)
        self._logodds = np.zeros((self.height, self.width), dtype=np.float32)

    def integrate_scan(self, pose, ranges, angle_min, angle_inc) -> None:
        """Ray-cast one scan into the log-odds grid: free along each beam, occupied at the
        endpoint. `pose` is the map->base_link (x, y, yaw). Invalid beams are dropped."""
        r = np.asarray(ranges, dtype=float)
        if r.size == 0:
            return
        ang = angle_min + np.arange(r.shape[0], dtype=float) * angle_inc   # sensor==base_link axes
        valid = np.isfinite(r) & (r > 0.0) & (r <= self.usable_range_m)
        r, ang = r[valid], ang[valid]
        if r.size == 0:
            return
        x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
        c, s = math.cos(th), math.sin(th)
        sx = x + c * self.scan_offset_x                    # sensor origin in map
        sy = y + s * self.scan_offset_x
        bx = self.scan_offset_x + r * np.cos(ang)          # endpoint in base_link
        by = r * np.sin(ang)
        px = x + c * bx - s * by                           # endpoint in map
        py = y + s * bx + c * by
        ex = np.floor((px - self.x0) / self.resolution).astype(np.int64)   # endpoint cells
        ey = np.floor((py - self.y0) / self.resolution).astype(np.int64)
        # Free-path samples: step from the sensor toward the endpoint at one cell per step,
        # keeping t < r AND excluding any sample that lands in the beam's own endpoint cell
        # (so an oblique beam can't decrement the cell it is about to mark occupied).
        step = self.resolution
        max_n = int(math.ceil(float(r.max()) / step))
        t = np.arange(max_n, dtype=float) * step                       # (max_n,)
        dirx = np.cos(th + ang)                                        # beam heading in map
        diry = np.sin(th + ang)
        fx = sx + t[None, :] * dirx[:, None]                          # (B, max_n)
        fy = sy + t[None, :] * diry[:, None]
        ix = np.floor((fx - self.x0) / self.resolution).astype(np.int64)
        iy = np.floor((fy - self.y0) / self.resolution).astype(np.int64)
        keep = (t[None, :] < r[:, None]) & ~((ix == ex[:, None]) & (iy == ey[:, None]))
        inb = keep & (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        np.add.at(self._logodds, (iy[inb], ix[inb]), -self.l_free)
        einb = (ex >= 0) & (ex < self.width) & (ey >= 0) & (ey < self.height)
        np.add.at(self._logodds, (ey[einb], ex[einb]), self.l_occ)
        np.clip(self._logodds, -self.l_clamp, self.l_clamp, out=self._logodds)

    def to_occupancy_grid(self, frame_id: str = 'map', stamp=None) -> OccupancyGrid:
        """Threshold the log-odds grid to -1 unknown / 0 free / 100 occupied."""
        grid = np.full((self.height, self.width), -1, dtype=np.int8)
        grid[self._logodds <= -self.free_thresh] = 0
        grid[self._logodds >= self.occ_thresh] = 100
        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        if stamp is not None:
            msg.header.stamp = stamp
        msg.info.resolution = float(self.resolution)
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = float(self.x0)
        msg.info.origin.position.y = float(self.y0)
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel().tolist()          # row-major, row 0 at origin.y
        return msg
