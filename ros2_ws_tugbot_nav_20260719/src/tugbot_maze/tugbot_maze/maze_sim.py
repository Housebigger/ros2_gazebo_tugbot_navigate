"""Test-only 2-D maze simulator: a vectorized raycaster + unicycle integrator
over the REAL 20260528 wall segments. Offline proof ground for the maze solvers:
the wall-follower's exit-reaching guarantee, and the flood-fill solver's cell-edge
model (`ground_truth_edge_open`) + inertia-aware locomotion. Not imported by any ROS node.

px -> map-frame transform for the scaled2x 20260528 world (verified three ways:
derivation, entrance gap, exit gap):
    Xm = -0.988719 + x_px * (24/359)
    Ym = 21.025070 - y_px * (24/359)
Walls are modeled as zero-thickness centerline segments; their physical
half-thickness is added back as a margin (collision) and subtracted from beam
ranges (so a beam reads the wall FACE, like the real LIDAR).
"""
from __future__ import annotations
import math
import os
from typing import List, Optional, Tuple

import numpy as np
import yaml

from tugbot_maze.flood_fill_brain import CELL_SIZE_M
from tugbot_maze.footprint import FOOT_X_FRONT, FOOT_X_REAR, FOOT_HALF_W

Segment = Tuple[float, float, float, float]   # (x0, y0, x1, y1), map frame

_PX_SCALE = 24.0 / 359.0
_X_OFFSET = -12.0 + 11.011281     # = -0.988719
_Y_OFFSET = 12.0 + 9.025070       # = 21.025070


def px_to_map(x_px: float, y_px: float) -> Tuple[float, float]:
    return (_X_OFFSET + x_px * _PX_SCALE, _Y_OFFSET - y_px * _PX_SCALE)


def default_segments_path() -> str:
    return os.path.join(os.path.dirname(__file__), '..', 'config',
                        'maze_wall_segments_20260528.yaml')


def load_segments(path: Optional[str] = None) -> List[Segment]:
    """Load wall centerline segments from the YAML, transformed to map frame."""
    if path is None:
        path = default_segments_path()
    with open(path) as f:
        doc = yaml.safe_load(f)
    segs: List[Segment] = []
    for s in doc['segments']:
        x0, y0 = px_to_map(float(s['p0_px'][0]), float(s['p0_px'][1]))
        x1, y1 = px_to_map(float(s['p1_px'][0]), float(s['p1_px'][1]))
        segs.append((x0, y0, x1, y1))
    return segs


def outer_boundary_box(path: Optional[str] = None) -> Tuple[float, float, float, float]:
    """Return (xmin, xmax, ymin, ymax) of the maze's outer boundary in map frame.

    Built from the segments flagged `outer: true` in the YAML. Used by the
    guarantee to assert the solver never leaves the maze (no exterior cheating).
    """
    if path is None:
        path = default_segments_path()
    with open(path) as f:
        doc = yaml.safe_load(f)
    xs: List[float] = []
    ys: List[float] = []
    for s in doc['segments']:
        if s.get('outer'):
            x0, y0 = px_to_map(float(s['p0_px'][0]), float(s['p0_px'][1]))
            x1, y1 = px_to_map(float(s['p1_px'][0]), float(s['p1_px'][1]))
            xs += [x0, x1]
            ys += [y0, y1]
    if not xs:
        raise RuntimeError(f"no 'outer: true' segments found in {path!r}")
    return (min(xs), max(xs), min(ys), max(ys))


def outer_segments(path: Optional[str] = None) -> List[Segment]:
    """Return the maze's OUTER boundary wall centerlines (segments flagged `outer: true`),
    in map frame. This is the legitimately-known perimeter used to seed online localization
    when the interior wall map is withheld -- NOT the full map (`load_segments`)."""
    if path is None:
        path = default_segments_path()
    with open(path) as f:
        doc = yaml.safe_load(f)
    segs: List[Segment] = []
    for s in doc['segments']:
        if s.get('outer'):
            x0, y0 = px_to_map(float(s['p0_px'][0]), float(s['p0_px'][1]))
            x1, y1 = px_to_map(float(s['p1_px'][0]), float(s['p1_px'][1]))
            segs.append((x0, y0, x1, y1))
    if not segs:
        raise RuntimeError(f"no 'outer: true' segments found in {path!r}")
    return segs


def ground_truth_edge_open(sim: "MazeSim", a, b, samples: int = 10) -> bool:
    """True if the robot can traverse from cell-center a to cell-center b without
    colliding (samples the straight connector against the real wall segments).
    a, b are (cx, cy) cells with centers at (CELL_SIZE_M*cx, CELL_SIZE_M*cy)."""
    ax, ay = CELL_SIZE_M * a[0], CELL_SIZE_M * a[1]
    bx, by = CELL_SIZE_M * b[0], CELL_SIZE_M * b[1]
    for i in range(samples + 1):
        t = i / samples
        if sim.collides(ax + (bx - ax) * t, ay + (by - ay) * t):
            return False
    return True


class MazeSim:
    def __init__(self, segments, start_xy, start_yaw, *, robot_radius_m=0.35,
                 wall_half_thickness_m=0.12, max_range_m=12.0, inertia=False,
                 v_max=0.5, w_max=0.5, lin_accel=0.5, ang_accel=0.8,
                 odom_drift_per_m=0.0, cmd_latency_steps=0):
        self.segs = np.asarray(segments, dtype=float).reshape(-1, 4)
        self.x = float(start_xy[0])
        self.y = float(start_xy[1])
        self.yaw = float(start_yaw)
        self.odom_drift_per_m = odom_drift_per_m
        self._drift_x = 0.0                            # accumulated odom drift (map frame)
        self._drift_y = 0.0
        self._last_x, self._last_y = self.x, self.y
        self.robot_radius_m = robot_radius_m
        self.wall_half_thickness_m = wall_half_thickness_m
        self.max_range_m = max_range_m
        # Precompute endpoint A (S,2) and edge e = B - A (S,2).
        if self.segs.shape[0] > 0:
            self._a = self.segs[:, 0:2]
            self._e = self.segs[:, 2:4] - self.segs[:, 0:2]
        else:
            self._a = np.empty((0, 2))
            self._e = np.empty((0, 2))
        self.inertia = inertia
        self.v_max = v_max
        self.w_max = w_max
        self.lin_accel = lin_accel
        self.ang_accel = ang_accel
        self.v_cur = 0.0
        self.w_cur = 0.0
        # Command-pipeline latency: a (v,w) command takes effect cmd_latency_steps ticks
        # later. Models the Gazebo velocity_smoother + diff-drive plugin delay, which (with
        # P-control) induces the rotate-in-place overshoot/oscillation the offline inertia
        # model alone did not reproduce. 0 = no latency (default, prior behavior).
        self.cmd_latency_steps = int(cmd_latency_steps)
        self._cmd_buf = []

    @property
    def pose(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.yaw)

    @property
    def reported_pose(self) -> Tuple[float, float, float]:
        """Odom pose = true pose + accumulated drift (what the solver would believe)."""
        return (self.x + self._drift_x, self.y + self._drift_y, self.yaw)

    def scan(self, n_beams=120, fov_rad=2 * math.pi, max_range=None):
        """Return (ranges_list, angle_min, angle_increment) from the current pose.

        Angles are robot-relative (forward = 0). Full-circle scans use a wrapping
        layout over [-pi, pi) with increment 2*pi/n_beams (matching a velodyne).
        """
        if max_range is None:
            max_range = self.max_range_m
        if fov_rad >= 2 * math.pi - 1e-9:
            angle_min = -math.pi
            angle_inc = 2 * math.pi / n_beams
        else:
            angle_min = -fov_rad / 2.0
            angle_inc = fov_rad / (n_beams - 1)
        idx = np.arange(n_beams)
        beam_ang = self.yaw + angle_min + idx * angle_inc          # (B,) world frame
        if self.segs.shape[0] == 0:
            return [float(max_range)] * n_beams, angle_min, angle_inc
        dx = np.cos(beam_ang)[:, None]                             # (B,1)
        dy = np.sin(beam_ang)[:, None]                             # (B,1)
        ex = self._e[:, 0][None, :]                               # (1,S)
        ey = self._e[:, 1][None, :]                               # (1,S)
        wx = (self._a[:, 0] - self.x)[None, :]                    # (1,S)
        wy = (self._a[:, 1] - self.y)[None, :]                    # (1,S)
        denom = dx * ey - dy * ex                                 # (B,S)
        wxe = wx * ey - wy * ex                                   # (1,S) -> broadcast
        wxd = wx * dy - wy * dx                                   # (B,S)
        with np.errstate(divide='ignore', invalid='ignore'):
            t = wxe / denom                                       # (B,S) ray param
            u = wxd / denom                                       # (B,S) segment param
        valid = (np.abs(denom) > 1e-12) & (t >= 0.0) & (u >= 0.0) & (u <= 1.0)
        t = np.where(valid, t, np.inf)
        rng = t.min(axis=1)                                       # (B,)
        rng = np.clip(rng - self.wall_half_thickness_m, 0.0, max_range)
        return rng.tolist(), angle_min, angle_inc

    def _point_seg_dists(self, x, y):
        if self.segs.shape[0] == 0:
            return np.empty((0,))
        p = np.array([x, y])
        ap = p[None, :] - self._a                                 # (S,2)
        ee = np.sum(self._e * self._e, axis=1)                    # (S,)
        t = np.where(ee > 1e-12, np.sum(ap * self._e, axis=1) / np.maximum(ee, 1e-12), 0.0)
        t = np.clip(t, 0.0, 1.0)
        proj = self._a + t[:, None] * self._e                     # (S,2)
        return np.linalg.norm(p[None, :] - proj, axis=1)          # (S,)

    def collides(self, x, y, yaw=None) -> bool:
        """Footprint collision oracle. With yaw, tests the TRUE asymmetric footprint rectangle
        (base_link X in [FOOT_X_REAR, FOOT_X_FRONT], |Y| <= FOOT_HALF_W) inflated by the wall
        half-thickness -- catches the rear gripper, tighter on the sides than the old 0.35 m circle.
        Without yaw (legacy x,y-only callers), falls back to a conservative bounding circle."""
        if self.segs.shape[0] == 0:
            return False
        m = self.wall_half_thickness_m
        if yaw is None:
            d = self._point_seg_dists(x, y)
            radius = max(abs(FOOT_X_REAR), FOOT_X_FRONT, FOOT_HALF_W,
                         math.hypot(FOOT_X_REAR, FOOT_HALF_W))
            return bool(np.any(d < radius + m))
        # yaw given: EXACT min distance from each wall-segment centerline to the TRUE footprint
        # rectangle [FOOT_X_REAR, FOOT_X_FRONT] x [-FOOT_HALF_W, FOOT_HALF_W] in base_link.
        # Collision iff that distance < wall_half_thickness (the wall surface reaches the body).
        c, s = math.cos(yaw), math.sin(yaw)
        ax = self.segs[:, 0] - x; ay = self.segs[:, 1] - y
        bx = self.segs[:, 2] - x; by = self.segs[:, 3] - y
        Ax =  c * ax + s * ay;  Ay = -s * ax + c * ay        # seg endpoint A in base_link (S,)
        Bx =  c * bx + s * by;  By = -s * bx + c * by        # seg endpoint B in base_link (S,)
        x0, x1 = FOOT_X_REAR, FOOT_X_FRONT
        y0, y1 = -FOOT_HALF_W, FOOT_HALF_W
        dx = Bx - Ax; dy = By - Ay
        # (1) Liang-Barsky: does segment intersect the rectangle? -> distance 0
        t0 = np.zeros(Ax.shape); t1 = np.ones(Ax.shape)
        intersects = np.ones(Ax.shape, dtype=bool)
        for pi, qi in ((-dx, Ax - x0), (dx, x1 - Ax), (-dy, Ay - y0), (dy, y1 - Ay)):
            parallel = np.abs(pi) < 1e-12
            intersects &= ~(parallel & (qi < 0))             # parallel & outside slab -> miss
            with np.errstate(divide='ignore', invalid='ignore'):
                r = qi / pi
            t0 = np.where((pi < 0) & ~parallel, np.maximum(t0, r), t0)   # entering
            t1 = np.where((pi > 0) & ~parallel, np.minimum(t1, r), t1)   # leaving
        intersects &= (t0 <= t1)
        # (2) disjoint case: min of seg-endpoints-to-box and box-corners-to-seg
        def _pt_to_box(px, py):
            ox = np.maximum(np.maximum(x0 - px, 0.0), px - x1)
            oy = np.maximum(np.maximum(y0 - py, 0.0), py - y1)
            return np.hypot(ox, oy)
        def _corner_to_seg(cx, cy):
            wx = cx - Ax; wy = cy - Ay
            L2 = dx * dx + dy * dy
            tt = np.where(L2 > 1e-12, (wx * dx + wy * dy) / np.maximum(L2, 1e-12), 0.0)
            tt = np.clip(tt, 0.0, 1.0)
            return np.hypot(cx - (Ax + tt * dx), cy - (Ay + tt * dy))
        mind = np.minimum.reduce([_pt_to_box(Ax, Ay), _pt_to_box(Bx, By),
                                  _corner_to_seg(x0, y0), _corner_to_seg(x1, y0),
                                  _corner_to_seg(x1, y1), _corner_to_seg(x0, y1)])
        mind = np.where(intersects, 0.0, mind)
        return bool(np.any(mind < m))

    def step(self, v, w, dt):
        if self.cmd_latency_steps > 0:                 # apply the command from N ticks ago
            self._cmd_buf.append((v, w))
            v, w = self._cmd_buf.pop(0) if len(self._cmd_buf) > self.cmd_latency_steps else (0.0, 0.0)
        if self.inertia:
            v_cmd = max(-self.v_max, min(self.v_max, v))
            w_cmd = max(-self.w_max, min(self.w_max, w))
            self.v_cur += max(-self.lin_accel * dt, min(self.lin_accel * dt, v_cmd - self.v_cur))
            self.w_cur += max(-self.ang_accel * dt, min(self.ang_accel * dt, w_cmd - self.w_cur))
            v, w = self.v_cur, self.w_cur
        nx = self.x + v * math.cos(self.yaw) * dt
        ny = self.y + v * math.sin(self.yaw) * dt
        if not self.collides(nx, ny, self.yaw):
            self.x, self.y = nx, ny
        # Rotation always applies (turning in place is safe; lets TURN_AWAY recover).
        self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))
        dx, dy = self.x - self._last_x, self.y - self._last_y
        dist = math.hypot(dx, dy)
        if dist > 1e-9 and self.odom_drift_per_m:
            # LATERAL (cross-track) slip, perpendicular to travel: matches the confirmed
            # real drift (odom drifted ~0.7 m in X while traveling north). This is the
            # component wall-referenced re-centering corrects; magnitude grows with distance.
            self._drift_x += self.odom_drift_per_m * (-dy)
            self._drift_y += self.odom_drift_per_m * dx
        self._last_x, self._last_y = self.x, self.y
