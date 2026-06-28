# Scan-to-Known-Map Localization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a point-to-line ICP scan-to-map localizer that produces an absolute map-frame pose by matching the live LIDAR against the known maze wall segments, exposed as `pose_source=scan_match`, to eliminate the wheel-odometry desync that is the dominant Gazebo failure (62% primary / 75% heavy / 100% RECONCILE in the `d356f4a` controlled baseline).

**Architecture:** A pure-NumPy `ScanMatchLocalizer` (ROS-free, unit-tested via the `maze_sim` forward model) + two small SE(2) helpers in `pose_tracking.py` + a thin `_lookup_scan_match()` branch in `flood_fill_solver.py`. Route discovery (the flood-fill brain) and `maze_motion` are untouched. `slam` / `odom_locked` remain for A/B comparison; the default `pose_source` does not change.

**Tech Stack:** Python 3.12, NumPy, pytest. ROS 2 Jazzy (node integration only). No new dependencies.

**Spec:** `docs/superpowers/specs/2026-06-28-scan-match-localization-design.md`

**Test command (run from the package root):**
`cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest <path> -v`

---

## File Structure

- **Create** `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/scan_match_localizer.py` — the localizer (one responsibility: scan→absolute pose).
- **Create** `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_scan_match_localizer.py` — localizer unit tests.
- **Modify** `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/pose_tracking.py` — add `inverse_2d`, `odom_prior`.
- **Modify** `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_pose_tracking.py` — add helper tests (create if absent).
- **Modify** `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/flood_fill_solver.py` — `_lookup_scan_match()`, localizer construction, scan-seq, DIAG `MATCH` line.
- **Modify** `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_motion_sim.py` — add the closed-loop scan-match tests.

All paths below are relative to the repo root `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`.

---

## Task 1: SE(2) prior helpers (`inverse_2d`, `odom_prior`)

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/pose_tracking.py`
- Test: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_pose_tracking.py`

- [ ] **Step 1: Write the failing test**

Create/append to `test/test_pose_tracking.py`:

```python
import math
from tugbot_maze.pose_tracking import compose_2d, inverse_2d, odom_prior


def _close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_inverse_2d_is_left_inverse():
    p = (1.3, -2.1, 0.7)
    assert _close(compose_2d(inverse_2d(p), p), (0.0, 0.0, 0.0))
    assert _close(compose_2d(p, inverse_2d(p)), (0.0, 0.0, 0.0))


def test_odom_prior_no_motion_returns_last_corrected():
    corrected = (5.0, 3.0, 1.0)
    odom = (2.0, 2.0, 0.5)
    assert _close(odom_prior(corrected, odom, odom), corrected)


def test_odom_prior_applies_odom_delta_in_body_frame():
    # corrected pose faces +y (yaw=pi/2); odom advances +1 along its own x.
    corrected = (0.0, 0.0, math.pi / 2)
    last_odom = (0.0, 0.0, 0.0)
    cur_odom = (1.0, 0.0, 0.0)            # +1 m forward in odom/base x
    prior = odom_prior(corrected, last_odom, cur_odom)
    # forward in body frame (yaw=pi/2) maps to +y in map
    assert _close(prior, (0.0, 1.0, math.pi / 2), tol=1e-9)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_pose_tracking.py -v`
Expected: FAIL with `ImportError: cannot import name 'inverse_2d'`.

- [ ] **Step 3: Implement the helpers**

Append to `tugbot_maze/pose_tracking.py`:

```python
def inverse_2d(p: Pose2D) -> Pose2D:
    """Inverse of a planar rigid transform."""
    x, y, t = p
    c, s = math.cos(t), math.sin(t)
    return (-(c * x + s * y), (s * x - c * y), -t)


def odom_prior(last_corrected: Pose2D, last_odom: Pose2D, cur_odom: Pose2D) -> Pose2D:
    """Propagate the last corrected map pose by the odom motion since the last tick.

    delta = (odom_last)^-1 o (odom_cur)  is the body-frame motion; applying it to
    the last corrected map pose gives the prior for the current tick's scan match.
    """
    delta = compose_2d(inverse_2d(last_odom), cur_odom)
    return compose_2d(last_corrected, delta)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_pose_tracking.py -v`
Expected: PASS (3 passed).

- [ ] **Step 5: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/pose_tracking.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_pose_tracking.py
git commit -m "feat: add inverse_2d + odom_prior SE(2) helpers for scan-match prior

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: `ScanMatchLocalizer` geometry core (construct + beams→points + association)

**Files:**
- Create: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/scan_match_localizer.py`
- Test: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_scan_match_localizer.py`

**Context:** `scan_offset_x` is the LIDAR mount offset along base_link x. The real `/scan` (`scan_omni`) is mounted at `SCAN_OFFSET_X = -0.1855`. `maze_sim.scan()` rays originate from the body center (no offset), so **all tests construct the localizer with `scan_offset_x=0.0`**.

- [ ] **Step 1: Write the failing test**

Create `test/test_scan_match_localizer.py`:

```python
import math
import numpy as np
from tugbot_maze.scan_match_localizer import ScanMatchLocalizer


def test_init_builds_unit_normals_perpendicular_to_edges():
    # one horizontal wall (along x), one vertical wall (along y)
    segs = [(-1.0, 2.0, 1.0, 2.0), (3.0, -1.0, 3.0, 1.0)]
    loc = ScanMatchLocalizer(segs, scan_offset_x=0.0)
    assert loc._n.shape == (2, 2)
    # unit length
    assert np.allclose(np.hypot(loc._n[:, 0], loc._n[:, 1]), 1.0)
    # perpendicular to edges (dot with edge == 0)
    assert np.allclose(np.sum(loc._n * loc._e, axis=1), 0.0, atol=1e-9)


def test_beams_to_points_places_endpoint_in_map_frame():
    loc = ScanMatchLocalizer([(0.0, 0.0, 1.0, 0.0)], scan_offset_x=0.0, usable_range_m=10.0)
    # robot at origin facing +x; a single beam straight ahead (angle 0), range 2.0
    pts = loc._beams_to_points((0.0, 0.0, 0.0), [2.0], -math.pi, 2 * math.pi / 1)
    # one beam at angle_min=-pi ... use a clean direct call instead:
    pts = loc._beams_to_points((0.0, 0.0, 0.0), [2.0, 2.0], 0.0, math.pi / 2)
    # beam 0 at angle 0 -> (2,0); beam 1 at angle pi/2 -> (0,2)
    assert np.allclose(pts[0], [2.0, 0.0], atol=1e-9)
    assert np.allclose(pts[1], [0.0, 2.0], atol=1e-9)


def test_beams_to_points_applies_scan_offset_and_pose():
    loc = ScanMatchLocalizer([(0.0, 0.0, 1.0, 0.0)], scan_offset_x=-0.1855, usable_range_m=10.0)
    # robot at (5, 5) facing +y (yaw=pi/2); sensor sits 0.1855 behind center -> at (5, 4.8145)
    # a beam at angle 0 (sensor x = body forward = +y in map), range 1.0 -> endpoint (5, 5.8145)
    pts = loc._beams_to_points((5.0, 5.0, math.pi / 2), [1.0], 0.0, math.pi / 2)
    assert np.allclose(pts[0], [5.0, 4.8145 + 1.0], atol=1e-6)


def test_associate_finds_nearest_segment_and_robot_facing_normal():
    loc = ScanMatchLocalizer([(-5.0, 2.0, 5.0, 2.0)], scan_offset_x=0.0)  # wall at y=2
    pts = np.array([[0.0, 1.8]])         # point just below the wall (robot side y<2)
    foot, n, dist = loc._associate(pts)
    assert np.allclose(foot[0], [0.0, 2.0], atol=1e-9)
    assert abs(dist[0] - 0.2) < 1e-9
    # normal must point toward the robot point (negative y here)
    assert n[0, 1] < 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_scan_match_localizer.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tugbot_maze.scan_match_localizer'`.

- [ ] **Step 3: Implement the module geometry core**

Create `tugbot_maze/scan_match_localizer.py`:

```python
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
            z = np.empty((n_pts, 2))
            return z, z, np.full(n_pts, np.inf)
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_scan_match_localizer.py -v`
Expected: PASS (4 passed). (The first `pts =` line in `test_beams_to_points_places_endpoint_in_map_frame` is harmless; the asserted second call drives the check.)

- [ ] **Step 5: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/scan_match_localizer.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_scan_match_localizer.py
git commit -m "feat: ScanMatchLocalizer geometry core (segments, beams->points, association)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: ICP solver + `correct()` (residual, Jacobian, observability gate, loop, clamp)

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/scan_match_localizer.py`
- Test: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_scan_match_localizer.py`

**Context:** Residual along the robot-facing normal is `r = n·(p - foot) - wall_half_thickness` (beams read the wall face, 0.12 m in front of the centerline). Jacobian row about robot position `c`: `[nx, ny, -nx·(py-cy) + ny·(px-cx)]`. Tests use the `maze_sim` forward model (`MazeSim(segs, start_xy, start_yaw).scan(...)`) with `scan_offset_x=0.0`.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_scan_match_localizer.py`:

```python
import pytest
from tugbot_maze.maze_sim import MazeSim, load_segments
from tugbot_maze.scan_match_localizer import _wrap

_SCAN_KW = dict(n_beams=360, fov_rad=2 * math.pi)


def _scan_at(segs, pose):
    sim = MazeSim(segs, (pose[0], pose[1]), pose[2])
    return sim.scan(**_SCAN_KW)


# Corridor poses (cell centers) with clear side walls and good observability.
_RECOVERY_POSES = [(2.0, 2.0, math.pi / 2), (2.0, 6.0, math.pi / 2),
                   (8.0, 2.0, 0.0), (4.0, 8.0, 0.0)]


@pytest.mark.parametrize("true_pose", _RECOVERY_POSES)
def test_recovers_true_pose_under_prior_drift(true_pose):
    segs = load_segments()
    loc = ScanMatchLocalizer(segs, scan_offset_x=0.0)
    ranges, amin, ainc = _scan_at(segs, true_pose)
    prior = (true_pose[0] + 0.35, true_pose[1] - 0.25, true_pose[2] + math.radians(7))
    est, info = loc.correct(prior, ranges, amin, ainc)
    assert not info["rejected"], f"unexpectedly rejected at {true_pose}: {info}"
    assert math.hypot(est[0] - true_pose[0], est[1] - true_pose[1]) < 0.05, info
    assert abs(_wrap(est[2] - true_pose[2])) < math.radians(2), info


def test_no_half_thickness_bias_at_true_pose():
    segs = load_segments()
    loc = ScanMatchLocalizer(segs, scan_offset_x=0.0)
    true_pose = (2.0, 2.0, math.pi / 2)
    ranges, amin, ainc = _scan_at(segs, true_pose)
    est, _ = loc.correct(true_pose, ranges, amin, ainc)   # prior == truth
    assert math.hypot(est[0] - true_pose[0], est[1] - true_pose[1]) < 0.02


def test_open_axis_falls_back_to_prior():
    # infinite-feeling N-S corridor: walls only on the x axis -> y unobservable
    walls = [(-1.0, -5.0, -1.0, 5.0), (1.0, -5.0, 1.0, 5.0)]
    loc = ScanMatchLocalizer(walls, scan_offset_x=0.0, usable_range_m=8.0)
    ranges, amin, ainc = _scan_at(walls, (0.0, 0.0, math.pi / 2))
    prior = (0.3, 0.4, math.pi / 2)                  # x off (constrained), y off (free)
    est, info = loc.correct(prior, ranges, amin, ainc)
    assert "y" in info["fell_back"], info
    assert abs(est[0] - 0.0) < 0.05, f"x not corrected: {est}"
    assert abs(est[1] - 0.4) < 0.05, f"y should stay at prior: {est}"


def test_rejects_correction_beyond_clamp():
    segs = load_segments()
    loc = ScanMatchLocalizer(segs, scan_offset_x=0.0, trans_clamp_m=0.5)
    true_pose = (2.0, 2.0, math.pi / 2)
    ranges, amin, ainc = _scan_at(segs, true_pose)
    prior = (true_pose[0] + 1.2, true_pose[1], true_pose[2])   # 1.2 m > clamp
    est, info = loc.correct(prior, ranges, amin, ainc)
    assert info["rejected"]
    assert est == prior                                       # no wild jump


def test_robust_to_outlier_returns():
    segs = load_segments()
    loc = ScanMatchLocalizer(segs, scan_offset_x=0.0)
    true_pose = (8.0, 2.0, 0.0)
    ranges, amin, ainc = _scan_at(segs, true_pose)
    ranges = list(ranges)
    for k in range(0, len(ranges), 11):                       # ~9% spurious short returns
        ranges[k] = 0.4
    prior = (true_pose[0] + 0.3, true_pose[1] - 0.2, true_pose[2] + math.radians(5))
    est, info = loc.correct(prior, ranges, amin, ainc)
    assert math.hypot(est[0] - true_pose[0], est[1] - true_pose[1]) < 0.08, info
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_scan_match_localizer.py -v`
Expected: FAIL with `AttributeError: 'ScanMatchLocalizer' object has no attribute 'correct'`.

- [ ] **Step 3: Implement the solver and `correct()`**

Append these methods to `ScanMatchLocalizer` in `tugbot_maze/scan_match_localizer.py`:

```python
    def _solve_one(self, pose: Pose2D, pts: np.ndarray, max_corr_dist):
        """One associate + gated Gauss-Newton step. Return (delta(3,), info)."""
        foot, n, dist = self._associate(pts)
        r = np.sum((pts - foot) * n, axis=1) - self.wall_half_thickness_m   # signed residual
        if max_corr_dist is None:
            inl = np.ones(pts.shape[0], dtype=bool)
        else:
            inl = np.abs(dist - self.wall_half_thickness_m) <= max_corr_dist
        n_inl = int(np.count_nonzero(inl))
        info: Dict = {"n_inliers": n_inl, "fell_back": set()}
        if n_inl < self.min_inliers:
            info["under_inliers"] = True
            return np.zeros(3), info
        ni, ri, pi = n[inl], r[inl], pts[inl]
        cx, cy = pose[0], pose[1]
        J = np.stack([ni[:, 0], ni[:, 1],
                      -ni[:, 0] * (pi[:, 1] - cy) + ni[:, 1] * (pi[:, 0] - cx)], axis=1)
        JtJ = J.T @ J
        Jtb = -J.T @ ri
        evals, evecs = np.linalg.eigh(JtJ[:2, :2])           # ascending
        yaw_info = float(JtJ[2, 2])
        damp = self.lm_lambda * max(float(np.trace(JtJ)), 1.0)
        delta = np.linalg.solve(JtJ + damp * np.eye(3), Jtb)
        fell = set()
        if evals[0] < self.min_eig:                          # suppress weak translational dir
            v = evecs[:, 0]
            delta[:2] = delta[:2] - (delta[:2] @ v) * v
            fell.add("x" if abs(v[0]) >= abs(v[1]) else "y")
        if yaw_info < self.min_yaw_info:                     # suppress weak yaw
            delta[2] = 0.0
            fell.add("yaw")
        info.update({"eig_min": float(evals[0]), "eig_max": float(evals[1]),
                     "yaw_info": yaw_info, "fell_back": fell,
                     "residual_rms": float(np.sqrt(np.mean(ri * ri)))})
        return delta, info

    def correct(self, prior_pose: Pose2D, ranges, angle_min, angle_inc):
        """Return (corrected_pose, info). info: n_inliers, residual_rms, eig_min, eig_max,
        fell_back (set in {'x','y','yaw'}), rejected (bool), iters (int)."""
        x0, y0, t0 = float(prior_pose[0]), float(prior_pose[1]), float(prior_pose[2])
        pose: Pose2D = (x0, y0, t0)
        info: Dict = {"n_inliers": 0, "residual_rms": float("nan"), "eig_min": 0.0,
                      "eig_max": 0.0, "fell_back": set(), "rejected": False, "iters": 0}
        for it in range(self.max_iters):
            pts = self._beams_to_points(pose, ranges, angle_min, angle_inc)
            max_corr = None if it == 0 else self.max_corr_dist_m
            delta, step = self._solve_one(pose, pts, max_corr)
            info.update(step)
            info["iters"] = it + 1
            if step.get("under_inliers"):
                info["rejected"] = True
                return (x0, y0, t0), info
            pose = (pose[0] + float(delta[0]), pose[1] + float(delta[1]),
                    _wrap(pose[2] + float(delta[2])))
            if math.hypot(float(delta[0]), float(delta[1])) < self.tol_m and abs(float(delta[2])) < 1e-3:
                break
        if (math.hypot(pose[0] - x0, pose[1] - y0) > self.trans_clamp_m
                or abs(_wrap(pose[2] - t0)) > self.yaw_clamp_rad):
            info["rejected"] = True
            return (x0, y0, t0), info
        return pose, info
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_scan_match_localizer.py -v`
Expected: PASS (all tests). If `min_eig` / `min_yaw_info` need tuning so the open-axis test gates correctly and the recovery tests stay observable, adjust those constructor defaults (the only tunables) and re-run; do not weaken the assertions.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/scan_match_localizer.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_scan_match_localizer.py
git commit -m "feat: ScanMatchLocalizer ICP solver + correct() with observability gate

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: Node integration (`pose_source=scan_match`)

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/flood_fill_solver.py`

**Context:** `_lookup_pose()` is called by both the 10 Hz control tick and the 5 s diag tick, so the scan-match correction must be **idempotent per scan** (guard on a scan sequence counter) to avoid double-applying the odom delta. During `startup`/`entering` (or before the first scan), track the live SLAM `map->base_link` TF and seed the anchors, so `driving` begins well-seeded.

- [ ] **Step 1: Add imports and the scan sequence counter**

In `tugbot_maze/flood_fill_solver.py`, extend the `pose_tracking` import and add the localizer + segments imports:

```python
from tugbot_maze.pose_tracking import compose_2d, quat_to_yaw, odom_prior
from tugbot_maze.scan_match_localizer import ScanMatchLocalizer
from tugbot_maze.maze_sim import load_segments
```

In `__init__`, after `self.junctions = JunctionLog()`, add the scan-match state and construct the localizer when selected:

```python
        self._scan_seq = 0
        self._sm_corrected = None
        self._sm_last_odom = None
        self._sm_seq = -1
        self._sm_info = None
        self.localizer = (ScanMatchLocalizer(load_segments())
                          if self.pose_source == 'scan_match' else None)
```

Change `_scan_cb` to bump the counter:

```python
    def _scan_cb(self, msg):
        self.scan_msg = msg
        self._scan_seq += 1
```

- [ ] **Step 2: Add the `scan_match` branch to `_lookup_pose` and the helper**

At the top of `_lookup_pose`, before the existing `odom_locked` logic, add:

```python
        if self.pose_source == 'scan_match':
            return self._lookup_scan_match()
```

Add the new method after `_lookup_pose`:

```python
    def _lookup_scan_match(self):
        odom_base = self._lookup_tf(self.odom_frame, self.base_frame)
        map_base = self._lookup_tf(self.map_frame, self.base_frame)
        # Bootstrap: track live SLAM TF and seed anchors until driving with a scan in hand.
        if self.phase in ('startup', 'entering') or self.scan_msg is None or odom_base is None:
            if map_base is not None:
                self._sm_corrected = map_base
                self._sm_last_odom = odom_base
                self._sm_seq = self._scan_seq
            return map_base if map_base is not None else self._sm_corrected
        if self._sm_corrected is None or self._sm_last_odom is None:
            if map_base is not None:
                self._sm_corrected = map_base
            self._sm_last_odom = odom_base
            return self._sm_corrected
        if self._scan_seq == self._sm_seq:                 # already corrected for this scan
            return self._sm_corrected
        prior = odom_prior(self._sm_corrected, self._sm_last_odom, odom_base)
        s = self.scan_msg
        est, info = self.localizer.correct(prior, s.ranges, s.angle_min, s.angle_increment)
        self._sm_corrected = est
        self._sm_last_odom = odom_base
        self._sm_seq = self._scan_seq
        self._sm_info = info
        return est
```

- [ ] **Step 3: Add the DIAG `MATCH` line**

In `_diag_tick`, after the existing `DIAG ...` log call, add:

```python
        if self.pose_source == 'scan_match' and self._sm_info is not None:
            i = self._sm_info
            self.get_logger().info(
                'MATCH rms=%.3f n=%d eigmin=%.2f fb=%s rejected=%s'
                % (i.get('residual_rms', float('nan')), i.get('n_inliers', 0),
                   i.get('eig_min', 0.0),
                   ','.join(sorted(i.get('fell_back', set()))) or '-',
                   i.get('rejected', False)))
```

- [ ] **Step 4: Verify import-safety and the full ROS-free suite**

The node is not instantiated offline (needs rclpy), so its verification is import-safety plus the unchanged suite.

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -c "import tugbot_maze.flood_fill_solver"`
Expected: no output, exit 0 (imports resolve).

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_maze_motion_sim.py test/test_scan_match_localizer.py test/test_pose_tracking.py -q`
Expected: all pass except the 3 pre-existing `[0.05-*]` xfails in `test_maze_motion_sim.py` (still xfailed, not xpassed).

- [ ] **Step 5: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/flood_fill_solver.py
git commit -m "feat: pose_source=scan_match wiring in flood_fill_solver (idempotent per scan)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: Closed-loop offline validation (localizer in the motion loop under drift)

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_motion_sim.py`

**Context:** This proves the mechanism end-to-end offline: re-anchoring to the known map every tick keeps the corrected pose locked to truth even as wheel odometry drifts arbitrarily, so the motion FSM behaves like the clean (drift-0) case. Construct the localizer with `scan_offset_x=0.0` (the sim scans from body center). Seed `corrected` from the true entrance pose.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_maze_motion_sim.py` (add imports at the top of the file):

```python
from tugbot_maze.scan_match_localizer import ScanMatchLocalizer
from tugbot_maze.pose_tracking import odom_prior


def _run_scan_match(drift, dt=0.1, max_steps=30000):
    segs = load_segments()
    sim = MazeSim(segs, cell_center(ENTRANCE_CELL), 0.0, inertia=True, odom_drift_per_m=drift)
    loc = ScanMatchLocalizer(segs, scan_offset_x=0.0)         # sim scans from body center
    m = MazeMotion()
    t = 0.0
    corrected = sim.pose                                       # known entrance (truth) seed
    last_odom = sim.reported_pose                             # == truth at t0 (drift starts at 0)
    max_pose_err = 0.0
    max_desync = 0
    collided = False
    for _ in range(max_steps):
        ranges, amin, ainc = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        cur_odom = sim.reported_pose
        prior = odom_prior(corrected, last_odom, cur_odom)
        corrected, _info = loc.correct(prior, ranges, amin, ainc)
        last_odom = cur_odom
        max_pose_err = max(max_pose_err, math.hypot(corrected[0] - sim.x, corrected[1] - sim.y))
        v, w, done = m.step(corrected, (ranges, amin, ainc), t)
        if done:
            return True, collided, max_desync, max_pose_err
        sim.step(v, w, dt)
        if sim.collides(sim.x, sim.y, sim.yaw):
            collided = True
        if m.phase == 'center':
            tc = pose_to_cell(sim.x, sim.y)
            max_desync = max(max_desync, abs(tc[0] - m.cell[0]) + abs(tc[1] - m.cell[1]))
        t += dt
    return (m.cell == EXIT_CELL), collided, max_desync, max_pose_err


def test_scan_match_pose_tracks_truth_under_high_drift():
    """At 10%/m odom drift (raw odom would be off by >1 cell mid-maze), the corrected
    pose stays locked to truth because it re-anchors to the known map every tick."""
    _reached, _collided, _desync, max_pose_err = _run_scan_match(0.10, max_steps=4000)
    assert max_pose_err < 0.15, f"scan-match pose drifted from truth: {max_pose_err:.3f} m"


def test_scan_match_solves_under_drift_that_desyncs_raw_odom():
    """Closed-loop solve on the corrected pose at 5%/m drift: reaches the exit, the cell
    tracker never desyncs, and the body never collides (accurate pose -> correct centering)."""
    reached, collided, max_desync, max_pose_err = _run_scan_match(0.05)
    assert max_pose_err < 0.15, f"pose drifted from truth: {max_pose_err:.3f} m"
    assert reached, "scan-match solve did not reach the exit"
    assert max_desync <= 1, f"cell tracker desynced by {max_desync}"
    assert not collided, "robot collided during the scan-match solve"
```

- [ ] **Step 2: Run tests to verify they fail (then pass once Tasks 2-3 are in)**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_maze_motion_sim.py -k scan_match -v`
Expected before Tasks 2-3 exist: import/collection error. After Tasks 2-3: the tests run.

- [ ] **Step 3: Make them pass**

No new production code — these exercise the localizer built in Tasks 2-3 plus the helpers from Task 1. If `test_scan_match_solves_under_drift_that_desyncs_raw_odom` fails on `reached`/`collided` at 0.05 (the corrected pose is accurate, so the run should mirror the clean drift-0 solve), first confirm `test_scan_match_pose_tracks_truth_under_high_drift` passes (the core claim); if pose-lock holds but the solve still fails, that is a real finding — report it as DONE_WITH_CONCERNS rather than weakening the pose-lock assertion or raising tolerances.

- [ ] **Step 4: Run to verify they pass**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_maze_motion_sim.py -k scan_match -v`
Expected: PASS (2 passed).

- [ ] **Step 5: Commit**

```bash
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_motion_sim.py
git commit -m "test: closed-loop scan-match localization holds pose-lock + solves under drift

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: Full-suite green + Gazebo controlled-batch acceptance (user-gated run)

**Files:** none (verification + handoff).

**Context:** The Gazebo run is launched by the user (the "set up the run" gate); do not auto-launch it. This task confirms the offline suite is green and documents the acceptance procedure.

- [ ] **Step 1: Run the focused ROS-free suite**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_scan_match_localizer.py test/test_pose_tracking.py test/test_maze_motion_sim.py -q`
Expected: all pass; the 3 pre-existing `[0.05-*]` xfails remain xfailed (not xpassed); no new failures.

- [ ] **Step 2: Confirm no regression in the broader ROS-free unit tests**

Run the core unit-test files that do not depend on the missing `tools/`/`doc/` phase artifacts (e.g. `test_flood_fill_brain.py test_cell_walls.py test_hop_controller.py test_wall_localize.py test_maze_sim.py`):
`cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_flood_fill_brain.py test/test_cell_walls.py test/test_hop_controller.py test/test_wall_localize.py test/test_maze_sim.py -q`
Expected: all pass (no localizer-induced breakage).

- [ ] **Step 3: Document the Gazebo acceptance command**

Acceptance is measured against the `d356f4a` controlled baseline (`log/batch_diag_20260628_175502`) using the existing batch tool with the new pose source. The user clears stray sims and triggers:
`cd ros2_ws_tugbot_nav_20260614 && export DISPLAY=:1 && bash tools/batch_diagnose_floodfill.sh 8 1200 false true scan_match true`
Then compare `log/batch_diag_<new>/report.txt` to the baseline. **Pass criteria:** DESYNC primary-mode share drops sharply, heavy-desync prevalence falls well below 75%, best-depth improves; stretch goal = first `EXIT_REACHED`.

- [ ] **Step 4: Commit (if any doc/notes were added) and hand off**

No code commit required for this task. Report the offline results and present the Gazebo acceptance command for the user to run.

---

## Self-Review

**Spec coverage:** §3 algorithm → Tasks 2-3 (beams→points, association, residual w/ half-thickness, Jacobian, GN loop). §4 interfaces → `correct()` signature (Task 3) + `_lookup_scan_match` (Task 4). §6 robustness → observability gate, outlier reject, min-inliers, clamp (Task 3, tested). §8 testing → Tasks 2,3,5 (maze_sim forward model: recovery, no-bias, gating, outliers, clamp, closed-loop). §9 acceptance → Task 6. All covered.

**Placeholder scan:** No TBD/stubs; every code step shows complete code; tunables have concrete defaults.

**Type consistency:** `correct(prior_pose, ranges, angle_min, angle_inc) -> (Pose2D, dict)` consistent across Task 3, Task 4, Task 5. `odom_prior(last_corrected, last_odom, cur_odom)` consistent (Task 1 def, Tasks 4-5 use). `_associate -> (foot, normal, dist)` and `_solve_one -> (delta, info)` used consistently. `info['fell_back']` is a `set` everywhere (DIAG joins it sorted). `scan_offset_x=0.0` in every test; `SCAN_OFFSET_X` default in the node.

**Note on numeric tunables:** `min_eig=5.0`, `min_yaw_info=3.0`, `lm_lambda=1e-3` are starting values; Task 3 Step 4 permits adjusting only these (not the assertions) so the open-axis gating and recovery tests both hold.
