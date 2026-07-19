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
from tugbot_maze.scan_match_localizer import ScanMatchLocalizer, _wrap

Segment = Tuple[float, float, float, float]
Cell = Tuple[int, int]

# ICP is grounded when at least one committed wall endpoint/midpoint is within this
# distance of the prior pose.  CELL_SIZE_M/2 = 1.0 m means the robot must be inside
# (or at the exact boundary of) a committed cell's coverage.  Any farther and the
# robot is in new territory -- fall back to the odom prior.
_LOCAL_RADIUS: float = CELL_SIZE_M / 2.0   # = 1.0 m

# --- yaw-only fallback (2026-07-19 spec) -------------------------------------
# Junction cells have almost no walls, so the gates above deterministically
# reject full ICP there and yaw bias stands uncorrected through dead-reckoning
# windows (the central-junction trap's fuel). Wall DIRECTION is observable at
# range even where 2-axis position is not (the 2026-06 fundamental wall was
# position, never yaw), so when full ICP is unavailable we fit rotation ONLY.
# A 1-DOF fit cannot drag position; x,y pass through bit-identical.
YAW_WINDOW_RAD = 0.2      # search window for the residual scan
YAW_GRID_STEP = 0.01      # 0.57deg resolution; convergence target is 0.02 rad
YAW_MIN_INLIERS = 60      # beams that must land near reference at the optimum
YAW_MIN_IMPROVE = 0.20    # required relative rms improvement vs d_theta = 0
YAW_STEP_CLAMP = 0.1      # max applied per-tick correction (rad)
# Absolute inlier floor for the SATURATED accept: a rotation fit on a handful
# of points is noise (corner-aliasing probe: bias 1.2 rad -> n0=0, best_n=4,
# relative improvement inf -- must decline). Half the interior floor because
# saturated geometry structurally caps achievable counts.
YAW_SATURATED_MIN_INLIERS = YAW_MIN_INLIERS // 2   # = 30
# rms hard ceiling for ANY accept (2026-07-19 amendment, "yaw-only insurance"):
# wall_half_thickness_m=0.12 makes healthy fits ~0.11; forensics showed this
# would have declined R3's weakest accept (n=182, rms=0.152) -- a fit that
# looked plausible on inlier count + improvement alone but was geometrically
# noisy enough that it should not have been trusted.
YAW_RMS_CEILING = 0.15

# --- clamp-lock escape (2026-07-19 amendment) --------------------------------
# Forensics (run R3): the inner ScanMatchLocalizer.correct() clamp
# ("reject-if-exceeds") discards a CONVERGED fit wholesale whenever the
# correction magnitude exceeds trans_clamp_m/yaw_clamp_rad, even when the fit
# itself is healthy (R3: n=421, eigmin=22) -- 575 consecutive rejections,
# nothing downstream can spend the recoverable position error ("clamp-lock").
# After CLAMP_LOCK_STREAK consecutive HEALTHY clamp-type rejections, spend a
# small bounded step toward the discarded converged pose instead of dead
# reckoning forever. This is deliberately rate-limited (at most one escape
# per 3 healthy rejections) and bounded (<=CLAMP_ESCAPE_TRANS m,
# <=YAW_STEP_CLAMP rad) -- the same conservative-bounded-step philosophy as
# the yaw-only fallback, just spending 2-DOF converged evidence instead of a
# 1-DOF rotation-only fit.
CLAMP_LOCK_STREAK = 3          # consecutive healthy clamp-type rejections before escaping
CLAMP_LOCK_MIN_INLIERS = 100   # 'healthy' floor: a converged, well-supported inner fit
CLAMP_ESCAPE_TRANS = 0.15      # m -- max translation spent per escape (vector scaled, not clipped)
# yaw escape step reuses YAW_STEP_CLAMP (0.1 rad) -- same constant as yaw-only.


def yaw_only_correct(prior_pose, ranges, angle_min, angle_inc, icp, *,
                     window=YAW_WINDOW_RAD, grid=YAW_GRID_STEP,
                     min_inliers=YAW_MIN_INLIERS, min_improve=YAW_MIN_IMPROVE,
                     step_clamp=YAW_STEP_CLAMP,
                     saturated_min_inliers=YAW_SATURATED_MIN_INLIERS,
                     rms_ceiling=YAW_RMS_CEILING):
    """1-DOF fallback: grid-search d_theta against icp's reference. Position
    NEVER changes.

    Candidate selection (per d_theta): maximize inlier count (points within
    icp's max_corr_dist_m), tie-broken by minimum rms. Inlier count is the
    trustworthy signal here -- verified empirically against this module's
    own perimeter-only fixture: with few correspondences the point-to-
    segment rms is dominated by which few points happen to remain and is
    NOT monotone across the search window, while inlier count rises
    cleanly and monotonically as d_theta approaches the true correction
    (even outside the window, short of full convergence).

    Two acceptance branches, keyed on whether the optimum is interior to
    the search window or saturates at its edge:

      - Interior optimum (a genuine local minimum -- the true bias is
        within +/-window): require best_n >= min_inliers AND the rms
        improves by >= min_improve relative to d_theta=0. Improvement edge
        cases: rms0 = inf (prior has NO inliers at all) counts as full
        improvement (1.0); rms0 <= 0 (prior already perfect) counts as 0,
        so an exact prior always declines rather than chasing noise.

      - Saturated optimum (best_n found at +/-window -- the search was
        truncated and the true bias likely lies further out): the rms
        ratio is unusable here (still noisy, no interior turning point --
        verified against a 0.3 rad true bias) and the absolute min_inliers
        floor is structurally unreachable (correspondences drop off
        approaching the tail of a large, uncorrected bias). Instead
        require the RELATIVE INLIER increase vs d_theta=0 to be
        >= min_improve -- a real bias produces a large, monotone rise in
        matched beams toward the window edge; noise does not -- AND
        best_n >= saturated_min_inliers as an absolute floor. The floor
        matters precisely when n0 = 0: the relative increase is then inf
        for ANY nonzero best_n, and without the floor a corner-aliasing
        handful of matches (bias 1.2 rad probe: n0=0, best_n=4) would be
        accepted; a rotation fit on a handful of points is noise.

    The INTERIOR branch ALSO requires best_rms <= rms_ceiling (2026-07-19
    amendment, "yaw-only insurance"): inlier count and improvement alone can
    still pass a geometrically noisy fit (forensics: R3's weakest accept,
    n=182, rms=0.152); healthy interior-optimum fits cluster well under the
    ceiling given wall_half_thickness_m=0.12. The SATURATED branch does NOT
    gain this ceiling: its own rms is already documented above as an
    unreliable signal (no interior turning point), which is exactly why it
    uses the relative-inlier-increase criterion instead -- a legitimate large
    over-window bias (the case this branch exists to walk down over
    successive ticks) systematically produces a higher rms at its
    window-edge optimum than the ceiling (measured: bias 0.3 rad -> rms
    0.165), so an absolute rms ceiling there would defeat the branch's own
    purpose rather than reject noise.

    The applied step is clamped to +/-step_clamp UNCONDITIONALLY, on both
    branches -- a saturating clamp: a bounded step is always applied, and
    biases larger than step_clamp converge over successive ticks (each
    call re-centers the search window on the corrected prior; see
    test_repeated_ticks_converge_without_oscillation). This differs in
    PHILOSOPHY from the full 3-DOF ICP, whose yaw_clamp_rad is a
    reject-if-exceeds gate (an over-large correction is discarded
    outright, prior returned); the fallback instead applies a deliberately
    MORE conservative magnitude (0.1 < ~0.175 rad) each tick, which is
    what lets it walk down large biases the full ICP would simply refuse.

    Cost: one eval per grid point -- 41 at defaults (the d_theta=0 eval
    seeds the loop and is not repeated); measured ~17 ms/call at
    production scale (900 beams x 53 segments), growing with the number
    of committed segments -- acceptable for <=10 Hz invocation on
    rejected ticks only.

    Note: icp._beams_to_points and icp._associate never return None -- on
    empty input they return an empty (0,2) points array / dist=np.full(0,
    inf) respectively -- so the guards below key off len()/mask.sum()==0,
    not identity/None checks."""
    x, y, yaw = float(prior_pose[0]), float(prior_pose[1]), float(prior_pose[2])
    max_corr = icp.max_corr_dist_m    # SAME correspondence threshold as full ICP

    def eval_at(dth):
        pts = icp._beams_to_points((x, y, yaw + dth), ranges, angle_min, angle_inc)
        if pts.shape[0] == 0:
            return math.inf, 0
        foot, n, dist = icp._associate(pts)
        mask = dist <= max_corr
        n_in = int(mask.sum())
        if n_in == 0:
            return math.inf, 0
        return float(np.sqrt(np.mean(dist[mask] ** 2))), n_in

    rms0, n0 = eval_at(0.0)
    best_dth, best_rms, best_n, best_k = 0.0, rms0, n0, 0
    steps = int(round(window / grid))
    for k in range(-steps, steps + 1):
        if k == 0:
            continue                      # seeded above from (rms0, n0)
        dth = k * grid
        rms, n_in = eval_at(dth)
        # More inliers wins; equal inliers -> lower rms (negated for max-compare).
        if (n_in, -rms) > (best_n, -best_rms):
            best_dth, best_rms, best_n, best_k = dth, rms, n_in, k

    if best_n == 0:
        return tuple(prior_pose), {'rejected': True,
                                   'reason': 'yaw_only_declined',
                                   'residual_rms': best_rms,
                                   'n_inliers': best_n}

    saturated = abs(best_k) == steps
    if saturated:
        improve = math.inf if n0 == 0 else (best_n - n0) / n0
        # No rms_ceiling here -- see docstring: rms is documented unreliable
        # on this branch, and legitimate large over-window biases exceed the
        # ceiling by design (this branch's own reason to exist).
        accepted = (best_n >= saturated_min_inliers
                    and improve >= min_improve)
    else:
        improve = 1.0 if not math.isfinite(rms0) else (
            0.0 if rms0 <= 0.0 else 1.0 - (best_rms / rms0))
        accepted = (best_n >= min_inliers and improve >= min_improve
                    and best_rms <= rms_ceiling)

    if not accepted:
        return tuple(prior_pose), {'rejected': True,
                                   'reason': 'yaw_only_declined',
                                   'residual_rms': best_rms,
                                   'n_inliers': best_n}
    applied = max(-step_clamp, min(step_clamp, best_dth))   # UNCONDITIONAL
    return ((x, y, yaw + applied),
            {'rejected': False, 'reason': 'yaw_only',
             'fell_back': {'yaw_only'}, 'yaw_step': applied,
             'saturated': saturated, 'improve': improve,
             'residual_rms': best_rms, 'n_inliers': best_n})


def _is_clamp_type_reject(info, *, min_inliers=CLAMP_LOCK_MIN_INLIERS) -> bool:
    """True iff an inner ScanMatchLocalizer rejection is the clamp-lock kind:
    a converged, well-supported fit (info['conv_pose'] present, n_inliers over
    the healthy floor) that was discarded purely by the reject-if-exceeds
    clamp -- NOT an inlier-floor (under_inliers) reject, which carries no
    recoverable converged pose at all."""
    return (bool(info.get('rejected'))
            and not info.get('under_inliers')
            and info.get('n_inliers', 0) >= min_inliers
            and 'conv_pose' in info)


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


def local_reference_cells(committed_cells, current_cell, sensed_cells):
    """Reference cells for online localization: the committed cells (global drift-free
    anchors) PLUS the current cell and its already-sensed neighbours -- the immediate local
    reference that keeps ICP drift-free during first-pass exploration, not just on revisits.
    Only sensed cells contribute (an unsensed cell has no known walls)."""
    sensed = set(sensed_cells)
    local = {current_cell}
    for dx, dy in DIRS.values():
        local.add((current_cell[0] + dx, current_cell[1] + dy))
    return set(committed_cells) | (local & sensed)


def confirmed_wall_segments(brain, cells: Iterable[Cell]) -> List[Segment]:
    """Grid-snapped segments for every WALL edge of the given cells, de-duplicated (a wall is
    shared by two cells). The CALLER chooses which cells (see local_reference_cells: committed
    anchors + the current cell and its sensed neighbours); passing only committed cells keeps a
    poorly-sensed wall out of the reference, while the local window trades a little of that
    safety for a drift-free anchor during first-pass exploration."""
    seen = set()
    for cell in cells:
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
        self._clamp_streak = 0    # consecutive healthy clamp-type inner-ICP rejections
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
        beams survive, the ICP rejects (under_inliers) and returns the odom prior.

        Only valid beams (finite, positive, within usable_range_m -- mirroring
        _beams_to_points' validity predicate) are considered: trig on raw inf
        ranges yields inf*0.0 = nan RuntimeWarnings, and invalid/out-of-range
        beams are excluded by _beams_to_points downstream anyway, so leaving
        them untouched in the returned list is semantics-preserving."""
        if self._icp._a.shape[0] == 0:
            return ranges
        r = np.asarray(ranges, dtype=float)
        valid = np.isfinite(r) & (r > 0.0) & (r <= self._icp.usable_range_m)
        if not np.any(valid):
            return ranges                                             # nothing maskable
        idx = np.nonzero(valid)[0]
        rv = r[idx]
        x, y, th = float(prior_pose[0]), float(prior_pose[1]), float(prior_pose[2])
        c, s = math.cos(th), math.sin(th)
        ang = angle_min + idx.astype(float) * angle_inc
        bx = self._icp.scan_offset_x + rv * np.cos(ang)
        by = rv * np.sin(ang)
        px = x + c * bx - s * by
        py = y + s * bx + c * by
        pts = np.stack([px, py], axis=1)                              # (V, 2)
        ap = pts[:, None, :] - self._icp._a[None, :, :]              # (V, S, 2)
        t = np.clip(
            np.sum(ap * self._icp._e[None, :, :], axis=2) / self._icp._len2[None, :],
            0.0, 1.0)
        foot = self._icp._a[None, :, :] + t[:, :, None] * self._icp._e
        dist = np.sqrt(np.sum((pts[:, None, :] - foot) ** 2, axis=2)).min(axis=1)  # (V,)
        threshold = self._icp.max_corr_dist_m + self._icp.wall_half_thickness_m
        far = dist > threshold
        if not np.any(far):
            return ranges                                             # fast path: nothing masked
        out = r.copy()
        out[idx[far]] = np.inf
        return out.tolist()

    def _yaw_fallback(self, prior_pose, ranges, angle_min, angle_inc, gate_reason,
                      already_masked=False):
        """Try the 1-DOF fallback on a rejection path. Far beams pollute the
        rotation fit exactly as they pollute full ICP, so gate-1/2 paths premask
        here; the inner-ICP path arrives already masked."""
        beams = ranges if already_masked else self._mask_far_beams(
            prior_pose, ranges, angle_min, angle_inc)
        est, info = yaw_only_correct(prior_pose, beams, angle_min, angle_inc, self._icp)
        if info['rejected']:
            info['reason'] = gate_reason + '+yaw_only_declined'
        else:
            info['gate_reason'] = gate_reason
        return est, info

    def _clamp_lock_escape(self, prior_pose, info):
        """Spend a bounded step from the odom prior toward the discarded
        converged pose (info['conv_pose']) after CLAMP_LOCK_STREAK consecutive
        HEALTHY clamp-type rejections (2026-07-19 amendment). Translation is
        scaled (direction preserved) to at most CLAMP_ESCAPE_TRANS m; yaw is
        clamped to +/-YAW_STEP_CLAMP rad, same constant as yaw-only. This is
        the only place a rejected inner-ICP fit's converged pose is ever
        spent -- see the module-level clamp-lock forensics above
        yaw_only_correct."""
        px, py, pyaw = float(prior_pose[0]), float(prior_pose[1]), float(prior_pose[2])
        cx, cy, cyaw = info['conv_pose']
        dx, dy = cx - px, cy - py
        conv_dist = math.hypot(dx, dy)
        if conv_dist > CLAMP_ESCAPE_TRANS:
            scale = CLAMP_ESCAPE_TRANS / conv_dist
            dx *= scale
            dy *= scale
        dyaw = max(-YAW_STEP_CLAMP, min(YAW_STEP_CLAMP, _wrap(cyaw - pyaw)))
        new_pose = (px + dx, py + dy, _wrap(pyaw + dyaw))
        return new_pose, {'rejected': False, 'reason': 'clamp_lock_escape',
                          'fell_back': {'clamp_escape'},
                          'residual_rms': info.get('residual_rms', float('nan')),
                          'n_inliers': info.get('n_inliers', 0),
                          'conv_dist': conv_dist, 'yaw_step': dyaw}

    def correct(self, prior_pose, ranges, angle_min, angle_inc, interior_segments):
        """Return (corrected_pose, info).

        Gates (in order):
          1. Sparse-interior: skip if fewer than min_interior_segs interior walls.
          2. Locality: skip if no interior segment is within local_radius of the prior.
          3. Beam premasking: remove beams too far from any reference wall.
          4. ICP (rejects if under min_inliers after premasking, or clamp-rejects an
             over-large converged correction).

        Every rejection path (1, 2, and 4) tries the 1-DOF yaw-only fallback before
        giving up to pure odom: wall DIRECTION is observable even where the full
        3-DOF fit is gated (junction cells, sparse local reference). x,y always pass
        through bit-identical when the fallback runs; see yaw_only_correct.

        Gates 1/2 never touch _clamp_streak (no inner ICP ran, so clamp-type vs
        not is undetermined). On the inner-ICP path (4), a clamp-type rejection
        (see _is_clamp_type_reject) increments the streak; any other outcome --
        an accepted ICP correction, a non-clamp rejection (under_inliers), or an
        accepted yaw-only fallback -- resets it to 0. At CLAMP_LOCK_STREAK
        consecutive clamp-type rejections, _clamp_lock_escape runs INSTEAD of
        the yaw-only fallback (2-DOF converged evidence is strictly more
        informative) and the streak resets (at most one escape per 3 healthy
        rejections -- a natural rate limit)."""
        sig = frozenset(tuple(s) for s in interior_segments)
        if sig != self._sig:
            self._rebuild(interior_segments)
            self._sig = sig
        # Gate 1: sparse-interior
        if len(interior_segments) < self._min_interior:
            return self._yaw_fallback(prior_pose, ranges, angle_min, angle_inc,
                                      'sparse_interior_gate')
        # Gate 2: locality — robot must be near at least one committed wall
        if not self._has_local_interior(prior_pose, interior_segments):
            return self._yaw_fallback(prior_pose, ranges, angle_min, angle_inc,
                                      'no_local_interior_walls')
        # Gate 3 + ICP: pre-mask far beams then run the ICP
        masked = self._mask_far_beams(prior_pose, ranges, angle_min, angle_inc)
        est, info = self._icp.correct(prior_pose, masked, angle_min, angle_inc)
        if info.get('rejected'):
            if _is_clamp_type_reject(info):
                self._clamp_streak += 1
            else:
                self._clamp_streak = 0
            if self._clamp_streak >= CLAMP_LOCK_STREAK:
                self._clamp_streak = 0
                return self._clamp_lock_escape(prior_pose, info)
            # ScanMatchLocalizer.correct's info dict carries no 'reason' key (only
            # 'rejected' + an 'under_inliers' flag on the inlier-floor path); the
            # clamp-exceeded path sets neither. 'icp_rejected' is therefore the
            # generic label actually used on every inner-ICP rejection today.
            inner = str(info.get('reason', 'icp_rejected'))
            est2, info2 = self._yaw_fallback(prior_pose, masked, angle_min,
                                             angle_inc, inner, already_masked=True)
            if not info2['rejected']:
                self._clamp_streak = 0
                return est2, info2
            info['reason'] = inner + '+yaw_only_declined'
        else:
            self._clamp_streak = 0
        return est, info
