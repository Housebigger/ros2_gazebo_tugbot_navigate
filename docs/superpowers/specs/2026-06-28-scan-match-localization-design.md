# Scan-to-Known-Map Localization — Design

**Date:** 2026-06-28
**Status:** design approved (user confirmed 2026-06-28)
**Topic:** Replace drifting `odom_locked` dead-reckoning with an absolute map-frame
pose obtained by matching the live 360° LIDAR scan against the known maze wall map.

---

## 1. Problem & evidence

The autonomous flood-fill solver never completes the maze. The controlled baseline
(8 runs, current main `d356f4a`, `log/batch_diag_20260628_175502`) shows the dominant
failure is **localization desync**, not locomotion:

- 8/8 TIMEOUT (0 exit, 0 infra).
- **DESYNC = 62% primary failure mode**; **75% of runs** show planner-cell ↔ odom-cell
  divergence of exactly 2 cells; **100% of runs** fired the `RECONCILE` recovery.
- 6/8 die >12 m from the exit; recurring wedge at cell (3,4).

Root cause: `odom_locked` pose = frozen `map→odom` ∘ live wheel `odom→base_link`. Wheel
odometry accumulates lateral drift; once the believed cell is off by ≥1, per-cell wall
sensing and route decisions corrupt and the run is lost. Wall-referenced single-axis
re-anchoring (`perimeter_offset`, `cell_center_offset`) and offset-adaptive sensing were
tried and plateaued — they rely on the ±22° cardinal-window projection that itself fails
off-center in open regions.

## 2. Goal

Compute an **absolute map-frame pose** every tick by matching the live `/scan` against
the known wall-segment map, seeded by the odom prior. Route discovery stays fully
autonomous (the brain/flood-fill is untouched). Using the known map for *localization*
(not for *routing*) is in scope and approved.

**Success:** in a controlled Gazebo batch on `pose_source=scan_match`, the DESYNC mode
collapses versus the `d356f4a` baseline, best-depth increases, and we target the first
`EXIT_REACHED`.

## 3. Approach — point-to-line ICP (scan-to-map)

Standard scan-to-map matching against line segments. No external dependencies; pure
NumPy. Each control tick (10 Hz):

1. **Predict (prior).** `prior = last_corrected_pose ⊕ odom_delta`, where `odom_delta` is
   the change in `odom→base_link` since the previous tick. The prior keeps the solution in
   the correct basin (avoids aliasing onto a repeated corridor) — we do **local
   correction**, never global relocalization.
2. **Project beams to points.** For each valid beam `i` (range in usable window), the
   endpoint in the map frame is:
   `p_i = T(prior) · ( (SCAN_OFFSET_X, 0) + range_i · (cos θ_i, sin θ_i) )`
   where `θ_i = angle_min + i·angle_inc` (sensor frame == base_link axes; the `scan_omni`
   static TF is a pure translation `x=-0.1855, y=0, z=0.5318`, no rotation), and `T(prior)`
   is the SE(2) body→map transform.
3. **Associate.** For each point `p_i`, find the nearest known wall segment (vectorized
   point-to-segment). Keep the segment's robot-facing unit normal `n_i` and the foot point
   `q_i`.
4. **Residual (wall half-thickness aware).** Beams read the wall **face**, segments are
   **centerlines**. Orient `n_i` toward the robot. The endpoint should sit
   `wall_half_thickness` (0.12 m) in front of the centerline, so the signed residual is
   `r_i = n_i · (p_i − q_i) − wall_half_thickness`. We drive `r_i → 0`. (Omitting the
   0.12 m term injects a systematic bias.)
5. **Solve a Gauss-Newton step.** Linearizing the residual w.r.t. a small body
   perturbation `δ = (dx, dy, dθ)` about the robot position `c`:
   `J_i = [ n_ix , n_iy , −n_ix·(p_iy − c_y) + n_iy·(p_ix − c_x) ]`,  `b_i = −r_i`.
   Solve `δ = (JᵀJ + λI)⁻¹ Jᵀb` (small Levenberg damping `λ` for conditioning), apply
   `pose += δ`, re-associate, iterate (≤ `max_iters`) until `‖δ‖` is below tolerance.
6. **Output** the corrected absolute map pose + an `info` diagnostic.

## 4. Components & interfaces

### New: `src/tugbot_maze/tugbot_maze/scan_match_localizer.py`

```python
class ScanMatchLocalizer:
    def __init__(self, segments, *,
                 scan_offset_x=SCAN_OFFSET_X,        # -0.1855 (footprint.py)
                 wall_half_thickness_m=0.12,
                 usable_range_m=8.0,                 # ignore far beams
                 beam_stride=1,                      # downsample (1 = all 900)
                 max_corr_dist_m=0.30,               # outlier reject after iter 0
                 max_iters=6, tol_m=0.002,
                 trans_clamp_m=0.5, yaw_clamp_rad=math.radians(10),
                 min_inliers=20, min_eig=5.0, min_yaw_info=3.0):
        # min_eig: translational-observability floor (eigenvalue of the 2x2 xy
        #   information block; ~= effective # of beams constraining that axis).
        # min_yaw_info: yaw-observability floor (JtJ[2,2]). Both calibrated in tests.
        # precompute segment endpoint A (S,2), edge e (S,2), length², unit normal (S,2)

    def correct(self, prior_pose, ranges, angle_min, angle_inc) -> tuple[
            tuple[float, float, float], dict]:
        """Return (corrected_pose, info).
        info keys: n_inliers, residual_rms, eig_min, eig_max,
                   fell_back: set[str] in {'x','y','yaw'}, rejected: bool, iters."""
```

- Self-contained: builds its own NumPy segment arrays from `segments`
  (`maze_sim.load_segments()` output); does **not** import `MazeSim`.
- Pure function of inputs (no ROS), so it is unit-testable offline.

### Modify: `src/tugbot_maze/tugbot_maze/flood_fill_solver.py`

- Extend `pose_source` to accept `scan_match` (keep `slam` and `odom_locked` unchanged for
  A/B comparison). On first valid scan, seed `last_corrected_pose` from the startup pose
  (entrance, known) and `last_odom_pose` from `odom→base_link`.
- In `_lookup_pose()` for `scan_match`: read current `odom→base_link`, compose
  `prior = last_corrected_pose ⊕ (odom_now ⊖ last_odom)`, call
  `ScanMatchLocalizer.correct(prior, *latest_scan)`, store and return the corrected pose;
  update `last_odom_pose`.
- Construct the localizer once at startup from `load_segments()`.
- Extend the `DIAG` line with `match=(rms,n_inl,eigmin)` and `fb=<axes>` so the diagnostic
  classifier can see localizer health.

### Reuse (unchanged)
- `maze_sim.load_segments()` — the known wall map (map frame).
- `footprint.SCAN_OFFSET_X` — LIDAR mount offset.
- `maze_motion.step(pose, scan, t)` — FSM consumes the corrected pose; **not modified**.

## 5. Data flow

`/scan` (900 beams, `scan_omni`) cached by node → each tick:
`prior = last_corrected ⊕ odom_delta` → `ScanMatchLocalizer.correct()` →
absolute map pose → `maze_motion.step(pose, scan, t)`. DIAG logs match health.

## 6. Robustness & fallbacks (directly handles the open-region wall)

- **Observability gate (the principled open-region answer).** Eigen-decompose the 2×2
  translational block of `JᵀJ`. If the smallest eigenvalue `< min_eig`, the correction
  component **along that weak eigenvector** is suppressed and the **odom-propagated prior is
  retained for that direction** (the dominant axis label, `x` or `y`, is added to
  `info.fell_back`). Yaw is gated separately by its own information term: if
  `JᵀJ[2,2] < min_yaw_info`, the prior yaw is retained (`'yaw'` added to `fell_back`) — kept
  distinct because yaw and translation carry different units and should not share an
  eigenvalue threshold. In a junction or long corridor with walls on one axis only, we
  correct the constrained axis and let odom carry the other until more walls come into view
  — instead of guessing. This is strictly more information than the cardinal window (ICP
  uses *all* visible walls, including far corridor walls within `usable_range`).
- **Outlier rejection.** After the first iteration, drop correspondences with
  `|n·(p−q) − half_thickness| > max_corr_dist_m` (dynamic/unmodeled returns).
- **Min inliers.** If inliers `< min_inliers`, skip correction this tick → use prior, set
  `info.rejected`.
- **Aliasing/glitch clamp.** If the total correction exceeds `trans_clamp_m` /
  `yaw_clamp_rad` vs the prior, reject (use prior) and flag. Persistent rejection = a real
  "localization lost" signal worth logging (not silent).

## 7. Configuration / defaults (all tunable)

`usable_range_m=8.0`, `beam_stride=1`, `max_corr_dist_m=0.30`, `max_iters=6`,
`tol_m=0.002`, `trans_clamp_m=0.5`, `yaw_clamp_rad=10°`, `min_inliers=20`,
`wall_half_thickness_m=0.12`, Levenberg `λ` small (e.g. 1e-6 scaled), starting
`min_eig=5.0` and `min_yaw_info=3.0` (both calibrated in tests so a single-wall axis is
gated). Defaults live as constructor kwargs.

## 8. Testing strategy (TDD; the trusted geometry, used both ways)

`MazeSim` is the forward model: place it at a **known true pose**, `scan()` yields the
synthetic 360° ranges; the localizer must **invert** them. (`scan()` already subtracts
`wall_half_thickness`, so it exercises the face-vs-centerline term.)

`test/test_scan_match_localizer.py`:
1. **Recovery under drift.** At many true poses across the maze (corridors + cells),
   inject a prior error (e.g. +0.4 m, −0.3 m, +8°); assert corrected pose within
   ~5 cm / ~2° of truth.
2. **No-thickness-bias.** Zero prior error; assert corrected pose ≈ truth to < 2 cm
   (guards the 0.12 m term).
3. **Open-region gating.** A pose in a one-wall junction; assert the unconstrained axis is
   in `info.fell_back` and the pose does **not** diverge (stays near prior on that axis).
4. **Outlier robustness.** Inject spurious short returns; assert recovery still holds.
5. **Aliasing clamp.** Prior error beyond `trans_clamp_m`; assert `info.rejected` and pose
   == prior (no wild jump).
6. **End-to-end under drift.** Wire the localizer into the existing `test_maze_motion_sim`
   loop (`MazeSim` with `odom_drift_per_m>0`), feeding the localizer the drifted odom as
   prior; assert a clean solve where the `odom_locked` baseline desyncs.

All offline, deterministic, ROS-free.

## 9. Acceptance criteria

- All new unit tests pass; the existing ROS-free suite stays green (3 `[0.05-*]` xfails
  remain xfailed; no new failures).
- Offline end-to-end: solves under `odom_drift_per_m` where `odom_locked` fails.
- **Gazebo controlled batch** (`tools/batch_diagnose_floodfill.sh N MAX false true
  scan_match true`) vs the `d356f4a` baseline: DESYNC primary-mode share drops sharply,
  heavy-desync prevalence falls, best-depth improves; stretch goal = first `EXIT_REACHED`.

## 10. Out of scope / non-goals

- Routing/exploration logic (flood-fill brain) — unchanged.
- Global relocalization / kidnapped-robot recovery (we always have an odom prior).
- AMCL / Nav2 / map_server (Approach B — rejected: heavier, corridor-aliasing risk,
  re-introduces the stack the project left).
- Changing the default `pose_source` — `scan_match` is opt-in until proven in Gazebo.

## 11. Risks

- **Compute per tick.** 900 beams × few iters at 10 Hz; mitigated by NumPy vectorization,
  `usable_range_m` pruning, and `beam_stride` if needed.
- **Real-LIDAR vs centerline-map mismatch** beyond half-thickness (e.g. wall posts,
  reflections) — handled by outlier rejection; monitored via `residual_rms` in DIAG.
- **Open junctions** with too few walls — handled by the observability gate (odom carries
  the unconstrained DOF); if a junction is persistently unobservable, DIAG will show it.
