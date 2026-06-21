# Odom-Anchored Centerline Through Open Junctions — Design

**Date:** 2026-06-20
**Workspace:** `ros2_ws_tugbot_nav_20260614`
**Branch:** `junction-centerline-precision` (off `main`; local only, **never pushed**)
**Status:** Approved (brainstorm + approach + methodology). Ready for adversarial review, then plan.

## Problem

The decisive back-out + junction-log milestone (merged `da27f33`) drove a Gazebo run to within
**9.78 m** of the exit, but `EXIT_REACHED` is still gated by physical wedges in the tight interior.
The run's evidence localizes it precisely: **17 of 24 wedge-recovers occurred at the single 4-way
junction (6,4)**, clustered at pose ≈ (11.6, 8.9) — the NW corner, between its W doorway (entry from
(5,4)) and N doorway (exit toward (6,5)).

Two diagnostics reframe the cause:

1. **The slack is fine — this is not a tight-squeeze problem.** Robot collision radius 0.35 m + wall
   half-thickness 0.12 m = **0.47 m** keep-out. In the 1.76 m corridor that leaves **0.53 m of
   lateral slack per side**; doorways are *wider* (2.0–2.07 m). The robot fits comfortably.
2. **It goes blind laterally at junctions.** In `_drive`, the corridor follower centers on the two
   side walls (drift-immune). At a junction — especially a 4-way — **both side walls vanish at
   once**, so `centerline_cross` uses its `fallback_cross`, which comes from
   `cross_track_offset(ox, oy, hop_dir)` = **0.0** when there is no side wall (it is wall-derived).
   The robot then has **heading-only control for ~0.5–1 m**, with **no mid-transition re-centering**
   (the full ~2 m hop is dead-reckoned from `hop_start`). Any drift — from a `turn_timeout` residual,
   diff-drive bias, or an off-center entry — goes uncorrected and the robot clips a corner wall-stub.

So the fix is to **restore a lateral reference through the open zone**, not to make the robot smaller.

## Goal

Give the corridor follower a valid centerline reference in the open-junction zone by replacing the
blind `0.0` fallback with the robot's lateral offset from the **grid corridor centerline**, computed
from the (drift-bounded, per-cell re-anchored) **odom pose**. This keeps the robot centered through
junctions and eliminates the off-center-entry → corner-clip wedge. Minimal, principled, reuses the
proven follower; develop and validate **offline-first** against a reproduced open-junction case.

## Scope (decided)

**Approach A — odom-anchored centerline.** **Files touched:**
- `tugbot_maze/hop_controller.py` — add one pure function `grid_cross_track(...)`.
- `tugbot_maze/maze_motion.py` — in `_drive`, source the open-junction `fallback` from
  `grid_cross_track` (odom/grid) instead of `cross_track_offset` (wall-derived), clamped by a new
  `grid_fallback_max_m = 0.40` tunable. Adjust the import.
- `test/test_hop_controller.py` — unit-test `grid_cross_track`.
- `test/test_maze_motion_sim.py` — add an offline open-junction convergence reproduction; keep the
  full-solve regression green.

**Do NOT touch:** `flood_fill_brain.py`, `cell_walls.py`, `wall_localize.py`, `junction_log.py`, the
ROS node, and within `maze_motion.py`: `_center`, sensing/commit, the brain routing, `_unstick`, the
back-out phase, the wedge detector, `_recover`, the turn law, and the arrival test. The turn-residual
contributor is **not** addressed directly — the odom centerline corrects drift regardless of source,
so the turn law is left untouched (keeps scope minimal and risk low).

## Design

### New pure helper (`hop_controller.py`)

`cross_track_offset` returns 0.0 when the perpendicular axis has no wall; `grid_cross_track` is the
always-valid odom/grid analogue with the **same sign convention** (`cross_left > 0` ⟺ robot LEFT of
centre):

```python
def grid_cross_track(pose_x: float, pose_y: float, cell, hop_dir, *,
                     cell_size_m: float = 2.0) -> float:
    """Signed lateral offset of the robot to the LEFT of the hop direction (+ = robot left of
    centre), measured from the GRID corridor centerline (the cell-centre line) using the odom pose.
    Unlike cross_track_offset (wall-derived, 0 when no side wall is visible), this is ALWAYS valid,
    so it gives the corridor follower a centerline reference through an open junction where both side
    walls vanish. Sign convention matches cross_track_offset. `cell` is the hop's SOURCE cell; it
    shares the perpendicular coordinate with the target (the hop is along a cardinal), so it defines
    the corridor centerline for the whole hop. Odom drift is bounded (~0.25 m, re-anchored per cell)
    and the caller clamps the result, so a small drift error can only nudge, never swing."""
    dx, dy = hop_dir
    if dy != 0:                                       # N/S travel -> lateral is the x axis
        off = pose_x - cell_size_m * cell[0]          # +E
        return -off if dy > 0 else off                # N: east=right=>-; S: east=left=>+
    off = pose_y - cell_size_m * cell[1]              # +N
    return off if dx > 0 else -off                    # E: north=left=>+; W: north=right=>-
```

`hop_controller.py` stays dependency-free (no `flood_fill_brain` import) — `cell_size_m` is a
parameter (default 2.0), matching the module's existing geometry-default style (`half_corridor_m`,
etc.). The caller passes `cell_size_m=CELL_SIZE_M`.

### Change in `_drive` (`maze_motion.py`)

Today (around `maze_motion.py:379-380`):
```python
ox, oy = cell_center_offset(ranges, amin, ainc, yaw)
fallback = cross_track_offset(ox, oy, self.hop_dir)        # open-junction fallback
```
becomes:
```python
fallback = max(-self.grid_fallback_max_m,
               min(self.grid_fallback_max_m,
                   grid_cross_track(x, y, self.cell, self.hop_dir, cell_size_m=CELL_SIZE_M)))  # odom grid centerline (valid in open junctions), clamped to the smooth creep-and-steer regime
```
The `cell_center_offset(...)` / `ox, oy` here were used **only** to feed `cross_track_offset`, so both
lines collapse into the single `grid_cross_track` call. New `MazeMotion.__init__` tunable:
`grid_fallback_max_m = 0.40` (m). (`cell_center_offset` is still imported and
used by `_center`; `cross_track_offset` becomes unused in `maze_motion` — remove it from the
`hop_controller` import and add `grid_cross_track`.) `x, y` come from `x, y, yaw = pose` at the top of
`_drive`; `self.cell`, `self.hop_dir`, and `CELL_SIZE_M` are already in scope.

### Why this is correct and safe

- **Additive to the open zone only.** `centerline_cross` uses `fallback_cross` **only when neither
  side wall is seen**. When ≥1 wall is visible (nearly all of every straight corridor) the proven
  wall-balanced centerline is used and `fallback` is ignored. So this changes behavior **only** in
  the fully-open junction zone — exactly where the robot was blind.
- **Turn-first, self-recovering, clamped to the smooth regime.** The follower bakes the cross-track
  into the heading setpoint (`corridor_drive_command`: `setpoint = cardinal + atan2(-cross, lookahead)`),
  so a large cross steers toward centre and *throttles forward speed via the heading error* (turn
  first) — **not** via a separate cross-velocity cap (that "only-nudge, never-zero-v" property belongs
  to `hop_drive_command`'s `cross_w_max`, a different law). The smooth, non-stalling window is
  `cross < ~0.48 m` (`= lookahead·tan(slow_angle) = 0.7·tan(0.6)`); beyond it `v` briefly drops to the
  creep floor while the robot turns toward centre, then re-accelerates. To keep the odom fallback
  inside this regime **and above the wedge-detector creep rate** (so it can never self-trigger a false
  wedge: `v·wedge_detect_s` must stay > `wedge_move_eps`), the fallback is clamped to
  `grid_fallback_max_m = 0.40 m` — below the ~0.44 m point where the heading-throttled `v` would fall
  under that rate. Within the clamp, the bounded odom estimate can only nudge, never swing off-cardinal.
- **Targets the odom grid centerline.** `grid_cross_track` measures the offset to the *odom* cell-centre
  line, so its convergence target is the odom centerline — off the true physical centerline by the
  bounded drift residual (≤ ~0.25 m « the 0.53 m slack; exact at zero drift, strictly better than the
  old heading-only `0.0` fallback at any drift). It is load-bearing on the solver's existing assumption
  that the perpendicular dcell stays synced (drift < one cell, which the offline regime enforces and
  `test_..._sim` guards via `max_desync ≤ 1`); during a hop `self.cell` is not re-anchored, so the
  centerline is stable for the whole hop.
- **Source-agnostic drift correction.** The robot now actively steers back to the grid centerline in
  the open zone, correcting drift from *any* cause (turn residual, diff-drive bias, off-center
  entry) — which is why the turn law needs no change.
- **Pure + testable.** `grid_cross_track` is a 4-line pure function; the integration is a one-line
  fallback swap.

## Testing & validation (offline-first)

`maze_sim` geometry is confirmed identical to the real Gazebo SDF (0.24 m walls, 0.47 m keep-out), so
the open-zone fallback can be exercised under **real collision physics** offline (the discriminating
centering test below); the literal (6,4)-junction corner-clip is validated in the Gazebo run.

**Unit — `test/test_hop_controller.py` (`grid_cross_track`):**
- All four `hop_dir`s, robot 0.4 m off the grid centerline of cell (5,5) (centre (10,10)):
  - N `(0,1)`: pose x=9.6 → `+0.4` (west = left); x=10.4 → `−0.4` (east = right).
  - S `(0,-1)`: pose x=10.4 → `+0.4` (east = left).
  - E `(1,0)`: pose y=10.4 → `+0.4` (north = left).
  - W `(-1,0)`: pose y=10.4 → `−0.4` (north = right).
- On the centerline → `0.0`.

**Offline reproduction — `test/test_maze_motion_sim.py` (new `test_grid_centerline_holds_through_open_junction`):**
- Side walls placed **beyond sensing range but within collision range** — at `x = ±2.0` (face
  distance ≈ 1.5–1.9 m after the 0.12 m half-thickness, all `> wall_seen_m = 1.3` ⇒ both *unseen* ⇒
  the fallback path runs, yet **close enough that a bad fallback actually collides**, so the
  no-collision assertion has teeth. An empty `[]` world can never collide — its no-collision
  assertion would be vacuous — so use real walls.) `MazeSim(walls, (0.4, 0.0), π/2, inertia=True)`
  (real accel-limited diff-drive physics; robot 0.4 m right of the grid centerline `x = 2·0 = 0`,
  facing N). Each tick: `d_left, d_right` from the scan (both unseen), `fallback = max(-0.40, min(0.40,
  grid_cross_track(sim.x, sim.y, (0,0), (0,1))))`, then `corridor_follow_command(sim.yaw, π/2, d_left,
  d_right, None, fallback_cross=fallback)`, `sim.step(v, w, 0.1)`. Over ~120 ticks assert
  `abs(sim.x) < 0.15` (converged) **and** `not collided`.
  **Without** the fix (`fallback = 0.0`) the robot holds its 0.4 m offset → the convergence assertion
  fails; **with** it, the odom-grid fallback re-centers it. (Mirrors the existing
  `test_symmetric_following_converges_to_centerline`, but in the unseen-wall zone where only the odom
  fallback can centre.)
- **Large-offset characterization** (`test_grid_centerline_recovers_from_large_offset`): same walls,
  start `(0.55, 0.0)` (beyond the 0.40 clamp). Assert `not collided` and eventual convergence
  (`abs(sim.x) < 0.2` by the end) — verifies the clamp keeps the robot in the creep-and-steer regime
  (it must re-centre without stalling/wedging or hitting the `x=±2.0` wall). If this case stalls or
  collides, tighten `grid_fallback_max_m` (offline tuning before any Gazebo run).

**Offline regression — must stay green:** the full-solve `test_reaches_exit_without_collision_or_desync`
(5 drift×latency cases) still reaches the exit collision-free with `max_desync ≤ 1`, and
`test_backout_is_exercised_end_to_end` still holds (back-out unaffected). The fallback change only
helps the open zone, so the solve must not regress.

**Gazebo (one confirmation run):**
`tools/run_flood_fill_maze.sh 1800 false true odom_locked true`. Clean stray sims first via the `!`
pkill. Success signal: the (6,4)-class wedge-recovers drop sharply (ideally near zero) and the robot
makes deeper exit-ward progress than the prior best (dist 9.78 m); `EXIT_REACHED` is the stretch goal.

**Success criteria:**
- All new unit tests pass; offline regression stays green.
- The open-junction convergence test passes (robot re-centers via the odom-grid fallback).
- In Gazebo: junction wedge-recovers sharply reduced vs. the prior run.

## Revisions from the adversarial review (2026-06-20)

3 lenses with per-finding verification confirmed the core design is sound (sign chain verified as
correct negative feedback; mechanism discriminating). Folded-in refinements:

1. **Weak offline test** (should). The original `MazeSim([])` empty-wall test can never collide
   (vacuous no-collision assertion) and doesn't exercise wall physics; the full-solve regression
   passes *without* the fix (non-discriminating). **Fixed:** real side walls at `x = ±2.0` (unseen but
   collidable) + `inertia=True`, plus a 0.55 m large-offset case; softened the offline "reproduces the
   wedge" prose (the literal (6,4) wedge is validated in Gazebo).
2. **Inaccurate safety rationale** (should). The "clamp → only-nudge, never-zero-v" property is
   `hop_drive_command`'s, not `corridor_drive_command`'s (used in `_drive`), where `cross ≳ 0.48 m`
   throttles `v` to a brief pirouette. **Fixed:** corrected the rationale and added a
   `grid_fallback_max_m = 0.40 m` clamp keeping the odom fallback in the smooth, no-false-wedge regime.
3. **Odom-centerline residual** (nit). **Clarified:** the convergence target is the *odom* centerline
   (residual ~ bounded drift, load-bearing on the solver's existing < 1-cell dcell-sync assumption).

## Constraints

- Local only — **never push** to origin.
- Commit messages end with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
- Agent `pkill`/`kill` is hook-blocked; stray Gazebo sims are cleared by the user via the `!` prefix:
  `! pkill -9 -f "gz sim|ruby.*gz|parameter_bridge|flood_fill_solver|ros2 launch"`.
- Foreground `sleep` is blocked; use background runs.
- Shell cwd is not stable between Bash calls — use absolute paths / `git -C`.
- Avoid backticks inside `git commit -m` strings (bash command-substitution can drop a word).
