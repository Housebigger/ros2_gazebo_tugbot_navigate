# Tighter Corridor Controller — Design

**Date:** 2026-06-19
**Workspace:** `ros2_ws_tugbot_nav_20260614`
**Branch:** `tighter-corridor-controller` (off `main`; local only, not pushed)
**Status:** Approved (brainstorm), ready for implementation plan.

## Problem

The flood-fill micromouse solver (`MazeMotion`) achieved the project's first near-complete
autonomous interior solve in Gazebo (28 cells, within 5.25 m of the exit), but a *consistent*
confirmed `EXIT_REACHED` is gated on **physical wedges** in the tight interior (0.35 m robot,
~1.76 m corridors, ~0.53 m clearance per side). When the robot enters a narrow section or
doorway off-center it physically pins; that failure then cascades — last-resort false wall-marks
corrupt the known map, the flood re-routes, and the robot takes a different (SW) path run-to-run.
Everything above the low-level controller is verified sound (topology, grid transform, spawn,
projection-median sensing, flood-fill brain, dcell re-anchoring, un-wedge recovery); the offline
end-to-end sim solves fully. The gap is purely **sim-vs-Gazebo low-level motion precision**.

Two concrete mechanisms produce the off-center entries:
1. **Straights lose their lateral reference at openings.** `_drive` steers on `cross_track`
   derived from `cell_center_offset` (cardinal wall min-ranges). At a junction/doorway where a
   side wall disappears, the cross-track goes to 0 → no centering → the robot drifts during the
   ~2 m hop and enters the next narrow section off-center.
2. **Rotate-in-place overshoots under command latency** → the next straight starts off-cardinal
   → it curves into a wall.

## Goal

Replace the low-level `drive` and `turn` laws with a **continuous symmetric wall-following**
drive (keep the robot on the true corridor centerline using both side walls — drift-immune, no
odom needed) and a **motion-profiled turn** (no latency overshoot), so the robot never leaves the
centerline envelope and never wedges. Preserve everything proven.

## Scope (decided)

**Targeted replacement of `drive` + `turn` only.**

- **Modify `tugbot_maze/hop_controller.py`** — add three pure functions:
  - `side_distances(perp, hop_dir) -> (d_left, d_right)` — map the projection-median per-cardinal
    perpendicular-distance dict to the left/right side-wall distances for the travel direction.
  - `corridor_follow_command(...)` — symmetric wall-following straight drive (replaces
    `corridor_drive_command` in the drive phase).
  - `profiled_turn_command(...)` — decel-limited rotate-in-place (replaces the PD turn law).
  - **Keep** `corridor_drive_command`, `centering_command`, `cross_track_offset`, `hop_command`,
    `hop_drive_command` (not deleted — `corridor_drive_command` stays as fallback/comparison;
    `cross_track_offset` is reused as the open-junction fallback; `centering_command` still drives
    the unchanged `_center` phase).
- **Modify `tugbot_maze/maze_motion.py`** — `_drive` calls `corridor_follow_command`; `_turn`
  calls `profiled_turn_command`. New tunables added to `MazeMotion.__init__`.
- **Do NOT touch:** `flood_fill_brain.py`, `cell_walls.py`, `wall_localize.py`, the ROS node
  adapter (`flood_fill_solver.py`), and within `maze_motion.py`: `_center`, the per-cell sense
  step, brain routing, dcell re-anchoring, the wedge detector, `_recover`, the arrival test, and
  the front-block-as-transient logic.

## Component 1 — Symmetric wall-following drive

Source of side distances: `cell_wall_perp_dist(ranges, amin, ainc, yaw)` (existing) returns a dict
of perpendicular distances to the wall in each of the 4 **map cardinal** directions
(`'E','W','N','S'`), using projection + median (robust to oblique grazes and ~0.45 m off-center).

`side_distances(perp, hop_dir)` maps the travel cardinal to the robot's left/right side walls
(robot-frame left = +90° from heading):

| hop_dir | facing | left wall | right wall |
|---------|--------|-----------|------------|
| (0,+1)  | N      | W         | E          |
| (0,−1)  | S      | E         | W          |
| (+1,0)  | E      | N         | S          |
| (−1,0)  | W      | S         | N          |

Returns `(d_left, d_right) = (perp[left], perp[right])`.

**Lateral error to centerline** (sign convention: `cross_left > 0` ⟺ robot is LEFT of centre).
A side distance counts as a *seen wall* when `< wall_seen_m` (≈ 1.3 m, matching the sensing
threshold); otherwise that side is *open*.

| sides seen | `cross_left` | rationale |
|------------|--------------|-----------|
| both       | `(d_right − d_left) / 2` | true centerline; drift-immune, no odom (THE key change) |
| right only | `d_right − half_corridor_m` | hold half-corridor from the one wall (d_right large ⟹ left of centre ⟹ +) |
| left only  | `half_corridor_m − d_left` | (d_left small ⟹ near left wall ⟹ left of centre ⟹ +) |
| neither    | fallback: `cross_track_offset(ox, oy, hop_dir)` if available, else `0.0` | open junction (~1 cell); brief grid/odom fallback or hold heading |

`half_corridor_m` ≈ 0.88 m (perpendicular distance from a centered robot to a side-wall face:
2.0 m cell pitch ⟹ ~1.0 m centre-to-boundary, minus ~0.12 m wall half-thickness).

**Steering & speed** (reuse the proven pure-pursuit + wedge-floor form of
`corridor_drive_command`):
```
setpoint = cardinal_yaw + atan2(-cross_left, lookahead_m)
err      = norm(setpoint - yaw)
w        = clamp(kp_ang * err, ±w_max)
throttle = max(0, 1 - |err| / slow_angle)              # heading: -> 0 when misaligned (turn first)
if near_wall_m given:                                  # front/side proximity slow, NEVER 0
    wedge = clamp((near_wall_m - wedge_stop_m) / (wedge_slow_m - wedge_stop_m), 0, 1)
    throttle *= max(wedge_v_floor / v_max, wedge)
v        = clamp(v_max * throttle, 0, v_max)
```
`near_wall_m` is the min side-wall distance (as `_drive` already computes), keeping the
never-zero creep-and-steer-away behavior. `cross_left` is clamped to `±max_cross_track_m` by the
caller (a bogus/huge estimate can only nudge, never swing the robot off-cardinal).

**Why this fixes mechanism 1:** as long as ≥1 side wall is visible (true in nearly all of every
corridor), the robot centers on the *physical* walls continuously — no dependence on the cell-grid
offset that vanished at openings. The old "side wall disappears → cross-track 0 → drift" hole is
closed; the only no-reference case is a fully-open intersection, which is short and falls back to
the grid/odom estimate.

## Component 2 — Motion-profiled turn

Rotate-in-place to `target_cardinal` on odom yaw, but with a **decel-limited** angular-velocity
profile instead of pure proportional control:
```
err = norm(target_cardinal - yaw)
w_mag = min(turn_w_max, sqrt(2 * ang_decel * |err|))   # ramps to 0 as |err| -> 0
w = sign(err) * w_mag  - kd_turn * yaw_rate            # small optional latency damping
w = clamp(w, ±turn_w_max)
```
As `|err| → 0` the commanded `w → 0`, so even with command-pipeline latency the robot is already
decelerating when the delayed command lands → **no overshoot** (the failure the PD law had). The
optional `kd_turn * yaw_rate` term is retained as light damping but the decel profile is the
primary mechanism.

Settle/handoff is unchanged from `_turn`: declared done after `turn_settle_ticks` consecutive
ticks within `yaw_tol_rad`, OR after `turn_timeout_s` (then proceed to drive — the corridor
follower holds heading).

## Component 3 — Integration into MazeMotion

- `_drive`: already computes the `perp` dict and `near`. Add
  `(d_left, d_right) = side_distances(perp, self.hop_dir)`, compute `cross_left` per the table
  (open-axis fallback uses the existing `cross_track_offset(ox, oy, self.hop_dir)`), clamp to
  `±max_cross_track_m`, and call `corridor_follow_command(yaw, target_cardinal, cross_left, near, …)`.
  **Unchanged:** arrival (`moved ≥ CELL − slack`), wedge detector → `_recover`, front-block /
  deadline → transient re-sense, attempt bookkeeping.
- `_turn`: call `profiled_turn_command(yaw, target_cardinal, yaw_rate, …)`; keep the settle-ticks +
  `turn_timeout` handoff exactly as now.
- `_center`, sensing, brain, re-anchor, `_recover`: **unchanged.**

**New `MazeMotion.__init__` tunables** (defaults): `ang_decel = 1.2` (rad/s²),
`wall_seen_m = 1.3` (m), `half_corridor_m = 0.88` (m).
- **Drive** (`corridor_follow_command`) reuses the responsive lateral gain — the existing
  `corridor_drive_command` default `kp_ang ≈ 1.5` — NOT the gentle `kp_turn`, plus `w_max`,
  `cruise_v` (as `v_max`), `lookahead_m`, `slow_angle`, `max_cross_track_m`, and the wedge-floor
  params (`wedge_slow_m`, `wedge_stop_m`, `wedge_v_floor`).
- **Turn** (`profiled_turn_command`) reuses `turn_w_max`, `kd_turn` (light damping), `yaw_tol_rad`,
  `turn_settle_ticks`, `turn_timeout_s`. It no longer uses `kp_turn` as a proportional gain (the
  decel profile replaces it); `kp_turn` is retained only for the unchanged `_center` phase.

## Testing & validation

**Unit (deterministic, fast) — `test/test_hop_controller.py`:**
- `side_distances`: all four `hop_dir`s map to the correct (left, right) cardinals.
- `corridor_follow_command`:
  - both walls equal → near-straight (`|w|` small, `v` near `v_max`).
  - robot off-center (one side closer) → steers toward the FARTHER wall (sign of `w` reduces
    `|cross_left|`).
  - single wall seen → converges to `half_corridor_m` from that wall.
  - neither wall seen → uses the supplied fallback `cross_left` (or holds heading at 0).
  - near front/side wall → `v` slows to the floor, never exactly 0.
- `profiled_turn_command`:
  - large `|err|` → `|w| ≈ turn_w_max`.
  - small `|err|` → `|w| ≈ sqrt(2·ang_decel·|err|)` (shrinks toward 0; no overshoot).
  - `|err| ≤ yaw_tol` handled by the caller's settle (function returns small/zero `w`).

**Offline regression (must stay green) — `test/test_maze_motion_sim.py`:**
- The end-to-end drive through `maze_sim` (`inertia=True` + collision + odom drift + cmd latency)
  must still reach `EXIT_CELL` collision-free with dcell synced, across the existing
  drift × latency regime. The new drive/turn are drop-in; any regression fails the build.
- Add one assertion: with both side walls present, symmetric following holds the robot within the
  centerline envelope (e.g. `|cross| < 0.4 m`) along a straight corridor (raycast left/right in
  `maze_sim`).

**Gazebo (one confirmation run):**
`tools/run_flood_fill_maze.sh 1800 false true odom_locked true` (GUI, 1800 s, odom_locked,
sense_debug). Clean stray sims first via the `!`-prefix pkill.

**Success criteria:**
- All new unit tests pass; offline regression stays green.
- **Minimal (stage success):** the physical-wedge class is eliminated — recover events from
  physical pins ≈ 0, the robot visibly holds the centerline (`< ~0.4 m` off), and it progresses
  past the historical ~14-cell plateau toward the exit.
- **Full:** confirmed `EXIT_REACHED`.
- **If wedges persist:** escalate to the deferred "SDF-fidelity offline wedge reproduction" (import
  the real SDF doorway geometry into `maze_sim`, reproduce an off-center pin, tune offline) before
  more Gazebo runs.

## Constraints

- Local only — **never push** to origin.
- Commit messages end with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
- Agent `pkill`/`kill` is hook-blocked; stray Gazebo sims are cleared by the user via the `!`
  prefix: `! pkill -9 -f "gz sim|ruby.*gz|parameter_bridge|flood_fill_solver|ros2 launch"`.
- Foreground `sleep` is blocked; use background runs.
- Shell cwd is not stable between Bash calls — use absolute paths / `git -C`.
