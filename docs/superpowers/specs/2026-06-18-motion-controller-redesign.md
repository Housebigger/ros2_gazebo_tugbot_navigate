# Motion-Controller Redesign (flood-fill maze solver) — Design

**Date:** 2026-06-18
**Branch:** `flood-fill-maze-solver` (workspace `ros2_ws_tugbot_nav_20260614`)
**Status:** design approved (Approach A + pure-class extraction), pending spec review.

## Goal

Replace the fragile cell-to-cell motion layer of the flood-fill maze solver with a robust,
**offline-testable** one, so the robot reliably drives the full ~100-cell maze to the exit
on wheel odometry + LIDAR, staying inside. The brain, sensing, localization, and discrete
cell tracking are already validated and are **kept unchanged**; only the motion layer changes.

## Background: why this redesign

Five Gazebo runs (commits `98a13b9`..`16bd8d1`) fixed sensing (projection-median) and made
the robot clear the original traps and execute direction changes — but it **plateaus at ~7
cells** because the diff-drive *motion control* is imprecise in the 1.76 m corridors:

1. **Lateral drift into walls** — during a hop the robot drifts ~1 m sideways and wedges
   against a corridor wall (`moved` stalls at 0).
2. **Turn stalls** — a 90° turn doesn't settle within tolerance (oscillates), so `moved≈0`.
3. **Cumulative `dcell` desync** — dead-reckoned hops that travel ≠ one cell make the
   discrete cell index drift from the physical cell (~1 cell after backtracking), after
   which the brain commands impossible hops.

These are low-level motion-control quality problems, not logic bugs. They were only
observable in Gazebo (~10 min/iteration) because the motion logic lives in the ROS node.

## Reused unchanged (validated)

- `flood_fill_brain.py` — `FloodFillBrain` (flood/next_cell, Trémaux cap).
- `cell_walls.py` — `cell_wall_perp_dist` / `sense_cell_walls` (projection-median, lateral-
  offset-robust; 0 false walls to 0.45 m off-center).
- `wall_localize.py` — `cell_center_offset` (map-frame offset from cell center; `None` per
  open axis).
- `hop_controller.py` — `cross_track_offset` (signed lateral offset, left-positive).
- `odom_locked` pose source (frozen `map→odom` ∘ live `odom→base_link`).
- Discrete `self.cell` advanced by committed hops (drift-immune).
- The maze grid (2 m cells, entrance (1,0), exit (10,9)).

## Architecture: extract a pure `MazeMotion` controller

Today the phase FSM + drive logic lives in `flood_fill_solver._control_tick`, so it is only
testable in Gazebo. Extract it into a **ROS-free `MazeMotion` class** (new `maze_motion.py`):

```python
class MazeMotion:
    """ROS-free maze motion controller: phase FSM + drive/turn controllers + flood-fill brain.
    Drives cell-to-cell to the exit using wall-referenced sensing (cell_walls) and
    odom-relative motion. Deterministic; no ROS / time / I/O beyond the `t` passed in."""

    def __init__(self, brain=None, *, cruise_v=0.3, center_tol_m=0.10, yaw_tol_rad=0.10,
                 turn_settle_ticks=3, hop_arrive_slack_m=0.05, front_block_m=0.7,
                 lookahead_m=0.7, max_cross_track_m=0.6, wedge_slow_m=0.50,
                 wedge_stop_m=0.40, hop_timeout_s=25.0, settle_s=0.4):
        ...
        self.cell = ENTRANCE_CELL
        self.phase = 'center'                 # center | turn | drive | done | stuck
        self.sensed = set()

    def step(self, pose, scan, t):
        """pose=(x,y,yaw) map-frame; scan=(ranges, angle_min, angle_inc); t=seconds (monotonic).
        Advances the FSM (senses/plans/marks via the brain as needed) and returns (v, w, done)."""
```

`flood_fill_solver.py` becomes a **thin ROS adapter**: it keeps startup + the 2 m entry drive
(TF / clock / params are ROS-specific), constructs a `MazeMotion`, and each tick does
`v, w, done = motion.step(pose, scan, t)` then publishes `(v, w)` (and `EXIT_REACHED` on
`done`). No control logic in the node.

## Phase FSM (inside `MazeMotion.step`)

```
center → turn → drive → (arrive) → center → … → done
```

- **`center`** — re-center + sense + plan. Use `cell_center_offset` → `(ox, oy)`; if an axis
  has a wall reference and `|offset| > center_tol_m`, null it with **cardinal-aligned 1-axis
  centering** (`centering_command`, already built/tested: face the axis, drive to null — no
  diagonal chase). This is the **along-track re-anchor**: at any cell with a front/back wall,
  the along-track position is pulled back to the true cell center, so dead-reckoning error
  can't accumulate across a corridor. When centered + settled (`settle_s`): sense the cell
  once (sticky), `nxt = brain.next_cell(self.cell)`; if `None` → `stuck` (stop); else set
  `hop_dir`, `target_cardinal = atan2(hop_dir)`, `hop_start = pose_xy`, → `turn`.
- **`turn`** — rotate in place to `target_cardinal` using **odom yaw** (drift-free over the
  turn): `w = clamp(kp·(target_cardinal − yaw))`, `v = 0`. Declared done only when
  `|err| < yaw_tol_rad` for `turn_settle_ticks` *consecutive* ticks (the **settle check**
  that prevents declaring done mid-overshoot). If `hop_dir` already equals the current
  heading, this completes immediately. → `drive`.
- **`drive`** — corridor-centerline drive toward `target_cardinal` (see controller below)
  until `moved ≥ CELL_SIZE − hop_arrive_slack_m` (**arrive**: advance `self.cell`, clear stall
  streak, → `center`) or a real front wall (`cell_wall_perp_dist[dir] < front_block_m` with
  `moved > 0.3`): **mark that edge a wall**, → `center` (re-plan). On `hop_timeout_s` with an
  *open* front (transient stall, not a wall): re-plan without marking, bounded by a
  per-(cell,dir) stall counter that marks only after `max_open_timeouts` (live-lock guard).

## Controllers

### 1. Corridor-centerline drive (pure-pursuit lateral + anti-wedge) — `hop_controller.corridor_drive_command`

Fixes the lateral drift/wedging. Instead of "hold yaw," aim at a point on the corridor
centerline a `lookahead` ahead and drive to it:

```python
def corridor_drive_command(yaw, cardinal_yaw, cross_track, near_wall_m=None, *,
                           v_max=0.3, w_max=0.5, kp_ang=1.5, lookahead_m=0.7,
                           slow_angle=0.6, wedge_slow_m=0.50, wedge_stop_m=0.40,
                           wedge_v_floor=0.10):
    """Drive forward along `cardinal_yaw` while converging onto the corridor centerline.
    cross_track = signed lateral offset (+ = robot LEFT of centre). Aims at the centerline
    point `lookahead_m` ahead, so the robot CURVES back toward centre as it approaches a
    side wall (inherently anti-wedge). near_wall_m (min perpendicular distance to the two
    side walls) additionally throttles forward speed toward wedge_v_floor as the nearer wall
    approaches wedge_stop_m -- a never-zero floor so it keeps creeping while steering away
    (no freeze, no wedge). Returns (v, w)."""
    setpoint = cardinal_yaw + math.atan2(-cross_track, lookahead_m)
    err = _norm(setpoint - yaw)
    w = clamp(kp_ang * err, -w_max, w_max)
    throttle = max(0.0, 1.0 - abs(err) / slow_angle)
    if near_wall_m is not None:
        wedge = clamp((near_wall_m - wedge_stop_m) / (wedge_slow_m - wedge_stop_m), 0.0, 1.0)
        throttle = max(wedge_v_floor / v_max, min(throttle, wedge))   # floor so it never freezes
    v = clamp(v_max * throttle, 0.0, v_max)
    return (v, w)
```

The FSM supplies `cross_track = cross_track_offset(ox, oy, hop_dir)` (ignored if
`|cross_track| > max_cross_track_m`, i.e. a bogus/desync estimate → hold the cardinal) and
`near_wall_m = min(perp[left], perp[right])` from `cell_wall_perp_dist` for the two cardinals
perpendicular to the hop direction.

### 2. Rotate-in-place turn (settled) — handled in the `turn` phase

`w = clamp(kp_turn·(target_cardinal − yaw))`, `v = 0`; done after `turn_settle_ticks`
consecutive ticks within `yaw_tol_rad`. `kp_turn` tuned low enough (against the inertia sim's
0.8 rad/s² angular accel) that overshoot stays within the settle band.

## The key enabler: end-to-end offline validation through `maze_sim`

New `test/test_maze_motion_sim.py` drives a `MazeMotion` instance **through `maze_sim` in
`inertia=True` mode** — which already models the diff-drive accel limits, the LIDAR raycaster,
**collisions**, and injectable **odom drift**:

```python
def _run(sim, motion, drift, dt=0.1, max_steps=20000):
    t = 0.0
    for _ in range(max_steps):
        ranges, amin, ainc = sim.scan(n_beams=360, fov_rad=2*math.pi)
        v, w, done = motion.step(sim.reported_pose, (ranges, amin, ainc), t)
        if done:
            return True
        sim.step(v, w, dt)            # unicycle + accel limits + collision rejection + drift
        t += dt
    return motion.cell == EXIT_CELL
```

Assertions (the strong validation that was missing — all offline, under drift):

- **Reaches the exit:** `motion.cell == EXIT_CELL` from the entrance, with `odom_drift_per_m`
  in `{0.0, 0.05, 0.10}`.
- **Never collides:** instrument the harness to flag any step where `sim.collides(sim.x, sim.y)`
  is true (the body entered a wall margin) → assert zero. This is the anti-wedge guarantee.
- **`dcell` stays synced:** at each `center`, assert `motion.cell == pose_to_cell(sim.x, sim.y)`
  (true pose) — the tracker never diverges from the physical cell by more than a tolerance.

The existing real-maze ground-truth sensing checks (`test_cell_walls.py`) stay green; the
relocalize-sim and hop_controller unit tests stay green; new unit tests cover
`corridor_drive_command` (curves toward centre; throttles near a wall; never returns v=0 from
the wedge term; envelope) and the turn settle logic.

## File structure

| File | Change |
|---|---|
| `tugbot_maze/maze_motion.py` | **New.** `MazeMotion` (phase FSM + brain + sensing calls). |
| `tugbot_maze/hop_controller.py` | **Add** `corridor_drive_command`. Keep `centering_command`, `cross_track_offset`, `hop_command` (`hop_drive_command` may be dropped if unused). |
| `tugbot_maze/flood_fill_solver.py` | **Refactor** to a thin adapter: startup + entry drive, then delegate each tick to `MazeMotion.step`; publish `(v, w)` and `EXIT_REACHED`. |
| `tugbot_maze/maze_sim.py` | Minor: ensure the harness has what it needs (inertia, collision, scan, drift — already present). |
| `test/test_maze_motion_sim.py` | **New.** End-to-end drive through the inertia+collision sim (reaches exit / no collision / dcell synced, under drift). |
| `test/test_hop_controller.py` | **Add** `corridor_drive_command` unit tests. |
| `test/test_flood_fill_solver_smoke.py` | Keep green (node still constructs/ticks). |

## Parameters (initial; tuned against the offline sim)

`cruise_v=0.3`, `lookahead_m=0.7`, `kp_ang=1.5`, `kp_turn≈1.2`, `yaw_tol_rad=0.10`,
`turn_settle_ticks=3`, `center_tol_m=0.10`, `hop_arrive_slack_m=0.05`, `front_block_m=0.7`,
`max_cross_track_m=0.6`, `wedge_slow_m=0.50`, `wedge_stop_m=0.40`, `wedge_v_floor=0.10`,
`hop_timeout_s=25`, `max_open_timeouts=3`. Final values are whatever passes the offline
end-to-end test under drift; expect 1–2 Gazebo confirmation runs after.

## Risks

- **Gain tuning** — mitigated by tuning against the inertia+collision sim before Gazebo.
- **Sim-vs-Gazebo fidelity gap** — the offline sim is an approximation; passing it is necessary
  but not sufficient. Keep the real-maze sensing checks; budget 1–2 Gazebo runs to confirm.
- **Refactor regressions** — the node refactor is covered by the smoke test plus the
  end-to-end sim test exercising the same `MazeMotion` the node uses.

## Out of scope

Pure reactive wall-following (Approach B), changes to the brain/sensing/localization, and
merging the branch (stays unmerged on `flood-fill-maze-solver`).
