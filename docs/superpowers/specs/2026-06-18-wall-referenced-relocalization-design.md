# Wall-Referenced Re-Localization — Design

**Date:** 2026-06-18
**Component:** `ros2_ws_tugbot_nav_20260614` flood-fill maze solver
**Status:** Design (approved verbally; spec under review)

## Problem & confirmed root cause

The flood-fill (micromouse) solver reaches the maze interior but gets trapped and
times out. After eliminating SLAM (solved with `pose_source=odom_locked`), maze
topology, the grid transform, spawn alignment, the sensing heuristic
(`sense_cell_walls` scores 400/400 at true cell centers), and robot-body lidar
occlusion (Phase 0: spawn scan min 1.48 m, no sub-meter returns), the root cause was
**confirmed by ground-truth comparison (2026-06-18)**:

> **Odom drifts.** Reading Gazebo's true pose off gz transport
> (`gz topic -e -t /world/<world>/dynamic_pose/info`, the `name:"tugbot"` pose) vs the
> solver's odom at cell (1,1): GT solver-pose **(1.50, 2.10)** while odom reported
> **(2.06–2.41, ~1.95)** — odom is **~0.7 m east of truth**. Spawn was aligned
> (offset ≈ 0), so this is **accumulated drift**, almost certainly wheel slip while
> wedging against walls.

This drift makes the robot sense from ~0.5 m off the assumed cell center — beyond the
heuristic's ±0.30 m tolerance — so a true cell-boundary wall sits ~1.5 m from the
drifted pose and reads past the 1.3 m threshold as **OPEN** (e.g. (1,1)-E). Wrong wall
map → wrong navigation → wedging → more slip → more drift: a vicious cycle.

## Goal

Make the solver **drift-immune**: it senses from the true cell center and keeps the
cell index correct regardless of how far odom has drifted. Success = autonomously
reach the exit cell staying inside the maze, in Gazebo.

## Core approach (micromouse-standard): stop trusting odom for absolute position

Odom is reliable only for *relative* motion over short distances. So:

1. **Discrete cell tracking.** Track `current_cell` as an integer `(cx, cy)` advanced
   by the *hop the brain commits* (e.g. "go E → (2,1)"), **not** by
   `pose_to_cell(odom)`. A discrete counter cannot be corrupted by drift.

2. **Wall-referenced re-centering each cell.** Before sensing, use the lidar to drive
   the robot to the true geometric center (equidistant between the corridor walls) and
   snap heading to the nearest cardinal. The correction is a **relative move** (drive by
   −offset), which is drift-immune over the short distance. Sensing then always happens
   from the true center, where the heuristic is 100 %.

3. **Odom used only for short relative motion**: the ~2 m hop drive and small centering
   moves. Drift over 2 m is sub-centimeter; re-centering resets cross-corridor error at
   every cell.

4. **Discrete exit check.** "Done" = `current_cell == EXIT_CELL` (not an absolute-pose
   radius test, which drift would break). A loose absolute check may remain as a
   secondary confirm.

## Components

### 1. `tugbot_maze/wall_localize.py` (ROS-free, unit-tested)

```
HALF_CORRIDOR_M = 0.88      # cell half-width (1.0) minus wall half-thickness (0.12)
WALL_DIST_M     = 1.3       # a min-range below this in a cardinal window => wall present

cell_center_offset(ranges, angle_min, angle_inc, yaw,
                   *, half_corridor_m=HALF_CORRIDOR_M, wall_dist_m=WALL_DIST_M)
    -> (off_x, off_y)        # robot position minus true cell center, MAP axes (+x=E,+y=N);
                              # each component is a float, or None if that axis is open
```

Implementation reuses `cell_walls.cell_wall_min_ranges` to get the min range per MAP
cardinal (N/E/S/W) in the ±22° window (rotated by `yaw`). Per axis (define offset so a
centered robot gives 0; +x = east, +y = north):

- **X axis (E,W):**
  - both walls present (`dE < wall_dist` and `dW < wall_dist`): `off_x = (dW − dE) / 2`
  - only E present: `off_x = half_corridor − dE`
  - only W present: `off_x = dW − half_corridor`
  - neither: `off_x = None` (open corridor along x; nothing to reference)
- **Y axis (N,S):** symmetric — both: `off_y = (dS − dN)/2`; only N: `half_corridor − dN`;
  only S: `dS − half_corridor`; neither: `None`.

`heading_snap(yaw) -> (cardinal_yaw, dyaw)`: nearest of {0, ±π/2, π}; `dyaw` is the
small rotation to null. The maze is axis-aligned, so snapping bounds heading drift.

### 2. `tugbot_maze/flood_fill_solver.py` changes

- **State:** add `self.cell` (int tuple), initialized to `ENTRANCE_CELL` when the entry
  drive completes. Remove reliance on `pose_to_cell(odom)` for the working cell.
- **`center` phase (rewritten):** each tick compute `(off_x, off_y)` from
  `wall_localize.cell_center_offset` at the current pose+yaw; build a relative target
  `(pose_x − off_x, pose_y − off_y)` (skip a `None` axis), drive toward it with
  `hop_command`; rotate to the snapped cardinal. When the correctable offsets are within
  `center_tol_m` (≈0.10) and heading within ~5° and the base has settled, **sense the
  cell once** (`_sense`, sticky), then ask the brain for the next cell.
- **`hop` phase:** turn to face the next-cell direction (a cardinal), then drive forward
  ~`CELL_SIZE_M` (2 m) measured as odom *displacement from the hop start* (relative,
  drift-small). On arrival set `self.cell = next_cell` and enter `center`.
- **Exit check:** `self.cell == EXIT_CELL` ⇒ done (publish `EXIT_REACHED`).
- **Watchdog/backup** retained for the rare blocked hop.

### 3. Offline drift-injection harness (the validation we lacked)

Extend `tugbot_maze/maze_sim.py`:
- Add an **odom-drift model** to `MazeSim`: the simulator keeps the *true* pose, but
  exposes a separate **reported (odom) pose** that accumulates a configurable drift
  (e.g. a per-meter systematic bias + small noise) so tests can reproduce the ~0.7 m
  drift. `wall_localize` reads the *true* surrounding walls (raycast), exactly like the
  real lidar senses true geometry.
- New test `test/test_wall_relocalize_sim.py`: run brain + wall-referenced centering +
  hop under **injected drift**, asserting (a) after each `center`, true position is
  within `center_tol` of the cell center, and (b) the robot reaches `EXIT_CELL` staying
  inside. This proves the fix offline before any Gazebo run.

## Data flow

```
/scan ─▶ cell_wall_min_ranges ─▶ wall_localize.cell_center_offset ─▶ center-drive (relative)
                                                                       │ (centered + cardinal)
                                                            cell_walls.sense_cell_walls
                                                                       ▼
self.cell = next (discrete) ◀── hop: turn to cardinal, drive ~2 m forward (relative odom displacement) ◀── brain.next_cell
```

## Error handling / edge cases

- **Open axis (offset None):** no correction on that axis; the perpendicular axis always
  has the corridor walls, and discrete tracking + the short hop bound the residual. A
  long straight run accumulates a little longitudinal residual, corrected at the next
  cell that has a perpendicular wall (frequent in a twisty perfect maze).
- **Cell with no walls within range (rare):** can't center; proceed on odom for that one
  cell (bounded), re-center at the next walled cell.
- **Blocked hop (odom displacement << 2 m by the watchdog):** mark that edge as wall in
  the brain and re-plan (defensive — clean sensing should prevent commanding it).
- **Heading-snap ambiguity at ±45°:** round to nearest; hysteresis not needed since the
  robot starts each hop from a cardinal.

## Testing / validation

1. **Unit** (`test_wall_localize.py`): centered scan → (0,0); robot offset +x → correct
   `off_x`; one-wall cases; open axis → None; heading snap.
2. **Offline guarantee** (`test_wall_relocalize_sim.py`): reaches exit under injected
   drift, staying inside; centered to `center_tol` each cell. Existing offline suite
   stays green.
3. **Gazebo:** `run_flood_fill_maze.sh … odom_locked …`; confirm via the external
   `gz topic -e` GT read that odom-vs-truth stays bounded (re-centering working) and the
   robot reaches the exit.

## Out of scope (YAGNI)

- No EKF/particle-filter fusion or global pose correction — physical re-centering +
  discrete tracking is sufficient and simpler.
- No elaborate cell-boundary/junction feature detection for longitudinal correction
  beyond what perpendicular walls already give.
- No new sensor or `scan_front` bridging (the omni `/scan` is fine; not occluded).

## What stays unchanged (verified correct)

`pose_source=odom_locked`, the maze topology/grid/transform/spawn, the `FloodFillBrain`
(flood + exit-greedy + Trémaux cap), `sense_cell_walls`, `hop_command`, the entry drive.
