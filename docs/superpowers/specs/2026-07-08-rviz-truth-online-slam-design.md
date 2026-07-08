# Truthful RViz for `online_slam`: solver owns `map→odom`

**Date:** 2026-07-08
**Workspace:** `ros2_ws_tugbot_nav_20260705`
**Status:** approved design → implementation plan next

## Problem

During a visual `online_slam` run, when the tugbot crosses the open **maze center** and
turns into the next corridor, the RViz map "goes chaotic" (`地图发生混乱`) for a moment,
then settles. No collision occurs and the robot completes the maze.

### Root cause (diagnosed, evidence-backed)

The launch runs **slam_toolbox unconditionally** (`online_async_launch.py`), which owns the
`/map` occupancy grid and the `map→odom` TF. RViz is anchored entirely to slam_toolbox:
Fixed Frame `map`, the Map display subscribes to `/map`, and the RobotModel + LaserScan are
drawn through slam_toolbox's `map→odom`.

The flood-fill solver, however, navigates off its **own** internal ICP pose
(`_sm_corrected`, in the `map` frame), computed by scan-matching the LIDAR to the growing
self-built wall reference. It publishes **no TF and no markers** — only `/cmd_vel_nav`.

So there are two decoupled pose estimates:

| estimate | drives the robot? | shown in RViz? |
|---|---|---|
| solver ICP (`_sm_corrected`) | **yes** | **never published** |
| slam_toolbox (`/map` + `map→odom`) | no (bootstrap only) | **everything you see** |

The maze center is the open, feature-poor region where **slam_toolbox's own scan-matcher
re-adjusts / loop-closes** — its `map→odom` lurches, so the occupancy grid, the robot model,
and the scan all jump relative to each other. Because that estimate does **not** steer the
robot, navigation is unaffected (0 collisions, clean completion, all logged DIAG poses
grid-aligned). RViz is simply showing the *wrong, wobblier* estimate. This is a
**visualization-honesty gap, not a navigation bug**.

## Goal

Make RViz reflect the pose the robot actually uses, so a visual acceptance run is
trustworthy and the center wobble disappears — **for `pose_source=online_slam` only**.
`scan_match` (the banked, pushed A/B baseline) is left completely untouched.

## Chosen approach (user-selected)

**The solver owns `map→odom`.** It publishes `map→odom` derived from its ICP pose; slam_toolbox
is silenced on that transform (but kept running, per user choice). Then the *existing* RViz
RobotModel + LaserScan become truthful for free, and — importantly — the **exit monitor**
(`maze_goal_monitor`, which detects `EXIT_REACHED` via `map→base_link` TF) also keys off the
truthful ICP pose. The solver becomes the single source of truth for pose, RViz, and exit
detection.

Rejected alternative (marker-overlay-only, no TF change): leaves RobotModel + raw LaserScan
still TF-wobbly, requiring marker/pointcloud gymnastics to hide them. The TF-ownership route
is cleaner and makes the native displays honest.

Scope: gated on `online_slam`. `scan_match` keeps slam_toolbox as its TF owner exactly as
banked (zero regression risk to the pushed baseline).

## Architecture

```
BEFORE:  slam_toolbox ──map→odom──┐                    (wobbles in open center)
         diff-drive ───odom→base──┴→ RViz + exit_monitor
         solver ICP ─── _sm_corrected (invisible, internal)

AFTER (online_slam):
         solver ICP ──map→odom──┐                      (truthful, grid-locked)
         diff-drive ──odom→base─┴→ RViz + exit_monitor
         + solver ──MarkerArray /maze/self_built_walls  (the growing self-built map)
         slam_toolbox: runs, builds internal map, map→odom silenced, /map hidden
```

Per control tick (`online_slam` only): the solver already computes `_sm_corrected` (the
ICP `map→base` pose) and reads `odom→base` from TF. It publishes
`map→odom = compose_2d(_sm_corrected, invert_2d(odom_base))`, closing the tree
`map→odom→base` at the true pose. The diff-drive continues to own `odom→base`; the solver
owns only `map→odom` (the transform slam_toolbox is silenced on).

## Components

### a. `tugbot_maze/pose_tracking.py` — pose-math helpers (additive)

Add two small, pure, unit-tested helpers next to the existing `compose_2d` / `odom_prior`:

- `invert_2d(pose: Pose2D) -> Pose2D` — inverse of a 2D rigid transform. Property:
  `compose_2d(invert_2d(p), p) == identity` and `compose_2d(p, invert_2d(p)) == identity`.
- `yaw_to_quat(yaw: float) -> tuple[float, float, float, float]` — `(x, y, z, w)` for a
  planar yaw (`z = sin(yaw/2)`, `w = cos(yaw/2)`, `x = y = 0`), for the TF message rotation.

### b. `tugbot_maze/flood_fill_solver.py` — three additions, all gated on `online_slam`

1. **Own the TF.** Create a `tf2_ros.TransformBroadcaster`. Each control tick, when
   `pose_source == 'online_slam'` and both `_sm_corrected` and `odom_base` are available,
   publish `map→odom = compose_2d(_sm_corrected, invert_2d(odom_base))` as a stamped
   `TransformStamped` (`header.frame_id = map_frame`, `child_frame_id = odom_frame`,
   rotation from `yaw_to_quat`).

2. **Bootstrap from the known entrance.** With slam_toolbox's `map→odom` silenced, the
   solver can no longer seed `_sm_corrected` from slam_toolbox's `map→base` during
   `startup`/`entering`. Instead, in `online_slam` mode, seed `_sm_corrected` from the known
   entrance pose read from new node params `entrance_x` / `entrance_y` / `entrance_yaw`
   (default `0.0, 0.0, 0.0`), and publish `map→odom` from the first tick so the exit monitor
   and RViz have a valid frame immediately. (`scan_match` bootstrap path is unchanged.)

3. **Publish the self-built map.** Create a `MarkerArray` publisher on
   `/maze/self_built_walls`. Content: **perimeter + all confirmed walls accumulated so far**
   (the growing self-built map — *not* just the localizer's local window), as a single
   `LINE_LIST` marker in the `map` frame (two points per wall segment). Republish when the
   confirmed-wall set grows (dedupe by a signature so it is not re-sent every tick). Wall
   segments reuse the existing grid-snapped geometry (`confirmed_wall_segments` +
   `outer_segments()` / perimeter already available to the solver).

### c. Launch — `tugbot_maze_explore.launch.py` / `tugbot_maze_slam_nav.launch.py`

- When `pose_source == 'online_slam'`: pass `transform_publish_period:=0.0` to slam_toolbox
  to silence its `map→odom` broadcast while the node keeps running and building its internal
  map. (See **Known risk** for the empirical verification + fallback.)
- Wire `entrance_x` / `entrance_y` / `entrance_yaw` (already declared launch args) into the
  **flood_fill_solver** node's parameters (it does not currently receive them).
- `scan_match` and all other modes: launch wiring unchanged.

### d. RViz — `tugbot_bringup/config/tugbot_nav.rviz` (and the mirrored `rviz/tugbot_nav.rviz`)

- Add a `visualization_msgs/MarkerArray` display `SelfBuiltWalls` subscribed to
  `/maze/self_built_walls`.
- Set the existing `/map` (Map) display `Enabled: false` by default — present-but-toggleable,
  so slam_toolbox's view can still be compared on demand.
- Keep RobotModel + LaserScan (now truthful via the solver's TF). Fixed Frame stays `map`.

## Data flow (per tick, `online_slam`)

`diff-drive → odom→base (TF)` · `ICP → _sm_corrected (map→base, internal)` ·
**solver publishes `map→odom = _sm_corrected ⊖ odom_base`** → tree closes at the true pose →
`maze_goal_monitor` reads the truthful `map→base` for exit detection; RViz RobotModel + Scan
render at the true pose; `SelfBuiltWalls` markers draw the growing map; slam_toolbox `/map`
hidden.

## Error handling

- Before the first entrance-seed, or if `odom_base` is `None` on a tick → **skip** that tick's
  TF publish. The exit monitor already tolerates missing TF (`waiting for TF …`). With
  entrance-seeding, TF is live from the first driving tick.
- MarkerArray publish wrapped in try/except that logs a warning and continues — never kills
  the node (matches the existing `_flush_junctions` discipline).
- A localizer hiccup already falls back to the odom prior; the TF is still published from that
  prior pose, so there is no TF gap.
- **Navigation is unchanged by construction:** control already runs off `_sm_corrected`; this
  work only *exposes* that pose via TF + markers and changes the bootstrap seed source.
  Completion should match the banked 16/16 baseline.

## Testing

**Unit (`test_pose_tracking.py`, new/extended):**
- `invert_2d` round-trips: `compose_2d(p, invert_2d(p)) ≈ identity` for several poses
  (including non-zero yaw and translation).
- `yaw_to_quat`: known angles (0, π/2, π, −π/2) produce correct `(x,y,z,w)`.
- `map→odom` composition: for chosen `map→base` and `odom→base`,
  `compose_2d(compose_2d(map_base, invert_2d(odom_base)), odom_base) ≈ map_base`.

**Unit (`test_flood_fill_solver` or a focused helper test):**
- The self-built-walls marker builder returns a `LINE_LIST` in the `map` frame with two
  points per segment, containing exactly perimeter + confirmed walls, and dedupes so an
  unchanged set does not trigger a republish.

**Offline (`test_online_slam_sim.py`, existing):**
- Still completes with no localization-error regression (`max_loc_err < 0.5`), confirming the
  additive TF/markers do not perturb the control loop.

**Gazebo (authority — the real acceptance):**
- A visual `online_slam` run: robot + scan + self-built walls stay **stable through the
  center** (the wobble is gone); `EXIT_REACHED` still fires (now off the ICP pose — final
  pose ≈ `(20.22, 18.00)`, distance to exit `(21.07, 18.08)` ≈ `0.85 m` < `1.2 m` radius ✓);
  **0 collisions** via the true-footprint oracle; completion time ~unchanged vs the banked
  baseline (no navigation regression).

## Known risk (flagged, with fallback)

Whether `transform_publish_period:=0.0` fully silences slam_toolbox's `map→odom` while
keeping the node alive could not be confirmed from docs (Context7 does not index
slam_toolbox). **Implementation step 1 verifies it empirically**: launch `online_slam` with
the param and run `ros2 run tf2_ros tf2_echo map odom` — only the solver should be
publishing. If slam_toolbox still broadcasts `map→odom` (→ TF conflict, defeating the fix),
the fallback is to **reframe slam_toolbox to a non-conflicting `map_slam` frame** (node still
runs and builds its map; its `/map` is hidden anyway) rather than silence the broadcast.
This is settled in the plan, not guessed here.

## Out of scope

- Dropping slam_toolbox entirely (the earlier "option 2"). It stays running.
- Any change to `scan_match`, `odom_locked`, `slam`, or the DFS/Nav2 explorers.
- Publishing the live LIDAR as a separate truthful point cloud — the native LaserScan
  display becomes truthful automatically once the solver owns the TF, so no scan-cloud is
  needed (YAGNI).

## Success criteria

1. In an `online_slam` visual run, the RViz robot + scan + self-built walls remain coherent
   through the maze-center turn — no map chaos.
2. `EXIT_REACHED` still fires, driven by the solver's ICP pose.
3. True-footprint oracle: 0 collisions; completion time and success unchanged vs the banked
   16/16 baseline.
4. `scan_match` behavior and wiring are byte-for-byte unchanged.
