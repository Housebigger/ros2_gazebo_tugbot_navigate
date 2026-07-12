# Autonomous Maze Navigation — `ros2_ws_tugbot_nav_20260705`

ROS 2 (Jazzy) + Gazebo Harmonic stack that drives the Tugbot from the maze entrance
(map `(0, 0)`, cell `(1,0)`) to the exit (map `(21.07, 18.08)`, cell `(10,9)`) of a
~10×10-cell *perfect maze* (a tree: 2 m cells, ~1.76 m corridors). This workspace solves
the **same `20260528` maze** as `ros2_ws_tugbot_nav_20260614`, but with one additional constraint:
**the interior wall map is withheld** — the robot must build it online while navigating.
Grid geometry, entrance/exit, and the outer perimeter are known from the start; every
interior wall is discovered at runtime. The new localization mode is `pose_source:=online_slam`.

## Headline result

**Offline (ROS-free): maze completes with interior map withheld.** Both drift=0.0 and
drift=0.03 simulations reach the exit with no collision in `test_online_slam_sim.py`.
Peak localization error is ~0.40 m (at drift=0.03; reduced from ~1.10 m by the inclusion
of local-cell freshly-sensed walls in the reference set). All assertions pass: exit reached,
no collision, peak error < 0.50 m.

**Gazebo acceptance: pending.** The controlled Gazebo batch (target ≥ 14/16
`EXIT_REACHED`, no interior map fed) has **not yet been run**. The fed-map baseline
(`pose_source=scan_match`, which delivered 16/16 in `ros2_ws_tugbot_nav_20260614`) is the
A/B **upper-bound reference** here. No `ros2_ws_tugbot_nav_20260705` Gazebo results exist
yet — this README will be updated after a batch completes.

## Explorer modes

| `explorer_type` (+ args) | Strategy | Status |
|---|---|---|
| **`flood_fill`** + `pose_source:=online_slam` | Cell-grid flood-fill routing + `MazeMotion`, localized by ICP against a **growing reference**: known perimeter + committed interior walls + current-cell freshly-sensed walls. **No interior map fed.** | ✅ **Offline green** (exit reached, no collision, peak loc-err ~0.40 m) — Gazebo batch **pending** |
| `flood_fill` + `pose_source:=scan_match` | Same solver, but fed the **full known wall map** — ICP against all walls from t=0 | A/B upper-bound baseline (16/16 in 20260614; use to bracket `online_slam`) |
| `flood_fill` + `pose_source:=odom_locked` / `slam` | Same solver on wheel-odometry / live-SLAM pose | ⚠️ A/B only — desyncs in the open interior (0/8 in 20260614); the reason scan-match was built |
| `guided_corridor_mode:=true` (GCN, in `…_20260522`) | Follow a pre-computed BFS corridor sequence | ✅ Reliable, but *guided* (not autonomous discovery) — the original reliable route |
| `wall_follower` | Reactive right/left-hand wall-following | ❌ Stateless; loops in the interior. Superseded by flood-fill. |
| `tremaux` / `frontier` / `maze_dfs` | Junction-graph Trémaux DFS / SLAM-frontier / greedy DFS | ⚠️ Earlier autonomous attempts — plateau ~8.7–11 m from the exit (Nav2 point-goals wedge in 1.76 m corridors) |

## How to run

```bash
cd ros2_ws_tugbot_nav_20260705
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash

# New: online_slam — no interior map fed, self-builds the reference while driving:
export DISPLAY=:1   # if running headed under a virtual display
bash tools/run_flood_fill_maze.sh 1200 false true online_slam true
#   MAX_SECONDS=1200 HEADLESS=false USE_RVIZ=true POSE_SOURCE=online_slam SENSE_DEBUG=true
#   Artifacts: log/flood_fill_run_<stamp>/ (result.txt, run_meta.txt, flood_fill_tail.txt, launch.log)

# A/B upper-bound baseline (full map fed — 20260614 reference):
bash tools/run_flood_fill_maze.sh 1200 false true scan_match true

# Controlled reliability batch (N=8, headless, no map fed):
bash tools/batch_diagnose_floodfill.sh 8 1200 true false online_slam true
#   -> log/batch_diag_<stamp>/report.txt  (EXIT_REACHED / failure-mode distribution)

# True-footprint collision oracle over a batch:
python3 tools/replay_collision_oracle.py log/batch_diag_<stamp>

# Offline suite (pure Python, no ROS, fast) — the gate for this phase:
cd src/tugbot_maze && python3 -m pytest \
  test/test_online_slam_sim.py \
  test/test_online_scan_match_localizer.py \
  test/test_scan_match_localizer.py test/test_pose_tracking.py \
  test/test_maze_motion_sim.py test/test_maze_sim.py \
  test/test_flood_fill_brain.py test/test_cell_walls.py \
  test/test_hop_controller.py test/test_wall_localize.py -q
```

### Two ways to launch a visualization run

`pose_source` accepts `online_slam`, `scan_match` (A/B upper bound), `odom_locked`, and `slam`.

**Method A — wrapper script (recommended).** Same command as always; a visual run just sets the
2nd arg (`HEADLESS`) to `false`:

```bash
tools/run_flood_fill_maze.sh 1000 false true online_slam
#                            ^MAX_S ^HEADLESS=false(GUI) ^USE_RVIZ=true ^POSE_SOURCE
```

The wrapper does three things a bare `ros2 launch` does **not**:
- **Process + Fast-DDS SHM cleanup and a unique `ROS_DOMAIN_ID` per run**, so back-to-back runs are
  safe (see the caveat under Method B).
- **`EXIT_REACHED` watchdog + automatic teardown** (kills the launch and strays).
- Writes artifacts to `log/flood_fill_run_<stamp>/` (`result.txt`, `run_meta.txt`, `launch.log`).

**Method B — native `ros2 launch`.** Full manual control. Note `explorer_type` defaults to
`maze_dfs`, so `explorer_type:=flood_fill` is **mandatory** or the flood-fill node never starts:

```bash
cd ros2_ws_tugbot_nav_20260705
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch tugbot_bringup tugbot_maze_explore.launch.py \
    headless:=false use_rviz:=true \
    explorer_type:=flood_fill entry_direct_distance_m:=2.0 \
    pose_source:=online_slam
```

Caveats vs. the wrapper (you must handle these yourself):
1. **No SHM/domain cleanup.** The first run is usually fine, but a repeat run often fails with
   `RTPS_TRANSPORT_SHM ... open_and_lock_file failed` (leaked Fast-DDS shared-memory port locks →
   Nav2 never activates; looks like a nav failure but is pure cleanup contamination). Before
   re-running:
   ```bash
   pkill -9 -f "gz sim|ros2 launch|parameter_bridge|slam_toolbox|flood_fill_solver" 2>/dev/null
   rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
   ```
2. **No auto-teardown.** Stop with `Ctrl-C`, then run the cleanup one-liner above to avoid orphan
   processes.

## How online localization works

The interior wall layout is unknown at the start, but the **grid geometry is exact**. A sensed
wall's centerline lies on a known cell-edge — not a floating estimate. This collapses the SLAM
problem to two sub-problems:

1. **Discover** which known cell-edges are walls (the flood-fill brain already does this — it
   senses each cell's four cardinals from LIDAR and corroborates them across visits).
2. **Localize** each tick by scan-matching the live beams against the confirmed walls, whose
   centerlines are exact (grid-snapped).

Because the reference geometry is grid-snapped, the ICP yields an **absolute, drift-free pose**
in the grid frame — exactly as in `ros2_ws_tugbot_nav_20260614` — the only difference being the
reference set **grows** as exploration proceeds instead of being complete from t=0.

### The three-layer reference set

Each tick, `OnlineScanMatchLocalizer.correct()` matches the live `/scan` against a reference
built from three sources:

> 1. **Known perimeter** (`outer_segments()`) — the boundary walls, known at start; the cold-start
>    anchor while the robot is near the entrance.
> 2. **Committed interior walls** (`confirmed_wall_segments(brain, ...)`) — all walls the
>    flood-fill brain has corroborated past its commit threshold (only `committed` edges enter;
>    a single poor sense is never fed to the ICP, so it cannot poison localization).
> 3. **Current cell's freshly-sensed walls** (`local_reference_cells(committed, cell, sensed)`)
>    — the cell the robot is currently in, grid-snapped from the live sense; adds near-field
>    reference the moment the robot enters a new cell, before the brain has committed anything.

Grid-snapping keeps each confirmed wall's centerline at its true geometric position, so the pose
remains absolute and drift-free even as the reference grows. **No pose-graph or loop-closure** is
needed: there is no accumulating map uncertainty to close because each segment's geometry is exact.

### Observability gate and odom fallback

When the confirmed set is too sparse to constrain a DOF (the eigenvalue gate on the ICP JᵀJ
block fails), that DOF **falls back to odom-propagation** — the same per-DOF gate the
`ros2_ws_tugbot_nav_20260614` `scan_match_localizer` already uses. `slam_toolbox` keeps only its
existing cold-start role (initial `map→base_link` bootstrap at the entrance).

The ICP core (`point-to-line`, observability-gated) is the **same validated math** as in
`scan_match_localizer.py`; `OnlineScanMatchLocalizer` reuses it with a mutable, per-tick segment
set instead of a static one.

## Architecture (`src/tugbot_maze/`)

A thin ROS node (`flood_fill_solver.py`) owns only ROS plumbing; **all** control is a ROS-free
core, validated offline by a ROS-free maze simulator *before* any Gazebo run:

| Unit | Role | ROS-free? |
|---|---|---|
| `flood_fill_brain.py` | Cell-grid routing — flood-fill / Trémaux-capped BFS `next_cell`; dead-end & reachability logic | yes |
| `maze_motion.py` | Motion FSM (`center → turn → drive`, + backout/recover/unstick); corridor pure-pursuit + profiled rotate-in-place + wall-referenced re-anchor; **includes reverse-to-center** | yes |
| `cell_walls.py` | Per-cell wall sensing — projects LIDAR beams onto cardinals, median over a window | yes |
| **`online_scan_match_localizer.py`** | **Point-to-line ICP scan-to-growing-reference → absolute pose** (the new component); `confirmed_wall_segments()` + `local_reference_cells()` helpers build the per-tick reference from perimeter + committed + current-cell sensed walls | yes |
| `scan_match_localizer.py` | Same ICP core against the **static full known map** — kept as the A/B upper-bound baseline (`pose_source=scan_match`) | yes |
| `pose_tracking.py` | SE(2) helpers (`compose_2d`, `inverse_2d`, `odom_prior`) for the per-tick odom prior | yes |
| `maze_sim.py` | **Test-only** raycaster + unicycle integrator over the real `20260528` wall segments, with the **true asymmetric-footprint collision oracle** (rear gripper at −0.468 m; the honest collision truth) | yes |
| `footprint.py` | Robot geometry constants (`FOOT_X_FRONT/REAR`, `FOOT_HALF_W`, LIDAR `SCAN_OFFSET_X=-0.1855`) | yes |
| `map_memory.py` / `wall_localize.py` / `hop_controller.py` | Map-integrity guard / perimeter & cell-center offsets / low-level motion commands | yes |
| `flood_fill_solver.py` | Thin node — `/scan` + TF → `_lookup_pose` (`online_slam`/`scan_match`/`odom_locked`/`slam`) → `MazeMotion` → `/cmd_vel_nav` at 10 Hz; DIAG + MATCH logging | no |

`slam_toolbox` runs only to bootstrap the initial `map→base_link` pose; once driving,
`online_scan_match_localizer` owns the pose. **Nav2 / costmaps are not in the flood-fill
control loop.**

Design spec: `docs/superpowers/specs/2026-07-05-online-slam-maze-design.md`.

## Relationship to `ros2_ws_tugbot_nav_20260614`

Only the **localization layer** changed. Every other component is carried over **unchanged**:

| Carried over unchanged | New / changed in 20260705 |
|---|---|
| `maze_motion.py` (incl. reverse-to-center fix), `flood_fill_brain.py`, `cell_walls.py`, `footprint.py`, `pose_tracking.py`, `hop_controller.py`, `map_memory.py`, `wall_localize.py` | `online_scan_match_localizer.py` (the one genuinely new component) |
| `maze_sim.py` true-footprint oracle | `confirmed_wall_segments()` / `local_reference_cells()` helpers |
| `batch_diagnose_floodfill.sh`, `replay_collision_oracle.py`, `run_flood_fill_maze.sh`, `diagnose_floodfill_runs.py` | `flood_fill_solver.py` wired for `pose_source:=online_slam` |
| `scan_match_localizer.py` (static full map; A/B upper-bound baseline) | `test_online_slam_sim.py`, `test_online_scan_match_localizer.py` |

The `20260614` world file (`tugbot_maze_world_20260528_clean_scaled2x.sdf`) is the same, so
Gazebo results are directly comparable.

## Diagnostic tooling

- `tools/diagnose_floodfill_runs.py` — classifies run artifacts into failure modes
  (final-approach / desync / sense-heavy / early-stuck) from the logs; reports the distribution.
- `tools/batch_diagnose_floodfill.sh` — runs N identical Gazebo runs and auto-classifies them,
  so decisions rest on the run *distribution*, not single-run anecdotes. Pass
  `POSE_SOURCE=online_slam` to evaluate the no-map mode; `POSE_SOURCE=scan_match` for the upper
  bound A/B. Both use the same classification logic.
- `tools/replay_collision_oracle.py` — replays the DIAG pose log for each run against the
  true asymmetric footprint to give the honest per-pose collision count.

## Test suite notes

The maintained **ROS-free offline suite** is the gate for this phase and is green:
`test_online_slam_sim.py`, `test_online_scan_match_localizer.py`, `test_hop_controller.py`,
`test_maze_motion_sim.py`, `test_maze_sim.py`, `test_flood_fill_brain.py`,
`test_cell_walls.py`, `test_scan_match_localizer.py`, `test_pose_tracking.py`,
`test_wall_localize.py`.

A broad `pytest` of the full package shows approximately **7 pre-existing failures**
(`wall_follow_solver` + some contract/integration tests) inherited from `ros2_ws_tugbot_nav_20260614`
and **unrelated to this phase** (they were present before any `online_slam` work). They are not
regressions introduced here. The offline suite listed above is the authoritative gate.
