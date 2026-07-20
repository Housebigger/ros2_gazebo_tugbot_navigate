# Autonomous Maze Navigation — `ros2_ws_tugbot_nav_20260722`

ROS 2 (Jazzy) + Gazebo Harmonic stack that drives an **ANYmal C quadruped** — as of this
20260717 iteration the dog **physically walks**: gravity is on, all 54 CERBERUS collision
bodies are restored, the `VelocityControl` "magic thruster" plugin is deleted, and all
12 joints run gz `JointPositionController` in **torque-PID mode** (`p_gain=320 / d_gain=8`,
`cmd_max=80` N·m), driven by a self-built trot controller (`tugbot_maze/legged/` — pure
`kinematics`/`trajectory`/`trot`/`stabilizer`/`params` modules, no ROS deps — plus the
`locomotion_controller` node running at 100 Hz: `/cmd_vel` → phase-aligned trot FSM → 12×
`/model/anymal_c/joint/<J>/cmd_pos`). `/odom` still comes from the gz `OdometryPublisher`
world-truth plugin, now at `dimensions=3` (was 2 — the stabilizer and fall detector both
need roll/pitch/z, which `dimensions=2` zeroes). The base moves **only** from leg-ground
contact forces now — from the maze entrance
(map `(0, 0)`, cell `(1,0)`) to the exit (map `(21.07, 18.08)`, cell `(10,9)`) of a
~10×10-cell *perfect maze* (a tree: 2 m cells, ~1.76 m corridors). This workspace solves
the **same `20260528` maze** as `ros2_ws_tugbot_nav_20260614`, but with one additional constraint:
**the interior wall map is withheld** — the robot must build it online while navigating.
Grid geometry, entrance/exit, and the outer perimeter are known from the start; every
interior wall is discovered at runtime. The new localization mode is `pose_source:=online_slam`.

## 3D lidar

The ANYmal C carries one `lidar_3d` (CERBERUS `front_laser` spec, but re-centered — see
below), a 16-beam `gpu_lidar` at `1800×16` beams (`1800` azimuth samples over the full
360°, `16` vertical rings over `±15°`), `10 Hz`. Unlike the retired 2D `scan_omni`, its
pose is `(0, 0, 0.35, 0, 0, 0)` — **centered on the back**, at the old 2D scanner's
position, deliberately *not* the off-center/rotated original CERBERUS `front_laser` pose
(the nav chain assumes `SCAN_OFFSET_X=0`). It publishes a full 3D point cloud on
`/lidar/points` (`sensor_msgs/msg/PointCloud2`; on the gz side this lands on
`/lidar/points/points` — `gpu_lidar` auto-splits its own `<topic>` into a `LaserScan`
topic plus a separate `<topic>/points` cloud topic, so the bridge points at the latter).

The legacy 2D `/scan` that the whole nav/localization chain consumes is now **synthesized**,
not sensed directly: the `scan_slice_projector` node subscribes `/lidar/points`, keeps only
the two near-horizontal `±1°` rings, min-folds their points into the legacy 900-bin
`LaserScan` contract (field values captured verbatim from the last 2D boot in
`src/tugbot_maze/test/scan_omni_baseline.json`), and republishes `/scan` at 10 Hz. The
nav/localization chain (`flood_fill_solver`, `maze_motion`, ICP, `footprint.py`) runs
**unchanged** — it still only ever sees a 900-bin `LaserScan`.

| ROS topic | Type |
|---|---|
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` |
| `/scan` | `sensor_msgs/msg/LaserScan` (synthesized by `scan_slice_projector`) |

RViz's default config has a `Lidar3D` PointCloud2 display subscribed to `/lidar/points`
(colored by height, alongside the existing `LaserScan`, `SelfBuiltMap`, and `FrontCamera`
views) so the raw 3D cloud is directly visible, not just the projected 2D slice.

**Live gate**: `bash tools/verify_lidar3d.sh` — alongside `bash tools/verify_front_camera.sh`
— boots the maze world headless and asserts `/lidar/points` streams real 3D clouds and the
projected `/scan` reproduces the legacy field contract exactly (900 samples, angle/range
fields, empty-bin encoding).

## Front camera

The ANYmal C carries one forward-facing RGB camera, `camera_front`, mounted on the base
link (`720×540 @ 20 Hz`, `126°` hfov). It faces **+x — the actual travel direction**
(stance feet push -x): none of the 7 original CERBERUS camera poses could be reused, since
that convention treats **-x** as "front". Pure observation — no vision feeds the
nav/localization chain (still LIDAR-only ICP, `pose_source=online_slam`).

| ROS topic | Type |
|---|---|
| `/camera/front/image` | `sensor_msgs/msg/Image` |
| `/camera/front/camera_info` | `sensor_msgs/msg/CameraInfo` |

Both publish `frame_id=anymal_c/base/camera_front`, anchored to `base_link` by a static
TF. RViz's default config has a `FrontCamera` Image panel subscribed to
`/camera/front/image`, alongside the existing SelfBuiltMap view.

**Live gate**: `bash tools/verify_front_camera.sh` boots the maze world headless (the dog
just stands) and asserts real frames publish — resolution, ~20 Hz rate, correct
`frame_id`, non-constant content.

## Headline result

**Offline (ROS-free): maze completes with interior map withheld.** Both drift=0.0 and
drift=0.03 simulations reach the exit with no collision in `test_online_slam_sim.py`.
Peak localization error is ~0.40 m (at drift=0.03; reduced from ~1.10 m by the inclusion
of local-cell freshly-sensed walls in the reference set). All assertions pass: exit reached,
no collision, peak error < 0.50 m.

**Gazebo acceptance: complete (this phase, legged walking).** The 20260717 physically-walking
dog reached the exit in both headless verification runs: **2/2 `EXIT_REACHED`** (~677 s and
~696 s sim-time), collision oracle **0.000%** on both runs, **0 falls**
(`FALL_DETECTED=0`), no interior map fed (`pose_source=online_slam`) — see the spec addendum
item 12 for full detail (ICP `MATCH` rms median 0.026, n median 686/688.5). The fed-map
baseline (`pose_source=scan_match`, which delivered 16/16 in `ros2_ws_tugbot_nav_20260614`)
remains the A/B **upper-bound reference**. `ros2_ws_tugbot_nav_20260705` onward each completed
their own Gazebo acceptance batches for the *localization* layer in their respective phases;
this iteration's acceptance is the *legged-locomotion* result above.

## Explorer modes

| `explorer_type` (+ args) | Strategy | Status |
|---|---|---|
| **`flood_fill`** + `pose_source:=online_slam` | Cell-grid flood-fill routing + `MazeMotion`, localized by ICP against a **growing reference**: known perimeter + committed interior walls + current-cell freshly-sensed walls. **No interior map fed.** | ✅ **Offline green** (exit reached, no collision, peak loc-err ~0.40 m) — ✅ **Gazebo: 2/2 `EXIT_REACHED`** (legged walk, 20260717; oracle 0.000%, 0 falls) |
| `flood_fill` + `pose_source:=scan_match` | Same solver, but fed the **full known wall map** — ICP against all walls from t=0 | A/B upper-bound baseline (16/16 in 20260614; use to bracket `online_slam`) |
| `flood_fill` + `pose_source:=odom_locked` / `slam` | Same solver on wheel-odometry / live-SLAM pose | ⚠️ A/B only — desyncs in the open interior (0/8 in 20260614); the reason scan-match was built |
| `guided_corridor_mode:=true` (GCN, in `…_20260522`) | Follow a pre-computed BFS corridor sequence | ✅ Reliable, but *guided* (not autonomous discovery) — the original reliable route |
| `wall_follower` | Reactive right/left-hand wall-following | ❌ Stateless; loops in the interior. Superseded by flood-fill. |
| `tremaux` / `frontier` / `maze_dfs` | Junction-graph Trémaux DFS / SLAM-frontier / greedy DFS | ⚠️ Earlier autonomous attempts — plateau ~8.7–11 m from the exit (Nav2 point-goals wedge in 1.76 m corridors) |

## How to run

```bash
cd ros2_ws_tugbot_nav_20260722
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
cd ros2_ws_tugbot_nav_20260722
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

### Yaw-only fallback and clamp-lock escape (20260721)

The 55-60% deterministic-rejection background state (gates firing in uncommitted
regions) used to mean pure dead reckoning: heading drift accumulated freely until
the next full ICP acceptance. Two bounded correctors now narrow that window, both
implemented in `online_scan_match_localizer.py` with zero navigation-chain changes:

- **Yaw-only fallback** (`yaw_only_correct`): when full ICP is gated
  (sparse-interior / no-local-interior-walls) or the inner ICP clamp-rejects, a
  1-DOF grid search (window +/-0.2 rad, step 0.01) re-scores yaw against the same
  reference set. Selection is inlier-primary (argmax inliers, rms tie-break —
  pure rms-argmin picks the wrong sign at low inlier counts). Acceptance is
  branch-gated: interior branch needs n>=60 inliers, >=20% rms improvement and
  rms <= 0.15; the saturated branch needs a >=20% relative inlier increase with
  an n>=30 floor. Every accepted step passes through an unconditional +/-0.1
  rad/tick saturating clamp; x/y are never touched.
- **Clamp-lock escape**: a converged-but-clamp-rejected inner ICP (n>=100,
  converged pose available) repeated 3 ticks in a row indicates the prior is
  pinned outside the accept region ("clamp lock" — the mechanism behind the one
  runaway TIMEOUT in this phase's first batch). The escape applies one bounded
  step toward the converged pose (translation <= 0.15 m direction-preserved,
  yaw <= 0.1 rad) and resets the streak; the correction self-damps as the
  distance decays.

Observability: the solver MATCH log line carries `fb=` (which fallback fired:
`yaw_only` / `clamp_escape`), `ystep=` (applied yaw step) and `gr=` (gate
reason); `tools/replay_collision_oracle.py` prints a `live_rate=` alongside the
official rate (deduplicating consecutive bit-identical poses, so stuck dwell
cannot inflate the collision statistic).

Result over the acceptance campaign (8 headless + 1 GUI): 7 clean completions
(worst oracle 0.690%, single-tick), zero clamp-lock recurrences, and the
remaining TIMEOUTs traced to routing-layer causes (exploration-graph
exhaustion), not localization — see the failure taxonomy in
`docs/superpowers/specs/2026-07-19-yaw-only-fallback-design.md`.

## Routing layer: exhaustion recovery (20260722)

The exploration graph can exhaust itself: every neighbour is visited, so the
escape ladder's tier-2 "give up the blocked edge" is structurally blocked and
the 90 s no-progress watchdog degenerates into a reverse-and-return ping-pong
that changes nothing. Forensics on two TIMEOUT runs measured the cost: 23-25
escapes at a metronomic 90.1 s cadence, 88% of the run inside the escape
regime, ~1650-1900 s of pure dead time. Two changes in `maze_motion.py` attack
that, with the rest of the navigation chain frozen:

- **Single-edge exit-directed UNSTICK.** `_unstick` used to re-open every cut
  edge of a trust tier at once (one observed rollback re-opened 16 committed
  walls and cost minutes of re-sensing). It now re-opens exactly ONE edge per
  invocation: within the first non-empty trust tier (locomotion -> non-committed
  sensed -> committed, order unchanged) it picks the edge whose far endpoint is
  geometrically nearest `EXIT_CELL`, ties preferring edges incident to the
  current cell. The `reopened` bound (both directed reps) and the
  exhausted-means-disconnected -> `stuck` termination are unchanged.
- **Zero-growth escape escalation, three-way.** `growth` = cells added to
  `visited` since the previous escape. `growth > 0` keeps the old ladder
  byte-identical. `growth == 0` is the exhaustion signal and branches on what
  map-layer action is actually available: a non-empty cut set diverts to the
  single-edge `_unstick` (fast 30 s window armed); an empty cut with a non-cut
  locomotion wall still believed re-verifies that wall instead (single-edge,
  `reopened`-bounded, nearest-to-exit first) -- a false hop-failure wall sitting
  INSIDE the connected component is invisible to cut-based unsticking, and in
  every trapped run that wall was the same edge on the winning route; with
  neither available the calm 90 s window is restored and the ordinary ladder
  reverse runs, so a physical re-approach is always the fallback and the robot
  can never freeze.

Observability: the ESCAPE log line carries `growth=`, `win=` (the adaptive
window in force), `divert=` (`unstick` / `reverify` / `ladder`) and `cut_n=`;
UNSTICK reopens carry `edge=`, `tier=` (incl. `loco_reverify`) and `dexit=`;
the exhausted line carries `R=` and `noncut_loco=`. `tools/p3_forensics.py`
replays a run's escape/unstick timeline, cadence and per-window phase mix.

Results, stated honestly. The freeze class this phase targeted is gone: across
all 10 runs on the amended stack (8 statistical-gate + 2 GUI acceptance) there
is not one `UNSTICK exhausted`, `stuck`, or mass re-open -- against 62-79
exhausted-freeze cycles in every one of the three pre-amendment runs that
exposed the defect. Every zero-growth escape is justified by its logged cut
state, and the re-verify lever hit the recurring poison edge in all four
trapped gate runs (a fifth time in the GUI TIMEOUT), with a successful
traversal afterwards in three of those four -- in the fourth the robot was
already standing on the far side of the edge, so routing correctly preferred
its exit-ward alternatives. What did NOT improve is completion incidence:
the statistical gate ran 4 EXIT / 4 TIMEOUT, versus 8 EXIT / 12 runs on the
predecessor stack (Fisher exact two-sided p = 0.648 - indistinguishable at
these sample sizes). Every remaining TIMEOUT is localization-rooted (P1 class):
a rejection-streak wedge marks a false wall while the believed pose is
0.7-0.9 m off centre, or a mislocalized sensing episode stamps OPEN onto a real
wall and the routing layer has no lever for a believed-OPEN edge. That is the
next phase's target, not this one's. See
`docs/superpowers/specs/2026-07-19-p3-routing-fix-design.md` for the failure
taxonomy and the full gate record.

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
| `maze_sim.py` | **Test-only** raycaster + unicycle integrator over the real `20260528` wall segments, with the **true-footprint collision oracle** (dynamic ANYmal C trot-gait envelope rectangle ±0.49 × ±0.37 m; the honest collision truth) | yes |
| `footprint.py` | Robot geometry constants (`FOOT_X_FRONT/REAR=±0.49`, `FOOT_HALF_W=0.37` — the dynamic trot envelope, dominates `legged/params.foot_envelope()`; locked by `test_footprint_covers_gait_envelope`), LIDAR `SCAN_OFFSET_X=0.0` — body-centre; `/scan` is now the `scan_slice_projector` projection of `lidar_3d`, still centered so the offset is unchanged | yes |
| `map_memory.py` / `wall_localize.py` / `hop_controller.py` | Map-integrity guard / perimeter & cell-center offsets / low-level motion commands (`hop_command` `v_max=0.4` aligned to the gait's `vx_max`) | yes |
| `flood_fill_solver.py` | Thin node — `/scan` + TF → `_lookup_pose` (`online_slam`/`scan_match`/`odom_locked`/`slam`) → `MazeMotion` → `/cmd_vel_nav` at 10 Hz; DIAG + MATCH logging | no |
| **`legged/kinematics.py` / `trajectory.py` / `trot.py` / `stabilizer.py` / `params.py`** | **New in 20260717** — pure functions, no ROS: per-leg SDF-anchored FK + numeric (damped Gauss-Newton) IK, sinusoidal-lift swing + stance foot trajectory, the diagonal-pair `LocomotionFSM` (INIT→STAND→TROT, phase-boundary-aligned mode switches), roll/pitch→foot-height stabilizer, and the single home for every gait constant | yes |
| **`locomotion_controller.py`** | **New in 20260717**, 100 Hz — `/cmd_vel` + `/odom` attitude → `LocomotionFSM` → 12× `/model/anymal_c/joint/<J>/cmd_pos`; also owns fall detection (`FALL_DETECTED`, terminal). Replaces the retired `gait_animator.py` open-loop animation node | no |

`slam_toolbox` runs only to bootstrap the initial `map→base_link` pose; once driving,
`online_scan_match_localizer` owns the pose. **Nav2 / costmaps are not in the flood-fill
control loop.** The nav/localization chain is unchanged by the 20260717 legged work — it
still ends at `/cmd_vel_nav`; downstream of that, `/cmd_vel` now drives `locomotion_controller`
(12 torque-PID joints) instead of the retired `VelocityControl` kinematic thruster.

Design spec: `docs/superpowers/specs/2026-07-05-online-slam-maze-design.md` (localization layer);
`docs/superpowers/specs/2026-07-17-legged-locomotion-design.md` (physical walking layer);
`docs/superpowers/specs/2026-07-18-lidar-3d-swap-design.md` (3D lidar sensor layer).

## Legged locomotion layer (20260717)

The 20260716 iteration gave the ANYmal C a kinematic base (`VelocityControl` + an open-loop
`gait_animator.py` trot animation, purely cosmetic). 20260717 replaces that with **real
physics**: gravity on, all 54 collision bodies restored, `VelocityControl` deleted, and the
12 joints driven in torque-PID mode by the `legged/` trot controller above — the base now
moves only because the legs push against the ground.

- **Model**: `tugbot_description/models/anymal_c/model.sdf` — 12× `JointPositionController`
  at `p_gain=320 / d_gain=8 / cmd_max=80`; foot collisions get explicit `mu=1.0` friction;
  `<initial_position>` matches the neutral stand pose (`legged/params.STAND_POSE`); spawn
  `z=0.62`; both maze and test worlds run `max_step_size=0.001` (1 ms) physics.
- **Footprint**: the dynamic trot envelope grew from the old static-stance rectangle
  (±0.39 × ±0.32) to **±0.49 × ±0.37** — swing legs reach outside neutral stance, and the
  old "half-width doesn't change" assumption missed the turning-stride lateral component.
  `test_footprint_covers_gait_envelope` locks `footprint.py` to `legged/params.foot_envelope()`
  so the two can't drift apart again.
- **Verification ladder** (`bash tools/verify_legged_walk.sh`, 4 rungs: stand → forced
  in-place trot → straight walk → in-place turn, each gating the next):
  ```bash
  bash tools/smoke_stand.sh          # quick single-rung stand-up-and-settle smoke test
  bash tools/verify_legged_walk.sh   # full 4-rung ladder (stand/step/walk/turn)
  ```
  Straight-line tracking measured at **~77% of commanded speed** (effective ~0.23 m/s off a
  0.3 m/s command) — an open-loop-stride-vs-PD-lag steady-state shortfall, insensitive to PID
  gains; documented rather than chased further because the solver is position-closed-loop, so
  the only consequence is a slower run (see the spec addendum, item 8).
- **Maze run** (budget raised for walking speed): `bash tools/run_flood_fill_maze.sh 3600 true
  false online_slam` (`MAX_SECONDS=3600 HEADLESS=true USE_RVIZ=false POSE_SOURCE=online_slam`).
- **GUI/RViz runs on this machine REQUIRE NVIDIA PRIME offload** (the physical display runs on a weak AMD iGPU; without offload the gpu_lidar render starves and ICP degrades — see spec addendum item 15): `export DISPLAY=:1 __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia && bash tools/run_flood_fill_maze.sh 3600 false true online_slam`
- **With the front camera, headless runs need PRIME offload too, not just GUI/RViz** — the
  AMD iGPU copes with `gpu_lidar` alone but collapses under lidar+camera load, degrading ICP
  into oracle false-positives/livelock. `run_flood_fill_maze.sh` and `verify_front_camera.sh`
  now default `DISPLAY`/`__NV_PRIME_RENDER_OFFLOAD`/`__GLX_VENDOR_LIBRARY_NAME` internally
  (see the comment block at the top of each script); a manual `export` still works and wins.
- **Fall detection**: `locomotion_controller` watches `/odom` attitude/z; `|roll|` or `|pitch|`
  `> 0.6 rad`, or base `z < 0.25 m`, sustained 1 s → logs `FALL_DETECTED` and the run wrapper
  treats it as a **terminal failure result** (no auto-recovery, by design — see the spec's
  non-goals).

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
