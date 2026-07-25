# tugbot-return: the original Tugbot rebuilt on the modern perception stack

**Date:** 2026-07-23
**Phase:** tugbot-return (follows ackermann-physics, merged `ce4acc4`)
**Workspace:** NEW `ros2_ws_tugbot_nav_20260727` (rsync clone of `ros2_ws_tugbot_nav_20260726`, excluding build/install/log/__pycache__/.pytest_cache)
**Branch:** `tugbot-return`

## Goal

Swap MR-Buggy3 back to the project's ORIGINAL Tugbot (differential-drive warehouse
robot, real physics) and measure whether a rigid diff-drive platform builds a more
stable accumulated 3D map than the Ackermann car. The experiment is an honest
comparison, not a bar to clear: the current baseline is the physical buggy's
**~450k voxels** (GUI 449,948 / headless 454,733, ≈18% of the 2.15–2.47M quadruped
baseline).

## User adjudications (recorded)

1. **Motion strategy = A, classic differential mode.** `ackermann_drive:=false`:
   in-place turns, rotate-to-face centering, and ×1 phase timeouts all return
   automatically. The Ackermann primitives (reverse-hop chains, back-and-arc,
   feasibility projection) stay in the code but are NOT engaged. Platform rigidity
   is the single experimental variable.
2. **Acceptance = A, honest comparison.** Hard gates are correctness-only
   (EXIT_REACHED, oracle 0.000%, both aliasing gates silent, zero ERROR lines).
   Voxel counts are reported against the 450k buggy baseline with no preset
   win/lose. A thicker tugbot map is a valid experimental conclusion. GUI visual
   verdict by the user decides keep/drop.

## Chosen approach (of three)

**Original physics + modern perception graft** (chosen): keep tugbot's stock
DiffDrive physics (gravity, real wheel contact, full collision set including the
rear gripper); graft on the dog-era perception/localization infrastructure
(16-beam 3D lidar, front camera, world-anchored odometry). Navigation chain
untouched.

Rejected: (2) kinematic tugbot via VelocityControl — maximally stable but defeats
the experiment (the kinematic route is already proven on the buggy) and is not
"the original tugbot"; (3) fully original nav chain using the stock 2D `scan_omni`
as the `/scan` source — would tear up the byte-frozen `/scan` projection contract
(the navigation lifeline) for zero benefit.

## Design

### 1. Model layer — `tugbot_description/models/tugbot/model.sdf`

The stock model survives intact in the workspace (811 lines). Changes:

**Keep (untouched):** DiffDrive plugin drive section (`/cmd_vel`,
`wheel_separation 0.514`, `wheel_radius 0.195`, left/right wheel joints), all
collision bodies (base box, gripper + hand + side slides, 4 wheels), IMU, visual
meshes, warning light.

**Remove (sensor pruning):**
- `scan_omni` (stock 2D lidar) — it publishes **directly on `/scan`**, colliding
  with the slice-projector-regenerated `/scan`. The projection chain is the ONLY
  legitimate `/scan` source (frozen contract since lidar-3d-swap). The whole
  link+sensor goes; keeping it muted would still burn GPU.
- `scan_front`, `scan_back`, `sensor_contact` (a gpu_lidar in the gripper),
  stock `color` camera (`/camera/image_raw`) — no consumers anywhere in the
  chain; pure GPU load. All removed.

**Graft (from the mr_buggy3/anymal_c spec, field-identical):**
- `lidar_3d` (gpu_lidar, 16 beams): `<lidar>` block byte-identical in spirit to
  the dog/buggy contract (same test as `test_lidar_centered_specs_match_dog`),
  topic `/lidar/points`, mounted **centered `x=0, y=0`** (SCAN_OFFSET_X=0
  contract preserved), mast height `z=0.55` — above the stock sensor pole top
  (0.5318) so the structure does not self-occlude the rings.
- `camera_front`: topic `/camera/front/image`, 720×540, pose `x>0` facing +x
  travel direction (same contract shape as the buggy's).

**Odometry red line:** add the world-anchored `OdometryPublisher` plugin
(`dimensions=3`, `odom_frame=odom`, `robot_base_frame=base_link`,
`odom_topic=/odom`, `tf_topic=/tf`, 30 Hz, NO xyz_offset — archived trap), and
**silence DiffDrive's own odometry**: the stock plugin currently publishes
`<odom_topic>/odom</odom_topic>` + `<tf_topic>/tf</tf_topic>` from wheel
integration, which DRIFTS under slip. Both aliasing gates (yaw gate `c52e951`,
position gate ackermann-physics 5b) snap to `/odom` on the premise that it is
drift-free; wheel-integrated odom on those topics breaks that premise. The
DiffDrive block keeps only its drive role — its odom/tf outputs are redirected
to inert topics (`<odom_topic>/diffdrive/odom_unused</odom_topic>`,
`<tf_topic>/diffdrive/tf_unused</tf_topic>`), pinned by contract test.

### 2. Motion layer — parameter flip only

`flood_fill_solver`'s declared parameter default flips: `ackermann_drive`
`True → False` (single source of truth in the solver's `declare_parameter`; no
launch-file override layered on top). Everything downstream is existing behavior gated on that flag:
`MazeMotion(ackermann=False)` restores in-place turns, `rotate_to_face=True`
centering, ×1 timeouts. **No Ackermann code is deleted** — `ackermann_maneuvers.py`
and its 16 tests stay green and dormant.

### 3. Footprint — honest restore of the original rectangle

`footprint.py` constants return to the tugbot's real asymmetric body (recovered
verbatim from the 20260705/20260712 workspaces):

```python
FOOT_X_FRONT = 0.262     # body front (base mesh)
FOOT_X_REAR  = -0.468    # rear gripper hand
FOOT_HALF_W  = 0.292     # half-width (base mesh / wheels at 0.257)
SCAN_OFFSET_X = 0.0      # 3D lidar centered — NOT the historic -0.1855 (that was 2D scan_omni)
```

SCAN_OFFSET_X stays 0 because the 3D lidar mounts centered; the historic −0.1855
belonged to the removed 2D sensor. The rear-gripper overhang's historical failure
mode ((3,9) rear graze) is already covered by the reverse-to-center fix
(2026-07-03, on main ever since).

### 4. World / bridge / launch

- World `tugbot_maze_world_20260528_clean_scaled2x.sdf`: spawn
  `model://tugbot` name `tugbot` pose `-11.011 -9.025 0.000` (historic original
  pose; wheels rest on ground at z=0). `model://mr_buggy3` no longer spawned
  (assets stay in repo, same convention as the dog).
- `tugbot_bridge.yaml`: `/cmd_vel`, `/odom`, `/tf`, `/lidar/points/points`
  (split-topic trap), `/camera/front/image`, IMU if present — buggy-specific
  leftovers pruned.
- `tugbot_gazebo.launch.py` static TFs point at `tugbot/base_link/lidar_3d` and
  `tugbot/base_link/camera_front` sensor frames (gz scoped-name convention —
  exact frame strings verified live at smoke, they follow
  `<model>/<link>/<sensor>`).

### 5. Contract tests

- **New `test_tugbot_return_contract.py`:** model named tugbot, `<static>false`,
  gravity on, DiffDrive drive block intact (separation/radius pinned) with odom
  redirected to inert topics, OdometryPublisher contract (the 8-assert block from
  the buggy test), NO sensor publishing on `/scan` inside the model, `scan_omni`/
  `scan_front`/`scan_back`/`sensor_contact`/stock camera absent, `lidar_3d`
  centered at x=y=0 z=0.55 with `<lidar>` field-identical to the dog's,
  `camera_front` forward + 720×540, world spawns tugbot not buggy at the exact
  pose, bridge has no buggy-only topics, static TFs point at tugbot frames.
- **Re-freeze `test_mr_buggy3_contract.py`:** the two world/launch-coupled tests
  (`test_world_spawns_buggy_not_dog`, `test_static_tfs_point_at_buggy_frames`)
  flip to assert the buggy is NOT spawned / TFs do NOT point at buggy frames —
  same convention as the dog contract re-freeze at buggy-swap. The pure-SDF
  assertions stay as-is (the asset remains in-repo).
- Footprint-consuming tests: any test hard-coding the ±0.15 buggy body adapts to
  the restored constants; adjudicate case-by-case in the plan, no blind edits.

### 6. Acceptance gates (honest-comparison protocol)

1. Suite green at the frozen baseline (4 known frozen legacy failures; every
   delta consciously adjudicated). Authoritative counts ONLY from main-session
   workspace-root runs.
2. Smoke: gz plugin semantics recorded (DiffDrive responds on `/cmd_vel`,
   `/odom` is world-anchored — teleport-check: believed pose tracks a commanded
   arc with zero drift against `/gt/dynamic_pose` EVAL-ONLY readout), `/scan`
   has exactly one publisher, lidar frame string captured live.
3. Headless ×2: EXIT_REACHED, oracle 0.000% (true-footprint replay with restored
   rectangle), POS_GATE/yaw-gate silent or single-recovery, zero ERROR, final
   `CLOUDMAP voxels=` recorded vs the 450k baseline + saturation-paradox check
   (late-run growth must decay; sustained late growth = ghost signal, first-class).
4. GUI (user-triggered only, never auto-launched): RViz 3D map visual verdict by
   the user — final keep/drop.

### 7. Risks (pre-registered)

- **Pivot-turn fan smear:** during in-place rotation a small localization yaw
  error paints distant walls into an arc-shaped thickening. This is exactly the
  quantity under test — if it shows up, it is a finding, not a bug to be tuned
  away silently (honest-comparison adjudication).
- **DiffDrive acceleration limits** (`max_acceleration 0.5`) shape stop/start
  ramps; classic-mode timing tolerances may need the historic (×1) values
  verified rather than assumed.
- **`/scan` double-publisher** is the highest-severity integration trap — hence
  the removal (not muting) of `scan_omni` and the one-publisher smoke assert.
- **Gripper rear overhang** returns the largest footprint of the project;
  reverse-to-center already on main, oracle replay validates with the restored
  rectangle.

### Non-goals

- No changes to localizers, projector chain, solver logic, or the cloud
  accumulator (all byte-frozen; the accumulator keeps the stamp-based TF fix).
- No tuning loop to force the voxel count under the buggy baseline (honest
  comparison per adjudication 2).
- No S-curve or new motion primitives (backlog untouched).

## Addendum — execution record (2026-07-23, tasks 0-6)

### 1. Suite accounting

- Task 0 baseline (fresh clone, main-session verified): **4 failed / 540
  passed / 0 xfailed, 544 collected**. Frozen quartet:
  `test_maze_asset_and_config_are_present` +
  `test_maze_launch_files_exist_and_use_maze_world` +
  `test_maze_world_scaffold_contract` +
  `test_maze_corridors_are_at_least_twice_tugbot_turning_diameter`.
  Plan fix en route: the authoritative suite spans BOTH
  `src/tugbot_maze/test/` AND `src/tugbot_bringup/test/` — tugbot_maze alone
  under-collects by the 5 bringup-contract tests (236e33a).
- After the footprint restore (Task 3, main-session verified): **10 failed /
  544 passed, 554 collected** (+10 tests from the tugbot contract file). The
  6 new frozen reds are all wall_follow footprint-coupled historical records
  (see §3). Unchanged through Task 4.

### 2. Model/world surgery highlights (Tasks 1-2)

- Contract file `test_tugbot_return_contract.py` (10 tests) pins the whole
  phase; the buggy contract's two world/launch tests re-frozen.
- Quality review caught a live trap before it fired: the launch static TFs
  still carried the BUGGY mount translations (lidar z 0.30, camera
  0.12/0.15) — a name-only frame rename would have projected the cloud
  ~0.25 m low with all assertions green. TFs now carry names AND
  translations (lidar z 0.55, camera 0.20/0.40), and the contract test pins
  the translations (96af4a6, f4e4f2c).

### 3. Footprint restore + era-4 adjudication (Task 3)

- Constants restored verbatim from the pre-dog workspaces:
  front 0.262 / rear −0.468 (gripper hand) / half-width 0.292;
  SCAN_OFFSET_X stays 0 (the historic −0.1855 belonged to the removed 2D
  scan_omni). Nine maze_sim geometry fixtures recomputed (bounding circle
  0.5516 + 0.12 = 0.6716), discriminating intents preserved.
- Wall-follow historical records, observed and frozen (not tuned):
  - exterior cheat era 4: corridor still passable at 0.292 half-width —
    steps=1721, outside=1611, cheat stays OPEN (same side as the buggy era).
  - sealed interior solves + 3 perturbations + faster-hand lock: the
    follower wedges in the interior (steps=None, outside=0, closest
    26.08/22.38 m) — 6 reds frozen by name with in-file breadcrumbs.
    Production navigation is flood_fill, whose offline suites stayed green.

### 4. Smoke findings (Task 5, PASS=6)

- **Empty-world default trap:** the gazebo-only launch defaults `world_sdf`
  to `tugbot_empty_world.sdf` (pre-dog relic that still includes
  model://tugbot, so model checks pass while the gt bridge — pinned to the
  maze world name — starves with zero publishers). The full explore stack
  always passed the maze world explicitly, which is why no prior phase saw
  it. Smoke now pins the world and asserts `maze-world-loaded`.
- **Bridge drops gz names:** ros_gz_bridge's Pose_V→TFMessage conversion
  leaves every frame_id/child_frame_id EMPTY on /gt/dynamic_pose; consumers
  must use the transforms[0] index convention (the solver's `_gt_cb` already
  documents the same trap). Anchor check keys transforms[0].
- **Zombie projector:** `scan_slice_projector` survives the parent launch
  SIGINT (11 zombies had accumulated across smoke iterations); the smoke's
  kill list now includes it (production run scripts always had it).
- Anchor check (honest grading: origin-invariant displacement comparison —
  drift-free tracking + frame orientation + liveness): err=0.0000,
  travel=1.381 m; bonus empirical record `odom0 == gt0 == (-11.011,
  -9.025)` — the OdometryPublisher reports world coordinates, so origin
  anchoring has a measured data point too. Live lidar frame recorded:
  `tugbot/base_link/lidar_3d`.
- Process archaeology: the pkill-self-match heredoc trap struck again during
  diagnosis (write and execute must be separate tool calls); its orphaned
  script instance explains a mid-diagnostic launch kill.

### 5. Headless acceptance ×2 (Task 6, main session)

| run | result | POS/YAW gates | ERROR | oracle | final voxels | saturation (last/mid) | stale_dropped |
|---|---|---|---|---|---|---|---|
| 113158 | EXIT_REACHED (~537 s) | 0 / 0 | 0 | 0.000% | **420,579** | 0.66 (decaying) | 0 |
| 114147 | EXIT_REACHED (~537 s) | 0 / 0 | 0 | 0.000% | **428,676** | 0.62 (decaying) | 0 |

- Oracle: 2 runs, 220 samples, 0 collisions (live_rate 0.000%); the Task 3
  review's rear-asymmetry watch item (gripper 0.468 vs buggy-era hardcoded
  rear margins) recorded ZERO rear-face contacts across both runs.
- Honest three-platform comparison (accumulated 3D map, final voxels):
  dog (trot) 2.15–2.47 M = 100% · physical buggy ~450 k ≈ 18% ·
  **tugbot (diff, in-place turns) 420–429 k ≈ 17%** — the cleanest maps of
  the project to date, WITHOUT the pivot-turn fan smear the risk register
  pre-filed (both aliasing gates silent all run, so pivot yaw error never
  materialized on this drift-free anchored-odom platform).

GUI acceptance: pending (user-triggered only).
