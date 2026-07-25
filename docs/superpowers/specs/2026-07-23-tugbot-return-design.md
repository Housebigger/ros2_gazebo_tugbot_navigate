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
