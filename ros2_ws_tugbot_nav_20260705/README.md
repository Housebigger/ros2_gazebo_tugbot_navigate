# Autonomous Maze Navigation — `ros2_ws_tugbot_nav_20260614`

ROS 2 (Jazzy) + Gazebo Harmonic stack that drives the Tugbot from the maze entrance
(map `(0, 0)`, cell `(1,0)`) to the exit (map `(21.07, 18.08)`, cell `(10,9)`) of a
~10×10-cell *perfect maze* (a tree: 2 m cells, ~1.76 m corridors). Several explorer
strategies are selectable via the `explorer_type` launch argument; the **reliable,
genuinely autonomous** one is the **micromouse-style cell-grid flood-fill solver with
scan-match localization** (`explorer_type:=flood_fill`, `pose_source:=scan_match`).

## Headline result

**Reliable autonomous completion: 16/16 `EXIT_REACHED`** across two independent 8-run
Gazebo batches, ~555–565 s/run (matching the *pre-computed* GCN baseline's time, but via
**autonomous discovery** — the robot is not told the route). The breakthrough was
**localization, not search**: matching the live 360° LIDAR against the known wall map for
an absolute pose each tick (scan-match) eliminated the wheel-odometry desync that had
capped every prior autonomous attempt. Offline true-footprint collision rate fell from the
honest ~25–61 % (under drifting odom) to ~0.33 %, and the lone (3,9) residual graze was then
**eliminated to 0 %** by a reverse-to-center fix (2026-07-03, below).

## Explorer modes

| `explorer_type` (+ args) | Strategy | Status |
|---|---|---|
| **`flood_fill`** + `pose_source:=scan_match` (default) | Cell-grid flood-fill (micromouse) routing + `MazeMotion` corridor FSM, localized by ICP scan-match against the known wall map | ✅ **Reliable autonomous 通关 — 16/16** |
| `flood_fill` + `pose_source:=odom_locked` / `slam` | Same solver on wheel-odometry / live-SLAM pose | ⚠️ A/B only — desyncs in the open interior (0/8); the reason scan-match was built |
| `guided_corridor_mode:=true` (GCN, in `…_20260522`) | Follow a pre-computed BFS corridor sequence | ✅ Reliable, but *guided* (not autonomous discovery) — the prior reliable route |
| `wall_follower` | Reactive right/left-hand wall-following | ❌ Its earlier "5/5" was a **cheat** (it followed the *outside* of the perimeter); sealing the entrance exposed that the stateless follower loops in the interior. Superseded by flood-fill. |
| `tremaux` / `frontier` / `maze_dfs` | Junction-graph Trémaux DFS / SLAM-frontier / greedy DFS | ⚠️ Earlier autonomous attempts — explore untrapped but **plateau ~8.7–11 m** from the exit (Nav2 point-goals wedge in the 1.76 m corridors) |

## How to run

```bash
cd ros2_ws_tugbot_nav_20260614
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash

# Reliable autonomous solver — flood-fill + scan-match (scan_match is the DEFAULT pose source):
export DISPLAY=:1   # if running headed under a virtual display
bash tools/run_flood_fill_maze.sh 1800 false true     # MAX_SECONDS HEADLESS USE_RVIZ  (pose_source defaults to scan_match)
#   Artifacts: log/flood_fill_run_<stamp>/ (result.txt, launch.log; DIAG pose/dcell/dist_to_exit + MATCH localizer health)

# Controlled reliability + failure-mode batch (N identical runs -> auto-classified distribution):
bash tools/batch_diagnose_floodfill.sh 8 1200 false true   # -> log/batch_diag_<stamp>/report.txt

# Offline suite (pure Python, no ROS, fast): localizer + motion-in-sim + brain + sensing
cd src/tugbot_maze && python3 -m pytest \
  test/test_scan_match_localizer.py test/test_pose_tracking.py test/test_maze_motion_sim.py \
  test/test_maze_sim.py test/test_flood_fill_brain.py test/test_cell_walls.py \
  test/test_hop_controller.py test/test_wall_localize.py -q
```

Launch directly:
`ros2 launch tugbot_bringup tugbot_maze_explore.launch.py explorer_type:=flood_fill pose_source:=scan_match`.
`pose_source` defaults to `scan_match`; `odom_locked` and `slam` remain selectable for A/B.

## The localization breakthrough (why this finally completes the maze)

A controlled diagnostic (`tools/batch_diagnose_floodfill.sh`) pinned the real failure: with
wheel-odometry pose (`odom_locked`), **75 % of runs desynced** — the robot believed it was
centered while it had drifted, so it mis-sensed walls and mis-routed (0/8 completions, 62 %
DESYNC, 100 % RECONCILE). The maze is *known* (a fixed perfect maze), so the fix is to use
that map for **localization** while keeping **route discovery autonomous**:

> `scan_match_localizer.py` — each tick, take the live `/scan`, seed with the odom-propagated
> prior, and run a **point-to-line ICP** that aligns the beam endpoints to the known wall-segment
> centerlines (true asymmetric footprint, observability-gated per axis). Output: an absolute
> map-frame pose, drift-free. This is "known-map localization, unknown-route exploration" — not a
> pre-computed path.

With an accurate pose, the cell tracker stays synced, sensing is correct, and the autonomy the
solver already had simply works: **0 % desync, 0 RECONCILE, 16/16 to the exit.**

## Architecture (`src/tugbot_maze/`)

A thin ROS node (`flood_fill_solver.py`) owns only ROS plumbing; **all** control is a ROS-free
core, validated offline by a ROS-free maze simulator *before* any Gazebo run:

| Unit | Role | ROS-free? |
|---|---|---|
| `flood_fill_brain.py` | Cell-grid routing — flood-fill / Trémaux-capped BFS `next_cell`; dead-end & reachability logic | yes |
| `maze_motion.py` | Motion FSM (`center → turn → drive`, + backout/recover/unstick); corridor pure-pursuit + profiled rotate-in-place + wall-referenced re-anchor | yes |
| `cell_walls.py` | Per-cell wall sensing — projects LIDAR beams onto cardinals, median over a window | yes |
| **`scan_match_localizer.py`** | **Point-to-line ICP scan-to-known-map → absolute pose** (the localization fix) | yes |
| `pose_tracking.py` | SE(2) helpers (`compose_2d`, `inverse_2d`, `odom_prior`) for the per-tick odom prior | yes |
| `maze_sim.py` | **Test-only** raycaster + unicycle integrator over the real `20260528` wall segments, with the **true asymmetric-footprint collision oracle** (rear gripper at −0.468 m; the honest collision truth) | yes |
| `footprint.py` | Robot geometry constants (`FOOT_X_FRONT/REAR`, `FOOT_HALF_W`, LIDAR `SCAN_OFFSET_X=-0.1855`) | yes |
| `map_memory.py` / `wall_localize.py` / `hop_controller.py` | Map-integrity guard / perimeter & cell-center offsets / low-level motion commands | yes |
| `flood_fill_solver.py` | Thin node — `/scan` + TF → `_lookup_pose` (`scan_match`/`odom_locked`/`slam`) → `MazeMotion` → `/cmd_vel_nav` at 10 Hz; DIAG + MATCH logging | no |

`slam_toolbox` runs only to bootstrap the initial `map→base_link` pose; once driving, scan-match
owns the pose. **Nav2 / costmaps are not in the flood-fill control loop.** Spec & plan:
`docs/superpowers/specs/2026-06-28-scan-match-localization-design.md`,
`docs/superpowers/plans/2026-06-28-scan-match-localization.md`. **Success-experience
writeup** (the journey + transferable lessons): `doc/doc_experience/maze_autonomy_success_experience.md`.

## Diagnostic tooling

- `tools/diagnose_floodfill_runs.py` — classifies run artifacts into failure modes
  (final-approach / desync / sense-heavy / early-stuck) from the logs; reports the distribution.
- `tools/batch_diagnose_floodfill.sh` — runs N identical Gazebo runs and auto-classifies them, so
  decisions rest on the run *distribution*, not single-run anecdotes. This is how the
  odom→scan-match win (and the abandoned guard regression below) were measured honestly.

## The road here (why earlier attempts didn't complete)

The autonomous-completion problem was open for the whole project; each approach is kept selectable
as a baseline:

1. **DFS / Trémaux / frontier** — clean ROS-free decision logic, but **plateau ~8.7–11 m** from the
   NE-corner exit: Nav2 point-goals **wedge** in the 1.76 m corridors (wedge→exclude→relocate cycles
   cap coverage). The gap was the narrow-corridor *navigation* layer, not the search.
2. **Reactive wall-following** — its "5/5 通关" was a **cheat**: entrance and exit are both on the
   outer boundary, so it followed the *outside* of the perimeter. Sealing the entrance forced an
   interior solve, and the *stateless* follower loops in a sub-region. Abandoned.
3. **Cell-grid flood-fill** — gave the two missing properties: **memory** (a cell graph, no aliasing)
   and **reliable corridor locomotion** (`MazeMotion`). It escaped the historical traps and got far,
   but plateaued on the **open-region two-axis localization wall**: in the interior, away from
   reference walls and far from the entrance, odometry drift + off-center mis-sensing produced phantom
   passages. That wall is exactly what scan-match localization broke.

## The (3,9) residual — resolved (2026-07-03)

The lone ~0.33 % true-footprint graze at **cell (3,9)** is now **eliminated (0 %)**. An oracle
replay of the DIAG poses (`tools/replay_collision_oracle.py`) pinned the mechanism: the robot
arrives ~0.4 m past centre along its **north travel axis**, and `centering_command` corrected that
overshoot by **rotating ~180° in place** to face south — and rotating while off-centre swept the
asymmetric rear gripper (corner radius ≈0.55 m) through the north perimeter wall face. The endpoints
are clear; only the mid-rotation arc grazed (hence the ~0.33 % transient).

**Fix (`hop_controller.centering_command`):** a diff-drive robot cannot strafe, **but it can reverse**.
When the axis to correct is anti-parallel to the heading (the cell centre is *behind* — an
along-travel overshoot), it now **reverse-translates** along the heading to null the offset, instead
of rotating 180°. Reverse retreats into the just-traversed, clear corridor (inherently safe, no
sweep); perpendicular lateral centering is unchanged. **Controlled Gazebo validation (16 post-fix
runs, 2 batches): 16/16 `EXIT_REACHED`, oracle collision rate 0/1788 = 0.000 %** (baseline 6/1808 =
0.332 %, all (3,9)); offline the fix also cleared 3 formerly-`xfail` drift=0.05 rear-corner grazes.
Spec/plan: `docs/superpowers/{specs,plans}/2026-07-03-reverse-to-center-boundary-overshoot*.md`.

*Prior attempt, for the record:* a **rotation-sweep clearance guard** (branch
`rotation-sweep-clearance-guard`, parked) had tried to *gate rotations* and made it **worse**
(0.33 %→4.43 %) — because the culprit rotation was the centering one, and gating it pinned the robot
off-centre. That failure is what pointed the fix at *centering* (specifically reversing the overshoot),
not rotation-gating.
