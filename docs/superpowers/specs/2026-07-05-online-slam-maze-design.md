# Online self-built-map maze solving (no prior wall map) — design

**Date:** 2026-07-05
**New workspace:** `ros2_ws_tugbot_nav_20260705`
**Goal:** Solve the *same* 20260528 maze **without being given the wall map** — build the interior wall map online while localizing against it — and reach the exit reliably (controlled Gazebo batch target **≥ 14/16 EXIT_REACHED**), with the map-fed 20260614 solver (`pose_source=scan_match`) kept as the A/B **upper-bound baseline**.

---

## 1. Scope: what is known vs unknown

The next stage drops the **prior wall layout**, not the grid geometry.

**Known at start:**
- Grid geometry — 2 m cells, ~10×10 extent (`CELL_SIZE_M=2.0`, cell centres at `(2c, 2r)`).
- Entrance cell `(1,0)` and exit cell `(10,9)`.
- **Outer boundary walls** — the enclosed known-extent maze implies its perimeter edges are walls (the bootstrap anchor; user-confirmed 2026-07-05).

**Unknown (to be discovered online):**
- All **interior** wall edges.

**Success criteria:** on the fixed 20260528 maze, with **no wall map fed to the solver**, pure online localization-and-mapping:
- Controlled Gazebo batch (≥ 8 runs, ideally 16): **≥ 14/16 EXIT_REACHED**.
- True-footprint collision rate comparable to 20260614 (oracle replay).
- Offline: localization error vs ground-truth pose stays bounded through the interior.
- A/B against `pose_source=scan_match` (fed the full map) as the upper bound.

## 2. Core idea: a known grid collapses "SLAM" to edge-discovery + confirmed-wall localization

"Wall layout unknown" ≠ "map geometry unknown." Because the grid is known, **a sensed wall's centerline lies on a known cell-edge** — it is not a free, uncertain estimate. So the SLAM problem collapses to:

1. **Discover** which known cell-edges are walls (the flood-fill brain already does this — it senses each cell's walls from LIDAR and corroborates them across visits).
2. **Localize** each tick by scan-matching the live beams against the **confirmed** walls, whose geometry is exact.

Because the reference geometry is exact (grid-snapped), the ICP yields an **absolute, drift-free pose in the grid frame** — exactly as in 20260614 — the only difference being the reference set **grows** as exploration proceeds instead of being complete from t=0. No pose-graph, no loop-closure needed: there is no accumulating map uncertainty to close.

## 3. Architecture

### 3.1 Workspace seed (clean)
Create `ros2_ws_tugbot_nav_20260705` by copying `ros2_ws_tugbot_nav_20260614` and **pruning the historical cruft**, keeping the proven stack:
- **Drop:** `build/`, `install/`, `log/` (regenerated); `archive/` and `doc/archive/`; one-shot `phaseNN_*` diagnostic/visual-overlay scripts under `src/tugbot_maze/tugbot_maze/` and any `tools/archive/`.
- **Keep:** the 6 ROS packages' current code, `tools/` current runners, `config/`, `assets/`, and the **same active world** `tugbot_maze_world_20260528_clean_scaled2x.sdf` (so results are directly comparable to 20260614).
- Spec/plan docs stay at the repo-root `docs/superpowers/` (shared, dated).
- Acceptance for this step: `colcon build --symlink-install` green; the existing offline suite passes; a fed-map `pose_source=scan_match` run still reaches the exit (proves the seed is intact before adding anything).

### 3.2 The online localizer (the one genuinely new component)
A localizer that scan-matches against a **dynamic, growing confirmed-wall segment set** instead of the static `load_segments()` known map.

- **Confirmed-wall segment set** — built each tick from two sources:
  - the **known perimeter** (all boundary edges of the known-extent grid), seeded at t=0; and
  - the flood-fill brain's **committed** interior walls (only `committed`/`k_corroborate`-corroborated edges — a poor single sense never enters the reference, so it cannot poison localization).
  Each confirmed wall edge `(cell, dir)` maps deterministically to its centerline segment at known grid geometry (e.g. the N edge of cell `(c,r)` is the horizontal segment `(2c−1, 2r+1)–(2c+1, 2r+1)`); `wall_half_thickness=0.12` is handled by the existing ICP residual.
- **ICP reuse** — feed this segment set to the **existing** point-to-line ICP core (`scan_match_localizer`'s `correct(prior_pose, ranges, angle_min, angle_inc)` math): residual `r = n·(p−foot) − wall_half_thickness`, normal oriented toward the robot centre, same per-DOF **observability gate** (eigenvalues of the JᵀJ translational block + separate yaw gate). The only change is the segment **source** and that it is **dynamic** (passed per tick / mutable), not fixed at construction.
- **Under-constrained fallback** — when the confirmed set is too sparse to constrain a DOF (the observability gate fails for that DOF), that DOF **falls back to odom-propagation** (short-term accurate) — the same gate 20260614 already has. `slam_toolbox` keeps only its existing **cold-start** role (initial `map→base_link` bootstrap at the entrance).

### 3.3 Feedback loop
`/scan` + odom → **online localizer** (ICP vs confirmed walls, gated) → grid-absolute pose → **flood_fill/MazeMotion** (unchanged) → `/cmd_vel`. The brain both **consumes** the pose and **produces** the confirmed-wall set: pose → sense walls → corroborate/commit → segments added → localization strengthens → more accurate sensing. The loop is bounded by the brain's existing corroboration/commit gate.

### 3.4 Bootstrap sequence (the risk-concentrated phase)
Start at the known entrance with the **known perimeter** as reference (strong anchor near the boundary). Drive the first interior hops on perimeter + short-term-accurate odom; as interior walls get committed, the reference set grows and interior localization strengthens. The interior — away from the boundary, where only self-committed walls anchor the pose — is where this approach earns its keep and where the risk lives.

## 4. Carries over (unchanged) vs new

| | Item |
|---|---|
| **Unchanged (reuse)** | `maze_motion.py` (incl. reverse-to-center), `flood_fill_brain.py`, `cell_walls.py`, `footprint.py`, `pose_tracking.py`, `hop_controller.py`, `maze_sim.py` oracle, diagnostic tools (`batch_diagnose_floodfill.sh`, `replay_collision_oracle.py`, run scripts) |
| **New / changed** | `online_scan_match_localizer.py` (or a dynamic-segments mode on the existing localizer); a `confirmed_wall_segments(brain, perimeter)` helper; solver-node wiring to build the confirmed set each tick and feed the localizer; a new `pose_source` value (e.g. `online_slam`); run/batch scripts defaulting to the new mode |
| **A/B baseline** | `pose_source=scan_match` (fed the full known map) = the upper bound |

The known-map `scan_match_localizer` and its `load_segments()` remain in the tree as the baseline; the online localizer should **share** the ICP core (extract/reuse, don't fork) so both paths exercise the same, already-validated math.

## 5. Testing & validation

1. **ROS-free unit** — `confirmed_wall_segments`: a committed wall edge → the correct grid-snapped centerline segment; the perimeter seed produces the known boundary segments; an *un*committed (poorly-sensed) wall is **excluded**.
2. **Offline `maze_sim` end-to-end** — the sim raycasts/collides against the **true** maze, but the **solver is given only** grid geometry + perimeter + entrance/exit (NOT the interior map) and must build it online. Mirror `test_maze_motion_sim.py` with the online localizer. Assert: reaches the exit; true-footprint collisions bounded; **localization error vs ground-truth pose stays bounded** (esp. through the interior). Include a drift-stress parametrization as the existing suite does.
3. **Controlled Gazebo batch (no map fed)** — **16 runs (two 8-run batches, as in the 20260614 acceptance)**: `≥ 14/16 EXIT_REACHED`, collision rate (oracle replay) comparable to 20260614. A/B vs `pose_source=scan_match` (fed map) as the upper bound. Reuse `batch_diagnose_floodfill.sh` + `replay_collision_oracle.py`. Same honest gate discipline: abandon/iterate on regression, nothing banked to `main` until the batch passes.

## 6. Key risks & mitigations

- **Cold-start chicken-and-egg** (sparse interior references early): known-perimeter anchor + short-term-accurate odom + per-DOF observability fallback + committed-walls-only reference. **Fully offline-reproducible in `maze_sim`** — characterize before Gazebo.
- **Mis-sensed wall poisoning the reference**: only `committed` (corroborated) walls enter the segment set; the brain's existing commit gate is the guard.
- **Sustained open-interior travel with weak references**: the observability gate drops to odom for the under-constrained DOF; the flood-fill cell re-anchor + growing reference recover it. If the offline test shows unbounded interior drift, escalate (e.g. a minimal `slam_toolbox` fallback in the gate) — decided by data, not upfront.

## 7. Out of scope / non-goals

- Arbitrary/generated or non-tree mazes, unknown grid extent, unknown exit (a later phase — this phase is "only wall layout unknown" on the fixed maze).
- Dynamic obstacles / sim-to-real.
- Discovering the **perimeter** online (user chose known-perimeter anchor).
- Replacing `slam_toolbox`'s existing cold-start bootstrap or the MazeMotion/flood-fill layers.

## 8. Success criteria (restated)

`ros2_ws_tugbot_nav_20260705` seeded clean and building green; an online localizer that scan-matches against the brain's confirmed walls (perimeter-seeded, committed-only, observability-gated, odom fallback); offline `maze_sim` completes the maze **without the map fed** with bounded localization error and low collisions; controlled Gazebo **≥ 14/16 EXIT_REACHED** with collisions comparable to the fed-map baseline. Iterate/abandon honestly on regression; bank to `main` only after the Gazebo gate passes.

## 9. Implementation outcome & what actually worked (2026-07-05)

**Result: gate exceeded.** Controlled Gazebo, no interior map fed — **16/16 EXIT_REACHED** (100%, ~545–555 s, matching the fed-map baseline), **0/1769 = 0.000 % true-footprint collisions**, 0 stuck/unstick/escape events; offline `maze_sim` (map withheld) completes at drift=0.0 and drift=0.03, peak localization error ~0.40 m. Merged to `main`.

**Two things the spec's "committed-only + observability gate" underspecified — discovered during TDD (Tasks 6, 6b):**

1. **Committed-only reference LAGS exploration.** On first-pass discovery the cells ahead aren't committed yet, so there's no local reference and the pose falls to pure odom (re-locking only on revisit). Offline this gave a 1.10 m peak error under drift=0.03 (the maze still completed, carried by MazeMotion's local sensing). **Fix (`local_reference_cells`): the reference is `committed ∪ current-cell ∪ sensed neighbours`** — the current cell's freshly-sensed, grid-snapped walls are an *immediate* anchor during exploration. Grid-snapping keeps this safe under <1 m error (a sensed wall still snaps to the correct edge, so ICP pulls toward truth, not into the drift). This cut drift=0.03 peak error 1.10 → 0.40 m and is what made Gazebo solid (16/16 vs the "marginal" the committed-only version would have given).

2. **Three defensive gates were needed** (not just the single observability gate), each fixing a diagnosed ICP failure mode, in order: **sparse-interior** (skip ICP when the interior reference is empty — else iteration-0 matches interior-hitting beams to far perimeter walls → false minimum), **locality** (skip when no reference segment is within ~1 m of the prior — pure odom in genuinely unmapped territory), and **beam-premask** (∞-out beams whose prior-projected endpoints are far from any reference before iteration 0). The observability gate + odom fallback remain underneath.

**Known debt (non-blocking, for a future pass):** `_mask_far_beams` reaches into `ScanMatchLocalizer` internals (`_a`, `_e`, `_len2`) — fragile to an ICP refactor; a small accessor would be cleaner. The perimeter (`outer_segments()`) is 4 closed walls with no entrance/exit gaps — did not cause trouble in the gate, but is a candidate refinement if opening-region localization is ever stressed.
