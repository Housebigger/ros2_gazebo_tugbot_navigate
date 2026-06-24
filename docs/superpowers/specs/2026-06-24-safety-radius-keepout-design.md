# Safety-Radius Reactive Keep-Out — Design

**Status:** approved. Branch `no-progress-watchdog`. Motivated by the user's RViz/Gazebo observation: the Tugbot repeatedly collides with walls, which corrupts its wheel-odometry (→ the 2-cell `dcell` desync seen at (7,4)) and LIDAR cell-sensing (→ garbled map). The objective oracle confirms 36–53% of moving poses were inside the wall margin.

## 1. Root & history

We deliberately **bypassed Nav2 `collision_monitor`** (it vetoed the solver at doorways → wedging). That removed the only collision-avoidance layer → the robot scrapes/hits walls → in Gazebo the contact perturbs the pose (wheel slip / shove) so `odom_locked` localization diverges (the desync) and LIDAR-while-jammed distorts sensing (the garbled map). **Preventing contact should remove the desync + map corruption at the source.** Constraint: must NOT re-introduce the Nav2 veto-wedge — keep the robot off walls *while still passing the ~2 m doorways*.

## 2. Geometry
Robot radius 0.35 m + wall half-thickness 0.12 m → LIDAR clearance at contact ≈ 0.47 m; centered robot sees side walls at ~0.88 m. Current `wedge_stop_m=0.40`/`wedge_slow_m=0.50` sit *below* contact (≈touching) — they react too late. The safety radius must react earlier.

## 3. Design: solver-internal reactive keep-out (safety bubble)

**Detection (`_drive`):** `keepout_clear` = nearest obstacle in ANY direction = `min(near_side_perp, full_LIDAR_scan_min)`, filtering spurious near-zero/`inf` returns. The full-scan min catches diagonal/corner returns the cardinal `perp` misses (corners are where it jams). Extract as a pure helper `_keepout_clearance(near, ranges)` for testability; pass it to the corridor follower as `near_wall_m`.

**`safety_radius = 0.60 m`** (~0.13 m margin before contact).

**In `corridor_drive_command`, when `near_wall_m < safety_radius`** (collision avoidance > heading stability):
- **Override the `max_cross_steer` cap** with `keepout_max_cross_steer = 0.8 rad`: `cap = keepout_max_cross_steer if (near_wall_m is not None and near_wall_m < safety_radius) else max_cross_steer`; clamp `cross_steer` to `±cap`. Lets the centering steer away from the near wall with full authority.
- **Slow proportionally**: raise `wedge_slow_m` 0.50 → 0.60 (= safety_radius) so braking begins at the safety radius and eases to the never-zero creep floor (`wedge_v_floor=0.10`). **Always a creep, never a hard stop** (chosen — lowest wedge risk; the robot can always maneuver off the wall).

**When `near_wall_m ≥ safety_radius`:** unchanged — normal capped following (heading stable). At open junctions there are no nearby walls → keep-out does not trigger → the cross-track cap still holds heading there. The override fires only in corridors when the robot drifts toward a wall.

## 4. Plumbing
New kwargs `safety_radius=0.60`, `keepout_max_cross_steer=0.8` on `corridor_drive_command` and `corridor_follow_command` (pass-through). `wedge_slow_m` default 0.50→0.60 in both functions and `MazeMotion.__init__`. `MazeMotion.__init__` adds `self.safety_radius`, `self.keepout_max_cross_steer`. `_drive` computes `keepout_clear` and passes it + the two new params to `corridor_follow_command`.

## 5. Why safe / why it should work
- Steering AWAY (not stopping) + a never-zero creep avoids the Nav2-style veto-wedge.
- The override is gated on REAL LIDAR clearance (not the desync-prone `cross_track`/grid), so it triggers on genuine proximity.
- Triggers only near walls (corridors), so junction heading stability (the cap) is preserved.
- Prevents contact → no odom/LIDAR corruption → the desync + garbled map don't arise.

## 6. Tests
- **Cap override** (`test/test_hop_controller.py`): `corridor_drive_command` with `near_wall_m < safety_radius` + large `cross_track` steers to the emergency cap (0.8); with `near_wall_m ≥ safety_radius`, to the normal cap (0.35). (Use `kp_ang=1.0, w_max=2.0` so `w` doesn't saturate and the two caps are distinguishable.)
- **Keep-out clearance** (`test/test_maze_motion.py`): `_keepout_clearance(near, ranges)` returns the full-scan min (spurious `<0.05`/`inf` filtered), or `near` when smaller / no finite returns.
- **Offline regression** green (the clean solve drives centered/collision-free, so the keep-out must not disturb it; if a wedge-slow test asserts the old 0.50, update it — tuning).

## 7. Validation
Diagnostic instrumentation stays IN. Gazebo watch: objective collision rate drops sharply (the real metric for this fix); no 2-cell `dcell` desync; clean RViz map; ideally deeper than ~6.9 m. Revert `2fe9e39` before any merge.
