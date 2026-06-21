# Bypass Nav2 collision_monitor for the Flood-Fill Solver — Design

**Date:** 2026-06-21
**Workspace:** `ros2_ws_tugbot_nav_20260614`
**Branch:** `floodfill-collision-monitor-bypass` (off `main`; local only, **never pushed**)
**Status:** Approved (brainstorm + approach A + flood-fill scope). Ready for adversarial review, then plan.

## Problem

Confirmed root cause of the Gazebo junction wedging: **Nav2's `collision_monitor` sits in the
flood-fill solver's command path and throttles it.** Verified pipeline:

```
flood_fill_solver → /cmd_vel_nav
  → velocity_smoother   (nav2_bringup navigation_launch.py:215 remaps its input cmd_vel→cmd_vel_nav)
  → /cmd_vel_smoothed
  → collision_monitor   (nav2_slam_params.yaml: cmd_vel_in=cmd_vel_smoothed, cmd_vel_out=cmd_vel)
  → /cmd_vel → ros_gz bridge → Gazebo robot
```

`collision_monitor` runs a single `FootprintApproach` zone (`action_type: approach`,
`time_before_collision: 1.2 s`, scan-sourced) that **scales forward velocity toward zero** whenever
the footprint, projected 1.2 s along the commanded motion, predicts a collision — i.e. at doorways /
junctions / turns. In the last Gazebo run the robot sat pinned at the (3,1) 4-way **centered and
heading-aligned** (`pose=(6.00, 2.41)`, x = cell-centre, `yaw=1.57` N) while `collision_monitor`
logged `"approach… away from collision"` repeatedly — its forward command was being vetoed → 3 s
no-progress → `recover` (reverse) → retry → loop.

**The smoking gun:** the offline sim (which has **no** `collision_monitor`) solves the maze cleanly
with geometry identical to the SDF (0.24 m walls, 0.47 m keep-out); Gazebo wedges. The dominant
difference between "solves" and "wedges" is this gate. The flood-fill solver is a self-contained
controller with its own anti-collision (wall sensing, symmetric centering, a never-zero wedge-floor);
routing it through Nav2's conservative monitor is counterproductive — it throttles exactly when the
robot needs to creep through a tight doorway.

## Goal

Remove `collision_monitor` from the flood-fill solver's command path — **flood-fill-scoped, config +
launch only** — so the solver's velocity commands reach the robot ungated, and re-run Gazebo to test
whether the junction wedging collapses. Keep `velocity_smoother` (its accel-limiting is harmless to
the solver's ≤0.3 m/s commands). Leave the (deprecated) Nav2-based explorers untouched.

## Scope (decided): Approach A — disable the collision_monitor zone, flood-fill-scoped

**Files touched:**
- `src/tugbot_navigation/config/nav2_slam_floodfill_params.yaml` — **new**: a copy of
  `nav2_slam_params.yaml` with a single semantic change (the `collision_monitor` zone disabled).
- `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py` — extend the `nav2_params_file`
  `PythonExpression` to select the variant when `explorer_type == 'flood_fill'`.
- `src/tugbot_navigation/CMakeLists.txt` — only if the `config/` install rule does not already glob
  the directory (verify; most likely it installs `config/` wholesale and needs no change).
- `test/` — a small config-guard test (parse both YAMLs, assert the intended diff).

**Do NOT touch:** the solver code (`flood_fill_solver.py`, `maze_motion.py`, etc.), the
`nav2_slam_params.yaml` base (other explorers keep `collision_monitor`), `navigation_launch.py`
(upstream), the run script (`run_flood_fill_maze.sh` already passes `explorer_type:=flood_fill`), the
bridge, and `velocity_smoother` config.

## Design

### Component 1 — the flood-fill params variant

`nav2_slam_floodfill_params.yaml` is a **verbatim copy** of `nav2_slam_params.yaml` with exactly one
change, in the `collision_monitor` block:

```yaml
    FootprintApproach:
      type: "polygon"
      action_type: "approach"
      footprint_topic: "/local_costmap/published_footprint"
      time_before_collision: 1.2
      simulation_time_step: 0.1
      min_points: 6
      visualize: False
      enabled: False          # was True — disables the only zone -> collision_monitor passes through
```

With its only polygon disabled, `collision_monitor` has no active zone, so it republishes its input
(`cmd_vel_smoothed`) to its output (`/cmd_vel`) **unchanged** — it still bridges the topic (so the
robot keeps receiving commands; no publisher conflict and no `stop_pub_timeout` stop), it just no
longer throttles. (Copying the whole file matches the project's existing `nav2_slam_phaseNN_params.yaml`
convention; the intended diff is the single `enabled:` line — document that the file is otherwise
identical to the base so reviewers can diff it.)

### Component 2 — select the variant for flood-fill

In `tugbot_maze_explore.launch.py`, prepend a clause to the existing `nav2_params_file`
`PythonExpression` (line ~27) so flood-fill takes the variant first (the phase profiles are not used
with flood-fill, so ordering it first is safe):

```python
    floodfill_nav2_params = os.path.join(navigation_share, 'config', 'nav2_slam_floodfill_params.yaml')
    nav2_params_file = PythonExpression([
        "'", floodfill_nav2_params, "' if '", explorer_type, "' == 'flood_fill' else '",
        # ... existing profile-conditional chain, unchanged, defaulting to LaunchConfiguration('params_file') ...
    ])
```

`explorer_type` is already a `LaunchConfiguration` in scope (line 18). No run-script change is needed:
`run_flood_fill_maze.sh` passes `explorer_type:=flood_fill`, so the variant is selected automatically;
every other explorer falls through to the existing chain and keeps `collision_monitor` active.

### Why this is correct and safe

- **Pass-through, not stop:** a disabled-zone `collision_monitor` still subscribes `cmd_vel_smoothed`
  and publishes `/cmd_vel`, forwarding the velocity unchanged — so the robot keeps moving and there's
  no second `/cmd_vel` publisher to conflict with.
- **Flood-fill-scoped:** the variant is selected only for `explorer_type == 'flood_fill'`; the base
  `nav2_slam_params.yaml` and all other explorers are untouched.
- **Reversible:** delete the variant + the one launch clause to restore the prior behavior.
- **`velocity_smoother` retained:** accel limits (2.5 m/s²) far exceed anything the solver commands, so
  it does not throttle; it just smooths — and keeps the existing topology intact.

## Testing & validation

This is an infrastructure change (YAML + launch), so there is no unit-level controller test; the
decisive test is the Gazebo run. Cheap guards first:

**Config guard — `test/test_floodfill_collision_monitor_bypass.py` (new, ROS-free):**
- Parse both YAMLs with `yaml.safe_load`. Assert:
  - the **base** `nav2_slam_params.yaml` has `collision_monitor → FootprintApproach → enabled == True`
    (the gate is active for everyone else), and
  - the **variant** `nav2_slam_floodfill_params.yaml` has `... enabled == False` (gate disabled), and
  - the two files are otherwise structurally equal **except** that one key (guards against drift /
    accidental over-edit — e.g. assert the `velocity_smoother`, `controller_server`, costmap blocks
    are byte-identical, or that the only differing leaf is `FootprintApproach.enabled`).
- Run from source: `cd src/tugbot_navigation && python3 -m pytest test/... -v` (paths to the config
  files are relative to the package source, not the install tree).

**Launch syntax:** `python3 -m py_compile src/tugbot_bringup/launch/tugbot_maze_explore.launch.py` →
`COMPILE_OK`.

**Build:** `colcon build --packages-select tugbot_navigation tugbot_bringup` (installs the new config
into the share tree and the launch edit). Confirm the variant lands in
`install/tugbot_navigation/share/tugbot_navigation/config/`.

**Gazebo (the decisive run):**
`tools/run_flood_fill_maze.sh 1800 false true odom_locked true` (clean stray sims first via the `!`
pkill). Success signals, in order of importance:
1. **No real collisions** — `collision_monitor` is off, so the solver's own control must keep the body
   off the walls. Watch the run for actual wall contact (visually + any collision indicators). This is
   the safety we removed and the primary thing to confirm.
2. **`collision_monitor` no longer gates** — the `"approach… away from collision"` throttling no
   longer stalls the solver (the messages may still appear from the node but must not pin it).
3. **Wedge-recovers drop sharply** vs the prior runs (24 / 27), especially at junctions.
4. **Deeper progress** than the prior best (dist 9.78 m); `EXIT_REACHED` is the stretch goal — and if
   reached, this is the first confirmed full autonomous interior solve.

**Interpretation:**
- Wedges collapse + no collisions ⇒ `collision_monitor` was the dominant root; proceed to bank and
  then re-evaluate whether the centerline fix (`junction-centerline-precision`) is still needed.
- Wedges persist ⇒ the root is (also) the solver's low-level control; keep this bypass (it's correct
  regardless) and return to the controller (e.g. the deferred junction-aware "Approach B").
- Real collisions appear ⇒ the solver's control does not fully transfer to Gazebo; do **not** ship the
  bypass as-is; the controller needs the precision work first.

## Constraints

- Local only — **never push** to origin.
- Commit messages end with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
- Agent `pkill`/`kill` is hook-blocked; stray Gazebo sims are cleared by the user via the `!` prefix:
  `! pkill -9 -f "gz sim|ruby.*gz|parameter_bridge|flood_fill_solver|ros2 launch"`.
- Foreground `sleep` is blocked; use background runs.
- Shell cwd is not stable between Bash calls — use absolute paths / `git -C`.
- Avoid backticks inside `git commit -m` strings (bash command-substitution can drop a word).
