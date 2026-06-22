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

`nav2_slam_floodfill_params.yaml` is a **verbatim copy** of `nav2_slam_params.yaml` with **two**
changes in the `collision_monitor` block — disabling the polygon AND its observation source, because
`collision_monitor` has **two independent gating paths**: the polygon (`approach` throttle) *and* a
polygon-independent **source loop** that issues a hard STOP ("invalid source") if an enabled source's
`getData` fails within `source_timeout` (here raised to 1.0 s, with `transform_tolerance: 0.5`, for
SLAM TF clears). Disabling only the polygon leaves the source-STOP active. So:

```yaml
    FootprintApproach:
      type: "polygon"
      action_type: "approach"
      footprint_topic: "/local_costmap/published_footprint"
      time_before_collision: 1.2
      simulation_time_step: 0.1
      min_points: 6
      visualize: False
      enabled: False          # was True — disables the approach-throttle zone
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "scan"
      min_height: 0.15
      max_height: 2.0
      enabled: False          # was True — disables the only source -> no polygon-independent source-timeout STOP
```

With **no active polygon and no active source**, `collision_monitor`'s `process()` loop has nothing
to act on (`robot_action = {DO_NOTHING, cmd_vel_in}`), so it republishes its input (`cmd_vel_smoothed`)
to its output (`/cmd_vel`) **unchanged** — it still bridges the topic (robot keeps receiving commands;
it remains the sole `/cmd_vel` publisher, so no conflict) and neither the approach-throttle nor the
source-timeout STOP can fire. (Copying the whole file matches the project's existing
`nav2_slam_phaseNN_params.yaml` convention; the intended diff is **exactly these two `enabled:` lines**
— the file is otherwise identical to the base, which the config-guard test enforces.)

**`velocity_smoother` is retained and NOT modified** (stays in scope: this is the collision_monitor
bypass). Note its transparency is **linear-only**: `max_accel/decel` linear `±2.5 m/s²` ≫ anything the
solver commands, but **angular `±0.8 rad/s²` is *below* the solver's `ang_decel=1.2`**, so turn ramps
are mildly rate-limited (the angular cap `0.5` equals the solver's `w_max`, and turns use
`turn_w_max=0.35`, so steady-state turns aren't throttled — only a small ramp latency). This is
accepted for now and explicitly watched in validation; if it proves to matter at junctions, a
follow-up raises the variant's angular accel.

### Component 2 — select the variant for flood-fill

In `tugbot_maze_explore.launch.py`, prepend a clause to the existing `nav2_params_file`
`PythonExpression` (line ~27) so flood-fill takes the variant first (the phase profiles are not used
with flood-fill, so ordering it first is safe):

```python
    floodfill_nav2_params = os.path.join(navigation_share, 'config', 'nav2_slam_floodfill_params.yaml')
    nav2_params_file = PythonExpression([
        "'", floodfill_nav2_params, "' if '", explorer_type, "' == 'flood_fill' else ",
        #                                                  ^ NOTE: ends with "else " — NO trailing quote.
        # The EXISTING chain below is appended VERBATIM; its first element is "'", which supplies the
        # opening quote for the next path. (A trailing quote here would produce `else ''<path>` -> a
        # launch-time eval SyntaxError that py_compile cannot catch.)
        "'", os.path.join(navigation_share, 'config', 'nav2_slam_phase26p_candidate_mppi_diagnostics_params.yaml'),
        "' if '", phase26p_candidate_mppi_diagnostics_profile, "' == 'true' else '",
        # ... rest of the existing chain unchanged, defaulting to LaunchConfiguration('params_file') ...
    ])
```
Equivalently: keep the existing list byte-for-byte and only PREPEND the four elements
`"'", floodfill_nav2_params, "' if '", explorer_type` plus the separator `"' == 'flood_fill' else "`
(trailing space, no quote) in front of the current line-28 `"'"`. The assembled expression is
`'<ff>' if '<explorer_type>' == 'flood_fill' else '<phase26p_candidate>' if ... else '<params_file>'`.

`explorer_type` is already a `LaunchConfiguration` in scope (line 18). No run-script change is needed:
`run_flood_fill_maze.sh` passes `explorer_type:=flood_fill`, so the variant is selected automatically;
every other explorer falls through to the existing chain and keeps `collision_monitor` active.

### Why this is correct and safe

- **Pass-through, not stop:** with **both** the polygon and the scan source disabled,
  `collision_monitor` still subscribes `cmd_vel_smoothed` and publishes `/cmd_vel`, forwarding the
  velocity unchanged — *neither* gating path (approach-throttle *nor* source-timeout STOP) can fire —
  so the robot keeps moving and `collision_monitor` remains the sole `/cmd_vel` publisher (no conflict).
- **Flood-fill-scoped:** the variant is selected only for `explorer_type == 'flood_fill'`; the base
  `nav2_slam_params.yaml` and all other explorers are untouched.
- **Reversible:** delete the variant + the one launch clause to restore the prior behavior.
- **`velocity_smoother` retained (linear-transparent):** linear accel limits (`±2.5 m/s²`) ≫ the
  solver's commands, so it doesn't throttle forward motion; angular is mildly rate-limited (see
  Component 1) — accepted for now, watched in validation.

## Testing & validation

This is an infrastructure change (YAML + launch), so there is no unit-level controller test; the
decisive test is the Gazebo run. Cheap guards first:

**Config guard — `test/test_floodfill_collision_monitor_bypass.py` (new, ROS-free):**
- Anchor paths via `pathlib.Path(__file__).resolve().parents[1] / 'config'` (NOT cwd-relative). Load
  both YAMLs with `yaml.safe_load`. The correct key path includes the `ros__parameters` level:
  `collision_monitor → ros__parameters → {FootprintApproach → enabled, scan → enabled}`.
- Assert the **base** `nav2_slam_params.yaml` has both `FootprintApproach.enabled == True` and
  `scan.enabled == True` (gate active for everyone else); the **variant** has both `== False`.
- **Whole-tree drift guard** (catches ANY accidental over-edit, not just listed blocks): `deepcopy`
  the variant, set its two changed leaves back to `True`, then assert the deepcopy equals the base dict
  exactly. (Unquoted `True`/`False` parse to Python `bool`.)

**Launch-expression eval guard — same test file (catches what `py_compile` cannot):** `py_compile`
returns OK on the launch because the `PythonExpression` list is valid *source*; the quoting defect
only fails at launch-time `eval()`. So assemble the expression string the way the launch does
(substitute `explorer_type='flood_fill'` then `'maze_dfs'`, and the profile flags `'false'`), `eval()`
it, and assert it returns the **floodfill** variant path for `flood_fill` and the **default**
`params_file` for `maze_dfs`. This guards the `''`-seam regression directly.

**Launch syntax:** `python3 -m py_compile .../tugbot_maze_explore.launch.py` → `COMPILE_OK` (necessary
but NOT sufficient — see the eval guard above).

**Build:** `colcon build --packages-select tugbot_navigation tugbot_bringup`; confirm the variant lands
in `install/tugbot_navigation/share/tugbot_navigation/config/`.

**Gazebo (the decisive run):**
`tools/run_flood_fill_maze.sh 1800 false true odom_locked true` (clean stray sims first via the `!`
pkill). **Before drawing any conclusion, confirm the bypass actually took effect** (else a persisting
wedge is unattributable):
- (i) the **variant was selected** — the run uses `nav2_slam_floodfill_params.yaml` (log/launch shows
  it), and `ros2 param get /collision_monitor FootprintApproach.enabled` (and `scan.enabled`) report
  `False` on the live node;
- (ii) **`collision_monitor` is pass-through** — no `"approach… away from collision"` / `"invalid
  source"` STOP lines gate the solver; spot-check `/cmd_vel` equals `/cmd_vel_smoothed` while driving;
- (iii) **`velocity_smoother` is not the gate** — its `/cmd_vel_smoothed` output is non-zero while the
  solver is commanding motion (rules out the retained `velocity_timeout: 1.0` zeroing on any >1 s
  publish gap).

Then the success signals, in order of importance:
1. **No real collisions** (the safety we removed). **Objective check:** post-process the recorded
   trajectory through the *same* offline oracle — `MazeSim.collides(x, y)` against `load_segments()`
   (`maze_sim.py`, robot 0.35 m + wall 0.12 m) — and flag any pose inside the margin. The run logs
   DIAG poses every 5 s in the solver map frame (same frame as `load_segments`, so it applies
   directly); for finer coverage, raise the pose-log rate or add a Gazebo contact-sensor topic. "Zero
   objective collisions" (not just "looked fine") is the pass/fail gate for this signal.
2. **Wedge-recovers drop sharply** vs the prior runs (24 / 27), especially at junctions.
3. **Deeper progress** than the prior best (dist 9.78 m); `EXIT_REACHED` is the stretch goal — and if
   reached, this is the first confirmed full autonomous interior solve.

**Interpretation (gated on confirmed pass-through above):**
- Pass-through confirmed + wedges collapse + zero objective collisions ⇒ `collision_monitor` was a
  dominant root; proceed to bank, then re-evaluate whether the centerline fix
  (`junction-centerline-precision`) is still needed. (The *decisive* evidence is this Gazebo
  with-vs-without-bypass A/B against the `main` baseline — stronger than the offline-vs-Gazebo
  contrast, which only *motivated* the hypothesis.)
- Pass-through confirmed + wedges persist ⇒ the root is (also) the solver's low-level control; keep
  this bypass (correct regardless) and return to the controller (deferred junction-aware "Approach B").
- Objective collisions appear ⇒ the solver's control does not fully transfer to Gazebo; do **not**
  ship the bypass as-is; the controller needs the precision work first.
- Pass-through NOT confirmed ⇒ fix the wiring (variant selection / param load / smoother zeroing)
  before interpreting anything.

## Revisions from the adversarial review (2026-06-21)

3 lenses with per-finding verification confirmed the core mechanism (disabled-polygon
`collision_monitor` republishes input unchanged; pipeline + CMake install + flood-fill scoping all
correct). Two **must-fix** issues and validation tightening, folded in:

1. **MUST-FIX — incomplete bypass.** `collision_monitor`'s source loop is polygon-independent: with the
   `scan` source still enabled and `source_timeout: 1.0`, a scan/TF stall fires a hard STOP regardless
   of the disabled polygon. **Fixed:** the variant now also sets `scan.enabled: False` (no active
   source ⇒ no source-timeout STOP) — true pass-through.
2. **MUST-FIX — launch-time `SyntaxError`.** The prepended `PythonExpression` clause's trailing quote +
   the existing chain's leading quote concatenate to `''<path>`, which `eval()` rejects at launch (and
   `py_compile` cannot catch). **Fixed:** the new clause ends with `else ` (no trailing quote); added a
   launch-expression **eval guard** test that `eval()`s the assembled string for both explorer types.
3. **Should — subjective safety check.** **Fixed:** added an objective collision oracle (reuse
   `MazeSim.collides` against `load_segments()` on the recorded trajectory) as the pass/fail gate.
4. **Should — unattributable negative result.** **Fixed:** "confirm the bypass took effect" (variant
   selected, params applied, `velocity_smoother` not zeroing) is now a precondition before any
   "controller is the root" conclusion.
5. **Nit — config-guard key path** omitted `ros__parameters`, and **velocity_smoother angular**
   transparency (`±0.8` < solver `1.2`). **Fixed:** corrected the key path + whole-tree diff; scoped the
   smoother "transparent" claim to linear and flagged the angular rate-limit for validation.

## Constraints

- Local only — **never push** to origin.
- Commit messages end with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
- Agent `pkill`/`kill` is hook-blocked; stray Gazebo sims are cleared by the user via the `!` prefix:
  `! pkill -9 -f "gz sim|ruby.*gz|parameter_bridge|flood_fill_solver|ros2 launch"`.
- Foreground `sleep` is blocked; use background runs.
- Shell cwd is not stable between Bash calls — use absolute paths / `git -C`.
- Avoid backticks inside `git commit -m` strings (bash command-substitution can drop a word).
