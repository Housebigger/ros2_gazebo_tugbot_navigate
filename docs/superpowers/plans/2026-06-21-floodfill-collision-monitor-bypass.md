# Bypass Nav2 collision_monitor for the Flood-Fill Solver — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove Nav2's `collision_monitor` gating from the flood-fill solver's command path (flood-fill-scoped, config + launch only), so the solver's velocity commands reach the robot ungated — to test whether the junction wedging collapses.

**Architecture:** A flood-fill Nav2 params variant with `collision_monitor`'s polygon AND scan source disabled (true pass-through), selected by a prepended clause in the launch's `nav2_params_file` `PythonExpression`. Two cheap guard tests (config drift + launch-expression quoting); the decisive test is the Gazebo run.

**Tech Stack:** ROS 2 Jazzy (Nav2 collision_monitor / velocity_smoother), YAML, Python launch, pytest. Pairs with spec `docs/superpowers/specs/2026-06-21-floodfill-collision-monitor-bypass-design.md` (read its *Revisions* section).

**Conventions for every task:**
- `REPO = /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`
- `SRC  = $REPO/ros2_ws_tugbot_nav_20260614/src`  (so `$SRC/tugbot_navigation/config`, `$SRC/tugbot_bringup/launch`, `$SRC/tugbot_maze/test`)
- Run tests: `cd "$SRC/tugbot_maze" && PYTHONPATH="$SRC/tugbot_maze" python3 -m pytest test/test_floodfill_collision_monitor_bypass.py -v` (the test is ROS-free and locates config/launch via `Path(__file__).resolve().parents[2]`).
- Work on branch `floodfill-collision-monitor-bypass` (already created off `main`; **never push**).
- Commit messages end with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`. **No backticks inside `git commit -m`.** Stage only each task's files via `git -C "$REPO" add <path>` (NEVER `git add -A`).

---

## File Structure

| File | Responsibility | Task |
|------|----------------|------|
| `src/tugbot_navigation/config/nav2_slam_floodfill_params.yaml` | **new** — copy of base with collision_monitor polygon + scan source disabled | 1 |
| `src/tugbot_maze/test/test_floodfill_collision_monitor_bypass.py` | **new** — config-guard (Task 1) + launch-expr eval-guard (Task 2) | 1, 2 |
| `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py` | prepend the `flood_fill` clause to the `nav2_params_file` `PythonExpression` | 2 |

(`CMakeLists.txt` needs **no** change — review confirmed `install(DIRECTORY config ...)` globs the whole dir, so the new YAML is installed on build.)

---

## Task 1: Flood-fill params variant + config-guard test

**Files:**
- Create: `src/tugbot_navigation/config/nav2_slam_floodfill_params.yaml`
- Create: `src/tugbot_maze/test/test_floodfill_collision_monitor_bypass.py`

- [ ] **Step 1: Write the failing config-guard test** — create `test/test_floodfill_collision_monitor_bypass.py`:

```python
import copy
from pathlib import Path

import yaml

_CONFIG = Path(__file__).resolve().parents[2] / 'tugbot_navigation' / 'config'
_BASE = _CONFIG / 'nav2_slam_params.yaml'
_VARIANT = _CONFIG / 'nav2_slam_floodfill_params.yaml'


def _load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _cm(doc):
    return doc['collision_monitor']['ros__parameters']


def test_base_has_both_collision_monitor_gates_enabled():
    cm = _cm(_load(_BASE))
    assert cm['FootprintApproach']['enabled'] is True
    assert cm['scan']['enabled'] is True


def test_variant_disables_both_collision_monitor_gates():
    cm = _cm(_load(_VARIANT))
    assert cm['FootprintApproach']['enabled'] is False
    assert cm['scan']['enabled'] is False


def test_variant_differs_from_base_only_by_the_two_enabled_flags():
    base, variant = _load(_BASE), _load(_VARIANT)
    patched = copy.deepcopy(variant)
    cm = patched['collision_monitor']['ros__parameters']
    cm['FootprintApproach']['enabled'] = True
    cm['scan']['enabled'] = True
    assert patched == base, "variant drifted from base beyond the two intended enabled flags"
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd "$SRC/tugbot_maze" && PYTHONPATH="$SRC/tugbot_maze" python3 -m pytest test/test_floodfill_collision_monitor_bypass.py -v`
Expected: FAIL — `FileNotFoundError` on `nav2_slam_floodfill_params.yaml` (variant not created yet).

- [ ] **Step 3: Create the variant** — copy the base, then disable both gates:

```bash
cp "$SRC/tugbot_navigation/config/nav2_slam_params.yaml" "$SRC/tugbot_navigation/config/nav2_slam_floodfill_params.yaml"
```
Then edit `nav2_slam_floodfill_params.yaml`, in the `collision_monitor: → ros__parameters:` block, change exactly two lines:
- under `FootprintApproach:` — `enabled: True` → `enabled: False`
- under `scan:` (the observation source) — `enabled: True` → `enabled: False`

Leave every other line identical to the base. (The two keys live at
`collision_monitor.ros__parameters.FootprintApproach.enabled` and `collision_monitor.ros__parameters.scan.enabled`.)

- [ ] **Step 4: Run to verify it passes**

Run: `cd "$SRC/tugbot_maze" && PYTHONPATH="$SRC/tugbot_maze" python3 -m pytest test/test_floodfill_collision_monitor_bypass.py -v`
Expected: PASS (the 3 tests). If `test_variant_differs...` fails, the copy drifted — re-copy and re-apply only the two `enabled` edits.

- [ ] **Step 5: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_navigation/config/nav2_slam_floodfill_params.yaml ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_floodfill_collision_monitor_bypass.py
git -C "$REPO" commit -m "feat: flood-fill nav2 params variant with collision_monitor disabled

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Select the variant in the launch + launch-expression eval guard

**Files:**
- Modify: `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`
- Modify: `src/tugbot_maze/test/test_floodfill_collision_monitor_bypass.py` (append the eval guard)

- [ ] **Step 1: Append the launch-expression eval-guard test** — add to `test/test_floodfill_collision_monitor_bypass.py`. It reconstructs the assembled `PythonExpression` string the way the launch builds it (flood-fill clause prepended to the existing profile chain) and `eval()`s it — catching the `''`-seam quoting bug that `py_compile` cannot. Keep this in sync with the launch edit in Step 3.

```python
import os


def _assembled_nav2_params(explorer_type, nav_share='/NAV'):
    """Mirror tugbot_maze_explore.launch.py's nav2_params_file PythonExpression assembly (flood_fill
    clause + abbreviated existing chain) and eval it. Guards the quoting seam. profiles all 'false'."""
    ff = os.path.join(nav_share, 'config', 'nav2_slam_floodfill_params.yaml')
    p26pc = os.path.join(nav_share, 'config', 'nav2_slam_phase26p_candidate_mppi_diagnostics_params.yaml')
    default = os.path.join(nav_share, 'config', 'nav2_slam_params.yaml')
    parts = [
        "'", ff, "' if '", explorer_type, "' == 'flood_fill' else ",   # new clause: ends 'else ' (NO quote)
        "'", p26pc, "' if '", 'false', "' == 'true' else '",            # existing chain's leading "'" follows
        default, "'",
    ]
    return eval(''.join(parts))


def test_launch_expr_selects_floodfill_variant_for_flood_fill():
    assert _assembled_nav2_params('flood_fill').endswith('nav2_slam_floodfill_params.yaml')


def test_launch_expr_falls_through_for_other_explorers():
    assert _assembled_nav2_params('maze_dfs').endswith('nav2_slam_params.yaml')
```

- [ ] **Step 2: Run to verify the guard passes** (it encodes the correct seam):

Run: `cd "$SRC/tugbot_maze" && PYTHONPATH="$SRC/tugbot_maze" python3 -m pytest test/test_floodfill_collision_monitor_bypass.py -k launch_expr -v`
Expected: PASS (2). (If you write the `parts` list with a trailing quote on the new clause — `"' == 'flood_fill' else '"` — `eval` raises `SyntaxError`, demonstrating the bug the launch must avoid.)

- [ ] **Step 3: Edit the launch** — in `tugbot_maze_explore.launch.py`, define the variant path near the other config paths and **prepend** the flood-fill clause to the existing `nav2_params_file = PythonExpression([ ... ])` list (currently starting ~line 27). The new list is:

```python
    floodfill_nav2_params = os.path.join(navigation_share, 'config', 'nav2_slam_floodfill_params.yaml')
    nav2_params_file = PythonExpression([
        "'", floodfill_nav2_params, "' if '", explorer_type, "' == 'flood_fill' else ",
        # ^ new clause ends with "else " (NO trailing quote); the existing chain's leading "'" (next
        #   line) supplies the opening quote. A trailing quote here -> `else ''<path>` -> launch eval
        #   SyntaxError (which py_compile does NOT catch).
        "'",
        os.path.join(navigation_share, 'config', 'nav2_slam_phase26p_candidate_mppi_diagnostics_params.yaml'),
        "' if '",
        phase26p_candidate_mppi_diagnostics_profile,
        "' == 'true' else '",
        # ... the rest of the EXISTING chain, byte-for-byte unchanged, through the final
        #     LaunchConfiguration('params_file') and its closing "'" ...
    ])
```
Concretely: keep the entire existing list from its current first element (`"'"`) onward **unchanged**, and only insert the five new leading elements
`"'", floodfill_nav2_params, "' if '", explorer_type, "' == 'flood_fill' else "` in front of it. Do not add or remove any quote in the existing chain.

- [ ] **Step 4: Verify launch compiles + the guard still matches**

Run: `cd "$SRC" && python3 -m py_compile tugbot_bringup/launch/tugbot_maze_explore.launch.py && echo COMPILE_OK`
Expected: `COMPILE_OK`.
Run the full guard suite: `cd "$SRC/tugbot_maze" && PYTHONPATH="$SRC/tugbot_maze" python3 -m pytest test/test_floodfill_collision_monitor_bypass.py -v`
Expected: PASS (all 5). (Definitive launch-eval validation happens at the Gazebo-run startup in Final Validation — a broken expression makes the launch die within seconds.)

- [ ] **Step 5: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_bringup/launch/tugbot_maze_explore.launch.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_floodfill_collision_monitor_bypass.py
git -C "$REPO" commit -m "feat: select collision_monitor-disabled params for the flood-fill explorer

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Final validation (after all tasks)

1. **Guard tests green:** `cd "$SRC/tugbot_maze" && PYTHONPATH="$SRC/tugbot_maze" python3 -m pytest test/test_floodfill_collision_monitor_bypass.py -v` → 5 passed.
2. **Build:** `cd "$REPO/ros2_ws_tugbot_nav_20260614" && source /opt/ros/jazzy/setup.bash && colcon build --packages-select tugbot_navigation tugbot_bringup`; confirm `install/tugbot_navigation/share/tugbot_navigation/config/nav2_slam_floodfill_params.yaml` exists.
3. **Gazebo confirmation (user-initiated; agent `pkill` is hook-blocked — user clears stray sims first via `! pkill -9 -f "gz sim|ruby.*gz|parameter_bridge|flood_fill_solver|ros2 launch"`):**
   `tools/run_flood_fill_maze.sh 1800 false true odom_locked true`.
   - **First confirm the bypass took effect** (else any persisting wedge is unattributable):
     - the launch started (no SyntaxError / `LAUNCH_DIED`);
     - `ros2 param get /collision_monitor FootprintApproach.enabled` and `... scan.enabled` report `False`;
     - no `"approach… away from collision"` / `"invalid source"` STOP lines gate the solver;
     - `/cmd_vel_smoothed` is non-zero while the solver commands motion (rules out `velocity_smoother` `velocity_timeout` zeroing).
   - **Then the success signals:** (1) **objective collision check** — post-process the recorded DIAG poses through `MazeSim.collides` against `load_segments()`; "zero collisions" is the safety gate; (2) wedge-recovers drop sharply vs 24/27; (3) deeper progress than dist 9.78 m; `EXIT_REACHED` is the stretch goal.
4. Do **not** merge to `main` or push — banking/merge is a separate, user-approved step after the Gazebo run. Both `junction-centerline-precision` and this branch remain unmerged pending the result.
