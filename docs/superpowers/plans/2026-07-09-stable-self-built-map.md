# Stable Self-Built Map (OccupancyGrid) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish the solver's grid-locked self-built map as a classic-SLAM-semantics OccupancyGrid on `/maze/self_built_map`, and make RViz's Map display show it by default — a stable, progressively-growing map product (grey unknown / white free / black walls) that can never rotate or ghost.

**Architecture:** A pure rendering function in `flood_fill_viz.py` turns (sensed cells, confirmed walls, perimeter) into a `nav_msgs/OccupancyGrid`. The solver publishes it inside the existing `_publish_self_built_walls` (same change-key, same guard, gated `online_slam`). RViz's Map display retargets from `/map` to the new topic and is enabled by default. slam_toolbox / Nav2 / the control chain are untouched — purely additive publishing.

**Tech Stack:** Python 3, numpy (already a workspace dep), `nav_msgs/OccupancyGrid` (exec_depend already present), RViz2 YAML config, pytest.

**Spec:** `docs/superpowers/specs/2026-07-09-stable-self-built-map-design.md`

**Workspace root for all commands:** `ros2_ws_tugbot_nav_20260705/`. Source per shell:
```bash
cd ros2_ws_tugbot_nav_20260705
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```
Python edits under `src/tugbot_maze/tugbot_maze/` are live (symlink-install, no rebuild). The RViz config change (Task 3) needs `colcon build --symlink-install --packages-select tugbot_bringup` (NEVER plain `colcon build` — it copy-installs and breaks `maze_sim`'s data path). Use `python3` (plain `python` does not exist).

---

## File structure

| File | Responsibility | Change |
|---|---|---|
| `src/tugbot_maze/tugbot_maze/flood_fill_viz.py` | Pure viz builders | **Modify**: add `self_built_occupancy_grid` |
| `src/tugbot_maze/test/test_flood_fill_viz.py` | Unit tests | **Modify**: add grid-semantics tests |
| `src/tugbot_maze/tugbot_maze/flood_fill_solver.py` | ROS node plumbing | **Modify**: `map_pub` + publish in `_publish_self_built_walls` |
| `src/tugbot_bringup/config/tugbot_nav.rviz` + `src/tugbot_bringup/rviz/tugbot_nav.rviz` | RViz displays | **Modify**: Map display → new topic, enabled |

Grounded facts (verified 2026-07-09): `nav_msgs` exec_depend already in `package.xml:12`; the Map display's main-Topic `Durability Policy` is ALREADY `Transient Local` (only `Value`, `Enabled`, and `Update Topic Value` need changing); `outer_segments()` returns 4 perimeter centerlines spanning x∈[0.95, 21.07], y∈[-1.04, 19.09] (pixel-converted, not perfectly axis-integer — the renderer must be data-driven, no hardcoded bounds); numpy is already used by `online_scan_match_localizer.py`.

---

### Task 1: Pure renderer `self_built_occupancy_grid` (TDD)

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/flood_fill_viz.py` (append)
- Modify: `src/tugbot_maze/test/test_flood_fill_viz.py` (append)

- [ ] **Step 1: Write the failing tests**

Append to `src/tugbot_maze/test/test_flood_fill_viz.py`:

```python
from tugbot_maze.flood_fill_viz import self_built_occupancy_grid


# Synthetic 8x8m world: 4 perimeter centerlines + one sensed cell (1,1)
# (interior [1,3]x[1,3]m) + one interior wall on that cell's north edge (y=3).
_PERIM = [(0.0, 0.0, 8.0, 0.0), (8.0, 0.0, 8.0, 8.0),
          (8.0, 8.0, 0.0, 8.0), (0.0, 8.0, 0.0, 0.0)]
_WALLS = [(1.0, 3.0, 3.0, 3.0)]


def _cell_at(g, x, y):
    """Occupancy value of the grid cell containing map point (x, y)."""
    ix = int((x - g.info.origin.position.x) / g.info.resolution)
    iy = int((y - g.info.origin.position.y) / g.info.resolution)
    return g.data[iy * g.info.width + ix]


def test_grid_metadata_from_perimeter_bbox():
    g = self_built_occupancy_grid({(1, 1)}, _WALLS, _PERIM,
                                  resolution=0.1, margin_m=0.5)
    assert g.header.frame_id == 'map'
    assert g.info.resolution == 0.1
    # bbox [0,8]^2 + 0.5 margin -> origin (-0.5,-0.5), 9x9 m -> 90x90 cells
    assert (g.info.width, g.info.height) == (90, 90)
    assert (g.info.origin.position.x, g.info.origin.position.y) == (-0.5, -0.5)
    assert g.info.origin.orientation.w == 1.0
    assert len(g.data) == 90 * 90


def test_grid_three_value_semantics():
    g = self_built_occupancy_grid({(1, 1)}, _WALLS, _PERIM,
                                  resolution=0.1, margin_m=0.5)
    assert _cell_at(g, 2.05, 2.05) == 0     # sensed cell interior = free
    assert _cell_at(g, 6.05, 6.05) == -1    # never-sensed area = unknown
    assert _cell_at(g, 4.05, 0.05) == 100   # perimeter wall band = occupied
    assert _cell_at(g, 2.05, 3.05) == 100   # interior wall OVERWRITES free
    assert _cell_at(g, 2.05, 2.55) == 0     # inside cell, off the wall band = free


def test_grid_wall_band_width():
    g = self_built_occupancy_grid(set(), _WALLS, _PERIM,
                                  resolution=0.1, margin_m=0.5,
                                  wall_half_thickness_m=0.12)
    # 0.35m above the wall centerline is outside the 0.12m band -> unknown
    assert _cell_at(g, 2.05, 3.35) == -1
    assert _cell_at(g, 2.05, 3.05) == 100
```

- [ ] **Step 2: Run to verify failure**

Run: `cd ros2_ws_tugbot_nav_20260705 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 -m pytest src/tugbot_maze/test/test_flood_fill_viz.py -q`
Expected: 3 pass (existing), 3 FAIL — `ImportError: cannot import name 'self_built_occupancy_grid'`.

- [ ] **Step 3: Implement**

In `src/tugbot_maze/tugbot_maze/flood_fill_viz.py`, extend the imports at the top:

```python
import math

import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
```
(keep the existing `from __future__` / typing lines; `geometry_msgs`/`visualization_msgs` imports already exist — just add `math`, `numpy`, `OccupancyGrid`.)

Append the function:

```python
def self_built_occupancy_grid(sensed_cells, wall_segments, perimeter_segments,
                              resolution: float = 0.05, margin_m: float = 0.5,
                              wall_half_thickness_m: float = 0.12,
                              frame_id: str = 'map', stamp=None) -> OccupancyGrid:
    """Render the self-built map as a classic-SLAM-semantics OccupancyGrid:
    unknown (-1) everywhere, sensed cells' 2x2m interiors free (0), and wall
    bands (perimeter + confirmed interior walls) occupied (100, overwrites free).
    Bounds come from the perimeter bbox + margin. Occupancy is decided at each
    grid cell's CENTER (inside a cell interior -> free; within
    wall_half_thickness_m of a wall centerline -> occupied)."""
    xs = [s[i] for s in perimeter_segments for i in (0, 2)]
    ys = [s[i] for s in perimeter_segments for i in (1, 3)]
    x0, y0 = min(xs) - margin_m, min(ys) - margin_m
    width = int(math.ceil((max(xs) + margin_m - x0) / resolution))
    height = int(math.ceil((max(ys) + margin_m - y0) / resolution))
    grid = np.full((height, width), -1, dtype=np.int8)

    # 1) sensed cell interiors -> free. Cell (c, r) interior spans
    #    [2c-1, 2c+1] x [2r-1, 2r+1] metres (cell centre = (2c, 2r)).
    for c, r in sensed_cells:
        ix0 = max(int((2 * c - 1 - x0) / resolution), 0)
        ix1 = min(int((2 * c + 1 - x0) / resolution), width)
        iy0 = max(int((2 * r - 1 - y0) / resolution), 0)
        iy1 = min(int((2 * r + 1 - y0) / resolution), height)
        grid[iy0:iy1, ix0:ix1] = 0

    # 2) wall bands -> occupied (overwrites free). Per segment, only the local
    #    window is evaluated: grid-cell centres within wall_half_thickness_m
    #    of the segment are set to 100.
    ht = wall_half_thickness_m
    cxs = x0 + (np.arange(width) + 0.5) * resolution
    cys = y0 + (np.arange(height) + 0.5) * resolution
    for sx0, sy0, sx1, sy1 in list(perimeter_segments) + list(wall_segments):
        wx0 = max(int((min(sx0, sx1) - ht - x0) / resolution), 0)
        wx1 = min(int(math.ceil((max(sx0, sx1) + ht - x0) / resolution)), width)
        wy0 = max(int((min(sy0, sy1) - ht - y0) / resolution), 0)
        wy1 = min(int(math.ceil((max(sy0, sy1) + ht - y0) / resolution)), height)
        if wx0 >= wx1 or wy0 >= wy1:
            continue
        X, Y = np.meshgrid(cxs[wx0:wx1], cys[wy0:wy1])
        ex, ey = sx1 - sx0, sy1 - sy0
        len2 = ex * ex + ey * ey
        if len2 == 0.0:
            t = np.zeros_like(X)
        else:
            t = np.clip(((X - sx0) * ex + (Y - sy0) * ey) / len2, 0.0, 1.0)
        d2 = (X - (sx0 + t * ex)) ** 2 + (Y - (sy0 + t * ey)) ** 2
        sub = grid[wy0:wy1, wx0:wx1]
        sub[d2 <= ht * ht] = 100

    msg = OccupancyGrid()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp
    msg.info.resolution = float(resolution)
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.position.x = float(x0)
    msg.info.origin.position.y = float(y0)
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.flatten().tolist()          # row-major, row 0 at origin.y
    return msg
```

- [ ] **Step 4: Run to verify pass**

Run: `python3 -m pytest src/tugbot_maze/test/test_flood_fill_viz.py -q`
Expected: PASS (6 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260705/src/tugbot_maze/tugbot_maze/flood_fill_viz.py ros2_ws_tugbot_nav_20260705/src/tugbot_maze/test/test_flood_fill_viz.py
git commit -F - <<'EOF'
feat: pure OccupancyGrid renderer for the self-built map (TDD)

Classic SLAM semantics from grid-locked data: unknown everywhere, sensed
cell interiors free, wall bands (perimeter + confirmed) occupied and
overwriting free. Bounds derived from the perimeter bbox + margin;
occupancy decided at grid-cell centres; numpy per-segment local windows.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task 2: Solver publishes `/maze/self_built_map`

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/flood_fill_solver.py`

- [ ] **Step 1: Extend imports**

Add to the imports (next to the existing `MarkerArray` import):
```python
from nav_msgs.msg import OccupancyGrid
```
and extend the viz import line to:
```python
from tugbot_maze.flood_fill_viz import (
    self_built_wall_markerarray, self_built_occupancy_grid)
```

- [ ] **Step 2: Add the publisher in `__init__`**

Immediately AFTER the line
```python
        self.walls_pub = self.create_publisher(MarkerArray, '/maze/self_built_walls', _walls_qos)
```
insert (reusing the same latched QoS profile):
```python
        self.map_pub = self.create_publisher(OccupancyGrid, '/maze/self_built_map', _walls_qos)
```

- [ ] **Step 3: Publish in `_publish_self_built_walls`**

The method's `try` block currently reads:
```python
        try:
            cells = self.motion.sensed | self.motion.committed
            segs = list(self._perimeter_segments) + confirmed_wall_segments(self.brain, cells)
            arr = self_built_wall_markerarray(
                segs, frame_id=self.map_frame, stamp=self.get_clock().now().to_msg())
            self.walls_pub.publish(arr)
            self._walls_key = key    # advance only on success -> retry on transient failure
        except Exception as e:                       # viz must never crash the solver
            self.get_logger().warning('self-built walls marker publish failed: %r' % e)
```
Replace it with:
```python
        try:
            cells = self.motion.sensed | self.motion.committed
            interior = confirmed_wall_segments(self.brain, cells)
            stamp = self.get_clock().now().to_msg()
            arr = self_built_wall_markerarray(
                list(self._perimeter_segments) + interior,
                frame_id=self.map_frame, stamp=stamp)
            self.walls_pub.publish(arr)
            # Same data rendered as a stable OccupancyGrid (grey unknown / white
            # free / black walls) -- the map product RViz shows by default.
            self.map_pub.publish(self_built_occupancy_grid(
                cells, interior, self._perimeter_segments,
                frame_id=self.map_frame, stamp=stamp))
            self._walls_key = key    # advance only on success -> retry on transient failure
        except Exception as e:                       # viz must never crash the solver
            self.get_logger().warning('self-built map/walls publish failed: %r' % e)
```
Also update the method docstring's first line from
`"""online_slam: publish perimeter + all confirmed interior walls as a MarkerArray so`
to
`"""online_slam: publish the self-built map as a MarkerArray + OccupancyGrid so`
(rest of the docstring unchanged).

- [ ] **Step 4: Verify import + suite**

```bash
python3 -c "import tugbot_maze.flood_fill_solver" && echo IMPORT_OK
python3 -m pytest src/tugbot_maze/test/ -q 2>&1 | tail -3
```
Expected: `IMPORT_OK`; `382 passed, 7 failed` — the 379 previous passes + 3 new Task-1 tests, and EXACTLY the 7 known pre-existing failures (6 × `test_wall_follow_maze_sim.py` + 1 × `test_maze_asset_and_config_are_present`). Zero new failures. (Node runtime behavior is validated in Task 5's Gazebo run.)

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260705/src/tugbot_maze/tugbot_maze/flood_fill_solver.py
git commit -F - <<'EOF'
feat: publish the self-built map as a latched OccupancyGrid (online_slam)

/maze/self_built_map rendered from the same data, same change-key, and same
never-crash guard as the walls MarkerArray. Purely additive; slam_toolbox,
Nav2, and the control chain untouched.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task 3: RViz Map display → the stable map, enabled by default

**Files:**
- Modify: `src/tugbot_bringup/config/tugbot_nav.rviz`
- Modify: `src/tugbot_bringup/rviz/tugbot_nav.rviz`

Apply the SAME three edits to BOTH files. In each file, find the display block `Class: rviz_default_plugins/Map` (verified current state: `Enabled: false`, Topic `Value: /map` with `Durability Policy: Transient Local` already set, `Update Topic` `Value: /map_updates`).

- [ ] **Step 1: Three edits per file**

1. `Enabled: false` → `Enabled: true` (the Map display's own Enabled, inside this block).
2. Topic `Value: /map` → `Value: /maze/self_built_map`.
3. Update Topic `Value: /map_updates` → `Value: /maze/self_built_map_updates`
   (we publish no updates topic — the display works on full-grid republish; this just avoids a stale subscription to slam_toolbox's updates).

Do NOT touch the main Topic's `Durability Policy: Transient Local` (already correct for the latched publisher) or any other field.

- [ ] **Step 2: Verify YAML + content**

```bash
python3 -c "import yaml; yaml.safe_load(open('src/tugbot_bringup/config/tugbot_nav.rviz')); yaml.safe_load(open('src/tugbot_bringup/rviz/tugbot_nav.rviz')); print('RVIZ_YAML_OK')"
grep -c "Value: /maze/self_built_map$" src/tugbot_bringup/config/tugbot_nav.rviz src/tugbot_bringup/rviz/tugbot_nav.rviz
grep -c "Value: /map$" src/tugbot_bringup/config/tugbot_nav.rviz src/tugbot_bringup/rviz/tugbot_nav.rviz
```
Expected: `RVIZ_YAML_OK`; first grep = 1 per file; second grep = 0 per file (no display left on `/map`).

- [ ] **Step 3: Rebuild so the installed config updates**

```bash
colcon build --symlink-install --packages-select tugbot_bringup 2>&1 | tail -2
source install/setup.bash
grep -c "Value: /maze/self_built_map$" install/tugbot_bringup/share/tugbot_bringup/rviz/tugbot_nav.rviz
```
Expected: build finishes; installed copy shows 1.

- [ ] **Step 4: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260705/src/tugbot_bringup/config/tugbot_nav.rviz ros2_ws_tugbot_nav_20260705/src/tugbot_bringup/rviz/tugbot_nav.rviz
git commit -F - <<'EOF'
feat: RViz Map display shows /maze/self_built_map, enabled by default

The stable self-built OccupancyGrid replaces slam_toolbox's /map as the
default map view (durability already Transient Local for the latch); the
/map topic still exists for manual comparison.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task 4: Full regression gate

**Files:** none (validation only)

- [ ] **Step 1: Full suite**

```bash
cd ros2_ws_tugbot_nav_20260705
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 -m pytest src/tugbot_maze/test/ -q 2>&1 | tail -3
```
Expected: `382 passed, 7 failed` — zero new failures vs. the 7 known pre-existing.

- [ ] **Step 2: Commit only if a fix was needed** (otherwise skip).

---

### Task 5: Gazebo acceptance — 1 GUI run (user-gated authority)

**Files:** none. **Do NOT auto-launch — wait for the user's "set up the run".**

- [ ] **Step 1: Launch**

```bash
tools/run_flood_fill_maze.sh 1000 false true online_slam
```

- [ ] **Step 2: Mid-run checks** (second sourced shell, matching `ROS_DOMAIN_ID` from run_meta.txt):

```bash
export ROS_DOMAIN_ID=<from run_meta>
ros2 topic list | grep -E "self_built_map|^/map$"     # both topics exist (slam untouched)
ros2 topic echo /maze/self_built_map --once --field info | head -5   # metadata sane
```
Expected: `/maze/self_built_map` AND `/map` both listed; resolution 0.05, width/height ≈ 423×423, origin ≈ (0.45, -1.54).

- [ ] **Step 3: User visual acceptance (the success criterion)**

In RViz the Map display now shows the self-built map: starts as grey unknown with the black perimeter, sensed cells turn white as the robot explores, confirmed walls turn black — **grid-aligned the whole run, no rotation/ghosting**, including through the maze-center turn. Green SelfBuiltWalls lines overlay exactly on the black wall bands.

- [ ] **Step 4: No-regression evidence**

After `EXIT_REACHED`:
```bash
python3 tools/replay_collision_oracle.py "$(ls -td log/flood_fill_run_* | head -1)"
```
Expected: 0 collisions; duration ~545–605 s (baseline range); `gz_trail.log` unaffected.

- [ ] **Step 5: Report results to the user** (validation only, no commit).

---

## Self-Review

**1. Spec coverage:**
- Pure renderer with classic SLAM semantics (bounds from perimeter bbox + margin, cell interiors free, wall bands occupied overwriting free, cell-center decision rule) → Task 1. ✓
- Solver publish: same change-key, same guard, latched QoS, gated online_slam, `/maze/self_built_map` → Task 2. ✓
- RViz retarget + enable + explicit Update Topic (durability already TL — grounded fact, no edit) → Task 3. ✓
- Zero-touch slam_toolbox/Nav2/control chain → no task touches them; Task 5 Step 2 verifies `/map` still exists. ✓
- Testing: unit semantics/metadata/band-width (T1), full suite (T2/T4), 1 GUI acceptance (T5, user-gated). ✓
- Overhead/error handling per spec → same guard (T2), latch QoS reused. ✓

**2. Placeholder scan:** none — all code steps show complete code; commands have expected outputs.

**3. Type consistency:** `self_built_occupancy_grid(sensed_cells, wall_segments, perimeter_segments, resolution, margin_m, wall_half_thickness_m, frame_id, stamp)` defined in Task 1 and called in Task 2 with `(cells, interior, self._perimeter_segments, frame_id=..., stamp=...)` — matches. `cells`/`interior` derived exactly as the existing method does; `_walls_qos`, `map_frame`, `_perimeter_segments` all pre-exist in the solver. Test helper `_cell_at` consistent with row-major `data` layout. ✓
