# Radar-Accumulated Occupancy Map + Red-Trail Removal — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `/maze/self_built_map` a radar-accumulated occupancy grid (log-odds ray-cast from the drift-free ICP pose) so the explored area grows in a radar-scatter style instead of 2×2 m cell blocks, and remove the gz_trail red ground-truth trail.

**Architecture:** A new pure module `radar_occupancy.py` ray-casts each LIDAR scan into a fixed log-odds grid (path cells → free, endpoint cell → occupied), using the same projection convention as the localizer (`SCAN_OFFSET_X` + pose rotate/translate), and thresholds to a classic −1/0/100 OccupancyGrid. The solver integrates each tick and republishes throttled on the existing `/maze/self_built_map`, replacing the grid-snapped block renderer (which is deleted). The gz_trail tool + its wrapper auto-start + tests + README notes are removed. Green walls + orange scatter cloud are unchanged.

**Tech Stack:** ROS 2 Jazzy, Python 3, NumPy, `nav_msgs/OccupancyGrid`, colcon (`--symlink-install`), pytest.

**Spec:** `docs/superpowers/specs/2026-07-12-radar-occupancy-map-design.md`
**Branch:** `scatter-cloud-map` (continues the scatter-cloud work; not yet merged).

**Paths (repo root):** `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`
- `$WS` = `ros2_ws_tugbot_nav_20260712`

**Canonical commands (verbatim; note `python3`, not `python`):**
- Run a pure test file (no build needed — imports `tugbot_maze` from `src`, msgs from ROS):
  `cd $WS && source /opt/ros/jazzy/setup.bash && PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/<file> -v`
- Full offline suite:
  `cd $WS && source /opt/ros/jazzy/setup.bash && PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q`
- Rebuild one package: `cd $WS && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select tugbot_maze`

**Suite baseline before this plan:** `389 passed, 7 failed` (the 7 are pre-existing sealed-test failures in `test_tugbot_maze_contract.py` + `test_wall_follow_maze_sim.py` — never touch them). Expected after each task: R1 → 397/7, R2 → 394/7, R3 → 386/7. The gate every task: **still exactly 7 pre-existing failures, zero new failures.**

> **Commit discipline:** trailer ends `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`. No backticks inside `git commit -m`; use a heredoc.

---

### Task R1: `RadarOccupancyGrid` pure module (TDD)

Build the ROS-free occupancy accumulator. Full TDD — tests first.

**Files:**
- Create: `$WS/src/tugbot_maze/tugbot_maze/radar_occupancy.py`
- Test: `$WS/src/tugbot_maze/test/test_radar_occupancy.py`

- [ ] **Step 1: Write the failing tests** — create `src/tugbot_maze/test/test_radar_occupancy.py`:

```python
import math

from tugbot_maze.footprint import SCAN_OFFSET_X
from tugbot_maze.radar_occupancy import RadarOccupancyGrid

# 8x8 m world perimeter centerlines (same bbox convention as the old renderer's tests).
_PERIM = [(0.0, 0.0, 8.0, 0.0), (8.0, 0.0, 8.0, 8.0),
          (8.0, 8.0, 0.0, 8.0), (0.0, 8.0, 0.0, 0.0)]


def _cell_at(g, x, y):
    ix = int((x - g.info.origin.position.x) / g.info.resolution)
    iy = int((y - g.info.origin.position.y) / g.info.resolution)
    return g.data[iy * g.info.width + ix]


def test_grid_metadata_from_perimeter_bbox():
    r = RadarOccupancyGrid(_PERIM, resolution=0.1, margin_m=0.5)
    g = r.to_occupancy_grid()
    assert g.header.frame_id == 'map'
    assert g.info.resolution == 0.1
    assert (g.info.width, g.info.height) == (90, 90)   # [0,8]+/-0.5 -> 9 m -> 90 cells
    assert (g.info.origin.position.x, g.info.origin.position.y) == (-0.5, -0.5)
    assert g.info.origin.orientation.w == 1.0
    assert len(g.data) == 90 * 90


def test_unobserved_is_unknown():
    r = RadarOccupancyGrid(_PERIM, resolution=0.1)
    assert set(r.to_occupancy_grid().data) == {-1}     # nothing integrated -> all unknown


def test_hit_marks_endpoint_occupied():
    # Robot at (4,4) facing +x; wall hit straight ahead at range 2.0.
    r = RadarOccupancyGrid(_PERIM, resolution=0.1)
    r.integrate_scan((4.0, 4.0, 0.0), [2.0], angle_min=0.0, angle_inc=0.1)
    ex = 4.0 + SCAN_OFFSET_X + 2.0
    assert _cell_at(r.to_occupancy_grid(), ex, 4.0) == 100


def test_ray_path_marks_free():
    r = RadarOccupancyGrid(_PERIM, resolution=0.1)
    r.integrate_scan((4.0, 4.0, 0.0), [2.0], angle_min=0.0, angle_inc=0.1)
    # ~1 m in front of the sensor, well before the endpoint -> free
    assert _cell_at(r.to_occupancy_grid(), 4.0 + SCAN_OFFSET_X + 1.0, 4.0) == 0


def test_projection_applies_pose_rotation():
    # Facing +y (yaw=pi/2); hit at range 2 -> endpoint ~ (4, 4 + SCAN_OFFSET_X + 2).
    r = RadarOccupancyGrid(_PERIM, resolution=0.1)
    r.integrate_scan((4.0, 4.0, math.pi / 2), [2.0], angle_min=0.0, angle_inc=0.1)
    assert _cell_at(r.to_occupancy_grid(), 4.0, 4.0 + SCAN_OFFSET_X + 2.0) == 100


def test_accumulation_is_monotonic_and_clamped():
    r = RadarOccupancyGrid(_PERIM, resolution=0.1, l_occ=0.85, l_clamp=5.0)
    ex = 4.0 + SCAN_OFFSET_X + 2.0
    for _ in range(100):                                # hammer the same endpoint
        r.integrate_scan((4.0, 4.0, 0.0), [2.0], angle_min=0.0, angle_inc=0.1)
    iy = int((4.0 - r.y0) / r.resolution)
    ix = int((ex - r.x0) / r.resolution)
    assert abs(float(r._logodds[iy, ix]) - r.l_clamp) <= 1e-4   # saturated at clamp, no overflow
    assert r.to_occupancy_grid().data[iy * r.width + ix] == 100


def test_invalid_beams_produce_no_update():
    r = RadarOccupancyGrid(_PERIM, resolution=0.1, usable_range_m=8.0)
    r.integrate_scan((4.0, 4.0, 0.0),
                     [float('inf'), float('nan'), -1.0, 0.0, 100.0],  # inf/nan/neg/zero/over-range
                     angle_min=0.0, angle_inc=0.1)
    assert set(r.to_occupancy_grid().data) == {-1}


def test_occupancy_values_are_three_valued():
    r = RadarOccupancyGrid(_PERIM, resolution=0.1)
    r.integrate_scan((4.0, 4.0, 0.0), [2.0, 3.0], angle_min=-0.2, angle_inc=0.2)
    assert set(r.to_occupancy_grid().data) <= {-1, 0, 100}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd $WS && source /opt/ros/jazzy/setup.bash && PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_radar_occupancy.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'tugbot_maze.radar_occupancy'`.

- [ ] **Step 3: Implement the module** — create `src/tugbot_maze/tugbot_maze/radar_occupancy.py`:

```python
"""Radar-accumulated occupancy grid for the online self-built map: integrate each LIDAR
scan by ray-casting from the sensor into a fixed log-odds grid -- cells along each beam's
free path decrease (free), the endpoint cell increases (occupied) -- using the same
projection convention as ScanMatchLocalizer (SCAN_OFFSET_X + pose rotate/translate) driven
by the drift-free ICP pose. Threshold to a classic -1/0/100 OccupancyGrid, so the explored
area grows in a radar-scatter style rather than as grid-snapped cell blocks. Pure NumPy +
std msgs -- no ROS node state, no rclpy.init needed."""
from __future__ import annotations
import math

import numpy as np
from nav_msgs.msg import OccupancyGrid

from tugbot_maze.footprint import SCAN_OFFSET_X


class RadarOccupancyGrid:
    def __init__(self, perimeter_segments, resolution: float = 0.05,
                 margin_m: float = 0.5, scan_offset_x: float = SCAN_OFFSET_X,
                 usable_range_m: float = 8.0, l_occ: float = 0.85,
                 l_free: float = 0.4, l_clamp: float = 5.0,
                 occ_thresh: float = 0.4, free_thresh: float = 0.4) -> None:
        segs = list(perimeter_segments)
        xs = [s[i] for s in segs for i in (0, 2)]
        ys = [s[i] for s in segs for i in (1, 3)]
        self.resolution = float(resolution)
        self.x0 = min(xs) - margin_m
        self.y0 = min(ys) - margin_m
        self.width = int(math.ceil((max(xs) + margin_m - self.x0) / resolution))
        self.height = int(math.ceil((max(ys) + margin_m - self.y0) / resolution))
        self.scan_offset_x = float(scan_offset_x)
        self.usable_range_m = float(usable_range_m)
        self.l_occ = float(l_occ)
        self.l_free = float(l_free)
        self.l_clamp = float(l_clamp)
        self.occ_thresh = float(occ_thresh)
        self.free_thresh = float(free_thresh)
        self._logodds = np.zeros((self.height, self.width), dtype=np.float32)

    def integrate_scan(self, pose, ranges, angle_min, angle_inc) -> None:
        """Ray-cast one scan into the log-odds grid: free along each beam, occupied at the
        endpoint. `pose` is the map->base_link (x, y, yaw). Invalid beams are dropped."""
        r = np.asarray(ranges, dtype=float)
        if r.size == 0:
            return
        ang = angle_min + np.arange(r.shape[0], dtype=float) * angle_inc   # sensor==base_link axes
        valid = np.isfinite(r) & (r > 0.0) & (r <= self.usable_range_m)
        r, ang = r[valid], ang[valid]
        if r.size == 0:
            return
        x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
        c, s = math.cos(th), math.sin(th)
        sx = x + c * self.scan_offset_x                    # sensor origin in map
        sy = y + s * self.scan_offset_x
        bx = self.scan_offset_x + r * np.cos(ang)          # endpoint in base_link
        by = r * np.sin(ang)
        px = x + c * bx - s * by                           # endpoint in map
        py = y + s * bx + c * by
        # Free-path samples: for each beam, step from the sensor toward the endpoint at one
        # cell per step, keeping t < r (the endpoint cell itself is handled as occupied).
        step = self.resolution
        max_n = int(math.ceil(float(r.max()) / step))
        t = np.arange(max_n, dtype=float) * step                       # (max_n,)
        dirx = np.cos(th + ang)                                        # beam heading in map
        diry = np.sin(th + ang)
        fx = sx + t[None, :] * dirx[:, None]                          # (B, max_n)
        fy = sy + t[None, :] * diry[:, None]
        keep = t[None, :] < r[:, None]                               # exclude endpoint sample
        ix = np.floor((fx - self.x0) / self.resolution).astype(np.int64)
        iy = np.floor((fy - self.y0) / self.resolution).astype(np.int64)
        inb = keep & (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        np.add.at(self._logodds, (iy[inb], ix[inb]), -self.l_free)
        ex = np.floor((px - self.x0) / self.resolution).astype(np.int64)
        ey = np.floor((py - self.y0) / self.resolution).astype(np.int64)
        einb = (ex >= 0) & (ex < self.width) & (ey >= 0) & (ey < self.height)
        np.add.at(self._logodds, (ey[einb], ex[einb]), self.l_occ)
        np.clip(self._logodds, -self.l_clamp, self.l_clamp, out=self._logodds)

    def to_occupancy_grid(self, frame_id: str = 'map', stamp=None) -> OccupancyGrid:
        """Threshold the log-odds grid to -1 unknown / 0 free / 100 occupied."""
        grid = np.full((self.height, self.width), -1, dtype=np.int8)
        grid[self._logodds <= -self.free_thresh] = 0
        grid[self._logodds >= self.occ_thresh] = 100
        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        if stamp is not None:
            msg.header.stamp = stamp
        msg.info.resolution = float(self.resolution)
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = float(self.x0)
        msg.info.origin.position.y = float(self.y0)
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel().tolist()          # row-major, row 0 at origin.y
        return msg
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd $WS && source /opt/ros/jazzy/setup.bash && PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_radar_occupancy.py -v`
Expected: PASS — 8 passed.

- [ ] **Step 5: Full suite (no regressions)**

Run: `cd $WS && source /opt/ros/jazzy/setup.bash && PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q`
Expected: `397 passed, 7 failed` (389 baseline + 8 new; the 7 are the pre-existing failures). Zero new failures.

- [ ] **Step 6: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260712/src/tugbot_maze/tugbot_maze/radar_occupancy.py \
        ros2_ws_tugbot_nav_20260712/src/tugbot_maze/test/test_radar_occupancy.py
git commit -F - <<'EOF'
feat: RadarOccupancyGrid pure accumulator (log-odds ray-cast -> OccupancyGrid)

TDD. Ray-casts each scan into a fixed log-odds grid (path->free, endpoint->
occupied) using the exact ScanMatchLocalizer projection (SCAN_OFFSET_X + pose
rotate/translate), thresholds to a classic -1/0/100 OccupancyGrid. ROS-free,
8 unit tests.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task R2: Switch `/maze/self_built_map` to the radar grid; delete the block renderer

Rewire the solver to build the map from `RadarOccupancyGrid` (throttled, online_slam-gated, never-crash), keep the green walls marker, and delete the now-dead `self_built_occupancy_grid` renderer + its unit tests.

**Files:**
- Modify: `$WS/src/tugbot_maze/tugbot_maze/flood_fill_solver.py`
- Modify: `$WS/src/tugbot_maze/tugbot_maze/flood_fill_viz.py`
- Modify: `$WS/src/tugbot_maze/test/test_flood_fill_viz.py`

- [ ] **Step 1: Solver — imports.** In `flood_fill_solver.py`, change the flood_fill_viz import block (currently two lines):
```python
from tugbot_maze.flood_fill_viz import (
    self_built_wall_markerarray, self_built_occupancy_grid)
```
to a single line:
```python
from tugbot_maze.flood_fill_viz import self_built_wall_markerarray
```
And add, immediately after `from tugbot_maze.scatter_cloud import ScatterCloud`:
```python
from tugbot_maze.radar_occupancy import RadarOccupancyGrid
```

- [ ] **Step 2: Solver — `__init__` accumulator.** Immediately after `self._scatter_last_pub = None` and before `self._walls_key = None`, insert:
```python
        # Radar-accumulated occupancy map (online_slam only): ray-cast each scan into a
        # log-odds grid so /maze/self_built_map grows in a radar-scatter style, not blocks.
        self._radar = (RadarOccupancyGrid(self._perimeter_segments, resolution=0.05)
                       if self.pose_source == 'online_slam' else None)
        self._radar_last_pub = None
```
(Leave `self.map_pub` and its QoS exactly as they are — the topic is unchanged.)

- [ ] **Step 3: Solver — drop the OccupancyGrid publish from `_publish_self_built_walls`.** Replace the whole method with (docstring updated; the `map_pub` publish and its comment removed; the warning text narrowed to "walls"):
```python
    def _publish_self_built_walls(self):
        """online_slam: publish the confirmed self-built walls as a MarkerArray so RViz
        shows the walls the robot built. Republished only when the sensed/committed set
        grows (cheap change-key), never allowed to kill the node. (The occupancy map itself
        is published separately by _publish_radar_map.)"""
        if self.pose_source != 'online_slam':
            return
        # Cheap growth trigger: republish when a cell is newly sensed/committed. Can lag by
        # <=1 cell if `committed` shrinks or a wall is refined within an already-sensed cell --
        # viz-only, self-heals on the next new cell.
        key = (len(self.motion.sensed), len(self.motion.committed))
        if key == self._walls_key:
            return
        try:
            cells = self.motion.sensed | self.motion.committed
            interior = confirmed_wall_segments(self.brain, cells)
            stamp = self.get_clock().now().to_msg()
            arr = self_built_wall_markerarray(
                list(self._perimeter_segments) + interior,
                frame_id=self.map_frame, stamp=stamp)
            self.walls_pub.publish(arr)
            self._walls_key = key    # advance only on success -> retry on transient failure
        except Exception as e:                       # viz must never crash the solver
            self.get_logger().warning('self-built walls publish failed: %r' % e)
```

- [ ] **Step 4: Solver — add `_publish_radar_map`.** Insert this method immediately after `_publish_scatter_cloud` (and before `_flush_junctions`):
```python
    def _publish_radar_map(self, now):
        """online_slam: ray-cast the live scan into the radar occupancy grid (same ICP pose
        + projection the localizer uses) and republish (throttled) as the OccupancyGrid on
        /maze/self_built_map. Pure viz -- read-only on _sm_corrected/scan_msg, never crashes."""
        if self.pose_source != 'online_slam' or self._radar is None:
            return
        if self._sm_corrected is None or self.scan_msg is None:
            return
        try:
            s = self.scan_msg
            self._radar.integrate_scan(
                self._sm_corrected, s.ranges, s.angle_min, s.angle_increment)
            due = (self._radar_last_pub is None or
                   (now - self._radar_last_pub).nanoseconds / 1e9 >= self.scatter_period_s)
            if due:      # the map evolves continuously -> republish every period while driving
                self.map_pub.publish(
                    self._radar.to_occupancy_grid(frame_id=self.map_frame,
                                                  stamp=now.to_msg()))
                self._radar_last_pub = now
        except Exception as e:                       # viz must never crash the solver
            self.get_logger().warning('radar map publish failed: %r' % e)
```

- [ ] **Step 5: Solver — call site.** In `_control_tick`'s `driving` branch, immediately after `self._publish_scatter_cloud(now)`, add:
```python
            self._publish_radar_map(now)
```

- [ ] **Step 6: Delete the block renderer.** In `flood_fill_viz.py`, delete the entire `self_built_occupancy_grid` function (from `def self_built_occupancy_grid(` through its `return msg`). Then remove the now-unused imports at the top — delete these three lines (the markerarray function does not use them):
```python
import math
```
```python
import numpy as np
```
```python
from nav_msgs.msg import OccupancyGrid
```
Update the module docstring so it no longer mentions the OccupancyGrid (it now only builds the wall MarkerArray). The new docstring:
```python
"""RViz visualization for the online self-built map: turn confirmed wall segments into a
MarkerArray so RViz shows the walls the robot actually built (perimeter + confirmed
interior walls). Pure builder -- no ROS node state."""
```
Keep `from geometry_msgs.msg import Point`, `from visualization_msgs.msg import Marker, MarkerArray`, `from typing import Iterable, Tuple`, `from __future__ import annotations`, and the `Segment = Tuple[...]` alias + `self_built_wall_markerarray` function exactly as they are.

- [ ] **Step 7: Delete the block-renderer tests.** In `test_flood_fill_viz.py`, delete everything from line `from tugbot_maze.flood_fill_viz import self_built_occupancy_grid` onward — i.e., that import, the `_PERIM`/`_WALLS` fixtures, the `_cell_at` helper, and the three tests `test_grid_metadata_from_perimeter_bbox`, `test_grid_three_value_semantics`, `test_grid_wall_band_width`. Keep the top imports and the two markerarray tests (`test_markerarray_is_line_list_two_points_per_segment`, `test_markerarray_empty_segments_is_valid`) unchanged. The file should end after the second markerarray test.

- [ ] **Step 8: Rebuild + full suite (no regressions)**

Run:
```bash
cd $WS && source /opt/ros/jazzy/setup.bash \
  && colcon build --symlink-install --packages-select tugbot_maze \
  && PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q
```
Expected: build `Finished <<< tugbot_maze`; suite `394 passed, 7 failed` (397 − 3 deleted grid tests; the 7 are pre-existing). Zero new failures. If a previously-passing test regresses, STOP and report BLOCKED.

- [ ] **Step 9: Import + attribute check**

Run:
```bash
cd $WS && source /opt/ros/jazzy/setup.bash && source install/setup.bash \
  && python3 -c "import tugbot_maze.flood_fill_solver as m; print('OK', hasattr(m.FloodFillSolver,'_publish_radar_map'))" \
  && python3 -c "import tugbot_maze.flood_fill_viz as v; print('renderer gone:', not hasattr(v,'self_built_occupancy_grid'))"
```
Expected: `OK True` and `renderer gone: True`.

- [ ] **Step 10: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260712/src/tugbot_maze/tugbot_maze/flood_fill_solver.py \
        ros2_ws_tugbot_nav_20260712/src/tugbot_maze/tugbot_maze/flood_fill_viz.py \
        ros2_ws_tugbot_nav_20260712/src/tugbot_maze/test/test_flood_fill_viz.py
git commit -F - <<'EOF'
feat: /maze/self_built_map is now a radar-accumulated occupancy grid

The map is built by ray-casting each scan into RadarOccupancyGrid (log-odds,
drift-free ICP pose) and republished throttled on the same topic, so the
explored area grows in a radar-scatter style instead of 2x2 m cell blocks.
_publish_self_built_walls keeps only the green wall MarkerArray. The dead
grid-snapped self_built_occupancy_grid renderer + its 3 unit tests are removed.
online_slam-gated, never-crash, read-only on nav state -- zero nav impact.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task R3: Remove the gz_trail red ground-truth trail

Delete the tool, its wrapper auto-start + kill hook, its tests, and the README notes about it.

**Files:**
- Modify: `$WS/tools/run_flood_fill_maze.sh`
- Delete: `$WS/tools/gz_trail.py`
- Delete: `$WS/src/tugbot_maze/test/test_gz_trail.py`
- Modify: `$WS/README.md`

- [ ] **Step 1: Run script — remove the kill-hook token.** In `tools/run_flood_fill_maze.sh`, inside `kill_all_sim()`, the line reads:
```
               flood_fill_solver gz_trail \
```
Change it to:
```
               flood_fill_solver \
```

- [ ] **Step 2: Run script — remove the auto-start block + PID kill.** Delete the header-comment line (near the top usage block):
```
#   HEADLESS=false also auto-starts tools/gz_trail.py (red ground-truth trail in the Gazebo scene).
```
Delete the trail launch block (the `TRAIL_PID=""` line through the closing `fi`):
```bash
TRAIL_PID=""
if [ "$HEADLESS" = "false" ]; then
    # Ground-truth red trail in the Gazebo scene (GUI runs only; read-only wrt navigation).
    python3 "$WS/tools/gz_trail.py" > "$ART/gz_trail.log" 2>&1 &
    TRAIL_PID=$!
    echo "[FLOODFILL] gz_trail PID=$TRAIL_PID (red ground-truth trail)" | tee -a "$ART/run_meta.txt"
fi
```
And delete the teardown line that kills the trail PID:
```bash
[ -n "$TRAIL_PID" ] && kill "$TRAIL_PID" 2>/dev/null
```

- [ ] **Step 3: Delete the tool + its test**

```bash
cd $WS
git rm tools/gz_trail.py src/tugbot_maze/test/test_gz_trail.py
```

- [ ] **Step 4: README — remove the trail references.** In `README.md`:
  1. Change `The wrapper does four things a bare` `ros2 launch` `does **not**:` → `three things`.
  2. Delete the bullet (three lines) starting `- **Auto-starts the red ground-truth trail**` through `Headless batches don't start it.`
  3. In the teardown bullet, change `(kills the launch, the trail drawer, and strays).` → `(kills the launch and strays).`
  4. In the artifacts bullet, change `(`result.txt`, `run_meta.txt`, `launch.log`, `gz_trail.log`).` → `(`result.txt`, `run_meta.txt`, `launch.log`).`
  5. Delete caveat #1 in the Method-B "Caveats" list — the item beginning `1. **No red trail.**` through its fenced block ending with the line `python3 tools/gz_trail.py            # add --style spheres for the bead chain` and its closing ```` ``` ````. Renumber the remaining caveats `2.`→`1.` and `3.`→`2.`.
  6. In the remaining SHM-cleanup `pkill` one-liner, change the pattern `"gz sim|ros2 launch|parameter_bridge|slam_toolbox|flood_fill_solver|gz_trail"` → `"gz sim|ros2 launch|parameter_bridge|slam_toolbox|flood_fill_solver"` (drop `|gz_trail`).
  7. Delete the trailing blockquote note about the Gazebo `/marker` reply type (the paragraph starting `> **Gazebo` `/marker` `note (Harmonic 8.11):**` through `you script markers by hand.`) — it exists only to document gz_trail's marker usage.

- [ ] **Step 5: Verify no gz_trail references remain**

Run:
```bash
cd $WS && grep -rn "gz_trail" tools/ src/ README.md ; echo "grep-rc=$?"
```
Expected: no matches, `grep-rc=1` (grep found nothing). If any line prints, remove it.

- [ ] **Step 6: Full suite (no regressions)**

Run: `cd $WS && source /opt/ros/jazzy/setup.bash && PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q`
Expected: `386 passed, 7 failed` (394 − 8 deleted gz_trail tests; the 7 pre-existing remain). Zero new failures.

- [ ] **Step 7: Sanity-check the run script still parses**

Run: `cd $WS && bash -n tools/run_flood_fill_maze.sh && echo "syntax OK"`
Expected: `syntax OK`.

- [ ] **Step 8: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add -A ros2_ws_tugbot_nav_20260712/tools/run_flood_fill_maze.sh \
           ros2_ws_tugbot_nav_20260712/README.md
git rm ros2_ws_tugbot_nav_20260712/tools/gz_trail.py \
       ros2_ws_tugbot_nav_20260712/src/tugbot_maze/test/test_gz_trail.py 2>/dev/null; true
git commit -F - <<'EOF'
chore: remove the gz_trail red ground-truth trail feature

Deletes tools/gz_trail.py + its tests, the wrapper's HEADLESS=false auto-start
and kill hook, and the README notes about the red trail and the /marker reply
type. GUI runs no longer draw the red path.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```
(If `git rm` in Step 3 already staged the deletions, the `git rm ... 2>/dev/null; true` here is a harmless no-op.)

---

## Acceptance (user-gated — do NOT auto-launch)

One GUI Gazebo run, launched only when the user says to. Criteria:
1. RViz `SelfBuiltMap` grows in a **radar-scatter style** — from all-grey, white free space and black walls emerge along the beams (not 2×2 m blocks) — aligned with the green walls + orange scatter, stable, no drift.
2. **No red trail** in the Gazebo scene.
3. `EXIT_REACHED`, oracle 0 collisions, run time comparable to baseline (pure-additive publish; no nav regression).

GUI launch: `export DISPLAY=:1 && bash tools/run_flood_fill_maze.sh 1000 false true online_slam` from `$WS`.

---

## Self-Review

**Spec coverage:** ① radar occupancy grid (log-odds ray-cast, hits→occupied, path→free, ICP pose) → R1 module + R2 wiring. ② −1/0/100 thresholding, same topic `/maze/self_built_map` → R1 `to_occupancy_grid` + R2. ③ pure radar (perimeter only sizes the grid, not pre-filled) → R1 `__init__` uses perimeter for bbox only. ④ green walls + orange scatter kept → R2 keeps `_publish_self_built_walls` marker + leaves scatter untouched. ⑤ delete block renderer + its tests → R2 Steps 6-7. ⑥ remove gz_trail (script + tool + tests + README) → R3. ⑦ unit tests (hit/free/unknown/accumulation/invalid/projection/three-valued/metadata) → R1 Step 1. ⑧ full suite zero-new-failure → R1/R2/R3 suite steps. ⑨ Gazebo 1-run acceptance → Acceptance section. All mapped.

**Placeholder scan:** none — every code step has complete code; every run step has an exact command + expected output; README edits are itemized with exact before/after text.

**Type/name consistency:** `RadarOccupancyGrid(perimeter_segments, resolution, margin_m, scan_offset_x, usable_range_m, l_occ, l_free, l_clamp, occ_thresh, free_thresh)`, `.integrate_scan(pose, ranges, angle_min, angle_inc)`, `.to_occupancy_grid(frame_id, stamp)`, attrs `x0/y0/width/height/resolution/l_clamp/_logodds` — identical across R1 tests, R1 impl, and R2 usage. Solver names `_publish_radar_map`, `_radar`, `_radar_last_pub`, reused `scatter_period_s`/`map_pub` — consistent across R2 edits and the Step 9 check. Projection matches `scatter_cloud`/`scan_match_localizer` (`bx=scan_offset_x+r*cos(ang); by=r*sin(ang)`; pose rotate/translate; beam heading `th+ang`).
