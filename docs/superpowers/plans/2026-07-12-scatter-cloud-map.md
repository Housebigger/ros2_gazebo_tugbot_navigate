# Radar-Scatter Accumulated Point-Cloud Overlay — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a purely-additive accumulated laser scatter cloud on `/maze/scatter_cloud` (PointCloud2, latched) that RViz overlays on the existing map, built in `flood_fill_solver` from the same drift-free ICP pose and `/scan` that drive navigation — giving the exploration map an authentic "radar scatter" look without touching any functional chain.

**Architecture:** A new pure module `scatter_cloud.py` projects each valid LIDAR return into the map frame using the *exact same projection the localizer uses* (`SCAN_OFFSET_X` + pose rotate/translate), voxel-dedups map points into an in-memory `set`, and exports the accumulated set as a `sensor_msgs/PointCloud2`. The solver feeds the live scan + `_sm_corrected` into it every control tick and republishes (throttled 0.5 s) — reusing the existing `online_slam`-gated, never-crash viz path. RViz gets one new PointCloud2 display (orange, world-scaled squares). All work lands in a fresh workspace `ros2_ws_tugbot_nav_20260712` clean-seeded from `ros2_ws_tugbot_nav_20260705`.

**Tech Stack:** ROS 2 Jazzy, Python 3, NumPy, `sensor_msgs/PointCloud2`, colcon (`--symlink-install`), pytest, RViz2, Gazebo Harmonic.

**Spec:** `docs/superpowers/specs/2026-07-12-scatter-cloud-map-design.md`

**Paths (repo root):** `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`
- Source workspace to clone: `ros2_ws_tugbot_nav_20260705`
- New workspace: `ros2_ws_tugbot_nav_20260712`
- Package under test: `ros2_ws_tugbot_nav_20260712/src/tugbot_maze`

**Canonical commands (use verbatim; `$WS` = new workspace dir):**
- Full build: `cd $WS && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install`
- Run pure tests (no build needed — imports `tugbot_maze` from `src`, `sensor_msgs` from ROS):
  `cd $WS && source /opt/ros/jazzy/setup.bash && PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python -m pytest src/tugbot_maze/test/<file> -v`
- Full offline suite:
  `cd $WS && source /opt/ros/jazzy/setup.bash && PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python -m pytest src/tugbot_maze/test -q`

> **Commit discipline:** every commit message ends with the trailer
> `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
> No backticks inside `git commit -m`; use a heredoc (`git commit -F - <<'EOF' … EOF`).

---

### Task 1: Seed the new workspace + feature branch

Clone `ros2_ws_tugbot_nav_20260705` into `ros2_ws_tugbot_nav_20260712` (excluding build artifacts), build it, capture the offline-suite baseline, and commit the seeded workspace on a feature branch. No feature code yet — this establishes a clean, building starting point identical to 20260705.

**Files:**
- Create: `ros2_ws_tugbot_nav_20260712/` (whole tree, seeded from `ros2_ws_tugbot_nav_20260705/`)

- [ ] **Step 1: Create the feature branch off main**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git checkout -b scatter-cloud-map
```

- [ ] **Step 2: Clone the workspace (exclude build/install/log/caches)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
rsync -a \
  --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  --exclude '.pytest_cache/' --exclude '__pycache__/' --exclude '*.pyc' \
  ros2_ws_tugbot_nav_20260705/ ros2_ws_tugbot_nav_20260712/
```

- [ ] **Step 3: Build with symlink-install (mandatory)**

Run:
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260712 \
  && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
```
Expected: `Summary: N packages finished` with **0 failed** (tugbot_bringup, tugbot_description, tugbot_exploration, tugbot_gazebo, tugbot_maze, tugbot_navigation all `Finished`).
⚠️ If a later step ever hits `FileNotFoundError` on `maze_wall_segments_*.yaml`, the workspace was built without `--symlink-install`: `rm -rf build/tugbot_maze install/tugbot_maze && colcon build --symlink-install`.

- [ ] **Step 4: Capture the offline-suite baseline**

Run:
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260712 \
  && source /opt/ros/jazzy/setup.bash \
  && PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python -m pytest src/tugbot_maze/test -q
```
Expected: a summary line like `379 passed, 7 xfailed …` (or similar). **Record the exact "N passed / M failed/xfailed" numbers** — this is the baseline every later task must not regress. Do not fix pre-existing failures; only ensure no *new* failures appear later.

- [ ] **Step 5: Commit the seeded workspace**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260712
git commit -F - <<'EOF'
chore: seed ros2_ws_tugbot_nav_20260712 from 20260705

Clean-seed for the radar-scatter phase: rsync clone of 20260705 src/tools/doc,
build artifacts excluded. Built with --symlink-install; offline suite matches
the 20260705 baseline. No feature code yet.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task 2: `ScatterCloud` pure module (TDD)

Build the ROS-free accumulator: project scan → map, voxel-dedup into a `set`, export as PointCloud2. Full TDD — this is where all the algorithmic correctness lives.

**Files:**
- Create: `ros2_ws_tugbot_nav_20260712/src/tugbot_maze/tugbot_maze/scatter_cloud.py`
- Test: `ros2_ws_tugbot_nav_20260712/src/tugbot_maze/test/test_scatter_cloud.py`

- [ ] **Step 1: Write the failing tests**

Create `src/tugbot_maze/test/test_scatter_cloud.py`:

```python
import math
import struct

from sensor_msgs.msg import PointCloud2, PointField
from tugbot_maze.footprint import SCAN_OFFSET_X
from tugbot_maze.scatter_cloud import ScatterCloud


def _decode_xy(msg):
    """Unpack the (x, y) of every point from a PointCloud2 (xyz float32 layout)."""
    data = bytes(msg.data)
    out = []
    for i in range(msg.width):
        x, y, _z = struct.unpack_from('<fff', data, i * msg.point_step)
        out.append((x, y))
    return out


def test_forward_beam_endpoint_includes_scan_offset():
    # Pose at origin facing +x; one beam straight ahead (ang=0), range 3.0 m.
    sc = ScatterCloud(voxel_m=0.05)
    sc.add_scan((0.0, 0.0, 0.0), [3.0], angle_min=0.0, angle_inc=0.1)
    pts = _decode_xy(sc.to_pointcloud2())
    assert len(pts) == 1
    px, py = pts[0]
    assert abs(px - (SCAN_OFFSET_X + 3.0)) <= sc.voxel_m
    assert abs(py - 0.0) <= sc.voxel_m


def test_projection_applies_pose_rotation_and_translation():
    # Pose (10, 5) facing +y (yaw=pi/2); beam straight ahead range 2.0 m.
    # base endpoint (SCAN_OFFSET_X+2.0, 0) rotated +90 and translated -> (10, 5 + SCAN_OFFSET_X + 2.0)
    sc = ScatterCloud(voxel_m=0.05)
    sc.add_scan((10.0, 5.0, math.pi / 2), [2.0], angle_min=0.0, angle_inc=0.1)
    (px, py), = _decode_xy(sc.to_pointcloud2())
    assert abs(px - 10.0) <= sc.voxel_m
    assert abs(py - (5.0 + SCAN_OFFSET_X + 2.0)) <= sc.voxel_m


def test_two_close_points_collapse_to_one_voxel():
    # Two beams at the same angle, endpoints ~1 cm apart -> one 5 cm voxel.
    sc = ScatterCloud(voxel_m=0.05)
    sc.add_scan((0.0, 0.0, 0.0), [1.000, 1.010], angle_min=0.0, angle_inc=0.0)
    assert len(sc) == 1


def test_accumulation_across_frames_is_union():
    sc = ScatterCloud(voxel_m=0.05)
    a = sc.add_scan((0.0, 0.0, 0.0), [1.0], angle_min=0.0, angle_inc=0.0)
    b = sc.add_scan((0.0, 0.0, 0.0), [5.0], angle_min=0.0, angle_inc=0.0)
    assert a == 1 and b == 1
    assert len(sc) == 2
    c = sc.add_scan((0.0, 0.0, 0.0), [1.0], angle_min=0.0, angle_inc=0.0)  # repeat -> nothing new
    assert c == 0
    assert len(sc) == 2


def test_invalid_beams_produce_no_points():
    sc = ScatterCloud(voxel_m=0.05, usable_range_m=8.0)
    added = sc.add_scan(
        (0.0, 0.0, 0.0),
        [float('inf'), float('nan'), -1.0, 0.0, 100.0],  # inf, nan, negative, zero, beyond usable range
        angle_min=0.0, angle_inc=0.1)
    assert added == 0
    assert len(sc) == 0


def test_pointcloud2_message_is_wellformed():
    sc = ScatterCloud(voxel_m=0.05)
    sc.add_scan((0.0, 0.0, 0.0), [1.0, 5.0], angle_min=0.0, angle_inc=0.5)
    msg = sc.to_pointcloud2(frame_id='map')
    assert isinstance(msg, PointCloud2)
    assert msg.header.frame_id == 'map'
    assert msg.height == 1
    assert msg.width == len(sc)
    assert msg.point_step == 12
    assert msg.row_step == 12 * msg.width
    assert msg.is_dense is True
    assert msg.is_bigendian is False
    assert [f.name for f in msg.fields] == ['x', 'y', 'z']
    assert all(f.datatype == PointField.FLOAT32 for f in msg.fields)
    assert len(bytes(msg.data)) == 12 * msg.width
    assert len(_decode_xy(msg)) == msg.width


def test_empty_cloud_is_valid():
    sc = ScatterCloud(voxel_m=0.05)
    msg = sc.to_pointcloud2(frame_id='map')
    assert msg.width == 0
    assert bytes(msg.data) == b''
    assert msg.row_step == 0
```

- [ ] **Step 2: Run the tests to verify they fail**

Run:
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260712 \
  && source /opt/ros/jazzy/setup.bash \
  && PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python -m pytest src/tugbot_maze/test/test_scatter_cloud.py -v
```
Expected: FAIL — `ModuleNotFoundError: No module named 'tugbot_maze.scatter_cloud'`.

- [ ] **Step 3: Implement the module**

Create `src/tugbot_maze/tugbot_maze/scatter_cloud.py`:

```python
"""Accumulated LIDAR scatter cloud for the online self-built map: project each valid
scan return into the map frame (using the exact same projection the ScanMatchLocalizer
uses -- SCAN_OFFSET_X plus a pose rotate/translate), voxel-dedup the map points into an
in-memory set that grows over the run, and export the accumulation as a PointCloud2.
Pure NumPy + std msgs -- no ROS node state, no rclpy.init needed."""
from __future__ import annotations
import math

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from tugbot_maze.footprint import SCAN_OFFSET_X   # DRY: the one true LIDAR mount offset


class ScatterCloud:
    def __init__(self, voxel_m: float = 0.05,
                 scan_offset_x: float = SCAN_OFFSET_X,
                 usable_range_m: float = 8.0) -> None:
        self.voxel_m = float(voxel_m)
        self.scan_offset_x = float(scan_offset_x)
        self.usable_range_m = float(usable_range_m)
        self._voxels: set[tuple[int, int]] = set()

    def add_scan(self, pose, ranges, angle_min, angle_inc) -> int:
        """Project one scan to the map frame and merge its voxel-snapped endpoints into
        the accumulated set. `pose` is the map->base_link (x, y, yaw). Returns the number
        of NEW voxels added by this scan. Mirrors ScanMatchLocalizer._beams_to_points."""
        r = np.asarray(ranges, dtype=float)
        if r.size == 0:
            return 0
        ang = angle_min + np.arange(r.shape[0], dtype=float) * angle_inc   # sensor==base_link axes
        valid = np.isfinite(r) & (r > 0.0) & (r <= self.usable_range_m)
        r, ang = r[valid], ang[valid]
        if r.size == 0:
            return 0
        bx = self.scan_offset_x + r * np.cos(ang)                          # endpoint in base_link
        by = r * np.sin(ang)
        x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
        c, s = math.cos(th), math.sin(th)
        px = x + c * bx - s * by                                           # base_link -> map
        py = y + s * bx + c * by
        kx = np.round(px / self.voxel_m).astype(np.int64)
        ky = np.round(py / self.voxel_m).astype(np.int64)
        before = len(self._voxels)
        self._voxels.update(zip(kx.tolist(), ky.tolist()))
        return len(self._voxels) - before

    def to_pointcloud2(self, frame_id: str = 'map', stamp=None) -> PointCloud2:
        """Export the accumulated voxel set as an xyz-float32 PointCloud2 (z=0).
        Point coords are the voxel keys mapped back to map metres (kx*voxel_m, ky*voxel_m),
        the strict inverse of the round(coord/voxel_m) snapping in add_scan."""
        msg = PointCloud2()
        msg.header.frame_id = frame_id
        if stamp is not None:
            msg.header.stamp = stamp
        msg.height = 1
        msg.width = len(self._voxels)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.is_dense = True
        if self._voxels:
            keys = np.array(sorted(self._voxels), dtype=np.float64)        # (N,2) deterministic order
            xyz = np.zeros((keys.shape[0], 3), dtype='<f4')
            xyz[:, 0] = keys[:, 0] * self.voxel_m
            xyz[:, 1] = keys[:, 1] * self.voxel_m
            msg.data = xyz.tobytes()
        else:
            msg.data = b''
        return msg

    def __len__(self) -> int:
        return len(self._voxels)
```

- [ ] **Step 4: Run the tests to verify they pass**

Run:
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260712 \
  && source /opt/ros/jazzy/setup.bash \
  && PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python -m pytest src/tugbot_maze/test/test_scatter_cloud.py -v
```
Expected: PASS — 7 passed.

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260712/src/tugbot_maze/tugbot_maze/scatter_cloud.py \
        ros2_ws_tugbot_nav_20260712/src/tugbot_maze/test/test_scatter_cloud.py
git commit -F - <<'EOF'
feat: ScatterCloud pure accumulator (project + voxel-dedup + PointCloud2)

TDD. Projects each valid scan return to the map frame using the exact
ScanMatchLocalizer convention (SCAN_OFFSET_X + pose rotate/translate),
voxel-dedups into an in-memory set that grows over the run, exports as an
xyz-float32 PointCloud2. ROS-free, 7 unit tests.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task 3: Wire the scatter cloud into the solver

Add the PointCloud2 publisher, a `ScatterCloud` member, a throttled `_publish_scatter_cloud`, and the call site — all reusing the existing `online_slam`-gated, never-crash viz path. No navigation logic changes.

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260712/src/tugbot_maze/tugbot_maze/flood_fill_solver.py`

- [ ] **Step 1: Add imports**

In `flood_fill_solver.py`, change the `sensor_msgs` import (currently `from sensor_msgs.msg import LaserScan`) to:

```python
from sensor_msgs.msg import LaserScan, PointCloud2
```

And add, immediately after the `from tugbot_maze.flood_fill_viz import (...)` block:

```python
from tugbot_maze.scatter_cloud import ScatterCloud
```

- [ ] **Step 2: Add the publisher + accumulator in `__init__`**

Immediately after the line `self.map_pub = self.create_publisher(OccupancyGrid, '/maze/self_built_map', _walls_qos)` and before `self._walls_key = None`, insert:

```python
        # Radar-scatter overlay (online_slam only): accumulate live scan returns into a
        # map-frame scatter cloud and republish (throttled) on top of the self-built map.
        self.scatter_pub = self.create_publisher(PointCloud2, '/maze/scatter_cloud', _walls_qos)
        self.scatter_period_s = float(self.declare_parameter('scatter_period_s', 0.5).value)
        self._scatter = ScatterCloud(voxel_m=0.05)
        self._scatter_last_pub = None
```

- [ ] **Step 3: Add the `_publish_scatter_cloud` method**

Insert this method immediately after `_publish_self_built_walls` (right before `def _flush_junctions`):

```python
    def _publish_scatter_cloud(self, now):
        """online_slam: accumulate the live scan into a map-frame scatter cloud (same ICP
        pose + projection the localizer uses) and republish (throttled) as a PointCloud2.
        Pure viz -- read-only on _sm_corrected/scan_msg, never allowed to kill the node."""
        if self.pose_source != 'online_slam':
            return
        if self._sm_corrected is None or self.scan_msg is None:
            return
        try:
            s = self.scan_msg
            added = self._scatter.add_scan(
                self._sm_corrected, s.ranges, s.angle_min, s.angle_increment)
            due = (self._scatter_last_pub is None or
                   (now - self._scatter_last_pub).nanoseconds / 1e9 >= self.scatter_period_s)
            if due and (added > 0 or self._scatter_last_pub is None):
                self.scatter_pub.publish(
                    self._scatter.to_pointcloud2(frame_id=self.map_frame,
                                                 stamp=now.to_msg()))
                self._scatter_last_pub = now
        except Exception as e:                       # viz must never crash the solver
            self.get_logger().warning('scatter cloud publish failed: %r' % e)
```

- [ ] **Step 4: Add the call site in `_control_tick`**

In the `if self.phase == 'driving':` branch, immediately after `self._publish_self_built_walls()`, add:

```python
            self._publish_scatter_cloud(now)
```

(`now` is already bound at the top of `_control_tick` as `now = self.get_clock().now()`.)

- [ ] **Step 5: Rebuild and run the full offline suite (no regressions)**

Run:
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260712 \
  && source /opt/ros/jazzy/setup.bash \
  && colcon build --symlink-install --packages-select tugbot_maze \
  && PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python -m pytest src/tugbot_maze/test -q
```
Expected: build `Finished <<< tugbot_maze`; suite shows the **same** passed/xfailed counts as the Task 1 baseline (plus the 7 new `test_scatter_cloud` tests → passed count = baseline + 7). Zero NEW failures.

- [ ] **Step 6: Verify the node imports and constructs cleanly**

Run (sanity import — catches wiring typos without a full sim):
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260712 \
  && source /opt/ros/jazzy/setup.bash && source install/setup.bash \
  && python -c "import tugbot_maze.flood_fill_solver as m; print('import OK:', hasattr(m.FloodFillSolver, '_publish_scatter_cloud'))"
```
Expected: `import OK: True`.

- [ ] **Step 7: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260712/src/tugbot_maze/tugbot_maze/flood_fill_solver.py
git commit -F - <<'EOF'
feat: publish accumulated scatter cloud from the solver (online_slam)

New /maze/scatter_cloud (PointCloud2, latched). Each control tick feeds the
live scan + the drift-free ICP pose (_sm_corrected) into ScatterCloud and
republishes throttled (0.5 s, only when new voxels appear). Reuses the
existing online_slam gate + never-crash try/except; read-only on nav state,
zero navigation impact. Full offline suite unchanged.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task 4: Add the RViz PointCloud2 display

Add one PointCloud2 display (orange, world-scaled flat squares, transient-local latch) to the single RViz config, subscribed to `/maze/scatter_cloud`, enabled by default, drawn on top of the walls/map.

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260712/src/tugbot_bringup/rviz/tugbot_nav.rviz`

- [ ] **Step 1: Insert the ScatterCloud display block**

In `tugbot_bringup/rviz/tugbot_nav.rviz`, find the end of the `SelfBuiltWalls` display — the lines:

```yaml
        Value: /maze/self_built_walls
      Value: true
  Enabled: true
  Global Options:
```

Insert the new display block between `      Value: true` (end of SelfBuiltWalls) and `  Enabled: true` (start of the panel-level keys), so it reads:

```yaml
        Value: /maze/self_built_walls
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 140; 0
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: ScatterCloud
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.03
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Transient Local
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /maze/scatter_cloud
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
  Enabled: true
  Global Options:
```

- [ ] **Step 2: Verify the RViz config is still valid YAML with the block present**

Run:
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260712 \
  && python -c "
import yaml
d = yaml.safe_load(open('src/tugbot_bringup/rviz/tugbot_nav.rviz'))
disp = d['Visualization Manager']['Displays']
sc = [x for x in disp if x.get('Name') == 'ScatterCloud']
assert len(sc) == 1, 'ScatterCloud display missing/duplicated'
x = sc[0]
assert x['Class'] == 'rviz_default_plugins/PointCloud2'
assert x['Topic']['Value'] == '/maze/scatter_cloud'
assert x['Topic']['Durability Policy'] == 'Transient Local'
assert x['Color'] == '255; 140; 0'
assert x['Enabled'] is True
print('ScatterCloud display OK')
"
```
Expected: `ScatterCloud display OK` (and no `yaml` parse error / no `AssertionError`).

- [ ] **Step 3: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260712/src/tugbot_bringup/rviz/tugbot_nav.rviz
git commit -F - <<'EOF'
feat: RViz ScatterCloud display for /maze/scatter_cloud (orange, default on)

PointCloud2 display, FlatColor orange (255;140;0), Flat Squares 0.03 m,
Transient Local latch to match the publisher, enabled by default, drawn on
top of SelfBuiltMap + SelfBuiltWalls.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

## Acceptance (user-gated — do NOT auto-launch)

After Tasks 1-4 pass and the final whole-change review is clean, the feature is verified by **one GUI Gazebo run**, launched only when the user says to set up the run (never auto-launched). Acceptance criteria (from the spec):

1. In RViz, `/maze/scatter_cloud` shows an **orange scatter cloud** that accumulates densely along the walls as the robot explores, **aligned point-for-point** with the green `SelfBuiltWalls` and the grey `SelfBuiltMap` — no drift, no rotation, no smear on turns.
2. `EXIT_REACHED`, **oracle 0 collisions**, run time comparable to the 20260705 baseline (pure-additive publish, no navigation regression).
3. All existing behaviour byte-for-byte unchanged: SelfBuiltMap / SelfBuiltWalls / rviz-truth TF / gz-trail / slam_toolbox / Nav2 / control chain untouched; `scan_match` mode unaffected.

The exact GUI launch command is whatever `ros2_ws_tugbot_nav_20260712/README.md` documents for an `online_slam` visualization run (inherited from 20260705).

---

## Self-Review

**Spec coverage:** ① scatter overlay (`/maze/scatter_cloud`, PointCloud2, latched) → Task 3. ② full accumulation + voxel dedup → Task 2. ③ projection mirrors `_beams_to_points` w/ `SCAN_OFFSET_X`, reuses `_sm_corrected` → Task 2 (module) + Task 3 (feeds real pose/scan). ④ orange / 0.05 m / 0.5 s → Task 2 (voxel), Task 3 (period), Task 4 (color). ⑤ online_slam gate + never-crash try/except → Task 3. ⑥ RViz display default-on, transient-local → Task 4. ⑦ new workspace clean-seed + symlink build → Task 1. ⑧ unit tests (projection/dedup/accumulation/invalid/message) + full suite zero-new-failure → Task 2 + Tasks 1/3. ⑨ Gazebo 1-run acceptance → Acceptance section. All spec sections mapped.

**Placeholder scan:** No TBD/TODO; every code step shows complete code; every run step shows an exact command + expected output.

**Type/name consistency:** `ScatterCloud(voxel_m, scan_offset_x, usable_range_m)`, `.add_scan(pose, ranges, angle_min, angle_inc) -> int`, `.to_pointcloud2(frame_id, stamp)`, `.__len__`, attribute `voxel_m` — identical across Task 2 tests, Task 2 impl, and Task 3 call. Solver names (`_publish_scatter_cloud`, `scatter_pub`, `_scatter`, `_scatter_last_pub`, `scatter_period_s`) consistent across Task 3 and the Step 6 import check. Projection matches `scan_match_localizer._beams_to_points` verbatim (`bx=scan_offset_x+r*cos(ang); by=r*sin(ang)`; pose rotate/translate).
