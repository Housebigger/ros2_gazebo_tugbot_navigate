# Truthful RViz for `online_slam` Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make RViz reflect the pose the robot actually uses in `pose_source=online_slam` — the solver publishes `map→odom` from its ICP pose and the growing self-built wall map as markers, so the native RViz robot/scan/map become truthful and the maze-center wobble disappears.

**Architecture:** The flood-fill solver already computes an absolute ICP pose (`_sm_corrected`, `map` frame) but publishes no TF/markers — RViz is anchored to slam_toolbox, which wobbles in the open center. This plan makes the solver publish `map→odom` (seeded from the known entrance), publish the confirmed walls as a `LINE_LIST` MarkerArray, and silences slam_toolbox's `map→odom` in `online_slam` mode. All changes are gated on `online_slam`; `scan_match` is untouched.

**Tech Stack:** ROS 2 Jazzy, Python, `tf2_ros.TransformBroadcaster`, `visualization_msgs/MarkerArray`, slam_toolbox params, RViz2 config, pytest.

**Spec:** `docs/superpowers/specs/2026-07-08-rviz-truth-online-slam-design.md`

**Workspace root for all commands:** `ros2_ws_tugbot_nav_20260705/` (paths below are relative to it unless noted). Build/source once per shell:
```bash
cd ros2_ws_tugbot_nav_20260705
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```
Note: the package is symlink-installed, so Python edits under `src/tugbot_maze/tugbot_maze/` take effect on re-run without rebuild; new files, `package.xml`, launch, and config changes require a rebuild. **Always rebuild with `--symlink-install`**: `colcon build --symlink-install --packages-select tugbot_maze tugbot_bringup tugbot_navigation`. A plain `colcon build` (no `--symlink-install`) converts the package to a copy-install, which breaks `maze_sim.default_segments_path()` — it resolves the maze data via `os.path.dirname(__file__)/../config/`, valid only when `__file__` points into `src/`. If the offline sim suddenly `FileNotFoundError`s on `maze_wall_segments_20260528.yaml`, rebuild with `--symlink-install` (do `rm -rf build/tugbot_maze install/tugbot_maze` first if switching install modes).

---

## File structure

| File | Responsibility | Change |
|---|---|---|
| `src/tugbot_maze/tugbot_maze/pose_tracking.py` | ROS-free 2D pose math | **Modify**: add `yaw_to_quat`, `map_to_odom` |
| `src/tugbot_maze/tugbot_maze/flood_fill_viz.py` | Pure builder: confirmed walls → MarkerArray | **Create** |
| `src/tugbot_maze/tugbot_maze/flood_fill_solver.py` | ROS node plumbing | **Modify**: entrance params, TF broadcaster + `map→odom` publish, entrance bootstrap seed, walls marker publish |
| `src/tugbot_maze/package.xml` | Package deps | **Modify**: add `visualization_msgs` exec_depend |
| `src/tugbot_navigation/config/slam_toolbox_params_online_slam.yaml` | slam_toolbox variant with TF silenced | **Create** |
| `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py` | Launch wiring | **Modify**: entrance_* → flood_fill node; conditional slam params for `online_slam` |
| `src/tugbot_bringup/config/tugbot_nav.rviz` + `src/tugbot_bringup/rviz/tugbot_nav.rviz` | RViz displays | **Modify**: add `SelfBuiltWalls`, disable `/map` |
| `src/tugbot_maze/test/test_pose_tracking.py` | Unit tests | **Modify**: cover new helpers |
| `src/tugbot_maze/test/test_flood_fill_viz.py` | Unit tests | **Create** |

**Testability note:** the ROS node (`flood_fill_solver.py`) has no unit-test harness in this repo (no test does `rclpy.init`), and the offline sim (`test_online_slam_sim.py`) drives the localizer+motion directly, bypassing the node. So Tasks 3–5 (node glue) are gated by (a) importing the module cleanly + full unit suite green, and (b) the Gazebo acceptance in Task 9. The pure logic they depend on (Tasks 1–2) is fully unit-tested.

---

### Task 1: Pose-math helpers `yaw_to_quat` and `map_to_odom`

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/pose_tracking.py`
- Test: `src/tugbot_maze/test/test_pose_tracking.py`

- [ ] **Step 1: Write the failing tests**

Append to `src/tugbot_maze/test/test_pose_tracking.py`:

```python
import math
from tugbot_maze.pose_tracking import (
    compose_2d, inverse_2d, yaw_to_quat, map_to_odom)


def test_yaw_to_quat_known_angles():
    for yaw, (ez, ew) in [(0.0, (0.0, 1.0)),
                          (math.pi / 2, (math.sqrt(0.5), math.sqrt(0.5))),
                          (math.pi, (1.0, 0.0)),
                          (-math.pi / 2, (-math.sqrt(0.5), math.sqrt(0.5)))]:
        x, y, z, w = yaw_to_quat(yaw)
        assert (x, y) == (0.0, 0.0)
        assert math.isclose(z, ez, abs_tol=1e-9)
        assert math.isclose(w, ew, abs_tol=1e-9)


def test_map_to_odom_roundtrip():
    # For any map->base and odom->base, compose(map_to_odom, odom->base) == map->base
    map_base = (3.0, -2.0, 1.1)
    odom_base = (0.7, 0.4, 0.3)
    m2o = map_to_odom(map_base, odom_base)
    back = compose_2d(m2o, odom_base)
    for a, b in zip(back, map_base):
        assert math.isclose(a, b, abs_tol=1e-9)
```

- [ ] **Step 2: Run to verify failure**

Run: `cd ros2_ws_tugbot_nav_20260705 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && python -m pytest src/tugbot_maze/test/test_pose_tracking.py -q`
Expected: FAIL — `ImportError: cannot import name 'yaw_to_quat'`.

- [ ] **Step 3: Implement**

Append to `src/tugbot_maze/tugbot_maze/pose_tracking.py`:

```python
def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    """(x, y, z, w) quaternion for a planar yaw (rad)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def map_to_odom(map_to_base: Pose2D, odom_to_base: Pose2D) -> Pose2D:
    """map->odom offset such that compose_2d(result, odom_to_base) == map_to_base.

    The solver holds map->base (its ICP pose) and reads odom->base from TF; publishing
    this offset as the map->odom transform makes the full TF tree resolve to the ICP pose.
    """
    return compose_2d(map_to_base, inverse_2d(odom_to_base))
```

- [ ] **Step 4: Run to verify pass**

Run: `python -m pytest src/tugbot_maze/test/test_pose_tracking.py -q`
Expected: PASS (all tests).

- [ ] **Step 5: Commit**

```bash
git add src/tugbot_maze/tugbot_maze/pose_tracking.py src/tugbot_maze/test/test_pose_tracking.py
git commit -F - <<'EOF'
feat: add yaw_to_quat and map_to_odom pose helpers

Pure 2D helpers for the solver to publish map->odom from its ICP pose.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task 2: Pure self-built-walls MarkerArray builder

**Files:**
- Create: `src/tugbot_maze/tugbot_maze/flood_fill_viz.py`
- Create: `src/tugbot_maze/test/test_flood_fill_viz.py`
- Modify: `src/tugbot_maze/package.xml`

- [ ] **Step 1: Add the dependency**

In `src/tugbot_maze/package.xml`, add next to the other `exec_depend` entries:

```xml
  <exec_depend>visualization_msgs</exec_depend>
```

- [ ] **Step 2: Write the failing test**

Create `src/tugbot_maze/test/test_flood_fill_viz.py`:

```python
from visualization_msgs.msg import Marker
from tugbot_maze.flood_fill_viz import self_built_wall_markerarray


def test_markerarray_is_line_list_two_points_per_segment():
    segs = [(0.0, 0.0, 2.0, 0.0), (2.0, 0.0, 2.0, 2.0)]
    arr = self_built_wall_markerarray(segs, frame_id='map')
    assert len(arr.markers) == 1
    m = arr.markers[0]
    assert m.type == Marker.LINE_LIST
    assert m.action == Marker.ADD
    assert m.header.frame_id == 'map'
    assert len(m.points) == 4                      # 2 endpoints * 2 segments
    assert (m.points[0].x, m.points[0].y) == (0.0, 0.0)
    assert (m.points[1].x, m.points[1].y) == (2.0, 0.0)
    assert m.scale.x > 0.0 and m.color.a > 0.0


def test_markerarray_empty_segments_is_valid():
    arr = self_built_wall_markerarray([], frame_id='map')
    assert len(arr.markers) == 1
    assert arr.markers[0].points == []
```

- [ ] **Step 3: Run to verify failure**

Run: `python -m pytest src/tugbot_maze/test/test_flood_fill_viz.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'tugbot_maze.flood_fill_viz'`.

- [ ] **Step 4: Implement**

Create `src/tugbot_maze/tugbot_maze/flood_fill_viz.py`:

```python
"""RViz visualization for the online self-built map: turn confirmed wall segments into a
MarkerArray so RViz shows the map the robot actually built (perimeter + confirmed interior
walls), rather than slam_toolbox's occupancy grid. Pure builder — no ROS node state."""
from __future__ import annotations
from typing import Iterable, Optional, Tuple

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

Segment = Tuple[float, float, float, float]


def self_built_wall_markerarray(segments: Iterable[Segment], frame_id: str = 'map',
                                stamp=None, ns: str = 'self_built_walls') -> MarkerArray:
    """A single green LINE_LIST marker with two points per wall segment, in `frame_id`."""
    m = Marker()
    m.header.frame_id = frame_id
    if stamp is not None:
        m.header.stamp = stamp
    m.ns = ns
    m.id = 0
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.scale.x = 0.05
    m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
    m.pose.orientation.w = 1.0
    for x0, y0, x1, y1 in segments:
        m.points.append(Point(x=float(x0), y=float(y0), z=0.0))
        m.points.append(Point(x=float(x1), y=float(y1), z=0.0))
    return MarkerArray(markers=[m])
```

- [ ] **Step 5: Run to verify pass**

Run: `python -m pytest src/tugbot_maze/test/test_flood_fill_viz.py -q`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/tugbot_maze/tugbot_maze/flood_fill_viz.py src/tugbot_maze/test/test_flood_fill_viz.py src/tugbot_maze/package.xml
git commit -F - <<'EOF'
feat: pure MarkerArray builder for the online self-built wall map

Green LINE_LIST of perimeter + confirmed interior walls for RViz.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task 3: Solver publishes `map→odom` from the ICP pose (online_slam)

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/flood_fill_solver.py` (imports, `__init__`, `_control_tick`, new `_publish_map_to_odom`)

This task adds all `__init__` plumbing (entrance params, TF broadcaster, walls publisher, perimeter cache) plus the `map→odom` publish. The bootstrap seed (Task 4) and marker publish (Task 5) reuse this plumbing.

- [ ] **Step 1: Extend imports**

In `src/tugbot_maze/tugbot_maze/flood_fill_solver.py`, replace the geometry import and the pose_tracking import, and add two imports:

```python
from geometry_msgs.msg import Twist, TransformStamped
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy
```
```python
from tugbot_maze.pose_tracking import (
    compose_2d, quat_to_yaw, odom_prior, inverse_2d, yaw_to_quat, map_to_odom)
from tugbot_maze.flood_fill_viz import self_built_wall_markerarray
```

- [ ] **Step 2: Add entrance params + plumbing in `__init__`**

After the `self.startup_delay_sec` param line (currently `flood_fill_solver.py:51`), add:

```python
        self.entrance_x = float(self.declare_parameter('entrance_x', 0.0).value)
        self.entrance_y = float(self.declare_parameter('entrance_y', 0.0).value)
        self.entrance_yaw = float(self.declare_parameter('entrance_yaw', 0.0).value)
        self._entrance_anchor = (self.entrance_x, self.entrance_y, self.entrance_yaw)
```

After the `self.tf_listener = ...` line (currently `flood_fill_solver.py:90`), add:

```python
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        _walls_qos = QoSProfile(depth=1)
        _walls_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL   # latch for late RViz joins
        self.walls_pub = self.create_publisher(MarkerArray, '/maze/self_built_walls', _walls_qos)
        self._perimeter_segments = outer_segments() if self.pose_source == 'online_slam' else []
        self._walls_key = None
```

- [ ] **Step 3: Add the `_publish_map_to_odom` method**

Add this method to `FloodFillSolver` (e.g., right after `_publish_cmd`):

```python
    def _publish_map_to_odom(self):
        """online_slam: own the map->odom transform from the ICP pose so RViz + the exit
        monitor see the pose the robot actually uses (slam_toolbox is silenced on this TF)."""
        if self.pose_source != 'online_slam':
            return
        if self._sm_corrected is None or self._sm_last_odom is None:
            return
        ox, oy, oyaw = map_to_odom(self._sm_corrected, self._sm_last_odom)
        tmsg = TransformStamped()
        tmsg.header.stamp = self.get_clock().now().to_msg()
        tmsg.header.frame_id = self.map_frame
        tmsg.child_frame_id = self.odom_frame
        tmsg.transform.translation.x = float(ox)
        tmsg.transform.translation.y = float(oy)
        tmsg.transform.translation.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(oyaw)
        tmsg.transform.rotation.x = qx
        tmsg.transform.rotation.y = qy
        tmsg.transform.rotation.z = qz
        tmsg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tmsg)
```

- [ ] **Step 4: Call it every control tick**

In `_control_tick`, immediately after `pose = self._lookup_pose()` (currently `flood_fill_solver.py:185`) and before the `if self.phase == 'done':` line, insert:

```python
        self._publish_map_to_odom()
```

(Placed before the `done` early-return so the TF keeps publishing after the robot parks.)

- [ ] **Step 5: Verify the module imports cleanly and the suite is green**

Run:
```bash
python -c "import tugbot_maze.flood_fill_solver" && echo IMPORT_OK
python -m pytest src/tugbot_maze/test/ -q
```
Expected: `IMPORT_OK`, and the unit suite passes (no import/syntax/NameError regressions). Runtime TF behavior is validated in Task 9.

- [ ] **Step 6: Commit**

```bash
git add src/tugbot_maze/tugbot_maze/flood_fill_solver.py
git commit -F - <<'EOF'
feat: solver owns map->odom from the ICP pose (online_slam)

Publishes map->odom = compose(_sm_corrected, inverse(odom->base)) every tick
so RViz and the exit monitor see the true ICP pose. Gated on online_slam;
adds TF broadcaster, entrance params, and the self-built-walls publisher.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task 4: Bootstrap the ICP pose from the known entrance (online_slam)

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/flood_fill_solver.py` (`_lookup_scan_match` bootstrap branch)

With slam_toolbox's `map→odom` silenced in `online_slam`, `map→base` TF is gone, so the solver can no longer seed `_sm_corrected` from it. Seed from the known entrance anchor instead. `scan_match` keeps its existing slam_toolbox bootstrap.

- [ ] **Step 1: Replace the bootstrap branch**

In `_lookup_scan_match`, replace the first bootstrap block (currently `flood_fill_solver.py:140-145`):

```python
        if self.phase in ('startup', 'entering') or self.scan_msg is None or odom_base is None:
            if map_base is not None:
                self._sm_corrected = map_base
                self._sm_last_odom = odom_base
                self._sm_seq = self._scan_seq
            return map_base if map_base is not None else self._sm_corrected
```

with:

```python
        if self.phase in ('startup', 'entering') or self.scan_msg is None or odom_base is None:
            if self.pose_source == 'online_slam':
                # No slam_toolbox map->base (its TF is silenced); anchor on the known entrance
                # and let wheel odometry carry the pose until ICP corrections begin (driving).
                if odom_base is not None:
                    self._sm_corrected = compose_2d(self._entrance_anchor, odom_base)
                    self._sm_last_odom = odom_base
                    self._sm_seq = self._scan_seq
                return self._sm_corrected
            if map_base is not None:                       # scan_match: unchanged
                self._sm_corrected = map_base
                self._sm_last_odom = odom_base
                self._sm_seq = self._scan_seq
            return map_base if map_base is not None else self._sm_corrected
```

- [ ] **Step 2: Verify the module imports cleanly and the suite is green**

Run:
```bash
python -c "import tugbot_maze.flood_fill_solver" && echo IMPORT_OK
python -m pytest src/tugbot_maze/test/ -q
```
Expected: `IMPORT_OK` and unit suite green. The entrance-seed behavior is validated end-to-end in Task 9 (and the equivalent "seed from the true start pose" is already what the offline sim in Task 8 exercises).

- [ ] **Step 3: Commit**

```bash
git add src/tugbot_maze/tugbot_maze/flood_fill_solver.py
git commit -F - <<'EOF'
feat: online_slam bootstrap from the known entrance anchor

With slam_toolbox's map->odom silenced, seed the ICP pose from entrance_*
and propagate on wheel odom until ICP corrections start. scan_match unchanged.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task 5: Publish the growing self-built wall map (online_slam)

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/flood_fill_solver.py` (new `_publish_self_built_walls`, call in driving branch)

- [ ] **Step 1: Add the method**

Add to `FloodFillSolver` (near `_publish_map_to_odom`):

```python
    def _publish_self_built_walls(self):
        """online_slam: publish perimeter + all confirmed interior walls as a MarkerArray so
        RViz shows the map the robot built. Republished only when the sensed/committed set
        grows (cheap change-key), and never allowed to kill the node."""
        if self.pose_source != 'online_slam':
            return
        key = (len(self.motion.sensed), len(self.motion.committed))
        if key == self._walls_key:
            return
        self._walls_key = key
        try:
            cells = self.motion.sensed | self.motion.committed
            segs = list(self._perimeter_segments) + confirmed_wall_segments(self.brain, cells)
            arr = self_built_wall_markerarray(
                segs, frame_id=self.map_frame, stamp=self.get_clock().now().to_msg())
            self.walls_pub.publish(arr)
        except Exception as e:                       # viz must never crash the solver
            self.get_logger().warning('self-built walls marker publish failed: %r' % e)
```

- [ ] **Step 2: Call it in the driving branch**

In `_control_tick`'s `driving` branch, after the `update_junctions(...)` block and the `JUNCTION` log (currently around `flood_fill_solver.py:220-226`), before the `if done and self.phase != 'done':` check, insert:

```python
            self._publish_self_built_walls()
```

- [ ] **Step 3: Verify the module imports cleanly and the suite is green**

Run:
```bash
python -c "import tugbot_maze.flood_fill_solver" && echo IMPORT_OK
python -m pytest src/tugbot_maze/test/ -q
```
Expected: `IMPORT_OK`, suite green.

- [ ] **Step 4: Commit**

```bash
git add src/tugbot_maze/tugbot_maze/flood_fill_solver.py
git commit -F - <<'EOF'
feat: publish the growing self-built wall map as a MarkerArray (online_slam)

Perimeter + confirmed interior walls on /maze/self_built_walls, republished
on set growth, guarded so viz never crashes the solver.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task 6: Launch wiring — entrance params + silence slam_toolbox for online_slam

**Files:**
- Create: `src/tugbot_navigation/config/slam_toolbox_params_online_slam.yaml`
- Modify: `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`

- [ ] **Step 1: Create the silenced slam_toolbox variant**

Copy the base params and set `transform_publish_period: 0.0` (primary silencing mechanism; the Task 9 empirical check confirms it, with a documented fallback):

```bash
cp src/tugbot_navigation/config/slam_toolbox_params.yaml \
   src/tugbot_navigation/config/slam_toolbox_params_online_slam.yaml
```

Then edit `src/tugbot_navigation/config/slam_toolbox_params_online_slam.yaml`: change the line
`transform_publish_period: 0.02` to `transform_publish_period: 0.0` and add a comment line above it:
```yaml
    # online_slam: the flood_fill_solver owns map->odom; slam_toolbox keeps mapping but must
    # not publish the transform (would conflict). 0.0 disables the TF broadcast thread.
    transform_publish_period: 0.0
```

- [ ] **Step 2: Wire entrance_* into the flood_fill node**

In `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`, in the `flood_fill_solver_node` `parameters` dict (currently `flood_fill_solver.py` launch block at lines 205-213), add these three entries alongside `pose_source`:

```python
                     'entrance_x': ParameterValue(LaunchConfiguration('entrance_x'), value_type=float),
                     'entrance_y': ParameterValue(LaunchConfiguration('entrance_y'), value_type=float),
                     'entrance_yaw': ParameterValue(LaunchConfiguration('entrance_yaw'), value_type=float),
```

- [ ] **Step 3: Conditionally select the slam params for online_slam**

In the same file, just before the `maze_slam_nav_launch = IncludeLaunchDescription(` block (currently line 70), add:

```python
    online_slam_slam_params = os.path.join(navigation_share, 'config', 'slam_toolbox_params_online_slam.yaml')
    slam_params_selected = PythonExpression([
        "'", online_slam_slam_params, "' if '", LaunchConfiguration('pose_source'),
        "' == 'online_slam' else '", LaunchConfiguration('slam_params_file'), "'"])
```

Then inside the `maze_slam_nav_launch` include's `launch_arguments`, change the `slam_params_file` entry (currently line 74) from
`'slam_params_file': LaunchConfiguration('slam_params_file'),`
to:
```python
            'slam_params_file': slam_params_selected,
```

(`PythonExpression`, `LaunchConfiguration`, `os`, and `navigation_share` are already imported/defined in this file.)

- [ ] **Step 4: Build and verify the launch parses**

Run:
```bash
colcon build --symlink-install --packages-select tugbot_maze tugbot_bringup tugbot_navigation >/dev/null 2>&1 && echo BUILD_OK
source install/setup.bash
python -c "import yaml; yaml.safe_load(open('src/tugbot_navigation/config/slam_toolbox_params_online_slam.yaml')); print('YAML_OK')"
ros2 launch tugbot_bringup tugbot_maze_explore.launch.py --show-args >/dev/null 2>&1 && echo LAUNCH_ARGS_OK
```
Expected: `BUILD_OK`, `YAML_OK`, `LAUNCH_ARGS_OK`.

- [ ] **Step 5: Commit**

```bash
git add src/tugbot_navigation/config/slam_toolbox_params_online_slam.yaml src/tugbot_bringup/launch/tugbot_maze_explore.launch.py
git commit -F - <<'EOF'
feat: launch wiring for truthful online_slam RViz

Wire entrance_x/y/yaw into the flood_fill node and, for pose_source=online_slam,
select a slam_toolbox params variant with transform_publish_period=0.0 so the
solver owns map->odom. Other modes unchanged.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task 7: RViz — show the self-built walls, hide slam_toolbox's `/map`

**Files:**
- Modify: `src/tugbot_bringup/config/tugbot_nav.rviz`
- Modify: `src/tugbot_bringup/rviz/tugbot_nav.rviz`

Apply the **same two edits to both files** (they are mirrored copies).

- [ ] **Step 1: Add the SelfBuiltWalls display**

In each file, inside the `Displays:` list (add as a new list item alongside the other displays, matching the surrounding indentation — the existing displays are indented 6 spaces under `Displays:`):

```yaml
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: SelfBuiltWalls
      Namespaces:
        {}
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /maze/self_built_walls
      Value: true
```

(Durability `Transient Local` matches the latched publisher so the map shows even if RViz connects late.)

- [ ] **Step 2: Disable the slam_toolbox Map display**

In each file, find the display block `Class: rviz_default_plugins/Map` (its `Topic` `Value` is `/map`) and change its `Enabled: true` to `Enabled: false`. Leave the rest of the block intact so it can be toggled back on for comparison.

- [ ] **Step 3: Verify both files are still valid YAML**

Run:
```bash
python -c "import yaml; yaml.safe_load(open('src/tugbot_bringup/config/tugbot_nav.rviz')); yaml.safe_load(open('src/tugbot_bringup/rviz/tugbot_nav.rviz')); print('RVIZ_YAML_OK')"
```
Expected: `RVIZ_YAML_OK`.

- [ ] **Step 4: Commit**

```bash
git add src/tugbot_bringup/config/tugbot_nav.rviz src/tugbot_bringup/rviz/tugbot_nav.rviz
git commit -F - <<'EOF'
feat: RViz shows the self-built wall map, hides slam_toolbox /map

Add a SelfBuiltWalls MarkerArray display (latched) and disable the /map
Map display by default (toggleable for comparison).

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task 8: Offline regression gate (navigation unchanged)

**Files:**
- Run only: `src/tugbot_maze/test/test_online_slam_sim.py` + full unit suite

- [ ] **Step 1: Rebuild and run the full unit + offline suite**

Run:
```bash
colcon build --symlink-install --packages-select tugbot_maze tugbot_bringup tugbot_navigation >/dev/null 2>&1 && echo BUILD_OK
source install/setup.bash
python -m pytest src/tugbot_maze/test/test_online_slam_sim.py src/tugbot_maze/test/test_pose_tracking.py src/tugbot_maze/test/test_flood_fill_viz.py -q
python -m pytest src/tugbot_maze/test/ -q
```
Expected: `BUILD_OK`; `test_online_slam_sim.py` both tests PASS (reaches exit, no collision, `max_loc_err < 0.5`) — proving the control loop is unchanged; new unit tests PASS; the full suite has no new failures vs. the pre-change baseline.

- [ ] **Step 2: Commit (if any incidental fixes were needed)**

If the suite required a fix, commit it:
```bash
git add -A
git commit -F - <<'EOF'
test: green offline + unit suite after truthful-RViz changes

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```
If nothing changed, skip the commit.

---

### Task 9: Gazebo acceptance (user-gated authority)

**Files:** none (validation only). **Do NOT auto-launch** — wait for the user's "set up the run".

- [ ] **Step 1: Visual run with the truthful view**

```bash
tools/run_flood_fill_maze.sh 1000 false true online_slam
```
Watch: robot + scan + green self-built walls stay coherent through the maze-center turn (the wobble is gone); the `/map` grid is hidden by default.

- [ ] **Step 2: Confirm slam_toolbox's `map→odom` is silenced (the flagged risk)**

While the run is up, in another sourced shell:
```bash
ros2 run tf2_ros tf2_echo map odom
```
Expected: transforms arrive (published by the solver) and are stable/grid-locked. Then confirm the *solver* is the only publisher — `ros2 topic echo /tf --once` should show `map→odom` with the solver's values; slam_toolbox should not be fighting it (no jitter).
**Fallback if slam_toolbox still publishes `map→odom`** (TF conflict / jitter persists): in `slam_toolbox_params_online_slam.yaml`, instead of `transform_publish_period: 0.0`, set `map_frame: map_slam` (slam_toolbox then publishes `map_slam→odom`, no conflict; its `/map` is hidden anyway). Rebuild and re-run.

- [ ] **Step 3: Confirm no regression**

After the run finishes (`EXIT_REACHED`), replay the collision oracle over the run's poses:
```bash
python3 tools/replay_collision_oracle.py "$(ls -td log/flood_fill_run_* | head -1)"
```
Expected: `EXIT_REACHED` reached; oracle `rate=0.000%` (0 collisions); completion time ~545–565 s (unchanged vs the banked baseline). `EXIT_REACHED` now fires off the solver's ICP pose (final pose ≈ (20.2, 18.0), within the 1.2 m exit radius).

- [ ] **Step 4: Report results** to the user (no commit — validation only).

---

## Self-Review

**1. Spec coverage:**
- Solver owns `map→odom` → Task 3. ✓
- Entrance bootstrap (slam silenced) → Task 4. ✓
- Self-built walls MarkerArray (all accumulated confirmed walls) → Task 2 (builder) + Task 5 (publish). ✓
- Silence slam_toolbox for online_slam + wire entrance_* → Task 6. ✓
- RViz: add SelfBuiltWalls, `/map` disabled-but-present → Task 7. ✓
- Exit monitor keys off ICP pose → falls out of Task 3 (solver owns the TF the monitor reads); verified in Task 9 Step 3. ✓
- Error handling (skip TF when unseeded/None; guarded marker publish) → Tasks 3, 5. ✓
- Tests: unit (Tasks 1, 2), offline regression (Task 8), Gazebo authority (Task 9). ✓
- Known risk (transform_publish_period=0.0) with fallback → Task 6 Step 1 + Task 9 Step 2. ✓
- Scope gating to online_slam only → every solver/launch change is guarded on `pose_source == 'online_slam'`; scan_match paths untouched. ✓

**2. Placeholder scan:** No TBD/TODO; every code step shows complete code; every command has expected output.

**3. Type consistency:** `map_to_odom(map_to_base, odom_to_base)` and `yaw_to_quat(yaw)` (Task 1) are used with matching signatures in Task 3. `self_built_wall_markerarray(segments, frame_id, stamp, ns)` (Task 2) is called with matching kwargs in Task 5. `_perimeter_segments`, `_walls_key`, `_entrance_anchor`, `tf_broadcaster`, `walls_pub` are all defined in Task 3 Step 2 before use in Tasks 4/5. `confirmed_wall_segments` and `outer_segments` are already imported in `flood_fill_solver.py`. `self.motion.sensed`/`.committed` are sets (confirmed). ✓
