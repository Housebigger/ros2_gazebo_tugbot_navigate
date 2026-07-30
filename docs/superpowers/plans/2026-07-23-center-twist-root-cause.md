# center-twist-root-cause Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Diagnose the small-amplitude fan/oblique wall twist in the accumulated 3D map at the maze center corridor with a discriminating gt-parallel-accumulator A/B, then fix per evidence in the same phase.

**Architecture:** New workspace `ros2_ws_tugbot_nav_20260728` (clone of 20260727), branch `center-twist-root-cause`. Three eval-only instruments (gt parallel accumulator node, cloud-capture + twist-metric offline tools, POSEDIAG bucketing analysis) + a diag-variant run script, two instrumented headless runs, a written verdict, then an evidence-named fix (solver gate layer preferred) re-verified with the same instruments. Localizers, production accumulator, projector chain: byte-frozen.

**Tech Stack:** ROS2 Jazzy (rclpy), numpy, pytest; gz Harmonic sim unchanged.

**Spec:** `docs/superpowers/specs/2026-07-23-center-twist-root-cause-design.md` (user adjudications: fan-twist phenomenology = yaw-first two-sided forensics; diagnose+fix one phase with honest downgrade).

---

## Standing process constraints (bind every task)

- Explicit `cd` to absolute paths every command block; `set +u` before sourcing ROS setup.bash; only `python3`.
- Build ONLY from inside the workspace: `colcon build --symlink-install`.
- Authoritative suite = main-session workspace-root run over BOTH test dirs: `python3 -m pytest src/tugbot_maze/test/ src/tugbot_bringup/test/ -q`. Baseline: **10 failed / 544 passed** (frozen quartet + 6 era-4 wall_follow, all by name in the tugbot-return record).
- `git add` explicit paths only; commits via heredoc `git commit -F - <<'EOF'`, NO backticks, both trailer lines (Co-Authored-By: Claude Fable 5 <noreply@anthropic.com> + Claude-Session URL).
- Subagents: ALL commands FOREGROUND. Long sim runs (Task 4/6) are MAIN-SESSION only.
- RED LINE: `/gt/dynamic_pose` feeds ONLY the eval instruments in this plan. Never control. `/maze/gt_cloud_map_3d` may be consumed by RViz/analysis only.
- FROZEN files (byte-zero-touch): both localizers, `scan_slice_projector.py`, `slice_to_scan.py`, `maze_perception.py`, `flood_fill_brain.py`, `wall_localize.py`, `cloud_map_accumulator.py`, `cloud_map_3d.py`, `maze_motion.py`, `hop_controller.py`, `ackermann_maneuvers.py`, `footprint.py`, all model SDFs, the world file. (`flood_fill_solver.py` may change ONLY in Task 5 if the verdict names it.)
- Coordinate anchors (from `maze_20260528_scaled_instance.yaml`): map = world + (11.011281, 9.02507); map entrance (0,0), map exit (21.072562, 18.083566). Maze center ≈ map (10.54, 9.04).

## File map

| File | Action | Responsibility |
|---|---|---|
| `ros2_ws_tugbot_nav_20260728/` | create (Task 0) | phase workspace |
| `src/tugbot_maze/tugbot_maze/gt_cloud_accumulator.py` | create (Task 1) | gt parallel accumulator (eval-only node) |
| `src/tugbot_maze/test/test_gt_cloud_accumulator.py` | create (Task 1) | pure-helper unit tests |
| `src/tugbot_maze/setup.py` | modify (Task 1) | console_script entry |
| `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py` | modify (Task 1) | `gt_cloud_diag` arg + gated Node |
| `tools/capture_cloud_map.py` | create (Task 2) | one-shot latched-map capture → .npy |
| `tools/map_twist_metric.py` | create (Task 2) | per-wall-band spread/angle metric, production vs gt |
| `src/tugbot_maze/test/test_map_twist_metric.py` | create (Task 2) | metric math unit tests |
| `tools/run_center_twist_diag.sh` | create (Task 2) | diag run variant (instruments on, capture BEFORE kill) |
| `tools/center_twist_decompose.py` | create (Task 3) | POSEDIAG residual bucketing (region/source) |
| `src/tugbot_maze/test/test_center_twist_decompose.py` | create (Task 3) | parser/bucketing unit tests |
| (Task 5: file named by the verdict — solver gate layer preferred) | modify | evidence-named fix |
| `docs/superpowers/specs/2026-07-23-center-twist-root-cause-design.md` | append (Tasks 4/6) | verdict + results addendum |

Note on tool placement: `tools/` = `ros2_ws_tugbot_nav_20260728/tools/`.

---

### Task 0: Workspace clone, branch, baseline

**Files:** Create `ros2_ws_tugbot_nav_20260728/` (whole tree).

- [ ] **Step 1: Branch off main**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git checkout -b center-twist-root-cause
```

- [ ] **Step 2: Clone the workspace**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
rsync -a --exclude build --exclude install --exclude log --exclude __pycache__ --exclude .pytest_cache \
  ros2_ws_tugbot_nav_20260727/ ros2_ws_tugbot_nav_20260728/
```

- [ ] **Step 3: Build**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260728
set +u; source /opt/ros/jazzy/setup.bash; set -u
colcon build --symlink-install
```

Expected: 6 packages, no errors.

- [ ] **Step 4: Baseline suite**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260728
set +u; source /opt/ros/jazzy/setup.bash; source install/setup.bash; set -u
python3 -m pytest src/tugbot_maze/test/ src/tugbot_bringup/test/ -q 2>&1 | tail -15
```

Expected: exactly `10 failed, 544 passed`; failures = the frozen quartet + the 6
era-4 wall_follow records. Any other outcome: STOP, report BLOCKED with the tail.

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260728
git commit -F - <<'EOF'
chore: new workspace ros2_ws_tugbot_nav_20260728 (clone of 20260727)

center-twist-root-cause phase workspace. Baseline suite on the fresh
clone: 10 failed / 544 passed / 0 xfailed (frozen quartet + 6 era-4
wall_follow records, names unchanged from the tugbot-return record).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01AVWsj4MmGMqfFqXeUuanUC
EOF
```

---

### Task 1: GT parallel accumulator (node + tests + wiring)

**Files:**
- Create: `ros2_ws_tugbot_nav_20260728/src/tugbot_maze/tugbot_maze/gt_cloud_accumulator.py`
- Create: `ros2_ws_tugbot_nav_20260728/src/tugbot_maze/test/test_gt_cloud_accumulator.py`
- Modify: `ros2_ws_tugbot_nav_20260728/src/tugbot_maze/setup.py` (one entry_points line)
- Modify: `ros2_ws_tugbot_nav_20260728/src/tugbot_bringup/launch/tugbot_maze_explore.launch.py` (arg + Node)

- [ ] **Step 1: Write the failing unit tests**

Create `src/tugbot_maze/test/test_gt_cloud_accumulator.py`:

```python
"""Unit tests for the gt parallel accumulator's PURE helpers (the node wraps
them; sim integration is exercised by the Task 4 diag runs). Pins the A/B
fairness rules: nearest-stamp pairing with the production TF timeout as the
max pairing distance, and the exact world->map compose with the lidar mount."""
import math

import numpy as np
import pytest

from tugbot_maze.gt_cloud_accumulator import (
    nearest_gt_sample, compose_map_sensor, MAX_PAIR_DT_S,
    WORLD_TO_MAP_X, WORLD_TO_MAP_Y, LIDAR_MOUNT_Z)


def test_pairing_constants_mirror_production():
    # Production accumulator: TF lookup timeout 0.05 s at the cloud stamp.
    # The gt instrument must not be allowed a LOOSER pairing window.
    assert MAX_PAIR_DT_S == 0.05
    # Coordinate anchors from maze_20260528_scaled_instance.yaml.
    assert WORLD_TO_MAP_X == pytest.approx(11.011281)
    assert WORLD_TO_MAP_Y == pytest.approx(9.02507)
    assert LIDAR_MOUNT_Z == pytest.approx(0.55)   # tugbot contract mount


def test_nearest_gt_sample_picks_closest():
    buf = [(1.00, 'a'), (1.10, 'b'), (1.20, 'c')]
    assert nearest_gt_sample(buf, 1.12) == (1.10, 'b')
    assert nearest_gt_sample(buf, 1.19) == (1.20, 'c')


def test_nearest_gt_sample_rejects_beyond_max_dt():
    buf = [(1.00, 'a')]
    assert nearest_gt_sample(buf, 1.00 + MAX_PAIR_DT_S + 0.001) is None
    assert nearest_gt_sample(buf, 1.00 + MAX_PAIR_DT_S - 0.001) == (1.00, 'a')


def test_nearest_gt_sample_empty_buffer():
    assert nearest_gt_sample([], 1.0) is None


def test_compose_identity_pose_maps_origin():
    # Robot at world origin, identity rotation: a sensor-frame point (0,0,0)
    # lands at map (offset_x, offset_y, LIDAR_MOUNT_Z).
    T = compose_map_sensor(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    p = T @ np.array([0.0, 0.0, 0.0, 1.0])
    assert p[0] == pytest.approx(WORLD_TO_MAP_X)
    assert p[1] == pytest.approx(WORLD_TO_MAP_Y)
    assert p[2] == pytest.approx(LIDAR_MOUNT_Z)


def test_compose_yawed_pose_rotates_sensor_points():
    # Robot at world (1, 2, 0), yaw +90deg: sensor +x becomes map +y.
    yaw = math.pi / 2
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    T = compose_map_sensor(1.0, 2.0, 0.0, 0.0, 0.0, qz, qw)
    p = T @ np.array([2.0, 0.0, 0.0, 1.0])          # 2 m ahead of the sensor
    assert p[0] == pytest.approx(1.0 + WORLD_TO_MAP_X, abs=1e-6)
    assert p[1] == pytest.approx(2.0 + WORLD_TO_MAP_Y + 2.0, abs=1e-6)
    assert p[2] == pytest.approx(LIDAR_MOUNT_Z, abs=1e-6)
```

- [ ] **Step 2: Run to verify failure**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260728
python3 -m pytest src/tugbot_maze/test/test_gt_cloud_accumulator.py -q
```

Expected: FAIL — `ModuleNotFoundError`/`ImportError` (module does not exist).

- [ ] **Step 3: Write the node**

Create `src/tugbot_maze/tugbot_maze/gt_cloud_accumulator.py`:

```python
#!/usr/bin/env python3
"""GT parallel accumulator (center-twist-root-cause, EVAL-ONLY, default OFF).

Accumulates the SAME /lidar/points clouds as the production
cloud_map_accumulator, but transformed by the GROUND-TRUTH pose from
/gt/dynamic_pose instead of the believed map->base TF. One run yields two
same-scale maps: if the gt map is clean while the production map twists, the
twist is 100% believed-pose error; if both twist, localization is exonerated
and the cause sits in the sensing/timing layer.

RED LINE: ground truth enters ONLY this instrument. It publishes nothing any
control or navigation node consumes (/maze/gt_cloud_map_3d is latched for
optional RViz inspection, at most one publish per 30 s -- Heisenberg guard:
the ghost-forensics lesson is that instrument load alters the phenomenon).

A/B fairness rules (the comparison is void without them):
- Same CloudMap3D parameters as production (0.05 m voxel, range <= 8 m,
  z-clip [-1, 3]) -- CloudMap3D() defaults, byte-shared code path.
- Same staleness discipline: frames older than 0.5 s are dropped (mirrors the
  production accumulator's stale-drop).
- Pairing window: nearest gt sample to the CLOUD stamp, |dt| <= 0.05 s -- the
  production TF lookup timeout. gt arrives at ~58 Hz vs 10 Hz clouds, so the
  typical pairing distance is <= ~9 ms.
- /gt/dynamic_pose frame ids are EMPTY (ros_gz_bridge drops gz names --
  tugbot-return finding): transforms[0] is the tugbot (the maze world's only
  non-static top-level model). Caveat: a second top-level dynamic model would
  break the fixed-index assumption.

The final voxel set is dumped as an .npy (N x 3 float32, map frame) into
dump_dir at shutdown for offline comparison by tools/map_twist_metric.py."""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage

from tugbot_maze.cloud_map_3d import CloudMap3D, should_publish, transform_to_matrix

# Coordinate anchors (maze_20260528_scaled_instance.yaml): map = world + offset.
WORLD_TO_MAP_X = 11.011281
WORLD_TO_MAP_Y = 9.02507
LIDAR_MOUNT_Z = 0.55          # tugbot contract: lidar_3d at base_link +0.55 (0 0 0.55)
MAX_PAIR_DT_S = 0.05          # == the production accumulator's TF lookup timeout
STALE_S = 0.5                 # == the production accumulator's stale-drop bound
GT_BUFFER_S = 2.0             # rolling gt sample window
TOPIC_PUB_MIN_PERIOD_S = 30.0  # Heisenberg guard on the latched publish


def nearest_gt_sample(buffer, t_s):
    """Nearest-stamp sample from [(t_s, payload), ...]; None if empty or the
    nearest is farther than MAX_PAIR_DT_S (fairness: no looser than the
    production TF pairing window)."""
    if not buffer:
        return None
    best = min(buffer, key=lambda item: abs(item[0] - t_s))
    if abs(best[0] - t_s) > MAX_PAIR_DT_S:
        return None
    return best


def compose_map_sensor(x, y, z, qx, qy, qz, qw):
    """T_map_sensor = Trans(world->map) @ T_world_base(gt) @ T_base_sensor.
    The gz model pose IS the base_link pose (model root); the sensor mount is
    a pure +z translation per the tugbot contract (lidar_3d pose 0 0 0.55)."""
    T_offset = transform_to_matrix(WORLD_TO_MAP_X, WORLD_TO_MAP_Y, 0.0,
                                   0.0, 0.0, 0.0, 1.0)
    T_world_base = transform_to_matrix(x, y, z, qx, qy, qz, qw)
    T_base_sensor = transform_to_matrix(0.0, 0.0, LIDAR_MOUNT_Z,
                                        0.0, 0.0, 0.0, 1.0)
    return T_offset @ T_world_base @ T_base_sensor


class GtCloudAccumulator(Node):
    def __init__(self):
        super().__init__('gt_cloud_accumulator')
        self.dump_dir = self.declare_parameter('dump_dir', '').value
        self._map = CloudMap3D()               # SAME defaults as production
        self._gt_buf = []                      # [(t_s, (x,y,z,qx,qy,qz,qw))]
        self._last_log_s = None
        self._last_topic_pub_s = None
        self._stale_dropped = 0
        self._unpaired = 0
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(PointCloud2, '/maze/gt_cloud_map_3d', qos)
        self.create_subscription(TFMessage, '/gt/dynamic_pose', self._gt_cb, 50)
        self.create_subscription(PointCloud2, '/lidar/points', self._cloud_cb, 5)
        self.get_logger().info('gt_cloud_accumulator started (EVAL-ONLY).')

    def _gt_cb(self, msg):
        # transforms[0] = the tugbot model root (bridge drops gz names; index
        # convention per the tugbot-return record). RECEIPT-TIME pairing: the
        # bridged per-transform header stamps are CONSTANT ZERO (measured in
        # the first diag run: voxels=0, unpaired=all, gt_buf unbounded) -- the
        # node clock (sim domain under use_sim_time, same domain as cloud
        # stamps) is the only usable stamp. Jitter at ~58 Hz transport is ~ms,
        # well inside the 0.05 s window. Fairness note: production pairs at
        # the cloud CAPTURE stamp via tf2 interpolation; this instrument pairs
        # cloud capture stamp vs gt RECEIPT stamp.
        if not msg.transforms:
            return
        t0 = msg.transforms[0]
        t_s = self.get_clock().now().nanoseconds / 1e9
        tr, q = t0.transform.translation, t0.transform.rotation
        self._gt_buf.append((t_s, (tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)))
        cutoff = t_s - GT_BUFFER_S
        while self._gt_buf and self._gt_buf[0][0] < cutoff:
            self._gt_buf.pop(0)

    def _cloud_cb(self, cloud):
        try:
            now = self.get_clock().now()
            t_cloud = Time.from_msg(cloud.header.stamp).nanoseconds / 1e9
            if (now.nanoseconds / 1e9) - t_cloud > STALE_S:
                self._stale_dropped += 1
                return
            paired = nearest_gt_sample(self._gt_buf, t_cloud)
            if paired is None:
                self._unpaired += 1
                return
            T = compose_map_sensor(*paired[1])
            pts = point_cloud2.read_points_numpy(
                cloud, field_names=('x', 'y', 'z'), skip_nans=False)
            added = self._map.add_cloud(pts, T)
            now_s = now.nanoseconds / 1e9
            if should_publish(self._last_log_s, now_s, added, 1.0):
                self._last_log_s = now_s
                self.get_logger().info(
                    'GTCLOUD voxels=%d stale_dropped=%d unpaired=%d'
                    % (len(self._map), self._stale_dropped, self._unpaired))
                if (self._last_topic_pub_s is None
                        or now_s - self._last_topic_pub_s >= TOPIC_PUB_MIN_PERIOD_S):
                    self.pub.publish(self._map.to_pointcloud2(
                        frame_id='map', stamp=now.to_msg()))
                    self._last_topic_pub_s = now_s
        except Exception as e:                 # eval must never crash or spam
            self.get_logger().error('gt cloud frame dropped: %r' % e,
                                    throttle_duration_sec=5.0)

    def dump(self):
        if not self.dump_dir:
            return
        try:
            pc = self._map.to_pointcloud2(frame_id='map')
            pts = point_cloud2.read_points_numpy(
                pc, field_names=('x', 'y', 'z'), skip_nans=False)
            path = os.path.join(self.dump_dir, 'gt_cloud_map.npy')
            np.save(path, pts.astype(np.float32))
            self.get_logger().info('GTCLOUD dumped %d voxels -> %s'
                                   % (len(pts), path))
        except Exception as e:
            self.get_logger().error('GTCLOUD dump failed: %r' % e)


def main(args=None):
    rclpy.init(args=args)
    node = GtCloudAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.dump()                            # SIGINT teardown path saves the map
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Run unit tests — green**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260728
python3 -m pytest src/tugbot_maze/test/test_gt_cloud_accumulator.py -q
```

Expected: 6 passed.

- [ ] **Step 5: Register the console script**

In `src/tugbot_maze/setup.py`, add to `console_scripts` (after the
cloud_map_accumulator line):

```python
            'gt_cloud_accumulator = tugbot_maze.gt_cloud_accumulator:main',
```

- [ ] **Step 6: Wire the launch arg + gated Node**

In `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`:

1. Next to the `pose_diag` DeclareLaunchArgument (~line 297), add:

```python
        DeclareLaunchArgument('gt_cloud_diag', default_value='false', description='center-twist forensics: EVAL-ONLY gt parallel accumulator (ground truth NEVER enters control); dumps gt_cloud_map.npy into junction_log_dir at shutdown.'),
```

2. Next to the `cloud_map_accumulator` Node (~line 224), add (verify
`IfCondition` is imported in this file; if not, add
`from launch.conditions import IfCondition`):

```python
    gt_cloud_accumulator = Node(
        package='tugbot_maze', executable='gt_cloud_accumulator', name='gt_cloud_accumulator',
        parameters=[{'use_sim_time': True,
                     'dump_dir': LaunchConfiguration('junction_log_dir')}],
        condition=IfCondition(LaunchConfiguration('gt_cloud_diag')),
        output='screen',
    )
```

3. Add `gt_cloud_accumulator` to the SAME TimerAction group that starts
`cloud_map_accumulator` (~line 367).

- [ ] **Step 7: Rebuild + full suite**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260728
set +u; source /opt/ros/jazzy/setup.bash; set -u
colcon build --symlink-install
set +u; source install/setup.bash; set -u
python3 -m pytest src/tugbot_maze/test/ src/tugbot_bringup/test/ -q 2>&1 | tail -5
```

Expected: `10 failed, 552 passed` (baseline + 8 new unit tests incl. the review-added buffer-trim and z pass-through pins; frozen names unchanged).

- [ ] **Step 8: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260728/src/tugbot_maze/tugbot_maze/gt_cloud_accumulator.py \
        ros2_ws_tugbot_nav_20260728/src/tugbot_maze/test/test_gt_cloud_accumulator.py \
        ros2_ws_tugbot_nav_20260728/src/tugbot_maze/setup.py \
        ros2_ws_tugbot_nav_20260728/src/tugbot_bringup/launch/tugbot_maze_explore.launch.py
git commit -F - <<'EOF'
feat: gt parallel accumulator - the center-twist A/B instrument

EVAL-ONLY node, default off (gt_cloud_diag launch arg): accumulates the
same clouds as production but transformed by the ground-truth pose,
same CloudMap3D parameters, same staleness discipline, pairing window
capped at the production TF timeout (0.05 s). Latched publish at most
every 30 s (Heisenberg guard); final voxel set dumped as npy at
shutdown. Ground truth enters only this instrument - red line intact.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01AVWsj4MmGMqfFqXeUuanUC
EOF
```

---

### Task 2: Capture + twist metric + diag run script

**Files:**
- Create: `ros2_ws_tugbot_nav_20260728/tools/capture_cloud_map.py`
- Create: `ros2_ws_tugbot_nav_20260728/tools/map_twist_metric.py`
- Create: `ros2_ws_tugbot_nav_20260728/src/tugbot_maze/test/test_map_twist_metric.py`
- Create: `ros2_ws_tugbot_nav_20260728/tools/run_center_twist_diag.sh`

- [ ] **Step 1: Write the failing metric unit tests**

Create `src/tugbot_maze/test/test_map_twist_metric.py`:

```python
"""Unit tests for the twist-metric math on synthetic voxel sets: a clean wall
must report near-zero spread and angle; a yaw-twisted copy must report the
injected angle and a larger p95 spread. Imports the analysis functions from
tools/ via a path shim (the tools dir is not a package)."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from map_twist_metric import segment_band_stats, wall_points  # noqa: E402


def _wall(x0, y0, x1, y1, n=400, z=0.8, noise=0.0, seed=7):
    rng = np.random.default_rng(seed)
    t = np.linspace(0.0, 1.0, n)
    xs = x0 + (x1 - x0) * t
    ys = y0 + (y1 - y0) * t
    if noise:
        xs = xs + rng.normal(0, noise, n)
        ys = ys + rng.normal(0, noise, n)
    return np.column_stack([xs, ys, np.full(n, z)]).astype(np.float32)


def test_clean_wall_near_zero_spread_and_angle():
    seg = (10.0, 8.0, 10.0, 11.0)                 # vertical wall x=10
    pts = _wall(*seg, noise=0.01)
    stats = segment_band_stats(pts, seg)
    assert stats['count'] > 300
    assert stats['spread_p95'] < 0.05             # ~voxel scale
    assert abs(stats['angle_deg']) < 1.5


def test_twisted_wall_reports_injected_angle():
    seg = (10.0, 8.0, 10.0, 11.0)
    # rotate the wall 3 degrees about its midpoint -> fan/oblique twist
    pts = _wall(*seg, noise=0.01)
    mid = np.array([10.0, 9.5, 0.0])
    a = math.radians(3.0)
    R = np.array([[math.cos(a), -math.sin(a), 0.0],
                  [math.sin(a), math.cos(a), 0.0],
                  [0.0, 0.0, 1.0]])
    twisted = (pts - mid) @ R.T + mid
    both = np.vstack([pts, twisted.astype(np.float32)])
    stats = segment_band_stats(both, seg)
    assert stats['spread_p95'] > 0.03             # double-image widens the band
    clean = segment_band_stats(pts, seg)
    assert stats['spread_p95'] > clean['spread_p95'] * 1.5


def test_wall_points_selects_band_only():
    seg = (10.0, 8.0, 10.0, 11.0)
    inside = _wall(*seg, n=50)
    far = _wall(12.0, 8.0, 12.0, 11.0, n=50)      # 2 m away: outside the band
    floor = _wall(*seg, n=50, z=0.02)             # below the wall z-band
    sel = wall_points(np.vstack([inside, far, floor]), seg)
    assert 40 <= len(sel) <= 60                   # only the in-band wall survives


def test_single_image_angle_reports_injected_rotation():
    # Angle-channel validity on a SINGLE rotated image. The clean+twisted
    # UNION attenuates the fitted angle ~2x (Task 2 review I2), which is why
    # angle is corroborating-only in the verdict protocol -- this test pins
    # that the channel itself is sound when given an unmixed image.
    seg = (10.0, 8.0, 10.0, 11.0)
    pts = _wall(*seg, noise=0.01)
    mid = np.array([10.0, 9.5, 0.0])
    a = math.radians(3.0)
    R = np.array([[math.cos(a), -math.sin(a), 0.0],
                  [math.sin(a), math.cos(a), 0.0],
                  [0.0, 0.0, 1.0]])
    rotated = ((pts - mid) @ R.T + mid).astype(np.float32)
    stats = segment_band_stats(rotated, seg)
    assert abs(abs(stats['angle_deg']) - 3.0) < 0.8


def test_stats_are_z_datum_invariant():
    # Task 1 quality review I2: the gt and production maps may sit on
    # different z datums. Shifting a cloud by +0.5 m in z and passing the
    # matching ground_z must reproduce the same band statistics.
    seg = (10.0, 8.0, 10.0, 11.0)
    pts = _wall(*seg, noise=0.01)
    shifted = pts + np.array([0.0, 0.0, 0.5], dtype=np.float32)
    a = segment_band_stats(pts, seg, ground_z=0.0)
    b = segment_band_stats(shifted, seg, ground_z=0.5)
    assert a['count'] == b['count']
    assert a['spread_p95'] == pytest.approx(b['spread_p95'], abs=1e-6)
    assert a['angle_deg'] == pytest.approx(b['angle_deg'], abs=1e-6)
```

- [ ] **Step 2: Run to verify failure**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260728
python3 -m pytest src/tugbot_maze/test/test_map_twist_metric.py -q
```

Expected: FAIL (module not found).

- [ ] **Step 3: Write the metric tool**

Create `tools/map_twist_metric.py`:

```python
#!/usr/bin/env python3
"""Center-region wall-twist metric (center-twist-root-cause).

Loads one or two dumped voxel sets (.npy, N x 3, map frame) and, for every
ground-truth wall segment intersecting the configured center box, reports the
wall-band statistics: voxel count, perpendicular spread (p50/p95 of |distance
from the segment line|), and the fitted principal-axis angle vs the segment
axis (degrees). With two files (production + gt) it prints a side-by-side
table -- the phase's improvement metric.

Wall segments come from tugbot_maze.maze_sim.load_segments() (map frame).
Usage:
  python3 tools/map_twist_metric.py PROD.npy [GT.npy] \
      [--center 10.54 9.04] [--half 3.2]
"""
import argparse
import math
import sys

import numpy as np

Z_BAND = (0.15, 1.5)          # wall band: excludes floor and lidar-mast returns
PERP_BAND_M = 0.35            # voxels within this distance of the segment line
END_TRIM_M = 0.15             # trim segment ends to avoid corner cross-talk


def wall_points(pts, seg, ground_z=0.0):
    """Voxels in the wall band of segment (x0,y0,x1,y1): perpendicular
    distance <= PERP_BAND_M, longitudinally inside the (end-trimmed) segment,
    z inside Z_BAND RELATIVE to ground_z. Datum-invariance contract (Task 1
    quality review I2): the gt and production maps may sit on different z
    datums (spawn-anchored odom vs world z), so the band is selected per
    cloud relative to its OWN ground level -- never absolute z."""
    x0, y0, x1, y1 = seg
    p = pts[(pts[:, 2] >= ground_z + Z_BAND[0])
            & (pts[:, 2] <= ground_z + Z_BAND[1])]
    d = np.array([x1 - x0, y1 - y0])
    L = float(np.hypot(*d))
    if L < 1e-9 or len(p) == 0:
        return p[:0]
    u = d / L
    rel = p[:, :2] - np.array([x0, y0])
    lon = rel @ u
    perp = rel @ np.array([-u[1], u[0]])
    keep = ((np.abs(perp) <= PERP_BAND_M)
            & (lon >= END_TRIM_M) & (lon <= L - END_TRIM_M))
    return p[keep]


def segment_band_stats(pts, seg, ground_z=0.0):
    """count / spread_p50 / spread_p95 (perpendicular, median-centered) and
    fitted principal-axis angle vs the segment axis (deg).

    Interpretation contract (Task 2 quality review, measured on synthetics):
    - spread_p95 DELTA (production minus gt, per segment) is the PRIMARY
      discriminator; absolute levels carry the common 0.24 m wall-thickness
      baseline (both faces in-band floor a clean interior wall at ~0.10).
    - angle is CORROBORATING-ONLY: parallel faces and the clean+twisted union
      attenuate the fitted angle ~2x (a 0.09 m fan reads ~-1.6 deg).
    - p50 is descriptive, NON-monotone in twist (fan fill can lower it).
    - sub-voxel twists read clean: a POSITIVE detection is trustworthy, a
      null is NOT proof of no twist."""
    sel = wall_points(pts, seg, ground_z)
    x0, y0, x1, y1 = seg
    d = np.array([x1 - x0, y1 - y0])
    u = d / np.hypot(*d)
    out = {'count': int(len(sel)), 'spread_p50': float('nan'),
           'spread_p95': float('nan'), 'angle_deg': float('nan')}
    if len(sel) < 30:
        return out
    perp = (sel[:, :2] - np.array([x0, y0])) @ np.array([-u[1], u[0]])
    centered = np.abs(perp - np.median(perp))
    out['spread_p50'] = float(np.percentile(centered, 50))
    out['spread_p95'] = float(np.percentile(centered, 95))
    xy = sel[:, :2] - sel[:, :2].mean(axis=0)
    cov = np.cov(xy.T)
    evals, evecs = np.linalg.eigh(cov)
    axis = evecs[:, int(np.argmax(evals))]
    ang = math.degrees(math.atan2(axis[1], axis[0])
                       - math.atan2(u[1], u[0]))
    while ang > 90.0:
        ang -= 180.0
    while ang < -90.0:
        ang += 180.0
    out['angle_deg'] = float(ang)
    return out


def center_segments(center, half):
    from tugbot_maze.maze_sim import load_segments
    cx, cy = center
    segs = []
    for seg in load_segments():
        x0, y0, x1, y1 = seg
        mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        if abs(mx - cx) <= half and abs(my - cy) <= half:
            segs.append(tuple(seg))
    return segs


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument('files', nargs='+', help='1-2 .npy voxel dumps (prod [gt])')
    ap.add_argument('--center', nargs=2, type=float, default=[10.54, 9.04])
    ap.add_argument('--half', type=float, default=3.2)
    args = ap.parse_args(argv)
    clouds = [np.load(f) for f in args.files]
    # Per-cloud ground datum (1st percentile of z): the two maps may sit on
    # different z datums (Task 1 quality review I2) -- bands are selected
    # relative to each cloud's own ground, and the datum delta is reported.
    grounds = [float(np.percentile(c[:, 2], 1.0)) for c in clouds]
    for f, g in zip(args.files, grounds):
        print('DATUM %s ground_z=%.3f' % (f, g))
        if not (-1.0 <= g <= 0.5):
            print('DATUM IMPLAUSIBLE %s: ground_z outside [-1.0, 0.5] -- '
                  'floor coverage sparse? Bands may be mis-selected.' % f)
    if len(grounds) == 2 and abs(grounds[0] - grounds[1]) > 0.1:
        print('DATUM WARNING: z datums differ by %.3f m (expected if odom is '
              'spawn-anchored; bands are per-cloud relative, comparison valid)'
              % abs(grounds[0] - grounds[1]))
    segs = center_segments(args.center, args.half)
    print('TWISTMETRIC center=(%.2f, %.2f) half=%.1f segments=%d'
          % (args.center[0], args.center[1], args.half, len(segs)))
    hdr = 'segment'.ljust(34) + ''.join(
        ('  [%d] n/p50/p95/ang' % i).ljust(28) for i in range(len(clouds)))
    print(hdr)
    agg = [[] for _ in clouds]
    deltas = []
    for seg in segs:
        row = ('(%.2f,%.2f)-(%.2f,%.2f)' % seg).ljust(34)
        stats = []
        for i, c in enumerate(clouds):
            s = segment_band_stats(c, seg, grounds[i])
            stats.append(s)
            row += ('  %5d/%.3f/%.3f/%+.2f' % (
                s['count'], s['spread_p50'], s['spread_p95'],
                s['angle_deg'])).ljust(28)
            if s['count'] >= 30:
                agg[i].append(s)
        print(row)
        # Paired per-segment delta (file0 - file1, i.e. production - gt): THE
        # discriminating statistic (Task 2 quality review I1). Absolute p95
        # levels carry the common wall-thickness baseline; the delta cancels
        # it. Only segments where BOTH clouds have count >= 30 pair up.
        if len(stats) == 2 and all(s['count'] >= 30 for s in stats):
            deltas.append((stats[0]['spread_p95'] - stats[1]['spread_p95'],
                           abs(stats[0]['angle_deg']) - abs(stats[1]['angle_deg'])))
    for i, rows in enumerate(agg):
        if rows:
            print('AGG[%d] bands=%d spread_p95_mean=%.4f |angle|_mean=%.3f'
                  % (i, len(rows),
                     float(np.mean([r['spread_p95'] for r in rows])),
                     float(np.mean([abs(r['angle_deg']) for r in rows]))))
    if deltas:
        dp = np.array([d[0] for d in deltas])
        da = np.array([d[1] for d in deltas])
        print('AGG_DELTA bands=%d dp95_mean=%.4f dp95_p95=%.4f dangle_mean=%.3f'
              % (len(deltas), float(dp.mean()),
                 float(np.percentile(dp, 95)), float(da.mean())))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
```

- [ ] **Step 4: Run the metric tests — green**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260728
python3 -m pytest src/tugbot_maze/test/test_map_twist_metric.py -q
```

Expected: 5 passed.

- [ ] **Step 5: Write the capture tool**

Create `tools/capture_cloud_map.py`:

```python
#!/usr/bin/env python3
"""One-shot capture of the latched production map (/maze/cloud_map_3d) to an
.npy voxel dump. MUST run while the stack is alive (the latched publisher
dies with the accumulator node). Exit 0 on success, 2 on timeout."""
import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--topic', default='/maze/cloud_map_3d')
    ap.add_argument('--timeout', type=float, default=20.0)
    args = ap.parse_args(argv)

    rclpy.init()
    node = Node('cloud_map_capture')
    got = {}
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL   # receive the latch
    node.create_subscription(PointCloud2, args.topic,
                             lambda m: got.setdefault('msg', m), qos)
    deadline = time.monotonic() + args.timeout
    while 'msg' not in got and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    rc = 2
    if 'msg' in got:
        pts = point_cloud2.read_points_numpy(
            got['msg'], field_names=('x', 'y', 'z'), skip_nans=False)
        np.save(args.out, pts.astype(np.float32))
        print('CAPTURE %s voxels=%d -> %s' % (args.topic, len(pts), args.out))
        rc = 0
    else:
        print('CAPTURE timeout on %s' % args.topic)
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
```

- [ ] **Step 6: Write the diag run script**

Create `tools/run_center_twist_diag.sh` (copy of `tools/run_flood_fill_maze.sh`
with the four marked changes; everything else byte-identical to the original —
read the original first and apply exactly these deltas):

1. Header comment: replace the first comment block with

```bash
# Center-twist diagnostic run (center-twist-root-cause): the standard
# flood-fill run with BOTH eval instruments on (POSEDIAG + the gt parallel
# accumulator) and a production-map capture BEFORE teardown (the latched
# /maze/cloud_map_3d dies with its node). Artifacts per run:
#   launch.log (POSEDIAG + GTCLOUD + CLOUDMAP lines)
#   cloud_map_3d.npy   (production map, captured pre-kill)
#   gt_cloud_map.npy   (gt map, dumped by the instrument at SIGINT)
# Usage: tools/run_center_twist_diag.sh [MAX_SECONDS] [HEADLESS] [USE_RVIZ]
```

2. Defaults: `POSE_SOURCE` stays `online_slam`; force the instruments by
replacing the `pose_diag:="${POSE_DIAG:-false}"` launch line with:

```bash
    pose_diag:=true gt_cloud_diag:=true \
```

3. After the result-detection loop and the `echo "$RESULT" > "$ART/result.txt"`
line, INSERT (before the `kill -INT` teardown):

```bash
# capture the production latched map BEFORE teardown (latch dies with the node)
# (skip when the launch itself died - the latch is already gone; review M3)
if [ "$RESULT" != "LAUNCH_DIED" ] && [ "$RESULT" != "DDS_SHM_FAIL" ]; then
    timeout 40 python3 "$(dirname "$0")/capture_cloud_map.py" \
        --out "$ART/cloud_map_3d.npy" | tee -a "$ART/run_meta.txt"
fi
```

4. Add `gt_cloud_accumulator` to the `kill_all_sim` pattern list (next to
`cloud_map_accumulator`).

Then: `chmod +x tools/run_center_twist_diag.sh tools/capture_cloud_map.py tools/map_twist_metric.py`

- [ ] **Step 7: Full suite**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260728
set +u; source /opt/ros/jazzy/setup.bash; source install/setup.bash; set -u
python3 -m pytest src/tugbot_maze/test/ src/tugbot_bringup/test/ -q 2>&1 | tail -5
```

Expected: `10 failed, 557 passed` (Task 1's 552 + 5 metric tests).

- [ ] **Step 8: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260728/tools/capture_cloud_map.py \
        ros2_ws_tugbot_nav_20260728/tools/map_twist_metric.py \
        ros2_ws_tugbot_nav_20260728/tools/run_center_twist_diag.sh \
        ros2_ws_tugbot_nav_20260728/src/tugbot_maze/test/test_map_twist_metric.py
git commit -F - <<'EOF'
feat: twist metric + capture tool + diag run variant

map_twist_metric turns the observed center twist into numbers: per
wall-band voxel count, median-centered perpendicular spread p50/p95,
and fitted principal-axis angle vs the true segment, side-by-side
production vs gt. capture_cloud_map grabs the latched production map
BEFORE teardown; the diag run script forces both instruments on and
captures pre-kill.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01AVWsj4MmGMqfFqXeUuanUC
EOF
```

---

### Task 3: POSEDIAG residual bucketing

**Files:**
- Create: `ros2_ws_tugbot_nav_20260728/tools/center_twist_decompose.py`
- Create: `ros2_ws_tugbot_nav_20260728/src/tugbot_maze/test/test_center_twist_decompose.py`

POSEDIAG line format (flood_fill_solver.py:~325, unchanged since the
localization-root-cause phase):
`POSEDIAG gt=(X, Y, YAW) odom=(X, Y, YAW) solver=(X, Y, YAW)` — gt in the
maze WORLD frame, odom/solver in the entrance-anchored MAP frame
(map = world + (11.011281, 9.02507); yaw frames aligned, entrance yaw 0).

- [ ] **Step 1: Write the failing parser/bucketing tests**

Create `src/tugbot_maze/test/test_center_twist_decompose.py`:

```python
"""Unit tests for the POSEDIAG bucketing analysis: parsing, world->map
compensation, residual math, and center-box bucketing."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from center_twist_decompose import parse_posediag_line, bucket_stats  # noqa: E402


LINE = ('[flood_fill_solver-18] [INFO] [1785000000.5] [flood_fill_solver]: '
        'POSEDIAG gt=(-0.471, -0.025, 0.010) odom=(10.540, 9.000, 0.000) '
        'solver=(10.640, 9.000, 0.030)')


def test_parse_extracts_three_triples_and_residuals():
    row = parse_posediag_line(LINE)
    assert row is not None
    # gt WORLD -> MAP: x -0.471+11.011281=10.540281, y -0.025+9.02507=9.00007
    assert row['gt_map'][0] == pytest.approx(10.540281, abs=1e-6)
    assert row['gt_map'][1] == pytest.approx(9.00007, abs=1e-6)
    assert row['yaw_res_solver'] == pytest.approx(0.020, abs=1e-9)   # 0.030-0.010
    assert row['yaw_res_odom'] == pytest.approx(-0.010, abs=1e-9)    # 0.000-0.010
    assert row['pos_res_solver'] == pytest.approx(0.0997, abs=1e-3)
    assert row['t_s'] == pytest.approx(1785000000.5)


def test_parse_rejects_non_posediag_lines():
    assert parse_posediag_line('DIAG pose=(1,2,3)') is None


def test_bucket_stats_splits_center_vs_rest():
    rows = []
    for i in range(10):
        rows.append({'t_s': float(i), 'gt_map': (10.5, 9.0),
                     'solver_map': (10.5, 9.0),
                     'yaw_res_solver': 0.03, 'yaw_res_odom': 0.0,
                     'pos_res_solver': 0.05})
    for i in range(10, 20):
        rows.append({'t_s': float(i), 'gt_map': (2.0, 2.0),
                     'solver_map': (2.0, 2.0),
                     'yaw_res_solver': 0.001, 'yaw_res_odom': 0.0,
                     'pos_res_solver': 0.01})
    b = bucket_stats(rows, center=(10.54, 9.04), half=3.2)
    assert b['center']['n'] == 10 and b['rest']['n'] == 10
    assert b['center']['yaw_p95'] > b['rest']['yaw_p95'] * 5
```

- [ ] **Step 2: Run to verify failure**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260728
python3 -m pytest src/tugbot_maze/test/test_center_twist_decompose.py -q
```

Expected: FAIL (module not found).

- [ ] **Step 3: Write the analysis tool**

Create `tools/center_twist_decompose.py`:

```python
#!/usr/bin/env python3
"""POSEDIAG residual bucketing for the center-twist forensics.

Parses POSEDIAG lines from one or more launch.logs and reports, per run and
per region bucket (center box vs rest):
  yaw_res_solver = solver_yaw - gt_yaw    (the residual that paints fan twist)
  yaw_res_odom   = odom_yaw   - gt_yaw    (sanity: ~0 on the drift-free rig)
  pos_res_solver = |solver_xy - gt_map_xy|
plus the top-10 worst |yaw_res_solver| windows with timestamps and positions.
Because odom is drift-free, (solver - odom) == ICP-injected error; the
decomposition prints that too (yaw_res_solver - yaw_res_odom).

Usage: python3 tools/center_twist_decompose.py LOG [LOG...] \
           [--center 10.54 9.04] [--half 3.2]
"""
import argparse
import math
import re
import sys

import numpy as np

WORLD_TO_MAP_X = 11.011281
WORLD_TO_MAP_Y = 9.02507

_RE = re.compile(
    r'\[(\d+\.\d+)\].*POSEDIAG '
    r'gt=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\) '
    r'odom=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\) '
    r'solver=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)')


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def parse_posediag_line(line):
    m = _RE.search(line)
    if not m:
        return None
    v = [float(g) for g in m.groups()]
    t_s = v[0]
    gt = (v[1] + WORLD_TO_MAP_X, v[2] + WORLD_TO_MAP_Y, v[3])
    odom = (v[4], v[5], v[6])
    sol = (v[7], v[8], v[9])
    return {
        't_s': t_s,
        'gt_map': gt[:2], 'solver_map': sol[:2],
        'yaw_res_solver': _wrap(sol[2] - gt[2]),
        'yaw_res_odom': _wrap(odom[2] - gt[2]),
        'pos_res_solver': math.hypot(sol[0] - gt[0], sol[1] - gt[1]),
    }


def _stats(rows):
    if not rows:
        return {'n': 0}
    yaw = np.abs([r['yaw_res_solver'] for r in rows])
    yod = np.abs([r['yaw_res_odom'] for r in rows])
    pos = np.array([r['pos_res_solver'] for r in rows])
    return {'n': len(rows),
            'yaw_p50': float(np.percentile(yaw, 50)),
            'yaw_p95': float(np.percentile(yaw, 95)),
            'yaw_max': float(yaw.max()),
            'yaw_odom_p95': float(np.percentile(yod, 95)),
            'pos_p50': float(np.percentile(pos, 50)),
            'pos_p95': float(np.percentile(pos, 95))}


def bucket_stats(rows, center, half):
    cx, cy = center
    inc, out = [], []
    for r in rows:
        x, y = r['gt_map']
        (inc if (abs(x - cx) <= half and abs(y - cy) <= half) else out).append(r)
    return {'center': _stats(inc), 'rest': _stats(out)}


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--center', nargs=2, type=float, default=[10.54, 9.04])
    ap.add_argument('--half', type=float, default=3.2)
    args = ap.parse_args(argv)
    for path in args.logs:
        rows = []
        with open(path, errors='replace') as f:
            for line in f:
                r = parse_posediag_line(line)
                if r:
                    rows.append(r)
        print('==== %s: %d POSEDIAG rows ====' % (path, len(rows)))
        if not rows:
            print('  (was the run launched with pose_diag:=true?)')
            continue
        b = bucket_stats(rows, tuple(args.center), args.half)
        for name in ('center', 'rest'):
            s = b[name]
            if s['n'] == 0:
                print('  %-6s n=0' % name)
                continue
            print('  %-6s n=%4d yaw_p50=%.4f yaw_p95=%.4f yaw_max=%.4f '
                  'yaw_odom_p95=%.4f pos_p50=%.3f pos_p95=%.3f'
                  % (name, s['n'], s['yaw_p50'], s['yaw_p95'], s['yaw_max'],
                     s['yaw_odom_p95'], s['pos_p50'], s['pos_p95']))
        worst = sorted(rows, key=lambda r: -abs(r['yaw_res_solver']))[:10]
        print('  worst |yaw_res_solver| windows:')
        for r in worst:
            print('    t=%.1f pos=(%.2f, %.2f) yaw_res=%+.4f pos_res=%.3f'
                  % (r['t_s'], r['gt_map'][0], r['gt_map'][1],
                     r['yaw_res_solver'], r['pos_res_solver']))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
```

- [ ] **Step 4: Run tests — green**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260728
python3 -m pytest src/tugbot_maze/test/test_center_twist_decompose.py -q
```

Expected: 3 passed. Then the full suite tail:

```bash
python3 -m pytest src/tugbot_maze/test/ src/tugbot_bringup/test/ -q 2>&1 | tail -3
```

Expected: `10 failed, 560 passed`.

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260728/tools/center_twist_decompose.py \
        ros2_ws_tugbot_nav_20260728/src/tugbot_maze/test/test_center_twist_decompose.py
git commit -F - <<'EOF'
feat: POSEDIAG residual bucketing for center-twist forensics

Parses the three-way POSEDIAG triples (gt world-frame compensated to
map), reports yaw/pos residuals bucketed center-box vs rest with the
drift-free-odom sanity column, and lists the worst yaw windows with
positions - the decomposition layer under the A/B verdict.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01AVWsj4MmGMqfFqXeUuanUC
EOF
```

---

### Task 4: Instrumented runs + WRITTEN VERDICT (MAIN SESSION ONLY)

**Files:** append verdict to `docs/superpowers/specs/2026-07-23-center-twist-root-cause-design.md`.

Controller-run in the main session (sims die with subagents).

- [ ] **Step 1: Two instrumented headless runs**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260728
tools/run_center_twist_diag.sh 1800 true false
tools/run_center_twist_diag.sh 1800 true false
```

Per run verify the artifact dir contains: `result.txt` = EXIT_REACHED,
`cloud_map_3d.npy` (capture logged its voxel count), `gt_cloud_map.npy`
(GTCLOUD dump line in launch.log), POSEDIAG lines present, `unpaired` counter
near zero (a large unpaired count voids the A/B — investigate before
proceeding).

- [ ] **Step 2: Run the analyses**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260728
set +u; source /opt/ros/jazzy/setup.bash; source install/setup.bash; set -u
python3 tools/map_twist_metric.py $ART1/cloud_map_3d.npy $ART1/gt_cloud_map.npy
python3 tools/map_twist_metric.py $ART2/cloud_map_3d.npy $ART2/gt_cloud_map.npy
python3 tools/center_twist_decompose.py $ART1/launch.log $ART2/launch.log
```

Also run the standard gates (oracle replay 0.000%, gate/ERROR greps, CLOUDMAP
finals + saturation) — instrument runs must not regress the baseline.

- [ ] **Step 3: Decide the fork and WRITE THE VERDICT before any fix**

Verdict statistics discipline (Task 2 quality review, synthetic-calibrated):
the PRIMARY discriminator is the paired per-segment spread_p95 DELTA
(`AGG_DELTA dp95_*`, production minus gt — cancels the common 0.24 m
wall-thickness baseline that floors clean interior walls at p95≈0.10); the
fitted angle is CORROBORATING-ONLY (~2x attenuated by face/union mixing);
p50 is non-monotone in twist; a null reading is NOT proof of no twist
(sub-voxel twists read clean) — a positive detection is trustworthy, absence
claims are bounded by the ~1-voxel resolution floor and must be worded that
way in the verdict.

Decision tree (from the spec):
- production dp95 clearly positive across center bands (angle corroborating)
  → branch (a) believed-pose. The decompose output then names the injection pattern
  (center-bucket yaw_p95 vs rest; worst-window positions; yaw_odom sanity ~0).
  Correlate worst windows with the solver's ICP/gate log lines around those
  timestamps (grep the launch.log tags observed there — e.g. rejected/accept
  markers) to name the mechanism.
- both maps show the same twist → branch (b) timing/pairing; forensics shift
  to stamp deltas (GTCLOUD unpaired/stale counters vs CLOUDMAP stale_dropped,
  TF publish cadence).
- both clean → branch (c): re-observe with the GUI instrument run before
  anything else.

Append `## Addendum — verdict` to the spec with: the metric tables, the
decompose buckets, the branch taken, the named mechanism, and the chosen
lever (or the downgrade decision). Commit docs:

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add docs/superpowers/specs/2026-07-23-center-twist-root-cause-design.md
git commit -F - <<'EOF'
docs: center-twist verdict - instrumented A/B + decomposition record

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01AVWsj4MmGMqfFqXeUuanUC
EOF
```

---

### Task 5: Evidence-named fix (scope set by the Task 4 verdict)

**Files:** named by the verdict. Constraints, not guesses:

- Branch (a) levers live in the SOLVER GATE LAYER (`flood_fill_solver.py` /
  `pose_tracking.py` gate helpers — the yaw gate + position gate precedent);
  localizer files stay byte-frozen. Candidate family (to be CONFIRMED by the
  measured mechanism, not assumed): a sub-threshold yaw trust rule at the
  same choke point where `apply_odom_yaw_gate` runs (e.g. blending or
  clamping the accepted yaw toward the drift-free odom yaw when the ICP yaw
  innovation is below the existing gate but above the measured clean-band
  noise). Any new rule: calibrate its threshold from the Task 4 measured
  clean-band vs twist-band distributions (the two-band vacuum calibration
  method from the position gate), TDD it in `test_odom_yaw_gate.py` style,
  and record the real-drifting-odom caveat.
- Branch (b) levers: the pairing/cadence element the stamp evidence names.
  The production accumulator may ONLY change if branch (b) evidence names it
  explicitly (spec non-goal otherwise).
- Downgrade: if the verdict says unfixable-in-phase, skip to Task 6 with the
  verdict document as the deliverable (adjudication 2).

Process per the established discipline: dispatch the fix as its own
implementer+review cycle AFTER the controller records the verdict; one
evidence-backed fix, no parameter roulette; full suite adjudication
(10 failed / 560 passed expected envelope plus any new fix tests).

---

### Task 6: Re-verify with the same instruments + GUI gate

- [ ] **Step 1 (MAIN SESSION): two more instrumented headless runs** (same
commands as Task 4) with the fix in place. Gates: EXIT, oracle 0.000%,
POS/YAW gates ≤1 recovery, zero ERROR, CLOUDMAP finals in the 420-450k band
(not increasing), saturation decaying.

- [ ] **Step 2: Twist metric comparison** — center-band `spread_p95` and
`|angle|` (production) move significantly toward the gt values vs the Task 4
tables; decompose center-bucket yaw_p95 shrinks toward the rest-bucket value.

- [ ] **Step 3: Append results to the spec addendum + commit** (same docs
commit pattern as Task 4; include before/after tables).

- [ ] **Step 4: STOP — GUI acceptance is user-triggered** (RViz center
corridor visual verdict). After the user's verdict: final whole-branch
review → finishing flow (merge --no-ff, suite on main, push, delete branch,
memory archive).

---

## Self-review record

- Spec coverage: instrument a→Task 1, b→Task 2, c→Task 3; protocol/verdict→Task 4; fix principles→Task 5; acceptance/downgrade→Task 6. Coordinate anchors and fairness rules carried into code.
- Placeholders: Task 5 intentionally defers exact code to the verdict (measure-first phase structure per adjudication 2); everything pre-writable is written. The diag script is specified as four exact deltas against a byte-copied original.
- Type consistency: `nearest_gt_sample(buffer, t_s)` / `compose_map_sensor(x..qw)` / `segment_band_stats(pts, seg)` / `wall_points(pts, seg)` / `parse_posediag_line(line)` / `bucket_stats(rows, center, half)` consistent between tools and tests; suite-count arithmetic 544→552→557→560.
