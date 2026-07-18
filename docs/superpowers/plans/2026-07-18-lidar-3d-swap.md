# 3D Lidar Swap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在新 workspace `ros2_ws_tugbot_nav_20260719` 把 2D 雷达 `scan_omni` 真正换成 16 线 3D 雷达 `lidar_3d`,自研投影节点从点云抽近水平切片继续喂 `/scan`(几何逐字段同旧),导航链零改动,迷宫全回归不退化,RViz 可见 3D 点云。

**Architecture:** 先捕获旧 /scan 的字段基线(verify-first 合同);纯 numpy 投影核(与 rclpy 解耦,离线 pytest);SDF 换传感器(CERBERUS front_laser 规格 + 全新居中姿态);bridge 只桥点云;`scan_slice_projector` 节点挂 `tugbot_gazebo.launch.py`;RViz 加 PointCloud2 显示;PRIME headless ×2 + GUI ×1 全回归。

**Tech Stack:** ROS 2 Jazzy、Gazebo Sim 8 gpu_lidar、ros_gz_bridge(PointCloudPacked→PointCloud2)、numpy(仓库既有依赖)、`sensor_msgs_py.point_cloud2`(ROS 自带)、pytest。

**Spec:** `docs/superpowers/specs/2026-07-18-lidar-3d-swap-design.md`(含"关键事实"节——实现前必读)

---

## 全局约束(每个任务都适用)

- 仓库根 `$REPO` = `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`;新 workspace `$WS` = `$REPO/ros2_ws_tugbot_nav_20260719`。
- 构建**必须** `colcon build --symlink-install`。每个命令块显式 `cd`。`set +u` 后再 source ROS。只有 `python3`。
- **禁止**直接工具调用里前台 `sleep`(脚本文件内可以)。**禁止**自动启动 GUI(headless 随意)。
- ⚠️ **pkill 自匹配坑**:任何含 "gz sim"/"ros2 launch" 模式的 pkill/pgrep 必须写进脚本文件执行,且**写脚本与执行分两次 Bash 调用**(同一调用里 heredoc 写+跑也会自杀,已两次实锤)。
- PRIME offload 已由 `tools/run_flood_fill_maze.sh` 与 `tools/verify_front_camera.sh` 自带 defaults(20260718 固化)——新的活体门脚本必须复制同一 env 块。
- git:heredoc 提交 `git commit -F - <<'EOF'...EOF`,信息**无反引号**,结尾 `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`;`git add` 只用显式路径。
- 纯模块测试(在 `$WS`):`PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q`;基线 **7 failed / 413 passed / 3 xfailed**,失败名单存档对照,不允许新增。

---

### Task 1: 新 workspace 20260719 + 分支 + 基线

**Files:** Create: `ros2_ws_tugbot_nav_20260719/`(克隆 20260718)

- [ ] **Step 1: 分支**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git checkout -b lidar-3d-swap
```

- [ ] **Step 2: 克隆 + 清产物**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
cp -a ros2_ws_tugbot_nav_20260718 ros2_ws_tugbot_nav_20260719
rm -rf ros2_ws_tugbot_nav_20260719/build ros2_ws_tugbot_nav_20260719/install ros2_ws_tugbot_nav_20260719/log
find ros2_ws_tugbot_nav_20260719 -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null
find ros2_ws_tugbot_nav_20260719 -name '.pytest_cache' -type d -exec rm -rf {} + 2>/dev/null
```

- [ ] **Step 3: 构建**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260719
set +u; source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Expected: 6 packages 全部成功。

- [ ] **Step 4: 基线套件 + 失败名单存档**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260719
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -3
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | grep '^FAILED' | sort > /tmp/lidar3d_baseline_failures.txt
wc -l /tmp/lidar3d_baseline_failures.txt
```

Expected: `7 failed, 413 passed, 3 xfailed`;名单 7 行。不符则 BLOCKED。

- [ ] **Step 5: 提交(显式路径,先 status 查残留)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260719
git status --short | grep -v '^A' | head -3
git commit -F - <<'EOF'
chore: clone workspace 20260718 -> 20260719 for the lidar-3d-swap phase

Byte-identical copy (build artifacts and caches stripped). Baseline
suite: 7 pre-existing failures / 413 passed / 3 xfailed, names archived.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 2: 捕获旧 /scan 字段基线(verify-first 合同)

**Files:**
- Create: `ros2_ws_tugbot_nav_20260719/tools/capture_scan_baseline.py`
- Create: `ros2_ws_tugbot_nav_20260719/src/tugbot_maze/test/scan_omni_baseline.json`(捕获产物,提交入库)

此时 workspace 仍是 2D 雷达——这是唯一能捕获旧合同的窗口(Task 4 之后传感器就没了)。

- [ ] **Step 1: 写捕获脚本**

新建 `tools/capture_scan_baseline.py`:

```python
#!/usr/bin/env python3
"""One-shot: capture the legacy 2D /scan message's field contract to JSON.
Run while the (still 2D) sim is up. The projector must reproduce these
fields verbatim so the nav chain can't tell the sensor changed."""
import json
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Capture(Node):
    def __init__(self):
        super().__init__('scan_baseline_capture')
        self.msg = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

    def on_scan(self, msg):
        self.msg = msg


def main():
    out_path = sys.argv[1]
    rclpy.init()
    node = Capture()
    import time
    t0 = time.monotonic()
    while node.msg is None and time.monotonic() - t0 < 20.0:
        rclpy.spin_once(node, timeout_sec=0.5)
    m = node.msg
    assert m is not None, 'no /scan within 20s'
    n = len(m.ranges)
    inf_count = sum(1 for r in m.ranges if math.isinf(r))
    nan_count = sum(1 for r in m.ranges if math.isnan(r))
    contract = {
        'n_bins': n,
        'angle_min': m.angle_min,
        'angle_max': m.angle_max,
        'angle_increment': m.angle_increment,
        'time_increment': m.time_increment,
        'scan_time': m.scan_time,
        'range_min': m.range_min,
        'range_max': m.range_max,
        'frame_id': m.header.frame_id,
        'observed_inf_bins': inf_count,
        'observed_nan_bins': nan_count,
    }
    with open(out_path, 'w') as f:
        json.dump(contract, f, indent=2, sort_keys=True)
    print(json.dumps(contract, indent=2, sort_keys=True))
    rclpy.shutdown()
    print('SCAN BASELINE CAPTURED')


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: 起 headless sim 并捕获(gate 式脚本,复用 verify_front_camera.sh 的骨架)**

先写脚本(一次调用):

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260719
cat > /tmp/capture_baseline_run.sh <<'SH'
#!/usr/bin/env bash
set +u
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260719
export DISPLAY="${DISPLAY:-:1}"
export __NV_PRIME_RENDER_OFFLOAD="${__NV_PRIME_RENDER_OFFLOAD:-1}"
export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}"
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=$((RANDOM % 100))
for pat in "ros2 launch" "gz sim" "ruby.*gz" parameter_bridge bridge_node ros_gz_bridge static_transform_publisher; do pkill -9 -f "$pat" 2>/dev/null; done
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
MAZE_WORLD="install/tugbot_gazebo/share/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
ros2 launch tugbot_gazebo tugbot_gazebo.launch.py headless:=true world_sdf:="$MAZE_WORLD" > /tmp/capture_baseline_launch.log 2>&1 &
LP=$!
sleep 18 & wait $!
python3 tools/capture_scan_baseline.py src/tugbot_maze/test/scan_omni_baseline.json
RC=$?
kill -INT $LP 2>/dev/null; sleep 2
for pat in "ros2 launch" "gz sim" "ruby.*gz" parameter_bridge bridge_node ros_gz_bridge static_transform_publisher; do pkill -9 -f "$pat" 2>/dev/null; done
exit $RC
SH
echo written
```

再执行(单独一次调用,勿与写入合并):

```bash
bash /tmp/capture_baseline_run.sh
```

Expected: 打印 JSON(n_bins=900、angle ±π、range 0.2/100 量级)+ `SCAN BASELINE CAPTURED`。记录 angle_increment 的精确值与 inf/nan 分布(空 bin 编码证据)。

- [ ] **Step 3: 提交脚本 + 基线 JSON**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260719/tools/capture_scan_baseline.py ros2_ws_tugbot_nav_20260719/src/tugbot_maze/test/scan_omni_baseline.json
git commit -F - <<'EOF'
feat: capture the legacy 2D /scan field contract as a committed fixture

The projector must reproduce these fields verbatim; captured live from
the last 2D-sensor boot before the swap.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 3: 纯投影核 slice_to_scan(TDD)

**Files:**
- Create: `ros2_ws_tugbot_nav_20260719/src/tugbot_maze/tugbot_maze/slice_to_scan.py`
- Test: `ros2_ws_tugbot_nav_20260719/src/tugbot_maze/test/test_slice_to_scan.py`

- [ ] **Step 1: 写失败测试**

新建 `test_slice_to_scan.py`:

```python
"""Pure projection core tests: synthetic 3D points -> 900-bin ranges.
Channel selection is by per-point vertical angle (no cloud-organization
assumption); the 16-ring +/-15deg lidar has rings at +/-1deg, +/-3deg, ...
so RING_BAND_RAD must accept exactly the two +/-1deg rings."""
import json
import math
from pathlib import Path

import pytest

from tugbot_maze.slice_to_scan import RING_BAND_RAD, SCAN_CONTRACT, fold_to_ranges


def _contract_kwargs():
    c = SCAN_CONTRACT
    return dict(n_bins=c['n_bins'], angle_min=c['angle_min'],
                angle_increment=c['angle_increment'],
                range_min=c['range_min'], range_max=c['range_max'])


def test_contract_matches_captured_baseline():
    fixture = json.loads((Path(__file__).parent / 'scan_omni_baseline.json').read_text())
    for key in ('n_bins', 'angle_min', 'angle_max', 'angle_increment',
                'time_increment', 'scan_time', 'range_min', 'range_max'):
        assert SCAN_CONTRACT[key] == pytest.approx(fixture[key], abs=1e-9), key


def test_ring_band_accepts_only_the_two_horizontal_rings():
    ring_angles = [math.radians(-15 + 2 * k) for k in range(16)]
    inside = [a for a in ring_angles if abs(a) <= RING_BAND_RAD]
    assert len(inside) == 2
    assert inside == pytest.approx([math.radians(-1), math.radians(1)])


def test_wall_point_lands_in_correct_bin_with_min_fold():
    kw = _contract_kwargs()
    az = 0.5
    z1 = 3.0 * math.tan(math.radians(1))   # on the +1deg ring at 3m horizontal
    pts = [(3.0 * math.cos(az), 3.0 * math.sin(az), z1),
           (5.0 * math.cos(az), 5.0 * math.sin(az), 5.0 * math.tan(math.radians(-1)))]
    ranges = fold_to_ranges(pts, **kw)
    idx = round((az - kw['angle_min']) / kw['angle_increment']) % kw['n_bins']
    assert ranges[idx] == pytest.approx(math.hypot(3.0, z1), abs=1e-6)  # min of the two
    assert sum(1 for r in ranges if math.isfinite(r)) == 1


def test_out_of_band_and_out_of_range_points_are_dropped():
    kw = _contract_kwargs()
    pts = [
        (2.0, 0.0, 2.0 * math.tan(math.radians(5))),    # 5deg ring: outside band
        (2.0, 0.0, -2.0 * math.tan(math.radians(15))),  # floor-seeking ring
        (0.05, 0.0, 0.0),                               # below range_min
        (150.0, 0.0, 0.0),                              # beyond range_max
        (0.0, 0.0, 1.0),                                # degenerate: zero horizontal
    ]
    ranges = fold_to_ranges(pts, **kw)
    assert all(math.isinf(r) for r in ranges)


def test_empty_bins_are_positive_inf():
    kw = _contract_kwargs()
    ranges = fold_to_ranges([], **kw)
    assert len(ranges) == kw['n_bins']
    assert all(math.isinf(r) and r > 0 for r in ranges)


def test_pi_boundary_wraps_to_a_valid_bin():
    kw = _contract_kwargs()
    pts = [(-4.0, -1e-9, 0.0)]   # azimuth ~ -pi
    ranges = fold_to_ranges(pts, **kw)
    assert sum(1 for r in ranges if math.isfinite(r)) == 1
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260719
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_slice_to_scan.py -v
```

Expected: FAIL,`No module named 'tugbot_maze.slice_to_scan'`。

- [ ] **Step 3: 写实现**

新建 `slice_to_scan.py`(**SCAN_CONTRACT 的数值从 Task 2 捕获的 JSON 逐值抄入**——下面的 angle/time 值是占位示意,以 JSON 为准;contract 测试会锁死):

```python
"""Pure projection core: 3D lidar cloud -> the legacy 2D /scan geometry.

The 16-ring +/-15deg lidar has rings every 2deg (+/-1, +/-3, ... +/-15);
we keep only points whose per-point vertical angle is within the two
+/-1deg rings (RING_BAND_RAD) and min-fold them into the legacy 900
azimuth bins. Per-point vertical angle (atan2(z, hypot(x,y))) avoids any
assumption about the cloud's row/column organization. Empty bins are
+inf, matching the observed legacy encoding (scan_omni_baseline.json).
SCAN_CONTRACT values are copied verbatim from that captured fixture;
test_contract_matches_captured_baseline pins them.
"""
import math

import numpy as np

RING_BAND_RAD = math.radians(1.1)   # accepts exactly the +/-1deg rings, rejects +/-3deg

SCAN_CONTRACT = {
    'n_bins': 900,
    'angle_min': -3.141592653589793,     # copy exact value from scan_omni_baseline.json
    'angle_max': 3.141592653589793,      # copy exact value
    'angle_increment': 0.006990424,      # copy exact value
    'time_increment': 0.0,               # copy exact value
    'scan_time': 0.0,                    # copy exact value
    'range_min': 0.2,                    # copy exact value
    'range_max': 100.0,                  # copy exact value
}


def fold_to_ranges(points_xyz, n_bins, angle_min, angle_increment,
                   range_min, range_max, band_rad=RING_BAND_RAD):
    """points_xyz: (N,3) array-like in the sensor frame. Returns a list of
    n_bins ranges: min-folded per azimuth bin, +inf where no return."""
    xyz = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
    ranges = np.full(n_bins, np.inf)
    if xyz.size:
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if xyz.size:
        r_h = np.hypot(xyz[:, 0], xyz[:, 1])
        keep = r_h > 0.0
        xyz, r_h = xyz[keep], r_h[keep]
        keep = np.abs(np.arctan2(xyz[:, 2], r_h)) <= band_rad
        xyz, r_h = xyz[keep], r_h[keep]
        r = np.hypot(r_h, xyz[:, 2])
        keep = (r >= range_min) & (r <= range_max)
        xyz, r = xyz[keep], r[keep]
        if r.size:
            az = np.arctan2(xyz[:, 1], xyz[:, 0])
            idx = np.rint((az - angle_min) / angle_increment).astype(np.int64) % n_bins
            np.minimum.at(ranges, idx, r)
    return ranges.tolist()
```

⚠️ 实现时把 SCAN_CONTRACT 每个值替换为 scan_omni_baseline.json 的精确值(contract 测试不过就是没抄对)。若捕获显示空 bin 编码不是 +inf(如 range_max+1 或 nan),把 `np.full` 的填充值与 docstring 一起改成实测编码,并同步 `test_empty_bins_are_positive_inf`(重命名为实测语义)——决策记附记。

- [ ] **Step 4: 跑测试确认通过 + 全套件对照**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260719
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_slice_to_scan.py -v
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -3
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | grep '^FAILED' | sort | diff /tmp/lidar3d_baseline_failures.txt -
```

Expected: 6 PASSED;套件 `7 failed, 419 passed, 3 xfailed`;diff 空。

- [ ] **Step 5: 提交**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260719/src/tugbot_maze/tugbot_maze/slice_to_scan.py ros2_ws_tugbot_nav_20260719/src/tugbot_maze/test/test_slice_to_scan.py
git commit -F - <<'EOF'
feat: pure numpy projection core slice_to_scan (3D cloud -> legacy scan)

Per-point vertical-angle ring selection (the two +/-1deg rings of the
16-ring lidar), min-fold into the captured legacy 900-bin contract,
+inf empty bins. Fully offline-tested against synthetic clouds and the
committed scan_omni_baseline.json fixture.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 4: SDF 传感器置换 + 守卫测试(TDD)+ bridge/TF

**Files:**
- Test: `ros2_ws_tugbot_nav_20260719/src/tugbot_maze/test/test_lidar_3d.py`(新建)
- Modify: `ros2_ws_tugbot_nav_20260719/src/tugbot_description/models/anymal_c/model.sdf`
- Modify: `ros2_ws_tugbot_nav_20260719/src/tugbot_gazebo/config/tugbot_bridge.yaml`
- Modify: `ros2_ws_tugbot_nav_20260719/src/tugbot_gazebo/launch/tugbot_gazebo.launch.py`

- [ ] **Step 1: 写失败守卫测试**

新建 `test_lidar_3d.py`:

```python
"""Guard: the 3D lidar must exist centered on the back (the nav chain
assumes SCAN_OFFSET_X=0), the original CERBERUS front_laser pose
(-0.364, z 0.142, yaw -1.571 — off-center AND rotated) must NOT be
copied, and the legacy 2D scan_omni must be gone."""
import math
import re
from pathlib import Path


def _model_sdf_text():
    ws_src = Path(__file__).resolve().parents[2]
    return (ws_src / 'tugbot_description' / 'models' / 'anymal_c' / 'model.sdf').read_text()


def _lidar_block():
    sdf = _model_sdf_text()
    m = re.search(r'<sensor name="lidar_3d" type="gpu_lidar">(.*?)</sensor>', sdf, re.S)
    assert m is not None, 'lidar_3d sensor missing from model.sdf'
    return m.group(1)


def test_scan_omni_is_gone():
    assert 'scan_omni' not in _model_sdf_text()


def test_lidar_3d_pose_is_centered_and_unrotated():
    block = _lidar_block()
    m = re.search(r'<pose>([^<]+)</pose>', block)
    assert m is not None, 'lidar_3d has no pose'
    x, y, z, roll, pitch, yaw = (float(v) for v in m.group(1).split())
    assert abs(x) < 0.05 and abs(y) < 0.05, f'lidar off-center ({x},{y}): breaks SCAN_OFFSET_X=0'
    assert abs(yaw) < 0.05, f'lidar yaw={yaw}: the CERBERUS -1.571 must not be copied'
    assert abs(roll) < 0.01 and abs(pitch) < 0.01
    assert 0.2 < z < 0.5, f'lidar z={z}: expected the old scan_omni height (~0.35)'


def test_lidar_3d_spec_matches_cerberus_front_laser():
    block = _lidar_block()
    assert '<update_rate>10</update_rate>' in block
    h = re.search(r'<horizontal>(.*?)</horizontal>', block, re.S).group(1)
    v = re.search(r'<vertical>(.*?)</vertical>', block, re.S).group(1)
    assert '<samples>1800</samples>' in h
    assert '<samples>16</samples>' in v
    vmin = float(re.search(r'<min_angle>([^<]+)</min_angle>', v).group(1))
    vmax = float(re.search(r'<max_angle>([^<]+)</max_angle>', v).group(1))
    assert vmin == -vmax and math.isclose(vmax, 0.261799, abs_tol=1e-5)
    assert '<topic>/lidar/points</topic>' in block
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260719
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_lidar_3d.py -v
```

Expected: 3 FAILED(lidar_3d missing;scan_omni 仍在)。

- [ ] **Step 3: model.sdf 置换**

在 `src/tugbot_description/models/anymal_c/model.sdf`:**整块删除** `scan_omni` sensor(`<sensor name="scan_omni" ...>` 到对应 `</sensor>`,约行 219–244),原位插入:

```xml
      <sensor name="lidar_3d" type="gpu_lidar">
        <!-- Centered on the back at the old scan_omni position: the nav
             chain assumes SCAN_OFFSET_X=0. The original CERBERUS
             front_laser pose (-0.364 0 0.142, yaw -1.571) is off-center
             and rotated and must not be copied. -->
        <pose>0 0 0.35 0 0 0</pose>
        <topic>/lidar/points</topic>
        <always_on>1</always_on>
        <visualize>false</visualize>
        <update_rate>10</update_rate>
        <lidar>
          <scan>
            <horizontal>
              <samples>1800</samples>
              <resolution>1.0</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
            <vertical>
              <samples>16</samples>
              <resolution>1</resolution>
              <min_angle>-0.261799</min_angle>
              <max_angle>0.261799</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.05</min>
            <max>100</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0</mean>
            <stddev>0.01</stddev>
          </noise>
        </lidar>
      </sensor>
```

(规格逐值来自原版 front_laser,tmp_resources/.../model.sdf 行 555–584;只有 name、pose、topic、visualize/always_on 是新的。其余 model.sdf 内容一律不动。)

- [ ] **Step 4: bridge yaml 置换**

`src/tugbot_gazebo/config/tugbot_bridge.yaml`:**删除** `/scan` 条目(`ros_topic_name: "/scan"` 那一块 5 行),文件末尾**追加**:

```yaml
- ros_topic_name: "/lidar/points"
  gz_topic_name: "/lidar/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: "GZ_TO_ROS"
```

- [ ] **Step 5: launch TF 更新**

`src/tugbot_gazebo/launch/tugbot_gazebo.launch.py`:把 `scan_tf` 节点改为(name、child-frame 换新;z 保持 0.35):

```python
    scan_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_3d_static_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.35',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'anymal_c/base/lidar_3d',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
```

(变量名 `scan_tf` 保留避免连锁改动;`anymal_c/base/lidar_3d` 是按 gz scoped-name 规律的预期值,Task 5 活体门实测 frame_id 后如不符再校正——相机阶段同款流程。)

- [ ] **Step 6: 跑守卫 + 全套件**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260719
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test/test_lidar_3d.py -v
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -3
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | grep '^FAILED' | sort | diff /tmp/lidar3d_baseline_failures.txt -
```

Expected: 3 PASSED;套件 `7 failed, 422 passed, 3 xfailed`;diff 空。

- [ ] **Step 7: 提交**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260719/src/tugbot_maze/test/test_lidar_3d.py ros2_ws_tugbot_nav_20260719/src/tugbot_description/models/anymal_c/model.sdf ros2_ws_tugbot_nav_20260719/src/tugbot_gazebo/config/tugbot_bridge.yaml ros2_ws_tugbot_nav_20260719/src/tugbot_gazebo/launch/tugbot_gazebo.launch.py
git commit -F - <<'EOF'
feat: swap the 2D scan_omni for the 16-beam lidar_3d

CERBERUS front_laser spec verbatim (1800x16, +/-15deg, 10Hz, 0.05-100m,
gaussian 0.01) with a NEW centered pose (0 0 0.35, yaw 0) preserving the
SCAN_OFFSET_X=0 nav assumption; the original pose is off-center and
rotated and a guard test pins this. Bridge now carries the point cloud
only (the /scan LaserScan entry is gone - /scan is reborn in the
projector node next task); static TF renamed to the new sensor frame.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

(此提交后、Task 5 之前,sim 里暂时没有 /scan——导航跑不了,这是预期的中间态;不要在此间隙跑迷宫回归。)

---

### Task 5: scan_slice_projector 节点 + launch 挂载 + 活体门

**Files:**
- Create: `ros2_ws_tugbot_nav_20260719/src/tugbot_maze/tugbot_maze/scan_slice_projector.py`
- Modify: `ros2_ws_tugbot_nav_20260719/src/tugbot_maze/setup.py`(entry point)
- Modify: `ros2_ws_tugbot_nav_20260719/src/tugbot_gazebo/launch/tugbot_gazebo.launch.py`(挂节点)
- Create: `ros2_ws_tugbot_nav_20260719/tools/lidar3d_check.py`
- Create: `ros2_ws_tugbot_nav_20260719/tools/verify_lidar3d.sh`

- [ ] **Step 1: 写节点**

新建 `scan_slice_projector.py`:

```python
#!/usr/bin/env python3
"""Subscribes /lidar/points (PointCloud2 from the 16-beam lidar) and
republishes the legacy /scan (LaserScan) via the pure slice_to_scan core,
so the 2D-native nav chain runs unchanged on the 3D sensor. One scan out
per cloud in (10Hz). Field values come from SCAN_CONTRACT (captured from
the last 2D boot); header stamp/frame are passed through from the cloud."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

from tugbot_maze.slice_to_scan import SCAN_CONTRACT, fold_to_ranges


class ScanSliceProjector(Node):
    def __init__(self):
        super().__init__('scan_slice_projector')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(PointCloud2, '/lidar/points', self.on_cloud, 5)

    def on_cloud(self, cloud):
        pts = point_cloud2.read_points_numpy(
            cloud, field_names=('x', 'y', 'z'), skip_nans=False)
        c = SCAN_CONTRACT
        msg = LaserScan()
        msg.header = cloud.header
        msg.angle_min = c['angle_min']
        msg.angle_max = c['angle_max']
        msg.angle_increment = c['angle_increment']
        msg.time_increment = c['time_increment']
        msg.scan_time = c['scan_time']
        msg.range_min = c['range_min']
        msg.range_max = c['range_max']
        msg.ranges = [float(r) for r in fold_to_ranges(
            pts, c['n_bins'], c['angle_min'], c['angle_increment'],
            c['range_min'], c['range_max'])]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanSliceProjector()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
```

(若运行时 `read_points_numpy` 不存在或 dtype 不合,回退用 `point_cloud2.read_points` 结构化数组 + `np.stack`;以实测为准,决策记附记。)

- [ ] **Step 2: setup.py entry point**

`src/tugbot_maze/setup.py` 的 console_scripts 列表加一行(`locomotion_controller` 之后):

```python
            'scan_slice_projector = tugbot_maze.scan_slice_projector:main',
```

- [ ] **Step 3: 挂进 tugbot_gazebo.launch.py**

`camera_tf` 节点定义之后加:

```python
    scan_projector = Node(
        package='tugbot_maze',
        executable='scan_slice_projector',
        name='scan_slice_projector',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
```

并把 `scan_projector,` 加进末尾 `LaunchDescription([...])` 列表(`camera_tf,` 之后)。

- [ ] **Step 4: 重建 + 写活体门**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260719
set +u; source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select tugbot_maze tugbot_gazebo tugbot_description
```

新建 `tools/lidar3d_check.py`:

```python
#!/usr/bin/env python3
"""Live gate: /lidar/points must stream 3D clouds and the projected /scan
must reproduce the legacy field contract exactly (the nav chain must not
be able to tell the sensor changed)."""
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2


class Check(Node):
    def __init__(self):
        super().__init__('lidar3d_check')
        self.clouds = []
        self.scans = []
        self.create_subscription(PointCloud2, '/lidar/points', self.on_cloud, 5)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

    def on_cloud(self, m):
        self.clouds.append((time.monotonic(), m))

    def on_scan(self, m):
        self.scans.append((time.monotonic(), m))


def main():
    contract = json.loads((Path(__file__).resolve().parents[1] /
                           'src/tugbot_maze/test/scan_omni_baseline.json').read_text())
    rclpy.init()
    node = Check()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8.0:   # ~80 clouds/scans at 10Hz nominal
        rclpy.spin_once(node, timeout_sec=0.5)
    assert len(node.clouds) >= 5, f'only {len(node.clouds)} clouds in 8s'
    assert len(node.scans) >= 5, f'only {len(node.scans)} scans in 8s'
    span = node.scans[-1][0] - node.scans[0][0]
    hz = (len(node.scans) - 1) / span if span > 0 else 0.0
    cloud = node.clouds[-1][1]
    n_pts = cloud.width * cloud.height
    scan = node.scans[-1][1]
    print(f'clouds={len(node.clouds)} scans={len(node.scans)} scan_hz={hz:.1f} '
          f'cloud_pts={n_pts} cloud_frame={cloud.header.frame_id} scan_frame={scan.header.frame_id}')
    assert 8000 <= n_pts <= 28800, f'cloud size {n_pts}: expected O(1800x16)'
    assert 6.0 <= hz <= 14.0, f'scan rate {hz:.1f}Hz outside 6-14 (nominal 10)'
    assert len(scan.ranges) == contract['n_bins'], f'{len(scan.ranges)} bins'
    for key, got in (('angle_min', scan.angle_min), ('angle_max', scan.angle_max),
                     ('angle_increment', scan.angle_increment),
                     ('time_increment', scan.time_increment),
                     ('scan_time', scan.scan_time),
                     ('range_min', scan.range_min), ('range_max', scan.range_max)):
        assert math.isclose(got, contract[key], abs_tol=1e-7), f'{key}: {got} != {contract[key]}'
    finite = [r for r in scan.ranges if math.isfinite(r)]
    assert len(finite) > 0.2 * len(scan.ranges), f'only {len(finite)} finite bins - projector starving?'
    assert min(finite) < 5.0, 'no nearby wall return - geometry suspicious'
    rclpy.shutdown()
    print('LIDAR3D CHECK PASS')


if __name__ == '__main__':
    main()
```

新建 `tools/verify_lidar3d.sh`(骨架逐行照抄 `tools/verify_front_camera.sh`——PRIME env 块、kill_all_sim、trap EXIT/INT/TERM、SHM 清理、`sleep 18 & wait $!`、world_sdf 钉迷宫世界——仅把 `python3 tools/front_camera_check.py` 换成 `python3 tools/lidar3d_check.py`,日志文件名换 `/tmp/lidar3d_launch.log`),`chmod +x`。

- [ ] **Step 5: 跑活体门 + frame_id 校正闭环**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260719
bash tools/verify_lidar3d.sh
```

Expected: `LIDAR3D CHECK PASS`,cloud_pts≈28800、scan_hz≈10、两个 frame 打印。
1. 若 cloud_frame ≠ `anymal_c/base/lidar_3d`:改 launch 里 `--child-frame-id` 为实测值,重跑;记附记。
2. 相机门回归:`bash tools/verify_front_camera.sh` 仍须 PASS。

- [ ] **Step 6: 全套件 + 提交**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260719
PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -3
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260719/src/tugbot_maze/tugbot_maze/scan_slice_projector.py ros2_ws_tugbot_nav_20260719/src/tugbot_maze/setup.py ros2_ws_tugbot_nav_20260719/src/tugbot_gazebo/launch/tugbot_gazebo.launch.py ros2_ws_tugbot_nav_20260719/tools/lidar3d_check.py ros2_ws_tugbot_nav_20260719/tools/verify_lidar3d.sh
git commit -F - <<'EOF'
feat: scan_slice_projector node + live gate - /scan reborn from 3D

The projector republishes /scan from /lidar/points via the pure core;
mounted in tugbot_gazebo.launch.py at sensor-plumbing level so bare sim
sessions keep /scan. Live gate asserts cloud flow, 10Hz scan, and
field-exact contract reproduction against the committed baseline.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 6: RViz Lidar3D 点云显示

**Files:** Modify: `ros2_ws_tugbot_nav_20260719/src/tugbot_bringup/rviz/tugbot_nav.rviz`

- [ ] **Step 1: Displays 列表末尾(FrontCamera 块的 `      Value: true` 之后、`  Enabled: true` 之前)插入**

```yaml
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 2.0
        Min Value: 0.0
        Value: false
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: AxisColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: Lidar3D
      Position Transformer: XYZ
      Selectable: false
      Size (Pixels): 2
      Size (m): 0.02
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /lidar/points
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
```

(AxisColor 按 z 渐变体现 3D 竖直延展;Value Bounds 0–2m 固定映射,不随远地板点跳变。)

- [ ] **Step 2: 校验**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260719
python3 - <<'EOF'
import yaml
cfg = yaml.safe_load(open('src/tugbot_bringup/rviz/tugbot_nav.rviz'))
d = cfg['Visualization Manager']['Displays']
print([x.get('Name') for x in d])
l3 = [x for x in d if x.get('Name') == 'Lidar3D']
assert len(l3) == 1 and l3[0]['Topic']['Value'] == '/lidar/points'
assert l3[0]['Color Transformer'] == 'AxisColor'
ls = [x for x in d if x.get('Class') == 'rviz_default_plugins/LaserScan']
assert ls and ls[0]['Topic']['Value'] == '/scan', 'legacy LaserScan display must stay'
print('RVIZ CONFIG OK')
EOF
```

Expected: 名单含既有 displays + `Lidar3D`,`RVIZ CONFIG OK`。

- [ ] **Step 3: 提交**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260719/src/tugbot_bringup/rviz/tugbot_nav.rviz
git commit -F - <<'EOF'
feat: RViz Lidar3D point cloud display (z-axis color) on /lidar/points

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 7: headless 完整迷宫回归 ×2(GPU 32× 增量的真实检验)

**Files:** 无代码改动;结果记 spec 附记

- [ ] **Step 1: 连续两次 headless(run 脚本自带 PRIME;后台跑,轮询产物)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260719
bash tools/run_flood_fill_maze.sh 3600 true false online_slam
# 第一跑判定通过后再跑第二次(避免连续失败浪费 2 小时)
```

- [ ] **Step 2: 每跑判定(命令同相机阶段)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260719
python3 tools/replay_collision_oracle.py log/flood_fill_run_<stamp> | tail -3
grep -c 'MATCH rms=nan' log/flood_fill_run_<stamp>/launch.log
grep -o 'MATCH rms=[0-9.]\+' log/flood_fill_run_<stamp>/launch.log | cut -d= -f2 | sort -n | awk '{a[NR]=$1} END {print "n=" NR, "median=" a[int((NR+1)/2)], "p90=" a[int(NR*0.9)]}'
grep -c 'FALL_DETECTED' log/flood_fill_run_<stamp>/launch.log
```

门槛:EXIT_REACHED、oracle **0.000%**、rms median ≤0.05(基线 0.024–0.034)、nan 率与基线域(51–65%)同量级、FALL 0。
**失败预案序**(spec):水平采样 1800→900 → 帧率 10→5Hz → 垂直 16→8 线;每次只动一档,重跑本任务;**不动导航参数、不动投影输出几何**;守卫测试同步更新;决策记附记。判定失败时先按 20260718 方法学排查(nan 单点闪报 vs 真实碰撞签名、nvidia-smi 渲染落位)再动预案。

- [ ] **Step 3: 结果 + 附记 + 提交**

spec 附记补记两跑数据(run stamp、result、oracle、rms/nan、预案是否动用)。

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add docs/superpowers/specs/2026-07-18-lidar-3d-swap-design.md
git commit -F - <<'EOF'
docs: record headless full-maze regression x2 on the 3D lidar

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 8: GUI + RViz 验收跑(用户发起,禁止自动启动)

- [ ] **Step 1: 等用户明确说开跑;届时**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260719
bash tools/run_flood_fill_maze.sh 3600 false true online_slam
```

- [ ] **Step 2: 判定同 Task 7;人工确认 RViz Lidar3D 点云(墙体有竖直延展、随狗移动生长)+ FrontCamera 仍正常;结果记附记**

---

### Task 9: 文档收尾

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260719/README.md`
- Modify: `docs/superpowers/specs/2026-07-18-lidar-3d-swap-design.md`(附记收全)

- [ ] **Step 1: README**:workspace 自述名 20260718→20260719;传感器描述改 3D(lidar_3d 规格、/lidar/points、/scan 由投影节点再生、RViz Lidar3D 面板、verify_lidar3d.sh 活体门);相机节保留。

- [ ] **Step 2: 附记收全(全部实现期偏差)+ 提交**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260719/README.md docs/superpowers/specs/2026-07-18-lidar-3d-swap-design.md
git commit -F - <<'EOF'
docs: README 3D lidar section + spec addendum finalized

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

## 完成后

全部任务 + 用户 GUI 验收通过后:全分支终审(最强模型)→ superpowers:finishing-a-development-branch:套件门(名单对照基线)→ `git merge --no-ff lidar-3d-swap -F <tempfile>`(merge 不读 stdin)→ main 复跑套件 → push origin → 删分支 → memory。
