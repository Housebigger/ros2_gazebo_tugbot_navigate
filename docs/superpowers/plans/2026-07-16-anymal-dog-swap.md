# ANYmal C 机器狗换装 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在新工作空间 `ros2_ws_tugbot_nav_20260716` 中,把迷宫仿真的 tugbot 换成 CERBERUS ANYmal C 机器狗:运动学底盘(VelocityControl+OdometryPublisher)+ 开环 trot 步态动画(12×JointPositionController + gait_animator 节点)+ 背部中心 2D 全向雷达;导航栈只换 `footprint.py` 常量。

**Architecture:** 狗模型 gravity off、腿碰撞体移除、身体碰撞体保留;`/cmd_vel`→VelocityControl(机体系),`/odom`+TF→OdometryPublisher(契约=原 DiffDrive);gait_animator 只读 `/odom`、只写 12 路关节目标,与导航零耦合。spec: `docs/superpowers/specs/2026-07-16-anymal-dog-swap-design.md`。

**Tech Stack:** ROS 2 Jazzy, Gazebo Sim 8.11 (Harmonic), gz-sim system plugins (velocity-control / odometry-publisher / joint-position-controller, 已确认在 `/opt/ros/jazzy/opt/gz_sim_vendor/lib/`), ros_gz_bridge, pytest。

---

## 全局约定(每个任务都要知道)

- **工作空间根**:`/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260716`(Task 1 创建)。下文相对路径均相对此根。仓库根(git 根、spec/plan 所在)是它的上一级目录。
- **构建**:必须 `colcon build --symlink-install`(plain build 会 COPY 安装,破坏 `maze_sim.default_segments_path()`)。构建前 `source /opt/ros/jazzy/setup.bash`。
- **纯模块测试不需构建**:`PYTHONPATH=$PWD/src/tugbot_maze python3 -m pytest src/tugbot_maze/test/... -q`(环境只有 `python3`,没有 `python`)。
- **离线套件基线**(20260712 末态,克隆后应复现):387 passed / 7 failed(7 个 pre-existing:sealed `test_wall_follow_maze_sim.py` + `test_tugbot_maze_contract.py::test_maze_asset_and_config_are_present`)。
- **git 提交**:不在 `git commit -m` 里用反引号;用 heredoc `git commit -F - <<'EOF'`;结尾 trailer `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`。分支 `anymal-dog-swap` 已存在(spec 已提交)。
- **前台 `sleep` 被阻断**:需要等待时把 sleep 写进脚本文件执行,或用 run_in_background。
- **不要自行启动 GUI Gazebo**;headless(`gz sim -s`)冒烟可以。
- **ANYmal 关节命名**:`{LF,RF,LH,RH}_{HAA,HFE,KFE}`(左前/右前/左后/右后 × 髋外展/髋屈伸/膝屈伸)。SDF 中左右腿关节轴已镜像(左 `+1 0 0`、右 `-1 0 0`),HAA 限位 ±0.72/0.49,HFE/KFE 限位 ±9.42(实际无约束)。零位姿 = 四腿垂直下伸,足底在 base 中心下方 0.63 m。

---

### Task 1: 新工作空间 `ros2_ws_tugbot_nav_20260716`

**Files:**
- Create: `ros2_ws_tugbot_nav_20260716/`(整树,rsync 克隆)

- [ ] **Step 1: rsync 克隆 20260712**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
rsync -a --exclude build/ --exclude install/ --exclude log/ \
  ros2_ws_tugbot_nav_20260712/ ros2_ws_tugbot_nav_20260716/
```

- [ ] **Step 2: 构建**

```bash
cd ros2_ws_tugbot_nav_20260716
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
```
Expected: 全部包构建成功,无 error。

- [ ] **Step 3: 基线测试复现**

```bash
PYTHONPATH=$PWD/src/tugbot_maze python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -3
```
Expected: `387 passed, 7 failed`(与 20260712 末态一致;7 个 pre-existing 见全局约定)。

- [ ] **Step 4: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260716
git commit -F - <<'EOF'
chore: seed workspace ros2_ws_tugbot_nav_20260716 from 20260712

rsync clone (excluding build/install/log), baseline suite reproduced
(387 passed / 7 pre-existing failed).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 2: 纯步态模块 `gait.py`(TDD)

**Files:**
- Create: `src/tugbot_maze/tugbot_maze/gait.py`
- Test: `src/tugbot_maze/test/test_gait.py`

- [ ] **Step 1: 写失败测试**

```python
"""Tests for the pure open-loop trot gait module (visual animation only)."""
import math

from tugbot_maze.gait import (
    JOINTS, STAND_POSE, HAA_LIMITS, trot_pose, amplitude_scale, stride_frequency,
)


def test_all_12_joints_present():
    assert sorted(JOINTS) == sorted(
        f'{leg}_{j}' for leg in ('LF', 'RF', 'LH', 'RH') for j in ('HAA', 'HFE', 'KFE'))
    assert sorted(STAND_POSE) == sorted(JOINTS)
    pose = trot_pose(1.234, 0.3, 0.1)
    assert sorted(pose) == sorted(JOINTS)


def test_stand_at_zero_speed():
    pose = trot_pose(2.0, 0.0, 0.0)
    for j in JOINTS:
        assert abs(pose[j] - STAND_POSE[j]) < 1e-9
    assert stride_frequency(0.0, 0.0) == 0.0


def test_periodic_two_pi():
    a = trot_pose(0.7, 0.3, 0.0)
    b = trot_pose(0.7 + 2 * math.pi, 0.3, 0.0)
    for j in JOINTS:
        assert abs(a[j] - b[j]) < 1e-9


def _lift(pose, leg):
    """Knee-flex deviation from stand = swing-lift indicator (>=0 by design)."""
    kfe = f'{leg}_KFE'
    return abs(pose[kfe] - STAND_POSE[kfe])


def test_diagonal_pairs_in_phase_and_antiphase():
    # At phase pi/2 pair A (LF+RH) is mid-swing, pair B (RF+LH) grounded.
    pose = trot_pose(math.pi / 2, 0.4, 0.0)
    assert _lift(pose, 'LF') > 1e-3 and _lift(pose, 'RH') > 1e-3
    assert _lift(pose, 'RF') < 1e-9 and _lift(pose, 'LH') < 1e-9
    assert abs(_lift(pose, 'LF') - _lift(pose, 'RH')) < 1e-9
    # Half a cycle later the roles swap.
    pose2 = trot_pose(3 * math.pi / 2, 0.4, 0.0)
    assert _lift(pose2, 'RF') > 1e-3 and _lift(pose2, 'LH') > 1e-3
    assert _lift(pose2, 'LF') < 1e-9 and _lift(pose2, 'RH') < 1e-9


def test_amplitude_scales_with_speed_and_saturates():
    slow = trot_pose(math.pi / 2, 0.1, 0.0)
    fast = trot_pose(math.pi / 2, 0.4, 0.0)
    faster = trot_pose(math.pi / 2, 2.0, 0.0)
    assert _lift(slow, 'LF') < _lift(fast, 'LF')
    assert abs(_lift(fast, 'LF') - _lift(faster, 'LF')) < 1e-9  # saturated at V_REF
    assert amplitude_scale(0.0, 0.0) == 0.0
    assert amplitude_scale(10.0, 0.0) == 1.0


def test_rotation_only_also_animates():
    pose = trot_pose(math.pi / 2, 0.0, 0.8)
    assert _lift(pose, 'LF') > 1e-3
    assert stride_frequency(0.0, 0.8) > 0.0


def test_haa_within_hardware_limits():
    for phase in (0.0, 1.0, 2.0, 3.0, 4.5, 6.0):
        pose = trot_pose(phase, 5.0, 5.0)
        for j, (lo, hi) in HAA_LIMITS.items():
            assert lo - 1e-9 <= pose[j] <= hi + 1e-9
```

- [ ] **Step 2: 跑测试确认失败**

```bash
cd ros2_ws_tugbot_nav_20260716
PYTHONPATH=$PWD/src/tugbot_maze python3 -m pytest src/tugbot_maze/test/test_gait.py -q
```
Expected: FAIL,`ModuleNotFoundError: No module named 'tugbot_maze.gait'`。

- [ ] **Step 3: 最小实现**

```python
"""Open-loop trot gait for the ANYmal C dog — VISUAL ANIMATION ONLY.

Pure module (no ROS imports). The kinematic base is moved by the Gazebo
VelocityControl plugin; these joint targets exist so the dog visibly walks.
Diagonal pairs LF+RH and RF+LH swing in anti-phase (classic trot). Amplitude
and stride frequency scale with commanded speed and vanish at standstill, so
the dog settles into STAND_POSE when the solver stops.

Sign conventions come from the model SDF (left/right joint axes are already
mirrored: left `+1 0 0`, right `-1 0 0`), so the SAME value commanded to a
left and right joint produces mirror-symmetric motion. Front and hind stand
angles are opposite (X-stance, knees pointing inward). Signs are eyeballed in
the Gazebo GUI during acceptance; the tests here pin structure, not looks.
"""
from __future__ import annotations

import math

JOINTS = [f'{leg}_{j}' for leg in ('LF', 'RF', 'LH', 'RH') for j in ('HAA', 'HFE', 'KFE')]

# X-stance (knees inward). Same value left/right thanks to mirrored axes.
STAND_POSE = {
    'LF_HAA': 0.0, 'LF_HFE': 0.4, 'LF_KFE': -0.8,
    'RF_HAA': 0.0, 'RF_HFE': 0.4, 'RF_KFE': -0.8,
    'LH_HAA': 0.0, 'LH_HFE': -0.4, 'LH_KFE': 0.8,
    'RH_HAA': 0.0, 'RH_HFE': -0.4, 'RH_KFE': 0.8,
}

# Hardware limits from model.sdf (HFE/KFE are +-9.42 rad, effectively free).
HAA_LIMITS = {
    'LF_HAA': (-0.72, 0.49), 'RF_HAA': (-0.49, 0.72),
    'LH_HAA': (-0.72, 0.49), 'RH_HAA': (-0.49, 0.72),
}

V_REF = 0.4        # m/s at which swing amplitude saturates
OMEGA_WEIGHT = 0.5  # rad/s contribution of |omega| toward amplitude
HFE_SWING = 0.25   # rad fore-aft hip swing at full amplitude
KFE_LIFT = 0.35    # rad extra knee flex during swing at full amplitude
F_MIN, F_MAX = 0.8, 2.2  # Hz stride frequency range while moving

_PAIR_PHASE = {'LF': 0.0, 'RH': 0.0, 'RF': math.pi, 'LH': math.pi}


def amplitude_scale(v: float, omega: float) -> float:
    """0..1 swing amplitude from commanded planar speed."""
    return min(1.0, (abs(v) + OMEGA_WEIGHT * abs(omega)) / V_REF)


def stride_frequency(v: float, omega: float) -> float:
    """Stride frequency in Hz; 0 at standstill so the phase freezes."""
    s = amplitude_scale(v, omega)
    if s <= 1e-3:
        return 0.0
    return F_MIN + (F_MAX - F_MIN) * s


def trot_pose(phase: float, v: float, omega: float) -> dict[str, float]:
    """12 joint targets for gait phase [rad] at speed (v, omega).

    Swing legs (sin(leg_phase) > 0) lift by flexing the knee further from
    stand while the hip swings fore-aft; the lift term is >= 0 so feet only
    ever rise from the stand pose (they never dig below ground).
    """
    s = amplitude_scale(v, omega)
    pose: dict[str, float] = {}
    for leg in ('LF', 'RF', 'LH', 'RH'):
        lp = phase + _PAIR_PHASE[leg]
        kfe0 = STAND_POSE[f'{leg}_KFE']
        hind = leg in ('LH', 'RH')
        swing_sign = -1.0 if hind else 1.0
        lift = max(0.0, math.sin(lp))
        pose[f'{leg}_HAA'] = STAND_POSE[f'{leg}_HAA']
        pose[f'{leg}_HFE'] = STAND_POSE[f'{leg}_HFE'] + swing_sign * HFE_SWING * s * math.sin(lp)
        pose[f'{leg}_KFE'] = kfe0 + math.copysign(1.0, kfe0) * KFE_LIFT * s * lift
    for j, (lo, hi) in HAA_LIMITS.items():
        pose[j] = min(hi, max(lo, pose[j]))
    return pose
```

- [ ] **Step 4: 跑测试确认通过**

```bash
PYTHONPATH=$PWD/src/tugbot_maze python3 -m pytest src/tugbot_maze/test/test_gait.py -q
```
Expected: 7 passed。

注意 `test_diagonal_pairs_in_phase_and_antiphase` 的一个坑:phase=π/2 时 pair B 的 leg_phase=3π/2,sin<0 → lift=0 ✓;HFE 却是 sin≠0(负摆),所以 lift 指标只看 KFE——测试就是这么写的,别改成看 HFE。

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260716/src/tugbot_maze/tugbot_maze/gait.py \
        ros2_ws_tugbot_nav_20260716/src/tugbot_maze/test/test_gait.py
git commit -F - <<'EOF'
feat: pure open-loop trot gait module (stand/periodic/anti-phase/saturation)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 3: `anymal_c` 模型(资产复制 + SDF 改造)

**Files:**
- Create: `src/tugbot_description/models/anymal_c/`(meshes/ + thumbnails/ + model.config + model.sdf)
- 源:仓库根 `tmp_resources/CERBERUS_ANYMAL_C_SENSOR_CONFIG_1/`

- [ ] **Step 1: 复制资产并改 model.config**

```bash
cd ros2_ws_tugbot_nav_20260716
cp -r ../tmp_resources/CERBERUS_ANYMAL_C_SENSOR_CONFIG_1 src/tugbot_description/models/anymal_c
```
把 `src/tugbot_description/models/anymal_c/model.config` 的 `<name>` 改为 `anymal_c`(include 按目录名解析,但 name 保持一致避免混淆)。检查 `src/tugbot_description/setup.py`(或 CMakeLists/`package.xml` 的 data_files 安装规则)是否按整个 `models/` 目录通配安装——tugbot 模型是 `models/tugbot/`,确认 `anymal_c` 会一并进 install(`--symlink-install` 下通常是目录级 symlink;构建后 `ls install/tugbot_description/share/tugbot_description/models/` 验证)。

- [ ] **Step 2: SDF 改造——删原生传感器**

编辑 `src/tugbot_description/models/anymal_c/model.sdf`,删除以下 12 个 `<sensor ...>...</sensor>` 整块(mesh visual 不动):
`imu_sensor`、`perception_head_imu_sensor`、`front_laser`、`front_bpearl`、`rear_bpearl`、`camera_0`…`camera_6`。

- [ ] **Step 3: SDF 改造——加 scan_omni(挂 base link)**

在 `<link name="base">` 内(任意 collision 之后)插入,规格逐字对齐 tugbot:

```xml
<sensor name="scan_omni" type="gpu_lidar">
  <pose>0 0 0.35 0 0 0</pose>
  <topic>/scan</topic>
  <always_on>1</always_on>
  <visualize>true</visualize>
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>900</samples>
        <resolution>1</resolution>
        <min_angle>-3.141592654</min_angle>
        <max_angle>3.141592654</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.2</min>
      <max>100.0</max>
      <resolution>0.01</resolution>
    </range>
  </lidar>
</sensor>
```

frame 将是 `anymal_c/base/scan_omni`(model/link/sensor 默认命名;Task 6 的静态 TF 用它)。z=0.35 是初值,Task 4 的自遮挡检查可能上调。

- [ ] **Step 4: SDF 改造——gravity off + 删腿部碰撞体**

- 13 个 link(`base` + 4×`{LF,RF,LH,RH}_HIP` + 4×`_THIGH` + 4×`_SHANK`)每个加子元素 `<gravity>false</gravity>`。
- 删除全部 HIP/THIGH/SHANK link 里的 `<collision>` 块(含 SHANK 里 lumped 的 FOOT 碰撞);**base 的碰撞体全部保留**。

- [ ] **Step 5: SDF 改造——加运动/关节插件**

`</model>` 前插入:

```xml
<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
  <topic>/cmd_vel</topic>
</plugin>
<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
  <odom_frame>odom</odom_frame>
  <robot_base_frame>base_link</robot_base_frame>
  <odom_topic>/odom</odom_topic>
  <tf_topic>/tf</tf_topic>
  <odom_publish_frequency>30</odom_publish_frequency>
  <dimensions>2</dimensions>
</plugin>
```

再加 12 个 JointPositionController(对 `{LF,RF,LH,RH}_{HAA,HFE,KFE}` 每个关节一块,只有 joint_name/topic 不同):

```xml
<plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
  <joint_name>LF_HAA</joint_name>
  <topic>/model/anymal_c/joint/LF_HAA/cmd_pos</topic>
  <use_velocity_commands>true</use_velocity_commands>
</plugin>
```

`use_velocity_commands=true` 绕开力矩 PID(gravity off 下运动学置速到目标),无需调 gain。

- [ ] **Step 6: SDF 校验**

```bash
source /opt/ros/jazzy/setup.bash
gz sdf --check src/tugbot_description/models/anymal_c/model.sdf
```
Expected: `Valid.`(若报 sdf 版本告警可忽略,error 必须清零)。

- [ ] **Step 7: 构建并确认安装**

```bash
colcon build --symlink-install --packages-select tugbot_description
ls install/tugbot_description/share/tugbot_description/models/anymal_c/model.sdf
```
Expected: 文件存在。

- [ ] **Step 8: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260716/src/tugbot_description/models/anymal_c
git commit -F - <<'EOF'
feat: anymal_c model - kinematic base + joint controllers + omni 2D lidar

CERBERUS ANYmal C copy, deeply reworked: native sensors (3 lidars, 7 cams,
2 IMUs) removed, tugbot-spec scan_omni added at body center (frame
anymal_c/base/scan_omni), VelocityControl + OdometryPublisher (DiffDrive
output contract) + 12x JointPositionController (velocity-command mode),
gravity off on all links, leg collisions removed (base collisions kept).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 4: 模型 headless 验证与标定(语义/自遮挡/站高)

**Files:**
- Create: `src/tugbot_gazebo/worlds/anymal_test_world.sdf`(平地 + anymal_c)
- Create: `tools/verify_anymal_model.sh`
- Modify(标定结果): `src/tugbot_description/models/anymal_c/model.sdf`(scan z 如需上调)

- [ ] **Step 1: 写测试世界**

`src/tugbot_gazebo/worlds/anymal_test_world.sdf`(从 `tugbot_empty_world.sdf` 复制骨架,替换 include):

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="anymal_test">
    <physics name="1ms" type="ignored">
      <max_step_size>0.003</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>
        </visual>
      </link>
    </model>
    <include>
      <uri>model://anymal_c</uri>
      <name>anymal_c</name>
      <pose>0 0 0.58 0 0 0</pose>
    </include>
  </world>
</sdf>
```

(若 `tugbot_empty_world.sdf` 的 plugin 骨架与上面不同,以它为准——目的是和现有 gz 版本兼容。)

- [ ] **Step 2: 写验证脚本**

`tools/verify_anymal_model.sh`(核心逻辑;sleep 在脚本内合法):

```bash
#!/usr/bin/env bash
# Headless verification of the anymal_c kinematic model. Run from the workspace root.
set -u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$PWD/install/tugbot_description/share/tugbot_description/models:${GZ_SIM_RESOURCE_PATH:-}"
WORLD=src/tugbot_gazebo/worlds/anymal_test_world.sdf
LOG=$(mktemp -d)
echo "logs in $LOG"
gz sim -s -r "$WORLD" >"$LOG/server.log" 2>&1 &
GZPID=$!
sleep 6

fail=0
check() { if [ "$2" = "ok" ]; then echo "PASS: $1"; else echo "FAIL: $1 ($3)"; fail=1; fi; }

# 1) /scan self-occlusion: on an empty plane every range must be inf/out-of-range.
SCAN=$(timeout 10 gz topic -e -t /scan -n 1 2>/dev/null)
NEAR=$(echo "$SCAN" | grep -oE 'ranges: [0-9.]+' | awk '{if ($2 < 8.0) c++} END {print c+0}')
[ "${NEAR:-1}" -eq 0 ] && check "scan self-occlusion (no return < 8 m)" ok || check "scan self-occlusion" bad "$NEAR near returns -> raise sensor z"

# 2) VelocityControl body-frame semantics: drive +x, then rotate 90deg, drive +x again.
gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.5}' ; sleep 3
gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.0}' ; sleep 1
P1=$(timeout 10 gz topic -e -t /odom -n 1 2>/dev/null)
X1=$(echo "$P1" | grep -A3 'position' | grep 'x:' | head -1 | awk '{print $2}')
gz topic -t /cmd_vel -m gz.msgs.Twist -p 'angular: {z: 0.7854}' ; sleep 2
gz topic -t /cmd_vel -m gz.msgs.Twist -p 'angular: {z: 0.0}' ; sleep 1
gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.5}' ; sleep 3
gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.0}' ; sleep 1
P2=$(timeout 10 gz topic -e -t /odom -n 1 2>/dev/null)
Y2=$(echo "$P2" | grep -A3 'position' | grep 'y:' | head -1 | awk '{print $2}')
python3 - "$X1" "$Y2" <<'PY'
import sys
x1, y2 = float(sys.argv[1]), float(sys.argv[2])
assert x1 > 0.8, f"forward drive moved only {x1:.2f} m"
assert abs(y2) > 0.5, f"post-rotation drive did not move in +y (y={y2:.2f}) -> NOT body-frame"
print("body-frame OK", x1, y2)
PY
[ $? -eq 0 ] && check "VelocityControl body-frame" ok || check "VelocityControl body-frame" bad "see above"

# 3) Joint control: command LF_KFE swing, expect LF_SHANK link pose to change.
B=$(timeout 10 gz topic -e -t /world/anymal_test/dynamic_pose/info -n 1 2>/dev/null | grep -A6 'name: "LF_SHANK"' | grep -m1 'z:' | awk '{print $2}')
gz topic -t /model/anymal_c/joint/LF_KFE/cmd_pos -m gz.msgs.Double -p 'data: -1.6' ; sleep 2
A=$(timeout 10 gz topic -e -t /world/anymal_test/dynamic_pose/info -n 1 2>/dev/null | grep -A6 'name: "LF_SHANK"' | grep -m1 'z:' | awk '{print $2}')
python3 -c "import sys; b,a=float('$B'),float('$A'); assert abs(a-b)>0.02, f'LF_SHANK did not move ({b}->{a})'; print('joint control OK',b,a)" \
  && check "JointPositionController moves LF_SHANK" ok || check "JointPositionController" bad "$B -> $A"

kill $GZPID 2>/dev/null
exit $fail
```

`chmod +x tools/verify_anymal_model.sh`。注意:`dynamic_pose/info` 的报文结构如与 grep 模式不符,先 `gz topic -e -t /world/anymal_test/dynamic_pose/info -n 1 | head -50` 看真实结构再调整解析——**脚本解析可以改,三项判定标准不能删**。

- [ ] **Step 3: 跑验证**

```bash
cd ros2_ws_tugbot_nav_20260716 && bash tools/verify_anymal_model.sh
```
Expected: 三项全 PASS。
- 自遮挡 FAIL → 把 model.sdf 里 scan_omni 的 z 提到 0.45 重试(还不行再 0.55,并记录)。
- body-frame FAIL(旋转后仍沿世界 +x)→ **回退方案**:删 VelocityControl 块,改用原 tugbot 的 DiffDrive 插件 + 两个隐形小轮(此时通知主会话,回退是设计里预案过的,不算偏离)。
- 关节 FAIL → 检查 topic 名与插件块拼写。

- [ ] **Step 4: 站姿/站高标定(数值)**

```bash
python3 - <<'PY'
# Foot bottom in the zero pose is 0.63 m below base (thigh 0.285 + shank-foot 0.345).
# X-stance (HFE 0.4, KFE -0.8 front): drop = 0.285*cos(0.4) + 0.345*cos(0.4) ~= 0.58.
import math
print(round(0.285*math.cos(0.4) + 0.345*math.cos(0.4), 3))
PY
```
用验证世界确认:起 gz、给 12 关节发 STAND_POSE 角(照 Step 2 的 cmd_pos 发法),从 `dynamic_pose/info` 读 4 个 SHANK 的 z,足底 z = SHANK_z − 0.345,应≈0(±0.05,gravity off 悬浮无需精确触地)。据此确定 spawn z(预期 ≈0.58),记下——Task 6 的迷宫世界 include 用同一个 z。若某腿足底明显穿地/离地不对称 → STAND_POSE 符号有误,修 `gait.py` 的 STAND_POSE(前后腿 HFE/KFE 符号翻转再验)。

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260716/src/tugbot_gazebo/worlds/anymal_test_world.sdf \
        ros2_ws_tugbot_nav_20260716/tools/verify_anymal_model.sh \
        ros2_ws_tugbot_nav_20260716/src/tugbot_description/models/anymal_c/model.sdf \
        ros2_ws_tugbot_nav_20260716/src/tugbot_maze/tugbot_maze/gait.py
git commit -F - <<'EOF'
feat: headless model verification (body-frame vel, joints, scan, stance z)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```
(gait.py/model.sdf 仅在标定改动时才会进这次提交。)

---

### Task 5: `gait_animator` 节点 + bridge + launch 接线

**Files:**
- Create: `src/tugbot_maze/tugbot_maze/gait_animator.py`
- Modify: `src/tugbot_maze/setup.py`(entry_points)
- Modify: `src/tugbot_gazebo/config/tugbot_bridge.yaml`(+12 关节条目,−2 相机条目)
- Modify: `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`(+gait_animator 节点)
- Modify: `tools/run_flood_fill_maze.sh`(kill 名单 +gait_animator)

- [ ] **Step 1: 写节点**

`src/tugbot_maze/tugbot_maze/gait_animator.py`:

```python
"""Drives the dog's 12 leg joints with an open-loop trot synced to /odom.

Visual only, zero nav coupling by construction: reads /odom, writes joint
position targets bridged to the model's JointPositionControllers. If this
node dies the dog just stops swinging its legs and glides; navigation is
untouched.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

from tugbot_maze.gait import JOINTS, stride_frequency, trot_pose


class GaitAnimator(Node):
    def __init__(self):
        super().__init__('gait_animator')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('model_name', 'anymal_c')
        model = self.get_parameter('model_name').value
        self._pubs = {
            j: self.create_publisher(Float64, f'/model/{model}/joint/{j}/cmd_pos', 10)
            for j in JOINTS
        }
        self._v = 0.0
        self._omega = 0.0
        self._phase = 0.0
        self._dt = 1.0 / float(self.get_parameter('rate_hz').value)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_timer(self._dt, self._tick)

    def _on_odom(self, msg):
        self._v = msg.twist.twist.linear.x
        self._omega = msg.twist.twist.angular.z

    def _tick(self):
        try:
            f = stride_frequency(self._v, self._omega)
            self._phase = (self._phase + 2.0 * math.pi * f * self._dt) % (2.0 * math.pi)
            for joint, angle in trot_pose(self._phase, self._v, self._omega).items():
                msg = Float64()
                msg.data = float(angle)
                self._pubs[joint].publish(msg)
        except Exception as exc:  # never crash the animation loop
            self.get_logger().warning(f'gait tick failed: {exc}')


def main():
    rclpy.init()
    node = GaitAnimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: setup.py 注册入口**

在 `src/tugbot_maze/setup.py` 的 `entry_points['console_scripts']` 列表加一行(照抄相邻行格式):

```python
'gait_animator = tugbot_maze.gait_animator:main',
```

- [ ] **Step 3: bridge yaml 增删**

`src/tugbot_gazebo/config/tugbot_bridge.yaml`:
- **删**结尾两条 camera 条目(`/camera/image_raw`、`/camera/camera_info`)——狗没有相机。
- **加** 12 条(每个关节一条,ROS→GZ):

```yaml
- ros_topic_name: "/model/anymal_c/joint/LF_HAA/cmd_pos"
  gz_topic_name: "/model/anymal_c/joint/LF_HAA/cmd_pos"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: "ROS_TO_GZ"
```

(joint 名依次:LF_HAA, LF_HFE, LF_KFE, RF_HAA, RF_HFE, RF_KFE, LH_HAA, LH_HFE, LH_KFE, RH_HAA, RH_HFE, RH_KFE。)

- [ ] **Step 4: launch 加节点**

`src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`:仿照 line ~211 的 flood_fill_solver Node 写法,加(无条件启动,所有 pose_source 通用):

```python
gait_animator = Node(
    package='tugbot_maze', executable='gait_animator', name='gait_animator',
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'model_name': 'anymal_c'}],
    output='screen',
)
```

(若该 launch 中 use_sim_time 的传法不同——比如统一 dict——照当地惯例。)并把 `gait_animator` 加进返回的 LaunchDescription 列表。

- [ ] **Step 5: kill 名单**

`tools/run_flood_fill_maze.sh` 的清场 pkill 处(找 `flood_fill_solver` 的那行模式串)把 `gait_animator` 加进匹配模式。

- [ ] **Step 6: 构建 + 冒烟入口存在**

```bash
cd ros2_ws_tugbot_nav_20260716
colcon build --symlink-install --packages-select tugbot_maze tugbot_gazebo tugbot_bringup
source install/setup.bash && ros2 pkg executables tugbot_maze | grep gait_animator
```
Expected: `tugbot_maze gait_animator`。

- [ ] **Step 7: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260716/src/tugbot_maze/tugbot_maze/gait_animator.py \
        ros2_ws_tugbot_nav_20260716/src/tugbot_maze/setup.py \
        ros2_ws_tugbot_nav_20260716/src/tugbot_gazebo/config/tugbot_bridge.yaml \
        ros2_ws_tugbot_nav_20260716/src/tugbot_bringup/launch/tugbot_maze_explore.launch.py \
        ros2_ws_tugbot_nav_20260716/tools/run_flood_fill_maze.sh
git commit -F - <<'EOF'
feat: gait_animator node + 12 joint bridge entries + launch wiring

Reads /odom twist, publishes trot joint targets at 30 Hz. Camera bridge
entries dropped (the dog carries no camera).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 6: 世界换装 + TF/RViz 清理

**Files:**
- Modify: `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf:1404-1408`(include 块)
- Modify: `src/tugbot_gazebo/launch/tugbot_gazebo.launch.py`(scan_tf 参数;删 camera_link_tf/camera_optical_tf)
- Modify: `src/tugbot_bringup/rviz/tugbot_nav.rviz`(删 Image 显示)

- [ ] **Step 1: 世界 include 换 anymal_c**

`tugbot_maze_world_20260528_clean_scaled2x.sdf` line 1404-1408 改为(z 用 Task 4 标定值,预期 0.58):

```xml
<include>
  <uri>model://anymal_c</uri>
  <name>anymal_c</name>
  <pose>-11.011 -9.025 0.58 0.000 0.000 0.000</pose>
</include>
```

- [ ] **Step 2: scan 静态 TF 改狗版**

`src/tugbot_gazebo/launch/tugbot_gazebo.launch.py` 的 `scan_tf` Node 参数改为(z 用 model.sdf 里 scan_omni 最终 pose z,初值 0.35):

```python
arguments=[
    '--x', '0.0', '--y', '0.0', '--z', '0.35',
    '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
    '--frame-id', 'base_link', '--child-frame-id', 'anymal_c/base/scan_omni',
],
```

- [ ] **Step 3: 删相机 TF 节点与 RViz Image 显示**

- 同文件删除 `camera_link_tf` 与 `camera_optical_tf` 两个 Node 定义,及它们在 LaunchDescription 返回列表里的引用。
- `src/tugbot_bringup/rviz/tugbot_nav.rviz`:删 `rviz_default_plugins/Image` 显示块(~line 266-278)及文件头部展开树里的 `/Image1`、`/Image1/Topic1` 两行(~line 13-14)。

- [ ] **Step 4: 残留引用清查**

```bash
cd ros2_ws_tugbot_nav_20260716
grep -rn 'tugbot/scan_omni\|model://tugbot\b\|camera/image_raw' src/ tools/ --include='*.py' --include='*.yaml' --include='*.sdf' --include='*.rviz' --include='*.sh' | grep -v 'models/tugbot/'
```
Expected: 除 `tugbot_nav_world.sdf`/`tugbot_empty_world.sdf`(旧世界,不在本功能路径,保留)外无残留;有则逐个修掉。

- [ ] **Step 5: 构建 + Commit**

```bash
cd ros2_ws_tugbot_nav_20260716 && colcon build --symlink-install --packages-select tugbot_gazebo tugbot_bringup
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260716/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf \
        ros2_ws_tugbot_nav_20260716/src/tugbot_gazebo/launch/tugbot_gazebo.launch.py \
        ros2_ws_tugbot_nav_20260716/src/tugbot_bringup/rviz/tugbot_nav.rviz
git commit -F - <<'EOF'
feat: maze world spawns anymal_c; scan TF and camera plumbing updated

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 7: footprint 常量换装 + 测试修复

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/footprint.py:1-10`
- Modify: `src/tugbot_maze/test/test_footprint.py`
- Modify: `src/tugbot_maze/test/test_maze_sim.py`(碰撞用例注释与期望值)
- Modify(如失败): `src/tugbot_maze/test/test_maze_motion_sim.py`

- [ ] **Step 1: 换常量与文档**

`footprint.py` 头部改为:

```python
"""ANYmal C dog collision footprint + sensor placement (see
tugbot_description/models/anymal_c/model.sdf). All in the base_link frame,
+x=forward, +y=left. Symmetric rectangle over the leg stance (hips at
x=+-0.30, feet at x=+-0.36 / y=+-0.29 plus foot ball 0.03) — no rear-gripper
asymmetry any more. The 2D omni lidar sits at the body centre."""
from __future__ import annotations
import math

FOOT_X_FRONT = 0.45      # front leg stance + margin
FOOT_X_REAR  = -0.45     # symmetric rear
FOOT_HALF_W  = 0.32      # foot lateral stance 0.288 + foot ball radius 0.03
SCAN_OFFSET_X = 0.0      # /scan (scan_omni) at body centre
```

其余函数(beam_endpoint/front_gap/rear_gap/inside_footprint/…)一律不动——消费者(scan_match_localizer、scatter_cloud、radar_occupancy、maze_sim、solver 安全门、oracle)全部经由这些常量自动继承。

- [ ] **Step 2: 跑受影响测试,逐个修期望值**

```bash
cd ros2_ws_tugbot_nav_20260716
PYTHONPATH=$PWD/src/tugbot_maze python3 -m pytest src/tugbot_maze/test/test_footprint.py src/tugbot_maze/test/test_maze_sim.py src/tugbot_maze/test/test_maze_motion_sim.py -q
```

修法(期望值全部按公式重推,不许拍脑袋):
- `test_footprint.py`:边界断言按新常量重算——`inside_footprint(-0.45,0)` 在边界上(实现是 `<=` 则 True,保持断言并加注释);`front_gap`/`rear_gap` 用例的期望间隙 = `墙面x − FOOT_X_FRONT`(或 `FOOT_X_REAR − 墙面x`),且波束端点公式含 `SCAN_OFFSET_X=0.0`(原 -0.1855 的补偿项消失,fixture 里构造的 range 值要跟着变)。
- `test_maze_sim.py` 碰撞用例(~line 130-180):每例注释里写了墙位与面位,新间隙 = 墙位 − 新面位;是否 collide 按 maze_sim 的判定阈值重推,更新断言与注释(如 x=-0.50 的墙:旧 rear -0.468 → 间隙 0.032;新 rear -0.45 → 间隙 0.05)。
- `test_maze_motion_sim.py`:行为级仿真,若失败先看是"期望的碰撞不再发生"(对称足迹更短,合理→更新期望)还是"新碰撞出现"(需报告,不许静默改绿)。

- [ ] **Step 3: 全套件回归**

```bash
PYTHONPATH=$PWD/src/tugbot_maze python3 -m pytest src/tugbot_maze/test -q 2>&1 | tail -3
```
Expected: `X passed, 7 failed`,失败的 7 个与基线完全相同(名单核对),X = 387 − 被删用例 + 新增用例(Task 2 已 +7)。**零新增失败**。

- [ ] **Step 4: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260716/src/tugbot_maze/tugbot_maze/footprint.py \
        ros2_ws_tugbot_nav_20260716/src/tugbot_maze/test/test_footprint.py \
        ros2_ws_tugbot_nav_20260716/src/tugbot_maze/test/test_maze_sim.py \
        ros2_ws_tugbot_nav_20260716/src/tugbot_maze/test/test_maze_motion_sim.py
git commit -F - <<'EOF'
feat: footprint constants for the symmetric ANYmal C stance, SCAN_OFFSET_X=0

All consumers (ICP projection, scatter, radar map, safety gates, oracle)
inherit via the shared constants; expectations re-derived, zero new failures.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 8: headless 整链冒烟(online_slam)

**Files:** 无新文件(产物在 `log/flood_fill_run_<ts>/`)

- [ ] **Step 1: 跑 headless 完整迷宫**

```bash
cd ros2_ws_tugbot_nav_20260716
source /opt/ros/jazzy/setup.bash && source install/setup.bash
bash tools/run_flood_fill_maze.sh 1000 true false online_slam
```
(wrapper 自带清场与超时;跑完看输出目录。)

- [ ] **Step 2: 判定**

```bash
LATEST=$(ls -dt log/flood_fill_run_* | head -1)
cat "$LATEST/result.txt"
grep -c 'COLLISION' "$LATEST"/*.log 2>/dev/null || true
grep -i 'gait tick failed\|gait_animator' "$LATEST/launch.log" | head
```
Expected:
- `result.txt` 含 `EXIT_REACHED`,用时 ≈ 基线(545–575 s 量级);
- oracle 碰撞 0;
- gait_animator 正常启动、无持续报错。
FAIL 时:用 systematic-debugging,先分清是狗模型层(cmd_vel 不动/odom 缺失/scan 缺失,用 Task 4 脚本复查)还是导航层(solver 日志)。**不许为了过冒烟改导航逻辑**——导航逻辑按 spec 不许动,发现要动即停下来报告。

- [ ] **Step 3: Commit(如有修正)+ 汇报**

冒烟通过后向主会话汇报:EXIT_REACHED 用时、碰撞数、gait_animator 状态,等待 GUI 验收指令。

---

## 收尾(主会话执行,非子代理任务)

1. **GUI 验收**:等用户说 "set up the run" 后跑
   `export DISPLAY=:1 && bash tools/run_flood_fill_maze.sh 1000 false true online_slam`,
   用户亲验 spec 的 Success criteria:狗小跑走迷宫、停下站定、无 tugbot 残留、雷达地图/橙散点/绿墙正常、EXIT_REACHED、0 碰撞。步态**观感**问题(符号别扭/幅度不对)在此轮调 `gait.py` 常量(HFE_SWING/KFE_LIFT/F_MIN/F_MAX/STAND_POSE),重验。
2. **whole-change review**(opus 级 code-review)→ 修 findings。
3. **finishing-a-development-branch**:用户验收通过后 `--no-ff` 合回 main、push origin、删分支、更新 memory(`online-slam-maze-solved.md` 加 2026-07-16 追记 + MEMORY.md 索引行)。

## Self-review 记录

- **Spec 覆盖**:模型改造(T3)、运动插件+回退(T3/T4)、scan_omni+TF(T3/T6)、gait 模块+节点+bridge+launch(T2/T5)、世界 include(T6)、footprint(T7)、相机清理(T5/T6)、验证三级(T2/T7 离线、T8 冒烟、收尾 GUI)——spec 各节均有对应任务。
- **占位符**:测试世界 SDF 骨架标注了"以 tugbot_empty_world.sdf 为准";验证脚本解析容错已注明"可改解析、不可删判定"。无 TBD。
- **类型/命名一致性**:`JOINTS`/`STAND_POSE`/`HAA_LIMITS`/`trot_pose`/`stride_frequency`/`amplitude_scale` 在 T2 定义、T5 按同名导入;关节 topic `/model/anymal_c/joint/<J>/cmd_pos` 在 T3(插件)、T4(验证)、T5(节点+bridge)三处一致;frame `anymal_c/base/scan_omni` 在 T3/T6 一致;spawn z 由 T4 标定、T6 引用。
