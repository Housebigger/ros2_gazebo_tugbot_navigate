# ackermann-physics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** MR-Buggy3 以真实物理(重力/轮地接触/阿克曼转向,0.41m 最小转弯半径)跑通迷宫,3D 建图质量保持轮式水平(体素 <1.0M)。

**Architecture:** 模型原地升级(恢复原版关节/碰撞/AckermannSteering,Harmonic 命名 + 修 right_joint 重复 bug,传感器/odom/TF 契约零变动);运动层加 **N 点掉头原语**(纯模块 `ackermann_maneuvers.py`,前进/倒车弧交替、游走限界、段间停顿等舵)替换两处原地旋转,`centering_command` 的 rotate-to-face 分支按模式跳过——"停-转-感"FSM 纪律全保。离线 harness 断言 ackermann 模式**永不发出 v=0,ω≠0**。

**Tech Stack:** ROS2 Jazzy, Gazebo Harmonic (gz-sim8), pytest。Spec: `docs/superpowers/specs/2026-07-23-ackermann-physics-design.md`

---

## 硬约束(每个子代理都要遵守)

- 每命令块显式 `cd` 绝对路径;`set +u` 后 source ROS;只有 `python3`;构建必须 `colcon build --symlink-install` **从工作空间内**跑。
- git add 显式路径(禁 `-A`);heredoc `git commit -F - <<'EOF'`;消息禁反引号;trailer `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`;不 push。
- **一切命令前台跑,禁止后台**(子代理后台进程随子代理死,已两次付学费);sim 一律主会话(Task 4/5/6)。
- 红线:`/gt/dynamic_pose` 只许离线评估。
- 不动区:`scan_slice_projector.py`、`slice_to_scan.py`、两定位器、`maze_perception.py`、`flood_fill_brain.py`、`cloud_map_3d.py`、`cloud_map_accumulator.py`、`wall_localize.py`、footprint 常量、桥表、RViz、世界文件(模型 include 不变)。

## 已核实的环境事实(不要重新考证)

- **原版 SDF**(`tmp_resources/MR-Buggy3/model.sdf`):质量 Base 3.0(ixx .0054/iyy .0252/izz .0252)、轮 0.05(ixx 3.331e-5/iyy 2.04e-5/izz 2.04e-5)、转向节 0.005(1.8e-6 全轴);碰撞 = Base 盒 .3×.09×.12 @ z+.06、轮筒 r.0365×l.03(pose 0 ±.015 0 −1.5707 0 0,mu .5/mu2 1.0/fdir1 0 0 1);转向 joint z 轴 ±0.6/effort 25/vel 1.0;轮 joint y 轴无限位;前轮位 (.112, ±.10, 0)、后轮位 (−.1135, ±.10, 0);插件参数 kingpin_width .18/steering_limit .5/wheel_base .2255/wheel_separation .2/wheel_radius .0365/velocity ±100/acceleration ±5/topic /cmd_vel;**原版 bug:right_joint 重复 FrontRight、漏 RearRight**。
- **运动学版**(20260725 `mr_buggy3/model.sdf`,要被替换):单 link `base`,含 lidar_3d(pose 0 0 0.30)与 camera_front(pose 0.12 0 0.15)两传感器块 + OdometryPublisher 插件块——**这三块原样搬进物理版**;VelocityControl 与 `<gravity>false</gravity>` 删除。
- **原地转依赖点**(20260725 `maze_motion.py`):站点 A =行 ~230-240(cardinal align,`profiled_turn_command`,返回 `(0.0, w, False)`);站点 B =行 ~598-608(turn 相,同原语同返回形);`centering_command`(hop_controller.py 28-72)的 rotate-to-face 分支 `if abs(dyaw) > yaw_tol: return (0.0, w, False)`;`backout_command` 直线倒车(Ackermann 兼容,不动)。
- **solver 构造点**:`flood_fill_solver.py` 行 ~78-85,`MazeMotion(self.brain, cruise_v=..., ..., pose_is_absolute=..., mem=self.mem)`。
- **离线 harness**:`test_maze_motion_sim.py` `_run(drift, latency)` 构造 `MazeMotion()`(默认参),MazeSim 纯运动学积分 (v,ω)——它**不会拒绝原地转**,所以 ackermann 语义必须靠"命令断言"锁:`|w|>0.02 ⇒ |v|≥0.01`。
- **测试基线**(20260725):`4 failed / 515 passed / 0 xfailed`,失败=4 个 legacy 资产契约。
- **体素基线**:运动学小车 649k/653k/711k;门线 <1.0M。
- 最大曲率 = tan(0.5)/0.2255 ≈ 2.42;计划取 **2.4**(留 1% 舵限余量)。

---

### Task 0: 新工作空间 + 分支 + 基线

- [ ] **Step 1**(与前两阶段同款):

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git checkout -b ackermann-physics
rsync -a --exclude build --exclude install --exclude log --exclude __pycache__ --exclude .pytest_cache \
  ros2_ws_tugbot_nav_20260725/ ros2_ws_tugbot_nav_20260726/
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260726
set +u; source /opt/ros/jazzy/setup.bash; colcon build --symlink-install 2>&1 | tail -3
source install/setup.bash
python3 -m pytest src -q 2>&1 | tail -2
```

Expected: 6 包 build 成功;`4 failed, 515 passed`(零 xfailed/xpassed;名单=资产契约四件)。

- [ ] **Step 2: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260726
git commit -F - <<'EOF'
chore: clone workspace 20260726 from 20260725 for ackermann-physics

Baseline: build clean, pytest 4 failed / 515 passed / 0 xfailed (the
frozen list: legacy maze-asset contracts only).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 1: 物理版 model.sdf + 契约测试更新(TDD)

**Files:**
- Modify: `$WS/src/tugbot_description/models/mr_buggy3/model.sdf`(整文件替换;$WS=`.../ros2_ws_tugbot_nav_20260726`)
- Modify: `$WS/src/tugbot_maze/test/test_mr_buggy3_contract.py`

- [ ] **Step 1: 先改契约测试(描述物理版,对运动学版必失败)**

在 `test_mr_buggy3_contract.py` 中:**删除** `test_zero_collision_bodies`、`test_gravity_off_on_every_link`、`test_velocity_control_plugin`、`test_no_ackermann_no_ignition_naming` 四个函数,**替换为**以下五个(其余 7 测不动):

```python
def test_physical_collision_bodies():
    sdf = _buggy_sdf()
    assert sdf.count('<collision') == 5          # base box + 4 wheel cylinders
    assert '<box>' in sdf and '<cylinder>' in sdf
    assert '<fdir1>0 0 1</fdir1>' in sdf         # anisotropic wheel friction restored


def test_gravity_on_physical():
    sdf = _buggy_sdf()
    assert '<gravity>false</gravity>' not in sdf  # real physics: gravity everywhere
    assert len(re.findall(r'<link name="[^"]+">', sdf)) == 7   # base + 4 wheels + 2 steering knuckles


def test_no_velocity_control():
    assert 'velocity-control' not in _buggy_sdf()  # kinematic drive removed


def test_ackermann_plugin_harmonic_with_joint_fix():
    sdf = _buggy_sdf()
    assert 'ignition' not in sdf                  # Fortress-era names never load on Harmonic
    p = _block(sdf, r'name="gz::sim::systems::AckermannSteering">(.*?)</plugin>')
    assert 'gz-sim-ackermann-steering-system' in sdf
    lefts = re.findall(r'<left_joint>([^<]+)</left_joint>', p)
    rights = re.findall(r'<right_joint>([^<]+)</right_joint>', p)
    # Upstream SDF listed front_right twice and omitted rear_right -- pinned FIXED here.
    assert lefts == ['front_left_wheel_joint', 'rear_left_wheel_joint']
    assert rights == ['front_right_wheel_joint', 'rear_right_wheel_joint']
    assert '<steering_limit>0.5</steering_limit>' in p
    assert '<wheel_base>.2255</wheel_base>' in p
    assert '<topic>/cmd_vel</topic>' in p


def test_steering_joints_limited():
    sdf = _buggy_sdf()
    for j in ('front_left_steering_joint', 'front_right_steering_joint'):
        b = _block(sdf, r'<joint name="' + j + r'" type="revolute">(.*?)</joint>')
        assert '<lower>-0.6</lower>' in b and '<upper>0.6</upper>' in b
```

跑该文件:5 个新测 FAIL(运动学 SDF 不满足),7 个旧测(model_named/odometry/lidar/camera/world/bridge/tf)仍 PASS。

- [ ] **Step 2: 整文件替换 model.sdf(完整内容)**

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <model name="mr_buggy3">
    <static>false</static>
    <self_collide>false</self_collide>
    <!-- PHYSICAL chassis (ackermann-physics phase): gravity on, real wheel contact,
         gz AckermannSteering (Harmonic naming; the upstream SDF's duplicate
         right_joint bug is FIXED here: front_right + rear_right). Geometry, masses,
         friction ported field-for-field from tmp_resources/MR-Buggy3; links/joints
         renamed dash-free; sensors/odometry kept from the kinematic phase (frames
         unchanged: mr_buggy3/base/lidar_3d, mr_buggy3/base/camera_front).
         Min turn radius = wheel_base/tan(steering_limit) ~= 0.41 m: the motion layer
         compensates with N-point turns (ackermann_maneuvers). -->
    <link name="base">
      <inertial>
        <mass>3.0</mass>
        <inertia>
          <ixx>0.0054</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.0252</iyy><iyz>0</iyz><izz>0.0252</izz>
        </inertia>
      </inertial>
      <visual name="base_visual">
        <pose>.1371 0 -.0115 -0.018 0 1.5707</pose>
        <geometry><mesh><uri>model://mr_buggy3/meshes/BaseReduced.dae</uri></mesh></geometry>
      </visual>
      <visual name="top_visual">
        <pose>.1371 0 -.0115 -0.018 0 1.5707</pose>
        <geometry><mesh><uri>model://mr_buggy3/meshes/TopReduced.dae</uri></mesh></geometry>
      </visual>
      <collision name="base_collision">
        <pose>0 0 .06 0 0 0</pose>
        <geometry><box><size>.3 .09 .12</size></box></geometry>
      </collision>
      <sensor name="lidar_3d" type="gpu_lidar">
        <!-- Centered: SCAN_OFFSET_X=0 contract; identical to the kinematic phase. -->
        <pose>0 0 0.30 0 0 0</pose>
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
      <sensor name="camera_front" type="camera">
        <always_on>1</always_on>
        <update_rate>20</update_rate>
        <pose>0.12 0 0.15 0 0 0</pose>
        <topic>/camera/front/image</topic>
        <camera name="camera_front">
          <horizontal_fov>2.19911</horizontal_fov>
          <camera_info_topic>/camera/front/camera_info</camera_info_topic>
          <lens>
            <intrinsics>
              <fx>183.43</fx>
              <fy>183.43</fy>
              <cx>360.5</cx>
              <cy>270.5</cy>
              <s>0</s>
            </intrinsics>
          </lens>
          <image>
            <width>720</width>
            <height>540</height>
          </image>
          <clip>
            <near>0.02</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
      </sensor>
    </link>
    <link name="front_right_steering">
      <pose relative_to="front_right_steering_joint">0 0 0 0 0 0</pose>
      <inertial>
        <mass>0.005</mass>
        <inertia><ixx>0.0000018</ixx><iyy>0.0000018</iyy><izz>0.0000018</izz></inertia>
      </inertial>
    </link>
    <link name="front_left_steering">
      <pose relative_to="front_left_steering_joint">0 0 0 0 0 0</pose>
      <inertial>
        <mass>0.005</mass>
        <inertia><ixx>0.0000018</ixx><iyy>0.0000018</iyy><izz>0.0000018</izz></inertia>
      </inertial>
    </link>
    <link name="front_right_wheel">
      <pose relative_to="front_right_wheel_joint">0 0 0 0 0 0</pose>
      <inertial>
        <mass>.05</mass>
        <inertia><ixx>0.00003331</ixx><iyy>0.0000204</iyy><izz>0.0000204</izz></inertia>
      </inertial>
      <visual name="front_right_wheel_visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry><mesh><uri>model://mr_buggy3/meshes/Wheel.dae</uri></mesh></geometry>
      </visual>
      <collision name="front_right_wheel_collision">
        <pose>0 .015 0 -1.5707 0 0</pose>
        <geometry><cylinder><length>0.03</length><radius>0.0365</radius></cylinder></geometry>
        <surface>
          <friction>
            <ode><mu>0.5</mu><mu2>1.0</mu2><fdir1>0 0 1</fdir1></ode>
          </friction>
        </surface>
      </collision>
    </link>
    <link name="front_left_wheel">
      <pose relative_to="front_left_wheel_joint">0 0 0 0 0 0</pose>
      <inertial>
        <mass>.05</mass>
        <inertia><ixx>0.00003331</ixx><iyy>0.0000204</iyy><izz>0.0000204</izz></inertia>
      </inertial>
      <visual name="front_left_wheel_visual">
        <pose>0 0 0 0 0 3.14159</pose>
        <geometry><mesh><uri>model://mr_buggy3/meshes/Wheel.dae</uri></mesh></geometry>
      </visual>
      <collision name="front_left_wheel_collision">
        <pose>0 -.015 0 -1.5707 0 0</pose>
        <geometry><cylinder><length>0.03</length><radius>0.0365</radius></cylinder></geometry>
        <surface>
          <friction>
            <ode><mu>0.5</mu><mu2>1.0</mu2><fdir1>0 0 1</fdir1></ode>
          </friction>
        </surface>
      </collision>
    </link>
    <link name="rear_right_wheel">
      <pose relative_to="rear_right_wheel_joint">0 0 0 0 0 0</pose>
      <inertial>
        <mass>.05</mass>
        <inertia><ixx>0.00003331</ixx><iyy>0.0000204</iyy><izz>0.0000204</izz></inertia>
      </inertial>
      <visual name="rear_right_wheel_visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry><mesh><uri>model://mr_buggy3/meshes/Wheel.dae</uri></mesh></geometry>
      </visual>
      <collision name="rear_right_wheel_collision">
        <pose>0 .015 0 -1.5707 0 0</pose>
        <geometry><cylinder><length>0.03</length><radius>0.0365</radius></cylinder></geometry>
        <surface>
          <friction>
            <ode><mu>0.5</mu><mu2>1.0</mu2><fdir1>0 0 1</fdir1></ode>
          </friction>
        </surface>
      </collision>
    </link>
    <link name="rear_left_wheel">
      <pose relative_to="rear_left_wheel_joint">0 0 0 0 0 0</pose>
      <inertial>
        <mass>.05</mass>
        <inertia><ixx>0.00003331</ixx><iyy>0.0000204</iyy><izz>0.0000204</izz></inertia>
      </inertial>
      <visual name="rear_left_wheel_visual">
        <pose>0 0 0 0 0 3.14159</pose>
        <geometry><mesh><uri>model://mr_buggy3/meshes/Wheel.dae</uri></mesh></geometry>
      </visual>
      <collision name="rear_left_wheel_collision">
        <pose>0 -.015 0 -1.5707 0 0</pose>
        <geometry><cylinder><length>0.03</length><radius>0.0365</radius></cylinder></geometry>
        <surface>
          <friction>
            <ode><mu>0.5</mu><mu2>1.0</mu2><fdir1>0 0 1</fdir1></ode>
          </friction>
        </surface>
      </collision>
    </link>
    <joint name="front_right_steering_joint" type="revolute">
      <parent>base</parent>
      <child>front_right_steering</child>
      <pose relative_to="base">.112 -.10 0 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-0.6</lower>
          <upper>0.6</upper>
          <velocity>1.0</velocity>
          <effort>25</effort>
        </limit>
      </axis>
    </joint>
    <joint name="front_left_steering_joint" type="revolute">
      <parent>base</parent>
      <child>front_left_steering</child>
      <pose relative_to="base">.112 .10 0 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-0.6</lower>
          <upper>0.6</upper>
          <velocity>1.0</velocity>
          <effort>25</effort>
        </limit>
      </axis>
    </joint>
    <joint name="front_right_wheel_joint" type="revolute">
      <parent>front_right_steering</parent>
      <child>front_right_wheel</child>
      <pose relative_to="front_right_steering_joint">0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.79769e+308</lower>
          <upper>1.79769e+308</upper>
        </limit>
      </axis>
    </joint>
    <joint name="front_left_wheel_joint" type="revolute">
      <parent>front_left_steering</parent>
      <child>front_left_wheel</child>
      <pose relative_to="front_left_steering_joint">0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.79769e+308</lower>
          <upper>1.79769e+308</upper>
        </limit>
      </axis>
    </joint>
    <joint name="rear_right_wheel_joint" type="revolute">
      <parent>base</parent>
      <child>rear_right_wheel</child>
      <pose relative_to="base">-.1135 -.10 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.79769e+308</lower>
          <upper>1.79769e+308</upper>
        </limit>
      </axis>
    </joint>
    <joint name="rear_left_wheel_joint" type="revolute">
      <parent>base</parent>
      <child>rear_left_wheel</child>
      <pose relative_to="base">-.1135 .10 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.79769e+308</lower>
          <upper>1.79769e+308</upper>
        </limit>
      </axis>
    </joint>
    <plugin filename="gz-sim-ackermann-steering-system" name="gz::sim::systems::AckermannSteering">
      <left_joint>front_left_wheel_joint</left_joint>
      <left_joint>rear_left_wheel_joint</left_joint>
      <right_joint>front_right_wheel_joint</right_joint>
      <right_joint>rear_right_wheel_joint</right_joint>
      <left_steering_joint>front_left_steering_joint</left_steering_joint>
      <right_steering_joint>front_right_steering_joint</right_steering_joint>
      <kingpin_width>.18</kingpin_width>
      <steering_limit>0.5</steering_limit>
      <wheel_base>.2255</wheel_base>
      <wheel_separation>.2</wheel_separation>
      <wheel_radius>0.0365</wheel_radius>
      <min_velocity>-100</min_velocity>
      <max_velocity>100</max_velocity>
      <min_acceleration>-5</min_acceleration>
      <max_acceleration>5</max_acceleration>
      <topic>/cmd_vel</topic>
    </plugin>
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>odom</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <odom_topic>/odom</odom_topic>
      <tf_topic>/tf</tf_topic>
      <odom_publish_frequency>30</odom_publish_frequency>
      <dimensions>3</dimensions>
    </plugin>
  </model>
</sdf>
```

- [ ] **Step 3: 验证 + commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260726
set +u; source /opt/ros/jazzy/setup.bash; source install/setup.bash
python3 -m pytest src/tugbot_maze/test/test_mr_buggy3_contract.py -q 2>&1 | tail -2
```

Expected: `12 passed`(7 旧 + 5 新;lidar/camera/odom/world/bridge/tf 契约在物理版下继续成立)。

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260726/src/tugbot_description/models/mr_buggy3/model.sdf \
        ros2_ws_tugbot_nav_20260726/src/tugbot_maze/test/test_mr_buggy3_contract.py
git commit -F - <<'EOF'
feat: physical mr_buggy3 (gravity, wheel contact, Ackermann steering)

Restores the upstream joints/collisions/friction/inertials field-for-field
with dash-free names, gravity on, VelocityControl removed; AckermannSteering
in Harmonic naming with the upstream duplicate-right_joint bug FIXED
(front_right + rear_right, contract-pinned). Sensors, odometry publisher
and all frames carry over unchanged from the kinematic phase. Contract
tests updated: 5 physical-chassis tests replace the 4 kinematic ones.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 2: `ackermann_maneuvers.py` 纯模块(TDD)

**Files:**
- Create: `$WS/src/tugbot_maze/tugbot_maze/ackermann_maneuvers.py`
- Test: `$WS/src/tugbot_maze/test/test_ackermann_maneuvers.py`(新)

- [ ] **Step 1: 失败测试(完整文件)**

```python
"""N-point-turn planning for the Ackermann chassis: pure math, no ROS."""
import math

from tugbot_maze.ackermann_maneuvers import (
    NPointTurnRunner, plan_n_point_turn, simulate_segments)


def _final_yaw(segs, yaw0=0.0):
    yaw, _ = simulate_segments(segs, yaw0)
    return yaw


def test_plan_empty_when_aligned():
    assert plan_n_point_turn(1.2, 1.2) == []


def test_plan_converges_90_in_two_segments():
    segs = plan_n_point_turn(0.0, math.pi / 2.0)
    assert len(segs) == 2
    assert abs(_final_yaw(segs) - math.pi / 2.0) < 1e-9


def test_plan_converges_180_in_three_segments():
    segs = plan_n_point_turn(0.0, math.pi)
    assert len(segs) == 3
    assert abs(abs(_final_yaw(segs)) - math.pi) < 1e-9


def test_plan_takes_shortest_direction():
    # 3.0 -> -3.0 rad: shortest is +0.283 (through pi), not -6.0.
    segs = plan_n_point_turn(3.0, -3.0)
    total = sum(v * L * k for v, k, L in segs)
    assert 0 < total < 0.3


def test_alternating_drive_direction():
    segs = plan_n_point_turn(0.0, math.pi)
    assert [s[0] for s in segs] == [1, -1, 1]


def test_curvature_at_limit_every_segment():
    for v, k, L in plan_n_point_turn(0.0, math.pi / 2.0, max_curvature=2.4):
        assert abs(k) == 2.4


def test_excursion_bounded():
    for target in (math.pi / 2.0, math.pi, -math.pi / 2.0):
        segs = plan_n_point_turn(0.0, target, excursion_limit=0.5)
        _, exc = simulate_segments(segs, 0.0)
        assert exc <= 0.5 + 1e-9


def test_runner_never_emits_in_place_rotation():
    r = NPointTurnRunner(0.0, math.pi / 2.0, t_now=0.0)
    t, cmds = 0.0, []
    while True:
        v, w, done = r.command(t)
        cmds.append((v, w))
        if done:
            break
        t += 0.1
        assert t < 60.0, 'runner never finished'
    assert any(abs(w) > 0.02 for _, w in cmds)              # it does turn
    for v, w in cmds:
        if abs(w) > 0.02:
            assert abs(v) >= 0.01                            # ... but never in place


def test_runner_pauses_between_segments():
    r = NPointTurnRunner(0.0, math.pi, t_now=0.0, v_mag=0.15, pause_s=0.6)
    t, saw_pause_after_motion = 0.0, False
    moving_prev = False
    while True:
        v, w, done = r.command(t)
        if done:
            break
        if moving_prev and v == 0.0 and w == 0.0:
            saw_pause_after_motion = True                    # steering-rack settle gap
        moving_prev = abs(v) > 0.0
        t += 0.05
    assert saw_pause_after_motion
```

- [ ] **Step 2: 跑测确认 ImportError,然后实现(完整文件)**

```python
"""N-point (K-turn) maneuver planning for the Ackermann chassis.

The gz AckermannSteering plugin cannot rotate in place (v=0, w!=0 stalls the
car; min turn radius = wheel_base/tan(steering_limit) ~= 0.41 m), so heading
changes are executed as alternating forward/reverse arcs at maximum curvature:
forward-left then reverse-right both advance the heading the same way while
their displacements largely cancel, keeping the excursion bounded inside the
2 m cell. Pure math, offline-testable; MazeMotion drives the runner
tick-by-tick and stops early once its own yaw tolerance is met."""
from __future__ import annotations
import math
from typing import List, Tuple

Segment = Tuple[int, float, float]     # (v_sign +-1, signed curvature 1/m, arc length m)

MAX_CURVATURE = 2.4      # tan(0.5)/0.2255 = 2.42; 1% margin under the steering limit
SEG_LEN_M = 0.45         # ~1.08 rad heading change per segment at max curvature
EXCURSION_LIMIT_M = 0.5  # max distance from the turn's start point (2 m cell, walls >=0.88)
TURN_V_MAG = 0.15        # slow maneuver speed: tame smoother ramps + contact transients
PAUSE_S = 0.6            # zero-speed gap between segments: steering rack swings (vel limit 1.0)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def simulate_segments(segs: List[Segment], yaw0: float) -> Tuple[float, float]:
    """(final_yaw, max_excursion): integrate the arc chain from the origin, sampling
    each segment at quarter points (the mid-arc bulge exceeds the chord endpoints)."""
    x = y = 0.0
    yaw = yaw0
    exc = 0.0
    for v_sign, k, L in segs:
        for _ in range(4):
            s = v_sign * (L / 4.0)
            new_yaw = yaw + s * k
            x += (math.sin(new_yaw) - math.sin(yaw)) / k
            y += -(math.cos(new_yaw) - math.cos(yaw)) / k
            yaw = new_yaw
            exc = max(exc, math.hypot(x, y))
    return yaw, exc


def plan_n_point_turn(yaw_now: float, yaw_target: float, *,
                      max_curvature: float = MAX_CURVATURE,
                      seg_len: float = SEG_LEN_M,
                      excursion_limit: float = EXCURSION_LIMIT_M) -> List[Segment]:
    """Alternating forward/reverse max-curvature arcs closing wrap(target-now).
    Shrinks the segment length until the simulated excursion fits the limit."""
    err = _wrap(yaw_target - yaw_now)
    if abs(err) < 1e-9:
        return []
    length = seg_len
    while True:
        segs: List[Segment] = []
        remaining, v_sign = err, 1
        while abs(remaining) > 1e-9:
            dpsi = max(-length * max_curvature, min(length * max_curvature, remaining))
            k = math.copysign(max_curvature, dpsi * v_sign)   # yaw rate v*k must carry sign(dpsi)
            segs.append((v_sign, k, abs(dpsi) / max_curvature))
            remaining -= dpsi
            v_sign = -v_sign
        _, exc = simulate_segments(segs, 0.0)
        if exc <= excursion_limit:
            return segs
        length *= 0.75
        if length < 0.10:      # unreachable for maze-scale turns; guards a bad param set
            raise ValueError('cannot satisfy excursion limit')


class NPointTurnRunner:
    """Executes a planned turn tick-by-tick on wall-clock time: each segment runs
    v = v_sign * v_mag, w = v * curvature for arc_len / v_mag seconds, with a
    zero-speed pause between segments for the steering rack to swing."""

    def __init__(self, yaw_now: float, yaw_target: float, t_now: float, *,
                 v_mag: float = TURN_V_MAG, pause_s: float = PAUSE_S,
                 max_curvature: float = MAX_CURVATURE, seg_len: float = SEG_LEN_M,
                 excursion_limit: float = EXCURSION_LIMIT_M) -> None:
        self.target = yaw_target
        self.v_mag = float(v_mag)
        self.pause_s = float(pause_s)
        self.segs = plan_n_point_turn(yaw_now, yaw_target, max_curvature=max_curvature,
                                      seg_len=seg_len, excursion_limit=excursion_limit)
        self.i = 0
        self.seg_start_t = t_now
        self.pause_until = t_now       # no leading pause

    def command(self, t: float) -> Tuple[float, float, bool]:
        """(v, w, exhausted). exhausted=True once every segment (and trailing pause)
        has run; the caller re-checks its own yaw tolerance and may replan."""
        if t < self.pause_until:
            return (0.0, 0.0, False)
        while self.i < len(self.segs):
            v_sign, k, L = self.segs[self.i]
            dur = L / self.v_mag
            if t - self.seg_start_t < dur:
                v = v_sign * self.v_mag
                return (v, v * k, False)
            self.i += 1
            self.seg_start_t = t + self.pause_s
            self.pause_until = t + self.pause_s
            return (0.0, 0.0, False)
        return (0.0, 0.0, True)
```

- [ ] **Step 3: 全测通过(`10 passed`)+ commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260726/src/tugbot_maze/tugbot_maze/ackermann_maneuvers.py \
        ros2_ws_tugbot_nav_20260726/src/tugbot_maze/test/test_ackermann_maneuvers.py
git commit -F - <<'EOF'
feat: N-point-turn planner and runner for the Ackermann chassis

Alternating forward/reverse max-curvature arcs close any heading change
with bounded in-cell excursion (quarter-point simulation, shrink-to-fit);
the runner executes segments on wall-clock time with steering-rack pauses
between direction flips and never emits in-place rotation. 10 unit tests.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 3: MazeMotion ackermann 分支 + centering 旁路 + solver 参数(TDD)

**Files:**
- Modify: `$WS/src/tugbot_maze/tugbot_maze/maze_motion.py`(构造参数 + 两站点 + 复位)
- Modify: `$WS/src/tugbot_maze/tugbot_maze/hop_controller.py`(centering_command 加 rotate_to_face)
- Modify: `$WS/src/tugbot_maze/tugbot_maze/flood_fill_solver.py`(参数一行 + 构造传参一行,免责条款)
- Test: 追加到 `$WS/src/tugbot_maze/test/test_maze_motion_sim.py` 与 `test_hop_controller.py`

- [ ] **Step 1: 失败测试**

`test_hop_controller.py` 追加:

```python
def test_centering_skips_rotate_to_face_when_disabled():
    # Lateral-only offset at yaw=0 (offset axis perpendicular to heading): with
    # rotate_to_face=True the primitive rotates in place; with False (Ackermann)
    # it must NOT rotate -- it reports done and leaves lateral centering to the
    # corridor pursuit.
    v, w, done = centering_command((0.0, 0.0, 0.0), None, 0.4, rotate_to_face=True)
    assert w != 0.0 and v == 0.0 and not done
    v, w, done = centering_command((0.0, 0.0, 0.0), None, 0.4, rotate_to_face=False)
    assert (v, w, done) == (0.0, 0.0, True)
```

`test_maze_motion_sim.py` 追加:

```python
def test_ackermann_defaults_false():
    import inspect
    from tugbot_maze.maze_motion import MazeMotion
    sig = inspect.signature(MazeMotion.__init__)
    assert sig.parameters['ackermann'].default is False


def test_ackermann_full_solve_never_commands_in_place():
    """Full offline solve with ackermann=True (zero drift): reaches the exit,
    zero collisions, and NO emitted command is an in-place rotation
    (|w|>0.02 with |v|<0.01) -- the constraint the gz AckermannSteering
    plugin physically enforces. MazeSim integrates (v,w) kinematically, so
    the assertion, not the sim, carries the Ackermann semantics."""
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0, inertia=True)
    m = MazeMotion(ackermann=True)
    t, done = 0.0, False
    for _ in range(60000):
        scan = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        v, w, done = m.step(sim.reported_pose, scan, t)
        assert not (abs(w) > 0.02 and abs(v) < 0.01), f'in-place rotation at t={t:.1f}'
        sim.step(v, w, 0.1)
        assert not sim.collides(*sim.pose), f'collision at t={t:.1f}'
        t += 0.1
        if done:
            break
    assert done, 'ackermann offline solve did not reach the exit'
```

(该文件顶部已 import MazeSim/load_segments/cell_center/ENTRANCE_CELL/math——grep 确认,缺则补到既有 import 行。)

- [ ] **Step 2: hop_controller.centering_command 改造**

签名加 `rotate_to_face: bool = True`(放 `yaw_rate` 之后)。决策体改为**候选循环**(替换从 `axis, off = max(cands, ...)` 到函数尾的整段):

```python
    for axis, off in sorted(cands, key=lambda c: -abs(c[1])):
        if axis == 'x':
            dx, dy = (-1.0 if off > 0 else 1.0), 0.0
        else:
            dx, dy = 0.0, (-1.0 if off > 0 else 1.0)
        yaw = pose[2]
        a = math.cos(yaw) * dx + math.sin(yaw) * dy      # +1 centre ahead, -1 behind, 0 perpendicular
        speed = min(v_max, max(v_min, kp_lin * abs(off)))
        # Anti-parallel: reverse-translate (works on diff-drive AND Ackermann).
        if a <= -_PARALLEL_COS:
            return (-speed, 0.0, False)
        want = math.atan2(dy, dx)
        dyaw = _norm(want - yaw)
        if abs(dyaw) > yaw_tol:
            if not rotate_to_face:
                continue          # Ackermann: no in-place facing; try the other axis or give up
            w = kp_ang * dyaw - kd_ang * yaw_rate                     # PD: damp latency overshoot
            return (0.0, max(-w_max, min(w_max, w)), False)
        return (speed, 0.0, False)                                    # aligned -> drive to null
    # Every remaining correction would need an in-place rotation: accept the offset and
    # let the corridor pursuit re-center during the next hop (Ackermann mode).
    return (0.0, 0.0, True)
```

(docstring 补一句 rotate_to_face 语义。)

- [ ] **Step 3: maze_motion 两站点 + 状态**

构造:`__init__` 加 `ackermann: bool = False`(存 `self.ackermann`,注释一行:Ackermann 底盘不能原地转,转向走 N 点程序、居中禁 rotate-to-face);`self._ack_turn = None` 初始化;import 行加 `from tugbot_maze.ackermann_maneuvers import NPointTurnRunner`。

新增私有方法(放 `_stall_event` 之前):

```python
    def _ackermann_turn_cmd(self, yaw, target_cardinal, t):
        """Drive the N-point-turn runner toward target_cardinal. Lazily (re)plans when
        the target changes or the previous program ran out before reaching tolerance
        (the caller's aligned/settled checks decide completion, same as in-place mode)."""
        if self._ack_turn is None or self._ack_turn.target != target_cardinal:
            self._ack_turn = NPointTurnRunner(yaw, target_cardinal, t)
        v, w, exhausted = self._ack_turn.command(t)
        if exhausted:
            self._ack_turn = None      # replan from the current yaw on the next tick
        return v, w
```

站点 A(align,~行 236-240)——原:

```python
            w = profiled_turn_command(yaw, self.latched_cardinal, self.yaw_rate,
                                      ang_decel=self.ang_decel, turn_w_max=self.turn_w_max,
                                      kd=self.kd_turn)
            return (0.0, w, False)
```

改:

```python
            if self.ackermann:
                v, w = self._ackermann_turn_cmd(yaw, self.latched_cardinal, t)
                return (v, w, False)
            w = profiled_turn_command(yaw, self.latched_cardinal, self.yaw_rate,
                                      ang_decel=self.ang_decel, turn_w_max=self.turn_w_max,
                                      kd=self.kd_turn)
            return (0.0, w, False)
```

站点 B(turn 相,~行 605-608)——原:

```python
        w = profiled_turn_command(yaw, self.target_cardinal, self.yaw_rate,
                                  ang_decel=self.ang_decel, turn_w_max=self.turn_w_max,
                                  kd=self.kd_turn)        # decel profile: no latency overshoot
        return (0.0, w, False)
```

改:

```python
        if self.ackermann:
            v, w = self._ackermann_turn_cmd(yaw, self.target_cardinal, t)
            return (v, w, False)
        w = profiled_turn_command(yaw, self.target_cardinal, self.yaw_rate,
                                  ang_decel=self.ang_decel, turn_w_max=self.turn_w_max,
                                  kd=self.kd_turn)        # decel profile: no latency overshoot
        return (0.0, w, False)
```

复位:站点 A 的 `aligned` 判定为真的分支起点、站点 B 的 `(settled ...) or timed_out` 完成分支内、以及 `backout` 进入处,各加一行 `self._ack_turn = None`(grep 定位;确保程序不跨相残留)。

居中调用(行 ~209-212):`centering_command(...)` 实参列表加 `rotate_to_face=not self.ackermann`。

- [ ] **Step 4: solver 两行(免责条款)**

`flood_fill_solver.py`:参数声明区加 `self.ackermann_drive = bool(self.declare_parameter('ackermann_drive', True).value)`;`MazeMotion(...)` 构造加 `ackermann=self.ackermann_drive,`。

- [ ] **Step 5: 验证 + commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260726
set +u; source /opt/ros/jazzy/setup.bash; source install/setup.bash
python3 -m pytest src/tugbot_maze/test/test_hop_controller.py src/tugbot_maze/test/test_ackermann_maneuvers.py -q 2>&1 | tail -2
python3 -m pytest src/tugbot_maze/test/test_maze_motion_sim.py -q 2>&1 | tail -2
python3 -m pytest src -q 2>&1 | tail -2
```

Expected: 前两组全绿;motion 文件全绿(既有 13 + 2 新;ackermann 全解测试离线可跑数十秒);全套件 **`4 failed, 528 passed`**(515 + 1 契约净增 + 10 maneuvers + 2 motion,零 xfailed/xpassed,名单=资产契约四件)。任何偏差 STOP 报告。

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260726/src/tugbot_maze/tugbot_maze/maze_motion.py \
        ros2_ws_tugbot_nav_20260726/src/tugbot_maze/tugbot_maze/hop_controller.py \
        ros2_ws_tugbot_nav_20260726/src/tugbot_maze/tugbot_maze/flood_fill_solver.py \
        ros2_ws_tugbot_nav_20260726/src/tugbot_maze/test/test_maze_motion_sim.py \
        ros2_ws_tugbot_nav_20260726/src/tugbot_maze/test/test_hop_controller.py
git commit -F - <<'EOF'
feat: Ackermann drive mode in the motion layer (N-point turns)

MazeMotion(ackermann=True) routes both in-place-rotation sites (cardinal
align + the turn phase) through the N-point-turn runner and disables the
centering rotate-to-face branch (lateral offsets defer to the corridor
pursuit); completion/timeout criteria unchanged, program state resets on
phase transitions. The offline full-solve test asserts the plugin's
physical constraint end-to-end: no emitted command is ever an in-place
rotation. Solver passes ackermann_drive (param, default True).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 4: 首启冒烟(主会话,240-300s)

- [ ] `bash tools/run_flood_fill_maze.sh 300 true false online_slam`(后台=主会话后台,允许)。
- [ ] 三查(ART=artifact 目录):①`grep -ac "CLOUDMAP voxels=" $ART/launch.log` > 0 且 DIAG 位置推进 ≥3 格(物理车真的在走+转弯完成);②零 `[ERROR]`(尤其 AckermannSteering 插件加载);③转向实证:取 turn 相前后 DIAG,确认 yaw 变化伴随位置弧线移动(非原地)。倒车段的 ω/v 符号行为如与预期不符(车转错方向),取证记录实测约定→按证据修 `segments_to_cmd` 一处(纯模块,重跑单测)→补 commit。任何其他异常停手取证。

---

### Task 5: headless×2 统计验收(主会话)

- [ ] 串行两跑 `bash tools/run_flood_fill_maze.sh 2400 true false online_slam`(物理+掉头变慢,单跑预算提到 2400s;3600s 总约仍富余)。
- [ ] 每跑五道门:EXIT_REACHED;oracle 0.000%;**体素 50k < N < 1.0M**;零 ERROR;MATCH 卡顿census 留档(>5.5s 计数)。
- [ ] 任一门失败停手取证(体素超门先取抖动谱:CLOUDMAP 增速曲线 + 对比运动学跑;oracle 失败先查是否 K turn 游走越界)。两跑全过 → spec 附记落数(体素对照运动学/狗、通关时长、掉头次数与用时)+ commit。

---

### Task 6: GUI 验收(用户门,禁止自启)

- [ ] 等用户明示后:`bash tools/run_flood_fill_maze.sh 2400 false true online_slam`。用户看:真实阿克曼弧线/N 点掉头、轮子真转、车身姿态自然;3D 图与运动学档相当;通关。通过后走 finishing(fable 终审 → merge --no-ff → main 复验 → push → 删分支 → 记忆归档)。

---

## Self-Review 备注

- Spec 覆盖:§4.1→Task 1;§4.2→Tasks 2-3;§4.3 预算→Task 5(2400s);§5 可动面=本计划触碰面一致;§6.1→Tasks 1-3、6.2→Task 4、6.3→Task 5、6.4→Task 6;§7 两处"不臆测"(插件 PID/倒车符号)→Task 4 实录钉定。
- 几何自查:2.4 曲率 × 0.45m 段 = 1.08 rad;90° 两段(1.08+0.49)、180° 三段(1.08+1.08+0.98)与测试断言一致;runner 的段间 pause 实现(advance 时返回一次 (0,0) 并置 pause_until)保证换向零速窗。
- 套件算术:515 +1(契约 11→12)+10(maneuvers)+2(motion)= 528。
- 类型一致:`Segment=(v_sign, curvature, arc_len)` 三元组在 planner/simulate/runner/测试间一致;`command(t) -> (v, w, exhausted)`;`rotate_to_face` 关键字在 hop/maze_motion 调用一致。
- 风险预置:offline 全解测试若因 K turn 慢而超 60000 步,先量步数再调上限(取证,不放宽断言)。
