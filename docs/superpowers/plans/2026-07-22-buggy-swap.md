# buggy-swap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把迷宫机器人从 ANYmal C 换成 MR-Buggy3 运动学底盘小车,导航/建图链行为不变,3D 图表面壳层显著变薄(末态体素 <1.4M vs 狗基线 2.15-2.47M)。

**Architecture:** 运动学配方照抄 20260716 先例:单 link 模型、`<gravity>false</gravity>`、**零碰撞体**、`gz-sim-velocity-control-system`(`/cmd_vel` 直控)+ 世界锚定 `OdometryPublisher`(`base_link` 标签契约)。传感器逐字段移植狗的 16 线雷达(居中,SCAN_OFFSET_X=0)与前视相机。footprint 换小车包络(oracle 诚实)。solver/MazeMotion/定位器/建图链**零改动**。

**Tech Stack:** ROS2 Jazzy, Gazebo Harmonic (gz-sim8), pytest。Spec: `docs/superpowers/specs/2026-07-22-buggy-swap-design.md`

---

## 硬约束(每个子代理都要遵守)

- 每个命令块显式 `cd` 绝对路径;`set +u` 后再 source ROS;只有 `python3`;构建必须 `colcon build --symlink-install`。
- git add 只用显式路径(禁 `-A`);heredoc `git commit -F - <<'EOF'`;消息内禁反引号;结尾 trailer `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`;不 push。
- **sim 一律主会话起**(Task 3/5/6);子代理不得启动任何 sim。GUI(Task 6)等用户明示。
- 红线:`/gt/dynamic_pose` 只许离线评估;POSEDIAG 三重默认关不动。
- 不动区:`scan_slice_projector.py`、`slice_to_scan.py`、两定位器、`maze_perception.py`、`maze_motion.py`、`flood_fill_brain.py`、`hop_controller.py`、`cloud_map_3d.py`、`cloud_map_accumulator.py`、`flood_fill_solver.py`、狗模型 `anymal_c/model.sdf` 及其契约测试(`test_lidar_3d.py`、`test_front_camera.py`、`test_legged_*.py`)、`locomotion_controller.py` 模块文件(只从 launch 摘除)。

## 已核实的环境事实(不要重新考证)

- **运动学配方原文**(20260716 anymal SDF):每 link `<gravity>false</gravity>`;全模型 **0 个 `<collision>`**;`<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"><topic>/cmd_vel</topic></plugin>`;`robot_base_frame=base_link` 是 TF 标签(实际 link 名 `base`)。
- **`/cmd_vel` ROS→GZ 桥已存在**(`tugbot_bridge.yaml` 第 7-11 行),VelocityControl 即插即用。
- **要删的桥表项**:12 条 `/model/anymal_c/joint/*/cmd_pos`(yaml 第 25-95 行区,LF/RF/LH/RH × HAA/HFE/KFE)。
- **世界文件** `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf` 第 1404-1408 行:`<include><uri>model://anymal_c</uri><name>anymal_c</name><pose>-11.011 -9.025 0.62 0 0 0</pose></include>`。
- **静态 TF**(`src/tugbot_gazebo/launch/tugbot_gazebo.launch.py`):`lidar_3d_static_tf` z=0.35 child `anymal_c/base/lidar_3d`(第 72-83 行);`camera_front_static_tf` x=0.45 z=0.20 child `anymal_c/base/camera_front`(第 85-97 行)。gz 传感器帧名规律 = `<模型>/<link>/<传感器>` → 小车为 `mr_buggy3/base/lidar_3d`(Task 3 冒烟实录确认)。
- **locomotion_controller**:`tugbot_maze_explore.launch.py` 第 233-240 行 Node 块 + LaunchDescription 列表里一行 `locomotion_controller,`。
- **狗雷达块**(逐字段移植源,anymal SDF 219-255):1800×16、±0.261799、range 0.05-100/0.01、noise gaussian 0.01、topic `/lidar/points`、10Hz;**狗相机块**(256-291):hfov 2.19911、fx=fy=183.43、cx/cy 360.5/270.5、720×540、noise 0.007、topic `/camera/front/image`、camera_info topic、20Hz。
- **MR-Buggy3 原始几何**(tmp_resources):轮子 joint 位于 Base 系 (±.112/−.1135, ±.10, 0),轮半径 0.0365、碰撞筒长 0.03 外沿 |y|≈0.13;本体盒 0.3×0.09×0.12;原模型自带 pose z=.04。**小车包络:x∈[−0.15, 0.15],y∈[−0.13, 0.13]**。
- **footprint 消费面**:`maze_sim.py`(oracle 矩形/包围圆)、`test_footprint.py`、`test_maze_sim.py`(8 个按狗尺寸手算的夹具,本计划已重算)、`test_wall_follow_maze_sim.py`(仅注释引用,但其 6 个**基线失败**测试跑的是带 footprint 的离线 sim——换小尺寸后可能翻绿,见 Task 4 裁决规则)。`test_front_camera.py`/`test_lidar_3d.py` 只读狗 SDF,不受影响。
- **测试基线**(20260724):`python3 -m pytest src -q` → **10 failed / 494→489 passed / 3 xfailed**……以 20260725 克隆后实测为准,预期 **10 failed / 489 passed / 3 xfailed**,失败名单=cloud-map-3d 阶段冻结的 10 名(4 maze 资产契约 + 6 wall_follow)。
- **狗基线体素**(质量门对照):2,289,212 / 2,470,002 / 2,154,122。

---

### Task 0: 新工作空间 + 分支 + 基线

- [ ] **Step 1: 分支 + 克隆 + 构建 + 基线**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git checkout -b buggy-swap
rsync -a --exclude build --exclude install --exclude log --exclude __pycache__ \
  ros2_ws_tugbot_nav_20260724/ ros2_ws_tugbot_nav_20260725/
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260725
set +u; source /opt/ros/jazzy/setup.bash; colcon build --symlink-install 2>&1 | tail -3
source install/setup.bash
python3 -m pytest src -q 2>&1 | tail -3
```

Expected: build 6 包成功;`10 failed, 489 passed, 3 xfailed`,FAILED 名单与 cloud-map-3d 冻结名单逐名一致(留档)。

- [ ] **Step 2: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260725
git commit -F - <<'EOF'
chore: clone workspace 20260725 from 20260724 for buggy-swap

Baseline: build clean, pytest 10 failed / 489 passed / 3 xfailed (the
frozen historical list from cloud-map-3d, unchanged).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 1: mr_buggy3 运动学模型(TDD)

**Files:**
- Create: `$WS/src/tugbot_description/models/mr_buggy3/model.sdf`、`model.config`、`meshes/{BaseReduced,TopReduced,Wheel}.dae`(从 `tmp_resources/MR-Buggy3/` 拷贝 meshes)
- Test: `$WS/src/tugbot_maze/test/test_mr_buggy3_contract.py`(新)

($WS = `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260725`)

- [ ] **Step 1: 写失败契约测试(完整文件,8 测)**

```python
"""Contract: the mr_buggy3 kinematic chassis. Recipe ported from the archived
anymal kinematic phase (20260716): gravity off, ZERO collision bodies (collision
truth lives in the offline footprint oracle), VelocityControl on /cmd_vel,
world-anchored OdometryPublisher with the base_link TF-label trick; 16-beam
lidar centered (SCAN_OFFSET_X=0) with specs field-identical to the dog's."""
import re
from pathlib import Path


def _ws_src():
    return Path(__file__).resolve().parents[2]


def _buggy_sdf():
    return (_ws_src() / 'tugbot_description' / 'models' / 'mr_buggy3' / 'model.sdf').read_text()


def _dog_sdf():
    return (_ws_src() / 'tugbot_description' / 'models' / 'anymal_c' / 'model.sdf').read_text()


def _block(text, pattern):
    m = re.search(pattern, text, re.S)
    assert m is not None, f'pattern not found: {pattern}'
    return m.group(1)


def test_model_named_mr_buggy3():
    sdf = _buggy_sdf()
    assert '<model name="mr_buggy3">' in sdf     # lowercase, no dash, no leading digit (ROS token rules)
    assert '<static>false</static>' in sdf       # VelocityControl needs a dynamic model


def test_zero_collision_bodies():
    assert '<collision' not in _buggy_sdf()      # kinematic recipe: oracle owns collision truth


def test_gravity_off_on_every_link():
    sdf = _buggy_sdf()
    links = re.findall(r'<link name="[^"]+">', sdf)
    assert len(links) == 1                       # single-link kinematic chassis
    assert sdf.count('<gravity>false</gravity>') == 1


def test_velocity_control_plugin():
    sdf = _buggy_sdf()
    assert 'gz-sim-velocity-control-system' in sdf
    p = _block(sdf, r'name="gz::sim::systems::VelocityControl">(.*?)</plugin>')
    assert '<topic>/cmd_vel</topic>' in p


def test_odometry_publisher_contract():
    p = _block(_buggy_sdf(), r'name="gz::sim::systems::OdometryPublisher">(.*?)</plugin>')
    assert '<odom_frame>odom</odom_frame>' in p
    assert '<robot_base_frame>base_link</robot_base_frame>' in p
    assert '<odom_topic>/odom</odom_topic>' in p
    assert '<tf_topic>/tf</tf_topic>' in p
    assert '<odom_publish_frequency>30</odom_publish_frequency>' in p
    assert '<dimensions>3</dimensions>' in p
    assert 'xyz_offset' not in p                 # archived trap: body-frame post-multiply teleports


def test_no_ackermann_no_ignition_naming():
    sdf = _buggy_sdf()
    assert 'ckermann' not in sdf                 # Ackermann drive removed (kinematic route)
    assert 'ignition' not in sdf                 # Fortress-era plugin names won't load on Harmonic


def test_lidar_centered_specs_match_dog():
    b = _block(_buggy_sdf(), r'<sensor name="lidar_3d" type="gpu_lidar">(.*?)</sensor>')
    d = _block(_dog_sdf(), r'<sensor name="lidar_3d" type="gpu_lidar">(.*?)</sensor>')
    pose = _block(b, r'<pose>([^<]+)</pose>').split()
    assert float(pose[0]) == 0.0 and float(pose[1]) == 0.0   # centered: SCAN_OFFSET_X=0 contract
    assert float(pose[2]) == 0.30                            # mast height (dog was 0.35)
    assert '<topic>/lidar/points</topic>' in b
    for tag in ('lidar',):                                   # scan/range/noise field-identical to the dog
        bb = re.sub(r'\s+', '', _block(b, r'<lidar>(.*?)</lidar>'))
        dd = re.sub(r'\s+', '', _block(d, r'<lidar>(.*?)</lidar>'))
        assert bb == dd


def test_camera_front_forward_and_spec():
    b = _block(_buggy_sdf(), r'<sensor name="camera_front" type="camera">(.*?)</sensor>')
    pose = _block(b, r'<pose>([^<]+)</pose>').split()
    assert float(pose[0]) > 0.0 and abs(float(pose[5])) < 0.3    # faces +x travel direction
    assert '<topic>/camera/front/image</topic>' in b
    assert '<width>720</width>' in b and '<height>540</height>' in b
```

- [ ] **Step 2: 跑测试确认失败**(FileNotFoundError:mr_buggy3/model.sdf)

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260725
set +u; source /opt/ros/jazzy/setup.bash; source install/setup.bash
python3 -m pytest src/tugbot_maze/test/test_mr_buggy3_contract.py -q 2>&1 | tail -3
```

- [ ] **Step 3: 建模型目录 + 拷 meshes + 写 model.config**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
mkdir -p ros2_ws_tugbot_nav_20260725/src/tugbot_description/models/mr_buggy3/meshes
cp tmp_resources/MR-Buggy3/meshes/BaseReduced.dae tmp_resources/MR-Buggy3/meshes/TopReduced.dae \
   tmp_resources/MR-Buggy3/meshes/Wheel.dae \
   ros2_ws_tugbot_nav_20260725/src/tugbot_description/models/mr_buggy3/meshes/
```

`model.config`(完整文件):

```xml
<?xml version="1.0"?>
<model>
  <name>mr_buggy3</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>Benjamin Perseghetti</name>
    <email>bperseghetti@rudislabs.com</email>
  </author>
  <description>NXP MR-Buggy3 rebuilt as a kinematic chassis for the maze project
    (VelocityControl + world-anchored odometry, zero collision bodies, 16-beam
    lidar + front camera ported from the anymal_c specs).</description>
</model>
```

- [ ] **Step 4: 写 model.sdf(完整文件)**

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <model name="mr_buggy3">
    <static>false</static>
    <self_collide>false</self_collide>
    <!-- Kinematic chassis (recipe = the archived 20260716 anymal kinematic phase):
         gravity off + ZERO collision bodies (collision truth lives in the offline
         true-footprint oracle; gravity-off + collisions is an archived attitude-bomb
         trap) + VelocityControl gliding the model at the commanded twist. Wheels are
         fixed decorative visuals -- the Ackermann drive (0.41 m min turn radius, no
         in-place turns) was removed; physical Ackermann is a deferred phase. -->
    <link name="base">
      <gravity>false</gravity>
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
      <visual name="wheel_fr_visual">
        <pose>.112 -.10 0 0 0 0</pose>
        <geometry><mesh><uri>model://mr_buggy3/meshes/Wheel.dae</uri></mesh></geometry>
      </visual>
      <visual name="wheel_fl_visual">
        <pose>.112 .10 0 0 0 3.14159</pose>
        <geometry><mesh><uri>model://mr_buggy3/meshes/Wheel.dae</uri></mesh></geometry>
      </visual>
      <visual name="wheel_rr_visual">
        <pose>-.1135 -.10 0 0 0 0</pose>
        <geometry><mesh><uri>model://mr_buggy3/meshes/Wheel.dae</uri></mesh></geometry>
      </visual>
      <visual name="wheel_rl_visual">
        <pose>-.1135 .10 0 0 0 3.14159</pose>
        <geometry><mesh><uri>model://mr_buggy3/meshes/Wheel.dae</uri></mesh></geometry>
      </visual>
      <sensor name="lidar_3d" type="gpu_lidar">
        <!-- Centered: SCAN_OFFSET_X=0 contract. Specs field-identical to the
             anymal_c lidar_3d; only the mount height differs (0.30 m mast above
             the 0.12 m body: ground ring starts ~1.1 m out, walls painted to
             ~2.4 m at 8 m usable range). -->
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
        <!-- Faces +x = travel direction; perched on the body front, above the
             0.12 m hood so the view is clear. Optics identical to the dog's
             camera_4-spec front camera. -->
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
    <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
      <topic>/cmd_vel</topic>
    </plugin>
    <!-- robot_base_frame=base_link is a TF child-frame LABEL only (the link is
         'base'); kept to preserve the odom->base_link contract for the nav stack
         (same trick as anymal_c). No xyz_offset: body-frame post-multiply
         teleports (archived trap). -->
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

- [ ] **Step 5: 跑测试确认通过 + commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260725
set +u; source /opt/ros/jazzy/setup.bash; source install/setup.bash
python3 -m pytest src/tugbot_maze/test/test_mr_buggy3_contract.py -q 2>&1 | tail -2
```

Expected: `8 passed`。CMakeLists 安装:`tugbot_description/CMakeLists.txt` 已整目录 install `models/`(grep `install(DIRECTORY models` 确认;若逐目录列举则加 mr_buggy3 一行),改后 `colcon build --symlink-install` 重建。

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260725/src/tugbot_description/models/mr_buggy3 \
        ros2_ws_tugbot_nav_20260725/src/tugbot_maze/test/test_mr_buggy3_contract.py
git commit -F - <<'EOF'
feat: mr_buggy3 kinematic chassis model (VelocityControl + odom + sensors)

Single-link gravity-off zero-collision model per the archived 20260716
kinematic recipe: VelocityControl on /cmd_vel, world-anchored
OdometryPublisher (base_link label contract, no xyz_offset), 16-beam lidar
centered at 0.30m mast (specs field-identical to the dog, pinned by
contract test), camera_4-spec front camera, wheels as fixed visuals.
8 contract tests.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

(若 CMakeLists 需改动,把它加进同一 commit 的 git add 列表。)

---

### Task 2: 世界/桥表/静态 TF/launch 接线(TDD)

**Files:**
- Modify: `$WS/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf:1404-1408`
- Modify: `$WS/src/tugbot_gazebo/config/tugbot_bridge.yaml`(删 12 条 anymal joint 项)
- Modify: `$WS/src/tugbot_gazebo/launch/tugbot_gazebo.launch.py`(两个静态 TF)
- Modify: `$WS/src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`(摘 locomotion_controller)
- Test: 追加 3 测到 `$WS/src/tugbot_maze/test/test_mr_buggy3_contract.py`

- [ ] **Step 1: 追加失败测试(3 个,追加到契约文件尾)**

```python
def test_world_spawns_buggy_not_dog():
    world = (_ws_src() / 'tugbot_gazebo' / 'worlds'
             / 'tugbot_maze_world_20260528_clean_scaled2x.sdf').read_text()
    assert 'model://mr_buggy3' in world
    assert 'model://anymal_c' not in world       # dog assets stay in repo but are not spawned
    m = re.search(r'<uri>model://mr_buggy3</uri>\s*<name>mr_buggy3</name>\s*<pose>([^<]+)</pose>', world)
    assert m is not None
    x, y, z = (float(v) for v in m.group(1).split()[:3])
    assert (x, y) == (-11.011, -9.025)           # entrance spawn, same xy as the dog
    assert abs(z - 0.04) < 1e-9                  # wheel radius 0.0365 + clearance (original model's own z)


def test_bridge_has_no_dog_joint_topics():
    bridge = (_ws_src() / 'tugbot_gazebo' / 'config' / 'tugbot_bridge.yaml').read_text()
    assert 'anymal_c/joint' not in bridge        # 12 legged cmd_pos entries removed
    assert '/cmd_vel' in bridge                  # VelocityControl feed stays
    assert '/lidar/points/points' in bridge      # cloud split-topic entry stays


def test_static_tfs_point_at_buggy_frames():
    launch = (_ws_src() / 'tugbot_gazebo' / 'launch' / 'tugbot_gazebo.launch.py').read_text()
    assert 'mr_buggy3/base/lidar_3d' in launch and 'anymal_c/base/lidar_3d' not in launch
    assert 'mr_buggy3/base/camera_front' in launch and 'anymal_c/base/camera_front' not in launch
```

跑该文件确认新增 3 测 FAIL(前 8 个仍 PASS)。

- [ ] **Step 2: 世界 include 换车**

`tugbot_maze_world_20260528_clean_scaled2x.sdf` 第 1404-1408 行:

```xml
    <include>
      <uri>model://mr_buggy3</uri>
      <name>mr_buggy3</name>
      <pose>-11.011 -9.025 0.04 0 0 0</pose>
    </include>
```

- [ ] **Step 3: 桥表删 12 条 joint 项**

`tugbot_bridge.yaml`:删除全部 12 个 `- ros_topic_name: "/model/anymal_c/joint/..."` 条目(每条 6 行含 direction;LF/RF/LH/RH × HAA/HFE/KFE)。其余项(clock/cmd_vel/odom/tf/camera×2/lidar/points/gt)不动。

- [ ] **Step 4: 静态 TF 改帧**

`tugbot_gazebo.launch.py`:`lidar_3d_static_tf` 的 `'--z', '0.35'` → `'--z', '0.30'`,child `anymal_c/base/lidar_3d` → `mr_buggy3/base/lidar_3d`;`camera_front_static_tf` 的 `'--x', '0.45'` → `'--x', '0.12'`、`'--z', '0.20'` → `'--z', '0.15'`,child → `mr_buggy3/base/camera_front`。

- [ ] **Step 5: 摘 locomotion_controller**

`tugbot_maze_explore.launch.py`:删第 233-240 行 Node 块与 LaunchDescription 列表中的 `locomotion_controller,` 一行(模块文件与其测试不动)。

- [ ] **Step 6: 测试 + 全套件 + commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260725
set +u; source /opt/ros/jazzy/setup.bash; colcon build --symlink-install 2>&1 | tail -3
source install/setup.bash
python3 -m pytest src/tugbot_maze/test/test_mr_buggy3_contract.py -q 2>&1 | tail -2
python3 -m pytest src -q 2>&1 | tail -2
python3 -c "import ast; ast.parse(open('src/tugbot_bringup/launch/tugbot_maze_explore.launch.py').read()); ast.parse(open('src/tugbot_gazebo/launch/tugbot_gazebo.launch.py').read()); print('launch syntax ok')"
```

Expected: 契约 `11 passed`;全套件 `10 failed, 500 passed, 3 xfailed`(489+11;名单不变);syntax ok。

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260725/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf \
        ros2_ws_tugbot_nav_20260725/src/tugbot_gazebo/config/tugbot_bridge.yaml \
        ros2_ws_tugbot_nav_20260725/src/tugbot_gazebo/launch/tugbot_gazebo.launch.py \
        ros2_ws_tugbot_nav_20260725/src/tugbot_bringup/launch/tugbot_maze_explore.launch.py \
        ros2_ws_tugbot_nav_20260725/src/tugbot_maze/test/test_mr_buggy3_contract.py
git commit -F - <<'EOF'
feat: wire mr_buggy3 into world/bridge/TF/launch, delaunch locomotion

World include swaps the dog for mr_buggy3 at the entrance (z=0.04); the 12
legged joint bridge entries are removed; static lidar/camera TFs point at
the mr_buggy3 frames (mast z=0.30, camera 0.12/0.15); the quadruped
locomotion_controller leaves the launch (module and its tests stay).
3 new contract tests (world include, bridge cleanliness, TF frames).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 3: 首启冒烟(主会话执行,不派子代理)

- [ ] **Step 1**(主会话后台):`bash tools/run_flood_fill_maze.sh 240 true false online_slam`(240s 短冒烟,预期 TIMEOUT 收尾,不求 EXIT)。
- [ ] **Step 2: 三查**(ART=脚本输出的 artifact 目录):
  - `grep -ac "CLOUDMAP voxels=" $ART/launch.log` **> 0** ——CLOUDMAP 出现 = 点云桥 + `mr_buggy3/base/lidar_3d` 帧名 + `map->odom` TF 全链打通(帧名臆测若错,这里第一时间暴露);
  - `grep -acE "\[ERROR\]" $ART/launch.log` == 0(尤其无 VelocityControl/插件加载错);
  - `grep -a "DIAG" $ART/launch.log | tail -3` 位置随时间变化(cmd_vel 真的在推车)。
- [ ] **Step 3**: 若 CLOUDMAP 恒 0:在主会话单独起 60s 裸 sim,`ros2 topic echo --once /lidar/points | head -3` 实录 frame_id,按实录改静态 TF child(及契约测试),补 commit 后重跑本冒烟。任何其他异常 → 停手取证报告,不带病进 Task 4。

---

### Task 4: footprint 换小车包络 + 受波及测试重算

**Files:**
- Modify: `$WS/src/tugbot_maze/tugbot_maze/footprint.py`(docstring + 3 常量)
- Modify: `$WS/src/tugbot_maze/test/test_footprint.py`(2 处)
- Modify: `$WS/src/tugbot_maze/test/test_maze_sim.py`(8 个夹具重算)

- [ ] **Step 1: footprint.py** —— docstring 全换 + 常量:

```python
"""MR-Buggy3 kinematic chassis collision footprint + sensor placement (see
tugbot_description/models/mr_buggy3/model.sdf). All in the base_link frame,
+x=forward, +y=left. Symmetric rectangle = the full wheel-inclusive envelope
with NO padding (tugbot-era convention: safety margins live in the
gates/thresholds, not in the footprint): wheel centers at x=+0.112/-0.1135
(radius 0.0365 -> extents ~+-0.15), wheel outer faces at |y|=0.13
(0.10 + 0.015 hub offset + 0.015 half-width); the 0.3 x 0.09 body box sits
inside. The 16-beam lidar sits at the body centre (SCAN_OFFSET_X=0)."""
```

```python
FOOT_X_FRONT = 0.15      # front wheel center 0.112 + wheel radius 0.0365, rounded up
FOOT_X_REAR  = -0.15     # rear wheel center -0.1135 - 0.0365, rounded (symmetric)
FOOT_HALF_W  = 0.13      # wheel outer face: 0.10 + 0.015 + 0.015
SCAN_OFFSET_X = 0.0      # /scan projected from the centered 3D lidar
```

- [ ] **Step 2: test_footprint.py 两处**

1. `test_inside_footprint...` 中 `assert inside_footprint(0.5, 0.0, margin=0.3) is True` 行换为:

```python
    assert inside_footprint(0.4, 0.0, margin=0.3) is True   # inflated: front+margin = 0.15+0.3 = 0.45 >= 0.4
```

(同函数上两行的注释里 0.49/0.37 数字同步改为 0.15/0.13。)

2. `test_footprint_covers_gait_envelope` 整个函数替换为:

```python
def test_footprint_covers_buggy_envelope():
    """The footprint rectangle must dominate the mr_buggy3 wheel-inclusive
    envelope (wheel centers x +0.112/-0.1135, radius 0.0365; outer wheel faces
    |y| = 0.10+0.015+0.015). The old legged gait-envelope tie-in (dog envelope
    0.439 x 0.353) retired with the platform swap -- legged/params.py still
    carries it for the archived dog."""
    assert FOOT_X_FRONT >= 0.112 + 0.0365
    assert -FOOT_X_REAR >= 0.1135 + 0.0365
    assert FOOT_HALF_W >= 0.10 + 0.015 + 0.015
```

- [ ] **Step 3: test_maze_sim.py 8 个夹具重算**(小车矩形 [−0.15,0.15]×[−0.13,0.13];包围圆 hypot(0.15,0.13)=0.1985,+0.12=0.3185)

1. `test_collides_true_near_wall_segment`:两个断言与注释换为

```python
    # collides(x,y) with no yaw uses the legacy bounding circle: hypot(FOOT_X_REAR,FOOT_HALF_W) =
    # hypot(0.15,0.13) = 0.1985 + margin 0.12 = 0.3185 (robot_radius_m=0.35 above is unused by the
    # no-yaw path -- see collides()).
    assert sim.collides(0.75, 0.0) is True       # 0.25 m to wall < bounding radius+margin (~0.3185)
    assert sim.collides(0.5, 0.0) is False       # 0.5 m to wall > bounding radius+margin (~0.3185)
```

2. `test_collides_rectangle_catches_rear_gripper`:段换 `[(-0.20, -0.30, -0.20, 0.30)]`,注释:rear face −0.15,0.20−0.15=0.05 < 0.12 → collision(函数名保留——它锤的是"精确矩形抓到包围圆抓不到的后向接触"这个行为)。
3. `test_collides_rectangle_tighter_on_sides_than_circle`:段换 `[(-0.30, 0.26, 0.30, 0.26)]`,注释:0.26−0.13=0.13 > 0.12 → NO collision;而包围圆 0.3185 会把 0.26 视为在内——矩形横向更紧的对照保留。
4. `test_collides_front_wall_within_inflated_front`:段换 `[(0.20, -0.30, 0.20, 0.30)]`,注释:front face 0.15,0.05 < 0.12 → collision。
5. `test_collides_yaw_aware_rotation`:段换 `[(-0.26, -0.30, -0.26, 0.30)]`,注释:yaw=0 时后向净距 0.26−0.15=0.11 < 0.12 会撞;yaw=π/2 时墙在左侧 0.26,侧向净距 0.26−0.13=0.13 > 0.12 → False。断言不变(`is False`)。
6. `test_collides_yaw_none_uses_bounding_circle`:段换 `[(-0.30, -0.30, -0.30, 0.30)]`,注释:0.30 < 0.3185 → True。断言不变。
7. `test_collides_false_when_clear`:代码不动,注释 0.37→0.13、0.63→0.87。
8. `test_collides_exact_rejects_corner_near_miss`:段换 `[(-0.25, -0.23, -0.249, -0.23)]`,注释:corner (−0.15,−0.13) 外每轴偏 0.10 → hypot 0.141 > 0.12 → False。
9. `test_collides_exact_catches_true_corner_contact`:段换 `[(-0.21, -0.19, -0.209, -0.19)]`,注释:每轴偏 0.06 → hypot 0.085 < 0.12 → True。

(`test_step_blocked_by_wall_keeps_position_but_allows_rotation` 攻墙位置恰在墙面上,任何尺寸都拒——不动。)

- [ ] **Step 4: 全套件 + 裁决规则**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260725
set +u; source /opt/ros/jazzy/setup.bash; source install/setup.bash
python3 -m pytest src -q 2>&1 | tail -2
python3 -m pytest src -q 2>&1 | grep FAILED
```

Expected: `10 failed, 500 passed, 3 xfailed`,名单不变。**裁决规则**:小 footprint 可能让 6 个 wall_follow 基线失败翻绿(离线 sim 碰撞变少)——若失败名单**仅在 wall_follow 组内减少**,STOP 不 commit,原样报告翻绿名单等裁决(那是诚实的改善,但冻结名单必须显式重冻,不许静默);名单出现**任何新增**同样 STOP。

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260725/src/tugbot_maze/tugbot_maze/footprint.py \
        ros2_ws_tugbot_nav_20260725/src/tugbot_maze/test/test_footprint.py \
        ros2_ws_tugbot_nav_20260725/src/tugbot_maze/test/test_maze_sim.py
git commit -F - <<'EOF'
feat: swap the collision footprint to the mr_buggy3 wheel envelope

FOOT_X 0.49 -> +-0.15, HALF_W 0.37 -> 0.13 (wheel-inclusive, no padding;
derivation in the docstring). Oracle honesty follows the platform: the
true-footprint oracle and the motion gap gates now measure the real car.
The 8 dog-dimension oracle fixtures in test_maze_sim recomputed for the
buggy rectangle (bounding circle 0.3185); the legged gait-envelope tie-in
retired for a buggy-envelope coverage test.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 5: headless×2 统计验收 + 质量门(主会话执行)

- [ ] **Step 1**(串行两跑):`bash tools/run_flood_fill_maze.sh 1500 true false online_slam`
- [ ] **Step 2: 每跑五道门**:①`result.txt` = EXIT_REACHED;②oracle replay(`python3 tools/replay_collision_oracle.py $ART`)= 0.000%;③末条 `CLOUDMAP voxels=N`:**50,000 < N < 1,400,000**(下界=图在长,上界=**本阶段质量主门**,狗基线 2.15-2.47M 的 ~60% 以下);④零 `[ERROR]`;⑤发布数 `grep -ac "CLOUDMAP"` 留档。
- [ ] **Step 3**: 任一门失败停手取证(质量门失败尤其要取证壳层来源:抽查 launch.log CLOUDMAP 增速曲线,勿直接调 voxel_m 化妆)。两跑全过 → spec 附记落数(体素对照表、通关时长对照)+ commit(docs)。

---

### Task 6: GUI 验收(用户门,禁止自启)

- [ ] 等用户明示后主会话起:`bash tools/run_flood_fill_maze.sh 1500 false true online_slam`。用户看:墙面薄、地面平、无波纹/重影;小车运动平稳;FrontCamera 正常;场景中是小车不是狗。通过口令后走 finishing(fable 全分支终审 → merge --no-ff → main 复验 → push → 删分支 → 记忆归档)。

---

## Self-Review 备注

- Spec 覆盖:§4.1 模型→Task 1;§4.2 世界/桥→Task 2;§4.3 footprint/launch→Task 2+4;§4.4 显示(零改动+RobotModel 非验收项)→无任务(如实);§6 验收 1→Tasks 1/2/4、2→Task 5、3(质量门)→Task 5 门③、4→Task 6;§7 风险:帧名/VelocityControl 首启钉定→Task 3。
- 几何重算自查:小车矩形 0.15/0.13、包围圆 0.1985+0.12=0.3185;9 处夹具的距离-阈值判断逐个手算过(0.05<0.12 撞、0.13>0.12 不撞、0.141>0.12 不撞、0.085<0.12 撞、0.30<0.3185 圆内、0.26 圆内矩形外对照成立)。
- 套件算术:489 +8(Task 1)+3(Task 2)= 500;Task 4 原地改不增减;wall_follow 翻绿风险显式裁决规则。
- 类型/命名一致:`mr_buggy3`、`mr_buggy3/base/lidar_3d`、`mr_buggy3/base/camera_front`、z=0.30/0.04、FOOT 0.15/0.13 在各任务间一致。
