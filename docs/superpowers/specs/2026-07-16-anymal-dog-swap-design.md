# ANYmal C 机器狗换装(tugbot → 四足机器狗)

**Date:** 2026-07-16
**Workspace:** `ros2_ws_tugbot_nav_20260716`(新建,从 20260712 克隆),分支 `anymal-dog-swap`
**Status:** approved design → implementation plan next

## Problem

20260712 版本(雷达散射地图)验收合并后,用户要求新一轮迭代:**迷宫世界不变,把 tugbot
小车换成机器狗**,模型参照 `tmp_resources/CERBERUS_ANYMAL_C_SENSOR_CONFIG_1`(SubT 比赛
CERBERUS 队的 ANYmal C,含 12 个腿关节、1×16 线雷达、2×半球 bpearl 雷达、7 相机、2 IMU,
**SDF 里没有任何 plugin——不带运动控制器**)。

现有导航栈的机器人契约:gz `/cmd_vel`(Twist,机体系 v+ω)→ 底盘运动;`/odom` +
`odom→base_link` TF;单线全向 2D `/scan`(900 束、±π、0.2–100m、10Hz);
`footprint.py` 的矩形足迹 + `SCAN_OFFSET_X` 投影约定贯穿 ICP 定位、雷达建图、散点云、
front/rear_gap 安全门、真足迹 oracle。

## 用户选定的决策

1. **运动方式 = 运动学底盘 + 步态动画**:底盘用 Gazebo `VelocityControl` 插件直接按
   `/cmd_vel` 运动(导航栈零改动);12 个腿关节播放与实际速度同步的开环对角小跑(trot)
   动画——看起来在走路,物理上是运动学驱动。真实腿式步态控制(CHAMP/ocs2 移植)明确
   排除(数周级独立工程,失稳会毁掉迷宫导航验收)。
2. **传感器 = 加装 2D 全向雷达**:在狗背部正中心加一个与 tugbot `scan_omni` 完全同规格的
   单线全向 gpu_lidar 发 `/scan`;原生 3D 雷达/相机/IMU 的 `<sensor>` 块删除(外观 mesh
   全保留),省 GPU。`SCAN_OFFSET_X = 0.0`。
3. **工作空间**:`ros2_ws_tugbot_nav_20260716`,从 20260712 rsync 克隆
   (`--exclude build/ --exclude install/ --exclude log/`),`colcon build --symlink-install`。
4. 迷宫世界墙体、flood-fill 大脑、online_slam ICP、雷达累积地图、橙色散点云、绿墙、
   Nav2 链、run wrapper 全部不变;适配刻意最小化。

## Architecture

```
Gazebo: anymal_c 模型(gravity off,腿碰撞体移除,身体碰撞体保留)
  ├─ VelocityControl 插件 ← gz /cmd_vel(机体系)        ← bridge ← ROS /cmd_vel(控制链原样)
  ├─ OdometryPublisher 插件 → gz /odom + /tf(odom→base_link) → bridge → ROS(契约=原 DiffDrive)
  ├─ 12 × JointPositionController(每腿关节一个)← gz 关节目标 topic ×12 ← bridge ×12
  └─ scan_omni(单线全向,背部中心)→ gz /scan → bridge → ROS /scan(ICP/建图/散点/安全门原样)

ROS: gait_animator 节点(新):订阅 /odom 实际 twist → 30Hz trot 相位 → 12 路 Float64 关节目标
     (只读 /odom、只写关节目标,与导航零耦合;节点挂了狗只是停止摆腿,导航照常)
```

## Components

### a. `tugbot_description/models/anymal_c/` — 模型副本(新建,深度改造)

从 `tmp_resources/CERBERUS_ANYMAL_C_SENSOR_CONFIG_1` 复制 meshes + model.config + model.sdf,
SDF 副本改造:

- **外观全保留**:身体/四腿/传感器头的全部 `<visual>` mesh 不动。
- **删原生 `<sensor>` 块**:`front_laser`、`front_bpearl`、`rear_bpearl`、`camera_0..6`、
  `imu_sensor`、`perception_head_imu_sensor`(bridge 里对应的两条 camera 条目一并删;若
  RViz 配置引用了 camera 显示则同步删)。
- **加 `scan_omni`**:规格逐字对齐 tugbot(900 束、±π、min 0.2 / max 100m、10Hz、
  `<topic>/scan</topic>`),挂在 base 顶部正中心(x=0, y=0, z≈0.35,实现时精调:须避开
  感知头/bpearl 护笼 mesh,且站姿下绝对高度低于墙顶 ~1.2m)。frame 命名与
  `base_link → <scan frame>` 静态 TF 同步更新(现 child = `tugbot/scan_omni/scan_omni`)。
- **运动插件**:
  - `gz-sim-velocity-control-system`:订阅 gz `/cmd_vel`。⚠️ **前置验证项**:确认
    Harmonic 下其速度语义为机体系(发 v_x 前进应随朝向走);若为世界系,回退方案 =
    隐形双轮 + 原 DiffDrive 插件(契约与 tugbot 逐字一致)。
  - `gz-sim-odometry-publisher-system`:`odom_topic=/odom`、`tf_topic=/tf`、
    `frame=odom`、`child_frame=base_link`、30Hz——对齐原 DiffDrive 输出契约。
  - 12 × `gz-sim-joint-position-controller-system`(LF/RF/LH/RH × HAA/HFE/KFE 各一),
    每个带独立 gz 目标 topic。
- **gravity off(所有 link)+ 腿部碰撞体移除**(HIP/THIGH/SHANK/FOOT 的 `<collision>`
  删除;base 的碰撞体保留):底盘是运动学驱动,不让腿触地反力与控制器打架。
- **出生高度**:悬浮 z = 站姿下足底贴地的 base 高度(按站姿关节角从 SDF 几何算出,
  实现时定数)。

### b. `tugbot_maze/gait.py` — 纯步态模块(新建)

纯 Python,pytest 可直接单测:

```python
STAND_POSE: dict[str, float]          # 12 关节站姿角
def trot_pose(phase, v, omega) -> dict[str, float]: ...
```

- 对角小跑:LF+RH 与 RF+LH 反相(相位差 π);HFE/KFE 正弦摆动,HAA 基本保持站姿角。
- 摆动幅度 ∝ clamp(|v| + k·|ω|),速度≈0 时平滑归位站姿(幅度→0 即站姿)。
- 相位推进由调用方按 `phase += 2π·f(v)·dt` 控制,f(v) ∝ 速度(步频随速度)。
- 所有输出角钳制在 SDF 关节 limit 内。

### c. `gait_animator` 节点(新建,注册进 `tugbot_maze` setup.py)

- 订阅 `/odom`(实际 twist,含平滑后效果),30Hz 定时器推进相位、调 `trot_pose`、
  发布 12 路 `std_msgs/Float64` 关节目标(bridge 新增 12 条 ROS→GZ
  `Float64 ↔ gz.msgs.Double` 条目)。
- 与导航**零耦合 by construction**:只读 `/odom`、只写关节目标;never-crash try/except;
  节点死掉 = 狗停止摆腿滑行,导航不受影响。
- 由 maze explore launch 无条件启动(所有 pose_source 通用,纯视觉)。

### d. 世界接入 + 全局引用修正

- 迷宫世界 SDF(`tugbot_maze_world_20260528_clean_scaled2x.sdf`)只改 include:
  `model://tugbot` → `model://anymal_c`(同出生点 x,y;z=站高;`<name>` 换新名)。
- 全局 grep 修引用:`tugbot/scan_omni`(scan 静态 TF child frame)、`model/tugbot`
  等 gz 实体路径。

### e. `tugbot_maze/footprint.py` — 足迹常量换装

```python
FOOT_X_FRONT = 0.45    # 前腿站位 0.36 + 裕量(对称矩形)
FOOT_X_REAR  = -0.45
FOOT_HALF_W  = 0.32    # 足端横向站位 0.288 + 足球半径 0.03
SCAN_OFFSET_X = 0.0    # 雷达在 base 正中心
```

front/rear_gap 安全门、真足迹 oracle、ICP 投影、散点云、雷达建图**全部自动继承**;
依赖 tugbot 尺寸/偏移的既有测试同步更新。reverse-to-center 等运动微调当年按 tugbot
非对称足迹(尾夹爪 -0.468)调参,对称足迹只会更宽松——离线仿真套件先行验证。

### f. Nav2 / RViz

- Nav2 参数不动(`robot_radius: 0.35` 维持现状;真正的安全门是 solver 的 true-footprint
  逻辑,狗半宽 0.31 与 tugbot 0.29 接近;MPPI 不在控制路径上)。
- RViz 配置仅改 frame/topic 引用(如 scan frame、camera 显示移除),其余不动。

## Error handling

- gait_animator 全体回调 never-crash try/except:失败记 warning,狗最多停止摆腿。
- VelocityControl 语义验证放在最前(冒烟第一步),不通过立即切隐形双轮回退,不影响
  后续任务结构。
- 模型改造后先 `gz sdf --check` / 空世界加载冒烟,再进迷宫。

## Testing

1. **`test_gait.py`(新)**:站姿(v=0 → STAND_POSE)、周期性(phase+2π 同姿)、对角
   反相(LF≡RH、RF≡LH、两对反相)、幅度随速度、限位钳制。
2. **footprint 相关既有测试**更新为狗常量;全离线套件零新增失败(基线 387 passed /
   7 pre-existing failed)。
3. **Gazebo 冒烟(headless,online_slam)**:先验证 VelocityControl 语义(发 v_x 走直线、
   发 ω 原地转),后整链 EXIT_REACHED。
4. **GUI 验收(用户亲验)**:狗以小跑步态走迷宫(速度≈0 时站定),雷达地图/橙散点/绿墙
   正常生长,EXIT_REACHED,oracle 0 碰撞,时长≈基线。

## Out of scope

- 真实腿式步态控制(关节闭环走路)。
- ANYmal 原生 3D 雷达/相机数据的桥接与利用。
- 迷宫世界、求解器大脑、定位/建图算法的任何行为改动。
- 包/topic 的 tugbot→anymal 全局改名(仅改必要引用,包名保持 tugbot_* 连续性)。

## Success criteria

1. GUI 运行中是 **ANYmal C 机器狗**在迷宫里以对角小跑步态行进(停下时站定,不是雕塑滑行
   ——底盘滑行但腿在摆动即达标),无 tugbot 残留。
2. online_slam 全链照常:ICP 定位、雷达累积地图、橙散点、绿墙、EXIT_REACHED、
   oracle 0 碰撞、时长≈基线。
3. 全离线套件零新增失败;新 gait 单测绿。
4. 迷宫世界墙体与求解器行为逐字不变;`scan_match` 等其他 pose_source 不受影响。
