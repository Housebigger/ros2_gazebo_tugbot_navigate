# Tugbot ROS 2 + Gazebo Visual Cruise

一个面向 ROS 2 Jazzy + Gazebo Harmonic 的 tugbot 视觉巡航工作空间。

本工程围绕“最小可运行闭环”组织：

- Gazebo 中加载 tugbot 与测试 world
- 相机图像通过感知节点提取蓝线中心误差
- 控制节点把误差转换为 `/cmd_vel`
- Gazebo 执行运动并回传 `/odom`

在这个最小闭环稳定的基础上，工程同时保留了多 world、多 launch、感知调试、搜索恢复，以及近期完成的 tugbot 模型视觉细节恢复能力。

## 1. 当前状态

当前工作空间已经完成并验证：

- 五包分层架构稳定：`tugbot_description / tugbot_gazebo / tugbot_perception / tugbot_control / tugbot_bringup`
- `sim_minimal.launch.py`、`full_system.launch.py`、`perception_debug.launch.py`、`full_system_zeroerr_outer.launch.py` 均可作为明确分层入口
- 感知层已包含 `detection_hold_frames`
- 控制层已接入 search recovery：`spin -> arc -> alternating arc -> reacquire -> PID`
- `tugbot_description` 已恢复关键视觉细节：logo、警示灯、顶部 lidar、尾部 gripper
- 视觉细节恢复采用“visual-only restore”策略，没有重新引入 legacy sensor 链路
- 当前测试结果：`python3 -m pytest tests -q` -> `30 passed`

## 2. 核心链路

```text
/camera/image_raw
  -> lane_detector_node
  -> /lane_tracking/error
  -> lane_controller_node
  -> /cmd_vel
  -> Gazebo DiffDrive
  -> /odom
```

## 3. 工程结构

```text
ros2_ws_tugbot_20260419/
├── README.md
├── LICENSE
├── CHANGELOG.md
├── CONTRIBUTING.md
├── CODE_OF_CONDUCT.md
├── SECURITY.md
├── .gitignore
├── artifacts/
├── tests/
└── src/
    ├── tugbot_description/
    ├── tugbot_gazebo/
    ├── tugbot_perception/
    ├── tugbot_control/
    └── tugbot_bringup/
```

说明：

- `build/`、`install/`、`log/` 为 colcon 构建产物，不属于源码主体
- `artifacts/` 保存阶段验证脚本、日志和部分截图证据
- 更高层级的经验文档放在仓库上层目录 `../doc/doc_experience/`

## 4. 各包职责

### `tugbot_description`

负责机器人模型与资源安装：

- `models/tugbot/model.sdf`
- `models/tugbot/model.config`
- `models/tugbot/meshes/*`

当前模型策略：

- 保留最小闭环所需 camera + diff drive 架构
- 补回关键外观 visual
- 不恢复 `scan_front`、`scan_back`、`scan_omni`、`camera_back`、`sensor_contact` 等 legacy sensor

### `tugbot_gazebo`

负责 Gazebo worlds 与桥接配置：

- `worlds/tugbot_lane_world.sdf`
- `worlds/tugbot_lane_world_debug.sdf`
- `worlds/tugbot_lane_world_zeroerr.sdf`
- `worlds/tugbot_lane_world_zeroerr_outer.sdf`
- `config/bridge.yaml`

### `tugbot_perception`

负责图像到车道误差：

- 蓝线检测
- 误差计算
- `detection_hold_frames` 短时丢检保持
- `temporal_debug.py` 帧级调试摘要

### `tugbot_control`

负责误差到速度控制：

- PID 控制
- 线速度衰减
- 丢线搜索恢复状态机

### `tugbot_bringup`

负责 launch 编排与调试入口：

- 最小仿真
- 完整闭环
- 感知调试
- zeroerr_outer 专用入口

## 5. Worlds 与 Launch

### Worlds

- `tugbot_lane_world.sdf`
  - formal baseline world
- `tugbot_lane_world_debug.sdf`
  - 感知观察与调试 world
- `tugbot_lane_world_zeroerr.sdf`
  - 带 `zeroerr.png` 纹理底图的 robustness world
- `tugbot_lane_world_zeroerr_outer.sdf`
  - 外环专项验证 world，移除 competing inner visual track
- `tugbot_lane_world_zeroerr_outer_probe_oldpose.sdf`
  - 历史 pose / mapping probe world

### Launch 入口

- `sim_minimal.launch.py`
  - 仅启动 Gazebo、bridge、image bridge
- `full_system.launch.py`
  - 在最小仿真基础上再启动感知与控制
- `perception_debug.launch.py`
  - 启动 Gazebo + 感知 + RViz，不启动控制
- `full_system_zeroerr_outer.launch.py`
  - 对 `full_system.launch.py` 的 world 包装入口，默认切到 `tugbot_lane_world_zeroerr_outer.sdf`

## 6. 运行环境

推荐环境：

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic

工作空间依赖的关键 ROS 组件包括：

- `ros_gz_sim`
- `ros_gz_bridge`
- `ros_gz_image`
- `rviz2`
- `rclpy`

## 7. 快速开始

### 7.1 构建

```bash
cd ~/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_20260419
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

### 7.2 Source 环境

```bash
cd ~/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_20260419
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 7.3 启动最小仿真

```bash
ros2 launch tugbot_bringup sim_minimal.launch.py
```

### 7.4 启动 formal world 完整系统

```bash
ros2 launch tugbot_bringup full_system.launch.py
```

### 7.5 启动 textured robustness world

```bash
WORLD_PREFIX=$(ros2 pkg prefix tugbot_gazebo)/share/tugbot_gazebo/worlds
ros2 launch tugbot_bringup full_system.launch.py \
  world_sdf:=$WORLD_PREFIX/tugbot_lane_world_zeroerr.sdf
```

### 7.6 启动 zeroerr_outer 专用入口

```bash
ros2 launch tugbot_bringup full_system_zeroerr_outer.launch.py
```

### 7.7 启动感知调试

```bash
ros2 launch tugbot_bringup perception_debug.launch.py
```

## 8. 关键参数

### 感知层参数

文件：`src/tugbot_perception/config/perception.yaml`

- `blue_threshold: 80`
- `blue_margin: 20`
- `crop_top_ratio: 0.45`
- `detection_hold_frames: 3`
- `debug_image_enabled: true`

### 控制层参数

文件：`src/tugbot_control/config/control.yaml`

- `error_timeout_sec: 0.4`
- `kp: 1.2`
- `ki: 0.08`
- `kd: 0.05`
- `base_linear_speed: 0.25`
- `min_linear_speed: 0.05`
- `max_angular_speed: 1.2`
- `search_enabled: true`
- `search_spin_angular_speed: 0.35`
- `search_spin_revolution_target: 6.283185307179586`
- `search_arc_linear_speed: 0.08`
- `search_arc_angular_speed: 0.20`
- `search_arc_revolution_target: 6.283185307179586`
- `search_default_turn_direction: 1.0`

## 9. tugbot 模型视觉细节恢复说明

本工程近期完成了一次重要模型修复：把简化版 tugbot 的关键外观细节补回，同时保持当前最小闭环不变。

### 恢复内容

恢复的视觉件包括：

- `movai_logo_visual`
- `beacon_led_visual`
- `beacon_cover_visual`
- `top_lidar_base_visual`
- `top_lidar_sensor_visual`
- `top_lidar_scan_visual`
- `rear_gripper_visual`
- `rear_gripper_hand_visual`

补回的关键 mesh 资产包括：

- `meshes/light_link/light.stl`
- `meshes/light_link/light_led.stl`
- `meshes/VLP16_base_1.dae`
- `meshes/VLP16_base_2.dae`
- `meshes/VLP16_scan.dae`
- `meshes/gripper2/gripper2.dae`
- `meshes/gripper2/gripper_hand.stl`

### 修复边界

这次修复是“恢复视觉细节”，不是恢复旧工程完整传感器树。以下旧结构仍然明确不在当前最小模型中：

- `camera_back`
- `scan_front`
- `scan_back`
- `scan_omni`
- `sensor_contact`

### 为什么这样做

因为当前工程的目标是维护一个清晰、稳定、可继续演进的 ROS 2 + Gazebo 最小闭环母体。直接把旧模型完整搬回，会把当前已经稳定的最小链路重新拉回复杂状态。

## 10. 测试与验证

运行测试：

```bash
python3 -m pytest tests -q
```

当前结果：

- `30 passed`

当前测试覆盖重点包括：

- workspace 文件布局契约
- launch 分层契约
- formal / debug / zeroerr / zeroerr_outer world 契约
- 感知蓝线检测与误差计算
- uniform gray 不误判为蓝线
- `detection_hold_frames` 丢检保持逻辑
- PID 输出方向与限幅
- search recovery：spin / arc / flip / reacquire
- `temporal_debug.py` 的帧级摘要与转换窗口提取
- tugbot 模型视觉细节恢复契约
- 恢复 mesh 资产存在性检查

## 11. 验证产物

目录：`artifacts/`

其中包含两类主要产物：

1. 搜索恢复证据
   - `cold_start_search_trace.py`
   - `cold_start_search_trace.json`
   - `cold_start_search_launch.log`
   - `repeat_cold_start_search_trace.py`
   - `repeat_cold_start_search_summary.json`

2. 模型视觉修复过程中的 GUI / 窗口抓图证据
   - `root_capture.png`
   - `gazebo_window_capture.png`
   - `gazebo_xwd.png`
   - `tugbot_crop.png`

说明：在 Wayland / X11 / EGL / VMware 组合环境下，Gazebo 截图服务可能返回成功但文件不落地，因此模型修复验证不能只依赖 GUI screenshot，仍需结合 world SDF、日志与人工观察。

## 12. 经验文档

阶段经验已沉淀到仓库上层文档目录：

- `../doc/doc_experience/zeroerr_outer_search_recovery_cold_start.md`
- `../doc/doc_experience/tugbot_model_visual_detail_restoration_20260422.md`

建议在继续调整 world、感知阈值、spawn pose、模型 visual 或搜索恢复逻辑之前，先阅读这些文档，避免重复踩坑。

## 13. 已知边界

- 当前工作空间更偏向研究与验证母体，而不是完整产品化系统
- 图形环境如果缺少稳定 3D 加速，Gazebo GUI 截图可能不稳定
- 当前模型修复只覆盖对画面辨识度影响最大的关键视觉件，尚未追求百分之百复刻旧版全细节结构
- 当前包元数据中 maintainer 信息仍沿用工作空间占位配置；如果要正式对外发布，请同步检查并完善 package metadata

## 14. 开源协作

如果你希望在此基础上继续扩展：

- 提交 PR 前请先阅读 `CONTRIBUTING.md`
- 社区行为要求请见 `CODE_OF_CONDUCT.md`
- 安全问题上报方式请见 `SECURITY.md`
- 变更历史请见 `CHANGELOG.md`

## 15. 许可

本工作空间采用 MIT License，详见 `LICENSE`。
