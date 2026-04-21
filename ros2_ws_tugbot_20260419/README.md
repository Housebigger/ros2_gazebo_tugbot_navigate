# Tugbot ROS 2 + Gazebo Visual Cruise

一个面向 ROS 2 Jazzy + Gazebo Harmonic 的 tugbot 视觉巡航重构工程。

本项目从零重建了一个分层清晰、可持续演进的最小闭环母体，用于验证：
- 相机图像感知
- 蓝线误差提取
- 控制输出 `/cmd_vel`
- Gazebo 执行与 `/odom` 回传
- formal / debug / textured / outer-loop 多场景联调

当前状态：
- 5 包分层架构稳定
- `full_system_zeroerr_outer.launch.py` 已加入
- `detection_hold_frames` 已加入感知层
- search recovery 已加入控制层
- `python3 -m pytest tests/ -q` -> `28 passed`

## 核心链路

`/camera/image_raw -> lane_detector_node -> /lane_tracking/error -> lane_controller_node -> /cmd_vel`

## 特性

- 清晰分层：description / gazebo / perception / control / bringup
- 参数外置：感知与控制参数全部放在 YAML
- launch 分层：最小仿真、完整系统、感知调试、zeroerr_outer 专用入口
- 多场景 world：formal、debug、textured robustness、outer-loop-only
- 感知稳态增强：
  - 修复 numpy/uint8 蓝色阈值整数回绕误判
  - 引入 `detection_hold_frames`
  - 提供 `temporal_debug.py` 做逐帧摘要与转换窗口分析
- 控制恢复策略：
  - `spin -> arc -> alternating arc -> reacquire -> PID`
- 已有 cold-start / repeat trace 产物，可直接复核搜索恢复证据链

## 工程结构

```text
ros2_ws_tugbot_20260419/
├── README.md
├── artifacts/
├── tests/
└── src/
    ├── tugbot_description/
    ├── tugbot_gazebo/
    ├── tugbot_perception/
    ├── tugbot_control/
    └── tugbot_bringup/
```

### 各包职责

- `tugbot_description`
  - 机器人模型、mesh 资产、模型安装布局
- `tugbot_gazebo`
  - Gazebo worlds 与 bridge 配置
- `tugbot_perception`
  - 图像 -> 车道误差
- `tugbot_control`
  - 误差 -> `/cmd_vel`
- `tugbot_bringup`
  - launch 编排与 RViz 调试入口

## World 与 Launch

### World

- `tugbot_lane_world.sdf`
  - formal baseline world
- `tugbot_lane_world_debug.sdf`
  - debug / perception inspection world
- `tugbot_lane_world_zeroerr.sdf`
  - textured robustness world，保留 `zeroerr.png` 与 formal 蓝色轨道
- `tugbot_lane_world_zeroerr_outer.sdf`
  - outer-loop-only world，保留 `zeroerr.png`，移除 competing `visual_track_*`
- `tugbot_lane_world_zeroerr_outer_probe_oldpose.sdf`
  - old-pose / old-mapping probe world

### Launch

- `sim_minimal.launch.py`
  - 最小仿真入口
- `full_system.launch.py`
  - 完整联调入口，默认使用 formal world
- `perception_debug.launch.py`
  - 感知调试入口，默认使用 debug world
- `full_system_zeroerr_outer.launch.py`
  - zeroerr_outer 专用完整联调入口

## 快速开始

### 1. 构建

```bash
cd ~/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_20260419
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

### 2. Source

```bash
cd ~/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_20260419
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 3. 启动

最小仿真：

```bash
ros2 launch tugbot_bringup sim_minimal.launch.py
```

formal world 完整系统：

```bash
ros2 launch tugbot_bringup full_system.launch.py
```

textured robustness world：

```bash
WORLD_PREFIX=$(ros2 pkg prefix tugbot_gazebo)/share/tugbot_gazebo/worlds
ros2 launch tugbot_bringup full_system.launch.py \
  world_sdf:=$WORLD_PREFIX/tugbot_lane_world_zeroerr.sdf
```

zeroerr_outer 专用入口：

```bash
ros2 launch tugbot_bringup full_system_zeroerr_outer.launch.py
```

感知调试：

```bash
ros2 launch tugbot_bringup perception_debug.launch.py
```

## 关键参数

### 感知层
文件：`src/tugbot_perception/config/perception.yaml`

- `blue_threshold: 80`
- `blue_margin: 20`
- `crop_top_ratio: 0.45`
- `detection_hold_frames: 3`
- `debug_image_enabled: true`

### 控制层
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

## 测试

运行：

```bash
python3 -m pytest tests/ -q
```

当前结果：
- `28 passed`

当前测试覆盖重点：
- workspace / launch / world / config 契约
- 蓝线检测与误差计算
- uniform gray 不再误判为蓝线
- `detection_hold_frames` 短时丢检保持
- PID 输出方向与限幅
- search recovery：spin / arc / arc 翻转 / reacquire 重置
- `temporal_debug.py` 的帧级摘要与转换窗口提取
- `full_system_zeroerr_outer.launch.py` 的存在与分层关系

## 当前验证结论

### 1. 结构层
- 五包分层、参数外置、launch 分层均已稳定
- formal world 未被 textured / robustness 场景覆盖
- zeroerr_outer 已有独立 wrapper launch

### 2. 搜索恢复层
- 代码、参数、测试均已覆盖 search recovery
- live / cold-start 证据表明搜索恢复已真实接入控制链路

### 3. zeroerr_outer 层
- zeroerr_outer 已不只是静态 world 文件
- 已有可复核的 cold-start 与 live 产物
- 可用于 textured outer-loop 验证与后续强制 arc 实验

## 验证产物

目录：`artifacts/`

重点文件：
- `cold_start_search_trace.py`
- `cold_start_search_trace.json`
- `cold_start_search_launch.log`
- `repeat_cold_start_search_trace.py`
- `repeat_cold_start_search_summary.json`

这些产物用于：
- 从冷启动瞬间记录 `/cmd_vel` 与 `/lane_tracking/error`
- 压缩 phase sequence（如 `spin` / `arc` / `pid_or_other`）
- 判断 search recovery 是否真正生效

## 已知边界

- 当前自然冷启动条件下，系统通常会在进入 arc 前先 reacquire
- 因此常见真实路径是：
  - `spin -> reacquire -> PID`
- 若要强行补齐 `spin -> arc -> reacquire` 的完整证据链，需要受控实验：
  - 临时减小 `search_spin_revolution_target`
  - 或调整 spawn / framing，让系统在约 `17.95 s` 之前不提前 reacquire

## 经验沉淀

阶段经验总结已单独沉淀到：
- `doc/docexperience/zeroerr_outer_search_recovery_cold_start.md`

其中包含：
- zeroerr_outer 场景设计意图
- search recovery 接入证据
- cold-start 证据链与时间解释
- 当前边界与下一步建议

## 路线图

下一步建议：
- 强制 arc 验证，补齐完整 `spin -> arc -> reacquire` 证据链
- 针对 zeroerr_outer 继续做更正式的动态纠偏验证
- 继续沉淀 world / threshold / spawn / recovery 经验到文档目录

## 许可

当前仓库内未单独声明 LICENSE；如需开源发布，建议后续补充。
