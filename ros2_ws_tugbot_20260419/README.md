# ros2_ws_tugbot_20260419

面向 ROS 2 Jazzy + Gazebo Harmonic 的 tugbot 视觉巡航规范重构工程。

本工程不是对旧工程做补丁式续修，而是从零重建的最小闭环母体，目标是：
- 结构纯净
- 命名统一
- 模块边界清晰
- 参数外置
- launch 分层
- 感知与控制解耦
- 易于继续扩展到更强感知、更复杂控制、更多传感器与真机替换

## 工程目标

建立一个最小但规范的视觉巡航联调工程，核心主链为：

`/camera/image_raw -> lane_detector_node -> /lane_tracking/error -> lane_controller_node -> /cmd_vel`

其中：
- Gazebo 负责世界、机器人与相机数据生成
- ROS 2 感知节点负责从图像中提取车道误差
- ROS 2 控制节点负责根据误差生成 `/cmd_vel`
- Gazebo DiffDrive 插件负责执行速度命令并产出 `/odom`

## 工程结构

```text
ros2_ws_tugbot_20260419/
├── README.md
├── tests/
│   ├── test_workspace_layout.py
│   ├── test_architecture_contracts.py
│   └── test_lane_logic.py
└── src/
    ├── tugbot_description/
    │   ├── models/tugbot/
    │   │   ├── model.config
    │   │   ├── model.sdf
    │   │   └── meshes/
    │   └── tugbot_description/package_layout.py
    ├── tugbot_gazebo/
    │   ├── worlds/tugbot_lane_world.sdf
    │   └── config/bridge.yaml
    ├── tugbot_perception/
    │   ├── config/perception.yaml
    │   └── tugbot_perception/lane_detector_node.py
    ├── tugbot_control/
    │   ├── config/control.yaml
    │   └── tugbot_control/lane_controller_node.py
    └── tugbot_bringup/
        ├── launch/sim_minimal.launch.py
        ├── launch/full_system.launch.py
        ├── launch/perception_debug.launch.py
        └── config/perception_debug.rviz
```

## 各包职责

### 1. tugbot_description
职责：只负责机器人描述与资产打包。

内容：
- 最小 tugbot `model.sdf`
- `model.config`
- 选择性迁移的 mesh 资源
- `package_layout.py` 用于递归安装模型目录

设计原则：
- 保留最小底盘与前视相机
- 保留 DiffDrive 插件
- 保留 Sensors 插件
- 不保留旧工程里的 back camera / lidar / imu / gripper 等冗余结构

### 2. tugbot_gazebo
职责：只负责 Gazebo world 与桥接配置。

内容：
- `tugbot_lane_world.sdf`
- `bridge.yaml`

设计原则：
- world 中只保留 ground plane + blue lane segments + `model://tugbot`
- `/cmd_vel` 与 `/odom` 用 `ros_gz_bridge`
- 图像用 `ros_gz_image`
- 不把业务逻辑塞进 Gazebo 层

### 3. tugbot_perception
职责：只做视觉感知。

输入：
- `/camera/image_raw`

输出：
- `/lane_tracking/error`
- `/lane_tracking/debug_image`

设计原则：
- 从图像中提取蓝色轨迹中心
- 将中心位置变为归一化横向误差
- 不直接发布 `/cmd_vel`
- 感知参数全部外置

### 4. tugbot_control
职责：只做控制决策。

输入：
- `/lane_tracking/error`

输出：
- `/cmd_vel`

设计原则：
- PID 逻辑集中在控制层
- 失去误差输入时立即停车
- 控制参数全部外置

### 5. tugbot_bringup
职责：只做系统编排。

包含三套 launch：
- `sim_minimal.launch.py`：最小仿真启动
- `full_system.launch.py`：完整联调启动
- `perception_debug.launch.py`：感知调试启动

设计原则：
- bringup 只负责 include 与节点装配
- 不在 launch 中写业务逻辑
- launch 分层清晰，可独立验证

## 参数化设计

### 感知参数：`src/tugbot_perception/config/perception.yaml`
当前已外置：
- `input_topic`
- `error_topic`
- `debug_image_topic`
- `blue_threshold`
- `blue_margin`
- `crop_top_ratio`
- `debug_image_enabled`

### 控制参数：`src/tugbot_control/config/control.yaml`
当前已外置：
- `error_topic`
- `cmd_vel_topic`
- `control_period`
- `error_timeout_sec`
- `kp`
- `ki`
- `kd`
- `integral_limit`
- `base_linear_speed`
- `min_linear_speed`
- `max_angular_speed`
- `speed_reduction_gain`

## 构建方法

```bash
cd ~/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_20260419
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

最近验证结果：
- `5 packages finished`

## source 方法

```bash
cd ~/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_20260419
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 启动方法

### 1. 最小仿真启动
只启动 Gazebo、桥接、图像桥。

```bash
ros2 launch tugbot_bringup sim_minimal.launch.py
```

### 2. 完整联调启动
启动 Gazebo + bridge + perception + control。

```bash
ros2 launch tugbot_bringup full_system.launch.py
```

### 3. 感知调试启动
启动 Gazebo + bridge + perception + RViz。

```bash
ros2 launch tugbot_bringup perception_debug.launch.py
```

## 关键节点

完整联调时的关键节点：
- `ros_gz_bridge`
- `ros_gz_image`
- `lane_detector_node`
- `lane_controller_node`

感知调试时会额外启动：
- `rviz2`

## 关键话题

- `/camera/image_raw`
- `/camera/camera_info`
- `/lane_tracking/error`
- `/lane_tracking/debug_image`
- `/cmd_vel`
- `/odom`

## 最小闭环验证结果

本工程已完成以下 live 验证：

1. 构建通过
- `colcon build --symlink-install` 成功

2. 测试通过
- `python3 -m pytest -q tests`
- 结果：`14 passed`

3. 最小仿真链路曾验证通过
- `sim_minimal.launch.py` 启动后，ROS 侧曾观测到：
  - `/camera/image_raw`
  - `/camera/camera_info`
  - `/cmd_vel`
  - `/odom`
- 说明最小 Gazebo + bridge + image bridge 主链曾成功拉起

4. 完整联调链路曾验证通过
- `full_system.launch.py` 启动后，ROS 侧曾观测到：
  - `/lane_tracking/error`
  - `/lane_tracking/debug_image`
  - `/cmd_vel`
  - `/odom`
- 说明 perception + control + bridge 装配关系成立

5. 感知与控制已解耦
- `/lane_tracking/error` 的发布者为 `lane_detector_node`
- `/cmd_vel` 的发布者为 `lane_controller_node`
- 旧工程那种“单节点图像直出 /cmd_vel”的结构已拆开

6. /cmd_vel 纯净发布已确认
- 清理旧工程残留后，`/cmd_vel` 只有一个预期发布者：`lane_controller_node`

7. /odom 回传已确认
- `/odom` 可实时 `echo`
- 已看到有效 `pose` 与 `twist` 数据
- 说明 Gazebo DiffDrive 执行链路成立

8. perception 误判缺陷已修复
- 已修复 numpy/uint8 路径中的 blue margin 整数回绕问题
- 已新增回归测试，防止均匀灰图被误判为蓝线

9. 当前主阻塞问题：视觉输入层异常
- 修复误判后，当前 live 验证中 `lane_detector_node` 输出为：
  - `lane_center=None`
  - `error=missing`
- 当前 `/camera/image_raw` 统计结果表现为几乎整张图统一灰色
- 因此系统目前处于“看不到线 -> 安全停车”的正确保护态，而不是稳定视觉巡航态

结论：
- 当前系统已经满足“结构规范 + 主链解耦 + 可构建 + 可启动 + 可联调基础成立”目标
- perception 的关键误判 bug 已修复
- 修复后的真实视觉闭环与动态纠偏验证，仍待视觉输入层恢复后完成

## 与旧工程相比的关键重构点

已完成的核心重构：
- 从单包 demo 结构重构为 5 包分层结构
- 从综合 world 重构为最小 lane world
- 从综合多传感器模型重构为最小前视相机模型
- 从单节点图像直控重构为 perception + control 两层
- 从硬编码参数重构为 perception / control 参数文件
- 从大耦合 launch 重构为三层 launch 体系
- 从整包继承改为选择性迁移 mesh 资产

## 后续扩展方向

下一步可继续沿以下方向扩展：

1. 感知增强
- HSV 参数化
- ROI 多段策略
- 调试 mask / overlay 图像
- 更鲁棒的轨迹中心估计

2. 控制增强
- 速度调度分层
- 更稳定的 anti-windup
- 停车 / 丢线恢复策略
- 曲率相关速度限制

3. 传感器扩展
- 增加 IMU
- 增加 lidar
- 增加 depth camera
- 逐步桥接更多 ROS 标准接口

4. 系统级扩展
- rosbag 录制脚本
- 参数 profile
- 自动化 launch 验证
- CI 测试

5. 上层能力扩展
- 视觉巡航 + 避障融合
- 里程计融合
- 路径跟踪
- Nav2 接入
- 真机底盘替换
