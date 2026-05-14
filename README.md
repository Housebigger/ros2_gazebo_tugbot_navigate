# ros2_gazebo_tugbot_navigate

Tugbot 在 ROS 2 Jazzy + Gazebo Harmonic 中的三阶段导航实验仓库：从视觉巡航最小闭环，到已知地图 Nav2 静态导航，再到未知环境自主探索建图与自建地图回放。

这个仓库不是只保存一个最终 demo，而是保留了三条互相衔接、可独立运行的工作空间。它适合用来学习和复现实验型移动机器人系统如何逐步搭建：先让仿真、桥接、感知和控制闭环稳定，再接入 Nav2 已知地图导航，最后扩展到 slam_toolbox 在线建图、frontier 自主探索、地图保存和静态地图导航回放。

## 当前结论

当前推荐入口是：

```text
ros2_ws_tugbot_nav_20260514/
```

它已经完成并记录了完整链路：

```text
Gazebo Tugbot 环境
  -> ros_gz_bridge 桥接 /scan /odom /tf /cmd_vel /clock
  -> slam_toolbox 在线建图并发布 live /map
  -> Nav2 基于 live /map 执行导航
  -> frontier_explorer 自动生成并发送 NavigateToPose 目标
  -> map_saver_cli 保存自建地图
  -> map_server + AMCL 加载自建地图
  -> Nav2 在自建地图上完成静态导航回放
```

推荐地图是 Phase 6 cleanup map：

```text
ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
```

最终验收摘要见：

```text
ros2_ws_tugbot_nav_20260514/doc/doc_report/phase9_final_acceptance_summary.md
```

## 三个工作空间

| 工作空间 | 定位 | 主要能力 | 推荐读者 |
| --- | --- | --- | --- |
| `ros2_ws_tugbot_20260419` | 视觉巡航最小闭环 | Gazebo + camera + 蓝线检测 + PID 控制 + 丢线搜索恢复 | 想理解最小仿真闭环、感知/控制分层的人 |
| `ros2_ws_tugbot_nav_20260513` | 已知静态地图 Nav2 基线 | Gazebo + LaserScan + AMCL + map_server + Nav2 + RViz 人工 goal | 想复现经典已知地图导航的人 |
| `ros2_ws_tugbot_nav_20260514` | 自主探索建图与自建地图回放 | 在线 SLAM、SLAM+Nav2、frontier 自主探索、地图保存、Phase 6 地图静态回放 | 想研究未知环境探索建图完整链路的人 |

### 1. `ros2_ws_tugbot_20260419`：视觉巡航最小闭环

该工作空间围绕“最小可运行闭环”组织，包含五个 ROS 2 包：

```text
tugbot_description  机器人模型与 mesh 资源
tugbot_gazebo       Gazebo world 与 ros_gz bridge 配置
tugbot_perception   图像到蓝线中心误差
tugbot_control      误差到 /cmd_vel 的 PID 与搜索恢复控制
tugbot_bringup      分层 launch 入口
```

核心链路：

```text
/camera/image_raw
  -> lane_detector_node
  -> /lane_tracking/error
  -> lane_controller_node
  -> /cmd_vel
  -> Gazebo DiffDrive
  -> /odom
```

已保留的代表入口：

```bash
cd ros2_ws_tugbot_20260419
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# 最小仿真底座
ros2 launch tugbot_bringup sim_minimal.launch.py

# formal world 完整视觉巡航闭环
ros2 launch tugbot_bringup full_system.launch.py

# 感知调试
ros2 launch tugbot_bringup perception_debug.launch.py

# zeroerr_outer 纹理外环场景闭环
ros2 launch tugbot_bringup full_system_zeroerr_outer.launch.py
```

该工作空间还包含 `tests/` 与 `artifacts/`，用于保存结构契约、感知逻辑、控制逻辑、搜索恢复和模型视觉细节恢复的验证证据。

### 2. `ros2_ws_tugbot_nav_20260513`：已知静态地图 Nav2 基线

该工作空间把 Tugbot 导航链路推进到经典 Nav2 已知地图导航。包含四个 ROS 2 包：

```text
tugbot_description  Tugbot 模型资源
tugbot_gazebo       Gazebo world、bridge 与仿真启动
tugbot_navigation   Nav2 参数、静态地图与导航资源
tugbot_bringup      Gazebo + RViz + Nav2 一键启动入口
```

核心链路：

```text
静态地图 /map
  + Gazebo LaserScan /scan
  + /odom 与 TF
  -> AMCL 定位
  -> Nav2 planner/controller/bt_navigator
  -> /cmd_vel
  -> Gazebo Tugbot 运动
```

快速运行：

```bash
cd ros2_ws_tugbot_nav_20260513
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Gazebo 单独启动
ros2 launch tugbot_gazebo tugbot_gazebo.launch.py

# Gazebo + RViz + AMCL + Nav2 一键启动
ros2 launch tugbot_bringup tugbot_nav.launch.py
```

RViz 使用方式：

1. 用 `2D Pose Estimate` 给 AMCL 设置初始位姿。
2. 确认 LaserScan 与地图墙体轮廓基本贴合。
3. 用 `2D Goal Pose` 发送 Nav2 目标点。
4. 观察 RViz 路径规划与 Gazebo 中 Tugbot 的运动。

该工作空间的 README 已记录 2026-05-13 live 验收：构建、测试、Gazebo、关键话题、AMCL/Nav2 lifecycle、`/navigate_to_pose` goal 和人工 RViz 验收均已通过。

### 3. `ros2_ws_tugbot_nav_20260514`：自主探索建图与导航回放

该工作空间是在不破坏 0513 已知地图基线的前提下独立演进出的最终推荐工作空间。包含五个 ROS 2 包：

```text
tugbot_description  Tugbot 模型资源
tugbot_gazebo       Gazebo world、bridge 与仿真启动
tugbot_navigation   Nav2、SLAM、地图与导航参数
tugbot_bringup      静态导航、SLAM、SLAM+Nav2、探索、Phase 6 回放入口
tugbot_exploration  frontier_explorer 自主探索节点
```

主要 launch 入口：

| Launch | 用途 | 地图来源 | AMCL | slam_toolbox |
| --- | --- | --- | --- | --- |
| `tugbot_nav.launch.py` | 继承 0513 的已知地图静态 Nav2 基线 | 原始静态地图 | 是 | 否 |
| `tugbot_slam.launch.py` | SLAM-only 在线建图 | live `/map` | 否 | 是 |
| `tugbot_slam_nav.launch.py` | SLAM + Nav2 手动导航 | live `/map` | 否 | 是 |
| `tugbot_explore.launch.py` | frontier 自主探索建图，可选保存地图 | live `/map` | 否 | 是 |
| `tugbot_nav_phase6_map.launch.py` | Phase 6 自建地图静态导航回放 | Phase 6 cleanup map | 是 | 否 |

快速运行：

```bash
cd ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

启动在线 SLAM：

```bash
ros2 launch tugbot_bringup tugbot_slam.launch.py
```

启动 SLAM + Nav2 手动导航：

```bash
ros2 launch tugbot_bringup tugbot_slam_nav.launch.py
```

启动 frontier 自主探索并保存地图：

```bash
ros2 launch tugbot_bringup tugbot_explore.launch.py \
  headless:=true \
  use_rviz:=false \
  max_goals:=20 \
  enable_cleanup_mode:=true \
  save_map:=true \
  map_save_path:=$(pwd)/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup
```

使用推荐 Phase 6 自建地图做静态导航回放：

```bash
ros2 launch tugbot_bringup tugbot_nav_phase6_map.launch.py \
  headless:=true \
  use_rviz:=false
```

如果需要人工可视化验收，把 `use_rviz:=true`，在 RViz 中先用 `2D Pose Estimate` 设置 AMCL 初始位姿，再用 `2D Goal Pose` 发送目标。

## 环境要求

已验证环境：

```text
Ubuntu 24.04
ROS 2 Jazzy
Gazebo Harmonic
Python 3
```

推荐安装依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz \
  ros-jazzy-tf-transformations \
  python3-colcon-common-extensions \
  python3-pytest
```

如果你的系统已经通过 ROS desktop/full 或源码方式安装了相关组件，可按实际环境调整。

## 仓库结构

```text
ros2_gazebo_tugbot_navigate/
├── README.md
├── .gitignore
├── ros2_ws_tugbot_20260419/
│   ├── README.md
│   ├── LICENSE
│   ├── CHANGELOG.md
│   ├── CONTRIBUTING.md
│   ├── CODE_OF_CONDUCT.md
│   ├── SECURITY.md
│   ├── artifacts/
│   ├── tests/
│   └── src/
│       ├── tugbot_description/
│       ├── tugbot_gazebo/
│       ├── tugbot_perception/
│       ├── tugbot_control/
│       └── tugbot_bringup/
├── ros2_ws_tugbot_nav_20260513/
│   ├── README.md
│   └── src/
│       ├── tugbot_description/
│       ├── tugbot_gazebo/
│       ├── tugbot_navigation/
│       └── tugbot_bringup/
└── ros2_ws_tugbot_nav_20260514/
    ├── README.md
    ├── doc/
    │   ├── doc_report/
    │   └── doc_experience/
    └── src/
        ├── tugbot_description/
        ├── tugbot_gazebo/
        ├── tugbot_navigation/
        ├── tugbot_bringup/
        └── tugbot_exploration/
```

说明：各工作空间下的 `build/`、`install/`、`log/` 是 colcon 构建产物，不是理解源码的首要入口。新 clone 后建议在对应工作空间内重新构建。

## 推荐阅读路线

如果你是第一次阅读这个仓库，建议按下面顺序：

1. 先读本 README，理解三个工作空间之间的递进关系。
2. 读 `ros2_ws_tugbot_20260419/README.md`，理解最小仿真闭环、感知、控制与 Gazebo bridge。
3. 读 `ros2_ws_tugbot_nav_20260513/README.md`，理解已知地图 AMCL + Nav2 的基线。
4. 读 `ros2_ws_tugbot_nav_20260514/README.md`，进入自主探索建图和 Phase 6 地图回放。
5. 读 `ros2_ws_tugbot_nav_20260514/doc/doc_report/phase9_final_acceptance_summary.md`，查看最终验收结论。
6. 如果要继续改探索策略，读 `ros2_ws_tugbot_nav_20260514/doc/doc_experience/tugbot_autonomous_mapping_success_experience.md`。

## 常用验证命令

构建某个工作空间：

```bash
cd ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

查看 ROS graph：

```bash
ros2 node list
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger '{}'
```

发送一个 Nav2 goal：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

运行 20260419 视觉巡航工作空间的 Python 测试：

```bash
cd ros2_ws_tugbot_20260419
python3 -m pytest tests -q
```

运行 0513/0514 包内 contract 测试示例：

```bash
cd ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## 已知边界

- 这是实验型 ROS 2/Gazebo 仓库，优先展示可复现实验链路与阶段验证，而不是已经产品化的机器人系统。
- Gazebo GUI 在 VMware、Wayland/X11、OpenGL/EGL 组合环境下可能出现图形 warning；headless 模式通常更适合自动化验证。
- `ros2_ws_tugbot_nav_20260514` 中 Phase 6 cleanup map 是当前推荐成果；Phase 7A 地图保留为负面实验，不建议作为默认地图。
- 自主探索仍依赖 SLAM 几何一致性和 frontier 候选点安全性；更复杂环境下建议继续做地图质量评分、planner precheck 和多 world 泛化测试。
- 当前仓库中部分工作空间保留了历史构建产物目录；正式分发或二次开发时可清理 `build/`、`install/`、`log/` 后重新构建。

## License / Acknowledgements

当前仓库内 `ros2_ws_tugbot_20260419/LICENSE` 为 MIT License。若将三个工作空间统一对外发布，建议在仓库根目录补充统一 LICENSE 文件，并同步检查各 ROS package 的 maintainer 与 license metadata。

感谢以下开源生态：

- ROS 2
- Gazebo / Gazebo Harmonic
- ros_gz packages
- Nav2 / Navigation2
- slam_toolbox

特别致谢：

- UM FST 徐老师
- bilibili 后来老师
