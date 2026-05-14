# Tugbot ROS 2 + Gazebo 自主探索建图与导航

一个基于 ROS 2 Jazzy、Gazebo Harmonic、Nav2、slam_toolbox 与 ros_gz_bridge 的 Tugbot 仿真工程，用于验证从未知环境自主探索建图到自建地图静态导航回放的完整链路。

本仓库对应工作空间：`ros2_ws_tugbot_nav_20260514`。

已验证的工程闭环：

```text
未知 Gazebo 环境
  -> slam_toolbox 在线建图
  -> frontier_explorer 自主探索
  -> Nav2 导航执行探索目标
  -> map_saver_cli 保存自建地图
  -> map_server + AMCL 加载自建地图
  -> Nav2 静态地图导航回放
```

明确结论：

```text
ros2_ws_tugbot_nav_20260514 已完成“未知地图自主探索建图 -> 地图保存 -> 自建地图静态导航回放”的工程闭环。
```

---

## 功能特性

- Gazebo Tugbot 仿真环境。
- `ros_gz_bridge` 桥接 `/scan`、`/odom`、`/tf`、`/cmd_vel`、`/clock` 等关键通道。
- SLAM-only 在线建图：`slam_toolbox` 发布实时 `/map`。
- SLAM + Nav2 手动目标导航：Nav2 使用实时 `/map`，不依赖静态地图。
- frontier 自主探索：`frontier_explorer` 自动识别 frontier 并发送 `NavigateToPose` goal。
- residual unknown cleanup 补扫：对残留 unknown 区域进行轻量补扫。
- `map_saver_cli` 保存 `.yaml + .pgm` 自建地图。
- Phase 6 自建地图静态导航回放：`map_server + AMCL + Nav2` 已验证通过。
- 保留 0513 原始静态地图导航入口，方便对比已知地图 Nav2 基线。
- Phase 7A 作为负面实验记录保留，不作为推荐地图。

---

## 环境要求

已验证环境：

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Nav2 / Navigation2
- slam_toolbox
- ros_gz_bridge / ros_gz_sim
- Python 3

推荐依赖安装示例：

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-navigation2 \
  ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz \
  ros-jazzy-tf-transformations
```

说明：不同机器的 ROS/Gazebo 安装方式可能略有差异。如果你的系统已经通过 desktop/full 或源码方式安装了相关包，可以按实际环境调整依赖安装命令。

---

## 构建方法

### 当前本机路径

如果你在当前开发机上使用这个工作空间：

```bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 用户 clone 后的通用路径

如果你从 GitHub clone 本仓库：

```bash
cd ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

每次打开新终端后，至少需要重新 source：

```bash
cd ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

---

## 启动入口总览

| Launch | 用途 | 是否使用静态地图 | 是否启动 AMCL | 是否使用 slam_toolbox |
| --- | --- | --- | --- | --- |
| `tugbot_nav.launch.py` | 继承 0513 的已知地图静态 Nav2 基线 | 是，使用原始静态地图 | 是 | 否 |
| `tugbot_slam.launch.py` | SLAM-only 在线建图 | 否 | 否 | 是 |
| `tugbot_slam_nav.launch.py` | SLAM + Nav2 手动导航 | 否，Nav2 使用实时 `/map` | 否 | 是 |
| `tugbot_explore.launch.py` | frontier 自主探索建图，可选保存地图 | 否，探索阶段使用实时 `/map` | 否 | 是 |
| `tugbot_nav_phase6_map.launch.py` | Phase 6 自建地图静态导航回放 | 是，使用 Phase 6 cleanup map | 是 | 否 |

### 1. 原 0513 静态地图导航基线

```bash
ros2 launch tugbot_bringup tugbot_nav.launch.py
```

用途：保留已知静态地图的 Nav2 基线，启动 `map_server + AMCL + Nav2`。

### 2. SLAM 在线建图

```bash
ros2 launch tugbot_bringup tugbot_slam.launch.py
```

用途：启动 Gazebo、bridge、scan static TF 与 `slam_toolbox`，在线生成 `/map`。不启动 AMCL，不启动静态 `map_server`。

### 3. SLAM + Nav2 手动导航

```bash
ros2 launch tugbot_bringup tugbot_slam_nav.launch.py
```

用途：在 `slam_toolbox` 实时地图上启动 Nav2，可手动发送 `/navigate_to_pose` goal。

### 4. frontier 自主探索建图

```bash
ros2 launch tugbot_bringup tugbot_explore.launch.py
```

用途：启动在线 SLAM + Nav2 + `frontier_explorer`，让小车自动探索未知区域。可通过 `save_map:=true` 保存探索地图。

### 5. Phase 6 自建地图静态导航回放

```bash
ros2 launch tugbot_bringup tugbot_nav_phase6_map.launch.py
```

用途：加载 Phase 6 cleanup map，启动 `map_server + AMCL + Nav2`，验证自建地图能否支持静态地图导航回放。

---

## 快速开始

下面给出一个推荐的最小使用流程。

### A. 构建工作空间

```bash
cd ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### B. 启动自主探索并保存地图

```bash
ros2 launch tugbot_bringup tugbot_explore.launch.py \
  headless:=true \
  use_rviz:=false \
  max_goals:=20 \
  enable_cleanup_mode:=true \
  save_map:=true \
  map_save_path:=/absolute/path/to/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup
```

请把 `/absolute/path/to/ros2_ws_tugbot_nav_20260514` 替换成你自己的工作空间绝对路径。

说明：仓库中已经包含通过 Phase 6 验证的推荐地图。上面的命令适合你想重新跑探索并覆盖保存地图时使用；如果只是体验静态地图导航回放，可以直接使用下一节的 Phase 6 map 启动入口。

### C. 查看保存地图

```bash
ls -lh src/tugbot_navigation/maps/explored/
cat src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
file src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
```

### D. 使用 Phase 6 地图做静态导航回放

```bash
ros2 launch tugbot_bringup tugbot_nav_phase6_map.launch.py \
  headless:=true \
  use_rviz:=false
```

如果使用 RViz，可将 `use_rviz:=true`，并通过 `2D Pose Estimate` 设置 AMCL 初始位姿，再发送 `2D Goal Pose`。

---

## 推荐地图

当前推荐地图是 Phase 6 cleanup map：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
```

推荐理由：

- Phase 6 在 Phase 5 改良探索完成条件后继续加入 residual unknown cleanup。
- Phase 6 地图相对 Phase 4 覆盖明显扩大。
- Phase 6 地图几何质量稳定，适合后续静态导航回放。
- Phase 8 已验证该地图可被 `map_server` 加载，AMCL 可定位，Nav2 可完成两个 `NavigateToPose` goal。

不推荐默认使用 Phase 7A 地图：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.pgm
```

原因：Phase 7A 地图覆盖更激进，但人工观察到重影、墙线重复、局部错层和扫描未对齐。该阶段证明过度补扫可能降低 SLAM 一致性，因此 Phase 7A 仅作为负面实验记录，不作为默认成果。

---

## 项目结构

```text
ros2_ws_tugbot_nav_20260514/
├── README.md
├── src/
│   ├── tugbot_description/
│   ├── tugbot_gazebo/
│   ├── tugbot_navigation/
│   ├── tugbot_bringup/
│   └── tugbot_exploration/
├── doc/
│   └── doc_report/
├── scripts/
├── build/
├── install/
└── log/
```

主要目录说明：

- `src/tugbot_description/`：Tugbot 机器人描述、URDF/Xacro、模型资源。
- `src/tugbot_gazebo/`：Gazebo world、仿真启动、ros_gz_bridge 相关配置。
- `src/tugbot_navigation/`：Nav2、SLAM、地图与导航参数。
- `src/tugbot_bringup/`：统一 launch 入口，包括静态导航、SLAM、SLAM+Nav2、探索与 Phase 6 地图回放。
- `src/tugbot_exploration/`：frontier 自主探索节点与候选目标生成逻辑。
- `doc/doc_report/`：Phase 0~9 验收报告与实验记录。
- `scripts/`：可选辅助脚本目录；当前核心运行入口主要在 `src/tugbot_bringup/launch/`。
- `build/`、`install/`、`log/`：colcon 构建产物，通常不作为源码重点阅读对象。

---

## 验收状态

更详细的阶段报告见：

```text
doc/doc_report/
```

| 阶段 | 内容 | 结论 |
| --- | --- | --- |
| Phase 0 | 0514 基线创建 | 通过 |
| Phase 1 | SLAM 在线建图 | 通过 |
| Phase 2 | SLAM + Nav2 手动导航 | 通过 |
| Phase 3 | frontier 自主探索闭环 | 通过 |
| Phase 4 | 地图保存 | 通过，但发现提前停止问题 |
| Phase 5 | frontier 完成条件改良 | 通过 |
| Phase 6 | coverage cleanup | 有条件通过，推荐地图产出 |
| Phase 7A | 参数强化复跑 | 执行通过，但地图质量不采纳 |
| Phase 8 | Phase 6 自建地图静态导航回放 | 通过 |
| Phase 9 | 0514 工程最终验收总结 | 通过 |

关键验收证据摘要：

- 探索阶段 `/map` 由 `slam_toolbox` 发布。
- 探索阶段不启动 AMCL / 静态 `map_server`。
- `frontier_explorer` 能自动发送 Nav2 目标。
- Phase 6 地图已成功保存为 `.yaml + .pgm`。
- Phase 8 中 `map_server` 成功加载 Phase 6 cleanup map。
- AMCL 能在 Phase 6 map 上定位。
- Nav2 已完成两个 `NavigateToPose` goal，且 `/cmd_vel` 与 `/odom` 证明小车真实移动。

---

## 手动导航示例

在 `tugbot_slam_nav.launch.py` 或 `tugbot_nav_phase6_map.launch.py` 启动并就绪后，可以发送 Nav2 goal：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
--feedback
```

如果使用静态地图回放入口并启用 RViz，建议先用 `2D Pose Estimate` 设置 AMCL 初始位姿。

---

## 已知限制

- Phase 6 地图仍有少量 residual unknown。
- Phase 7A 证明过度补扫可能降低地图质量，表现为重影、墙线重复、局部错层和扫描未对齐。
- 当前未实现 `ComputePathToPose` 批量预检查；目前主要使用轻量级候选点安全检查与 Nav2 执行反馈。
- 当前未实现自动地图相似度评估。
- RViz 图形验收仍建议人工确认，尤其是墙线、障碍物边缘和局部错层。
- 保存地图后 launch 自动退出策略仍可优化。

---

## 后续计划

以下只是可选 TODO，本 README 不代表已经执行：

- planner precheck：为 frontier/cleanup 候选目标增加批量 `ComputePathToPose` 可达性预检查。
- 地图质量自动评分：统计重影、墙线一致性、unknown ratio、occupied 边界稳定性等指标。
- 回环检测参数调优：进一步降低长时间探索中的 SLAM 累积误差。
- 更精细 coverage planner：减少无效补扫，优先补全关键 residual unknown。
- 保存地图后自动退出 launch：让探索保存完成后更适合无人值守批处理。
- 多环境泛化测试：在更多 Gazebo world 中验证自主探索与静态回放链路。

---

## License / Citation / Acknowledgements

License: MIT

本项目使用并感谢以下开源生态：

- ROS 2
- Nav2 / Navigation2
- slam_toolbox
- Gazebo / Gazebo Harmonic
- ros_gz packages

特别致谢恩师 ：

- UM FST 徐老师
- bilibili 后来老师
