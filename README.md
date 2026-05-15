# ros2_gazebo_tugbot_navigate

Tugbot 在 ROS 2 Jazzy + Gazebo Harmonic 中的阶段化导航实验仓库：从视觉巡航最小闭环，到已知静态地图 Nav2 导航，再到未知环境自主探索建图、地图归档与自建地图静态回放。

这个仓库不是单一最终 demo，而是保留了三条互相衔接、可独立运行的工作空间。它适合用来学习和复现实验型移动机器人系统如何逐步搭建：先让仿真、桥接、感知和控制闭环稳定，再接入 Nav2 已知地图导航，最后扩展到 slam_toolbox 在线建图、frontier/perimeter 自主探索、地图保存、推荐地图归档和静态地图导航回放。

说明：当前仓库中的最新自主探索工作空间目录名是 `ros2_ws_tugbot_nav_20260514/`。

## 当前结论

当前推荐入口是：

```text
ros2_ws_tugbot_nav_20260514/
```

该工作空间已经完成并记录了完整链路：

```text
Gazebo Tugbot 环境
  -> ros_gz_bridge 桥接 /scan /odom /tf /cmd_vel /clock /camera
  -> slam_toolbox 在线建图并发布 live /map
  -> Nav2 基于 live /map 执行导航
  -> frontier_explorer 自动生成并发送 NavigateToPose 目标
  -> perimeter_then_frontier 外轮廓优先探索
  -> no-spin / 低角速度稳定建图
  -> map_saver_cli 保存自建地图
  -> Phase 14 推荐地图人工验收与归档
  -> map_server + AMCL 加载自建地图
  -> Nav2 在 Phase 14 推荐地图上完成静态回放 bringup 验证
```

当前推荐成果地图已经从旧的 Phase 6 cleanup map 更新为 Phase 14 perimeter no-spin map：

```text
ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml
ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.pgm
```

当前推荐静态回放入口是 Phase 16 新增的：

```bash
cd ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch tugbot_bringup tugbot_nav_phase14_map.launch.py \
  headless:=false \
  use_rviz:=true
```

最终推荐阅读的最新验收报告见：

```text
ros2_ws_tugbot_nav_20260514/doc/doc_report/phase16_phase14_static_replay_entry_report.md
```

关键历史报告：

```text
ros2_ws_tugbot_nav_20260514/doc/doc_report/phase14_no_spin_mapping_stability_report.md
ros2_ws_tugbot_nav_20260514/doc/doc_report/phase15_phase14_map_manual_acceptance_archive_report.md
ros2_ws_tugbot_nav_20260514/doc/doc_report/phase16_phase14_static_replay_entry_report.md
```

## 三个工作空间

| 工作空间 | 定位 | 主要能力 | 推荐读者 |
| --- | --- | --- | --- |
| `ros2_ws_tugbot_20260419` | 视觉巡航最小闭环 | Gazebo + camera + 蓝线检测 + PID 控制 + 丢线搜索恢复 | 想理解最小仿真闭环、感知/控制分层的人 |
| `ros2_ws_tugbot_nav_20260513` | 已知静态地图 Nav2 基线 | Gazebo + LaserScan + AMCL + map_server + Nav2 + RViz 人工 goal | 想复现经典已知地图导航的人 |
| `ros2_ws_tugbot_nav_20260514` | 自主探索建图、推荐地图归档与静态回放 | 在线 SLAM、SLAM+Nav2、frontier/perimeter 探索、no-spin 稳定建图、Phase 14 推荐地图、Phase 14 静态回放入口 | 想研究未知环境探索建图完整链路的人 |

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

代表入口：

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

### 3. `ros2_ws_tugbot_nav_20260514`：自主探索建图、推荐地图归档与静态回放

该工作空间是在不破坏 0513 已知地图基线的前提下独立演进出的当前推荐工作空间。包含五个 ROS 2 包：

```text
tugbot_description  Tugbot 模型资源
tugbot_gazebo       Gazebo world、bridge、camera 与仿真启动
tugbot_navigation   Nav2、SLAM、地图、参数与推荐地图资源
tugbot_bringup      静态导航、SLAM、SLAM+Nav2、探索、Phase 6/Phase 14 回放入口
tugbot_exploration  frontier_explorer 自主探索节点
```

主要能力：

- SLAM-only 在线建图：`slam_toolbox` 发布实时 `/map`。
- SLAM + Nav2 手动导航：Nav2 使用实时 `/map`，不依赖静态地图。
- frontier 自主探索建图：`frontier_explorer` 自动识别 frontier 并发送 `NavigateToPose` goal。
- `perimeter_then_frontier` 外轮廓优先探索策略：先尝试沿大跨度墙体生成 perimeter waypoint，再 fallback 到 frontier。
- no-spin / 低角速度探索模式：默认关闭大角度原地旋转，降低 slam_toolbox scan matching 错位与地图重影风险。
- residual unknown cleanup：对残留 unknown 区域进行轻量补扫。
- static box pillar 障碍物：原 cone / snow cone 障碍物已替换为两个 static box pillar。
- camera 可视化：Tugbot 前向 camera 可在 RViz Image Display 中查看 `/camera/image_raw`。
- Phase 14 perimeter no-spin map：当前推荐成果地图。
- Phase 16 静态回放入口：`tugbot_nav_phase14_map.launch.py` 默认加载 Phase 14 推荐地图，启动 `map_server + AMCL + Nav2`，不启动 `slam_toolbox` 或 `frontier_explorer`。

主要 launch 入口：

| Launch | 推荐程度 | 用途 | 地图来源 | AMCL | slam_toolbox | 备注 |
| --- | --- | --- | --- | --- | --- | --- |
| `tugbot_nav.launch.py` | 保留 / 基线 | 原 0513 静态地图导航基线 | 原始静态地图 `map_1725111373.yaml` | 是 | 否 | 用于回归验证原始已知地图导航链路 |
| `tugbot_slam.launch.py` | 调试入口 | SLAM-only 在线建图 | live `/map` | 否 | 是 | 不启动 Nav2 navigation，不启动 AMCL |
| `tugbot_slam_nav.launch.py` | 调试入口 | SLAM + Nav2 手动目标导航 | live `/map` | 否 | 是 | 适合手动发送 NavigateToPose goal 验证在线地图导航 |
| `tugbot_explore.launch.py` | 当前推荐探索入口 | 未知地图自主探索建图、perimeter/frontier、cleanup、可选保存地图 | live `/map` | 否 | 是 | 推荐显式使用 `exploration_strategy:=perimeter_then_frontier`；默认 no-spin / 低角速度；不要覆盖 Phase 14 推荐地图 |
| `tugbot_nav_phase14_map.launch.py` | 当前推荐静态回放入口 | Phase 14 推荐地图静态导航回放 | Phase 14 perimeter no-spin map | 是 | 否 | 新增于 Phase 16；加载当前推荐地图；不重新建图 |
| `tugbot_nav_phase6_map.launch.py` | 历史稳定回放入口 | Phase 6 cleanup map 静态导航回放 | Phase 6 cleanup map | 是 | 否 | 上一版稳定地图回放；不加载 Phase 14 推荐地图 |

快速构建：

```bash
cd ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

人工可视化自主探索，推荐使用 Phase 14 策略并保存为普通实验文件名：

```bash
ros2 launch tugbot_bringup tugbot_explore.launch.py \
  headless:=false \
  use_rviz:=true \
  exploration_strategy:=perimeter_then_frontier \
  max_goals:=20 \
  enable_cleanup_mode:=true \
  save_map:=true \
  map_save_path:=/absolute/path/to/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_manual
```

注意：请把 `/absolute/path/to/ros2_ws_tugbot_nav_20260514` 替换成自己的工作空间绝对路径。普通探索结果不要保存到 `tugbot_nav_world_slam_phase14_perimeter_no_spin` 路径，避免覆盖当前推荐成果地图。

使用当前推荐 Phase 14 地图做静态导航回放：

```bash
ros2 launch tugbot_bringup tugbot_nav_phase14_map.launch.py \
  headless:=false \
  use_rviz:=true
```

该入口会：

- 加载 `src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml`。
- 启动 `map_server + AMCL + Nav2`。
- 不启动 `slam_toolbox`。
- 不启动 `frontier_explorer`。
- 不进行重新建图或保存地图。
- 启用 RViz 后，建议先用 `2D Pose Estimate` 设置 AMCL 初始位姿，再用 `2D Goal Pose` 或 `NavigateToPose` 发送目标。

使用 Phase 6 上一版稳定地图做静态导航回放：

```bash
ros2 launch tugbot_bringup tugbot_nav_phase6_map.launch.py \
  headless:=false \
  use_rviz:=true
```

Phase 6 cleanup map 仍被保留为上一版稳定地图。Phase 7A budget map 仅作为负面实验记录保留，不推荐作为默认成果。

## 推荐地图与历史地图

### 当前推荐地图：Phase 14 perimeter no-spin map

```text
ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml
ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.pgm
```

推荐理由：

- Phase 14 人工验收通过。
- no-spin + 低角速度 + `perimeter_then_frontier` 策略改善了地图重影问题。
- 地图墙线清晰，外轮廓完整，内墙稳定。
- 无明显重影、错层、多层黑边。
- box pillar 能被稳定测绘。
- camera image 正常显示。
- Phase 15 已将人工验收通过的 `tugbot_nav_world_slam_perimeter_test.yaml/.pgm` 归档为 Phase 14 推荐地图，并修正 YAML `image` 字段指向新的 `.pgm` 文件名。
- Phase 16 已新增独立静态回放入口，默认加载该推荐地图。

### 上一版稳定地图：Phase 6 cleanup map

```text
ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
```

Phase 6 cleanup map 作为上一版稳定地图保留。Phase 8 已验证该地图可被 `map_server` 加载，AMCL 可定位，Nav2 可完成静态地图导航回放。

### 负面实验地图：Phase 7A budget map

```text
ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.yaml
ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.pgm
```

Phase 7A 执行过程通过，但地图质量不采纳。原因包括重影、墙线重复、局部错层、扫描未对齐，以及过度补扫降低 SLAM 一致性。该地图仅作为负面实验记录保留，不推荐默认使用。

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
4. 读 `ros2_ws_tugbot_nav_20260514/README.md`，进入自主探索建图、Phase 14 推荐地图和 Phase 14 静态回放。
5. 读 `ros2_ws_tugbot_nav_20260514/doc/doc_report/phase16_phase14_static_replay_entry_report.md`，查看最新 Phase 16 静态回放入口结论。
6. 回看 `ros2_ws_tugbot_nav_20260514/doc/doc_report/phase15_phase14_map_manual_acceptance_archive_report.md`，理解 Phase 14 推荐地图如何归档为当前成果。
7. 回看 `ros2_ws_tugbot_nav_20260514/doc/doc_report/phase14_no_spin_mapping_stability_report.md`，理解 no-spin / 低角速度策略为什么被采用。
8. 如果要继续改探索策略，读 `ros2_ws_tugbot_nav_20260514/doc/doc_experience/tugbot_autonomous_mapping_success_experience.md`。

## 常用验证命令

构建当前推荐工作空间：

```bash
cd ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

运行 contract test：

```bash
cd ros2_ws_tugbot_nav_20260514
python3 -m pytest -q src/tugbot_bringup/test/test_contract.py
```

查看 Phase 14 静态回放入口参数：

```bash
cd ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tugbot_bringup tugbot_nav_phase14_map.launch.py --show-args
```

启动 Phase 14 静态地图回放后，可查看 ROS graph：

```bash
ros2 node list
ros2 topic list
ros2 topic info -v /map
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

## Phase 14 / Phase 15 / Phase 16 关系

```text
Phase 14：no-spin / 低角速度 + perimeter_then_frontier 策略得到人工验收通过的高质量地图。
Phase 15：将人工验收通过的地图归档为当前推荐成果地图：tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml/.pgm。
Phase 16：新增 tugbot_nav_phase14_map.launch.py，让当前推荐地图拥有独立、语义清晰的静态导航回放入口。
```

Phase 16 的边界：

- 新增 `tugbot_nav_phase14_map.launch.py`。
- 新增 `nav2_phase14_map_params.yaml`，语义上对应 Phase 14 推荐地图静态回放。
- 默认 map 指向 Phase 14 推荐地图。
- 启动 `map_server + AMCL + Nav2`。
- 不启动 `slam_toolbox`、`frontier_explorer` 或探索 launch。
- 不重新生成地图，不保存新地图。
- 不修改 Phase 14 推荐地图本身。
- 保留 Phase 6 cleanup map 作为上一版稳定地图。
- 保留 Phase 7A budget map 作为负面实验记录。

## 已知边界

- 这是实验型 ROS 2/Gazebo 仓库，优先展示可复现实验链路与阶段验证，而不是已经产品化的机器人系统。
- Gazebo GUI 在 VMware、Wayland/X11、OpenGL/EGL 组合环境下可能出现图形 warning；headless 模式通常更适合自动化验证。
- `ros2_ws_tugbot_nav_20260514` 中 Phase 14 perimeter no-spin map 是当前推荐成果；Phase 6 cleanup map 是上一版稳定成果；Phase 7A budget map 是负面实验记录。
- `tugbot_explore.launch.py` 默认策略仍是 `frontier`，推荐演示 Phase 14 策略时请显式设置 `exploration_strategy:=perimeter_then_frontier`。
- 普通探索、headless 验证或新实验应保存到新文件名，避免覆盖 Phase 14 推荐地图、Phase 6 稳定地图或 Phase 7A 负面实验地图。
- Phase 16 只验证了 Phase 14 静态回放入口可以启动、`map_server` 加载 Phase 14 map、AMCL/Nav2 lifecycle active、`/map` 由 `/map_server` 发布；没有重新执行长时间探索，也没有重新生成地图。
- 当前 camera 仅用于可视化，不参与 Nav2 避障；Nav2 仍主要依赖 `/scan`。
- 自主探索仍依赖 SLAM 几何一致性和 frontier/perimeter 候选点安全性；更复杂环境下建议继续做地图质量评分、planner precheck 和多 world 泛化测试。
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
