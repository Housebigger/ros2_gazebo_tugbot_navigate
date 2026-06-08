# Tugbot ROS 2 + Gazebo Maze Exploration 20260522

`ros2_ws_tugbot_nav_20260522` 是从 `ros2_ws_tugbot_nav_20260514` 复制演进出的迷宫自主探索工作空间。

目标：继承 0514 已验证的 Gazebo + ros_gz_bridge + slam_toolbox + Nav2 + frontier/perimeter + no-spin 成功链路，在迷宫环境中让 Tugbot 从入口开始自主探索，并最终从出口驶出。

## 当前 Active World：Phase35-pre

当前人工验收通过的 active candidate Gazebo world 是：

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

验收链路：

```text
Phase33F: PASS_AS_SCALED_CLEAN_WORLD_VISUALLY_ACCEPTED
Phase34:  PASS_AS_SCALED_CLEAN_WORLD_SLAM_DATAFLOW_AND_MAP_VISUAL_ACCEPTED
```

Active core files：

```text
src/tugbot_maze/assets/maze_20260528.png
src/tugbot_maze/config/maze_20260528_scaled_instance.yaml
src/tugbot_maze/config/maze_wall_segments_20260528.yaml
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
log/phase34_scaled_clean_world_slam_smoke/phase34_scaled_clean_world_slam_smoke_map_snapshot.yaml
log/phase34_scaled_clean_world_slam_smoke/phase34_scaled_clean_world_slam_smoke_map_snapshot.pgm
```

说明文档：

```text
doc/ACTIVE_MAZE_WORLD.md
```

Phase35-pre 清理原则：旧 world/map/image artifacts 不做不可恢复删除，已归档到：

```text
archive/deprecated_maze_worlds/
doc/archive/phase_old_maze_artifacts/
```

明确 deprecated：

- `maze_20260522.jpg` decorative route deprecated；
- `manual_simplified_first_pass` scaffold deprecated for navigation conclusions；
- unscaled clean candidate superseded by scaled2x world；
- legacy static/explored maps 不再作为当前 maze workflow 默认地图。

本状态不声明导航成功；Phase35-pre 不启动导航、不修改 Nav2/MPPI/controller 参数、不继续 fallback/terminal acceptance。

## 启动入口总览

0522 迷宫工作空间继承 0514，并保留继承 0513 到 0514 的稳定启动入口，避免把静态地图回放、SLAM 在线建图、SLAM + Nav2 手动导航、frontier 自主探索建图混在同一个 launch 中。

当前 maze workflow 默认 world 已更新为 scaled2x active candidate：

```text
tugbot_maze_world_20260528_clean_scaled2x.sdf
```

- `tugbot_maze_slam.launch.py`：Maze SLAM-only 入口，当前默认加载 active scaled2x world。
- `tugbot_maze_slam_nav.launch.py`：Maze SLAM + Nav2 入口，当前默认加载 active scaled2x world；仅在后续明确允许 Nav2/manual goal 的阶段使用。
- `tugbot_maze_explore.launch.py`：Maze 自主探索入口，当前默认加载 active scaled2x world；仅在后续明确允许自主探索的阶段使用。
- `tugbot_nav.launch.py` / `tugbot_nav_phase*_map.launch.py`：旧静态地图回放入口，保留为 legacy/provenance，不代表当前 scaled clean maze workflow。
- `tugbot_slam.launch.py` / `tugbot_slam_nav.launch.py` / `tugbot_explore.launch.py`：继承 0514 非 maze world 入口，保留用于历史/非 maze baseline，不代表当前 scaled clean maze workflow。

## 构建

```bash
cd ros2_ws_tugbot_nav_20260522
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 当前推荐 smoke：Maze SLAM-only

仅验证 Gazebo scaled clean maze world、bridge、slam_toolbox 与 live `/map`，不启动 Nav2，不启动 maze_explorer：

```bash
ros2 launch tugbot_bringup tugbot_maze_slam.launch.py \
  headless:=false \
  use_rviz:=true
```

也可显式传入 active world：

```bash
ros2 launch tugbot_bringup tugbot_maze_slam.launch.py \
  world_sdf:=/absolute/path/to/ros2_ws_tugbot_nav_20260522/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf \
  headless:=false \
  use_rviz:=true
```

## Later-only: Maze SLAM + Nav2 手动导航

当前阶段不要启动。仅在后续明确允许 Nav2/manual goal 的 phase 使用：

```bash
ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py \
  headless:=false \
  use_rviz:=true
```

## Later-only: Maze 自主探索基础入口

当前阶段不要启动。仅在后续明确允许自主探索的 phase 使用：

```bash
ros2 launch tugbot_bringup tugbot_maze_explore.launch.py \
  headless:=false \
  use_rviz:=true \
  max_goals:=30 \
  save_map:=true \
  map_save_path:=/absolute/path/to/ros2_ws_tugbot_nav_20260522/src/tugbot_navigation/maps/maze/maze_slam_trial_001
```

成功判据仍是到达配置的出口坐标/区域，而不是完整探索全图；但 Phase35-pre 不评估导航成功。

## 历史诊断状态摘要

Phase26-Final closure 报告保留为 MPPI 黑盒诊断链路归档：

```text
doc/doc_report/phase26_final_diagnostic_closure_report.md
```

Phase27-alt fallback 设计/实现保留为 default-off 历史能力；Phase35-pre 不继续 fallback/terminal acceptance。

```text
near_exit_fallback_enabled default: false
```

## 迷宫图片说明

旧参考图 `maze_20260522.jpg` 是 3D/浮雕风格 JPEG，有阴影、渐变和水印，不是干净的黑白俯视 occupancy map。该 decorative route 已 deprecated，不再作为当前导航结论依据。

当前 active clean source image：

```text
src/tugbot_maze/assets/maze_20260528.png
```

该 clean 2D binary maze 经 Phase31~Phase34 生成、放大、marker 修正、visual recheck 与 SLAM/map smoke 人工验收，当前 active world 为 scaled2x SDF。
