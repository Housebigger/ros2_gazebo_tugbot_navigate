# Phase 12：将 cone 障碍物替换为 static box pillar

## 1. 修改文件清单

本阶段只修改当前 20260514 工作空间，未修改 0513 工程，未修改 Phase 6 推荐地图文件，未删除旧阶段报告。

修改文件：

- `src/tugbot_gazebo/worlds/tugbot_nav_world.sdf`
  - 将两个原 cone 导航障碍物替换为 `box_pillar_0` / `box_pillar_1`。
- `src/tugbot_bringup/test/test_contract.py`
  - 将 Phase 11 cone static 契约升级为 Phase 12 box pillar 契约。
  - 保留 Phase 10 / Phase 11 关键契约检查。
- `README.md`
  - 轻量补充 static box pillar、`/scan` 与 Gazebo collision 一致性说明。
- `doc/doc_report/phase12_box_pillar_obstacle_report.md`
  - 本报告。

未修改：

- `src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml`
- `src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm`
- 旧阶段报告文件。

## 2. 原 cone 问题分析：为什么不用 cone

人工复检发现，即使两个锥形雪糕筒已经设为 static，Tugbot 在未知地图自主探索时仍容易撞到锥形筒底部，并随后出现 SLAM 地图错位、重影和堆叠。

原因是 cone 对当前 2D LiDAR + Nav2 costmap 链路不友好：

- Tugbot 的 2D LiDAR 扫描高度只能扫到 cone 中间偏上的截面；
- cone 在激光扫描高度处的截面小于底部实际碰撞范围；
- Nav2 costmap 基于 `/scan` 看到的是较小障碍边界；
- Gazebo 物理碰撞体底部更宽；
- 因此会出现“雷达认为能过，底盘实际撞到”的不一致；
- 撞击后即使 cone 不移动，也可能造成 odom / 实际运动 / scan matching 不一致，诱发地图错位、重影和堆叠。

box pillar 具有竖直侧面，2D LiDAR 在扫描高度看到的边界与 Gazebo collision 的水平外轮廓更一致，更适合当前 2D SLAM + Nav2 避障验证。

## 3. 新 box pillar 名称、位置、尺寸

在 `src/tugbot_gazebo/worlds/tugbot_nav_world.sdf` 中，原：

- `cone`
- `cone_0`

已替换为：

| 新模型名 | pose x | pose y | pose z | size x | size y | size z |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `box_pillar_0` | `3.805579848063759` | `-2.5960888656381456` | `0.6` | `0.50` | `0.50` | `1.20` |
| `box_pillar_1` | `6.0753818100601649` | `2.0667701473945264` | `0.6` | `0.50` | `0.50` | `1.20` |

说明：

- x/y 沿用原 cone / cone_0 的中心位置；
- z 设置为高度一半 `0.6`，对应 `size z = 1.20`；
- `1.20 m` 高度覆盖 Tugbot 2D LiDAR 扫描高度；
- 两个 box pillar 使用不同醒目颜色，便于 Gazebo GUI 人工辨认。

## 4. static 设置证据

SDF 中两个新模型均包含：

```xml
<static>true</static>
```

自动结构化检查结果：

```text
models_present ['box_pillar_0', 'box_pillar_1']
box_pillar_0 pose= 3.805579848063759 -2.5960888656381456 0.6 0 0 0 static= true
box_pillar_1 pose= 6.0753818100601649 2.0667701473945264 0.6 0 0 0 static= true
cone_geometry_count 0
```

## 5. visual / collision 设置证据

两个 box pillar 均使用一致的 visual / collision box geometry：

```xml
<collision name='box_pillar_collision'>
  <geometry>
    <box>
      <size>0.50 0.50 1.20</size>
    </box>
  </geometry>
</collision>

<visual name='box_pillar_visual'>
  <geometry>
    <box>
      <size>0.50 0.50 1.20</size>
    </box>
  </geometry>
</visual>
```

自动结构化检查结果：

```text
box_pillar_0 collision_size= 0.50 0.50 1.20
box_pillar_0 visual_size= 0.50 0.50 1.20
box_pillar_1 collision_size= 0.50 0.50 1.20
box_pillar_1 visual_size= 0.50 0.50 1.20
```

源码搜索确认当前目标 world 内不再包含 cone 导航障碍物：

```text
search_files pattern: <model name=['"]cone|<name>cone|model://.*cone|traffic|snow|<cone>
result: total_count = 0
```

## 6. contract test 加强内容

`src/tugbot_bringup/test/test_contract.py` 新增 / 更新 `test_phase12_box_pillar_obstacles_replace_cones_contract`，检查：

- world SDF 不再包含 `cone` / `cone_0` 模型；
- world SDF 不再包含 collision cone geometry；
- world SDF 包含 `box_pillar_0` / `box_pillar_1`；
- 两个 box pillar 均为 `<static>true</static>`；
- 两个 box pillar 的 visual geometry 是 box；
- 两个 box pillar 的 collision geometry 是 box；
- visual / collision size 均为 `0.50 0.50 1.20`；
- pose x/y 沿用原 cone 中心位置，z 为 `0.6`。

同时完整 contract test 仍覆盖原 Phase 10 关键契约：

- `tugbot_explore.launch.py` 默认仍是 Phase 6 稳健参数，而不是 Phase 7A 激进参数；
- `tugbot_nav_phase6_map.launch.py` 使用 Phase 6 cleanup map；
- README 不默认覆盖 Phase 6 地图；
- camera topic / RViz Image Display / “camera 仅用于可视化，Nav2 仍主要依赖 `/scan`”说明仍保留。

## 7. build 结果

执行命令：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
colcon build --symlink-install
```

结果：

```text
Summary: 5 packages finished [1.39s]
```

结论：build 通过。

## 8. contract test 结果

执行命令：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest -q src/tugbot_bringup/test/test_contract.py
```

结果：

```text
14 passed in 0.01s
```

结论：contract test 全部通过。

## 9. 短时间 Gazebo / RViz / ROS topic 验证结果

执行命令：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source install/setup.bash
ros2 launch tugbot_bringup tugbot_explore.launch.py \
  headless:=false \
  use_rviz:=true \
  max_goals:=5 \
  enable_cleanup_mode:=true \
  save_map:=false
```

验证范围：短时间 bringup / Gazebo / RViz / ROS topic 验证，未运行长时间探索实验，未保存地图。

### 9.1 Gazebo / RViz / ROS graph

短时间启动后 ROS graph 关键节点存在：

```text
/ros_gz_bridge
/slam_toolbox
/frontier_explorer
/local_costmap/local_costmap
/global_costmap/global_costmap
/rviz2
```

`/scan` topic：

```text
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
Publisher: /ros_gz_bridge
Subscription count: 5
Subscribers include: /global_costmap, /local_costmap, /slam_toolbox, /collision_monitor, /rviz2
```

`/camera/image_raw` topic：

```text
Type: sensor_msgs/msg/Image
Publisher count: 1
Publisher: /ros_gz_bridge
Subscription count: 1
Subscriber: /rviz2
```

costmap topics 存在：

```text
/global_costmap/costmap
/global_costmap/obstacle_layer
/local_costmap/costmap
/local_costmap/voxel_layer
/scan
/map
/map_metadata
```

### 9.2 Gazebo live scene 中 box pillar 存在

通过 Gazebo scene info 采样：

```text
box_pillar_0_count: 1
box_pillar_1_count: 1
cone_count: 0
cone_0_count: 0
```

scene info 中可见两个 visual 均为 box，尺寸为：

```text
x: 0.5
y: 0.5
z: 1.2
```

注：Gazebo `scene/info` 主要暴露 visual scene 信息，不完整展示 collision/static 字段；collision/static 以 SDF 源码与 contract test 为权威证据。

### 9.3 是否确认 `/scan` 能观测 box pillar

短时间 `/scan` 采样文件：`/tmp/phase12_scan_once.yaml`

解析结果：

```text
range_count: 129
finite_count: 128
finite_min: 3.1559557914733887
finite_max: 3.944643497467041
finite_under_8m_count: 128
```

结论：`/scan` 正常发布，RViz / Nav2 / SLAM / collision monitor 均已订阅 `/scan`。由于本次为短时间验证，没有长时间人工驾驶逐个绕柱贴近验证；但从 world geometry、box 竖直侧面、scan topic 正常发布、costmap 订阅 `/scan` 与 Gazebo scene 中 box pillar 存在可以确认：box pillar 已进入可由 2D LiDAR / costmap 观测的场景链路。最终近距离绕障效果仍建议人工验收时在 RViz LaserScan 与 costmap 中观察确认。

### 9.4 costmap 与 camera 功能

短时间 topic echo 验证：

```text
/local_costmap/costmap: width=60 height=60 resolution=0.05000000074505806 has_data=true
/global_costmap/costmap: width=315 height=155 resolution=0.05000000074505806 has_data=true
/camera/image_raw: width=640 height=480 encoding present, data present
```

结论：

- local/global costmap 正常发布；
- `/camera/image_raw` 正常发布；
- RViz 订阅 `/camera/image_raw`；
- camera 可视化链路未被破坏；
- README 中仍明确 camera 仅用于可视化，当前 Nav2 避障仍主要依赖 `/scan`。

### 9.5 进程清理

短时间验证结束后已停止 launch / Gazebo / RViz / Nav2 / SLAM 相关进程。最终检查仅剩 ROS daemon：

```text
/usr/bin/python3 ... ros2-daemon ...
```

## 10. 当前遗留问题

- 本阶段没有运行长时间自主探索实验，因此不把“完全不会再发生 SLAM 重影/堆叠”作为自动验收结论。
- `/scan` 与 costmap topic 已验证正常，box pillar 已在 Gazebo live scene 中出现；但“贴近 box pillar 时 local/global costmap 边界更明显、小车不再撞柱”仍需要人工在 Gazebo/RViz 中近距离观察确认。
- 旧 Phase 6 cleanup map 没有重建；它仍反映旧阶段建图成果。本阶段明确不修改 Phase 6 推荐地图文件。
- 当前 Nav2 避障仍主要依赖 2D `/scan`，camera 仍只是可视化增强，不参与 costmap 或避障决策。

## 11. Phase 12 验收结论

Phase 12 工程修改已完成：

- 两个 cone / snow cone / traffic cone 导航障碍物已替换为 static box pillar；
- 新障碍物命名为 `box_pillar_0` / `box_pillar_1`；
- x/y 位置沿用原 cone 中心，z 设置为 `0.6`；
- 尺寸为 `0.50 x 0.50 x 1.20 m`；
- visual 与 collision 均为 box geometry 且尺寸一致；
- SDF XML parse 通过；
- contract test 全部通过；
- colcon build 通过；
- 短时间 Gazebo/RViz/ROS topic 验证通过；
- camera 可视化链路未破坏；
- 未修改 0513 工程；
- 未修改 Phase 6 推荐地图文件；
- 未删除旧阶段报告；
- 未运行长时间探索实验。

结论：Phase 12 可进入人工验收。建议人工验收重点观察 Gazebo 中两个原 cone 位置是否已变为 static box pillar，RViz LaserScan / local costmap / global costmap 是否能稳定显示 box pillar 边界，以及 Tugbot 接近障碍物时是否不再因为 cone 底部未被雷达识别而发生底盘碰撞。
