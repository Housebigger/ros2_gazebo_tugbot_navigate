# Phase 13：perimeter-first 外轮廓优先探索策略实验

## 1. 修改文件清单

本阶段只修改当前 20260514 工作空间：

`/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514`

遵守边界：

- 未修改 0513 工程；
- 未修改 Phase 6 推荐地图文件；
- 未删除 Phase 7A 地图；
- 未改变 `tugbot_explore.launch.py` 的默认稳定 frontier 行为；
- camera 仍只用于可视化，不参与导航；
- box pillar 障碍物保持 static；
- 未运行长时间全量探索实验，只做短程 / 中程机制验证。

修改文件：

- `src/tugbot_exploration/tugbot_exploration/frontier_explorer.py`
  - 在现有 frontier + cleanup 探索器中新增可选 `perimeter_then_frontier` 状态机。
  - 新增外轮廓墙体 cluster 检测、perimeter waypoint 生成、cw/ccw 排序、逐点 NavigateToPose 执行、失败跳过 / fallback 逻辑。
  - 保留既有 frontier / cleanup 逻辑作为默认与回退路径。
- `src/tugbot_bringup/launch/tugbot_explore.launch.py`
  - 新增 `exploration_strategy` 和 `perimeter_*` launch 参数。
  - 默认 `exploration_strategy='frontier'`，必须显式传入 `exploration_strategy:=perimeter_then_frontier` 才启用新策略。
- `src/tugbot_bringup/test/test_contract.py`
  - 新增 Phase 13 contract test，检查默认策略不被改成 perimeter、参数存在、日志关键词存在、camera / box pillar 不被探索器引用。
  - 保留现有 Phase 10 / Phase 12 / Phase 6 map / README 防覆盖等契约。
- `doc/doc_report/phase13_perimeter_first_exploration_report.md`
  - 本报告。

## 2. 策略设计说明

Phase 13 采用“外轮廓先扫，再切回 frontier + cleanup”的实验策略：

1. 启动时仍由 `tugbot_explore.launch.py` bringup Gazebo、slam_toolbox、Nav2、RViz、bridge 与 `frontier_explorer`。
2. 默认策略仍为 `frontier`，不影响既有稳定探索。
3. 当显式设置 `exploration_strategy:=perimeter_then_frontier` 时，探索器进入 Phase 13 状态机。
4. 状态机流程：
   - `pending_initial_spin`：等待 TF / map 可用；
   - `initial_spin`：调用 Nav2 Spin 原地环视一圈，让初始局部地图更完整；
   - `detect_walls`：从当前 `/map` 的 occupied cells 中提取大跨度疑似墙体 cluster；
   - `execute_waypoints`：在墙体内侧安全距离生成 waypoint，并用 Nav2 `NavigateToPose` 逐个执行；
   - `frontier`：perimeter 完成、失败或无 waypoint 时切回原 frontier + cleanup。
5. 不实现底层 wall-follow controller，不直接控制 `/cmd_vel` 做贴墙控制，仍全部通过 Nav2 `NavigateToPose` 执行。
6. 若墙体 cluster 识别失败或 waypoint 为空，立即 fallback 到 frontier，不允许卡死。

## 3. 新增参数清单

`src/tugbot_bringup/launch/tugbot_explore.launch.py` 中新增并传递以下参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `exploration_strategy` | `frontier` | 探索策略；可选 `frontier` / `perimeter_then_frontier`。默认不启用新策略。 |
| `perimeter_enable_initial_spin` | `true` | perimeter 阶段前是否执行原地环视。 |
| `perimeter_spin_angle` | `6.28` | 初始环视角度。 |
| `perimeter_wall_min_cluster_size` | `80` | 疑似墙体 occupied cluster 最小 cell 数。 |
| `perimeter_wall_min_length_m` | `1.5` | 疑似墙体 bbox 最小长边长度。 |
| `perimeter_wall_aspect_ratio_min` | `4.0` | 疑似墙体 bbox 最小长宽比。 |
| `perimeter_wall_offset_m` | `0.75` | waypoint 相对墙体向地图内侧偏移距离。 |
| `perimeter_waypoint_spacing_m` | `1.0` | 沿墙采样 waypoint 间距。 |
| `perimeter_max_waypoints` | `20` | perimeter 阶段最多 waypoint 数。 |
| `perimeter_direction` | `ccw` | waypoint 排序方向，可选 `ccw` / `cw`。 |
| `perimeter_switch_to_frontier_after_done` | `true` | perimeter 阶段结束后是否切回 frontier。 |
| `perimeter_goal_timeout_sec` | `60.0` | 单个 perimeter waypoint 的 NavigateToPose 超时。 |

## 4. wall cluster 检测方法

实现在 `frontier_explorer.py` 的 `_detect_perimeter_wall_clusters()`：

1. 从 `/map` 的 `OccupancyGrid` 中提取 occupied cells：`value > 50`。
2. 为了把细碎墙段连成可识别长段，对 occupied cells 做约 `0.20 m` 半径的轻量桥接 / 膨胀。
3. 对桥接后的 occupied cells 做连通域聚类。
4. 对每个 cluster 计算：
   - cell 数量；
   - bbox；
   - 长边长度；
   - 短边厚度；
   - 长宽比；
   - 水平 / 垂直主方向。
5. 过滤条件：
   - `len(cluster) >= perimeter_wall_min_cluster_size`；
   - `bbox 长边 >= perimeter_wall_min_length_m`；
   - `aspect_ratio >= perimeter_wall_aspect_ratio_min`。
6. 按墙体长度和 cluster size 降序排序，优先使用大跨度墙段。

中程验证日志显示：

```text
perimeter wall cluster detection: occupied_cells=184 bridged_cells=2286 raw_clusters=8 detected wall clusters=3 min_cluster_size=80 min_length=1.50 aspect_min=4.00 top_length=4.25 top_aspect=9.44
```

结论：第一版在当前初始局部图上可以识别疑似大跨度墙体 cluster。

## 5. waypoint 生成方法

实现在 `frontier_explorer.py` 的 `_generate_perimeter_waypoints()` 与 `_sort_perimeter_waypoints()`：

1. 对每个 wall cluster 根据主方向采样：
   - 垂直墙：沿 y 方向按 `perimeter_waypoint_spacing_m` 采样；
   - 水平墙：沿 x 方向按 `perimeter_waypoint_spacing_m` 采样。
2. 根据墙体 center cell 和 map center 估计地图内侧方向。
3. 将采样点向内侧偏移 `perimeter_wall_offset_m`，得到候选 waypoint。
4. 验证候选点：
   - 必须位于地图内部；
   - 必须是 known free cell；
   - 若偏移点不是 free，则在局部半径内搜索最近安全 free cell；
   - 到 occupied cell 的距离必须不少于 `0.45 m`；
   - 与已接受 waypoint 保持约 `0.65 * spacing` 以上间距；
   - 不落入探索器已有 blacklist。
5. waypoint yaw 指向对应墙体中心，便于扫描墙面 / 房间边界。
6. 排序：
   - 以所有 accepted waypoint 的几何中心为参考；
   - 根据极角排序；
   - `perimeter_direction=ccw` 正序，`cw` 反序；
   - 从距离机器人最近的 waypoint 开始执行。
7. 最多保留 `perimeter_max_waypoints` 个。

中程验证日志显示：

```text
accepted perimeter waypoints=4 rejected perimeter waypoints=0 wall_clusters=3 offset_m=0.75 spacing_m=1.00 clearance_min_m=0.45
generated perimeter waypoints sorted: accepted=4 direction=ccw start_index=3 center=(2.214, -0.473)
generated perimeter waypoints: generated=4 accepted perimeter waypoints=4 rejected perimeter waypoints=0 direction=ccw max=4
```

结论：第一版能在 known free 区域生成安全 waypoint，并按 `ccw` 方向排序。

## 6. perimeter 执行日志

### 6.1 短程 headless 机制验证

执行了 bounded 短程验证，限制 `perimeter_max_waypoints:=4`，避免长时间探索：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source install/setup.bash
timeout 75s ros2 launch tugbot_bringup tugbot_explore.launch.py \
  headless:=true \
  use_rviz:=false \
  exploration_strategy:=perimeter_then_frontier \
  max_goals:=1 \
  enable_cleanup_mode:=true \
  save_map:=false \
  perimeter_max_waypoints:=4
```

关键日志：

```text
perimeter_then_frontier initial spin starting: perimeter_spin_angle=6.28; visual sensors remain outside navigation control
perimeter_initial spin finished: status=4 error_code=0 error_msg= success=True; waiting 2.0s for map update
perimeter wall cluster detection: occupied_cells=184 bridged_cells=2286 raw_clusters=8 detected wall clusters=3 ... top_length=4.25 top_aspect=9.44
generated perimeter waypoints: generated=4 accepted perimeter waypoints=4 rejected perimeter waypoints=0 direction=ccw max=4
Executing perimeter waypoint 1/4 ...
perimeter waypoint succeeded: success=1 failure=0 current_index=1/4
Executing perimeter waypoint 2/4 ...
perimeter waypoint succeeded: success=2 failure=0 current_index=2/4
Executing perimeter waypoint 3/4 ...
perimeter waypoint succeeded: success=3 failure=0 current_index=3/4
Executing perimeter waypoint 4/4 ...
perimeter waypoint succeeded: success=4 failure=0 current_index=4/4
Perimeter phase complete; switching to frontier: reason=all perimeter waypoints processed detected wall clusters=3 generated perimeter waypoints=4 accepted perimeter waypoints=4 rejected perimeter waypoints=0 success=4 failure=0 switch_to_frontier=True
Selected frontier goal #1: ...
```

结果：短程机制验证通过。

### 6.2 中程 Gazebo / RViz 可视化验证

执行用户要求方向的可视化命令，但为了遵守“不运行过长实验”，使用 bounded timeout，并将 `perimeter_max_waypoints:=4` 限制为短中程 waypoint 数：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source install/setup.bash
timeout 135s ros2 launch tugbot_bringup tugbot_explore.launch.py \
  headless:=false \
  use_rviz:=true \
  exploration_strategy:=perimeter_then_frontier \
  max_goals:=20 \
  enable_cleanup_mode:=true \
  save_map:=true \
  map_save_path:=/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_perimeter_test \
  perimeter_max_waypoints:=4
```

关键日志摘录：

```text
perimeter_then_frontier initial spin starting: perimeter_spin_angle=6.28; visual sensors remain outside navigation control
perimeter wall cluster detection: occupied_cells=184 bridged_cells=2286 raw_clusters=8 detected wall clusters=3 min_cluster_size=80 min_length=1.50 aspect_min=4.00 top_length=4.25 top_aspect=9.44
generated perimeter waypoints sorted: accepted=4 direction=ccw start_index=3 center=(2.214, -0.473)
generated perimeter waypoints: generated=4 accepted perimeter waypoints=4 rejected perimeter waypoints=0 direction=ccw max=4
Executing perimeter waypoint 1/4: x=2.202 y=-0.023 yaw=0.185 source_cluster=0 clearance_cells=15 success=0 failure=0
perimeter waypoint succeeded: success=1 failure=0 current_index=1/4
Executing perimeter waypoint 2/4: x=2.152 y=-1.923 yaw=1.178 source_cluster=0 clearance_cells=15 success=1 failure=0
perimeter waypoint succeeded: success=2 failure=0 current_index=2/4
Executing perimeter waypoint 3/4: x=2.252 y=-1.023 yaw=0.993 source_cluster=0 clearance_cells=15 success=2 failure=0
perimeter waypoint succeeded: success=3 failure=0 current_index=3/4
Executing perimeter waypoint 4/4: x=2.252 y=1.077 yaw=-0.903 source_cluster=0 clearance_cells=15 success=3 failure=0
perimeter waypoint succeeded: success=4 failure=0 current_index=4/4
Perimeter phase complete; switching to frontier: reason=all perimeter waypoints processed detected wall clusters=3 generated perimeter waypoints=4 accepted perimeter waypoints=4 rejected perimeter waypoints=0 success=4 failure=0 switch_to_frontier=True
Selected frontier goal #1: x=2.602 y=-1.173 cluster_size=1015 distance=2.258 score=1016.746 search_mode=strict clearance_cells=10 free_neighbor_count=81
Selected frontier goal #2: x=5.302 y=-2.673 cluster_size=954 distance=3.091 score=955.269 search_mode=strict clearance_cells=13 free_neighbor_count=78
```

结果：

- 初始原地环视执行成功；
- 识别出 3 个疑似墙体 cluster；
- 生成 4 个 perimeter waypoint，0 个 rejected；
- 4 个 waypoint 均由 Nav2 `NavigateToPose` 成功到达；
- perimeter 阶段结束后成功切回 frontier；
- frontier 阶段开始继续选择 frontier goal；
- 因 bounded timeout 主动截断，没有等待完整探索完成；
- 因未到探索完成条件，`save_map:=true` 未触发最终 map_saver，因此未生成 `tugbot_nav_world_slam_perimeter_test.yaml/.pgm`。

## 7. 切回 frontier 的条件

Phase 13 实现中以下情况会切回原 frontier + cleanup：

1. `exploration_strategy` 不是 `perimeter_then_frontier`：直接使用默认 frontier。
2. 传入非法 `exploration_strategy`：打印 warning 并 fallback 到 frontier。
3. 初始 spin 完成后进入 wall detection。
4. wall cluster 检测失败，或没有符合阈值的疑似墙体。
5. waypoint 生成为空。
6. 所有 perimeter waypoint 均执行完毕。
7. 单个 waypoint 失败 / 超时：记录失败计数与 blacklist，跳过该 waypoint，继续下一个；全部处理完后切回 frontier。
8. 若 `perimeter_switch_to_frontier_after_done=false`，perimeter 完成后标记探索结束；默认值为 `true`，因此默认切回 frontier。

实测切回日志：

```text
Perimeter phase complete; switching to frontier: reason=all perimeter waypoints processed detected wall clusters=3 generated perimeter waypoints=4 accepted perimeter waypoints=4 rejected perimeter waypoints=0 success=4 failure=0 switch_to_frontier=True
```

## 8. 地图覆盖对比

已知人工测试结论：当前纯 frontier + cleanup 能完成主体地图测绘，但存在头尾两个房间覆盖不足；单纯增加 `max_goals` 容易导致过度探索和 SLAM 重影，不适合作为唯一方案。

Phase 13 bounded 验证观察：

- perimeter_then_frontier 在 frontier 之前主动完成了一段沿墙内侧巡航；
- `/map` 统计在切回 frontier 后继续更新，并能继续产生有效 frontier goal；
- 该策略把“先扫外轮廓”作为前置阶段，而不是通过无限增加 frontier goal 数量解决漏扫；
- 由于本轮只运行短程 / 中程验证，未跑完整地图收敛，因此不能严肃宣称最终覆盖率已经超过纯 frontier；
- 本轮也没有生成最终 saved map，因此不能用 `*.pgm` 做像素级覆盖率对比。

当前可确认的是“机制改善”：

- 外轮廓识别、waypoint 生成、Nav2 waypoint 执行、切回 frontier 四条主链路均成立；
- 后续人工验收可在同一参数下放宽 timeout / waypoint 数，观察头尾房间覆盖是否改善。

## 9. 是否改善头尾房间漏扫

本阶段结论分层如下：

- 机制层面：有改善潜力。策略确实让机器人在 frontier 前先贴近墙体内侧巡航，并可切回 frontier 补扫 unknown。
- 中程验证层面：尚不能定论。当前 bounded 验证只完成 4 个 perimeter waypoint 与 2 个 frontier goal 选择，未完成全图探索。
- 验收层面：需要人工在 Gazebo / RViz 中运行更长但仍受控的测试，重点观察头尾两个房间是否比纯 frontier 更早、更稳定地进入视野。

所以 Phase 13 第一版验收建议为：

- 接受为“策略原型与机制验证通过”；
- 不把它声明为“最终解决头尾漏扫”的完成版；
- 下一轮若继续推进，可增加 waypoint 数到默认 20，或调整墙体排序 / 外墙覆盖范围，再做受控 A/B 对比。

## 10. camera / box pillar / scan / costmap 链路检查

本阶段没有把 camera 引入导航逻辑。contract test 中显式检查：

```python
assert 'camera' not in explorer.lower()
assert 'box_pillar' not in explorer
```

中程可视化验证日志确认 camera 与 scan bridge 正常创建：

```text
Creating GZ->ROS Bridge: [/scan (gz.msgs.LaserScan) -> /scan (sensor_msgs/msg/LaserScan)]
Creating GZ->ROS Bridge: [/camera/image_raw (gz.msgs.Image) -> /camera/image_raw (sensor_msgs/msg/Image)]
Creating GZ->ROS Bridge: [/camera/camera_info (gz.msgs.CameraInfo) -> /camera/camera_info (sensor_msgs/msg/CameraInfo)]
```

Nav2 costmap 仍订阅 `/scan`：

```text
local_costmap.local_costmap: Subscribed to Topics: scan
global_costmap.global_costmap: Subscribed to Topics: scan
```

说明：camera 仍为可视化链路，导航仍由 `/scan`、SLAM、costmap、Nav2 构成；box pillar world 内容未在 Phase 13 中修改。

## 11. contract test 与构建结果

### 11.1 colcon build

执行命令：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
colcon build --symlink-install
```

结果：

```text
Summary: 5 packages finished [1.36s]
```

结论：build 通过。

### 11.2 py_compile

执行命令：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source install/setup.bash
python3 -m py_compile src/tugbot_exploration/tugbot_exploration/frontier_explorer.py
```

结果：exit code 0，无语法错误。

### 11.3 contract test

执行命令：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source install/setup.bash
python3 -m pytest -q src/tugbot_bringup/test/test_contract.py
```

结果：

```text
15 passed in 0.02s
```

结论：Phase 13 新契约与既有 Phase 10 / 12 契约均通过。

## 12. 进程清理结果

验证完成后已清理 live 进程，并检查无 Gazebo / RViz / Nav2 / SLAM / frontier_explorer 残留。

检查命令：

```bash
pgrep -af 'ros2 launch tugbot_bringup tugbot_explore|gz sim|parameter_bridge|ros_gz_bridge|frontier_explorer|slam_toolbox|async_slam_toolbox_node|controller_server|planner_server|bt_navigator|rviz2' || true
```

结果：无匹配 live 进程。

## 13. 当前遗留问题

1. 第一版墙体内侧判断使用“cluster center 相对 map center”的启发式，不是严格拓扑内外判定。
2. waypoint 排序基于极角，能形成 cw/ccw 巡航顺序，但不保证完整外轮廓闭环覆盖。
3. 当前验证限制 `perimeter_max_waypoints:=4`，属于短中程机制验证，不代表默认 20 waypoint 的最终表现。
4. 没有完整跑到探索完成，因此本阶段未生成最终 perimeter test map 文件。
5. 头尾房间漏扫改善尚需人工长一点的受控验收或后续 A/B 对比。
6. 若后续发现 waypoint 聚集在局部墙段，可进一步加入“按 cluster 分配 waypoint budget”或“优先外墙端点 / 门廊附近采样”的策略。
7. 当前不做 wall-follow controller，这是本阶段有意边界；精确贴墙巡航若要做，应另开 Phase。

## 14. Phase 13 验收结论

Phase 13 第一版“perimeter-first 外轮廓优先探索策略实验”达到原型验收标准：

- 默认探索策略仍为 `frontier`，未破坏稳定默认行为；
- `perimeter_then_frontier` 必须显式启用；
- 新增参数已加入 launch 并由 contract test 覆盖；
- 初始原地环视可执行；
- wall cluster 检测可识别大跨度疑似墙体；
- waypoint 可在 known free 且远离 occupied cell 的位置生成；
- waypoint 可按 ccw 排序并由 Nav2 `NavigateToPose` 执行；
- 4/4 perimeter waypoint 在中程验证中成功；
- perimeter 完成后可切回 frontier；
- camera / box pillar / scan / costmap 链路未被破坏；
- py_compile、pytest contract、colcon build 全部通过；
- live 进程已清理。

验收建议：

- 本阶段可作为 Phase 13 策略原型通过；
- 等待人工在 RViz / Gazebo 中复验外轮廓巡航轨迹与头尾房间覆盖改善；
- 若人工确认方向有效，再进入下一阶段做 waypoint 分布优化和受控 A/B 覆盖对比。
