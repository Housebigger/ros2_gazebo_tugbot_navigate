# Phase 10 README 与 Phase 6 静态回放入口纠错复检报告

## 1. 人工复检发现的问题

人工复检提出两个问题：

1. 运行 `ros2 launch tugbot_bringup tugbot_nav_phase6_map.launch.py` 时，观察到小车后半段疑似撞到锥形雪糕筒，并出现类似地图错位堆叠的视觉现象。需要确认 Phase 6 静态地图回放入口是否错误混入 Phase 7A 地图、探索链路、SLAM 或 cleanup 参数。
2. README 的“启动自主探索并保存地图”命令使用 `headless:=true use_rviz:=false`，更适合自动验证，不适合作为开源 README 的人工快速开始主命令；且 README 不应默认把新探索结果保存到 Phase 6 推荐成果地图路径，避免覆盖推荐地图。

## 2. 修改边界

本轮遵守边界：

- 未修改 0513 工程。
- 未修改 Phase 6 推荐地图文件本身：
  - `src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml`
  - `src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm`
- 未删除 Phase 7A 地图。
- 未运行长时间探索实验。
- 未修改原 `src/tugbot_navigation/config/nav2_params.yaml`。

本轮修改文件：

```text
README.md
src/tugbot_bringup/launch/tugbot_nav_phase6_map.launch.py
src/tugbot_bringup/test/test_contract.py
src/tugbot_navigation/config/nav2_phase6_map_params.yaml
```

## 3. 静态检查结果

检查目标文件：

```text
src/tugbot_bringup/launch/tugbot_nav_phase6_map.launch.py
README.md
src/tugbot_bringup/test/test_contract.py
```

重点搜索项包括：

```text
tugbot_nav_world_slam_phase7_budget
phase7
tugbot_explore.launch.py
tugbot_slam_nav.launch.py
tugbot_slam.launch.py
slam_toolbox
frontier_explorer
save_map
cleanup
map_save_path
map_1725111373.yaml
```

### 3.1 Phase 6 静态回放 launch

`src/tugbot_bringup/launch/tugbot_nav_phase6_map.launch.py` 当前静态检查结论：

| 检查项 | 结果 |
| --- | --- |
| 引用 `tugbot_nav_world_slam_phase6_cleanup.yaml` | PASS |
| 引用 `map_1725111373.yaml` | PASS，未引用 |
| 引用 `tugbot_nav_world_slam_phase7_budget` | PASS，未引用 |
| 包含 `phase7` | PASS，未包含 |
| 包含 `tugbot_explore.launch.py` | PASS，未包含 |
| 包含 `tugbot_slam_nav.launch.py` | PASS，未包含 |
| 包含 `tugbot_slam.launch.py` | PASS，未包含 |
| 包含 `slam_toolbox` | PASS，未包含 |
| 包含 `frontier_explorer` | PASS，未包含 |
| 包含 `save_map` / `map_save_path` | PASS，未包含 |
| 包含探索 cleanup 参数 | PASS，未包含探索/cleanup 行为参数；仅地图文件名本身含 `phase6_cleanup` |

静态结论：

```text
tugbot_nav_phase6_map.launch.py 没有发现 Phase 7A 地图、Phase 7A 参数、SLAM、frontier_explorer 或探索保存链路的错误引用。
```

### 3.2 README

README 原问题确认：

- 原快速开始探索命令默认使用 `headless:=true use_rviz:=false`，不适合作为人工可视化快速开始主命令。
- 原快速开始探索命令把 `map_save_path` 指向 Phase 6 推荐地图路径，存在覆盖推荐成果地图的风险。

README 当前修复结果：

- 新增/调整为“人工可视化探索并保存新地图”：
  - `headless:=false`
  - `use_rviz:=true`
  - `map_save_path:=.../tugbot_nav_world_slam_manual`
- 增加“自动/headless 验证探索命令”：
  - `headless:=true`
  - `use_rviz:=false`
  - `map_save_path:=.../tugbot_nav_world_slam_new`
- 明确说明：
  - Phase 6 cleanup map 是推荐成果地图；
  - 不建议普通用户直接覆盖；
  - 若要重新探索，请保存为新文件名。
- README 仍保留 Phase 7A 作为负面实验记录说明，但不作为推荐地图。

### 3.3 contract test

`src/tugbot_bringup/test/test_contract.py` 已加强：

1. 检查 `tugbot_nav_phase6_map.launch.py` 必须引用 `tugbot_nav_world_slam_phase6_cleanup.yaml`。
2. 检查不得引用 `tugbot_nav_world_slam_phase7_budget`。
3. 检查不得包含 `slam_toolbox`。
4. 检查不得包含 `frontier_explorer`。
5. 检查不得包含 `save_map`、`map_save_path` 与探索 cleanup 参数。
6. 检查 README 的人工可视化命令包含 `headless:=false use_rviz:=true`。
7. 检查 README 的 `headless:=true use_rviz:=false` 被标注为自动/headless 验证。
8. 检查 README 不把 `phase6_cleanup` 作为默认探索保存路径。
9. 检查 Phase 6 回放入口使用专用 `nav2_phase6_map_params.yaml`。
10. 检查专用参数仍包含 `/scan` obstacle 输入、`robot_radius: 0.35`、`collision_monitor`，并将全局 costmap inflation 调整为 `0.50`。

## 4. 是否发现 Phase 7A 错误引用

结论：未发现。

具体结论：

```text
tugbot_nav_phase6_map.launch.py 默认 map 指向：
install/tugbot_navigation/share/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml

未发现：
- map_1725111373.yaml
- tugbot_nav_world_slam_phase7_budget.yaml
- slam_toolbox
- frontier_explorer
- tugbot_explore.launch.py
- tugbot_slam_nav.launch.py
- tugbot_slam.launch.py
- save_map
- map_save_path
```

因此，人工观察到的“地图错位堆叠”不应优先归因于 Phase 7A 地图或 slam_toolbox 被错误启动；本轮 live 验证也确认 `/map` 由 `/map_server` 单独发布。

## 5. tugbot_nav_phase6_map.launch.py 修复情况

虽然入口本身未发现 Phase 7A/SLAM/探索链路错误引用，但考虑到人工观察存在撞锥桶风险，本轮做了最小范围安全修正：

- 保持默认地图为 Phase 6 cleanup map：
  - `tugbot_nav_world_slam_phase6_cleanup.yaml`
- 保持通过 `nav2_bringup/bringup_launch.py` 启动：
  - `map_server`
  - `amcl`
  - Nav2 navigation stack
- 未启动：
  - `slam_toolbox`
  - `frontier_explorer`
- 将默认 Nav2 参数文件从原通用参数改为 Phase 6 回放专用参数：
  - 原：`nav2_params.yaml`
  - 新：`nav2_phase6_map_params.yaml`
- 显式传入：
  - `use_composition: False`

显式关闭 composition 的原因：便于 live 复检时在 `ros2 node list` 中直接看到 `/map_server`、`/amcl`、`/lifecycle_manager_localization` 和 `/lifecycle_manager_navigation` 等独立节点，避免 composed bringup 造成节点可见性歧义。

## 6. 是否新增 nav2_phase6_map_params.yaml

结论：已新增。

新增文件：

```text
src/tugbot_navigation/config/nav2_phase6_map_params.yaml
```

来源与改动：

- 从 `src/tugbot_navigation/config/nav2_params.yaml` 复制。
- 不修改原 `nav2_params.yaml`。
- 保留：
  - `robot_radius: 0.35`
  - local/global costmap 对 `/scan` 的订阅
  - `collision_monitor`
- 针对 Phase 6 静态回放做更保守的障碍物安全距离调整：
  - `global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius` 从 `0.35` 提高到 `0.50`
- local costmap 原本已有：
  - `inflation_radius: 0.70`

解释：

```text
撞锥桶风险更像是静态导航控制/局部障碍物安全裕度问题，而不是 Phase 7A 参数或 SLAM 错误链路问题。因此本轮选择新增 Phase 6 回放专用 Nav2 参数文件，避免影响原 tugbot_nav.launch.py 与通用 nav2_params.yaml。
```

## 7. README 修复情况

README 快速开始现已区分两类命令。

### 7.1 人工可视化探索命令

当前 README 使用：

```bash
ros2 launch tugbot_bringup tugbot_explore.launch.py \
  headless:=false \
  use_rviz:=true \
  max_goals:=20 \
  enable_cleanup_mode:=true \
  save_map:=true \
  map_save_path:=/absolute/path/to/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_manual
```

该命令默认打开 Gazebo GUI 和 RViz，适合作为人工快速开始主命令。

### 7.2 自动/headless 验证命令

当前 README 使用：

```bash
ros2 launch tugbot_bringup tugbot_explore.launch.py \
  headless:=true \
  use_rviz:=false \
  max_goals:=20 \
  enable_cleanup_mode:=true \
  save_map:=true \
  map_save_path:=/absolute/path/to/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_new
```

该命令明确标注为自动/headless 验证，不作为人工主命令。

### 7.3 避免覆盖 Phase 6 推荐地图

当前 README 中 `map_save_path:=...` 不再指向：

```text
tugbot_nav_world_slam_phase6_cleanup
```

README 明确说明 Phase 6 cleanup map 是推荐成果地图，不建议普通用户直接覆盖；若要重新探索，请保存为新文件名。

## 8. 验证命令与结果

### 8.1 build

命令：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
colcon build --symlink-install
```

结果：

```text
Summary: 5 packages finished [1.56s]
```

### 8.2 focused pytest

命令：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest -q src/tugbot_bringup/test/test_contract.py
```

结果：

```text
11 passed in 0.01s
```

### 8.3 colcon test

命令：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select tugbot_bringup --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/tugbot_bringup
```

结果：

```text
Summary: 1 package finished [0.15s]
Summary: 0 tests, 0 errors, 0 failures, 0 skipped
```

说明：该包的 colcon/ament 测试清单当前未自动收集 `src/tugbot_bringup/test/test_contract.py`，因此本轮额外以 `python3 -m pytest` 显式运行 contract test，并得到 11 passed。

## 9. short live 验证结果

启动命令：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source install/setup.bash
ros2 launch tugbot_bringup tugbot_nav_phase6_map.launch.py headless:=true use_rviz:=false
```

### 9.1 节点检查

命令：

```bash
ros2 node list | grep -E "map_server|amcl|slam_toolbox|frontier_explorer|lifecycle_manager"
```

结果：

```text
/amcl
/lifecycle_manager_localization
/lifecycle_manager_navigation
/map_server
```

结论：

- `/map_server` 存在。
- `/amcl` 存在。
- `/slam_toolbox` 不存在。
- `/frontier_explorer` 不存在。
- localization/navigation lifecycle manager 存在。

### 9.2 /map publisher 检查

命令：

```bash
ros2 topic info -v /map
```

关键结果：

```text
Publisher count: 1

Node name: map_server
Node namespace: /
Topic type: nav_msgs/msg/OccupancyGrid
Endpoint type: PUBLISHER
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (1)
  Durability: TRANSIENT_LOCAL
```

结论：

```text
/map publisher 是 /map_server。
```

### 9.3 map_server yaml_filename 检查

命令：

```bash
ros2 param get /map_server yaml_filename
```

结果：

```text
String value is: /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/install/tugbot_navigation/share/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
```

结论：

```text
map_server 加载的是 Phase 6 cleanup map，不是 Phase 7A 地图，也不是旧 map_1725111373.yaml。
```

### 9.4 AMCL/Nav2 lifecycle 检查

命令：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 service call /lifecycle_manager_localization/is_active std_srvs/srv/Trigger "{}"
ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger "{}"
```

结果：

```text
/map_server: active [3]
/amcl: active [3]
/lifecycle_manager_localization/is_active: success=True
/lifecycle_manager_navigation/is_active: success=True
```

结论：

```text
localization lifecycle active；navigation lifecycle active。
```

### 9.5 Phase 6 专用 Nav2 安全参数检查

命令：

```bash
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param get /global_costmap/global_costmap obstacle_layer.scan.topic
ros2 param get /collision_monitor scan.topic
```

结果：

```text
Double value is: 0.5
String value is: /scan
String value is: scan
```

结论：

- Phase 6 回放专用参数已生效。
- global costmap inflation radius 当前为 `0.50`。
- global obstacle layer 订阅 `/scan`。
- collision monitor 的 scan 输入仍启用。

## 10. 进程清理

short live 验证完成后已停止 launch 进程，并检查目标节点清空。

清理检查结果：

```text
CLEAN
```

## 11. 当前遗留问题

1. 本轮只做短时间 launch 级验证，未运行长时间探索实验，也未执行人工 RViz goal 驾驶路线复现。
2. 撞锥桶风险已通过 Phase 6 专用 Nav2 参数提高 global costmap inflation radius 做最小安全裕度增强，但是否完全消除仍需人工可视化回放验收。
3. 如果人工继续观察到“地图堆叠”视觉现象，本轮证据表明应优先排查 RViz 显示层叠、AMCL 初始位姿、TF/odom 漂移或 costmap/obstacle 可视化，而不是 Phase 7A 地图或 slam_toolbox 错误启动。
4. `colcon test` 当前未自动收集 Python contract test；本轮以显式 `python3 -m pytest` 完成验证。若后续需要 CI 化，可另行把该测试接入 ament/pytest 配置。

## 12. Phase 10 验收结论

Phase 10 当前结论：PASS_FOR_MANUAL_ACCEPTANCE。

验收摘要：

| 验收项 | 结果 |
| --- | --- |
| Phase 6 回放入口只使用 Phase 6 cleanup map | PASS |
| 未发现 Phase 7A 地图/参数错误引用 | PASS |
| 未启动 slam_toolbox | PASS |
| 未启动 frontier_explorer | PASS |
| `/map` publisher 是 `/map_server` | PASS |
| `/map_server yaml_filename` 指向 Phase 6 cleanup map | PASS |
| `/map_server` lifecycle active | PASS |
| `/amcl` lifecycle active | PASS |
| localization/navigation lifecycle active | PASS |
| README 人工命令默认打开 Gazebo GUI + RViz | PASS |
| README 自动/headless 命令单独标注 | PASS |
| README 不默认覆盖 Phase 6 推荐地图 | PASS |
| 新增 Phase 6 专用 Nav2 参数文件 | PASS |
| 原 `nav2_params.yaml` 未修改 | PASS |
| live 进程已停止并清理 | PASS |

最终结论：

```text
tugbot_nav_phase6_map.launch.py 没有混入 Phase 7A、SLAM 或 frontier 探索链路。入口现已使用 Phase 6 cleanup map + map_server + AMCL + Nav2 navigation stack，并改用 Phase 6 回放专用 Nav2 参数文件提升障碍物安全裕度。README 已修正为人工可视化命令默认打开 Gazebo GUI 与 RViz，并避免默认覆盖 Phase 6 推荐成果地图。

Phase 10 已完成，等待人工验收。
```
