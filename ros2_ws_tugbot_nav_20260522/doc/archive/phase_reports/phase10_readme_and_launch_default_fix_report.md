# Phase 10 README 与 Launch 默认入口纠错复检报告

## 1. 人工复检发现的问题

本轮 Phase 10 在原 README 与 Phase 6 静态回放入口复检基础上追加一个人工发现：

1. `tugbot_nav_phase6_map.launch.py` 被怀疑可能误用 Phase 7A 或错误链路，人工观察中出现后半段撞锥桶与地图错位堆叠现象。
2. `README.md` 中探索 quickstart 曾把人工入口写成 `headless:=true use_rviz:=false`，且地图保存示例存在覆盖推荐成果地图的风险。
3. 新增发现：直接运行 `ros2 launch tugbot_bringup tugbot_explore.launch.py` 时，人工怀疑默认探索参数可能被 Phase 7A 激进参数污染。
4. Phase 7A 已由人工判定为“执行过程通过，但地图质量不采纳，不作为默认成果，不继续 Phase 7B”，因此 Phase 7A 参数不得污染默认探索入口。

## 2. 静态检查结果

检查文件：

- `README.md`
- `src/tugbot_bringup/launch/tugbot_explore.launch.py`
- `src/tugbot_bringup/launch/tugbot_nav_phase6_map.launch.py`
- `src/tugbot_bringup/test/test_contract.py`

关键结论：

1. 当前源码中的 `tugbot_explore.launch.py` 默认值已经是 Phase 6 稳健参数：
   - `max_goals = 20`
   - `max_cleanup_goals = 5`
   - `cleanup_search_radius_max_m = 2.0`
   - `cleanup_min_obstacle_distance_m = 0.35`
   - `target_unknown_ratio = 0.03`
2. `tugbot_explore.launch.py` 保留用户通过命令行覆盖参数的能力，例如用户显式输入 `max_cleanup_goals:=12` 仍可生效，但默认不再是 Phase 7A 激进值。
3. `tugbot_nav_phase6_map.launch.py` 引用的是：
   - `tugbot_nav_world_slam_phase6_cleanup.yaml`
   - `nav2_phase6_map_params.yaml`
4. `tugbot_nav_phase6_map.launch.py` 源文件中未发现以下错误引用：
   - `map_1725111373.yaml`
   - `tugbot_nav_world_slam_phase7_budget.yaml`
   - `slam_toolbox`
   - `frontier_explorer`
   - `tugbot_explore.launch.py`
   - `tugbot_slam.launch.py`
   - `tugbot_slam_nav.launch.py`
   - `save_map`
   - `map_save_path`
5. `README.md` 中 Phase 7A 相关内容只保留为“不推荐默认使用/负面实验记录”，不作为快速开始默认命令。

## 3. tugbot_explore.launch.py 是否存在 Phase 7A 默认参数污染

本轮检查结论：当前文件未发现 Phase 7A 激进默认值。

执行参数检查后，`--show-args` 显示：

```text
'max_goals':
    Maximum successful goals before stopping.
    (default: '20')

'cleanup_search_radius_max_m':
    Maximum observation-goal search radius around unknown cluster center.
    (default: '2.0')

'cleanup_min_obstacle_distance_m':
    Minimum cleanup goal clearance from occupied cells.
    (default: '0.35')

'target_unknown_ratio':
    Unknown-cell ratio target before cleanup can stop.
    (default: '0.03')

'max_cleanup_goals':
    Maximum cleanup observation goals.
    (default: '5')
```

因此默认探索入口当前是 Phase 6 稳健配置，不是 Phase 7A 激进配置。

## 4. 已恢复 / 确认的默认参数

`src/tugbot_bringup/launch/tugbot_explore.launch.py` 当前默认参数为：

| 参数 | 当前默认值 | Phase 10 要求 | 结论 |
| --- | ---: | ---: | --- |
| `max_goals` | `20` | `20` | PASS |
| `max_cleanup_goals` | `5` | `5` | PASS |
| `cleanup_search_radius_max_m` | `2.0` | `2.0` | PASS |
| `cleanup_min_obstacle_distance_m` | `0.35` | `0.35` | PASS |
| `target_unknown_ratio` | `0.03` | `0.03` | PASS |

同时 contract test 明确禁止这些参数默认回退为 Phase 7A 激进值：

| 参数 | 禁止默认值 |
| --- | ---: |
| `max_goals` | `25` |
| `max_cleanup_goals` | `12` |
| `cleanup_search_radius_max_m` | `3.0` |
| `cleanup_min_obstacle_distance_m` | `0.30` |
| `target_unknown_ratio` | `0.05` |

## 5. README 修复内容

已修改 `README.md`，明确区分三类命令。

### A. 人工可视化探索命令

README 中推荐人工快速开始使用：

```bash
ros2 launch tugbot_bringup tugbot_explore.launch.py \
  headless:=false \
  use_rviz:=true \
  max_goals:=20 \
  enable_cleanup_mode:=true \
  save_map:=true \
  map_save_path:=/absolute/path/to/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_manual
```

该命令默认打开 Gazebo GUI 与 RViz，适合人工观察。

### B. 自动/headless 验证命令

README 中单独保留自动/headless 验证命令：

```bash
ros2 launch tugbot_bringup tugbot_explore.launch.py \
  headless:=true \
  use_rviz:=false \
  max_goals:=20 \
  enable_cleanup_mode:=true \
  save_map:=true \
  map_save_path:=/absolute/path/to/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_headless_test
```

该命令已明确标注为 CI、本机无人值守或自动/headless 验证用途。

### C. Phase 7A 激进实验参数

README 新增/强化说明：Phase 7A 激进参数只能作为“不推荐默认使用/负面实验记录”，不得写入快速开始默认命令，也不得成为 `tugbot_explore.launch.py` 默认值。

记录的 Phase 7A 参数为：

```bash
max_goals:=25
max_cleanup_goals:=12
cleanup_search_radius_max_m:=3.0
cleanup_min_obstacle_distance_m:=0.30
target_unknown_ratio:=0.05
```

### D. 避免覆盖 Phase 6 推荐成果地图

README 中普通探索保存路径不再使用：

```text
tugbot_nav_world_slam_phase6_cleanup
```

改为新文件名示例：

```text
tugbot_nav_world_slam_manual
tugbot_nav_world_slam_headless_test
```

同时继续说明：Phase 6 cleanup map 是推荐成果地图，不建议普通用户直接覆盖；若重新探索，应保存为新文件名。

## 6. tugbot_nav_phase6_map.launch.py 检查与修复情况

当前 `src/tugbot_bringup/launch/tugbot_nav_phase6_map.launch.py` 满足以下要求：

1. 使用 Phase 6 cleanup map：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
```

2. 不使用旧地图：

```text
map_1725111373.yaml
```

3. 不使用 Phase 7A 地图：

```text
tugbot_nav_world_slam_phase7_budget.yaml
```

4. 不启动探索/SLAM 链路：

```text
slam_toolbox
frontier_explorer
tugbot_explore.launch.py
tugbot_slam.launch.py
tugbot_slam_nav.launch.py
```

5. 通过 Nav2 `bringup_launch.py` 启动静态地图回放链路：

```text
map_server + AMCL + Nav2 navigation stack
```

6. 为减少 Nav2 bringup 默认参数歧义，Phase 6 静态回放 wrapper 显式传入：

```python
'slam': 'False'
'use_localization': 'True'
'use_composition': 'False'
```

说明：`--show-args` 中仍会显示 Nav2 上游 bringup 自带的 `slam_toolbox` 参数说明，这是 `/opt/ros/jazzy/share/nav2_bringup/launch/bringup_launch.py` 的通用 launch argument 描述，不代表本项目 wrapper 源文件启动了 `slam_toolbox`。实际 live ROS graph 已确认 `/slam_toolbox` 不存在。

## 7. 是否新增 nav2_phase6_map_params.yaml

本轮沿用此前 Phase 10 已新增的专用参数文件：

```text
src/tugbot_navigation/config/nav2_phase6_map_params.yaml
```

用途：仅服务 Phase 6 静态地图回放入口，避免修改原始通用：

```text
src/tugbot_navigation/config/nav2_params.yaml
```

当前契约检查确认：

- `robot_radius: 0.35`
- `topic: /scan`
- `collision_monitor:` 存在
- `inflation_radius: 0.50`

## 8. contract test 加强内容

已加强 `src/tugbot_bringup/test/test_contract.py`。

新增/加强内容包括：

1. 新增 `_launch_default(...)` 辅助函数，直接从 launch 源码解析 `DeclareLaunchArgument(..., default_value=...)`。
2. 新增 `test_phase10_explore_defaults_are_phase6_stable_not_phase7a_aggressive()`：
   - 检查 `tugbot_explore.launch.py` 默认值必须为 Phase 6 稳健参数；
   - 检查默认值不得为 Phase 7A 激进参数。
3. 加强 `test_phase8_phase6_static_map_nav_launch_contract()`：
   - 必须引用 `tugbot_nav_world_slam_phase6_cleanup.yaml`；
   - 必须引用 `nav2_phase6_map_params.yaml`；
   - 必须显式传入 `slam=False`、`use_localization=True`、`use_composition=False`；
   - 不得引用 Phase 7A 地图、旧地图、SLAM/探索 launch 或探索保存参数。
4. 加强 `test_phase10_readme_quickstart_and_phase6_replay_contract()`：
   - README 人工可视化命令必须包含 `headless:=false`、`use_rviz:=true`；
   - README headless 命令必须标注为自动/headless 验证，并包含 `headless:=true`、`use_rviz:=false`；
   - README 普通探索 `map_save_path` 不得指向 Phase 6 cleanup 推荐成果地图；
   - README 必须把 Phase 7A 参数标注为不推荐默认使用/负面实验记录；
   - README 快速开始推荐命令不得包含 Phase 7A 激进参数。

## 9. build / py_compile / pytest 结果

执行命令：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
colcon build --symlink-install
source install/setup.bash
python3 -m py_compile \
  src/tugbot_bringup/launch/tugbot_explore.launch.py \
  src/tugbot_bringup/launch/tugbot_nav_phase6_map.launch.py
python3 -m pytest -q src/tugbot_bringup/test/test_contract.py
```

结果：

```text
Summary: 5 packages finished [1.48s]
12 passed in 0.01s
```

`py_compile` 无错误输出，视为通过。

## 10. --show-args 结果

### tugbot_explore.launch.py

执行：

```bash
ros2 launch tugbot_bringup tugbot_explore.launch.py --show-args | sed -n '/max_goals/,+3p;/max_cleanup_goals/,+3p;/cleanup_search_radius_max_m/,+3p;/cleanup_min_obstacle_distance_m/,+3p;/target_unknown_ratio/,+3p'
```

结果：

```text
'max_goals':
    Maximum successful goals before stopping.
    (default: '20')

'cleanup_search_radius_max_m':
    Maximum observation-goal search radius around unknown cluster center.
    (default: '2.0')

'cleanup_min_obstacle_distance_m':
    Minimum cleanup goal clearance from occupied cells.
    (default: '0.35')

'target_unknown_ratio':
    Unknown-cell ratio target before cleanup can stop.
    (default: '0.03')

'max_cleanup_goals':
    Maximum cleanup observation goals.
    (default: '5')
```

### tugbot_nav_phase6_map.launch.py

执行：

```bash
ros2 launch tugbot_bringup tugbot_nav_phase6_map.launch.py --show-args | grep -E "phase6|phase7|map_1725111373|slam_toolbox|frontier|cleanup|save_map" || true
```

结果摘录：

```text
(default: '/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/install/tugbot_navigation/share/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml')
(default: '/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/install/tugbot_navigation/share/tugbot_navigation/config/nav2_phase6_map_params.yaml')
Full path to the ROS2 parameters file to use for the slam_toolbox node
(default: '/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_sync.yaml')
```

解释：前两行是本项目 Phase 6 静态回放入口的正确默认 map 与 params。后两行来自 Nav2 upstream `bringup_launch.py` 的通用 SLAM launch argument 描述；wrapper 已显式传入 `slam=False`，live ROS graph 也确认未启动 `/slam_toolbox`。

## 11. Phase 6 静态回放短时间 live 检查结果

执行：

```bash
ros2 launch tugbot_bringup tugbot_nav_phase6_map.launch.py headless:=true use_rviz:=false
```

短时间启动后检查：

```bash
ros2 node list | grep -E "map_server|amcl|slam_toolbox|frontier_explorer|lifecycle_manager"
ros2 topic info -v /map
ros2 param get /map_server yaml_filename
```

结果：

```text
--- nodes ---
/amcl
/lifecycle_manager_localization
/lifecycle_manager_navigation
/map_server
```

确认：

- `/map_server` 存在。
- `/amcl` 存在。
- `/slam_toolbox` 不存在。
- `/frontier_explorer` 不存在。
- lifecycle managers 存在。

`/map` topic 检查：

```text
Type: nav_msgs/msg/OccupancyGrid
Publisher count: 1

Node name: map_server
Node namespace: /
Endpoint type: PUBLISHER
```

确认 `/map` publisher 为 `/map_server`。

`/map_server yaml_filename` 检查：

```text
String value is: /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/install/tugbot_navigation/share/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
```

确认 map_server 使用 Phase 6 cleanup map。

验证结束后已停止后台 launch，并轮询 ROS graph 至：

```text
CLEAN
```

当前无 `/map_server`、`/amcl`、`/slam_toolbox`、`/frontier_explorer`、`lifecycle_manager` 残留目标节点。

## 12. 是否确认 /map publisher 为 /map_server

已确认。

`ros2 topic info -v /map` 显示：

```text
Publisher count: 1
Node name: map_server
Endpoint type: PUBLISHER
```

因此 `/map` 发布者是 `/map_server`，不是 `slam_toolbox`。

## 13. 是否确认 slam_toolbox / frontier_explorer 未启动

已确认。

`ros2 node list | grep -E "map_server|amcl|slam_toolbox|frontier_explorer|lifecycle_manager"` 仅返回：

```text
/amcl
/lifecycle_manager_localization
/lifecycle_manager_navigation
/map_server
```

没有 `/slam_toolbox`，没有 `/frontier_explorer`。

## 14. 当前遗留问题

1. 本轮只执行短时间 launch / ROS graph / 参数验证，没有执行长时间探索实验，也没有复现人工观察中的撞锥桶全过程。
2. 如果人工后续仍观察到 Phase 6 静态回放撞锥桶，应继续按 Nav2 runtime 证据区分：地图几何、AMCL 初始位姿、TF/odom、global/local costmap、footprint/robot_radius、inflation、controller 行为或 Gazebo 碰撞体问题；不应在没有 `/slam_toolbox` 节点证据时误判为 Phase 7A/SLAM 链路污染。
3. `--show-args` 中出现的 `slam_toolbox` 文案来自 Nav2 upstream 通用 bringup 参数说明，不代表本项目 wrapper 默认启动了 SLAM。实际 live graph 是权威证据。

## 15. Phase 10 验收结论

Phase 10 扩展范围验收结论：`PASS_FOR_MANUAL_ACCEPTANCE`。

理由：

1. README 已区分人工可视化探索、自动/headless 验证和 Phase 7A 不推荐实验参数。
2. README 普通探索保存路径不再默认覆盖 Phase 6 cleanup 推荐成果地图。
3. `tugbot_explore.launch.py` 默认参数已确认是 Phase 6 稳健值，不是 Phase 7A 激进值。
4. `tugbot_nav_phase6_map.launch.py` 使用 Phase 6 cleanup map，并显式 `slam=False`、`use_localization=True`。
5. contract test 已覆盖 Phase 7A 默认参数污染防回归、README quickstart 防误导、Phase 6 静态回放链路防污染。
6. build、py_compile、pytest 均通过。
7. 短时间 live 检查确认 `/map` 由 `/map_server` 发布，`/map_server yaml_filename` 指向 Phase 6 cleanup map，且 `/slam_toolbox`、`/frontier_explorer` 未启动。

完成后已停止 live 进程，等待人工验收。
