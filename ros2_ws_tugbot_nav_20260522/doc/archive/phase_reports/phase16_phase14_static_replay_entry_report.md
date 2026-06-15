# Phase 16: Phase 14 推荐地图静态导航回放入口报告

## 验收结论

`PASS_FOR_MANUAL_ACCEPTANCE`

Phase 16 已新增独立的 Phase 14 推荐地图静态导航回放入口：

```bash
ros2 launch tugbot_bringup tugbot_nav_phase14_map.launch.py \
  headless:=false \
  use_rviz:=true
```

该入口默认加载当前推荐稳定地图：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.pgm
```

本阶段没有修改 Phase 14 推荐地图文件本身，没有修改 Phase 6 cleanup map，没有删除 Phase 7A 地图，没有修改原 `nav2_params.yaml`，没有重新生成地图，也没有运行长时间探索实验。

---

## 1. 本阶段目标

Phase 15 后 README 已明确当前推荐地图是 Phase 14 perimeter no-spin map，但原有静态回放入口：

```bash
ros2 launch tugbot_bringup tugbot_nav_phase6_map.launch.py
```

仍然加载 Phase 6 cleanup map，不加载 Phase 14 推荐地图。

因此 Phase 16 的目标是：

1. 新增一个语义清晰的 Phase 14 推荐地图静态导航回放入口。
2. 保留 Phase 6 静态回放入口作为上一版稳定地图回放入口。
3. 更新 README 的“启动入口总览”和“快速开始”。
4. 加强 contract test，防止 Phase 14 静态回放入口误引用旧地图或误启动 SLAM/探索链路。

---

## 2. 新增与修改文件

### 新增

```text
src/tugbot_bringup/launch/tugbot_nav_phase14_map.launch.py
src/tugbot_navigation/config/nav2_phase14_map_params.yaml
doc/doc_report/phase16_phase14_static_replay_entry_report.md
```

### 修改

```text
README.md
src/tugbot_bringup/test/test_contract.py
```

### 未修改

```text
src/tugbot_navigation/config/nav2_params.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.pgm
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.pgm
```

---

## 3. `tugbot_nav_phase14_map.launch.py` 说明

新增入口：

```text
src/tugbot_bringup/launch/tugbot_nav_phase14_map.launch.py
```

结构参考 `tugbot_nav_phase6_map.launch.py`，但默认 map 改为 Phase 14 推荐地图：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml
```

该 launch 支持以下参数：

```text
world_sdf
headless
use_rviz
use_sim_time
autostart
map
params_file
rviz_config
```

其中用户要求的关键参数均已支持：

```text
headless
use_rviz
use_sim_time
autostart
map
params_file
```

Nav2 bringup 明确传入：

```python
'use_composition': 'False'
'slam': 'False'
'use_localization': 'True'
```

### 默认启动链路

该入口启动：

- Gazebo world：`tugbot_nav_world.sdf`
- `ros_gz_bridge`
- scan static TF：`scan_omni_static_tf`
- camera static TF（由 Gazebo launch 保持提供）
- `map_server`
- `AMCL`
- Nav2 navigation stack
- 可选 RViz

### 明确不启动

该入口不启动：

- `slam_toolbox`
- `frontier_explorer`
- `tugbot_explore.launch.py`
- `tugbot_slam.launch.py`
- `tugbot_slam_nav.launch.py`
- 地图保存逻辑
- cleanup 探索逻辑

### 禁止误引用旧地图

contract test 已保护该 launch 不引用：

```text
map_1725111373.yaml
tugbot_nav_world_slam_phase6_cleanup.yaml
tugbot_nav_world_slam_phase7_budget.yaml
```

---

## 4. Nav2 参数文件选择

本阶段新增：

```text
src/tugbot_navigation/config/nav2_phase14_map_params.yaml
```

选择新增文件而不是直接复用 `nav2_phase6_map_params.yaml` 的理由：

1. 开源语义更清晰，避免 `tugbot_nav_phase14_map.launch.py` 使用 `phase6` 命名参数造成误解。
2. Phase 14 静态回放入口应在文件名层面表达“当前推荐地图回放”。
3. 当前内容从 `nav2_phase6_map_params.yaml` 复制而来，保持静态地图回放安全参数一致。
4. 该参数文件不绑定具体地图内容；地图由 launch 的 `map` 参数传入。
5. 未修改原 `nav2_params.yaml`。

`nav2_phase14_map_params.yaml` 保留了静态地图回放所需的 AMCL、map_server 由 launch map 参数配置、Nav2 controller/planner/costmap/collision_monitor 等配置。

---

## 5. README 更新

README 已更新必要章节。

### 启动入口总览

新增行：

```text
`tugbot_nav_phase14_map.launch.py`
```

对应语义：

- 推荐程度：当前推荐静态回放入口
- 主要用途：Phase 14 推荐地图静态导航回放
- 地图来源：Phase 14 perimeter no-spin map
- 定位方式：AMCL
- 是否启动 slam_toolbox：否
- 备注：加载当前推荐地图，适合验证最终推荐地图的静态导航能力

同时保留并澄清：

- `tugbot_nav_phase6_map.launch.py` 仍加载 Phase 6 cleanup map。
- `tugbot_nav_phase14_map.launch.py` 用于加载当前推荐 Phase 14 map。

### 快速开始

新增小节：

```text
### E. 使用当前推荐稳定地图做静态导航回放
```

命令：

```bash
ros2 launch tugbot_bringup tugbot_nav_phase14_map.launch.py \
  headless:=false \
  use_rviz:=true
```

README 已说明：

- 该命令加载当前推荐地图：`src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml`
- 启动 `map_server + AMCL + Nav2`
- 不启动 `slam_toolbox`
- 不启动 `frontier_explorer`
- 不进行重新建图
- RViz 启动后建议先使用 `2D Pose Estimate` 设置初始位姿
- 然后用 `2D Goal Pose` 或 `NavigateToPose` 发送目标点
- 该入口用于“推荐稳定地图回放导航”，不是未知地图探索

原 Phase 6 小节保留并调整为：

```text
### F. 使用 Phase 6 上一版稳定地图做静态导航回放
```

Phase 7A 负面实验记录顺延为 G 小节。

---

## 6. Contract test 加强

修改文件：

```text
src/tugbot_bringup/test/test_contract.py
```

新增/加强检查内容：

1. `tugbot_nav_phase14_map.launch.py` 必须存在。
2. 必须引用：

```text
tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml
```

3. 不得引用：

```text
map_1725111373.yaml
tugbot_nav_world_slam_phase6_cleanup.yaml
tugbot_nav_world_slam_phase7_budget.yaml
```

4. 不得包含：

```text
slam_toolbox
frontier_explorer
tugbot_explore.launch.py
tugbot_slam.launch.py
tugbot_slam_nav.launch.py
save_map
map_save_path
cleanup
```

5. 必须包含或传入：

```text
'use_composition': 'False'
'slam': 'False'
'use_localization': 'True'
```

6. README 必须包含：

```text
ros2 launch tugbot_bringup tugbot_nav_phase14_map.launch.py
当前推荐静态回放入口
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml
不启动 slam_toolbox
不启动 frontier_explorer
```

7. 保持已有契约：

- Phase 6 静态回放入口仍加载 Phase 6 cleanup map。
- `tugbot_explore.launch.py` 默认仍是 Phase 6 稳健参数。
- Phase 14 推荐地图仍是当前推荐成果。
- Phase 7A 仍为负面实验记录。
- camera / box pillar / no-spin 说明仍保留。

---

## 7. TDD 过程

先补 Phase 16 contract test，再运行定向测试确认 RED。

RED 结果：

```text
pytest -q src/tugbot_bringup/test/test_contract.py -k 'phase16'

2 failed, 15 deselected in 0.11s
```

失败原因符合预期：

1. `tugbot_nav_phase14_map.launch.py` 尚不存在。
2. README 尚未包含 Phase 14 静态回放入口说明。

随后完成最小实现，再运行 GREEN。

GREEN 结果：

```text
pytest -q src/tugbot_bringup/test/test_contract.py -k 'phase16'

2 passed, 15 deselected in 0.01s
```

完整 contract test 结果：

```text
pytest -q src/tugbot_bringup/test/test_contract.py

17 passed in 0.02s
```

---

## 8. py_compile / build 结果

py_compile 通过：

```bash
python3 -m py_compile \
  src/tugbot_bringup/launch/tugbot_nav_phase14_map.launch.py \
  src/tugbot_bringup/test/test_contract.py
```

结果：通过，无输出错误。

colcon build 通过：

```bash
colcon build --symlink-install
```

最终结果：

```text
Summary: 5 packages finished [1.46s]
```

注：首次 Phase 16 build 曾通过并显示 `Summary: 5 packages finished [1.55s]`；补强 `async_slam_toolbox_node` 静态契约后重新 build，最终 authoritative 结果为 `[1.46s]`。

---

## 9. show-args 结果

执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tugbot_bringup tugbot_nav_phase14_map.launch.py --show-args
```

关键默认值：

```text
map:
  /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/install/tugbot_navigation/share/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml

params_file:
  /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/install/tugbot_navigation/share/tugbot_navigation/config/nav2_phase14_map_params.yaml

use_sim_time: true
autostart: true
headless: true
use_rviz: false
slam: False
use_localization: True
use_composition: False
```

结论：Phase 14 静态回放入口默认 map 与 params_file 指向正确。

---

## 10. 短时间 headless launch / ROS graph 验证

执行短时间验证命令：

```bash
ros2 launch tugbot_bringup tugbot_nav_phase14_map.launch.py \
  headless:=true \
  use_rviz:=false
```

验证期间没有启动 RViz，没有进行长时间探索，没有保存地图。

### map_server 加载证据

launch 日志显示 `map_server` 实际加载 Phase 14 推荐地图：

```text
[map_io]: Loading yaml file: .../maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml
[map_io]: Loading image_file: .../maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.pgm
[map_io]: Read map ...tugbot_nav_world_slam_phase14_perimeter_no_spin.pgm: 315 X 156 map @ 0.05 m/cell
```

### ROS graph 证据

`ros2 node list` 观察到关键节点：

```text
/map_server
/amcl
/controller_server
/planner_server
/bt_navigator
/behavior_server
/velocity_smoother
/collision_monitor
/waypoint_follower
/ros_gz_bridge
/scan_omni_static_tf
/camera_link_static_tf
/camera_optical_frame_static_tf
```

### /map publisher

`ros2 topic info -v /map` 显示：

```text
Publisher count: 1
Node name: map_server
Topic type: nav_msgs/msg/OccupancyGrid
Durability: TRANSIENT_LOCAL
```

订阅者包括：

```text
/global_costmap/global_costmap
/amcl
```

结论：`/map` publisher 为 `/map_server`，符合 Phase 14 静态地图回放要求。

### lifecycle

`ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger '{}'` 返回：

```text
success=True
```

### 禁止节点检查

验证命令：

```bash
ros2 node list | grep -E 'slam_toolbox|frontier_explorer' || true
```

结果为空。

结论：短时间 headless 验证未观察到 `slam_toolbox` 或 `frontier_explorer` 节点。

验证结束后已停止 launch 相关进程，并再次检查 ROS graph 为空。

---

## 11. 当前遗留与边界说明

1. 本阶段只验证了静态回放入口可以启动、map_server 加载 Phase 14 map、AMCL/Nav2 lifecycle active、`/map` 由 `/map_server` 发布。
2. 本阶段没有进行 RViz 人工导航目标点验收。
3. 本阶段没有执行长时间探索实验。
4. 本阶段没有重新生成地图。
5. 本阶段没有保存新地图。
6. Phase 14 推荐地图质量结论仍沿用 Phase 14/15 的人工验收结论；Phase 16 只新增静态回放入口。

---

## 12. Phase 16 验收结论

`PASS_FOR_MANUAL_ACCEPTANCE`

理由：

1. 新增 `tugbot_nav_phase14_map.launch.py`。
2. 默认 map 指向 Phase 14 perimeter no-spin 推荐地图。
3. 新增 `nav2_phase14_map_params.yaml`，避免 Phase 14 launch 使用 phase6 命名参数导致开源语义混乱。
4. `tugbot_nav_phase14_map.launch.py` 不引用原 0513 map、Phase 6 cleanup map 或 Phase 7A budget map。
5. 不启动 `slam_toolbox`、`frontier_explorer` 或探索 launch。
6. 明确传入 `slam:=False`、`use_localization:=True`、`use_composition:=False`。
7. 短时间 launch 验证显示 `/map` publisher 为 `/map_server`。
8. `map_server` 实际加载 Phase 14 推荐 YAML/PGM。
9. AMCL / Nav2 navigation lifecycle active。
10. contract test、py_compile、colcon build 均通过。
11. README 已补充当前推荐稳定地图静态导航回放命令和说明。

等待人工验收。
