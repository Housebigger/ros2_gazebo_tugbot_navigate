# Phase 8 Phase 6 自建地图静态导航回放验证报告

## 1. 结论摘要

Phase 8 已完成。使用 Phase 6 cleanup map 作为静态地图，启动 Gazebo + ros_gz_bridge + scan static TF + Nav2 bringup，在 map_server + AMCL + Nav2 navigation stack 模式下完成静态地图导航回放验证。

最终验收结论：PASS。

关键结论：

```text
使用地图：Phase 6 cleanup map
map_server：已启动并加载 Phase 6 yaml/pgm
AMCL：已启动并可基于 Phase 6 map 发布 map -> base_link 定位
Nav2 lifecycle：localization 与 navigation 均 active
/map publisher：/map_server
NavigateToPose：Goal A、Goal B 均 SUCCEEDED
/cmd_vel：导航过程中有非零速度输出
/odom：导航过程中有明显位移
旧 map_1725111373.yaml：未使用
Phase 7A 地图：未使用
frontier 探索链路：未启动
```

## 2. 新增/修改文件清单

新增文件：

```text
src/tugbot_bringup/launch/tugbot_nav_phase6_map.launch.py
```

修改文件：

```text
src/tugbot_bringup/test/test_contract.py
```

修改说明：

1. 新增 `tugbot_nav_phase6_map.launch.py`，作为 Phase 8 专用静态地图回放导航入口。
2. 扩展 `test_contract.py`，增加 Phase 8 launch contract，确保：
   - launch 使用 `tugbot_nav_world.sdf`；
   - 使用 `tugbot_nav_world_slam_phase6_cleanup.yaml`；
   - 使用现有 `nav2_params.yaml`；
   - 通过 `nav2_bringup/bringup_launch.py` 启动 map_server、AMCL 和 Nav2 navigation stack；
   - 保留 `headless`、`use_rviz`、`use_sim_time`、`autostart` 参数；
   - 不引用 `map_1725111373.yaml`；
   - 不引用 Phase 7A `tugbot_nav_world_slam_phase7_budget`；
   - 不启动 `slam_toolbox` 或在线 SLAM launch；
   - 不把 Phase 6 map 写入原 `tugbot_nav.launch.py`、`tugbot_slam.launch.py`、`tugbot_slam_nav.launch.py`、`tugbot_explore.launch.py`。

未修改：

```text
src/tugbot_bringup/launch/tugbot_nav.launch.py
src/tugbot_bringup/launch/tugbot_slam.launch.py
src/tugbot_bringup/launch/tugbot_slam_nav.launch.py
src/tugbot_bringup/launch/tugbot_explore.launch.py
src/tugbot_navigation/config/nav2_params.yaml
0513 工程
```

## 3. 使用的地图文件路径

源地图路径：

```text
/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
```

launch 运行时由 colcon install/share 解析到：

```text
/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/install/tugbot_navigation/share/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/install/tugbot_navigation/share/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
```

YAML 内容：

```yaml
image: tugbot_nav_world_slam_phase6_cleanup.pgm
mode: trinary
resolution: 0.050
origin: [-8.900, -5.977, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

PGM 文件状态：

```text
尺寸：322 x 170
文件大小：54755 bytes
```

## 4. 是否确认未使用旧地图和 Phase 7A 地图

确认未使用旧地图：

```text
map_1725111373.yaml 未出现在 tugbot_nav_phase6_map.launch.py 中。
map_server 日志中加载的是 tugbot_nav_world_slam_phase6_cleanup.yaml。
```

确认未使用 Phase 7A 地图：

```text
tugbot_nav_world_slam_phase7_budget 未出现在 tugbot_nav_phase6_map.launch.py 中。
map_server 日志中加载的是 tugbot_nav_world_slam_phase6_cleanup.yaml，而不是 phase7_budget。
```

map_server 日志证据：

```text
[map_io]: Loading yaml file: .../maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
[map_io]: Loading image_file: .../maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
[map_io]: Read map .../tugbot_nav_world_slam_phase6_cleanup.pgm: 322 X 170 map @ 0.05 m/cell
```

## 5. Phase 8 launch 设计

新增 launch：

```text
src/tugbot_bringup/launch/tugbot_nav_phase6_map.launch.py
```

其职责：

1. 启动 `tugbot_gazebo.launch.py`，并指定 `tugbot_nav_world.sdf`；
2. 继承 `tugbot_gazebo.launch.py` 内的 Gazebo、`ros_gz_bridge` 与 `scan_omni_static_tf`；
3. 通过 `nav2_bringup/bringup_launch.py` 启动：
   - `map_server`
   - `amcl`
   - controller/planner/behavior/bt_navigator 等 Nav2 navigation stack；
4. 使用现有 `src/tugbot_navigation/config/nav2_params.yaml`；
5. 默认地图为 Phase 6 cleanup map；
6. 保留参数：
   - `world_sdf`
   - `map`
   - `params_file`
   - `rviz_config`
   - `use_sim_time`
   - `autostart`
   - `headless`
   - `use_rviz`

启动命令：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source install/setup.bash
ros2 launch tugbot_bringup tugbot_nav_phase6_map.launch.py headless:=true use_rviz:=false
```

## 6. 构建与静态检查结果

执行命令：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
python3 -m py_compile src/tugbot_bringup/launch/tugbot_nav_phase6_map.launch.py
python3 -m pytest -q src/tugbot_bringup/test/test_contract.py
colcon build --symlink-install
```

结果：

```text
py_compile：PASS
pytest：10 passed in 0.01s
colcon build：Summary: 5 packages finished [1.56s]
```

## 7. 启动后 ROS graph 与 lifecycle 检查

启动后节点检查：

```text
/amcl
/behavior_server
/bt_navigator
/bt_navigator_navigate_through_poses_rclcpp_node
/bt_navigator_navigate_to_pose_rclcpp_node
/collision_monitor
/controller_server
/docking_server
/global_costmap/global_costmap
/lifecycle_manager_localization
/lifecycle_manager_navigation
/local_costmap/local_costmap
/map_server
/planner_server
/ros_gz_bridge
/route_server
/scan_omni_static_tf
/smoother_server
/velocity_smoother
/waypoint_follower
```

map_server / AMCL：PASS。

Nav2 lifecycle：

```text
/lifecycle_manager_localization/is_active -> success=True
/lifecycle_manager_navigation/is_active -> success=True
```

最终检查同样保持 active：

```text
/lifecycle_manager_localization/is_active -> success=True
/lifecycle_manager_navigation/is_active -> success=True
```

## 8. /map publisher 检查

`/map` topic 检查结果：

```text
Type: nav_msgs/msg/OccupancyGrid
Publisher count: 1
Node name: map_server
Node namespace: /
QoS Durability: TRANSIENT_LOCAL
```

订阅者包括：

```text
/global_costmap/global_costmap
/amcl
```

结论：`/map` 由 `/map_server` 发布。符合 Phase 8 静态地图回放目标。

对比 Phase 1~6：本阶段允许并要求启动 map_server 与 AMCL；不再使用 slam_toolbox 发布 `/map`。

## 9. /scan、/odom、/tf、/cmd_vel 检查

topic 检查摘要：

```text
/scan    publisher: /ros_gz_bridge; subscribers include /amcl, /local_costmap, /global_costmap, /collision_monitor
/odom    publisher: /ros_gz_bridge; subscribers include /controller_server, /bt_navigator
/tf      publishers: /ros_gz_bridge and /amcl
/cmd_vel publishers: /collision_monitor and /docking_server; subscriber: /ros_gz_bridge
```

说明：

1. `/scan` 可用，AMCL 与 costmap 均可订阅；
2. `/odom` 可用，Nav2 controller 与 BT navigator 可订阅；
3. `/tf` 同时有 Gazebo bridge 与 AMCL 发布；
4. `/cmd_vel` 有 Nav2 链路输出并被 `ros_gz_bridge` 接收。

## 10. initialpose 处理情况

按 Phase 8 要求发送了接近默认起点的 initialpose：

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0685]}}"
```

AMCL 日志证据：

```text
[amcl]: initialPoseReceived
[amcl]: Setting pose (51.177000): 0.000 0.000 0.000
```

随后 `/amcl_pose` 可读：

```text
position: x=-0.0206, y=0.0179
orientation: yaw approx 0.006 rad
```

备注：Nav2 bringup 激活 AMCL 时也出现过默认初始位姿日志：

```text
[amcl]: Setting pose (3.042000): 1.000 1.000 0.000
```

本阶段随后显式发布了 `(0, 0, 0)` initialpose，并确认 AMCL 接收。报告中以显式发布后的定位结果为准。

## 11. map -> base_link TF 检查

发送 initialpose 后，`tf2_echo map base_link` 可查询到 TF：

```text
Translation: [-0.021, 0.018, 0.000]
Rotation yaw approx 0.006 rad
```

最终 Goal B 后检查：

```text
Translation: [2.053, -0.892, 0.000]
Rotation yaw approx -0.141 rad
```

结论：AMCL 能在 Phase 6 静态地图上提供 `map -> base_link` 定位链路。

## 12. NavigateToPose goal 测试结果

### 12.1 Goal A

目标：

```text
x=1.0
y=0.0
orientation.w=1.0
```

执行命令：

```bash
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

结果：

```text
Goal accepted
Result: error_code: 0
Goal finished with status: SUCCEEDED
```

位移证据：

```text
/odom before Goal A: x=0.000, y=0.000
/odom after Goal A:  x=0.887, y=-0.004
start -> after Goal A distance approx 0.887 m
```

AMCL 位置：

```text
/amcl_pose after Goal A: x=0.782, y=-0.002
```

### 12.2 Goal B

目标：

```text
x=2.0
y=-1.0
orientation.w=1.0
```

执行命令：

```bash
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: -1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

结果：

```text
Goal accepted
Result: error_code: 0
Goal finished with status: SUCCEEDED
```

位移证据：

```text
/odom after Goal B: x=2.073, y=-0.895
Goal A after -> Goal B after distance approx 1.483 m
start -> after Goal B distance approx 2.258 m
```

AMCL 位置：

```text
/amcl_pose after Goal B: x=2.032, y=-0.886
```

结论：Phase 6 自建静态地图可支持至少两个 NavigateToPose goal，超过本阶段“至少完成 1 个 goal”的验收标准。

## 13. /cmd_vel 输出证据

Goal A 期间 `/cmd_vel` 非零样本：

```text
linear.x: 0.0406
angular.z: 0.0096
linear.x: 0.1714
angular.z: 0.0131
linear.x: 0.4184
```

Goal B 期间 `/cmd_vel` 非零样本：

```text
linear.x: 0.0149
angular.z: -0.0152
linear.x: 0.0807
angular.z: -0.0759
linear.x: 0.0852
angular.z: -0.3633
```

结论：Nav2 在 Phase 6 静态地图上规划并输出了运动控制指令，`ros_gz_bridge` 对 `/cmd_vel` 有订阅。

## 14. /odom 位移证据

关键 `/odom` 位置：

```text
启动后 / Goal A 前：x=0.000, y=0.000
Goal A 后：          x=0.887, y=-0.004
Goal B 后：          x=2.073, y=-0.895
```

累计位移：

```text
start -> Goal A after: 0.887 m
Goal A after -> Goal B after: 1.483 m
start -> Goal B after: 2.258 m
```

结论：小车确实在 Gazebo 中移动，非仅 action 逻辑返回成功。

## 15. 原 Phase 0/1/2/3/4/5/6/7A 入口保护情况

本阶段没有修改以下入口：

```text
src/tugbot_bringup/launch/tugbot_nav.launch.py
src/tugbot_bringup/launch/tugbot_slam.launch.py
src/tugbot_bringup/launch/tugbot_slam_nav.launch.py
src/tugbot_bringup/launch/tugbot_explore.launch.py
```

`test_contract.py` 增加的契约测试明确断言 Phase 6 map 没有写入上述原入口，避免把 Phase 8 静态地图回放行为污染到已有导航、SLAM、SLAM+Nav、探索链路中。

## 16. 当前遗留问题

1. `nav2_params.yaml` 中 global costmap inflation radius 日志提示：

```text
The configured inflation radius (0.350) is smaller than the computed inscribed radius (0.353) of your footprint
```

本次未修改原 `nav2_params.yaml`，且两个 goal 均成功，因此该项记为已有参数告警，不阻塞 Phase 8 通过。后续如要提高静态导航安全裕度，可在单独阶段新增派生参数文件，而不是直接破坏原 nav2 参数。

2. AMCL 启动时 Nav2 bringup 曾自动设置默认 initial pose `(1.0, 1.0, 0.0)`，随后本阶段已按要求显式发布 `(0.0, 0.0, 0.0)` initialpose，并确认 map -> base_link TF 与 goal 成功。后续人工 RViz 验收时也可使用 2D Pose Estimate 进一步校准。

3. 本阶段验证的是近距离静态地图导航回放。更远距离、多点巡航、狭窄区域和全图覆盖导航可作为 Phase 9 或后续导航鲁棒性阶段。

## 17. Phase 8 验收结论

按用户给出的验收重点逐项结论：

| 验收项 | 结果 | 证据 |
| --- | --- | --- |
| map_server 能加载 Phase 6 yaml | PASS | map_server 日志加载 `tugbot_nav_world_slam_phase6_cleanup.yaml/.pgm` |
| AMCL 能基于 Phase 6 地图定位 | PASS | `/amcl` active，`/amcl_pose` 可读，`map -> base_link` 可查 |
| Nav2 能在 Phase 6 地图上规划 | PASS | `/navigate_to_pose` Goal A/B accepted 并 succeeded |
| 小车至少完成 1 个 NavigateToPose goal | PASS | Goal A、Goal B 均 SUCCEEDED |
| /cmd_vel 有输出 | PASS | Goal A/B 期间均采集到非零 `/cmd_vel` |
| /odom 有位移 | PASS | start -> Goal B after approx 2.258 m |
| 不使用旧 map_1725111373.yaml | PASS | launch 与 map_server 日志均未引用旧地图 |
| 不使用 Phase 7A 地图 | PASS | launch 与 map_server 日志均未引用 `phase7_budget` |
| 原探索链路不被破坏 | PASS | 未修改 `tugbot_explore.launch.py`，未启动 frontier 探索 |

最终结论：

```text
Phase 8 PASS。
Phase 6 cleanup map 可以作为静态地图被 map_server 加载；AMCL 可定位；Nav2 可规划并控制 Tugbot 完成 NavigateToPose goal。
完成 Phase 8，已停止 live 进程，等待人工验收。
```
