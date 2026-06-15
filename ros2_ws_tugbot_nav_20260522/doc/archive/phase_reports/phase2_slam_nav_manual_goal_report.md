# Phase 2 Report - SLAM + Nav2 手动导航入口

## 1. 阶段目标

在 0514 工作空间中新增在线 SLAM + Nav2 手动导航入口：

```bash
ros2 launch tugbot_bringup tugbot_slam_nav.launch.py
```

该入口满足本阶段边界：

- 使用 Gazebo `tugbot_nav_world.sdf`；
- 复用已验证的 `ros_gz_bridge`、`/scan`、`/odom`、`/tf`、`/clock` 链路；
- 启动 `slam_toolbox` online async mapping；
- Nav2 使用 `slam_toolbox` 实时发布的 `/map`；
- 不加载旧静态地图 `map_1725111373.yaml`；
- 不启动静态 `map_server`；
- 不启动 AMCL；
- 只验证手动 `NavigateToPose`，不进入 frontier 自主探索。

## 2. 新增 / 修改文件清单

### 新增文件

1. `src/tugbot_bringup/launch/tugbot_slam_nav.launch.py`

   用途：Phase 2 专用 SLAM + Nav2 手动导航启动入口。

   启动内容：

   - include `tugbot_gazebo.launch.py`，复用 Gazebo + bridge + scan 静态 TF；
   - include `slam_toolbox/launch/online_async_launch.py`；
   - include `nav2_bringup/launch/navigation_launch.py`；
   - 默认 world：`tugbot_nav_world.sdf`；
   - 默认 SLAM 参数：`slam_toolbox_params.yaml`；
   - 默认 Nav2 参数：`nav2_slam_params.yaml`；
   - 保留 `headless`、`use_rviz`、`use_sim_time`、`autostart`、`use_composition`、`use_respawn`、`log_level` 等参数。

2. `src/tugbot_navigation/config/nav2_slam_params.yaml`

   用途：Phase 2 专用 Nav2 参数文件，避免直接修改原 `nav2_params.yaml`。

   关键设计：

   - 从原 Nav2 参数中保留 navigation stack、planner、controller、costmap、behavior、velocity smoother 等配置；
   - 删除顶部 `amcl:` 参数块；
   - 不包含 `map_server:`；
   - 不包含 `yaml_filename`；
   - `global_costmap/static_layer` 订阅 `/map`，该 `/map` 由 `slam_toolbox` 实时发布。

### 修改文件

1. `src/tugbot_bringup/test/test_contract.py`

   修改内容：新增 `test_slam_nav_manual_goal_contract()`，静态检查：

   - `tugbot_slam_nav.launch.py` 包含 Gazebo、SLAM、Nav2 navigation 启动链路；
   - 新入口使用 `navigation_launch.py`，不使用 `bringup_launch.py`；
   - 新入口不引用 `map_1725111373.yaml`；
   - 新 Nav2 参数不包含 `amcl:`、`map_server:`、`yaml_filename`；
   - 新 Nav2 参数保留 `bt_navigator`、`planner_server`、`controller_server`、`/scan`、`map` frame 等关键配置。

## 3. 是否保留 tugbot_nav.launch.py 不变

结论：保留不变。

Phase 2 未修改 `src/tugbot_bringup/launch/tugbot_nav.launch.py`。当前 sha256：

```text
656397f327b6abf55c394613639d949efe1fe9fb71729634034d5e26ed16c184  src/tugbot_bringup/launch/tugbot_nav.launch.py
```

静态地图导航入口仍显示旧地图与原 Nav2 参数：

```text
map:         .../tugbot_navigation/maps/map_1725111373.yaml
params_file: .../tugbot_navigation/config/nav2_params.yaml
```

## 4. 是否保留 tugbot_slam.launch.py 不变

结论：保留不变。

Phase 2 未修改 Phase 1 的 `src/tugbot_bringup/launch/tugbot_slam.launch.py`。当前 sha256：

```text
0b2387de5272fb702dbadc97e244e1247ae0dc0798c467fe6ebe2aafd2c24f77  src/tugbot_bringup/launch/tugbot_slam.launch.py
```

`ros2 launch tugbot_bringup tugbot_slam.launch.py --show-args` 仍显示：

```text
world_sdf:        .../tugbot_gazebo/worlds/tugbot_nav_world.sdf
slam_params_file: .../tugbot_navigation/config/slam_toolbox_params.yaml
```

未引入 Nav2、旧静态地图或 `bringup_launch.py`。

## 5. colcon build 结果

执行命令：

```bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

结果：通过。

```text
Starting >>> tugbot_bringup
Starting >>> tugbot_description
Starting >>> tugbot_gazebo
Starting >>> tugbot_navigation
Finished <<< tugbot_gazebo [0.13s]
Finished <<< tugbot_description [0.15s]
Finished <<< tugbot_bringup [0.17s]
Finished <<< tugbot_navigation [0.16s]

Summary: 4 packages finished [0.34s]
```

补充静态检查：

```bash
python3 -m py_compile src/tugbot_bringup/launch/tugbot_slam_nav.launch.py
python3 -m pytest -q src/tugbot_bringup/test/test_contract.py
```

结果：

```text
5 passed in 0.02s
```

## 6. tugbot_slam_nav.launch.py 启动结果

执行命令：

```bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tugbot_bringup tugbot_slam_nav.launch.py headless:=true use_rviz:=false
```

结果：启动成功，后台进程保持运行至验证完成。

关键启动日志：

```text
[gazebo-1]: process started
[bridge_node-2]: process started
[static_transform_publisher-3]: process started
[ros_gz_bridge]: Creating GZ->ROS Bridge: [/clock ... -> /clock]
[ros_gz_bridge]: Creating ROS->GZ Bridge: [/cmd_vel ... -> /cmd_vel]
[ros_gz_bridge]: Creating GZ->ROS Bridge: [/odom ... -> /odom]
[ros_gz_bridge]: Creating GZ->ROS Bridge: [/scan ... -> /scan]
[ros_gz_bridge]: Creating GZ->ROS Bridge: [/tf ... -> /tf]
[slam_toolbox]: Configuring
[slam_toolbox]: Activating
Registering sensor: [Custom Described Lidar]
[controller_server]: process started
[planner_server]: process started
[bt_navigator]: process started
[lifecycle_manager_navigation]: Managed nodes are active
```

运行进程证据：

```text
ros2 launch tugbot_bringup tugbot_slam_nav.launch.py headless:=true use_rviz:=false
gz sim -s -r .../tugbot_nav_world.sdf
ros_gz_bridge/bridge_node
static_transform_publisher --frame-id base_link --child-frame-id tugbot/scan_omni/scan_omni
slam_toolbox/async_slam_toolbox_node
nav2_controller/controller_server
nav2_planner/planner_server
nav2_bt_navigator/bt_navigator
```

## 7. /scan、/odom、/map 采样结果

### topic 列表关键项

```text
/clock
/cmd_vel
/cmd_vel_nav
/cmd_vel_smoothed
/global_costmap/costmap
/global_costmap/costmap_raw
/global_costmap/static_layer
/local_costmap/costmap
/local_costmap/costmap_raw
/map
/map_metadata
/odom
/scan
/slam_toolbox/scan_visualization
/tf
/tf_static
```

### /scan

结论：有数据，frame 与 Phase 1 验证一致。

```text
frame_id: tugbot/scan_omni/scan_omni
```

`/scan` ownership：

```text
Publisher:    /ros_gz_bridge
Subscribers:  /slam_toolbox
              /global_costmap/global_costmap
              /local_costmap/local_costmap
              /collision_monitor
```

### /odom

结论：有数据。

导航前采样：

```text
position.x: 1.1137e-11
position.y: -1.0975e-24
orientation.w: 1.0
```

导航后采样：

```text
position.x: 0.3361720594665697
position.y: -0.0004135446502093262
orientation.z: -0.0020292828856140963
orientation.w: 0.9999979410033654
```

### /map

结论：有数据，且为真实 OccupancyGrid。

```text
frame_id: map
resolution: 0.05000000074505806
width: 315
height: 154
data_len: 48510
unknown_-1: 41601
free_0: 6710
occupied_100: 199
unique_sample: [-1, 0, 100]
```

## 8. /map publisher 是否为 /slam_toolbox

结论：是。

`ros2 topic info -v /map`：

```text
Type: nav_msgs/msg/OccupancyGrid
Publisher count: 1
Node name: slam_toolbox
Node namespace: /
Durability: TRANSIENT_LOCAL
```

`/global_costmap/global_costmap` 同时作为 `/map` subscriber 存在：

```text
Node name: global_costmap
Node namespace: /global_costmap
Endpoint type: SUBSCRIPTION
Durability: TRANSIENT_LOCAL
```

这说明 Nav2 global costmap 使用的是 `slam_toolbox` 实时发布的 `/map`，而不是静态 `map_server`。

## 9. TF 检查结果

执行命令：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

结果：可获得稳定 `map -> base_link` 变换。

```text
At time 203.940000000
- Translation: [0.000, -0.000, 0.000]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, -0.000, 1.000]
```

说明：`tf2_echo` 起始出现一次：

```text
Invalid frame ID "map" ... frame does not exist
```

随后连续输出稳定 transform，判定为启动/订阅暖机瞬态，不作为失败。

当前 TF 关键链路：

```text
map -> odom -> base_link -> tugbot/scan_omni/scan_omni
```

其中 `map -> odom` 由 `slam_toolbox` 提供。

## 10. Nav2 lifecycle 状态

`/lifecycle_manager_navigation/is_active`：

```text
success=True
```

逐节点 lifecycle：

```text
/slam_toolbox active [3]
/controller_server active [3]
/smoother_server active [3]
/planner_server active [3]
/route_server active [3]
/behavior_server active [3]
/bt_navigator active [3]
/waypoint_follower active [3]
/velocity_smoother active [3]
/collision_monitor active [3]
/docking_server active [3]
```

Nav2 节点检查：

```text
/behavior_server
/bt_navigator
/controller_server
/lifecycle_manager_navigation
/planner_server
/smoother_server
/velocity_smoother
/waypoint_follower
```

结论：planner、controller、bt_navigator 等 navigation lifecycle 节点均 active。

## 11. 是否确认 AMCL / map_server 未启动

结论：已确认未启动。

执行命令：

```bash
ros2 node list | grep -E "amcl|map_server"
```

结果：空输出。

进程检查中也未出现 AMCL 或 map_server 进程。

静态检查也确认：

```text
src/tugbot_bringup/launch/tugbot_slam_nav.launch.py
  map_1725111373.yaml: False
  bringup_launch.py: False
  amcl:: False
  map_server:: False
  yaml_filename: False

src/tugbot_navigation/config/nav2_slam_params.yaml
  map_1725111373.yaml: False
  bringup_launch.py: False
  amcl:: False
  map_server:: False
  yaml_filename: False
```

## 12. NavigateToPose 手动 goal 测试结果

测试目标：

```text
frame_id: map
x: 0.5
y: 0.0
orientation.w: 1.0
```

测试脚本使用 `/navigate_to_pose` action client 发送近距离 goal，并订阅 `/cmd_vel`、`/odom`、`/map` 记录证据。

结果：goal 被接收并成功完成。

```text
ACTION_SERVER_AVAILABLE=True
GOAL_ACCEPTED=True
RESULT_AVAILABLE=True
RESULT_STATUS=4
RESULT_OBJECT= nav2_msgs.action.NavigateToPose_Result(error_code=0, error_msg='')
FEEDBACK_COUNT=134
```

说明：ROS 2 action status `4` 表示 `STATUS_SUCCEEDED`，且 Nav2 result `error_code=0`。

Nav2 日志也记录：

```text
[bt_navigator]: Begin navigating from current location (0.00, -0.00) to (0.50, 0.00)
[controller_server]: Received a goal, begin computing control effort.
```

## 13. /cmd_vel 是否输出

结论：是。

测试期间 `/cmd_vel` 订阅记录：

```text
CMD_SAMPLE_COUNT=26
CMD_NONZERO_COUNT=26
```

非零速度样例：

```text
linear.x: 0.03174864873290062, angular.z: 0.007052713073790073
linear.x: 0.05733392760157585, angular.z: 0.0022839184384793043
linear.x: 0.0796474814414978,  angular.z: 0.0026548048481345177
linear.x: 0.10218209028244019, angular.z: 0.008701369166374207
linear.x: 0.12771974503993988, angular.z: 0.0033965730108320713
```

`/cmd_vel` ownership：

```text
Publishers:   /collision_monitor, /docking_server
Subscriber:   /ros_gz_bridge
```

本次 goal 执行中实际非零输出由 Nav2 controller -> velocity_smoother -> collision_monitor 链路进入 `/cmd_vel`，并由 bridge 送入 Gazebo。

## 14. /odom 是否变化

结论：是。

goal 前：

```text
x: 1.1137003737507321e-11
y: -1.0975157966370603e-24
yaw: -5.636283244978472e-13
```

goal 后：

```text
x: 0.2603952760409423
y: -0.00010875181902358329
yaw: -0.0035958439430030164
```

变化量：

```text
ODOM_DELTA=(0.26039527602980533, -0.00010875181902358329, -0.0035958439424393883)
```

后续 `/odom` 再采样：

```text
x: 0.3361720594665697
y: -0.0004135446502093262
```

说明小车确实被 Nav2 goal 驱动前进。

## 15. /map 在运动过程中是否继续由 slam_toolbox 更新

结论：是，`/map` 持续由 `slam_toolbox` 发布。

运动后采样到多帧 `/map`：

```text
MAP_SAMPLE_COUNT=4
MAP_SAMPLE=(368, 400000000, 315, 154, 48510, 41601, 6710, 199)
MAP_SAMPLE=(370, 401000000, 315, 154, 48510, 41601, 6710, 199)
MAP_SAMPLE=(372, 402000000, 315, 154, 48510, 41601, 6710, 199)
MAP_SAMPLE=(374, 400000000, 315, 154, 48510, 41601, 6710, 199)
```

本次近距离 0.5m goal 主要在初始已观测区域内移动，因此 occupancy 计数未明显变化；但 `/map` 时间戳持续推进，且 `/map` publisher 确认为 `/slam_toolbox`。本阶段目标是验证 SLAM + Nav2 手动导航闭环，不要求探索全图。

## 16. 当前遗留问题

1. `global_costmap` 启动时有一条 inflation radius 警告：

   ```text
   The configured inflation radius (0.350) is smaller than the computed inscribed radius (0.353) of your footprint
   ```

   该警告未阻止 lifecycle active，也未阻止本次 `NavigateToPose` 成功。当前阶段不扩大调参 scope，保留为后续可优化项。

2. 自动验证使用：

   ```bash
   headless:=true use_rviz:=false
   ```

   原因是当前 VM/OpenGL 环境存在 Gazebo/RViz 图形加速警告。launch 文件保留 `use_rviz:=true` 默认能力，RViz 视觉显示仍建议人工在图形环境中确认。

3. 本阶段只发送 0.5m 近距离 goal，验证 Nav2 接收、规划、发布 `/cmd_vel` 并驱动车辆运动。未做远距离地图覆盖或 frontier 自动探索；这属于后续 Phase 3 范围。

## 17. 清理结果

验证结束后已停止 Phase 2 启动进程。

清理后检查：

```text
--- post-cleanup process check ---
<empty>

--- post-cleanup node check ---
<empty>
```

未留下 Gazebo / slam_toolbox / Nav2 后台进程污染后续验收。

## 18. Phase 2 验收结论

结论：Phase 2 自动验证通过，建议进入人工验收。

已满足验收点：

- 新增 `tugbot_slam_nav.launch.py`；
- 原 `tugbot_nav.launch.py` 未破坏；
- Phase 1 `tugbot_slam.launch.py` 未破坏；
- 使用 `navigation_launch.py` 启动 Nav2 navigation stack；
- 未加载 `map_1725111373.yaml`；
- 未启动 AMCL；
- 未启动静态 map_server；
- `slam_toolbox` active，并发布 `/map`；
- `/map` publisher 为 `/slam_toolbox`；
- Nav2 navigation lifecycle 节点均 active；
- `/scan`、`/odom`、`/map` 均有数据；
- `map -> base_link` TF 可用；
- `/navigate_to_pose` action server 可用；
- 手动 0.5m goal 被接收并成功完成；
- `/cmd_vel` 有非零输出；
- `/odom` 发生实际位移；
- 阶段完成后未进入 Phase 3。

推荐人工验收命令：

```bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tugbot_bringup tugbot_slam_nav.launch.py headless:=true use_rviz:=false
```

另开终端发送近距离 goal：

```bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
--feedback
```
