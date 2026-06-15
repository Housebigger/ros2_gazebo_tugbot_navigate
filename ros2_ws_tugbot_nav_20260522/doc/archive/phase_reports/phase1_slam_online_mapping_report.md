# Phase 1 Report - SLAM 在线建图入口

## 1. 目标

在 0514 工作空间中新增一个 SLAM 在线建图启动链路：

```bash
ros2 launch tugbot_bringup tugbot_slam.launch.py
```

该链路应满足：

- 使用 Gazebo 中的 Tugbot；
- 使用已验证的 `/scan`、`/odom`、`/tf`、`/clock` 桥接链路；
- 启动 `slam_toolbox` online async 模式；
- 实时生成 `/map`；
- 不加载旧静态地图 `map_1725111373.yaml`；
- 不启动 `map_server`；
- 不启动 `AMCL`；
- 不启动 Nav2 navigation 服务器；
- 不修改 0513 工程；
- 不破坏现有 `tugbot_nav.launch.py` 静态地图导航入口。

## 2. 新增 / 修改文件清单

### 新增文件

1. `src/tugbot_navigation/config/slam_toolbox_params.yaml`

   用途：0514 Phase 1 专用 `slam_toolbox` 参数文件。

   关键参数：

   ```yaml
   slam_toolbox:
     ros__parameters:
       use_sim_time: true
       mode: mapping
       map_frame: map
       odom_frame: odom
       base_frame: base_link
       scan_topic: /scan
       max_laser_range: 12.0
       minimum_time_interval: 0.5
       transform_timeout: 0.2
       tf_buffer_duration: 30.0
       stack_size_to_use: 40000000
       enable_interactive_mode: true
   ```

2. `src/tugbot_bringup/launch/tugbot_slam.launch.py`

   用途：新增 SLAM-only online mapping 启动入口。

   启动内容：

   - include `tugbot_gazebo.launch.py`，复用已验证 Gazebo + bridge + scan 静态 TF；
   - include `/opt/ros/jazzy/share/slam_toolbox/launch/online_async_launch.py`；
   - 默认 world 使用 0514 install 空间中的 `tugbot_nav_world.sdf`；
   - 默认 SLAM 参数使用 0514 install 空间中的 `slam_toolbox_params.yaml`；
   - 默认 RViz 复用现有 `tugbot_nav.rviz`；
   - 提供 `headless`、`use_rviz`、`use_sim_time`、`autostart` 等 launch 参数。

### 修改文件

1. `src/tugbot_bringup/package.xml`

   修改内容：新增运行依赖：

   ```xml
   <exec_depend>slam_toolbox</exec_depend>
   ```

2. `src/tugbot_navigation/package.xml`

   修改内容：新增运行依赖：

   ```xml
   <exec_depend>slam_toolbox</exec_depend>
   ```

3. `src/tugbot_bringup/test/test_contract.py`

   修改内容：

   - 增加 `test_slam_online_mapping_contract()`，检查 SLAM 参数和 launch 不包含 Nav2 bringup / 旧静态地图入口；
   - 修正既有 scan 静态 TF contract，使其符合 0513/0514 已验证实际 frame：`base_link -> tugbot/scan_omni/scan_omni`。

## 3. 是否保留 tugbot_nav.launch.py 不变

结论：保留不变。

`src/tugbot_bringup/launch/tugbot_nav.launch.py` 的 sha256 在 Phase 1 修改前后保持：

```text
656397f327b6abf55c394613639d949efe1fe9fb71729634034d5e26ed16c184  src/tugbot_bringup/launch/tugbot_nav.launch.py
```

`ros2 launch tugbot_bringup tugbot_nav.launch.py --show-args` 仍显示旧静态地图导航入口保留，关键默认路径仍为：

```text
world_sdf: .../install/tugbot_gazebo/share/tugbot_gazebo/worlds/tugbot_nav_world.sdf
map:       .../install/tugbot_navigation/share/tugbot_navigation/maps/map_1725111373.yaml
params:    .../install/tugbot_navigation/share/tugbot_navigation/config/nav2_params.yaml
```

说明：Phase 1 没有删除或替换静态地图导航功能。

## 4. colcon build 结果

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
Finished <<< tugbot_gazebo [0.12s]
Finished <<< tugbot_bringup [0.16s]
Finished <<< tugbot_navigation [0.15s]
Finished <<< tugbot_description [0.17s]

Summary: 4 packages finished [0.32s]
```

补充检查：

```bash
python3 -m py_compile src/tugbot_bringup/launch/tugbot_slam.launch.py
python3 -m pytest -q src/tugbot_bringup/test/test_contract.py
```

结果：

```text
PY_COMPILE_OK
4 passed in 0.01s
```

## 5. tugbot_slam.launch.py 启动结果

启动命令：

```bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tugbot_bringup tugbot_slam.launch.py headless:=true use_rviz:=false
```

说明：本次自动验证使用 `use_rviz:=false`，避免当前 VM/OpenGL 环境中的 RViz shader 报错干扰 SLAM-only 证据采集。launch 文件默认仍支持 RViz：`use_rviz:=true`。

进程证据：

```text
ros2 launch tugbot_bringup tugbot_slam.launch.py headless:=true use_rviz:=false
gz sim -s -r .../tugbot_nav_world.sdf
static_transform_publisher --frame-id base_link --child-frame-id tugbot/scan_omni/scan_omni
/opt/ros/jazzy/lib/slam_toolbox/async_slam_toolbox_node --ros-args -r __node:=slam_toolbox
```

启动日志关键内容：

```text
[ros_gz_bridge]: Creating GZ->ROS Bridge: [/clock ... -> /clock]
[ros_gz_bridge]: Creating ROS->GZ Bridge: [/cmd_vel ... -> /cmd_vel]
[ros_gz_bridge]: Creating GZ->ROS Bridge: [/odom ... -> /odom]
[ros_gz_bridge]: Creating GZ->ROS Bridge: [/scan ... -> /scan]
[ros_gz_bridge]: Creating GZ->ROS Bridge: [/tf ... -> /tf]
[slam_toolbox]: Node using stack size 40000000
[slam_toolbox]: Configuring
[slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[LifecycleLaunch] Slamtoolbox node is activating.
[slam_toolbox]: Activating
Registering sensor: [Custom Described Lidar]
```

运行节点：

```text
/launch_ros_24027
/ros_gz_bridge
/scan_omni_static_tf
/slam_toolbox
/transform_listener_impl_59b52ea52340
```

`slam_toolbox` lifecycle：

```text
active [3]
```

参数抽查：

```text
mode         -> mapping
scan_topic   -> /scan
map_frame    -> map
odom_frame   -> odom
base_frame   -> base_link
use_sim_time -> True
```

## 6. /scan、/odom、/map 采样结果

### topic 列表检查

```text
/clock
/map
/map_metadata
/odom
/scan
/slam_toolbox/scan_visualization
/tf
/tf_static
```

### /scan

结论：有数据。

```text
SCAN_ONCE_RC=0
/tmp/phase1_scan_once.txt size: 3674 bytes
```

`/scan` ownership：

```text
Publisher:    /ros_gz_bridge
Subscription: /slam_toolbox
```

### /odom

结论：有数据。

```text
ODOM_ONCE_RC=0
/tmp/phase1_odom_once.txt size: 1055 bytes
```

`/odom` ownership：

```text
Publisher: /ros_gz_bridge
```

### /map

结论：由 `slam_toolbox` 实时生成，有数据。

```text
MAP_ONCE_RC=0
/tmp/phase1_map_once.txt size: 998 bytes
```

`/map` ownership：

```text
Publisher: /slam_toolbox
Type: nav_msgs/msg/OccupancyGrid
Durability: TRANSIENT_LOCAL
```

`/map` 头部采样：

```text
header:
  frame_id: map
info:
  resolution: 0.05000000074505806
  width: 315
  height: 154
  origin:
    position:
      x: -8.673286567586647
      y: -5.447895384637929
```

occupancy 内容统计：

```text
frame_id: map
resolution: 0.05000000074505806
width: 315
height: 154
data_len: 48510
count_unknown_-1: 41601
count_free_0: 6710
count_occupied_100: 199
unique_values_sample: [-1, 0, 100]
```

这说明 `/map` 不是空 topic：它包含 unknown、free、occupied 三类 occupancy 值，来源为 `slam_toolbox`。

### /map_metadata

结论：有数据。

```text
resolution: 0.05000000074505806
width: 315
height: 154
origin:
  position:
    x: -8.673286567586647
    y: -5.447895384637929
```

## 7. TF 检查结果

执行命令：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

结果：可查询。

说明：开始时出现一次 startup transient：

```text
Waiting for transform map -> base_link: Invalid frame ID "map" ... frame does not exist
```

随后稳定输出：

```text
At time 49.896000000
- Translation: [0.000, -0.000, 0.000]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, -0.000, 1.000]

At time 50.904000000
- Translation: [0.000, -0.000, 0.000]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, -0.000, 1.000]
```

`/tf` 单次采样中可以看到 `slam_toolbox` 发布的 `map -> odom`：

```text
frame_id: map
child_frame_id: odom
transform:
  translation:
    x: -0.0
    y: -0.0
    z: -0.0
  rotation:
    w: 1.0
```

当前已验证实际 frame 链路：

```text
map -> odom -> base_link -> tugbot/scan_omni/scan_omni
```

其中：

- `map -> odom` 由 `slam_toolbox` 发布；
- `odom -> base_link` 来自 Gazebo DiffDrive 经 `/tf` bridge；
- `base_link -> tugbot/scan_omni/scan_omni` 来自既有静态 TF。

## 8. 是否确认 AMCL / map_server / Nav2 未启动

结论：已确认未启动。

反向检查命令：

```bash
ros2 node list | grep -E 'amcl|map_server|bt_navigator|planner_server|controller_server'
```

输出为空。

本次运行节点只有：

```text
/launch_ros_24027
/ros_gz_bridge
/scan_omni_static_tf
/slam_toolbox
/transform_listener_impl_59b52ea52340
```

因此：

- 不存在 `/amcl`；
- 不存在 `/map_server`；
- 不存在 `/bt_navigator`；
- 不存在 `/planner_server`；
- 不存在 `/controller_server`；
- 未启动 Nav2 bringup。

## 9. 是否确认未修改 0513 工程

结论：已确认未修改 0513 工程。

检查结果：

```text
0513 lacks src/tugbot_bringup/launch/tugbot_slam.launch.py
0513 lacks src/tugbot_navigation/config/slam_toolbox_params.yaml
```

说明：Phase 1 新增文件只存在于 0514。0513 的 `tugbot_nav.launch.py` 与 0514 当前 `tugbot_nav.launch.py` 对比无差异输出。

## 10. 清理状态

SLAM-only live 验证完成后，已终止本次后台 launch。

检查命令：

```bash
ps -ef | egrep 'ros2 launch tugbot_bringup tugbot_slam|gz sim|parameter_bridge|rviz2|slam_toolbox|amcl|map_server|bt_navigator|planner_server|controller_server' | grep -v grep
```

输出为空，说明本次验证进程已清理。

## 11. 当前遗留问题

1. 当前自动验证使用 `use_rviz:=false`，因为本环境存在 VM/OpenGL/RViz shader 报错风险。`tugbot_slam.launch.py` 已保留默认 RViz 启动能力，人工验收可在图形环境中运行：

   ```bash
   ros2 launch tugbot_bringup tugbot_slam.launch.py headless:=false use_rviz:=true
   ```

2. Gazebo / SDF 仍会输出：

   ```text
   XML Element[frame_id], child of element[sensor], not defined in SDF. Copying[frame_id] as children of [sensor].
   ```

   该 warning 已在 0513 基线中存在，不影响本次 `/scan`、`/odom`、`/map` 与 TF 验证。

3. 本阶段按约束只验证 SLAM 在线建图，不进行 Nav2 手动导航，不进行 frontier 探索，也不做复杂地图质量对比。

4. 当前短时静止验证已证明 `/map` 由 SLAM 实时生成且包含 occupancy 数据。后续 Phase 2/Phase 3 可在导航或探索闭环中进一步验证运动过程中的地图扩展。

## 12. Phase 1 验收结论

Phase 1 结论：通过。

判定依据：

- 已新增 `slam_toolbox_params.yaml`；
- 已新增 `tugbot_slam.launch.py`；
- 已复用 0514 已验证 Gazebo + ros_gz_bridge + scan 静态 TF 链路；
- 未加载 `map_1725111373.yaml`；
- 未启动 `map_server`；
- 未启动 AMCL；
- 未启动 Nav2 bringup / navigation 服务器；
- `colcon build --symlink-install` 通过；
- `slam_toolbox` online async 成功启动并处于 `active [3]`；
- `/scan` 有数据，且被 `slam_toolbox` 订阅；
- `/odom` 有数据；
- `/map` 由 `slam_toolbox` 发布，包含 unknown/free/occupied occupancy 数据；
- `tf2_echo map base_link` 可持续输出 transform；
- 0513 工程未修改；
- 0514 原静态地图导航入口 `tugbot_nav.launch.py` 保持不变；
- 已按阶段边界停止，未进入 Phase 2。

下一步：等待人工验收 Phase 1。人工验收通过后，再进入 Phase 2：SLAM + Nav2 手动导航入口。
