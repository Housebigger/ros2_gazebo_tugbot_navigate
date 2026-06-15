# Phase 0 Report - 0514 工作空间基线创建与验证

## 1. 目标

在不修改 0513 工程的前提下，基于已验证的 0513 Tugbot Nav2 静态地图导航工程创建独立 0514 工作空间，并验证原有静态地图导航启动入口仍然可用。

- 旧工程：`/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260513`
- 新工程：`/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514`
- 基线启动入口：`ros2 launch tugbot_bringup tugbot_nav.launch.py`

## 2. 0514 工程是否成功创建

结论：成功创建。

验证结果：

```text
NEW_WORKSPACE_EXISTS=yes
OLD_WORKSPACE_EXISTS=yes
```

0514 顶层内容验证：

```text
NEW_TOP_LEVEL=
  doc
  .pytest_cache
  README.md
  ros2_ws_tugbot_nav_20260513_phase0_inventory.md
  scripts
  src
```

保留项验证：

```text
REQUIRED_PATHS=
  src exists
  doc exists
  scripts exists
  README.md exists
```

## 3. build/install/log 是否已清理

结论：已清理。

验证结果：

```text
REMOVED_PATHS=
  build absent
  install absent
  log absent
```

说明：Phase 0 构建前，0514 工作空间中的 `build`、`install`、`log` 均不存在，符合从源码重新构建的要求。

## 4. colcon build 是否通过

结论：通过。

执行命令：

```bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

构建结果：

```text
Starting >>> tugbot_bringup
Starting >>> tugbot_description
Starting >>> tugbot_gazebo
Starting >>> tugbot_navigation
Finished <<< tugbot_navigation [1.54s]
Finished <<< tugbot_bringup [1.57s]
Finished <<< tugbot_gazebo [1.57s]
Finished <<< tugbot_description [1.78s]

Summary: 4 packages finished [1.95s]
```

备注：第一次在 `set -u` shell 中 source `/opt/ros/jazzy/setup.bash` 时触发 `AMENT_TRACE_SETUP_FILES: unbound variable`。随后按 ROS 2 常规方式在非 `set -u` shell 中执行，构建通过。源码未因此修改。

## 5. tugbot_nav.launch.py 是否能启动

结论：能启动。

执行命令：

```bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tugbot_bringup tugbot_nav.launch.py headless:=true
```

先验证包发现与 launch 参数：

```text
PKG_PREFIXES=
  tugbot_bringup -> /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/install/tugbot_bringup
  tugbot_gazebo -> /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/install/tugbot_gazebo
  tugbot_navigation -> /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/install/tugbot_navigation
  tugbot_description -> /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/install/tugbot_description
```

launch 默认参数解析成功，关键路径均指向 0514 install 空间：

```text
world_sdf default: /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/install/tugbot_gazebo/share/tugbot_gazebo/worlds/tugbot_nav_world.sdf
map default: /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/install/tugbot_navigation/share/tugbot_navigation/maps/map_1725111373.yaml
params_file default: /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/install/tugbot_navigation/share/tugbot_navigation/config/nav2_params.yaml
rviz_config default: /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/install/tugbot_bringup/share/tugbot_bringup/rviz/tugbot_nav.rviz
```

运行期 ROS 图验证：

```text
NODE_LIST=
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
/rviz2
/scan_omni_static_tf
/smoother_server
/transform_listener_impl_56ec1beadfe0
/transform_listener_impl_70490c002200
/transform_listener_impl_77537c005c50
/velocity_smoother
/waypoint_follower
```

关键 topic 验证：

```text
/clock: Publisher count 1, Subscription count 23
/scan:  Publisher count 1, Subscription count 5
/odom:  Publisher count 1, Subscription count 2
/tf:    Publisher count 2, Subscription count 10
/cmd_vel: Publisher count 2, Subscription count 1
/map:   Publisher count 1, Subscription count 3
```

关键 topic 数据采样：

```text
SCAN_ONCE_OK
ODOM_ONCE_OK
MAP_ONCE_OK
```

Nav2 lifecycle 验证：

```text
/lifecycle_manager_localization/is_active -> success=True
/lifecycle_manager_navigation/is_active -> success=True
```

启动日志中出现：

```text
[lifecycle_manager_navigation]: Managed nodes are active
```

说明：0514 中原 0513 基线入口 `tugbot_nav.launch.py` 可以启动 Gazebo、bridge、AMCL、map_server、Nav2 与 RViz，并能发布/订阅基线所需关键 topic。

备注：RViz 在当前图形环境中出现 GLSL shader 报错：

```text
active samplers with a different type refer to the same texture image unit
```

该现象来自 RViz/OpenGL 渲染环境；本次 Phase 0 使用 headless Gazebo 进行基线验证，ROS 图、Nav2 lifecycle 与关键 topic 数据均已通过。因此不判定为基线启动失败。

## 6. 是否确认未修改 0513 工程

结论：已确认未修改 0513 工程。

验证内容：

1. 0513 顶层目录仍存在原有构建产物与源码文档：

```text
OLD_TOP_LEVEL=
  build
  doc
  install
  log
  .pytest_cache
  README.md
  ros2_ws_tugbot_nav_20260513_phase0_inventory.md
  scripts
  src
```

2. 0513 的关键路径仍存在：

```text
OLD_REQUIRED_DIRS=
  src exists
  doc exists
  scripts exists
  README.md exists
  build exists
  install exists
  log exists
```

3. 对比 0513 与 0514 的 `src`、`doc`、`scripts`、`README.md`，在生成本报告前没有发现差异输出。

说明：本 Phase 0 后半段仅在 0514 中执行构建、启动验证与报告生成；未对 0513 执行写入操作。

## 7. 清理状态

启动验证结束后，已终止本次 `ros2 launch tugbot_bringup tugbot_nav.launch.py headless:=true` 后台进程。

验证结果：

```text
ps -ef | egrep 'ros2 launch tugbot_bringup tugbot_nav|gz sim|parameter_bridge|rviz2|component_container|nav2|amcl|map_server|controller_server|planner_server' | grep -v grep
```

输出为空，表示本次 Phase 0 验证启动的 Gazebo/Nav2/RViz/bridge 进程已清理。

## 8. Phase 0 验收结论

Phase 0 结论：通过。

判定依据：

- 0514 工作空间已成功从 0513 基线复制创建；
- 0514 构建前 `build`、`install`、`log` 已清理；
- 0514 中 `src`、`doc`、`scripts`、`README.md` 已保留；
- `colcon build --symlink-install` 成功通过，4 个包构建完成；
- `source install/setup.bash` 后，`tugbot_bringup`、`tugbot_gazebo`、`tugbot_navigation`、`tugbot_description` 均可由 ROS 2 发现；
- 原 0513 基线入口 `ros2 launch tugbot_bringup tugbot_nav.launch.py` 在 0514 中可启动；
- `/clock`、`/scan`、`/odom`、`/tf`、`/cmd_vel`、`/map` 等关键 topic 可用；
- AMCL、map_server、Nav2 navigation lifecycle 均处于 active；
- 已确认未修改 0513 工程；
- 已按阶段边界停止，未进入 Phase 1。

下一步：等待人工验收 Phase 0。人工验收通过后，再进入 Phase 1：新增 SLAM 在线建图入口。
