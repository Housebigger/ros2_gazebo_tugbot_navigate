# ros2_ws_tugbot_nav_20260513 Phase 0 Inventory

生成时间：2026-05-13
工作根目录：`/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo`
新工作空间：`/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260513`

## 1. 参考资源存在性

- `project_reference/ros2_ws8`：存在。核心包是 `nav_car`。
- `tugbot_without_plugins`：存在。核心模型文件是 `model.sdf` / `model.config` / `meshes/`。
- `doc/doc_manual`：存在。与本轮最相关的是 `chapter8导航.pdf`。

## 2. ros2_ws8 原始结构与关键文件

### 2.1 包名

- 原始包名：`nav_car`
- 包路径：`project_reference/ros2_ws8/src/nav_car`
- 类型：`ament_python`

### 2.2 原始 launch 文件

- `project_reference/ros2_ws8/src/nav_car/launch/build_map.launch.py`
- `project_reference/ros2_ws8/src/nav_car/launch/cartographer_build_map_with_odom.launch.py`
- `project_reference/ros2_ws8/src/nav_car/launch/cartographer_build_map_without_odom.launch.py`
- `project_reference/ros2_ws8/src/nav_car/launch/path_planner.launch.py`
- `project_reference/ros2_ws8/src/nav_car/launch/publish_map.launch.py`
- `project_reference/ros2_ws8/src/nav_car/launch/self_map_nav.launch.py`

重点启动入口：

- 建图/联合仿真：`ros2 launch nav_car cartographer_build_map_with_odom.launch.py`
- 静态地图导航：`ros2 launch nav_car self_map_nav.launch.py`
- 其他辅助：`build_map.launch.py`、`publish_map.launch.py`、`path_planner.launch.py`

### 2.3 Nav2 参数文件

- `project_reference/ros2_ws8/src/nav_car/param/param_nav2.yaml`

核心文件：`project_reference/ros2_ws8/src/nav_car/param/param_nav2.yaml`

关键原始设置摘录：

- `amcl.base_frame_id: base_footprint`
- `amcl.odom_frame_id: odom`
- `amcl.global_frame_id: map`
- `amcl.scan_topic: scan`
- `bt_navigator.robot_base_frame: base_link`
- `bt_navigator.odom_topic: /odom`
- local/global costmap 均订阅 `/scan`
- local/global costmap 原始 `robot_radius: 0.18`

### 2.4 地图文件

- `project_reference/ros2_ws8/src/nav_car/map/cartographer/map_1726057922.pgm`
- `project_reference/ros2_ws8/src/nav_car/map/cartographer/map_1726057922.yaml`
- `project_reference/ros2_ws8/src/nav_car/map/slam_toolbox/map_1725111373.pgm`
- `project_reference/ros2_ws8/src/nav_car/map/slam_toolbox/map_1725111373.yaml`

本轮优先迁移 `map/slam_toolbox/map_1725111373.yaml/.pgm`，因为 `self_map_nav.launch.py` 默认使用它。

### 2.5 RViz 文件

- `project_reference/ros2_ws8/src/nav_car/sdf/nav.rviz`
- `project_reference/ros2_ws8/src/nav_car/sdf/rviz.rviz`

`self_map_nav.launch.py` 默认使用 `sdf/rviz.rviz`。

### 2.6 Gazebo / bridge / robot 模型文件

- `project_reference/ros2_ws8/src/nav_car/sdf/nav_car.sdf`
- `project_reference/ros2_ws8/src/nav_car/sdf/nav_world.sdf`

核心原始链路：

- world：`sdf/nav_world.sdf`
- robot model：`sdf/nav_car.sdf`
- bridge：`sdf/nav_bridge.yaml`

`nav_car` 示例模式：

- `ros_gz_sim/gz_sim.launch.py` 启动 Gazebo world
- `ros_gz_sim create -string <robot_desc>` 生成机器人
- `ros_gz_bridge parameter_bridge` 使用 YAML 桥接 `/cmd_vel`、`/scan`、`/imu`、`/tf`、`/joint_states`
- `robot_state_publisher` 使用 SDF 文本作为 `robot_description`
- `nav2_bringup/bringup_launch.py` 启动 Nav2
- `rviz2` 加载 RViz 配置

## 3. ros2_ws8 原始启动命令

```bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/project_reference/ros2_ws8
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch nav_car self_map_nav.launch.py
```

建图参考命令：

```bash
ros2 launch nav_car cartographer_build_map_with_odom.launch.py
```

## 4. Tugbot 模型资源梳理

### 4.1 模型资源文件

- `tugbot_without_plugins/meshes/VLP16_base_1.dae`
- `tugbot_without_plugins/meshes/VLP16_base_2.dae`
- `tugbot_without_plugins/meshes/VLP16_scan.dae`
- `tugbot_without_plugins/meshes/base/logo-new.png`
- `tugbot_without_plugins/meshes/base/movai-logo.png`
- `tugbot_without_plugins/meshes/base/movai_logo.dae`
- `tugbot_without_plugins/meshes/base/tugbot_simp.dae`
- `tugbot_without_plugins/meshes/base/tugbot_simp.stl`
- `tugbot_without_plugins/meshes/gripper2/gripper2.dae`
- `tugbot_without_plugins/meshes/gripper2/gripper_hand.stl`
- `tugbot_without_plugins/meshes/light_link/light.stl`
- `tugbot_without_plugins/meshes/light_link/light_led.stl`
- `tugbot_without_plugins/meshes/wheel/wheel.dae`
- `tugbot_without_plugins/model.config`
- `tugbot_without_plugins/model.sdf`
- `tugbot_without_plugins/thumbnails/1.png`
- `tugbot_without_plugins/tugbot_plugin_diff_sensors_world_plugin_keypress_0713.sdf`
- `tugbot_without_plugins/tugbot_plugin_diff_world_plugin_keypress_0705.sdf`
- `tugbot_without_plugins/tugbot_without_plugins_world.sdf`

### 4.2 从 `tugbot_without_plugins/model.sdf` grep/解析确认的 link

- `base_link`
- `imu_link`
- `warnign_light`
- `camera_front`
- `camera_back`
- `scan_front`
- `scan_back`
- `scan_omni`
- `gripper`
- `gripper_hand`
- `wheel_front`
- `wheel_back`
- `wheel_left`
- `wheel_right`

### 4.3 从 `tugbot_without_plugins/model.sdf` grep/解析确认的 joint

- `imu_link_joint`: `fixed`
- `warnign_light_joint`: `revolute`
- `camera_front_joint`: `fixed`
- `camera_back_joint`: `fixed`
- `scan_front_joint`: `fixed`
- `scan_back_joint`: `fixed`
- `scan_omni_joint`: `fixed`
- `gripper_joint`: `revolute`
- `gripper_hand_joint`: `revolute`
- `wheel_front_joint`: `ball`
- `wheel_back_joint`: `ball`
- `wheel_left_joint`: `revolute`
- `wheel_right_joint`: `revolute`

重点轮子 joint（不得臆造，已从模型确认）：

- 左驱动轮：`wheel_left_joint`，type=`revolute`
- 右驱动轮：`wheel_right_joint`，type=`revolute`
- 前/后支撑轮：`wheel_front_joint`、`wheel_back_joint`，type=`ball`

### 4.4 从 `tugbot_without_plugins/model.sdf` grep/解析确认的 sensor

- link=`imu_link`, sensor=`imu`, type=`imu`, topic=`(no explicit topic)`
- link=`camera_front`, sensor=`color`, type=`camera`, topic=`camera_front_color`
- link=`camera_front`, sensor=`depth`, type=`depth_camera`, topic=`(no explicit topic)`
- link=`camera_back`, sensor=`color`, type=`camera`, topic=`camera_back_color`
- link=`camera_back`, sensor=`depth`, type=`depth_camera`, topic=`(no explicit topic)`
- link=`scan_front`, sensor=`scan_front`, type=`gpu_lidar`, topic=`(no explicit topic)`
- link=`scan_back`, sensor=`scan_back`, type=`gpu_lidar`, topic=`(no explicit topic)`
- link=`scan_omni`, sensor=`scan_omni`, type=`gpu_lidar`, topic=`(no explicit topic)`
- link=`gripper`, sensor=`sensor_contact`, type=`gpu_lidar`, topic=`(no explicit topic)`

本轮 Nav2 最小适配优先使用 2D 激光：`scan_omni`，frame/link 为 `scan_omni`，topic 将显式设置为 `/scan`。

### 4.5 已存在插件线索

`tugbot_without_plugins/model.sdf` 已含 Gazebo Sim 插件雏形：

- `gz-sim-diff-drive-system`
  - `left_joint = wheel_left_joint`
  - `right_joint = wheel_right_joint`
  - `wheel_separation = 0.514`
  - `wheel_radius = 0.195`
  - 原始 topic 为 `/tugbot/cmd_vel`
- `gz-sim-sensors-system`

本轮按 ros2_ws8 的 ROS-facing 约定改为：

- `/cmd_vel`
- `/odom`
- `/scan`
- `/tf`
- `/tf_static` 由 `robot_state_publisher` 发布固定树

## 5. doc/doc_manual 教程理解摘录

Chapter 8 说明 Nav2 导航依赖建图、定位、规划、控制、行为树和生命周期管理；运行时核心输入包括里程计、激光雷达、IMU/相机等传感器信息，输出包括速度控制指令、代价地图和定位信息。本轮只迁移 ros2_ws8 的导航示例链路，不扩展 frontier exploration。

## 6. 本轮迁移判断

### 保留/迁移

- ros2_ws8 的 launch 分层思路：Gazebo -> spawn robot -> bridge -> robot_state_publisher -> Nav2 -> RViz。
- ros2_ws8 的 `nav_bridge.yaml` 话题桥接形式。
- ros2_ws8 的 Nav2 参数作为基线副本，再做 frame/topic/半径最小适配。
- ros2_ws8 的地图和 RViz 配置。
- tugbot_without_plugins 的完整 meshes/model.config/model.sdf 资源。

### 最小重写

- Tugbot 模型内 DiffDrive topic/odom/tf 输出；必须保留已确认 joint 名。
- Tugbot 2D scan topic；优先把 `scan_omni` 改为 ROS 导航需要的 `/scan`。
- 新包 install 规则，确保 nested meshes 不被压平。
- 新 launch 文件，避免改原 reference workspace。

### 明确不做

- 不开发 frontier exploration。
- 不开发复杂自主探索算法。
- 不修改 `project_reference/ros2_ws8`。
- 不修改 `tugbot_without_plugins`。
