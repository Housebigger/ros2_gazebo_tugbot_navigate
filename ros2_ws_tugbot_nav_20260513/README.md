# ros2_ws_tugbot_nav_20260513

Tugbot ROS 2 Jazzy + Gazebo + Nav2 验证工作空间。

## 环境准备

```bash
cd ~/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260513
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

如修改源码或配置后，先构建：

```bash
colcon build --symlink-install
source install/setup.bash
```

## Gazebo 单独启动

默认 headless/server-only 启动：

```bash
ros2 launch tugbot_gazebo tugbot_gazebo.launch.py
```

显式打开 GUI：

```bash
ros2 launch tugbot_gazebo tugbot_gazebo.launch.py headless:=false
```

可选检查命令：

```bash
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic echo /tf_static --once
```

手动速度验证：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 5
```

## Nav2 一键启动

```bash
cd ~/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260513
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tugbot_bringup tugbot_nav.launch.py
```

当前一键启动默认会打开 Gazebo GUI，因为 `tugbot_nav.launch.py` 已声明并向
`tugbot_gazebo.launch.py` 传递：

```bash
headless:=false
```

如果需要 headless/server-only 模式，使用：

```bash
ros2 launch tugbot_bringup tugbot_nav.launch.py headless:=true
```

该 launch 会启动：

- Gazebo server + GUI（默认）；
- ros_gz_bridge；
- Tugbot scan 静态 TF；
- RViz；
- map_server / AMCL / Nav2 controller / planner / bt_navigator 等 Nav2 节点。

常用检查命令：

```bash
ros2 node list
ros2 topic list
ros2 run tf2_ros tf2_echo map base_link
ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger '{}'
ros2 service call /lifecycle_manager_localization/is_active std_srvs/srv/Trigger '{}'
```

CLI 发送一个 Nav2 goal 示例：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

## RViz 人工定位与 Nav2 Goal 使用说明

### 一、定位初值调准方法

启动：

```bash
ros2 launch tugbot_bringup tugbot_nav.launch.py
```

启动后，Gazebo 和 RViz 都会打开。此时小车不会自动运动，这是正常现象。Nav2 启动后，AMCL 需要知道机器人在静态地图 `/map` 中的大致初始位姿。

RViz 中黑色轮廓来自静态地图 `/map`，红色点线来自实时激光雷达 `/scan`。如果刚启动时红色 LaserScan 与黑色地图墙体轮廓明显错位，通常表示 AMCL 初始位姿没有调准。

操作方法：

1. 在 RViz 顶部工具栏点击 `2D Pose Estimate`。
2. 在地图中找到 Gazebo 里 Tugbot 实际所在的大概位置。
3. 鼠标左键按住该位置。
4. 朝 Tugbot 车头方向拖动，指定初始朝向。
5. 松开鼠标。
6. 观察红色 LaserScan 是否与黑色地图轮廓贴合。
7. 如果仍然错位，可以重复执行 `2D Pose Estimate` 微调。

判断标准：

- 红色 LaserScan 点线应大体贴合黑色地图墙体边界。
- RobotModel / footprint 应位于地图中的合理位置。
- TF 状态应稳定为 OK。
- `tf2_echo map base_link` 应能持续输出 transform。

### 二、Nav2 Goal 目标点设置方法

启动工程后，小车不会自动运动。要让小车导航，需要在 RViz 中发送目标点。

在当前 RViz 配置中，顶部工具栏里的 `2D Goal Pose` 就是设置 Nav2 目标点的按钮，也可以理解为 Nav2 Goal。

操作方法：

1. 先确认已经使用 `2D Pose Estimate` 调准初始位姿。
2. 点击 RViz 顶部工具栏中的 `2D Goal Pose`。
3. 在地图中的可通行空白区域点击目标位置。
4. 按住鼠标并拖动，指定小车到达后的目标朝向。
5. 松开鼠标。
6. 观察 RViz 中是否生成规划路径。
7. 观察 Gazebo 中 Tugbot 是否开始运动。
8. 等待小车到达目标点位和姿态。

建议：

- 第一次测试时目标点不要设置太远。
- 不要把目标点点在墙体、障碍物或地图外。
- 优先选择当前房间或相邻开阔区域。
- 等小车能稳定到达近距离目标后，再测试更远目标点。

### 三、当前导航工程的工作原理说明

当前 Tugbot 导航工程采用“预加载静态地图 + AMCL 定位 + 激光雷达实时感知 + Nav2 全局/局部代价地图”的导航方式。

其中：

1. 静态地图 `/map`
   - 由 `map_server` 加载。
   - RViz 中黑色墙体轮廓来自 `/map`。
   - Nav2 根据该地图进行全局路径规划。
   - 它表示机器人预先知道的固定环境布局。

2. 实时激光 `/scan`
   - 由 Gazebo 中 Tugbot 的激光雷达仿真产生。
   - RViz 中红色点线来自 `/scan`。
   - 用于 AMCL 定位匹配。
   - 也用于 local costmap 进行局部障碍物感知和避障。

3. AMCL
   - 根据 `/map`、`/scan`、`/odom` 和 TF 估计机器人在 `map` 坐标系中的位姿。
   - `2D Pose Estimate` 的作用是给 AMCL 一个合理的初始位姿。
   - 初始位姿越准，LaserScan 与地图轮廓越容易贴合。

4. Nav2
   - global planner 基于静态地图和全局代价地图规划全局路径。
   - controller 根据局部代价地图和机器人当前位置输出 `/cmd_vel`。
   - Gazebo 中的差速驱动插件接收 `/cmd_vel` 后驱动 Tugbot 运动。

5. 当前工程不是纯粹“盲扫探索”
   - 小车预先知道静态地图中的墙体和固定障碍布局。
   - 同时也会根据实时 `/scan` 感知当前周围障碍。
   - 因此它属于“已知地图导航 + 实时局部感知避障”。

## 已验证通过项（2026-05-13 live）

- `colcon build --symlink-install` 通过。
- `colcon build --symlink-install --packages-select tugbot_bringup` 通过。
- `colcon test --event-handlers console_direct+ && colcon test-result --verbose` 通过。
- Gazebo headless 单独启动可保持运行。
- ROS 侧关键话题存在：`/cmd_vel`、`/odom`、`/scan`、`/tf`、`/tf_static`。
- Gazebo 侧关键话题存在：`/clock`、`/cmd_vel`、`/odom`、`/scan`、`/tf`。
- `/odom` 与 `/scan` 已确认有 live 数据。
- 手动发布 `/cmd_vel` 后，Tugbot odom 位姿发生变化，确认底盘可运动。
- `headless:=false` 已可启动，不再因模型 sensors 插件导致 Gazebo server 崩溃。
- `ros2 launch tugbot_bringup tugbot_nav.launch.py` 可一键启动 Gazebo GUI、RViz、Nav2。
- `/lifecycle_manager_navigation/is_active` 与 `/lifecycle_manager_localization/is_active` 返回 `success=True`。
- `tf2_echo map base_link` 已能输出 transform。
- `/map` 与 `/scan` 均有 live 数据；RViz 配置包含 Map、RobotModel、LaserScan，Fixed Frame 为 `map`。
- `/navigate_to_pose` action 存在；CLI 发送 Nav2 goal 被接受并最终 `SUCCEEDED`。

## 当前工程状态

当前工程状态：完全通过。

完全通过：Gazebo + Tugbot + RViz + AMCL + Nav2 导航闭环已完成验证。

人工验收已确认：

- Gazebo GUI 可正常启动。
- RViz 可正常显示地图、机器人模型和 LaserScan。
- 使用 2D Pose Estimate 后，红色 LaserScan 与黑色地图轮廓能够贴合。
- 使用 2D Goal Pose 后，Nav2 能根据地图规划避障路径。
- Tugbot 能够运动并到达目标点位和目标姿态。
- 当前工程状态为：完全通过。

## RViz 人工目视确认项

人工打开 `ros2 launch tugbot_bringup tugbot_nav.launch.py` 后，请在 RViz 中目视确认：

- Fixed Frame 为 `map`。
- Map 图层可见，且地图位置、尺度无明显异常。
- RobotModel / robot footprint 位于地图中的合理位置。
- LaserScan 点云/扫描线可见，并随机器人姿态处于合理方位。
- 发送 Nav2 goal 后，RViz 中能看到规划路径或导航反馈变化。
- 机器人模型/footprint 在导航过程中有连续位姿更新。

## 已知问题 / 后续可清理项

- VMware/OpenGL 相关 warning 属于虚拟机图形环境问题。
- Gazebo SDF 中 `sensor/frame_id` 非标准标签 warning 后续可清理。
- Nav2 `inflation_radius (0.350)` 略小于 footprint inscribed radius `0.353`，后续可将 `inflation_radius` 调整到 `0.40`。
- `docking_server` warning 当前不影响基础导航，因为本工程暂不做 docking 功能。
- 启动初期可能出现短暂 TF/cache warning，系统稳定后 `tf2_echo map base_link` 正常。
- Nav2 已能接受并完成一个短距离 CLI goal；更复杂路径、避障鲁棒性、长期运行稳定性仍需后续专项验证。

## 本轮修改文件清单 / 最小修复记录

### 2026-05-13 Nav2 一键启动 GUI / RViz 最小修复

- `src/tugbot_bringup/launch/tugbot_nav.launch.py`
  - 新增 `headless` 参数并向 `tugbot_gazebo.launch.py` 传递。
  - RViz 延迟 8 秒启动。
- `src/tugbot_bringup/rviz/tugbot_nav.rviz`
  - Map `/map` 的 `Durability Policy` 改为 `Transient Local`。
- `colcon build --symlink-install --packages-select tugbot_bringup` 通过。

### 早前最小修复记录

- `src/tugbot_description/models/tugbot/model.sdf`
  - 移除模型内部重复的 `gz-sim-sensors-system` 插件，改为使用 world-level sensors system，避免 Gazebo server 在当前环境中启动崩溃。
- `src/tugbot_gazebo/launch/tugbot_gazebo.launch.py`
  - 将 scan 静态 TF child frame 修正为 Gazebo LaserScan 实际 frame：`tugbot/scan_omni/scan_omni`。
- `src/tugbot_navigation/config/nav2_params.yaml`
  - 将 MPPI `CostCritic.consider_footprint` 设为 `false`，与当前 costmap 使用 `robot_radius` 的配置保持一致，避免 Nav2 controller 配置失败。
