# Phase 4 Report - 长时间 frontier 探索、地图保存与基础验收

## 1. 阶段目标

在 Phase 3 已通过人工验收的 frontier 自主探索闭环基础上，Phase 4 进行更长时间探索，扩大 SLAM 地图覆盖范围，并在探索结束后保存地图为 `.yaml + .pgm` 文件。

本阶段边界：

- 不修改 `ros2_ws_tugbot_nav_20260513`；
- 不破坏 0514 的 `tugbot_nav.launch.py`；
- 不破坏 `tugbot_slam.launch.py`；
- 不破坏 `tugbot_slam_nav.launch.py`；
- 保留 `tugbot_explore.launch.py` 已有 Phase 3 能力，并只做参数增强；
- 不修改原 `nav2_params.yaml`；
- 不做复杂地图相似度算法；
- Phase 4 完成后停止，等待人工验收。

## 2. 新增 / 修改文件清单

### 修改文件

1. `src/tugbot_exploration/tugbot_exploration/frontier_explorer.py`

   新增：

   - `save_map` 参数，默认 `false`；
   - `map_save_path` 参数，默认：
     `/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam`；
   - `map_save_timeout_sec` 参数，默认 `30.0`；
   - 探索完成后调用：
     `ros2 run nav2_map_server map_saver_cli -f <map_save_path>`；
   - 保存前自动创建输出目录；
   - 日志记录保存命令、返回码、stdout/stderr；
   - 检查 `.yaml` 和 `.pgm` 是否存在且非空。

2. `src/tugbot_bringup/launch/tugbot_explore.launch.py`

   新增 / 调整 launch 参数：

   - `save_map`，默认 `false`；
   - `map_save_path`，默认保存到 `src/tugbot_navigation/maps/explored/tugbot_nav_world_slam`；
   - `map_save_timeout_sec`，默认 `30.0`；
   - `max_goals` 默认从 Phase 3 的 `5` 提高为 `10`；
   - 保留 `finish_no_frontier_cycles` 默认 `5`。

3. `src/tugbot_bringup/test/test_contract.py`

   扩展 contract 测试，覆盖：

   - `save_map`；
   - `map_save_path`；
   - `map_saver_cli`；
   - `.yaml/.pgm` 输出；
   - README Phase 4 关键说明。

4. `README.md`

   更新 0514 工程定位、四个启动入口、frontier 探索并保存地图命令、验收标准和地图检查命令。

### 新增目录 / 输出文件

1. `src/tugbot_navigation/maps/explored/`

2. `src/tugbot_navigation/maps/explored/tugbot_nav_world_slam.yaml`

3. `src/tugbot_navigation/maps/explored/tugbot_nav_world_slam.pgm`

## 3. 是否保留 Phase 0 / 1 / 2 / 3 入口不变

结论：保留。

Phase 4 未修改以下冻结入口 / 配置文件：

```text
656397f327b6abf55c394613639d949efe1fe9fb71729634034d5e26ed16c184  src/tugbot_bringup/launch/tugbot_nav.launch.py
0b2387de5272fb702dbadc97e244e1247ae0dc0798c467fe6ebe2aafd2c24f77  src/tugbot_bringup/launch/tugbot_slam.launch.py
1f20010cd9f5082f08e800849d81a8d2c74dd14de135c8e8620780958f08c099  src/tugbot_bringup/launch/tugbot_slam_nav.launch.py
65e22ee9021ede7416720ed3e6fdcf53b379816da0dac1b49a75b96d44ae1570  src/tugbot_navigation/config/nav2_params.yaml
ec9314d55ec018d64e1a294c381eebf4d5281f8eafddfc0771ec9907377ce866  src/tugbot_navigation/config/nav2_slam_params.yaml
dc5566f11e261cb4beb6ada6ee6dd45484ddd764c34c2e8c33625fdba2d92e8f  src/tugbot_navigation/config/slam_toolbox_params.yaml
```

Phase 4 修改 / 输出文件 hash：

```text
e1e978faa93eb97afcca884cda5a4f76e28beba16947828d8d2660d39d2eb6ea  src/tugbot_bringup/launch/tugbot_explore.launch.py
7a6ed580ca2d1e93dbf946f72ef1f3d383d4514c6da8f168512fd36319b8c924  src/tugbot_exploration/tugbot_exploration/frontier_explorer.py
13b60b8b1ba90e46aa633eb5bdaaf2b35d6a4cf1771af15ae3894b1da6fda75d  src/tugbot_bringup/test/test_contract.py
dd330915ab6b3f571b36b5da4e48c95769129bf2d722a8d0a7c9d25616eacf5c  README.md
b33e34f2fc7e9474af95b42a3d26c6dd79bd93599f3089bf1ec64eb19fcd80b0  src/tugbot_navigation/maps/explored/tugbot_nav_world_slam.yaml
a129c4aa65daba982a75a4eff145e061b6142da85524cf2090b1fe2ea974fe0b  src/tugbot_navigation/maps/explored/tugbot_nav_world_slam.pgm
```

## 4. colcon build 结果

执行命令：

```bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

结果：通过。

```text
Starting >>> tugbot_exploration
Starting >>> tugbot_description
Starting >>> tugbot_gazebo
Starting >>> tugbot_navigation
Finished <<< tugbot_gazebo [0.30s]
Finished <<< tugbot_navigation [0.30s]
Finished <<< tugbot_description [0.32s]
Finished <<< tugbot_exploration [1.14s]
Starting >>> tugbot_bringup
Finished <<< tugbot_bringup [0.11s]

Summary: 5 packages finished [1.41s]
```

## 5. py_compile / pytest / show-args 结果

执行：

```bash
source install/setup.bash
python3 -m py_compile src/tugbot_exploration/tugbot_exploration/frontier_explorer.py
python3 -m py_compile src/tugbot_bringup/launch/tugbot_explore.launch.py
python3 -m pytest -q src/tugbot_bringup/test/test_contract.py
ros2 launch tugbot_bringup tugbot_explore.launch.py --show-args
ros2 launch tugbot_bringup tugbot_slam_nav.launch.py --show-args
ros2 launch tugbot_bringup tugbot_slam.launch.py --show-args
ros2 launch tugbot_bringup tugbot_nav.launch.py --show-args
```

结果：通过。

```text
.......                                                                  [100%]
7 passed in 0.01s
  163 /tmp/phase4_explore_show_args.txt
  111 /tmp/phase4_nav_show_args.txt
   99 /tmp/phase4_slam_nav_show_args.txt
   95 /tmp/phase4_slam_show_args.txt
  468 total
PHASE4_STATIC_VERIFY_OK
```

## 6. 长时间探索启动命令

有效验证 session：`proc_f3e0b86b78a0`。

启动命令：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tugbot_bringup tugbot_explore.launch.py \
  headless:=true \
  use_rviz:=false \
  max_goals:=8 \
  save_map:=true \
  map_save_path:=/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam
```

本次探索没有达到 `max_goals=8`，而是在完成 5 个 goal 后因连续 5 次无有效 frontier 结束：

```text
Exploration complete: no valid frontier remained
```

这符合 Phase 4 边界：探索结束或达到 `max_goals` 后保存地图。

## 7. /map publisher 检查

检查命令：

```bash
ros2 topic info -v /map
```

结果：`/map` 由 `/slam_toolbox` 发布。

```text
Type: nav_msgs/msg/OccupancyGrid
Publisher count: 1
Node name: slam_toolbox
Node namespace: /
Endpoint type: PUBLISHER
Durability: TRANSIENT_LOCAL
```

`/map` 订阅者包括：

```text
/global_costmap
topic type: nav_msgs/msg/OccupancyGrid
/frontier_explorer
topic type: nav_msgs/msg/OccupancyGrid
/slam_toolbox
topic type: nav_msgs/msg/OccupancyGrid
```

## 8. AMCL / map_server 禁止项检查

检查命令：

```bash
ros2 node list | grep -E 'amcl|map_server' || true
```

结果：空。

同时 live 必要节点存在：

```text
/bt_navigator
/bt_navigator_navigate_through_poses_rclcpp_node
/bt_navigator_navigate_to_pose_rclcpp_node
/controller_server
/frontier_explorer
/planner_server
/slam_toolbox
```

结论：Phase 4 探索过程中未启动 AMCL，也未启动静态 map_server。

## 9. frontier goal 执行统计

`frontier_explorer` 启动参数：

```text
frontier_explorer started: map_topic=/map action_name=/navigate_to_pose min_obstacle_distance_m=0.45 min_goal_distance_m=0.50 max_goal_distance_m=4.00 max_goals=8 save_map=True map_save_path=/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam
```

目标执行统计：

```text
Goal #1 selected: x=1.752 y=-0.673 cluster_size=1717 distance=1.877
Goal #1 accepted, result success=True, completed_goals=1/8

Goal #2 selected: x=1.852 y=-3.573 cluster_size=2284 distance=3.005
Goal #2 accepted, result success=True, completed_goals=2/8

Goal #3 selected: x=5.852 y=-3.427 cluster_size=1244 distance=3.977
Goal #3 accepted, result success=True, completed_goals=3/8

Goal #4 selected: x=5.902 y=-0.288 cluster_size=660 distance=3.050
Goal #4 accepted, result success=True, completed_goals=4/8

Goal #5 selected: x=3.852 y=1.008 cluster_size=378 distance=2.449
Goal #5 accepted, result success=True, completed_goals=5/8
```

Action status 中 5 个 goal 状态均为 `status: 4`，对应 SUCCEEDED。

未观察到失败 goal 或 blacklist 增加：

```text
blacklist=0
```

结束原因：

```text
No valid frontier candidate (5/5 cycles)
Exploration complete: no valid frontier remained
```

结论：完成多个 frontier goal，达到 Phase 4 “至少 3 个”的基础验收要求。

## 10. /cmd_vel 输出证据

探索过程中 `controller_server` 收到 goal 并输出控制，bridge 也确认 ROS `/cmd_vel` 到 Gazebo 的消息桥接：

```text
[controller_server]: Received a goal, begin computing control effort.
[ros_gz_bridge]: Passing message from ROS geometry_msgs/msg/Twist to Gazebo gz.msgs.Twist (showing msg only once per type)
[controller_server]: Passing new path to controller.
[controller_server]: Reached the goal!
```

后置 probe 在探索已结束、机器人静止后采样，因此 `/cmd_vel` 样本为 0：

```text
PHASE4_POST_CMD_TOTAL 0
PHASE4_POST_CMD_NONZERO 0
```

解释：该后置 probe 采样发生在 5 个 goal 已完成、探索已结束并保存地图之后，不能代表运动期间。运动期间证据以 `controller_server` 和 bridge 日志为准。

## 11. /odom 位移证据

最终 odom：

```text
x: 3.1685166336488364
y: 0.8212420795833277
z: 0.0
```

首个 goal 从初始位置附近开始：

```text
Begin navigating from current location (0.00, -0.00) to (1.75, -0.67)
```

后续 goal 起点显示机器人已移动：

```text
Begin navigating from current location (1.72, -0.57) to (1.85, -3.57)
Begin navigating from current location (1.88, -3.57) to (5.85, -3.43)
Begin navigating from current location (5.81, -3.34) to (5.90, -0.29)
Begin navigating from current location (5.93, -0.29) to (3.85, 1.01)
```

结论：Tugbot 有明显位移，且完成了 5 段自主导航。

## 12. /map 初始与结束统计

frontier_explorer 日志中的地图尺寸变化：

```text
初始：Map 315x154 res=0.050, frontier_clusters=266, valid_candidates=17, goals=0/8
中间：Map 315x155 res=0.050, frontier_clusters=59,  valid_candidates=10, goals=2/8
中间：Map 315x156 res=0.050, frontier_clusters=80,  valid_candidates=6,  goals=4/8
结束：Map 315x156 res=0.050, frontier_clusters=66,  valid_candidates=0,  goals=5/8
```

后置 `/map` 统计：

```text
PHASE4_POST_MAP_COUNT 6
PHASE4_POST_MAP_FIRST {'stamp': (432, 300000000), 'width': 315, 'height': 156, 'unknown': 21914, 'free': 26024, 'occupied': 1202}
PHASE4_POST_MAP_LAST  {'stamp': (442, 302000000), 'width': 315, 'height': 156, 'unknown': 21914, 'free': 26024, 'occupied': 1202}
```

解释：

- live 探索期间 `/map` 高度从 154 增长到 156；
- frontier cluster 数量从 266 下降到 66；
- valid candidates 从 17 下降到 0；
- 后置 probe 发生在探索完成后，地图内容已稳定，但 stamp 仍在推进；
- 这证明本次探索确实推动 SLAM 地图覆盖扩大并最终收敛到无有效 frontier。

## 13. 地图保存命令与结果

frontier_explorer 自动执行保存命令：

```bash
ros2 run nav2_map_server map_saver_cli -f /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam
```

返回码：

```text
Map save returncode=0
```

map_saver 输出：

```text
Saving map from 'map' topic to '/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam' file
Received a 315 X 156 map @ 0.05 m/pix
Writing map occupancy data to .../tugbot_nav_world_slam.pgm
Writing map metadata to .../tugbot_nav_world_slam.yaml
Map saved
Map saved successfully
```

frontier_explorer 校验：

```text
Map save succeeded: yaml=/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam.yaml bytes=143 pgm=/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam.pgm bytes=49155
```

## 14. .yaml / .pgm 文件检查

目录检查：

```text
total 56K
-rw-rw-r-- 1 hyh hyh 49K May 14 00:52 tugbot_nav_world_slam.pgm
-rw-rw-r-- 1 hyh hyh 143 May 14 00:52 tugbot_nav_world_slam.yaml
```

YAML 内容：

```yaml
image: tugbot_nav_world_slam.pgm
mode: trinary
resolution: 0.050
origin: [-8.673, -5.474, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

PGM 文件类型：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam.pgm: Netpbm image data, size = 315 x 156, rawbits, greymap
```

结论：`.yaml + .pgm` 已成功生成，文件非空，yaml 引用 pgm，resolution/origin/image 字段合理，可作为后续 Nav2 静态地图输入候选。

## 15. README 更新摘要

`README.md` 已更新以下内容：

1. 0514 工程定位：
   - 继承 0513 静态地图导航基线；
   - 新增 SLAM 在线建图；
   - 新增 SLAM + Nav2 手动导航；
   - 新增 frontier 自主探索建图；
   - 新增地图保存。

2. 启动入口：
   - 静态地图导航：`ros2 launch tugbot_bringup tugbot_nav.launch.py`；
   - SLAM 在线建图：`ros2 launch tugbot_bringup tugbot_slam.launch.py`；
   - SLAM + Nav2 手动导航：`ros2 launch tugbot_bringup tugbot_slam_nav.launch.py`；
   - frontier 自主探索建图：`ros2 launch tugbot_bringup tugbot_explore.launch.py`；
   - frontier 探索并保存地图的完整 headless 命令。

3. 验收标准：
   - 不加载旧静态地图；
   - `/map` 由 slam_toolbox 发布；
   - AMCL/map_server 不启动；
   - frontier_explorer 自动选择目标；
   - Nav2 自动执行目标；
   - `/cmd_vel` 有输出；
   - `/odom` 有位移；
   - `/map` 持续更新；
   - 最终生成 `.yaml + .pgm`；
   - 保存地图和 Gazebo 真实环境轮廓目视基本吻合。

## 16. 进程清理结果

Phase 4 live 验证结束后，已停止 launch session 并确认无残留进程：

```text
--- post Phase4 cleanup process check ---
<empty>

--- post Phase4 cleanup ROS graph check ---
<empty>
```

地图文件仍保留：

```text
--- map files still present ---
total 56K
-rw-rw-r-- 1 hyh hyh 49K May 14 00:52 tugbot_nav_world_slam.pgm
-rw-rw-r-- 1 hyh hyh 143 May 14 00:52 tugbot_nav_world_slam.yaml
```

## 17. 当前遗留问题

1. 本阶段未做复杂地图相似度算法。

   这是按 Phase 4 边界刻意保留的限制，当前仅做基础工程验收和文件检查。

2. 后置 `/cmd_vel` probe 没有采到非零速度。

   原因是 probe 运行时探索已结束、机器人已静止。运动期间证据来自 controller / bridge / action 日志与 odom 位置变化。

3. 本次 run 未达到 `max_goals=8`。

   实际完成 5 个 goal 后，连续 5 次无有效 frontier，按 `finish_no_frontier_cycles=5` 正常结束并保存地图。该行为符合“探索结束或达到 max_goals 后保存地图”的验收定义。

4. 保存地图仍需人工目视确认与 Gazebo 真实环境轮廓基本吻合。

   已生成可查看的 `.pgm` 与 `.yaml`，但本阶段不做自动相似度评价。

## 18. Phase 4 验收结论

Phase 4 基础工程验收：通过。

逐项结论：

- 小车能自动执行多个 frontier goal：通过，完成 5 个 goal；
- 地图覆盖范围明显扩大：通过，地图尺寸从 315x154 增至 315x156，frontier candidates 从 17 收敛到 0；
- 探索结束后保存地图：通过，因无有效 frontier 结束后自动保存；
- 输出地图文件可被后续 Nav2 静态地图导航使用：基础文件条件通过，`.yaml + .pgm` 存在且字段合理；
- README 更新当前 0514 工程启动与验收方法：通过；
- `/map` 仍由 slam_toolbox 发布：通过；
- `/amcl` 不存在：通过；
- 静态 `map_server` 不存在：通过；
- 不破坏 Phase 0/1/2/3 入口：通过，冻结文件 hash 已记录。

按用户要求，Phase 4 到此停止，不进入后续阶段，等待人工验收。
