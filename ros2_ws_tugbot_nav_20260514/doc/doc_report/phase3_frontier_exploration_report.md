# Phase 3 Report - Frontier 自主探索建图功能

## 1. 阶段目标

在 0514 工作空间 Phase 2 已验证通过的 `SLAM + Nav2 手动导航` 基础上，新增 frontier 自主探索节点与启动入口：

```bash
ros2 launch tugbot_bringup tugbot_explore.launch.py
```

本阶段目标是跑通第一版闭环：

- `/map` 由 `slam_toolbox` 实时发布；
- `frontier_explorer` 订阅 `/map`；
- 根据 unknown / free 边界识别 frontier；
- 聚类并选择安全目标；
- 通过 Nav2 `/navigate_to_pose` action 自动发送目标；
- 小车能自动执行至少 1 个 goal，并在 `max_goals:=3` 下尝试多个 goal；
- 本阶段不做复杂地图质量对比，不进入地图保存 Phase 4。

## 2. 新增 / 修改文件清单

### 新增文件

1. `src/tugbot_exploration/package.xml`

   新增 Python 包 `tugbot_exploration`。

2. `src/tugbot_exploration/setup.py`

   注册 console script：

   ```text
   frontier_explorer = tugbot_exploration.frontier_explorer:main
   ```

3. `src/tugbot_exploration/setup.cfg`

   设置 ROS 2 Python executable 安装路径。

4. `src/tugbot_exploration/resource/tugbot_exploration`

   ament resource marker。

5. `src/tugbot_exploration/tugbot_exploration/__init__.py`

   Python package marker。

6. `src/tugbot_exploration/tugbot_exploration/frontier_explorer.py`

   Frontier 自主探索节点。

7. `src/tugbot_bringup/launch/tugbot_explore.launch.py`

   Phase 3 自主探索启动入口。

### 修改文件

1. `src/tugbot_bringup/package.xml`

   新增 runtime 依赖：

   ```xml
   <exec_depend>tugbot_exploration</exec_depend>
   ```

2. `src/tugbot_bringup/test/test_contract.py`

   新增 `test_frontier_exploration_contract()`，覆盖：

   - `tugbot_exploration` 包结构；
   - `frontier_explorer` 必要 ROS 2 依赖、OccupancyGrid、NavigateToPose、TF、frontier 分类逻辑；
   - `tugbot_explore.launch.py` 复用 `tugbot_slam_nav.launch.py`；
   - 不引用旧静态地图；
   - 不使用 `bringup_launch.py`；
   - 不传入静态 `map` launch argument。

## 3. 是否保留 Phase 0 / 1 / 2 入口不变

结论：保留。

Phase 3 未修改以下文件：

```text
656397f327b6abf55c394613639d949efe1fe9fb71729634034d5e26ed16c184  src/tugbot_bringup/launch/tugbot_nav.launch.py
0b2387de5272fb702dbadc97e244e1247ae0dc0798c467fe6ebe2aafd2c24f77  src/tugbot_bringup/launch/tugbot_slam.launch.py
1f20010cd9f5082f08e800849d81a8d2c74dd14de135c8e8620780958f08c099  src/tugbot_bringup/launch/tugbot_slam_nav.launch.py
65e22ee9021ede7416720ed3e6fdcf53b379816da0dac1b49a75b96d44ae1570  src/tugbot_navigation/config/nav2_params.yaml
ec9314d55ec018d64e1a294c381eebf4d5281f8eafddfc0771ec9907377ce866  src/tugbot_navigation/config/nav2_slam_params.yaml
dc5566f11e261cb4beb6ada6ee6dd45484ddd764c34c2e8c33625fdba2d92e8f  src/tugbot_navigation/config/slam_toolbox_params.yaml
```

新增文件 hash：

```text
ea6b99381100621e7f388ec7b92293b0a5014db07c574ff74a11c6d209537620  src/tugbot_bringup/launch/tugbot_explore.launch.py
16cfb05ed1f2cfc01f7f2572ca6f0186719471f99338d6e4d9746494f66cf14b  src/tugbot_exploration/tugbot_exploration/frontier_explorer.py
```

## 4. frontier_explorer 节点设计

节点名：

```text
/frontier_explorer
```

核心订阅 / 依赖：

- 订阅 `/map`，类型 `nav_msgs/msg/OccupancyGrid`；
- 使用 `tf2_ros` 查询 `map -> base_link`；
- 使用 `nav2_msgs/action/NavigateToPose` action client 连接 `/navigate_to_pose`；
- 不直接发布 `/cmd_vel`，运动全部交给 Nav2。

Frontier 分类：

```python
unknown cell: value == -1
free cell:    value == 0
occupied:     value > 50
frontier:     free cell 邻近至少一个 unknown cell
```

第一版筛选逻辑：

- 8 邻域聚类 frontier cells；
- 默认 `min_cluster_size:=5`；
- 目标点必须来自 known free cell；
- 距离 occupied cell 默认至少 `min_obstacle_distance_m:=0.45`；
- 距离机器人默认 `[0.5, 4.0]` m；
- goal 失败后按 `blacklist_radius_m:=0.5` 拉黑；
- 默认 `max_goals:=5`，本次 live 验证使用 `max_goals:=3`。

## 5. 构建结果

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
Finished <<< tugbot_gazebo [0.28s]
Finished <<< tugbot_description [0.29s]
Finished <<< tugbot_navigation [0.29s]
Finished <<< tugbot_exploration [1.10s]
Starting >>> tugbot_bringup
Finished <<< tugbot_bringup [0.12s]

Summary: 5 packages finished [1.35s]
```

修复 `FrontierCandidate(cluster=...)` 字段名错误后，重新构建受影响包：

```text
Starting >>> tugbot_exploration
Finished <<< tugbot_exploration [1.44s]

Summary: 1 package finished [1.59s]
```

## 6. py_compile / pytest 结果

执行：

```bash
python3 -m py_compile src/tugbot_exploration/tugbot_exploration/frontier_explorer.py
python3 -m py_compile src/tugbot_bringup/launch/tugbot_explore.launch.py
python3 -m pytest -q src/tugbot_bringup/test/test_contract.py
```

结果：通过。

```text
......                                                                   [100%]
6 passed in 0.01s
```

Package discovery：

```text
/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/install/tugbot_exploration
```

Launch show-args 检查通过：

```text
  151 /tmp/phase3_explore_show_args.txt
   99 /tmp/phase3_slam_nav_show_args.txt
   95 /tmp/phase3_slam_show_args.txt
  111 /tmp/phase3_nav_show_args.txt
```

## 7. tugbot_explore.launch.py 启动结果

执行：

```bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tugbot_bringup tugbot_explore.launch.py headless:=true use_rviz:=false max_goals:=3
```

有效验证 run：`proc_978c22b074d5`。

启动成功，关键节点包括：

```text
/bt_navigator
/bt_navigator_navigate_through_poses_rclcpp_node
/bt_navigator_navigate_to_pose_rclcpp_node
/controller_server
/frontier_explorer
/planner_server
/slam_toolbox
```

关键 topic 包括：

```text
/cmd_vel
/cmd_vel_nav
/cmd_vel_smoothed
/cmd_vel_teleop
/map
/map_metadata
/odom
/scan
/slam_toolbox/scan_visualization
/tf
/tf_static
```

## 8. frontier_explorer 节点状态

节点成功启动：

```text
[frontier_explorer]: frontier_explorer started: map_topic=/map action_name=/navigate_to_pose min_obstacle_distance_m=0.45 min_goal_distance_m=0.50 max_goal_distance_m=4.00 max_goals=3
```

`/map` subscription 确认：

```text
Node name: frontier_explorer
Node namespace: /
Topic type: nav_msgs/msg/OccupancyGrid
Endpoint type: SUBSCRIPTION
Reliability: RELIABLE
Durability: VOLATILE
```

`/navigate_to_pose` action server 存在：

```text
Action: /navigate_to_pose
Action servers: 1
    /bt_navigator
```

## 9. /map publisher 检查

结论：`/map` 仍由 `slam_toolbox` 发布。

```text
Type: nav_msgs/msg/OccupancyGrid
Publisher count: 1

Node name: slam_toolbox
Node namespace: /
Endpoint type: PUBLISHER
Durability: TRANSIENT_LOCAL
```

同时确认 `/global_costmap/global_costmap` 与 `/frontier_explorer` 都订阅 `/map`。

## 10. AMCL / map_server 禁止项检查

执行：

```bash
ros2 node list | grep -E 'amcl|map_server' || true
```

结果为空。

结论：

- `/amcl` 不存在；
- 静态 `map_server` 不存在；
- 旧静态地图未加载；
- `/map` 来源仍为 `slam_toolbox`。

## 11. Frontier cluster 识别结果

有效 live run 中，frontier_explorer 持续输出地图尺寸、cluster 数量和候选目标数量。

第一次选点前：

```text
Map 315x154 res=0.050, frontier_clusters=266, valid_candidates=17, blacklist=0, goals=0/3
Selected frontier goal #1: x=1.752 y=-0.673 cluster_size=1717 distance=1.877 score=1716.253
```

第二次选点前：

```text
Map 315x155 res=0.050, frontier_clusters=126, valid_candidates=12, blacklist=0, goals=1/3
Selected frontier goal #2: x=1.852 y=-3.528 cluster_size=2443 distance=2.948 score=2441.605
```

第三次选点前：

```text
Map 315x155 res=0.050, frontier_clusters=76, valid_candidates=14, blacklist=0, goals=2/3
Selected frontier goal #3: x=5.852 y=-3.078 cluster_size=1000 distance=3.989 score=996.522
```

结论：frontier cluster 识别与多轮目标选择已跑通。

## 12. 自动 goal 发送结果

### Goal #1

```text
Selected frontier goal #1: x=1.752 y=-0.673 cluster_size=1717 distance=1.877 score=1716.253
NavigateToPose goal sent: x=1.752 y=-0.673
Begin navigating from current location (0.00, -0.00) to (1.75, -0.67)
NavigateToPose goal accepted
Reached the goal!
Goal succeeded
NavigateToPose result: status=4 error_code=0 error_msg= success=True
Goal succeeded; completed_goals=1/3
```

### Goal #2

```text
Selected frontier goal #2: x=1.852 y=-3.528 cluster_size=2443 distance=2.948 score=2441.605
NavigateToPose goal sent: x=1.852 y=-3.528
Begin navigating from current location (1.74, -0.58) to (1.85, -3.53)
NavigateToPose goal accepted
Reached the goal!
Goal succeeded
NavigateToPose result: status=4 error_code=0 error_msg= success=True
Goal succeeded; completed_goals=2/3
```

### Goal #3

```text
Selected frontier goal #3: x=5.852 y=-3.078 cluster_size=1000 distance=3.989 score=996.522
NavigateToPose goal sent: x=5.852 y=-3.078
Begin navigating from current location (1.89, -3.53) to (5.85, -3.08)
NavigateToPose goal accepted
```

第三个 goal 初期多次出现 planner 失败：

```text
GridBased plugin failed to plan from (1.89, -3.53) to (5.85, -3.08): "Failed to create plan with tolerance of: 0.500000"
[compute_path_to_pose] [ActionServer] Aborting handle.
```

随后 Nav2 recovery 后进入 controller：

```text
Running spin
wait completed successfully
Running backup
backup completed successfully
Received a goal, begin computing control effort.
NavigateToPose feedback: distance_remaining=4.171 recoveries=9
```

Action status topic 最终显示 3 个 goal 都进入 `status: 4`：

```text
status_list:
- ... status: 4
- ... status: 4
- ... status: 4
```

注：ROS 2 action status `4` 表示 `STATUS_SUCCEEDED`。

## 13. /cmd_vel 是否输出

结论：有输出。

第一次 goal 执行期间，bridge 记录 ROS -> Gazebo `/cmd_vel`：

```text
[ros_gz_bridge]: Passing message from ROS geometry_msgs/msg/Twist to Gazebo gz.msgs.Twist
```

控制器也持续收到并执行路径：

```text
[controller_server]: Received a goal, begin computing control effort.
[controller_server]: Passing new path to controller.
```

补充说明：后续我在第三 goal 长尾阶段采样 12 秒时，`/cmd_vel` 刚好为 0 条，因为当时动作已基本进入收束 / 静止状态；但前面 goal 生命周期日志已经证明 `/cmd_vel` 曾经通过 bridge 下发。

## 14. /odom 是否变化

结论：发生明显位移。

Goal #1 开始：

```text
Begin navigating from current location (0.00, -0.00) to (1.75, -0.67)
```

Goal #2 开始：

```text
Begin navigating from current location (1.74, -0.58) to (1.85, -3.53)
```

Goal #3 开始：

```text
Begin navigating from current location (1.89, -3.53) to (5.85, -3.08)
```

后续 `/odom` 采样位置：

```text
x: 5.124563869902938
y: -1.9348276953719186
z: 0.0
```

因此小车至少从原点移动到多个 frontier 目标附近，位移证据成立。

## 15. /map 是否持续更新

结论：持续更新。

启动时地图尺寸：

```text
Map 315x154 res=0.050
```

Goal #1 后地图尺寸扩展：

```text
StaticLayer: Resizing costmap to 315 X 155 at 0.050000 m/pix
Map 315x155 res=0.050
```

后续 map timestamp 采样：

```text
MAP_SAMPLE_COUNT 3
MAP_FIRST (498, 600000000, 315, 155)
MAP_LAST  (502, 602000000, 315, 155)
```

结论：`/map` topic 持续由 slam_toolbox 发布新消息，且探索过程中尺寸从 `315x154` 扩展到 `315x155`。

## 16. max_goals:=3 测试结果

本次 `max_goals:=3` 验证结果：

- 自动识别 frontier cluster：通过；
- 自动发送 goal：通过，发送 3 个；
- 至少 1 个完整 goal 生命周期：通过；
- 多个 goal 生命周期：通过，Goal #1、Goal #2 成功完成；
- Goal #3：被发送并接受，期间出现 planner 失败与 recovery，后续 action status 显示 `status: 4`；
- `/cmd_vel`：有输出；
- `/odom`：明显变化；
- `/map`：持续更新；
- AMCL/map_server：不存在。

## 17. 调试记录

本阶段出现过两个旧 run 的失败，它们已经修复并清理：

1. `proc_84ccdfe4e93a`
2. `proc_2477726f667b`

旧失败根因均为：

```text
TypeError: FrontierCandidate.__init__() got an unexpected keyword argument 'cluster'
```

修复：

```python
FrontierCandidate(cells=cluster, ...)
```

修复后重新 build / py_compile / pytest，并以 `proc_978c22b074d5` 作为有效 live 验证 run。

注意：上述两个旧 run 的 watch pattern 后续可能继续延迟通知，例如 `frontier_explorer started`。它们不是当前 live 状态，不应作为当前失败判断依据。

## 18. 当前遗留问题

1. Goal #3 目标接近 `max_goal_distance_m:=4.0` 上限，规划初期出现多次：

   ```text
   GridBased plugin failed to plan ... Failed to create plan with tolerance of: 0.500000
   ```

   Nav2 recovery 后仍进入 controller，并且 action status 最终为 `4`。第一版闭环已满足 Phase 3，但后续可在 Phase 4 或后续调优中优化目标可达性评分。

2. `frontier_explorer` 当前第一版安全性是基于 occupancy grid 的静态距离过滤，尚未引入 path feasibility precheck 或 costmap service 检查。这符合本阶段“不一次性引入复杂地图对比算法 / 先跑通闭环”的约束。

3. global costmap 仍保留 Phase 2 已知 warning：

   ```text
   configured inflation radius (0.350) is smaller than computed inscribed radius (0.353)
   ```

   本阶段未修改 `nav2_slam_params.yaml`，避免破坏 Phase 2 已验收基线。

## 19. 清理确认

验证结束后已停止 launch 进程，并检查无残留：

```text
--- final cleanup check ---
<empty>
```

ROS graph 检查也为空：

```text
ros2 node list
<empty>
```

## 20. Phase 3 验收结论

结论：Phase 3 第一版 frontier 自主探索闭环通过。

满足项：

- 新增 `tugbot_exploration` 包；
- 新增 `frontier_explorer.py`；
- 新增 `tugbot_explore.launch.py`；
- 原 `tugbot_nav.launch.py`、`tugbot_slam.launch.py`、`tugbot_slam_nav.launch.py` 保持不变；
- 原 `nav2_params.yaml` 保持不变；
- `/map` 由 `slam_toolbox` 实时发布；
- `/amcl` 不存在；
- 静态 `map_server` 不存在；
- `frontier_explorer` 能订阅 `/map`；
- `frontier_explorer` 能识别 frontier cluster；
- `frontier_explorer` 能自动发送多个 NavigateToPose goal；
- Nav2 能执行 goal；
- `/cmd_vel` 有输出；
- `/odom` 有明显位移；
- `/map` 持续更新并有尺寸扩展；
- `max_goals:=3` 验证完成；
- 进程已清理；
- 按要求停止，等待人工验收，不进入 Phase 4。
