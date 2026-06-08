# 迷宫自主探索 + 死胡同回退 Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task. 这是一个计划文件，不在本阶段改业务代码。

**Goal:** 让 Tugbot 从迷宫入口出发，在不知道完整迷宫拓扑的前提下用 SLAM 建图，自主探索路线；遇到死胡同能标记并回退到上一分岔口，选择另一条未完全探索路线，最终到达已知出口坐标附近。

**Architecture:** 新增一个真正的 `tugbot_maze.maze_explorer` 节点，替换当前 placeholder。该节点订阅 `/map`、读取 TF 中机器人位姿，维护一个拓扑探索状态机：入口、分岔口、通道、死胡同、已尝试边、待探索出口。运动执行仍交给 Nav2 的 `/navigate_to_pose` action；SLAM、Gazebo、Nav2 launch 链保持现有可工作的 baseline。出口坐标用于成功判定和候选分支排序偏置，但不能用预生成全图或最短路径作弊。

**Tech Stack:** ROS 2 Jazzy, Gazebo `gz sim`, `slam_toolbox`, Nav2 `NavigateToPose`, Python `rclpy`, `nav_msgs/OccupancyGrid`, `tf2_ros`, pytest 合约/单元测试。

---

## 0. 当前上下文

工作空间：

`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

已有基础能力：

- `tugbot_maze_slam.launch.py`：Gazebo + SLAM 已通过 headless smoke。
- `tugbot_maze_slam_nav.launch.py`：Gazebo + SLAM + Nav2 launch 已存在。
- `tugbot_maze_explore.launch.py`：当前启动的是继承来的 `tugbot_exploration/frontier_explorer`，并启动 `maze_goal_monitor`。
- `src/tugbot_maze/tugbot_maze/maze_explorer.py` 当前只是 placeholder。
- `src/tugbot_maze/tugbot_maze/maze_goal_monitor.py` 已能根据出口坐标发布 `/maze/exit_reached` 并输出 `MAZE_EXIT_REACHED`。
- 当前迷宫通道宽度约束已测试：通道宽度至少为 Tugbot 自转外直径两倍。

关键约束：

- 必须先 source：`. /opt/ros/jazzy/setup.sh`
- 构建后运行还需 source：`. install/setup.sh`
- 不使用旧 `gazebo` 命令，使用 `gz sim`。
- Runtime explorer 不能使用完整 Gazebo world 几何直接规划最短路；只能用 `/scan`、`/odom`、`/tf`、实时 `/map`、Nav2 feedback 和已知的入口/出口坐标。

---

## 1. 成功标准 / 验收标准

### 1.1 功能验收

1. 小车从入口坐标附近启动。
2. 小车在 `/map` 尚不完整时能持续选择新的探索目标。
3. 小车能识别当前通道没有新的可探索方向时，将该位置/通道标记为 dead-end。
4. 小车能导航回最近的、仍有未探索方向的分岔口。
5. 分岔口的每条可行方向最多被标记为：`untried`、`in_progress`、`explored`、`dead_end`、`blocked` 之一。
6. 小车到达出口半径内时，`maze_goal_monitor` 输出 `MAZE_EXIT_REACHED`，探索节点停止继续派发目标。
7. 若所有可达分支探索完仍未到达出口，节点明确输出失败原因，而不是无限乱走。

### 1.2 工程验收

1. Python 单元测试覆盖拓扑图状态、死胡同标记、回退栈、候选排序。
2. 合约测试确认 `tugbot_maze_explore.launch.py` 默认启动新的 `tugbot_maze/maze_explorer`，不再默认启动旧 `frontier_explorer`。
3. `python3 -m py_compile` 通过。
4. `python3 -m pytest -q src/tugbot_maze/test src/tugbot_bringup/test` 通过。
5. `rosdep check --from-paths src --ignore-src -r --rosdistro jazzy` 通过。
6. `colcon build --symlink-install --event-handlers console_direct+` 通过。
7. headless exploration smoke 中能观察到：`/maze/exit_reached`、`/maze/explorer_state`、`/maze/dead_ends` 或等价状态 topic/log。
8. 至少一次完整仿真在限定时间内到达出口。

---

## 2. 推荐总方案

### 2.1 为什么不用纯 frontier_explorer

现有 `frontier_explorer` 适合“覆盖未知区域”，但用户目标是“迷宫式分岔搜索 + 死胡同标记 + 回退到上一分岔口”。这需要显式拓扑记忆，而不只是每次选择一个 frontier goal。

### 2.2 采用“拓扑 DFS + Nav2 执行”的方案

核心思想：

1. 从 live occupancy grid 中提取局部可通行方向。
2. 把入口、分岔口、死胡同、出口附近点记录为 graph nodes。
3. 两个 graph node 之间的通道记录为 graph edges。
4. 用 DFS/Trémaux 风格策略：
   - 在分岔口优先选择 `untried` 方向。
   - 若该方向最终无路可走，标记为 `dead_end`。
   - 回退到最近还有 `untried` 方向的分岔口。
   - 若某个方向朝出口更近，可提高优先级，但不能跳过未知区域直接规划。
5. 每次实际移动目标用 Nav2 `NavigateToPose` 执行；探索节点只负责“选择下一个局部导航目标”。

### 2.3 分阶段策略

先在不启动 Gazebo 的纯 Python 层把探索状态机测稳，再接 Nav2 action，最后做 headless Gazebo 集成。避免一上来长时间调仿真。

---

## 3. 计划分层

Level A：基础数据结构和纯算法。

Level B：从 `/map` 提取候选分支/死胡同。

Level C：实现 ROS 2 `maze_explorer` 状态机 + Nav2 action。

Level D：替换 launch 默认 explorer，保留旧 frontier explorer 作为 fallback。

Level E：headless 仿真验证、调参、记录结果。

Level F：文档、回归测试和可选增强。

---

## 4. 详细任务清单

### Phase 1：建立纯 Python 拓扑模型

#### Task 1.1 创建探索状态数据结构

**Objective:** 定义分岔点、边、方向状态、探索状态，先不接 ROS。

**Files:**

- Create: `src/tugbot_maze/tugbot_maze/maze_topology.py`
- Create: `src/tugbot_maze/test/test_maze_topology.py`

**Steps:**

1. 写 pytest：创建一个起点 node，添加三个 branch directions，断言初始状态都是 `untried`。
2. 实现 enum/string 常量：`UNTRIED`、`IN_PROGRESS`、`EXPLORED`、`DEAD_END`、`BLOCKED`。
3. 实现 dataclass：`TopoNode`、`TopoEdge`、`BranchOption`、`MazeTopology`。
4. 运行：
   `. /opt/ros/jazzy/setup.sh && python3 -m pytest -q src/tugbot_maze/test/test_maze_topology.py`

#### Task 1.2 实现最近节点匹配和节点去重

**Objective:** 防止机器人在同一个分岔口附近重复创建多个节点。

**Files:**

- Modify: `src/tugbot_maze/tugbot_maze/maze_topology.py`
- Modify: `src/tugbot_maze/test/test_maze_topology.py`

**Steps:**

1. 测试：两个距离小于 `junction_merge_radius_m` 的 node 应合并/复用。
2. 测试：距离超过阈值时创建新 node。
3. 实现 `find_or_create_node(x, y, node_type)`。
4. 运行单测。

#### Task 1.3 实现 DFS 回退栈

**Objective:** 在遇到死胡同时能找到最近的未完全探索分岔口。

**Files:**

- Modify: `src/tugbot_maze/tugbot_maze/maze_topology.py`
- Modify: `src/tugbot_maze/test/test_maze_topology.py`

**Steps:**

1. 测试：A -> B -> C 是死胡同，C dead-end 后应返回 B 或 A 中最近仍有 `untried` branch 的节点。
2. 实现 `mark_dead_end(edge_or_node)`。
3. 实现 `next_backtrack_target(current_node)`。
4. 运行单测。

#### Task 1.4 实现候选分支排序

**Objective:** 已知出口坐标参与排序，但不允许当作全图路径答案。

**Files:**

- Modify: `src/tugbot_maze/tugbot_maze/maze_topology.py`
- Modify: `src/tugbot_maze/test/test_maze_topology.py`

**Steps:**

1. 测试：多个 `untried` branch 中，朝出口方向更近的分支 score 更高。
2. 测试：已 `dead_end` 的分支永不被选中。
3. 实现 `choose_next_branch(current_node, exit_xy)`。
4. 运行单测。

---

### Phase 2：从 OccupancyGrid 提取迷宫局部结构

#### Task 2.1 创建 occupancy grid 工具函数

**Objective:** 把 `/map` 转成可查询的栅格工具。

**Files:**

- Create: `src/tugbot_maze/tugbot_maze/grid_utils.py`
- Create: `src/tugbot_maze/test/test_grid_utils.py`

**Steps:**

1. 测试 world 坐标到 grid cell 的转换。
2. 测试 grid cell 到 world 坐标的转换。
3. 测试 occupied/free/unknown 判断。
4. 实现 `world_to_cell`、`cell_to_world`、`cell_value`、`is_free`、`is_occupied`、`is_unknown`。
5. 运行单测。

#### Task 2.2 实现安全膨胀/通行性判断

**Objective:** 考虑机器人半径和墙体安全距离，避免目标太贴墙。

**Files:**

- Modify: `src/tugbot_maze/tugbot_maze/grid_utils.py`
- Modify: `src/tugbot_maze/test/test_grid_utils.py`

**Steps:**

1. 测试障碍物周围 `clearance_radius_m` 内不能作为目标。
2. 测试宽通道中央可作为目标。
3. 实现 `has_clearance(cell, radius_m)`。
4. 运行单测。

#### Task 2.3 实现射线/扇区方向采样

**Objective:** 从机器人或分岔点周围识别“前后左右/若干角度”的可通行方向。

**Files:**

- Modify: `src/tugbot_maze/tugbot_maze/grid_utils.py`
- Create: `src/tugbot_maze/test/test_branch_detection.py`

**Steps:**

1. 构造一个 T 字路口小地图。
2. 测试检测到 3 个开放方向。
3. 构造一个死胡同小地图。
4. 测试只检测到 1 个回退方向，且没有新的未探索方向。
5. 实现 `sample_open_directions(map, pose, angle_step_deg, lookahead_m)`。
6. 运行单测。

#### Task 2.4 实现分岔口/死胡同分类

**Objective:** 根据开放方向数量和已来方向判断当前位置类型。

**Files:**

- Create: `src/tugbot_maze/tugbot_maze/maze_perception.py`
- Modify: `src/tugbot_maze/test/test_branch_detection.py`

**Steps:**

1. 测试直通通道：不创建 junction。
2. 测试 T 字路口：创建 junction。
3. 测试十字路口：创建 junction。
4. 测试 dead-end：创建 dead_end node 并触发回退。
5. 实现 `classify_local_topology(map, pose, previous_heading)`。
6. 运行单测。

---

### Phase 3：实现真正的 ROS 2 maze_explorer 节点

#### Task 3.1 替换 placeholder 节点骨架

**Objective:** `maze_explorer.py` 从 placeholder 变为可运行的状态机节点，但先不发目标。

**Files:**

- Modify: `src/tugbot_maze/tugbot_maze/maze_explorer.py`
- Modify: `src/tugbot_maze/test/test_tugbot_maze_contract.py`

**Steps:**

1. 合约测试：文件中不再包含 “placeholder extension point”。
2. 合约测试：节点声明关键参数：`entrance_x`、`entrance_y`、`exit_x`、`exit_y`、`map_topic`、`action_name`。
3. 实现参数声明和基础 logger。
4. 运行合约测试。

#### Task 3.2 订阅地图并读取 TF 位姿

**Objective:** 节点能获得 live `/map` 和 `map -> base_link`。

**Files:**

- Modify: `src/tugbot_maze/tugbot_maze/maze_explorer.py`

**Steps:**

1. 增加 `/map` subscription，QoS 使用 transient local/reliable，参考现有 `frontier_explorer`。
2. 增加 `tf2_ros.Buffer` 和 `TransformListener`。
3. 实现 `_lookup_robot_pose()`。
4. 本阶段用 py_compile 验证。

#### Task 3.3 发布探索状态 topic

**Objective:** 方便 headless 验证探索是否按 DFS 运行。

**Files:**

- Modify: `src/tugbot_maze/tugbot_maze/maze_explorer.py`
- Modify: `src/tugbot_maze/package.xml`

**Steps:**

1. 选择简单消息类型：优先用 `std_msgs/String` 发布 JSON 状态到 `/maze/explorer_state`。
2. 发布内容包含：`mode`、`current_node_id`、`goal_kind`、`known_junctions`、`dead_ends`、`backtrack_target`、`exit_distance_m`。
3. 若 package.xml 已有 `std_msgs`，无需新增依赖。
4. py_compile 验证。

#### Task 3.4 接入 NavigateToPose action client

**Objective:** 节点能向 Nav2 派发目标，并处理成功/失败/超时。

**Files:**

- Modify: `src/tugbot_maze/tugbot_maze/maze_explorer.py`

**Steps:**

1. 增加 `ActionClient(self, NavigateToPose, action_name)`。
2. 实现 `_send_nav_goal(x, y, yaw, goal_kind)`。
3. 实现 goal accepted/rejected callback。
4. 实现 result callback：成功则更新 edge 状态，失败则按 `blocked` 或 `dead_end` 处理。
5. 增加 `goal_timeout_sec` 参数。
6. py_compile 验证。

#### Task 3.5 实现主状态机 tick

**Objective:** 节点周期性决定下一步动作。

**Files:**

- Modify: `src/tugbot_maze/tugbot_maze/maze_explorer.py`

**States:**

- `WAIT_FOR_MAP`
- `WAIT_FOR_NAV2`
- `AT_NODE_ANALYZE`
- `SEND_EXPLORE_GOAL`
- `NAVIGATING`
- `MARK_DEAD_END`
- `BACKTRACKING`
- `EXIT_REACHED`
- `FAILED_EXHAUSTED`

**Steps:**

1. 实现 timer。
2. 无 map/TF/action server 时等待。
3. 到达出口半径内时停止并 log `MAZE_EXPLORER_EXIT_REACHED`。
4. 当前位置是分岔口时选 `untried` branch。
5. 当前位置是死胡同时标记并回退。
6. 没有任何未探索分支时输出 `MAZE_EXPLORER_EXHAUSTED`。
7. py_compile 验证。

---

### Phase 4：局部探索目标生成

#### Task 4.1 实现沿开放方向生成短目标

**Objective:** 不直接把目标发到很远未知区，而是沿可通行方向前进一小段。

**Files:**

- Modify: `src/tugbot_maze/tugbot_maze/maze_perception.py`
- Modify: `src/tugbot_maze/tugbot_maze/maze_explorer.py`
- Modify: `src/tugbot_maze/test/test_branch_detection.py`

**Steps:**

1. 测试：开放方向 0 度，lookahead 1.0m，生成安全目标在通道中央。
2. 测试：若 1.0m 处靠近墙，则缩短到最近安全点。
3. 实现 `make_branch_goal(direction, preferred_step_m, min_step_m)`。
4. 接入 explorer 的 `SEND_EXPLORE_GOAL`。

#### Task 4.2 实现目标去重和黑名单

**Objective:** 避免重复派发失败目标。

**Files:**

- Modify: `src/tugbot_maze/tugbot_maze/maze_topology.py`
- Modify: `src/tugbot_maze/tugbot_maze/maze_explorer.py`
- Modify: `src/tugbot_maze/test/test_maze_topology.py`

**Steps:**

1. 测试：同一坐标附近失败多次后 branch 变 `blocked`。
2. 添加参数：`blacklist_radius_m`、`max_failures_per_branch`。
3. Nav2 rejected/failed 后更新黑名单。
4. 运行单测。

#### Task 4.3 实现“上一分岔口”回退目标

**Objective:** 死胡同后准确回退到最近仍有未探索方向的 junction。

**Files:**

- Modify: `src/tugbot_maze/tugbot_maze/maze_explorer.py`
- Modify: `src/tugbot_maze/test/test_maze_topology.py`

**Steps:**

1. 单测：栈顶 junction 仍有 untried branch 时，回退目标是该 junction。
2. 单测：栈顶 junction 已探索完时，继续向更早 junction 回退。
3. 实现 `_select_backtrack_goal()`。
4. Nav2 成功回到 junction 后进入 `AT_NODE_ANALYZE`。

---

### Phase 5：launch 切换和参数化

#### Task 5.1 修改 explore launch 默认使用新 maze_explorer

**Objective:** 默认 runtime planner 变成迷宫 DFS explorer，同时保留旧 frontier explorer fallback 参数。

**Files:**

- Modify: `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`
- Modify: `src/tugbot_bringup/test/test_maze_bringup_contract.py`

**Steps:**

1. 合约测试：launch 中默认出现 `package='tugbot_maze'`、`executable='maze_explorer'`。
2. 合约测试：保留 `explorer_type` launch argument，允许 `maze_dfs` 或 `frontier_baseline`。
3. 使用 `IfCondition`/`UnlessCondition` 或 launch substitutions 控制两个 explorer 二选一。
4. 默认 `explorer_type:=maze_dfs`。
5. 运行合约测试。

#### Task 5.2 把入口/出口坐标统一从 maze_config 进入 launch

**Objective:** 避免 launch 里硬编码散落多个坐标。

**Files:**

- Modify: `src/tugbot_maze/config/maze_instance.yaml`
- Modify: `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`
- Optional Create: `src/tugbot_maze/tugbot_maze/maze_config.py`

**Steps:**

1. 确认 YAML 已包含 entrance/exit 坐标。
2. 若 launch 直接解析 YAML 太复杂，先保留 launch args，但在 README 和合约中要求默认值匹配 YAML。
3. 后续可加 `maze_config.py` 给节点自行读取 YAML。
4. 合约测试检查默认坐标和 YAML 一致。

#### Task 5.3 新增 DFS explorer 参数 YAML

**Objective:** 集中管理探索参数，便于调参。

**Files:**

- Create: `src/tugbot_maze/config/maze_explorer_params.yaml`
- Modify: `src/tugbot_maze/setup.py`
- Modify: `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`

**Suggested parameters:**

- `junction_merge_radius_m: 0.75`
- `dead_end_open_direction_threshold: 1`
- `branch_angle_step_deg: 30`
- `branch_lookahead_m: 1.5`
- `branch_goal_step_m: 1.0`
- `min_goal_step_m: 0.45`
- `clearance_radius_m: 0.45`
- `goal_timeout_sec: 45.0`
- `backtrack_goal_tolerance_m: 0.35`
- `exit_radius_m: 0.6`
- `publish_debug_state_hz: 1.0`

**Steps:**

1. 创建 YAML。
2. launch 加载该 YAML。
3. 合约测试确认 YAML 被安装并被 launch 引用。

---

### Phase 6：快速验证链

#### Task 6.1 运行 Python 和合约测试

**Commands:**

```bash
cd /home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522
. /opt/ros/jazzy/setup.sh
python3 -m py_compile src/tugbot_maze/tugbot_maze/*.py src/tugbot_bringup/launch/*.launch.py
python3 -m pytest -q src/tugbot_maze/test src/tugbot_bringup/test
```

**Expected:** py_compile 无输出；pytest 全部 passed。

#### Task 6.2 运行 rosdep 和构建

**Commands:**

```bash
cd /home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522
. /opt/ros/jazzy/setup.sh
rosdep check --from-paths src --ignore-src -r --rosdistro jazzy
colcon build --symlink-install --event-handlers console_direct+
```

**Expected:** rosdep 显示系统依赖满足；6 个包 build finished。

#### Task 6.3 运行 launch 文件静态/短时 smoke

**Commands:**

```bash
cd /home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522
. /opt/ros/jazzy/setup.sh
. install/setup.sh
ros2 launch tugbot_bringup tugbot_maze_explore.launch.py headless:=true use_rviz:=false max_goals:=5
```

**Expected observations:**

- 节点包含 `/maze_explorer`、`/maze_goal_monitor`、`/slam_toolbox`、`/ros_gz_bridge`。
- topic 包含 `/maze/explorer_state`、`/maze/exit_reached`、`/map`、`/scan`、`/navigate_to_pose/_action/status`。
- explorer 日志显示当前 mode、junction count、dead-end count、selected branch。

---

### Phase 7：完整 headless 探索实验

#### Task 7.1 运行限定时间完整实验

**Objective:** 让机器人从入口探索到出口，保存日志和地图。

**Command:**

```bash
cd /home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522
. /opt/ros/jazzy/setup.sh
. install/setup.sh
ros2 launch tugbot_bringup tugbot_maze_explore.launch.py \
  headless:=true \
  use_rviz:=false \
  explorer_type:=maze_dfs \
  save_map:=true \
  map_save_path:=src/tugbot_navigation/maps/maze/maze_dfs_trial
```

**Verification while running:**

```bash
ros2 topic echo /maze/explorer_state --once
ros2 topic echo /maze/exit_reached --once
ros2 topic hz /scan
ros2 lifecycle get /slam_toolbox
```

**Success token:**

- `MAZE_EXIT_REACHED` from monitor。
- `MAZE_EXPLORER_EXIT_REACHED` from explorer。

#### Task 7.2 失败时分类调参

若失败，按原因分类：

1. Nav2 频繁失败：降低目标步长、增大 clearance、检查 costmap inflation。
2. 识别不到分岔：降低 branch detection threshold、增加 lookahead 或调整 angle step。
3. 重复同一死胡同：检查 node merge radius、branch blacklisting、edge 状态更新。
4. SLAM 漂移严重：减少原地旋转、降低角速度、调 slam params。
5. 迷宫局部被认为 blocked：确认通道宽度和 costmap robot_radius/inflation 是否过大。

---

### Phase 8：可视化和诊断增强

#### Task 8.1 发布 dead-end markers

**Objective:** 在 RViz 里看到已标记死胡同和分岔口。

**Files:**

- Modify: `src/tugbot_maze/package.xml`
- Modify: `src/tugbot_maze/tugbot_maze/maze_explorer.py`

**Steps:**

1. 增加 `visualization_msgs` 依赖。
2. 发布 `/maze/topology_markers`。
3. junction 用蓝色 sphere，dead-end 用红色 X/marker，当前 backtrack target 用黄色。
4. RViz 手动检查。

#### Task 8.2 保存探索轨迹和拓扑 JSON

**Objective:** 实验结束后能复盘每次分岔选择和死胡同标记。

**Files:**

- Modify: `src/tugbot_maze/tugbot_maze/maze_explorer.py`
- Create output at runtime, e.g. `log/maze_dfs_runs/<timestamp>_topology.json`

**Steps:**

1. 添加参数 `save_topology_json`、`topology_save_path`。
2. 在 exit reached / exhausted 时保存 nodes、edges、dead_ends、visited order。
3. 不把运行产物提交进源码，默认放 `log/` 或用户指定路径。

---

### Phase 9：文档和回归

#### Task 9.1 更新 README

**Files:**

- Modify: `README.md`

**Content:**

- 新增“迷宫 DFS 自主探索”章节。
- 说明入口/出口坐标、运行命令、成功 token、调参入口。
- 明确 runtime 不读取完整 world geometry 做路径规划。

#### Task 9.2 增加回归检查说明

**Files:**

- Modify: `README.md`
- Optional Create: `docs/reports/phase-maze-dfs-explorer.md`

**Content:**

- 记录测试命令和结果。
- 记录一次成功实验的运行时间、goal 数量、dead-end 数量、是否保存 map。

---

## 5. 文件变更总览

预计新增：

- `src/tugbot_maze/tugbot_maze/maze_topology.py`
- `src/tugbot_maze/tugbot_maze/grid_utils.py`
- `src/tugbot_maze/tugbot_maze/maze_perception.py`
- `src/tugbot_maze/config/maze_explorer_params.yaml`
- `src/tugbot_maze/test/test_maze_topology.py`
- `src/tugbot_maze/test/test_grid_utils.py`
- `src/tugbot_maze/test/test_branch_detection.py`
- Optional: `docs/reports/phase-maze-dfs-explorer.md`

预计修改：

- `src/tugbot_maze/tugbot_maze/maze_explorer.py`
- `src/tugbot_maze/setup.py`
- `src/tugbot_maze/package.xml`
- `src/tugbot_maze/config/maze_instance.yaml`
- `src/tugbot_maze/test/test_tugbot_maze_contract.py`
- `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`
- `src/tugbot_bringup/test/test_maze_bringup_contract.py`
- `README.md`

---

## 6. 关键风险和缓解

### Risk 1：OccupancyGrid 中分岔检测不稳定

缓解：先用小型人工 grid 单测；真实仿真中发布 debug state/markers；调 `branch_angle_step_deg`、`branch_lookahead_m`、`clearance_radius_m`。

### Risk 2：Nav2 到短目标也失败

缓解：目标只选在已知 free 且有 clearance 的 cell；目标步长从 0.8m-1.0m 开始；必要时调 `nav2_slam_params.yaml` 的 inflation/goal checker。

### Risk 3：SLAM 漂移导致回退点不准

缓解：减少不必要原地旋转；回退到 junction 时允许一定容差；node merge radius 设为 0.6-0.8m；必要时保存轨迹诊断。

### Risk 4：DFS 探索陷入重复循环

缓解：每个 branch 必须有状态；每个 Nav2 failure 必须改变 branch/blacklist 状态；加入 max attempts 和 `FAILED_EXHAUSTED` 终止态。

### Risk 5：出口坐标在 map frame 中有偏差

缓解：继续使用 `maze_goal_monitor` 半径判定；半径初始 0.6m，可在可视化验证后微调；保持入口/出口坐标和 Gazebo world 一致。

---

## 7. 推荐执行顺序

1. Phase 1：纯拓扑模型。
2. Phase 2：grid/perception 单测。
3. Phase 3：ROS 2 explorer 节点骨架 + action client。
4. Phase 4：目标生成和回退策略。
5. Phase 5：launch 切换。
6. Phase 6：快速验证链。
7. Phase 7：完整 headless 探索实验。
8. Phase 8：可视化和诊断增强。
9. Phase 9：文档和回归报告。

---

## 8. 下一步建议

建议先实现 Phase 1 和 Phase 2，因为它们不依赖 Gazebo 长时间运行，能快速把“死胡同标记、分岔回退、未探索路线选择”的核心逻辑测稳。等纯算法稳定后，再接入 Nav2 action 和真实仿真。
