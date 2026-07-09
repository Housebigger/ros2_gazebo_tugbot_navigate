# 稳定地图产出：自建图渲染为 OccupancyGrid（`/maze/self_built_map`）

**Date:** 2026-07-09
**Workspace:** `ros2_ws_tugbot_nav_20260705`
**Status:** approved design → implementation plan next

## Problem

用户在可视化运行中打开 RViz 的 Map 显示（`/map`），看到 slam_toolbox 的占据栅格
旋转、重影、歪斜——"map 生成不稳定"。用户的真实需求：**一张稳定的、随探索逐步
生成的地图产出**（经典 SLAM 建图体验），而不是把产图功能砍掉。

### 诊断结论（2026-07-09，运行 `flood_fill_run_20260709_084302` 实证）

- 导航侧定位极紧：45 次 ICP 校正 0% 拒绝、残差中位 0.034 m、零持续分歧、
  EXIT_REACHED、oracle 0/111 碰撞。**乱的只有 slam_toolbox 的 `/map`**。
- 根因：重复走廊 + 开阔中心导致 slam_toolbox 自身 scan-matching 混叠 → 其内部位姿
  旋转漂移 → 错配扫描永久烙进占据栅格（mapping 模式不可撤销）；且其内部 map 坐标系
  相对求解器网格锁定的 map 坐标系整体旋转。这是本工程 2026-06-28 定性的"根本墙"，
  `online_slam` 的设计本来就是绕开它——**让通用 SLAM 在此迷宫出好图不可行也不必要**。
- 稳定地图的数据源已经存在：求解器自建图（perimeter + 已确认墙 + 已感知 cell），
  网格锁定、驱动导航、16/16 验证。缺的只是把它渲染成 OccupancyGrid。

### 依赖链调查存档（本轮曾考虑"移除 slam_toolbox"，被用户否决，结论留档备查）

- 控制链 `solver → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed →
  collision_monitor → /cmd_vel → bridge → Gazebo`：Nav2 的两个节点在控制通路上，
  **不可移除**（`navigation_launch.py:312` remapping + floodfill yaml 证实）。
- `global_costmap/static_layer` 订阅 `/map`（`nav2_slam_floodfill_params.yaml:9`），
  且 Nav2 costmap 激活会等 static_layer 首图；lifecycle 顺序中 planner_server 先于
  velocity_smoother —— **直接移除 slam_toolbox 会卡死激活链、瘫痪控制**。若未来要
  移除：须先从 floodfill 专属 yaml 的 global costmap 插件表中去掉 `static_layer`，
  且以 `pose_source==online_slam` 为条件（`scan_match` 模式的 bootstrap 仍依赖
  slam_toolbox）。本轮 **零接触** slam_toolbox / Nav2 / 控制链。

## 用户选定的决策

1. **不移除任何功能链**：slam_toolbox、Nav2、控制链原样。本功能是**纯增量发布**。
2. **经典 SLAM 语义**：未知 = -1（灰）、已感知 cell 内域 = 0（白）、确认墙 +
   perimeter = 100（黑）。地图随探索"从灰色未知中长出来"，但零漂移零重影。
3. **验证降级为 1 轮 GUI**（用户接受）：改动性质为纯增量可视化发布，风险等级与
   已入库的 rviz-truth / gz-trail 相同（当时均 1 轮 GUI 验收）。

## Architecture

```
flood_fill 求解器（数据已存在，全部网格锁定）:
  sensed|committed cells + confirmed_wall_segments + perimeter
       ├→ MarkerArray  /maze/self_built_walls   （已有，绿线，保留不动）
       └→ OccupancyGrid /maze/self_built_map    （新增，本 spec 的产物）
                └→ RViz Map 显示改订阅它，默认 Enabled: true
slam_toolbox 继续发它的 /map（无人订阅显示；想对比可手动加显示）
```

与绿线**同源、同变化键、同 guard**：cell 新感知/新确认时二者一并重发。

## Components

### a. `tugbot_maze/flood_fill_viz.py` — 纯渲染函数（追加）

```python
self_built_occupancy_grid(sensed_cells, wall_segments, perimeter_segments,
                          resolution=0.05, margin_m=0.5,
                          wall_half_thickness_m=0.12, stamp=None) -> OccupancyGrid
```

- **范围**：由 `perimeter_segments` 的包围盒向外扩 `margin_m` 自动计算（本迷宫约
  460×420 格）；`info.origin.position` = 包围盒最小角，`frame_id='map'`。
- **渲染顺序**（后写覆盖先写）：
  1. 全图初始化 -1（未知）；
  2. 每个 sensed cell `(c, r)`：其内域 `[2c-1, 2c+1] × [2r-1, 2r+1]`（米）填 0（空闲）；
  3. 每条墙段（perimeter + confirmed）：沿线段两侧各 `wall_half_thickness_m` 的
     条带填 100（占据）——墙覆盖空闲。
- numpy 实现（求解器环境已有 numpy），行主序 `data`（int8 → list）。纯函数，
  不含 ROS 节点状态，pytest 可直接单测（消息构造无需 `rclpy.init`）。

### b. `tugbot_maze/flood_fill_solver.py` — 发布（追加到现有方法）

- 新 publisher：`/maze/self_built_map`，`nav_msgs/OccupancyGrid`，QoS depth=1 +
  TRANSIENT_LOCAL（latch，RViz 晚连可得）。
- 在现有 `_publish_self_built_walls` 的**同一变化键、同一 try/except** 内一并渲染
  发布（gated `pose_source == 'online_slam'`；异常只记 warning，永不杀节点）。
- 数据：`cells = motion.sensed | motion.committed`；
  `walls = confirmed_wall_segments(brain, cells)`；perimeter = `self._perimeter_segments`。

### c. RViz — `tugbot_bringup/config/tugbot_nav.rviz` + `rviz/tugbot_nav.rviz`（两份同改）

现有 Map 显示：`Topic Value: /map → /maze/self_built_map`、`Enabled: false → true`、
Topic `Durability Policy → Transient Local`（匹配 latch）、`Update Topic Value` 显式改为
`/maze/self_built_map_updates`（rviz 配置中它是独立字段，不会自动跟随主题名；我们不
发布 updates 话题——显示只靠主话题的全图重发工作，updates 缺席无害）。其余字段不动。

## 开销

每次重发 ≈ 460×420 ≈ 190 KB；变化键触发约百次/趟 → 总量 ~20 MB 摊在 ~10 分钟，
可忽略。渲染为 numpy 数组填充，毫秒级。

## Error handling

- 渲染/发布包在现有 viz try/except 里：失败记 warning、下个变化键重试（键仅在
  发布成功后推进——沿用 walls marker 的既有纪律）。
- 对导航**零影响 by construction**：只读取求解器已有集合，不写任何状态、不碰
  TF/控制/定位。

## Testing

- **单测**（`test_flood_fill_viz.py` 追加）：合成小输入（如 2 个 sensed cell +
  1 条内墙 + 4 条 perimeter）断言——三值语义各自出现且位置正确（cell 内域 0、
  墙条带 100、外部 -1）；元数据（resolution、origin、width/height 与包围盒+margin
  一致）；墙覆盖空闲（cell 边界上的墙格为 100 而非 0）。
- **全量离线套件**：零新增失败（当前基线 379 passed / 7 known pre-existing）。
- **Gazebo（1 轮 GUI，用户验收）**：RViz 中地图从全灰 + 黑色外周墙开始，随探索
  白色区域逐格生长、黑墙逐段出现，全程网格对齐无旋转无重影；EXIT_REACHED、
  oracle 0 碰撞、时长与基线相当。

## Out of scope（存档备查）

- slam_toolbox 移除（依赖链调查结论见上文"存档"节；若未来做：先拆 static_layer 雷 +
  以 pose_source 为条件）。
- Nav2 化石节点剪裁（bt_navigator/planner/route/behavior/waypoint/docking 在
  flood_fill 下无人调用，但 velocity_smoother/collision_monitor 在控制链上）。
- `scan_match` 等其他模式的地图发布（本功能 gated `online_slam`）。

## Success criteria

1. GUI 运行中 RViz 默认显示 `/maze/self_built_map`：从未知中逐步生成、白底黑墙、
   全程稳定（无旋转/重影/歪斜）。
2. EXIT_REACHED + oracle 0 碰撞 + 时长与基线相当（纯增量发布，无导航回归）。
3. slam_toolbox / Nav2 / 控制链行为逐字不变；`scan_match` 模式不受影响。
4. 单测覆盖渲染语义；全量套件零新增失败。
