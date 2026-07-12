# 雷达累积占据栅格建图 + 移除红色轨迹线

**Date:** 2026-07-12
**Workspace:** `ros2_ws_tugbot_nav_20260712`（承接 [scatter-cloud spec](2026-07-12-scatter-cloud-map-design.md)，同分支 `scatter-cloud-map`）
**Status:** approved design → implementation plan next

## Problem

GUI 验收时用户提出两条改动：

1. **去掉红色轨迹线**：`tools/run_flood_fill_maze.sh` 在 `HEADLESS=false` 时自动起
   `tools/gz_trail.py`，在 Gazebo 场景里画小车真值红线。用户不再需要这个功能。
2. **实时地图改成雷达散射样式生成**：`/maze/self_built_map` 现在由
   `self_built_occupancy_grid`（`flood_fill_viz.py`）从**网格几何**渲染——每个已感知
   cell 的 2×2m 内域整块填成 free、已知 perimeter + 确认墙整条填成 occupied，所以已探索区
   是**一格一格 2×2m 方块**长出来的、太规整。用户要的是**真正的雷达建图**：地图本身由
   激光回波累积而成——命中标占据、光束路径标空闲，已探索区随激光**散射式生长**。

用户已选定 **#2 = 雷达累积占据栅格**（而非"散点云即地图"或"free 区散射纹理"）。

## 用户选定的决策

1. **地图 = 雷达累积占据栅格**：每束激光命中的端点格标占据、沿光束 ray-cast 的路径格标
   空闲，用无漂移 ICP 位姿 `_sm_corrected` 逐帧累积（log-odds）。阈值化输出经典三值
   OccupancyGrid：**-1 未知(灰) / 0 空闲(白) / 100 占据(黑)**。发到**同一话题**
   `/maze/self_built_map`，RViz `SelfBuiltMap` 显示不改。
2. **纯雷达**：连已知 perimeter 也**扫到才显现**（不预置外墙轮廓）——perimeter 仅用来
   定栅格范围，不预填占据。最真实的散射建图观感。
3. **保留**：绿色 `SelfBuiltWalls`（确认墙 MarkerArray）+ 橙色 `/maze/scatter_cloud`
   散点云，逐字不变。
4. **删除旧渲染**：`self_built_occupancy_grid`（方块渲染）在 solver 不再使用，连同其单测
   一并移除（确认无他用）。`self_built_wall_markerarray`（绿墙）保留。
5. **去掉红色轨迹线**：wrapper 不再自动起 gz_trail；彻底删 `tools/gz_trail.py` 及其单测。
6. **验证降级为 1 轮 GUI**（沿用本阶段惯例，用户亲验）。

## Architecture

```
flood_fill 求解器（每个控制 tick，gated online_slam）:
  self.scan_msg + _sm_corrected (map->base_link, 无漂移 ICP)
    ├→ RadarOccupancyGrid.integrate_scan(pose, scan)   # ray-cast: 路径格 free / 端点格 occ (log-odds)
    │     └→（节流 0.5s）to_occupancy_grid() → /maze/self_built_map  （替换方块渲染，同话题/QoS）
    ├→ ScatterCloud.add_scan(...)  → /maze/scatter_cloud   （橙散点，不变）
    └→ self_built_wall_markerarray(...) → /maze/self_built_walls  （绿墙，change-key，不变）
RViz: SelfBuiltMap(灰底/散射生长) + SelfBuiltWalls(绿) + ScatterCloud(橙) 叠加
移除: tools/gz_trail.py 及 wrapper 的自动启动 → GUI 运行无红线
```

## Components

### a. `tugbot_maze/radar_occupancy.py` — 纯模块（新建）

纯 NumPy + std msgs，无 ROS 节点状态，pytest 可直接单测。

```python
from tugbot_maze.footprint import SCAN_OFFSET_X

class RadarOccupancyGrid:
    def __init__(self, perimeter_segments, resolution=0.05, margin_m=0.5,
                 scan_offset_x=SCAN_OFFSET_X, usable_range_m=8.0,
                 l_occ=0.85, l_free=0.4, l_clamp=5.0,
                 occ_thresh=0.4, free_thresh=0.4) -> None: ...
    def integrate_scan(self, pose, ranges, angle_min, angle_inc) -> None: ...
    def to_occupancy_grid(self, frame_id='map', stamp=None): ...  # nav_msgs/OccupancyGrid
```

- **固定栅格**：范围由 `perimeter_segments` 包围盒 + `margin_m` 定（同现 `self_built_occupancy_grid`），
  `origin=(min_x-margin, min_y-margin)`，`width/height` 由 bbox+margin 与 `resolution` 算出。
  内部维护 `L`：`float32` log-odds 数组 `(height, width)`，初值 0（未知）。**perimeter 只用于
  定范围，不预填占据**。
- **`integrate_scan`**（投影与 [scatter](2026-07-12-scatter-cloud-map-design.md) 同约定，含 `SCAN_OFFSET_X`）：
  1. 有效束筛选：`isfinite(r) & (r>0) & (r<=usable_range_m)`（无回波/超量程束整体丢弃，
     不 carve free）。
  2. 传感器原点（map）：`sx=x+c*scan_offset_x, sy=y+s*scan_offset_x`；端点（map）：
     `px=x+c*bx-s*by, py=y+s*bx+c*by`，`bx=scan_offset_x+r*cos(ang), by=r*sin(ang)`。
  3. **Ray-cast**：每束沿 `(sx,sy)->(px,py)` 按 `resolution` 步长采样路径点（`t∈[0,r)`，不含端点）
     → 这些格 `L += -l_free`（空闲）；端点格 `L += +l_occ`（占据）。向量化：构造
     `(num_beams × max_steps)` 的距离网格 + mask，`np.add.at(L, (iy,ix), -l_free)` 累加空闲、
     `np.add.at(L, (ey,ex), +l_occ)` 累加占据（`np.add.at` 正确处理重复索引）。越界索引裁掉。
  4. `L` 钳制到 `[-l_clamp, +l_clamp]`（防饱和）。
- **`to_occupancy_grid`**：`grid=int8 全 -1`；`grid[L <= -free_thresh]=0`（空闲）；
  `grid[L >= +occ_thresh]=100`（占据）。未观测（`L==0`）落在两阈值之间 → 保持 -1。
  返回 `OccupancyGrid`：`resolution/width/height/origin` 同栅格，`data=grid.ravel().tolist()`。

### b. `tugbot_maze/flood_fill_solver.py` — 改发布（不改导航）

- **`__init__`**：新增 `self._radar = RadarOccupancyGrid(self._perimeter_segments, resolution=0.05)`
  **仅 online_slam** 时构造（否则 `None`，因 perimeter 为空无法定 bbox）；新增 `_radar_last_pub = None`。
  `map_pub`（`/maze/self_built_map`）与其 latched QoS **保留不变**。
- **拆分现有 `_publish_self_built_walls`**：**移除其中的 `map_pub.publish(self_built_occupancy_grid(...))`**，
  只保留绿墙 marker 发布（change-key 纪律不变）；删去 `self_built_occupancy_grid` 的 import。
- **新方法 `_publish_radar_map(self, now)`**（结构镜像现有 `_publish_scatter_cloud`）：
  gated online_slam + None-guard + never-crash try/except。每 tick
  `self._radar.integrate_scan(self._sm_corrected, s.ranges, s.angle_min, s.angle_increment)`；
  节流 `>= scatter_period_s`（复用现有 0.5s 参数）则
  `self.map_pub.publish(self._radar.to_occupancy_grid(frame_id=self.map_frame, stamp=now.to_msg()))`
  并更新 `_radar_last_pub`。地图持续演化，故**按周期无条件重发**（不像散点云那样"有新点才发"）。
- **`_control_tick` driving 分支**：在 `self._publish_self_built_walls()` /
  `self._publish_scatter_cloud(now)` 旁加 `self._publish_radar_map(now)`。

### c. `tugbot_maze/flood_fill_viz.py` — 删旧渲染

- **删除 `self_built_occupancy_grid`** 函数（solver 不再调用，确认无他用后移除）。
- 保留 `self_built_wall_markerarray`（绿墙）。
- 对应删 `test/test_flood_fill_viz.py` 中针对 `self_built_occupancy_grid` 的测试；保留
  markerarray 的测试。

### d. `tools/run_flood_fill_maze.sh` + `tools/gz_trail.py` — 移除红色轨迹线

- `run_flood_fill_maze.sh`：删掉 `HEADLESS=false` 时自动启动 `gz_trail.py` 的代码段，及
  `kill_all_sim` 里的 `gz_trail` 匹配项。
- 删除 `tools/gz_trail.py` 及其单测 `tools/test_gz_trail.py`（若存在）/相应测试。
- README 中提及"红色真值轨迹"的说明相应删除/更新。

### e. RViz

`SelfBuiltMap` 显示订阅的仍是 `/maze/self_built_map`（Map 显示，三值配色）——**零改动**。

## 开销

- `integrate_scan`：每 tick 向量化 ray-cast，~640 束 × ~160 格 ≈ 10万 `np.add.at` 累加，毫秒级。
- 发布：0.5s 一次全量 OccupancyGrid（本迷宫 ~423×423 ≈ 190KB），摊在 ~10 分钟可忽略；
  latch depth=1 只留最新。log-odds 数组定长，不随时间增长。

## Error handling

- radar 积分/发布包在新方法的 never-crash try/except：失败记 warning、下 tick 重试。
- 对导航**零影响 by construction**：只读 `_sm_corrected`/`scan_msg`，不写任何导航状态、不碰
  TF/控制/定位。`L` 是纯内存，节点重启自然清空重建。

## Testing

- **单测 `test_radar_occupancy.py`（新建）**：
  1. **命中→占据**：位姿 `(0,0,0)`、正前方一束 range `r`，端点格阈值化为 100。
  2. **路径→空闲**：同束的中途格（原点与端点之间）阈值化为 0。
  3. **未扫→未知**：从未被任何光束覆盖的格保持 -1。
  4. **累积单调**：同一束多帧积分后端点 log-odds 增大（钳制在 `l_clamp` 内不溢出）。
  5. **无效束剔除**：inf/nan/负/零/超量程束不产生任何 free/occ 更新。
  6. **投影含偏移**：端点落点含 `SCAN_OFFSET_X`（与 scatter/localizer 同约定），非零位姿验证
     旋转平移。
  7. **OccupancyGrid 报文**：`resolution/width/height/origin` 与 perimeter bbox+margin 一致；
     `data` 长度 = width*height；取值只含 {-1,0,100}。
- **全量离线套件**：零新增失败（基线 389 passed / 7 pre-existing，**减去**被删的
  `self_built_occupancy_grid` 测试数、**加上**新的 radar 测试数）。
- **Gazebo（1 轮 GUI，用户亲验）**：RViz 中 `SelfBuiltMap` 从全灰开始，随探索**散射式**长出
  白色通道 + 黑色墙（非 2×2m 方块），与绿墙/橙散点对齐；**无红色轨迹线**；`EXIT_REACHED`、
  oracle 0 碰撞、时长与基线相当。

## Out of scope

- 散点云语义/外观（上一 spec，保留不变）。
- 绿墙 marker（保留不变）。
- `scan_match` 等其他模式的雷达地图（本功能 gated online_slam）。
- slam_toolbox / Nav2 / 控制链（零接触）。

## Success criteria

1. GUI 运行中 `SelfBuiltMap` 由**雷达累积占据栅格**生成：已探索区随激光散射式生长（灰→散射长出
   白通道 + 黑墙），非方块；与绿墙/橙散点对齐、稳定无漂移。
2. **无红色轨迹线**（gz_trail 彻底移除）。
3. `EXIT_REACHED` + oracle 0 碰撞 + 时长≈基线（纯增量发布，无导航回归）。
4. 绿墙 / 橙散点 / rviz-truth TF / slam_toolbox / Nav2 / 控制链逐字不变；`scan_match` 不受影响。
5. radar_occupancy 单测覆盖命中/空闲/未知/累积/无效/投影/报文；全量套件零新增失败。
