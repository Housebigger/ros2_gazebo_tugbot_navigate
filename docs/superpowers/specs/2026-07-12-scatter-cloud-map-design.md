# 雷达散射建图：累积激光散点云叠加显示（`/maze/scatter_cloud`）

**Date:** 2026-07-12
**Workspace:** `ros2_ws_tugbot_nav_20260712`（本 spec 的产物，从 `ros2_ws_tugbot_nav_20260705` clean-seed）
**Status:** approved design → implementation plan next

## Problem

现在 `/maze/self_built_map` 是从**已知网格几何**渲染出的完美干净墙带（`-1/0/100`），
看起来太理想化——不像一台真实机器人用激光雷达"扫"出来的图。用户想要探索过程中的
建图带上**雷达散射效果**：把原始 LIDAR 回波端点当散点，投到 map 坐标系里随探索
累积成一片描出墙轮廓的稠密"散点云"，叠在现有地图之上，呈现真实激光建图的观感。

### 现状（已具备的数据源）

`flood_fill_solver` 每个控制 tick 已经同时握着：

- **当前 `/scan`**（`sensor_msgs/LaserScan`：`ranges / angle_min / angle_increment`），存于
  `self.scan_msg`；
- **驱动导航的无漂移 ICP 位姿** `_sm_corrected`（map→base_link，`Pose2D`），由
  `OnlineScanMatchLocalizer` 每 tick 算出，正是驱动小车运动、锚定 RViz 真值 TF 的那个位姿。

并且已有一条 `_publish_self_built_walls` 的发布路径：**gated `online_slam`、包在永不崩的
try/except 里、变化键推进**，同时发绿墙 MarkerArray 与干净 OccupancyGrid。散点云本质就是
"把每帧端点投到 map、体素去重后累积、发出来给 RViz 画"，与这条路径同源同纪律。

### 对齐约定（实证，`scan_match_localizer.ScanMatchLocalizer._beams_to_points`）

localizer 把一束 range `r`、角度 `ang` 的回波投到 map 的方式：

```
bx = SCAN_OFFSET_X + r * cos(ang)     # 端点在 base_link 系（含 LIDAR 安装偏移）
by =                r * sin(ang)
# 再按位姿 (x, y, th) 旋转平移到 map：
c, s = cos(th), sin(th)
mx = x + c * bx - s * by
my = y + s * bx + c * by
```

有效束筛选：`isfinite(r) & (r > 0) & (r <= usable_range_m)`。`correct()` 返回的
`corrected_pose`（即 `_sm_corrected`）是 **map→base_link** 位姿。散点云只要**复用这同一套
投影（含 `SCAN_OFFSET_X`）+ 同一个 `_sm_corrected`**，就能与绿墙、与 ICP 匹配所用的参考
逐点对齐——这是本功能"散点贴着墙长"的正确性根基。

## 用户选定的决策

1. **表示：激光散点云叠加显示**——原始扫描端点当散点画在 map 系里，累积成描出墙轮廓的
   散点云，叠在现有地图之上；本身**不是**占据栅格。底层干净 OccupancyGrid 保持不动。
2. **累积行为：全程累积（稠密建图感）**——每帧有效端点永久保留（细体素去重限内存），
   散点云越积越密，最终描出已探索区的完整墙轮廓。像一张随探索长成的累积激光地图。
3. **实现路径 A：solver 内置 PointCloud2 累积器**——新纯模块 + solver 新发布
   `/maze/scatter_cloud`（PointCloud2, latched），复用现有 viz 路径、用同一 ICP 位姿与
   scan（对齐最准、类型最地道、纯增量）。
4. **参数取向（用户确认）**：散点颜色**橙色**、体素粒度 **0.05m**、发布节流 **0.5s**。
5. **验证降级为 1 轮 GUI**（用户接受）：改动性质为纯增量可视化发布，风险等级与已入库的
   rviz-truth / stable-map / gz-trail 相同。

## Architecture

```
flood_fill 求解器（每个控制 tick，gated pose_source == 'online_slam'）:
  self.scan_msg (ranges, angle_min, angle_inc) + _sm_corrected (map→base_link)
       └→ ScatterCloud.add_scan(pose, scan)     # 投影 → 体素去重 → 并入累积 set
            └→（节流 ≥0.5s 且有新增点）to_pointcloud2() → /maze/scatter_cloud
                                                          （PointCloud2, latched）
RViz 新增 PointCloud2 显示订阅它，叠在:
  SelfBuiltMap(/maze/self_built_map, 灰底) + self_built_walls(/maze/self_built_walls, 绿线) 之上
现有: 干净 OccupancyGrid、绿墙、slam_toolbox /map —— 全部原样不动
```

## Components

### a. `tugbot_maze/scatter_cloud.py` — 纯模块（新建）

纯 Python + numpy，**不含 ROS 节点状态**，pytest 可直接单测（PointCloud2 消息构造无需
`rclpy.init`）。

```python
from tugbot_maze.scan_match_localizer import SCAN_OFFSET_X   # DRY：与 localizer 同偏移

class ScatterCloud:
    def __init__(self, voxel_m: float = 0.05,
                 scan_offset_x: float = SCAN_OFFSET_X,
                 usable_range_m: float = 8.0) -> None: ...

    def add_scan(self, pose, ranges, angle_min, angle_inc) -> int:
        """把一帧扫描投到 map、体素去重后并入累积 set；返回本帧新增体素数。
        pose = (x, y, yaw) 的 map→base_link 位姿（接受 Pose2D 或 3-元组）。"""

    def to_pointcloud2(self, frame_id: str = 'map', stamp=None):
        """把累积体素集导出为 sensor_msgs/PointCloud2（仅 XYZ, float32）。"""

    def __len__(self) -> int:
        """累积体素数。"""
```

- **投影**：镜像 `ScanMatchLocalizer._beams_to_points`（`bx = scan_offset_x + r·cos(ang)`,
  `by = r·sin(ang)`，按 pose 的 `(x, y, yaw)` 旋转平移到 map）。`beam_stride = 1`——每束都用，
  求稠密。`scan_offset_x` 默认从 `scan_match_localizer.SCAN_OFFSET_X` 导入，保 DRY 与对齐。
- **有效束**：`np.isfinite(r) & (r > 0.0) & (r <= usable_range_m)`——与 localizer 同窗口，
  只画它信任的真实回波（丢弃 inf/nan/超量程的"无回波"束）。
- **去重**：map 点按 `voxel_m`（默认 0.05m）取整成整数键 `(round(x/voxel_m), round(y/voxel_m))`
  存进 `set[tuple[int, int]]`，天然限内存（墙只占薄带，量级几万点）。累积集为纯内存，
  节点重启自然清空重建。
- **PointCloud2**：`height = 1`、`width = len(voxels)`、3 个 `FLOAT32` 字段（x, y, z）、
  `point_step = 12`、`row_step = 12 * width`、`is_dense = True`、`is_bigendian = False`；
  每点 z = 0.0；`data` 用体素键还原回 map 坐标 `(kx * voxel_m, ky * voxel_m)`（与取整用的
  `round(coord/voxel_m)` 严格互逆，单测据此断言）打包为 little-endian float32 字节串。
  颜色不进 data——交给 RViz 显示的 FlatColor（仅 XYZ 更省字节）。

### b. `tugbot_maze/flood_fill_solver.py` — 发布（追加，不改导航）

- **新 publisher**：`/maze/scatter_cloud`，`sensor_msgs/PointCloud2`，QoS `depth=1` +
  `DurabilityPolicy.TRANSIENT_LOCAL`（latch，RViz 晚连可得）——与 `walls_pub` / `map_pub`
  同一 `_walls_qos` 风格。
- **新成员**：`self._scatter = ScatterCloud(voxel_m=0.05)`；节流用 `self._scatter_last_pub`
  时间戳（用节点时钟 `self.get_clock().now()`）；`scatter_period` = 0.5s（可 declare 成参数）。
- **累积 + 节流发布**：复用现有 viz 路径的**同一 gated（`pose_source == 'online_slam'`）+
  同一 try/except**。每 tick（有 `scan_msg` 且 `_sm_corrected` 可用时）：
  1. `added = self._scatter.add_scan(self._sm_corrected, s.ranges, s.angle_min, s.angle_increment)`；
  2. 若 `now - self._scatter_last_pub >= scatter_period` **且**（`added > 0` 或首次发布）：
     `self.scatter_pub.publish(self._scatter.to_pointcloud2(frame_id=self.map_frame, stamp=stamp))`；
     更新 `self._scatter_last_pub`。
  （累积每 tick、发布节流：省带宽的同时保证探索期间持续刷新。）
- 异常只记 warning、下 tick 重试，**永不杀节点**。对导航**零影响 by construction**：只读
  `_sm_corrected` 与 `scan_msg`，不写任何状态、不碰 TF/控制/定位/`walls_pub`/`map_pub`。

**放置位置**：在现有 `_publish_self_built_walls`（或其调用点 `_control_tick`）附近追加一段
散点累积+发布逻辑。绿墙/底图走它们既有的变化键（cell 新感知/新确认时重发），散点走自己的
时间节流键（每帧都有新回波，故不能挂在墙的变化键上）——两者独立、互不影响。

### c. RViz — `tugbot_bringup/rviz/tugbot_nav.rviz`（+ 若存在的同名副本一并改）

新增一个 `rviz_default_plugins/PointCloud2` 显示：

- `Enabled: true`（默认开）
- Topic `Value: /maze/scatter_cloud`
- Topic `Durability Policy: Transient Local`（匹配 latch）、`Reliability Policy: Reliable`、
  `History Policy: Keep Last`、`Depth: 5`
- `Style: Flat Squares`、`Size (m): 0.03`（世界尺度小方点，随缩放一致；非像素点）
- `Color Transformer: FlatColor`、`Color: 255; 140; 0`（橙色，在灰底+绿墙上醒目）
- `Decay Time: 0`（我们自己累积，RViz 不衰减）、`Position Transformer: XYZ`

其余现有显示（SelfBuiltMap、self_built_walls、RobotModel、TF、LaserScan 等）**逐字不动**。
清理时若发现 `config/` 下已无同名副本（20260705 清理阶段已删 `config/tugbot_nav.rviz`），
则只改 `rviz/tugbot_nav.rviz` 一份。

## 开销

- **累积**：每 tick 一次 numpy 投影 + set 更新，毫秒级。
- **发布**：0.5s 一次全量云，几万点 × 12B ≈ 数百 KB/次，摊在 ~10 分钟可忽略；latch depth=1
  只留最新一帧。体素去重把点数钉死在墙带规模，不随时间无界增长。

## Error handling

- 散点累积/发布包在现有 viz try/except 里：失败记 warning、下 tick 重试。
- 对导航零影响 by construction（只读求解器已有的 `_sm_corrected` / `scan_msg`，不写状态）。
- 累积集纯内存，节点重启自然清空重建（无持久化、无跨进程状态）。

## Testing

- **单测 `test_scatter_cloud.py`（新建）**：
  1. **投影正确**：位姿 `(0,0,0)`、一束正前方（`ang=0`）range `r` 的回波，落到 map 点
     `(SCAN_OFFSET_X + r, 0)`（含偏移）；再取一个非零位姿验证旋转+平移。
  2. **体素去重**：两个相距 < `voxel_m` 的 map 点塌成一个体素（`len == 1`）。
  3. **跨帧累积**：连续两帧不同位姿 `add_scan`，累积体素为两帧并集（`len` 增长、旧点不丢）。
  4. **无回波剔除**：含 `inf` / `nan` / `> usable_range_m` 的束不产生点。
  5. **PointCloud2 报文合法**：`height==1`、`width==len(cloud)`、3 个 FLOAT32 字段、
     `point_step==12`、`len(data)==12*width`、`frame_id`、`is_dense==True`；解回的 XYZ
     与累积体素还原坐标 `(kx*voxel_m, ky*voxel_m, 0)` 一致。
- **全量离线套件**：零新增失败（沿用 20260705 基线；`colcon test` 或 pytest 全绿）。
- **Gazebo（1 轮 GUI，用户验收）**：RViz 中散点云沿墙**逐步长密**、与绿墙 + 底图**逐点对齐、
  无偏移无漂移**；EXIT_REACHED、oracle 0 碰撞、时长与基线相当。

## Out of scope（存档备查）

- **散点转占据栅格**：用户已选"叠加显示"而非"建栅格"；不做 hit/miss 累积占据。
- **ray-cast 标空闲**：散点云只画**命中端点**，不沿光束标自由空间（那是占据建图，非散点云）。
- **`scan_match` 等其他模式的散点**：本功能 gated `online_slam`（唯一拥有真值 TF + 无漂移
  ICP 位姿的模式）。
- **slam_toolbox / Nav2 / 控制链**：零接触，逐字不变。
- **散点云做导航输入**：纯可视化，只读，对导航零影响。

## Success criteria

1. GUI 运行中 RViz 默认显示 `/maze/scatter_cloud`：随探索沿墙**累积成稠密橙色散点云**，
   与绿墙 / 底图**逐点对齐、无漂移**（全程稳定）。
2. EXIT_REACHED + oracle 0 碰撞 + 时长与基线相当（纯增量发布，无导航回归）。
3. 现有所有功能逐字不变：SelfBuiltMap / self_built_walls / rviz-truth TF / gz-trail /
   slam_toolbox / Nav2 / 控制链行为一字不改；`scan_match` 模式不受影响。
4. 单测覆盖投影 / 去重 / 累积 / 报文；全量离线套件零新增失败。
5. 新工作空间 `ros2_ws_tugbot_nav_20260712` 从 20260705 clean-seed、`--symlink-install`
   构建通过，`maze_sim` 数据路径可用。
