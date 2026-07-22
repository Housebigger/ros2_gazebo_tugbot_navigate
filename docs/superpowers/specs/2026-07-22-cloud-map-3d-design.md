# 3D 累积环境地图(cloud-map-3d)设计

日期:2026-07-22
工作空间:`ros2_ws_tugbot_nav_20260724`(克隆 `ros2_ws_tugbot_nav_20260723`,含 c52e951 定位修复),分支 `cloud-map-3d`
上游:承接 localization-root-cause(SHA `c52e951`)——ICP 位姿已可信(odom 先验 yaw 门,8/8 EXIT、oracle 全 0.000%),累积图与实时点云贴合已获用户画面确认。这是本阶段"敢做累积 3D 图"的前提。

## 1. 目标

机器狗运动过程中,在 RViz 里随行进逐步"长出"一张 **map 系累积 3D 散点环境地图**(16 线雷达点云,按高度染色,墙面/地面/立体结构可辨)。同时**全清旧 2D 地图 viz 三件套**(用户裁决:三件全清),3D 累积图成为唯一自建地图产品。

非目标(YAGNI):3D 占据体素/OctoMap(free-space 信息以后要再立项)、点云着色以外的语义、地图落盘/重载、任何导航行为改动。

## 2. 用户裁决记录

- 产品形态:**A. 累积 3D 散点云**(否决 B 占据体素——计算/渲染重一个量级且 RViz 原生无 OctoMap 显示;否决 C 两个都做)。
- 2D 清理:**A. 三件全清**(`/maze/scatter_cloud`、`/maze/self_built_map`、`/maze/self_built_walls` 及模块/测试/RViz 显示)。2D 散点被 3D 严格包含;占据栅格的"空闲/未知"信息本阶段无消费者,若以后要,在 3D 侧重建。
- 架构:**方案 1 独立 TF 驱动累积节点**(否决"挂进 solver"——给导航命脉节点加 10Hz×28.8k 点处理、且平面位姿有 trot 俯仰涂抹;否决 RViz Decay Time——无去重内存无界、非产品)。

## 3. 已核实的环境事实(设计依据)

- 3D 点云已桥接:gz `/lidar/points/points` → ROS `/lidar/points`(PointCloud2,10Hz,1800×16 束,垂直 ±15°,量程 0.05–100m)。传感器挂狗背中心 `<pose>0 0 0.35 0 0 0</pose>`(SCAN_OFFSET_X=0 契约)。
- TF 链完备:solver 发 `map->odom`(ICP 修正);gz OdometryPublisher 发 `odom->base_link` **30Hz、`<dimensions>3</dimensions>` 全 3D**(含走路 roll/pitch/z);launch 已有静态 TF `base_link -> anymal_c/base/lidar_3d`。故 `lookup_transform('map', cloud.header.frame_id)` 一次复合即得"ICP xy/yaw ⊕ odom roll/pitch/z"。trot 俯仰实测可达 ~4°,8m 处 ≈0.5m 垂直涂抹——全 3D TF 把它补掉,这是独立节点方案的核心红利。
- 2D viz 三件套消费者排查:仅 `flood_fill_solver.py` 与各自测试(`test_scatter_cloud.py`、`test_radar_occupancy.py`、`test_flood_fill_viz.py`)及 `tugbot_nav.rviz`,别无引用。清理面干净。
- 命脉不动区:`scan_slice_projector.py → /scan → maze_perception/ICP 定位链` 零接触;`slice_to_scan.py` 零接触。

## 4. 组件设计

### 4.1 `src/tugbot_maze/tugbot_maze/cloud_map_3d.py`(新,纯核心)

`CloudMap3D` 类,镜像 ScatterCloud 风格(纯 NumPy + sensor_msgs 消息构造,无 rclpy 节点态,可离线测):

- `__init__(voxel_m=0.05, usable_range_m=8.0, z_min=-1.0, z_max=3.0)`。
- `add_cloud(points_sensor_xyz, T_map_sensor) -> int`:传感器系预滤(`norm(p) <= usable_range_m`、有限值),4×4 齐次变换到 map 系,map 系 z 裁剪 `[z_min, z_max]`(防杂散/地下反射),`round(p / voxel_m)` 三维整键入 set,返回新增体素数。
- **z 基准修正(实现期)**:odom 系锚在 spawn 位姿(z=0.62),map 系里**地面 ≈ −0.62**,故 z_min 取 **−1.0**——spec 初稿的 −0.2 隐含"map z=0 在地面"的错误假设,会把地面整个裁掉。
- **导出实现修正(质量审查)**:40 万体素时 Python `sorted()` 导出 ~760ms 会卡回调线程,改 `np.fromiter + np.lexsort`(字节级同输出,~264ms 实测)。
- `to_pointcloud2(frame_id='map', stamp) -> PointCloud2`:体素键 ×voxel_m 还原为 xyz-float32,确定序(sorted),与 ScatterCloud 导出同构。
- `__len__` = 体素数。
- 体量上界:迷宫墙面+地面 ≈ 30–40 万体素,消息 ~4MB——RViz 与 DDS 无压力。

### 4.2 `src/tugbot_maze/tugbot_maze/cloud_map_accumulator.py`(新,节点)

- 订阅 `/lidar/points`(10Hz);每帧 `lookup_transform('map', cloud.header.frame_id, Time())`(取最新可用;查不到——启动初期 solver 尚未发 `map->odom`——静默丢帧,throttled warning)。
- `read_points_numpy` 取 xyz(与 scan_slice_projector 同款读法),喂 `CloudMap3D.add_cloud`。
- 发布 `/maze/cloud_map_3d`(PointCloud2,latched QoS = 旧 walls 的 `_walls_qos` 同款 transient_local):**≤1Hz 节流 + 仅当本周期有新增体素**(首帧必发),避免空转刷 4MB 消息。
- 异常处理与 scan_slice_projector 同款:回调整体 try/except,永不 crash(虽是独立进程,仍不给日志刷屏)。

### 4.3 launch / run 脚本接线

- `tugbot_bringup/launch/tugbot_maze_explore.launch.py`:新增 `cloud_map_accumulator` 节点,**与旧 viz 同款 online_slam 门控**(`pose_source==online_slam` 才起),headless 也跑(统计验收要断言"云在长")。
- `tools/run_flood_fill_maze.sh`:`kill_all_sim` 的 pkill 名单加 `cloud_map_accumulator`。

### 4.4 RViz(`tugbot_bringup/rviz/tugbot_nav.rviz`)

- 删三个显示:SelfBuiltMap(Map)、SelfBuiltWalls(MarkerArray)、ScatterCloud(PointCloud2)。
- 新增 `CloudMap3D`:PointCloud2,topic `/maze/cloud_map_3d`,AxisColor 按 Z 彩虹染色,durability 与发布端匹配(transient_local)。
- 保留:Lidar3D 实时帧(与累积图对照——上一阶段抓错位正是这个视角)、LaserScan、FrontCamera、Path、TF、RobotModel、Grid。

### 4.5 清理(三件全清)

- 删模块:`scatter_cloud.py`、`radar_occupancy.py`、`flood_fill_viz.py`。
- 删测试:`test_scatter_cloud.py`、`test_radar_occupancy.py`、`test_flood_fill_viz.py`。
- `flood_fill_solver.py` 摘除:三个 import、`map_pub`/`scatter_pub`/`walls_pub` 三个 publisher、`_publish_scatter_cloud`/`_publish_self_built_walls` 与 radar 发布逻辑、`_scatter`/`_radar` 状态与 `scatter_period_s` 参数。solver 净减代码,不加一行。
- 全仓 grep 确认三个话题名/模块名零残留(文档性引用除外,spec/memory 是历史记录不改)。

## 5. 红线与门控

- 真值通道(`/gt/dynamic_pose`)不进任何新代码;新节点只消费 TF 与 `/lidar/points`。odom 经 TF 参与 viz 位姿——odom 已是被裁定许可的依赖(solver 本就以其为先验),且本节点纯 viz、不回写任何导航状态。
- 导航行为应**零差异**:唯一共享资源是 DDS 带宽与 CPU(~4MB/s 级,localhost 可忽略;PRIME 惯例已把渲染压力挪到 4070)。验收含行为回归门。

## 6. 验收标准

1. **单测**(离线,`pytest`):CloudMap3D 手算齐次变换对拍(含带 roll/pitch 的位姿)、体素去重、`to_pointcloud2` 与 `add_cloud` 互逆、空帧/全无效帧安全、z 裁剪与量程滤波生效;节点侧纯逻辑(节流/仅增长才发)可测部分测。既有全套件零新增失败(基线 7 failed / 492 passed / 3 xfailed,名单不变)。
2. **headless×2**(PRIME,online_slam):EXIT_REACHED + oracle 0.000% + `/maze/cloud_map_3d` 末态体素数 > 50,000(保守下界:全程仅地面即远超此数;断言机制=grep launch.log 末条 `CLOUDMAP voxels=N` 日志——节点每次发布记一行,与录制 latched 消息证据等价且无需配对 DDS 域的录制进程)+ 无新增 ERROR 级日志。
3. **GUI 验收**(用户):3D 图随狗行进逐步长出、与 Lidar3D 实时帧贴合(无重影/涂抹)、高度染色可辨墙面与地面;旧三件显示已不在。

## 7. 风险与诚实边界

- **TF 时刻取最新而非逐帧插值**:ICP 修正落地瞬间,累积云会有单帧级"旧位姿点"混入;体素去重吸收其大部分。若 GUI 见可辨重影,后手是按 cloud 时间戳查 TF(tf2 buffer 有历史)——一行改动,不预做(YAGNI)。
- **latched 4MB 消息**:RViz 晚连也能拿到全图;DDS 对大 latched 消息在本机 localhost 已有 walls 先例,若见传输报错则降为 volatile+定期重发。
- **占据栅格信息丢失是真丢失**:全清后"已探明空闲区/未知区"再无产品呈现;用户已裁决接受,若以后要在 3D 侧重建(记 backlog,不在本阶段)。

## 附记(实施期)

1. **统计门通过(2026-07-22,headless×2,PRIME/online_slam)**:两跑均 EXIT_REACHED、oracle **0.000%**(142/165 样本零碰撞,live_rate 同 0)、零 `[ERROR]` 日志;CLOUDMAP 末态体素 **2,289,212 / 2,470,002**(发布 677/767 次,节点全程存活)。工件:`log/flood_fill_run_20260722_180642`、`log/flood_fill_run_20260722_182002`。
2. **体量估计修正(诚实记录)**:spec 预估 30-40 万体素/~4MB 消息,实测 **~2.3-2.5M 体素/~28-30MB latched 消息**——差一个量级。原因:表面被测距高斯噪声 + 走行位姿抖动刷厚成数体素壳层(估计只按理想单层表面算)。三道门在此体量下仍全过(headless);若 GUI 端渲染迟滞,一行后手=调大 `CloudMap3D(voxel_m)`(0.05→0.075/0.10 约降 3-8×)。
3. **测试基线修正**:20260723 实测基线为 10 failed / 494 passed / 3 xfailed(计划初稿的 7/492 过期);Task 3 后为 10 / 489 / 3(−18 删 +13 新),失败名单逐名冻结不变。
4. **GUI 验收结论(2026-07-22,用户)**:GUI 跑日志门同过(EXIT_REACHED、oracle 0.000%/140 样本、2,154,122 体素、零 ERROR);用户判定**建图链本身达标,但四足平台运动摇晃过大、限制 3D 图质量上限**(与附记 2 的表面壳层成因一致)。用户裁决:本分支按"通过,带已知平台局限"合并;**下一阶段换轮式平台 MR-Buggy3(运动学底盘)**专攻建图质量,3D 建图链平台无关、原样沿用。
