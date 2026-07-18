# 3D Lidar Swap Design (2026-07-18)

## 背景与目标

相机阶段(20260718,merge `288833f`)后,ANYmal C 有:2D 雷达 `scan_omni`(900×1@10Hz,
背部中心,`/scan` 喂整条导航链)+ 前向相机 + 4 盏灯。本阶段把 **2D 雷达真正换成 3D**:
拆除 `scan_omni`,装 1 台 16 线 3D 雷达(原版 CERBERUS `front_laser`,VLP-16 型),
通过**自研投影节点**从点云抽水平切片继续喂 `/scan` —— 导航链一行不动,同时全 3D 点云
上 RViz。传感器诚实换代,为后续 3D 感知阶段打地基。

**定位:传感器置换 + 投影适配。** 导航链零改动(仍是 2D LaserScan ICP,
`pose_source=online_slam`);验收 = /scan 几何逐字段同旧 + 迷宫全回归不退化 + 3D 点云可见。

## 工作空间

- 新 workspace **`ros2_ws_tugbot_nav_20260719`**,完整克隆 `ros2_ws_tugbot_nav_20260718`。
- 构建必须 `colcon build --symlink-install`;GUI/headless 运行的 PRIME 要求已由 run 脚本
  自带(20260718 阶段固化)。

## 关键事实(实现前必读)

- **行进方向 = 模型 +x;CERBERUS 原版"前"= −x**(相机阶段已实锤并写守卫)。原版
  `front_laser` 姿态是 `(-0.327, -0.005, 0.206, 0, 0, -1.571)` —— 偏后 + yaw −90°,
  **不可照搬**:导航链假设雷达在 base 中心(SCAN_OFFSET_X=0,20260716 教训)。
- 原版 `front_laser` 规格(tmp_resources/CERBERUS_ANYMAL_C_SENSOR_CONFIG_1/model.sdf
  行 555 起):update_rate 10、水平 1800 样本 −π..π、垂直 16 线 ±0.261799 rad(±15°)、
  range 0.05–100、高斯噪声(照抄原噪声块)。
- 当前 `/scan` 消费者(flood_fill_solver、maze_motion、footprint 等)全部假设:
  LaserScan、900 样本、angle −π..π、range 0.2–100、10Hz、frame 经 TF 到 base_link。
- gz gpu_lidar 同时发 gz.msgs.LaserScan 与 PointCloudPacked 两个话题;16 线时其
  LaserScan 话题的语义(单环?展平?)**未定义,不桥不猜**——只桥点云,实测
  `gz topic -l` 确认话题名。
- GPU 预算:2D 9k rays/s → 16 线 288k rays/s(32×)。渲染已在 4070(PRIME 固化),
  但余量必须回归实测。本机 GPU 争用史:核显塌陷(20260718)、GUI iGPU(20260717)。

## 组件 1:3D 雷达传感器(SDF)

文件:`src/tugbot_description/models/anymal_c/model.sdf`,`base` link:

- **删除** `scan_omni` 整个 sensor 块(及其 `<topic>/scan</topic>`)。
- **新增** `<sensor name="lidar_3d" type="gpu_lidar">`:规格照抄原版 front_laser
  (1800×16、±15° 垂直、10Hz、0.05–100m、原噪声块);**姿态全新
  `<pose>0 0 0.35 0 0 0</pose>`**(旧 scan_omni 位置,背部中心,无 yaw)——保
  SCAN_OFFSET_X=0;topic 显式命名(目标 gz 侧点云落 `/lidar/points`,以实测为准)。
  传感器名用 `lidar_3d` 而非 `front_laser`:它不在前部,沿用原名会误导。

## 组件 2:桥接 + TF

- `tugbot_bridge.yaml`:**删除** `/scan` 的 LaserScan 桥条目;**新增**
  `gz.msgs.PointCloudPacked` → `sensor_msgs/msg/PointCloud2`(GZ_TO_ROS,
  ROS 侧 `/lidar/points`)。
- `tugbot_gazebo.launch.py`:`scan_omni` 静态 TF 改为新传感器 frame
  (`anymal_c/base/lidar_3d`,以实测 frame_id 为准),平移同 (0,0,0.35)。

## 组件 3:投影节点 scan_slice_projector(新,自研)

文件:`src/tugbot_maze/tugbot_maze/scan_slice_projector.py`(纯 Python +
`sensor_msgs_py.point_cloud2`,ROS 自带;不引入 numpy 之外的新依赖,numpy 仅当
仓库既有代码已用才可用)。

- 订阅 `/lidar/points`(PointCloud2),发布 `/scan`(LaserScan)。
- **切片算法**:从结构化点云抽**近水平环**(垂直角最接近 0° 的 2 个通道,±1°;
  按行主序 H×V 索引,通道选择用垂直角计算而非猜索引);对每点算方位角
  atan2(y,x),落入 900 个 bin(−π..π),**每 bin 取最小距离**(min-fold,
  保守避障语义);无点的 bin 置 +inf(LaserScan 惯例 range_max+ 表示无回波,
  与旧 gz 2D 雷达行为对齐,实现时以旧 /scan 实测消息为准对齐空 bin 编码)。
- **输出几何逐字段复刻旧 scan_omni 的 /scan**:900 样本、angle_min/max −π..π、
  range 0.2–100(投影层裁剪到旧限)、10Hz(每云一发)、header.frame_id =
  新传感器 frame、stamp 沿用点云 stamp。
- 挂进 `tugbot_gazebo.launch.py`(与 bridge/静态 TF 同级——传感器适配属 sensor 级
  管线,裸 sim 会话也应有 /scan;计划期修订,原写 explore launch)。无参数化需求
  (YAGNI)。
- **离线可测**:核心切片函数与 rclpy 解耦(纯函数:点数组→900 ranges),
  pytest 用合成点云断言精确 bin 落位、min-fold、空 bin、通道选择。

## 组件 4:RViz

- `tugbot_nav.rviz`:LaserScan 显示保留(话题仍 /scan);**新增 PointCloud2 显示**
  `Lidar3D` 订阅 `/lidar/points`(样式参考既有 ScatterCloud;颜色按 z 轴渐变以
  展示 3D 感)。

## 数据流

```
gz lidar_3d(16线) → PointCloudPacked → bridge → /lidar/points(PointCloud2)
                                                   ├→ RViz Lidar3D 显示(3D 点云)
                                                   └→ scan_slice_projector
                                                        → /scan(900 bins,几何同旧)
                                                            → 导航链(零改动)
```

## 验收标准(全回归门槛)

1. **投影单元测试**:合成点云(已知方位/距离/通道)→ 断言 bin 落位、min-fold、
   空 bin 编码、±1° 通道选择;既有套件不新增失败(基线:7 failed / 413+N passed /
   3 xfailed)。
2. **守卫测试**:`lidar_3d` 存在、pose = (0,0,0.35,0,0,0)(容差)、垂直 16 线 ±15°、
   `scan_omni` 已从 model.sdf 消失。
3. **活体门**(扩展 verify 脚本或新脚本):`/lidar/points` 在流(≥1 云/秒、点数
   量级合理)且 `/scan` 在流(~10Hz、900 样本、angle/range 字段与旧值逐字段相等)、
   相机门继续通过。
4. **完整迷宫回归**:PRIME headless ×2 + GUI(RViz 看 3D 点云)×1,均 EXIT_REACHED、
   oracle 0.000%、ICP rms 中位基线域(0.024–0.034)。**GPU 32× ray 增量是本阶段
   主风险**,这是真实检验。
5. **人工**:RViz 里看到 3D 点云(墙体有垂直延展)+ 用户确认。

## 风险与预案

- **GPU 余量**(主风险):若 ICP 劣化/假碰撞/活锁 → 预案序:水平采样 1800→900
  → 帧率 10→5Hz → 垂直 16→8 线。**不动导航参数,不动投影输出几何**。
- **投影切片质量**:trot 姿态晃动 ±2° 会让固定通道扫到地面/越墙(远距离时)——
  ±1° 双通道 min-fold 天然保守;若回归中出现地面误检,收紧通道到单环或加
  z-band 过滤(决策记附记)。
- **性能**:纯 Python 处理 28.8k 点@10Hz;若 CPU 顶不住(观察 solver tick 间隔),
  预案:通道预索引 + array 模块批处理;不达标再考虑既有依赖内的向量化。

## 明确不做

- 不装 bpearl ×2(半球顶扫对无顶迷宫价值低);导航链一行不动;相机/灯不动;
  不做 3D SLAM/3D 避障;不引入 pointcloud_to_laserscan 系统包(已评审过的方案 A)。

## 附记(实现期决策记录)

1. gz gpu_lidar 话题裂分实锤(verify-first 预言命中):`<topic>` 被 gz 自己的 gz.msgs.LaserScan 占用,PointCloudPacked 自动发在 `<topic>/points` —— bridge 的 gz 侧已改指 `/lidar/points/points`(ROS 侧名不变);原同名映射是静默类型错配、零消息。
2. 合同捕获(run 前最后的 2D 窗口):angle_increment=2π/899(gz 用 samples−1 分度)、角界为 float32 π(±3.1415927410125732)、空 bin=+inf(564/900,nan 0)、time 字段全 0 —— 投影核 SCAN_CONTRACT 逐位复刻并被测试锁死。
3. 投影核评审网:变异测试杀死 floor-代-rint 与去模运算两个静默变体;isfinite 过滤被证明黑盒行为等价(非有限点在下游各级必被拦)= 纵深防御,诚实记录于提交信息。
4. 投影节点评审加固:on_cloud 全体 try/except(throttle 5s)防单帧畸形云杀死 /scan 独食源;scan_slice_projector 补进全部三个 run 脚本与两个 gate 脚本的 kill 列表(ros2 launch SIGINT 级联回收不可靠,实测孤儿)。
5. ⚠️ 首跑回归 TIMEOUT(run 120504)完整归因:RTF=1.000、ICP rms 0.028/nan 47% 健康、0 摔倒 —— 非 GPU 非定位;机制=solver 转弯瞬间 yaw 陈旧致真墙方位误归类(15 STALL 中 14 个附近确有真墙)→ 假 front_block → 90s 周期逃逸循环,入口在 (5,5) 中央交叉区(2026-06 已记录的历史麻烦区),两跑前 310s 路径逐格相同、分叉纯随机。伴随 22.3% oracle(逃逸/倒车机动中的真实碰撞)。
6. 3D 雷达本身被三路 A/B 洗清:出生点静止投影 /scan 与旧 2D 基线 finite 数逐数一致(336/900,近距弧=真实入口墙);/scan 时延 A/B 新 16ms vs 旧 12ms(+4ms);环几何反证——双环 min-fold 抗越顶(墙高 1.2m,俯仰极值下 d*=9.95m)优于旧单波束(6.51m),幻影 0.6m 低于地面击中几何下限(0.845m)不可能来自地面。
7. spec 预案梯子被机理分析逐条判不对症:降水平采样加剧单射线影响;降帧率加倍决策陈旧;16→8 线会使 ±1.1° 带内无环、投影全 inf 饿死(梯子地雷,勿踩)。
8. 用户决策(2026-07-18):走统计路线。5 跑总账:SENSE_DEBUG 跑 EXIT+0.000%(sensed-wall 135/135 与真值全符,最小裕度 0.27m);统计三连 143602/144754 EXIT+0.000%,145945 EXIT+0.719%(1/139 单 tick:mid-turn、cell(1,3) off x−0.27、该 tick ICP rms=0.033 健康、足迹触达 0.614m vs 墙距 0.73m 名义净空 ~0.1m —— 归类边缘转身擦碰歧义带,与已修的 (3,9) 转身族相关但属路口内偏心转身,solver 冻结范围外)。TIMEOUT 记为历史陷阱模式偶发表达。
9. 递延(建议下阶段):solver front_block 的 yaw 新鲜度/双 tick 确认门(机制对症,惠及所有传感器配置);SENSE 行目前与 DIAG 同 5s 节流,步态频率级封堵不可见,如需按 tick 感知诊断需解节流。
10. GUI+RViz 验收(2026-07-18):run 174342,EXIT_REACHED,oracle 0.000%(0/138),rms median 0.026,nan 81/136,0 STALL,0 摔倒;gz server 546MiB + GUI 701MiB + RViz 全程 4070。用户人工确认 Lidar3D 点云(墙体竖直延展、随行进生长)、LaserScan 贴合点云腰线、FrontCamera 正常,验收通过。
