# 真物理阿克曼(ackermann-physics)设计

日期:2026-07-23
工作空间:`ros2_ws_tugbot_nav_20260726`(克隆 `ros2_ws_tugbot_nav_20260725`,含 bfc07a4 运动学小车),分支 `ackermann-physics`
上游:承接 buggy-swap(SHA `bfc07a4`)——运动学小车已验证建图收益(体素降到狗基线 26-30%),但用户确认其转向不符合小车物理特性(设计时接受的 trade-off),立项恢复真物理。狗的先例路线同构:anymal 先运动学(20260716)后真走(legged-locomotion)。

## 1. 目标

MR-Buggy3 以**真实物理**跑迷宫:重力、轮地接触、阿克曼转向几何(0.41m 最小转弯半径、不能原地转),导航照常通关,3D 建图质量保持轮式水平(量化门 §6)。

非目标(YAGNI):悬挂/轮胎侧偏等高保真动力学;弧线原生的全新运动规划(N 点掉头保 FSM 是本阶段路线);占据栅格;GUI 轮速仪表。

## 2. 用户裁决记录

- **立项确认(2026-07-23)**:用户观察到运动学小车"只是视觉模型,转弯不符合小车物理特性",要求按真实物理运行;经评估(合理、完整阶段规模、N 点掉头路线)后用户裁决:buggy-swap 先合并(已完成 `bfc07a4`),开本阶段。
- **路线 = N 点掉头原语,保住既有"停-转-感"FSM 纪律**(否决弧线原生大改:感知纪律要重造、面大;详见 §4.2)。

## 3. 已核实的环境事实(设计依据)

- **原版 MR-Buggy3 SDF**(`tmp_resources/MR-Buggy3/model.sdf`,逐行读过):
  - 5 link:Base(mass 3.0,惯量已给)+ 4 轮(mass 0.05)+ 2 转向节(mass 0.005);碰撞体 = Base 盒 0.3×0.09×0.12 @ z+0.06 + 轮筒 r0.0365×l0.03(mu 0.5 / mu2 1.0 / fdir1 0 0 1);模型自带 pose z=.04(轮触地高度)。
  - 转向 joint:revolute z 轴,限位 ±0.6、effort 25、velocity 1.0;轮 joint:revolute y 轴无限位。
  - AckermannSteering 插件参数:kingpin_width .18、steering_limit 0.5、wheel_base .2255、wheel_separation .2、wheel_radius .0365、velocity ±100、acceleration ±5、topic `/cmd_vel`;**插件命名是 Fortress 的 `ignition-gazebo-ackermann-steering-system`,Harmonic 下须改 `gz-sim-ackermann-steering-system` / `gz::sim::systems::AckermannSteering`**。
  - **原版 bug(首读即标)**:`<right_joint>` 把 `FrontRightWheelJoint` 写了两遍、漏了 `RearRightWheelJoint`——移植时修正为 FrontRight + RearRight。
- **几何**:最小转弯半径 R = wheel_base/tan(steering_limit) = 0.2255/tan(0.5) ≈ **0.41m**;最大曲率 ≈ 2.44 1/m。`v=0, ω≠0` 时插件不动车——原地转指令全部失效,是本阶段核心约束。
- **N 点掉头几何验算**:每段弧长 L 的航向变化 = L/R;L=0.4-0.5m → ~1 rad/段;90° 两段、180° 三四段;段端点相对起点的游走半径 ≤ ~0.5m,在 2m 格 × 0.3m 车内余量充分(格心起步时距墙面 ≥0.88)。
- **现运动栈的原地转依赖点**(逐一排查过):`profiled_turn_command`(转向 FSM 的原地旋转)、`centering_command` 的 rotate-to-face 侧向分支、死端 180° 掉头、UNSTICK/backout(直线倒车——Ackermann 天然支持,不用改)。走廊纯追踪输出 (v,ω) 弧线指令,曲率需求远低于 2.44(冒烟实测钉定,不臆测)。
- **运动学阶段保留物**(20260725,全部沿用):雷达 0.30m 桅杆挂 `base` link(帧 `mr_buggy3/base/lidar_3d` 不变)、camera_front、世界锚定 OdometryPublisher(dims=3;**真物理下仍是模型世界位姿相对 spawn 的无漂移量,yaw 门契约不破**)、footprint ±0.15×±0.13(就是真实包络,零改动)、stale-scan 位姿一致性门(与转向方式无关,照用)。
- **控制链**:solver `/cmd_vel_nav` → Nav2 velocity_smoother/collision_monitor → `/cmd_vel` → 桥 → Ackermann 插件(桥表 `/cmd_vel` ROS→GZ 已在)。
- **体素基线**:运动学小车 649k/653k/711k;狗 2.15-2.47M。

## 4. 组件设计

### 4.1 模型:`mr_buggy3/model.sdf` 原地升级(不另开模型目录)

- **恢复**:原版 4 轮 + 2 转向节的 link/joint/碰撞/摩擦/惯量(数值逐字段照抄原版);重力开(删所有 `<gravity>false</gravity>`);**link/joint 全部重命名为无连字符小写**(`base`、`front_left_wheel`、`front_left_steering`……)——传感器仍挂 `base`,TF/桥表/launch 契约零变动。
- **换驱动**:删 VelocityControl;加 AckermannSteering(Harmonic 命名,参数照抄原版,**修 right_joint 重复 bug**,topic `/cmd_vel`)。
- **保留**:lidar_3d(0 0 0.30)、camera_front、OdometryPublisher、`<static>false</static>`;轮子视觉回到各自轮 link(会真转)。
- 世界 include pose 维持 `(−11.011, −9.025, 0.04)`。

### 4.2 运动层:N 点掉头原语(核心工作)

- 新纯模块 `src/tugbot_maze/tugbot_maze/ackermann_maneuvers.py`(ROS-free,离线全测):
  - `plan_n_point_turn(yaw_now, yaw_target, max_curvature, seg_len) -> [(v_sign, curvature, arc_len), ...]`:前进弧/倒车弧交替,每段航向变化 = arc_len × curvature,方向交替使位置游走有界;**显式限界**:所有段端点相对起点的位移 ≤ 0.5m(生成时逐段累计校验,超界即缩段重排)。
  - `segments_to_cmd(seg, v_mag) -> (v, w)`:v = v_sign×v_mag,w = v×curvature(Ackermann 语义:曲率=ω/v)。
  - 纯直线倒车与前进由既有指令直接表达,不进本模块。
- `MazeMotion` 加 `ackermann: bool = False` 构造参数(solver 按平台置 True;离线既有测试默认 False 不受扰):
  - 转向 FSM:ackermann 模式下,原 `profiled_turn_command` 原地旋转状态改为执行 N 点程序段序列(段间零速换向;完成判据 = 航向进入 yaw_tol,兜底超时沿用现值);
  - `centering_command` 的 rotate-to-face 侧向分支在 ackermann 模式**跳过**(沿轴前进/倒车分支保留——直线,Ackermann 可行);侧向回中依赖走廊追踪自然收敛 + 小车宽容差,**先测量**:统计门若见感知质量受损(落墙质量/居中分布),再加 S 弯平移原语(记 backlog,不预做);
  - 死端掉头 = N 点 180°;UNSTICK/backout 直线倒车不变。
- 感知纪律不变:转向程序完成、航向对齐后才落墙;K turn 期间位置游走由生成器限界,`_track_cell` 的跨格判定天然安全(游走 ≤0.5m < 格半宽)。

### 4.3 调参与预算

- 巡航速度沿用;掉头每次 ~5-10s(vs 原地转 ~2-3s),通关预期从 ~8min 升到 ~12-15min,3600s 预算不动。
- velocity_smoother 加减速限幅照旧(真物理下有真实惯量,平滑反而更贴)。

## 5. 红线与不动区

- 真值 `/gt/dynamic_pose` 只许离线评估;odom 无漂移前提在真物理 + 世界锚定 OdometryPublisher 下依旧成立。
- 本阶段可动面:`mr_buggy3/model.sdf`、`ackermann_maneuvers.py`(新)、`maze_motion.py`(转向/居中 FSM 的 ackermann 分支)、`hop_controller.py`(若 N 点程序需要其原语签名微调——先取证)、solver 的 MazeMotion 构造参数一行、相关测试。
- 不动区:`scan_slice_projector.py`、`slice_to_scan.py`、两定位器、`maze_perception.py`、`flood_fill_brain.py`、`cloud_map_3d.py`、`cloud_map_accumulator.py`、`wall_localize.py`、footprint 常量、桥表/世界(除模型自身)/RViz。

## 6. 验收标准

1. **契约/单测**:mr_buggy3 SDF 契约测试更新为物理版(关节/碰撞/摩擦存在、AckermannSteering Harmonic 命名 + **right_joint 修复钉死**(FrontRight+RearRight 各一次)、无 VelocityControl、无 gravity=false、雷达/相机/odom 契约不变);`ackermann_maneuvers` 离线单测(段序列航向收敛、游走限界、90°/180° 段数、curvature≤2.44、`segments_to_cmd` 语义);MazeMotion ackermann 分支的离线 sim 测试(现有 `_run` harness 扩 ackermann=True 档,至少直行+90°+180° 场景);基线名单(4 failed)逐名不变。
2. **冒烟(主会话)**:短跑实录——车真的走(轮转、转向节摆)、追踪曲率需求实测 ≤2.44、CLOUDMAP 在长、零 ERROR;倒车曲率符号约定实录钉定。
3. **headless×2(主会话,PRIME/online_slam)**:EXIT_REACHED + oracle 0.000% + 零 ERROR + **体素 < 1.0M**(真物理接触抖动允许比运动学 65-71 万略升,仍须远优于狗 2.15M;两跑差异与增速曲线如实记录)。
4. **GUI 验收(用户)**:转弯呈真实阿克曼弧线/N 点掉头、轮子真转、车身姿态自然;3D 图质量与运动学档相当;通关。

## 7. 风险与诚实边界

- **gz Ackermann 插件行为**(转向 PID 稳定性、倒车时 ω/v 符号约定、低速换向瞬态)以冒烟实录为准——两处"不臆测"点。
- **轮地接触抖动对图的影响是本阶段真正要测的量**,体素门就是尺子;若 >1.0M 先取证抖动谱再谈杠杆(不化妆)。
- **K turn 与 stale-scan 门的交互**:掉头段间车在动、扫描在流,位姿一致性门照常工作;若门在掉头中误触发(位姿-墙参照瞬时分歧),其后果是弃轴跳过(良性),统计门会暴露频次。
- **侧向居中被简化**是有意的测量前置:S 弯平移只有在数据要求时才加。
- **通关时长上升**是路线代价,如实呈现,不与运动学档比速度。

## 附记(实施期)

1. **计划错账与实施者抓漏(2026-07-23)**:Task 2 测试计数 9 被计划误标 10(算术连锁修正,套件预期 527);Task 1 SDF 注释与子串测试自相矛盾由前阶段同款机制避免。
2. **裁决一:超时 ×4 缩放**(Task 2 审查量化:每段舵机行程损耗 ~0.09 rad → 90° 转向需 2-3 个 replan 程序、~8-12s)——ackermann 模式 center/align/turn 4/6/5 → 16/24/20s。
3. **裁决二:可行域投影层**:端到端不变量测试抓出计划外第三个原地转源(DRIVE 相近墙 keep-out 律 v=0/ω 打满,恰在 front_block/wedge 被抑制的区间),裁决为 `MazeMotion.step` 出口单点投影 `clamp_to_ackermann`;复审再抓"前爬向障碍"险 → **纯枢转意图投影为倒退弧**(退向刚驶过净空,reverse-to-center 同源);循环断言以**投影前原始指令审计**去循环化(实测 59 枢转形原始指令,上界 168 钉档)。
4. **冒烟(主会话)**:倒车弧符号**结局级实证正确**(车推进至迷宫中部完成多次转向;首转 DIAG 显示 yaw 推进伴随弧线位移);零 ERROR、零看门狗打断;转向 ~10s/次与预估吻合。
5. **统计门首跑抓出 P1 尾巴:纯位置 ±1 格混叠(yaw 干净)**。取证链:2 个 oracle 样本 = **信念位姿穿真实静态墙**(世界墙盒与 oracle 段逐坐标对账一致,57 模型全 static 不可推)+ odomcell 领先 dcell;POSEDIAG 复跑(6287 三路样本)钉死:gt-odom≈0(odom 仍真值)、|yaw| p90=0.03(c52e951 yaw 门有效)、|odom−solver| 位置**完美双峰**(清洁 4222 样本 p99 0.182/max 0.822 vs 混叠 2052 样本钉 ~2.0,0.5-1.5 带仅 2 样本),**单窗锁死至跑终**(EXIT 在混叠态声明,修前两跑作废)。物理车慢速停走流重新暴露了 yaw 门结构性看不见的位置半边。
6. **修复 = odom 先验位置门(5b,SHA 04b07a4)**:`apply_odom_pos_gate` 镜像 yaw 门(用户裁定的既许 odom 依赖 + 恢复式同款):界 1.0m(半格),触发即位置 snap 回无漂移 odom、保留 accept 的 yaw;solver 接线在 yaw 门之后,定位器文件零接触。审查确认:无旁路更新路径、双门齐发合成 (odom_x,odom_y,odom_yaw) 正确、fail-safe(误触发=近真值换真值)、无振荡吸引子(双峰带空)。单跑标定的诚实边界由统计门补足。
7. **统计门通过(修后全新两跑)**:均 EXIT_REACHED、oracle **0.000%**(128/128)、零 ERROR;体素 **727,706 / 744,844**(<1.0M 门 ✓,vs 运动学档 649-711k 仅微升——真物理接触抖动的图代价 ~5-10%);**POS_GATE 各恰触发 1 次**(pdis 1.406)即恢复——恢复式压制与 yaw 门 25→2 同构。工件:`log/flood_fill_run_20260723_030942`、`_032112`;取证:`_023128`(幻影跑)、`_024830`(POSEDIAG)。
