# MR-Buggy3 运动学换车(buggy-swap)设计

日期:2026-07-22
工作空间:`ros2_ws_tugbot_nav_20260725`(克隆 `ros2_ws_tugbot_nav_20260724`,含 06be081 3D 建图链),分支 `buggy-swap`
上游:承接 cloud-map-3d(SHA `06be081`)——3D 建图链达标但**四足 trot 摇晃刷厚表面**(体素 2.3-2.5M,比理想单层高 3-5×),封了图质量上限。本阶段以轮式平台消掉 roll/pitch/z 抖动源。

## 1. 目标

把迷宫机器人从 ANYmal C 四足狗换成 **MR-Buggy3 四轮小车(运动学底盘)**,导航链与 3D 建图链行为不变,**3D 环境地图表面壳层显著变薄**(量化门见 §6)。

非目标(YAGNI,入 backlog):真物理阿克曼(0.41m 最小转弯半径 → MazeMotion 弧线转向 FSM 大改);轮子转动动画;占据栅格重建。

## 2. 用户裁决记录

- **先合并 cloud-map-3d 再换车**(已完成,`06be081`):建图链平台无关,原样沿用;平台摇晃记为已知局限。
- **换车深度 = A. 运动学底盘**(否决真物理阿克曼——工作量大一个量级、建图增益并无提升):cmd_vel 模型级速度直控,原地转天然可用,MazeMotion/solver/定位链**一行不改**;真物理留后续阶段(狗的先例路线:先运动学后真物理)。

## 3. 已核实的环境事实(设计依据)

- **MR-Buggy3**(`tmp_resources/MR-Buggy3`,NXP,SDF 1.8):轴距 0.2255m、轮距 0.2m、轮半径 0.0365m、本体碰撞盒 0.3×0.09×0.12m;自带 AckermannSteering 插件(`ignition-gazebo-*` Fortress 命名,Harmonic 下不可直用)+ NPU 俯视相机;**无雷达无里程计**。转向限位 0.5 rad → 最小转弯半径 ≈0.41m,不能原地转——运动学路线绕开此约束。
- **运动学驱动先例**(20260716 anymal-dog-swap 原文):`gz-sim-velocity-control-system` + `<topic>/cmd_vel</topic>`,bridge `/cmd_vel` ROS→gz(Twist→gz.msgs.Twist)直通;`robot_base_frame=base_link` 是 TF 子帧**标签**(实际 link 名不必叫 base_link),沿用即保 odom→base_link 输出契约。
- **运动学阶段三个已档案的坑**(anymal-dog-swap spec):①OdometryPublisher 世界锚定 + xyz_offset 机体系后乘会瞬移——不用 xyz_offset;②gravity-off + 碰撞体 = 姿态累积炸弹——**运动学模型不带碰撞体**(碰撞真相由离线 true-footprint oracle 负责,这是 2026-06-28 起的既有架构);③ROS 禁数字开头 token——命名注意。
- **狗的 16 线雷达规格**(照抄源):`<sensor type="gpu_lidar">` 1800×16 束、垂直 ±15°、量程 0.05-100m、10Hz、`<topic>/lidar/points</topic>`、居中安装(SCAN_OFFSET_X=0 契约);gz 点云真身在 `<topic>/points`(裂分陷阱,bridge 已配 `/lidar/points/points`→`/lidar/points`)。
- **前视相机规格**(dog-front-camera 阶段 camera_4 规格):`/camera/front/image` + RViz FrontCamera;**PRIME 硬约束**:带相机的跑(headless 也)必须 offload 到 4070(已固化进 run 脚本,不动)。
- **入口 spawn**:世界文件 `<include>` anymal_c @ (−11.011, −9.025, 0.62)。小车换成同 xy、z≈0.05。
- **z 基准链**:odom 系锚在 spawn 位姿 → 小车地面在 map z≈−0.05、雷达 ≈+0.25;CloudMap3D z 带 [−1, 3] 天然覆盖,零改动。
- **狗基线体素数**(同迷宫、同建图链):2,289,212 / 2,470,002 / 2,154,122(cloud-map-3d 三跑)。

## 4. 组件设计

### 4.1 模型 `src/tugbot_description/models/mr_buggy3/`(新,从 tmp_resources 移植改造)

- meshes(BaseReduced/TopReduced/Wheel.dae)+ model.config 原样拷入;model.sdf **重写**:
  - **保留**:Base link 的视觉(车体+顶盖)、四个轮子的**视觉**(以固定 pose 挂在 Base link 上,不再是独立 link/joint——纯装饰,运动学不转轮)。
  - **删除**:AckermannSteering 插件、全部转向/轮 joint 与 link、原碰撞体(**全模型零碰撞体**,先例同款)、NPU 相机。
  - **新增**:①`gz-sim-velocity-control-system`(`<topic>/cmd_vel</topic>`);②`gz-sim-odometry-publisher-system`(odom_frame=odom、robot_base_frame=base_link、tf_topic=/tf、30Hz、dimensions=3,**无 xyz_offset**);③16 线 gpu_lidar,**居中** `<pose>0 0 0.30 0 0 0</pose>`(SCAN_OFFSET_X=0 契约;0.30m=车体顶 0.12m 上加短杆,±15° 视场下地面环 ~1.1m 起、墙面覆盖到 ~2.4m 高),`<topic>/lidar/points</topic>`,规格逐字段照抄狗;④前视相机照 camera_4 规格,前缘朝 +x(`/camera/front/image` 话题链不变)。
  - 模型名 `mr_buggy3`(全小写下划线,避开 `-` 与数字开头 token 险)。

### 4.2 世界与桥接

- 世界文件 `<include>` 换:`anymal_c` → `mr_buggy3`,pose `(−11.011, −9.025, 0.05, 0, 0, 0)`。狗模型文件**留仓不删**(契约测试 test_lidar_3d 等引用 anymal SDF;保留回摆可能)。
- `tugbot_bridge.yaml`:①`/cmd_vel` ROS→gz 直通项(若现桥表因 legged 阶段改过则恢复);②**删 anymal 12 关节 `/model/anymal_c/joint/*/0/cmd_pos` 项**(死话题);③`/lidar/points/points`→`/lidar/points`、`/odom`、`/tf`、`/gt/dynamic_pose`(eval-only)、相机项全保留;④静态 TF `base_link → <雷达帧>` 的 z 由 0.35 改 0.30,frame_id 按新模型实测钉定(gz 传感器帧名=模型/链路/传感器路径,首启实录)。

### 4.3 导航链适配(最小面,行为不变)

- `footprint.py`:包络换小车(含轮全包络 **长 ±0.15m、宽 ±0.13m**),`SCAN_OFFSET_X=0` 不变;**停车/避让阈值全部不动**(0.7m front_block 对 0.3m 车更保守,只利不弊)。喂 oracle 与运动停车规则两处,契约测试同步换数。
- launch:`locomotion_controller`(四足 trot 控制器)**从 launch 移除**(不删模块文件);`FALL_DETECTED` 判据链随之自然失活(run 脚本 grep 留着无害)。
- solver/MazeMotion/定位器/建图链:**零改动**(cmd_vel 语义、/scan 契约、TF 链、ICP、CloudMap3D 全部平台无关)。
- 速度参数:沿用现值(纯追踪 ~0.3m/s 级)。运动学直控下跟踪应比 trot 显著更贴,预算 3600s 上限不改、实际预期更快。

### 4.4 显示

- RViz:RobotModel 源自动跟 `/robot_description`?(现状:狗经 robot_state_publisher;小车运动学阶段先例是否有 RSP 以实测为准——若无,RViz RobotModel 显示降级为 TF + 雷达/相机视角,不作为验收项)。CloudMap3D/Lidar3D/LaserScan/FrontCamera/Path/TF 全部不动。

## 5. 红线与不动区

- 真值 `/gt/dynamic_pose` 只许离线评估(POSEDIAG 三重默认关不动);odom 无漂移前提在 VelocityControl+世界锚定 OdometryPublisher 下依旧成立(yaw 门契约不变)。
- 不动区:`scan_slice_projector.py`、`slice_to_scan.py`、两定位器、`maze_perception.py`、`maze_motion.py`、`flood_fill_brain.py`、`hop_controller.py`、`cloud_map_3d.py`、`cloud_map_accumulator.py`、`flood_fill_solver.py`(除非实测逼出一行级适配,须先取证再动并记 spec 附记)。

## 6. 验收标准

1. **单测/契约**:footprint 契约测试换小车数;新增 mr_buggy3 SDF 契约测试(雷达居中/规格逐字段、VelocityControl/OdometryPublisher 存在、零碰撞体、无 Ackermann 残留);既有基线 10 failed 名单逐名不变(anymal 相关契约测试因狗文件留仓而不受影响)。
2. **headless×2**(PRIME,online_slam):EXIT_REACHED + oracle 0.000%(footprint 已换小车包络)+ CLOUDMAP 图在长 + 零 ERROR。
3. **图质量量化门(本阶段主门)**:同迷宫跑完,**末态体素数 < 1.4M**(狗基线 2.15-2.47M 的 ~60%;机理预期:消掉 trot 抖动后表面壳层 3-5 层 → 1-2 层,剩余增厚来自测距噪声,属传感器本底)。若通关路径长度差异大导致覆盖面积不可比,以"发布数归一的体素增速"辅助判读并如实记录。
4. **GUI 验收(用户)**:墙面薄、地面平、无波纹/重影;小车运动平稳(无摇晃);FrontCamera 正常;旧狗不在场景中。

## 7. 风险与诚实边界

- **VelocityControl 首启钉定**:gz topic 命名/是否需要 world 系 or body 系速度语义,以首次冒烟跑实录为准(先例是 `/cmd_vel` 直通,预期零障碍)。
- **雷达帧名**:新模型的 gz 传感器帧路径(`mr_buggy3/<link>/lidar_3d`)首启实录后钉进静态 TF 与契约测试——不臆测。
- **RViz RobotModel**:运动学小车若无 robot_state_publisher 支持,模型显示可能缺失/降级——非验收项,如实告知。
- **体素门是代理指标**:体素数下降=壳层变薄的代理,受路径覆盖差异干扰;门带辅助判读路径,不做数字化妆。
- **视觉诚实**:轮子不转、纯滑行,是用户选定的 trade-off;真物理阿克曼在 backlog。

## 附记(实施期)

1. **首启冒烟(2026-07-22)**:帧名臆测 `mr_buggy3/base/lidar_3d` 实证正确(CLOUDMAP 219 发布/24.4 万体素在长),零 ERROR,VelocityControl `/cmd_vel` 直通零障碍;240s 已推进到迷宫中部——运动学小车明显快于狗。
2. **统计门首跑抓出真实回归(旧扫描 × 平台速度)**:修前 run 1 oracle 2/106(1.887%),(10,6) 东墙擦碰(车角侵入墙板 6cm)。取证闭环:该跑恰有两个 6.4s MATCH 卡顿(节律 5.00s)紧邻事件之前(终审复核措辞:卡顿在碰撞样本前 ~4-9s 结束,非两侧括住)、事发 tick ICP 拒配 rms 0.150;两清跑零卡顿零擦碰(3/3 对应)。机制:~1.4s 管线卡顿→墙参照居中用旧扫描对新位姿,单墙公式 `ox=0.88−E`(E<1.3 才算墙)使 ox 从 +0.31 翻 −0.42(上限 −0.42,与实测东移 0.48 吻合);狗 0.23 m/s 下同卡顿在容差内,小车 0.4 m/s 放大过阈——**平台换速暴露的旧潜伏缺陷**。
3. **修复 = 位姿一致性门(4b,SHA cad116c)**:`gate_offset_against_pose`(墙参照偏移与位姿推算偏移分歧 >0.35 即弃该轴;清跑一致 0.02-0.15、旧扫描矛盾 0.73,两侧余量宽),挂 maze_motion 两调用点:居中处弃轴跳过、落墙质量门 fail-closed。**模式限定 `pose_is_absolute`**(solver 在 online_slam/scan_match 置 True):离线漂移压力测试喂原始漂移 odom(位姿即不可信量,墙参照居中正是其矫正器)——门在该情境保持惰性,回归被压力测试当场抓住并裁决;审查追证误触发无死锁(居中超时路径不受门控,首感知不被阻断,自愈)。solver 一行、maze_motion 两点,均走 spec §5 免责条款。
4. **基线重冻(裁决)**:小 footprint 使 6 个 wall_follow 密封内部基线失败**全部真翻绿**、外圈作弊夹具按"三代历史记录"体裁续写(小车 0.15/0.13 重开外圈通道,实测 1611 界外样本,断言翻回 outside>0;生产 flood_fill 网格路由不外绕)、3 个腿式包络 xfail 退役。新冻结:**4 failed(全 legacy 资产契约)/ 515 passed / 0 xfailed**。
5. **统计门通过(修后全新两跑)**:均 EXIT_REACHED、oracle **0.000%**(104/104 样本)、零 ERROR、零卡顿;末态体素 **649,427 / 652,813**(发布 499/498)。**质量主门大幅达标:~65 万 vs 狗基线 2.15-2.47M,降至 26-30%**(门线 1.4M),两跑体素差 0.5% 显示覆盖确定性。工件:`log/flood_fill_run_20260722_225227`、`_230207`;修前证据:`_215559`(擦碰跑)、`_220946`、`_222125`(清跑)。
6. **GUI 验收通过(2026-07-23,用户)**:GUI 跑日志门同过(EXIT_REACHED、oracle 0.000%/104 样本、711,039 体素、零 ERROR)。用户确认建图质量达标,并确认了运动学底盘的既定视觉局限(转向不符合小车物理特性——正是设计时接受的 trade-off);**下一阶段 ackermann-physics 立项**:恢复真物理阿克曼(关节/碰撞/重力/AckermannSteering 插件),核心杠杆 = N 点掉头原语替换原地旋转、保住既有"停-转-感"FSM 纪律(0.41m 转弯半径在 2m 格几何余量已验算)。
