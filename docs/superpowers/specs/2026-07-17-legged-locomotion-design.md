# ANYmal C 真实四腿物理行走 — 设计文档(20260717 阶段)

**日期**:2026-07-17
**上游阶段**:20260716 ANYmal C 换装(运动学底盘 + 开环步态动画,merge `9d0f92b`)
**用户决策**(AskUserQuestion 逐项确认):
1. 验收标准 = **一步到位**:物理行走 + 自主走完迷宫(online_slam EXIT_REACHED);
2. 里程计 = **继续用 gz OdometryPublisher 世界真值**(导航/定位链零改动);
3. 碰撞 = **恢复原始模型全部 54 个碰撞体**(完全诚实物理);
4. 控制方案 = **A. 自研步态控制器**(CPG trot + 解析 IK + gz 力矩级 PD 跟踪 + /odom 姿态反馈)。
5. 整体设计:用户已确认("设计OK")。

---

## 1. 目标

狗**真正靠 12 个关节的力矩发力行走**:

- 重力开启(删除全部 13 个 `<gravity>false</gravity>`);
- 恢复原始 CERBERUS 模型全部 54 个碰撞体,足端真实踩地、撞墙真实受力;
- **删除 VelocityControl 插件**(上一阶段的"魔法推力底盘"),基座运动只能来自腿-地面接触力;
- 12× JointPositionController 切换到**力矩 PID 模式**(受 SDF effort 80 Nm / velocity 7.5 rad/s 限位约束);
- 新 locomotion 控制器把 /cmd_vel 翻译成关节位置目标,由 gz 内的力矩 PD 在物理频率跟踪。

**最终验收**:online_slam 模式自主走完迷宫 —— EXIT_REACHED + oracle 碰撞 0.000% + 0 摔倒事件,headless ×2 复跑 + GUI 一次(用户目视验收)。

## 2. 非目标(明确排除)

- **腿式里程计 / 状态估计**:/odom 继续用仿真器真值(用户确认)。
- **RL / MPC / WBC**:不引入学习策略或全身控制;手写模型法步态。
- **摔倒爬起恢复**:摔倒 = 终止性失败事件,诚实暴露,不做自动恢复。
- **scan_match 模式修复**:上一阶段已知损坏(solver bootstrap 的 odom 零起算假设),继续递延;默认 pose_source 仍是 online_slam。
- **solver / ICP / Nav2 / 雷达配置改动**:导航与定位链是不变量(第 6 节列出唯一允许的参数对齐)。

## 3. 工作空间与世界

- 新工作空间 `ros2_ws_tugbot_nav_20260717`,rsync 克隆 `ros2_ws_tugbot_nav_20260716`(排除 build/install/log),惯例同前。构建必须 `colcon build --symlink-install`(普通 build 会破坏 maze_sim 数据路径)。
- 迷宫世界 `tugbot_maze_world_20260528_clean_scaled2x.sdf` 唯一改动:**物理步长 3ms → 1ms**(`max_step_size 0.001`,`real_time_update_rate 1000`)。接触密集的四足 + 刚性 PD 在 3ms/ODE 下有失稳风险;1ms 是四足仿真常规值。RTF 下降属预期(不追求实时),超时预算相应放宽(第 7 节)。
- 测试世界 `anymal_test_world.sdf` 同步改 1ms(验证阶梯在其中跑)。
- 若 ODE 接触仍抖动:上调 ODE solver 迭代数(`<solver><iters>`,默认 50 → 100+),这是预案不是初始配置。

## 4. 模型改造(`tugbot_description/models/anymal_c/model.sdf`)

**恢复 / 删除:**

- 从 `tmp_resources/CERBERUS_ANYMAL_C_SENSOR_CONFIG_1/model.sdf` 逐 link 恢复**全部 54 个碰撞体**(基座 22、每腿 8:HIP/THIGH×2+KFE/SHANK×2+FOOT×2)。足端为 r=0.03 球体。
- 足端碰撞**显式加摩擦** `<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2>`(原模型足端无 surface 配置,ODE 默认摩擦不可依赖;防打滑)。
- 删除全部 13 个 `<gravity>false</gravity>`。
- `<self_collide>` 保持默认 false(腿间不互撞,省接触求解、避免步态干涉)。
- **删除 VelocityControl 插件**。

**保持不动:**

- OdometryPublisher(世界锚定真值 odom,30 Hz,dimensions=2)——上一阶段的入口锚配准(launch `entrance_x/y=11.011/9.025`)继续成立;
- 背部中心 2D 全向雷达(pose `0 0 0.35`,topic /scan,900 束);
- 13 link 真实惯量、视觉网格;
- **bridge 17 条零改动**(JointPositionController 的自动命令话题 `/model/anymal_c/joint/<J>/0/cmd_pos` 与控制模式无关,双侧命名映射照旧)。

**JointPositionController ×12 改造:**

- 删除 `<use_velocity_commands>true</use_velocity_commands>` → 回到默认**力矩 PID 模式**(PD 在物理步进频率 1 kHz 运行,输出力矩,受 effort 限位钳制);
- 增加增益,初值:`<p_gain>250</p_gain>`、`<d_gain>5</d_gain>`、`<i_gain>0</i_gain>`,按 HAA/HFE/KFE 三组分别可调(调参预算见第 9 节);`<cmd_max>80</cmd_max>` 与 effort 限位一致;
- `<initial_position>` 保持 X-stance(HAA 0,LF/RF_HFE 0.4,LH/RH_HFE −0.4,LF/RF_KFE −0.8,LH/RH_KFE 0.8)。

**出生位姿:**世界 include 的 z 从悬浮 0.580 改为**站立落地高度**(初值 0.55,以"settle 后四足触地、站高 ≈0.50、无弹跳"实测微调;X-stance 初始关节角与 initial_position 一致,落地即接近站姿)。

## 5. 运动控制器(方案 A)

新纯模块包 `tugbot_maze/tugbot_maze/legged/` + 薄壳节点。**退役并删除**:`gait.py`、`gait_animator.py`、`test_gait.py`(开环动画层被真实控制器整体取代);launch 里 gait_animator Node 换成 locomotion_controller。

### 5.1 模块边界(全部纯函数,无 ROS 依赖,可独立单测)

| 模块 | 职责 | 接口 |
|---|---|---|
| `legged/kinematics.py` | 单腿 3-DoF 解析 FK/IK(髋滚转 HAA + 髋俯仰 HFE + 膝 KFE),四腿镜像 | `leg_fk(leg, q3) -> p3`(髋坐标系足心),`leg_ik(leg, p3) -> q3`(不可达时返回最近可达解并标记) |
| `legged/trajectory.py` | 足端轨迹:摆动相摆线抬脚(默认抬高 0.07 m),支撑相反向匀速蹬地;步幅由 (vx, ω) 映射(ω 经髋点位置转成每腿切向分量) | `foot_target(leg, phase, vx, omega, params) -> p3` |
| `legged/trot.py` | 对角双足相位机(LF+RH / RF+LH 反相,duty 0.5)+ 模式机 INIT→STAND→TROT(滞回切换) | `LocomotionFSM.step(dt, cmd, odom_attitude) -> {joint: angle}` |
| `legged/stabilizer.py` | /odom 四元数 roll/pitch → 四足端 z 差动补偿(P 反馈),压体摆 | `height_offsets(roll, pitch, gains) -> {leg: dz}` |
| `legged/params.py` | 全部步态参数单点集中(dataclass):站高 0.50、步频 f_trot=2.0 Hz、抬脚 0.07、vx_max=0.4 m/s、ω_max=0.7 rad/s、STAND↔TROT 滞回阈值等 | `LeggedParams` |

**运动学常量**:髋点 (±0.2999, ±0.104, 0)(基座系)、HFE 枢轴 (±0.3598, ±0.18781, 0)、大腿(HFE→KFE)0.285 m、小腿(KFE→足心)≈0.316 m(含侧向偏置,由 SDF 精确推导)、足球半径 0.03。**精确值由 FK-对-SDF 一致性测试锁定**(测试解析原始 SDF 的链几何,断言模块常量与之吻合),避免手算错误进入代码。

### 5.2 `locomotion_controller` 节点(~100 Hz)

- 订阅:`/cmd_vel`(Twist;vx、ω,vy 忽略——solver 不发)、`/odom`(姿态反馈);
- 发布:12× `/model/anymal_c/joint/<J>/cmd_pos`(现有 bridge 话题)+ `/locomotion/state`(诊断:模式/相位/限幅后指令);
- 模式机:**INIT**(启动 settle 2 s,持续发 X-stance)→ **STAND**(|vx|<0.03 且 |ω|<0.05:四足静撑,不踏步)→ **TROT**(指令超阈值,CPG 踏步);STAND↔TROT 滞回防抖;另设诊断参数 `force_trot`(默认 false)供验证阶梯第 2 层强制原地踏步;
- 输入限幅:vx→[−0.4, 0.4],ω→[−0.7, 0.7](与 footprint 包络推导耦合,见第 6 节);
- 每 tick try/except(继承 gait_animator 的"动画层不崩"约定——控制器异常时保持最后指令并告警,不 crash 节点)。

## 6. 诚实性传导:footprint 与参数对齐

- 摆动腿瞬时越出静态 X-stance 包络,footprint 必须扩到**动态包络**:
  `FOOT_X = 0.3598(stance) + vx_max/(2·f_trot)(半步幅 0.10) + 0.03(足球) ≈ 0.49`,取 **FOOT_X_FRONT=0.49 / FOOT_X_REAR=−0.49**;`FOOT_HALF_W=0.32` 不变(trot 无侧移步幅);`SCAN_OFFSET_X=0.0` 不变。
- 转身扫掠对角 `hypot(0.49,0.32)≈0.586`,扫掠直径 1.17 m < 走廊净宽 1.76 m,几何可行。
- **一致性锁定测试**:断言 `FOOT_X_FRONT >= stance + vx_max/(2*f_trot) + ball`,参数改动时测试强制 footprint 同步。
- `test_footprint.py` / `test_maze_sim.py` / 离线运动套件按 0.49 包络**重推导**(0.39→0.49 的流程与上一阶段 0.35→0.39 先例相同);离线切角用例可能新增 xfail,逐例评估、有先例才允许 xfail。
- **solver 速度对齐**:核对 hop_controller / 运动层的命令幅值,超出 (0.4, 0.7) 的钳到限内。对导航栈唯一允许的改动类别是**运动层配置常量**(速度幅值;若阶梯 3/4 层暴露跟踪滞后,允许放宽到位容差常量),不改任何逻辑。
- oracle 语义不变:判定与迷宫墙的碰撞(足端踩地不计);`tools/replay_collision_oracle.py` 继续是官方判定。

## 7. 失效处理与运行预算

- **摔倒检测**(wrapper/monitor,基于 DIAG 位姿流):|roll|>0.6 rad 或 |pitch|>0.6 rad 或 base z<0.25 m 持续 1 s → 记 `FALL_DETECTED` 事件并立即判 run 失败(快速止损,不等超时)。
- **超时预算**:行走均速 ~0.35 m/s(对比运动学底盘),EXIT 预算放宽到 **2500 s**;1ms 步长 RTF 下降,超时语义(sim time vs wall time)在实现期核实并按 sim time 口径设定,wall-clock 上限相应放大;每次运行记录 RTF。
- 诊断:DIAG 增加 roll/pitch(上一阶段已有 yaw);`/locomotion/state` 落盘供事后分析。

## 8. 分层验证阶梯

每层是 headless 断言脚本(扩展 `tools/verify_anymal_model.sh` 形态,基于位移/姿态断言),**过了才进下层**:

1. **站立**:spawn + settle 5 s → base z∈[0.42, 0.56]、|roll|,|pitch|<5°、位置漂移 <0.05 m;
2. **原地踏步**:强制 TROT(诊断参数)10 s → 不摔、z 波动 <0.06 m、水平漂移 <0.3 m;
3. **直行**:cmd_vel 0.3 m/s × 10 s → 前向位移 ≥2.4 m(≥80% 跟踪率)、0 摔;
4. **原地转**:ω=0.5 rad/s × 8 s → 累计 yaw ≥3.2 rad(≥80%)、0 摔、位置漂移 <0.4 m;
5. **迷宫全程**:`run_flood_fill_maze.sh` online_slam headless → EXIT_REACHED + oracle 0.000% + 0 FALL_DETECTED + 0 位姿瞬移(>2 m/5 s 检测),×2 复跑;然后 GUI 一次由用户目视验收。

**纯模块 pytest**(`PYTHONPATH=$PWD/src/tugbot_maze:$PYTHONPATH python3 -m pytest src/tugbot_maze/test -q`):IK↔FK 往返(含四腿镜像与不可达钳制)、FK-对-SDF 常量锁定、足端轨迹连续性与抬脚非负、对角相位反相、稳定器符号方向(roll 正→左侧足下探)、模式机滞回、footprint-步态一致性;既有套件不回归(上一阶段基线 397 通过,其中 gait 动画的 10 项随退役删除并由 legged 新测试取代,其余项不允许回归;7 项 pre-existing failed 维持原状)。

## 9. 主要风险与止损

| 风险 | 缓解 / 止损 |
|---|---|
| 步态/增益调参不收敛(**最大风险**) | 验证阶梯逐层止损,失败层即暂停上报;参数单点集中(params.py + SDF 增益);预案:ODE iters 上调、步频/站高/抬脚扫参 |
| RTF 掉速(1ms 步长 + 54 碰撞体) | 不追求实时;超时按 sim time;每次运行记录 RTF 供预算校准 |
| 体摆动使雷达俯仰丢束(lidar 离地 ~0.85 m,pitch 4°@4 m ≈ ±0.28 m,墙高 1.2 m) | 稳定器压摆幅(目标 |pitch|<3°);监控 MATCH n 健康度(健康 n≈600);仍丢束则降雷达安装高(z 0.35→0.25,连带 TF/launch 同步改) |
| trot 跟踪 cmd_vel 滞后/振荡,solver 运动层不适 | 限幅对齐 + 阶梯 3/4 层量化跟踪率,进迷宫前暴露;必要时放宽运动层到位容差(改配置常量) |
| 足端打滑 | 足端显式 μ=1.0;踏步层(阶梯 2)直接暴露 |
| 撞墙绊倒 | 历史 oracle 0.000%(导航层从不入碰墙距离);真摔即 FALL_DETECTED 诚实判败 |

## 10. 交付物

1. 新工作空间 `ros2_ws_tugbot_nav_20260717`(克隆 + 全链构建通过);
2. `model.sdf`:54 碰撞体恢复 + 重力开 + VelocityControl 删除 + 12× 力矩 PID 控制器 + 足端摩擦;
3. 两个世界文件 1ms 物理步长;出生 z 调整;
4. `legged/` 纯模块包(kinematics/trajectory/trot/stabilizer/params)+ `locomotion_controller` 节点 + launch/wrapper 更新(gait_animator 退役);
5. footprint 0.49 包络 + 测试重推导 + 一致性锁定测试;
6. 摔倒检测 + DIAG roll/pitch + 超时预算放宽;
7. 验证阶梯脚本(站立/踏步/直行/转向)+ 迷宫全程验证记录(headless ×2 + GUI);
8. 本 spec + 实现计划文档;README 更新。

## 11. 流程约束(沿用项目惯例)

- 开发在分支上进行(subagent-driven,每任务两段评审 + 终审);合并 `--no-ff -F <tempfile>`,commit 尾行 `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`,消息内禁反引号;origin 保持同步。
- 禁止前台 sleep(脚本文件内可);ROS setup.bash 前 `set +u`;GUI 运行等用户明确指令,headless `gz sim -s` 随时可用;环境只有 `python3`。
- 用户目视 GUI = 最终验收权。
