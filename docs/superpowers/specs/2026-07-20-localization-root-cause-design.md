# Localization Root-Cause Fix Design (2026-07-20)

## 背景与目标

p3-routing-fix 阶段(20260722,merge `a6ff175`)消灭了 P3 路由类冻结,但通关率
无统计学改善,残余 TIMEOUT 全为 P1 定位类。合并后三路专项取证(附记 5 /
记忆 `localization-alongtrack-root-cause`)对两个 P1 现象——"错位感知期"与
"0.6m 对齐幻影"——下了判决:**同一根因 = 沿行进方向(along-track)的位姿滞后,
与 yaw 无关**;幻影不是幻影(是被毒地图标成 OPEN 的真墙),机制是栅格周期性
混叠 + 定位器在走廊中心线被 `_has_local_interior` 关掉 + 0.5m 平移钳使无恢复窗口。

本阶段做定位层根因修复,采**先测量后分叉**的方法学:项目已两次栽在机制诊断
错(yaw-freshness-gate 整阶段负结果;本会话内 yaw 折桶/自体回波两次误判),
故先用一次廉价测量验证或推翻判决,再据结果决定修哪一层——不盲修。

## 工作空间与分支

- 新 workspace **`ros2_ws_tugbot_nav_20260723`**,完整克隆 `ros2_ws_tugbot_nav_20260722`。
- 分支 `localization-root-cause`;`colcon build --symlink-install`;PRIME 由 run 脚本自带。

## 关键事实(实现前必读)

- `/odom` 来自 gz `OdometryPublisher`(`model.sdf:1472-1479`,`<odom_frame>odom`、
  世界锚定、30Hz),已 bridge(`tugbot_bridge.yaml:13`)。腿式无轮,里程由模型
  世界位姿算 → **odom 增量大概率即真实位移**;`pose_tracking.py` 注释明说
  "in sim, wheel odometry barely drifts and never aliases"。故"77% 跟踪"是
  **控制器跟踪指令路径**的比率,非 odom 测量误差——odom 尺度分支很可能测完即死。
- solver 先验:`flood_fill_solver.py:192` `prior = odom_prior(self._sm_corrected,
  self._sm_last_odom, odom_base)` —— 用 odom 增量把上次校正位姿往前推。故拒配流
  期间是纯航位推算;若 odom 干净,沿轨误差整份落在 ICP。
- `_has_local_interior`:`online_scan_match_localizer.py:316`,`_LOCAL_RADIUS =
  CELL_SIZE_M/2 = 1.0`(:45);每段只采 3 点(两端+中点)→ 沿走廊中心线 21 探针
  18 关(取证实测),定位器恰在居中控制器行驶的那条线上被关掉。
- 三自由度 `residual_rms`:`scan_match_localizer.py:140` 已算、已随 info 打印、
  **从未设门**;一自由度路径已有 `YAW_RMS_CEILING=0.15`(`online_scan_match_localizer.py:69`)。
- `correct()`:`online_scan_match_localizer.py:420`;门 1 sparse-interior(:449)、
  门 2 locality `_has_local_interior`(:453)、门 3 beam premask(:457)。
- **⚠️ 坐标系陷阱**:map 原点是入口 (11.011, 9.025)(`tugbot_maze_explore.launch.py:313-315`),
  非整数 9/10;取整成 9.0 偏恰好一格、给出反答案。所有真值/map 位姿换算必用此原点。

## 红线(不可违背)

**真值位姿与 `/odom` 只许写日志、只许离线分析,绝不喂给 `correct()` 或任何控制器/
定位输入。** Task A 的诊断开关默认关闭;产品跑(统计门主体、GUI 验收)不带它。
把真值当定位输入即作弊,直接判本阶段无效。

## Task A:三路位姿分解(评估脚手架,默认关,不入回路)

**产物定位**:评估专用脚手架,默认关(环境变量或参数开),绝不进控制链。

1. solver 节点加默认关闭的诊断:每 solver tick 抓**三个独立位姿源**——
   (i) **纯真值**:gz `/world/<name>/dynamic_pose/info` 里 anymal_c 的世界位姿
   (若该话题不可得,退回 `/odom` 并在附记标注"真值=odom、odom 分支不可独立检验");
   (ii) **`/odom`**:solver 先验实际积分的里程源;(iii) **solver map 位姿**。
   连同 tick 时刻写入独立日志行 `POSEDIAG`(map 原点用 (11.011, 9.025) 换算到
   世界系)。开关关时零开销、零日志。**三个源必须各自独立**——若真值与 odom 同源,
   `真值−odom` 段恒零、odom 分支无从检验,故纯真值须来自 dynamic_pose 而非 odom。
2. 离线脚本 `tools/pose_decompose.py`(只读)把误差拆成两段 × 三轴:
   - `纯真值 − odom`(odom 本身是否漂移)→ 沿轨/侧向/yaw;判决预测此段≈0
     (odom 世界锚定),即 odom 分支死;
   - `odom − solver`(ICP / map->odom 质量)→ 沿轨/侧向/yaw;判决预测沿轨 +2~+6m。
   报告各段各轴的 p10/p50/p90 与符号一致性,并在 ICP 段内再分:航位推算窗
   (拒配流期间)贡献 vs 被接受修正(rms 尖峰)贡献。
3. **判决预测(可证伪)**:`真值−odom` 沿轨≈0;`odom−solver` 沿轨 **+2~+6m 恒正**、
   侧向 <0.15m、yaw <0.1 rad。若三者皆小 → 判决整份被推翻(见分叉 C3)。

## Task B:believed 轨迹边合法性检测器(入产品,零真值)

**产物定位**:永久在线诊断,纯观测不改路由决策。

1. `maze_motion` 每次 `self.cell` 变化(`_track_cell` 内)断言刚跨过的边
   `(prev_cell → cell)` 不是已知墙(`brain.is_wall`)。仅用 `brain.is_wall`、
   `self.cell`、`self.prev_cell` —— **不碰任何位姿源**。
2. 命中即记事件 `ILLEGAL_EDGE cell=%s prev=%s dir=%s committed=%s t=%.1f`
   (solver 侧 drain,追加字段向后兼容)。
3. **不改任何路由行为**:不阻止跨越、不改 escape/unstick、不改落墙——只排事件。
4. 成为统计门**先行止损指标**:首次 `ILLEGAL_EDGE` 时刻(取证实测 t=+490,
   早于超时 3100s 且早于地图中毒)与每跑 `ILLEGAL_EDGE` 次数。

## Task C:按 A 分叉的止血(入产品,序贯不齐射)

分叉点 = Task A 的判决。

### C1 — A 判 ICP 主导(判决预测情形)

**序贯加杠杆,不一次全上**(项目两次栽在机制诊断错):
- 若**航位推算窗**主导 → 先修 `_has_local_interior`:把"每段采 3 点"改成
  **到线段的真实最近距离**(点到线段投影,夹到端点),保留"参照不足就别配"
  的原意,去掉"恰在中心线失效"的病灶。`_LOCAL_RADIUS` 维持 1.0 或按修法需要
  具名调整(记附记)。
- 若**坏被接受修正**主导 → 给三自由度 ICP 加 `residual_rms` 天花板(暂定
  ~0.12,以 Task A 接受态 p90≈0.089 与中毒 tick rms 0.157/0.169 之间取值,
  具名常量、引实测);超天花板则拒该修正(退回先验),与一自由度
  `YAW_RMS_CEILING` 对称。
- **每加一个杠杆,用 Task B 中毒指标 A/B 对照复测;指标未归零再加下一个。**
  两个杠杆都上仍不归零 → 记附记,评估是否升级到后手(多假设定位,本阶段不做)。

### C2 — A 判 odom 主导(判决被推翻情形)

转 odom 尺度标定 + 直行/转弯分离误差模型;**ICP 分支暂不动**。此时需独立细化
方案(可能触及腿式里程/足端里程),spec 附记记录转向决策,必要时新开子阶段。

### C3 — A 判三者皆小(判决整份被推翻)

止步:改抓逐 bin 原始 scan、重新取证。本阶段转为**诊断阶段**,不盲修;产物 =
新的成因判决 + 下一阶段候选。

## 验收门(统计门)

- **主门 = Task B 中毒指标**:headless ×N(分批,坏跑即停取证)A/B 对照——修法栈
  首次 `ILLEGAL_EDGE` 时刻显著后移 / 每跑次数显著下降甚至归零。
- **辅门 = 通关率不倒退**(EXIT rate 不低于 20260722 基线的 4/8 域)+ oracle
  每跑 ≤0.719%(live_rate 一并报告)。
- 通关率因 P1 之外仍有其他随机因素,只作不倒退护栏,不作主判据
  (承接"零 TIMEOUT 门对纯定位阶段结构性不可达"的教训)。
- **GUI ×1**(用户发起):EXIT + oracle 同门槛 + 用户画面确认(尤其看累积图与
  实时点云是否恢复贴合——上阶段的错位视觉证据是本阶段修好与否的直观判据)。

## TDD 离线验收(核心)

- **Task A**:`pose_decompose.py` 在 20260722 既有坏跑 artifact 上复跑,复现判决
  预测的沿轨符号与量级(或推翻之);开关关时零开销回归。
- **Task B**:合法边(非墙)不触发、非法边(已知墙)触发、committed 字段正确、
  不改任何路由决策(既有 motion-sim / flood-fill-sim 零回归)。
- **Task C(按分叉)**:`_has_local_interior` 最近距离改法红铁证(中心线上旧代码
  门关、新代码门开;参照不足场景仍别配);rms 天花板边界测试(接受态 p90 不误拒、
  中毒 rms 被拒);全套件基线名单逐字对照零漂移。

## 明确不做

- 多假设/晶格感知定位(C 失败后的后手,不是先手)。
- 路由层继续打补丁(条件化 visited 否决、落墙一致性检查、`mark` 对称写洞)——
  留 backlog;A+C 后若研磨机仍在再取。
- 投影/雷达/相机/Nav2 参数;STALL/front_block 机制(yaw-freshness-gate 负结果领地)。
- 真值/`/odom` 进任何控制或定位回路(红线)。

## 附记(实现期决策记录)

1. **Task B committed 字段语义(2026-07-21,计划/测试矛盾裁决)**:计划内联代码写
   `src in committed`(源格),但计划自带的 `test_illegal_event_reports_committed_state`
   提交的是**目标格** (5,6) 并断言 `committed=True`——二者矛盾。以测试为准,
   `ILLEGAL_EDGE` 的 `committed=` 字段取**目标格**(belief 落入的格)的 committed
   状态(`self.cell in self.committed`)。纯诊断字段,无控制消费,任一选择皆可,
   择目标格更贴"belief 滑入已冻结格"的语义。实现见 `5a4523a`。

2. **Task A 分叉裁决 + 计划推翻(2026-07-21,控制器裁决;用户裁定修法方向 A)。**
   **测量基础设施两次返工后**(BUG1:gz `Pose_V→TFMessage` 桥 frame_id 全空,
   `_gt_cb` 回落 `transforms[0]` 模型根世界位姿=`d0ff011`;BUG2:pose_decompose
   不做坐标系换算,gt 世界系被 11m 入口偏移淹没,改为从数据自导偏移换算=`a491a03`)
   拿到三源 POSEDIAG 分解。**红线运行时确认**:默认关跑到达 driving 仍 POSEDIAG=0、
   solver 侧零真值订阅。
   - **odom 分支彻底死**:三跑 `gt-odom` 全轴≈0(along/lat p50=0.00、yaw p50=0.01),
     帧偏移首末一致到 0.004-0.006m。**odom = 世界锚定真值,不漂移**;"77% 跟踪"是
     控制器跟踪指令比率、非 odom 测量误差。odom 尺度标定分支(C2)永久划掉。
   - **分叉 = ICP 分支**:清跑 odom-solver 厘米级(健康),陷阱跑 `041139`
     (28486 POSEDIAG≈近满小时)巨大(along ±8~9m、lat p50=3.3m、yaw p50=1.54 rad)。
   - **真机制 = 栅格混叠(L4)+ 无 odom 先验门的坏接受(L2),不是航位推算窗**:
     深度跑逐 tick——yaw 在 ~2s 内阶跃到 **-90°(-π/2)并整跑 2500s 钉死**(97.7% tick);
     干净 ICP-off 窗走 6.93m 仅涨 yaw 0.12(**航位推算完美,L1 放大不了误差**);
     种子是 **57 次亚阈值 ICP accept 净 +1.06 rad 全朝错误方向**(每次<10° 单步钳位)
     把位姿迁进混叠;发散后 **232 次 accept 是健康拟合(rms 0.008-0.064、eigmin>100)
     却处 3-5m/90° 错位——ICP 自信地精确错着**。四杠杆:L4 主(归宿+锁定)、L2 主
     (载具)、**L1 仅次要、L3 平移钳完全 inert**。
   - **⚠️ 计划的两个 ICP 杠杆均被推翻**:Task 5(locality gate)让 ICP 跑更多、
     反造更多混叠 accept;Task 6(rms 天花板)对混叠无效——混叠 accept rms 健康
     0.008-0.064,天花板拦不住。**Task 5/6 作废,替换为下列新杠杆。**
   - **新修法(用户裁决 A):odom 先验 yaw 一致性门**。odom yaw 无漂移
     (|gt-odom yaw| p50=0.013)。既有 10° 单步 yaw 钳位比较的是**上一次校正位姿**
     (自身已漂),故 57 次小 accept 能累积逃逸;新门把比较基准改为**无漂移的 odom
     先验 yaw**、对 solver 累积 yaw 偏离设界,超界否决 accept(在 ICP 弱约束/稀疏
     参照区收紧,密参照区仍信 ICP)。从根掐掉 57+232 次混叠 accept。**红线裁定
     (用户)**:solver 本就用 odom 做先验、本就有单步 yaw 钳位,收紧既有门为对
     odom 先验的累积界=收紧既有依赖,不算破红线;真值 dynamic_pose 仍绝不进回路。
   - **⚠️ 单跑证据 + 战役确认中**:上述基于 `041139` 一跑(被杀、无 result)。
     第一次战役约 15.5h 前随子代理结束而死(子代理后台 sim 随子代理终止的陷阱);
     已在主会话重启 4 跑(`bu6929hqo`,能跨轮存活)确认:混叠是否一致 ~90°/错一格、
     门限标定、门是否不伤密参照走廊里的正常 ICP。
   - **Task 7 指标口径改订(BUG2 副产品)**:ILLEGAL_EDGE 检测器结构性太弱
     (只在"belief 跨过已相信的墙"时触发,与真值穿墙近乎正交;020402 跑 2 次真
     reconcile-desync 仍报 0),降为在线弱佐证;**统计门主指标改用 POSEDIAG 的
     gt-vs-solver 位姿/格误差**(评估语境,真值离线可用),辅以 reconcile-snap 幅度。

3. **门限标定(2026-07-21,双陷阱跑 210254/041139 交叉验证 + 4 清跑对照)。**
   信号 `ydis = wrap(solver_yaw − odom_yaw)`(map 系,纯从 POSEDIAG odom/solver 算,
   即门在线可见的量)。
   - **机制一致**:两陷阱跑(210254 pin +1.449、041139 pin +1.556 rad)均为
     **单次永久 +π/2 一格移位锁定、同号、0 次恢复的单向陷阱**;210254 的 marked=60
     是同一个永久锁,非多次发散。
   - **清跑 `|ydis|` 天花板**:4 清跑池化 26850 tick,**max=0.297 rad、p99=0.129、
     从未越 0.3**;odom 全程无漂移(odom-vs-真值 yaw max 0.070、pos max 0.016m)。
   - **迁移动态**:onset 单调不回头,0.3→0.8 约 1s、0.3→满锁 55-67s;`ydis` 是
     差分量,转弯自动抵消(非发散 tick 在 6.7-25 rad/s 转速下 max|ydis|=0.49,
     且恰在 onset tick 本身)——**转弯不误报**。
   - **门限 = 0.5 rad,无条件、始终在**。下界离清跑 max 0.297 有 +0.203 rad
     (1.68×;3.9×p99;零清跑越 0.3);上界离 pin 1.45-1.56 有 −0.95 rad;
     锁定前 55-67s 触发。可用区间 [0.4,0.7],0.5 为平衡中点。
   - **回落安全**:触发 tick 处 odom-vs-真值 yaw p90=0.016-0.031、pos<0.01m,而被弃
     的 solver accept yaw~π/2 错、pos 至 16.6m 错——**回落 odom 先验严格更优**。
   - **新 Task 5 设计**:solver 节点 `correct()` 返回后加门——
     `odom_map = compose_2d(entrance_anchor, odom_base)`(纯 odom 在 map 系,即
     POSEDIAG 的 odom 字段);若 ICP 未拒 ∧ `|wrap(est_yaw − odom_map_yaw)| >
     YDIS_GATE_RAD(0.5)` → 弃 est、保 odom 传播先验 `prior`、记 `YAW_GATE` 事件。
     纯判定抽成 ROS-free 可测 helper。既有 10° 单步 yaw 钳位比较**上一次校正位姿**
     (自身漂),新门比较**无漂移 odom 先验**、抓累积逃逸——收紧既有门(红线内)。

4. **统计门 + 门动作从"封顶"升级为"恢复"(2026-07-21/22,用户裁决 A)。**
   判定脚本 `gate_score.py`(gt-solver |yaw| 分布 + pinned>0.8 分数 + YAW_GATE 计数)
   自检:修法前陷阱跑判 ALIAS-LOCK(pinned 87-90%、p90 1.6),清跑判 clean。
   - **封顶门首批 5 跑**:主门过(5/5 EXIT vs 修法前 4/8、全跑 pinned=0%、无 π/2 锁),
     但**辅门败 3/5**(oracle 3.6/3.6/7.1%,YAW_GATE 触发 23-25 次)。诊断:门把 yaw
     误差**封顶在 ~0.5 rad 但不恢复**——子门 accept 累积的 ~0.5 残差被 odom 先验搬运、
     转成 ~0.7m 位置偏心,碰撞**全部集中在 (6,4) 硬路口**(y 偏心 0.66-0.82m 蹭北墙);
     非触发跑 oracle 0.000%。
   - **门动作改恢复**(`33e6ba1`,用户裁决 A):拒配时不仅弃 est,**把 yaw snap 到
     无漂移 odom yaw、位置仍留先验**——主动消残差而非封顶。红线:仍只用 odom(不碰
     dynamic_pose 真值),用户裁定这步更深的 odom 依赖可接受。
   - **恢复门复跑 5 跑**:**双门全过**——5/5 EXIT、全跑 pinned=0%、**oracle 全
     0.000%**。铁证在触发次数:从封顶的 23-25 次降到 **2 次**(snap 即刻恢复、门不必
     反复救),触发跑 gt-solver |yaw| max 仅 0.46-0.51(够门限即拉回、无残差)。
     (+3 加固批补至 N≈8 进行中。)
   - **结论**:odom 先验 yaw 门(恢复式)消除栅格混叠 P1 失败——混叠锁定归零、
     残差蹭墙归零、通关率 4/8→5/5。计划两杠杆(locality gate/rms 天花板)被单一
     对症杠杆替代。
