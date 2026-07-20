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
