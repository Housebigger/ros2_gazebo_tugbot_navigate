# P3 Routing Fix Design (2026-07-19)

## 背景与目标

yaw-only-fallback 阶段(20260721,merge `cbe1c8a`)确立失败分类学并锁定:剩余
TIMEOUT 的主因是 **P3 = 探索图耗尽(路由类)**。取证签名(坏跑 175751 纯 P3、
201541 MIXED):

- **ESCAPE Tier-2 退化**:两坏跑各 23-25 次 ESCAPE,几乎全部
  `tier=2 gave_up_edge=False`——"不许放弃通往 VISITED 格的边"
  (`maze_motion.py` `_escape` 内 `if nb not in self.visited`)在探索后期
  结构性挡死 give-up(所有邻格都已访问),Tier 2 退化为 Tier 1(倒退一格),
  对路由零改变。每周期白烧 90s,25 次 ≈ 2250s,占满 3600s 上限的大头。
- **UNSTICK 全重开**:`_unstick` 层内 `for (c,d) in (incident or cand)` 把同
  信任层切割边一次全开——两坏跑各出现一次 `n=16`(16 条已提交墙批发回滚),
  随后数分钟逐格重感知 + 反复乐观撞墙。
- **出口邻格进过又退出**:出口相邻格已访问但被假墙隔断;通往 VISITED 格的边
  不能 give up(机制 1),重开只等 UNSTICK(机制 2)——正确答案被两机制合谋锁死。

本阶段做机制对症修复(方案 A),统计门 = **零 P3 类坏跑**(按分类学判定,
P1 类自恢复定位残留按 20260721 已归档口径豁免)。

## 工作空间与分支

- 新 workspace **`ros2_ws_tugbot_nav_20260722`**,完整克隆 `ros2_ws_tugbot_nav_20260721`。
- 分支 `p3-routing-fix`;`colcon build --symlink-install`;PRIME 由 run 脚本自带。

## 关键事实(实现前必读;行号为 20260721 版 `maze_motion.py`)

- 看门狗状态块 :126-142(`no_progress_s=90.0`、`confine_k=6`、
  `max_escape_tier=2`、`failed_hop_limit=3`、`escape_count` 单调可观测)。
- step 顶部看门狗 :151-156(`explore_t` 超窗 ∧ `_confined` → `_escape`)。
- `_track_cell` :295(visited 增长即 `explore_t=t` + `escape_tier=0`)。
- `_confined` :311(窗内 distinct cells ≤ confine_k)。
- `_route` :320(`next_cell` None 或出口 flood 不可达 → `_unstick`)。
- `_unstick` :384(信任层 locomotion → 非提交 sensed → committed;层内
  incident-or-cand **全重开**;`reopened` 双向表示计一次的有界性;耗尽 → `stuck`)。
- `_escape` :422(Tier1 倒退一格;Tier2 give-up 被 visited 规则 :442 挡住;
  无有效倒退 → `_unstick` 每 tick 至多一次)。
- EXIT_CELL 与格几何在 motion 层已可得(出口距离度量纯几何,**不需要动 brain**)。
- ESCAPE/UNSTICK DIAG 事件字符串由 `self.events` 排出,solver 侧 drain(追加
  字段向后兼容)。

## Task 1:取证先行(实现前必做,产物记附记)

离线复盘 175751(201541 交叉验证):

1. 每个 90s 窗内的相位分布(drive/center/turn/backout 占比)与重试的边;
2. UNSTICK 为何仅触发 2 次——ping-pong 期间出口是否 flood 可达;
3. 钉死两参数:**快窗值**(暂定 30s,以窗内"一次完整重开→重感知→再路由"
   周期实测上取整)与**单边选择度量有效性**(被锁死的出口边在切割候选中的
   dexit 排位——应为第 1)。

## 组件 1:UNSTICK 单边化(`maze_motion._unstick`)

1. 信任层顺序**不变**(locomotion → 非提交 sensed → committed)。
2. 层内不再全开:选**一条**边——对侧端点 `nb` 到 EXIT_CELL 的几何距离
   (格坐标欧氏距离)最小者;平局先取 incident 于 `self.cell`,再取确定性序
   (排序元组)。只重开该边(双向表示、`locomotion_walls`/`failed_hops`/
   `committed`/`corrob` 的簿记与现逻辑同款,仅作用于单边)。
3. `reopened` 有界性、耗尽 → `stuck`、重开后 `escape_tier=0` + settle +
   `phase='center'` 语义原样保留。
4. DIAG 行扩展:`UNSTICK reopen cell=%s edge=(%s,%s) tier=%s dexit=%.2f`
   (替换原 `n=%d`;solver 侧无解析依赖,纯日志)。

## 组件 2:ESCAPE 无增长升级(`_escape` + 看门狗)

1. 新状态 `_esc_visited_n: int = 0`(上次 escape 时的 `len(self.visited)`)与
   具名常量 `NO_PROGRESS_FAST_S = 30.0`(Task 1 定数)、当前窗变量
   `_no_progress_win`(初值 = `no_progress_s`)。
2. `_escape` 触发时计算 `growth = len(visited) - _esc_visited_n`,随后更新
   `_esc_visited_n`:
   - **growth == 0(耗尽态)**:`escape_count`/`escape_tier` 递增与 ESCAPE
     DIAG 事件**照常发出**(单调可观测不断链),然后跳过倒退,直接
     `return self._unstick(t)`(现在 = 单边重开);置
     `_no_progress_win = NO_PROGRESS_FAST_S`。`explore_t = t` 重置照旧
     (无忙触发)。
   - **growth > 0**:现行为原样(Tier1 倒退 / Tier2 give-up 尝试;
     visited 边规则 :442 一行不动)。
3. 看门狗判据把 `no_progress_s` 换为 `_no_progress_win`;`_track_cell` 的
   visited 增长分支额外恢复 `_no_progress_win = no_progress_s`。
   `_confined` 的窗口引用同步用 `_no_progress_win`。
4. `_unstick` 每 tick 至多一次的约束保持(耗尽分支即该唯一调用)。
5. DIAG:ESCAPE 行追加 ` growth=%d win=%.0f`。

## TDD 离线验收(核心)

测试新文件 `test_p3_routing_fix.py`(现有测试组织为准):

- **(a) UNSTICK 单边化**:构造多切割边场景(含跨信任层)→ 断言只开一条、
  层序保持、选中 dexit 最小者、平局取 incident;连续调用逐边推进;耗尽仍到
  `stuck`(有界性回归)。
- **(b) 旧行为红铁证**:同场景旧代码一次全开(n>1)——实现前先跑当前代码
  确认红。
- **(c) ESCAPE 升级**:合成"全访问 + 出口邻边假墙"耗尽 ping-pong——旧代码
  90s 周期倒退不解(红);新代码零增长 escape 直进 unstick、快窗生效、
  增长后窗恢复 90s;growth>0 路径行为与旧逐字节一致。
- **(d) 端到端**:离线 motion-sim 耗尽场景通关;正常解迷宫场景
  (test_maze_motion_sim / test_flood_fill_maze_sim)零回归。
- 全套件基线名单逐字对照零漂移(合并后 main 基线:7 failed / 453 passed /
  3 xfailed)。

## Gazebo 验收(统计门)

- **headless ×8**(3+3+2 分批,坏跑即停取证再续):门 = **零 P3 类坏跑**
  (按 P1/P2/P3 分类学逐跑判定;P1 类自恢复残留豁免)+ oracle 每跑 ≤0.719%
  (live_rate 一并报告)+ ICP rms 基线域。
- 对照基线:20260721 修案栈 8 跑 = 6 清 + 1 纯 P3 + 1 MIXED(P1+P3)。
- **GUI ×1**(用户发起):EXIT + oracle 同门槛 + 用户确认。
- 统计门失败 → 按方法学先取证再动,不盲调参数。

## 风险与预案

- **假墙多时单边重开偏慢**:快窗保证每 ~30s 一次地图变更;Task 1 实测典型
  假墙数;若取证显示单边节律仍不够,预案 = 层内每次重开 incident 全部
  (仍禁跨层全开),决策记附记。
- **选边度量选错先后**:`reopened` 有界保证最终遍历到正确边;出口向度量让
  出口边排前(Task 1 验证排位)。
- **快窗过触发**:仅在"跨 escape 零增长"后生效,任何 visited 增长即复原;
  `_confined` 判据继续兜底(合法长回溯不触发)。

## 明确不做

- 不动走廊跟踪(_drive/_center/_turn)、感知、定位器(20260721 的 yaw-only
  栈原样)、投影/雷达/相机、Nav2 参数。
- 不动 `flood_fill_brain.py`、`map_memory.py`。
- 不新增 FSM 模式(方案 B 的重验模式不做)。
- Tier-2 "不放弃 visited 边"规则一行不动。
- 不改 STALL/front_block 机制(yaw-freshness-gate 负结果领地)。

## 附记(实现期决策记录)

1. **Task 2 取证(2026-07-20)**:工具 `ros2_ws_tugbot_nav_20260722/tools/p3_forensics.py`
   (离线解析 launch.log,事件时间线 + 逐窗相位/格足迹 + dexit)复现两坏跑签名:
   175751 = 25 ESCAPE + 2 UNSTICK(tier2/gave_up=False ×20、tier1/False ×4、
   tier2/True ×1);201541 = 23 ESCAPE + 2 UNSTICK(15/6/2)。三问结论:
   - **节律**:两跑 escape 时段各占跑长 88%(首 ESCAPE 均在 +441s,跑长 3607s)。
     ESC 间隔 p50=90.1s、p90≈154/155s;175751 的 24 个间隔中 20 个、201541 的
     22 个中 16 个 = 90.1±0.2s——正是慢看门狗步进本身。这些窗内 distinct cells
     仅 3-7、相位混合 drive 55-65% + turn 15-20% + backout 10-20%(几乎每窗含
     backout)= reverse-and-return ping-pong 实锤;纯节律死时间 ≈1903s(53%)/
     ≈1656s(46%)。仅有的真实重探窗(673.2s/24 格、405.3s/14 格;对照跑
     695.1s/24 格、451.2s/15 格)全在前半程,此后 visited 零增长。UNSTICK 仅
     2 次 ⇔ `_route` 几乎从不失败:ping-pong 全程出口 flood 可达(乐观边仍在),
     地图零变更纯由 escape 梯子"不放弃 visited 边"造成——机制定性成立。
   - **UNSTICK 恢复周期 → 钉死 `NO_PROGRESS_FAST_S = 30.0`(维持暂定值)**:
     唯一未被慢看门狗污染的完整"重开→重感知→再路由"实测 = 201541 n=16 →
     下一事件 19.2s(2 格,turn+drive);175751 n=16→88.6s 与 201541 n=1→87.8s
     均恰为 90s 节律(下一 escape 按节律触发,是上界非周期长度)。逐格 hop 时长
     (DIAG dcell 变更间隔,n=225/214):p50=15.0s、p90=20.1s、p95≈25-30s,
     >30s 仅 3-5%(且即停滞驻留本身)。单 hop + 重感知 + 再路由 30s 内覆盖率
     ≥95%,19.2s 实测整周期留有余量 → 30.0 保留。另证:n=1→n=16 批发重开
     本身瞬时(175751 内相隔 0.5s)。
   - **dexit 排位合理性**:全部事件格 dexit ∈ [3.16, 9.22](最近 (7,8)=3.16,
     高频 (7,7)/(8,6)=3.61),无一进入出口邻域;出口邻格远端点 dexit 1.0-1.41
     << 3.16,故出口向切割候选在度量下严格排前于所有 ping-pong 区边——度量与
     锁死地理一致(完整切割候选重放归 Task 3 单测机械钉死)。
   - 正则适配:DIAG 格须用 `dcell=`(裸 `cell=` 会连带 `odomcell=`),phase 为
     斜杠复合词(`entering/center`)须用 `\S+`;ESCAPE/UNSTICK 正则与
     maze_motion 事件串逐字对上,未改。
