# Yaw-Only Fallback Design (2026-07-19)

## 背景与目标

yaw-freshness-gate 阶段(负结果,归档见其 spec 附记)锁定病根:online_slam 的
ICP 门控在未提交区域**确定性拒配**(nan/航位推算占 55-60% 背景态),长航位推算
窗里的**站立 yaw 偏差**喂养中央路口陷阱(历史触发率 ~20-30%)。结构性事实:
**路口格本身几乎没有墙**(3-4 出口 = 0-1 段墙),locality 门(1.0m 内须有内部
墙)在路口中心天然失败——陷阱住在路口不是巧合。

本阶段做定位层修复:**yaw-only 回退修正(1-DOF)**。全 ICP 被门控时,仅解转角、
位置保持 odom 先验。理论根基:2026-06 的"根本墙"是二轴**位置**在开放区不可
观测;**yaw 从来可观测**(墙的方向在远处约束航向,周界墙亦然)。1-DOF 结构上
不可能复现历史位置腐败,却每 tick 扶正 yaw,抽走误分桶的燃料。

## 工作空间

- 新 workspace **`ros2_ws_tugbot_nav_20260721`**,完整克隆 `ros2_ws_tugbot_nav_20260719`
  (20260720 名字属停放分支 yaw-freshness-gate,避免目录冲突)。
- `colcon build --symlink-install`;PRIME 由 run 脚本自带。
- **20260720 停放分支的双信号门不带过来**——本阶段从 main(20260719 状态)出发。

## 关键事实(实现前必读)

- `online_scan_match_localizer.py`(193 行)结构:`OnlineScanMatchLocalizer.correct()`
  四步——Gate 1 sparse_interior(interior_segments < min_interior_segs=1)、
  Gate 2 no_local_interior_walls(1.0m 内无内部墙端点/中点)、Gate 3 远束预掩膜
  `_mask_far_beams`、ICP(under_inliers 拒配)。三条拒配路径都返回
  `(prior_pose, {'rejected': True, 'reason': ...})` = 纯航位推算。
- 参考 = 周界(永在)+ `local_reference_cells(committed, current, sensed)` 的
  内部墙;`ScanMatchLocalizer`(scan_match_localizer.py,167 行)是底层 ICP,
  持有点-段残差机制(20260614 起未动)。
- 拒配率 55-60% 是背景态;站立 yaw 偏差episode可达 85s(yaw-freshness-gate 取证)。
- 求解器消费:`flood_fill_solver._lookup_scan_match` 拒配时退 odom 合成先验。
- DIAG `MATCH rms=...` 行是事后审计与 oracle 的输入链。

## 组件:yaw-only 回退(单文件 + 测试)

文件:`src/tugbot_maze/tugbot_maze/online_scan_match_localizer.py`

1. **纯函数 `yaw_only_correct(prior_pose, ranges, angle_min, angle_inc, icp)`**
   (模块级,icp 为持有全参考的 ScanMatchLocalizer;ranges 为已预掩膜光束):
   - 在 ±`YAW_WINDOW_RAD = 0.2` 窗内解最优 dθ:对候选 dθ 把光束端点按
     `(x, y, yaw+dθ)` 投影,复用 icp 的点-段最近距离残差,黄金分割(或等价
     一维搜索)最小化 rms;
   - **接受条件**:落在参考近旁的 inlier 束数 ≥ `YAW_MIN_INLIERS = 60`,且
     最优 dθ 的 rms 相对 dθ=0 改善 ≥ `YAW_MIN_IMPROVE = 0.20`(20%);
   - **钳位**:应用的 |dθ| ≤ `YAW_STEP_CLAMP = 0.1` rad(超窗最优也只走 0.1);
   - 返回 `((x, y, yaw+dθ_clamped), {'rejected': False, 'reason': 'yaw_only',
     'yaw_step': dθ_clamped, 'residual_rms': rms, 'n_inliers': n})`;
     不达标返回 `(prior_pose, {'rejected': True, 'reason': 原拒配理由 +
     '+yaw_only_declined'})`。**x, y 在任何路径下逐位不变。**
2. **接线**:`correct()` 的三条拒配路径(sparse_interior_gate、
   no_local_interior_walls、ICP under_inliers)统一改为:先
   `yaw_only_correct(...)`,其接受则返回之;否则按原样返回纯先验。
   注意 under_inliers 路径的 ranges 已premask,前两条路径需先跑
   `_mask_far_beams` 再喂 yaw-only(远束会污染 1-DOF 拟合,同理需掩膜)。
3. 常量具名 + 注释引用本 spec 的机理(约束式注释)。求解器与 MazeMotion
   一律不动;DIAG 行自然携带新 reason(solver 的 MATCH 日志打 info dict,
   无需改动——实现时核实字段兼容,若 solver 对 `rejected=False` 有额外
   假设(如读取 fb/eigmin 字段)以实测为准适配日志侧,不改行为)。

## TDD 离线(核心验收)

新测试文件 `test_yaw_only_fallback.py`:

- **恢复**:合成"路口"场景(仅周界参考 + 机器人在迷宫中部),注入 yaw 偏差
  0.05 / 0.15 / 0.3 rad(0.3 超窗)→ 修正后 |yaw 残差| 显著缩小(0.3 案受
  0.1 钳位,断言单步走满 0.1 方向正确);**x, y 逐位不变**(每案)。
- **拒绝**:光束与参考完全无关(空旷/全远)→ 干净回退,reason 带
  `yaw_only_declined`,pose 逐位等于先验。
- **收敛不振荡**:同一场景反复调用(模拟连续 tick,每次用上次输出为先验)
  → dθ 序列单调收缩,3-5 tick 内 |残差| < 0.02 rad,无翻摆。
- **改善阈值**:构造 rms 改善 <20% 的场景(偏差本来就≈0)→ 拒绝(防
  噪声驱动的空修正)。
- 既有 ICP 测试(test_online_scan_match_localizer.py、
  test_scan_match_localizer.py)零破坏;全套件无新增失败(基线
  7 failed / 424 passed / 3 xfailed,名单对照)。

## Gazebo 验收(统计门,用户已裁)

- **headless ×8**:全 EXIT_REACHED、**零 TIMEOUT**、oracle 每跑 ≤0.719%
  (期望 0.000%)、rms 基线域。若真实陷阱率仍 ~30%,8 连清概率 <6% ——
  强证据。分批跑(3+3+2),任一坏跑立即停批取证(方法学惯例)。
- **行为证据**:DIAG 中 `yaw_only` 触发率(应显著存在,替代原纯拒配)、
  yaw_step 分布(应小而频,无 0.1 钳位饱和连串)。
- **GUI ×1**(用户发起)+ 用户确认。

## 风险与预案

- **yaw-only 自身振荡/劣化**:收紧钳位(0.1→0.05)或抬高改善阈(20→35%);
  最后手段:参数开关 `enable_yaw_only=False` 停用回退(构造器 kwarg,默认开)。
- **远墙错配拉偏 yaw**:premask + inlier 阈值 + 改善阈三重守门;若统计门
  出现新型劣化,停批取证对比 yaw_step 时间线与真值。
- **陷阱仍触发**(yaw 偏差非唯一燃料):停批取证;本阶段不叠加其他方案。

## 明确不做

- 方案 A(locality 半径放宽)、B(参考富集)、C(提交激进化);不动
  flood_fill_solver / maze_motion(停放分支的双信号门不回归);不动 2-DOF
  位置逻辑与既有 ICP 参数;不动传感器/投影层。

## 修案(2026-07-19,统计门第一批取证后,用户批准)

第一批 3 跑(1 清 + 2 TIMEOUT)触发停批取证,结论重写了阶段认知:

- **yaw-only 无罪但无效**:400+ 几何试验中接受步方向零次恶化(H-耦合反证);
  但位置漂移 d≥0.1m 时接受率坍缩至 0-25%——它在关键场景恰好拒绝,无保护力。
- **82.9% oracle 是卡死驻留伪影**(571/595 flag 为同一冻结位姿采样 47 分钟);
  活体率 R2/R3 = 17.3%/16.3%,均在修前带内——本批统计上与基线不可区分。
- **真凶 = clamp-lock**:R3 终态 ICP 每 tick 收敛(n=421、eigmin=22)但修正量
  超钳位 → `ScanMatchLocalizer` 的 reject-if-exceeds 门**整体丢弃**修正,575 次
  连续拒绝直到超时。栈内无任何机制能花掉位置误差。

### 修案组件(在原组件之上追加)

1. **clamp-lock 逃逸**(主组件):
   - `scan_match_localizer.py` 仅加**纯诊断字段**:clamp 拒绝路径的 info 附
     `'conv_pose': pose`(被丢弃的收敛位姿)与既有字段并存——行为零改动
     (scan_match 模式忽略多余键)。
   - `OnlineScanMatchLocalizer` 维护 `_clamp_streak`:内层拒绝为 clamp 型
     (rejected 且非 under_inliers 且 `n_inliers >= CLAMP_LOCK_MIN_INLIERS=100`)
     时 +1,任何接受或非 clamp 拒绝清零。
   - streak `>= CLAMP_LOCK_STREAK=3` 时**逃逸**:从先验向 conv_pose 应用受限
     步——平移钳 `CLAMP_ESCAPE_TRANS=0.15` m、yaw 钳 0.1 rad(与 yaw-only 同
     常量);返回 rejected=False、reason='clamp_lock_escape'、
     `fell_back={'clamp_escape'}`(MATCH 行免费显示 fb=clamp_escape)、
     conv 距离等诊断;逃逸后 streak 清零(每 3 次健康拒绝至多一步,天然限速)。
   - 优先级:clamp-lock 逃逸在 yaw-only 之前判定(它有 2-DOF 收敛证据,信息
     严格更多)。
2. **yaw-only 保险**(取证建议):接受额外要求 `best_rms <= YAW_RMS_CEILING
   = 0.15`(取证:会拒掉 R3 最弱接受 n=182/rms=0.152;墙半厚 0.12 下健康
   拟合 ≈0.11)。|wz| 门被否:需改 solver 接口,且该通道未被定罪。
3. **卡死驻留度量修正**:`tools/replay_collision_oracle.py` 在官方率旁增报
   `live_rate`(连续逐位相同 (x,y,yaw) 的样本去重后重算)——官方率语义不变。
4. **DIAG 增强**(解冻 flood_fill_solver.py **仅日志行**):MATCH 行尾追加
   `ystep=%.3f gr=%s`(info.get('yaw_step', 0)、info.get('gate_reason','-'));
   行为零改动。5s 采样对取证的伤害本批已实证。

### 修案验收

- 新组件 TDD(clamp 型判定、streak 语义、受限步数值、优先级、conv_pose 字段、
  rms 上限、oracle live_rate 对 R3 工件回放 ≈16.3%)。
- 统计门重置:**headless ×8 重新计数**(分批 3+3+2,停批取证同前);判定
  采用官方 oracle 率(≤0.719%)与 live_rate 双报告,门槛按官方率。

## 附记(实现期决策记录)

1. Task 2 实现期合同修正(控制器裁决):纯 rms-argmin 在低内点数下非单调,超窗场景选错符号(实测 +0.02 vs 应 −0.2)——主选改内点数 argmax、rms 决胜;接受门分支化(内点最优:n≥60 + rms 改善≥20%;饱和/窗沿:相对内点增幅≥20% + 绝对下限 n≥30,下限补于 n0=0 时 improve=inf 无条件放行的数值洞,实测 4 内点错向接受被封);钳位裁回无条件 ±0.1/tick(与全 ICP 的 reject 门哲学不同——本回退是饱和钳位,幅度更保守 0.1<0.175);计划自带测试矛盾(bias 0.15 单调用 vs 无条件钳位)修正为两调用收敛。
2. 接线期发现:内层 ICP info 无 'reason' 键(通用 'icp_rejected' 兜底);_mask_far_beams 原对含 inf 光束先乘后滤触发 NaN RuntimeWarning(gate-1/2 多 inf 路口扫描首次流经)——改有效光束前滤,300/300 差分下游等价;夹具房间中心 61/360 束贴 60 下限(生产 900 束≈×2.5 裕度);bias 0.19 带(0.19–0.25)实证"全 ICP 钳位拒绝而 yaw-only 接受"的存在意义。
3. ⚠️ 统计门第一批灾难与平反:1 清 + 2 TIMEOUT(其一 oracle 82.9%)。取证(400+ 几何试验):H-耦合反证(纯位置误差仅 ~0.5% 买到接受,接受步方向零恶化);82.9% 系卡死驻留伪影(571/595 为同一冻结位姿 47 分钟采样),活体率 16-17% 在修前带内;真凶 = clamp-lock——内层 ICP 每 tick 收敛(n=421、eigmin=22)但 reject-if-exceeds 门 575 连续整体丢弃,栈内无机制能花掉位置误差。
4. 修案(用户批准):clamp-lock 逃逸(conv_pose 诊断字段 + 3 连健康钳拒 → 向收敛位姿限步 0.15m/0.1rad,自阻尼无失控)、yaw-only 内点分支 rms 上限 0.15(饱和分支豁免——合法大偏差下行本落 0.15-0.17)、oracle live_rate 双报告(逐位去重)、MATCH 行 ystep/gr 增强(5s 采样的取证之痛)。
5. 重启统计门 8 跑(修案栈):**6 清**(5×0.000% + 1×0.690% 单 tick)+ 1×P3 + 1×MIXED(P1+P3)。**P2 = 0**(全程真钳型拒绝最长连串 1,逃逸零触发为正确行为);yaw_only 共 51 次良性触发(全部 gr=no_local_interior_walls 前沿场景,步长 ≤0.1,前后 flag 密度与背景无差)。
6. 失败分类学(本阶段取证产物):P1 路口陷阱(定位类,yaw 时序/无墙路口)、P2 clamp-lock(定位类,位置钳拒)、P3 探索图耗尽(路由类:UNSTICK 强拆已佐证墙、90.1s no-progress 节律 gave_up_edge=False 91%、出口邻格两至而背出)。TIMEOUT ~30% 底噪为三类混合——"零 TIMEOUT"门对定位阶段按构造不可达成。
7. 门裁定(用户,按类分门后):字面未达(P1 残留 1/8:run 201541 的 (6,3) 无墙 T 路口 66s 航位风暴,**跑内自愈**、未致超时,其超时直接原因为 P3)。P1 残留本质是**可观测性边界**——无墙=无参考,任何接受策略无米下锅;修复需感知/建图侧信息(如更快提交、路口专用行为),入 backlog。用户裁决:诚实合并——栈经 8 跑 + 453 测试验证无害且轻度有益(前沿 yaw 修正 + P2 类风险消除),门结果如实归档。
8. 递延 backlog:P1 残留(路口可观测性,感知/建图侧);P3(路由层:UNSTICK 全拆策略、no-progress 节律、出口邻格背出不利用);gr= 字段在 declined 行恒为 '-'(仅接受路径填充,分类取证靠 reason 字符串——如需日志级门型分布需补一行日志);饱和分支天然 rms 边界夹具未构造成(以 kwarg 边界测试代,差距诚实记录)。
9. **GUI 验收(2026-07-19,用户画面确认通过)**:EXIT_REACHED @832s,oracle 官方 0.000%(166 采样零碰撞)/ live_rate 0.000%;`fb=yaw_only` 6 次、全部 `gr=no_local_interior_walls`、`ystep` 0.06–0.10 rad(饱和 clamp 内);`fb=clamp_escape` 0 次(P2 逃逸待命未动用);ICP rms 中位 0.123 / 最大 0.234(基线域)。与 headless 统计门画像互相印证(约 6.4 次良性 yaw_only/跑)。阶段按用户"诚实合并"裁决进入合并收尾。
