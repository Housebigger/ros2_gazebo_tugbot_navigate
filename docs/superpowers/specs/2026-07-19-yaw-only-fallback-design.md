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

(实现过程中的偏差与决策记在这里,同前四阶段惯例。)
