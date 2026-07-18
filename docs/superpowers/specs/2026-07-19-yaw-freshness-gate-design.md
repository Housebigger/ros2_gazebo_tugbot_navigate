# Yaw Freshness Gate Design (2026-07-19)

## 背景与目标

lidar-3d-swap 阶段(20260719,merge `ea5b108`)的 TIMEOUT 取证定位了一个 solver
固有机制:**转弯瞬间 yaw 陈旧 → `cell_wall_perp_dist` 把真实侧墙误分桶为"前方
0.6m" → 假 front_block STALL → 90s 周期逃逸循环**(入口在 (5,5) 中央交叉区,
随机触发,历史阶段约 1/5)。既有的 `front_block_max_yaw=0.30` 对齐门失明:
它与分桶用**同一个 yaw**,yaw 错时两者一起错(取证 STALL 行 `yaw_err=0.01`)。

本阶段做机制对症加固:给 `_front_blocked` 加**双信号门**(跨 tick 确认 + 转弯
冷却),两个信号都独立于该 yaw 的绝对值。**首次解冻导航链**,改动面收敛为
`maze_motion.py` 单文件。

## 工作空间

- 新 workspace **`ros2_ws_tugbot_nav_20260720`**,完整克隆 `ros2_ws_tugbot_nav_20260719`。
- `colcon build --symlink-install`;PRIME 由 run 脚本自带。

## 关键事实(实现前必读)

- 触发点:`maze_motion.py` 的 `_front_blocked(perp, dirn, moved, yaw)`(现约行 518):
  `perp[dirn] < front_block_m(0.7) and moved > 0.3 and |target_cardinal−yaw| < 0.30`。
- 分桶:`cell_wall_perp_dist(ranges, amin, ainc, yaw)` 在 `_drive` 每 tick 计算。
- 取证签名:误分桶是**单 tick 瞬态**(15 个 STALL 全部;5 跑中位相邻 DIAG 无复现);
  真前墙是**持续**短读。转弯活动(wz 指令、yaw 变化)是陈旧窗口的先导。
- solver tick:MazeMotion 由 solver 主循环驱动(scan 到达节奏,~10Hz);
  0.23 m/s 行走下多等 1 tick ≈ 2.3cm 额外接近,0.7m 阈值 vs 0.49m 足迹前伸,
  裕度充足。
- 真 front_block 必须保留:门只能过滤瞬态/转弯窗口,稳态短读照常触发。

## 组件:MazeMotion 双信号门(单文件)

文件:`src/tugbot_maze/tugbot_maze/maze_motion.py`

1. `__init__` 新增 kwargs(与 `front_block_m` 同风格,默认值即产品值,solver
   不需要传):
   - `front_block_confirm_ticks: int = 2` —— 连续短读确认数
   - `front_block_turn_cooldown_s: float = 0.5` —— 转弯活动后的冷却期
2. 新增内部状态:
   - `_fb_streak: int = 0`、`_fb_streak_dirn: Optional[str] = None` —— 连续
     短读计数;读数不短、或 dirn 变化时清零(换向的读数不可累计)
   - `_last_turn_t: float = -inf`、`_prev_yaw/_prev_yaw_t` —— 每 tick 更新:
     若 |上一 tick 输出的 wz| > 0.1 rad/s 或 |Δyaw|/Δt > 0.3 rad/s,
     刷新 `_last_turn_t = t`
3. `_front_blocked` 语义(在现有三条件之上叠加,原条件一律保留):
   - 每 tick:若 `perp[dirn] < front_block_m`(且现有 moved/对齐条件满足)则
     `_fb_streak += 1`,否则清零
   - 触发要求:`_fb_streak >= front_block_confirm_ticks` **且**
     `t - _last_turn_t >= front_block_turn_cooldown_s`
   - 冷却未过时不累计触发但**保留 streak 递增**(冷却结束后若仍短读,
     以最短延迟触发——真墙在冷却窗内逼近的场景不丢保护,只延迟)
4. STALL DIAG 行增加 `fb_streak=<n> cooldown=<ok|wait>` 字段(调试可见,
   格式向后兼容:追加在行尾)。
5. wedge 检测、centering、keepout、hop 机制一律不动;`front_block_max_yaw`
   原门保留不动。

## TDD 离线复现(核心验收)

测试:`src/tugbot_maze/test/test_maze_motion.py` 追加(或新文件
`test_front_block_gate.py`,以现有测试组织为准):

- **(a) 陈旧 yaw 复现场景(bug 铁证)**:合成 tick 序列——转弯活动刚结束
  (<0.5s)+ 单 tick 短读(下一 tick 恢复长读)。断言:**修复前代码触发
  front_block(红),修复后不触发(绿)**。红相位在实现前先跑当前代码确认。
- **(b) 真前墙保护(不许过杀)**:直行稳态(无转弯活动 >0.5s)+ 连续 ≥2 tick
  短读。断言:照常触发,且触发延迟恰为 confirm_ticks−1 个 tick。
- **(c) 边界**:冷却期内连续短读 → 冷却结束下一满足 tick 立即触发;
  streak 被长读清零;dirn 变化清零;wz 指令与 yaw 变化率两种转弯信号
  各自都能刷新冷却。
- 既有全套件无新增失败(基线 7 failed / 424 passed / 3 xfailed,
  失败名单逐字对照)。离线 motion-sim 端到端(test_maze_motion_sim /
  test_flood_fill_maze_sim)必须仍通过——门不得破坏正常解迷宫。

## Gazebo 验收(统计门)

- **headless ×6**(run 脚本自带 PRIME):全部 EXIT_REACHED、**零 TIMEOUT**、
  oracle 每跑 ≤0.719%(单 tick 边缘级;0.000% 为期望值)、ICP rms 基线域。
  对照:修复前 5 跑 = 3 清 + 1×0.719% + 1×TIMEOUT(22.3%)。
- **GUI ×1**(用户发起):EXIT + oracle 同门槛 + 用户确认。
- 若统计门失败(仍出 TIMEOUT):按 20260718 方法学先取证再动——不盲调参数。

## 风险与预案

- **过杀真前墙**(主风险):测试 (b) 铁证 + 统计门中 oracle 不得恶化;若
  出现撞墙类 flag 上升,优先缩短 cooldown 0.5→0.3,其次 confirm_ticks 2→1
  (即退回仅冷却门);决策记附记。
- **欠杀**(陷阱仍触发):机制多因素时可能残留;统计门捕获,届时按取证
  数据决定是否叠加 ICP 新鲜度门(B 案,本阶段不做)。

## 明确不做

- 不动 wedge/centering/keepout/hop;不做 B 案(ICP 新鲜度)与 C 案(感知
  冗余);不动投影/雷达/相机;不动 Nav2/ICP 参数;不改 STALL 逃逸机制本身。

## 附记(实现期决策记录)

**⚠️ 阶段结论(2026-07-19,用户裁决):负结果,分支 `yaw-freshness-gate` 停放不合并(先例:2026-06 footprint 分支)。** 门按设计工作但对真实机制无效;病根在定位层,solver 侧无米下锅。全量取证如下。

1. 实现完成且质量过审(分支提交 `7c6e61d`→`dc71069`→`706d360`):双信号门(confirm_ticks=2 + 0.5s 冷却)、行为红铁证(旧代码单 tick 闪报即 STALL,独立 worktree 重放确认)、被取代的单发合同测试 `test_front_block_gated_by_heading` 经授权原地升级、套件 7f/432p/3x 基线名单零漂移。
2. 评审网重大战果:0.3 rad/s 转弯活动阈值经实测被推翻——稳态 trot |yaw_rate|(0.1s 窗)mean 0.61 / p50 0.65 / p95 1.03 / p99 1.06 / max 1.09 rad/s,**77% tick 超 0.3**(步态摆动 ~2.02Hz、等效幅度 0.081 rad);旧阈值会让冷却永续刷新、门被静默禁用。阈值改 1.2(p99 上取整+0.1,具名常量引实测),命令转弯由 phase=='turn' 兜底,ICP yaw 跳变(≥4 rad/s)仍被捕获;工具 `tools/capture_trot_yaw_rate.py` 在分支上。
3. 统计门 6 跑:4 清(0.000%)+ 1 TIMEOUT(18.1%)+ 1 慢通(2650s,16.0%)——对照修复前 2/5 坏跑,**无改善**。
4. 机理判决(取证 BAD1=021704/BAD2=031719):27/27 STALL 全 cooldown=ok、15/15 front_block 全 fb_streak=2(结构封顶)——门没拦错任何东西;但假 front_block 是**站立性 yaw 偏差**非单 tick 闪报:(3,3)N 假墙(真值净空 2.56-3.06m)85 秒内同读数复发 6 次,每次 retry 偏差仍在,2 tick+0.5s 结构上过滤不了。唯一事件级假率 10/15(67%)。
5. B 案(ICP 新鲜度门)被数据否决:nan/航位推算是 **55-60% 的背景态**(online_slam 的 sparse-interior/no-local-committed 门控在未提交区域确定性拒配,非采样噪声);N=2 能压 5/7 假触发但同时压 3/4 真墙触发——真假都泡在 nan 窗里,solver 侧无区分信息。
6. 反向安全疑点(单样本):collision 相位分布变 drive 主导(72-85%),(5,5) 真墙场景中门确认前一 tick 足迹已处碰撞状态——confirm 延迟在真墙场景可能倒贴裕度。
7. 清跑机制:同样穿越陷阱格,区别是 retry 前是否恰好落下一个新鲜 accepted ICP 修正(时机运气);90s 逃逸看门狗设定活锁外周期。
8. **病根定位(下阶段候选)**:定位层——未提交区域的长航位推算窗。方向:开大 ICP 接受域/更激进的墙体提交策略/未提交区域的专用 yaw 修正。需独立 spec。
9. 分支保留价值(如未来复用):行为红测试基础设施、fb_streak/cooldown DIAG 字段、trot yaw_rate 实测工具与数据。
