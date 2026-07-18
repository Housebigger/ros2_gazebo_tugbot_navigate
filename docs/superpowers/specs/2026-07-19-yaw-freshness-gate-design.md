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

(实现过程中的偏差与决策记在这里,同前三阶段惯例。)
