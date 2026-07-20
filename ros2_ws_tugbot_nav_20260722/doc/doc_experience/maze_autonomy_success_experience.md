# 自主走迷宫通关 · 成功经验

> 工作空间：`ros2_ws_tugbot_nav_20260614`　|　成文：2026-07-01（历程跨 2026-06）
> 一句话结论:**真正的杠杆是「诚实的测量 + 正确的诊断 + 把约束当资产」,而不是更聪明的搜索算法。**

---

## 1. 背景:一个纠缠数周的平台期

目标:让 Tugbot 在一个已知的 ~10×10 格完美迷宫(树结构,2m 格,~1.76m 走廊)里,从入口 `(0,0)` 自主走到出口 `(21.07,18.08)`。

先后试遍了多种"探索/路线"策略,全部卡在离出口 ~8–11m 的平台期:

- **DFS / Trémaux / frontier**:决策逻辑干净、单测充分,但 Nav2 点目标在 1.76m 窄走廊里**楔住**(wedge→exclude→relocate 循环),覆盖率被封顶,~8.7–11m 处止步。
- **反应式 wall-following**:一度报"5/5 通关",后被识破是**作弊**——入口和出口都在外边界上,它沿外墙外侧绕过去了;封住入口强制走内部后,无状态的跟墙会在子区域里打转。
- **格栅 flood-fill(micromouse)**:补上了前两者缺的两样——**记忆**(格图,无别名)和**可靠走廊运动**(`MazeMotion`);它逃出了历史陷阱、走得很远,但又撞上**开放区两轴定位墙**:内部远离参照墙、远离入口,里程计漂移 + 偏离中心误感 → 幻象通道 → 误路。

数周努力始终在"假定的瓶颈(搜索)"上加码,而真瓶颈在别处。

## 2. 突破:瓶颈是「定位」,不是「搜索」

一轮**全局回顾 + 受控批量诊断**把失败模式量化:用里程计死推(`pose_source=odom_locked`)时,**75% 的运行 desync**——机器人自以为居中实则漂移,于是误感墙、误路(odom 下 0/8 通关,62% DESYNC 主导,100% 触发 RECONCILE)。瓶颈是**机器人不知道自己在哪**。

迷宫是**已知**的,于是关键的重构是:**用已知地图做定位,而把路线发现留给自主。**

> `scan_match_localizer.py` —— 每个 tick 用实测 360° `/scan`,以 odom 增量为先验,做 **point-to-line ICP**,把激光端点对齐到已知墙段中心线(真实非对称足迹,按轴做可观测性门控),输出**绝对 map 位姿**,消除累积漂移。路线仍由 flood-fill **自主发现**——即"已知地图定位、未知路线探索",不是预计算路径。

定位一准,失同步归零,原有自主性自然生效:

| | odom 基线(无定位修复) | **scan-match** |
|---|---|---|
| 自主通关 | 0/8 | **16/16(两批受控实验)** |
| 用时 | 全部超时 | **~560s/次**(与预计算的 GCN 同速) |
| DESYNC 主导 / RECONCILE | 62% / 100% | **0% / 0** |
| 真实足迹碰撞率 | ~25–61% | **~0.33% → 0%**(见 §4:2026-07-03「倒车回中」修复清零) |

把"已知地图"从"GCN 作弊的理由"重新定义为"合法的定位资产",是整件事的转折点。

## 3. 可迁移的方法论经验

1. **先量准瓶颈,再优化。** 全局回顾 + 受控诊断先确认"该修定位",才避免了继续在搜索/运动上空耗。卡点不在你以为的地方时,再努力也白费。

2. **先埋点,再推断;诚实地量。** 真实足迹碰撞 oracle(`maze_sim.collides` + `footprint.py`)揭穿了"虚构 0.35m 圆"把真实 ~25–61% 碰撞粉饰成的"15%"。这把尺子重新定义了项目的真实状态;后来 Gazebo oracle 回放又两次抓住离线测试看不见的回归。

3. **用分布替代轶事。** `tools/batch_diagnose_floodfill.sh` 跑 N 次同条件、自动出失败模式分布(`diagnose_floodfill_runs.py`),取代"看一次跑得怎样"。本项目运行间非确定性极强,单次会骗人,分布不会。

4. **数据说不行就砍掉自己的心血——而失败的数据本身会指向真解。** wall-following 的"5/5"被识破作弊;为消除 0.33% 残留而建的 **rotation-sweep guard**——经完整 spec→plan→subagent-TDD→多轮评审建好——Gazebo 一测是回归(0.33%→0.43%→4.43%,反而更糟)→ **果断放弃、不合并**。但放弃不是终点:正是"gate 旋转反而更糟"这条回归数据,证伪了"旋转门控"这个方向,把注意力逼向**居中**——后来的「倒车回中」修复据此把残留**清零**(见 §4)。"建好却不上线一个失败的改动"本身是成功;而**诚实测量一次失败,往往比多做十次盲目尝试更能指明真解**。

5. **分层验证,知道哪层对哪个结论权威。** 离线单测管逻辑(快、确定);真实足迹 oracle 管碰撞真相;受控 Gazebo 批量管真实分布。离线 sim 复现不出 backout 擦碰,所以碰撞消除只能由 Gazebo 定论——**用错层下结论会翻车**。

6. **流程纪律产生质量。** 每个改动走 brainstorm→spec→plan→subagent-TDD→两段评审(规格符合性→代码质量)→Gazebo 验收→才 bank。两段评审抓到真 bug(`_associate` 数组别名、法向量朝向导致梯度翻转);"先在 Gazebo 证明再合并"这道闸两次挡住回归。

**元经验:长平台期后要"退一步"重新诊断,而不是在假定瓶颈上更用力地磨。** 数周搜索算法工作平台期;一个几百行、瞄准真瓶颈的定位改动,一天内通关。

## 4. 残留的收尾:从放弃 guard 到诊断清零(2026-07-03)

那处 0.33% 残留最终被**彻底清零**,过程本身就是全文方法论的一次缩影:

1. **先诊断,别猜。** 把两批 16/16 基线的 DIAG 位姿用真实足迹 oracle 回放(`tools/replay_collision_oracle.py`,复现 6/1808=0.332%,全在 (3,9)),逐帧还原机制:小车从 (3,8) 朝**正北** hop 进 (3,9),**一致过冲到中心以北 ~0.4m**;`centering_command` 为修这个北偏,**原地转 ~180° 去朝南**——而在偏心位做这个大旋转,后夹爪(角半径 ≈0.55m)正好扫过北侧外围墙。起终点都不撞,只有**旋转途中的弧线**擦墙(所以是 0.33% 的瞬态)。
2. **失败的 guard 反而指了路。** 之前的 rotation-sweep guard 去 gate 旋转、越修越糟——因为**闯祸的正是"为回中而转"这个动作**。这条证伪把方向从"旋转门控"逼向"居中"。
3. **把约束当资产的修法。** 差速车不能横移,**但能倒车**。于是让 `centering_command`:当待修的轴与当前朝向**共线但方向相反**(中心在身后=沿行进方向过冲)时,**倒车纯平移回中**,取代转 180°;倒车退回的是**刚开过来、必然空的**走廊,天然安全;垂直轴的横向居中不动。改动只在一个函数。
4. **分层验证到底。** 单测(倒车分支)→ 离线真实足迹 sim(**顺带清掉了 3 个 drift=0.05 的历史擦碰**)→ 受控 Gazebo:**16 次 post-fix(两批)16/16 通关、碰撞 0/1788 = 0.000%**(基线 6/1808)。设计/计划见 `docs/superpowers/{specs,plans}/2026-07-03-reverse-to-center-boundary-overshoot*.md`。

从"带残留的成功"到"残留清零",靠的不是更用力,而是**把一次诚实的失败当作诊断信号**。下一个人从真相起步。

## 5. 关键工件索引

- **核心代码**(均在 `src/tugbot_maze/`,`main`):`scan_match_localizer.py`(定位)、`maze_motion.py`(运动 FSM)、`flood_fill_brain.py`(路线)、`cell_walls.py`(感墙)、`maze_sim.py`(测试用真实足迹 oracle)、`footprint.py`(几何)、`pose_tracking.py`(SE(2) 先验)。
- **设计/计划**:`docs/superpowers/specs/2026-06-28-scan-match-localization-design.md` + 同名 plan;早期诊断见 `docs/superpowers/specs/2026-06-28-perimeter-absolute-reanchor-design.md` 等。
- **诊断工具**:`tools/diagnose_floodfill_runs.py`、`tools/batch_diagnose_floodfill.sh`、`tools/run_flood_fill_maze.sh`(`pose_source` 默认 `scan_match`)、`tools/replay_collision_oracle.py`(真实足迹碰撞回放,量化 (3,9) 残留与清零)。
- **(3,9) 残留修复**:`hop_controller.centering_command`(倒车回中)+ `docs/superpowers/{specs,plans}/2026-07-03-reverse-to-center-boundary-overshoot*.md`。
- **离线套件**:`test/test_scan_match_localizer.py`、`test_pose_tracking.py`、`test_maze_motion_sim.py`、`test_maze_sim.py`、`test_flood_fill_brain.py`、`test_hop_controller.py` 等。
- **受控基线/验证**:`log/batch_diag_20260628_232253` 与 `…_20260629_071328`(各 8/8,pre-fix,0.33%);`…_20260703_081852` 与 `…_20260703_160727`(各 8/8,post-fix,0%)。
- **已 park 分支**:`rotation-sweep-clearance-guard`(失败的残留修复,其证伪数据指引了后来的倒车回中解)、`footprint-collision-model`、`offcenter-robust-sensing`。

> 注:仓库历史于 2026-06-29 做过一次清理(git-filter-repo 剥离 log/build/install 大文件)重写了所有提交 SHA;按提交信息(而非 SHA)定位历史更可靠。

## 6. 里程碑时间线

```
DFS/Trémaux/frontier 平台期 (~8.7–11m)
  → wall-following「5/5」(后证为外墙作弊)
  → flood-fill:补上记忆 + 走廊运动,但撞上开放区定位墙
  → 全局回顾 + 受控诊断:定位 desync 是真瓶颈(75%)
  → scan-match 已知地图定位:16/16 自主通关,碰撞 ~0.33%(已银行到 main)
  → 残留 (3,9) 擦碰:rotation-sweep guard 尝试失败(gate 旋转反而更糟)→ 放弃,但其数据证伪了"旋转门控"
  → oracle 回放定位机制(过冲+180°回中旋转)→「倒车回中」修复:16/16 post-fix、碰撞 0/1788 = 0%(已银行到 main)
```
