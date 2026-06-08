# Tugbot Maze Navigation Phase 1-19 阶段性总结与后续改进方向

生成时间：2026-05-23

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 0. 最初目标回顾

最初目标可以概括为：

在 ROS 2 Jazzy + Gazebo 中，为 Tugbot 构建一个迷宫自主导航工作空间：机器人从入口出发，依靠运行时传感、SLAM、Nav2、TF、地图反馈自主探索，最终到达出口。迷宫图片只允许用于生成 Gazebo 世界和定义入口/出口真值，不允许在运行时直接作为路径答案。整个过程按 phase-gated、TDD、contract test、headless smoke、JSONL metrics 的方式推进，避免凭单次日志猜参数。

## 1. 当前总体状态

当前总体状态：

- 工作空间可以构建、测试、运行。
- `maze_explorer` 已经具备 DFS/topology exploration、Nav2 action lifecycle hardening、structured goal diagnostics、geometry diagnostics、Nav2 log correlation、local costmap embedded diagnostics。
- Phase 19 smoke 最终达到：
  - `EXIT_REACHED`
  - `exit_distance_m: 0.5451`
  - `blocked_branch_count: 0`
  - `blacklisted_goal_count: 0`
- 但 timeout 仍未解决：
  - Phase 17 timeout: 1
  - Phase 18 timeout: 2
  - Phase 19 timeout: 3
- 所以当前接受的是“诊断体系/鲁棒性阶段性完成”，不是“导航策略已经优化完成”。

## 2. Phase 1-19 阶段汇总

### Phase 1：工作空间与 ROS/Gazebo 基线

目标：

- 建立新的 ROS 2 + Gazebo Tugbot workspace。
- 保留已验证 baseline，不直接破坏旧工作区。
- 确认 Jazzy/Gazebo/Nav2/slam_toolbox/ros_gz_bridge 可用。

成果：

- 新 workspace 可 build。
- 基础 bringup/launch/package/test 结构建立。
- 明确运行模式分离原则：
  - SLAM mapping
  - SLAM + Nav2
  - autonomous exploration
  - static-map replay

意义：

- 给后续 maze-specific 开发提供安全实验环境。

### Phase 2：迷宫世界与任务真值

目标：

- 从迷宫/场景参考生成 Gazebo 世界。
- 定义入口、出口、出口半径、运行时策略边界。

成果：

- 入口/出口真值进入配置/metadata。
- 明确：图片是 world generation truth，不是 runtime planning answer。
- 开始形成 Tugbot maze 专用 bringup 和 assets/config 结构。

意义：

- 把“迷宫任务”从普通 Nav2 demo 变成有成功判据的场景任务。

### Phase 3：迷宫几何与可通行性验证

目标：

- 确认 maze corridor 对 Tugbot 尺寸可行。
- 避免因为世界几何太窄导致后续调参误判。

成果：

- 增加 corridor width / robot turning envelope 检查。
- 以 STL/尺寸推导 Tugbot 转弯外径。
- 检查 SDF 墙体间距不低于机器人通行阈值。

意义：

- 把“导航失败”先从“世界不可通行”中分离出来。

### Phase 4：DFS/topology/perception 基础

目标：

- 不依赖预计算路径，实现迷宫 DFS 拓扑探索基础。
- 建立 branch、junction、dead-end、backtracking 等抽象。

成果：

- 增加 `tugbot_maze` package。
- 实现 MazeTopology、BranchOption、branch state、dead-end/backtrack 逻辑。
- 增加 occupancy-grid 纯 Python 工具和测试。
- 加入 centered branch goal、target blacklist、失败分类雏形。

意义：

- 形成“机器人看地图、识别局部拓扑、选择分支”的核心算法基础。

### Phase 5：ROS 2 maze_explorer 节点与 launch smoke

目标：

- 把纯 Python DFS/topology 集成成真实 ROS 2 node。
- 接入 `/map`、TF、Nav2 NavigateToPose。
- 在 Gazebo headless 下跑通。

成果：

- 新增 `maze_explorer`。
- 发布 `/maze/explorer_state`。
- autonomous launch 默认使用 DFS explorer，可保留 frontier fallback。
- headless smoke 验证 Gazebo + SLAM + Nav2 + explorer 基本工作。

意义：

- 从算法测试进入真实仿真闭环。

### Phase 6：较长 DFS exploration 与 reverse-direction 实验

目标：

- 不以一次是否到出口作为唯一标准，先看探索是否持续增长。
- 观察 known_junctions、edges、goal_success_count、failures。

成果：

- 长时间 JSONL 记录 explorer state。
- 发现机器人在 junction 附近可能反复选择反向/近似反向目标。
- 尝试 reverse-direction filtering。

结论：

- blanket reverse filtering 不是万能；可能增加 Nav2 status=6 或 stagnation。
- 后续需要 contextual reverse allowance。

### Phase 7：preemption/stale-result 问题定位

目标：

- 分析长 run 中 goal failure 是否真的是 blocked_nav2。
- 检查 Nav2 action preemption 与 stale result。

成果：

- 通过 timeseries/log 发现很多失败是旧 goal result 回来后污染当前状态。
- 识别 `Received goal preemption request` / stale result churn。

结论：

- 不能把所有 action failure 都当作真实 blocked branch。
- 需要 goal sequence ID 和 stale-result handling。

### Phase 8：goal preemption hardening

目标：

- 防止旧 Nav2 result 污染当前 goal。
- 恢复更合理的 reverse branch 行为。

成果：

- 增加 goal sequence IDs。
- 增加 stale/preempted result ignoring。
- 引入 `GOAL_PREEMPTED`。
- 增加 settle/cooldown，避免连续发 goal 导致 preemption churn。
- 恢复 contextual reverse allowance。

意义：

- topology 污染大幅降低。

### Phase 9：preemption hardening 验证

目标：

- 用 Phase 7 vs Phase 9 JSONL 对比验证硬化效果。

成果：

- 真正的 `blocked_nav2` 明显下降。
- stale/preempted result 被单独计数。
- topology growth 更稳定。

结论：

- Phase 8 的 lifecycle hardening 有效。
- 下一问题变成 terminal/timeout 后 active goal 如何处理。

### Phase 10：terminal cancel lifecycle

目标：

- 到达出口或 exhausted 后，正确处理仍在运行的 Nav2 goal。
- 不把 terminal cancel result 误记成失败。

成果：

- 进入 terminal state 时取消 active goal。
- 新增分类：
  - `GOAL_CANCELED_AFTER_EXIT`
  - terminal cancel result
- 发布更清晰状态/日志 token。

意义：

- 到达出口后的 action result 不再污染 failure counters。

### Phase 11：timeout cancel lifecycle

目标：

- goal timeout 后主动 cancel Nav2 goal。
- 把 timeout-canceled result 从普通 failure 中剥离。

成果：

- timeout 后 cancel goal。
- 新增：
  - `GOAL_CANCELED_AFTER_TIMEOUT`
  - timeout cancel counters
  - `timeout_cancel_goal_sequence_id`
- recorder 支持 terminal linger，保留 terminal 后几秒事件。

意义：

- timeout 和 cancel result 的因果关系开始清楚。

### Phase 12：长 run 验证 Phase 10/11

目标：

- 验证 action lifecycle hardening 后能否稳定到出口。

成果：

- long-run 达到 `EXIT_REACHED`。
- terminal cancel 和 timeout cancel 被正确分类。
- 没有 blocked/blacklist topology pollution。

结论：

- action lifecycle 基本稳定。
- 剩余核心问题转向：为什么仍有 timeout。

### Phase 13：timeout root-cause 初探

目标：

- 分析 Phase 12 多出来的 timeout 是否和 near-exit 有关。
- 增加 near-exit timeout policy 诊断。

成果：

- 新增 effective timeout diagnostics。
- 增加 near-exit timeout 参数：
  - `near_exit_goal_timeout_sec`
  - `near_exit_timeout_extension_radius_m`
- 对 near-exit goal 使用更长 timeout。

结论：

- near-exit timeout policy 有帮助但不是完整根因。
- timeout 分布仍然较分散。

### Phase 14：structured `/maze/goal_events`

目标：

- 不再只看 `/maze/explorer_state` 总体状态。
- 增加 per-goal structured event，便于 success vs timeout 对比。

成果：

- 新增 `/maze/goal_events`。
- 每个 goal 记录：
  - dispatch
  - success
  - timeout
  - failure
  - terminal_cancel
  - stale/cancel result
- 记录 goal_sequence、target、dispatch_pose、elapsed、reason、branch state 等。

意义：

- 从“最终状态分析”转向“每个 goal 的因果分析”。

### Phase 15：goal_events 与 Nav2/controller log correlation

目标：

- 自动 join goal_events 和 Nav2/controller logs。
- 看 timeout 是否与 progress checker / recovery / abort cluster 相关。

成果：

- 新增 `tools/analyze_goal_events_with_nav2_log.py`。
- 输出 per-goal：
  - progress_failure_count
  - recovery_count
  - controller_abort_count
- 发现关键 timeout goals 常伴随 Nav2 progress/recovery/abort clusters。

结论：

- 部分 timeout 看起来是 Nav2 controller/progress 问题，而不是 DFS 状态机问题。
- 进入 Phase 16 参数假设验证。

### Phase 16：Nav2 progress/controller 参数假设

目标：

- 测试更宽松 progress/controller profile 是否能减少 timeout。

成果：

- 增加 Phase 16 Nav2 progress/controller tolerance profile。
- 运行 smoke 对比。

结论：

- progress failure logs 下降，但 timeout/blacklist 反而变差。
- 拒绝该 profile 作为 fix。
- 确立原则：
  - 不能只因 `Failed to make progress` 变少就接受参数。
  - 必须看 timeout、blocked、blacklist、EXIT_REACHED。

### Phase 17：geometry diagnostics

目标：

- 不直接改 branch selection。
- 先把 `/map` geometry 信号加入 goal_events。

成果：

- `/maze/goal_events` 新增 geometry fields：
  - `target_cell_occupancy`
  - `target_clearance_m`
  - `line_of_sight_occupied_count`
  - `line_of_sight_unknown_count`
  - `line_of_sight_min_clearance_m`
  - `path_corridor_min_clearance_m`
  - `target_near_wall`
  - `target_crosses_wall_corner`
  - `target_crosses_narrow_passage`
  - `target_yaw_corridor_conflict`
- Phase 17 smoke 最终：
  - `EXIT_REACHED`
  - timeout: 1
  - blocked: 0
  - blacklist: 0

结论：

- 唯一 timeout 没有明显 `/map` geometry danger flags。
- 也没有 Nav2 progress/recovery cluster。
- `/map` geometry alone 不足以解释 timeout。
- 下一步应该采 local costmap，而不是直接改 branch selection。

### Phase 18：external local costmap sampler/analyzer

目标：

- 增加 local costmap runtime diagnostics。
- 自动 join geometry + Nav2 + local cost data。
- 继续不改 branch selection。

成果：

- 新增：
  - `tools/summarize_goal_geometry_nav2.py`
  - `tools/record_local_costmap_diagnostics.py`
- 外部采样 `/local_costmap/costmap`。
- 发现 `/local_costmap/costmap_raw` 有多 ROS type，不能直接用简单 OccupancyGrid subscriber。
- Phase 18 smoke 最终：
  - `FAILED_EXHAUSTED`
  - timeout: 2
  - blocked: 1
  - blacklist: 1

关键限制：

- local costmap recorder 启动太晚，没有覆盖两个 timeout interval。

结论：

- Phase 18 作为工具/诊断增强通过。
- 作为导航改善失败。
- 下一步应该把 local cost sampling 集成进 `maze_explorer` goal event publication。

### Phase 19：integrated local cost diagnostics

目标：

- 修复 local cost coverage。
- 把 `/local_costmap/costmap` sampling 集成进 `maze_explorer`。
- 每个 `/maze/goal_events` dispatch/timeout payload 都携带 local-cost fields。

成果：

- `maze_explorer` 订阅 `/local_costmap/costmap`。
- goal event payload 新增：
  - dispatch sample age
  - dispatch target/robot in bounds
  - dispatch path sample count
  - dispatch coverage ratio
  - dispatch target/path local cost
  - timeout robot local cost max/mean
  - timeout obstacle cluster count
  - footprint corridor inflation squeezed
- 新增：
  - `tools/summarize_goal_event_local_costs.py`
- Phase 19 smoke：
  - `EXIT_REACHED`
  - `exit_distance_m: 0.5451`
  - `goal_count: 11`
  - `goal_success_count: 6`
  - `goal_failure_count: 4`
  - `timeout_cancel_count: 3`
  - `blocked_branch_count: 0`
  - `blacklisted_goal_count: 0`

local-cost 诊断：

- timeout dispatch coverage mean/min: `1.0 / 1.0`
- timeout sample age: 约 `0.045s`
- timeout target/robot in bounds: `3 / 3`
- 3 个 timeout 中：
  - 2 个有明显 timeout robot cost cluster / squeezed
  - 1 个没有
- 只有 seq 10 有明显 dispatch target/path high cost。
- 只有 1/3 timeout 是 `/map` narrow-passage。

结论：

- Phase 19 成功修复了 local-cost 采样覆盖。
- 但 timeout 信号仍 mixed。
- 不能从单次 run 推导 branch-selection 改动。

## 3. 当前能力状态

### 3.1 已完成能力

1. Workspace/build/test 基础

- ROS 2 Jazzy workspace 可 build。
- `tugbot_maze` / `tugbot_bringup` / `tugbot_navigation` targeted colcon build/test 通过。
- 当前 pytest：
  - `61 passed`
- colcon test：
  - `61 tests, 0 errors, 0 failures`

2. 仿真闭环

- Gazebo + SLAM + Nav2 + maze_explorer headless 可运行。
- 多次 smoke 可到达 `EXIT_REACHED`。
- recorder 和 JSONL analysis workflow 已稳定。

3. Explorer 状态机

- DFS/topology/backtracking 基础可用。
- stale/preempted result handling 已硬化。
- timeout cancel / terminal cancel 已分类。
- blocked/blacklist pollution 已被显著控制。

4. 诊断体系

- `/maze/explorer_state`
- `/maze/goal_events`
- Nav2/controller log correlation
- geometry summary
- local costmap embedded diagnostics
- per-goal local-cost outcome summary

5. 当前最重要的诊断结论

- timeout 不再是“黑盒”。
- Phase 19 已能看到每个 timeout 的：
  - `/map` geometry
  - local costmap dispatch/timeout cost
  - Nav2 progress/recovery/abort cluster
  - branch/blacklist causation

### 3.2 当前未完成/未解决问题

1. timeout 仍偏多

- Phase 19 虽然到出口，但 timeout count 是 3。
- 不能说导航策略已经优化完成。

2. local-cost 信号 mixed

- seq 2、seq 10 有明显 timeout robot cost cluster。
- seq 5 没有 local-cost cluster，但有 `/map` narrow-passage。
- 说明可能不止一种 timeout 类型。

3. branch selection 改动证据不足

- Phase 19 没有 blocked/blacklist。
- timeout 没有统一关联：
  - 不是每次都 narrow passage
  - 不是每次 dispatch target/path local cost 高
  - 不是每次 timeout robot cluster 高
- 所以现在改 branch selection 仍然偏早。

4. Nav2/controller 行为仍是主要相关信号

- Phase 19 中 3/3 timeout 都有 progress/recovery cluster。
- 但 Phase 16 已证明“简单放宽 progress/controller 参数”不是好方向。
- 需要更细分地分析：是局部 cost squeeze、heading/path alignment、controller oscillation，还是 goal placement。

## 4. 对最初目标的完成度评估

### 目标 1：搭建 ROS 2 + Gazebo Tugbot maze workspace

状态：完成。

证据：

- workspace build/test 通过。
- Gazebo/Nav2/SLAM/explorer headless smoke 可运行。

### 目标 2：机器人自主探索，不使用预计算路径

状态：完成基础版。

证据：

- runtime 使用 `/map`、TF、Nav2、local costmap。
- 迷宫图片/世界真值不作为 runtime path answer。
- DFS/topology exploration 已能多次达到出口。

### 目标 3：到达出口

状态：阶段性完成，但不稳定。

证据：

- Phase 12、17、19 都达到 `EXIT_REACHED`。
- 但 Phase 18 `FAILED_EXHAUSTED`。
- Phase 19 虽 `EXIT_REACHED`，但 timeout 增多。

### 目标 4：鲁棒、可诊断、可迭代

状态：当前最大的成果，基本完成。

证据：

- 已经从“看最终日志”进化到 per-goal structured diagnostics。
- 能比较 success vs timeout。
- 能区分 stale/preempted、timeout cancel、terminal cancel。
- 能关联 Nav2 logs、geometry、local costmap。

### 目标 5：优化导航策略

状态：尚未完成，且不应贸然改。

原因：

- timeout 根因仍 mixed。
- 单次 Phase 19 不足以证明 branch selection 应该怎么变。
- 已有原则要求：
  - 不凭单次日志猜参数
  - 不因 progress failure 变少就接受 profile
  - 不因 `/map` geometry 单一信号就改 branch selection

## 5. 接下来建议方向

我建议进入 Phase 20：重复采样与 timeout taxonomy，而不是直接改策略。

### Phase 20 目标

用 Phase 19 的 integrated diagnostics 连续跑 2~3 次相同配置 smoke，建立 timeout 分类和稳定性判断。

### 具体做法

1. 重复运行 Phase 19 smoke，保持参数不变

固定参数：

- baseline Nav2 profile
- 不启用 Phase 16 progress profile
- 不改 branch selection
- `max_goals:=12`
- `goal_timeout_sec:=35.0`
- `near_exit_goal_timeout_sec:=55.0`
- `branch_goal_step_m:=0.9`
- `clearance_radius_m:=0.34`
- local-cost embedded diagnostics 开启

输出：

- `phase20_run1_goal_events.jsonl`
- `phase20_run1_explorer_state.jsonl`
- `phase20_run1_nav2_analysis.json`
- `phase20_run1_geometry_summary.json`
- `phase20_run1_cost_summary.json`
- run2/run3 同理

2. 增加 multi-run aggregator

新增一个脚本：

`tools/compare_goal_diagnostics_runs.py`

输入多组：

- goal_events JSONL
- nav2 analysis JSON
- geometry summary
- local cost summary
- final explorer state

输出：

- 每 run final mode / timeout / blocked / blacklist
- timeout taxonomy：
  - Type A: high timeout robot local-cost cluster
  - Type B: high dispatch target/path local cost
  - Type C: `/map` narrow-passage / low corridor clearance
  - Type D: Nav2 progress/recovery cluster only
  - Type E: near-exit timeout
- repeated-signal table：
  - 哪些信号跨 run 稳定出现
  - 哪些只是偶发

3. 设定接受/拒绝标准

继续沿用验收：

必须：

- `EXIT_REACHED`
- blocked/blacklist 不增加
- timeout 不增加，最好下降
- 不凭单次 run 改参数

对 branch selection 的准入条件：

只有当多次 run 显示 timeout 反复满足以下组合，才考虑改 branch target：

- `/map` path corridor min clearance 低，或 `target_crosses_narrow_passage=true`
- dispatch target/path local cost 高
- timeout robot local-cost cluster 高
- caused blocked/blacklist 或反复 timeout 在同类区域
- success goals 不共享同样高风险特征

4. 如果 Phase 20 证明 local-cost cluster 稳定，再进入 Phase 21

Phase 21 可以是“低风险 branch target scoring，不改变状态机复杂度”。

可能方向：

- 不新增复杂状态机。
- 只在已有 candidate ranking 中加入诊断性 penalty。
- 例如：
  - dispatch target local cost 高：降低排序
  - path local cost max/mean 高：降低排序
  - `/map` narrow passage 且 local cost 高：降低排序
- 仍保留 fallback，不直接 blacklist。
- 验收必须比 Phase 19 更好：
  - `EXIT_REACHED`
  - timeout 下降
  - blocked/blacklist 不增加

5. 如果 Phase 20 证明 progress/recovery cluster 是主因，但 local-cost 不稳定

不要回到 Phase 16 的粗粒度 progress profile。

建议做更细的 Nav2/controller diagnostics：

- 记录 controller failure 前后的：
  - cmd_vel 是否接近 0
  - robot odom displacement
  - yaw error
  - path remaining length
  - costmap footprint collision check

目标是判断：

- 卡住不动？
- oscillation？
- heading alignment 问题？
- goal tolerance 太紧？
- local planner path 被 inflation 挤压？

这会比直接调 progress_checker 更稳。

6. 如果 Phase 20 证明 near-exit timeout 反复出现

考虑 Phase 21/22 中独立处理 near-exit goal：

- near-exit 不一定需要完整 branch goal。
- 可以切换到 “exit approach mode”：
  - 只在 exit radius 附近做保守目标
  - 避免从 near-exit 区域发远离出口的 branch
- 但这也需要多 run 支持，不能单次决定。

## 6. 一句话结论

Phase 1-19 已经把项目从“能不能跑 ROS/Gazebo Tugbot maze”推进到“能自主探索并多次到出口，且每个 timeout 都有 geometry/Nav2/local-cost 证据链”。当前不缺观察能力，缺的是多 run 证据来证明该改哪一个导航策略。下一步最稳的是 Phase 20 重复采样 + timeout taxonomy；只有跨 run 信号稳定后，再进入低风险 branch scoring 或 Nav2/controller targeted tuning。
