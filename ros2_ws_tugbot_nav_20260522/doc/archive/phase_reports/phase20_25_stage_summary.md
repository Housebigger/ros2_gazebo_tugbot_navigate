# Tugbot Maze Navigation Phase 20-25 阶段总结与 Phase 26 建议

生成时间：2026-05-25

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 0. 报告目的

本报告回顾 Phase 1-19 的阶段基线，并重点总结 Phase 20-25：

- Phase 20-24：从 multi-run timeout taxonomy 逐步收敛到 controller late-silent + local path/footprint cost failure-window 诊断。
- Phase 25：基于诊断证据做窄范围、可回退的 Nav2/MPPI 参数干预，并通过重复 smoke 和 acceptance comparator 判断是否可进入 candidate baseline。
- 最后给出 Phase 26 的建议方向。

本报告不把任何单次 run 的改善当作 baseline promotion 依据；判断仍遵循此前约定：timeout/subtype 改善、blocked/blacklist 不回归、EXIT_REACHED 或 exit behavior 不变差、并通过 repeat validation。

## 1. Phase 1-19 简要回顾：进入 Phase 20 前的状态

Phase 1-19 已完成从“可运行 ROS/Gazebo Tugbot maze workspace”到“可诊断自主探索系统”的基础建设。

关键能力：

1. Workspace / launch / mode separation

- ROS 2 Jazzy + Gazebo + Nav2 + slam_toolbox + ros_gz_bridge 工作空间可构建、测试、headless smoke。
- 已区分 SLAM mapping、SLAM+Nav2、autonomous exploration、static-map replay。
- 迷宫图片只作为 world generation truth 和入口/出口真值，不作为 runtime path answer。

2. DFS/topology exploration

- `maze_explorer` 已集成 DFS/topology/backtracking、junction/branch/dead-end 抽象。
- 通过 `/map`、TF、Nav2 action、local costmap 运行时反馈自主探索。
- Phase 12、17、19 等 run 已能达到 `EXIT_REACHED`。

3. Action lifecycle hardening

- Phase 7-9 定位并修复 stale/preempted Nav2 result 污染。
- Phase 10-11 增加 terminal cancel / timeout cancel lifecycle。
- stale/preempted、timeout cancel、terminal cancel 已分离计数，不再简单污染 blocked/blacklist。

4. Per-goal diagnostics

- Phase 14 增加 structured `/maze/goal_events`。
- Phase 15 关联 Nav2/controller logs。
- Phase 17 增加 `/map` geometry diagnostics。
- Phase 18-19 将 `/local_costmap/costmap` sampling 集成进 goal events，解决 local cost coverage 问题。

Phase 19 代表性状态：

```text
final_mode: EXIT_REACHED
exit_distance_m: 0.5451
goal_count: 11
goal_success_count: 6
timeout_cancel_count: 3
blocked_branch_count: 0
blacklisted_goal_count: 0
```

进入 Phase 20 前的核心问题：

```text
timeout 仍偏多，且信号 mixed。
不能仅凭单次 run 或 `/map` geometry/local-cost 单一信号改 branch selection。
Phase 16 已证明粗粒度 progress/controller tolerance 放宽会让 timeout/blacklist 变差。
下一步需要 multi-run taxonomy 和更细 controller/local-cost diagnostics。
```

## 2. Phase 20-25 总体主线

Phase 20-25 的主线可以概括为：

```text
Phase 20: 多 run taxonomy，确认 timeout 不是单一稳定空间/branch 问题。
Phase 21: 增加 odom/cmd_vel controller dynamics，发现 timeout 不是全程卡死，而是 late stall。
Phase 22: 增加 last-window taxonomy，确认所有重复 timeout 都是 late_controller_silent。
Phase 23: 增加 failure-window runtime diagnostics，区分 footprint/path blocked 与 side/timing/unclear subtype。
Phase 24: 增加 subtype classifier、post-recovery alignment、runtime snapshot recorder/enrichment，确认 path 更新仍在、robot-to-path distance 小，干预应聚焦 local path/footprint cost relief。
Phase 25: 做窄范围 reversible intervention。local inflation relief 被拒；MPPI CostCritic cost_weight 有收益但存在 exit-distance regression 和 run/profile variance，最终不 promote canonical baseline。
```

## 3. Phase 20：多 run timeout taxonomy

目标：

- 用 Phase 19 integrated diagnostics 连续跑 baseline repeat smoke。
- 观察 timeout 是否稳定集中在某类 local-cost / geometry / Nav2 signal。
- 决定是否可以进入 branch scoring 或 controller diagnostics。

Phase 20 repeat smoke 结果：

| run | final_mode | timeout_count | success_count | goal_count | blocked | blacklist | exit_distance_m |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| phase20_run1 | FAILED_EXHAUSTED | 5 | 7 | 12 | 0 | 0 | 1.0269 |
| phase20_run2 | FAILED_EXHAUSTED | 3 | 9 | 12 | 0 | 0 | 1.1974 |
| phase20_run3 | FAILED_EXHAUSTED | 0 | 6 | 12 | 1 | 1 | 2.0152 |

Aggregator 结论：

```text
exit_reached_count: 0
timeout_count_values: [5, 3, 0]
timeout_count_stable: false
all_blocked_blacklist_zero: false
blocked_or_blacklist_run_count: 1
recommended_next_phase: controller_diagnostics_candidate
```

Timeout taxonomy totals：

```text
timeout_count_total: 8
map_narrow_passage: 6 / 8
high_timeout_robot_cost: 6 / 8
timeout_squeezed: 6 / 8
```

Phase 20 决策：

- 不进入 branch scoring。
- 因为 0/3 repeat runs reached exit，并且 failure mode 在 timeout 与 blocked/blacklist 之间漂移。
- success goals 也可能共享部分 high-cost/narrow-passage 特征，说明 dispatch-time feature 不能直接作为 branch rejection 依据。
- 下一步应增加 controller/odom/cmd_vel diagnostics。

## 4. Phase 21：Controller dynamics diagnostics

目标：

- 判断 timeout / blocked_nav2 是否是：
  - stuck-with-cmd
  - oscillation
  - controller-silent
  - slow-progress
  - 还是 full-interval healthy motion 后 timeout

新增能力：

- `tools/analyze_controller_dynamics.py`
- runtime odom/cmd_vel recorder
- Phase 21 smoke wrapper 和 post-run analysis chain
- 更强的 orphan ROS/Gazebo/Nav2 cleanup 检查

Phase 21C representative smoke：

```text
final_mode: EXIT_REACHED
exit_distance_m: 0.5913
goal_success_count: 4
timeout_cancel_count: 3
blocked_branch_count: 0
blacklisted_goal_count: 0
```

Nav2 correlation：

```text
timeout_count: 3
timeout_with_progress_failure_count: 3
timeout_with_recovery_count: 3
```

Controller dynamics 初步发现：

- full goal interval 看起来是 `healthy_motion`。
- 但 timeout 前 last 10s 出现明显 near-zero cmd_vel + near-zero odom。
- 同时 local costmap 显示 robot 周围 high cost / squeezed。

Phase 21 结论：

```text
timeout 不是简单“机器人从头到尾没动”。
更像是 goal 前中段有运动，但末段进入 controller-near-silent late stall。
需要 windowed taxonomy，不能只看 full-interval 分类。
```

## 5. Phase 22：Windowed controller taxonomy 与 repeat validation

Phase 22A 目标：

- 将 last 10s window 纳入 controller dynamics classification。
- 把 Phase 21C 中 full-interval `healthy_motion` 但末段 near-zero 的 timeout 重新标为：

```text
healthy_motion_but_late_stall
late_controller_silent
```

Phase 22A re-analysis：

```text
seq 2/4/7 timeout => healthy_motion_but_late_stall + late_controller_silent
success goals 不被误判为 late stall
terminal_cancel_after_exit 单独标记，不参与 timeout/failure late stall
```

Phase 22B repeat smokes：

| run | final_mode | goals | success | timeout/failure | timeout_count | blocked/blacklist | exit_distance_m |
| --- | --- | ---: | ---: | ---: | ---: | --- | ---: |
| phase21_run1 | EXIT_REACHED | 8 | 4 | 3 | 3 | 0 / 0 | 0.5913 |
| phase22_run1 | EXIT_REACHED | 9 | 4 | 4 | 4 | 0 / 0 | 0.5993 |
| phase22_run2 | FAILED_EXHAUSTED | 12 | 10 | 2 | 2 | 0 / 0 | 1.7235 |

Windowed taxonomy aggregate：

```text
timeout total: 9
late_controller_silent timeout: 9
healthy_motion_but_late_stall timeout: 9
late_stuck_with_cmd timeout: 0
late_oscillation timeout: 0
```

Local-cost correlation：

```text
squeezed timeout: 9 / 9
timeout robot cost max mean: about 99
```

Phase 22 决策：

- Timeout stable signature = full-interval motion + last-window controller silence + squeezed/high local cost。
- 下一步不是 branch scoring，而是 failure-window introspection：看 footprint、front wedge、side clearance、path-ahead cost、Nav2 recovery/abort timing。

## 6. Phase 23：Failure-window runtime diagnostics

Phase 23A：先从已有 artifacts 做 introspection。

发现：

```text
existing artifacts lack footprint/front/path-ahead costs and Nav2 event timestamps.
```

所以 Phase 23B 增加 runtime diagnostics：

- `/maze/goal_events` timeout local-cost fields：
  - footprint cost max/mean/p95
  - footprint inflated/lethal cell count
  - front wedge cost / clearance
  - left/right side cost / clearance
  - path_ahead_0_5m / path_ahead_1_0m cost
- Nav2 analyzer 增加 progress/recovery/controller abort timestamps。
- failure-window analyzer 关联 near-zero timing、cost fields、Nav2 event timing。

Phase 23B representative run：

```text
final_mode: SETTLING
exit_distance_m: 1.5950
goal_success_count: 7
timeout_cancel_count: 5
blocked_branch_count: 0
blacklisted_goal_count: 0
Nav2 timeout_count: 4
```

Timeout subtype split：

```text
seq 2: side_cost_or_timing_late_silent + cmd_silent_after_recovery_abort
seq 4: side_cost_or_timing_late_silent + cmd_silent_after_recovery_abort
seq 8: footprint_path_blocked_late_silent + cmd_silent_before_progress_failure
seq 11: footprint_path_blocked_late_silent + cmd_silent_after_recovery_abort
```

Phase 23 结论：

- Timeout late-silent 不是单一类别。
- 至少存在两类：
  1. severe footprint/path obstruction
  2. side-cost / timing / unclear local obstruction
- 需要把 subtype taxonomy 标准化，并继续补齐 post-recovery alignment / path-refresh 证据。

## 7. Phase 24：Subtype classifier、post-recovery alignment 与 snapshot enrichment

### 7.1 Phase 24A：Timeout subtype classifier

目标：

- 把 Phase 23B runtime diagnostics 标准化为 timeout subtype taxonomy。

核心 subtype：

```text
footprint_path_blocked_late_silent
side_cost_or_timing_late_silent
unclassified_late_silent
```

Phase 23B + Phase 24A aggregate：

```text
late-silent timeouts: 7
footprint_path_blocked_late_silent: 4 / 7 = 57.1%
side/timing: 2 / 7
unclassified: 1 / 7
```

Timing observation：

```text
大多数 near-zero starts after progress/recovery/abort cycles。
```

Phase 24A 决策：

- Severe footprint/path obstruction 是 plurality，但不是全部。
- 还不能直接调参；应先确认 post-recovery 后 path 是否刷新、robot 是否偏离 path、cost snapshots 是否支持 local path/footprint intervention。

### 7.2 Phase 24B：Post-recovery path/local-cost alignment

目标：

- 检查 recovery 后 controller 是否继续收到 `Passing new path to controller`。
- 检查 timeout 发生时 robot-to-path alignment 是否明显偏离。

结论：

- 现有日志显示多数 timeout recovery 后仍有 path updates。
- 现有 artifacts 不足以回答 pre/post recovery cost snapshots 和 robot-to-path distance。
- 至少有一个 late-silent timeout 的 timeout-side local-cost snapshot 为 0/0，说明 coarse timeout snapshot 不能解释全部问题。

Phase 24B 决策：

```text
增加 runtime recorder，记录 /local_costmap/costmap、/plan、/odom、cmd_vel_nav、/maze/goal_events。
```

### 7.3 Phase 24C：Runtime post-recovery recorder

Phase24C run1：

```text
final_mode: EXIT_REACHED
exit_distance_m: 0.5773
goal_success_count: 5
timeout_cancel_count: 2
blocked_branch_count: 0
blacklisted_goal_count: 0
```

贡献：

- Recorder foundation 被验证。
- 能记录 path updates、near-zero snapshots、path-ahead cost、robot-to-path distance。

限制：

```text
pre/post recovery snapshots 仍缺失，因为 recovery events 只在 launch logs 中。
```

### 7.4 Phase 24D：Post-run enrichment + periodic snapshots

新增：

- `tools/enrich_post_recovery_snapshots.py`
- periodic active-goal snapshots
- phase24c_run2 enrichment coverage

Phase24C run2 作为后续 Phase25 baseline：

```text
final_mode: FAILED_EXHAUSTED
exit_distance_m: 0.7568
goal_success_count: 8
timeout_cancel_count: 4
blocked_branch_count: 0
blacklisted_goal_count: 0
Nav2 timeout_count: 4
footprint_path_blocked_late_silent: 2
```

Enriched observations：

1. Periodic snapshots solved coverage limitation。
2. Robot-to-path distance remains small。
3. Controller keeps receiving path updates after recovery。
4. 对 footprint/path blocked subtype，pre/post/near-zero path_1m cost consistently high。

Phase 24D 决策：

```text
最强干预信号不是 path alignment，而是 local path-ahead / footprint cost。
Phase 25 应做窄范围、可回退 local path-cost / footprint-cost relief experiment。
仍不建议 branch scoring 或粗粒度 progress checker tuning。
```

## 8. Phase 25：窄范围 intervention 与 candidate-baseline 验证

Phase 25 的所有实验都以 `phase24c_run2` 作为主要 baseline：

```text
baseline: phase24c_run2
final_mode: FAILED_EXHAUSTED
goal_success_count: 8
timeout_cancel_count: 4
blocked_branch_count: 0
blacklisted_goal_count: 0
footprint_path_blocked_late_silent: 2
exit_distance_m: 0.7568
canonical CostCritic cost_weight: 3.81
```

### 8.1 Phase 25A：Local inflation relief，被拒

实验：

```text
local inflation_radius: 0.70 -> 0.55
local cost_scaling_factor: 3.0 -> 4.5
```

phase25a_run1：

```text
final_mode: EXIT_REACHED
exit_distance_m: 0.5638
goal_success_count: 6
timeout_cancel_count: 5
blocked_branch_count: 0
blacklisted_goal_count: 0
footprint_path_blocked_late_silent: 3
```

判定：拒绝。

原因：

```text
footprint_path_blocked_late_silent: 2 -> 3 变差
timeout_cancel_count: 4 -> 5 变差
goal_success_count: 8 -> 6 下降
blocked/blacklist 虽保持 0，但 primary target 失败
```

### 8.2 Phase 25B/C：MPPI CostCritic cost_weight 2.5，有 target 收益但 exit-distance regression

Phase 25B 实验：

```text
FollowPath.CostCritic.cost_weight: 3.81 -> 2.5
```

phase25b_run1：

```text
final_mode: FAILED_EXHAUSTED
goal_success_count: 9
timeout_cancel_count: 3
blocked_branch_count: 0
blacklisted_goal_count: 0
footprint_path_blocked_late_silent: 0
exit_distance_m: 1.7342
```

初次 hard gates：accepted。

但 caveat：

```text
exit_distance_m: 0.7568 -> 1.7342，明显变差。
```

Phase 25C repeat validation phase25b_run2：

```text
final_mode: FAILED_EXHAUSTED
goal_success_count: 9
timeout_cancel_count: 3
blocked_branch_count: 0
blacklisted_goal_count: 0
footprint_path_blocked_late_silent: 0
exit_distance_m: 1.2926
```

Phase 25C 判定：

- `cost_weight=2.5` 重复消除了 footprint/path blocked subtype，并降低 timeout、提高 success。
- 但 exit_distance 仍 materially worse than baseline。
- 不直接 promote 2.5；下一步测试更小 delta。

### 8.3 Phase 25D：CostCritic cost_weight 3.0，被拒

实验：

```text
FollowPath.CostCritic.cost_weight: 3.81 -> 3.0
```

phase25d_run1：

```text
final_mode: FAILED_EXHAUSTED
goal_success_count: 8
timeout_cancel_count: 4
blocked_branch_count: 0
blacklisted_goal_count: 0
footprint_path_blocked_late_silent: 2
exit_distance_m: 0.8055
```

判定：拒绝。

原因：

- exit_distance 接近 baseline，但 target metrics 回到 baseline。
- footprint/path blocked 没有下降，timeout 没有下降。

### 8.4 Phase 25E/F：CostCritic cost_weight 2.75，成为 candidate，但不直接 promote

实验：

```text
FollowPath.CostCritic.cost_weight: 3.81 -> 2.75
```

phase25e_run1：

```text
final_mode: EXIT_REACHED
goal_success_count: 6
timeout_cancel_count: 2
blocked_branch_count: 0
blacklisted_goal_count: 0
footprint_path_blocked_late_silent: 0
exit_distance_m: 0.5945
```

旧 strict comparator 因 raw success count 低于 baseline 而 reject；但该 run 已提前 `EXIT_REACHED`，goal_count 更短，raw success count 不可直接和 exhausted 12-goal baseline 比。

Phase 25F 改进：terminal-success-aware comparator。

规则：

```text
if experiment final_mode == EXIT_REACHED:
  bypass raw goal_success_count >= baseline goal_success_count
  retain safety gates: blocked/blacklist no regression
  retain subtype/timeouts/exit behavior gates
  record success ratio / efficiency as advisory
```

phase25e_run2：

```text
final_mode: EXIT_REACHED
goal_success_count: 7
timeout_cancel_count: 3
blocked_branch_count: 0
blacklisted_goal_count: 0
footprint_path_blocked_late_silent: 1
exit_distance_m: 0.5686
```

Terminal-aware comparison vs baseline：accepted。

Passed gates：

```text
footprint_path_blocked_reduced: 2 -> 1
timeouts_reduced: 4 -> 3
blocked_branch_no_regression: 0 -> 0
blacklisted_goal_no_regression: 0 -> 0
exit_behavior_preserved_or_improved: FAILED_EXHAUSTED -> EXIT_REACHED
success_gate: EXIT_REACHED bypasses raw success count gate
```

Phase 25F 结论：

```text
cost_weight=2.75 是 candidate baseline，不只是单次改善。
但仍不能静默覆盖 canonical baseline。
需要 candidate-baseline validation。
```

### 8.5 Phase 25G/H：Candidate-baseline validation，最终不 promote

Phase 25G 创建明确 candidate params：

```text
nav2_slam_candidate_costcritic_275_params.yaml
FollowPath.CostCritic.cost_weight: 2.75
canonical nav2_slam_params.yaml 保持 cost_weight: 3.81
```

candidate_baseline_run1：

```text
final_mode: FAILED_EXHAUSTED
goal_success_count: 8
timeout_cancel_count: 4
blocked_branch_count: 0
blacklisted_goal_count: 0
footprint_path_blocked_late_silent: 1
exit_distance_m: 1.3373
accepted: false
```

Phase 25H repeat candidate_baseline_run2：

```text
final_mode: FAILED_EXHAUSTED
goal_success_count: 8
timeout_cancel_count: 4
blocked_branch_count: 0
blacklisted_goal_count: 0
footprint_path_blocked_late_silent: 1
exit_distance_m: 1.0835
accepted: false
```

Phase 25H 最终判定：

```text
Do not promote cost_weight: 2.75 to canonical baseline.
Preserve 2.75 as experimental/candidate artifact only.
Keep nav2_slam_params.yaml unchanged.
```

原因：

```text
candidate_baseline_run1 failed promotion.
candidate_baseline_run2 also failed promotion.
Neither candidate run reached EXIT_REACHED.
Neither candidate run reduced run-level timeout_cancel_count vs baseline.
```

重要异常：

```text
Phase25E profile runs reached EXIT_REACHED twice.
Candidate baseline runs did not reach EXIT_REACHED twice.
Both profiles should be behaviorally equivalent because both use cost_weight 2.75.
```

这说明 Phase 26 的首要问题不是继续盲调 2.65 / 2.6，而是确认 profile path / launch selection / params equivalence / run-to-run variance。

## 9. Phase 20-25 的总体结论

### 9.1 诊断体系已经成熟到可支持 targeted intervention

Phase 20-25 新增或稳定使用了：

- multi-run timeout taxonomy
- odom/cmd_vel controller dynamics recorder/analyzer
- last-window late-stall classification
- failure-window analyzer
- footprint/front/side/path-ahead local-cost diagnostics
- Nav2 progress/recovery/controller-abort timestamps
- post-recovery path/local-cost alignment analyzer
- runtime post-recovery recorder
- post-run recovery snapshot enrichment
- terminal-success-aware comparator
- candidate-baseline validation gate

这让项目从“看到 timeout”推进到“知道 timeout 多数是什么类型、发生在 recovery/abort 后还是之前、local path/footprint cost 是否参与、path 是否还在更新、robot 是否仍贴近 path”。

### 9.2 当前主根因方向不是 DFS branch state machine

目前证据更支持：

```text
Nav2/MPPI controller 在 narrow/local-cost pressure 下，goal 前中段可运动，但后段进入 late_controller_silent。
其中一部分由 footprint/path-ahead high cost 强烈解释，另一部分属于 side/timing/unclassified late-silent。
```

不支持的方向：

- 直接增加复杂 branch scoring。
- 仅凭 `/map` geometry 修改 branch target。
- 粗粒度放宽 progress checker/controller tolerance。
- 用 broad local inflation relief 作为默认修复。

### 9.3 Phase 25 intervention 的经验

1. Local inflation relief 不可靠

- 虽然 phase25a_run1 reached exit，但 target subtype 和 timeout 都变差。
- 不能作为默认。

2. CostCritic cost_weight 确实影响 failure subtype

- 2.5 重复消除了 footprint/path blocked subtype，但 exit_distance 明显变差。
- 3.0 保留 exit distance，但 target metrics 回到 baseline。
- 2.75 在 phase25e_run1/2 表现最好：两次 EXIT_REACHED，timeout/subtype/exit_distance 均优于 baseline。

3. Candidate-baseline validation 暴露新问题

- 同样 nominally `cost_weight=2.75` 的 candidate profile 两次不达出口。
- 因此不能 promote。
- 当前 blocker 是 profile-path equivalence / launch selection / run variance，而不是再盲目试新数值。

## 10. 当前推荐 baseline 状态

Canonical baseline：保持不变。

```text
config: nav2_slam_params.yaml
FollowPath.CostCritic.cost_weight: 3.81
```

Experimental profiles：保留但不 promote。

```text
nav2_slam_phase25a_local_cost_relief_params.yaml       rejected
nav2_slam_phase25b_costcritic_relief_params.yaml       useful signal, not baseline
nav2_slam_phase25d_costcritic_mid_params.yaml          rejected
nav2_slam_phase25e_costcritic_compromise_params.yaml   strong candidate signal
nav2_slam_candidate_costcritic_275_params.yaml         failed candidate-baseline validation
```

Accepted knowledge：

```text
cost_weight=2.75 remains interesting, but promotion is blocked by candidate run failure / possible variance.
```

## 11. Phase 26 建议

### Phase 26A：Profile-path / params equivalence audit（优先级最高）

目标：

确认 Phase25E profile 与 candidate baseline profile 是否真的行为等价，排除 launch wrapper / params file path / install overlay / stale install / wrong config selection 问题。

建议任务：

1. Compare params files semantically

```text
diff nav2_slam_phase25e_costcritic_compromise_params.yaml \
     nav2_slam_candidate_costcritic_275_params.yaml
```

但不要只看 textual diff；应写一个 YAML semantic diff 工具或测试，忽略 comments/order，重点检查：

```text
controller_server.ros__parameters.FollowPath.CostCritic.cost_weight
controller_server.ros__parameters.FollowPath.CostCritic.*
local_costmap.*
global_costmap.*
progress_checker / goal_checker
```

2. Add params fingerprint to every smoke artifact

每次 wrapper 启动时记录：

```text
params_file_absolute_path
sha256(params_file)
selected profile name
launch arguments
installed share path used by launch
source workspace overlay path
```

建议输出：

```text
log/<run_id>_params_fingerprint.json
```

3. Runtime param dump verification

在 Nav2 controller server active 后 dump runtime params：

```text
ros2 param dump /controller_server --output-dir log/<run_id>_runtime_params
```

并从 dump 中提取：

```text
FollowPath.CostCritic.cost_weight
FollowPath.CostCritic.critical_cost
FollowPath.CostCritic.consider_footprint
local_costmap inflation_layer inflation_radius/cost_scaling_factor
```

4. Acceptance criteria for Phase 26A

```text
phase25e profile and candidate profile are proven equivalent or exact differences are identified.
Each future run has params fingerprint + runtime param dump.
No new tuning until this is resolved.
```

### Phase 26B：Run-to-run variance characterization

只有 Phase 26A 确认 config/profile path 没有问题后，再做 variance characterization。

建议 repeat matrix：

```text
canonical baseline cost_weight 3.81: 3 runs
phase25e/candidate cost_weight 2.75: 3 runs
optional cost_weight 2.5: 2 runs only if needed for contrast
```

每个 run 必须生成：

```text
*_explorer_state.jsonl
*_goal_events.jsonl
*_controller_dynamics.jsonl
*_goal_nav2_analysis.json
*_geometry_nav2_summary.json
*_goal_event_cost_summary.json
*_goal_controller_dynamics.json
*_failure_windows.json
*_timeout_subtypes.json
*_post_recovery_snapshots.jsonl
*_post_recovery_enriched.json
*_params_fingerprint.json
*_runtime_params dump
```

Aggregate 指标：

```text
EXIT_REACHED count / N
median exit_distance_m
P75 exit_distance_m
timeout_cancel_count median/P75
footprint_path_blocked_late_silent median/P75
side_cost_or_timing_late_silent median/P75
unclassified_late_silent count
blocked_branch_count total
blacklisted_goal_count total
```

建议 promotion gate：

```text
candidate must satisfy across repeats:
- EXIT_REACHED count >= baseline, preferably >= 2/3
- median timeout_cancel_count < baseline median
- P75 timeout_cancel_count <= baseline P75
- footprint_path_blocked_late_silent median < baseline median
- blocked_branch_count total == 0 or no worse than baseline
- blacklisted_goal_count total == 0 or no worse than baseline
- P75 exit_distance_m not materially worse than baseline, unless EXIT_REACHED dominates
```

### Phase 26C：If equivalence bug is found, fix and replay candidate-baseline validation

如果 Phase 26A 找到 real config/launch mismatch：

1. 先写 contract test，锁定正确 params file selection。
2. 修复 launch/wrapper/install selection。
3. 重新运行 candidate_baseline_run1/run2 equivalent validation。
4. 只有 repeat validation 重新通过，才考虑 baseline promotion。

不要在修复 profile selection 的同时改 cost_weight，否则无法归因。

### Phase 26D：If variance is real, change acceptance model before further tuning

如果 Phase 26A 证明 profile 等价，而 Phase 26B 显示 run-to-run variance 很大：

- 不再用 single-run accepted/rejected 作为 promotion 依据。
- 改为 repeat-run acceptance。
- 每个 candidate 至少 3 次 smoke，使用 median/P75 gate。
- 明确区分：
  - diagnostic signal useful
  - one-run improvement
  - repeat-validated candidate
  - canonical baseline

### Phase 26E：Side/timing 与 unclassified late-silent 的下一层诊断

在 footprint/path blocked 已被 CostCritic 部分缓解后，剩余问题更多会转向：

```text
side_cost_or_timing_late_silent
unclassified_late_silent
```

建议新增诊断字段：

- near-zero onset 前后的 robot yaw error / angular velocity。
- path curvature / local plan heading discontinuity。
- goal heading vs corridor heading alignment。
- MPPI selected trajectory cost proxy（如果无法直接读 critic score，可记录 plan/path local curvature + side clearance + cmd angular trend）。
- recovery 后 first valid cmd delay。
- recovery 后 path update 到 near-zero onset 的时间差。

这些诊断应作为 Phase 27 的依据；Phase 26 优先解决 config equivalence / variance。

## 12. Phase 26 当前不建议做的事

```text
1. 不要 promote cost_weight=2.75 到 canonical nav2_slam_params.yaml。
2. 不要继续盲测 2.65 / 2.6，除非 Phase26A/B 已完成。
3. 不要启用 Phase25A local inflation relief profile。
4. 不要回到 Phase16 式 broad progress_checker/controller tolerance 放宽。
5. 不要引入复杂 branch scoring/state-machine changes。
6. 不要只凭单次 EXIT_REACHED 或单次 FAILED_EXHAUSTED 决定参数命运。
```

## 13. 建议的 Phase 26A 最小实施清单

为了保持 phase-gated 和 TDD 风格，建议 Phase 26A 只做下面这些最小闭环：

1. 新增 semantic params diff test

```text
src/tugbot_maze/test/test_phase26a_params_equivalence.py
```

测试内容：

```text
- phase25e profile and candidate 275 profile have same CostCritic cost_weight
- canonical baseline remains 3.81
- candidate profile remains 2.75
- no unexpected local_costmap / global_costmap / progress_checker deltas between phase25e and candidate profile
```

2. 新增 params fingerprint helper

```text
tools/fingerprint_nav2_params.py
```

输出：

```json
{
  "run_id": "...",
  "params_file": "absolute/path.yaml",
  "sha256": "...",
  "selected_profile": "...",
  "costcritic_cost_weight": 2.75,
  "canonical_baseline_cost_weight": 3.81
}
```

3. 修改 smoke wrapper

```text
tools/run_phase21_controller_diagnostics_smoke.sh
```

在每次 run 开始前写：

```text
log/<run_id>_params_fingerprint.json
```

4. 增加 runtime dump step

如果 Nav2 已 active，保存：

```text
log/<run_id>_runtime_params/controller_server.yaml
```

5. 只跑一个短 smoke 验证 artifact chain

建议 run id：

```text
phase26a_fingerprint_smoke
```

验收：

```text
pytest targeted tests pass
full pytest pass if cost acceptable
wrapper produces params_fingerprint.json
runtime dump contains expected cost_weight
cleanup confirms no orphan ROS/Gazebo/Nav2 processes
```

## 14. 一句话结论

Phase 1-19 已经完成自主探索与诊断基础；Phase 20-24 将 timeout 根因从“混合黑盒失败”收敛为“controller late-silent + local path/footprint cost pressure，并伴随少量 side/timing/unclassified subtype”；Phase 25 证明 CostCritic cost_weight 是有效但敏感的干预方向，其中 2.75 有强 candidate 信号但 candidate-baseline repeat 失败。因此下一阶段不应继续盲目调参或 promote baseline，而应先做 Phase 26A profile-path/params equivalence audit，再用 repeat-run variance gate 决定是否重试 2.75、调整数值，或转向更细的 side/timing late-silent 诊断。
