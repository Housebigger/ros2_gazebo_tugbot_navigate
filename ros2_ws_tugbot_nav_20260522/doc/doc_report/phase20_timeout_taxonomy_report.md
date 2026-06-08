# Phase 20 多 run timeout taxonomy 报告

生成时间：2026-05-24

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. Phase 20 目标

Phase 20 按以下拆分执行：

- Phase 20A：新增 multi-run aggregator 的 TDD contract。
- Phase 20B：保持 Phase 19 参数不变，跑 3 次 repeat smoke。
- Phase 20C：跑 aggregator，形成 timeout taxonomy 结论。

本阶段只做分析，不把 taxonomy 阈值用于控制，不改 branch selection，不启用 Phase 16 Nav2 progress profile。

固定 smoke 参数：

- baseline Nav2 profile
- `max_goals:=12`
- `goal_timeout_sec:=35.0`
- `near_exit_goal_timeout_sec:=55.0`
- `near_exit_timeout_extension_radius_m:=1.0`
- `goal_settle_sec:=1.5`
- `branch_goal_step_m:=0.9`
- `open_direction_lookahead_m:=1.8`
- `lateral_centering_search_m:=1.0`
- `clearance_radius_m:=0.34`
- `min_open_distance_m:=0.5`

## 2. Phase 20A：Aggregator 工具

新增：

- `tools/compare_goal_diagnostics_runs.py`
- `src/tugbot_maze/test/test_phase20_compare_goal_diagnostics_runs.py`

TDD RED：

- contract test 先使用 Phase 17/18/19 existing artifacts 调用 aggregator。
- 初始失败，说明接口/输出结构不满足 contract。

GREEN 后能力：

- 支持多 run 输入。
- 输出 run-level summary。
- 输出 timeout-level rows。
- 输出 success-level rows。
- 输出 taxonomy by outcome。
- 输出 `recommended_next_phase`，但只作为分析标签，不自动改控制策略。

当前 taxonomy 阈值：

- `HIGH_COST_THRESHOLD = 70`
- `HIGH_TIMEOUT_ROBOT_COST_THRESHOLD = 90`
- `LOW_CORRIDOR_CLEARANCE_M = 0.55`
- `NEAR_EXIT_DISTANCE_M = 1.0`

标签包括：

- `map_narrow_passage`
- `low_path_corridor_clearance`
- `high_dispatch_target_cost`
- `high_dispatch_path_cost`
- `high_timeout_robot_cost`
- `timeout_squeezed`
- `nav2_progress_cluster`
- `near_exit`
- `caused_blocked_or_blacklist`

验证：

- `python3 -m py_compile tools/compare_goal_diagnostics_runs.py` 通过。
- focused pytest 通过。
- `python3 -m pytest -q src/tugbot_maze/test`：`62 passed`。
- `colcon build --packages-select tugbot_maze --symlink-install` 通过。

## 3. Phase 20B：3 次 repeat smoke 结果

Artifacts：

Run 1：

- `log/phase20_run1_launch.log`
- `log/phase20_run1_goal_events.jsonl`
- `log/phase20_run1_explorer_state.jsonl`
- `log/phase20_run1_goal_nav2_analysis.json`
- `log/phase20_run1_geometry_nav2_summary.json`
- `log/phase20_run1_goal_event_cost_summary.json`

Run 2：

- `log/phase20_run2_launch.log`
- `log/phase20_run2_goal_events.jsonl`
- `log/phase20_run2_explorer_state.jsonl`
- `log/phase20_run2_goal_nav2_analysis.json`
- `log/phase20_run2_geometry_nav2_summary.json`
- `log/phase20_run2_goal_event_cost_summary.json`

Run 3：

- `log/phase20_run3_launch.log`
- `log/phase20_run3_goal_events.jsonl`
- `log/phase20_run3_explorer_state.jsonl`
- `log/phase20_run3_goal_nav2_analysis.json`
- `log/phase20_run3_geometry_nav2_summary.json`
- `log/phase20_run3_goal_event_cost_summary.json`

Aggregator output：

- `log/phase20_runs_aggregate.json`

Run-level summary：

| run | final_mode | timeout_count | success_count | goal_count | blocked | blacklist | exit_distance_m |
|---|---|---:|---:|---:|---:|---:|---:|
| phase20_run1 | FAILED_EXHAUSTED | 5 | 7 | 12 | 0 | 0 | 1.0269 |
| phase20_run2 | FAILED_EXHAUSTED | 3 | 9 | 12 | 0 | 0 | 1.1974 |
| phase20_run3 | FAILED_EXHAUSTED | 0 | 6 | 12 | 1 | 1 | 2.0152 |

Important note：

- run1 是用户消息到达时已经启动的 run。它已产生完整 run1 artifacts，最终状态为 `FAILED_EXHAUSTED`，后续已清理残留 launch。
- run2 的 primary state recorder 达到 `max-samples=340` 时还未 terminal；后续追加了 tail recorder 到同一个 state JSONL，最终状态为 `FAILED_EXHAUSTED`。
- run3 无 timeout，但出现 blocked/blacklist，并最终 `FAILED_EXHAUSTED`。

## 4. Phase 20C：Aggregator 结果

`log/phase20_runs_aggregate.json` summary：

- run_count: `3`
- exit_reached_count: `0`
- timeout_count_values: `[5, 3, 0]`
- timeout_count_total: `8`
- timeout_count_min: `0`
- timeout_count_max: `5`
- timeout_count_stable: `false`
- all_blocked_blacklist_zero: `false`
- blocked_or_blacklist_run_count: `1`
- branch_selection_ready: `false`
- recommended_next_phase: `controller_diagnostics_candidate`

Timeout taxonomy totals across run1-3：

- nav2_progress_cluster: `8 / 8`
- map_narrow_passage: `6 / 8`
- low_path_corridor_clearance: `6 / 8`
- high_timeout_robot_cost: `6 / 8`
- timeout_squeezed: `6 / 8`
- high_dispatch_path_cost: `4 / 8`
- high_dispatch_target_cost: `3 / 8`
- near_exit: `1 / 8`
- caused_blocked_or_blacklist: `0 / 8`

Success taxonomy totals across run1-3：

- high_dispatch_path_cost: `6`
- high_dispatch_target_cost: `6`
- low_path_corridor_clearance: `11`
- map_narrow_passage: `10`
- nav2_progress_cluster: `6`
- near_exit: `4`
- high_timeout_robot_cost: `0`
- timeout_squeezed: `0`
- caused_blocked_or_blacklist: `0`

## 5. Answering the Phase 20C questions

### 5.1 timeout 是否稳定集中在同类 local-cost cluster？

部分集中，但不完全稳定。

证据：

- 6/8 timeout 有 `high_timeout_robot_cost`。
- 6/8 timeout 有 `timeout_squeezed`。
- 但 2/8 timeout 没有 high robot cost / squeezed，只剩 Nav2 progress cluster。
- run3 完全没有 timeout，但有 blocked/blacklist。

结论：

- timeout 与 local-cost cluster 有强相关，但不是唯一类型。
- 不能只靠 local-cost cluster 直接设计 branch penalty。

### 5.2 success 是否也共享 high-cost 特征？

共享部分 dispatch-side high-cost / geometry 特征，但不共享 timeout-side robot cluster 特征。

证据：

- success 中也有：
  - high_dispatch_path_cost: `6`
  - high_dispatch_target_cost: `6`
  - low_path_corridor_clearance: `11`
  - map_narrow_passage: `10`
  - nav2_progress_cluster: `6`
- success 中没有：
  - high_timeout_robot_cost: `0`
  - timeout_squeezed: `0`

结论：

- dispatch target/path high cost 和 `/map` narrow-passage 不能单独作为 branch-selection penalty。
- timeout-side robot cost cluster 更能区分 timeout vs success，但它是 outcome-time 信号，不一定能直接用于 dispatch-time branch selection。

### 5.3 blocked/blacklist 是否仍保持 0？

否。

证据：

- run1 blocked/blacklist: `0 / 0`
- run2 blocked/blacklist: `0 / 0`
- run3 blocked/blacklist: `1 / 1`

结论：

- Phase 20 fixed-parameter repeat 没有保持 blocked/blacklist 非回归。
- run3 的 failure 模式从 timeout 转成 blocked_nav2/blacklist，说明同一 baseline 下 failure mode 不稳定。

### 5.4 EXIT_REACHED 成功率如何？

0/3。

证据：

- run1: `FAILED_EXHAUSTED`
- run2: `FAILED_EXHAUSTED`
- run3: `FAILED_EXHAUSTED`

结论：

- Phase 19 单次 `EXIT_REACHED` 不稳定。
- 当前 baseline 不能视为 navigation-stable。

### 5.5 timeout count 是否稳定或波动？

波动明显。

证据：

- run1 timeout_count: `5`
- run2 timeout_count: `3`
- run3 timeout_count: `0`

结论：

- timeout count 不稳定。
- 同一参数下，failure 可以表现为 timeout，也可以表现为 blocked_nav2/blacklist。

## 6. Phase 20 解释

Phase 20 说明：

1. Phase 19 的 `EXIT_REACHED` 是可实现但不稳定，不是可靠成功。
2. timeout 与 Nav2 progress/recovery/abort cluster 高度相关：8/8 timeout 都有该标签。
3. local-cost timeout robot cluster 也是强信号：6/8 timeout 有 high robot cost 和 squeezed。
4. 但 dispatch-side high cost、`/map` narrow-passage、low corridor clearance 也常出现在 success goals 中，不能单独用于 branch scoring。
5. run3 没有 timeout，却出现 blocked/blacklist，说明根因不是单纯 “goal timeout 时间太短”，也不是单纯 “branch target geometry 太差”。
6. 当前最稳定的跨 run 信号是 controller/Nav2 failure dynamics，而不是 branch-selection geometry。

## 7. Phase 21 建议

不建议 Phase 21 直接做 branch scoring。

理由：

- `branch_selection_ready` 为 `false`。
- EXIT_REACHED 成功率 0/3。
- timeout count 不稳定。
- blocked/blacklist 在 run3 回归。
- success 也共享很多 dispatch-side high-cost 和 narrow-passage 特征。
- caused_blocked_or_blacklist 在 timeout 中是 0/8，说明 timeout 本身没有稳定导致 branch/blacklist。

建议 Phase 21 做 controller diagnostics，而不是 branch scoring。

Phase 21 目标：定位 Nav2 controller failure dynamics。

建议新增诊断项：

- `/cmd_vel` 或 `cmd_vel_nav`：controller failure 前后是否输出接近 0。
- `/odom`：failure 前 5-10 秒真实位移、角速度、是否卡住/oscillation。
- robot pose delta：每个 goal 的前后位移和 yaw 变化。
- local costmap footprint/robot-neighborhood cost：当前已有 timeout-side robot cluster，可继续保留。
- Nav2 log interval：保留 progress failure、recovery、abort 与 goal interval 对齐。
- failure type 区分：timeout vs blocked_nav2 vs terminal exhausted。

Phase 21 验收建议：

- 不改 branch selection。
- 不改 Nav2 参数。
- 先新增 controller/odom/cmd_vel diagnostics。
- 至少跑 2 次 smoke，看 blocked_nav2 与 timeout 是否共享同一 controller-level 模式。
- 只有在明确 root cause 后再决定 Phase 22 是 controller tuning、goal tolerance、branch scoring，还是 exit approach。

## 8. 当前不建议做的事

暂不建议：

- 直接加入 branch scoring penalty。
- 直接根据 dispatch local cost blacklist target。
- 直接恢复或修改 Phase 16 progress profile。
- 直接加入 near-exit approach mode。
- 直接把 high local cost 阈值用于控制。

原因：

- 当前阈值只适合 taxonomy，不适合 control。
- success goals 共享部分 high-cost/narrow-passage 特征。
- failure mode 在 timeout 与 blocked_nav2 之间切换。

## 9. 一句话结论

Phase 20 证明，当前 20260522 版工程的主要问题不是“缺诊断”，而是同一 baseline 下 Nav2/controller failure dynamics 不稳定：3 次 repeat smoke 全部 `FAILED_EXHAUSTED`，timeout 数量从 5 到 3 到 0 波动，run3 出现 blocked/blacklist。timeout 和 local-cost robot cluster 有强相关，但 dispatch-side branch-selection 信号不能区分 success 与 timeout。Phase 21 最合理方向是 controller/odom/cmd_vel 级诊断，而不是 branch scoring 或参数猜测。
