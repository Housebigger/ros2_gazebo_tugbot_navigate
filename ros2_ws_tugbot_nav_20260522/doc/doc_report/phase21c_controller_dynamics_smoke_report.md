# Phase 21C Controller Dynamics Smoke Report

生成时间：2026-05-24

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 21C 的目标不是优化导航策略，而是用真实 runtime 数据验证 timeout / blocked_nav2 是否属于以下 controller dynamics 模式：

- `stuck_with_cmd`
- `controller_silent`
- `oscillation_candidate`
- `slow_progress`
- `healthy_motion_but_failed`

本阶段仍不改：

- branch selection
- Nav2 参数
- Phase 16 progress profile
- near-exit approach mode

## 2. 执行

先做 preflight：

- 检查旧 ROS/Gazebo/Nav2/recorder 残留。
- 发现 Phase 20 遗留的 `ros_gz_bridge`、static TF publishers、`maze_goal_monitor`。
- 清理残留后 `ros2 daemon stop`，确认 ROS graph 清空。

第一次运行 wrapper 失败：

```text
/opt/ros/jazzy/setup.sh: line 124: AMENT_TRACE_SETUP_FILES: unbound variable
```

原因：`tools/run_phase21_controller_diagnostics_smoke.sh` 使用 `set -u`，而 ROS setup 脚本对未绑定变量不兼容。

修复：

```bash
set +u
. /opt/ros/jazzy/setup.sh
. install/setup.sh
set -u
```

之后运行：

```bash
bash tools/run_phase21_controller_diagnostics_smoke.sh phase21_run1
```

运行成功，exit code 0。

## 3. Artifacts

生成文件：

- `log/phase21_run1_launch.log`
- `log/phase21_run1_goal_events.jsonl`
- `log/phase21_run1_explorer_state.jsonl`
- `log/phase21_run1_controller_dynamics.jsonl`
- `log/phase21_run1_goal_nav2_analysis.json`
- `log/phase21_run1_geometry_nav2_summary.json`
- `log/phase21_run1_goal_event_cost_summary.json`
- `log/phase21_run1_goal_controller_dynamics.json`

artifact sizes 已确认：

- `phase21_run1_controller_dynamics.jsonl`: 2,434,658 bytes
- `phase21_run1_explorer_state.jsonl`: 346,774 bytes
- `phase21_run1_goal_controller_dynamics.json`: 7,633 bytes
- `phase21_run1_goal_events.jsonl`: 38,939 bytes
- `phase21_run1_goal_nav2_analysis.json`: 6,823 bytes

post-run cleanup：

- wrapper 清理大多数进程。
- 检查后发现 `maze_explorer` 仍短暂残留。
- 已手动清理并再次 `ros2 daemon stop`。
- 最终无 ROS/Gazebo/Nav2/recorder 残留。

## 4. Final state

`phase21_run1_explorer_state.jsonl` 最终状态：

```json
{
  "mode": "EXIT_REACHED",
  "exit_distance_m": 0.591274697000305,
  "goal_count": 8,
  "goal_success_count": 4,
  "goal_failure_count": 3,
  "timeout_cancel_count": 3,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "last_terminal_reason": "exit_reached"
}
```

Nav2 summary：

```json
{
  "goal_count": 8,
  "success_count": 4,
  "success_with_progress_failure_count": 0,
  "success_with_recovery_count": 0,
  "timeout_count": 3,
  "timeout_with_progress_failure_count": 3,
  "timeout_with_recovery_count": 3
}
```

## 5. Controller dynamics summary

`phase21_run1_goal_controller_dynamics.json` summary：

```json
{
  "goal_count": 7,
  "healthy_motion_count": 7,
  "stuck_with_cmd_count": 0,
  "controller_silent_count": 0,
  "oscillation_candidate_count": 0,
  "slow_progress_count": 0,
  "insufficient_data_count": 0
}
```

注意：controller analyzer 覆盖 7 个具备完整 dispatch/outcome interval 的 goal。Goal 1 没有完整 interval，因此不计入 controller summary。

## 6. Per-goal controller dynamics

| seq | outcome | classification | odom_distance_m | yaw_abs_sum_rad | cmd_linear_mean | near_zero_cmd_ratio | last_10s_odom_distance_m | last_10s_cmd_linear_mean | progress/recovery/abort |
|---:|---|---|---:|---:|---:|---:|---:|---:|---|
| 2 | timeout | healthy_motion | 1.843969 | 1.639638 | 0.052700 | 0.338549 | 0.029608 | 0.003231 | 2 / 3 / 2 |
| 3 | success | healthy_motion | 0.736453 | 0.190347 | 0.204368 | 0.013333 | 0.736453 | 0.204368 | 0 / 0 / 0 |
| 4 | timeout | healthy_motion | 1.067868 | 1.700893 | 0.030523 | 0.584286 | 0.031757 | 0.003559 | 2 / 3 / 2 |
| 5 | success | healthy_motion | 0.706668 | 0.200993 | 0.220360 | 0.014925 | 0.706668 | 0.220360 | 0 / 0 / 0 |
| 6 | success | healthy_motion | 0.673735 | 0.011547 | 0.296285 | 0.020833 | 0.673735 | 0.296285 | 0 / 0 / 0 |
| 7 | timeout | healthy_motion | 1.338159 | 1.630858 | 0.036842 | 0.456825 | 0.035546 | 0.003610 | 2 / 1 / 2 |
| 8 | terminal_cancel | healthy_motion | 1.407521 | 1.604079 | 0.058765 | 0.064718 | 0.548012 | 0.054177 | 1 / 1 / 1 |

## 7. Local cost summary for timeouts

Timeout local-cost summary：

```text
timeout count: 3
coverage mean/min: 1.0 / 1.0
dispatch target cost mean: 30.67
dispatch path cost max mean: 46.0
timeout robot cost max mean: 99.33
timeout obstacle cluster sum: 354
squeezed count: 3 / 3
```

即：三个 timeout 都有 strong timeout-side local-cost cluster / squeezed。

## 8. 关键解释

Phase 21C 的重要发现：

1. 这次 run 成功到出口：`EXIT_REACHED`。
2. 仍有 3 个 timeout，且 3/3 timeout 都有 Nav2 progress/recovery/abort cluster。
3. 3/3 timeout 都有 strong timeout-side local-cost cluster / squeezed。
4. 但 controller dynamics 没有把 timeout 归类为：
   - `stuck_with_cmd`
   - `controller_silent`
   - `oscillation_candidate`
   - `slow_progress`
5. analyzer 把全部 goal 都归为 `healthy_motion`，原因是整段 goal interval 内 odom_distance 都足够大。
6. 但 timeout goals 的 last 10s 出现明显停滞：
   - seq 2 last_10s_odom_distance_m = 0.0296
   - seq 4 last_10s_odom_distance_m = 0.0318
   - seq 7 last_10s_odom_distance_m = 0.0355
   - last_10s_cmd_linear_mean 约 0.003-0.004

这说明现有 classification 过粗：

- full-interval movement 使 timeout 被归为 `healthy_motion`
- 但最后 10 秒已经显示 controller 近乎 silent + odom 近乎停滞

因此更准确的分类应增加 windowed failure mode，例如：

- `late_controller_silent`
- `late_stall_after_progress`
- 或 `healthy_motion_but_late_stall`

## 9. 当前结论

Phase 21C run1 不支持“机器人全程 stuck_with_cmd”的解释。

也不支持“全程 oscillation”的解释。

它更像：

```text
机器人在 goal interval 前中段有明显运动，但 timeout 前最后 10 秒进入 controller-near-silent + near-zero-odom stall，同时 local costmap 显示 robot 周围 high cost / squeezed。
```

这和 Phase 20 的结论兼容：

- timeout 与 Nav2 progress/recovery/abort cluster 强相关。
- timeout 与 timeout-side local-cost cluster 强相关。
- 但 dispatch-time branch geometry/cost 不足以安全直接做 branch scoring。

## 10. Phase 22 建议

不建议立即 branch scoring。

不建议直接调 progress checker。

建议 Phase 22A 先改进 analyzer，而不是改导航行为：

### Phase 22A：windowed controller dynamics taxonomy

目标：让 analyzer 区分 full-interval motion 和 failure-window motion。

新增分类：

- `late_controller_silent`
  - last 10s cmd linear/ang near zero
  - last 10s odom distance near zero
- `late_stuck_with_cmd`
  - last 10s cmd active
  - last 10s odom near zero
- `late_oscillation`
  - last 10s yaw delta/angular cmd 高
  - xy movement 低
- `healthy_motion_but_late_stall`
  - full interval odom distance healthy
  - last window stall
- `healthy_motion_but_timed_out`
  - full interval healthy
  - last window not stalled
  - still timeout

然后用 Phase 21C run1 artifacts 作为 fixture 写 RED/GREEN test：

- seq 2/4/7 应被分类为 `healthy_motion_but_late_stall` 或 `late_controller_silent`
- success seq 3/5/6 不应被误判为 late stall

### Phase 22B：再跑 1-2 次 smoke

等 windowed analyzer 稳定后，再跑：

- phase22_run1
- phase22_run2

判断 late stall 是否跨 run 重复。

只有如果 late stall 稳定出现，再决定 Phase 23 是：

- controller tuning
- goal tolerance / timeout policy
- local costmap/inflation/controller critic targeted analysis
- 或极保守 branch scoring

## 11. 当前不做的事

仍不建议：

- branch scoring
- local-cost threshold control
- Nav2 progress checker 粗调
- near-exit approach mode

原因：

- 本次 run 已到出口，但 timeout 前 last-window stall 才是新证据。
- 需要先把 analyzer 分类做准，再谈策略。

## 12. 一句话结论

Phase 21C run1 成功到出口，但 3 个 timeout 都表现为“整段 goal 有运动、最后 10 秒近乎停住”，且同时伴随 local-cost squeezed / high robot cost。下一步不应直接改导航策略，而应在 Phase 22A 增强 analyzer，加入 last-window failure taxonomy，把 `healthy_motion_but_late_stall` 从普通 `healthy_motion` 中分离出来。
