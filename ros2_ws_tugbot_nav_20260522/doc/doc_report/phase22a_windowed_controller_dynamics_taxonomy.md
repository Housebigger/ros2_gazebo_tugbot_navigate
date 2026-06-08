# Phase 22A Windowed Controller Dynamics Taxonomy

生成时间：2026-05-24

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 22A 目标是增强 `tools/analyze_goal_controller_dynamics.py`，把 full-interval motion 和 failure-window motion 分开。

Phase 21C 发现：

- timeout goals 在整个 goal interval 内 odom distance 足够大，因此旧 analyzer 归类为 `healthy_motion`。
- 但 timeout 前 last 10s 里 cmd_vel 和 odom 位移都接近 0。
- 这类情况应从普通 `healthy_motion` 中分离出来。

本阶段仍不改导航行为，不跑 Gazebo。

## 2. 新增测试

新增真实 regression fixture：

- `src/tugbot_maze/test/test_phase22_windowed_controller_dynamics.py`

使用 Phase 21C 真实 artifacts：

- `log/phase21_run1_goal_events.jsonl`
- `log/phase21_run1_controller_dynamics.jsonl`
- `log/phase21_run1_goal_nav2_analysis.json`

RED 期望：

- timeout seq 2 / 4 / 7 不应再是普通 `healthy_motion`。
- 它们应被识别为 `healthy_motion_but_late_stall`。
- 它们应具有 `late_controller_silent=true`。
- success seq 3 / 5 / 6 不应误判为 late stall。
- terminal_cancel seq 8 标记为 `terminal_cancel_after_exit`，不参与 timeout/failure late stall。

初始 RED 失败：

```text
KeyError: 'healthy_motion_but_late_stall_count'
```

说明旧 analyzer 没有 windowed taxonomy。

补充 synthetic contract：

- 修改 `src/tugbot_maze/test/test_phase21_controller_dynamics_analysis.py`
- 新增 synthetic case 覆盖：
  - `late_controller_silent`
  - `late_stuck_with_cmd`
  - `late_oscillation`
  - `healthy_motion_but_timed_out`

初始 RED 失败：

```text
KeyError: 'late_controller_silent_count'
```

## 3. 实现变更

修改：

- `tools/analyze_goal_controller_dynamics.py`

新增 flags：

- `late_controller_silent`
- `late_stuck_with_cmd`
- `late_oscillation`
- `healthy_motion_but_late_stall`
- `healthy_motion_but_timed_out`
- `timeout_or_failure_late_stall`
- `terminal_cancel_after_exit`

新增 summary counts：

- `late_controller_silent_count`
- `late_stuck_with_cmd_count`
- `late_oscillation_count`
- `healthy_motion_but_late_stall_count`
- `healthy_motion_but_timed_out_count`
- `timeout_or_failure_late_stall_count`
- `terminal_cancel_after_exit_count`

分类逻辑：

- full interval still computes:
  - `healthy_motion`
  - `stuck_with_cmd`
  - `oscillation_candidate`
  - `controller_silent`
  - `slow_progress`
- last-window computes:
  - last 10s odom distance
  - last 10s yaw delta
  - last 10s cmd linear/ang mean
- timeout/failure + full healthy + late stall => `healthy_motion_but_late_stall`
- timeout + full healthy + no late stall => `healthy_motion_but_timed_out`
- terminal cancel after exit is tracked but not counted as timeout/failure late stall。

Important precedence:

- Existing full-interval classifications like `stuck_with_cmd`, `oscillation_candidate`, `controller_silent` are preserved for synthetic Phase 21 tests.
- Windowed late-stall classification only overrides when full interval is otherwise healthy motion and outcome is timeout/failure.

## 4. Verification

通过：

```bash
python3 -m py_compile tools/analyze_goal_controller_dynamics.py
```

通过 focused tests：

```bash
python3 -m pytest -q \
  src/tugbot_maze/test/test_phase21_controller_dynamics_analysis.py \
  src/tugbot_maze/test/test_phase22_windowed_controller_dynamics.py
```

结果：

```text
3 passed in 0.17s
```

通过完整 tugbot_maze tests：

```bash
python3 -m pytest -q src/tugbot_maze/test
```

结果：

```text
67 passed in 0.39s
```

## 5. Phase 21C artifact re-analysis

重新分析 Phase 21C：

输出：

- `log/phase21_run1_goal_controller_dynamics_phase22a.json`

summary：

```json
{
  "goal_count": 7,
  "healthy_motion_count": 4,
  "healthy_motion_but_late_stall_count": 3,
  "healthy_motion_but_timed_out_count": 0,
  "late_controller_silent_count": 3,
  "late_stuck_with_cmd_count": 0,
  "late_oscillation_count": 0,
  "timeout_or_failure_late_stall_count": 3,
  "terminal_cancel_after_exit_count": 1,
  "stuck_with_cmd_count": 0,
  "controller_silent_count": 0,
  "oscillation_candidate_count": 0,
  "slow_progress_count": 0,
  "insufficient_data_count": 0
}
```

Per-goal result：

| seq | outcome | classification | late_controller_silent | last_10s_odom_distance_m | last_10s_cmd_linear_abs_mean |
|---:|---|---|---|---:|---:|
| 2 | timeout | healthy_motion_but_late_stall | true | 0.029608 | 0.003231 |
| 3 | success | healthy_motion | false | 0.736453 | 0.204368 |
| 4 | timeout | healthy_motion_but_late_stall | true | 0.031757 | 0.003559 |
| 5 | success | healthy_motion | false | 0.706668 | 0.220360 |
| 6 | success | healthy_motion | false | 0.673735 | 0.296285 |
| 7 | timeout | healthy_motion_but_late_stall | true | 0.035546 | 0.003610 |
| 8 | terminal_cancel | healthy_motion | false | 0.548012 | 0.054177 |

## 6. 结论

Phase 22A 修正了 Phase 21C 的关键误分类：

旧分类：

```text
seq 2/4/7 timeout => healthy_motion
```

新分类：

```text
seq 2/4/7 timeout => healthy_motion_but_late_stall + late_controller_silent
```

这更准确描述了实际现象：

> timeout goals 在 goal 前中段有足够运动，但最后 10 秒进入 cmd_vel 近零、odom 位移近零的 late stall。

success goals 未被误判为 late stall。

terminal_cancel after exit 被标注为 `terminal_cancel_after_exit`，不参与 timeout/failure late stall。

## 7. 下一步建议

Phase 22B：用 windowed analyzer 跑 1-2 次真实 smoke。

目标：验证 `healthy_motion_but_late_stall + late_controller_silent` 是否跨 run 稳定出现。

继续保持：

- 不改 branch selection
- 不改 Nav2 参数
- 不启用 Phase 16 profile
- 不使用 taxonomy 阈值控制

如果 late stall 稳定出现，Phase 23 可以考虑：

1. controller-level root cause：为什么最后 10 秒 controller 近乎 silent？
2. local costmap / inflation / controller critic analysis：因为 Phase 21C 的 timeout 也有 squeezed/high robot cost。
3. goal tolerance / timeout policy targeted analysis：但不能做 Phase 16 那样的粗粒度 progress checker 调参。
4. branch scoring 仍不是优先项，除非后续证明 dispatch-time features 能可靠区分 success vs late stall。
