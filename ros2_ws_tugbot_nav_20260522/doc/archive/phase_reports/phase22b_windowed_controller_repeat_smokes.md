# Phase 22B Windowed Controller Dynamics Repeat Smokes

生成时间：2026-05-24

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 22B 目标：用 Phase 22A 的 windowed controller dynamics analyzer 跑 1-2 次真实 smoke，验证以下模式是否跨 run 稳定：

```text
healthy_motion_but_late_stall + late_controller_silent
```

继续保持：

- 不改 branch selection
- 不改 Nav2 参数
- 不启用 Phase 16 profile
- 不使用 taxonomy 阈值控制

## 2. 执行准备

对 `tools/run_phase21_controller_diagnostics_smoke.sh` 做了两个小修正：

1. 支持 `phase22_runN` run_id。
2. cleanup pattern 加入 `maze_explorer`，避免 Phase 21C 后看到的短暂残留。

验证：

```bash
bash -n tools/run_phase21_controller_diagnostics_smoke.sh
python3 -m pytest -q \
  src/tugbot_maze/test/test_phase21_controller_diagnostics_smoke_tools.py \
  src/tugbot_maze/test/test_phase21_controller_dynamics_analysis.py \
  src/tugbot_maze/test/test_phase22_windowed_controller_dynamics.py
```

结果：

```text
5 passed in 0.17s
```

Preflight：

- 检查 ROS/Gazebo/Nav2/recorder 残留。
- `ros2 daemon stop`。
- 未发现真实残留进程。

## 3. Smoke runs

运行：

```bash
bash tools/run_phase21_controller_diagnostics_smoke.sh phase22_run1
bash tools/run_phase21_controller_diagnostics_smoke.sh phase22_run2
```

两次 wrapper 都 exit code 0。

Post-run cleanup 检查：

- phase22_run1 后无残留。
- phase22_run2 后无残留。

## 4. Artifacts

新增 aggregate：

- `log/phase22b_windowed_runs_aggregate.json`

每个 run 生成：

- `log/phase22_runN_launch.log`
- `log/phase22_runN_goal_events.jsonl`
- `log/phase22_runN_explorer_state.jsonl`
- `log/phase22_runN_controller_dynamics.jsonl`
- `log/phase22_runN_goal_nav2_analysis.json`
- `log/phase22_runN_geometry_nav2_summary.json`
- `log/phase22_runN_goal_event_cost_summary.json`
- `log/phase22_runN_goal_controller_dynamics.json`

Phase 21C comparison baseline：

- `phase21_run1`

## 5. Run-level summary

| run | final_mode | goals | success | timeout/failure | timeout_count | blocked/blacklist | exit_distance_m |
|---|---|---:|---:|---:|---:|---|---:|
| phase21_run1 | EXIT_REACHED | 8 | 4 | 3 | 3 | 0 / 0 | 0.5913 |
| phase22_run1 | EXIT_REACHED | 9 | 4 | 4 | 4 | 0 / 0 | 0.5993 |
| phase22_run2 | FAILED_EXHAUSTED | 12 | 10 | 2 | 2 | 0 / 0 | 1.7235 |

Nav2 timeout progress/recovery：

| run | timeout_count | timeout_with_progress_failure | timeout_with_recovery |
|---|---:|---:|---:|
| phase21_run1 | 3 | 3 | 3 |
| phase22_run1 | 4 | 4 | 4 |
| phase22_run2 | 2 | 2 | 2 |

## 6. Windowed controller taxonomy

| run | controller goals | healthy_motion | healthy_motion_but_late_stall | late_controller_silent | late_stuck_with_cmd | late_oscillation | healthy_motion_but_timed_out |
|---|---:|---:|---:|---:|---:|---:|---:|
| phase21_run1 | 7 | 4 | 3 | 3 | 0 | 0 | 0 |
| phase22_run1 | 8 | 4 | 4 | 4 | 0 | 0 | 0 |
| phase22_run2 | 12 | 9 | 2 | 2 | 0 | 0 | 0 |

Timeout sequences classified as `healthy_motion_but_late_stall + late_controller_silent`：

| run | timeout sequences | late-silent timeout sequences |
|---|---|---|
| phase21_run1 | 2, 4, 7 | 2, 4, 7 |
| phase22_run1 | 2, 4, 7, 8 | 2, 4, 7, 8 |
| phase22_run2 | 2, 10 | 2, 10 |

Across these 3 diagnostic runs：

```text
timeout total = 9
late_controller_silent timeout = 9
healthy_motion_but_late_stall timeout = 9
late_stuck_with_cmd timeout = 0
late_oscillation timeout = 0
healthy_motion_but_timed_out = 0
```

## 7. Last-window values

Timeout last 10s odom distance：

| run | seq -> last_10s_odom_distance_m |
|---|---|
| phase21_run1 | 2: 0.029608, 4: 0.031757, 7: 0.035546 |
| phase22_run1 | 2: 0.034418, 4: 0.032711, 7: 0.026439, 8: 0.031489 |
| phase22_run2 | 2: 0.032708, 10: 0.039081 |

Timeout last 10s cmd linear mean：

| run | seq -> last_10s_cmd_linear_abs_mean |
|---|---|
| phase21_run1 | 2: 0.003231, 4: 0.003559, 7: 0.003610 |
| phase22_run1 | 2: 0.003621, 4: 0.003431, 7: 0.002833, 8: 0.003207 |
| phase22_run2 | 2: 0.003690, 10: 0.004053 |

The last-window stall is tight and repeatable:

- odom movement roughly `0.026 - 0.039 m` in last 10s
- linear cmd mean roughly `0.0028 - 0.0041 m/s` in last 10s

## 8. Local-cost correlation

Timeout local-cost summary：

| run | timeout count | squeezed count | timeout robot cost max mean | timeout obstacle cluster sum | dispatch target cost mean | dispatch path cost max mean |
|---|---:|---:|---:|---:|---:|---:|
| phase21_run1 | 3 | 3 | 99.33 | 354 | 30.67 | 46.00 |
| phase22_run1 | 4 | 4 | 99.25 | 908 | 45.75 | 57.25 |
| phase22_run2 | 2 | 2 | 99.00 | 222 | 0.00 | 20.00 |

Across the 3 runs：

```text
timeout count = 9
squeezed timeout = 9
timeout robot cost max mean ~= 99
```

This strongly supports:

```text
late_controller_silent + local-cost squeezed/high robot cost
```

as a stable timeout signature.

Important nuance：

- phase22_run2 dispatch target cost mean is 0.0 and dispatch path cost mean is 20.0.
- Yet both timeouts still became late_controller_silent + squeezed/high timeout robot cost.

So dispatch-time target/path cost still does not safely justify branch scoring.

## 9. Interpretation

Phase 22B confirms the Phase 22A hypothesis across additional smokes：

Timeouts are consistently:

```text
healthy full-interval motion
+ last-10s controller near-silent
+ last-10s odom near-zero
+ timeout-side local-cost squeezed/high robot cost
+ Nav2 progress/recovery cluster
```

They are not:

- full-interval stuck_with_cmd
- full-interval controller_silent
- full-interval oscillation
- late_stuck_with_cmd
- late_oscillation
- healthy_motion_but_timed_out

## 10. Phase 23 recommendation

Do not do branch scoring yet.

Do not do coarse progress checker tuning yet.

Phase 23 should investigate root cause of:

```text
Why does the controller enter near-silent command output during the final 10s, while the local costmap says the robot is squeezed/high-cost?
```

Recommended Phase 23A：controller/local-cost failure-window introspection.

Add diagnostics around each late stall window：

1. Controller command details：
   - cmd linear/angular distributions, not just means
   - last nonzero cmd timestamp
   - duration since cmd went near-zero

2. Local costmap around robot footprint：
   - robot cell cost
   - footprint cells cost stats
   - front/side clearance estimates in local costmap frame
   - whether inflated obstacle surrounds footprint

3. Path/costmap relationship：
   - path segment through local costmap near robot
   - cost along next 0.5m / 1.0m of path
   - whether path enters lethal/inflated cells near robot

4. Nav2/controller log window：
   - final progress-failure window timestamps
   - recovery clear costmap timing
   - controller abort timing

Potential Phase 23B decisions after diagnostics：

- controller critic / local planner parameters
- local costmap inflation/footprint analysis
- goal tolerance / timeout policy
- only later: extremely conservative branch scoring if dispatch-time evidence improves

## 11. One-line conclusion

Phase 22B confirms across 3 diagnostic runs that all 9 timeouts share the same signature: full-interval healthy motion followed by last-10s `late_controller_silent` stall, with timeout-side local-cost squeezed/high robot cost. The next work should target controller/local-cost failure-window introspection, not branch scoring.
