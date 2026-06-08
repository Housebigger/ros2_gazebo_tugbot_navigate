# Phase 23A Controller / Local-Cost Failure-Window Introspection

生成时间：2026-05-24

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 23A 的核心问题：

```text
为什么 timeout 前最后 10 秒 controller 几乎不输出有效 cmd_vel，而 local costmap 显示 robot squeezed/high cost？
```

本阶段先做 analysis-only introspection，不改：

- branch selection
- Nav2 参数
- controller 参数
- local costmap 参数
- goal tolerance / timeout policy

## 2. 新增测试

新增：

- `src/tugbot_maze/test/test_phase23_failure_window_introspection.py`

测试构造 synthetic artifacts：

- controller analysis JSON
- controller dynamics JSONL
- local-cost summary JSON
- Nav2 analysis JSON

Contract 覆盖：

- last-window cmd velocity distribution：
  - min / mean / max / p95
  - last nonzero cmd time
  - near-zero cmd start time
  - near-zero duration
- local-cost outcome fields：
  - timeout robot local cost max/mean
  - obstacle cluster count
  - squeezed flag
- Nav2 timing fields：
  - first/last progress failure
  - first/last clear costmap
  - first/last controller abort
  - near-zero cmd 与 first progress failure 的时间关系
- 当前 artifact 是否包含更细 diagnostics：
  - footprint cost stats
  - front/side clearance
  - path-ahead cost
  - Nav2 event timestamps

RED 已确认：

```text
can't open file 'tools/analyze_failure_windows.py'
```

## 3. 新增 analyzer

新增：

- `tools/analyze_failure_windows.py`

用法：

```bash
python3 tools/analyze_failure_windows.py \
  --controller-analysis log/phase22_run1_goal_controller_dynamics.json \
  --controller-dynamics log/phase22_run1_controller_dynamics.jsonl \
  --local-cost-summary log/phase22_run1_goal_event_cost_summary.json \
  --nav2-analysis log/phase22_run1_goal_nav2_analysis.json \
  --output-json log/phase22_run1_failure_windows.json
```

输出：

- stdout CSV
- JSON：`summary` + `failure_windows`

当前 analyzer 可从现有 artifacts 得到：

- failure window start/end
- cmd linear/angular min/mean/max/p95
- last nonzero cmd timestamp
- near-zero cmd start timestamp
- near-zero cmd duration
- timeout local-cost fields
- available diagnostics flags

当前 analyzer 不能从现有 artifacts 得到：

- footprint cell cost stats
- front/side clearance
- path ahead 0.5m / 1.0m cost
- real Nav2 event timestamps

这些需要 Phase 23B 新增 runtime recorder 或增强 Nav2 log analyzer。

## 4. Verification

通过：

```bash
python3 -m py_compile tools/analyze_failure_windows.py
```

通过 focused tests：

```bash
python3 -m pytest -q \
  src/tugbot_maze/test/test_phase23_failure_window_introspection.py \
  src/tugbot_maze/test/test_phase21_controller_dynamics_analysis.py \
  src/tugbot_maze/test/test_phase22_windowed_controller_dynamics.py
```

结果：

```text
4 passed in 0.19s
```

通过完整 tugbot_maze tests：

```bash
python3 -m pytest -q src/tugbot_maze/test
```

结果：

```text
68 passed in 0.43s
```

## 5. Existing artifact analysis

对以下 run 运行 analyzer：

- `phase21_run1`
- `phase22_run1`
- `phase22_run2`

生成：

- `log/phase21_run1_failure_windows.json`
- `log/phase22_run1_failure_windows.json`
- `log/phase22_run2_failure_windows.json`
- `log/phase23a_failure_windows_aggregate.json`

## 6. Failure-window summary

### phase22_run1

4 个 timeout failure windows：

| seq | cmd_mean | cmd_max | cmd_p95 | near_zero_duration_s | robot_cost_max | squeezed | obstacle_cluster |
|---:|---:|---:|---:|---:|---:|---|---:|
| 2 | 0.003621 | 0.013921 | 0.007502 | 2.249907 | 100 | true | 158 |
| 4 | 0.003431 | 0.010774 | 0.008017 | 2.250105 | 97 | true | 2 |
| 7 | 0.002833 | 0.012017 | 0.007387 | 8.425919 | 100 | true | 305 |
| 8 | 0.003207 | 0.010987 | 0.008775 | 3.449841 | 100 | true | 443 |

### phase22_run2

2 个 timeout failure windows：

| seq | cmd_mean | cmd_max | cmd_p95 | near_zero_duration_s | robot_cost_max | squeezed | obstacle_cluster |
|---:|---:|---:|---:|---:|---:|---|---:|
| 2 | 0.003690 | 0.014001 | 0.007678 | 1.700895 | 99 | true | 125 |
| 10 | 0.004053 | 0.012364 | 0.009753 | 0.400053 | 99 | true | 97 |

### phase21_run1 note

`phase21_run1_goal_controller_dynamics.json` was produced before Phase 22A windowed taxonomy, so its classification remains `healthy_motion` in that file. However, cmd/local-cost failure-window values are still usable:

- 3/3 squeezed
- robot cost max 99/99/100
- cmd_mean around 0.0032-0.0036
- near-zero duration 1.95s / 2.85s / 6.00s

If desired, regenerate `phase21_run1_goal_controller_dynamics.json` with the Phase 22A analyzer to align classifications.

## 7. Aggregate observations

Across phase22_run1 + phase22_run2：

```text
failure_window_count = 6
healthy_motion_but_late_stall = 6 / 6
late_controller_silent = 6 / 6
squeezed = 6 / 6
high_timeout_robot_cost = 6 / 6
```

Cmd velocity in failure window：

```text
cmd_linear_abs_mean: 0.0028 - 0.0041 m/s
cmd_linear_abs_max: 0.0108 - 0.0140 m/s
cmd_linear_abs_p95: 0.0074 - 0.0098 m/s
```

This confirms the command output is not merely “low average because of a few zeros”; it is consistently tiny throughout the failure window.

Near-zero duration varies：

```text
0.40s - 8.43s
```

Interpretation：

- Some failures transition to near-zero very late, close to timeout/abort.
- Some enter near-zero much earlier in the final window.
- Existing artifacts do not yet contain enough Nav2 event timestamps to align near-zero onset with progress failure/recovery/abort.

Local-cost：

- All failure windows have `footprint_corridor_inflation_squeezed=true`.
- All have `timeout_robot_local_cost_max >= 97`.
- Obstacle cluster size varies widely: `2` to `443`.

Interpretation：

- High local robot cost and squeezed flag are stable.
- Cluster size magnitude is not stable enough alone to drive policy.

## 8. Missing diagnostics

Current available_diagnostics flags are all false for：

- `footprint_cost_stats`
- `front_side_clearance`
- `path_ahead_cost`
- `nav2_event_times`

Therefore, current artifacts cannot yet answer：

1. Is the footprint itself inside inflated/lethal cost, or just adjacent?
2. Is the front corridor blocked while sides remain open?
3. Does the local path ahead enter inflated/lethal cells?
4. Did near-zero cmd begin before or after progress failure / recovery / abort?

## 9. Phase 23B recommendation

Phase 23B should add runtime diagnostics, not tune behavior yet。

Recommended additions：

1. Enhance local-cost diagnostics in `/maze/goal_events` or a new recorder：
   - timeout footprint cost max/mean/p95
   - footprint inflated/lethal cell counts
   - front wedge clearance/cost
   - left/right side clearance/cost
   - path ahead 0.5m and 1.0m cost max/mean

2. Enhance Nav2 log analyzer：
   - extract event timestamps for progress failure
   - extract clear costmap recovery timestamps
   - extract controller abort timestamps
   - include these in `goal_nav2_analysis.json`

3. Re-run one smoke and feed Phase 23A analyzer：
   - answer whether near-zero cmd starts before or after progress failure/recovery/abort
   - answer whether local path/footprint geometry explains controller silence

## 10. Current do-not-do list

Still do not：

- branch scoring
- local-cost threshold branch control
- controller/Nav2 parameter tuning
- progress checker tuning
- near-exit approach mode

Reason：we have narrowed the symptom, but not the controller/local-cost causal chain.
