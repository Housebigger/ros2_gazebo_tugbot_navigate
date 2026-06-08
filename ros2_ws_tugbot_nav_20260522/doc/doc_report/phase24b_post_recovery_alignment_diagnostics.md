# Phase 24B Post-Recovery Path / Local-Cost Alignment Diagnostics

生成时间：2026-05-25

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 24B 目标：在不改变导航行为的前提下，分析 Phase 24A 发现的主导 timing signature：

```text
cmd_silent_after_recovery_abort: 6 / 7
```

重点问题：

```text
clear-costmap / recovery / controller abort 后，controller 是否仍收到 path update？
如果收到 path，为什么 cmd_vel 仍进入 near-zero？
现有 local-cost timeout snapshot 是否足够解释？
```

用户指定的诊断目标：

```text
clear-costmap recovery 前后 local cost/path snapshot
recovery 时 active path ahead cost
robot pose 到 local/global path 最近点距离
recovery 后 controller 是否继续收到 Passing new path to controller
recovery 后到 near-zero onset 之间 path update count/time
controller 收到 path 但 cmd 仍 near-zero 的 interval
```

## 2. Discovery

现有 artifacts 已有：

- launch log 中的 `Passing new path to controller`
- Nav2 analyzer 中的:
  - `clear_costmap_times`
  - `progress_failure_times`
  - `controller_abort_times`
- controller dynamics recorder 中的 cmd/odom samples
- failure-window JSON 中的 timeout-side local-cost snapshot

现有 artifacts 暂无：

- recovery 前/后 local cost snapshot
- recovery 时 active path ahead cost
- robot pose 到 active local/global path 最近点距离
- controller internal critic/debug 输出

因此 Phase 24B 先实现 analysis-only post-recovery alignment analyzer，不新增 smoke。是否需要新 runtime recorder 留给 Phase 25/24C 决定。

## 3. TDD

新增测试：

```text
src/tugbot_maze/test/test_phase24b_post_recovery_alignment.py
```

测试内容：

1. synthetic post-recovery alignment contract
   - recovery time = 100
   - path updates at 101 / 108 / 112
   - near-zero starts at 110
   - 验证：
     - recovery 后 path update count
     - near-zero 前 path update count/time
     - near-zero 时仍有 path update
     - controller_received_path_but_cmd_near_zero
     - local-cost timeout snapshot carry
2. phase24 fixture regression
   - 对 `phase24_run1` artifacts 输出 post-recovery fields

RED 已确认：

```text
can't open file tools/analyze_post_recovery_alignment.py
```

## 4. 实现

新增：

```text
tools/analyze_post_recovery_alignment.py
```

用法：

```bash
python3 tools/analyze_post_recovery_alignment.py \
  --failure-windows log/phase24_run1_failure_windows.json \
  --nav2-analysis log/phase24_run1_goal_nav2_analysis.json \
  --controller-dynamics log/phase24_run1_controller_dynamics.jsonl \
  --launch-log log/phase24_run1_launch.log \
  --output-json log/phase24_run1_post_recovery_alignment.json
```

输出：

- stdout CSV
- JSON:
  - `summary`
  - `post_recovery_alignment`

关键字段：

- `last_recovery_time`
- `near_zero_cmd_start_time`
- `near_zero_after_last_recovery_sec`
- `path_updates_after_recovery`
- `path_updates_after_recovery_before_near_zero`
- `path_updates_while_cmd_near_zero`
- `first_path_update_after_recovery_time`
- `last_path_update_after_recovery_time`
- `last_path_update_before_near_zero_time`
- `path_update_to_near_zero_sec`
- `cmd_samples_after_recovery`
- `near_zero_cmd_samples_after_recovery`
- `controller_received_path_but_cmd_near_zero`
- local-cost timeout snapshot fields
- `available_diagnostics`

## 5. Verification

Focused tests：

```text
5 passed in 0.23s
```

Full tests：

```text
75 passed in 0.66s
```

Build：

```text
colcon build --packages-select tugbot_maze --symlink-install
Summary: 1 package finished
```

## 6. Existing artifact analysis

Generated：

```text
log/phase23_run1_post_recovery_alignment.json
log/phase24_run1_post_recovery_alignment.json
log/phase24b_post_recovery_alignment_aggregate.json
```

Inputs：

- `phase23_run1_failure_windows_after_rebuild.json`
- `phase23_run1_goal_nav2_analysis.json`
- `phase23_run1_controller_dynamics.jsonl`
- `phase23_run1_launch.log`
- `phase24_run1_failure_windows.json`
- `phase24_run1_goal_nav2_analysis.json`
- `phase24_run1_controller_dynamics.jsonl`
- `phase24_run1_launch.log`

## 7. Phase23 run1 result

Summary：

```json
{
  "row_count": 5,
  "with_recovery_time_count": 5,
  "with_path_updates_after_recovery_count": 4,
  "with_path_updates_before_near_zero_count": 3,
  "controller_received_path_but_cmd_near_zero_count": 4,
  "with_local_cost_timeout_snapshot_count": 4,
  "with_pre_post_recovery_local_cost_snapshots_count": 0,
  "with_robot_to_active_path_distance_count": 0
}
```

Per row：

```text
seq 2:
  path_updates_after_recovery: 5
  path_updates_before_near_zero: 3
  path_updates_while_near_zero: 2
  controller_received_path_but_cmd_near_zero: true
  near_zero_after_recovery: +3.22s
  footprint/path: 73 / 34

seq 4:
  path_updates_after_recovery: 0
  path_updates_before_near_zero: 0
  path_updates_while_near_zero: 3
  controller_received_path_but_cmd_near_zero: false
  near_zero_after_recovery: -2.61s
  footprint/path: 0 / 0

seq 8:
  path_updates_after_recovery: 4
  path_updates_before_near_zero: 0
  path_updates_while_near_zero: 9
  controller_received_path_but_cmd_near_zero: true
  near_zero_after_recovery: -5.81s
  footprint/path: 99 / 100

seq 11:
  path_updates_after_recovery: 19
  path_updates_before_near_zero: 10
  path_updates_while_near_zero: 9
  controller_received_path_but_cmd_near_zero: true
  near_zero_after_recovery: +16.17s
  footprint/path: 99 / 100

seq 12 non-timeout failure:
  path_updates_after_recovery: 17
  path_updates_before_near_zero: 16
  path_updates_while_near_zero: 1
  controller_received_path_but_cmd_near_zero: true
```

## 8. Phase24 run1 result

Summary：

```json
{
  "row_count": 4,
  "with_recovery_time_count": 4,
  "with_path_updates_after_recovery_count": 4,
  "with_path_updates_before_near_zero_count": 3,
  "controller_received_path_but_cmd_near_zero_count": 4,
  "with_local_cost_timeout_snapshot_count": 4,
  "with_pre_post_recovery_local_cost_snapshots_count": 0,
  "with_robot_to_active_path_distance_count": 0
}
```

Per row：

```text
seq 3:
  path_updates_after_recovery: 8
  path_updates_before_near_zero: 3
  path_updates_while_near_zero: 5
  controller_received_path_but_cmd_near_zero: true
  near_zero_after_recovery: +4.66s
  footprint/path: 99 / 100

seq 4:
  path_updates_after_recovery: 3
  path_updates_before_near_zero: 2
  path_updates_while_near_zero: 1
  controller_received_path_but_cmd_near_zero: true
  near_zero_after_recovery: +2.33s
  footprint/path: 99 / 63

seq 6:
  path_updates_after_recovery: 7
  path_updates_before_near_zero: 1
  path_updates_while_near_zero: 6
  controller_received_path_but_cmd_near_zero: true
  near_zero_after_recovery: +2.56s
  footprint/path: 0 / 0

seq 10:
  path_updates_after_recovery: 3
  path_updates_before_near_zero: 0
  path_updates_while_near_zero: 7
  controller_received_path_but_cmd_near_zero: true
  near_zero_after_recovery: -4.56s
  footprint/path: 99 / 100
```

## 9. Aggregate

Generated：

```text
log/phase24b_post_recovery_alignment_aggregate.json
```

Aggregate summary：

```json
{
  "run_count": 2,
  "row_count": 9,
  "with_recovery_time_count": 9,
  "with_path_updates_after_recovery_count": 8,
  "with_path_updates_before_near_zero_count": 6,
  "controller_received_path_but_cmd_near_zero_count": 8,
  "with_pre_post_recovery_local_cost_snapshots_count": 0,
  "with_robot_to_active_path_distance_count": 0
}
```

## 10. Interpretation

### 10.1 Existing logs already answer one key question

For 8/9 failure-window rows：

```text
controller received at least one Passing new path to controller after recovery
and near-zero cmd samples existed after recovery.
```

So the dominant failure is not simply：

```text
recovery happens and controller no longer receives path
```

It is more like：

```text
controller continues receiving paths after recovery,
but resulting cmd_vel remains near-zero or becomes near-zero.
```

### 10.2 Path updates before near-zero are common

For 6/9 rows：

```text
path_updates_after_recovery_before_near_zero > 0
```

Meaning：

```text
post-recovery, controller often receives new paths before near-zero onset.
```

This argues against a pure “no path refresh after clear-costmap” explanation.

### 10.3 Existing artifacts are insufficient for requested pre/post recovery cost/path snapshots

Current availability：

```text
with_pre_post_recovery_local_cost_snapshots_count: 0 / 9
with_robot_to_active_path_distance_count: 0 / 9
```

So Phase 24B cannot yet answer：

- recovery 前后 local cost/path 是否发生关键变化？
- recovery 时 active path ahead cost 是多少？
- robot pose 到 active local/global path 最近点距离是多少？

Need runtime recorder / instrumentation.

### 10.4 Unclassified case remains important

Phase24 seq 6：

```text
footprint/path timeout snapshot: 0 / 0
path_updates_after_recovery: 7
path_updates_before_near_zero: 1
path_updates_while_near_zero: 6
controller_received_path_but_cmd_near_zero: true
```

This is strong evidence that at least one late-silent timeout is not explained by timeout-side local-cost snapshot. It may require path/pose alignment or controller critic/debug state.

## 11. Phase 24B conclusion

Phase 24B implemented the post-recovery alignment analyzer and applied it to existing Phase23/24 artifacts.

Key conclusion：

```text
Most post-recovery late-silent failures continue to receive path updates.
The missing piece is not “no path after recovery”; it is why path updates still produce near-zero cmd.
```

The remaining unknowns require new runtime instrumentation, specifically:

```text
pre/post recovery local-cost snapshots
active path ahead cost at recovery time
robot-to-active-path distance
possibly controller critic/debug information
```

## 12. Recommendation for Phase 24C / Phase 25

Do not perform intervention yet.

Recommended Phase 24C：runtime post-recovery recorder.

Recorder should subscribe or derive：

- `/local_costmap/costmap`
- `/plan` or relevant Nav2 global/local plan topic if available
- `/odom`
- `/cmd_vel_nav`
- `/maze/goal_events`

Recorder should trigger around recovery/abort timestamps or continuously sample during active goal, then export per-goal snapshots:

- local cost/path snapshot immediately before clear-costmap recovery
- local cost/path snapshot immediately after clear-costmap recovery
- active path-ahead cost at recovery time
- robot pose to path nearest distance at recovery and near-zero onset
- path update count/time after recovery
- cmd near-zero interval while paths continue arriving

Only after this should Phase 25 choose interventions:

- footprint/path dominated -> local footprint/inflation/controller critic tuning
- post-recovery path-aligned but cmd-zero -> controller critic/debug or controller reset/recovery behavior
- path/pose misalignment -> goal/path refresh or recovery behavior
