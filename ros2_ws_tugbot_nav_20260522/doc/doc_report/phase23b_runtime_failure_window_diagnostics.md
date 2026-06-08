# Phase 23B Runtime Failure-Window Diagnostics Report

生成时间：2026-05-24

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 23B 新增 runtime diagnostics，不改导航行为。

核心问题：

```text
为什么 timeout 前最后 10 秒 controller 几乎不输出有效 cmd_vel，而 local costmap 显示 robot squeezed/high cost？
```

本阶段新增：

1. `/maze/goal_events` timeout local-cost diagnostics：
   - footprint cost max/mean/p95
   - footprint inflated/lethal cell counts
   - front wedge cost/clearance
   - left/right side cost/clearance
   - path ahead 0.5m / 1.0m cost max/mean
2. Nav2 log analyzer timestamps：
   - progress failure timestamps
   - clear costmap timestamps
   - controller abort timestamps
3. 真实 smoke：`phase23_run1`
4. failure-window analyzer 对齐 cmd/local-cost/Nav2 timing

仍不改：

- branch selection
- local-cost threshold control
- controller/Nav2 参数
- progress checker
- near-exit approach mode

## 2. TDD

新增测试：

- `src/tugbot_maze/test/test_phase23b_local_cost_runtime_diagnostics.py`
- `src/tugbot_maze/test/test_phase23b_nav2_event_timestamps.py`

RED 已确认：

```text
3 failed
- missing _local_cost_values_in_oriented_box / wedge / path ahead helpers
- missing new local-cost fields in summary/failure-window tools
- missing progress_failure_times in Nav2 analyzer output
```

## 3. 实现

修改：

- `src/tugbot_maze/tugbot_maze/maze_explorer.py`
- `tools/summarize_goal_event_local_costs.py`
- `tools/analyze_goal_events_with_nav2_log.py`
- `tools/analyze_failure_windows.py`
- `tools/run_phase21_controller_diagnostics_smoke.sh`

### 3.1 maze_explorer local-cost diagnostics

新增 timeout fields：

- `timeout_footprint_cost_max`
- `timeout_footprint_cost_mean`
- `timeout_footprint_cost_p95`
- `timeout_footprint_inflated_cell_count`
- `timeout_footprint_lethal_cell_count`
- `timeout_front_wedge_cost_max`
- `timeout_front_wedge_cost_mean`
- `timeout_front_wedge_clearance_m`
- `timeout_left_side_cost_max`
- `timeout_left_side_clearance_m`
- `timeout_right_side_cost_max`
- `timeout_right_side_clearance_m`
- `timeout_path_ahead_0_5m_cost_max`
- `timeout_path_ahead_0_5m_cost_mean`
- `timeout_path_ahead_1_0m_cost_max`
- `timeout_path_ahead_1_0m_cost_mean`

新增 helper：

- `_local_cost_values_in_oriented_box`
- `_local_cost_values_in_wedge`
- `_local_cost_path_ahead_values`
- `_update_cost_stats`
- `_cost_percentile`

### 3.2 Nav2 analyzer timestamps

`tools/analyze_goal_events_with_nav2_log.py` 新增 per-goal fields：

- `progress_failure_times`
- `clear_costmap_times`
- `controller_abort_times`
- `first_progress_failure_time`
- `last_progress_failure_time`
- `first_clear_costmap_time`
- `last_clear_costmap_time`
- `first_controller_abort_time`
- `last_controller_abort_time`

新增 summary fields：

- `timeout_with_controller_abort_count`
- `success_with_controller_abort_count`

### 3.3 failure-window analyzer

`tools/analyze_failure_windows.py` 现在能 carry：

- footprint stats
- front/side clearance
- path-ahead cost
- Nav2 event timestamps

## 4. Verification

通过：

```bash
python3 -m py_compile \
  src/tugbot_maze/tugbot_maze/maze_explorer.py \
  tools/summarize_goal_event_local_costs.py \
  tools/analyze_goal_events_with_nav2_log.py \
  tools/analyze_failure_windows.py
```

Focused tests：

```text
6 passed in 0.08s
```

Full tests：

```text
71 passed in 0.46s
```

Build：

```text
colcon build --packages-select tugbot_maze --symlink-install
Summary: 1 package finished
```

## 5. Smoke run

Preflight：无 ROS/Gazebo/Nav2/recorder 残留。

运行：

```bash
bash tools/run_phase21_controller_diagnostics_smoke.sh phase23_run1
```

Wrapper exit code：`0`

Artifacts：

- `log/phase23_run1_launch.log`
- `log/phase23_run1_goal_events.jsonl`
- `log/phase23_run1_explorer_state.jsonl`
- `log/phase23_run1_controller_dynamics.jsonl`
- `log/phase23_run1_goal_nav2_analysis.json`
- `log/phase23_run1_geometry_nav2_summary.json`
- `log/phase23_run1_goal_event_cost_summary.json`
- `log/phase23_run1_goal_controller_dynamics.json`
- `log/phase23_run1_failure_windows.json`

Note：第一次运行后发现 smoke 可能使用了 rebuild 前的 installed script；重新 `colcon build --symlink-install` 后 re-parse artifacts，已确认新 fields 存在，并生成：

- `log/phase23_run1_goal_event_cost_summary_after_rebuild.json`
- `log/phase23_run1_failure_windows_after_rebuild.json`

## 6. phase23_run1 summary

Final state：

```json
{
  "final_mode": "SETTLING",
  "goal_count": 12,
  "goal_success_count": 7,
  "goal_failure_count": 5,
  "timeout_cancel_count": 5,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "exit_distance_m": 1.5949985320600033
}
```

Nav2 summary：

```json
{
  "goal_count": 12,
  "success_count": 7,
  "timeout_count": 4,
  "timeout_with_progress_failure_count": 4,
  "timeout_with_recovery_count": 4,
  "timeout_with_controller_abort_count": 4,
  "success_with_progress_failure_count": 2,
  "success_with_recovery_count": 1,
  "success_with_controller_abort_count": 2
}
```

Controller summary：

```json
{
  "goal_count": 12,
  "healthy_motion_but_late_stall_count": 4,
  "late_controller_silent_count": 4,
  "healthy_motion_count": 7,
  "slow_progress_count": 1
}
```

## 7. Failure-window diagnostics

`log/phase23_run1_failure_windows_after_rebuild.json` summary：

```json
{
  "failure_window_count": 5,
  "healthy_motion_but_late_stall_count": 4,
  "late_controller_silent_count": 4,
  "squeezed_count": 4,
  "high_timeout_robot_cost_count": 4,
  "with_footprint_cost_stats_count": 5,
  "with_path_ahead_cost_count": 5,
  "with_nav2_event_times_count": 5
}
```

There are 4 timeout late-stall windows plus 1 non-timeout failure row.

### Timeout seq 2

```text
classification: healthy_motion_but_late_stall
cmd_linear_abs_mean: 0.004515
cmd_linear_abs_max: 0.016292
near_zero_duration: 2.75s
robot_cost_max: 100
footprint_cost_max: 73
footprint_mean: 25.36
footprint_inflated/lethal: 6 / 0
front_wedge_cost_max: 99
front_clearance: 0.448m
left_cost_max: 0, left_clearance: 0.45m
right_cost_max: 100, right_clearance: 0.0m
path_ahead_0.5m_max: 34
path_ahead_1.0m_max: 34
first_progress_failure_time: 1779670534.405641
last_controller_abort_time: 1779670548.576591
near_zero starts 17.41s after first progress failure
```

Interpretation：right side blocked/high cost, front has high-cost cell but path ahead low; near-zero happens after progress failures/aborts.

### Timeout seq 4

```text
cmd_mean: 0.003549
robot_cost_max: 97
footprint_cost_max: 0
front_wedge_cost_max: 0
path ahead max: 0 / 0
right_cost_max: 99, right_clearance: 0.158m
near-zero starts 7.52s after first progress failure
```

Interpretation：diagnostics show side high cost but footprint/front/path clear. This may be a costmap/pose/path timing mismatch or controller/internal infeasibility not captured by coarse front/path probes.

### Timeout seq 8

```text
cmd_mean: 0.004443
robot_cost_max: 100
footprint_cost_max: 99
footprint_mean: 56.69
footprint_inflated/lethal: 41 / 20
front_wedge_cost_max: 100
front_clearance: 0.128m
left_cost_max: 100
right_cost_max: 99
path_ahead_0.5m_max: 99
path_ahead_1.0m_max: 100
near-zero starts 5.79s before first progress failure
```

Interpretation：strongly blocked / inflated footprint and path-ahead obstruction. Near-zero precedes progress failure.

### Timeout seq 11

```text
cmd_mean: 0.003035
robot_cost_max: 100
footprint_cost_max: 99
footprint_mean: 82.73
footprint_inflated/lethal: 89 / 63
front_wedge_cost_max: 100
front_clearance: 0.009m
left/right cost_max: 100 / 100
path_ahead_0.5m_max: 100
path_ahead_1.0m_max: 100
near-zero starts 37.17s after first progress failure
```

Interpretation：severe local blockage/footprint inflation/path obstruction. Near-zero happens after earlier progress/recovery/abort cycles.

## 8. Main findings

Phase 23B confirms 4/4 timeout windows in phase23_run1 are still：

```text
healthy_motion_but_late_stall + late_controller_silent
```

and all 4 have：

```text
squeezed=true
high timeout robot cost
Nav2 progress/recovery/controller_abort events
```

But the deeper diagnostics split them into at least two subtypes：

### Type A：severe footprint/path obstruction

- seq 8
- seq 11

Signs：

- footprint cost max 99
- lethal footprint cells present
- front wedge max 100
- path ahead max 99-100
- side costs high

This looks like controller near-silence caused by local costmap/footprint/path infeasibility.

### Type B：side-cost / timing / unclear local obstruction

- seq 2
- seq 4

Signs：

- robot cost max high
- one side high cost
- footprint/path ahead may be low or only mildly inflated
- near-zero often starts after progress failure/abort cycles

This is not explained by simple path-ahead obstruction. Need more precise alignment with controller internals or local path footprint over time.

## 9. Caveats

1. phase23_run1 final mode was `SETTLING`, not terminal `EXIT_REACHED` or `FAILED_EXHAUSTED`.
   - Wrapper still exited 0 because state recorder completed its sentinel/timeout behavior.
   - Artifacts are valid for failure-window diagnostics, but final run-level success/failure should be interpreted cautiously.

2. New local-cost fields were visible after rebuild/reparse.
   - For future smokes, ensure `colcon build --symlink-install` is done before run.

3. `front_side_clearance` availability is now true in the failure-window analyzer after updating field mapping.

## 10. Phase 24 recommendation

Do not branch score yet.

Do not tune controller globally yet.

Recommended Phase 24A：classify timeout subtypes from Phase 23B diagnostics and add one more focused smoke if needed.

Suggested taxonomy：

- `footprint_path_blocked_late_silent`
  - high footprint cost/lethal count
  - high front/path-ahead cost
  - late controller silent
- `side_cost_or_timing_late_silent`
  - high robot/side cost but low footprint/path-ahead cost
  - near-zero begins after progress/recovery/abort
- `cmd_silent_before_progress_failure`
  - near-zero begins before first progress failure
- `cmd_silent_after_recovery_abort`
  - near-zero begins after abort/recovery cycles

Only after subtype stability is clear should Phase 24/25 consider targeted interventions：

- local costmap footprint/inflation parameters
- controller critic/local planner behavior near inflated cells
- goal tolerance/timeout policy
- possibly recovery behavior

Still do not use dispatch-time branch scoring as the primary fix.
