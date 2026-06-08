# Phase 24A Timeout Subtype Classifier + Focused Validation

生成时间：2026-05-25

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 24A 目标是把 Phase 23B 的 runtime diagnostics 标准化成 timeout subtype taxonomy，并做 focused validation。

新增 taxonomy：

```text
footprint_path_blocked_late_silent
side_cost_or_timing_late_silent
cmd_silent_before_progress_failure
cmd_silent_after_recovery_abort
```

仍不做：

- branch scoring
- local-cost threshold branch control
- controller/Nav2 全局调参
- progress checker 粗调
- near-exit approach mode

## 2. TDD

新增测试：

```text
src/tugbot_maze/test/test_phase24a_timeout_subtypes.py
```

测试包含：

1. synthetic classifier contract
   - severe footprint/path blocked + near-zero before progress failure
   - side-cost/timing late silent + near-zero after progress failure
   - non-timeout failure 不参与 timeout subtype
2. Phase 23B fixture regression
   - phase23_run1 seq 8/11 => `footprint_path_blocked_late_silent`
   - phase23_run1 seq 2/4 => `side_cost_or_timing_late_silent`
   - phase23_run1 seq 8 => `cmd_silent_before_progress_failure`
   - phase23_run1 seq 2/4/11 => `cmd_silent_after_recovery_abort`

RED 已确认：

```text
can't open file tools/analyze_timeout_subtypes.py
```

## 3. 实现

新增：

```text
tools/analyze_timeout_subtypes.py
```

用法：

```bash
python3 tools/analyze_timeout_subtypes.py \
  --failure-windows log/phase23_run1_failure_windows_after_rebuild.json \
  --output-json log/phase23_run1_timeout_subtypes.json
```

输出：

- stdout CSV
- JSON:
  - `summary`
  - `timeout_subtypes`

### 3.1 Controller subtype rules

`footprint_path_blocked_late_silent`：

```text
footprint_cost_max >= 90
and (
  footprint_lethal_cell_count > 0
  or footprint_cost_p95 >= 90
  or footprint_inflated_cell_count >= 10
)
and (
  path_ahead_0.5m_cost_max >= 90
  or path_ahead_1.0m_cost_max >= 90
  or front_wedge_cost_max >= 90
)
```

`side_cost_or_timing_late_silent`：

```text
not footprint_path_blocked
and (
  timeout_robot_local_cost_max >= 90
  or left_side_cost_max >= 90
  or right_side_cost_max >= 90
  or squeezed == true
)
```

`unclassified_late_silent`：

```text
late silent timeout, but neither severe footprint/path nor side/high-cost signal is present
```

### 3.2 Timing subtype rules

`cmd_silent_before_progress_failure`：

```text
near_zero_cmd_to_first_progress_failure_sec < 0
```

`cmd_silent_after_recovery_abort`：

```text
near_zero_cmd_to_first_progress_failure_sec >= 0
```

`cmd_silent_timing_unknown`：

```text
Nav2 timestamp / near-zero timestamp missing
```

## 4. Verification

Focused：

```text
6 passed in 0.14s
```

Full tests：

```text
73 passed in 0.53s
```

Build：

```text
colcon build --packages-select tugbot_maze --symlink-install
Summary: 1 package finished
```

## 5. Existing Phase 23B validation

Generated：

```text
log/phase23_run1_timeout_subtypes.json
```

Summary：

```json
{
  "timeout_count": 4,
  "controller_subtype_counts": {
    "footprint_path_blocked_late_silent": 2,
    "side_cost_or_timing_late_silent": 2
  },
  "timing_subtype_counts": {
    "cmd_silent_after_recovery_abort": 3,
    "cmd_silent_before_progress_failure": 1
  },
  "severe_footprint_path_ratio": 0.5,
  "unclassified_count": 0
}
```

Per goal：

```text
seq 2  -> side_cost_or_timing_late_silent + cmd_silent_after_recovery_abort
seq 4  -> side_cost_or_timing_late_silent + cmd_silent_after_recovery_abort
seq 8  -> footprint_path_blocked_late_silent + cmd_silent_before_progress_failure
seq 11 -> footprint_path_blocked_late_silent + cmd_silent_after_recovery_abort
```

Because sample size was only 4 and severe ratio was exactly 0.5, Phase 24A ran one additional focused smoke.

## 6. Focused smoke: phase24_run1

Preflight：无 ROS/Gazebo/Nav2/recorder 残留。

Before run：

```bash
colcon build --packages-select tugbot_maze --symlink-install
```

Run：

```bash
bash tools/run_phase21_controller_diagnostics_smoke.sh phase24_run1
```

Wrapper exit code：`0`

Run summary：

```json
{
  "final_mode": "FAILED_EXHAUSTED",
  "goal_count": 12,
  "goal_success_count": 8,
  "goal_failure_count": 4,
  "timeout_cancel_count": 4,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "exit_distance_m": 1.0559290302133728
}
```

Nav2 summary：

```json
{
  "timeout_count": 4,
  "timeout_with_progress_failure_count": 4,
  "timeout_with_recovery_count": 4,
  "timeout_with_controller_abort_count": 4,
  "success_with_progress_failure_count": 3,
  "success_with_recovery_count": 3,
  "success_with_controller_abort_count": 3
}
```

Controller summary：

```json
{
  "healthy_motion_but_late_stall_count": 3,
  "healthy_motion_but_timed_out_count": 1,
  "late_controller_silent_count": 3,
  "healthy_motion_count": 6,
  "slow_progress_count": 1
}
```

Generated：

```text
log/phase24_run1_failure_windows.json
log/phase24_run1_timeout_subtypes.json
```

Phase 24A classifier includes only late-silent timeout windows, so phase24_run1 subtype count is 3.

Summary：

```json
{
  "timeout_count": 3,
  "controller_subtype_counts": {
    "footprint_path_blocked_late_silent": 2,
    "unclassified_late_silent": 1
  },
  "timing_subtype_counts": {
    "cmd_silent_after_recovery_abort": 3
  },
  "severe_footprint_path_ratio": 0.666667,
  "unclassified_count": 1
}
```

Per goal：

```text
seq 3  -> footprint_path_blocked_late_silent + cmd_silent_after_recovery_abort
seq 6  -> unclassified_late_silent + cmd_silent_after_recovery_abort
seq 10 -> footprint_path_blocked_late_silent + cmd_silent_after_recovery_abort
```

Notable：

- phase24 seq 6 had:
  - footprint/path/front costs all low
  - side cost only 63
  - robot cost max 40
  - still late silent after recovery/abort
- This becomes a new “unclassified late-silent” case, likely controller/recovery timing or diagnostics still missing.

## 7. Aggregate: Phase 23B + Phase 24A smoke

Generated：

```text
log/phase24a_timeout_subtypes_aggregate.json
```

Aggregate summary：

```json
{
  "run_count": 2,
  "timeout_subtype_count": 7,
  "controller_subtype_counts": {
    "footprint_path_blocked_late_silent": 4,
    "side_cost_or_timing_late_silent": 2,
    "unclassified_late_silent": 1
  },
  "timing_subtype_counts": {
    "cmd_silent_after_recovery_abort": 6,
    "cmd_silent_before_progress_failure": 1
  },
  "combined_subtype_counts": {
    "footprint_path_blocked_late_silent+cmd_silent_after_recovery_abort": 3,
    "footprint_path_blocked_late_silent+cmd_silent_before_progress_failure": 1,
    "side_cost_or_timing_late_silent+cmd_silent_after_recovery_abort": 2,
    "unclassified_late_silent+cmd_silent_after_recovery_abort": 1
  },
  "severe_footprint_path_ratio": 0.571429
}
```

## 8. Interpretation

### 8.1 Severe footprint/path obstruction is the plurality, not exclusive majority

Across 7 late-silent timeout windows：

```text
footprint_path_blocked_late_silent: 4 / 7 = 57.1%
```

This is the largest category, but not overwhelming enough to justify one global parameter change yet.

### 8.2 Most near-zero starts after progress/recovery/abort cycles

```text
cmd_silent_after_recovery_abort: 6 / 7
cmd_silent_before_progress_failure: 1 / 7
```

This suggests the dominant pattern is not always：

```text
controller silently gives up first -> progress checker later detects it
```

More often it is：

```text
progress failure / recovery / abort cycle happens first -> later cmd becomes near-zero
```

So Phase 24B should inspect post-recovery controller state / path validity / local costmap reset effects, not only pre-failure local obstruction.

### 8.3 There is one truly unclassified late-silent timeout

phase24_run1 seq 6：

```text
unclassified_late_silent + cmd_silent_after_recovery_abort
```

Its current probes did not show severe footprint/path/side/robot high cost. This means one of the following:

- controller internal critic state is rejecting commands for reasons not represented by our coarse probes
- local/global path relationship over time is missing
- sampling at timeout misses earlier local-cost state
- recovery/abort timing changes the relevant local-cost state
- there may be an action/controller lifecycle or path-update issue

## 9. Phase 24A conclusion

Phase 24A achieved the goal: timeout subtype taxonomy is standardized and validated on Phase 23B + one focused smoke.

Evidence now supports this statement:

```text
Late-silent timeouts are not one homogeneous failure mode.
```

Observed subtypes:

1. `footprint_path_blocked_late_silent`
   - 4/7
   - strongest candidate for local costmap footprint/inflation/controller critic intervention
2. `side_cost_or_timing_late_silent`
   - 2/7
   - needs side-clearance / footprint alignment / controller path relation analysis
3. `unclassified_late_silent`
   - 1/7
   - requires deeper controller/path/recovery timing diagnostics

Timing:

- 6/7 are `cmd_silent_after_recovery_abort`
- 1/7 is `cmd_silent_before_progress_failure`

So intervention should be targeted around post-recovery/post-abort behavior and local-path feasibility, not branch scoring.

## 10. Recommendation for Phase 24B / 25

Do not change branch selection.

Do not globally tune progress checker yet.

Recommended Phase 24B：post-recovery path/local-cost alignment diagnostics.

Add diagnostics around recovery/abort cycles：

- local cost/path snapshot immediately before and after clear-costmap recovery
- active local/global path ahead cost at recovery time
- robot pose vs local path nearest point distance
- whether controller receives a path after recovery but emits near-zero cmd
- count/time of `Passing new path to controller` after recovery before near-zero onset

For Phase 25 intervention candidates, only after Phase 24B confirms subtype stability:

- if `footprint_path_blocked_late_silent` dominates：consider local footprint/inflation/controller critic tuning
- if `cmd_silent_after_recovery_abort` dominates：consider recovery behavior / post-clear path refresh / controller reset handling
- if `unclassified_late_silent` repeats：instrument controller critic/debug output or path/pose relation more deeply
