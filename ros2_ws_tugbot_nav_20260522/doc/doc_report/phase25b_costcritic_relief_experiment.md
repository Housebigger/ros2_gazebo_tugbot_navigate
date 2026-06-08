# Phase 25B MPPI CostCritic Narrow Experiment

生成时间：2026-05-25

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 25B 目标：controller-side single-family experiment，只测试 MPPI `CostCritic` 如何解释 high-cost cells。

背景：Phase 25A 缩小 / 加快衰减 local inflation 后失败：

```text
footprint_path_blocked_late_silent: 2 -> 3
timeouts: 4 -> 5
successes: 8 -> 6
```

因此 Phase 25B 不继续改 local inflation，而改 controller-side CostCritic。

实验原则：

```text
one parameter family only
separate profile
baseline vs experiment
same acceptance gates
```

## 2. Discovery

Baseline MPPI CostCritic：

```yaml
FollowPath:
  CostCritic:
    enabled: true
    cost_power: 1
    cost_weight: 3.81
    critical_cost: 300.0
    consider_footprint: false
    collision_cost: 1000000.0
    near_goal_distance: 1.0
    trajectory_point_step: 2
```

Phase 25B 选择只 modestly lower `cost_weight`。

## 3. TDD

新增测试：

```text
src/tugbot_maze/test/test_phase25b_costcritic_relief_experiment.py
```

RED 已确认：

```text
3 failed
- Phase25B CostCritic params missing
- launch/wrapper missing phase25b profile
- compare_phase25b_metrics.py missing
```

## 4. 实现

新增 profile：

```text
src/tugbot_navigation/config/nav2_slam_phase25b_costcritic_relief_params.yaml
```

该文件复制 baseline params，只改：

```yaml
FollowPath:
  CostCritic:
    cost_weight: 2.5
```

Baseline：

```yaml
cost_weight: 3.81
```

未改：

```text
branch selection
progress checker
goal timeout
local costmap inflation
recovery behavior
other controller critics
trajectory_point_step
critical_cost
collision_cost
footprint radius
```

Launch integration：

```text
src/tugbot_bringup/launch/tugbot_maze_explore.launch.py
```

新增：

```text
phase25b_costcritic_relief_profile
```

Wrapper integration：

```text
tools/run_phase21_controller_diagnostics_smoke.sh
```

新增：

```text
phase25b_runN
PHASE25B_PROFILE=true only when run id matches phase25b_runN
phase25b_costcritic_relief_profile:="$PHASE25B_PROFILE"
```

Comparator：

```text
tools/compare_phase25b_metrics.py
```

Acceptance gates 与 Phase 25A 相同。

## 5. Verification

Focused：

```text
6 passed in 0.07s
```

Full tests：

```text
87 passed in 0.81s
```

Build：

```text
colcon build --packages-select tugbot_navigation tugbot_bringup tugbot_maze --symlink-install
Summary: 3 packages finished
```

## 6. Experiment smoke：phase25b_run1

Preflight：无真实 ROS/Gazebo/Nav2/recorder 残留。

Run：

```bash
bash tools/run_phase21_controller_diagnostics_smoke.sh phase25b_run1
```

Wrapper exit code：`0`

Run summary：

```json
{
  "final_mode": "FAILED_EXHAUSTED",
  "goal_count": 12,
  "goal_success_count": 9,
  "goal_failure_count": 3,
  "timeout_cancel_count": 3,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "exit_distance_m": 1.7342131462699826
}
```

Nav2 summary：

```json
{
  "goal_count": 12,
  "success_count": 9,
  "timeout_count": 3,
  "timeout_with_progress_failure_count": 3,
  "timeout_with_recovery_count": 3,
  "timeout_with_controller_abort_count": 3,
  "success_with_progress_failure_count": 3,
  "success_with_recovery_count": 3,
  "success_with_controller_abort_count": 3
}
```

Controller summary：

```json
{
  "goal_count": 11,
  "healthy_motion_but_late_stall_count": 3,
  "late_controller_silent_count": 3,
  "timeout_or_failure_late_stall_count": 3,
  "healthy_motion_count": 8
}
```

## 7. Generated artifacts

```text
log/phase25b_run1_failure_windows.json
log/phase25b_run1_timeout_subtypes.json
log/phase25b_run1_post_recovery_enriched.json
log/phase25b_run1_phase25b_compare_input.json
log/phase25b_run1_acceptance_vs_phase24c_run2.json
```

Baseline：

```text
log/phase24c_run2_phase25b_compare_input.json
```

## 8. Timeout subtype result

`phase25b_run1_timeout_subtypes.json`：

```json
{
  "timeout_count": 3,
  "controller_subtype_counts": {
    "side_cost_or_timing_late_silent": 3
  },
  "timing_subtype_counts": {
    "cmd_silent_after_recovery_abort": 3
  },
  "severe_footprint_path_count": 0,
  "severe_footprint_path_ratio": 0.0,
  "unclassified_count": 0
}
```

关键：

```text
footprint_path_blocked_late_silent: 0
```

## 9. Enrichment coverage

`phase25b_run1_post_recovery_enriched.json`：

```json
{
  "recovery_count": 12,
  "with_pre_recovery_count": 12,
  "with_post_recovery_count": 12,
  "with_near_zero_count": 12,
  "sufficient_density_count": 12,
  "needs_periodic_snapshot_count": 0,
  "controller_received_path_but_cmd_near_zero_count": 0
}
```

Recorder/enrichment coverage OK。

## 10. Acceptance comparison

Baseline：`phase24c_run2`

```json
{
  "final_mode": "FAILED_EXHAUSTED",
  "goal_success_count": 8,
  "timeout_cancel_count": 4,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "footprint_path_blocked_late_silent": 2
}
```

Experiment：`phase25b_run1`

```json
{
  "final_mode": "FAILED_EXHAUSTED",
  "goal_success_count": 9,
  "timeout_cancel_count": 3,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "footprint_path_blocked_late_silent": 0
}
```

Comparator result：

```json
{
  "accepted": true
}
```

All acceptance gates passed：

```text
footprint_path_blocked_reduced: 2 -> 0
timeouts_reduced: 4 -> 3
blocked_branch_no_regression: 0 -> 0
blacklisted_goal_no_regression: 0 -> 0
success_no_regression: 8 -> 9
exit_behavior_preserved_or_improved: FAILED_EXHAUSTED -> FAILED_EXHAUSTED
```

## 11. Conclusion

Phase 25B is accepted by the predefined metrics for this single smoke comparison.

Most important result：

```text
footprint_path_blocked_late_silent eliminated in phase25b_run1: 2 -> 0
```

Also improved：

```text
timeouts: 4 -> 3
successes: 8 -> 9
```

No safety regression：

```text
blocked_branch_count: 0
blacklisted_goal_count: 0
```

Caveat：final mode remains `FAILED_EXHAUSTED` and exit distance worsened numerically：

```text
baseline exit_distance_m: 0.7568
phase25b exit_distance_m: 1.7342
```

The predefined exit behavior gate only checks terminal mode rank, so Phase25B passes. But because exit distance worsened, this should be treated as “accepted for repeat validation”, not immediately merged as default.

## 12. Recommendation

Do not merge into baseline yet after one run.

Recommended Phase 25C：repeat validation for Phase25B CostCritic profile.

Run at least one additional smoke：

```text
phase25b_run2
```

Use the same analysis pipeline and compare against both：

```text
phase24c_run2 baseline
phase25b_run1 prior experiment
```

Add an additional advisory metric, not necessarily hard gate yet：

```text
exit_distance_m should not worsen materially
```

If repeated run preserves：

```text
footprint_path_blocked_late_silent <= baseline
timeouts <= baseline
successes >= baseline
blocked/blacklist = 0
exit distance acceptable or exit reached
```

then consider promoting the CostCritic profile or trying a smaller delta between 3.81 and 2.5.
