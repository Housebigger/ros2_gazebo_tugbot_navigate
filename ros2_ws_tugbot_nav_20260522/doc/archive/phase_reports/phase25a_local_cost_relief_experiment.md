# Phase 25A Local Path-Cost / Footprint-Cost Relief Experiment

生成时间：2026-05-25

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 25A 目标：只针对 `footprint_path_blocked_late_silent` 做一个窄范围、可回退的 local path-cost / footprint-cost relief experiment。

实验原则：

```text
baseline vs one local-cost/controller-related parameter set
一次只改一组小参数
可回退
必须用 metrics 判断
```

Acceptance metrics：

```text
footprint_path_blocked_late_silent count 下降
timeout count 下降
blocked_branch_count 不回归
blacklisted_goal_count 不回归
success count 不下降
exit-reaching behavior 保持或改善
```

## 2. Discovery

当前 Nav2 live-SLAM params：

```text
src/tugbot_navigation/config/nav2_slam_params.yaml
```

相关 baseline local costmap settings：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.35
      inflation_layer:
        cost_scaling_factor: 3.0
        inflation_radius: 0.70
```

MPPI cost critic baseline：

```yaml
FollowPath:
  CostCritic:
    cost_weight: 3.81
    critical_cost: 300.0
    consider_footprint: false
```

本阶段选择只改 local_costmap inflation，避免同时混入 controller critic / progress checker / branch selection。

## 3. TDD

新增测试：

```text
src/tugbot_maze/test/test_phase25a_local_cost_relief_experiment.py
```

RED 已确认：

```text
3 failed
- Phase25A overlay params missing
- launch/wrapper missing phase25a profile
- compare_phase25a_metrics.py missing
```

GREEN 后通过。

## 4. 实现

新增可回退 profile：

```text
src/tugbot_navigation/config/nav2_slam_phase25a_local_cost_relief_params.yaml
```

该文件复制 baseline params，只改 local costmap inflation：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      inflation_layer:
        cost_scaling_factor: 4.5
        inflation_radius: 0.55
```

Baseline 是：

```yaml
cost_scaling_factor: 3.0
inflation_radius: 0.70
```

解释：这是一个更窄、更快衰减的 local inflation relief 试验，目标是减少狭窄走廊中的 high local path-ahead cost / footprint blocked late-silent。

未改：

```text
branch selection
progress checker
goal timeout
controller critics
recovery behavior
footprint radius
```

修改 launch：

```text
src/tugbot_bringup/launch/tugbot_maze_explore.launch.py
```

新增：

```text
phase25a_local_cost_relief_profile
```

当该 launch arg 为 true 时使用：

```text
nav2_slam_phase25a_local_cost_relief_params.yaml
```

修改 wrapper：

```text
tools/run_phase21_controller_diagnostics_smoke.sh
```

新增：

```text
phase25a_runN
```

当 run id 为 `phase25a_runN` 时：

```text
PHASE25A_PROFILE=true
phase25a_local_cost_relief_profile:="$PHASE25A_PROFILE"
```

新增 comparator：

```text
tools/compare_phase25a_metrics.py
```

输入：

```text
baseline compare input JSON
experiment compare input JSON
```

输出：

```text
accepted
checks
baseline_summary
experiment_summary
```

## 5. Verification

Focused：

```text
6 passed in 0.10s
```

Full tests：

```text
84 passed in 0.80s
```

Build：

```text
colcon build --packages-select tugbot_navigation tugbot_bringup tugbot_maze --symlink-install
Summary: 3 packages finished
```

## 6. Experiment smoke：phase25a_run1

Preflight：无真实 ROS/Gazebo/Nav2/recorder 残留。

Run：

```bash
bash tools/run_phase21_controller_diagnostics_smoke.sh phase25a_run1
```

Wrapper exit code：`0`

Run summary：

```json
{
  "final_mode": "EXIT_REACHED",
  "goal_count": 12,
  "goal_success_count": 6,
  "goal_failure_count": 5,
  "timeout_cancel_count": 5,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "exit_distance_m": 0.5638289311085901
}
```

Nav2 summary：

```json
{
  "goal_count": 12,
  "success_count": 6,
  "timeout_count": 5,
  "timeout_with_progress_failure_count": 5,
  "timeout_with_recovery_count": 5,
  "timeout_with_controller_abort_count": 5,
  "success_with_progress_failure_count": 2,
  "success_with_recovery_count": 2,
  "success_with_controller_abort_count": 2
}
```

Controller summary：

```json
{
  "goal_count": 12,
  "healthy_motion_but_late_stall_count": 4,
  "healthy_motion_but_timed_out_count": 1,
  "late_controller_silent_count": 4,
  "timeout_or_failure_late_stall_count": 4,
  "terminal_cancel_after_exit_count": 1
}
```

## 7. Generated analysis artifacts

```text
log/phase25a_run1_failure_windows.json
log/phase25a_run1_timeout_subtypes.json
log/phase25a_run1_post_recovery_enriched.json
log/phase25a_run1_phase25a_compare_input.json
log/phase25a_run1_acceptance_vs_phase24c_run2.json
```

Baseline compare input regenerated from：

```text
log/phase24c_run2_phase25a_compare_input.json
```

## 8. Timeout subtype result

`phase25a_run1_timeout_subtypes.json`：

```json
{
  "timeout_count": 4,
  "controller_subtype_counts": {
    "footprint_path_blocked_late_silent": 3,
    "side_cost_or_timing_late_silent": 1
  },
  "timing_subtype_counts": {
    "cmd_silent_after_recovery_abort": 3,
    "cmd_silent_before_progress_failure": 1
  },
  "severe_footprint_path_ratio": 0.75,
  "unclassified_count": 0
}
```

Note：subtype analyzer 只统计 late-silent timeout rows；run summary timeout_cancel_count 是 5，其中一个是 `healthy_motion_but_timed_out`。

## 9. Enrichment coverage

`phase25a_run1_post_recovery_enriched.json`：

```json
{
  "recovery_count": 17,
  "with_pre_recovery_count": 17,
  "with_post_recovery_count": 17,
  "with_near_zero_count": 17,
  "sufficient_density_count": 17,
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

Experiment：`phase25a_run1`

```json
{
  "final_mode": "EXIT_REACHED",
  "goal_success_count": 6,
  "timeout_cancel_count": 5,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "footprint_path_blocked_late_silent": 3
}
```

Comparator result：

```json
{
  "accepted": false
}
```

Passed：

```text
blocked_branch_no_regression: true
blacklisted_goal_no_regression: true
exit_behavior_preserved_or_improved: true
```

Failed：

```text
footprint_path_blocked_reduced: false
  baseline 2 -> experiment 3

timeouts_reduced: false
  baseline 4 -> experiment 5

success_no_regression: false
  baseline 8 -> experiment 6
```

## 11. Conclusion

Phase 25A local inflation relief profile is rejected by the predefined metrics.

Although `phase25a_run1` reached exit, it regressed the target subtype and core run metrics：

```text
footprint_path_blocked_late_silent increased 2 -> 3
timeout_cancel_count increased 4 -> 5
goal_success_count decreased 8 -> 6
```

Blocked/blacklist stayed safe：

```text
blocked_branch_count: 0 -> 0
blacklisted_goal_count: 0 -> 0
```

But the experiment failed the primary target and should not be accepted as default.

## 12. Recommendation

Revert/keep disabled the Phase 25A profile. It is already isolated behind:

```text
phase25a_local_cost_relief_profile:=true
```

Do not merge it into baseline.

Do not branch score.

Do not globally tune progress checker.

Recommended next step：Phase 25B, a different single-family intervention, likely controller-side rather than inflation-side.

Candidate：

```text
MPPI CostCritic narrow experiment
```

Rationale：Phase 25A reducing local inflation radius made target subtype worse, suggesting the issue is not solved by making local inflation less conservative. Next experiment should test controller interpretation of high cost rather than shrinking the cost field.

Example narrow candidate for Phase 25B：

```yaml
FollowPath:
  CostCritic:
    cost_weight: lower modestly OR trajectory_point_step changed modestly
```

But it must follow the same rules：

```text
one parameter family only
separate profile
baseline vs experiment
same acceptance gates
```
