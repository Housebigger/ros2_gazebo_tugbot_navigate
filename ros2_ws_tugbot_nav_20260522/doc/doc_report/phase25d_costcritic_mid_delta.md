# Phase 25D Smaller MPPI CostCritic Delta Experiment

生成时间：2026-05-25

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 25D 目标：测试更小 MPPI CostCritic delta。

配置：

```yaml
FollowPath:
  CostCritic:
    cost_weight: 3.0
```

理由：

```text
2.5 重复消除了 footprint_path_blocked_late_silent，并改善 timeouts/successes；
但 2.5 让 exit_distance_m 比 baseline 差。

3.0 介于 baseline 3.81 和 Phase25B 2.5 之间，可能保留 subtype/timeouts 收益，同时降低 exit-distance regression。
```

继续遵守：

```text
one parameter family only
separate profile
baseline vs experiment
same hard gates
exit_distance advisory
```

## 2. TDD

新增测试：

```text
src/tugbot_maze/test/test_phase25d_costcritic_mid_experiment.py
```

RED 已确认：

```text
3 failed
- Phase25D CostCritic mid-delta params missing
- launch/wrapper missing phase25d profile
- compare_phase25d_metrics.py missing
```

## 3. 实现

新增 profile：

```text
src/tugbot_navigation/config/nav2_slam_phase25d_costcritic_mid_params.yaml
```

该文件复制 baseline `nav2_slam_params.yaml`，只改：

```yaml
FollowPath:
  CostCritic:
    cost_weight: 3.0
```

保持不变：

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
phase25d_costcritic_mid_profile
```

Wrapper integration：

```text
tools/run_phase21_controller_diagnostics_smoke.sh
```

新增：

```text
phase25d_runN
PHASE25D_PROFILE=true only when run id matches phase25d_runN
phase25d_costcritic_mid_profile:="$PHASE25D_PROFILE"
```

Comparator：

```text
tools/compare_phase25d_metrics.py
```

它复用 Phase25B hard gates + exit_distance advisory。

## 4. Verification

Focused：

```text
5 passed in 0.10s
```

Full tests：

```text
92 passed in 0.91s
```

Build：

```text
colcon build --packages-select tugbot_navigation tugbot_bringup tugbot_maze --symlink-install
Summary: 3 packages finished
```

## 5. Smoke：phase25d_run1

Preflight：无真实 ROS/Gazebo/Nav2/recorder 残留。

Run：

```bash
bash tools/run_phase21_controller_diagnostics_smoke.sh phase25d_run1
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
  "exit_distance_m": 0.8055346792124355
}
```

Nav2 summary：

```json
{
  "goal_count": 12,
  "success_count": 8,
  "timeout_count": 4,
  "timeout_with_progress_failure_count": 4,
  "timeout_with_recovery_count": 4,
  "timeout_with_controller_abort_count": 4,
  "success_with_progress_failure_count": 1,
  "success_with_recovery_count": 1,
  "success_with_controller_abort_count": 1
}
```

Controller summary：

```json
{
  "goal_count": 12,
  "healthy_motion_but_late_stall_count": 4,
  "late_controller_silent_count": 4,
  "timeout_or_failure_late_stall_count": 4,
  "healthy_motion_count": 7,
  "slow_progress_count": 1
}
```

## 6. Generated artifacts

```text
log/phase25d_run1_failure_windows.json
log/phase25d_run1_timeout_subtypes.json
log/phase25d_run1_post_recovery_enriched.json
log/phase25d_run1_phase25d_compare_input.json
log/phase25d_run1_acceptance_vs_phase24c_run2.json
log/phase25d_run1_acceptance_vs_phase25b_run1.json
log/phase25d_run1_acceptance_vs_phase25b_run2.json
```

## 7. Timeout subtype result

`phase25d_run1_timeout_subtypes.json`：

```json
{
  "timeout_count": 4,
  "controller_subtype_counts": {
    "footprint_path_blocked_late_silent": 2,
    "side_cost_or_timing_late_silent": 2
  },
  "timing_subtype_counts": {
    "cmd_silent_after_recovery_abort": 4
  },
  "severe_footprint_path_count": 2,
  "severe_footprint_path_ratio": 0.5,
  "unclassified_count": 0
}
```

This is effectively back to baseline subtype mix.

## 8. Enrichment coverage

`phase25d_run1_post_recovery_enriched.json`：

```json
{
  "recovery_count": 14,
  "with_pre_recovery_count": 14,
  "with_post_recovery_count": 14,
  "with_near_zero_count": 14,
  "sufficient_density_count": 14,
  "needs_periodic_snapshot_count": 0,
  "controller_received_path_but_cmd_near_zero_count": 0
}
```

Coverage OK。

## 9. Comparison vs baseline phase24c_run2

Baseline：`phase24c_run2`

```json
{
  "final_mode": "FAILED_EXHAUSTED",
  "goal_success_count": 8,
  "timeout_cancel_count": 4,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "footprint_path_blocked_late_silent": 2,
  "exit_distance_m": 0.7567794671891142
}
```

Experiment：`phase25d_run1`

```json
{
  "final_mode": "FAILED_EXHAUSTED",
  "goal_success_count": 8,
  "timeout_cancel_count": 4,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "footprint_path_blocked_late_silent": 2,
  "exit_distance_m": 0.8055346792124355
}
```

Hard gates：

```json
{
  "accepted": false
}
```

Passed：

```text
blocked_branch_no_regression: 0 -> 0
blacklisted_goal_no_regression: 0 -> 0
success_no_regression: 8 -> 8
exit_behavior_preserved_or_improved: FAILED_EXHAUSTED -> FAILED_EXHAUSTED
```

Failed：

```text
footprint_path_blocked_reduced: 2 -> 2
timeouts_reduced: 4 -> 4
```

Advisory exit distance：

```json
{
  "baseline": 0.7567794671891142,
  "experiment": 0.8055346792124355,
  "delta_m": 0.048755,
  "material_worsening_threshold_m": 0.5,
  "worsened_materially": false,
  "acceptable_or_exit_reached": true
}
```

## 10. Comparison vs Phase25B runs

Against `phase25b_run1`:

```text
footprint_path_blocked_late_silent: 0 -> 2  worse
timeouts: 3 -> 4 worse
successes: 9 -> 8 worse
exit_distance: 1.734 -> 0.806 better
```

Against `phase25b_run2`:

```text
footprint_path_blocked_late_silent: 0 -> 2 worse
timeouts: 3 -> 4 worse
successes: 9 -> 8 worse
exit_distance: 1.293 -> 0.806 better
```

So Phase25D trades away the Phase25B subtype/timeout benefit in exchange for near-baseline exit distance.

## 11. Conclusion

Phase 25D `cost_weight=3.0` is rejected by hard gates.

It improves the exit-distance advisory relative to Phase25B and is close to baseline：

```text
baseline exit_distance_m: 0.757
phase25d exit_distance_m: 0.806
```

But it fails the actual Phase 25 target：

```text
footprint_path_blocked_late_silent remains 2
timeouts remain 4
successes remain 8
```

This means the beneficial CostCritic effect appears to require a stronger reduction than 3.0, but 2.5 causes exit-distance regression.

## 12. Recommendation

Do not promote Phase25D.

Do not promote Phase25B 2.5 directly either, because of exit-distance advisory.

Recommended next step：Phase 25E, intermediate CostCritic value between 2.5 and 3.0, for example：

```yaml
FollowPath:
  CostCritic:
    cost_weight: 2.75
```

Rationale：

```text
3.0 behaves like baseline on target metrics.
2.5 improves target metrics but worsens exit distance.
2.75 may be the useful threshold compromise.
```

Keep same rules：

```text
one parameter family only
separate profile
baseline vs experiment
same hard gates
exit_distance advisory
```

Continue not doing：

```text
branch scoring
progress checker 粗调
```
