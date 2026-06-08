# Phase 25E MPPI CostCritic Compromise Experiment

生成时间：2026-05-25

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 25E 目标：测试 Phase25B `2.5` 和 Phase25D `3.0` 之间的中间值。

配置：

```yaml
FollowPath:
  CostCritic:
    cost_weight: 2.75
```

理由：

```text
3.0 behaves like baseline on target metrics.
2.5 improves target metrics but worsens exit distance.
2.75 may be the useful threshold compromise.
```

继续遵守：

```text
one parameter family only
separate profile
baseline vs experiment
same hard gates
exit_distance advisory
```

继续不做：

```text
branch scoring
progress checker 粗调
```

## 2. TDD

新增测试：

```text
src/tugbot_maze/test/test_phase25e_costcritic_compromise_experiment.py
```

RED 已确认：

```text
3 failed
- Phase25E CostCritic compromise params missing
- launch/wrapper missing phase25e profile
- compare_phase25e_metrics.py missing
```

## 3. 实现

新增 profile：

```text
src/tugbot_navigation/config/nav2_slam_phase25e_costcritic_compromise_params.yaml
```

该文件复制 baseline `nav2_slam_params.yaml`，只改：

```yaml
FollowPath:
  CostCritic:
    cost_weight: 2.75
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
phase25e_costcritic_compromise_profile
```

Wrapper integration：

```text
tools/run_phase21_controller_diagnostics_smoke.sh
```

新增：

```text
phase25e_runN
PHASE25E_PROFILE=true only when run id matches phase25e_runN
phase25e_costcritic_compromise_profile:="$PHASE25E_PROFILE"
```

Comparator：

```text
tools/compare_phase25e_metrics.py
```

复用 hard gates + exit_distance advisory。

## 4. Verification

Focused：

```text
6 passed in 0.07s
```

Full tests：

```text
95 passed in 0.93s
```

Build：

```text
colcon build --packages-select tugbot_navigation tugbot_bringup tugbot_maze --symlink-install
Summary: 3 packages finished
```

## 5. Smoke：phase25e_run1

Preflight：无真实 ROS/Gazebo/Nav2/recorder 残留。

Run：

```bash
bash tools/run_phase21_controller_diagnostics_smoke.sh phase25e_run1
```

Wrapper exit code：`0`

Run summary：

```json
{
  "final_mode": "EXIT_REACHED",
  "goal_count": 9,
  "goal_success_count": 6,
  "goal_failure_count": 2,
  "timeout_cancel_count": 2,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "exit_distance_m": 0.5945372288918425
}
```

Nav2 summary：

```json
{
  "goal_count": 9,
  "success_count": 6,
  "timeout_count": 2,
  "timeout_with_progress_failure_count": 2,
  "timeout_with_recovery_count": 2,
  "timeout_with_controller_abort_count": 2,
  "success_with_progress_failure_count": 0,
  "success_with_recovery_count": 0,
  "success_with_controller_abort_count": 0
}
```

Controller summary：

```json
{
  "goal_count": 9,
  "healthy_motion_but_late_stall_count": 2,
  "late_controller_silent_count": 2,
  "timeout_or_failure_late_stall_count": 2,
  "healthy_motion_count": 6,
  "slow_progress_count": 1,
  "terminal_cancel_after_exit_count": 1
}
```

## 6. Generated artifacts

```text
log/phase25e_run1_failure_windows.json
log/phase25e_run1_timeout_subtypes.json
log/phase25e_run1_post_recovery_enriched.json
log/phase25e_run1_phase25e_compare_input.json
log/phase25e_run1_acceptance_vs_phase24c_run2.json
log/phase25e_run1_acceptance_vs_phase25b_run1.json
log/phase25e_run1_acceptance_vs_phase25b_run2.json
log/phase25e_run1_acceptance_vs_phase25d_run1.json
```

## 7. Timeout subtype result

`phase25e_run1_timeout_subtypes.json`：

```json
{
  "timeout_count": 2,
  "controller_subtype_counts": {
    "side_cost_or_timing_late_silent": 2
  },
  "timing_subtype_counts": {
    "cmd_silent_after_recovery_abort": 2
  },
  "severe_footprint_path_count": 0,
  "severe_footprint_path_ratio": 0.0,
  "unclassified_count": 0
}
```

Target subtype：

```text
footprint_path_blocked_late_silent = 0
```

## 8. Enrichment coverage

`phase25e_run1_post_recovery_enriched.json`：

```json
{
  "recovery_count": 7,
  "with_pre_recovery_count": 7,
  "with_post_recovery_count": 7,
  "with_near_zero_count": 7,
  "sufficient_density_count": 7,
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

Experiment：`phase25e_run1`

```json
{
  "final_mode": "EXIT_REACHED",
  "goal_success_count": 6,
  "timeout_cancel_count": 2,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "footprint_path_blocked_late_silent": 0,
  "exit_distance_m": 0.5945372288918425
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
footprint_path_blocked_reduced: 2 -> 0
timeouts_reduced: 4 -> 2
blocked_branch_no_regression: 0 -> 0
blacklisted_goal_no_regression: 0 -> 0
exit_behavior_preserved_or_improved: FAILED_EXHAUSTED -> EXIT_REACHED
```

Failed：

```text
success_no_regression: 8 -> 6
```

Advisory exit distance：

```json
{
  "baseline": 0.7567794671891142,
  "experiment": 0.5945372288918425,
  "delta_m": -0.162242,
  "material_worsening_threshold_m": 0.5,
  "worsened_materially": false,
  "acceptable_or_exit_reached": true
}
```

## 10. Comparison vs other CostCritic profiles

Against Phase25B `2.5` runs：

```text
phase25e_run1 reaches exit; phase25b runs do not.
exit_distance improves substantially.
timeouts improve 3 -> 2.
footprint_path_blocked remains 0 -> 0.
But success count is lower: 9 -> 6, because run terminated at EXIT_REACHED after 9 goals instead of exhausting 12-goal budget.
```

Against Phase25D `3.0`：

```text
footprint_path_blocked_late_silent: 2 -> 0 better
timeouts: 4 -> 2 better
exit behavior: FAILED_EXHAUSTED -> EXIT_REACHED better
exit_distance: 0.806 -> 0.595 better
successes: 8 -> 6 lower due to early exit termination
```

## 11. Interpretation

The strict comparator rejects Phase25E only because `goal_success_count` is lower:

```text
baseline success count: 8
phase25e success count: 6
```

But this run ended early with：

```text
final_mode = EXIT_REACHED
goal_count = 9
```

So `goal_success_count` is not directly comparable to 12-goal exhausted runs.

Important signals are strong：

```text
EXIT_REACHED achieved
exit_distance improved vs baseline
footprint_path_blocked_late_silent eliminated
timeouts reduced
blocked/blacklist stayed 0
```

This suggests Phase25E is promising, but the existing comparator needs an explicit terminal-success-aware success metric before promotion.

## 12. Conclusion

Phase 25E `cost_weight=2.75` is not accepted by the current strict hard gates, but it is the strongest candidate so far qualitatively:

```text
EXIT_REACHED
exit distance: 0.595m
footprint_path_blocked_late_silent: 0
timeouts: 2
blocked/blacklist: 0
```

However, because the strict comparator penalizes lower success count on a shorter successful run, do not promote it directly yet.

## 13. Recommendation

Recommended next phase：Phase 25F.

Goal：make the acceptance comparator terminal-success-aware, then repeat-validate Phase25E.

Suggested acceptance adjustment：

```text
If experiment final_mode == EXIT_REACHED:
  success_no_regression can be replaced by terminal_success_success_ratio / goal_efficiency advisory
else:
  keep success_no_regression as before
```

Then run：

```text
phase25e_run2
```

Decision rule：

```text
If phase25e_run2 also reaches EXIT_REACHED or keeps target/core metrics improved without blocked/blacklist regression,
consider Phase25E cost_weight=2.75 as candidate baseline.
```

Continue not doing：

```text
branch scoring
progress checker 粗调
```
