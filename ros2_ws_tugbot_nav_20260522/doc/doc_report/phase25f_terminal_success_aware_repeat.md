# Phase 25F Terminal-Success-Aware Comparator + Phase25E Repeat Validation

生成时间：2026-05-25

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 25F 目标：

```text
make acceptance comparator terminal-success-aware, then repeat-validate Phase25E.
```

具体：

```text
If experiment final_mode == EXIT_REACHED:
  不再要求 raw goal_success_count >= baseline_goal_success_count
  改为记录 success ratio / goal efficiency advisory
  保留 safety gates: blocked/blacklist no regression
  保留 subtype/timeouts/exit behavior gates
else:
  保持当前 success_no_regression
```

Repeat run：

```text
phase25e_run2
```

继续不做：

```text
branch scoring
progress checker 粗调
```

## 2. TDD

新增测试：

```text
src/tugbot_maze/test/test_phase25f_terminal_success_aware_comparator.py
```

RED 已确认：

```text
2 failed, 1 passed
```

失败原因符合预期：

```text
EXIT_REACHED still rejected when raw success count regresses
checks['success_gate'] missing
```

## 3. Comparator changes

修改：

```text
tools/compare_phase25e_metrics.py
```

新增 terminal-success-aware gate：

```text
success_gate:
  if experiment final_mode == EXIT_REACHED:
    pass, with criterion = "experiment EXIT_REACHED bypasses raw success count gate"
  else:
    require experiment goal_success_count >= baseline goal_success_count
```

保留 hard gates：

```text
footprint_path_blocked_reduced
timeouts_reduced
blocked_branch_no_regression
blacklisted_goal_no_regression
exit_behavior_preserved_or_improved
```

新增 advisory：

```json
"success_efficiency": {
  "baseline_goal_count": 12,
  "baseline_success_count": 8,
  "baseline_success_ratio": 0.6666666666666666,
  "experiment_goal_count": 11,
  "experiment_success_count": 7,
  "experiment_success_ratio": 0.6363636363636364,
  "raw_success_count_regressed": true,
  "success_ratio_delta": -0.030303
}
```

## 4. Verification

Focused：

```text
6 passed in 0.13s
```

Full tests：

```text
98 passed in 1.03s
```

Build：

```text
colcon build --packages-select tugbot_navigation tugbot_bringup tugbot_maze --symlink-install
Summary: 3 packages finished
```

## 5. Repeat smoke：phase25e_run2

Preflight：无真实 ROS/Gazebo/Nav2/recorder 残留。

Run：

```bash
bash tools/run_phase21_controller_diagnostics_smoke.sh phase25e_run2
```

Wrapper exit code：`0`

Run summary：

```json
{
  "final_mode": "EXIT_REACHED",
  "goal_count": 11,
  "goal_success_count": 7,
  "goal_failure_count": 3,
  "timeout_cancel_count": 3,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "exit_distance_m": 0.56859917394816
}
```

Nav2 summary：

```json
{
  "goal_count": 11,
  "success_count": 7,
  "timeout_count": 3,
  "timeout_with_progress_failure_count": 3,
  "timeout_with_recovery_count": 3,
  "timeout_with_controller_abort_count": 3,
  "success_with_progress_failure_count": 2,
  "success_with_recovery_count": 2,
  "success_with_controller_abort_count": 2
}
```

Controller summary：

```json
{
  "goal_count": 11,
  "healthy_motion_but_late_stall_count": 3,
  "late_controller_silent_count": 3,
  "timeout_or_failure_late_stall_count": 3,
  "healthy_motion_count": 7,
  "slow_progress_count": 1,
  "terminal_cancel_after_exit_count": 1
}
```

## 6. Generated artifacts

```text
log/phase25e_run2_failure_windows.json
log/phase25e_run2_timeout_subtypes.json
log/phase25e_run2_post_recovery_enriched.json
log/phase25e_run2_phase25f_compare_input.json
log/phase25e_run2_terminal_success_acceptance_vs_phase24c_run2.json
log/phase25e_run2_terminal_success_acceptance_vs_phase25e_run1.json
```

## 7. Timeout subtype result

`phase25e_run2_timeout_subtypes.json`：

```json
{
  "timeout_count": 3,
  "controller_subtype_counts": {
    "footprint_path_blocked_late_silent": 1,
    "side_cost_or_timing_late_silent": 1,
    "unclassified_late_silent": 1
  },
  "timing_subtype_counts": {
    "cmd_silent_after_recovery_abort": 2,
    "cmd_silent_before_progress_failure": 1
  },
  "severe_footprint_path_count": 1,
  "severe_footprint_path_ratio": 0.333333,
  "unclassified_count": 1
}
```

Target subtype relative to baseline：

```text
footprint_path_blocked_late_silent: 2 -> 1
```

Not as clean as phase25e_run1, but still improved vs baseline.

## 8. Enrichment coverage

`phase25e_run2_post_recovery_enriched.json`：

```json
{
  "recovery_count": 9,
  "with_pre_recovery_count": 9,
  "with_post_recovery_count": 9,
  "with_near_zero_count": 9,
  "sufficient_density_count": 9,
  "needs_periodic_snapshot_count": 0,
  "controller_received_path_but_cmd_near_zero_count": 0
}
```

Coverage OK。

## 9. Terminal-aware comparison vs baseline phase24c_run2

Baseline：`phase24c_run2`

```json
{
  "final_mode": "FAILED_EXHAUSTED",
  "goal_count": 12,
  "goal_success_count": 8,
  "timeout_cancel_count": 4,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "footprint_path_blocked_late_silent": 2,
  "exit_distance_m": 0.7567794671891142
}
```

Experiment：`phase25e_run2`

```json
{
  "final_mode": "EXIT_REACHED",
  "goal_count": 11,
  "goal_success_count": 7,
  "timeout_cancel_count": 3,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "footprint_path_blocked_late_silent": 1,
  "exit_distance_m": 0.56859917394816
}
```

Terminal-aware hard gates：

```json
{
  "accepted": true
}
```

Passed：

```text
footprint_path_blocked_reduced: 2 -> 1
timeouts_reduced: 4 -> 3
blocked_branch_no_regression: 0 -> 0
blacklisted_goal_no_regression: 0 -> 0
exit_behavior_preserved_or_improved: FAILED_EXHAUSTED -> EXIT_REACHED
success_gate: EXIT_REACHED bypasses raw success count gate
```

Advisories：

```json
{
  "exit_distance_m": {
    "baseline": 0.7567794671891142,
    "experiment": 0.56859917394816,
    "delta_m": -0.18818,
    "worsened_materially": false,
    "acceptable_or_exit_reached": true
  },
  "success_efficiency": {
    "baseline_success_ratio": 0.6666666666666666,
    "experiment_success_ratio": 0.6363636363636364,
    "success_ratio_delta": -0.030303,
    "raw_success_count_regressed": true
  }
}
```

Recommendation：

```text
candidate_baseline_or_repeat_validation
```

## 10. Comparison vs phase25e_run1

`phase25e_run1`：

```json
{
  "final_mode": "EXIT_REACHED",
  "goal_count": 9,
  "goal_success_count": 6,
  "timeout_cancel_count": 2,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "footprint_path_blocked_late_silent": 0,
  "exit_distance_m": 0.5945372288918425
}
```

`phase25e_run2`：

```json
{
  "final_mode": "EXIT_REACHED",
  "goal_count": 11,
  "goal_success_count": 7,
  "timeout_cancel_count": 3,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "footprint_path_blocked_late_silent": 1,
  "exit_distance_m": 0.56859917394816
}
```

Strict comparison vs run1 says not accepted because run2 has:

```text
footprint_path_blocked: 0 -> 1
timeouts: 2 -> 3
```

But repeat validation signals remain acceptable vs baseline：

```text
Both runs reach EXIT_REACHED.
Both improve exit_distance vs baseline.
Both reduce timeout count vs baseline.
Both improve footprint_path_blocked vs baseline.
Both keep blocked/blacklist 0.
```

## 11. Conclusion

Phase25E `cost_weight=2.75` is now repeat-validated as candidate baseline under terminal-success-aware gates.

Evidence：

```text
phase25e_run1:
  EXIT_REACHED
  exit_distance 0.595
  footprint_path_blocked 0
  timeouts 2
  blocked/blacklist 0

phase25e_run2:
  EXIT_REACHED
  exit_distance 0.569
  footprint_path_blocked 1
  timeouts 3
  blocked/blacklist 0
```

Baseline `phase24c_run2`：

```text
FAILED_EXHAUSTED
exit_distance 0.757
footprint_path_blocked 2
timeouts 4
blocked/blacklist 0
```

Therefore：

```text
cost_weight=2.75 is a candidate baseline, not merely a one-off improvement.
```

Caveat：

```text
phase25e_run2 still has one footprint_path_blocked_late_silent and one unclassified_late_silent timeout.
```

So I recommend promoting 2.75 as candidate baseline only after one more confirmation or candidate-baseline smoke, not silently overwriting the original baseline.

## 12. Recommendation

Recommended next phase：Phase 25G candidate-baseline validation.

Options：

1. Candidate baseline profile validation:

```text
Run phase25e_run3 or candidate_baseline_run1 with cost_weight=2.75.
Accept if EXIT_REACHED or terminal-aware gates pass vs phase24c_run2.
```

2. If the user wants to move faster:

```text
Promote cost_weight=2.75 into a clearly named candidate params file, not the canonical baseline yet.
Keep nav2_slam_params.yaml unchanged until candidate run is confirmed.
```

Continue not doing：

```text
branch scoring
progress checker 粗调
```
