# Phase 25C Repeat Validation for Phase25B CostCritic Profile

生成时间：2026-05-25

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 25C 目标：repeat-validate Phase25B MPPI CostCritic profile。

Run：

```text
phase25b_run2
```

比较对象：

```text
phase24c_run2 baseline
phase25b_run1 prior experiment
```

新增 advisory metric：

```text
exit_distance_m should not worsen materially
```

判断标准：

```text
footprint_path_blocked_late_silent <= baseline
timeouts <= baseline
successes >= baseline
blocked/blacklist = 0
exit distance acceptable or EXIT_REACHED
```

## 2. TDD

新增测试：

```text
src/tugbot_maze/test/test_phase25c_repeat_validation_metrics.py
```

RED 已确认：

```text
KeyError: 'advisory'
```

即 `compare_phase25b_metrics.py` 还没有输出 exit distance advisory。

## 3. 实现

修改：

```text
tools/compare_phase25b_metrics.py
```

新增：

```json
"advisory": {
  "exit_distance_m": {
    "baseline": ...,
    "experiment": ...,
    "delta_m": ...,
    "material_worsening_threshold_m": 0.5,
    "worsened_materially": true/false,
    "acceptable_or_exit_reached": true/false
  }
}
```

规则：

```text
worsened_materially = experiment_exit_distance - baseline_exit_distance > 0.5m
acceptable_or_exit_reached = experiment final_mode == EXIT_REACHED or not worsened_materially
```

新增 recommendation：

```text
candidate_baseline_or_smaller_delta_validation
repeat_or_smaller_delta_before_baseline_promotion
reject_or_retry_different_single_family_experiment
```

注意：exit_distance 仍是 advisory，不影响原 hard gates 的 `accepted` 字段。

## 4. Verification

Focused：

```text
5 passed in 0.10s
```

Full tests：

```text
89 passed in 0.90s
```

Build：

```text
colcon build --packages-select tugbot_navigation tugbot_bringup tugbot_maze --symlink-install
Summary: 3 packages finished
```

## 5. Repeat smoke：phase25b_run2

Preflight：无真实 ROS/Gazebo/Nav2/recorder 残留。

Run：

```bash
bash tools/run_phase21_controller_diagnostics_smoke.sh phase25b_run2
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
  "exit_distance_m": 1.2926106282516905
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
  "success_with_progress_failure_count": 2,
  "success_with_recovery_count": 2,
  "success_with_controller_abort_count": 2
}
```

Controller summary：

```json
{
  "goal_count": 12,
  "healthy_motion_but_late_stall_count": 3,
  "late_controller_silent_count": 3,
  "timeout_or_failure_late_stall_count": 3,
  "healthy_motion_count": 7,
  "slow_progress_count": 2
}
```

## 6. Generated artifacts

```text
log/phase25b_run2_failure_windows.json
log/phase25b_run2_timeout_subtypes.json
log/phase25b_run2_post_recovery_enriched.json
log/phase25b_run2_phase25c_compare_input.json
log/phase25b_run2_acceptance_vs_phase24c_run2.json
log/phase25b_run2_acceptance_vs_phase25b_run1.json
```

## 7. Timeout subtype result

`phase25b_run2_timeout_subtypes.json`：

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

Repeat result preserved the most important Phase25B signal：

```text
footprint_path_blocked_late_silent = 0
```

## 8. Enrichment coverage

`phase25b_run2_post_recovery_enriched.json`：

```json
{
  "recovery_count": 9,
  "with_pre_recovery_count": 9,
  "with_post_recovery_count": 7,
  "with_near_zero_count": 9,
  "sufficient_density_count": 7,
  "needs_periodic_snapshot_count": 2,
  "controller_received_path_but_cmd_near_zero_count": 0
}
```

Coverage was not perfect：2 recovery rows lacked post-recovery dense snapshot, but the subtype/failure-window metrics were still available.

## 9. Comparison vs phase24c_run2 baseline

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

Experiment：`phase25b_run2`

```json
{
  "final_mode": "FAILED_EXHAUSTED",
  "goal_success_count": 9,
  "timeout_cancel_count": 3,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "footprint_path_blocked_late_silent": 0,
  "exit_distance_m": 1.2926106282516905
}
```

Hard gates：

```json
{
  "accepted": true
}
```

All hard gates passed：

```text
footprint_path_blocked_reduced: 2 -> 0
timeouts_reduced: 4 -> 3
success_no_regression: 8 -> 9
blocked_branch_no_regression: 0 -> 0
blacklisted_goal_no_regression: 0 -> 0
exit_behavior_preserved_or_improved: FAILED_EXHAUSTED -> FAILED_EXHAUSTED
```

Advisory：

```json
{
  "exit_distance_m": {
    "baseline": 0.7567794671891142,
    "experiment": 1.2926106282516905,
    "delta_m": 0.535831,
    "material_worsening_threshold_m": 0.5,
    "worsened_materially": true,
    "acceptable_or_exit_reached": false
  }
}
```

Recommendation：

```text
repeat_or_smaller_delta_before_baseline_promotion
```

## 10. Comparison vs phase25b_run1 prior experiment

Prior：`phase25b_run1`

```json
{
  "final_mode": "FAILED_EXHAUSTED",
  "goal_success_count": 9,
  "timeout_cancel_count": 3,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "footprint_path_blocked_late_silent": 0,
  "exit_distance_m": 1.7342131462699826
}
```

Repeat：`phase25b_run2`

```json
{
  "final_mode": "FAILED_EXHAUSTED",
  "goal_success_count": 9,
  "timeout_cancel_count": 3,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "footprint_path_blocked_late_silent": 0,
  "exit_distance_m": 1.2926106282516905
}
```

Strict comparator result：

```json
{
  "accepted": false
}
```

Why false：hard gates require strictly reduced footprint subtype and timeouts. Since prior was already 0 / 3, repeat cannot be strictly lower：

```text
footprint_path_blocked_reduced: 0 -> 0 false by strict <
timeouts_reduced: 3 -> 3 false by strict <
```

But for repeat validation, this is not a practical regression. Important repeat metrics were stable：

```text
footprint_path_blocked_late_silent: 0 -> 0
timeouts: 3 -> 3
successes: 9 -> 9
blocked/blacklist: 0 -> 0
exit_distance_m improved: 1.734 -> 1.293
```

## 11. Conclusion

Phase25B CostCritic profile is repeat-validated on target/core metrics：

```text
footprint_path_blocked_late_silent: baseline 2 -> run1 0 -> run2 0
timeouts: baseline 4 -> run1 3 -> run2 3
successes: baseline 8 -> run1 9 -> run2 9
blocked/blacklist: all 0/0
```

However, exit distance remains worse than baseline：

```text
baseline: 0.757m
run1:     1.734m
run2:     1.293m
```

Phase25B run2 improved exit distance vs run1, but still trips the advisory threshold vs baseline：

```text
+0.536m > 0.5m threshold
```

Therefore：

```text
Do not promote 2.5 directly to baseline yet.
```

## 12. Recommendation

Recommended next phase：Phase 25D, smaller CostCritic delta.

Candidate：

```yaml
FollowPath:
  CostCritic:
    cost_weight: 3.0
```

Rationale：

- `2.5` repeatedly eliminated `footprint_path_blocked_late_silent` and improved timeouts/successes.
- But `2.5` worsened exit distance vs baseline.
- A midpoint `3.0` may retain subtype/timeouts benefit while reducing exit-distance regression.

Keep same rules：

```text
one parameter family only
separate profile
baseline vs experiment
same hard gates
exit_distance advisory
```

Do not branch score.
Do not globally tune progress checker.
