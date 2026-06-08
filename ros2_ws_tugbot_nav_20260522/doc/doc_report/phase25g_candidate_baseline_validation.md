# Phase 25G Candidate-Baseline Validation

生成时间：2026-05-25

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 25G 目标：candidate-baseline validation。

要求：

```text
不要直接覆盖 canonical baseline。
创建 clearly named candidate params/profile。
保持 nav2_slam_params.yaml unchanged。
跑 candidate_baseline_run1。
Accept if it reaches EXIT_REACHED or terminal-aware gates pass vs phase24c_run2。
```

继续不做：

```text
branch scoring
progress checker 粗调
```

## 2. TDD

新增测试：

```text
src/tugbot_maze/test/test_phase25g_candidate_baseline_profile.py
```

RED 已确认：

```text
2 failed
- candidate params file missing
- launch/wrapper missing candidate_baseline_runN support
```

## 3. 实现

新增 candidate params：

```text
src/tugbot_navigation/config/nav2_slam_candidate_costcritic_275_params.yaml
```

该文件复制 canonical baseline：

```text
src/tugbot_navigation/config/nav2_slam_params.yaml
```

只改：

```yaml
FollowPath:
  CostCritic:
    cost_weight: 2.75
```

Canonical baseline 保持不变：

```text
nav2_slam_params.yaml still has cost_weight: 3.81
```

Launch integration：

```text
src/tugbot_bringup/launch/tugbot_maze_explore.launch.py
```

新增：

```text
candidate_costcritic_275_profile
```

Wrapper integration：

```text
tools/run_phase21_controller_diagnostics_smoke.sh
```

新增 run id：

```text
candidate_baseline_runN
```

当 run id 匹配时：

```bash
CANDIDATE_COSTCRITIC_275_PROFILE=true
candidate_costcritic_275_profile:="$CANDIDATE_COSTCRITIC_275_PROFILE"
```

## 4. Verification

Focused：

```text
5 passed in 0.09s
```

Full tests：

```text
100 passed in 1.03s
```

Build：

```text
colcon build --packages-select tugbot_navigation tugbot_bringup tugbot_maze --symlink-install
Summary: 3 packages finished
```

## 5. Smoke：candidate_baseline_run1

Preflight：无真实 ROS/Gazebo/Nav2/recorder 残留。

Run：

```bash
bash tools/run_phase21_controller_diagnostics_smoke.sh candidate_baseline_run1
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
  "exit_distance_m": 1.337332079816478
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
  "success_with_progress_failure_count": 3,
  "success_with_recovery_count": 3,
  "success_with_controller_abort_count": 3
}
```

Controller summary：

```json
{
  "goal_count": 12,
  "healthy_motion_but_late_stall_count": 4,
  "late_controller_silent_count": 4,
  "timeout_or_failure_late_stall_count": 4,
  "healthy_motion_count": 6,
  "slow_progress_count": 2
}
```

## 6. Generated artifacts

```text
log/candidate_baseline_run1_failure_windows.json
log/candidate_baseline_run1_timeout_subtypes.json
log/candidate_baseline_run1_post_recovery_enriched.json
log/candidate_baseline_run1_phase25g_compare_input.json
log/candidate_baseline_run1_terminal_success_acceptance_vs_phase24c_run2.json
log/candidate_baseline_run1_terminal_success_acceptance_vs_phase25e_run1.json
log/candidate_baseline_run1_terminal_success_acceptance_vs_phase25e_run2.json
```

## 7. Timeout subtype result

`candidate_baseline_run1_timeout_subtypes.json`：

```json
{
  "timeout_count": 4,
  "controller_subtype_counts": {
    "footprint_path_blocked_late_silent": 1,
    "side_cost_or_timing_late_silent": 2,
    "unclassified_late_silent": 1
  },
  "timing_subtype_counts": {
    "cmd_silent_after_recovery_abort": 4
  },
  "severe_footprint_path_count": 1,
  "severe_footprint_path_ratio": 0.25,
  "unclassified_count": 1
}
```

Target subtype relative to baseline：

```text
footprint_path_blocked_late_silent: 2 -> 1
```

But timeout count did not improve：

```text
timeouts: 4 -> 4
```

## 8. Enrichment coverage

`candidate_baseline_run1_post_recovery_enriched.json`：

```json
{
  "recovery_count": 16,
  "with_pre_recovery_count": 16,
  "with_post_recovery_count": 16,
  "with_near_zero_count": 16,
  "sufficient_density_count": 16,
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

Candidate：`candidate_baseline_run1`

```json
{
  "final_mode": "FAILED_EXHAUSTED",
  "goal_count": 12,
  "goal_success_count": 8,
  "timeout_cancel_count": 4,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "footprint_path_blocked_late_silent": 1,
  "exit_distance_m": 1.337332079816478
}
```

Terminal-aware hard gates：

```json
{
  "accepted": false
}
```

Passed：

```text
footprint_path_blocked_reduced: 2 -> 1
blocked_branch_no_regression: 0 -> 0
blacklisted_goal_no_regression: 0 -> 0
success_gate: 8 -> 8
exit_behavior_preserved_or_improved: FAILED_EXHAUSTED -> FAILED_EXHAUSTED
```

Failed：

```text
timeouts_reduced: 4 -> 4
```

Advisory exit distance：

```json
{
  "baseline": 0.7567794671891142,
  "experiment": 1.337332079816478,
  "delta_m": 0.580553,
  "worsened_materially": true,
  "acceptable_or_exit_reached": false
}
```

Success efficiency：

```json
{
  "baseline_success_ratio": 0.6666666666666666,
  "experiment_success_ratio": 0.6666666666666666,
  "success_ratio_delta": 0.0,
  "raw_success_count_regressed": false
}
```

Recommendation：

```text
reject_or_retry_different_single_family_experiment
```

## 10. Comparison vs Phase25E runs

Against phase25e_run1：

```text
candidate_baseline_run1 loses EXIT_REACHED.
timeouts worsen: 2 -> 4
footprint_path_blocked worsens: 0 -> 1
exit_distance worsens: 0.595 -> 1.337
```

Against phase25e_run2：

```text
candidate_baseline_run1 loses EXIT_REACHED.
timeouts worsen: 3 -> 4
footprint_path_blocked same: 1 -> 1
exit_distance worsens: 0.569 -> 1.337
```

## 11. Conclusion

Phase 25G candidate-baseline validation failed.

Do not promote `cost_weight: 2.75` into canonical `nav2_slam_params.yaml` based on `candidate_baseline_run1`.

Reason：

```text
candidate_baseline_run1 did not reach exit.
timeouts did not improve vs baseline.
exit_distance materially worsened vs baseline.
```

Even though it still reduced footprint_path_blocked_late_silent from 2 to 1, it did not satisfy the candidate-baseline acceptance rule.

Important nuance：

```text
candidate profile file is effectively the same CostCritic value as Phase25E.
phase25e_run1/run2 were strong, but candidate_baseline_run1 was not.
This suggests either run variance or sensitivity to the slightly different launch/profile path is not enough to justify canonical promotion yet.
```

## 12. Recommendation

Hold promotion.

Keep：

```text
nav2_slam_params.yaml unchanged
nav2_slam_candidate_costcritic_275_params.yaml as candidate artifact
```

Recommended next step：

```text
Phase 25H: repeat candidate baseline once more with candidate_baseline_run2 before deciding to abandon or promote.
```

Decision rule：

```text
Promote only if candidate_baseline_run2 reaches EXIT_REACHED or terminal-aware gates pass vs phase24c_run2,
and exit_distance advisory is acceptable_or_exit_reached.
```

If candidate_baseline_run2 also fails, then do not promote 2.75; preserve it as experimental profile only and consider either:

```text
1. investigate run-to-run variance / spawn-path sensitivity
2. test narrower value around 2.65 or 2.6
3. keep Phase25E as optional experimental profile, not baseline
```

Continue not doing：

```text
branch scoring
progress checker 粗调
```
