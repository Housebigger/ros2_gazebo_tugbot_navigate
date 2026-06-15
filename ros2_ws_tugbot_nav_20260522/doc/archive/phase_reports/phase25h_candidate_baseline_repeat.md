# Phase 25H Candidate-Baseline Repeat Validation

生成时间：2026-05-25

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 25H 目标：repeat candidate baseline once more with `candidate_baseline_run2` before deciding to abandon or promote.

Hold promotion rule：

```text
保持 nav2_slam_params.yaml unchanged
保留 nav2_slam_candidate_costcritic_275_params.yaml as candidate artifact
```

Decision rule：

```text
Promote only if candidate_baseline_run2 reaches EXIT_REACHED
or terminal-aware gates pass vs phase24c_run2,
and exit_distance advisory is acceptable_or_exit_reached.
```

继续不做：

```text
branch scoring
progress checker 粗调
```

## 2. Pre-check

确认 canonical baseline 未改：

```text
src/tugbot_navigation/config/nav2_slam_params.yaml
  FollowPath.CostCritic.cost_weight: 3.81
```

确认 candidate artifact 存在：

```text
src/tugbot_navigation/config/nav2_slam_candidate_costcritic_275_params.yaml
  FollowPath.CostCritic.cost_weight: 2.75
```

Focused tests：

```text
5 passed in 0.09s
```

## 3. Smoke：candidate_baseline_run2

Preflight：无真实 ROS/Gazebo/Nav2/recorder 残留。

Run：

```bash
bash tools/run_phase21_controller_diagnostics_smoke.sh candidate_baseline_run2
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
  "exit_distance_m": 1.083488150434483
}
```

Nav2 summary：

```json
{
  "goal_count": 12,
  "success_count": 8,
  "timeout_count": 3,
  "timeout_with_progress_failure_count": 3,
  "timeout_with_recovery_count": 3,
  "timeout_with_controller_abort_count": 3,
  "success_with_progress_failure_count": 2,
  "success_with_recovery_count": 2,
  "success_with_controller_abort_count": 2
}
```

Note：run summary `timeout_cancel_count` remains 4 because final failure includes a failed goal classification, while Nav2 timeout rows are 3.

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

## 4. Generated artifacts

```text
log/candidate_baseline_run2_failure_windows.json
log/candidate_baseline_run2_timeout_subtypes.json
log/candidate_baseline_run2_post_recovery_enriched.json
log/candidate_baseline_run2_phase25h_compare_input.json
log/candidate_baseline_run2_terminal_success_acceptance_vs_phase24c_run2.json
log/candidate_baseline_run2_terminal_success_acceptance_vs_candidate_baseline_run1.json
log/candidate_baseline_run2_terminal_success_acceptance_vs_phase25e_run1.json
log/candidate_baseline_run2_terminal_success_acceptance_vs_phase25e_run2.json
```

## 5. Timeout subtype result

`candidate_baseline_run2_timeout_subtypes.json`：

```json
{
  "timeout_count": 3,
  "controller_subtype_counts": {
    "footprint_path_blocked_late_silent": 1,
    "side_cost_or_timing_late_silent": 2
  },
  "timing_subtype_counts": {
    "cmd_silent_after_recovery_abort": 3
  },
  "severe_footprint_path_count": 1,
  "severe_footprint_path_ratio": 0.333333,
  "unclassified_count": 0
}
```

Relative to baseline phase24c_run2：

```text
footprint_path_blocked_late_silent: 2 -> 1
```

But run-level timeout_cancel_count did not improve：

```text
timeout_cancel_count: 4 -> 4
```

## 6. Enrichment coverage

`candidate_baseline_run2_post_recovery_enriched.json`：

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

## 7. Terminal-aware comparison vs baseline phase24c_run2

Baseline：

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

Candidate run2：

```json
{
  "final_mode": "FAILED_EXHAUSTED",
  "goal_count": 12,
  "goal_success_count": 8,
  "timeout_cancel_count": 4,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "footprint_path_blocked_late_silent": 1,
  "exit_distance_m": 1.083488150434483
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
  "experiment": 1.083488150434483,
  "delta_m": 0.326709,
  "worsened_materially": false,
  "acceptable_or_exit_reached": true
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

## 8. Comparison vs candidate_baseline_run1

Candidate run1：

```text
FAILED_EXHAUSTED
timeout_cancel_count 4
footprint_path_blocked 1
exit_distance 1.337
```

Candidate run2：

```text
FAILED_EXHAUSTED
timeout_cancel_count 4
footprint_path_blocked 1
exit_distance 1.083
```

Run2 improved exit distance relative to run1, but still did not pass promotion gates.

## 9. Comparison vs Phase25E runs

Against `phase25e_run1`：

```text
candidate_baseline_run2 loses EXIT_REACHED
timeouts worsen: 2 -> 4
footprint_path_blocked worsens: 0 -> 1
exit_distance worsens: 0.595 -> 1.083
```

Against `phase25e_run2`：

```text
candidate_baseline_run2 loses EXIT_REACHED
timeouts worsen: 3 -> 4
footprint_path_blocked same: 1 -> 1
exit_distance worsens: 0.569 -> 1.083
```

## 10. Decision

Phase 25H confirms：do not promote `cost_weight: 2.75` to canonical baseline.

Reason：

```text
candidate_baseline_run1 failed promotion.
candidate_baseline_run2 also failed promotion.
Neither candidate run reached EXIT_REACHED.
Neither candidate run reduced run-level timeout_cancel_count vs baseline.
Candidate run2 did have acceptable exit-distance advisory vs baseline, but hard gate still failed on timeouts.
```

Therefore：

```text
Preserve 2.75 as experimental/candidate artifact only.
Keep nav2_slam_params.yaml unchanged.
```

## 11. Recommended next step

Recommended Phase 26A：investigate run-to-run variance / profile-path sensitivity before more tuning.

Why：

```text
Phase25E runs using phase25e profile reached EXIT_REACHED twice.
Candidate baseline runs using candidate profile did not reach exit twice.
Both profiles use cost_weight 2.75 and should be behaviorally equivalent.
```

Before testing 2.65 / 2.6, verify whether there is a real profile/config difference or launch selection issue.

Suggested checks：

```text
1. diff nav2_slam_phase25e_costcritic_compromise_params.yaml vs nav2_slam_candidate_costcritic_275_params.yaml, ignoring comments.
2. confirm launch logs show the expected params file path for phase25e_run and candidate_baseline_run.
3. add a params-file fingerprint to launch log or wrapper artifact.
4. if files are equivalent, treat the difference as run variance and run multiple seeds/repeats before changing cost_weight again.
```

Continue not doing：

```text
branch scoring
progress checker 粗调
```
