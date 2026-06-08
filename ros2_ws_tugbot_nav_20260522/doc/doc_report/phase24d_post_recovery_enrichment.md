# Phase 24D Post-Run Enrichment of Runtime Snapshots

生成时间：2026-05-25

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 24D 目标：把 Phase 24C runtime recorder 输出的 snapshots 与 Nav2 recovery timestamps 对齐，重建：

```text
pre_recovery snapshot
post_recovery snapshot
recovery-time path-ahead cost
recovery-time robot-to-path distance
near-zero snapshot
```

输入：

```text
*_post_recovery_snapshots.jsonl
*_goal_nav2_analysis.json
```

输出 per-goal / per-recovery enriched alignment。

## 2. TDD

新增测试：

```text
src/tugbot_maze/test/test_phase24d_enrich_post_recovery_snapshots.py
src/tugbot_maze/test/test_phase24d_periodic_recorder_contract.py
```

RED 已确认：

```text
can't open file tools/enrich_post_recovery_snapshots.py
```

后续发现 phase24c_run1 snapshot density 不足，又新增 RED：

```text
record_post_recovery_snapshots.py missing periodic_active_goal
```

## 3. 实现

新增：

```text
tools/enrich_post_recovery_snapshots.py
```

用法：

```bash
python3 tools/enrich_post_recovery_snapshots.py \
  --snapshots log/phase24c_run1_post_recovery_snapshots.jsonl \
  --nav2-analysis log/phase24c_run1_goal_nav2_analysis.json \
  --output-json log/phase24c_run1_post_recovery_enriched.json
```

输出：

- stdout CSV
- JSON:
  - `summary`
  - `enriched_recovery_snapshots`

核心字段：

- `goal_sequence`
- `outcome`
- `recovery_time`
- `pre_recovery_snapshot`
- `post_recovery_snapshot`
- `near_zero_snapshot`
- `pre_recovery_age_sec`
- `post_recovery_age_sec`
- `near_zero_age_sec`
- `snapshot_density_sufficient`
- `path_update_count_after_recovery`
- `pre/post/near_zero_path_ahead_1_0m_cost_max`
- `pre/post/near_zero_robot_to_path_distance_m`
- `controller_received_path_but_cmd_near_zero`

Density rule：

```text
snapshot_density_sufficient = pre_age <= 2.5s and post_age <= 2.5s
```

## 4. Recorder enhancement

Phase24c_run1 enrichment showed：

```text
with_pre_recovery_count: 7
with_post_recovery_count: 0
sufficient_density_count: 0
needs_periodic_snapshot_count: 7
```

原因：recorder only emitted near-zero/path_update rows, not periodic snapshots during active goals.

因此增强：

```text
tools/record_post_recovery_snapshots.py
```

新增：

- `--periodic-snapshot-sec`
- `periodic_active_goal` snapshots
- `last_periodic_snapshot_time`
- active goal 期间每秒 emit snapshot

测试通过：

```text
src/tugbot_maze/test/test_phase24d_periodic_recorder_contract.py
```

## 5. Verification

Focused：

```text
5 passed in 0.07s
```

Full tests：

```text
80 passed in 0.72s
```

Build：

```text
colcon build --packages-select tugbot_maze --symlink-install
Summary: 1 package finished
```

## 6. phase24c_run1 enrichment before periodic enhancement

Generated：

```text
log/phase24c_run1_post_recovery_enriched.json
```

Summary：

```json
{
  "recovery_count": 7,
  "with_pre_recovery_count": 7,
  "with_post_recovery_count": 0,
  "with_near_zero_count": 7,
  "sufficient_density_count": 0,
  "needs_periodic_snapshot_count": 7,
  "controller_received_path_but_cmd_near_zero_count": 0
}
```

Conclusion：phase24c_run1 snapshots density was insufficient. This justified periodic active-goal snapshots and another smoke.

## 7. Smoke after periodic enhancement: phase24c_run2

Preflight：无真实 ROS/Gazebo/Nav2/recorder 残留。

Run：

```bash
bash tools/run_phase21_controller_diagnostics_smoke.sh phase24c_run2
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
  "exit_distance_m": 0.7567794671891142
}
```

Controller summary：

```json
{
  "goal_count": 12,
  "healthy_motion_but_late_stall_count": 3,
  "late_controller_silent_count": 5,
  "timeout_or_failure_late_stall_count": 4
}
```

Subtype summary：

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
  "severe_footprint_path_ratio": 0.5,
  "unclassified_count": 0
}
```

## 8. phase24c_run2 enriched recovery result

Generated：

```text
log/phase24c_run2_post_recovery_enriched.json
```

Summary：

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

This confirms periodic snapshots solved density.

## 9. Key enriched observations

### Timeout seq 2: side/timing late silent

Recovery 1：

```text
pre_age: 0.093s
post_age: 1.107s
pre path_1m: 84
post path_1m: 84
near_zero path_1m: 99
pre robot_to_path: 0.014m
post robot_to_path: 0.037m
near_zero robot_to_path: 0.085m
path updates after recovery: 20
```

Recovery 2/3：

```text
pre path_1m: 34
post path_1m: 34
near_zero path_1m: 99
robot_to_path remains small: ~0.013-0.085m
path updates after recovery: 6
```

Interpretation：near-zero onset has higher path-ahead cost than immediate recovery pre/post snapshots.

### Timeout seq 8: side/timing late silent

Recovery 1：

```text
pre/post/near_zero path_1m: 0 / 0 / 0
robot_to_path: 0.054 / 0.041 / 0.054m
path updates after recovery: 26
```

Recovery 2/3：

```text
pre/post path_1m: 100 / 100
near_zero path_1m: 0
robot_to_path: ~0.015-0.054m
```

Interpretation：cost state flips over time; near-zero is not always aligned with the highest recovery-time path cost.

### Timeout seq 11: footprint/path blocked late silent

Multiple recoveries：

```text
pre/post path_1m mostly 99-100
near_zero path_1m 99
robot_to_path remains small: ~0.064-0.133m
path updates after recovery: 41 / 21 / 11 ...
```

Interpretation：this is strong footprint/path-cost infeasibility evidence with many path updates and good robot-path alignment.

### Timeout seq 12: footprint/path blocked / late_controller_silent

Multiple recoveries：

```text
pre/post/near_zero path_1m: 100 / 100 / 100
robot_to_path: ~0.092m near-zero
path updates after recovery: 26 / 16 / 6
```

Interpretation：also strong path-cost infeasibility with path updates continuing.

## 10. Main conclusions

### 10.1 Periodic snapshots solved Phase24C limitation

Before：

```text
with_post_recovery_count = 0
sufficient_density_count = 0
```

After phase24c_run2：

```text
with_pre_recovery_count = 16/16
with_post_recovery_count = 16/16
sufficient_density_count = 16/16
```

### 10.2 Robot-to-path distance remains small

Across enriched timeout recoveries, robot-to-path distance remains roughly：

```text
~0.01m - 0.13m
```

So failures are not primarily due to robot being far from active `/plan`.

### 10.3 Controller keeps receiving path updates

For timeout recoveries, path updates after recovery are frequent：

```text
6 - 41 path updates after recovery
```

So failures are not due to lack of path after recovery.

### 10.4 The strongest intervention signal is local path-ahead cost, not path alignment

For footprint/path blocked subtype, pre/post/near-zero path_1m is consistently high：

```text
seq 11 / seq 12: path_1m ≈ 99-100
```

For side/timing subtype, path cost may be low at recovery but high at near-zero, or vice versa. This argues for targeted local-cost/controller critic analysis instead of branch scoring.

## 11. Recommendation for Phase 25

Do not branch score.

Do not globally tune progress checker.

Now there is enough evidence to consider targeted Phase 25 intervention experiments, gated by metrics.

Recommended first intervention candidate：

```text
Phase 25A: local path-cost / footprint-cost relief experiment
```

Scope should be narrow and reversible：

- compare baseline vs one local-cost/controller-related parameter set
- target only footprint/path blocked subtype
- acceptance metrics:
  - lower `footprint_path_blocked_late_silent` count
  - lower timeout count
  - no blocked/blacklist regression
  - no success regression
  - still preserve exit-reaching behavior

Potential intervention areas：

- local costmap inflation / footprint padding
- controller critic cost sensitivity
- local planner behavior near inflated cells
- recovery behavior if post-clear path remains high cost

Keep branch selection unchanged.
