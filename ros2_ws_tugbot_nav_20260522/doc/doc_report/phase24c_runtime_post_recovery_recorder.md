# Phase 24C Runtime Post-Recovery Recorder Report

生成时间：2026-05-25

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 24C 目标：新增 runtime post-recovery recorder，在不改变导航行为的前提下，记录 post-recovery path/local-cost/pose/cmd alignment。

用户指定需订阅或推导：

```text
/local_costmap/costmap
/plan 或 Nav2 实际发布的 path topic
/odom
/cmd_vel_nav
/maze/goal_events
```

需要输出 per-goal snapshots：

```text
clear-costmap recovery 前 local cost/path snapshot
clear-costmap recovery 后 local cost/path snapshot
recovery 时 active path-ahead cost
near-zero onset 时 active path-ahead cost
robot pose 到 active path 最近点距离
recovery 后 path update count/time
controller 收到 path 但 cmd 仍 near-zero 的 interval
```

## 2. Discovery

定位到：

- `tools/run_phase21_controller_diagnostics_smoke.sh` 是当前 Phase 21-24 smoke wrapper。
- Nav2 config 中没有显式 path topic 参数，但常用 plan topic 可先试 `/plan`。
- 当前 package script 工具不通过 setup.py 安装，wrapper 直接 `python3 tools/*.py` 调用。

## 3. TDD

新增测试：

```text
src/tugbot_maze/test/test_phase24c_post_recovery_recorder_contract.py
src/tugbot_maze/test/test_phase24c_post_recovery_snapshot_summary.py
```

RED 已确认：

```text
3 failed
- record_post_recovery_snapshots.py missing
- summarize_post_recovery_snapshots.py missing
- smoke wrapper did not include POST_RECOVERY_SNAPSHOTS / recorder invocation
```

## 4. 实现

新增：

```text
tools/record_post_recovery_snapshots.py
tools/summarize_post_recovery_snapshots.py
```

修改：

```text
tools/run_phase21_controller_diagnostics_smoke.sh
```

### 4.1 Recorder

`record_post_recovery_snapshots.py` subscribes：

- `/local_costmap/costmap`
- `/plan`
- `/odom`
- `cmd_vel_nav`
- `/maze/goal_events`

输出 JSONL：

```text
log/${RUN_ID}_post_recovery_snapshots.jsonl
```

核心 fields：

- `event`
- `snapshot_type`
- `goal_sequence`
- `wall_time`
- `has_local_costmap`
- `has_path`
- `has_odom`
- `path_point_count`
- `robot_pose`
- `robot_to_path_distance_m`
- `path_ahead_0_5m_cost_max/mean`
- `path_ahead_1_0m_cost_max/mean`
- `post_recovery_path_update_count`
- `controller_received_path_but_cmd_near_zero`
- `last_cmd`

Important limitation：当前 recorder 只能可靠输出 `near_zero_onset` 和 `path_update` runtime snapshots。`pre_recovery` / `post_recovery` runtime trigger 还没有和 Nav2 clear-costmap event stream 实时对齐，因为 clear-costmap events 目前只在 launch log 中，非 ROS topic。

### 4.2 Summary

`summarize_post_recovery_snapshots.py` converts JSONL to per-goal JSON：

```text
log/${RUN_ID}_post_recovery_snapshots_summary.json
```

并输出 CSV columns：

- `goal_sequence`
- `pre_recovery_path_ahead_1_0m_cost_max`
- `post_recovery_path_ahead_1_0m_cost_max`
- `near_zero_path_ahead_1_0m_cost_max`
- `near_zero_robot_to_path_distance_m`
- `controller_received_path_but_cmd_near_zero`
- `post_recovery_path_update_count`

### 4.3 Smoke wrapper integration

Wrapper 新增：

- supports `phase24c_runN`
- `POST_RECOVERY_SNAPSHOTS`
- `POST_RECOVERY_SUMMARY`
- starts `record_post_recovery_snapshots.py`
- cleanup includes recorder
- post-run calls `summarize_post_recovery_snapshots.py`

## 5. Verification

Focused：

```text
5 passed in 0.16s
```

Full tests：

```text
78 passed in 0.67s
```

Build：

```text
colcon build --packages-select tugbot_maze --symlink-install
Summary: 1 package finished
```

## 6. Smoke: phase24c_run1

Preflight：无真实 ROS/Gazebo/Nav2/recorder 残留。

Run：

```bash
bash tools/run_phase21_controller_diagnostics_smoke.sh phase24c_run1
```

Wrapper exit code：`0`

Final summary：

```json
{
  "final_mode": "EXIT_REACHED",
  "goal_count": 8,
  "goal_success_count": 5,
  "goal_failure_count": 2,
  "timeout_cancel_count": 2,
  "blocked_branch_count": 0,
  "blacklisted_goal_count": 0,
  "exit_distance_m": 0.5773163347179346
}
```

Nav2 summary：

```json
{
  "goal_count": 8,
  "success_count": 5,
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
  "goal_count": 8,
  "healthy_motion_but_late_stall_count": 1,
  "healthy_motion_but_timed_out_count": 1,
  "late_controller_silent_count": 1,
  "terminal_cancel_after_exit_count": 1
}
```

## 7. Phase24C recorder coverage

Generated：

```text
log/phase24c_run1_post_recovery_snapshots.jsonl
log/phase24c_run1_post_recovery_snapshots_summary.json
```

Sizes：

```text
phase24c_run1_post_recovery_snapshots.jsonl: 33992 bytes
phase24c_run1_post_recovery_snapshots_summary.json: 3030 bytes
```

Event counts：

```text
goal_event/dispatch: 8
path_update/path_update: 127
snapshot/near_zero_onset: 8
goal_event/success: 5
goal_event/timeout: 2
goal_event/terminal_cancel: 1
```

Snapshot summary：

```json
{
  "goal_count": 8,
  "with_near_zero_onset_count": 8,
  "with_robot_to_path_distance_count": 8,
  "with_pre_recovery_count": 0,
  "with_post_recovery_count": 0,
  "controller_received_path_but_cmd_near_zero_count": 0
}
```

Per-goal near-zero snapshots：

```text
seq 1: near_zero path_1m=100, robot_to_path=0.039, path_updates=2
seq 2: near_zero path_1m=100, robot_to_path=0.098, path_updates=3
seq 3: near_zero path_1m=0,   robot_to_path=0.024, path_updates=5
seq 4: near_zero path_1m=0,   robot_to_path=0.049, path_updates=2
seq 5: near_zero path_1m=0,   robot_to_path=0.026, path_updates=4
seq 6: near_zero path_1m=0,   robot_to_path=0.016, path_updates=3
seq 7: near_zero path_1m=0,   robot_to_path=0.035, path_updates=10
seq 8: near_zero path_1m=99,  robot_to_path=0.072, path_updates=3
```

Interpretation：

- `/plan` subscription is working: 127 path updates recorded.
- `/local_costmap/costmap` + `/odom` are working: near-zero snapshots contain path-ahead cost and robot-to-path distance.
- Robot-to-path distance is small in all near-zero snapshots: about 0.016 - 0.098 m.
- Near-zero path-ahead cost varies strongly by goal.

## 8. Failure-window / subtype analysis for phase24c_run1

Generated additionally：

```text
log/phase24c_run1_failure_windows.json
log/phase24c_run1_timeout_subtypes.json
```

Failure windows：

```text
seq 2: timeout, healthy_motion_but_timed_out
  timeout path_1m=36
  footprint=73
  near_zero_start=1779684738.294414

seq 7: timeout, healthy_motion_but_late_stall
  timeout path_1m=100
  footprint=54
  near_zero_start=1779684807.867866
```

Subtype：

```text
seq 7 -> side_cost_or_timing_late_silent + cmd_silent_after_recovery_abort
```

Note：phase24c_run1 reached exit, with only one late-silent timeout. This run is useful for recorder coverage, but not enough for intervention decision.

## 9. Important limitation discovered

The recorder currently does not produce `pre_recovery` / `post_recovery` snapshots in real smoke：

```text
with_pre_recovery_count: 0
with_post_recovery_count: 0
```

Reason：clear-costmap recovery events are currently detected from launch logs after the run, not available as a live ROS topic to the recorder.

The recorder can observe:

- local costmap
- path updates
- odom
- cmd_vel
- goal events
- near-zero onset

But it cannot yet trigger exactly at clear-costmap recovery time without either:

1. runtime log tail / parser integration, or
2. publishing recovery events as ROS diagnostic topic, or
3. post-run enrichment: align recorder’s continuous/path/near-zero samples with Nav2 log recovery timestamps.

## 10. Phase 24C conclusion

Phase 24C successfully added and validated the runtime recorder foundation.

Confirmed working：

```text
/plan path updates
/local_costmap/costmap path-ahead cost sampling
/odom robot-to-path distance
/cmd_vel_nav near-zero onset
/maze/goal_events goal grouping
```

New evidence from phase24c_run1：

- robot remains close to path at near-zero onset:
  - 0.016 - 0.098 m
- near-zero can happen with path-ahead high cost or zero cost depending on goal
- phase24c late-silent timeout seq 7 had:
  - timeout path_1m=100
  - near-zero snapshot path_1m=0
  - robot_to_path=0.035

This discrepancy suggests timing matters strongly: timeout-side snapshot and near-zero-onset snapshot can differ.

## 11. Recommendation for next phase

Do not perform intervention yet.

Recommended Phase 24D：post-run enrichment of runtime snapshots with Nav2 recovery timestamps.

Goal：use existing runtime recorder data plus launch-log-derived recovery timestamps to reconstruct:

```text
pre_recovery snapshot
post_recovery snapshot
recovery-time path-ahead cost
recovery-time robot-to-path distance
near-zero snapshot already available
```

Implementation direction：

- add `tools/enrich_post_recovery_snapshots.py`
- input:
  - `*_post_recovery_snapshots.jsonl`
  - `*_goal_nav2_analysis.json`
- for each goal/recovery time:
  - choose nearest snapshot/path_update/near-zero before and after recovery
  - output per-goal enriched pre/post recovery alignment
- if raw snapshot density is insufficient, adjust recorder to emit periodic snapshots during active goal, not only near-zero onset/path-update metadata.

Only after Phase 24D should Phase 25 choose targeted interventions.
