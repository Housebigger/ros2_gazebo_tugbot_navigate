# Phase 26J — Forward-Shift Artifact Join / Failure-Window Context

Date: 2026-05-25
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 一句话结论

Phase26J 完成 analysis-only artifact join。Phase26I 发现的 high-local-cost explored closer-to-exit cases 中，只有 `phase26g_candidate_run2` seq=8 同时满足 route-divergence、timeout、footprint/path blocked、late-silent、near-zero path-ahead high cost；其余 high-local-cost forward-shift cases 是 non-divergent / success context。没有任何 clean-low-cost route-divergence candidate，因此不进入 Phase27，不改 branch selection，不调 Nav2/controller 参数。

## 目标与边界

用户要求 Phase26J 仍保持 analysis-only，不进入 Phase27，聚焦：

1. 对每个 Phase26I forward-shift later_goal_sequence 标注 timeout/success/terminal outcome。
2. join `*_timeout_subtypes.json`。
3. join `*_post_recovery_enriched.json`。
4. 判断 high-local-cost explored closer-to-exit 是否稳定伴随 footprint/path blocked、late-silent、post-recovery near-zero path-ahead high cost。
5. 分离 harmless explored closer candidate 和真正 `chosen moves away rejected moves toward exit` route-divergence case。

本阶段没有启动 ROS/Gazebo/Nav2，没有修改 runtime branch choice，没有修改 Nav2 或 controller 参数。

## 新增/修改文件

新增 analyzer：

- `tools/analyze_phase26j_forward_shift_artifact_join.py`

新增 tests：

- `src/tugbot_maze/test/test_phase26j_forward_shift_artifact_join.py`

新增 output artifact：

- `log/phase26j_forward_shift_artifact_join.json`
- `log/phase26j_forward_shift_artifact_join.out`

新增报告：

- `doc/doc_report/phase26j_forward_shift_artifact_join.md`

## TDD 过程

RED：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase26j_forward_shift_artifact_join.py -q
```

预期失败：

- `tools/analyze_phase26j_forward_shift_artifact_join.py` 不存在。
- 2 个 tests fail，证明新 Phase26J contract 正在约束 analyzer。

GREEN：

实现 analyzer 后 targeted tests 通过：

```bash
python3 -m py_compile tools/analyze_phase26j_forward_shift_artifact_join.py
python3 -m pytest src/tugbot_maze/test/test_phase26j_forward_shift_artifact_join.py src/tugbot_maze/test/test_phase26i_forward_shift_stats.py src/tugbot_maze/test/test_phase26h_route_history_audit.py -q
```

结果：`7 passed`。

Phase26 family tests：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase26*.py -q
```

结果：`33 passed`。

## Analyzer 功能

命令：

```bash
python3 tools/analyze_phase26j_forward_shift_artifact_join.py \
  --log-dir log \
  --runs phase26g_baseline_run1,phase26g_baseline_run2,phase26g_candidate_run1,phase26g_candidate_run2 \
  --failed-runs phase26g_candidate_run1,phase26g_candidate_run2 \
  --success-runs phase26g_baseline_run1,phase26g_baseline_run2 \
  --output-json log/phase26j_forward_shift_artifact_join.json
```

Analyzer 输出：

- per-run final_mode。
- per-case source/later goal sequence。
- later outcome：success / timeout / terminal_cancel / unknown。
- timeout subtype join：controller_subtype、timing_subtype、combined_subtype、classification、reasons。
- post-recovery enriched join：pre/post/near-zero path-ahead cost、robot-to-path distance、path update count。
- failure window flags：
  - `footprint_path_blocked`
  - `late_silent`
  - `near_zero_path_ahead_high_cost`
  - `pre_recovery_path_ahead_high_cost`
  - `post_recovery_path_ahead_high_cost`
  - `path_updates_after_recovery`
- route_context：
  - `route_divergence_chosen_moves_away_rejected_moves_toward_exit`
  - `non_divergent_chosen_also_toward_exit`
  - `non_divergent_or_unknown_chosen_progress`
- evidence_class：
  - `clean_route_divergence_candidate`
  - `route_divergence_high_cost_timeout_failure_window`
  - `route_divergence_high_cost_without_timeout_failure_window`
  - `harmless_or_non_divergent_high_cost`
  - `harmless_or_non_divergent_clean`

## Matched group summary

| group | same-start forward-shift | closer-to-exit | clean route-divergence | high-cost route-divergence + timeout failure-window | harmless/non-divergent high-cost | high-cost closer + footprint/path blocked | high-cost closer + near-zero path high-cost |
|---|---:|---:|---:|---:|---:|---:|---:|
| EXIT_REACHED | 1 | 1 | 0 | 0 | 1 | 0 | 0 |
| FAILED_EXHAUSTED | 3 | 2 | 0 | 1 | 2 | 1 | 2 |

Decision：

```json
{
  "phase27_candidate_signal": "not_supported",
  "high_cost_failure_window_overlap_in_failed_runs": true,
  "clean_route_divergence_failed_only_signal": false,
  "recommendation": "analysis_only_no_phase27_no_runtime_change"
}
```

## Per-case interpretation

### phase26g_baseline_run1 seq=7 — EXIT_REACHED / harmless high-cost

- source seq: 6
- later seq: 7
- later outcome: success
- start_node_id: 6
- target_exit_dist_delta: `-0.739723 m`
- target_forward_shift_distance: `0.744682 m`
- angle_delta: `0.091297 rad`
- chosen_exit_progress_delta: `+0.129112 m`
- cost_classification: `high_local_cost`
- route_context: `non_divergent_chosen_also_toward_exit`
- evidence_class: `harmless_or_non_divergent_high_cost`
- timeout_subtype: none
- post_recovery:
  - snapshot_density_sufficient: true
  - pre/post/near_zero path-ahead 1.0m max: 0/0/0
  - path_update_count_after_recovery: 12

Interpretation：

- 这是 successful run 中的 explored closer-to-exit case。
- 但是 chosen target 也 toward exit，later outcome success。
- 因此它不是 route-divergence failure trigger。

### phase26g_candidate_run1 seq=12 — FAILED_EXHAUSTED / not closer

- source seq: 9
- later seq: 12
- later outcome: success
- start_node_id: 8
- target_exit_dist_delta: `+0.213383 m`
- target_forward_shift_distance: `0.439453 m`
- angle_delta: `0.141127 rad`
- chosen_exit_progress_delta: `-0.007344 m`
- cost_classification: `high_local_cost`
- closer_to_exit: false
- route_context: `non_divergent_or_unknown_chosen_progress`
- evidence_class: `harmless_or_non_divergent_high_cost`
- post_recovery pre/post/near_zero path-ahead high cost: 100/100/100

Interpretation：

- 虽然有 same-start forward-shift，但 candidate 不是 closer-to-exit。
- later outcome 是 success，不是 timeout。
- 不构成 Phase27 evidence。

### phase26g_candidate_run2 seq=8 — FAILED_EXHAUSTED / true route-divergence high-cost timeout failure-window

- source seq: 7
- later seq: 8
- later outcome: timeout
- start_node_id: 7
- target_exit_dist_delta: `-0.211911 m`
- target_forward_shift_distance: `0.739221 m`
- angle_delta: `0.058621 rad`
- chosen_exit_progress_delta: `-0.289882 m`
- cost_classification: `high_local_cost`
- route_context: `route_divergence_chosen_moves_away_rejected_moves_toward_exit`
- evidence_class: `route_divergence_high_cost_timeout_failure_window`
- timeout_subtype:
  - classification: `healthy_motion_but_late_stall`
  - controller_subtype: `footprint_path_blocked_late_silent`
  - timing_subtype: `cmd_silent_after_recovery_abort`
  - combined_subtype: `footprint_path_blocked_late_silent+cmd_silent_after_recovery_abort`
- post_recovery:
  - snapshot_density_sufficient: true
  - pre_recovery_path_ahead_1_0m_cost_max: 0
  - post_recovery_path_ahead_1_0m_cost_max: 0
  - near_zero_path_ahead_1_0m_cost_max: 99
  - path_update_count_after_recovery: 25
- failure flags:
  - footprint_path_blocked: true
  - late_silent: true
  - near_zero_path_ahead_high_cost: true
  - path_updates_after_recovery: true

Interpretation：

- 这是唯一真正 route-divergence + explored closer-to-exit + timeout case。
- 但它不是 clean-low-cost；它同时伴随 high local cost、footprint/path blocked、late-silent、near-zero path-ahead high cost。
- 因此仍不能证明 “clean branch 被 explored state 错误抑制导致 failed”。

### phase26g_candidate_run2 seq=10 — FAILED_EXHAUSTED / non-divergent high-cost success

- source seq: 7
- later seq: 10
- later outcome: success
- start_node_id: 7
- target_exit_dist_delta: `-0.112792 m`
- target_forward_shift_distance: `0.693047 m`
- angle_delta: `0.151474 rad`
- chosen_exit_progress_delta: `+0.185683 m`
- cost_classification: `high_local_cost`
- route_context: `non_divergent_chosen_also_toward_exit`
- evidence_class: `harmless_or_non_divergent_high_cost`
- post_recovery pre/post/near_zero path-ahead high cost: 100/100/100

Interpretation：

- 它是 explored closer-to-exit，但 chosen target 也 toward exit，later outcome success。
- 说明 explored closer candidate 本身并不必然触发 failure。

## Phase27 gate 判定

Phase26J 后仍不满足 Phase27 gate：

1. Clean-low-cost route-divergence candidate count：0。
2. Failed-only clean route-divergence signal：false。
3. 唯一 route-divergence failure case 是 high-cost timeout failure-window。
4. EXIT_REACHED 中也有 harmless high-cost explored closer-to-exit case。
5. Candidate_run2 中另一个 explored closer-to-exit case seq=10 是 non-divergent success。

因此：

- 不进入 Phase27。
- 不改 branch selection。
- 不调 Nav2/controller params。

## 下阶段建议

推荐进入 Phase26K，仍 analysis-only。

Phase26K 推荐目标：

1. 聚焦 high-cost route-divergence timeout failure-window，而不是 branch-state intervention：
   - 以 `phase26g_candidate_run2` seq=8 为 anchor；
   - 对比同 run seq=10（high-cost explored closer but success）与 baseline_run1 seq=7（EXIT_REACHED harmless high-cost success）。
2. 做 spatial/local-cost alignment：
   - 比较 dispatch target、chosen path、explored candidate target、near-zero robot pose、path-ahead high-cost segment 的空间关系。
   - 判断 seq=8 的 near-zero high-cost 是否位于 chosen-away route、explored candidate corridor、还是 shared near-exit choke point。
3. 如果 high-cost choke point 是 shared：继续 local-cost/controller diagnostics。
4. 如果 high-cost 只出现在 chosen-away route，而 explored candidate corridor clean：再考虑补充 runtime diagnostics；仍不直接 Phase27。
5. 只有当后续 matched repeats 出现 clean-low-cost failed-only route-divergence，才重新打开 Phase27 branch-selection experiment。

## Verification

已完成：

```bash
python3 -m py_compile tools/analyze_phase26j_forward_shift_artifact_join.py
python3 -m pytest src/tugbot_maze/test/test_phase26j_forward_shift_artifact_join.py src/tugbot_maze/test/test_phase26i_forward_shift_stats.py src/tugbot_maze/test/test_phase26h_route_history_audit.py -q
# 7 passed

python3 -m pytest src/tugbot_maze/test/test_phase26*.py -q
# 33 passed
```

Process cleanup check：

```bash
ps -eo pid,cmd | grep -E 'ros2 launch tugbot_bringup|record_explorer_state_series|record_controller_dynamics|record_post_recovery_snapshots|ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|maze_explorer|slam_toolbox|controller_server|planner_server|bt_navigator|gz sim|ruby .*gz sim' | grep -v grep || true
```

结果：无残留匹配进程。
