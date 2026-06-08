# Phase 26I — Same-Start Explored Forward-Shift Statistics

Date: 2026-05-25
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 一句话结论

Phase26I 完成 analysis-only forward-shift 统计。Phase26G matched artifacts 中，FAILED_EXHAUSTED candidate runs 的确出现 same-start explored branch target 前推且更靠近 exit 的 case，但全部属于 `high_local_cost`，没有 `clean_low_cost explored closer-to-exit` case。EXIT_REACHED baseline runs 也出现 1 个 high-local-cost explored closer-to-exit case。因此 Phase26I 不支持进入 Phase27；不改 branch selection，不调 Nav2/controller 参数。

## 目标与边界

用户要求 Phase26I 仍保持 analysis-only，聚焦：

1. 统计 “same-start-node explored branch 后续 target 前推并更靠近 exit” 的频率：
   - target forward-shift distance
   - source target_exit_dist vs later explored candidate target_exit_dist
   - angle_delta_rad
2. 分离：
   - clean-low-cost explored closer-to-exit cases
   - high-local-cost explored closer-to-exit cases
3. 对 EXIT_REACHED vs FAILED_EXHAUSTED runs 做对照：
   - 只有 failed runs 稳定出现 clean-low-cost explored-forward-shift，而 successful runs 不出现，才考虑 Phase27。

本阶段没有启动 ROS/Gazebo/Nav2，没有修改 runtime branch choice，没有修改 Nav2 或 controller 参数。

## 新增/修改文件

新增 analyzer：

- `tools/analyze_phase26i_forward_shift_stats.py`

新增 tests：

- `src/tugbot_maze/test/test_phase26i_forward_shift_stats.py`

新增 output artifact：

- `log/phase26i_forward_shift_stats.json`
- `log/phase26i_forward_shift_stats.out`

## TDD 过程

RED：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase26i_forward_shift_stats.py -q
```

预期失败：

- `tools/analyze_phase26i_forward_shift_stats.py` 不存在。
- 2 个 tests fail，证明新 contract 正在约束 Phase26I analyzer。

GREEN：

- 实现 analyzer 后 targeted tests 通过。

```bash
python3 -m py_compile tools/analyze_phase26i_forward_shift_stats.py
python3 -m pytest src/tugbot_maze/test/test_phase26i_forward_shift_stats.py src/tugbot_maze/test/test_phase26h_route_history_audit.py src/tugbot_maze/test/test_phase26f_branch_choice_diagnostics_analysis.py -q
```

结果：`7 passed`。

Phase26 family tests：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase26*.py -q
```

结果：`31 passed`。

## Analyzer 功能

命令：

```bash
python3 tools/analyze_phase26i_forward_shift_stats.py \
  --log-dir log \
  --runs phase26g_baseline_run1,phase26g_baseline_run2,phase26g_candidate_run1,phase26g_candidate_run2 \
  --failed-runs phase26g_candidate_run1,phase26g_candidate_run2 \
  --success-runs phase26g_baseline_run1,phase26g_baseline_run2 \
  --output-json log/phase26i_forward_shift_stats.json
```

Analyzer 输出：

- per-run final_mode。
- per-run forward_shift_cases。
- same-start-node + angle-match explored rejected candidate matching。
- source/later goal_sequence。
- source target vs later explored candidate target。
- target_forward_shift_distance_m。
- source target_exit_dist vs later explored candidate target_exit_dist。
- target_exit_dist_delta_m。
- angle_delta_rad。
- clean_low_cost vs high_local_cost classification。
- FAILED_EXHAUSTED vs EXIT_REACHED aggregate comparison。
- explicit guardrails：
  - `do_not_enter_phase27_from_phase26i_alone`
  - `do_not_change_branch_selection`
  - `do_not_tune_nav2_or_controller_params_from_phase26i`
  - `require_failed_vs_exit_reached_repeat_stability_before_intervention`

## Phase26G matched runs summary

| group | runs | same-start explored forward-shift | explored closer-to-exit | clean-low-cost closer | high-local-cost closer | target_shift median | target_exit_delta median | angle_delta median |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| EXIT_REACHED | 2 | 1 | 1 | 0 | 1 | 0.744682 m | -0.739723 m | 0.091297 rad |
| FAILED_EXHAUSTED | 2 | 3 | 2 | 0 | 2 | 0.693047 m | -0.112792 m | 0.141127 rad |

关键判断：

- FAILED_EXHAUSTED runs：`clean_low_cost_explored_closer_count = 0`
- EXIT_REACHED runs：`clean_low_cost_explored_closer_count = 0`
- FAILED_EXHAUSTED runs：`high_local_cost_explored_closer_count = 2`
- EXIT_REACHED runs：`high_local_cost_explored_closer_count = 1`

因此，用户定义的 Phase27 前置条件不成立：failed runs 没有稳定出现 clean-low-cost explored-forward-shift；successful runs 也不是完全没有 explored closer-to-exit，只是它同样是 high-local-cost。

## Per-run details

### phase26g_baseline_run1 — EXIT_REACHED

- same-start explored forward-shift: 1
- explored closer-to-exit: 1
- clean-low-cost closer: 0
- high-local-cost closer: 1

Case：

- source seq: 6
- later seq: 7
- start_node_id: 6
- source target: `[2.275207, 3.100765]`
- later explored candidate target: `[3.01958, 3.122232]`
- source target_exit_dist: `1.727734 m`
- later candidate target_exit_dist: `0.988010 m`
- target_exit_dist_delta: `-0.739723 m`
- target_forward_shift_distance: `0.744682 m`
- angle_delta: `0.091297 rad`
- cost_classification: `high_local_cost`
- closer_to_exit: true

### phase26g_baseline_run2 — EXIT_REACHED

- same-start explored forward-shift: 0
- explored closer-to-exit: 0
- clean-low-cost closer: 0
- high-local-cost closer: 0

### phase26g_candidate_run1 — FAILED_EXHAUSTED

- same-start explored forward-shift: 1
- explored closer-to-exit: 0
- clean-low-cost closer: 0
- high-local-cost closer: 0

Case：

- source seq: 9
- later seq: 12
- start_node_id: 8
- source target: `[3.406744, 3.226083]`
- later explored candidate target: `[3.468582, 3.661163]`
- source target_exit_dist: `0.634875 m`
- later candidate target_exit_dist: `0.848259 m`
- target_exit_dist_delta: `+0.213383 m`
- target_forward_shift_distance: `0.439453 m`
- angle_delta: `0.141127 rad`
- cost_classification: `high_local_cost`
- closer_to_exit: false

Interpretation：

- 它是 same-start forward-shift，但不是 closer-to-exit。
- 不构成 Phase27 branch-selection intervention evidence。

### phase26g_candidate_run2 — FAILED_EXHAUSTED

- same-start explored forward-shift: 2
- explored closer-to-exit: 2
- clean-low-cost closer: 0
- high-local-cost closer: 2

Case 1：

- source seq: 7
- later seq: 8
- start_node_id: 7
- source target: `[2.996487, 3.176174]`
- later explored candidate target: `[3.547953, 3.668447]`
- source target_exit_dist: `1.018860 m`
- later candidate target_exit_dist: `0.806950 m`
- target_exit_dist_delta: `-0.211911 m`
- target_forward_shift_distance: `0.739221 m`
- angle_delta: `0.058621 rad`
- cost_classification: `high_local_cost`
- closer_to_exit: true

Case 2：

- source seq: 7
- later seq: 10
- start_node_id: 7
- source target: `[2.996487, 3.176174]`
- later explored candidate target: `[3.437795, 3.710553]`
- source target_exit_dist: `1.018860 m`
- later candidate target_exit_dist: `0.906068 m`
- target_exit_dist_delta: `-0.112792 m`
- target_forward_shift_distance: `0.693047 m`
- angle_delta: `0.151474 rad`
- cost_classification: `high_local_cost`
- closer_to_exit: true

Interpretation：

- 这是 Phase26H focus evidence 的扩展：seq=8 不是孤例，同 run 后续 seq=10 也有类似 same-start explored forward-shift closer-to-exit。
- 但两个 case 都是 high-local-cost，而不是 clean-low-cost。
- 所以证据更支持 “near-exit explored-state 与 local-cost/controller pressure 叠加”，而不是 “clean branch 被 explored state 错误抑制导致 failed”。

## Phase27 gate 判定

Phase26I gate：

```json
{
  "stable_clean_low_cost_failed_only_signal": false,
  "phase27_candidate_signal": "not_supported",
  "recommendation": "analysis_only_no_runtime_change"
}
```

不进入 Phase27 的原因：

1. Failed runs 的 same-start explored closer-to-exit 确实存在，但 clean-low-cost count 为 0。
2. Successful runs 也出现 explored closer-to-exit case，虽然同样是 high-local-cost。
3. 当前 pattern 不满足 “failed-only + clean-low-cost + stable” 条件。
4. Phase26H/26I 共同指向 high-local-cost / near-exit / controller stall 与 explored-state 叠加，而不是单纯 branch-history suppression。

## 下阶段建议

推荐进入 Phase26J，仍 analysis-only，不进入 Phase27。

Phase26J 推荐目标：

1. 把 Phase26I 的 same-start explored-forward-shift cases 与 timeout subtype / post-recovery enriched artifacts join：
   - 对每个 later_goal_sequence 标注 timeout/success/terminal outcome。
   - join `*_timeout_subtypes.json`。
   - join `*_post_recovery_enriched.json`。
   - 统计 high-local-cost explored closer-to-exit 是否总是伴随 footprint/path blocked、late-silent、post-recovery near-zero path-ahead high cost。
2. 分离 “explored closer-to-exit 但 chosen target 仍 toward exit” vs “chosen moves away rejected moves toward exit”：
   - Phase26H 已聚焦 route-divergence；Phase26I 统计了 forward-shift；Phase26J 应把二者合并，避免把 harmless explored candidate 当成 failure trigger。
3. 如果 Phase26J 仍显示 evidence 主要是 high-local-cost：
   - 不做 branch-state intervention。
   - 回到 narrow controller/local-cost diagnostics，但仍不直接调参，先确认 near-exit local-cost 的 spatial consistency。
4. 只有当后续新 matched repeats 显示：
   - FAILED_EXHAUSTED 中反复出现 clean-low-cost explored closer-to-exit；
   - EXIT_REACHED 中不出现；
   - 且 later chosen route 明确 moves away / worse exit distance；
   才考虑 Phase27 branch-selection experiment。

## Verification

已完成：

```bash
python3 -m py_compile tools/analyze_phase26i_forward_shift_stats.py
python3 -m pytest src/tugbot_maze/test/test_phase26i_forward_shift_stats.py src/tugbot_maze/test/test_phase26h_route_history_audit.py src/tugbot_maze/test/test_phase26f_branch_choice_diagnostics_analysis.py -q
# 7 passed

python3 -m pytest src/tugbot_maze/test/test_phase26*.py -q
# 31 passed
```

Process cleanup check：

```bash
ps -eo pid,cmd | grep -E 'ros2 launch tugbot_bringup|record_explorer_state_series|record_controller_dynamics|record_post_recovery_snapshots|ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|maze_explorer|slam_toolbox|controller_server|planner_server|bt_navigator|gz sim|ruby .*gz sim' | grep -v grep || true
```

结果：无残留匹配进程。
