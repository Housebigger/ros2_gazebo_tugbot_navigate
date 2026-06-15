# Phase 26H — Near-Exit Route-History / Explored-State Audit

Date: 2026-05-25
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 一句话结论

Phase26H 完成 analysis-only route-history audit。`phase26g_candidate_run2` seq=8 的 rejected `explored` candidate 并不是找不到来源的随机拒绝；它可追溯为同一 start node / branch angle 上的 prior success（seq=7）后被 topology 标记为 explored。该 candidate 确实更靠近 exit，但同时 near-exit、local-cost constrained，并与 seq=8 的 `footprint_path_blocked_late_silent + cmd_silent_after_recovery_abort` timeout 叠加。当前证据仍不足以进入 Phase27，不应改 branch selection，不应调 Nav2/controller 参数。

## 目标与边界

用户要求 Phase26H 仍保持 analysis-only，聚焦：

1. 审计 `phase26g_candidate_run2` seq=8 中 rejected explored candidate：
   - 什么时候被标记 explored？
   - 是真实经过、success、timeout、terminal near-exit，还是 backtrack route 导致？
2. 扩展 analyzer，而不是改 runtime：
   - 分离 `lower_rank_not_selected` / `explored` / `blacklisted`；
   - 对 near-exit + local-cost-constrained + explored candidate 单独计数；
   - join timeout subtype 与 post-recovery enriched artifacts。
3. 如需更高样本量，后续再补 matched repeat；本阶段不补 smoke、不调参。

本阶段没有启动 ROS/Gazebo/Nav2，没有修改 runtime branch choice，没有修改 Nav2 参数。

## 新增/修改文件

新增 analyzer：

- `tools/analyze_phase26h_route_history_audit.py`

新增 tests：

- `src/tugbot_maze/test/test_phase26h_route_history_audit.py`

新增 output artifact：

- `log/phase26h_route_history_audit.json`

## TDD 过程

新增 Phase26H tests 后 RED：

- `tools/analyze_phase26h_route_history_audit.py` 不存在，2 个 tests 失败。

GREEN：

- 实现 analyzer 后，synthetic tests 通过。
- 追加一个 same-start-node / branch-angle provenance test，用于覆盖真实 Phase26G seq=8 这类 target 坐标已前推、但 branch state 来自同一起点同一方向 prior success 的情况。

最终 targeted：

```bash
python3 -m pytest -q src/tugbot_maze/test/test_phase26h_route_history_audit.py -v
```

结果：`3 passed`。

## Analyzer 功能

命令：

```bash
python3 tools/analyze_phase26h_route_history_audit.py \
  --log-dir log \
  --runs phase26g_baseline_run1,phase26g_baseline_run2,phase26g_candidate_run1,phase26g_candidate_run2 \
  --focus-run phase26g_candidate_run2 \
  --focus-goal-sequence 8 \
  --output-json log/phase26h_route_history_audit.json
```

Analyzer 输出：

- per-run rejection reason counts：
  - `lower_rank_not_selected`
  - `explored`
  - `blacklisted`
- per-run near-exit + local-cost + explored candidate count。
- per-run route-divergence explored case count。
- focus audit：
  - explored candidate payload。
  - provenance classification。
  - timeout subtype join。
  - post-recovery enriched join。
- explicit guardrails：
  - `do_not_enter_phase27`
  - `do_not_change_branch_selection`
  - `do_not_tune_nav2_or_controller_params_from_phase26h`
  - `audit_explored_state_provenance_before_runtime_intervention`

## Phase26G matched run audit summary

| run | dispatch | lower_rank_not_selected | explored | blacklisted | near_exit+local_cost+explored | route_divergence+explored |
|---|---:|---:|---:|---:|---:|---:|
| phase26g_baseline_run1 | 8 | 18 | 1 | 0 | 0 | 0 |
| phase26g_baseline_run2 | 11 | 27 | 0 | 0 | 0 | 0 |
| phase26g_candidate_run1 | 11 | 26 | 1 | 0 | 1 | 0 |
| phase26g_candidate_run2 | 12 | 27 | 2 | 0 | 2 | 1 |

Aggregate：

- explored rejected candidates: 4
- near-exit + local-cost-constrained + explored candidates: 3
- route-divergence explored cases: 1
- blacklisted rejected candidates: 0

解释：

- `explored` rejection 不是 candidate-only 独有，baseline_run1 也出现过 1 次。
- 但 near-exit + local-cost-constrained + explored 更集中在 candidate failed runs。
- 唯一 route-divergence+explored case 是 `phase26g_candidate_run2` seq=8。

## Focus audit: phase26g_candidate_run2 seq=8

### Dispatch / route-divergence

- run: `phase26g_candidate_run2`
- goal_sequence: 8
- classification: `chosen_moves_away_rejected_moves_toward_exit`
- chosen target: `[2.774027, 2.218735]`
- chosen target_exit_dist: `1.453748 m`
- chosen exit_progress_delta_m: `-0.289882 m`
- best rejected target: `[3.547953, 3.668447]`
- best rejected target_exit_dist: `0.806950 m`
- best rejected exit_progress_delta_m: `+0.356916 m`
- best rejected rejection_reason: `explored`

### Explored candidate local-cost context

Rejected explored candidate fields：

- `is_near_exit_candidate: true`
- `dispatch_path_local_cost_max: 99`
- `target_local_cost: 99`
- `target_local_cost_max_radius: 99`
- `path_corridor_min_clearance_m: 0.350 m`
- `target_clearance_m: 0.350 m`
- Phase26H classification: `near_exit_local_cost_constrained: true`

Interpretation：

- 它确实更靠近 exit，但不是 clean low-cost candidate。
- 该 candidate 位于 high local-cost / narrow-clearance context。

### Provenance: 什么时候被标记 explored？

Analyzer provenance：

```json
{
  "classification": "prior_success_same_start_node_angle_match",
  "match_basis": "same_start_node_branch_angle",
  "source_goal_sequence": 7,
  "source_outcome": "success",
  "source_result_reason": "succeeded",
  "source_goal_kind": "explore",
  "source_target": [2.996487, 3.176174],
  "source_target_exit_dist": 1.01886044382927,
  "angle_delta_rad": 0.058621,
  "target_match_distance_m": 0.739221
}
```

结论：

- seq=8 的 rejected explored candidate 来自同一 start node / 相近 branch angle 的 prior explore success：seq=7。
- 不是 timeout 导致。
- 不是 terminal near-exit 导致。
- 不是 backtrack route 导致。
- 不是 blacklisted 导致。
- 坐标不完全相同，是因为同一 branch 在后续 dispatch pose 下重新生成了更前方/更靠近 exit 的 centered branch target；但 topology merge 仍按 angle / branch matching 继承了 explored state。

### Timeout subtype join

seq=8 timeout subtype：

- controller_subtype: `footprint_path_blocked_late_silent`
- timing_subtype: `cmd_silent_after_recovery_abort`
- combined_subtype: `footprint_path_blocked_late_silent+cmd_silent_after_recovery_abort`
- classification: `healthy_motion_but_late_stall`
- late_controller_silent: true
- footprint_path_blocked: true
- footprint_cost_max: 99
- footprint_lethal_cell_count: 27
- front_wedge_cost_max: 100
- path_ahead_1_0m_cost_max: 100

Interpretation：

- seq=8 的 failure 不只是 branch-history issue；它同时有强 local footprint/path blocking + late-silent controller pattern。

### Post-recovery enriched join

seq=8 post-recovery：

- snapshot_density_sufficient: true
- pre_recovery_path_ahead_1_0m_cost_max: 0
- post_recovery_path_ahead_1_0m_cost_max: 0
- near_zero_path_ahead_1_0m_cost_max: 99
- near_zero_robot_to_path_distance_m: 0.0227 m
- path_update_count_after_recovery: 25

Interpretation：

- recovery 后仍有 path updates，robot-to-path distance 很小。
- 但 near-zero onset 时 path ahead 又出现 high cost。
- 这更像 near-exit local-cost/controller stall 与 route-history state 叠加，而不是单纯 “没有选更靠近出口的 branch”。

## Phase27 gate 判定

Phase26H 后仍不满足 Phase27 gate：

1. rejected candidate 确实更靠近 exit：成立，但只在一个 route-divergence+explored focus case 中成立。
2. explored provenance 已明确：来自 seq=7 prior success same-start-node angle match。
3. 但该 candidate 是 near-exit + local-cost constrained，高成本不是 clean branch target。
4. seq=8 同时存在 footprint/path blocked late-silent timeout。
5. evidence 仍是单个 route-divergence+explored case，不足以 justify runtime intervention。

因此：

- 不进入 Phase27。
- 不改 branch selection。
- 不调 Nav2/controller 参数。
- 不 promote candidate profile。

## Verification

Targeted tests：

```bash
python3 -m pytest -q \
  src/tugbot_maze/test/test_phase26h_route_history_audit.py \
  src/tugbot_maze/test/test_phase26g_matched_branch_diagnostics_repeats.py \
  src/tugbot_maze/test/test_phase26f_branch_choice_diagnostics_analysis.py -v
```

Result：`9 passed`。

Static：

```bash
python3 -m compileall -q \
  tools/analyze_phase26h_route_history_audit.py \
  src/tugbot_maze/test/test_phase26h_route_history_audit.py
python3 /home/hyh/.hermes/skills/software-development/ros2-gazebo-navigation-workspaces/scripts/check_ros2_workspace_contracts.py .
```

Result：ROS 2 workspace static contracts passed。

Build/test：

```bash
colcon build --symlink-install
colcon test --event-handlers console_direct+ --packages-select tugbot_maze tugbot_bringup
colcon test-result --verbose
```

Result：

- build: 6 packages finished。
- tests: `129 tests, 0 errors, 0 failures, 0 skipped`。

Cleanup：

- ROS/Gazebo/Nav2/recorder process grep: no output。

## 下一阶段建议

推荐 Phase26I，仍 analysis-only，不做 runtime intervention。

Phase26I 建议目标：

1. 扩展 route-history audit 到 “same-start-node angle explored later target moved closer exit” 的统计：
   - 统计 explored candidate target 相比 source success target 是否更靠近 exit；
   - 统计 target forward-shift distance；
   - 分离 clean-low-cost explored vs high-local-cost explored。
2. 加入 per-dispatch start_node/branch-angle provenance summary 到 analyzer 输出，减少人工解释。
3. 如果要补样本，只补 1 baseline + 1 candidate repeat，继续用 Phase26E diagnostics，不调参。
4. 只有当多次 failed runs 稳定显示：
   - same-start-node explored target 前推后明显更靠近 exit；
   - local-cost 不高；
   - timeout subtype 不被 footprint/path blocked 主导；
   - EXIT_REACHED runs 不出现相同模式；
   才考虑 Phase27 做窄范围、可回滚的 explored-branch near-exit exception。

当前仍不建议：

- 直接给 explored branch near-exit exception。
- 直接修改 branch scoring。
- 直接放宽 explored state。
- 盲测新 CostCritic 权重。
