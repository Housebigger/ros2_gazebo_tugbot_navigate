# Phase 26D Branch-choice Snapshot Alignment

日期：2026-05-25

## 一句话结论

Phase 26D 未启动任何新调参 run，只用 Phase26B 的 `goal_events` 做 branch-choice snapshot alignment。结论是：`phase26b_candidate_run2` 在 seq9 仍沿 exit-progress 方向推进，但 seq10 选择了一个 exit-neutral / slightly-away 的分支并 timeout；随后 seq11 选择了一个“成功但远离出口”的分支，robot_exit_dist 从 0.759m 对应的 near-exit 区域被带向 target_exit_dist 1.222m，最终没有触发 EXIT_REACHED。现有 artifacts 能证明 seq11 branch moved away from exit，但不能解释“为什么它在 3 个候选分支中被选中”，因为缺少 chosen_branch_rank、rejected_branch_summary、candidate branch target_exit_dist/clearance/local-cost/reverse-backtrack context。

补充成功判据：本工程运行成功的标志是小车到达出口坐标；不要求探索完整迷宫/完整地图。因此 Phase26D 的评价重点不是 coverage，而是 branch choice 是否让机器人进入 exit trigger region。

## 本阶段范围

用户要求：

- 进入 Phase 26D：branch-choice snapshot alignment。
- 解释：
  - seq10 timeout 后为什么 seq11 选择了远离 exit 的 successful branch。
  - candidate_run2 的 branch choice 与 candidate_run1/3 的 near-exit terminal path 差异在哪里。
- 先做 post-run analyzer：
  - `tools/analyze_phase26d_branch_choice_snapshots.py`
  - `log/phase26d_branch_choice_alignment.json`
  - `doc/doc_report/phase26d_branch_choice_snapshot_alignment.md`
- 若现有 artifacts 不足，再给 `maze_explorer` goal event 增加 runtime diagnostics。

## 新增文件

- `src/tugbot_maze/test/test_phase26d_branch_choice_snapshots.py`
  - TDD contract test。
  - 验证 analyzer 明确 success condition 是 reaching exit coordinates，不要求 full-map exploration。
  - 验证 analyzer 能识别 seq10 -> seq11 moved-away branch 与 missing candidate ranking diagnostics。
- `tools/analyze_phase26d_branch_choice_snapshots.py`
  - Phase26D post-run analyzer。
  - 只读取 `*_goal_events.jsonl`，不启动 ROS/Gazebo。
  - 输出 branch-choice alignment、exit-progress delta、artifact gaps。
- `log/phase26d_branch_choice_alignment.json`
  - 用 Phase26B candidate 3-runs 生成的 branch-choice alignment。

## TDD 与验证

RED：先写测试，运行后失败，原因符合预期：

```text
tools/analyze_phase26d_branch_choice_snapshots.py: No such file or directory
```

GREEN/verification：实现 analyzer 后通过：

```bash
python3 -m py_compile tools/analyze_phase26d_branch_choice_snapshots.py
python3 -m pytest -q src/tugbot_maze/test/test_phase26c_variance_trigger_alignment.py src/tugbot_maze/test/test_phase26d_branch_choice_snapshots.py
python3 -m pytest -q src/tugbot_maze/test src/tugbot_bringup/test
```

结果：

- Phase26C/D targeted tests：`2 passed`
- package pytest：`138 passed`

未启动任何新 smoke / tuning run。

## Success definition

Analyzer 输出固定记录：

```json
{
  "success_condition": "robot_reaches_exit_coordinates",
  "full_map_exploration_required": false,
  "exit_radius_m": 0.6
}
```

这会影响 Phase26D 解读：

- 如果 robot 已进入 exit trigger region，哪怕后续目标 target_exit_dist 变大，也可能已经成功。
- `terminal_cancel result_reason=exit_reached` 是成功信号。
- 不应把“没有探索完整地图”视为失败。

## Candidate successful runs 的 near-exit path

### phase26b_candidate_run1

Late branch-choice sequence：

| seq | event | classification | robot_exit_dist | target_exit_dist | delta(robot-target) | branch_angle | candidates | topology | dispatch_cost |
|---:|---|---|---:|---:|---:|---:|---:|---|---:|
| 4 | timeout | exit_progress_branch | 4.142 | 3.570 | 0.572 | 0.218 | 4 | junction | 41 |
| 5 | success | exit_progress_branch | 3.585 | 2.712 | 0.874 | 0.217 | 4 | junction | 0 |
| 6 | success | exit_progress_branch | 2.840 | 1.990 | 0.850 | 0.366 | 4 | junction | 0 |
| 7 | success | exit_progress_branch | 2.122 | 1.630 | 0.492 | -1.228 | 4 | junction | 46 |
| 8 | timeout | target_inside_exit_radius | 1.381 | 0.565 | 0.815 | 0.325 | 3 | junction | 73 |
| 9 | terminal_cancel | exit_reached_terminal_success | 0.647 | 1.142 | -0.495 | 1.895 | 2 | corridor | 84 |

Interpretation：candidate_run1 的 seq8 目标已经在 exit radius 内（target_exit_dist 0.565 <= 0.6），虽然 seq8 timeout，但 robot 已接近出口，seq9 terminal_cancel 触发 EXIT_REACHED。注意 seq9 的 target_exit_dist 变大并不代表失败，因为 success condition 是 robot 到达出口坐标，而不是目标点更接近出口或地图探索完成。

### phase26b_candidate_run3

Late branch-choice sequence：

| seq | event | classification | robot_exit_dist | target_exit_dist | delta(robot-target) | branch_angle | candidates | topology | dispatch_cost |
|---:|---|---|---:|---:|---:|---:|---:|---|---:|
| 5 | success | exit_progress_branch | 3.323 | 2.445 | 0.878 | 0.230 | 4 | junction | 0 |
| 6 | success | exit_progress_branch | 2.573 | 1.705 | 0.868 | 0.232 | 4 | junction | 0 |
| 7 | success | exit_progress_branch | 1.825 | 0.968 | 0.857 | 0.167 | 4 | junction | 0 |
| 8 | timeout | exit_progress_branch | 1.083 | 0.769 | 0.314 | 0.088 | 4 | junction | 99 |
| 9 | timeout | branch_moves_away_from_exit | 0.613 | 1.135 | -0.522 | -1.483 | 3 | junction | 73 |
| 10 | terminal_cancel | exit_reached_terminal_success | 0.838 | 1.033 | -0.194 | 1.659 | 2 | corridor | 73 |

Interpretation：candidate_run3 也出现 moved-away branch，但 robot 已经足够接近出口；因此即使后续 branch target_exit_dist 变大，terminal monitor 仍触发 EXIT_REACHED。再次说明：成功不是探索完地图，而是 robot 到达 exit coordinates。

## Focus：phase26b_candidate_run2 seq10 -> seq11

Analyzer diagnosis：

```text
post_timeout_branch_choice_moved_away_from_exit_but_candidate_ranking_missing
```

关键 sequence：

| seq | event | classification | robot_exit_dist | target_exit_dist | delta(robot-target) | branch_angle | candidates | topology | dispatch_cost |
|---:|---|---|---:|---:|---:|---:|---:|---|---:|
| 7 | success | exit_progress_branch | 3.103 | 2.263 | 0.840 | 0.321 | 4 | junction | 0 |
| 8 | success | exit_progress_branch | 2.387 | 1.595 | 0.793 | 0.309 | 4 | junction | 0 |
| 9 | success | exit_progress_branch | 1.713 | 1.019 | 0.694 | 0.259 | 4 | junction | 0 |
| 10 | timeout | exit_neutral_branch | 1.093 | 1.134 | -0.042 | -1.391 | 3 | junction | 0 |
| 11 | success | successful_branch_moved_away_from_exit | 0.759 | 1.222 | -0.463 | 1.749 | 3 | junction | 48 |
| 12 | success | exit_neutral_branch | 1.291 | 1.208 | 0.083 | -1.183 | 4 | junction | 0 |

解释：

1. seq7/8/9 都是 exit-progress branch。
   - robot_exit_dist 持续下降：3.103 -> 2.387 -> 1.713。
   - target_exit_dist 也持续接近 exit：2.263 -> 1.595 -> 1.019。

2. seq10 开始分叉。
   - robot_exit_dist 1.093，target_exit_dist 1.134。
   - exit_progress_delta = -0.042，属于 exit-neutral / slightly-away。
   - branch_angle = -1.391，方向明显与前面 exit-progress branches 不同。
   - seq10 timeout 且 Phase26C 已确认存在 footprint/path blocked late-silent + persistent local-cost pressure。

3. seq11 是关键 moved-away success。
   - robot_exit_dist 0.759，但 target_exit_dist 1.222。
   - exit_progress_delta = -0.463。
   - classification = `successful_branch_moved_away_from_exit`。
   - 这说明 Nav2 成功执行了一个远离 exit 的目标。
   - 因为工程成功条件是到达出口坐标，seq11 的“导航成功”不能等同于“工程成功”。它反而把 robot 带离 exit trigger region。

4. seq12 没有恢复到 near-exit terminal path。
   - robot_exit_dist 1.291，target_exit_dist 1.208。
   - exit_progress_delta = 0.083，仅 exit-neutral。
   - max_goals 用尽后仍未 EXIT_REACHED。

## 为什么 seq11 会被选中？

现有 artifacts 能回答“seq11 选了什么、结果怎样”，但不能回答“为什么它在候选分支中排名最高”。缺失字段：

- `chosen_branch_rank`
- `rejected_branch_summary`
- `candidate_branch_target_exit_distances`
- `candidate_branch_local_path_costs`
- `reverse_backtrack_context`

因此 Phase26D 的严格结论是：

- 可以证明 seq11 是 moved-away successful branch。
- 可以证明 candidate_run1/3 虽然也有 moved-away branch，但在已经足够接近 exit 的情况下触发了 EXIT_REACHED。
- 不能仅凭现有 artifacts 证明 DFS 为什么在 seq10 timeout 后选择 seq11，而不是选择更接近 exit 的替代分支。

## Candidate_run2 与 candidate_run1/3 的本质差异

1. candidate_run1/3 的 moved-away branch 发生时，robot 已经接近或触发 exit region。
   - run1 seq8 target_inside_exit_radius，seq9 terminal success。
   - run3 seq8/9 已非常接近 exit，seq10 terminal success。

2. candidate_run2 的 moved-away branch 发生在尚未触发 exit 的状态。
   - seq10 timeout 后，robot_exit_dist 约 1.09m。
   - seq11 moved-away success 后，target_exit_dist 1.222m。
   - seq12 仍没有进入 exit trigger region。

3. 因此 candidate_run2 的失败不是“探索不完整”，而是“未能在 goal budget 内到达 exit coordinates”。

## Decision

- 不 promote `CostCritic.cost_weight=2.75`。
- 不 reject `2.75` solely from candidate_run2。
- 不盲测 `2.65/2.6`。
- 不启用 Phase25A local inflation relief。
- 不做 broad branch scoring。
- 进入 runtime diagnostics 前，先承认现有 artifacts 不足以解释 branch ranking。

## 下一步建议：Phase 26E runtime branch-choice diagnostics

建议 Phase 26E 做最小 runtime instrumentation，而不是策略改动：在 `maze_explorer` 的 goal event 中增加 branch-choice diagnostics。

建议新增字段：

- `chosen_branch_rank`
- `chosen_branch_score_components`，仅 diagnostics，不用于决策
- `candidate_branch_count`
- `candidate_branches[]`，每个 candidate 包含：
  - `branch_angle`
  - `target`
  - `target_exit_dist`
  - `exit_progress_delta_m`
  - `target_clearance_m`
  - `path_corridor_min_clearance_m`
  - `dispatch_path_local_cost_max`
  - `dispatch_path_local_cost_mean`
  - `target_local_cost`
  - `target_local_cost_max_radius`
  - `is_reverse_candidate`
  - `is_backtrack_context`
  - `is_near_exit_candidate`
  - `rejection_reason` if rejected
- `selected_due_to_context`：例如 DFS untried / backtrack / corridor advance / near_exit continuation。

Phase26E 验收门槛：

- 不改变 branch choice 行为。
- 只增加 diagnostics。
- TDD contract tests 必须证明新增字段存在于 dispatch goal event。
- smoke run 只用于验证 artifact coverage，不用于 promote/reject 参数。

