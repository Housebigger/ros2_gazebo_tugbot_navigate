# Phase 26G — Matched Branch-Choice Diagnostics Repeats

Date: 2026-05-25
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 一句话结论

Phase26G 已完成 analysis-only matched repeats：2 次 canonical baseline 都到达出口，2 次 `candidate_costcritic_275` 都未到达出口；所有 repeats 都带 Phase26E `candidate_branches[]` diagnostics 且 coverage 通过。Phase26F analyzer 只发现 1 个 route-divergence case，且该 case 同时处于 near-exit、local-cost-constrained、reverse/rejected-candidate context；证据不足以进入 Phase27，不应改 branch selection，不应调 Nav2/controller 参数，不应 promote `cost_weight=2.75`。

## 目标与边界

目标：

1. 收集 matched baseline/candidate repeats。
2. repeats 必须带 Phase26E `candidate_branches[]` diagnostics。
3. baseline 与 candidate 使用相同 run budget / wrapper semantics。
4. 不改参数、不改 branch-selection 行为。
5. 跑完后继续用 `tools/analyze_phase26f_branch_choice_diagnostics.py` 汇总。
6. 只有当 matched repeats 稳定显示下列条件时，才考虑 Phase27：
   - failed runs 中 chosen-away 更集中；
   - rejected candidate 确实更靠近 exit；
   - 不是 reverse/backtrack/local-cost constrained 特例主导；
   - EXIT_REACHED runs 与 FAILED_EXHAUSTED runs 在 branch-choice evidence 上有稳定差异。

本阶段保持 analysis-only：

- 没有修改 Nav2 参数语义。
- 没有修改 branch choice 逻辑。
- 没有启用 Phase25A local inflation relief。
- 没有 promote `cost_weight=2.75`。
- 没有进入 Phase27。

## 代码与契约变更

### Wrapper run-id 支持

文件：`tools/run_phase21_controller_diagnostics_smoke.sh`

新增 run-id：

- `phase26g_baseline_runN`
- `phase26g_candidate_runN`

profile 语义：

- `phase26g_baseline_runN` 使用 canonical baseline：
  - params: `src/tugbot_navigation/config/nav2_slam_params.yaml`
  - selected_profile: `canonical_baseline`
  - expected runtime CostCritic weight: `3.81`
- `phase26g_candidate_runN` 使用既有 candidate artifact：
  - params: `src/tugbot_navigation/config/nav2_slam_candidate_costcritic_275_params.yaml`
  - selected_profile: `candidate_costcritic_275`
  - expected runtime CostCritic weight: `2.75`

同时把 Phase26G run-id 纳入 cleanup patterns，避免 long-running ROS/Gazebo/Nav2 recorder orphan 影响后续 runs。

### Contract tests

新增：`src/tugbot_maze/test/test_phase26g_matched_branch_diagnostics_repeats.py`

覆盖：

1. wrapper 接受 `phase26g_baseline_runN` / `phase26g_candidate_runN`。
2. candidate run 只选择既有 `candidate_costcritic_275` profile，不改变 baseline。
3. cleanup patterns 覆盖 Phase26G baseline/candidate run-id。
4. Phase26F analyzer 与 Phase26E coverage checker 仍保持 analysis-only guardrails。

TDD 过程：

- RED：新增 Phase26G tests 后，3 个 assertions 失败，原因是 wrapper 尚不支持 Phase26G run-id/profile/cleanup。
- GREEN：最小修改 wrapper 后，Phase26G targeted tests 通过。

## Repeats 与 artifacts

运行 repeats：

- baseline:
  - `phase26g_baseline_run1`
  - `phase26g_baseline_run2`
- candidate:
  - `phase26g_candidate_run1`
  - `phase26g_candidate_run2`

每个 run 产物包含：

- `log/<run>_params_fingerprint.json`
- `log/<run>_runtime_params/controller_server.yaml`
- `log/<run>_runtime_params/controller_server_summary.json`
- `log/<run>_explorer_state.jsonl`
- `log/<run>_goal_events.jsonl`
- `log/<run>_goal_nav2_analysis.json`
- `log/<run>_goal_controller_dynamics.json`
- `log/<run>_timeout_subtypes.json`
- `log/<run>_post_recovery_enriched.json`
- `log/<run>_branch_diagnostics_coverage.json`

Phase26G 汇总 artifact：

- `log/phase26g_branch_choice_diagnostics_analysis.json`

## Params/runtime proof

Source fingerprint 与 runtime dump 均证明 profile chain 正确：

| run | selected_profile | source CostCritic weight | runtime expected | runtime dumped CostCritic weight |
|---|---:|---:|---:|---:|
| phase26g_baseline_run1 | canonical_baseline | 3.81 | 3.81 | 3.81 |
| phase26g_baseline_run2 | canonical_baseline | 3.81 | 3.81 | 3.81 |
| phase26g_candidate_run1 | candidate_costcritic_275 | 2.75 | 2.75 | 2.75 |
| phase26g_candidate_run2 | candidate_costcritic_275 | 2.75 | 2.75 | 2.75 |

注：runtime dump summary 中实际字段位于 `controller_server.FollowPath.CostCritic.cost_weight`。

## Branch diagnostics coverage

所有 runs 的 Phase26E branch diagnostics coverage 均通过：

| run | pass_coverage | dispatch_events | nonempty candidate_branches dispatches | candidate rows | selected_due_to_context |
|---|---:|---:|---:|---:|---|
| phase26g_baseline_run1 | true | 8 | 8 | 27 | topology_exit_bias_score |
| phase26g_baseline_run2 | true | 11 | 11 | 38 | topology_exit_bias_score |
| phase26g_candidate_run1 | true | 11 | 11 | 38 | topology_exit_bias_score |
| phase26g_candidate_run2 | true | 12 | 11 | 40 | topology_exit_bias_score, backtrack |

`phase26g_candidate_run2` 有一个 backtrack dispatch，`candidate_branch_count=0`、`chosen_branch_rank=null`，coverage 仍通过，因为这是 backtrack context，不是 Phase26E candidate diagnostics 缺失。

## Run-level navigation outcome

| run | profile | final_mode | exit_distance_m | goals | success | failure | timeout_cancel | blocked_branch | blacklisted_goal |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| phase26g_baseline_run1 | canonical_baseline | EXIT_REACHED | 0.587 | 9 | 5 | 3 | 3 | 0 | 0 |
| phase26g_baseline_run2 | canonical_baseline | EXIT_REACHED | 0.596 | 11 | 7 | 3 | 3 | 0 | 0 |
| phase26g_candidate_run1 | candidate_costcritic_275 | FAILED_EXHAUSTED | 1.250 | 12 | 9 | 3 | 3 | 0 | 0 |
| phase26g_candidate_run2 | candidate_costcritic_275 | FAILED_EXHAUSTED | 0.854 | 12 | 9 | 3 | 3 | 0 | 0 |

解释：

- 本阶段仍不以单次 smoke promote/reject 参数，但 matched 2x2 repeats 已显示 candidate profile 在 Phase26G 条件下未达到出口。
- baseline 两次都达到出口，candidate 两次都 exhausted。
- blocked/blacklist 都保持 0，没有 topology pollution regression。

## Phase26F analyzer 汇总

命令：

```bash
python3 tools/analyze_phase26f_branch_choice_diagnostics.py \
  --log-dir log \
  --baseline-runs phase26g_baseline_run1,phase26g_baseline_run2 \
  --candidate-runs phase26g_candidate_run1,phase26g_candidate_run2 \
  --output-json log/phase26g_branch_choice_diagnostics_analysis.json
```

输出核心：

| group | run_count | final modes | dispatch_count | chosen_toward | chosen_away | route_divergence_cases |
|---|---:|---|---:|---:|---:|---:|
| baseline | 2 | both EXIT_REACHED | 19 | 15 | 2 | 0 |
| candidate | 2 | both FAILED_EXHAUSTED | 23 | 17 | 4 | 1 |

Analyzer decision：

- `stable_route_divergence_evidence: false`
- `recommendation: analysis_only_collect_more_or_repeat`
- guardrails:
  - `do_not_enter_phase27`
  - `do_not_change_branch_selection`
  - `do_not_tune_nav2_or_controller_params_from_phase26f`
  - `require_matched_repeats_before_intervention`

## Route-divergence case interpretation

唯一 route-divergence case：

- run: `phase26g_candidate_run2`
- goal_sequence: 8
- classification: `chosen_moves_away_rejected_moves_toward_exit`
- chosen branch:
  - target_exit_dist: `1.454 m`
  - exit_progress_delta_m: `-0.290 m`
  - dispatch path local cost max: `73`
  - target local cost max radius: `99`
- best rejected branch:
  - rejection_reason: `explored`
  - target_exit_dist: `0.807 m`
  - exit_progress_delta_m: `+0.357 m`
  - dispatch path local cost max: `99`
  - target local cost max radius: `99`
  - near-exit candidate: true
- contexts:
  - `reverse`
  - `near_exit`
  - `local_cost_constrained`
  - `rejected_candidate_context`

该 case 不能直接支持 Phase27 branch-selection intervention，原因：

1. 只有 1 个 route-divergence case，不稳定。
2. best rejected candidate 的 `rejection_reason` 是 `explored`，不是普通 lower-rank miss。
3. rejected target 更靠近 exit，但 local-cost/target-cost 同样很高，属于 near-exit + local-cost-constrained 特例。
4. baseline success runs 中也有 chosen-away，但没有 route-divergence case，说明 chosen-away 本身不足以解释 outcome。
5. candidate failed runs 更差的 final mode 可能仍与 stochastic simulation-controller interaction、near-exit local cost pressure、route history/explored-state interaction 混合有关。

## Phase27 gate 判定

用户提出的 Phase27 前置条件逐项判断：

1. failed runs 中 chosen-away 更集中：部分成立但不足。
   - baseline EXIT_REACHED group: 2 chosen-away / 19 dispatches。
   - candidate FAILED_EXHAUSTED group: 4 chosen-away / 23 dispatches。
   - candidate 更高，但样本太小且不是稳定 route-divergence。

2. rejected candidate 确实更靠近 exit：只在 1 个 case 成立。
   - `phase26g_candidate_run2` seq=8 成立。
   - 但 rejected candidate 是 `explored` 且 local-cost constrained。

3. 不是 reverse/backtrack/local-cost constrained 特例主导：不成立。
   - 唯一 route-divergence case 同时带 reverse、near-exit、local-cost-constrained contexts。
   - `phase26g_candidate_run2` 另有 backtrack context。

4. EXIT_REACHED vs FAILED_EXHAUSTED 在 branch-choice evidence 上有稳定差异：不成立。
   - outcome 差异很明显：baseline 2/2 exit，candidate 0/2 exit。
   - branch-choice evidence 还不够稳定：route-divergence case 只有 1 个。

结论：Phase26G 不满足进入 Phase27 的证据门槛。

## Verification

已执行：

```bash
python3 -m pytest -q \
  src/tugbot_maze/test/test_phase26g_matched_branch_diagnostics_repeats.py \
  src/tugbot_maze/test/test_phase26f_branch_choice_diagnostics_analysis.py \
  src/tugbot_maze/test/test_phase26e_branch_choice_diagnostics.py -v
```

结果：`12 passed`。

静态检查：

```bash
python3 -m compileall -q \
  tools/run_phase21_controller_diagnostics_smoke.sh \
  tools/analyze_phase26f_branch_choice_diagnostics.py \
  tools/check_phase26e_branch_diagnostics_coverage.py \
  src/tugbot_maze/test/test_phase26g_matched_branch_diagnostics_repeats.py
bash -n tools/run_phase21_controller_diagnostics_smoke.sh
python3 /home/hyh/.hermes/skills/software-development/ros2-gazebo-navigation-workspaces/scripts/check_ros2_workspace_contracts.py .
```

结果：

- `bash -n` passed。
- ROS 2 workspace static contracts passed。

Build/test：

```bash
colcon build --symlink-install
colcon test --event-handlers console_direct+ --packages-select tugbot_maze tugbot_bringup
colcon test-result --verbose
```

结果：

- build: 6 packages finished。
- tests: `126 tests, 0 errors, 0 failures, 0 skipped`。

Process cleanup：

- Phase26G runs 后检查 ROS/Gazebo/Nav2/recorder 进程：无残留输出。

## 下一阶段建议

不建议直接进入 Phase27。推荐 Phase26H，仍 analysis-only，但聚焦 “near-exit explored/local-cost constrained divergence” 而不是立即改 branch selection。

Phase26H 建议目标：

1. 对 Phase26G candidate failed runs 做 post-run near-exit route-history audit：
   - route-divergence case 的 rejected `explored` target 是什么时候被标记 explored；
   - explored-state 是否来自真实成功经过、timeout、terminal/near-exit reach、还是 backtrack route；
   - near-exit `explored` 是否过早屏蔽了更靠近 exit 的 candidate。
2. 增强 analyzer，而不是改 runtime behavior：
   - 对 route-divergence cases 输出 candidate `state` / `rejection_reason` 分布；
   - 分离 `lower_rank_not_selected` vs `explored` vs `blacklisted`；
   - 对 near-exit + high local cost + explored candidate 单独计数；
   - 与 timeout subtype / post-recovery local-cost enriched artifact join。
3. 可选增加 1 次 baseline + 1 次 candidate repeat，但仅当需要提高样本量；如果做 repeat，必须继续带 Phase26E diagnostics，并继续使用同一 wrapper semantics。

仍不建议：

- promote `cost_weight=2.75`。
- 盲测 `2.65` / `2.6`。
- 启用 Phase25A local inflation relief。
- 修改 branch scoring / branch rejection。
- 基于这 1 个 route-divergence case 进入 Phase27。

## 状态

Phase26G：PASS as analysis-only evidence collection。

Phase27：NOT GATED / DO NOT ENTER YET。
