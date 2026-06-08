# Phase 26C Variance Trigger Alignment

日期：2026-05-25

## 一句话结论

Phase 26C 未启动任何新调参 run，只使用 Phase26B 的 6-run artifacts 做 failed-vs-success 对齐。`phase26b_candidate_run2` timeout 数不高但 exit distance 明显偏大，主因不是 profile-path 错误、也不是 recovery 后缺少 path refresh，而是：near-exit 前后的 route divergence 叠加 persistent local-cost pressure。具体表现为 seq10 发生 `footprint_path_blocked_late_silent + cmd_silent_after_recovery_abort`，post-recovery 仍有 path updates，但 near-zero cmd 时 path-ahead cost 仍高；之后 seq11/seq12 继续执行非 near-exit 分支，其中 seq11 是“成功但目标更远离出口”的 late success，最终 max_goals/exhaustion 时离出口仍为 1.338566m。

## 本阶段范围

用户要求：

- 进入 Phase 26C：variance trigger alignment。
- 不启动新调参 run。
- 使用 Phase26B 6-run artifacts。
- 对齐成功 run 与失败 run 的分叉点：
  - goal sequence
  - 空间位置
  - late-silent subtype
  - path-ahead cost
  - post-recovery path/cmd 行为
- 重点解释 `phase26b_candidate_run2` 为什么 timeout count 不高但 exit distance 明显偏大。

本阶段严格遵守：不 promote `2.75`，不盲测 `2.65/2.6`，不启用 Phase25A local inflation relief，不做 branch scoring / complex state-machine changes。

## 新增文件

- `src/tugbot_maze/test/test_phase26c_variance_trigger_alignment.py`
  - Phase26C TDD contract test。
  - 用 synthetic failed-vs-success artifacts 验证 analyzer 能识别：
    - high exit distance without high timeout count
    - severe late-silent sequence
    - post-recovery path refresh present
    - persistent path-ahead cost pressure
    - late success moving away from exit
- `tools/compare_phase26b_failed_vs_success_runs.py`
  - Phase26C post-run analyzer。
  - 只读取现有 artifacts，不启动 ROS/Gazebo。
  - 输出 failed-vs-success alignment 与 focus-run explanation。
- `log/phase26b_failed_vs_success_alignment.json`
  - Phase26B 6-run alignment 输出。

## TDD 与验证

RED：先写 `test_phase26c_variance_trigger_alignment.py`，运行后失败，原因符合预期：

```text
tools/compare_phase26b_failed_vs_success_runs.py: No such file or directory
```

GREEN/verification：实现 analyzer 后通过：

```bash
python3 -m py_compile tools/compare_phase26b_failed_vs_success_runs.py
python3 -m pytest -q src/tugbot_maze/test/test_phase26b_variance_characterization.py src/tugbot_maze/test/test_phase26c_variance_trigger_alignment.py
python3 -m pytest -q src/tugbot_maze/test src/tugbot_bringup/test
```

结果：

- Phase26B/C targeted tests：`3 passed`
- package pytest：`137 passed`

未启动任何新 smoke / tuning run。

## Alignment summary

输入 runs：

- `phase26b_baseline_run1`：EXIT_REACHED，exit_distance=0.546035
- `phase26b_baseline_run2`：FAILED_EXHAUSTED，exit_distance=0.883552
- `phase26b_baseline_run3`：EXIT_REACHED，exit_distance=0.528414
- `phase26b_candidate_run1`：EXIT_REACHED，exit_distance=0.577205
- `phase26b_candidate_run2`：FAILED_EXHAUSTED，exit_distance=1.338566
- `phase26b_candidate_run3`：EXIT_REACHED，exit_distance=0.560856

整体对齐：

- EXIT_REACHED runs：4
  - median exit_distance：0.553445
- FAILED_EXHAUSTED runs：2
  - median exit_distance：1.111059
- Failed runs：
  - `phase26b_baseline_run2`
  - `phase26b_candidate_run2`
- Failed-run timeout sequences：
  - baseline_run2：seq2, seq10, seq11, seq12
  - candidate_run2：seq3, seq4, seq10
- Failed-run timeout controller subtype counts：
  - `footprint_path_blocked_late_silent`：4
  - `side_cost_or_timing_late_silent`：2
  - unclassified/missing subtype：1

## 成功 run 的 late-stage pattern

### phase26b_baseline_run1

Late stage：

- seq8 success near_exit：robot_exit_dist 1.826 -> target_exit_dist 0.968
- seq9 timeout near_exit：robot_exit_dist 1.086 -> target_exit_dist 0.829
  - subtype：`footprint_path_blocked_late_silent`
  - path_ahead_1m cost：100
  - post-recovery path updates：41
  - near-zero robot-to-path distance：0.031m
- seq10 timeout：robot_exit_dist 0.667 -> target_exit_dist 1.119
- seq11 terminal_cancel exit_reached：final exit_distance 0.546

Interpretation：即使 near-exit footprint/path blocked timeout 存在，robot 已经足够靠近出口；后续 terminal monitor 触发 EXIT_REACHED。

### phase26b_baseline_run3

Late stage：

- seq7 timeout：unclassified late-silent，path cost 0，非严重 footprint/path blocked
- seq8 success：robot_exit_dist 2.339 -> target_exit_dist 1.805
- seq9 timeout：side/timing，path_ahead_1m cost 100
- seq10 terminal_cancel exit_reached：robot_exit_dist 0.943 -> target_exit_dist 0.655，final exit_distance 0.528

Interpretation：late timeout 存在，但最后一个 terminal goal 朝 exit-progress 方向推进。

### phase26b_candidate_run1

Late stage：

- seq7 success：robot_exit_dist 2.122 -> target_exit_dist 1.630
- seq8 timeout near_exit：robot_exit_dist 1.381 -> target_exit_dist 0.565
  - subtype：`footprint_path_blocked_late_silent`
  - path_ahead_1m cost：100
  - post-recovery path updates：40
  - near-zero robot-to-path distance：0.086m
- seq9 terminal_cancel exit_reached：final exit_distance 0.577

Interpretation：candidate 成功 run 也有 near-exit severe late-silent timeout；成功原因不是没有 local-cost pressure，而是 timeout 时已经在 near-exit corridor，terminal monitor 后续仍能触发。

### phase26b_candidate_run3

Late stage：

- seq7 success near_exit：robot_exit_dist 1.825 -> target_exit_dist 0.968
- seq8 timeout near_exit：robot_exit_dist 1.083 -> target_exit_dist 0.769
  - subtype：`footprint_path_blocked_late_silent`
  - path_ahead_1m cost：100
  - post-recovery path updates：35
  - near-zero robot-to-path distance：0.034m
- seq9 timeout：robot_exit_dist 0.613 -> target_exit_dist 1.135
- seq10 terminal_cancel exit_reached：final exit_distance 0.561

Interpretation：与 candidate_run1 类似，near-exit severe pressure 存在，但 robot 已足够接近 exit，最终 EXIT_REACHED。

## Failed run 对齐

### phase26b_baseline_run2

Late stage：

- seq9 success 但 target 更远离出口：robot_exit_dist 1.050 -> target_exit_dist 1.393，branch_angle -2.649
- seq10 timeout：robot_exit_dist 1.412 -> target_exit_dist 1.226
  - subtype：`side_cost_or_timing_late_silent`
  - path_ahead_1m cost：100
  - post-recovery path updates：25
  - near-zero robot-to-path distance：0.015m
- seq11 timeout near_exit：robot_exit_dist 1.597 -> target_exit_dist 0.707
  - subtype：`footprint_path_blocked_late_silent`
  - path_ahead_1m cost：100
  - post-recovery path updates：40
  - near-zero robot-to-path distance：0.021m
- seq12 timeout：robot_exit_dist 0.755 -> target_exit_dist 1.141
  - subtype：`footprint_path_blocked_late_silent`
  - near-zero path_ahead cost：100
  - post-recovery path updates：25
- final FAILED_EXHAUSTED，exit_distance 0.884

Interpretation：baseline failed run 也不是 path refresh 缺失；它在 late stage 出现“成功但更远离出口”的分支后，又连续遇到 severe local-cost late-silent timeout。

### phase26b_candidate_run2（focus）

关键数据：

- final_mode：FAILED_EXHAUSTED
- exit_distance_m：1.338566
- timeout_cancel_count：3
- severe_late_silent_sequences：seq3, seq10
- primary explanation：`route_divergence_after_near_exit_timeout_with_persistent_local_cost_pressure`

Late-stage sequence：

| seq | event | robot_exit_dist | target_exit_dist | branch_angle | near_exit | subtype/path evidence |
|---:|---|---:|---:|---:|---|---|
| 7 | success | 3.103 | 2.263 | 0.321 | false | exit-progress |
| 8 | success | 2.387 | 1.595 | 0.309 | false | exit-progress |
| 9 | success | 1.713 | 1.019 | 0.259 | false | exit-progress |
| 10 | timeout | 1.093 | 1.134 | -1.391 | false | footprint/path blocked late-silent, path_ahead=99 |
| 11 | success | 0.759 | 1.222 | 1.749 | false | success but target moves away from exit |
| 12 | success | 1.291 | 1.208 | -1.183 | false | still not near-exit terminal path |

Seq10 evidence：

- subtype：`footprint_path_blocked_late_silent`
- timing：`cmd_silent_after_recovery_abort`
- failure-window path_ahead_1m cost：99
- timeout footprint cost max：99
- near_zero_cmd_duration_sec：8.436s
- post-recovery path_update_count_max：18
- near_zero_path_ahead_1m_cost_max：99
- near_zero_robot_to_path_distance_min：0.075m
- `controller_received_path_but_cmd_near_zero_any`：false

Interpretation：

1. Timeout 数不高不代表接近出口成功。
   - candidate_run2 只有 3 个 timeout，但 seq10 发生在 near-exit 前的关键 route choice 区域。
   - 后续 seq11/seq12 是 successful navigation，但没有把 robot 带入 terminal exit trigger。

2. `phase26b_candidate_run2` 的分叉不是“没有 path refresh”。
   - seq10 recovery 后仍有 path updates（max 18）。
   - robot near-zero 时仍贴近 path（robot-to-path 0.075m）。
   - path-ahead cost 仍为 99。
   - 这更像 controller/local-cost pressure 下路径存在但不可有效推进，而不是 planner/path publication 缺失。

3. 最关键的 route divergence 是 seq10 -> seq11。
   - seq10 timeout 时 robot_exit_dist 1.093，target_exit_dist 1.134，没有继续显著靠近 exit。
   - seq11 success 从 robot_exit_dist 0.759 去 target_exit_dist 1.222，是“成功但远离出口”的分支。
   - seq12 success 后 robot_exit_dist 又回到 1.291 附近。
   - 因此 final exit_distance 1.338566 大，不是因为 timeout 太多，而是因为 late-stage successful goals 把 exploration route 带离 exit trigger region。

4. 该失败形态与 successful candidate runs 的差异：
   - candidate_run1/candidate_run3 也有 severe near-exit timeout 和 high path cost。
   - 但它们 timeout 后很快触发 terminal cancel / EXIT_REACHED。
   - candidate_run2 则在 seq10 timeout 后转入非 terminal route，并用成功 goals 消耗了剩余 goal budget。

## Root-cause hypothesis

当前证据支持以下 hypothesis：

`phase26b_candidate_run2` 的高 exit_distance 是 route divergence after local-cost late-silent timeout，而不是 CostCritic 2.75 source/runtime 加载错误，也不是 recovery 后缺少 path refresh。机器人在 seq10 附近已经接近出口前区域，但 local cost / footprint-path pressure 让 controller 进入 late-silent；随后 DFS 选择了可成功执行但目标离出口更远的分支，消耗了剩余 max_goals，最终 FAILED_EXHAUSTED。

## Decision

- 不 promote `CostCritic.cost_weight=2.75`。
- 不 reject `2.75` solely based on candidate_run2。
- 不盲测 `2.65/2.6`。
- 不启用 Phase25A local inflation relief。
- 不做 branch scoring / complex state-machine changes。
- Phase26C 证据把问题从“参数候选是否有效”进一步收敛到“late-stage route divergence after local-cost pressure”。

## 下阶段建议：Phase 26D

建议 Phase 26D 仍不做 broad tuning，而做最小化 deterministic replay / decision snapshot：

1. 增加 DFS branch-choice snapshot analyzer（post-run first）
   - 输入：`/maze/goal_events`。
   - 输出每个 junction 的 candidate branches、chosen branch、branch_angle、robot_exit_dist、target_exit_dist、near_exit、local-cost fields。
   - 重点比较：candidate_run2 seq10/seq11 与 candidate_run1/3 near-exit terminal path。

2. 如果现有 artifacts 不足以解释 branch choice，再加 runtime diagnostics，而不是改策略：
   - 在 `maze_explorer` goal event 中记录 chosen branch rank / rejected branch summary。
   - 记录每个 candidate branch 的 target_exit_dist、clearance、local path cost、reverse/backtrack context。

3. Phase26D 的验收门槛：
   - 能解释 seq10 timeout 后为什么 seq11 选择远离 exit 的 successful branch。
   - 若要提出干预，必须是 narrow and reversible，例如 near-exit post-timeout retry policy 或 branch-choice tie-breaker diagnostics；但必须先证明不是 broad branch scoring。

推荐产物：

- `tools/analyze_phase26d_branch_choice_snapshots.py`
- `log/phase26d_branch_choice_alignment.json`
- `doc/doc_report/phase26d_branch_choice_snapshot_alignment.md`
