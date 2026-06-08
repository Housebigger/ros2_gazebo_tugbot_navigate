# Phase 26B Repeat-run Variance Characterization

日期：2026-05-25

## 一句话结论

Phase 26B 按计划完成 canonical baseline `CostCritic.cost_weight=3.81` 3 runs 与 candidate `CostCritic.cost_weight=2.75` 3 runs。两组都出现 `2/3 EXIT_REACHED + 1/3 FAILED_EXHAUSTED`，且每个 run 的 source fingerprint 与 runtime `/controller_server` dump 均一致，因此 Phase25E/candidate-baseline 差异主要应先解释为 run-to-run variance / stochastic simulation-controller interaction / start timing variance，而不是 profile-path 错误。`2.75` 仍不能 promote；也不建议继续盲测 `2.65/2.6`。

## 本阶段目标

Phase 26B 的目标不是调参，而是用重复运行刻画 variance：

- canonical baseline：`src/tugbot_navigation/config/nav2_slam_params.yaml`，`CostCritic.cost_weight=3.81`，3 runs。
- candidate profile：`src/tugbot_navigation/config/nav2_slam_candidate_costcritic_275_params.yaml`，`CostCritic.cost_weight=2.75`，3 runs。
- 每个 run 必须保留：
  - `*_params_fingerprint.json`
  - `*_runtime_params/controller_server.yaml`
  - `*_runtime_params/controller_server_summary.json`
  - `*_explorer_state.jsonl`
  - `*_goal_events.jsonl`
  - `*_goal_nav2_analysis.json`
  - `*_goal_controller_dynamics.json`
  - `*_timeout_subtypes.json`
  - `*_post_recovery_enriched.json`

## 本阶段新增/修改文件

- `src/tugbot_maze/test/test_phase26b_variance_characterization.py`
  - 新增 Phase26B contract tests。
  - 覆盖 wrapper run id、post-analysis artifacts、aggregator 统计行为。
- `tools/run_phase21_controller_diagnostics_smoke.sh`
  - 新增 run id：
    - `phase26b_baseline_runN`
    - `phase26b_candidate_runN`
  - 新增 Phase26B candidate profile selection：`PHASE26B_CANDIDATE_PROFILE=true` -> `candidate_costcritic_275_profile:=true`。
  - post-run chain 新增：
    - `tools/analyze_failure_windows.py` -> `*_failure_windows.json`
    - `tools/analyze_timeout_subtypes.py` -> `*_timeout_subtypes.json`
    - `tools/enrich_post_recovery_snapshots.py` -> `*_post_recovery_enriched.json`
- `tools/aggregate_phase26b_variance.py`
  - 新增 Phase26B 聚合器。
  - 读取每个 run 的 fingerprint/runtime/state/timeout/post-recovery artifact。
  - 按 profile 分组输出 repeat-run variance summary。
- `log/phase26b_variance_summary.json`
  - Phase26B 6-run 聚合结果。

## TDD 与静态验证

RED：新增 `test_phase26b_variance_characterization.py` 后先运行，确认失败：

```bash
python3 -m pytest -q src/tugbot_maze/test/test_phase26b_variance_characterization.py -v
```

失败原因符合预期：wrapper 尚不支持 Phase26B run id，且 `tools/aggregate_phase26b_variance.py` 尚不存在。

GREEN/verification：实现后通过：

```bash
python3 -m py_compile tools/aggregate_phase26b_variance.py tools/dump_controller_runtime_params.py tools/fingerprint_nav2_params.py
bash -n tools/run_phase21_controller_diagnostics_smoke.sh
python3 -m pytest -q src/tugbot_maze/test/test_phase26a_params_equivalence.py src/tugbot_maze/test/test_phase26a1_runtime_params_dump.py src/tugbot_maze/test/test_phase26b_variance_characterization.py
python3 -m pytest -q src/tugbot_maze/test src/tugbot_bringup/test
colcon build --symlink-install --packages-select tugbot_maze tugbot_bringup tugbot_navigation
colcon test --packages-select tugbot_maze tugbot_bringup tugbot_navigation --event-handlers console_direct+
colcon test-result --verbose
```

结果：

- Phase26A/A.1/B targeted tests：`12 passed`
- package pytest：`136 passed`
- colcon build：`3 packages finished`
- colcon test-result：`Summary: 112 tests, 0 errors, 0 failures, 0 skipped`

## 运行矩阵

| Group | Run ID | Params profile | Source cost_weight | Runtime cost_weight |
|---|---|---|---:|---:|
| baseline | `phase26b_baseline_run1` | canonical_baseline | 3.81 | 3.81 |
| baseline | `phase26b_baseline_run2` | canonical_baseline | 3.81 | 3.81 |
| baseline | `phase26b_baseline_run3` | canonical_baseline | 3.81 | 3.81 |
| candidate | `phase26b_candidate_run1` | candidate_costcritic_275 | 2.75 | 2.75 |
| candidate | `phase26b_candidate_run2` | candidate_costcritic_275 | 2.75 | 2.75 |
| candidate | `phase26b_candidate_run3` | candidate_costcritic_275 | 2.75 | 2.75 |

所有 6 个 run 的 required artifacts 均存在且非空；最终残留 ROS/Gazebo/Nav2 进程检查无输出。

## Per-run summary

| Run | Profile | final_mode | goals | success | failure | timeouts | blocked | blacklist | exit_distance_m | footprint/path blocked | side/timing | unclassified |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| phase26b_baseline_run1 | canonical_baseline | EXIT_REACHED | 11 | 7 | 3 | 3 | 0 | 0 | 0.546035 | 1 | 1 | 0 |
| phase26b_baseline_run2 | canonical_baseline | FAILED_EXHAUSTED | 12 | 8 | 4 | 4 | 0 | 0 | 0.883552 | 2 | 2 | 0 |
| phase26b_baseline_run3 | canonical_baseline | EXIT_REACHED | 10 | 5 | 4 | 4 | 0 | 0 | 0.528414 | 0 | 3 | 1 |
| phase26b_candidate_run1 | candidate_costcritic_275 | EXIT_REACHED | 9 | 5 | 3 | 3 | 0 | 0 | 0.577205 | 1 | 2 | 0 |
| phase26b_candidate_run2 | candidate_costcritic_275 | FAILED_EXHAUSTED | 12 | 9 | 3 | 3 | 0 | 0 | 1.338566 | 2 | 0 | 0 |
| phase26b_candidate_run3 | candidate_costcritic_275 | EXIT_REACHED | 10 | 5 | 4 | 4 | 0 | 0 | 0.560856 | 1 | 2 | 0 |

## Group-level variance summary

### Canonical baseline (`3.81`)

- final_mode values：`EXIT_REACHED`, `FAILED_EXHAUSTED`, `EXIT_REACHED`
- exit_reached_count：`2/3`
- timeout values：`[3, 4, 4]`
- timeout median：`4.0`
- footprint/path blocked values：`[1, 2, 0]`
- footprint/path blocked median：`1.0`
- exit_distance values：`[0.546035, 0.883552, 0.528414]`
- exit_distance median：`0.546035`
- blocked/blacklist：all `0/0`
- source/runtime cost weight：`3.81 / 3.81`

### Candidate (`2.75`)

- final_mode values：`EXIT_REACHED`, `FAILED_EXHAUSTED`, `EXIT_REACHED`
- exit_reached_count：`2/3`
- timeout values：`[3, 3, 4]`
- timeout median：`3.0`
- footprint/path blocked values：`[1, 2, 1]`
- footprint/path blocked median：`1.0`
- exit_distance values：`[0.577205, 1.338566, 0.560856]`
- exit_distance median：`0.577205`
- blocked/blacklist：all `0/0`
- source/runtime cost weight：`2.75 / 2.75`

## Interpretation

1. Profile-path / runtime-load 已排除。
   - Baseline 3 runs 均为 source/runtime `3.81`。
   - Candidate 3 runs 均为 source/runtime `2.75`。
   - 因此 Phase25E 与 candidate-baseline 差异不能再归因于 wrapper 选错 YAML 或 runtime 未加载 candidate params。

2. 两组都存在 final-mode variance。
   - baseline：`2/3 EXIT_REACHED`，`1/3 FAILED_EXHAUSTED`。
   - candidate：`2/3 EXIT_REACHED`，`1/3 FAILED_EXHAUSTED`。
   - 这支持“run-to-run variance / stochastic simulation-controller interaction / start timing variance”解释。

3. Candidate 在 timeouts median 上略优，但不是 promotion-grade。
   - baseline timeout median：`4.0`
   - candidate timeout median：`3.0`
   - 但 candidate 的 `FAILED_EXHAUSTED` run exit distance 为 `1.338566m`，且 final-mode 仍不稳定。
   - footprint/path blocked median 两组相同：`1.0`。

4. Candidate 没有引入 blocked/blacklist topology regression。
   - 6 个 run 的 `blocked_branch_count=0` 且 `blacklisted_goal_count=0`。
   - 这说明 candidate 的风险主要不是拓扑污染，而是 controller/local-cost late-silent variance 与 near-exit/exit-distance instability。

5. 仍不能 promote `cost_weight=2.75`。
   - Candidate 没有表现出足够稳定的 final-mode 优势。
   - Candidate median exit distance `0.577205m` 略差于 baseline median `0.546035m`。
   - Candidate timeouts 变少是积极信号，但不足以覆盖 final-mode 与 exit-distance variance。

## Aggregator recommendation

`tools/aggregate_phase26b_variance.py` 输出：

```text
candidate_variance_characterized_not_promotion_ready
```

含义：variance 已被刻画，candidate 有部分积极信号，但不具备 baseline promotion 条件。

## Decision

- 不 promote `CostCritic.cost_weight=2.75` 到 canonical baseline。
- 保留 `src/tugbot_navigation/config/nav2_slam_candidate_costcritic_275_params.yaml` 作为 candidate artifact。
- canonical baseline 保持 `src/tugbot_navigation/config/nav2_slam_params.yaml` 的 `CostCritic.cost_weight=3.81`。
- 不盲测 `2.65/2.6`。
- 不启用 Phase25A local inflation relief。
- 不做 branch scoring 或复杂 state-machine changes。

## 下阶段建议：Phase 26C

建议 Phase 26C 不继续扩展调参搜索，而是做“variance trigger alignment”：用 Phase26B 的 6-run artifacts 对齐成功 run 与失败 run 在何处开始分叉。

推荐问题：

1. `FAILED_EXHAUSTED` run 是否集中在相同 goal sequence / spatial region / near-exit branch？
2. Candidate 与 baseline 的 failed run 是否共享同类 late-silent subtype，还是 profile-specific？
3. `exit_distance_m` 偏大的 run 是否对应：
   - late `controller_silent`
   - path-ahead cost spike
   - near-zero cmd onset after recovery
   - recovery 后 path update 仍存在但 controller 不再输出有效 cmd
4. `phase26b_candidate_run2` 为什么 timeout count 不高但 exit distance 明显偏大？
5. 是否需要把 random/start timing/initial pose stabilization 也纳入 run metadata，避免把启动时序差异误判为 Nav2 参数差异？

建议 Phase26C 产物：

- `tools/compare_phase26b_failed_vs_success_runs.py`
- `log/phase26b_failed_vs_success_alignment.json`
- `doc/doc_report/phase26c_variance_trigger_alignment.md`

Phase26C 验收门槛：

- 必须先基于现有 6-run artifacts 做 post-run analysis，不启动新调参 run。
- 若要提出新干预，必须能定位到具体 subtype + goal sequence + local-cost/path/cmd evidence。
- 如果证据仍指向启动时序/随机性，应优先设计 deterministic replay / startup stabilization experiment，而不是继续调 CostCritic。
