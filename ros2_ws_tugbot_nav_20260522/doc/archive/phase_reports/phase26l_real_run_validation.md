# Phase 26L Real-Run Validation — Runtime Spatial Diagnostics Gate

Date: 2026-05-25
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 一句话结论

Phase26L-real-run 完成 controlled matched repeat smoke 与真实 artifacts gate。更新后的 recorder 成功在真实运行中写入 high-cost cell coordinates、centroid、nearest point、first high-cost distance，以及 chosen/explored corridor cost 字段；但是 failed-only gate 没有发现任何 `chosen_high_explored_clean_contract_signal`。真实证据不支持 Phase27，不应改 branch selection。两个真实 timeout 组都继续落在 `side_cost_or_timing_late_silent+cmd_silent_after_recovery_abort`，下一步应继续 local-cost/controller timing diagnostics。

## 边界

本阶段仍是 analysis-first / diagnostics-only：

- 没有修改 branch-selection 行为。
- 没有修改 Nav2/controller 参数。
- 没有提升任何 candidate baseline。
- controlled smoke 只用于收集真实 artifacts 与验证 Phase26L gate。

## 运行配置

Wrapper：

- `tools/run_phase21_controller_diagnostics_smoke.sh`

为 Phase26L real-run 增加 run ids：

- `phase26l_baseline_runN`
- `phase26l_candidate_runN`

并确保 `record_post_recovery_snapshots.py` 以：

```bash
--max-high-cost-points 40
```

收集更完整但仍有上限的 high-cost coordinate payload。

## Controlled matched repeat smoke

运行：

```bash
bash tools/run_phase21_controller_diagnostics_smoke.sh phase26l_baseline_run1
bash tools/run_phase21_controller_diagnostics_smoke.sh phase26l_candidate_run1
```

实际结果：

### phase26l_baseline_run1

- final_mode: `FAILED_EXHAUSTED`
- goal_count: `12`
- goal_success_count: `10`
- goal_failure_count: `2`
- timeout_cancel_count: `2`
- blocked_branch_count: `0`
- blacklisted_goal_count: `0`
- exit_distance_m: `1.7626523254755457`

Nav2 summary：

- timeout_count: `2`
- timeout_with_progress_failure_count: `2`
- timeout_with_recovery_count: `2`
- timeout_with_controller_abort_count: `2`

Controller summary：

- healthy_motion_count: `9`
- healthy_motion_but_late_stall_count: `2`
- late_controller_silent_count: `2`
- timeout_or_failure_late_stall_count: `2`

Timeout subtype：

- `side_cost_or_timing_late_silent+cmd_silent_after_recovery_abort`: `2`
- severe_footprint_path_count: `0`

### phase26l_candidate_run1

- final_mode: `EXIT_REACHED`
- goal_count: `8`
- goal_success_count: `5`
- goal_failure_count: `2`
- timeout_cancel_count: `2`
- blocked_branch_count: `0`
- blacklisted_goal_count: `0`
- exit_distance_m: `0.564895258466082`

Nav2 summary：

- timeout_count: `2`
- timeout_with_progress_failure_count: `2`
- timeout_with_recovery_count: `2`
- timeout_with_controller_abort_count: `2`

Controller summary：

- healthy_motion_count: `5`
- healthy_motion_but_late_stall_count: `2`
- late_controller_silent_count: `2`
- terminal_cancel_after_exit_count: `1`
- timeout_or_failure_late_stall_count: `2`

Timeout subtype：

- `side_cost_or_timing_late_silent+cmd_silent_after_recovery_abort`: `2`
- severe_footprint_path_count: `0`

## Artifact coverage

关键真实 artifacts 均已生成：

Baseline：

- `log/phase26l_baseline_run1_goal_events.jsonl`
- `log/phase26l_baseline_run1_post_recovery_snapshots.jsonl`
- `log/phase26l_baseline_run1_controller_dynamics.jsonl`
- `log/phase26l_baseline_run1_launch.log`
- `log/phase26l_baseline_run1_goal_controller_dynamics.json`
- `log/phase26l_baseline_run1_timeout_subtypes.json`
- `log/phase26l_baseline_run1_post_recovery_enriched.json`

Candidate：

- `log/phase26l_candidate_run1_goal_events.jsonl`
- `log/phase26l_candidate_run1_post_recovery_snapshots.jsonl`
- `log/phase26l_candidate_run1_controller_dynamics.jsonl`
- `log/phase26l_candidate_run1_launch.log`
- `log/phase26l_candidate_run1_goal_controller_dynamics.json`
- `log/phase26l_candidate_run1_timeout_subtypes.json`
- `log/phase26l_candidate_run1_post_recovery_enriched.json`

Snapshot Phase26L field coverage：

- baseline snapshots: `171`
  - all `171` contain `path_ahead_1_0m_high_cost_points`
  - all `171` contain `path_ahead_1_0m_high_cost_count`
  - all `171` contain `path_ahead_1_0m_high_cost_centroid`
  - all `171` contain `chosen_route_corridor_cost_max`
  - all `171` contain `explored_candidate_corridor_cost_max`
- candidate snapshots: `125`
  - all `125` contain the same Phase26L spatial contract fields

## Phase26L real gate

Command：

```bash
python3 tools/analyze_phase26l_runtime_spatial_contract.py \
  --log-dir log \
  --failed-runs phase26l_baseline_run1,phase26l_candidate_run1 \
  --compare-runs phase26l_candidate_run1 \
  --output-json log/phase26l_real_run_gate.json \
  | tee log/phase26l_real_run_gate.out
```

Decision：

```json
{
  "phase27_candidate_signal": "not_supported",
  "next_recommendation": "complete_runtime_spatial_contract_and_repeat_failed_only_matches"
}
```

Failed-only summary：

- failed run count: `2`
- spatial_contract_complete_case_count: `2`
- chosen_high_explored_clean_contract_signal_count: `0`

Failed-only cases：

### baseline seq=2

- outcome: timeout
- route_context: `unknown`
- spatial_contract_complete: `true`
- path_ahead_high_cost_count: `9`
- chosen_route_corridor_cost_max: `40`
- chosen_route_high_cost: `false`
- explored_candidate_corridor_cost_max: `null`
- explored_candidate_corridor_clean: `false`
- classification: `insufficient_or_dirty_explored_corridor`

Interpretation：high-cost cells exist, but not in a chosen-away/explored-clean branch-choice context. No Phase27 signal.

### baseline seq=10

- outcome: timeout
- route_context: `non_divergent_chosen_also_toward_exit`
- spatial_contract_complete: `false`
- path_ahead_high_cost_count: `0` at selected near-zero snapshot
- chosen_route_corridor_cost_max: `0`
- explored_candidate_corridor_cost_max: `99`
- explored_candidate_corridor_clean: `false`
- classification: `insufficient_or_dirty_explored_corridor`

Interpretation：not a chosen-away route-divergence case; explored candidate corridor is dirty/high-cost.

### candidate seq=2

- outcome: timeout
- route_context: `unknown`
- spatial_contract_complete: `true`
- path_ahead_high_cost_count: `8`
- chosen_route_corridor_cost_max: `40`
- chosen_route_high_cost: `false`
- explored_candidate_corridor_cost_max: `null`
- explored_candidate_corridor_clean: `false`
- classification: `insufficient_or_dirty_explored_corridor`

Interpretation：similar to baseline seq=2; high-cost cells exist but not as chosen-away/explored-clean route-divergence evidence.

### candidate seq=7

- outcome: timeout
- route_context: `non_divergent_chosen_also_toward_exit`
- spatial_contract_complete: `false`
- path_ahead_high_cost_count: `0` at selected near-zero snapshot
- chosen_route_corridor_cost_max: `46`
- chosen_route_high_cost: `false`
- explored_candidate_corridor_cost_max: `0`
- explored_candidate_corridor_clean: `true`
- classification: `insufficient_or_dirty_explored_corridor`

Interpretation：explored corridor appears clean, but chosen route is not high-cost and route context is non-divergent; no Phase27 signal.

## Spatial alignment cross-check

Additional Phase26K-style alignment was run for baseline seq=10 vs candidate seq=7：

- output: `log/phase26l_real_run_spatial_alignment_seq10.json`
- decision: `phase27_candidate_signal: not_supported`
- spatial_interpretation: `shared_choke_or_ambiguous`

Important nuance：periodic high-cost samples can still look chosen-route-dominant, but near-zero selected snapshots classify as shared/ambiguous or non-divergent. This reinforces that Phase27 branch-selection changes are not justified.

## Analyzer hardening

During real-run analysis, the same run was used both as failed-only and comparison input. The analyzer originally stored reports under a single `runs` map, so comparison analysis could overwrite failed-only view for overlapping run ids. This was fixed with TDD:

- new test: `src/tugbot_maze/test/test_phase26l_overlap_compare_contract.py`
- analyzer now emits separate:
  - `failed_runs`
  - `compare_runs`
  - compatibility `runs` map with `compare::<run_id>` keys for compare entries

This matters because Phase26L gate must always be computed only from failed-only views.

## Verification

Syntax + focused tests：

```bash
python3 -m py_compile tools/record_post_recovery_snapshots.py tools/analyze_phase26l_runtime_spatial_contract.py
python3 -m pytest \
  src/tugbot_maze/test/test_phase26l_runtime_spatial_contract.py \
  src/tugbot_maze/test/test_phase26l_real_run_wrapper_contract.py \
  src/tugbot_maze/test/test_phase26l_overlap_compare_contract.py \
  -q
```

Result：`6 passed`.

Regression tests：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase24*.py src/tugbot_maze/test/test_phase26*.py -q
```

Result：`51 passed`.

Cleanup check：

```bash
ps -eo pid,cmd | grep -E 'run_phase21_controller_diagnostics_smoke|ros2 launch tugbot_bringup|record_explorer_state_series|record_controller_dynamics|record_post_recovery_snapshots|ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|maze_explorer|slam_toolbox|controller_server|planner_server|bt_navigator|gz sim|ruby .*gz sim' | grep -v grep || true
```

Result：no matching processes.

## Decision

Phase27 remains blocked.

Why：

1. Real failed-only matched runs produced `0` chosen-high/explored-clean contract signals.
2. The failed cases with complete high-cost coordinates were not chosen-away route-divergence cases.
3. The route-divergence-like later cases were non-divergent or had dirty/high-cost explored corridors.
4. Candidate run reached exit, but its timeout signatures still match controller/local-cost timing symptoms rather than branch-selection evidence.
5. No blocked/blacklist regression occurred, but that alone is not enough to justify branch-selection changes.

## Next recommendation

Continue diagnostics, not Phase27 behavior changes.

Recommended next phase: Phase26M controller/local-cost timing window refinement.

Focus：

1. For `side_cost_or_timing_late_silent+cmd_silent_after_recovery_abort` cases, inspect cmd silence timing relative to:
   - recovery clear-costmap events
   - path updates
   - first high-cost distance
   - robot-to-path distance
   - chosen/explored corridor cost
2. Compare baseline seq=2 / candidate seq=2 as matched early-timeout pair.
3. Compare baseline seq=10 / candidate seq=7 as late near-exit timeout pair, but treat route context as non-divergent/shared rather than Phase27 evidence.
4. Do not tune controller or branch scoring until a new hypothesis is tied to these timing/cost windows and has a narrow reversible test.
