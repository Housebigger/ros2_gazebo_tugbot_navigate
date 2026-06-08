# Phase 26M — Controller / Local-Cost Timing Window Refinement

Date: 2026-05-25
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 一句话结论

Phase26M 完成 analysis-only timing-window refinement。对 Phase26L real-run 的两组优先 timeout pair 做 join 后，4/4 timeout 都是 `side_cost_or_timing_late_silent+cmd_silent_after_recovery_abort`。在每个 case 中，recovery clear-costmap 后 5s 内 cmd_vel 几乎立刻进入 near-zero（0.066s～0.606s，5s near-zero ratio=1.0），但 path updates 仍持续（4～8 次）。这说明当前主线仍是 controller/local-cost timing 或 recovery 后控制恢复问题，不是 branch selection；Phase27 继续 blocked。

## 边界

本阶段没有启动 ROS/Gazebo/Nav2，没有改 branch selection，没有调 Nav2/controller 参数。只读取 Phase26L real-run artifacts。

## 新增文件

Analyzer：

- `tools/analyze_phase26m_timing_windows.py`

Test：

- `src/tugbot_maze/test/test_phase26m_timing_windows.py`

Artifacts：

- `log/phase26m_timing_windows.json`
- `log/phase26m_timing_windows.out`

## TDD 过程

RED：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase26m_timing_windows.py -q
```

预期失败：

- `tools/analyze_phase26m_timing_windows.py` 不存在。

GREEN：

实现 analyzer 后：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase26m_timing_windows.py -q
```

结果：`1 passed`。

随后扩展 test，要求 analyzer 从 `*_controller_dynamics.jsonl` 提取：

- `cmd_near_zero_after_recovery_sec`
- `cmd_near_zero_ratio_5s_after_recovery`

并完成通过。

## Analyzer 功能

命令：

```bash
python3 tools/analyze_phase26m_timing_windows.py \
  --log-dir log \
  --pair early:phase26l_baseline_run1:2:phase26l_candidate_run1:2 \
  --pair near_exit:phase26l_baseline_run1:10:phase26l_candidate_run1:7 \
  --output-json log/phase26m_timing_windows.json \
  | tee log/phase26m_timing_windows.out
```

Analyzer join inputs：

- `*_goal_nav2_analysis.json`
- `*_goal_controller_dynamics.json`
- `*_timeout_subtypes.json`
- `*_post_recovery_snapshots.jsonl`
- `*_controller_dynamics.jsonl`

Analyzer 输出每个 case：

- combined/controller/timing subtype
- recovery clear-costmap time
- controller abort/progress failure delta from recovery
- near-zero snapshot time delta from recovery
- first cmd near-zero delta from recovery
- 5s post-recovery cmd near-zero ratio
- path update count after recovery
- pre/post/near-zero path-ahead cost
- near-zero high-cost count and first high-cost distance
- robot-to-path distance
- chosen/explored corridor cost

## Pair 1: early timeout pair

Pair：

- baseline: `phase26l_baseline_run1` seq=2
- candidate: `phase26l_candidate_run1` seq=2

Matched pattern：

- `persistent_cmd_silent_after_recovery_with_path_updates`

Interpretation：

- `timing_or_controller_silence_not_branch_selection`

Baseline seq=2：

- combined_subtype: `side_cost_or_timing_late_silent+cmd_silent_after_recovery_abort`
- recovery_time: `1779718598.034463`
- cmd_near_zero_after_recovery_sec: `0.066265`
- cmd_near_zero_ratio_5s_after_recovery: `1.0`
- path_updates_after_recovery: `4`
- pre/post path_ahead_1_0m_cost_max: `34 / 34`
- near_zero_path_ahead_1_0m_cost_max: `99`
- near_zero_first_high_cost_distance_m: `0.6`
- near_zero_high_cost_count: `9`
- near_zero_robot_to_path_distance_m: `0.08674008596819262`
- chosen_route_corridor_cost_max: `40`
- explored_candidate_corridor_cost_max: `null`

Candidate seq=2：

- combined_subtype: `side_cost_or_timing_late_silent+cmd_silent_after_recovery_abort`
- recovery_time: `1779718836.5433936`
- cmd_near_zero_after_recovery_sec: `0.2157`
- cmd_near_zero_ratio_5s_after_recovery: `1.0`
- path_updates_after_recovery: `6`
- pre/post path_ahead_1_0m_cost_max: `34 / 34`
- near_zero_path_ahead_1_0m_cost_max: `99`
- near_zero_first_high_cost_distance_m: `0.65`
- near_zero_high_cost_count: `8`
- near_zero_robot_to_path_distance_m: `0.08559322364283034`
- chosen_route_corridor_cost_max: `40`
- explored_candidate_corridor_cost_max: `null`

Key readout：

- 两个 early timeout 都不是 chosen-route corridor high-cost（chosen corridor max=40）。
- recovery 后 path updates 仍持续。
- cmd 在 recovery 后 0.07～0.22s 内 near-zero，且后 5s 全 near-zero。
- high-cost 出现在 later near-zero snapshot 的 path-ahead，但这更像 controller/local-cost timing interaction，不是 branch-selection 证据。

## Pair 2: near-exit timeout pair

Pair：

- baseline: `phase26l_baseline_run1` seq=10
- candidate: `phase26l_candidate_run1` seq=7

Matched pattern：

- `non_divergent_or_shared_near_exit_timing_window`

Interpretation：

- `near_exit_pair_is_not_phase27_branch_evidence`

Baseline seq=10：

- combined_subtype: `side_cost_or_timing_late_silent+cmd_silent_after_recovery_abort`
- recovery_time: `1779718710.580778`
- cmd_near_zero_after_recovery_sec: `0.515651`
- cmd_near_zero_ratio_5s_after_recovery: `1.0`
- path_updates_after_recovery: `6`
- pre/post path_ahead_1_0m_cost_max: `100 / 100`
- near_zero_path_ahead_1_0m_cost_max: `0`
- near_zero_first_high_cost_distance_m: `null`
- near_zero_high_cost_count: `0`
- near_zero_robot_to_path_distance_m: `0.024661610584981432`
- chosen_route_corridor_cost_max: `0`
- explored_candidate_corridor_cost_max: `99`
- explored_candidate_first_high_cost_distance_m: `0.8775`

Candidate seq=7：

- combined_subtype: `side_cost_or_timing_late_silent+cmd_silent_after_recovery_abort`
- recovery_time: `1779718908.1559718`
- cmd_near_zero_after_recovery_sec: `0.606157`
- cmd_near_zero_ratio_5s_after_recovery: `1.0`
- path_updates_after_recovery: `8`
- pre/post path_ahead_1_0m_cost_max: `100 / 100`
- near_zero_path_ahead_1_0m_cost_max: `0`
- near_zero_first_high_cost_distance_m: `null`
- near_zero_high_cost_count: `0`
- near_zero_robot_to_path_distance_m: `0.043648964851482314`
- chosen_route_corridor_cost_max: `46`
- explored_candidate_corridor_cost_max: `0`

Key readout：

- 两个 near-exit timeouts 都在 recovery 后 0.52～0.61s 内进入 cmd near-zero，后 5s 全 near-zero。
- path updates 仍有 6～8 次，说明不是简单“没有 path refresh”。
- near-zero snapshot 本身 path-ahead cost=0，但 pre/post recovery snapshot path_ahead=100，说明高成本窗口与 near-zero timing 存在时序错位。
- baseline explored candidate corridor dirty/high-cost，candidate explored corridor clean，但 pair 已被 Phase26L/26K 判为 non-divergent/shared，不构成 Phase27 branch evidence。

## Cross-pair common pattern

4/4 timeout cases：

- combined_subtype: `side_cost_or_timing_late_silent+cmd_silent_after_recovery_abort`
- controller_classification: `healthy_motion_but_late_stall`
- timing_subtype: `cmd_silent_after_recovery_abort`
- recovery 后 first cmd near-zero 很快出现：`0.066s`, `0.216s`, `0.516s`, `0.606s`
- recovery 后 5s cmd near-zero ratio: `1.0` in all 4 cases
- path_updates_after_recovery: `4`, `6`, `6`, `8`

因此更合理的下一步 hypothesis 是：

> recovery clear-costmap / controller abort 周期后，controller 持续收到 path updates，但 cmd_vel_nav 迅速进入 near-zero 并保持 silent；local-cost pressure 与 controller/recovery timing 交互导致 late stall。

这不是 branch-selection hypothesis。

## Decision

Phase27 remains blocked.

Decision output：

```json
{
  "phase27_candidate_signal": "not_supported",
  "next_recommendation": "continue_controller_local_cost_timing_diagnostics"
}
```

Guardrails：

- do_not_change_branch_selection
- do_not_tune_nav2_or_controller_from_phase26m_alone
- require_timing_root_cause_before_intervention

## Verification

Focused verification：

```bash
python3 -m py_compile tools/analyze_phase26m_timing_windows.py
python3 -m pytest \
  src/tugbot_maze/test/test_phase26m_timing_windows.py \
  src/tugbot_maze/test/test_phase26l_runtime_spatial_contract.py \
  src/tugbot_maze/test/test_phase26l_real_run_wrapper_contract.py \
  src/tugbot_maze/test/test_phase26l_overlap_compare_contract.py \
  -q
```

Result：`7 passed`.

Regression：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase24*.py src/tugbot_maze/test/test_phase26*.py -q
```

Result：`52 passed`.

Cleanup check：

```bash
ps -eo pid,cmd | grep -E 'run_phase21_controller_diagnostics_smoke|ros2 launch tugbot_bringup|record_explorer_state_series|record_controller_dynamics|record_post_recovery_snapshots|ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|maze_explorer|slam_toolbox|controller_server|planner_server|bt_navigator|gz sim|ruby .*gz sim' | grep -v grep || true
```

Result：no matching processes.

## Next recommendation

Recommended next phase: Phase26N, still analysis-first.

Focus on proving/disproving the timing hypothesis before any intervention：

1. Add or derive a per-goal timeline around timeout goals：
   - dispatch
   - recovery clear-costmap
   - progress failure
   - controller abort
   - first cmd near-zero after recovery
   - path_update cadence
   - local-cost high windows
2. Reconstruct whether cmd near-zero begins before or after controller abort/progress failure using controller dynamics, not only recorder snapshot near-zero onset.
3. If the timeline consistently shows cmd near-zero immediately after recovery while path updates continue, inspect MPPI/controller logs/critic failure conditions around that window.
4. Only after root cause is localized should a narrow reversible intervention be proposed; not branch scoring.
