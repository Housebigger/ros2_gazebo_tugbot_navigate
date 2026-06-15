# Phase 26N — Per-Goal Controller / Local-Cost Timeline Alignment

Date: 2026-05-25
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 一句话结论

Phase26N 继续 analysis-first，没有启动 ROS/Gazebo/Nav2，也没有改 branch scoring 或 Nav2/controller 参数。对 Phase26L 的 4 个 matched timeout cases 构建 per-goal timeline 后，4/4 都显示：progress failure / controller abort 先发生，随后 recovery clear-costmap，之后 cmd_vel_nav 在 0.066s～0.606s 内进入 near-zero；同时 recovery 后 path updates 仍持续 4～8 次。因此当前证据不支持“cmd near-zero 在 progress failure / controller abort 之前就开始”的假设，而支持“recovery/abort 后 controller 仍收到路径更新但输出迅速 silent”的故障定位。下一阶段应进入 MPPI/controller logs 与 critic 条件的窗口级诊断，不应从 Phase26M/N 直接调参或改 branch selection。

## 边界

本阶段只读取已有 Phase26L/Phase26M artifacts：

- `*_goal_events.jsonl`
- `*_goal_nav2_analysis.json`
- `*_timeout_subtypes.json`
- `*_post_recovery_snapshots.jsonl`
- `*_controller_dynamics.jsonl`

本阶段明确没有：

- 启动新的 Gazebo/Nav2 长跑；
- 修改 branch selection / branch scoring；
- 修改 Nav2 progress/controller/MPPI 参数；
- 推动 Phase27。

## 新增文件

Analyzer：

- `tools/analyze_phase26n_goal_timeline.py`

Test：

- `src/tugbot_maze/test/test_phase26n_goal_timeline.py`

Artifacts：

- `log/phase26n_goal_timeline.json`
- `log/phase26n_goal_timeline.out`

## TDD 过程

RED：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase26n_goal_timeline.py -q
```

预期失败：

- `tools/analyze_phase26n_goal_timeline.py` 不存在。

GREEN：

实现 analyzer 后：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase26n_goal_timeline.py -q
```

结果：`1 passed`。

随后针对 Phase26M/N 邻近链路做 focused regression：

```bash
python3 -m py_compile tools/analyze_phase26n_goal_timeline.py
python3 -m pytest src/tugbot_maze/test/test_phase26n_goal_timeline.py -q
```

结果：`1 passed`。

```bash
python3 -m pytest \
  src/tugbot_maze/test/test_phase26m_timing_windows.py \
  src/tugbot_maze/test/test_phase26n_goal_timeline.py \
  src/tugbot_maze/test/test_phase24d_enrich_post_recovery_snapshots.py \
  src/tugbot_maze/test/test_phase24c_post_recovery_recorder_contract.py \
  src/tugbot_maze/test/test_phase23b_nav2_event_timestamps.py \
  -q
```

结果：`7 passed`。

Phase24/25/26 regression：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase24*.py src/tugbot_maze/test/test_phase25*.py src/tugbot_maze/test/test_phase26*.py -q
```

结果：`72 passed`。

Cleanup check：

```bash
ps -eo pid,cmd | grep -E 'run_phase21_controller_diagnostics_smoke|ros2 launch tugbot_bringup|record_explorer_state_series|record_controller_dynamics|record_post_recovery_snapshots|ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|maze_explorer|slam_toolbox|controller_server|planner_server|bt_navigator|gz sim|ruby .*gz sim' | grep -v grep || true
```

结果：空输出，未发现残留 ROS/Gazebo/Nav2 进程。

## Analyzer 命令

```bash
python3 tools/analyze_phase26n_goal_timeline.py \
  --log-dir log \
  --case phase26l_baseline_run1:2 \
  --case phase26l_candidate_run1:2 \
  --case phase26l_baseline_run1:10 \
  --case phase26l_candidate_run1:7 \
  --output-json log/phase26n_goal_timeline.json \
  | tee log/phase26n_goal_timeline.out
```

Analyzer 输出：

- `classification`
- `cmd_near_zero_relation`
  - recovery time
  - progress failure time
  - controller abort time
  - first cmd near-zero time
  - first cmd near-zero after recovery delta
  - 是否早于 progress failure / controller abort
- `path_update_cadence`
- `local_cost_windows`
- `timeline[]`：按时间排序的 dispatch / recovery / progress failure / controller abort / first cmd near-zero / path_update / local-cost high window / timeout。

## Cases

### Case 1 — baseline early timeout

Run/seq：`phase26l_baseline_run1:2`

- classification: `cmd_near_zero_after_recovery_with_paths_continuing`
- recovery time: `1779718598.034463`
- progress failure time: `1779718598.019699`
- controller abort time: `1779718598.0205848`
- first cmd near-zero time: `1779718598.1007276`
- first cmd near-zero after recovery: `0.066265s`
- cmd near-zero before progress failure: `false`
- cmd near-zero before controller abort: `false`
- path updates after recovery: `4`
- first/last path update after recovery: `0.041124s / 3.120691s`
- high local-cost windows after recovery: `0`

Readout：progress failure / controller abort 比最后一次 recovery 早约 0.014s；cmd near-zero 在 recovery 后 0.066s 才出现。recovery 后 path update 先于 first cmd near-zero 出现，且继续到 3.12s。

### Case 2 — candidate early timeout

Run/seq：`phase26l_candidate_run1:2`

- classification: `cmd_near_zero_after_recovery_with_paths_continuing`
- recovery time: `1779718836.5433936`
- progress failure time: `1779718836.528734`
- controller abort time: `1779718836.5293233`
- first cmd near-zero time: `1779718836.759094`
- first cmd near-zero after recovery: `0.215700s`
- cmd near-zero before progress failure: `false`
- cmd near-zero before controller abort: `false`
- path updates after recovery: `6`
- first/last path update after recovery: `0.191032s / 5.320500s`
- high local-cost windows after recovery: `0`

Readout：与 baseline early timeout 一致。cmd near-zero 不是在 progress failure/controller abort 前开始，而是在 recovery 后约 0.216s 出现；path updates 继续到 5.32s。

### Case 3 — baseline near-exit timeout

Run/seq：`phase26l_baseline_run1:10`

- classification: `cmd_near_zero_after_recovery_with_paths_continuing`
- recovery time: `1779718710.580778`
- progress failure time: `1779718710.5660148`
- controller abort time: `1779718710.5665317`
- first cmd near-zero time: `1779718711.0964284`
- first cmd near-zero after recovery: `0.515651s`
- cmd near-zero before progress failure: `false`
- cmd near-zero before controller abort: `false`
- path updates after recovery: `6`
- first/last path update after recovery: `0.491029s / 5.620800s`
- high local-cost windows after recovery: `6`
- first high-cost window after recovery: `0.320968s`
- max path-ahead cost: `100`

Readout：near-exit baseline 中 local-cost high window 在 recovery 后 0.321s 出现，早于 first cmd near-zero 0.516s；path update 在 0.491s 出现并持续。这里更像 recovery 后 local-cost/controller timing window 与 MPPI critic 条件交互，而不是 branch scoring 证据。

### Case 4 — candidate near-exit timeout

Run/seq：`phase26l_candidate_run1:7`

- classification: `cmd_near_zero_after_recovery_with_paths_continuing`
- recovery time: `1779718908.1559718`
- progress failure time: `1779718908.136206`
- controller abort time: `1779718908.1365342`
- first cmd near-zero time: `1779718908.7621286`
- first cmd near-zero after recovery: `0.606157s`
- cmd near-zero before progress failure: `false`
- cmd near-zero before controller abort: `false`
- path updates after recovery: `8`
- first/last path update after recovery: `0.581470s / 7.760745s`
- high local-cost windows after recovery: `8`
- first high-cost window after recovery: `0.079227s`
- max path-ahead cost: `100`

Readout：high local-cost window 几乎立即出现在 recovery 后 0.079s，path update 在 0.581s，first cmd near-zero 在 0.606s；后续 path updates 继续到 7.76s。这是最强的“recovery 后 path updates 持续但 controller 输出 silent”案例。

## Cross-case pattern

4/4 cases：

- `combined_subtype = side_cost_or_timing_late_silent+cmd_silent_after_recovery_abort`
- classification: `cmd_near_zero_after_recovery_with_paths_continuing`
- first cmd near-zero after recovery: `0.066s`, `0.216s`, `0.516s`, `0.606s`
- cmd near-zero before progress failure: `0/4`
- cmd near-zero before controller abort: `0/4`
- path updates after recovery: `4`, `6`, `6`, `8`
- near-exit cases有 sustained high local-cost windows after recovery: `6`, `8`

核心问题回答：

> cmd near-zero 是在 progress failure / controller abort 之前就开始，还是 recovery/abort 后才开始？

Phase26N 的答案是：在这 4 个 Phase26L timeout cases 中，cmd near-zero 不是在 progress failure / controller abort 前开始；它是在 progress failure/controller abort 以及 recovery clear-costmap 之后 0.066s～0.606s 内出现。并且 recovery 后 controller 仍收到 path updates。

## Decision

Phase27 remains blocked.

Decision output：

```json
{
  "phase27_candidate_signal": "not_supported",
  "next_recommendation": "inspect_controller_mppi_critic_logs_before_intervention"
}
```

Guardrails：

- do_not_change_branch_selection
- do_not_tune_controller_without_mppi_critic_evidence
- timeline_evidence_only_not_intervention

## Next recommendation: Phase26O

建议进入 Phase26O，仍 analysis-first。

目标：在同一批 timeout windows 上采集/解析 controller_server / MPPI 级别证据，解释为什么 recovery 后 path updates 仍持续但 cmd_vel_nav 迅速 near-zero/silent。

优先诊断问题：

1. MPPI 是否在 post-recovery window 中持续生成 trajectory，但被 CostCritic / ObstaclesCritic / PathAlign / PathFollow / GoalCritic 等 critic 压到 near-zero？
2. controller_server 是否在 recovery 后仍 active 且收到 plan，但内部因 cost/trajectory invalid/retry/exceptions 输出 zero velocity？
3. high local-cost windows 与 first cmd near-zero 的相对顺序是否稳定：near-exit cases 中 high-cost 早于 cmd near-zero，early cases 中 high-cost evidence 不充分。
4. 是否需要更细粒度日志或 debug topic：例如 MPPI critic scores、trajectory validity、controller cycle outcome、plan accepted timestamp、costmap footprint collision check。

仍然不要直接调参。只有当 Phase26O 能把 silent 输出定位到具体 critic/condition 后，才讨论 narrow reversible intervention。
