# Phase 26O — Controller Server / MPPI Evidence Availability Analysis

Date: 2026-05-25
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 一句话结论

Phase26O 仍是 analysis-first：没有启动 ROS/Gazebo/Nav2，没有修改 branch scoring，没有改 Nav2/controller 参数。对 Phase26N 的同一批 4 个 timeout windows join launch/controller logs 后，4/4 都显示 controller_server 与 MPPI 已配置/激活，recovery 后 controller_server 收到新 goal，且 launch log 中仍有 fresh path updates；但现有 logs/artifacts 没有 per-cycle MPPI critic scores、trajectory validity/rejection reason、zero velocity reason 或 controller cycle outcome。因此 Phase26O 只能确认“controller active + fresh paths + cmd silent”这个边界，不能确认是哪一个 critic 压制速度。下一阶段应先开启/记录 MPPI debug evidence（critics_stats / optimal trajectory / controller cycle outcome / trajectory validator result），仍不能调参或进入 Phase27。

## 边界

本阶段只读取已有 artifacts：

- `log/phase26n_goal_timeline.json`
- `log/phase26l_baseline_run1_launch.log`
- `log/phase26l_candidate_run1_launch.log`
- `log/phase26l_*_params_fingerprint.json`
- source params YAML for MPPI debug/instrumentation availability

本阶段明确没有：

- 启动新的 Gazebo/Nav2 长跑；
- 修改 branch selection / branch scoring；
- 修改 Nav2 progress/controller/MPPI 参数；
- 用 Phase26O log availability 直接提出 tuning；
- 推动 Phase27。

## 新增文件

Analyzer：

- `tools/analyze_phase26o_controller_mppi_evidence.py`

Test：

- `src/tugbot_maze/test/test_phase26o_controller_mppi_evidence.py`

Artifacts：

- `log/phase26o_controller_mppi_evidence.json`
- `log/phase26o_controller_mppi_evidence.out`

## TDD 过程

RED：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase26o_controller_mppi_evidence.py -q
```

预期失败：

- `tools/analyze_phase26o_controller_mppi_evidence.py` 不存在。

GREEN：

实现 analyzer 后：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase26o_controller_mppi_evidence.py -q
```

结果：`1 passed`。

随后扩展测试，要求 analyzer 读取 params fingerprint 指向的 YAML，并输出：

- `visualize`
- `publish_critics_stats`
- `publish_optimal_trajectory`

再次通过。

## Analyzer 命令

```bash
python3 tools/analyze_phase26o_controller_mppi_evidence.py \
  --log-dir log \
  --timeline-json log/phase26n_goal_timeline.json \
  --output-json log/phase26o_controller_mppi_evidence.json \
  | tee log/phase26o_controller_mppi_evidence.out
```

Analyzer 输出：

- `controller_state`
  - MPPI controller configured / activated
  - recovery 后是否收到 new goal
  - control loop missed / exception / invalid control evidence
- `plan_freshness`
  - recovery 后 launch log 中的 `Passing new path to controller` 次数与时间
  - Phase26N recorder 的 path-update count 对照
- `mppi_critic_evidence`
  - loaded critics
  - params 中的 `visualize`, `publish_critics_stats`, `publish_optimal_trajectory`
  - 是否存在 per-cycle critic scores / trajectory validity / zero velocity reason logs
- `local_cost_vs_cmd_timing`
  - high local-cost window 是否早于 first cmd near-zero
- `instrumentation_gaps`
- `inference`

## Cross-case summary

4/4 cases：

- inference: `controller_active_and_fresh_paths_but_no_per_cycle_mppi_critic_evidence`
- MPPI configured: yes
- MPPI activated: yes
- recovery 后 controller_server 收到 new goal: yes
- recovery 后 launch log 中仍有 path updates: yes
- per-cycle critic scores present: 0/4
- trajectory validity/rejection reason logs present: 0/4
- zero velocity reason/controller cycle outcome logs present: 0/4
- high local-cost before first cmd near-zero: 2/4（两个 near-exit cases）

Loaded critics in both baseline/candidate logs：

- ConstraintCritic
- CostCritic
- GoalCritic
- GoalAngleCritic
- PathAlignCritic
- PathFollowCritic
- PathAngleCritic
- PreferForwardCritic

Params/instrumentation availability：

- `visualize: true` in baseline and candidate params
- `publish_critics_stats`: not explicitly set in current params, so default false by Nav2 docs
- `publish_optimal_trajectory`: not explicitly set in current params, so default false by Nav2 docs
- no launch-log evidence of `~/critics_stats`, optimal trajectory publication, trajectory validation failures, or explicit zero-velocity reason

Nav2 MPPI docs note：

- `visualize` publishes debugging trajectories and critic statistics; it can slow the controller substantially.
- `publish_critics_stats` publishes `nav2_msgs/msg/CriticsStats` with critic names, whether they changed costs, and sum of costs added by each critic; useful for debugging/tuning but not generic runtime use.
- `publish_optimal_trajectory` publishes the optimal trajectory for visualization/debugging/lower-level systems.

## Case details

### Case 1 — baseline early timeout

Run/seq：`phase26l_baseline_run1:2`

- inference: `controller_active_and_fresh_paths_but_no_per_cycle_mppi_critic_evidence`
- received goal after recovery: `true`, at `0.059868s`
- launch-log path updates after recovery: `5`
- first launch-log path update after recovery: `1.109982s`
- Phase26N recorder path updates: `4`
- high local-cost windows after recovery: `0`
- first cmd near-zero after recovery: `0.066265s`
- per-cycle critic scores: absent
- trajectory validity logs: absent
- zero velocity reason logs: absent

Interpretation：controller_server 迅速收到新 goal，后续仍有 path updates；但 cmd near-zero 早于 launch-log first path update。Phase26N recorder 曾捕获更早 path_update，因此 launch log 粒度不足以解释 silent 原因。

### Case 2 — candidate early timeout

Run/seq：`phase26l_candidate_run1:2`

- inference: `controller_active_and_fresh_paths_but_no_per_cycle_mppi_critic_evidence`
- received goal after recovery: `true`, at `0.209865s`
- launch-log path updates after recovery: `5`
- first launch-log path update after recovery: `1.259976s`
- Phase26N recorder path updates: `6`
- high local-cost windows after recovery: `0`
- first cmd near-zero after recovery: `0.215700s`
- per-cycle critic scores: absent
- trajectory validity logs: absent
- zero velocity reason logs: absent

Interpretation：与 baseline early timeout 一致。现有 launch log 只能证明 controller/path 仍活跃，不能说明 MPPI 为什么输出 near-zero。

### Case 3 — baseline near-exit timeout

Run/seq：`phase26l_baseline_run1:10`

- inference: `controller_active_and_fresh_paths_but_no_per_cycle_mppi_critic_evidence`
- received goal after recovery: `true`, at `0.509929s`
- launch-log path updates after recovery: `5`
- first launch-log path update after recovery: `1.560032s`
- Phase26N recorder path updates: `6`
- high local-cost windows after recovery: `6`
- first high-cost window after recovery: `0.320968s`
- first cmd near-zero after recovery: `0.515651s`
- high local-cost before first cmd near-zero: `true`
- per-cycle critic scores: absent
- trajectory validity logs: absent
- zero velocity reason logs: absent

Interpretation：near-exit baseline 支持 high local-cost window 早于 first cmd near-zero，但现有 MPPI evidence 不足以判断是 CostCritic、PathAlign/Follow，还是 trajectory validator/footprint collision condition 造成 silent。

### Case 4 — candidate near-exit timeout

Run/seq：`phase26l_candidate_run1:7`

- inference: `controller_active_and_fresh_paths_but_no_per_cycle_mppi_critic_evidence`
- received goal after recovery: `true`, at `0.600044s`
- launch-log path updates after recovery: `7`
- first launch-log path update after recovery: `1.650157s`
- Phase26N recorder path updates: `8`
- high local-cost windows after recovery: `8`
- first high-cost window after recovery: `0.079227s`
- first cmd near-zero after recovery: `0.606157s`
- high local-cost before first cmd near-zero: `true`
- per-cycle critic scores: absent
- trajectory validity logs: absent
- zero velocity reason logs: absent

Interpretation：这是最强的 high-cost-before-silent case，但仍缺 critic/cycle-level evidence。不能据此直接调 CostCritic 或 branch scoring。

## 核心问题回答

1. MPPI 是否仍在生成 trajectory，但被 CostCritic / ObstaclesCritic / PathAlign / PathFollow / GoalCritic 等 critic 压到 near-zero？

现有 evidence 不足。可以确认 MPPI 已配置/激活，loaded critics 包含 CostCritic、GoalCritic、PathAlignCritic、PathFollowCritic 等；但没有 per-cycle critic scores、selected trajectory cost 或 trajectory validity logs，不能判断具体 critic 是否压制速度。

2. controller_server 是否仍 active 且收到 fresh plan，但内部因 cost/trajectory invalid/retry/exception 输出 zero velocity？

可以确认 controller_server/MPPI active，recovery 后收到 new goal，并持续收到/传递 fresh path updates。没有 exception/invalid-control/zero-velocity reason logs，因此不能确认内部具体 reason。

3. near-exit cases 中 high local-cost window 早于 first cmd near-zero，这个顺序是否稳定？

在 Phase26L 的两个 near-exit timeout cases 中稳定成立：

- baseline seq10: high-cost at 0.321s，cmd near-zero at 0.516s
- candidate seq7: high-cost at 0.079s，cmd near-zero at 0.606s

但 early timeout cases 没有 post-recovery high-cost windows，因此这个 pattern 目前只适用于 near-exit subtype，不应泛化到全部 timeouts。

4. 是否需要开启或解析更细粒度 controller/MPPI debug 信息？

需要。Phase26O 的主要产出就是 instrumentation gap：当前 logs 只能证明 controller/path 活跃，不能解释 zero/near-zero output reason。下一步应先采集 MPPI critic/controller-cycle 证据，再谈 intervention。

## Decision

Phase27 remains blocked.

Decision output：

```json
{
  "phase27_candidate_signal": "not_supported",
  "parameter_tuning_signal": "not_supported",
  "next_recommendation": "add_or_enable_mppi_controller_debug_evidence_before_intervention"
}
```

Guardrails：

- do_not_change_branch_selection
- do_not_tune_controller_from_phase26n_or_phase26o_log_availability_alone
- collect_critic_or_controller_cycle_reason_before_intervention

## Next recommendation: Phase26P

建议进入 Phase26P，仍 analysis-first。目标不是调参，而是做最小侵入的 evidence collection。

Phase26P 建议方向：

1. 增加一个 diagnostics-only MPPI evidence profile 或 wrapper，不改变 planner/controller semantic tuning：
   - 只开启/记录 debug topics 或 logs；
   - 保持 branch selection 和 MPPI cost/velocity 参数不变；
   - 若必须设置 `publish_critics_stats` / `publish_optimal_trajectory` / visualization 相关项，作为 instrumentation profile，不作为 candidate tuning profile。
2. 记录 post-recovery windows 的：
   - `/controller_server/FollowPath/critics_stats` 或实际 topic 名称；
   - optimal trajectory / transformed global plan / trajectories marker（按实际 Nav2 topic）；
   - cmd_vel_nav、plan updates、local cost snapshots、goal_events 的同一时间基准。
3. Phase26P analyzer 应 join：
   - first cmd near-zero 前后 1s；
   - high local-cost window；
   - critic changed-cost flags / cost sums；
   - selected/optimal trajectory velocities；
   - trajectory validator accept/reject result。
4. 只有当 Phase26P 明确定位到具体 critic/condition（例如 CostCritic collision/near-collision, PathAlign occupancy, trajectory validator reject, or no valid control），才讨论 narrow reversible intervention。

## Verification

Focused verification：

```bash
python3 -m py_compile tools/analyze_phase26o_controller_mppi_evidence.py
python3 -m pytest src/tugbot_maze/test/test_phase26o_controller_mppi_evidence.py -q
```

Result：`1 passed`。

Adjacent diagnostics regression：

```bash
python3 -m pytest \
  src/tugbot_maze/test/test_phase26m_timing_windows.py \
  src/tugbot_maze/test/test_phase26n_goal_timeline.py \
  src/tugbot_maze/test/test_phase26o_controller_mppi_evidence.py \
  src/tugbot_maze/test/test_phase24d_enrich_post_recovery_snapshots.py \
  src/tugbot_maze/test/test_phase24c_post_recovery_recorder_contract.py \
  src/tugbot_maze/test/test_phase23b_nav2_event_timestamps.py \
  -q
```

Result：`8 passed`。

Phase24/25/26 regression：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase24*.py src/tugbot_maze/test/test_phase25*.py src/tugbot_maze/test/test_phase26*.py -q
```

Result：`73 passed`。

Cleanup check：

```bash
ps -eo pid,cmd | grep -E 'run_phase21_controller_diagnostics_smoke|ros2 launch tugbot_bringup|record_explorer_state_series|record_controller_dynamics|record_post_recovery_snapshots|ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|maze_explorer|slam_toolbox|controller_server|planner_server|bt_navigator|gz sim|ruby .*gz sim' | grep -v grep || true
```

Result：空输出，未发现残留 ROS/Gazebo/Nav2 进程。
