# Phase 26L — Runtime Spatial Diagnostics Contract (Analysis-Only)

Date: 2026-05-25
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 一句话结论

Phase26L 保持 analysis-only，没有改 branch selection、Nav2/controller 参数或 runtime 行为。本阶段补齐了后续真实运行所需的 runtime spatial diagnostics contract：snapshot 现在可以记录 path-ahead high-cost cell coordinates、high-cost count、nearest high-cost point、centroid、first high-cost distance，并分别采样 chosen route corridor 与 selected explored candidate corridor 的 local-cost。对既有 Phase26G/26K artifacts 运行 analyzer 后，结论仍是 `phase27_candidate_signal: not_supported`，因为旧 artifacts 没有这些新空间字段；synthetic/future-style artifact 只证明 contract gate 能识别“chosen-away route high-cost + explored candidate corridor clean”的形状，不构成 Phase27 依据。

## 目标与边界

用户要求进入 Phase26L，仍 analysis-only：

1. 扩展 recorder/analyzer，使 snapshot 记录 path-ahead high-cost cell coordinates。
2. 记录 high-cost cell count、nearest high-cost point、high-cost centroid。
3. 对 chosen route 与 explored candidate corridor 分别采样 local-cost corridor。
4. 统计 first high-cost distance from dispatch / sampled segment start。
5. 只有后续真实 artifacts 证明 chosen-away route high-cost、explored candidate corridor clean，且 failed-only matched repeats 稳定，才重新讨论 Phase27。

本阶段没有启动 ROS/Gazebo/Nav2，没有运行 autonomous smoke，没有改变实际导航策略。

## 新增/修改文件

修改 recorder：

- `tools/record_post_recovery_snapshots.py`

新增 analyzer：

- `tools/analyze_phase26l_runtime_spatial_contract.py`

新增 tests：

- `src/tugbot_maze/test/test_phase26l_runtime_spatial_contract.py`

新增 artifacts：

- `log/phase26l_runtime_spatial_contract_existing_artifacts.json`
- `log/phase26l_runtime_spatial_contract_existing_artifacts.out`
- `log/phase26l_runtime_spatial_contract_synthetic_signal.json`

## TDD 过程

RED：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase26l_runtime_spatial_contract.py -q
```

预期失败并已观察到：

- recorder 缺少 `path_ahead_1_0m_high_cost_points` 等 Phase26L spatial fields。
- analyzer `tools/analyze_phase26l_runtime_spatial_contract.py` 不存在。
- 3 个 tests fail，证明 contract 在实现前确实约束缺口。

GREEN：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase26l_runtime_spatial_contract.py -q
```

结果：`3 passed`。

## Recorder contract

`record_post_recovery_snapshots.py` 保持原有 Phase24C/24D recorder 语义，并新增 Phase26L 诊断字段：

Path-ahead 1.0m high-cost fields：

- `path_ahead_1_0m_high_cost_points`
- `path_ahead_1_0m_high_cost_count`
- `path_ahead_1_0m_high_cost_centroid`
- `path_ahead_1_0m_nearest_high_cost_point`
- `path_ahead_1_0m_first_high_cost_distance_m`

Chosen route corridor fields：

- `chosen_route_corridor_cost_max`
- `chosen_route_corridor_cost_mean`
- `chosen_route_first_high_cost_distance_m`
- `chosen_route_high_cost_count`

Selected explored candidate corridor fields：

- `selected_explored_candidate_target`
- `explored_candidate_corridor_cost_max`
- `explored_candidate_corridor_cost_mean`
- `explored_candidate_first_high_cost_distance_m`
- `explored_candidate_high_cost_count`

Implementation notes：

- `CostmapView._line_samples()` emits world coordinates, cost value, and distance along the sampled segment.
- `_high_cost_summary()` summarizes high-cost points using threshold `>= 70`.
- `_route_corridor_summary()` samples from `dispatch_pose` to chosen target or selected explored candidate target.
- `--max-high-cost-points` bounds serialized high-cost coordinate payload size; default is `20`.
- The recorder still subscribes to `/local_costmap/costmap` rather than `/local_costmap/costmap_raw`.

## Analyzer contract

命令形态：

```bash
python3 tools/analyze_phase26l_runtime_spatial_contract.py \
  --log-dir log \
  --failed-runs phase26g_candidate_run2 \
  --compare-runs phase26g_candidate_run2,phase26g_baseline_run1 \
  --output-json log/phase26l_runtime_spatial_contract_existing_artifacts.json
```

Analyzer 输出：

- per-run cases
- per-case route context
- spatial contract completeness
- path-ahead high-cost cell evidence
- chosen route high-cost flag
- explored candidate corridor clean flag
- classification
- decision guardrails

Core classification：

- `chosen_high_explored_clean_contract_signal`
  - failed-like outcome (`timeout` or `failure`)
  - route context is `route_divergence_chosen_moves_away_rejected_moves_toward_exit`
  - high-cost cell coordinates are present
  - chosen corridor has high cost
  - explored candidate corridor is clean
- `insufficient_or_dirty_explored_corridor`
  - missing required fields, missing high-cost coordinates, explored corridor not clean, non-failed case, or route context not strong enough

## Existing artifacts result

Command：

```bash
python3 tools/analyze_phase26l_runtime_spatial_contract.py \
  --log-dir log \
  --failed-runs phase26g_candidate_run2 \
  --compare-runs phase26g_candidate_run2,phase26g_baseline_run1 \
  --output-json log/phase26l_runtime_spatial_contract_existing_artifacts.json \
  | tee log/phase26l_runtime_spatial_contract_existing_artifacts.out
```

Decision：

```json
{
  "phase27_candidate_signal": "not_supported",
  "next_recommendation": "complete_runtime_spatial_contract_and_repeat_failed_only_matches"
}
```

Key details from existing artifacts：

- failed run count: `1`
- spatial contract complete case count: `0`
- chosen-high/explored-clean signal count: `0`
- `phase26g_candidate_run2` seq=8 remains route-divergence evidence, but old snapshots do not contain:
  - high-cost cell coordinates
  - high-cost centroid
  - nearest high-cost point
  - chosen/explored corridor cost samples

Therefore existing artifacts cannot prove explored candidate corridor is clean and cannot justify Phase27.

## Synthetic/future-style contract result

A synthetic artifact with complete Phase26L fields was used only to verify the analyzer gate. It produced：

```json
{
  "phase27_candidate_signal": "possible_only_after_real_matched_repeat_validation",
  "next_recommendation": "collect_real_phase26l_artifacts_before_any_phase27"
}
```

This is expected and intentionally guarded. It means the analyzer can detect the desired evidence shape, not that we have real evidence.

## Verification

Syntax：

```bash
python3 -m py_compile \
  tools/record_post_recovery_snapshots.py \
  tools/analyze_phase26l_runtime_spatial_contract.py \
  tools/analyze_phase26k_spatial_local_cost_alignment.py
```

Result：pass.

Targeted tests：

```bash
python3 -m pytest \
  src/tugbot_maze/test/test_phase26l_runtime_spatial_contract.py \
  src/tugbot_maze/test/test_phase26k_spatial_local_cost_alignment.py \
  src/tugbot_maze/test/test_phase24c_post_recovery_recorder_contract.py \
  -q
```

Result：`7 passed`.

Phase26 family tests：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase26*.py -q
```

Result：`38 passed`.

Phase24 + Phase26 regression tests：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase24*.py src/tugbot_maze/test/test_phase26*.py -q
```

Result：`48 passed`.

Cleanup/process check：

```bash
ps -eo pid,cmd | grep -E 'ros2 launch tugbot_bringup|record_explorer_state_series|record_controller_dynamics|record_post_recovery_snapshots|ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|maze_explorer|slam_toolbox|controller_server|planner_server|bt_navigator|gz sim|ruby .*gz sim' | grep -v grep || true
```

Result：no matching ROS/Gazebo/Nav2/recorder processes.

## Interpretation

Phase26K showed the current evidence is ambiguous: high-cost samples looked more chosen-route-dominant, but near-zero ray and chosen/explored route geometry overlapped near dispatch. Phase26L does not resolve that with old logs; instead it makes the next real run capable of resolving it.

The new required evidence for reopening Phase27 discussion is strict：

1. Real Phase26L artifacts, not synthetic contract rows.
2. Failed-only matched repeats, not a single failure case.
3. Stable same-start-node route divergence.
4. Chosen-away route corridor high-cost.
5. Explored candidate corridor clean.
6. High-cost coordinates/centroid/nearest point align spatially with chosen route rather than shared dispatch choke.
7. No blocked/blacklist regression and no exit-reaching regression in comparison runs.

Until all above are true, Phase27 remains blocked.

## Recommendation for next phase

Next phase should be Phase26M or Phase26L-real-run validation, still analysis-first:

1. Run a controlled matched repeat smoke with the updated recorder enabled.
2. Capture:
   - `*_goal_events.jsonl`
   - `*_post_recovery_snapshots.jsonl`
   - controller dynamics artifacts if already in wrapper
   - Nav2/controller logs if available
3. Run `analyze_phase26l_runtime_spatial_contract.py` on failed-only runs.
4. Only if repeated failed cases produce `chosen_high_explored_clean_contract_signal`, compare against successful/EXIT_REACHED cases for absence of the same pattern.
5. If the signal is still mixed/shared/dirty, continue local-cost/controller diagnostics rather than branch-selection changes.

No Phase27 behavior change is recommended from Phase26L alone.
