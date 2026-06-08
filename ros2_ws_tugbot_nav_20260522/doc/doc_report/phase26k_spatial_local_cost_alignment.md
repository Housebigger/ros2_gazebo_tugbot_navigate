# Phase 26K — Spatial / Local-Cost Alignment for High-Cost Forward-Shift Cases

Date: 2026-05-25
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 一句话结论

Phase26K 完成 analysis-only spatial/local-cost alignment。以 `phase26g_candidate_run2` seq=8 为 anchor，对比同 run seq=10 与 `phase26g_baseline_run1` seq=7 后，当前 artifacts 显示 high-cost samples 主要沿 chosen route 分布，但 anchor near-zero ray 与 chosen/explored 两条 route 在 dispatch 附近几何上重合，classification 为 `shared_choke_or_ambiguous`。这仍不支持 Phase27，也不能证明 explored candidate corridor 是 clean 可安全替代路线；下一步应继续 runtime spatial diagnostics，而不是改 branch selection 或调参。

## 目标与边界

用户要求 Phase26K 仍保持 analysis-only，聚焦 spatial/local-cost alignment：

1. 以 `phase26g_candidate_run2` seq=8 为 anchor。
2. 对比同 run seq=10 和 `phase26g_baseline_run1` seq=7。
3. 比较 chosen target、explored candidate target、near-zero robot pose、path-ahead high-cost segment 的空间关系。
4. 判断 seq=8 的 near-zero high-cost 是在 chosen-away route、explored candidate corridor，还是 shared near-exit choke point。
5. 如果是 shared choke point，继续 local-cost/controller diagnostics；如果 explored candidate corridor clean 且 chosen-away route high-cost，再考虑补 runtime diagnostics，但仍不直接 Phase27。

本阶段没有启动 ROS/Gazebo/Nav2，没有修改 runtime branch choice，没有修改 Nav2 或 controller 参数。

## 新增/修改文件

新增 analyzer：

- `tools/analyze_phase26k_spatial_local_cost_alignment.py`

新增 tests：

- `src/tugbot_maze/test/test_phase26k_spatial_local_cost_alignment.py`

新增 output artifact：

- `log/phase26k_spatial_local_cost_alignment.json`
- `log/phase26k_spatial_local_cost_alignment.out`

新增报告：

- `doc/doc_report/phase26k_spatial_local_cost_alignment.md`

## TDD 过程

RED：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase26k_spatial_local_cost_alignment.py -q
```

预期失败：

- `tools/analyze_phase26k_spatial_local_cost_alignment.py` 不存在。
- 2 个 tests fail，证明 Phase26K spatial alignment contract 正在约束 analyzer。

GREEN：

实现 analyzer 后 targeted tests 通过：

```bash
python3 -m py_compile tools/analyze_phase26k_spatial_local_cost_alignment.py
python3 -m pytest src/tugbot_maze/test/test_phase26k_spatial_local_cost_alignment.py src/tugbot_maze/test/test_phase26j_forward_shift_artifact_join.py src/tugbot_maze/test/test_phase26i_forward_shift_stats.py -q
```

结果：`6 passed`。

Phase26 family tests：

```bash
python3 -m pytest src/tugbot_maze/test/test_phase26*.py -q
```

结果：`35 passed`。

## Analyzer 功能

命令：

```bash
python3 tools/analyze_phase26k_spatial_local_cost_alignment.py \
  --log-dir log \
  --anchor-run phase26g_candidate_run2 \
  --anchor-seq 8 \
  --compare-cases phase26g_candidate_run2:10:candidate_run2_seq10,phase26g_baseline_run1:7:baseline_run1_seq7 \
  --output-json log/phase26k_spatial_local_cost_alignment.json
```

Analyzer 输出：

- anchor case geometry：
  - dispatch_xy
  - chosen_target
  - explored_candidate_target
  - chosen/explored target_exit_dist
  - chosen/explored target local cost
- high-cost path-ahead sample alignment：
  - point distance to chosen route segment
  - point distance to explored candidate route segment
  - point alignment classification
- near-zero alignment：
  - near-zero robot pose
  - 1m heading ray endpoint
  - ray distance to chosen/explored route segments
  - ray endpoint distance to chosen/explored target
- comparison overlap：
  - min high-cost sample distance between anchor and comparison cases
- decision：
  - spatial_interpretation
  - phase27_candidate_signal
  - next_recommendation
  - guardrails

## Anchor: phase26g_candidate_run2 seq=8

### Geometry

- dispatch_xy: `[2.843252, 3.128520]`
- chosen target: `[2.774027, 2.218735]`
- chosen target_exit_dist: `1.453748 m`
- chosen target local cost: `73`
- explored candidate target: `[3.547953, 3.668447]`
- explored candidate target_exit_dist: `0.806950 m`
- explored candidate target local cost: `99`
- explored candidate target local-cost max radius: `99`
- route_context: `route_divergence_chosen_moves_away_rejected_moves_toward_exit`
- outcome: timeout

### High-cost distribution

- high_cost_sample_count: `28`
- classification: `chosen_route_dominant`
- point_alignment_counts:
  - chosen_route: `17`
  - ambiguous_between_chosen_and_explored: `8`
  - shared_dispatch_or_corridor_choke: `3`

Interpretation：

- 大部分 high-cost samples 更贴近 chosen route。
- 但早期/near-zero 附近 chosen 与 explored route 在 dispatch 附近仍存在几何重叠/ambiguous 区域。
- 当前 recorder 只记录 path-ahead ray cost，不记录 high-cost cells 的实际坐标，因此不能断言 explored corridor clean。

### Near-zero alignment

Near-zero snapshot：

- robot_pose: `[2.521591, 3.117904, -0.021749]`
- path_ahead_0_5m_cost_max: `0`
- path_ahead_1_0m_cost_max: `99`
- path_ahead_1_0m_cost_mean: `23.476`
- robot_to_path_distance_m: `0.02268`
- post_recovery_path_update_count: `5`

Near-zero ray alignment：

- classification: `shared_dispatch_or_corridor_choke`
- point alignment: `ambiguous_between_chosen_and_explored`
- distance_to_chosen_route_m: `0.319929`
- distance_to_explored_candidate_route_m: `0.321836`
- ray_distance_to_chosen_route_m: `0.017609`
- ray_distance_to_explored_candidate_route_m: `0.017609`
- ray_endpoint_to_chosen_target_m: `1.152547`
- ray_endpoint_to_explored_candidate_target_m: `0.572909`

Interpretation：

- Near-zero 时 robot 的 1m heading ray 与 chosen/explored 两条 route segment 距离几乎相同，因为它们从同一 dispatch/start region 分叉。
- path-ahead 1m 高成本不一定只属于 chosen-away route；在当前几何近似与 recorder 信息下，它更像 shared/ambiguous near-dispatch choke。

## Comparison 1: phase26g_candidate_run2 seq=10

- outcome: success
- route_context: `non_divergent_chosen_also_toward_exit`
- chosen target: `[3.183484, 2.245433]`
- explored candidate target: `[3.437795, 3.710553]`
- high_cost_sample_count: `10`
- high_cost distribution: `chosen_route_dominant`
- point_alignment_counts:
  - chosen_route: `10`
- near_zero classification: `chosen_route`
- min high-cost overlap with anchor: `0.005303 m`

Interpretation：

- seq=10 是 success，却与 anchor seq=8 的 high-cost samples 存在极近空间 overlap。
- 这说明 high-cost chosen-route area 不是单独决定 failure 的充分条件。
- 它也提示局部 choke/near-exit geometry 可能被多个 successful/failed paths 共用。

## Comparison 2: phase26g_baseline_run1 seq=7

- outcome: success
- final mode: EXIT_REACHED run
- route_context: `non_divergent_chosen_also_toward_exit`
- chosen target: `[2.470591, 2.157290]`
- explored candidate target: `[3.019580, 3.122232]`
- high_cost_sample_count: `6`
- high_cost distribution: `chosen_route_dominant`
- point_alignment_counts:
  - chosen_route: `6`
- near-zero path-ahead 1.0m cost max: `0`
- min high-cost overlap with anchor: `0.302933 m`

Interpretation：

- baseline success run 也出现 chosen-route high-cost samples。
- near-zero snapshot 本身没有 high-cost，但 later route high-cost 与 anchor high-cost region 距离约 0.303m，仍处于近邻局部区域。
- 这进一步削弱了 “seq=8 failure 只因没有选择 explored candidate” 的解释。

## Decision

Analyzer decision：

```json
{
  "spatial_interpretation": "shared_choke_or_ambiguous",
  "phase27_candidate_signal": "not_supported",
  "next_recommendation": "analysis_only_continue_spatial_local_cost_diagnostics"
}
```

Guardrails：

- `do_not_enter_phase27`
- `do_not_change_branch_selection`
- `do_not_tune_nav2_or_controller_params_from_phase26k`
- `do_not_infer_clean_explored_corridor_without_runtime_cost_cells`

## Phase27 gate 判定

Phase26K 仍不支持 Phase27：

1. Anchor seq=8 仍是 high-cost / timeout / route-divergence case，不是 clean-low-cost branch-history suppression。
2. Near-zero path-ahead high cost 当前只能定位为 shared/ambiguous near-dispatch/choke area。
3. seq=10 success 与 anchor seq=8 high-cost samples 极近 overlap，说明该 high-cost area 可出现在 success context。
4. baseline_run1 seq=7 success 也有 chosen-route high-cost samples，且与 anchor high-cost region 相邻。
5. 当前 artifacts 不包含 path-ahead high-cost cell 坐标，不能证明 explored candidate corridor clean。

因此：

- 不进入 Phase27。
- 不改 branch selection。
- 不调 Nav2/controller params。

## 下阶段建议

推荐进入 Phase26L，仍 analysis-only。

Phase26L 推荐目标：补 runtime spatial diagnostics 设计/contract，而不是改行为：

1. 扩展 recorder/analyzer 设计，使 snapshot 记录 path-ahead high-cost cell coordinates：
   - high-cost cell count
   - nearest high-cost point to robot
   - high-cost centroid
   - high-cost points projected on chosen path vs candidate corridors
2. 对 chosen route 与 explored candidate corridor 分别采样 local-cost corridor：
   - target corridor max/mean cost
   - first high-cost distance from dispatch
   - overlap with robot near-zero heading ray
3. 先用 synthetic tests 固化 contract。
4. 只有后续真实 artifacts 证明：
   - chosen-away route high-cost；
   - explored candidate corridor clean；
   - 且 pattern 在 failed-only matched repeats 中稳定；
   才重新讨论 Phase27。

## Verification

已完成：

```bash
python3 -m py_compile tools/analyze_phase26k_spatial_local_cost_alignment.py
python3 -m pytest src/tugbot_maze/test/test_phase26k_spatial_local_cost_alignment.py src/tugbot_maze/test/test_phase26j_forward_shift_artifact_join.py src/tugbot_maze/test/test_phase26i_forward_shift_stats.py -q
# 6 passed

python3 -m pytest src/tugbot_maze/test/test_phase26*.py -q
# 35 passed
```

Process cleanup check：

```bash
ps -eo pid,cmd | grep -E 'ros2 launch tugbot_bringup|record_explorer_state_series|record_controller_dynamics|record_post_recovery_snapshots|ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|maze_explorer|slam_toolbox|controller_server|planner_server|bt_navigator|gz sim|ruby .*gz sim' | grep -v grep || true
```

结果：无残留匹配进程。
