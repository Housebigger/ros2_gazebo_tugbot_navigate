# Phase 26A Profile-Path / Params Equivalence Audit

生成时间：2026-05-25

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

Phase 26A 按 Phase 20-25 总结的推荐方案执行，目标不是继续调参，而是先解决 Phase 25H 暴露的问题：

```text
Phase25E profile runs reached EXIT_REACHED twice.
Candidate baseline runs using candidate 2.75 profile did not reach exit twice.
Both profiles nominally use CostCritic cost_weight=2.75 and should be behaviorally equivalent.
```

因此 Phase 26A 只做 profile-path / params equivalence audit 与 artifact hardening：

1. 用 TDD 增加 semantic params equivalence test。
2. 增加 params fingerprint helper。
3. 修改 smoke wrapper，在每次 run 前输出 params fingerprint artifact。
4. 修改 smoke wrapper，尝试保存 `/controller_server` runtime params dump。
5. 做 targeted tests、full package tests、build/test 验证和 orphan process cleanup check。

## 2. TDD RED

新增测试：

```text
src/tugbot_maze/test/test_phase26a_params_equivalence.py
```

首次运行：

```text
python3 -m pytest -q src/tugbot_maze/test/test_phase26a_params_equivalence.py -v
```

RED 结果：

```text
collected 5 items
2 passed, 3 failed
```

预期失败点：

```text
1. tools/fingerprint_nav2_params.py 不存在。
2. smoke wrapper 尚未声明 PARAMS_FINGERPRINT artifact。
3. smoke wrapper 尚未声明 RUNTIME_PARAMS_DIR / runtime param dump。
```

这确认测试确实覆盖 Phase 26A 需要的新行为，而不是测试已有行为。

## 3. 实现内容

### 3.1 新增 params fingerprint helper

新增：

```text
tools/fingerprint_nav2_params.py
```

功能：

- 读取 Nav2 params YAML。
- 计算 params file SHA256。
- 提取关键 semantic sections：
  - `FollowPath.CostCritic`
  - `FollowPath.CostCritic.cost_weight`
  - local costmap inflation layer
  - global costmap inflation layer
  - progress checker
  - goal checker
- 支持与 baseline params 比较 cost_weight delta。
- 支持与另一个 params file 做 semantic equivalence check。
- 支持写入 `--output-json` 或 stdout。

示例输出字段：

```json
{
  "run_id": "...",
  "selected_profile": "...",
  "params_file": {
    "path": "...",
    "sha256": "...",
    "costcritic_cost_weight": 2.75,
    "costcritic": {},
    "local_inflation": {},
    "global_inflation": {},
    "progress_checker": {},
    "goal_checker": null
  },
  "baseline": {
    "costcritic_cost_weight": 3.81
  },
  "baseline_delta": {
    "costcritic_cost_weight": -1.06
  },
  "compare": {
    "equivalent": true,
    "semantic_differences": []
  }
}
```

### 3.2 Smoke wrapper artifact hardening

修改：

```text
tools/run_phase21_controller_diagnostics_smoke.sh
```

新增 run id：

```text
phase26a_fingerprint_smoke
```

新增 artifacts：

```text
log/<run_id>_params_fingerprint.json
log/<run_id>_runtime_params/controller_server.yaml
```

Wrapper 现在会根据 run id 选择 profile：

```text
canonical_baseline
phase25a_local_cost_relief
phase25b_costcritic_relief
phase25d_costcritic_mid
phase25e_costcritic_compromise
candidate_costcritic_275
```

并在 launch 前调用：

```text
python3 tools/fingerprint_nav2_params.py \
  --params-file "$NAV2_PARAMS_FILE" \
  --baseline-params-file src/tugbot_navigation/config/nav2_slam_params.yaml \
  --run-id "$RUN_ID" \
  --selected-profile "$SELECTED_PROFILE" \
  --output-json "$PARAMS_FINGERPRINT"
```

Runtime dump 逻辑：

```text
mkdir -p "$RUNTIME_PARAMS_DIR"
ros2 param dump /controller_server > "$RUNTIME_PARAMS_DIR/controller_server.yaml"
```

注：本次 `phase26a_fingerprint_smoke` 成功生成 params fingerprint，但 runtime dump directory 未留下文件。原因很可能是 dump helper 在 controller_server 可发现但参数服务尚未稳定时尝试失败后退出/被 cleanup，不影响 params fingerprint 主目标；后续 Phase26A.1 可把 runtime dump helper 改为“node exists but dump fails也继续 retry”，并增加 artifact-level test。

## 4. Semantic equivalence audit 结果

手动执行：

```text
python3 tools/fingerprint_nav2_params.py \
  --params-file src/tugbot_navigation/config/nav2_slam_phase25e_costcritic_compromise_params.yaml \
  --compare-params-file src/tugbot_navigation/config/nav2_slam_candidate_costcritic_275_params.yaml \
  --baseline-params-file src/tugbot_navigation/config/nav2_slam_params.yaml \
  --run-id phase26a_manual_check \
  --selected-profile phase25e_costcritic_compromise
```

关键结果：

```text
phase25e params cost_weight: 2.75
candidate 275 params semantic equivalent: true
semantic_differences: []
baseline cost_weight: 3.81
baseline_delta cost_weight: -1.06
```

SHA256：

```text
canonical nav2_slam_params.yaml:
  e8c2e68690fd359ddad20237523cea8c68f725f8319de0d33381ee4010106d93

phase25e_costcritic_compromise params:
  0fb31750c917d0f74802a7fec0fe0d52b31c3730afa078c05a164780dd524109

candidate_costcritic_275 params:
  074b3a5880b8b1452334858d12421646ab440e17dbee6c91d55ad1025716d1fc
```

解释：

- YAML semantic content 等价。
- SHA 不同是允许的，说明 comments/order/text 可能不同；semantic diff 为 0。
- Phase25E 与 candidate profile 的 params 文件本身不是差异来源。

## 5. Phase26A fingerprint smoke

执行：

```text
bash tools/run_phase21_controller_diagnostics_smoke.sh phase26a_fingerprint_smoke
```

Wrapper exit code：`0`

生成 artifacts：

```text
log/phase26a_fingerprint_smoke_params_fingerprint.json
log/phase26a_fingerprint_smoke_launch.log
log/phase26a_fingerprint_smoke_goal_events.jsonl
log/phase26a_fingerprint_smoke_explorer_state.jsonl
log/phase26a_fingerprint_smoke_controller_dynamics.jsonl
log/phase26a_fingerprint_smoke_post_recovery_snapshots.jsonl
log/phase26a_fingerprint_smoke_goal_nav2_analysis.json
log/phase26a_fingerprint_smoke_geometry_nav2_summary.json
log/phase26a_fingerprint_smoke_goal_event_cost_summary.json
log/phase26a_fingerprint_smoke_goal_controller_dynamics.json
log/phase26a_fingerprint_smoke_post_recovery_snapshots_summary.json
```

Fingerprint artifact verified：

```text
run_id: phase26a_fingerprint_smoke
selected_profile: canonical_baseline
params_file: nav2_slam_params.yaml
params_file cost_weight: 3.81
baseline cost_weight: 3.81
baseline_delta cost_weight: 0.0
```

Run summary：

```text
final_mode: EXIT_REACHED
exit_distance_m: 0.5825
goal_count: 12
goal_success_count: 7
goal_failure_count: 4
timeout_cancel_count: 4
blocked_branch_count: 0
blacklisted_goal_count: 0
```

Nav2/controller summary：

```text
Nav2 timeout_count: 4
timeout_with_progress_failure_count: 4
timeout_with_recovery_count: 4
timeout_with_controller_abort_count: 4
healthy_motion_but_late_stall_count: 3
healthy_motion_but_timed_out_count: 1
late_controller_silent_count: 3
terminal_cancel_after_exit_count: 1
```

本 smoke 的目的只是验证 artifact chain，不用于调参结论。

## 6. Verification

### 6.1 Targeted Phase26A tests

```text
python3 -m pytest -q src/tugbot_maze/test/test_phase26a_params_equivalence.py -v
```

结果：

```text
5 passed in 0.22s
```

### 6.2 Tool/static checks

```text
python3 -m py_compile tools/fingerprint_nav2_params.py
bash -n tools/run_phase21_controller_diagnostics_smoke.sh
```

结果：均通过。

### 6.3 Package pytest

```text
python3 -m pytest -q src/tugbot_maze/test src/tugbot_bringup/test
```

结果：

```text
129 passed in 1.34s
```

### 6.4 Colcon build/test

```text
colcon build --symlink-install --packages-select tugbot_maze tugbot_bringup tugbot_navigation
colcon test --packages-select tugbot_maze tugbot_bringup tugbot_navigation --event-handlers console_direct+
colcon test-result --verbose
```

结果：

```text
Build: 3 packages finished
Colcon test: tugbot_maze 105 passed
colcon test-result: 105 tests, 0 errors, 0 failures, 0 skipped
```

### 6.5 Orphan process check

检查命令覆盖：

```text
ros2 launch tugbot_bringup tugbot_maze_explore
record_explorer_state_series
record_controller_dynamics
record_post_recovery_snapshots
ros_gz_bridge / parameter_bridge
maze_goal_monitor / maze_explorer
slam_toolbox
controller_server / planner_server / bt_navigator
gz sim
```

结果：

```text
no matching ROS/Gazebo/Nav2 processes remain
```

## 7. 当前结论

Phase 26A 完成了第一轮 profile-path / params equivalence audit：

```text
phase25e_costcritic_compromise params 与 candidate_costcritic_275 params 在 YAML semantic content 上等价。
两者 CostCritic cost_weight 均为 2.75。
canonical baseline 仍为 cost_weight 3.81。
```

因此 Phase25E 与 candidate baseline 行为差异，不太可能来自两个 source params 文件本身。

剩余可能原因：

```text
1. runtime params dump 尚未稳定落盘，需要继续 harden 后确认 controller_server 实际加载值。
2. install overlay / launch selection / stale installed share 仍需 runtime artifact 证明。
3. 如果 runtime dump 也证明等价，则 Phase25E vs candidate difference 应按 run-to-run variance 处理。
```

## 8. 下一步建议

建议继续 Phase 26A.1，而不是进入新参数调优：

1. Harden runtime param dump helper

当前 helper 一旦看到 `/controller_server` 但 dump 失败，可能提前结束。建议改为：

```text
for up to N seconds:
  if /controller_server exists:
    try ros2 param dump /controller_server > controller_server.yaml.tmp
    if tmp non-empty and contains controller_server:
      mv tmp controller_server.yaml
      success
    else retry
```

2. 增加 artifact-level contract test

新增测试应验证 wrapper 中有 retry-on-dump-failure 逻辑，并且 output file 名称固定：

```text
log/<run_id>_runtime_params/controller_server.yaml
```

3. 跑一次 candidate fingerprint smoke

建议新增/使用 run id：

```text
candidate_baseline_run3 或 phase26a_candidate_fingerprint_smoke
```

目标不是评价导航表现，而是确认：

```text
params_fingerprint selected_profile == candidate_costcritic_275
params_fingerprint cost_weight == 2.75
runtime controller_server dump 中 FollowPath.CostCritic.cost_weight == 2.75
launch log / params artifact 能证明选中了 candidate params
```

4. 若 runtime dump 也证明等价，再进入 Phase 26B repeat-run variance characterization。

## 9. 不建议事项

```text
不要 promote cost_weight=2.75 到 canonical baseline。
不要继续盲测 2.65 / 2.6。
不要启用 Phase25A local inflation relief。
不要引入 branch scoring。
不要把 phase26a_fingerprint_smoke 的 EXIT_REACHED 当作新 baseline 证据；它只是 artifact-chain smoke。
```

## 10. 一句话结论

Phase 26A 已证明 Phase25E profile 与 candidate 2.75 profile 的 source YAML 语义等价，并把每次 smoke 的 params fingerprint artifact 接入 wrapper；canonical baseline 仍保持 3.81。下一步应先 harden runtime `/controller_server` param dump 并用 candidate fingerprint smoke 证明运行时实际加载值，再决定 Phase25E/candidate 差异是否属于 run-to-run variance。
