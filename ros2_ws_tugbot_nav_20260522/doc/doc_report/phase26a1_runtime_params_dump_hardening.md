# Phase 26A.1 Runtime Param Dump Hardening + Candidate Fingerprint Smoke

生成时间：2026-05-25

工作空间：`/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## 1. 目标

本阶段按 Phase 26A 报告建议执行，不进入新参数调优。

目标：

1. harden runtime param dump helper。
   - 解决 Phase 26A 中 runtime dump 目录没有留下 `controller_server.yaml` 的问题。
   - 不再“看到 `/controller_server` 后 dump 失败就提前结束”。
   - 必须持续 retry，直到文件非空且能解析到 `FollowPath.CostCritic.cost_weight`。

2. 增加 artifact-level contract test。
   - 固定验证 wrapper 会产出：
     - `log/<run_id>_runtime_params/controller_server.yaml`
     - `log/<run_id>_runtime_params/controller_server_summary.json`

3. 再跑 candidate fingerprint smoke。
   - 目标不是评价导航表现。
   - 目标是证明运行时实际加载：
     - `selected_profile == candidate_costcritic_275`
     - fingerprint cost_weight == `2.75`
     - runtime `/controller_server` dump 中 CostCritic cost_weight == `2.75`

## 2. TDD RED

新增测试：

```text
src/tugbot_maze/test/test_phase26a1_runtime_params_dump.py
```

首次运行：

```text
python3 -m pytest -q src/tugbot_maze/test/test_phase26a1_runtime_params_dump.py -v
```

RED 结果：

```text
collected 4 items
1 passed, 3 failed
```

预期失败点：

```text
1. tools/dump_controller_runtime_params.py 不存在。
2. wrapper 尚未支持 phase26a_candidate_fingerprint_smoke。
3. wrapper 尚未调用 retrying runtime dump helper。
```

后续又补充了一个 RED：

```text
ros2 param dump /controller_server 实际输出顶层 key 是 `/controller_server`，不是 `controller_server`。
```

新增测试 `test_phase26a1_runtime_dump_extractor_accepts_ros2_slash_node_key` 先失败，再修复 extractor。

## 3. 实现内容

### 3.1 新增 retrying runtime param dump helper

新增：

```text
tools/dump_controller_runtime_params.py
```

功能：

- 支持两种模式：

```text
--extract-cost-weight <yaml>
```

用于测试/验证已存在 runtime dump。

```text
--node /controller_server \
--output log/<run_id>_runtime_params/controller_server.yaml \
--summary-json log/<run_id>_runtime_params/controller_server_summary.json \
--expected-cost-weight <expected> \
--timeout-sec 120 \
--interval-sec 1.0
```

用于 runtime retry dump。

- dump 成功判据：
  - `ros2 param dump` exit code 0
  - stdout 非空
  - YAML 可解析
  - 可提取 `FollowPath.CostCritic.cost_weight`
  - 若传入 expected，则实际值等于 expected

- 支持 ROS 2 dump 的两种 node key：

```text
controller_server
/controller_server
```

- 写入两个 artifact：

```text
controller_server.yaml
controller_server_summary.json
```

### 3.2 修改 smoke wrapper

修改：

```text
tools/run_phase21_controller_diagnostics_smoke.sh
```

新增 run id：

```text
phase26a_candidate_fingerprint_smoke
```

该 run id 会启用：

```text
CANDIDATE_COSTCRITIC_275_PROFILE=true
SELECTED_PROFILE="candidate_costcritic_275"
NAV2_PARAMS_FILE="src/tugbot_navigation/config/nav2_slam_candidate_costcritic_275_params.yaml"
RUNTIME_EXPECTED_COST_WEIGHT="2.75"
```

Wrapper runtime dump helper 调用：

```text
python3 tools/dump_controller_runtime_params.py \
  --node /controller_server \
  --output "$RUNTIME_PARAMS_DIR/controller_server.yaml" \
  --summary-json "$RUNTIME_PARAMS_DIR/controller_server_summary.json" \
  --expected-cost-weight "$RUNTIME_EXPECTED_COST_WEIGHT" \
  --timeout-sec 120 \
  --interval-sec 1.0
```

同时为历史 profiles 设置 expected cost weight：

```text
canonical_baseline: 3.81
phase25b_costcritic_relief: 2.5
phase25d_costcritic_mid: 3.0
phase25e_costcritic_compromise: 2.75
candidate_costcritic_275: 2.75
```

## 4. Verification

### 4.1 Targeted tests

```text
python3 -m pytest -q \
  src/tugbot_maze/test/test_phase26a_params_equivalence.py \
  src/tugbot_maze/test/test_phase26a1_runtime_params_dump.py
```

结果：

```text
10 passed in 0.35s
```

### 4.2 Package pytest

```text
python3 -m pytest -q src/tugbot_maze/test src/tugbot_bringup/test
```

结果：

```text
134 passed in 1.39s
```

### 4.3 Static checks

```text
python3 -m py_compile tools/dump_controller_runtime_params.py tools/fingerprint_nav2_params.py
bash -n tools/run_phase21_controller_diagnostics_smoke.sh
```

结果：通过。

### 4.4 Colcon build/test

```text
colcon build --symlink-install --packages-select tugbot_maze tugbot_bringup tugbot_navigation
colcon test --packages-select tugbot_maze tugbot_bringup tugbot_navigation --event-handlers console_direct+
colcon test-result --verbose
```

结果：

```text
Build: 3 packages finished
Colcon test: tugbot_maze 110 passed
colcon test-result: 110 tests, 0 errors, 0 failures, 0 skipped
```

## 5. Candidate fingerprint smoke

执行：

```text
bash tools/run_phase21_controller_diagnostics_smoke.sh phase26a_candidate_fingerprint_smoke
```

Wrapper exit code：`0`

### 5.1 Fingerprint artifact 验证

文件：

```text
log/phase26a_candidate_fingerprint_smoke_params_fingerprint.json
```

关键字段：

```text
run_id: phase26a_candidate_fingerprint_smoke
selected_profile: candidate_costcritic_275
params_file: src/tugbot_navigation/config/nav2_slam_candidate_costcritic_275_params.yaml
params_file cost_weight: 2.75
baseline cost_weight: 3.81
baseline_delta cost_weight: -1.06
sha256: 074b3a5880b8b1452334858d12421646ab440e17dbee6c91d55ad1025716d1fc
```

### 5.2 Runtime controller_server dump 验证

文件：

```text
log/phase26a_candidate_fingerprint_smoke_runtime_params/controller_server.yaml
log/phase26a_candidate_fingerprint_smoke_runtime_params/controller_server_summary.json
```

Summary：

```json
{
  "attempts": 4,
  "controller_server": {
    "FollowPath": {
      "CostCritic": {
        "cost_weight": 2.75
      }
    }
  },
  "expected_cost_weight": 2.75,
  "node": "/controller_server",
  "output": "log/phase26a_candidate_fingerprint_smoke_runtime_params/controller_server.yaml"
}
```

独立 extractor 验证：

```text
python3 tools/dump_controller_runtime_params.py \
  --extract-cost-weight log/phase26a_candidate_fingerprint_smoke_runtime_params/controller_server.yaml
```

输出：

```json
{
  "controller_server": {
    "FollowPath": {
      "CostCritic": {
        "cost_weight": 2.75
      }
    }
  }
}
```

结论：

```text
candidate fingerprint smoke 已证明运行时 /controller_server 实际加载 CostCritic cost_weight=2.75。
```

### 5.3 Run summary

注意：本 run 目标是 artifact/runtime params 证明，不用于 baseline promotion。

本次导航 summary：

```text
final_mode: FAILED_EXHAUSTED
goal_count: 12
goal_success_count: 7
goal_failure_count: 5
timeout_cancel_count: 5
blocked_branch_count: 0
blacklisted_goal_count: 0
exit_distance_m: 1.1447
```

Nav2/controller summary：

```text
Nav2 timeout_count: 5
timeout_with_progress_failure_count: 5
timeout_with_recovery_count: 5
timeout_with_controller_abort_count: 5
healthy_motion_but_late_stall_count: 5
late_controller_silent_count: 5
```

此结果与 Phase25G/H candidate-baseline 不达出口方向一致，但本阶段不做参数评价，只确认 profile/runtime load。

## 6. Orphan process cleanup check

检查范围：

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

Phase 26A.1 达成三个目标：

1. runtime param dump helper 已 harden。
   - 能 retry。
   - 能验证文件非空且包含 CostCritic cost_weight。
   - 支持 `/controller_server` 顶层 key。

2. artifact-level contract test 已加入并通过。
   - wrapper 固定输出 runtime params artifact。
   - candidate smoke id 固定。
   - expected runtime cost weight 固定按 profile 匹配。

3. candidate fingerprint smoke 已证明运行时实际加载值。
   - source params fingerprint：candidate 2.75。
   - runtime `/controller_server` dump：CostCritic cost_weight 2.75。

因此 Phase25E/candidate 差异目前可排除：

```text
source params YAML 不等价导致的差异：已排除。
wrapper 选错 candidate params file：已排除。
runtime controller_server 未加载 2.75：已排除。
```

剩余最合理解释：

```text
run-to-run variance / stochastic simulation-controller interaction / start timing variance。
```

## 8. 下一步建议

建议进入 Phase 26B：repeat-run variance characterization，而不是继续调参。

推荐矩阵：

```text
canonical baseline cost_weight 3.81: 3 runs
candidate cost_weight 2.75: 3 runs
```

每个 run 必须保留：

```text
*_params_fingerprint.json
*_runtime_params/controller_server.yaml
*_runtime_params/controller_server_summary.json
*_explorer_state.jsonl
*_goal_events.jsonl
*_goal_nav2_analysis.json
*_goal_controller_dynamics.json
*_timeout_subtypes.json
*_post_recovery_enriched.json
```

建议先补一个 aggregator，把 runtime params summary 合入 compare input，避免后续人工检查。

Phase 26B acceptance 应使用 repeat statistics，而不是 single-run accepted/rejected：

```text
EXIT_REACHED count / N
median exit_distance_m
P75 exit_distance_m
median/P75 timeout_cancel_count
median/P75 footprint_path_blocked_late_silent
side_cost_or_timing_late_silent count
unclassified_late_silent count
blocked/blacklist total
```

## 9. 不建议事项

```text
不要 promote cost_weight=2.75。
不要继续盲测 2.65 / 2.6。
不要启用 Phase25A local inflation relief。
不要引入 branch scoring。
不要把本次 candidate fingerprint smoke 的 FAILED_EXHAUSTED 作为单次 reject 依据；它只是 runtime-profile proof run。
```

## 10. 一句话结论

Phase 26A.1 已通过 runtime `/controller_server` dump 证明 candidate smoke 实际加载的是 `candidate_costcritic_275` 且 `FollowPath.CostCritic.cost_weight=2.75`；因此 Phase25E 与 candidate baseline 的差异不再像是 params file 或 runtime load 错误，更应按 run-to-run variance 处理。下一步应进入 Phase 26B repeat-run variance characterization，而不是继续调新参数。
