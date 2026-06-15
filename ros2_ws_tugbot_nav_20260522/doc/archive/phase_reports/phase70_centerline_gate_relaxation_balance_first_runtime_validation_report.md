# Phase70 Centerline Gate Relaxation / Balance-First Runtime Validation

run_id: `phase70_centerline_gate_relaxation_balance_first_runtime_validation`
artifact_dir: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/log/phase70_centerline_gate_relaxation_balance_first_runtime_validation`

## Upstream acceptance context

Phase69 已人工验收，结论保持：`CENTERLINE_GATE_NO_APPLY`。

Phase70 不覆盖 Phase69 结论，不把 first dispatch 写成出口成功，不宣称 autonomous exploration success，不宣称 exit success。

## Objective

将 runtime centerline target refinement 从 Phase69 的 all-metrics gate 放宽为 balance-first gate：

- 前置条件仍必须成立：corridor / two-side-wall / same-corridor / occupancy free。
- 安全底线不放宽：footprint lethal 不增加、front-wedge lethal 不增加、min_clearance 不低于 safety floor、forward progress 不明显下降。
- 优化目标优先降低 `balance_error_m`，再比较 `min_clearance_m`、local-cost、front-wedge/footprint risk。
- 保留 `original_target`、`refined_target`、实际 dispatch target 与 gate reason 双记录。

## Guardrails

- bounded runtime only。
- max_goals=1。
- 不调 Nav2/MPPI/controller 参数。
- 不调 inflation / robot_radius / clearance_radius_m / map threshold。
- 不改 branch scoring。
- 不接 cmd_vel corridor-following。
- 不改 fallback/terminal acceptance。
- 不使用旧 scaffold world/map。
- no autonomous exploration success claim。
- no exit success claim。
- 不进入 Phase71，等待人工验收。

## Implementation summary

### `src/tugbot_maze/tugbot_maze/maze_perception.py`

新增 `refine_corridor_centerline_target(..., gate_mode='balance_first', min_clearance_floor_m=..., forward_progress_tolerance_m=...)`。

Balance-first candidate 必须满足：

- `same_corridor == true`
- `two_side_wall_evidence == true`
- `target_has_clearance == true`
- `occupancy_free == true`
- `balance_error_improved == true`
- `safety_floor_ok == true`
- `footprint_lethal_not_increased == true`
- `front_wedge_lethal_not_increased == true`
- `forward_progress_not_obviously_lowered == true`

保留 strict/all-metrics eligible count，并新增 `balance_first_eligible_candidate_count`。

### `src/tugbot_maze/tugbot_maze/maze_explorer.py`

新增参数：

- `centerline_target_refinement_gate_mode，Phase70 wrapper 使用 `balance_first``。
- `centerline_target_refinement_min_clearance_floor_m`。
- `centerline_target_refinement_forward_progress_tolerance_m`。

新增 diagnostics 字段：

- `centerline_refinement_gate_mode`
- `centerline_refinement_strict_eligible_candidate_count`
- `centerline_refinement_balance_first_eligible_candidate_count`
- `original_target`
- `refined_target`
- `centerline_refinement_reason`
- `centerline_refinement_gate_conditions`

### Runtime wrapper/analyzer

新增：

- `tools/run_phase70_centerline_gate_relaxation_balance_first_runtime_validation.sh`
- `tools/analyze_phase70_centerline_gate_relaxation_balance_first_runtime_validation.py`
- `src/tugbot_maze/test/test_phase70_centerline_gate_relaxation_balance_first_runtime_validation.py`

Runtime path 使用 Phase65 inner ingress waypoint: `(2.0, 0.0, 0.0)`，`replay_count=2`，`max_goals=1`。

## Runtime evidence

- `classification`: `BALANCE_FIRST_GATE_NO_APPLY`
- `replay_count`: `2`
- `max_goals`: `1`
- `balance_first_applied_count`: `0`
- `balance_first_gate_no_apply_count`: `2`
- `guardrail_violation`: `False`
- `nav2_config_diff_empty`: `True`
- `cleanup_empty`: `True`
- `branch_scoring_changed`: `False`
- `complete_autonomous_success_claimed`: `False`
- `exit_success_claimed`: `False`

### Per replay

#### replay_01

- `inner_ingress_goal_success`: `True`
- `dispatch_count`: `1`
- `outcome_count`: `1`
- `timeout_count`: `0`
- `balance_first_applied_count`: `0`
- `balance_first_gate_no_apply_count`: `1`
- `final_mode`: `FAILED_EXHAUSTED`
- `final_goal_count`: `1`

Goal sequence `1`:
- dispatch observed: `True`
- outcome_event: `success`; timeout: `False`; result_reason: `succeeded`
- target: `[2.6030466985915393, 0.19501714467838877]`
- original_target: `[2.6030466985915393, 0.19501714467838877]`
- refined_target: `[2.6030466985915393, 0.19501714467838877]`
- balance-first applied: `False`; reason: `lethal_cost_regression`; gate_mode: `balance_first`
- candidate counts: strict=`2`, balance_first=`0`, selected=`0`
- balance_error_m original/selected: `0.4000000059604645` / `None`
- min_clearance_m original/selected: `0.3500000052154064` / `None`
- footprint_lethal original/selected: `70` / `None`
- front_wedge_lethal original/selected: `242` / `None`
- robot distance improvement: `0.43776851579621895`; final distance to target: `0.33237757483379676`

#### replay_02

- `inner_ingress_goal_success`: `True`
- `dispatch_count`: `1`
- `outcome_count`: `2`
- `timeout_count`: `2`
- `balance_first_applied_count`: `0`
- `balance_first_gate_no_apply_count`: `1`
- `final_mode`: `FAILED_EXHAUSTED`
- `final_goal_count`: `1`

Goal sequence `1`:
- dispatch observed: `True`
- outcome_event: `timeout`; timeout: `True`; result_reason: `goal_timeout`
- target: `[2.039827038473372, 1.0242372694944333]`
- original_target: `[2.039827038473372, 1.0242372694944333]`
- refined_target: `[2.039827038473372, 1.0242372694944333]`
- balance-first applied: `False`; reason: `lethal_cost_regression`; gate_mode: `balance_first`
- candidate counts: strict=`0`, balance_first=`0`, selected=`0`
- balance_error_m original/selected: `0.05000000074505806` / `None`
- min_clearance_m original/selected: `0.5500000081956387` / `None`
- footprint_lethal original/selected: `0` / `None`
- front_wedge_lethal original/selected: `0` / `None`
- robot distance improvement: `0.6441808749827473`; final distance to target: `0.3710161506791167`

## Focused tests and validation

- `python3 -m py_compile src/tugbot_maze/tugbot_maze/maze_perception.py src/tugbot_maze/tugbot_maze/maze_explorer.py tools/analyze_phase70_centerline_gate_relaxation_balance_first_runtime_validation.py`: PASS。
- `bash -n tools/run_phase70_centerline_gate_relaxation_balance_first_runtime_validation.sh`: PASS。
- `pytest -q src/tugbot_maze/test/test_phase70_centerline_gate_relaxation_balance_first_runtime_validation.py`: `6 passed in 0.39s`。
- `colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install`: `2 packages finished [1.07s]`。
- `git diff -- src/tugbot_navigation/config | wc -c`: `0`。
- cleanup process scan: empty。

## Acceptance classification

Final Phase70 classification: `BALANCE_FIRST_GATE_NO_APPLY`。

Interpretation: balance-first guard 已接入并执行，但两次 bounded replay 均未 apply centerline refinement；`balance_first_eligible_candidate_count=0`，reason 为 safety/lethal gate 阻断。
Replay 中出现的 dispatch 仅表示 first dispatch observed，不是出口成功，也不是 autonomous exploration success。

允许分类：

- `BALANCE_FIRST_CENTERLINE_APPLIED_AND_IMPROVES`
- `BALANCE_FIRST_APPLIED_TIMEOUT_REMAINS`
- `BALANCE_FIRST_GATE_NO_APPLY`
- `BALANCE_FIRST_REGRESSION`
- `INSUFFICIENT_EVIDENCE`
- `GUARDRAIL_VIOLATION_STRATEGY_CHANGED`

## Stop condition

Phase70 已完成，停止等待人工验收；不进入 Phase71。
