# Phase62 First Dispatch Local Cost / Traversability Diagnostics Report

Run id: `phase62_first_dispatch_local_cost_traversability_diagnostics`
Artifact dir: `log/phase62_first_dispatch_local_cost_traversability_diagnostics/`
Status: complete; stop for human acceptance. Do not enter Phase63.

## Context and preserved conclusions

Phase61 人工验收已通过，结论保持：`SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED`。
Phase61 恢复 first dispatch，但这不是自主探索成功（not autonomous exploration success），也不是出口成功（not exit success）。

Phase62 只诊断 first dispatch target 为什么 map cell 为 free/clear，但 local costmap 显示 high-cost / lethal-radius，例如 `state=lethal_or_obstacle`、`value=49`、`max_radius_cost=99`，并判断是否可能导致 Nav2 timeout、卡墙或无有效前进。

同时保留历史结论：

- Phase57: `TIMEOUT_INCONCLUSIVE_DATA_GAP`
- Phase58/59: `CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE`
- Phase59 runtime: `TOPOLOGY_CONSISTENCY_CONFIRMED_NO_CANDIDATE`
- Phase60: `SINGLE_OPEN_DIRECTION_MISCLASSIFIED_AS_DEAD_END`
- Phase61: `SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED`

## Guardrails

全部保持：

- 不调 Nav2/MPPI/controller 参数。
- 不调 `clearance_radius_m`。
- 不调 map sufficiency threshold。
- 不改 branch selection scoring。
- 不继续入口策略、fallback、terminal acceptance。
- 不宣称 autonomous exploration success / exit success。
- bounded runtime diagnostics only。
- `max_goals=1`。
- cleanup 已执行且为空。

Nav2 config diff: empty / 0 bytes.

## Implemented diagnostics

新增 read-only diagnostics：

- dispatch target 周边 local costmap patch。
- target cell state。
- target footprint cost。
- front wedge clearance/cost。
- robot-to-target progress。
- external `cmd_vel` / `cmd_vel_smoothed` recorder。
- Nav2 feedback/result summary。
- planner/controller log correlation。
- map/costmap/scan timestamp consistency from maze_explorer dispatch context。

新增/更新文件：

- `src/tugbot_maze/tugbot_maze/maze_explorer.py`
- `tools/run_phase62_first_dispatch_local_cost_traversability_diagnostics.sh`
- `tools/analyze_phase62_first_dispatch_local_cost_traversability_diagnostics.py`
- `src/tugbot_maze/test/test_phase62_first_dispatch_local_cost_traversability_diagnostics.py`
- `doc/doc_report/phase62_first_dispatch_local_cost_traversability_diagnostics_report.md`

## Runtime setup

- World: `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf`
- Metadata: `src/tugbot_maze/config/maze_20260528_scaled_instance.yaml`
- Replays: 1 bounded replay
- Ingress-then-maze_explorer chain
- `max_goals=1`
- `near_exit_fallback_enabled=false`
- `post_ingress_single_open_exception_enabled=true`
- `topology_consistency_enabled=true`

First attempt was inconclusive because ingress was sent immediately after readiness and Nav2 aborted ingress with `Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose`. Wrapper was kept bounded and adjusted only by adding post-readiness settle time. The second bounded replay produced runtime first-dispatch diagnostics.

## Key runtime evidence

Analyzer classification:

`CORRIDOR_TOO_NARROW`

First dispatch observed: true.
Runtime evidence required: true.
Guardrail violation: false.
Autonomous success claimed: false.
First dispatch is exit success: false.

First dispatch target:

```text
target = [0.558242117171631, 0.5108474753216039]
dispatch_pose = [0.8740772410891823, 0.020696297436460016, 0.03198252951540831]
target_cell_occupancy = 0
target_clearance_m = 0.4031128934217776
path_corridor_min_clearance_m = 0.4031128934217776
target_crosses_narrow_passage = true
selected_due_to_context = topology_exit_bias_score
candidate_branch_count = 2
```

Local cost at dispatch:

```text
dispatch_target_local_cost = 49
dispatch_target_local_cost_max_radius = 99
dispatch_path_local_cost_max = 49
dispatch_path_local_cost_mean = 12.846153846153847
```

Phase62 target cell state:

```text
available = true
cell = [24, 39]
in_bounds = true
state = lethal_or_obstacle
value = 49
max_radius_cost = 99
```

Local costmap patch around target:

```text
radius_cells = 3
sample_count = 49
max = 89
mean = 45.36734693877551
high_cost_count = 6
lethal_count = 0
```

Target footprint cost:

```text
sample_count = 120
max = 99
mean = 36.55833333333333
p95 = 95.0
high_cost_count = 21
lethal_count = 8
```

Front wedge at dispatch:

```text
front_wedge_clearance_m = 0.75
sample_count = 135
max = 63
mean = 11.37037037037037
high_cost_count = 0
lethal_count = 0
```

External runtime recorder timeline later saw front wedge cost rise near the target:

```text
latest front_wedge max = 100
latest front_wedge high_cost_count = 131
latest front_wedge lethal_count = 103
```

Robot-to-target progress:

```text
pose_sample_count = 504
distance_at_dispatch_m ~= 0.5831
final_distance_m ~= 0.2421
distance_improvement_m ~= 0.3410
total_motion_m ~= 1.1299
robot_stuck = false
```

cmd_vel:

```text
cmd_vel_sample_count = 685
nonzero_command_count = 685
controller_no_cmd_vel = false
linear_x_abs max = 0.1600
linear_x_abs mean = 0.0665
angular_z_abs max = 0.1948
angular_z_abs mean = 0.0865
near_zero_command_ratio = 0.0
```

Nav2 feedback/result:

```text
feedback_sample_count = 1714
number_of_recoveries_max = 0
result_status = 4
result_reason = succeeded
```

Note: the runtime recorder's raw feedback stream includes duplicate/initial zero-distance samples, so the embedded maze_explorer feedback summary is kept as supporting context rather than the sole progress metric. Odom-based progress is the primary progress evidence.

Timestamp consistency at dispatch from maze_explorer:

```text
all_available = true
local_costmap_age_sec = 0.375
map_age_sec = 0.9
scan_age_sec = 0.099
max_age_sec = 0.9
```

Planner/controller log correlation:

```text
Failed to make progress = 0
Controller patience exceeded = 0
Goal failed = 0
No valid = 0
costmap mentions = 76
collision mentions = 27
```

The global costmap launch warning about inflation radius vs footprint exists in logs, but Phase62 did not change Nav2 parameters.

## Interpretation

The first dispatch target is globally/map-clear (`target_cell_occupancy=0`) and the robot does not immediately lack commands or become stuck. It progresses substantially toward the target and Nav2 reports success for this max_goals=1 bounded run.

However, the target is in a narrow corridor region:

- `path_corridor_min_clearance_m ~= 0.403 m`.
- `target_crosses_narrow_passage = true`.
- local target radius includes lethal/high-cost cells (`max_radius_cost=99`).
- footprint sampling at the target overlaps high/lethal local cost cells (`max=99`, `lethal_count=8`, `high_cost_count=21`).

So the map free/clear vs local lethal-radius mismatch is best explained by footprint/corridor geometry and local cost inflation/obstacle proximity around a narrow passage, not by immediate no-cmd_vel, no-global-plan, or stale costmap evidence in this run.

Because the selected classification list asks for one conclusion, Phase62 chooses:

`CORRIDOR_TOO_NARROW`

with supporting secondary factor:

`LOCAL_COSTMAP_INFLATION_DOMINANT`

The evidence does not support `SENSOR_COSTMAP_MISALIGNMENT` as the primary classification because dispatch-time map/local_costmap/scan sample ages were available and bounded (`max_age_sec=0.9`).

## Validation

- `python3 -m py_compile src/tugbot_maze/tugbot_maze/maze_explorer.py tools/analyze_phase62_first_dispatch_local_cost_traversability_diagnostics.py`: PASS
- `bash -n tools/run_phase62_first_dispatch_local_cost_traversability_diagnostics.sh`: PASS
- `pytest -q src/tugbot_maze/test/test_phase62_first_dispatch_local_cost_traversability_diagnostics.py`: `5 passed in 0.01s`
- `colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install`: PASS (`2 packages finished`)
- `git diff -- src/tugbot_navigation/config | wc -c`: `0`
- cleanup check: empty
- analyzer rerun: PASS

## Stop condition

Phase62 complete. Stop for human acceptance. Do not enter Phase63.
