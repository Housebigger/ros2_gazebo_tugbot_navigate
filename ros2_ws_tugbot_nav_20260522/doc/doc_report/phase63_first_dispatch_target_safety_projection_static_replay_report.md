# Phase63 First Dispatch Target Safety Projection Design / Static Replay Report

Run id: `phase63_first_dispatch_target_safety_projection_static_replay`
Artifact dir: `log/phase63_first_dispatch_target_safety_projection_static_replay/`
Status: complete; stop for human acceptance. Do not enter Phase64. 不进入 Phase64.

## Context and preserved conclusions

Phase62 已人工验收通过，结论保持：`CORRIDOR_TOO_NARROW`，辅助因素 `LOCAL_COSTMAP_INFLATION_DOMINANT`。
Phase61 结论保持：`SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED`。

Phase63 只做 First Dispatch Target Safety Projection Design / Static Replay。它基于 Phase62 first dispatch 证据，设计一个很窄的 target safety projection 思路，但不接入 runtime dispatch、不发送 ROS goal、不进行长跑。

重要边界：first dispatch 仍然不是自主探索成功（not autonomous exploration success），也不是出口成功（not exit success）。

## Guardrails

全部保持：

- static/log replay only。
- no runtime dispatch integration。
- 不调 Nav2/MPPI/controller 参数。
- 不调 `clearance_radius_m`。
- 不调 map sufficiency threshold。
- 不改 branch selection scoring。
- 不改入口策略 / fallback / terminal acceptance。
- 不宣称 autonomous exploration success / exit success。
- 不做长跑；本阶段不启动 Gazebo/Nav2 runtime。

Nav2 config diff: empty / 0 bytes.
Cleanup check: empty.

## Implemented artifacts

- `tools/analyze_phase63_first_dispatch_target_safety_projection_static_replay.py`
- `src/tugbot_maze/test/test_phase63_first_dispatch_target_safety_projection_static_replay.py`
- `doc/doc_report/phase63_first_dispatch_target_safety_projection_static_replay_report.md`
- `log/phase63_first_dispatch_target_safety_projection_static_replay/phase63_first_dispatch_target_safety_projection_static_replay.json`
- `log/phase63_first_dispatch_target_safety_projection_static_replay/phase63_first_dispatch_target_safety_projection_static_replay.md`

No `maze_explorer.py` runtime dispatch integration was added in Phase63.

## Projection design under test

Future runtime eligibility would be intentionally narrow:

1. Candidate comes from post-ingress single-open exception context.
2. Original target is map-clear / occupancy clear.
3. Local target radius or footprint cost is high/lethal.
4. Projection is only along the same open direction / corridor centerline.
5. Projection is limited to a small nearby window around the original target.
6. Candidate must preserve same-corridor evidence and improve local safety metrics.

Phase63 does not implement this runtime behavior. It only static-replays the design against the Phase62 artifact.

Static replay projection distances:

```text
[-0.30, -0.20, -0.10, 0.00, +0.10, +0.20, +0.30] m
```

Safety thresholds used for static design screening:

```text
radius_max_cost < 70
footprint_max_cost < 90
footprint_lethal_count <= 0
front_wedge_max_cost < 90, with replay coverage required
same_corridor = true
occupancy = 0
improves_over_original = true
```

## Source Phase62 first dispatch evidence

```text
target = [0.558242117171631, 0.5108474753216039]
occupancy = 0
path_corridor_min_clearance_m = 0.4031128934217776
target_crosses_narrow_passage = True
local_radius_max = 99
local_cell_state = lethal_or_obstacle
footprint_max = 99
footprint_lethal_count = 8
footprint_high_cost_count = 21
front_wedge_max = 63
```

Dispatch source fields from Phase62 artifact:

```text
selected_due_to_context = topology_exit_bias_score
local_topology = corridor
branch_angle = 1.602778856310305
single_open_exception_applied = False
```

Note: Phase63 target projection design is scoped for future post-ingress single-open exception candidates. The accepted Phase62 runtime artifact's actual first dispatch source fields show `single_open_exception_applied=false` and `selected_due_to_context=topology_exit_bias_score`; therefore Phase63 treats this as static design evidence only, not a runtime eligibility claim.

## Projection candidate comparison

| projection_m | target | radius_max | footprint_max | footprint_lethal | front_wedge_max | same_corridor | improves | safe | rejection_reason |
|---:|---|---:|---:|---:|---:|---|---|---|---|
| -0.3 | `[0.567835, 0.211001]` | 99 | 99 | 4 | 89 | True | True | False | `radius_cost_not_safe,footprint_cost_not_safe` |
| -0.2 | `[0.564638, 0.31095]` | 99 | 99 | 4 | 89 | True | True | False | `radius_cost_not_safe,footprint_cost_not_safe` |
| -0.1 | `[0.56144, 0.410899]` | 99 | 99 | 4 | 89 | True | False | False | `not_safer_than_original,radius_cost_not_safe,footprint_cost_not_safe` |
| 0.0 | `[0.558242, 0.510847]` | 99 | 99 | 4 | 77 | True | False | False | `not_safer_than_original,radius_cost_not_safe,footprint_cost_not_safe` |
| 0.1 | `[0.555044, 0.610796]` | 89 | 99 | 4 | 70 | True | True | False | `radius_cost_not_safe,footprint_cost_not_safe` |
| 0.2 | `[0.551847, 0.710745]` | 89 | 89 | 0 | None | True | False | False | `not_safer_than_original,radius_cost_not_safe,front_wedge_not_safe` |
| 0.3 | `[0.548649, 0.810694]` | 89 | 89 | 0 | None | True | False | False | `not_safer_than_original,radius_cost_not_safe,front_wedge_not_safe` |

## Classification

`NO_SAFE_PROJECTION_IN_CORRIDOR`

Reasoning:

- Replay evidence was available and all generated candidates remained in the same narrow projection corridor.
- Some candidates improved the composite safety score versus the original target.
- However no candidate satisfied all safety constraints simultaneously.
- The best scored candidate was:

```text
projection_distance_m = -0.3
target = [0.5678352403919105, 0.21100089457258064]
same_corridor = True
improves_over_original = True
safe_projection = False
rejection_reason = radius_cost_not_safe,footprint_cost_not_safe
radius_max = 99
footprint_max = 99
footprint_lethal_count = 4
front_wedge_max = 89
```

So Phase63 classification is `NO_SAFE_PROJECTION_IN_CORRIDOR`, not `SAFE_PROJECTION_FOUND`.

## Validation

- `python3 -m py_compile tools/analyze_phase63_first_dispatch_target_safety_projection_static_replay.py`: PASS
- `pytest -q src/tugbot_maze/test/test_phase63_first_dispatch_target_safety_projection_static_replay.py`: `5 passed in 0.01s`
- `colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install`: PASS (`2 packages finished`)
- `git diff -- src/tugbot_navigation/config | wc -c`: `0`
- cleanup check: empty

## Stop condition

Phase63 complete. Stop for human acceptance. Do not enter Phase64. 不进入 Phase64.
