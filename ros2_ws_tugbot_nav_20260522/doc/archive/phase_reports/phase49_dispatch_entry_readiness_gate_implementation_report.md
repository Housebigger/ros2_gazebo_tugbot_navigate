# Phase49 maze_explorer Dispatch Entry Readiness Gate Implementation Report

## Final classification

`DISPATCH_ENTRY_READINESS_GATE_IMPLEMENTED`

Phase49 按 Phase48 design lock 执行：不直接调整 `clearance_radius_m`，不调 Nav2 / MPPI / controller 参数，不继续 fallback / terminal acceptance，不声明自主探索成功。本阶段只在 `maze_explorer` 首次 topology sampling / dispatch entry 前加入自动探索器入口 readiness gate。

## Why this phase exists

Phase43 的关键现象是首次 topology sampling 过早进入失败路径：

- `goal_count=0`
- `dispatch_events=0`
- final mode: `FAILED_EXHAUSTED`
- `entered_failed_exhausted_before_map_sufficient=true`
- 首次 topology sampling 四方向全部 rejected

Phase47 已证明 manual Gazebo / SLAM / Nav2 / TF / RViz Goal baseline 可用，结论为 `MANUAL_NAV2_GOAL_BASELINE_OK`。

因此 Phase49 先修采样时机 / dispatch-entry 入口保护，而不是直接调 clearance 策略，避免误伤已恢复的手动导航基线。

## Implementation summary

Modified `src/tugbot_maze/tugbot_maze/maze_explorer.py`:

1. 新增 state:

   - `WAIT_FOR_DISPATCH_ENTRY_READINESS`

2. 新增输入 / 依赖：

   - `/scan` via `sensor_msgs/msg/LaserScan`
   - `/local_costmap/costmap` 继续使用完整 `OccupancyGrid`
   - `/map` 继续使用完整 `OccupancyGrid`
   - TF `map -> base_link`
   - Nav2 lifecycle `GetState` service
   - `/navigate_to_pose` action server readiness
   - `/goal_pose` subscriber count

3. 新增 dispatch-entry gate 参数：

   - `scan_topic`
   - `goal_pose_topic`
   - `dispatch_readiness_required_lifecycle_nodes`
   - `dispatch_readiness_near_robot_radius_m`
   - `dispatch_readiness_min_map_known_ratio`
   - `dispatch_readiness_min_map_free_ratio`
   - `dispatch_readiness_min_local_costmap_known_ratio`
   - `dispatch_readiness_min_local_costmap_free_ratio`
   - `dispatch_readiness_min_scan_finite_count`
   - `dispatch_readiness_max_local_costmap_age_sec`

4. Gate 顺序：

   `_explore_once()` 现在在 `self._analyze_and_dispatch(robot_pose)` 之前执行：

   - Nav2 action server check
   - dispatch-entry readiness gate
   - only if gate passed: enter `AT_NODE_ANALYZE` and topology sampling

5. 若 gate 未通过：

   - mode 设置为 `WAIT_FOR_DISPATCH_ENTRY_READINESS`
   - publish structured `/maze/explorer_state`
   - 不调用 `_analyze_and_dispatch()`
   - 不调用 `_mark_exhausted()`
   - 不进入 `FAILED_EXHAUSTED`
   - 不 dispatch goal

6. State payload 新增 structured diagnostics：

   - `dispatch_readiness_gate`
   - `dispatch_readiness_gate_passed`
   - `dispatch_readiness_blocking_reasons`
   - `dispatch_readiness_first_pass_time_sec`

Gate diagnostics 包含：

- `nav2_lifecycle_active`
- `navigate_to_pose_action_ready`
- `goal_pose_subscriber_ready`
- `scan_sufficient`
- `map_sufficient`
- `tf_sufficient`
- `local_costmap_sufficient`
- per-node lifecycle states
- scan finite count / nearest obstacle
- map known/free/occupied/unknown ratios near robot
- local costmap known/free/occupied/unknown ratios near robot
- local costmap sample age
- robot pose in map

## Launch / package integration

Modified `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`:

- passes `scan_topic: /scan`
- passes `goal_pose_topic: /goal_pose`
- exposes dispatch readiness parameters as launch arguments
- preserves existing `clearance_radius_m` default `0.38`

Modified `src/tugbot_maze/package.xml`:

- added `sensor_msgs`
- added `lifecycle_msgs`

## Guardrails honored

- Did not tune `clearance_radius_m`.
- Did not modify Nav2 / MPPI / controller params.
- Did not edit `src/tugbot_navigation/config`.
- Did not use old scaffold world/map.
- Did not continue fallback / terminal acceptance.
- Did not claim autonomous exploration success.
- Did not run a long exploration.
- Did not rely on truncated `ros2 topic echo` map/costmap samples.

## Verification

RED:

- Created `src/tugbot_maze/test/test_phase49_dispatch_entry_readiness_gate.py`.
- Initial focused run failed as expected:
  - `5 failed`
  - missing dispatch-entry readiness gate, scan subscription, lifecycle GetState, state diagnostics, launch args, package deps.

GREEN / focused:

- `python3 -m py_compile src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_bringup/launch/tugbot_maze_explore.launch.py src/tugbot_maze/test/test_phase49_dispatch_entry_readiness_gate.py`: passed
- `python3 -m pytest src/tugbot_maze/test/test_phase49_dispatch_entry_readiness_gate.py -q`: `5 passed`

Targeted regression:

- Phase36/37/41/42/43/44/45/46/47/48/49 plus relevant maze explorer contract tests:
  - initial broad run: `67 passed`, `1 failed`
  - failure was pre-existing missing legacy asset `src/tugbot_maze/assets/maze_20260522.jpg`, unrelated to Phase49 code.
  - targeted maze_explorer/package dependency regression excluding that missing legacy asset test: `7 passed`

Guardrail verification:

- `git diff -- src/tugbot_navigation/config`: empty
- No Gazebo / SLAM / Nav2 / maze_explorer runtime launched in Phase49 implementation verification.

## Final status

`DISPATCH_ENTRY_READINESS_GATE_IMPLEMENTED`

Phase49 implementation is complete at code/test/report level. It stops before any long autonomous run or clearance strategy change. Next runtime phase, if approved, should be bounded and should verify that `maze_explorer` stays in `WAIT_FOR_DISPATCH_ENTRY_READINESS` until lifecycle/action/topic/scan/map/TF/local-costmap evidence is sufficient, and only then enters first topology sampling.
