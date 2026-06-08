# Phase64.5 First Dispatch Visual Reality Check / Manual Nav Comparison

Status: ready for human visual/manual observation.
Classification: VISUAL_EVIDENCE_INCONCLUSIVE

说明：Phase64.5 只提供可视化与人工对照证据。当前分类保持 `VISUAL_EVIDENCE_INCONCLUSIVE`，直到用户在 Gazebo/RViz 中观察 first dispatch target marker、robot_radius、inflation envelope、nearest wall/corridor width，并手动给出 Nav2 Goal 对照反馈。

## Preserved conclusions

- Phase61: `SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED`。
- Phase62: `CORRIDOR_TOO_NARROW`。
- Phase62 auxiliary: `LOCAL_COSTMAP_INFLATION_DOMINANT`。
- Phase63: `NO_SAFE_PROJECTION_IN_CORRIDOR`。
- Phase64: `GEOMETRY_FEASIBILITY_BLOCKED`。

这些都不是 autonomous exploration success，也不是 exit success。

## Guardrails

- 不调 Nav2/MPPI/controller 参数。
- 不调 clearance_radius_m/map threshold。
- 不改 branch scoring。
- 不改入口策略/fallback/terminal acceptance。
- 不接入 target projection。
- 不改 runtime dispatch。
- 不启动 maze_explorer。
- 允许用户手动 Nav2 Goal 做视觉对照，但不改变配置。
- 不声明 autonomous exploration success。
- 不声明 exit success。

## Added artifacts

- `src/tugbot_maze/tugbot_maze/phase64_5_first_dispatch_visual_overlay.py`
- `src/tugbot_bringup/launch/phase64_5_first_dispatch_visual_reality_check.launch.py`
- `src/tugbot_maze/test/test_phase64_5_first_dispatch_visual_reality_check.py`
- `doc/doc_report/phase64_5_first_dispatch_visual_reality_check_report.md`

## Overlay marker topic

`/phase64_5/first_dispatch_visual_markers`

Marker elements:

- `first dispatch target marker`: red sphere at Phase62/64 automatic first dispatch target。
- `dispatch_pose`: cyan arrow at robot pose when first dispatch was sent。
- `robot_radius_circle_at_target`: Nav2 `robot_radius` envelope around target。
- `mesh_radius_circle_at_target`: Tugbot mesh body radius around target。
- `inflation_envelope_circle_at_target`: `robot_radius + local inflation radius` safety envelope。
- `effective_corridor_width_line`: replay-derived corridor width at first-dispatch target。
- `nearest_wall_distance_line`: nearest wall / clearance helper line。
- `manual_goal_comparison_note`: reminds that manual Nav2 Goal comparison is not autonomous/exit success。

## Reused Phase64 evidence shown in RViz

- first_dispatch target_map: `[0.558242, 0.510847]`
- dispatch_pose_map: `[0.874077, 0.020696, 0.031983]`
- robot_radius_m: `0.35`
- mesh_radius_m: `0.369122`
- inflation_radius_m: `0.70`
- inflation full radius shown around target: `1.05`
- replay effective corridor width at target: `0.806226 m`
- target_clearance_m: `0.403113 m`
- nearest_sdf_wall: `maze_wall_outer_026_outer_003`
- nearest_sdf_wall_distance_m: `0.625145 m`
- local_radius_max_cost: `99`
- footprint_max_cost: `99`
- footprint_lethal_count: `8`

## Launch command

From workspace root:

```bash
cd /home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install
source install/setup.bash
ros2 launch tugbot_bringup phase64_5_first_dispatch_visual_reality_check.launch.py
```

If the installed share-path default cannot locate the repo-local log artifact, pass it explicitly:

```bash
ros2 launch tugbot_bringup phase64_5_first_dispatch_visual_reality_check.launch.py \
  phase64_artifact:=/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/log/phase64_corridor_width_robot_footprint_feasibility_decision/phase64_corridor_width_robot_footprint_feasibility_decision.json
```

## RViz setup

1. Fixed Frame: `map`。
2. 确认 Gazebo 显示 active clean scaled2x world。
3. 确认 RViz 中 Map、LaserScan、TF、RobotModel、costmaps 正常。
4. 添加 MarkerArray display：topic 选择 `/phase64_5/first_dispatch_visual_markers`。
5. 观察红色 first dispatch target marker 是否贴近墙/窄通道。
6. 观察 green/yellow body circles 与 orange inflation envelope 是否明显覆盖墙体/高风险区域。
7. 观察 purple corridor width line 与 white nearest-wall helper line 是否符合 Phase64 几何判断。

## Manual Nav2 Goal comparison steps

允许用户手动 Nav2 Goal 对照，但只作为视觉/人工证据，不改变配置：

1. 等待 Nav2 lifecycle active，并确认 `/navigate_to_pose` action server 可用。
2. 在 RViz 中手动发送一个你认为“道路可通过”的 Nav2 Goal。
3. 对比手动 goal 与红色 first dispatch target marker：
   - 手动 goal 是否明显远离红色危险点？
   - 手动 goal 是否位于更居中的走廊中心线？
   - 手动路径是否避开 orange inflation envelope 和 nearest wall helper 指示的区域？
4. 记录截图建议：
   - RViz：显示 map/costmap/robot model/MarkerArray/manual goal/path。
   - Gazebo：显示 Tugbot 与墙体相对位置。
5. 记录文字观察：
   - 自动 first dispatch 是否视觉上贴墙？
   - 手动 Nav2 Goal 是否避开 risky target？
   - 机器人是否只是通过了手动选择的更安全点，而不是证明自动 first dispatch target 安全？

## Classification guidance after human observation

- `VISUALLY_CONFIRMED_TOO_CLOSE`: red first-dispatch target/body/inflation marker 明显贴墙或覆盖障碍/墙体。
- `VISUALLY_MARGINAL_BUT_PASSABLE`: target 看起来边界很窄但人工认为仍可通过，需保留 Phase64 geometry mismatch caveat。
- `MANUAL_GOAL_AVOIDS_RISKY_TARGET`: 手动 Nav2 Goal 可通过，但目标明显避开自动 first-dispatch risky target。
- `VISUAL_EVIDENCE_INCONCLUSIVE`: marker/costmap/截图/人工观察不足，不能裁决。

## Current classification

`VISUAL_EVIDENCE_INCONCLUSIVE`

原因：Phase64.5 已实现 overlay 与人工观察流程，但尚未由用户完成 Gazebo/RViz 人工观察并反馈截图/日志。因此不能把 Phase64 推翻，也不能声明手动可通过等同自动 first dispatch 安全。

## Stop condition

Phase64.5 到此停止，等待人工验收；不进入 Phase65。
