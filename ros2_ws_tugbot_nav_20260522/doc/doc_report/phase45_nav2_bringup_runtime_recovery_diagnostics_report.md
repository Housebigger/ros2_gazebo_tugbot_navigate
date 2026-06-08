# Phase45 Nav2 Bringup Runtime Recovery Diagnostics Report

## 结论

Phase45 bounded Nav2 bringup/runtime diagnostics 已完成。

最终分类：`LIFECYCLE_NOT_ACTIVE`

分类理由：`lifecycle not active for: ['/controller_server', '/planner_server', '/bt_navigator', '/behavior_server', '/waypoint_follower', '/velocity_smoother', '/smoother_server']`

Phase44 人工验收通过，结论保持：`MANUAL_NAV2_BASELINE_FAIL`。本报告不声明自主探索成功，也不继续分析 `maze_explorer` clearance/root-cause。

## Guardrails

- 未修改 Nav2/MPPI/controller 参数语义。
- 未修改 `maze_explorer`。
- 未运行 `maze_explorer` / `maze_goal_monitor` / `frontier_explorer`。
- 未继续 fallback/terminal acceptance。
- 未使用旧 scaffold world/map。
- 未声明自主探索成功。
- 未恢复或推进自动探索。
- 仅新增只读诊断脚本、静态/运行时检查、contract tests 和报告。
- cleanup 已执行，最终 cleanup artifact 为空：`True`。

## Active world / truth

- active world: `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf`
- active metadata: `src/tugbot_maze/config/maze_20260528_scaled_instance.yaml`
- truth frame: `map`
- entrance_map: `(0.0, 0.0, 0.0)`
- exit_map: `(21.072562, 18.083566)`
- exit_radius: `1.2`

## Phase44 failure carried forward

Phase44 人工观察：Tugbot 位于迷宫入口处，但 RViz 手动 2D/Nav2 Goal pose 应用后小车没有响应。

Phase44 post-observation evidence：

- `/navigate_to_pose`: `Action servers: 0`
- `/bt_navigator`: `Node not found`
- `/goal_pose`: `Publisher count: 1`, `Subscription count: 0`

Phase45 因此只诊断 Nav2 bringup/runtime 链路为什么没有形成可用手动导航 baseline。

## Static launch analysis

手动链路入口：`src/tugbot_bringup/launch/tugbot_maze_slam_nav.launch.py`

静态分析结果：

- manual launch exists: `True`
- includes `nav2_bringup`: `True`
- Nav2 launch file: `navigation_launch.py`
- uses active scaled2x world: `True`
- includes SLAM `online_async_launch.py`: `True`
- passes `params_file`: `True`
- passes `autostart`: `True`
- passes `use_sim_time`: `True`
- forbidden explorer references: `[]`

静态结论：Nav2 bringup 并没有漏 include；`navigation_launch.py` 路径和核心 launch 参数传入存在。

## Runtime evidence

Run id：`phase45_nav2_bringup_runtime_recovery_diagnostics`

Runtime 命令：

```bash
PHASE45_RUNTIME_SEC=5 PHASE45_STARTUP_WAIT_SEC=35 PHASE45_POST_CAPTURE_WAIT_SEC=5 PHASE45_HEADLESS=true PHASE45_USE_RVIZ=false tools/run_phase45_nav2_bringup_runtime_recovery_diagnostics.sh
```

关键 runtime evidence：

- `lifecycle_manager_navigation_present` in `nodes.txt`: `True`
- `slam_toolbox_present`: `True`
- required Nav2 nodes present: `{'/behavior_server': True, '/bt_navigator': True, '/controller_server': True, '/planner_server': True, '/smoother_server': True, '/velocity_smoother': True, '/waypoint_follower': True}`
- forbidden explorer nodes seen: `[]`
- `/navigate_to_pose` listed: `True`
- `/navigate_to_pose` action server count: `1`
- `/goal_pose` subscription count: `1`
- params snapshot files: `['behavior_server.yaml', 'bt_navigator.yaml', 'controller_server.yaml', 'planner_server.yaml', 'slam_toolbox.yaml', 'smoother_server.yaml', 'velocity_smoother.yaml', 'waypoint_follower.yaml']`

`navigate_to_pose_action_info.txt` captured:

```text
Action: /navigate_to_pose
Action clients: 2
    /bt_navigator
    /waypoint_follower
Action servers: 1
    /bt_navigator
```

`goal_pose_topic_info.txt` captured:

```text
Type: geometry_msgs/msg/PoseStamped
Publisher count: 0
Subscription count: 1
```

Lifecycle states captured:

```json
{
  "/behavior_server": "inactive",
  "/bt_navigator": "inactive",
  "/collision_monitor": "inactive",
  "/controller_server": "inactive",
  "/docking_server": "missing",
  "/lifecycle_manager_navigation": "missing",
  "/planner_server": "inactive",
  "/route_server": "inactive",
  "/slam_toolbox": "active",
  "/smoother_server": "inactive",
  "/velocity_smoother": "inactive",
  "/waypoint_follower": "inactive"
}
```

## Interpretation

Phase45 runtime differs from Phase44 post-observation in one important way: Nav2 nodes and `/navigate_to_pose` action server are present during bounded diagnostics. However, lifecycle states sampled at capture time show `controller_server`, `planner_server`, `bt_navigator`, `behavior_server`, `waypoint_follower`, `velocity_smoother`, and `smoother_server` still `inactive` while `/slam_toolbox` is `active`.

Launch log shows lifecycle manager was created and started bringup, and reached configuration of multiple nodes, including `bt_navigator` creating `navigate_to_pose`. The log ends around `Configuring docking_server`, with no later activation evidence in the bounded capture window. The analyzer therefore classifies this run as `LIFECYCLE_NOT_ACTIVE`, not `NAV2_BRINGUP_NOT_INCLUDED`, not `NAV2_NODES_NOT_STARTED`, not `BT_NAVIGATOR_MISSING`, and not `NAVIGATE_TO_POSE_SERVER_MISSING`.

A launch-log inflation-radius warning was recorded, but Phase45 guardrails prohibit Nav2/controller parameter tuning here; it is preserved as evidence, not acted on in this phase.

## Artifacts

- `log/phase45_nav2_bringup_runtime_recovery_diagnostics/actions.txt`
- `log/phase45_nav2_bringup_runtime_recovery_diagnostics/cleanup_processes_after.txt`
- `log/phase45_nav2_bringup_runtime_recovery_diagnostics/goal_pose_topic_info.txt`
- `log/phase45_nav2_bringup_runtime_recovery_diagnostics/launch.log`
- `log/phase45_nav2_bringup_runtime_recovery_diagnostics/lifecycle_states.txt`
- `log/phase45_nav2_bringup_runtime_recovery_diagnostics/nav2_config_diff.txt`
- `log/phase45_nav2_bringup_runtime_recovery_diagnostics/nav2_launch_static_analysis.json`
- `log/phase45_nav2_bringup_runtime_recovery_diagnostics/nav2_process_tree.txt`
- `log/phase45_nav2_bringup_runtime_recovery_diagnostics/navigate_to_pose_action_info.txt`
- `log/phase45_nav2_bringup_runtime_recovery_diagnostics/nodes.txt`
- `log/phase45_nav2_bringup_runtime_recovery_diagnostics/params_snapshot`
- `log/phase45_nav2_bringup_runtime_recovery_diagnostics/phase45_nav2_bringup_diagnostics.json`
- `log/phase45_nav2_bringup_runtime_recovery_diagnostics/precheck.txt`
- `log/phase45_nav2_bringup_runtime_recovery_diagnostics/topics.txt`

## Added / changed files

- `tools/analyze_phase45_nav2_bringup_runtime_recovery.py`
- `tools/run_phase45_nav2_bringup_runtime_recovery_diagnostics.sh`
- `src/tugbot_maze/test/test_phase45_nav2_bringup_runtime_recovery_diagnostics.py`
- `doc/doc_report/phase45_nav2_bringup_runtime_recovery_diagnostics_report.md`

## Verification

- `python3 -m py_compile tools/analyze_phase45_nav2_bringup_runtime_recovery.py src/tugbot_maze/test/test_phase45_nav2_bringup_runtime_recovery_diagnostics.py`: passed
- `bash -n tools/run_phase45_nav2_bringup_runtime_recovery_diagnostics.sh`: passed
- Phase45 focused tests: `5 passed in 0.01s`
- Phase36/37/41/42/43/44/45 focused regression: `40 passed in 0.39s`
- `git diff -- src/tugbot_navigation/config`: empty = `True`
- cleanup check: `cleanup_processes_after.txt` empty = `True`

## Phase45 final classification

`LIFECYCLE_NOT_ACTIVE`

Phase45 停止，等待人工验收；不进入 Phase46。
