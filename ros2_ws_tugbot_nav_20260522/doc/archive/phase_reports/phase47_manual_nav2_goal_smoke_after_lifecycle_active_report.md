# Phase47 Manual Nav2 Goal Smoke After Lifecycle Active Report

## Final classification

`MANUAL_NAV2_GOAL_BASELINE_OK`

Phase47 已根据人工观察完成收尾。

## Human observation summary

用户在 Phase47 visible Gazebo/RViz + SLAM + Nav2 run 中确认：

- RViz Fixed Frame = `map`。
- Gazebo 中 Tugbot 位于 active scaled2x maze 内。
- RViz 中 Map / LaserScan / TF / RobotModel 显示正常。
- readiness gates 满足后，在 RViz 中发送人工 Nav2 Goal。
- 手动 Nav2 Goal 后小车能够运动。
- 小车能够沿迷宫通道规划/避障并到达出口附近。
- 截图显示 RViz 地图与 Gazebo active scaled2x world 对应，机器人已位于右下出口附近。
- 这不是自主探索成功，也不是 `maze_explorer` 成功；只是手动 Nav2 baseline smoke 成功。

## Preserved phase conclusions

- Phase46 结论保持：`NAV2_LIFECYCLE_ACTIVE_RECOVERED`。
- Phase42 结论保持：`BOUNDED_SMOKE_NO_DISPATCH_TOPOLOGY_SAMPLING`。
- Phase43 结论保持：`TOPOLOGY_REJECTION_CAUSE_IDENTIFIED`。
- Phase37 仍不得写成自主探索成功。

## Updated interpretation

当前 Gazebo/SLAM/Nav2/TF/manual RViz Nav2 Goal 基线可用。

因此 Phase42/43 的问题重新限定为 `maze_explorer 自动拓扑采样/dispatch 入口问题`，而不是 Gazebo/SLAM/Nav2/TF/manual Nav2 Goal baseline 不可用。

这不改变 Phase42/43 已验收的 no-dispatch / topology rejection 结论，也不代表自主探索成功。

## Guardrails

- 未修改 Nav2/MPPI/controller 参数。
- 未修改 `maze_explorer`。
- 未运行 `maze_explorer` / `maze_goal_monitor` / `frontier_explorer`。
- 未使用旧 scaffold world/map。
- 不声明自主探索成功。
- 不声明 `maze_explorer` 成功。
- Phase47 仅证明 manual RViz Nav2 Goal baseline 可用。

## Readiness gates before manual goal

Phase47 wrapper 在允许人工 RViz Nav2 Goal 前已等待并通过：

- `/controller_server active [3]`
- `/planner_server active [3]`
- `/bt_navigator active [3]`
- `/navigate_to_pose Action servers: 1`
- `/goal_pose Subscription count: 1`

### lifecycle_readiness.txt

```text
# lifecycle readiness 2026-06-01T12:00:18+08:00
Node not found
Node not found
Node not found
# gate snapshot 2026-06-01T12:00:21+08:00
Node not found
Node not found
Node not found
controller_ok=0
planner_ok=0
bt_navigator_ok=0
navigate_to_pose_action_servers_1=0
goal_pose_subscription_count_1=0
# lifecycle readiness 2026-06-01T12:00:29+08:00
active [3]
active [3]
active [3]
# gate snapshot 2026-06-01T12:00:47+08:00
active [3]
active [3]
active [3]
controller_ok=1
planner_ok=1
bt_navigator_ok=1
navigate_to_pose_action_servers_1=1
goal_pose_subscription_count_1=1
```

### navigate_to_pose_action_info.txt

```text
Action: /navigate_to_pose
Action clients: 2
    /bt_navigator
    /waypoint_follower
Action servers: 1
    /bt_navigator
```

### goal_pose_topic_info.txt

```text
Type: geometry_msgs/msg/PoseStamped
Publisher count: 1
Subscription count: 1
```

## Cleanup

Phase47 visible Gazebo/RViz + SLAM + Nav2 run 已停止。

`cleanup_processes_after.txt`:

```text
[empty]
```

cleanup check: empty

## Files added/updated

- `tools/run_phase47_manual_nav2_goal_smoke_after_lifecycle_active.sh`
- `src/tugbot_maze/test/test_phase47_manual_nav2_goal_smoke_after_lifecycle_active.py`
- `doc/doc_report/phase47_manual_nav2_goal_smoke_after_lifecycle_active_report.md`

## Artifacts

- `log/phase47_manual_nav2_goal_smoke_after_lifecycle_active/launch.log`
- `log/phase47_manual_nav2_goal_smoke_after_lifecycle_active/launch.pid`
- `log/phase47_manual_nav2_goal_smoke_after_lifecycle_active/nodes.txt`
- `log/phase47_manual_nav2_goal_smoke_after_lifecycle_active/topics.txt`
- `log/phase47_manual_nav2_goal_smoke_after_lifecycle_active/actions.txt`
- `log/phase47_manual_nav2_goal_smoke_after_lifecycle_active/lifecycle_readiness.txt`
- `log/phase47_manual_nav2_goal_smoke_after_lifecycle_active/navigate_to_pose_action_info.txt`
- `log/phase47_manual_nav2_goal_smoke_after_lifecycle_active/goal_pose_topic_info.txt`
- `log/phase47_manual_nav2_goal_smoke_after_lifecycle_active/phase47_manual_nav2_goal_smoke_readiness.json`
- `log/phase47_manual_nav2_goal_smoke_after_lifecycle_active/manual_goal_observation_notes.md`
- `log/phase47_manual_nav2_goal_smoke_after_lifecycle_active/cleanup_processes_after.txt`

## Verification

- Phase47 final-classification RED: focused report test failed before report update because `MANUAL_NAV2_GOAL_BASELINE_OK` was absent.
- `bash -n tools/run_phase47_manual_nav2_goal_smoke_after_lifecycle_active.sh`: passed.
- `python3 -m py_compile src/tugbot_maze/test/test_phase47_manual_nav2_goal_smoke_after_lifecycle_active.py`: passed.
- Phase47 focused tests: passed.
- Phase36/37/41/42/43/44/45/46/47 focused regression: passed.
- `git diff -- src/tugbot_navigation/config`: empty.
- cleanup check: empty.

## Final status

`MANUAL_NAV2_GOAL_BASELINE_OK`

Phase47 完成，停止等待人工验收；不进入 Phase48。
