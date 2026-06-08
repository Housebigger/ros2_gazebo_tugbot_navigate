# Phase44 Manual Nav2 Baseline Recovery and Simplification Check Report

## 当前状态

Phase44 已完成人工 RViz 观察、post-observation evidence 记录和 cleanup。

当前分类：`MANUAL_NAV2_BASELINE_FAIL`

说明：本阶段以人工 RViz 2D Pose Estimate + Nav2 Goal 验收为主。自动 wrapper 只采集 Gazebo + SLAM + Nav2 + RViz readiness evidence，不发送 Nav2 Goal，不运行自动探索，不声明自主探索成功。

允许的最终分类集合：

- `MANUAL_NAV2_BASELINE_OK`
- `MANUAL_NAV2_BASELINE_FAIL`
- `MANUAL_NAV2_INCONCLUSIVE_NEEDS_HUMAN_RVIZ_CHECK`

## Guardrails

- 不修改 Nav2/MPPI/controller 参数。
- 不修改 maze_explorer。
- 不运行 maze_explorer。
- 不运行 maze_goal_monitor。
- 不运行 frontier_explorer。
- 不继续 fallback/terminal acceptance。
- 不使用旧 scaffold world/map。
- 不声明自主探索成功。

## Active world / truth

- active world: `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf`
- active metadata: `src/tugbot_maze/config/maze_20260528_scaled_instance.yaml`
- truth frame: `map`
- entrance_map: `(0.0, 0.0, 0.0)`
- exit_map: `(21.072562, 18.083566)`
- exit_radius: `1.2`

Phase43 人工验收结论保持：`TOPOLOGY_REJECTION_CAUSE_IDENTIFIED`。不得写成自主探索成功，也不得继续直接修改 maze_explorer 策略。

## 最小手动导航启动命令

推荐人工验收命令：

```bash
cd /home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522
PHASE44_KEEP_ALIVE_FOR_HUMAN=1 tools/run_phase44_manual_nav2_baseline_recovery.sh
```

该 wrapper 启动：

- active scaled2x Gazebo world
- Tugbot / bridge / robot_state_publisher / TF
- slam_toolbox
- Nav2
- RViz

该 wrapper 不启动：

- maze_explorer
- maze_goal_monitor
- frontier_explorer

## 人工 RViz 操作步骤

1. 等待 RViz 和 Gazebo 打开，确认 RViz Fixed Frame 为 `map`。
2. 在 RViz 中确认 RobotModel、LaserScan、global/local costmap、footprint 显示合理。
3. 使用 `2D Pose Estimate` 在入口附近设置/确认机器人初始位姿；map-frame convention 下入口约为 `(0.0, 0.0, yaw=0.0)`。
4. 使用 `Nav2 Goal` 设置出口附近目标，推荐目标点靠近 `exit_map=(21.072562, 18.083566)`，在出口半径 `1.2m` 内即可。
5. 观察：
   - 是否生成 global plan；
   - local costmap/footprint 是否合理避墙；
   - Tugbot 是否运动；
   - Nav2 是否到达出口附近或失败；
   - 如失败，记录 planner/controller/action/log 现象。
6. 可保存截图到：
   - `log/phase44_manual_nav2_baseline_recovery/rviz_2d_pose_estimate.png`
   - `log/phase44_manual_nav2_baseline_recovery/rviz_nav2_goal_to_exit.png`
   - `log/phase44_manual_nav2_baseline_recovery/gazebo_manual_nav2_goal_motion.png`
7. 结束后 Ctrl-C wrapper，触发 cleanup。

## Artifacts

Wrapper/runtime artifacts 位于：

`log/phase44_manual_nav2_baseline_recovery/`

预期关键文件：

- `launch.log`
- `nodes.txt`
- `topics.txt`
- `actions.txt`
- `navigate_to_pose_action_info.txt`
- `lifecycle_states.txt`
- `map_scan_tf_odom_sample.json`
- `phase44_manual_nav2_baseline_recovery.json`
- `nav2_config_diff.txt`
- `screenshot_locations_and_manual_steps.txt`
- `cleanup_processes_after.txt`

## 验收解释

若人工 Nav2 Goal 能规划并到达出口附近：分类为 `MANUAL_NAV2_BASELINE_OK`，说明 Gazebo/SLAM/Nav2/TF/robot 基线仍可用，Phase42/43 问题限定为 maze_explorer 自动拓扑采样入口问题。

若人工 Nav2 Goal 也失败：分类为 `MANUAL_NAV2_BASELINE_FAIL`，说明当前工程基线可能被破坏，需要恢复到此前手动导航成功的 launch/config/world/map 组合。

若只完成 readiness evidence、未做人工 RViz goal：分类保持 `MANUAL_NAV2_INCONCLUSIVE_NEEDS_HUMAN_RVIZ_CHECK`。

## 验证记录

待本 phase 完成时填入：

- `bash -n tools/run_phase44_manual_nav2_baseline_recovery.sh`
- `python3 -m py_compile tools/record_phase44_manual_nav2_baseline_evidence.py`
- focused tests
- `git diff -- src/tugbot_navigation/config` empty
- cleanup check

## Stop condition

Phase44 完成后停止等待人工验收，不进入 Phase45。


## Human RViz observation update

Human observation: Tugbot is physically/visually at the maze entrance, but after applying the manual RViz 2D/Nav2 Goal pose, the robot did not respond.

Post-observation live evidence was captured in:

- `log/phase44_manual_nav2_baseline_recovery/manual_goal_no_response_evidence.txt`

Key evidence:

- `/navigate_to_pose` action info after the manual no-response observation reported `Action servers: 0`.
- `/bt_navigator` lifecycle query reported `Node not found`.
- `/goal_pose` topic had `Publisher count: 1` and `Subscription count: 0`.
- No `maze_explorer`, `maze_goal_monitor`, or `frontier_explorer` was involved.

Interpretation: the no-response behavior is a manual Nav2 baseline failure for this live run. The immediate observed blocker is that RViz goal publication is not being consumed by a live NavigateToPose action server / bt_navigator path. This is not autonomous exploration success and not a maze_explorer strategy result.

## Updated Phase44 classification

`MANUAL_NAV2_BASELINE_FAIL`

This means Phase44 cannot currently bound Phase42/43 solely to `maze_explorer` automatic topology sampling. Current engineering baseline needs recovery/investigation of the manual SLAM+Nav2 launch/runtime chain before any Phase45 or further autonomous work.


## Final cleanup and verification update

Cleanup 已执行并复查：

- `log/phase44_manual_nav2_baseline_recovery/cleanup_processes_after.txt` is empty.
- `git diff -- src/tugbot_navigation/config` is empty.
- No ROS/Gazebo/Nav2/RViz/recorder residual process matched after cleanup.

最终分类保持：`MANUAL_NAV2_BASELINE_FAIL`。

Phase44 停止于此，等待人工验收；不进入 Phase45。
