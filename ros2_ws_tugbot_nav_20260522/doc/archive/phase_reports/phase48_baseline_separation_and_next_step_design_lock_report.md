# Phase48 Baseline Separation and Next-Step Design Lock Report

## Final classification

`BASELINE_SEPARATION_AND_NEXT_STEP_DESIGN_LOCKED`

Phase48 是 design-lock / route-convergence phase。本阶段不跑实验，不启动 Gazebo/SLAM/Nav2/maze_explorer runtime，不引入新的 navigation 结论；只把 Phase42/43/46/47 已验收证据分层，锁定后续自动探索修复的前置条件与禁止混淆边界。

## Locked baseline state

手动 Nav2 baseline 已经确认可用。

已验收链路如下：

- active world: `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf`
- active truth frame: `map`
- manual launch family: `tugbot_maze_slam_nav.launch.py`
- Phase46: `NAV2_LIFECYCLE_ACTIVE_RECOVERED`
- Phase47: `MANUAL_NAV2_GOAL_BASELINE_OK`

Phase47 人工观察确认：

- RViz Fixed Frame = `map`。
- Gazebo 中 Tugbot 位于 active scaled2x maze 内。
- RViz 中 Map / LaserScan / TF / RobotModel 显示正常。
- readiness gates 满足后，在 RViz 中发送人工 Nav2 Goal。
- 手动 Nav2 Goal 后小车能够运动。
- 小车能够沿迷宫通道规划/避障并到达出口附近。
- 截图显示 RViz 地图与 Gazebo active scaled2x world 对应，机器人已位于右下出口附近。

因此当前 Gazebo/SLAM/Nav2/TF/manual RViz Nav2 Goal 基线可用。

这不是 autonomous exploration success，也不是 `maze_explorer` 成功；它只证明手动 Nav2 baseline smoke 可用。

## Preserved automatic-exploration conclusions

Phase48 不重写任何早期验收结论：

- Phase37 保持：`BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH`，不得写成自主探索成功。
- Phase42 保持：`BOUNDED_SMOKE_NO_DISPATCH_TOPOLOGY_SAMPLING`。
- Phase43 保持：`TOPOLOGY_REJECTION_CAUSE_IDENTIFIED`。
- Phase46 保持：`NAV2_LIFECYCLE_ACTIVE_RECOVERED`。
- Phase47 保持：`MANUAL_NAV2_GOAL_BASELINE_OK`。

自动探索失败点限定在 `maze_explorer` 首次 topology sampling / dispatch entry：

- Phase42：`maze_explorer` 启动，`/navigate_to_pose` action server 可用，但 `goal_count=0`、`dispatch_events=0`、最终 `FAILED_EXHAUSTED`，分类为 `BOUNDED_SMOKE_NO_DISPATCH_TOPOLOGY_SAMPLING`。
- Phase43：首次 topology sampling 四个方向全部被拒绝，直接证据为 `clearance_radius_blocked`，`raw_open_direction_count=0`、`filtered_open_direction_count=0`、`candidate_branch_count=0`，分类为 `TOPOLOGY_REJECTION_CAUSE_IDENTIFIED`。

由 Phase46/47 可知，不应再把 Phase42/43 no-dispatch 归因成 Nav2 lifecycle 未 active 或手动 Nav2 Goal baseline 不可用。

## Mandatory preconditions for future autonomous-exploration fixes

后续所有自动探索修复必须以前置条件为入口 gate。缺任一项时，不得解释、比较或接受 `maze_explorer` 自动探索结果。

1. 等待 Nav2 lifecycle active。

   必须在允许 `maze_explorer` 自动 dispatch 前确认：

   - `/controller_server active [3]`
   - `/planner_server active [3]`
   - `/bt_navigator active [3]`
   - `/navigate_to_pose Action servers: 1`
   - `/goal_pose Subscription count: 1`

2. 等待 scan/map sufficient。

   自动探索入口必须采集并判断完整的 map/scan/TF/local costmap sufficiency evidence，而不是用截断 topic echo 代替证据。最低证据包括：

   - map/odom/base_link TF 可用且 frame alignment 符合 active map-frame truth。
   - `/scan` finite_count 与 nearest_obstacle 合理。
   - `/map` 在 robot 附近 known/free/unknown/occupied ratio 足以支持 topology sampling。
   - `/local_costmap/costmap` 在 robot 附近 known/free/occupied ratio 足以支持 clearance 判断。
   - 首次 topology sampling 前后的 `maze_explorer` state/diagnostic payload 有可追踪时间线。

3. 不破坏手动基线。

   每个自动探索修复候选必须保持 manual baseline separation：

   - 不修改 Nav2/MPPI/controller 参数，除非某个后续 phase 明确以 Nav2 参数实验为目标，并先证明 Phase47-style manual baseline 仍可回归。
   - 不让 `maze_explorer` 策略改动影响 `tugbot_maze_slam_nav.launch.py` 的手动 SLAM+Nav2 baseline。
   - 自动探索修复后如重新运行 smoke，必须能追溯到 Phase47 的 manual Nav2 Goal baseline 未被破坏。

## Problem-class separation lock

禁止再把 lifecycle 未 active、手动 Goal 无响应、maze_explorer no-dispatch 混成一个问题。

后续报告、测试、分析必须明确区分三类问题：

### 1. Lifecycle activation problem

定义：Nav2 lifecycle nodes 尚未 active，或者 `/navigate_to_pose` action server / `/goal_pose` subscriber 尚未就绪。

已知历史边界：

- Phase45 是 `LIFECYCLE_NOT_ACTIVE`。
- Phase46 在更长 bounded wait/capture 下恢复为 `NAV2_LIFECYCLE_ACTIVE_RECOVERED`。

后续不能在 lifecycle 未 active 时评价 `maze_explorer` dispatch 能力。

### 2. Manual Nav2 Goal baseline problem

定义：Gazebo/SLAM/Nav2/TF/manual RViz goal 链路自身不可用，例如 RViz goal 无响应、无法规划、无法运动、TF/map/scan display 不正常。

已知历史边界：

- Phase44 是 `MANUAL_NAV2_BASELINE_FAIL`。
- Phase47 在 Phase46 lifecycle active 前提下恢复并人工确认 `MANUAL_NAV2_GOAL_BASELINE_OK`。

不得用 Phase42/43 no-dispatch 否定 Phase47 手动基线。Phase47 证明的是手动 goal baseline；它不证明 `maze_explorer` 成功。

### 3. maze_explorer topology sampling/dispatch entry problem

定义：在 Gazebo/SLAM/Nav2/TF/manual baseline 可用、Nav2 lifecycle/action/topic readiness 已满足后，`maze_explorer` 仍在首次 topology sampling / branch candidate generation 前后无法形成可 dispatch goal。

已知历史边界：

- Phase42 是 no-dispatch topology sampling failure。
- Phase43 将直接原因定位为首次 topology sampling 四方向 `clearance_radius_blocked`。

不得用 Nav2 lifecycle 或手动 Goal 问题解释 Phase42/43 no-dispatch；后续自动探索修复应聚焦 `maze_explorer` 首次 topology sampling 的入口时序、map/scan/local-cost sufficiency、clearance sampling geometry 与 diagnostics。

## Next-step design lock

后续如果进入 Phase49 或其他自动探索修复 phase，必须遵守以下设计入口：

1. 先复用 Phase47-style readiness gate，确保 lifecycle/action/topic baseline：
   - `/controller_server active [3]`
   - `/planner_server active [3]`
   - `/bt_navigator active [3]`
   - `/navigate_to_pose Action servers: 1`
   - `/goal_pose Subscription count: 1`

2. 再加入 Phase39/43-style scan/map sufficiency gate：
   - 不依赖 `ros2 topic echo` 截断数组作为地图依据。
   - 用 rclpy 或完整 artifact 采集 map/scan/TF/local costmap。
   - 在 `maze_explorer` 首次 topology sampling 前记录 sufficiency 快照。

3. 自动探索修复范围必须锁定在 `maze_explorer` 首次 topology sampling / dispatch entry：
   - 若改代码，优先做诊断/等待门/入口保护，不直接调 Nav2/MPPI/controller。
   - 若要修改 topology sampling/clearance 策略，必须先有新 evidence 证明是策略问题，而不是数据尚未 ready。
   - 不引入 fallback/terminal acceptance 作为绕过入口问题的成功标准。

4. 每次候选修改后必须保护 manual baseline：
   - 至少保留 Phase47-style manual smoke contract。
   - 若 Nav2 config 有 diff，必须明确属于后续授权 phase，否则视为 guardrail violation。

## Guardrails honored in Phase48

- 本阶段不跑实验。
- 未运行 Gazebo/SLAM/Nav2/maze_explorer runtime。
- 未修改 Nav2/MPPI/controller 参数。
- 未修改 `maze_explorer`。
- 未继续 fallback/terminal acceptance。
- 未使用旧 scaffold world/map。
- 不声明自主探索成功。
- 不声明 `maze_explorer` 成功。
- 仅新增 design-lock report 与 contract test。

## Verification

Phase48 verification items:

- RED: Phase48 contract test failed before this report existed.
- `python3 -m py_compile src/tugbot_maze/test/test_phase48_baseline_separation_and_next_step_design_lock.py`: passed.
- Phase48 focused test: `4 passed in 0.00s`.
- Phase36/37/41/42/43/44/45/46/47/48 focused regression: `53 passed in 0.39s`.
- `git diff -- src/tugbot_navigation/config: empty`.
- `cleanup check: empty`.
- live process check: empty.

## Final status

`BASELINE_SEPARATION_AND_NEXT_STEP_DESIGN_LOCKED`

Phase48 完成后应停止等待人工验收，不进入 Phase49，除非用户明确要求。
