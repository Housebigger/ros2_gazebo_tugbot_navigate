# Phase 9 0514 工程最终验收总结报告

## 1. 本阶段边界

Phase 9 只进行最终验收文档汇总。

本阶段未执行源码修改、未新增功能、未运行长时间实验，也未改变既有 launch/config/map 文件。

工程路径：

```text
/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
```

报告路径：

```text
doc/doc_report/phase9_final_acceptance_summary.md
```

人工验收状态：

```text
Phase 8 人工验收已通过。
Phase 9 进入 0514 工程最终总结。
```

## 2. 工程定位

`ros2_ws_tugbot_nav_20260514` 是在不修改 0513 工程的前提下，从 0513 已验证的 Tugbot 已知静态地图 Nav2 基线独立演进出的 0514 工作空间。

0514 工程完成了三层递进闭环：

1. 继承 0513 已知地图 Nav2 基线；
2. 在此基础上完成未知地图下的在线 SLAM、自主 frontier 探索和地图保存；
3. 使用自建地图完成静态地图 Nav2 回放验证。

最终工程闭环如下：

```text
Gazebo Tugbot 环境
  -> slam_toolbox 在线建图
  -> Nav2 基于 live /map 导航
  -> frontier_explorer 自主发送探索目标
  -> map_saver_cli 保存自建地图
  -> map_server 加载自建地图
  -> AMCL 在自建地图上定位
  -> Nav2 在自建地图上完成 NavigateToPose 回放
```

因此，0514 工程不只是保留了 0513 的“已知地图静态导航”能力，还额外验证了“未知地图自主探索建图 -> 地图保存 -> 自建地图静态导航回放”的完整工程链路。

## 3. 阶段结论汇总

| 阶段 | 主题 | 结论 | 关键说明 |
| --- | --- | --- | --- |
| Phase 0 | 0514 基线创建 | 通过 | 从 0513 创建独立 0514 工作空间，构建通过，原 `tugbot_nav.launch.py` 基线可用。 |
| Phase 1 | SLAM 在线建图 | 通过 | 新增 `tugbot_slam.launch.py`，`/map` 由 `slam_toolbox` 发布，不启动 `map_server` / AMCL / Nav2 navigation。 |
| Phase 2 | SLAM + Nav2 手动导航 | 通过 | 新增 `tugbot_slam_nav.launch.py`，Nav2 使用 `slam_toolbox` 实时 `/map`，手动 `NavigateToPose` 验证通过。 |
| Phase 3 | frontier 自主探索闭环 | 通过 | 新增 `frontier_explorer` 与 `tugbot_explore.launch.py`，explorer 订阅 `/map` 并自动发送 Nav2 目标。 |
| Phase 4 | 长时间探索与地图保存 | 通过，但发现提前停止问题 | 保存 `.yaml + .pgm` 成功；人工验收发现地图只覆盖部分 Gazebo 环境，日志显示 `frontier_clusters=66` 但 `valid_candidates=0` 时过早结束。 |
| Phase 5 | frontier 完成条件改良 | 通过 | 区分 raw frontier 与 valid candidate；增加 strict/relaxed search、recovery scan gate、地图稳定判断；live 达到 `completed_goals=20/20`，不再因 valid candidate 暂时为 0 直接结束。 |
| Phase 6 | coverage cleanup | 有条件通过，推荐地图产出 | 增加 residual unknown cleanup；完成 `20` 个 frontier goals 与 `5` 个 cleanup goals；保存 Phase 6 cleanup map；仍有少量 residual unknown，但地图几何质量更适合后续静态导航。 |
| Phase 7A | 参数强化复跑 | 执行通过，但地图质量不采纳 | 更高 goal/cleanup budget 执行成功并保存地图，但人工观察到重影、墙线重复、局部错层、扫描未对齐，故不作为当前最佳成果。 |
| Phase 8 | Phase 6 自建地图静态导航回放 | 通过 | 新增 `tugbot_nav_phase6_map.launch.py`；`map_server` 加载 Phase 6 cleanup map；AMCL 定位成功；Nav2 完成两个 `NavigateToPose` goal；Phase 8 已人工验收通过。 |

最终阶段判断：

```text
Phase 0~8 工程链路已闭合。
Phase 6 cleanup map 是当前推荐地图。
Phase 7A 地图保留为负面实验，不作为默认成果。
Phase 8 已证明 Phase 6 自建地图可用于静态地图导航回放。
```

## 4. 当前推荐地图

### 4.1 推荐地图

推荐作为当前最佳自建静态地图成果：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
```

绝对路径：

```text
/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
```

Phase 6 地图统计摘要：

```text
尺寸：322 x 170
free：45348
unknown：5857
occupied：3535
unknown_ratio：0.1070
```

推荐理由：

1. Phase 6 相比 Phase 4 覆盖显著扩大；
2. Phase 6 在 Phase 5 的完成条件改良基础上增加了 residual unknown cleanup；
3. Phase 6 地图几何质量优于 Phase 7A；
4. Phase 8 已证明该地图可被 `map_server` 加载，并支持 AMCL + Nav2 静态导航回放；
5. 该地图已完成 `unknown map exploration -> saved map -> static replay navigation` 的链路验证。

### 4.2 不推荐地图

不推荐作为当前默认成果：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.pgm
```

绝对路径：

```text
/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.yaml
/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.pgm
```

不推荐 Phase 7A 的原因：

1. 地图出现明显重影；
2. 墙线重复；
3. 局部错层；
4. 扫描未对齐；
5. 过度补扫导致 SLAM 一致性下降；
6. 虽然 free cells 增加，但 unknown_total_count 与 unknown_ratio 也明显升高；
7. 对后续静态导航而言，几何一致性和墙线清晰度优先级高于单纯扩大覆盖范围。

Phase 7A 的定位是负面实验记录：它证明“更多 goal + 更高 cleanup budget + 更激进补扫”并不必然提升地图质量，过度补扫反而可能放大 SLAM 累积误差。

## 5. 启动入口说明

| 启动入口 | 用途 | 是否使用静态地图 | 是否启动 AMCL | 是否使用 slam_toolbox | 说明 |
| --- | --- | --- | --- | --- | --- |
| `tugbot_nav.launch.py` | 继承 0513 的已知地图静态 Nav2 基线 | 是 | 是 | 否 | 使用原静态地图 `map_1725111373.yaml` 和原 `nav2_params.yaml`，通过 Nav2 bringup 启动 `map_server + AMCL + Nav2`。 |
| `tugbot_slam.launch.py` | 在线 SLAM-only 建图入口 | 否 | 否 | 是 | 启动 Gazebo/bridge/scan TF 与 `slam_toolbox` online async；`/map` 来自 `slam_toolbox`；不启动 Nav2 navigation。 |
| `tugbot_slam_nav.launch.py` | 在线 SLAM + Nav2 手动导航入口 | 否 | 否 | 是 | 启动 `slam_toolbox` 与 Nav2 navigation stack；Nav2 使用 live `/map`；不启动静态 `map_server` 和 AMCL。 |
| `tugbot_explore.launch.py` | 在线 SLAM + Nav2 + frontier 自主探索入口 | 否 | 否 | 是 | 复用 `tugbot_slam_nav.launch.py`，额外启动 `frontier_explorer`；可选 `save_map:=true` 保存探索地图。 |
| `tugbot_nav_phase6_map.launch.py` | Phase 6 自建地图静态导航回放入口 | 是 | 是 | 否 | Phase 8 新增独立入口；默认加载 `tugbot_nav_world_slam_phase6_cleanup.yaml`，启动 `map_server + AMCL + Nav2`，不污染原导航/SLAM/探索入口。 |

入口保护结论：

```text
原 tugbot_nav.launch.py 未被替换为 Phase 6 map。
原 tugbot_slam.launch.py 未被破坏。
原 tugbot_slam_nav.launch.py 未被破坏。
原 tugbot_explore.launch.py 保留在线探索能力。
Phase 8 使用独立 tugbot_nav_phase6_map.launch.py 做自建地图回放。
```

## 6. 关键验收证据

### 6.1 在线探索阶段 `/map` 由 slam_toolbox 发布

Phase 1~6 的在线建图、SLAM+Nav2、frontier 探索与 cleanup 阶段均以 `slam_toolbox` 发布的 live `/map` 为地图源。

关键阶段证据：

```text
Phase 1：tugbot_slam.launch.py 启动 slam_toolbox online async，实时生成 /map。
Phase 2：Nav2 使用 slam_toolbox live /map，未加载旧静态地图。
Phase 3：frontier_explorer 订阅 /map 并自动发送 NavigateToPose goal。
Phase 5：报告确认 /map 由 slam_toolbox 发布，不启动 AMCL，不启动静态 map_server。
Phase 6：报告确认 /map 仍由 slam_toolbox 发布，AMCL/map_server 未启动。
```

结论：探索建图阶段没有退化为已知静态地图导航，而是真正基于在线 SLAM 地图推进。

### 6.2 探索阶段 AMCL/map_server 不启动

Phase 1~6 的核心边界均要求探索阶段不启动静态 `map_server` 与 AMCL。

已验证结果：

```text
Phase 1：不启动 map_server，不启动 AMCL，不启动 Nav2 navigation。
Phase 2：不启动静态 map_server，不启动 AMCL。
Phase 3：tugbot_explore.launch.py 不使用 bringup_launch.py，不传入静态 map argument。
Phase 5：/map 由 slam_toolbox 发布；不启动 AMCL；不启动静态 map_server。
Phase 6：/map 仍由 slam_toolbox 发布；AMCL/map_server 未启动。
```

这保证了 Phase 3~6 的地图确实来自未知地图自主探索，而不是旧静态地图。

### 6.3 frontier_explorer 自动发送目标

Phase 3 起新增 `tugbot_exploration` 包和 `frontier_explorer` console script：

```text
frontier_explorer = tugbot_exploration.frontier_explorer:main
```

Phase 3 验证了第一版 frontier 自主探索闭环：

```text
/map -> frontier_explorer -> /navigate_to_pose
```

Phase 5 进一步验证：

```text
/frontier_explorer subscribers:
  /map: nav_msgs/msg/OccupancyGrid
  /tf: tf2_msgs/msg/TFMessage
  /tf_static: tf2_msgs/msg/TFMessage

/frontier_explorer action clients:
  /navigate_to_pose: nav2_msgs/action/NavigateToPose
  /spin: nav2_msgs/action/Spin
```

Phase 5 live run 达到：

```text
completed_goals=20/20
success=20
relaxed_search_executed=True
```

结论：frontier_explorer 已经不是静态脚本或离线分析，而是能在 ROS graph 中自动发送 Nav2 目标并推动小车探索。

### 6.4 Phase 4 提前停止问题已被 Phase 5/6 处理

Phase 4 暴露的问题：

```text
frontier_clusters=66
valid_candidates=0
```

旧逻辑把 `valid_candidates=0` 当作探索完成，导致只覆盖部分 Gazebo 环境。

Phase 5 改良后：

```text
初始 frontier_clusters=266, valid_candidates=69, search_mode=strict
中段 frontier_clusters=18,  valid_candidates=6,  search_mode=relaxed
末段 frontier_clusters=11,  valid_candidates=3
completed_goals=20/20
终止原因：reached max_goals=20
```

Phase 6 进一步完成：

```text
completed_goals=20
cleanup_goals_completed=5/5
终止原因：frontier goal budget reached and cleanup goal budget exhausted
```

结论：Phase 5/6 不再仅凭 `valid_candidates=0` 连续计数就结束探索，而是通过 raw frontier、valid candidate、relaxed search、recovery/cleanup、goal budget 和地图统计共同约束完成条件。

### 6.5 Phase 6 地图保存成功

Phase 6 推荐地图已保存成功：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml 158 bytes
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm 54755 bytes
```

YAML 内容摘要：

```yaml
image: tugbot_nav_world_slam_phase6_cleanup.pgm
mode: trinary
resolution: 0.050
origin: [-8.900, -5.977, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

地图统计摘要：

```text
尺寸：322 x 170
free：45348
unknown：5857
occupied：3535
unknown_ratio：0.1070
```

与 Phase 4 对比：

```text
Phase 4 free：26024，unknown：21914
Phase 6 free：45348，unknown：5857
```

结论：Phase 6 地图相对 Phase 4 覆盖明显扩大，并作为当前推荐成果进入 Phase 8 静态地图回放验证。

### 6.6 Phase 8 map_server 加载 Phase 6 地图

Phase 8 使用独立入口：

```text
src/tugbot_bringup/launch/tugbot_nav_phase6_map.launch.py
```

加载地图：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
```

map_server 日志证据：

```text
[map_io]: Loading yaml file: .../maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
[map_io]: Loading image_file: .../maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
[map_io]: Read map .../tugbot_nav_world_slam_phase6_cleanup.pgm: 322 X 170 map @ 0.05 m/cell
```

`/map` topic 检查：

```text
Type: nav_msgs/msg/OccupancyGrid
Publisher count: 1
Node name: map_server
QoS Durability: TRANSIENT_LOCAL
```

结论：Phase 8 中 `/map` 明确由 `/map_server` 发布，符合静态地图回放目标。

### 6.7 AMCL 定位成功

Phase 8 启动节点包含：

```text
/amcl
/map_server
/lifecycle_manager_localization
/lifecycle_manager_navigation
/planner_server
/controller_server
/bt_navigator
```

Nav2 lifecycle：

```text
/lifecycle_manager_localization/is_active -> success=True
/lifecycle_manager_navigation/is_active -> success=True
```

显式发布 initialpose 后，AMCL 接收并发布定位：

```text
[amcl]: initialPoseReceived
[amcl]: Setting pose (...): 0.000 0.000 0.000
/amcl_pose 可读
map -> base_link TF 可查
```

结论：AMCL 能在 Phase 6 自建静态地图上提供定位链路。

### 6.8 Nav2 完成两个 NavigateToPose goal

Phase 8 执行了两个静态地图回放导航目标。

Goal A：

```text
目标：x=1.0, y=0.0, orientation.w=1.0
结果：Goal accepted；error_code=0；status=SUCCEEDED
```

Goal B：

```text
目标：x=2.0, y=-1.0, orientation.w=1.0
结果：Goal accepted；error_code=0；status=SUCCEEDED
```

位移证据：

```text
启动后 / Goal A 前：x=0.000, y=0.000
Goal A 后：          x=0.887, y=-0.004
Goal B 后：          x=2.073, y=-0.895

start -> Goal A after：0.887 m
Goal A after -> Goal B after：1.483 m
start -> Goal B after：2.258 m
```

`/cmd_vel` 证据：

```text
Goal A 与 Goal B 期间均采集到非零 /cmd_vel。
```

结论：Phase 8 不只是 action 返回成功，小车也确实在 Gazebo 中产生运动，并能在 Phase 6 自建地图上完成 Nav2 静态导航回放。

## 7. 最终验收结论

最终结论：

```text
ros2_ws_tugbot_nav_20260514 已完成“未知地图自主探索建图 -> 地图保存 -> 自建地图静态导航回放”的工程闭环。
```

分层结论：

1. 0514 工程成功继承并保护了 0513 已知地图 Nav2 基线；
2. 0514 工程新增并验证了 `slam_toolbox` 在线建图入口；
3. 0514 工程新增并验证了在线 SLAM + Nav2 手动导航入口；
4. 0514 工程新增并验证了 frontier 自主探索节点；
5. 0514 工程修复了 Phase 4 暴露的 frontier premature finish 问题；
6. 0514 工程通过 Phase 6 cleanup 产出当前推荐自建地图；
7. 0514 工程识别并记录了 Phase 7A 过度补扫导致地图质量下降的负面实验；
8. 0514 工程通过 Phase 8 证明 Phase 6 自建地图可被 `map_server + AMCL + Nav2` 用于静态导航回放；
9. Phase 8 已人工验收通过，0514 工程最终验收链路成立。

推荐当前交付状态：

```text
最终验收：PASS
推荐地图：Phase 6 cleanup map
推荐静态回放入口：tugbot_nav_phase6_map.launch.py
推荐保留原入口：tugbot_nav / tugbot_slam / tugbot_slam_nav / tugbot_explore 均保留
Phase 7A：保留为负面实验，不采用为默认成果
```

## 8. 已知遗留问题

1. Phase 6 地图仍有少量 residual unknown。

   Phase 6 最终统计：

   ```text
   unknown_total_count=5857
   unknown_ratio=0.1070
   unknown_cluster_count=5
   cleanup_candidate_count=2
   ```

   说明：Phase 6 cleanup 生效，但在 `max_cleanup_goals=5` 的阶段预算下没有把所有 residual unknown 清理完。

2. Phase 7A 证明过度补扫会导致地图质量下降。

   人工验收观察到：

   ```text
   重影、墙线重复、局部错层、扫描未对齐、地图外缘 unknown 扩张
   ```

   说明：后续不应只追求更多 goal 或更高 coverage 数字，而应同时约束 SLAM 一致性和地图几何质量。

3. 当前未实现 ComputePathToPose 批量预检查。

   Phase 5/6 采用 lightweight precheck：候选点必须在 known free 区域、附近有足够 free cells、满足 obstacle clearance，并通过 blacklist/distance 过滤。Nav2 planner action 级批量预检查仍可作为后续增强。

4. 当前未实现自动地图相似度评估。

   Phase 4~8 主要依据 PGM 统计、ROS graph、Nav2 action 结果和人工 RViz/截图验收判断。尚未实现面向地图质量的自动相似度或几何一致性评分。

5. RViz 图形验收仍建议人工确认。

   Phase 8 已通过结构化证据证明 map_server/AMCL/Nav2 链路可用，但对于墙线重影、局部错层、残余 unknown 的视觉质量判断，人工 RViz/图像检查仍是必要补充。

6. 保存地图后自动退出 launch 尚未实现。

   Phase 5/6 已能保存地图，但 live launch/session 仍由外部停止。后续可考虑 explorer 保存地图后触发更明确的 shutdown/exit 策略。

## 9. 后续可选优化建议

以下内容只作为后续建议，本阶段不执行：

1. planner precheck

   在 frontier candidate 派发前批量调用 `ComputePathToPose`，过滤 planner 无法生成路径的目标点，减少 Nav2 执行期失败。

2. 地图质量自动评分

   建立轻量地图质量指标，例如墙线重复检测、局部错层检测、unknown/free/occupied 变化趋势、局部重影评分等，辅助人工验收。

3. 回环检测参数调优

   针对 Phase 7A 暴露的过度补扫重影问题，后续可单独研究 `slam_toolbox` 的回环、scan matching、correlation search、local correction 接受阈值等参数。

4. 更精细的 coverage planner

   在 frontier 和 cleanup 之间引入更明确的 coverage planning，区分可达 residual unknown、地图外缘 unknown、低收益补扫目标和高风险重复观测目标。

5. 保存地图后自动退出 launch

   在 `frontier_explorer` 完成 map save 后增加可选 shutdown 行为，例如 explorer 退出、发出完成事件，或由上层 launch/runner 管理生命周期。

## 10. 最终交付清单

最终推荐地图：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
```

最终推荐静态回放入口：

```text
src/tugbot_bringup/launch/tugbot_nav_phase6_map.launch.py
```

保留的原始/中间入口：

```text
src/tugbot_bringup/launch/tugbot_nav.launch.py
src/tugbot_bringup/launch/tugbot_slam.launch.py
src/tugbot_bringup/launch/tugbot_slam_nav.launch.py
src/tugbot_bringup/launch/tugbot_explore.launch.py
```

主要阶段报告：

```text
doc/doc_report/phase0_report.md
doc/doc_report/phase1_slam_online_mapping_report.md
doc/doc_report/phase2_slam_nav_manual_goal_report.md
doc/doc_report/phase3_frontier_exploration_report.md
doc/doc_report/phase4_exploration_map_save_report.md
doc/doc_report/phase5_frontier_completion_improvement_report.md
doc/doc_report/phase6_coverage_cleanup_report.md
doc/doc_report/phase7a_cleanup_budget_rerun_report.md
doc/doc_report/phase7a_manual_acceptance_report.md
doc/doc_report/phase8_phase6_map_replay_navigation_report.md
doc/doc_report/phase9_final_acceptance_summary.md
```

最终一句话结论：

```text
0514 Tugbot ROS 2 / Gazebo 工程已经从继承 0513 已知地图导航基线，演进到未知地图自主探索建图、地图保存，并完成自建地图静态导航回放验证；当前推荐采用 Phase 6 cleanup map 作为最终自建地图成果。
```
