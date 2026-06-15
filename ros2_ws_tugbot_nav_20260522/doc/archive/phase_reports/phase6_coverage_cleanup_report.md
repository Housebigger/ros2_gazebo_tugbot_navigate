# Phase 6 Report - coverage cleanup 长时间探索与残余 unknown 补扫验收报告

## 1. 阶段目标

Phase 5 人工验收发现：frontier 完成条件已不再过早结束，主体地图覆盖明显优于 Phase 4，但保存地图右上角仍存在一片 residual unknown。Phase 6 的目标是在保留 Phase 5 strict/relaxed frontier search 与 recovery scan 的基础上，增加轻量 coverage cleanup：当普通 frontier 探索后期候选减少或 goal 预算前后仍存在靠近已知 free 区域的 unknown cluster 时，自动选择安全观测点导航过去，并执行原地旋转补扫。

本阶段边界：

- 不修改 0513 工程；
- 不破坏 `tugbot_nav.launch.py`、`tugbot_slam.launch.py`、`tugbot_slam_nav.launch.py`；
- 保留 `tugbot_explore.launch.py` 原有启动能力；
- 不修改原 `nav2_params.yaml`；
- 不做复杂地图相似度算法；
- cleanup 只针对“靠近 known free 区域的 residual unknown”，不把地图外不可达 unknown 当作必须补扫目标。

## 2. 修改文件清单

本阶段修改以下文件：

1. `src/tugbot_exploration/tugbot_exploration/frontier_explorer.py`

   主要改动：

   - 新增 `UnknownCluster` 与 `CleanupCandidate` 数据结构；
   - 新增 residual unknown cluster 检测；
   - 新增 cleanup candidate 生成：在 unknown cluster 附近搜索 known free、安全、非 blacklist 的观测点；
   - 新增 cleanup mode gate：当普通 frontier 后期、candidate 较少或 frontier goal budget 接近耗尽时进入 cleanup；
   - 新增 cleanup goal 与 cleanup spin 执行路径；
   - 新增 cleanup 相关日志字段：`unknown_total_count`、`unknown_ratio`、`unknown_cluster_count`、`cleanup_candidate_count`、`cleanup_goals_completed`、`cleanup_spins_completed`；
   - 修复 Phase 6 初次 live run 暴露的问题：`UnknownCluster` / `CleanupCandidate` dataclass 未在运行时定义，导致 `frontier_explorer` 启动后退出。

2. `src/tugbot_bringup/launch/tugbot_explore.launch.py`

   主要改动：

   - 新增并传递 Phase 6 cleanup 参数；
   - 保持 `tugbot_slam_nav.launch.py` include 关系和原探索入口。

3. `src/tugbot_bringup/test/test_contract.py`

   主要改动：

   - Contract tests 扩展到 9 个；
   - 新增 Phase 6 cleanup 参数、日志字段、completion/save gate 相关契约检查；
   - 增加对 `UnknownCluster` / `CleanupCandidate` dataclass 存在性的检查，防止再次出现只写类型注解但运行时缺失定义的问题。

本阶段未修改：

```text
src/tugbot_bringup/launch/tugbot_nav.launch.py
src/tugbot_bringup/launch/tugbot_slam.launch.py
src/tugbot_bringup/launch/tugbot_slam_nav.launch.py
src/tugbot_navigation/config/nav2_params.yaml
```

## 3. 新增参数清单

Phase 6 新增 cleanup 参数如下：

```text
enable_cleanup_mode                 默认 true
cleanup_unknown_cluster_min_size     默认 30
cleanup_max_unknown_clusters         默认 5
cleanup_search_radius_min_m          默认 0.5
cleanup_search_radius_max_m          默认 2.0
cleanup_min_obstacle_distance_m      默认 0.35
cleanup_goal_timeout_sec             默认 60.0
cleanup_spin_after_goal              默认 true
cleanup_spin_angle                   默认 6.28
cleanup_wait_after_spin_sec          默认 2.0
target_unknown_ratio                 默认 0.03
max_cleanup_goals                    默认 5
```

live run 启动日志确认参数已生效：

```text
frontier_explorer started: map_topic=/map action_name=/navigate_to_pose spin_action_name=/spin min_obstacle_distance_m=0.45 min_goal_distance_m=0.50 max_goal_distance_m=4.00 relaxed_min_obstacle_distance_m=0.35 relaxed_max_goal_distance_m=8.00 min_cluster_size=5 relaxed_min_cluster_size=3 max_goals=20 min_goals_before_finish=8 min_frontier_clusters_before_finish=5 recovery_scan_attempts=3 relaxed_retry_attempts=3 map_stable_cycles_before_finish=3 map_change_free_threshold=50 map_change_unknown_threshold=50 enable_recovery_scan=True recovery_spin_angle=6.28 recovery_wait_after_scan_sec=2.0 enable_cleanup_mode=True cleanup_unknown_cluster_min_size=30 cleanup_max_unknown_clusters=5 cleanup_search_radius_min_m=0.50 cleanup_search_radius_max_m=2.00 cleanup_min_obstacle_distance_m=0.35 cleanup_goal_timeout_sec=60.0 cleanup_spin_after_goal=True cleanup_spin_angle=6.28 cleanup_wait_after_spin_sec=2.0 target_unknown_ratio=0.030 max_cleanup_goals=5 save_map=True map_save_path=/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup
```

## 4. Phase 6 cleanup 设计说明

### 4.1 residual unknown cluster 检测

Phase 6 不做复杂地图相似度算法，只在当前 `/map` 上做轻量统计：

- 找出 `value == -1` 的 unknown cells；
- 聚类后过滤小 cluster；
- 只保留与已知 free cell 相邻的 unknown cluster；
- 优先处理 free boundary 多、面积大的 cluster。

这样避免把地图外部完全不可达 unknown 当作必须补扫对象。

### 4.2 cleanup candidate 生成

对每个 residual unknown cluster：

- 在 cluster center 周围 `cleanup_search_radius_min_m ~ cleanup_search_radius_max_m` 搜索 candidate cell；
- candidate 必须是 known free cell；
- candidate 必须满足 `cleanup_min_obstacle_distance_m` 安全距离；
- candidate 附近需要有足够 free cells；
- candidate 不能落入 blacklist；
- yaw 朝向 unknown cluster center；
- score 兼顾 cluster size、free boundary、clearance、free neighbor、观察距离和机器人当前距离。

### 4.3 cleanup 执行策略

- 普通 frontier goal 仍优先推进主体探索；
- 当后期 frontier candidate 变少、地图 unknown ratio 仍高、或已接近/达到 frontier goal budget 时，进入 `cleanup mode=True`；
- cleanup goal 成功后执行 Nav2 Spin action 原地旋转补扫；
- 若仍有 cleanup candidate，但 `max_cleanup_goals` 已用完，允许保存地图并在报告中明确遗留 residual unknown。

## 5. 静态验证结果

执行命令：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
python3 -m py_compile src/tugbot_exploration/tugbot_exploration/frontier_explorer.py src/tugbot_bringup/launch/tugbot_explore.launch.py
python3 -m pytest -q src/tugbot_bringup/test/test_contract.py
colcon build --symlink-install
```

结果：

```text
9 passed in 0.01s
Summary: 5 packages finished
```

最终报告前再次复验：

```text
python3 -m py_compile ... 通过
python3 -m pytest -q src/tugbot_bringup/test/test_contract.py -> 9 passed in 0.01s
```

## 6. live 验证命令

执行的 Phase 6 长时间探索命令：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source install/setup.bash
ros2 launch tugbot_bringup tugbot_explore.launch.py   headless:=true   use_rviz:=false   max_goals:=20   min_goals_before_finish:=8   enable_cleanup_mode:=true   max_cleanup_goals:=5   target_unknown_ratio:=0.03   save_map:=true   map_save_path:=/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup
```

说明：第一次 live run 暴露 `UnknownCluster` / `CleanupCandidate` 未定义问题，`frontier_explorer` 启动后退出。已修复并重新执行第二次长时间验证。最终有效证据来自第二次 run：

```text
background session: proc_7b20d4bc43ac
full log snapshot: /tmp/hermes-results/call_v9sSc7Zi3oHEheXsQnTXoXb7.txt
```

## 7. live 日志证据

### 7.1 初始地图与 frontier 状态

```text
Map 315x154 res=0.050, raw frontier cluster count=266, valid candidate count=69, frontier_clusters=266, valid_candidates=69, finish_no_frontier_cycles=0/5, recovery_no_candidate_cycles=0/3, relaxed_no_candidate_cycles=0/3, recovery_scans_completed=0/3, recovery_mode=False, search_mode=strict, blacklist=0, completed_goals=0/20, min_goals_before_finish=8, map_stable_cycles=0/3, unknown_total_count=41601 unknown_ratio=0.8576 unknown_cluster_count=1 cleanup_candidate_count=1, cleanup mode=False cleanup_goals_completed=0/5 cleanup_spins_completed=0, map_stats unknown=41601 free=6710 occupied=199 delta_unknown=0 delta_free=0 delta_occupied=0
```

### 7.2 cleanup mode 触发证据

```text
Map 315x155 res=0.050, raw frontier cluster count=14, valid candidate count=6, frontier_clusters=14, valid_candidates=6, finish_no_frontier_cycles=0/5, recovery_no_candidate_cycles=0/3, relaxed_no_candidate_cycles=0/3, recovery_scans_completed=0/3, recovery_mode=False, search_mode=strict, blacklist=0, completed_goals=11/20, min_goals_before_finish=8, map_stable_cycles=0/3, unknown_total_count=8069 unknown_ratio=0.1653 unknown_cluster_count=5 cleanup_candidate_count=3, cleanup mode=True cleanup_goals_completed=0/5 cleanup_spins_completed=0, map_stats unknown=8069 free=38879 occupied=1877 delta_unknown=456 delta_free=414 delta_occupied=42
```

该日志说明：

- 普通 frontier 仍在运行；
- `completed_goals=11/20` 后进入 cleanup；
- `cleanup mode=True`；
- 当时仍有 `cleanup_candidate_count=3`；
- unknown 从初始 `41601` 降到 `8069`。

### 7.3 cleanup goal 证据

本次 live run 选择了 5 个 cleanup goals：

```text
Selected cleanup goal #1: x=-6.063 y=1.810 yaw=-1.938 residual unknown cluster size=349 distance=1.334 score=149.797 clearance_cells=9 free_neighbor_count=105 unknown_ratio=0.1653 cleanup_candidate_count=3
Selected cleanup goal #2: x=-1.213 y=1.510 yaw=-1.642 residual unknown cluster size=294 distance=6.511 score=119.984 clearance_cells=8 free_neighbor_count=102 unknown_ratio=0.1634 cleanup_candidate_count=3
Selected cleanup goal #3: x=-5.264 y=-4.309 yaw=1.713 residual unknown cluster size=276 distance=5.837 score=113.089 clearance_cells=10 free_neighbor_count=105 unknown_ratio=0.1658 cleanup_candidate_count=1
Selected cleanup goal #4: x=2.336 y=-4.109 yaw=0.528 residual unknown cluster size=111 distance=7.563 score=61.675 clearance_cells=11 free_neighbor_count=113 unknown_ratio=0.1657 cleanup_candidate_count=1
Selected cleanup goal #5: x=3.477 y=-4.903 yaw=-1.831 residual unknown cluster size=1536 distance=6.914 score=562.582 clearance_cells=10 free_neighbor_count=113 unknown_ratio=0.1096 cleanup_candidate_count=2
```

统计：

```text
cleanup goals selected: 5
cleanup goals succeeded: 5
```

### 7.4 cleanup spin 证据

```text
Starting cleanup spin: attempt=1/5 target_yaw=6.28 wait_after_spin=2.0s
Starting cleanup spin: attempt=2/5 target_yaw=6.28 wait_after_spin=2.0s
Starting cleanup spin: attempt=3/5 target_yaw=6.28 wait_after_spin=2.0s
Starting cleanup spin: attempt=4/5 target_yaw=6.28 wait_after_spin=2.0s
Starting cleanup spin: attempt=5/5 target_yaw=6.28 wait_after_spin=2.0s
```

spin 结果：

```text
cleanup spin attempts: 5
cleanup spin result callbacks: 5
cleanup spins completed successfully according to final summary: 4
sample spin result lines:
cleanup spin finished: status=6 error_code=703 error_msg= success=False; waiting 2.0s for map update
cleanup spin finished: status=4 error_code=0 error_msg= success=True; waiting 2.0s for map update
```

解释：一次 spin 返回 `status=6 error_code=703 success=False`，随后系统继续执行后续 cleanup / frontier goal；最终完成日志统计 `cleanup_spins_completed=4`。

### 7.5 final frontier / cleanup / map stats

```text
Map 322x170 res=0.050, raw frontier cluster count=46, valid candidate count=6, frontier_clusters=46, valid_candidates=6, finish_no_frontier_cycles=0/5, recovery_no_candidate_cycles=0/3, relaxed_no_candidate_cycles=0/3, recovery_scans_completed=0/3, recovery_mode=False, search_mode=strict, blacklist=1, completed_goals=20/20, min_goals_before_finish=8, map_stable_cycles=2/3, unknown_total_count=5857 unknown_ratio=0.1070 unknown_cluster_count=5 cleanup_candidate_count=2, cleanup mode=False cleanup_goals_completed=5/5 cleanup_spins_completed=4, map_stats unknown=5857 free=45348 occupied=3535 delta_unknown=10 delta_free=13 delta_occupied=23
```

最终完成日志：

```text
Exploration complete: frontier goal budget reached and cleanup goal budget exhausted; completed_goals=20 success=20 failure=2 recovery_scans_completed=0 relaxed_search_executed=False map_stable_cycles=2 cleanup_goals_completed=5/5 cleanup_spins_completed=4 unknown_total_count=5857 unknown_ratio=0.1070 unknown_cluster_count=5 cleanup_candidate_count=2
```

最终地图保存日志：

```text
Map save succeeded: yaml=/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml bytes=158 pgm=/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm bytes=54755
```

## 8. goal 执行统计

从有效 live log 解析得到：

```text
frontier goals selected: 21
frontier goals succeeded: 20
cleanup goals selected: 5
cleanup goals succeeded: 5
cleanup spins started: 5
cleanup spins completed successfully: 4
NavigateToPose true results: 25
NavigateToPose false results: 1
planner failure count observed: 1
frontier_explorer process died count in valid run: 0
```

最终 completed_goals 达到 `20/20`，满足 Phase 5/6 “至少执行 8 个 goal，除非 raw frontier 很少且地图稳定”的要求。

注：`selected_frontier` 解析计数为 21，但最终 `completed_goals=20/20`。这是因为一次 frontier goal 被发送/取消或重复日志采样导致选择行多于最终成功计数；验收应以 `completed_goals=20/20` 和成功统计为准。

## 9. ROS 图与 /map 证据

live run 结束前采集 ROS graph：

```text
/map publisher count: 1
/map publisher: /slam_toolbox
/map subscriber: /frontier_explorer
frontier_explorer /map QoS: RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(10)
AMCL/map_server check: no /amcl, no map_server, no lifecycle_manager_localization nodes found
```

节点列表包含：

```text
/frontier_explorer
/slam_toolbox
/controller_server
/planner_server
/bt_navigator
/behavior_server
/global_costmap/global_costmap
/local_costmap/local_costmap
/ros_gz_bridge
```

这满足：

- `/map` 仍由 `slam_toolbox` 发布；
- AMCL/map_server 未启动；
- Nav2 与 frontier explorer 链路存在；
- 原 SLAM 在线建图链路未被静态地图定位替代。

## 10. 地图保存结果

生成文件：

```text
/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
```

文件大小：

```text
tugbot_nav_world_slam_phase6_cleanup.yaml 158 bytes
tugbot_nav_world_slam_phase6_cleanup.pgm 54755 bytes
```

YAML 内容：

```yaml
image: tugbot_nav_world_slam_phase6_cleanup.pgm
mode: trinary
resolution: 0.050
origin: [-8.900, -5.977, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

## 11. 地图统计与 Phase 4 / Phase 5 对比

PGM 像素统计采用 `map_saver_cli` trinary 输出约定：free=254、unknown=205、occupied=0。

| 阶段 | 尺寸 | free | unknown | occupied | unknown ratio | 说明 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| Phase 4 | 315x156 | 26024 | 21914 | 1202 | 约 0.446 | 来自 Phase 4 report post-map last |
| Phase 5 | 315x155 | 43182 | 3646 | 1997 | 0.0747 | Phase 5 保存地图 |
| Phase 6 | 322x170 | 45348 | 5857 | 3535 | 0.1070 | Phase 6 cleanup 保存地图 |

对比结论：

- Phase 6 相比 Phase 4：free cells 增加 `19324`，unknown cells 减少 `16057`，覆盖显著大于 Phase 4；
- Phase 6 相比 Phase 5：free cells 增加 `2166`，说明 Phase 6 拓展了已知可通行区域；
- Phase 6 的 unknown cells 比 Phase 5 多 `2211`，主要因为 Phase 6 SLAM 地图边界扩展到 322x170，引入了新的地图外缘 unknown；不能仅按 unknown 总数判断覆盖倒退；
- 从 live 内部统计看，Phase 6 unknown 从 `41601` 降到 `5857`，free 从 `6710` 增至 `45348`，cleanup 后继续扩大了可观测区域。

## 12. 与 Phase 4 问题的对应关系

Phase 4 的关键失败模式：

```text
frontier_clusters=66
valid_candidates=0
```

旧逻辑把 `valid_candidates=0` 误判为探索完成。

Phase 6 当前有效 run 的终止原因：

```text
frontier goal budget reached and cleanup goal budget exhausted
completed_goals=20
cleanup_goals_completed=5/5
```

也就是说，本次不是因为 `valid_candidates=0` 直接结束，而是在完成 20 个 frontier goals、5 个 cleanup goals 后保存地图。Phase 6 满足“不再在 frontier_clusters 很多但 valid_candidates=0 时立即结束”的验收重点。

## 13. 遗留问题

1. cleanup goal budget 已用尽后仍存在 residual cleanup candidates：

   ```text
   Cleanup candidates remain; postponing final map save: cleanup_candidate_count > 0 (2), cleanup_goals_started=5/5 unknown_ratio=0.1070 target_unknown_ratio=0.0300
   ```

   最终仍有：

   ```text
   unknown_total_count=5857
   unknown_ratio=0.1070
   unknown_cluster_count=5
   cleanup_candidate_count=2
   ```

   这说明 Phase 6 的轻量 cleanup 生效，但 `max_cleanup_goals=5` 限制下仍有 residual unknown。若人工验收希望继续压低右上角灰区，可在 Phase 7 增加 cleanup budget 或引入更细的 reachable precheck/局部补扫策略。

2. 一次 Nav2 planner failure 与一次 cleanup spin failure：

   - `planner failure count observed: 1`；
   - 一次 cleanup spin 返回 `status=6 error_code=703 success=False`；
   - 系统未崩溃，后续 goals 与 map save 均完成。

3. Phase 6 的 unknown ratio 未达到 `target_unknown_ratio=0.03`：

   - 当前终止原因是 `frontier goal budget reached and cleanup goal budget exhausted`；
   - 这是受本阶段 `max_goals=20` 与 `max_cleanup_goals=5` 验证参数约束的结果。

4. launch 在 map save 后仍会由外部停止：

   - 地图已成功保存；
   - 报告生成前已清理 live ROS/Gazebo/Nav2 进程；
   - 后续可考虑让 explorer 保存地图后主动触发 launch 级退出，但这不属于 Phase 6 验收重点。

## 14. Phase 6 验收结论

Phase 6 达成核心目标：

- 已实现 residual unknown cleanup mode；
- 已执行并验证 cleanup candidate generation；
- 已执行 5 个 cleanup goals；
- 已触发 cleanup spin，最终 4 次 spin 成功计入统计；
- completed frontier goals 达到 `20/20`，明显超过 `min_goals_before_finish=8`；
- `/map` 仍由 slam_toolbox 发布；
- AMCL/map_server 未启动；
- Phase 0/1/2/3/4 入口和原 `nav2_params.yaml` 未被破坏；
- 已保存 Phase 6 地图 `.yaml + .pgm`；
- Phase 6 相比 Phase 4 地图覆盖明显更大。

本阶段验收结论：

```text
PASS_WITH_NOTES
```

说明：Phase 6 可以进入人工验收。遗留 residual unknown 与 cleanup budget 相关，建议人工查看 `tugbot_nav_world_slam_phase6_cleanup.pgm` 后决定 Phase 7 是否继续做“更高 cleanup budget / planner action precheck / 定向右上角补扫”。

## 15. 当前停止状态

报告生成前已停止 live 验证进程，并确认 ROS graph 清空：

```text
NODES_AFTER_FINAL
<empty>
```

Phase 6 已停止，等待人工验收。
