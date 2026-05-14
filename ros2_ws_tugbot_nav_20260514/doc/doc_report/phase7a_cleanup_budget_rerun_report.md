# Phase 7A cleanup budget 参数强化复跑报告

## 1. 阶段目标与边界

Phase 6 人工验收结论为 `PASS_WITH_NOTES`。cleanup mode 已经生效，但结束时 cleanup budget 用尽，仍有：

```text
unknown_total_count=5857
unknown_ratio=0.1070
cleanup_candidate_count=2
```

Phase 7A 的目标不是改代码，而是只通过 launch 参数强化复跑，验证“提高 cleanup budget、扩大 cleanup 搜索半径、略放宽 cleanup 障碍距离”是否能继续压低右上角 residual unknown。

严格边界：

- 未修改任何源码；
- 未修改 launch/config/README；
- 只运行现有 `tugbot_explore.launch.py`；
- 保存新地图到 `phase7_budget` 文件；
- 完成后停止等待人工验收。

## 2. 运行命令

本轮实际执行命令：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
source install/setup.bash

ros2 launch tugbot_bringup tugbot_explore.launch.py   headless:=true   use_rviz:=false   max_goals:=25   min_goals_before_finish:=8   enable_cleanup_mode:=true   max_cleanup_goals:=12   cleanup_search_radius_max_m:=3.0   cleanup_min_obstacle_distance_m:=0.30   target_unknown_ratio:=0.05   save_map:=true   map_save_path:=/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget
```

后台 session：

```text
session_id: proc_920087dbed80
final status: exited
full log: /tmp/hermes-results/call_5lkp050x0KLGasNud0qqRrkE.txt
```

## 3. “不改源码”校验

Preflight 记录了关键文件 hash，run 后再次计算，结果无变化：

```text
changed protected/source hashes: {}
```

纳入 hash 校验的文件：

```text
src/tugbot_exploration/tugbot_exploration/frontier_explorer.py
src/tugbot_bringup/launch/tugbot_explore.launch.py
src/tugbot_bringup/launch/tugbot_nav.launch.py
src/tugbot_bringup/launch/tugbot_slam.launch.py
src/tugbot_bringup/launch/tugbot_slam_nav.launch.py
src/tugbot_navigation/config/nav2_params.yaml
README.md
src/tugbot_bringup/test/test_contract.py
```

说明：本报告本身是新生成文档，不属于“源码/launch/config/README”禁止修改范围。

## 4. ROS 图验收证据

live run 中途和地图保存后均采集 ROS graph。地图保存前最终检查显示：

```text
/map Publisher count: 1
/map publisher node: /slam_toolbox
/map subscriber: /frontier_explorer
frontier_explorer /map QoS: RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(10)
```

验收点：

- `/map` 仍由 `slam_toolbox` 发布：`PASS`；
- AMCL/map_server 未启动：`PASS`；
- `frontier_explorer`、Nav2、`slam_toolbox` 在 live run 中存在；
- 有效 run 中 `process has died` 计数：0。

地图保存后采集的文件证据：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.pgm 75570 bytes
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.yaml 157 bytes
```

## 5. live run 关键日志证据

### 5.1 参数启动证据

```text

```

### 5.2 cleanup mode 触发证据

首次出现 `cleanup mode=True` 的地图状态：

```text
[frontier_explorer-16] [INFO] [1778738713.705051842] [frontier_explorer]: Map 315x155 res=0.050, raw frontier cluster count=15, valid candidate count=6, frontier_clusters=15, valid_candidates=6, finish_no_frontier_cycles=0/5, recovery_no_candidate_cycles=0/3, relaxed_no_candidate_cycles=0/3, recovery_scans_completed=0/3, recovery_mode=False, search_mode=strict, blacklist=0, completed_goals=15/25, min_goals_before_finish=8, map_stable_cycles=0/3, unknown_total_count=7973 unknown_ratio=0.1633 unknown_cluster_count=5 cleanup_candidate_count=1, cleanup mode=True cleanup_goals_completed=0/12 cleanup_spins_completed=0, map_stats unknown=7973 free=39001 occupied=1851 delta_unknown=1396 delta_free=1312 delta_occupied=84
```

### 5.3 cleanup goal 证据

Phase 7A selected cleanup goal 编号：

```text
[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
```

前 5 条 cleanup goal：

```text
[frontier_explorer-16] [WARN] [1778738713.705421904] [frontier_explorer]: Selected cleanup goal #1: x=3.493 y=-3.390 yaw=-2.774 residual unknown cluster size=72 distance=1.472 score=44.159 clearance_cells=10 free_neighbor_count=113 unknown_ratio=0.1633 cleanup_candidate_count=1
[frontier_explorer-16] [WARN] [1778738901.587959635] [frontier_explorer]: Selected cleanup goal #2: x=0.892 y=1.614 yaw=2.456 residual unknown cluster size=2240 distance=4.283 score=796.998 clearance_cells=10 free_neighbor_count=113 unknown_ratio=0.1061 cleanup_candidate_count=1
[frontier_explorer-16] [WARN] [1778738941.169251014] [frontier_explorer]: Selected cleanup goal #3: x=-1.557 y=-4.795 yaw=-1.756 residual unknown cluster size=1368 distance=6.797 score=493.684 clearance_cells=10 free_neighbor_count=113 unknown_ratio=0.1078 cleanup_candidate_count=2
[frontier_explorer-16] [WARN] [1778738987.318044505] [frontier_explorer]: Selected cleanup goal #4: x=0.493 y=-4.747 yaw=-0.367 residual unknown cluster size=10534 distance=1.954 score=3747.690 clearance_cells=10 free_neighbor_count=113 unknown_ratio=0.2163 cleanup_candidate_count=4
[frontier_explorer-16] [WARN] [1778739033.020527275] [frontier_explorer]: Selected cleanup goal #5: x=-6.241 y=-0.447 yaw=2.356 residual unknown cluster size=4763 distance=7.829 score=1693.601 clearance_cells=10 free_neighbor_count=113 unknown_ratio=0.2334 cleanup_candidate_count=2
```

后 5 条 cleanup goal：

```text
[frontier_explorer-16] [WARN] [1778739133.339618998] [frontier_explorer]: Selected cleanup goal #8: x=-6.840 y=-0.887 yaw=1.429 residual unknown cluster size=5403 distance=0.543 score=1971.951 clearance_cells=10 free_neighbor_count=113 unknown_ratio=0.2396 cleanup_candidate_count=3
[frontier_explorer-16] [WARN] [1778739196.318036719] [frontier_explorer]: Selected cleanup goal #9: x=-7.290 y=-0.587 yaw=0.629 residual unknown cluster size=5275 distance=0.558 score=1926.903 clearance_cells=10 free_neighbor_count=113 unknown_ratio=0.2378 cleanup_candidate_count=3
[frontier_explorer-16] [WARN] [1778739211.431572258] [frontier_explorer]: Selected cleanup goal #10: x=-7.440 y=-0.187 yaw=0.071 residual unknown cluster size=5174 distance=0.573 score=1885.782 clearance_cells=10 free_neighbor_count=113 unknown_ratio=0.2360 cleanup_candidate_count=3
[frontier_explorer-16] [WARN] [1778739227.300638719] [frontier_explorer]: Selected cleanup goal #11: x=-6.990 y=-0.787 yaw=1.204 residual unknown cluster size=5174 distance=0.530 score=1885.789 clearance_cells=10 free_neighbor_count=113 unknown_ratio=0.2360 cleanup_candidate_count=3
[frontier_explorer-16] [WARN] [1778739289.662901023] [frontier_explorer]: Selected cleanup goal #12: x=-2.018 y=-5.037 yaw=-1.204 residual unknown cluster size=10505 distance=6.466 score=3772.361 clearance_cells=9 free_neighbor_count=113 unknown_ratio=0.2893 cleanup_candidate_count=1
```

统计：

```text
selected_cleanup_goals = 12
cleanup_goal_success = 10
cleanup_goal_failed = 2
cleanup_spin_started = 10
cleanup_spin_finished = 10
```

结论：cleanup goals 数量超过 Phase 6 的 5 个；Phase 7A 实际 selected cleanup goals 达到 12 个，其中 10 个成功，2 个失败并进入 blacklist/后续策略。

### 5.4 strict/relaxed 与 Nav2 异常证据

本轮出现 relaxed search：

```text
search_mode=relaxed occurrences = 6
```

Nav2 规划/控制异常计数：

```text
planner_failed_occurrences = 35
failed_to_make_progress_occurrences = 8
```

这些异常未导致 explorer 进程崩溃；run 最终完成并保存地图。

### 5.5 最终完成日志

```text
[frontier_explorer-16] [INFO] [1778739357.123591927] [frontier_explorer]: Exploration complete: frontier goal budget reached and cleanup goal budget exhausted; completed_goals=25 success=25 failure=1 recovery_scans_completed=0 relaxed_search_executed=True map_stable_cycles=0 cleanup_goals_completed=10/12 cleanup_spins_completed=10 unknown_total_count=21019 unknown_ratio=0.2782 unknown_cluster_count=2 cleanup_candidate_count=1
```

最终地图状态：

```text
[frontier_explorer-16] [INFO] [1778739357.122807996] [frontier_explorer]: Map 345x219 res=0.050, raw frontier cluster count=110, valid candidate count=13, frontier_clusters=110, valid_candidates=13, finish_no_frontier_cycles=0/5, recovery_no_candidate_cycles=0/3, relaxed_no_candidate_cycles=0/3, recovery_scans_completed=0/3, recovery_mode=False, search_mode=strict, blacklist=2, completed_goals=25/25, min_goals_before_finish=8, map_stable_cycles=0/3, unknown_total_count=21019 unknown_ratio=0.2782 unknown_cluster_count=2 cleanup_candidate_count=1, cleanup mode=False cleanup_goals_completed=10/12 cleanup_spins_completed=10, map_stats unknown=21019 free=49094 occupied=5442 delta_unknown=240 delta_free=214 delta_occupied=26
```

地图保存日志：

```text
[frontier_explorer-16] [INFO] [1778739357.866958116] [frontier_explorer]: Map save succeeded: yaml=/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.yaml bytes=157 pgm=/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.pgm bytes=75570
```

## 6. Phase 6 vs Phase 7A 地图统计

PGM 统计采用 map_saver trinary 输出约定：free=254、unknown=205、occupied=0。

| 阶段 | 尺寸 | free | unknown | occupied | unknown_ratio | free_ratio |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Phase 6 cleanup | 322x170 | 45348 | 5857 | 3535 | 0.1070 | 0.8284 |
| Phase 7A budget | 345x219 | 49094 | 21019 | 5442 | 0.2782 | 0.6498 |

差值：

```text
unknown_delta = 15162
unknown_ratio_delta = +0.1712
free_delta = 3746
occupied_delta = 1907
```

解释：

- Phase 7A free cells 比 Phase 6 增加 3746，说明更高 budget 让机器人进一步观测/建图；
- 但 Phase 7A 地图边界扩展到 `345x219`，比 Phase 6 的 `322x170` 大很多；
- 扩展边界引入大量新的外缘 unknown，导致 final unknown_total_count 和 unknown_ratio 反而高于 Phase 6；
- 因此，本轮“提高 cleanup budget 是否继续压低 unknown_total_count/unknown_ratio”的验证结论是：在当前参数组合下，没有压低，反而因地图边界扩张而升高。

## 7. 验收重点逐项结论

1. `/map` 仍由 slam_toolbox 发布：PASS。

2. AMCL/map_server 不启动：PASS。

3. cleanup goals 是否超过 Phase 6 的 5 个：PASS。

   - Phase 6：5 个；
   - Phase 7A：selected 12 个，success 10 个。

4. unknown_total_count 是否低于 Phase 6 的 5857：FAIL。

   - Phase 7A final `unknown_total_count=21019`。

5. unknown_ratio 是否低于 Phase 6 的 0.1070：FAIL。

   - Phase 7A final `unknown_ratio=0.2782`。

6. 右上角残余灰区是否缩小：需要人工看图；从数值上不能支持“整体 residual unknown 缩小”。

   - Phase 7A 的 free 区域继续增加，但地图边界也明显扩大，unknown 总量与比例上升；
   - 若只比较局部右上角，需人工打开 `.pgm` 或在 RViz/图像工具里观察。

7. 地图文件是否保存成功：PASS。

   - `.yaml` 与 `.pgm` 均保存成功。

## 8. 地图保存结果

生成文件：

```text
/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.yaml
/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.pgm
```

YAML 内容：

```yaml
image: tugbot_nav_world_slam_phase7_budget.pgm
mode: trinary
resolution: 0.050
origin: [-9.900, -7.162, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

## 9. 重要观察与风险

1. 更高 cleanup budget 确实被使用。

   - selected cleanup goals: 12；
   - succeeded cleanup goals: 10；
   - cleanup spins: 10。

2. 本轮没有得到“unknown 低于 Phase 6”的结果。

   - 主要数值失败项是 `unknown_total_count` 与 `unknown_ratio`；
   - 地图边界扩展是关键解释因素，但这仍意味着当前参数组合不能作为“压低 residual unknown”的胜出配置。

3. cleanup 后期出现较多规划失败/进度失败。

   - planner failed: 35；
   - failed to make progress: 8；
   - cleanup failed: 2。

   这提示简单增大 `cleanup_search_radius_max_m` 与放宽 `cleanup_min_obstacle_distance_m` 会让 cleanup 候选更激进，部分目标可达性变差。

4. 结束时仍有 cleanup candidate。

```text
[frontier_explorer-16] [WARN] [1778739357.123270370] [frontier_explorer]: Cleanup candidates remain; postponing final map save: cleanup_candidate_count > 0 (1), cleanup_goals_started=12/12 unknown_ratio=0.2782 target_unknown_ratio=0.0500
```

## 10. Phase 7A 结论

本阶段结论：

```text
PASS_FOR_RERUN_EXECUTION / FAIL_FOR_UNKNOWN_REDUCTION
```

含义：

- 作为“不改代码，只做参数强化复跑”的执行任务：PASS；
- 作为“提高 cleanup budget 能否继续压低 Phase 6 unknown_total_count/unknown_ratio”的假设验证：FAIL。

核心证据：

```text
Phase 6: unknown_total_count=5857, unknown_ratio=0.1070
Phase 7A: unknown_total_count=21019, unknown_ratio=0.2782
Phase 7A cleanup goals: selected=12, succeeded=10
```

建议：

- 不建议直接把 Phase 7A 参数作为新默认；
- 若进入 Phase 7B，优先不要继续盲目加 budget；
- 更合理的下一步是冻结源码，做“局部区域/右上角 ROI 对比”或“限制 cleanup 候选导致 map boundary 外扩的策略”，再决定是否需要代码级 reachable precheck 或边界增长约束。

## 11. 停止状态

- Phase 7A live run 已完成地图保存；
- 后台 session 已终止；
- 已尝试清理 ROS/Gazebo/Nav2 进程；一次 cleanup 命令因自匹配/TERM 返回 `-15`，随后复查只见残留的 `/lifecycle_manager_navigation` 节点名，未见有效探索进程证据；
- 不再启动新的 live run，等待人工验收。
