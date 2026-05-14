# Phase 7A 人工验收结论报告

## 1. 结论摘要

Phase 7A cleanup budget 参数强化复跑已经完成，但不作为当前默认成果采用。

人工验收最终结论：

```text
Phase 7A 执行过程通过；
Phase 7A 不适合作为当前最佳地图；
当前最佳地图回退采用 Phase 6 输出；
不再继续 Phase 7B 探索优化；
下一步进入自建地图回放导航验证。
```

本报告用于记录 Phase 7A 的负面实验价值：单纯提高 `max_goals`、提高 `max_cleanup_goals`、扩大 cleanup 搜索半径并放宽 cleanup 障碍距离，并不必然改善地图质量；过度补扫可能导致 SLAM 重影、地图外缘 unknown 扩张和局部错位。

## 2. Phase 7A 执行结果摘要

Phase 7A 按参数强化复跑要求执行，未修改源码、launch、config 或 README，只运行现有 `tugbot_explore.launch.py`。

关键运行参数：

```text
max_goals=25
min_goals_before_finish=8
enable_cleanup_mode=true
max_cleanup_goals=12
cleanup_search_radius_max_m=3.0
cleanup_min_obstacle_distance_m=0.30
target_unknown_ratio=0.05
save_map=true
map_save_path=src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget
```

Phase 7A 执行层面结果：

```text
/map 仍由 slam_toolbox 发布：PASS
AMCL/map_server 未启动：PASS
cleanup goals 超过 Phase 6 的 5 个：PASS
地图文件保存成功：PASS
执行状态：PASS_FOR_RERUN_EXECUTION
```

Phase 7A 关键统计来自执行报告：

```text
selected_cleanup_goals=12
cleanup_goal_success=10
cleanup_goal_failed=2
cleanup_spin_started=10
cleanup_spin_finished=10
completed_goals=25/25
unknown_total_count=21019
unknown_ratio=0.2782
```

Phase 7A 执行报告位置：

```text
doc/doc_report/phase7a_cleanup_budget_rerun_report.md
```

## 3. 人工观察到的地图质量问题

人工验收观察到 Phase 7A 地图虽然覆盖范围扩大，但视觉质量明显下降，主要问题包括：

1. 明显重影；
2. 墙线重复；
3. 局部错层；
4. 扫描结果未对齐；
5. 地图外缘 unknown 明显扩张；
6. 整体视觉质量低于 Phase 6。

这些问题说明：Phase 7A 的“更多 goal + 更多 cleanup + 更激进候选点”虽然增加了运动和观测次数，但也增加了 SLAM 累积误差、局部匹配不稳定和地图边界扩张风险。对于后续静态地图导航，地图几何一致性和墙线清晰度比单纯覆盖范围更重要。

## 4. 与 Phase 6 的取舍判断

Phase 6 人工验收结论为 `PASS_WITH_NOTES`。它仍存在少量 residual unknown，但主体地图测绘质量更稳定，墙线和障碍物结构更适合后续静态地图导航回放。

Phase 6 与 Phase 7A 的数值对比：

| 阶段 | 地图文件 | 尺寸 | free | unknown | occupied | unknown_ratio | 人工质量判断 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| Phase 6 | `tugbot_nav_world_slam_phase6_cleanup.pgm` | 322x170 | 45348 | 5857 | 3535 | 0.1070 | 当前最佳，采用 |
| Phase 7A | `tugbot_nav_world_slam_phase7_budget.pgm` | 345x219 | 49094 | 21019 | 5442 | 0.2782 | 负面实验，不采用 |

取舍结论：

- Phase 7A free cells 比 Phase 6 更多，说明覆盖范围确有扩大；
- 但 Phase 7A unknown_total_count 与 unknown_ratio 也明显升高；
- 更关键的是，人工验收发现 Phase 7A 出现重影、墙线重复、局部错层和扫描未对齐；
- 因此不能只看 coverage 扩大，必须优先采用几何质量更稳定的 Phase 6 地图。

最终选择：

```text
当前最佳地图：Phase 6 cleanup 输出
Phase 7A 角色：负面实验记录，不作为默认地图或当前最佳成果
```

## 5. 当前推荐地图文件

推荐用于下一阶段静态地图导航回放验证的地图文件：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
```

绝对路径：

```text
/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
```

文件状态：

```text
tugbot_nav_world_slam_phase6_cleanup.yaml: 158 bytes
tugbot_nav_world_slam_phase6_cleanup.pgm: 54755 bytes
```

Phase 6 地图 YAML 内容：

```yaml
image: tugbot_nav_world_slam_phase6_cleanup.pgm
mode: trinary
resolution: 0.050
origin: [-8.900, -5.977, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

## 6. 不采用 Phase 7A 作为默认成果的原因

Phase 7A 不采用为默认成果，原因如下：

1. 地图视觉质量下降。

   人工验收已确认出现重影、墙线重复、局部错层和扫描未对齐。

2. unknown 指标没有优于 Phase 6。

   Phase 6：

   ```text
   unknown_total_count=5857
   unknown_ratio=0.1070
   ```

   Phase 7A：

   ```text
   unknown_total_count=21019
   unknown_ratio=0.2782
   ```

3. 过度补扫导致地图边界扩张。

   Phase 7A 地图尺寸从 Phase 6 的 `322x170` 扩大到 `345x219`，引入大量新的外缘 unknown。覆盖范围扩大不等于有效地图质量提升。

4. 更激进 cleanup 参数带来更高运行风险。

   Phase 7A 执行报告记录：

   ```text
   planner_failed_occurrences=35
   failed_to_make_progress_occurrences=8
   cleanup_goal_failed=2
   ```

   这说明放宽 cleanup 候选约束后，目标可达性和局部控制压力变差。

5. 对静态导航回放而言，稳定清晰的地图优先于更大的但有重影的地图。

   后续 AMCL / Nav2 静态地图导航更依赖墙线一致、障碍物边界清晰、局部几何稳定。Phase 7A 的重影和错层会增加定位与规划风险。

## 7. Phase 7A 负面实验记录价值

Phase 7A 的价值在于明确排除了一个低收益方向：

```text
单纯提高 max_goals；
单纯提高 max_cleanup_goals；
单纯扩大 cleanup_search_radius_max_m；
单纯放宽 cleanup_min_obstacle_distance_m；
并不必然改善最终地图质量。
```

本轮实验支持以下工程判断：

- cleanup budget 不是越大越好；
- 过度补扫可能引入 SLAM 重影和局部错位；
- 地图外缘 unknown 扩张会干扰全局 unknown 指标；
- 后续不应继续 Phase 7B 式的探索参数盲调；
- 应切换到使用 Phase 6 质量较稳定地图进行静态地图导航验证。

## 8. 下一步建议：使用 Phase 6 地图做静态地图导航回放验证

建议下一阶段进入“自建地图回放导航验证”。推荐以 Phase 6 地图作为静态地图输入，验证 Nav2 在已保存地图上的定位、规划和导航能力。

建议目标：

1. 使用 Phase 6 `.yaml/.pgm` 启动静态地图导航；
2. 验证 `/map` 由 map_server 发布，而不是 slam_toolbox；
3. 验证 AMCL 正常启动并发布定位相关 TF；
4. 验证 Nav2 lifecycle active；
5. 选择若干 Phase 6 地图内可达目标点做 NavigateToPose；
6. 记录成功/失败目标、路径生成、cmd_vel、最终位姿与地图匹配情况；
7. 若导航回放通过，再把 Phase 6 地图确认为当前阶段推荐交付地图。

建议使用的地图参数：

```text
map:=/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
```

## 9. 最终人工验收结论

```text
Phase 7A 执行过程：PASS
Phase 7A 地图质量：FAIL_FOR_DEFAULT_ADOPTION
当前推荐地图：Phase 6 cleanup map
Phase 7B：不继续
下一阶段：Phase 6 静态地图导航回放验证
```

Phase 7A 负面实验记录保留，Phase 6 地图作为当前最佳地图进入下一阶段。
