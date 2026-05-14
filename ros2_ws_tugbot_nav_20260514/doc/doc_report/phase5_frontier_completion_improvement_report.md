# Phase 5 Report - frontier 完成条件与候选生成策略改良验收报告

## 1. 阶段目标

Phase 4 人工验收发现：小车探索过的区域测绘质量可接受，但探索在地图只覆盖部分 Gazebo 环境时提前结束。Phase 4 日志中的关键症状是：

```text
frontier_clusters=66
valid_candidates=0
Exploration complete: no valid frontier remained
```

这说明旧版 `frontier_explorer` 把“仍有 raw frontier，但当前严格过滤下没有 valid navigation candidate”误判为“探索完成”。Phase 5 的目标是修正这个 premature finish 边界：

- 区分 raw frontier cluster 与 valid candidate；
- 在 `frontier_clusters > 0` 但 `valid_candidates == 0` 时进入 relaxed search / recovery exploration，而不是直接结束；
- 加强完成条件：最少 goal 数、raw frontier 数、地图稳定性、relaxed/recovery 尝试状态共同决定是否结束；
- 保持 SLAM 精度、Nav2 链路、地图保存功能不被破坏；
- 不修改 0513 工程，不破坏 Phase 0/1/2/3/4 入口，不修改原 `nav2_params.yaml`。

## 2. 修改文件清单

本阶段修改以下文件：

1. `src/tugbot_exploration/tugbot_exploration/frontier_explorer.py`

   主要改动：

   - 区分 raw frontier clusters 与 valid navigation candidates；
   - 新增 strict / relaxed 两套候选搜索参数；
   - 新增更严格的 exploration completion gate；
   - 新增 recovery scan 参数与执行路径；
   - 改良 frontier goal 生成逻辑：不再只依赖 cluster centroid，而是在 frontier cluster 附近搜索安全 free cell；
   - 新增 map stats / delta 统计与日志；
   - 新增 goal success/failure、blacklist、search mode、recovery mode 统计日志；
   - 将 `/map` subscription QoS 调整为 `RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(10)`，避免 late subscriber 错过 slam_toolbox latched map；
   - 优化候选点搜索性能，限制每个 representative 的搜索 cell 数量。

2. `src/tugbot_bringup/launch/tugbot_explore.launch.py`

   主要改动：

   - `max_goals` 默认从 `10` 提高到 `20`；
   - 新增并传递 Phase 5 参数：relaxed search、completion gate、map stability、recovery scan；
   - 保留原 `tugbot_explore.launch.py` 启动能力；
   - 未破坏 `tugbot_nav.launch.py`、`tugbot_slam.launch.py`、`tugbot_slam_nav.launch.py`。

3. `src/tugbot_bringup/test/test_contract.py`

   主要改动：

   - Contract tests 从 7 个扩展为 8 个；
   - 更新 `max_goals` 默认值期望为 `20`；
   - 增加 Phase 5 参数、日志字段、completion gate 相关契约检查。

4. `README.md`

   - 已检查；本阶段未将 README 作为主要改动文件。
   - Phase 5 的权威验收记录以本报告为准。

本阶段未修改：

```text
src/tugbot_bringup/launch/tugbot_nav.launch.py
src/tugbot_bringup/launch/tugbot_slam.launch.py
src/tugbot_bringup/launch/tugbot_slam_nav.launch.py
src/tugbot_navigation/config/nav2_params.yaml
```

工作目录不是 git 仓库，无法用 `git diff/status` 作为证据：

```text
fatal: not a git repository (or any of the parent directories): .git
```

## 3. 新增参数清单

新增 / 强化的 frontier explorer 参数：

```text
min_goals_before_finish              默认 8
min_frontier_clusters_before_finish  默认 5
recovery_scan_attempts               默认 3
relaxed_retry_attempts               默认 3
map_stable_cycles_before_finish      默认 3
map_change_free_threshold            默认 50
map_change_unknown_threshold         默认 50

relaxed_min_obstacle_distance_m      默认 0.35
relaxed_max_goal_distance_m          默认 8.0
relaxed_min_cluster_size             默认 3

enable_recovery_scan                 默认 true
recovery_spin_angle                  默认 6.28
recovery_wait_after_scan_sec         默认 2.0
```

保留 / 调整的关键默认参数：

```text
max_goals                            默认 20
finish_no_frontier_cycles            默认 5，但不再能单独决定完成
min_obstacle_distance_m              默认 0.45
max_goal_distance_m                  默认 4.0
min_cluster_size                     默认 5
save_map                             默认 false
```

Phase 5 live run 启动日志确认参数生效：

```text
min_obstacle_distance_m=0.45
min_goal_distance_m=0.50
max_goal_distance_m=4.00
relaxed_min_obstacle_distance_m=0.35
relaxed_max_goal_distance_m=8.00
min_cluster_size=5
relaxed_min_cluster_size=3
max_goals=20
min_goals_before_finish=8
min_frontier_clusters_before_finish=5
recovery_scan_attempts=3
relaxed_retry_attempts=3
map_stable_cycles_before_finish=3
map_change_free_threshold=50
map_change_unknown_threshold=50
enable_recovery_scan=True
recovery_spin_angle=6.28
recovery_wait_after_scan_sec=2.0
save_map=True
```

## 4. 完成条件新旧对比

### Phase 4 旧逻辑

旧逻辑的核心风险：

```text
valid_candidates == 0 连续达到 finish_no_frontier_cycles
=> Exploration complete
```

这会把“有 raw frontier，但没有通过严格过滤的导航候选点”错误视为“探索完成”。Phase 4 的人工验收正是这个问题：

```text
frontier_clusters=66
valid_candidates=0
No valid frontier candidate (5/5 cycles)
Exploration complete: no valid frontier remained
```

### Phase 5 新逻辑

Phase 5 不允许仅凭 `valid_candidates == 0` 结束。新的完成判断需要同时考虑：

1. `completed_goals >= min_goals_before_finish`；
2. `valid_candidates == 0`；
3. `frontier_clusters <= min_frontier_clusters_before_finish`，或者 recovery scan 已经尝试到上限后仍无可达目标；
4. 地图 unknown/free/occupied 统计连续若干轮变化很小；
5. 已经执行过 relaxed candidate search；
6. 已经执行过 recovery scan 或 recovery scan 条件不适用；
7. `finish_no_frontier_cycles` 只作为 no-frontier 稳定计数之一，不能单独宣布完成。

日志现在会同时输出：

```text
raw frontier cluster count
valid candidate count
frontier_clusters
valid_candidates
finish_no_frontier_cycles
recovery_no_candidate_cycles
relaxed_no_candidate_cycles
recovery_scans_completed
recovery_mode
search_mode
map_stable_cycles
map_stats unknown/free/occupied
map delta_unknown/delta_free/delta_occupied
```

## 5. 候选点生成策略改良

Phase 5 不再只使用 cluster centroid 作为目标点。候选 goal 生成策略改为：

1. 对每个 frontier cluster 取若干 representative points；
2. 在 representative 附近 0.3m 到 1.0m 范围内搜索 known free cell；
3. 目标点必须不在 unknown；
4. 目标点附近需要有足够 free cells，作为 lightweight reachability / safety precheck；
5. 目标点需要满足 obstacle clearance；
6. 如果 centroid 不安全，继续尝试 cluster 边缘点以及向机器人方向回退的点；
7. 每个 cluster 最多生成若干候选 goal；
8. 最后进行距离过滤、障碍距离过滤、blacklist 过滤和评分排序。

本阶段未实现 Nav2 `ComputePathToPose` action 级预检查。原因是本阶段优先修复 premature finish，且用户允许先采用 lightweight precheck；直接接入 planner action 会引入更复杂的 action lifecycle、超时、Nav2 planner 状态与候选批量检查成本，适合在 Phase 6 单独做。

当前已实现的 lightweight precheck：

- 候选点必须是 known free cell；
- 候选点附近必须有足够 free cells；
- 候选点必须满足 obstacle clearance；
- blacklist 半径保持较小，避免过早屏蔽大范围 frontier。

## 6. strict search 与 relaxed search 证据

Phase 5 live run 中同时出现 strict 与 relaxed search，证明 relaxed candidate search 已实际参与探索。

Strict search 初始证据：

```text
Map 315x154 res=0.050, raw frontier cluster count=266, valid candidate count=69, frontier_clusters=266, valid_candidates=69, finish_no_frontier_cycles=0/5, recovery_no_candidate_cycles=0/3, relaxed_no_candidate_cycles=0/3, recovery_scans_completed=0/3, recovery_mode=False, search_mode=strict, blacklist=0, completed_goals=0/20, min_goals_before_finish=8, map_stable_cycles=0/3, map_stats unknown=41601 free=6710 occupied=199 delta_unknown=0 delta_free=0 delta_occupied=0
```

中段 strict search 证据：

```text
Map 315x155 res=0.050, raw frontier cluster count=85, valid candidate count=30, frontier_clusters=85, valid_candidates=30, finish_no_frontier_cycles=0/5, recovery_no_candidate_cycles=0/3, relaxed_no_candidate_cycles=0/3, recovery_scans_completed=0/3, recovery_mode=False, search_mode=strict, blacklist=0, completed_goals=2/20, min_goals_before_finish=8, map_stable_cycles=0/3, map_stats unknown=30871 free=17347 occupied=607 delta_unknown=1953 delta_free=1865 delta_occupied=88
```

Relaxed search 证据 1：

```text
Map 315x156 res=0.050, raw frontier cluster count=18, valid candidate count=6, frontier_clusters=18, valid_candidates=6, finish_no_frontier_cycles=0/5, recovery_no_candidate_cycles=0/3, relaxed_no_candidate_cycles=0/3, recovery_scans_completed=0/3, recovery_mode=False, search_mode=relaxed, blacklist=2, completed_goals=13/20, min_goals_before_finish=8, map_stable_cycles=1/3, map_stats unknown=9333 free=38058 occupied=1749 delta_unknown=5 delta_free=3 delta_occupied=2
Selected frontier goal #14: x=-2.781 y=0.733 cluster_size=26 distance=5.221 score=26.446 search_mode=relaxed clearance_cells=11 free_neighbor_count=81
```

Relaxed search 证据 2：

```text
Map 316x156 res=0.050, raw frontier cluster count=15, valid candidate count=18, frontier_clusters=15, valid_candidates=18, finish_no_frontier_cycles=0/5, recovery_no_candidate_cycles=0/3, relaxed_no_candidate_cycles=0/3, recovery_scans_completed=0/3, recovery_mode=False, search_mode=relaxed, blacklist=2, completed_goals=15/20, min_goals_before_finish=8, map_stable_cycles=1/3, map_stats unknown=9395 free=38049 occupied=1852 delta_unknown=11 delta_free=15 delta_occupied=26
Selected frontier goal #16: x=2.372 y=-1.016 cluster_size=274 distance=4.248 score=276.383 search_mode=relaxed clearance_cells=11 free_neighbor_count=81
```

结论：strict search 仍作为默认路径；当局部候选更难生成时，relaxed search 能继续提供 goal，未发生 Phase 4 那种“有 frontier 但候选暂时为 0 就结束”的单点误判。

## 7. recovery scan 触发情况

本次 Phase 5 live run 中：

```text
recovery_scans_completed=0/3
recovery_mode=False
```

最终完成日志：

```text
Exploration complete: reached max_goals=20; completed_goals=20 success=20 failure=4 recovery_scans_completed=0 relaxed_search_executed=True map_stable_cycles=0
```

解释：本次验证过程中 strict/relaxed search 始终能够生成 valid candidate；没有进入“`valid_candidates == 0` 且 `frontier_clusters > min_frontier_clusters_before_finish`”的 recovery scan 触发条件。因此 recovery scan 代码路径已实现并保留参数，但本次 live 证据显示它未被需要。

验收重点中的第 3 条“如果出现 `valid_candidates=0` 但 `frontier_clusters>5`，应触发 relaxed search 或 recovery scan”：本次 run 未出现该 exact 状态；但 relaxed search 已被实际执行，且完成条件已禁止仅凭 `valid_candidates=0` 结束。

## 8. 长时间探索验证命令

构建与验证路径：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514
colcon build --symlink-install
source install/setup.bash
```

Phase 5 live 验证命令：

```bash
ros2 launch tugbot_bringup tugbot_explore.launch.py \
  headless:=true \
  use_rviz:=false \
  max_goals:=20 \
  min_goals_before_finish:=8 \
  save_map:=true \
  map_save_path:=/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5
```

完整 live run 日志保存在：

```text
/tmp/hermes-results/call_YYM8SnmIbtwMfMadYghWF5ZA.txt
```

ROS launch log 目录：

```text
/home/hyh/.ros/log/2026-05-14-10-37-32-391320-hyh-VMware-Virtual-Platform-9850
```

## 9. 静态验证结果

### py_compile

```bash
python3 -m py_compile src/tugbot_exploration/tugbot_exploration/frontier_explorer.py
python3 -m py_compile src/tugbot_bringup/launch/tugbot_explore.launch.py
```

结果：通过。

### pytest contract

```bash
python3 -m pytest -q src/tugbot_bringup/test/test_contract.py
```

结果：通过。

```text
8 passed in 0.01s
```

### colcon build

```bash
colcon build --symlink-install
```

结果：通过。

```text
Summary: 5 packages finished
```

## 10. ROS 图与关键节点证据

Phase 5 live 验证中，`/frontier_explorer` 节点存在并订阅 / 调用预期接口：

```text
/frontier_explorer subscribers:
  /clock: rosgraph_msgs/msg/Clock
  /map: nav_msgs/msg/OccupancyGrid
  /tf: tf2_msgs/msg/TFMessage
  /tf_static: tf2_msgs/msg/TFMessage

/frontier_explorer action clients:
  /navigate_to_pose: nav2_msgs/action/NavigateToPose
  /spin: nav2_msgs/action/Spin
```

`/map` subscription QoS 已调整为：

```text
Reliability: RELIABLE
History (Depth): KEEP_LAST (10)
Durability: TRANSIENT_LOCAL
```

这与 slam_toolbox 的 transient local `/map` 发布方式匹配。

Phase 5 仍保持：

- `/map` 由 `slam_toolbox` 发布；
- 不启动 AMCL；
- 不启动静态 map_server；
- Nav2 的 `/navigate_to_pose` action 链路可用。

## 11. goal 执行数量与成功/失败统计

Phase 5 live run 成功达到 `max_goals=20`，而不是因为 no-candidate / no-frontier 提前结束。

关键日志：

```text
Goal succeeded; completed_goals=1/20
Goal succeeded; completed_goals=2/20
Goal succeeded; completed_goals=3/20
...
Goal succeeded; completed_goals=18/20
Goal succeeded; completed_goals=19/20
Goal succeeded; completed_goals=20/20
```

最终完成日志：

```text
Exploration complete: reached max_goals=20; completed_goals=20 success=20 failure=4 recovery_scans_completed=0 relaxed_search_executed=True map_stable_cycles=0
```

本次日志中可直接检索到的 timeout blacklist 事件：

```text
Goal failed (timeout); blacklisting x=-1.051 y=-1.079 radius=0.50
Goal failed (timeout); blacklisting x=-2.831 y=1.233 radius=0.50
```

说明：live log 的最终 summary 统计 `failure=4`，其中包含 action result failure / timeout 等内部失败计数；可见文本中明确的 timeout blacklist 为 2 次。无论失败计数按哪一层统计，explorer 均没有停止，而是清理/blacklist 后继续探索，最终完成 20 个成功 goal。

## 12. raw frontier cluster 与 valid candidate 变化

Phase 5 live run 中，raw frontier 与 valid candidate 变化如下：

```text
初始：Map 315x154, frontier_clusters=266, valid_candidates=69, completed_goals=0/20, search_mode=strict
中段：Map 315x155, frontier_clusters=85,  valid_candidates=30, completed_goals=2/20, search_mode=strict
中段：Map 315x155, frontier_clusters=32,  valid_candidates=33, completed_goals=10/20, search_mode=strict
中段：Map 315x156, frontier_clusters=18,  valid_candidates=6,  completed_goals=13/20, search_mode=relaxed
中段：Map 316x156, frontier_clusters=15,  valid_candidates=18, completed_goals=15/20, search_mode=relaxed
末段：Map 315x155, frontier_clusters=11,  valid_candidates=3,  completed_goals=19/20, search_mode=strict
```

统计范围：

```text
frontier state log count: 22
frontier_clusters: min=11, max=266
valid_candidates:  min=3,  max=69
relaxed search states: 2
```

与 Phase 4 的关键差异：

- Phase 4 在 `completed_goals=5/8` 且 `frontier_clusters=66, valid_candidates=0` 时结束；
- Phase 5 至少完成了 20 个 successful goals；
- Phase 5 的终止原因是 `reached max_goals=20`，不是 valid candidate 暂时为空。

## 13. 地图初始与结束统计

Phase 5 live run 中 frontier_explorer 日志记录的地图统计：

初始：

```text
unknown=41601 free=6710 occupied=199
```

末段：

```text
unknown=7239 free=39738 occupied=1848
```

变化：

```text
unknown 减少约 34362 cells
free 增加约 33028 cells
occupied 增加约 1649 cells
```

保存出的 Phase 5 PGM 文件像素统计：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5.pgm
size: 315 x 155
bytes: 48840
pixel counts: {0: 1997, 205: 3646, 254: 43182}
```

手动补充保存地图与自动保存地图一致：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5_manual.pgm
size: 315 x 155
bytes: 48840
pixel counts: {0: 1997, 205: 3646, 254: 43182}
```

## 14. 与 Phase 4 保存地图覆盖对比

Phase 4 报告中的 post-map 统计：

```text
unknown=21914
free=26024
occupied=1202
```

当前 Phase 4 保存地图文件统计：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam.pgm
size: 315 x 155
pixel counts: {0: 1286, 205: 21721, 254: 25818}
```

Phase 5 保存地图文件统计：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5.pgm
size: 315 x 155
pixel counts: {0: 1997, 205: 3646, 254: 43182}
```

按照 map_saver 的 trinary 输出习惯，这里可读为：

- `205` 对应 unknown；
- `254` 对应 free；
- `0` 对应 occupied。

因此 Phase 5 相比 Phase 4：

```text
unknown: 约 21721 -> 3646，显著减少
free:    约 25818 -> 43182，显著增加
occupied:约 1286  -> 1997，环境结构覆盖也增加
```

结论：Phase 5 保存地图覆盖明显大于 Phase 4。它不只是多跑了几个 goal，而是实质性扩大了 SLAM map 中的 known free 区域。

## 15. 地图保存结果

自动保存文件：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5.yaml  150 bytes
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5.pgm   48840 bytes
```

YAML 内容：

```yaml
image: tugbot_nav_world_slam_phase5.pgm
mode: trinary
resolution: 0.050
origin: [-8.692, -5.472, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

frontier_explorer 自动保存日志：

```text
Saving map with command: ros2 run nav2_map_server map_saver_cli -f /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5
Map save returncode=0
Map save succeeded: yaml=/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5.yaml bytes=150 pgm=/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5.pgm bytes=48840
```

手动补充保存命令也通过：

```bash
ros2 run nav2_map_server map_saver_cli -f /home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5_manual
```

手动保存输出：

```text
Received a 315 X 155 map @ 0.05 m/pix
Map saved successfully
```

手动补充保存文件：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5_manual.yaml  157 bytes
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5_manual.pgm   48840 bytes
```

## 16. 文件 hash / 输出物记录

```text
b963e3b23e2479e3c124be3d8ccb1543f99113d56a9203b81a42547eb7e46d11  src/tugbot_exploration/tugbot_exploration/frontier_explorer.py
41f78923e8d4a2db4b4596cd39e6df22bbf81f9df9f6df1fd19524736568c298  src/tugbot_bringup/launch/tugbot_explore.launch.py
09f156a4e5e9ef9247d75bc9343de8fe6d99de0ea77688c98113e535f156ebe2  src/tugbot_bringup/test/test_contract.py
ecd462fd1c877193e62f0c5b3d1e447b3ccf3e79c56e1ac5516e2b7e79fc4185  src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5.yaml
96dc0a383f94a45e06e9088c13c4b69102d31237d5102de4809901b35addc315  src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5.pgm
081076ae7b13ecbb19b4355b9dec47cdb2d6449af6b6d725790156174e60ed96  src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5_manual.yaml
96dc0a383f94a45e06e9088c13c4b69102d31237d5102de4809901b35addc315  src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5_manual.pgm
```

## 17. 当前遗留问题

1. 本阶段未实现 Nav2 `ComputePathToPose` action 级候选预检查。

   当前采用 lightweight precheck：候选点必须位于 known free、附近 free cells 足够、满足 obstacle clearance，并通过 blacklist/distance 过滤。ComputePathToPose 建议作为 Phase 6 单独接入。

2. recovery scan 本次未触发。

   原因不是功能缺失，而是本次 strict/relaxed search 一直能够找到 valid candidate。后续若构造 `valid_candidates=0` 且 `frontier_clusters>5` 的场景，应进一步专门验证 Spin / Navigate yaw recovery 行为。

3. Nav2 偶发 timeout / cancellation / controller progress warning。

   观察到过：

   ```text
   Failed to make progress
   Planner loop missed its desired rate
   NavigateToPose result: status=5 ... success=False
   ```

   但 explorer 能 blacklist / 清理后继续探索，最终达到 20 个 successful goals。本阶段不把这视为 frontier completion blocker。

4. 自动保存地图后 launch/session 未自动退出。

   地图已成功保存，进程已手动清理。后续可考虑在 `frontier_explorer` 完成保存后增加更明确的 shutdown/exit 策略。

5. README 仍主要记录 Phase 4/通用启动说明。

   本阶段权威说明已写入本报告。若 Phase 5 通过人工验收，建议再把 README 的默认 `max_goals`、Phase 5 参数和长时间探索命令同步更新。

## 18. 验收标准逐项结论

1. 不再在 `frontier_clusters` 很多但 `valid_candidates=0` 时立即结束。

   结论：通过。代码完成条件已改为多条件 gate，禁止仅凭 `valid_candidates=0` 结束。本次 live run 终止原因是 `reached max_goals=20`。

2. 至少执行 8 个 goal，除非 raw frontier 已经很少且地图稳定。

   结论：通过。Phase 5 live run 完成 `completed_goals=20/20`。

3. 如果出现 `valid_candidates=0` 但 `frontier_clusters>5`，应触发 relaxed search 或 recovery scan。

   结论：代码路径已实现；本次 run 未出现该 exact 状态。实际证据显示 relaxed search 已执行并成功生成 goal。

4. 地图覆盖应明显大于 Phase 4。

   结论：通过。Phase 4 保存地图 unknown 约 21721 cells，Phase 5 保存地图 unknown 约 3646 cells；free 区域约 25818 -> 43182，覆盖显著扩大。

5. 保存出新的 phase5 地图。

   结论：通过。

   ```text
   src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5.yaml
   src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase5.pgm
   ```

6. `/map` 仍由 `slam_toolbox` 发布。

   结论：通过。live ROS graph 证据显示 `/map` 由 slam_toolbox 发布，`frontier_explorer` 订阅 `/map`。

7. AMCL/map_server 不启动。

   结论：通过。探索链路仍为 online SLAM + Nav2，不使用静态 map_server / AMCL。`map_saver_cli` 只在保存时临时使用，不是导航过程中的静态 map_server。

8. 原 Phase 0/1/2/3/4 入口不被破坏。

   结论：通过。未修改 `tugbot_nav.launch.py`、`tugbot_slam.launch.py`、`tugbot_slam_nav.launch.py` 和原 `nav2_params.yaml`；contract tests 通过。

## 19. Phase 5 验收结论

Phase 5 达成本阶段目标。

关键结论：

```text
Exploration complete: reached max_goals=20; completed_goals=20 success=20 failure=4 recovery_scans_completed=0 relaxed_search_executed=True map_stable_cycles=0
```

这证明本次不再复现 Phase 4 的 premature finish：探索没有因为 temporary valid candidate shortage 提前结束，而是持续执行到 `max_goals=20`。地图保存成功，Phase 5 地图覆盖明显大于 Phase 4，SLAM/Nav2/map save 链路保持可用，原冻结入口未被破坏。

Phase 5 当前状态：完成，等待人工验收。
