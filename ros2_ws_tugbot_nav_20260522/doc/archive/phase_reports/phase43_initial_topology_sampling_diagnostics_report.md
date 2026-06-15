# Phase43 Startup Initial Topology Sampling Diagnostics Report

## 结论

Phase43 bounded diagnostics 完成。结论分类：`TOPOLOGY_REJECTION_CAUSE_IDENTIFIED`。

严格保持 Phase42 结论 `BOUNDED_SMOKE_NO_DISPATCH_TOPOLOGY_SAMPLING`，未写成自主探索成功，也未写成 Nav2 action failure。本阶段只新增只读诊断字段、runtime 采集器/分析器/wrapper、contract tests 与报告；没有修改 Nav2/MPPI/controller 参数，也没有修改 maze_explorer 的探索策略。

## 关键证据

- active world: `tugbot_maze_world_20260528_clean_scaled2x.sdf`
- active metadata: `maze_20260528_scaled_instance.yaml`
- active truth frame: `map`
- entrance_map: `(0.0, 0.0, 0.0)`
- exit_map: `(21.072562, 18.083566)`, radius `1.2`
- explorer_state samples: `12`
- goal_events samples: `0`
- dispatch_events: `0`
- final mode: `FAILED_EXHAUSTED`
- goal_count: `0`
- last_local_topology_kind: `unknown`
- last_open_direction_count: `0`
- last_candidate_count: `0`
- last_terminal_reason: `no untried branches remain`

## Frame alignment evidence

首次拓扑采样时 robot pose in map:

```json
[
  1.0685924042060368e-11,
  -1.040083094747332e-24,
  2.7398959966203704e-13
]
```

robot 到 active entrance 的距离：`1.0685924042060368e-11` m。

runtime TF evidence 显示 `map->base_link`、`odom->base_link`、`map->odom` 均可用；latest snapshot 中 robot_to_active_entrance 仍约 `1.0685945389520468e-11` m。此证据不支持 Phase39 旧 frame mismatch 继续存在于 Phase41/42 map-frame truth convention 下。

## 首次拓扑采样证据

首次拓扑采样 snapshot:

- sampled_direction_count: `4`
- raw_open_direction_count: `0`
- filtered_open_direction_count: `0`
- candidate_branch_count: `0`
- reject_reason_counts: `{"clearance_radius_blocked": 4}`

四个采样方向全部为 `clearance_radius_blocked`，`safe_distance_m=0.0`，`map_cell_state=free`，`clearance_result=blocked`。这说明首采样没有形成 open direction / branch candidate 的直接原因已经被只读诊断字段定位：以当前 `clearance_radius_m=0.38` 的地图清障检查在 robot 附近立即失败。

注意：Phase43 没有通过放宽阈值、调大采样半径、改分支选择来修复该现象；仅记录原因。

## map / costmap / scan sufficiency around first topology

首次 topology 采样 recorder elapsed: `14.682981729507446` s。

Before snapshot near robot:

- map known_ratio: `0.7661725067385444`, free_ratio: `0.7567385444743935`, unknown_ratio: `0.23382749326145552`, occupied_ratio: `0.009433962264150943`
- local_costmap known_ratio: `1.0`, free_ratio: `0.8393874643874644`, unknown_ratio: `0.0`, occupied_ratio: `0.1606125356125356`
- global_costmap known_ratio: `None`, free_ratio: `None`, unknown_ratio: `None`, occupied_ratio: `None`
- scan finite_count: `322`, nearest_obstacle_m: `1.4781546592712402`

After/latest bounded startup snapshot near robot:

- map known_ratio: `0.7661725067385444`, free_ratio: `0.7567385444743935`, unknown_ratio: `0.23382749326145552`, occupied_ratio: `0.009433962264150943`
- local_costmap known_ratio: `1.0`, free_ratio: `0.8393874643874644`, unknown_ratio: `0.0`, occupied_ratio: `0.1606125356125356`
- global_costmap known_ratio: `0.8247978436657682`, free_ratio: `0.6462264150943396`, unknown_ratio: `0.1752021563342318`, occupied_ratio: `0.17857142857142858`
- scan finite_count: `322`, nearest_obstacle_m: `1.4781546592712402`

解释：scan flow 在 bounded runtime 中是可用的，local costmap 后续也可用；但是 maze_explorer 首次 topology sampling 发生时，其自身只读诊断显示 local_costmap 尚不可用于该采样，且 map clearance check 已在 0.05m endpoint 处失败。最终进入 `FAILED_EXHAUSTED`，且 `entered_failed_exhausted_before_map_sufficient=true`。

## Artifacts

- `log/phase43_initial_topology_sampling_diagnostics/phase43_initial_topology_sampling_diagnostics.json`
- `log/phase43_initial_topology_sampling_diagnostics/first_topology_sampling_snapshot.json`
- `log/phase43_initial_topology_sampling_diagnostics/map_scan_odom_tf_costmap_samples.json`
- `log/phase43_initial_topology_sampling_diagnostics/phase43_initial_topology_full_data.json`
- `log/phase43_initial_topology_sampling_diagnostics/phase43_robot_vs_entrance_overlay.png`
- `log/phase43_initial_topology_sampling_diagnostics/phase43_initial_topology_rejection_overlay.png`
- `log/phase43_initial_topology_sampling_diagnostics/phase43_initial_topology_sampling_diagnostics_cleanup_processes_after.txt`

## 新增/修改文件

- `tools/run_phase43_initial_topology_sampling_diagnostics.sh`
- `tools/analyze_phase43_initial_topology_sampling_diagnostics.py`
- `src/tugbot_maze/test/test_phase43_initial_topology_sampling_diagnostics.py`
- `src/tugbot_maze/tugbot_maze/maze_explorer.py`：仅新增 Phase43 read-only topology sampling diagnostics 字段输出；不改变策略路径。
- `doc/doc_report/phase43_initial_topology_sampling_diagnostics_report.md`

## 验证

- `python3 -m py_compile tools/analyze_phase43_initial_topology_sampling_diagnostics.py src/tugbot_maze/tugbot_maze/maze_explorer.py` passed
- `bash -n tools/run_phase43_initial_topology_sampling_diagnostics.sh` passed
- focused/regression pytest:
  - `src/tugbot_maze/test/test_phase36_autonomous_readiness.py`
  - `src/tugbot_maze/test/test_phase37_bounded_smoke_wrapper.py`
  - `src/tugbot_maze/test/test_phase41_world_to_ros_pose_frame_convention.py`
  - `src/tugbot_maze/test/test_phase42_map_frame_truth_bounded_smoke.py`
  - `src/tugbot_maze/test/test_phase43_initial_topology_sampling_diagnostics.py`
  - result: `29 passed in 0.35s`
- Phase36 readiness: `READY_FOR_BOUNDED_AUTONOMOUS_EXPLORATION_SMOKE`, blockers empty
- `git diff -- src/tugbot_navigation/config` empty
- cleanup check empty; no ROS/Gazebo/Nav2/recorder residual process matched after cleanup

## Guardrail status

- no Nav2/MPPI/controller parameter edit: satisfied
- no maze_explorer strategy edit: satisfied; diagnostic-only state payload additions
- no fallback/terminal acceptance continuation: satisfied
- no old scaffold world/map: satisfied
- no autonomous exploration success claim: satisfied
- no long run: satisfied; bounded startup diagnostics only
- no ros2 topic echo truncation as map/costmap basis: satisfied; evidence uses rclpy recorder/serialized summaries

## Phase43 final classification

`TOPOLOGY_REJECTION_CAUSE_IDENTIFIED`

The identified rejection cause is `clearance_radius_blocked` on all four initial topology sample directions, before any goal dispatch, with robot frame alignment OK at active entrance and scan flow available in bounded runtime evidence.

停止于 Phase43，等待人工验收。
