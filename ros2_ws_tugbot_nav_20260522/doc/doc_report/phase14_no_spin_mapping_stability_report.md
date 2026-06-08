# Phase 14 No-Spin Mapping Stability Report

工作区：`/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514`

报告文件：`doc/doc_report/phase14_no_spin_mapping_stability_report.md`

结论：`PASS_FOR_MANUAL_ACCEPTANCE`

> 本报告只记录 Phase 14 已完成的代码、参数、contract test、build、show-args、短程 live 验证与 45 秒 headless smoke 结果。  
> 本次补写报告没有重新运行 ROS/Gazebo/RViz，没有重新 build，没有重新生成地图，也没有修改源码、launch、config 或 README。

---

## 1. 人工发现的问题

Phase 13 人工验收时发现：

- 启动和探索过程中的原地大角度自转，容易让新的扫描结果偏离已经稳定的地图结构。
- RViz 中出现过墙线重影、局部错层、多层黑边等现象。
- 问题重点不是 camera，也不是 box pillar 本身。
- 重点问题是：大角度 spin 对 `slam_toolbox` 建图稳定性的影响。

Phase 14 的目标因此不是重做 camera / box pillar，也不是重新生成推荐地图，而是压制默认大角度 spin，降低探索阶段角速度激进程度，并验证 perimeter/frontier 链路仍可启动和生成目标点。

---

## 2. Phase 14 修改文件清单

Phase 14 已完成修改涉及以下文件：

- `src/tugbot_exploration/tugbot_exploration/frontier_explorer.py`
- `src/tugbot_bringup/launch/tugbot_explore.launch.py`
- `src/tugbot_navigation/config/nav2_slam_params.yaml`
- `src/tugbot_navigation/behavior_trees/navigate_to_pose_w_replanning_no_spin.xml`
- `src/tugbot_navigation/behavior_trees/navigate_through_poses_w_replanning_no_spin.xml`
- `src/tugbot_navigation/CMakeLists.txt`
- `src/tugbot_bringup/test/test_contract.py`
- `README.md`

说明：本次当前操作仅补写本报告文件；没有再次修改上述源码、launch、config 或 README。

---

## 3. 关闭 spin 默认行为的参数清单

Phase 14 将探索相关 spin 行为改为默认关闭，同时保留命令行 override 能力。

默认参数如下：

- `perimeter_enable_initial_spin`: 默认 `false`
- `cleanup_spin_after_goal`: 默认 `false`
- `enable_recovery_scan`: 默认 `false`
- `recovery_spin_angle`: 默认 `1.57`
- `cleanup_spin_angle`: 默认 `1.57`

含义：

- 默认不再在 perimeter 探索启动时执行 initial spin。
- 默认不再在 cleanup goal 后执行 spin。
- 默认不再执行 recovery scan spin。
- spin 角度参数仍保留为 launch 可 override 的能力，便于后续人工需要时显式开启或调整。

---

## 4. `perimeter_then_frontier` 行为调整

Phase 14 对 `perimeter_then_frontier` 的调整结果：

- `perimeter_then_frontier` 不再依赖 initial spin。
- live 验证中已经出现 `initial spin disabled/skipped` 日志。
- wall cluster detection 正常。
- 短程验证中检测到：`detected wall clusters=3`。
- 短程验证中接受 perimeter waypoint：`accepted perimeter waypoints=11`。
- 短程验证中拒绝 perimeter waypoint：`rejected perimeter waypoints=1`。
- 已发送第一个 perimeter waypoint。
- 若地图信息不足以生成 perimeter waypoint，应 fallback to frontier。

这说明关闭 initial spin 后，perimeter-first 逻辑仍能从当前地图中检测 wall clusters、生成 perimeter waypoints，并向 Nav2 发送第一个目标点。

---

## 5. `nav2_slam_params.yaml` 角速度调整说明

Phase 14 在 `nav2_slam_params.yaml` 中降低探索 / SLAM 场景下的旋转激进程度：

- `wz_max: 0.5`
- `az_max: 0.8`
- `behavior_server max_rotational_vel: 0.5`
- `behavior_server rotational_acc_lim: 0.8`
- `velocity_smoother max_velocity: [0.5, 0.0, 0.5]`
- `velocity_smoother max_accel: [2.5, 0.0, 0.8]`

边界说明：

- 未修改原 `nav2_params.yaml`。
- 未修改 `nav2_phase6_map_params.yaml`。

因此 Phase 14 的角速度收敛主要作用于 SLAM / exploration 参数链路，不覆盖 Phase 6 推荐地图相关参数文件。

---

## 6. no-spin Behavior Tree 修正说明

Phase 14 排查发现 ROS Jazzy 默认 Nav2 BT：

`/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`

包含如下恢复行为：

```xml
<Spin spin_dist="1.57" .../>
```

这意味着即使 exploration 侧关闭 initial spin / cleanup spin，Nav2 默认恢复行为仍可能通过 BT 触发 Spin。

因此 Phase 14 新增 no-spin BT：

- `src/tugbot_navigation/behavior_trees/navigate_to_pose_w_replanning_no_spin.xml`
- `src/tugbot_navigation/behavior_trees/navigate_through_poses_w_replanning_no_spin.xml`

修正策略：

- 保留 `Clear local costmap`。
- 保留 `Clear global costmap`。
- 保留 `Wait`。
- 保留 `BackUp`。
- 移除 `Spin`。

集成结果：

- `nav2_slam_params.yaml` 已引用 no-spin BT。
- `src/tugbot_navigation/CMakeLists.txt` 已安装 `behavior_trees`。
- 安装验证结果：`spin_present=False`。
- 安装验证结果：`recovery_no_spin=True`。

---

## 7. show-args 结果

`ros2 launch ... --show-args` 已验证以下默认值：

- `perimeter_enable_initial_spin` default `false`
- `enable_recovery_scan` default `false`
- `recovery_spin_angle` default `1.57`
- `cleanup_spin_after_goal` default `false`
- `cleanup_spin_angle` default `1.57`

结论：launch 层默认 no-spin 行为已经暴露并可由命令行 override。

---

## 8. contract test / build / py_compile 结果

Phase 14 已完成以下验证：

- `colcon build --symlink-install` 通过。
  - 结果：`Summary: 5 packages finished [1.46s]`
- `py_compile` 通过。
- `pytest` 通过。
  - 结果：`16 passed in 0.10s`

这些结果说明：

- Python 语法检查通过。
- contract test 已覆盖 Phase 14 相关行为。
- 工作区 build 通过。

---

## 9. 短程可视化验证结果

短程可视化验证启动命令：

```bash
ros2 launch tugbot_bringup tugbot_explore.launch.py headless:=false use_rviz:=true exploration_strategy:=perimeter_then_frontier max_goals:=8 enable_cleanup_mode:=true save_map:=false
```

验证结果：

- 未看到启动后 360° 原地自转。
- `frontier_explorer` 打印 `initial spin disabled/skipped`。
- wall cluster 检测成功。
- 生成并执行 perimeter waypoint。
- `/cmd_vel` angular 抽样最大约 `+0.118 rad/s`。
- `/cmd_vel` angular 抽样最小约 `-0.078 rad/s`。
- 未观察到高角速度 spin。
- camera bridge / TF 仍启动。
- box_pillar contract 仍通过。
- `/scan` / Nav2 / costmap 链路仍启动。

解释：短程 RViz/Gazebo live 验证证明默认启动阶段没有再出现明显 360° 原地自转，且 perimeter-first 链路没有因关闭 spin 而失效。

---

## 10. 45 秒 headless smoke 结果

45 秒 headless smoke 验证结果：

- lifecycle active。
- `frontier_explorer` 启动。
- initial spin skipped。
- wall cluster detection result 输出。
- perimeter `NavigateToPose` goal sent。
- 没有 BT Spin 行为证据。

解释：headless smoke 进一步确认无 RViz 条件下，no-spin 默认行为、frontier explorer 启动、wall cluster detection 和 perimeter goal dispatch 均能工作。

---

## 11. 地图重影是否缓解

当前结论必须分层表达：

已经完成并验证的部分：

- 机制上已压制默认大角度 spin。
- Nav2 默认 BT 中的 `Spin` 恢复行为已通过 no-spin BT 移除。
- 探索角速度采样更温和。
- 短程 live 验证未见高角速度 spin 证据。

尚不能粉饰为已完全解决的部分：

- 墙线重影、局部错层、多层黑边是否在长一点的人工 RViz 视觉验收中明显改善，仍需人工确认。
- Phase 14 短程验证没有进行长时间探索，也没有保存新地图，因此不能宣称最终地图重影问题已经完全消失。

结论：Phase 14 已经消除一个明确的高风险机制，即默认大角度 spin；但地图视觉质量改善仍等待人工 RViz 验收确认。

---

## 12. 当前遗留问题

短程验证中仍观察到以下现象：

- `Failed to make progress`
- `Optimizer reset`
- `controller loop missed desired rate`
- costmap recovery

解释：

- 这些现象属于 Nav2 控制、局部代价地图、waypoint 可达性或局部规划稳定性问题。
- 它们不是 no-spin 改造失败的证据。
- Phase 14 的目标是关闭默认大角度 spin、移除 Nav2 BT Spin、降低探索角速度，并验证 perimeter/frontier 链路不被破坏。

Phase 14 没有执行以下动作：

- 未进行长时间探索。
- 未保存地图。
- 未重新生成推荐地图。
- 未污染 Phase 6 推荐地图。

---

## 13. Phase 14 验收结论

验收建议：`PASS_FOR_MANUAL_ACCEPTANCE`

理由：

- 默认大角度 spin 已关闭。
- Nav2 默认 BT Spin 已移除。
- 探索角速度已降低。
- `perimeter_then_frontier` 仍能生成 waypoint。
- 默认 frontier 行为未被破坏。
- camera / box pillar / scan / costmap 链路未被破坏。
- contract test、py_compile、build 均通过。
- show-args 默认值符合 no-spin 预期。
- 短程 live 与 45 秒 headless smoke 均未出现 BT Spin 或启动后 360° 原地自转证据。

保留条件：

- 是否最终改善 RViz 中墙线重影、局部错层、多层黑边，需要人工进行更长一点的 RViz 视觉验收确认。

最终状态：

- `PASS_FOR_MANUAL_ACCEPTANCE`
- 停止于报告补写。
- 等待人工验收。
