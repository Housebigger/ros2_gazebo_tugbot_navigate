# Tugbot 自主探索建图与导航项目成功经验总结

## 1. 背景与最终成果

本项目从 0513 已验证的“已知静态地图 Nav2 导航基线”出发，复制并演进出独立的 0514 工作空间：

```text
ros2_ws_tugbot_nav_20260514
```

0514 工程最终完成了从未知环境到自建地图导航回放的完整链路：

```text
未知环境自主探索建图 -> 保存地图 -> 使用自建地图静态导航回放
```

最终成果包括：

- SLAM 在线建图：`slam_toolbox` 发布实时 `/map`；
- SLAM + Nav2 手动导航：Nav2 使用 live `/map` 执行 `NavigateToPose`；
- frontier 自主探索：`frontier_explorer` 自动识别 frontier 并发送 Nav2 goal；
- coverage cleanup：对 residual unknown 做有限补扫；
- 地图保存：通过 `map_saver_cli` 输出 `.yaml + .pgm`；
- Phase 6 自建地图静态导航回放：`map_server + AMCL + Nav2` 加载 Phase 6 cleanup map 并完成导航；
- GitHub-ready README：将工程能力、启动入口、推荐地图、验收状态与限制整理为开源仓库首页。

最终推荐地图：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
```

核心结论：

```text
ros2_ws_tugbot_nav_20260514 已完成“未知地图自主探索建图 -> 地图保存 -> 自建地图静态导航回放”的工程闭环。
```

---

## 2. 最大成功经验：不要破坏已验证基线

本项目最重要的工程经验是：先保护能跑的链路，再旁路扩展新能力。

经验可以浓缩为一句话：

```text
能跑的链路不要直接改，新增功能优先旁路扩展。
```

具体做法：

1. 先把 0513 定义为稳定基线。

   0513 已经证明 `tugbot_nav.launch.py + 静态地图 + AMCL + Nav2` 能跑，因此 0514 不直接在 0513 上冒险改动。

2. 复制出独立 0514 工作空间。

   新实验在 `ros2_ws_tugbot_nav_20260514` 中进行，避免污染 0513。

3. 每个阶段保护旧入口。

   例如：

   - `tugbot_nav.launch.py` 保留原静态地图导航基线；
   - `tugbot_slam.launch.py` 只负责 SLAM-only；
   - `tugbot_slam_nav.launch.py` 只负责 SLAM + Nav2；
   - `tugbot_explore.launch.py` 只负责在线探索；
   - `tugbot_nav_phase6_map.launch.py` 单独负责自建地图回放。

4. 新功能优先通过新增 launch、新参数文件、新包来实现。

   这样出问题时可以快速回退到上一阶段，而不是在一个大 launch 或一个大参数文件里纠缠。

5. 不把阶段性实验结果直接覆盖成默认成果。

   Phase 7A 虽然执行通过，但地图质量下降，因此只作为负面实验记录，不替换 Phase 6 推荐地图。

建议：

- 不要在已验证 launch 里直接塞入新模式；
- 不要把在线 SLAM、静态地图、frontier、cleanup、回放都堆到一个入口；
- 不要为了“看起来省文件”而牺牲可回退性。

---

## 3. 阶段化推进的价值

本项目采用了 phase-gated 方法：每个阶段只解决一个主问题，每个阶段都有明确验收标准，每个阶段生成报告，阶段完成后停止等待人工验收。

| 阶段 | 主问题 | 经验 |
| --- | --- | --- |
| Phase 0 | 0514 基线创建 | 先确认新工作空间能继承 0513 静态导航基线。 |
| Phase 1 | SLAM 在线建图 | 只接 `slam_toolbox`，不引入 Nav2 复杂性。 |
| Phase 2 | SLAM + Nav2 手动导航 | 先证明 live `/map` 可被 Nav2 使用，再考虑自主探索。 |
| Phase 3 | frontier 自主探索闭环 | 第一版目标是跑通 `/map -> frontier -> NavigateToPose`。 |
| Phase 4 | 地图保存 | 加入 `map_saver_cli`，并发现 premature finish 问题。 |
| Phase 5 | 修正 premature finish | 区分 raw frontier 和 valid candidate，避免过早结束。 |
| Phase 6 | residual unknown cleanup | 有限补扫，产出当前推荐地图。 |
| Phase 7A | 参数强化复跑 | 负面实验，证明过度补扫会降低地图质量。 |
| Phase 8 | 自建地图回放 | 使用 Phase 6 map 做 `map_server + AMCL + Nav2` 静态导航验证。 |
| Phase 9 | 最终验收总结 | 固化工程结论、推荐地图和已知限制。 |

关键价值：

- 问题定位更简单：每次只引入一个主要变量；
- 验收边界更清楚：失败时知道是哪个层次的问题；
- 人工判断更可靠：每个阶段都有文档和证据；
- 工程可回退：不因后续实验破坏前面已通过结果。

建议：

```text
复杂机器人系统不要一口气集成全部能力。
先让单链路跑通，再逐层叠加，最后做闭环回放。
```

---

## 4. SLAM 与 AMCL 的边界要清楚

在线建图和静态地图导航是两种不同模式，不能混淆。

### 在线探索建图模式

使用：

- `slam_toolbox`
- live `/map`
- Nav2 navigation stack
- `frontier_explorer`

不使用：

- 静态 `map_server`
- AMCL
- 旧地图 `map_1725111373.yaml`

此时 `/map` 的发布者应是：

```text
/map publisher == /slam_toolbox
```

含义：系统正在在线建图，`slam_toolbox` 负责 `map -> odom`。

### 静态地图回放模式

使用：

- `map_server`
- AMCL
- Nav2 navigation stack
- 已保存的 Phase 6 cleanup map

不使用：

- `slam_toolbox`
- frontier 探索链路
- Phase 7A budget map

此时 `/map` 的发布者应是：

```text
/map publisher == /map_server
```

含义：系统正在使用静态地图，AMCL 负责定位。

### 判断规则

| `/map` publisher | 系统模式 | 正常搭配 | 风险 |
| --- | --- | --- | --- |
| `/slam_toolbox` | 在线建图 / 探索模式 | SLAM + Nav2 + frontier | 不应启动 AMCL/map_server |
| `/map_server` | 静态地图回放模式 | map_server + AMCL + Nav2 | 不应启动 slam_toolbox |

教训：

- 不要在 online SLAM 探索阶段同时启动 AMCL；
- 不要让 `map_server` 偷偷加载旧地图；
- `/map` publisher 是判断系统模式的关键证据；
- 若模式混乱，后续 frontier、Nav2、地图保存结论都会失真。

---

## 5. launch 入口设计经验

本项目最终形成 5 个主要 launch 入口，各自职责明确。

| Launch | 模式 | 用途 | 静态地图 | AMCL | slam_toolbox |
| --- | --- | --- | --- | --- | --- |
| `tugbot_nav.launch.py` | 0513 基线 | 原静态地图导航 | 是 | 是 | 否 |
| `tugbot_slam.launch.py` | SLAM-only | 在线建图 | 否 | 否 | 是 |
| `tugbot_slam_nav.launch.py` | SLAM + Nav2 | 手动目标导航 | 否 | 否 | 是 |
| `tugbot_explore.launch.py` | SLAM + Nav2 + frontier | 自主探索建图 | 否 | 否 | 是 |
| `tugbot_nav_phase6_map.launch.py` | 静态回放 | Phase 6 自建地图导航 | 是 | 是 | 否 |

成功经验：

1. 不要把所有功能堆进一个 launch。

   一个 launch 同时支持静态地图、在线 SLAM、frontier、cleanup、回放，会导致参数和运行模式难以解释。

2. 不同模式用不同入口。

   入口名本身就是系统模式说明，降低误操作概率。

3. 保留通用 launch 参数。

   推荐保留：

   - `headless`
   - `use_rviz`
   - `use_sim_time`
   - `autostart`

4. 自动验证尽量 headless。

   自动化运行建议：

   ```text
   headless:=true use_rviz:=false
   ```

   人工验收时再打开 RViz。

5. 新地图回放使用独立 wrapper launch。

   Phase 8 没有把 Phase 6 map 写进原 `tugbot_nav.launch.py`，而是新增 `tugbot_nav_phase6_map.launch.py`。这避免污染 0513 静态地图基线。

---

## 6. frontier 探索经验

frontier 探索的工程目标不是第一版就完美，而是分层演进。

### 第一版只要跑通闭环

Phase 3 的正确目标是：

```text
/map -> frontier_explorer -> /navigate_to_pose -> 小车移动 -> /map 扩展
```

第一版只要能：

- 订阅 `/map`；
- 找到 unknown/free 边界；
- 聚类 frontier；
- 选择 known free goal；
- 调用 Nav2 `NavigateToPose`；
- goal 失败后 blacklist 并继续。

不要在第一版就追求完美 coverage planner。

### 不能只看 valid_candidates=0

Phase 4 的核心教训是：

```text
frontier_clusters=66
valid_candidates=0
```

这并不代表探索完成。

它只说明：

```text
当前严格过滤条件下，没有合格导航候选点。
```

但 raw frontier 仍然存在，地图还有大片未探索区域。如果此时直接结束，就会保存出只覆盖部分环境的地图。

### raw frontier 和 valid candidate 必须分开

推荐日志和状态至少包含：

- raw frontier cluster count；
- valid candidate count；
- completed goal count；
- strict / relaxed search mode；
- finish_no_frontier_cycles；
- recovery_no_candidate_cycles；
- recovery / cleanup 状态。

### strict search 与 relaxed search 分层

推荐策略：

1. strict search 优先，保证候选点安全；
2. strict 找不到但 raw frontier 仍多时，进入 relaxed search；
3. relaxed search 只有限放宽，不无限降低安全阈值；
4. 仍找不到候选时再做 recovery scan / cleanup，而不是直接结束。

### 候选点生成经验

有效候选点应满足：

- 落在 known free 区域；
- 周围有足够 free cells；
- 与障碍物保持足够距离；
- 不在 blacklist 半径内；
- 不离机器人过近或过远；
- cluster centroid 不安全时，可以尝试 cluster 边缘点或向机器人方向回退点。

### blacklist 不宜过激

goal 失败后应 blacklist，但半径不能过大。过大的 blacklist 会快速清空候选空间，造成“有 frontier 但没有 valid candidate”的假完成。

### goal 失败不是终止理由

Nav2 偶发 timeout、planner failure 或 controller progress warning 是探索系统的一部分。合理策略是：

```text
记录失败 -> blacklist/降权 -> 继续找下一个目标
```

而不是：

```text
第一个失败 goal -> 结束探索
```

---

## 7. cleanup 补扫经验

主体探索完成后，地图中仍可能有 residual unknown。

cleanup 的目标不是“消灭所有灰色区域”，而是有选择地补扫与 known free 相邻、可能可达、对地图可用性有价值的 unknown 区域。

### cleanup 适合处理什么

适合处理：

- 靠近已知 free 区域的 residual unknown；
- 主体路径附近漏扫的小灰块；
- 可通过短距离 Nav2 goal 接近并通过旋转补扫的区域。

不适合强行处理：

- 地图外部边界 unknown；
- 不可达区域；
- 障碍物背后或墙外 unknown；
- 需要过度贴边才能观察的区域。

### cleanup spin 的价值

cleanup goal 到达后，原地 spin 可以增加局部激光观测角度，对补扫残余 unknown 有帮助。

但 spin 也要有 budget 和失败处理。spin 失败不应让系统崩溃，只要后续 goals 与地图保存仍可完成，就可以作为有条件通过项记录。

### budget 要适度

Phase 6 使用有限 cleanup budget，得到当前推荐地图。Phase 7A 提高 goal 和 cleanup budget 后，虽然执行更多动作，但地图质量下降。

经验：

```text
cleanup budget 不是越大越好。
补扫越激进，SLAM 累积误差和局部错层风险越高。
```

---

## 8. 地图质量不是越大越好

Phase 6 和 Phase 7A 的对比，是本项目最有价值的工程取舍之一。

| 阶段 | 地图 | 特点 | 结论 |
| --- | --- | --- | --- |
| Phase 6 | `tugbot_nav_world_slam_phase6_cleanup` | 覆盖显著大于 Phase 4，几何质量较稳定，仍有少量 residual unknown | 当前推荐成果 |
| Phase 7A | `tugbot_nav_world_slam_phase7_budget` | 覆盖更激进，但出现重影、墙线重复、局部错层、扫描未对齐 | 负面实验，不采用 |

Phase 7A 的价值不是“失败”，而是证明：

```text
更高 max_goals、更高 cleanup budget、更宽松候选点约束，并不必然产生更好的地图。
```

地图质量不能只看：

- free cells 是否更多；
- 地图边界是否更大；
- unknown 总数是否单调下降。

还必须看：

- 墙线是否清晰；
- 是否有重影；
- 是否局部错层；
- 扫描是否对齐；
- 静态导航时 AMCL 与 Nav2 是否可用。

核心经验：

```text
自主探索建图的目标不是无限扩大覆盖，
而是在覆盖率、导航可用性和地图一致性之间取得平衡。
```

---

## 9. 验收证据应该结构化

机器人项目的“看起来能跑”不够，必须沉淀结构化证据。

本项目中有效的证据类型如下。

| 证据类型 | 作用 |
| --- | --- |
| `colcon build` 结果 | 证明工作空间可构建。 |
| `py_compile` | 快速发现 Python launch / node 语法错误。 |
| `pytest contract` | 防止 launch、参数、地图路径、模式边界回归。 |
| `ros2 node list` | 检查关键节点是否启动，以及是否误启动 AMCL/map_server/slam_toolbox。 |
| `ros2 topic info -v /map` | 判断 `/map` 发布者和 QoS，是区分在线 SLAM 与静态地图模式的核心证据。 |
| `/scan`、`/odom`、`/map` 采样 | 证明传感器、里程计和地图数据存在。 |
| `tf2_echo map base_link` | 检查定位链路和 TF 是否可用。 |
| lifecycle manager active | 证明 Nav2 localization/navigation 进入 active 状态。 |
| `NavigateToPose` action result | 证明 Nav2 goal 执行成功或失败原因。 |
| `/cmd_vel` 非零输出 | 证明控制器实际发出运动命令。 |
| `/odom` 位移 | 证明小车真实移动，而不是只有 action 返回。 |
| `map_saver_cli` 返回码 | 证明地图保存命令成功执行。 |
| `.yaml/.pgm` 文件检查 | 证明地图文件存在、非空、互相引用正确。 |
| RViz/PGM 人工目视验收 | 检查重影、墙线重复、错层等自动指标难以覆盖的问题。 |

建议：

- 报告中不要只写“成功”；
- 要写成功证据来自哪个节点、topic、action、文件或日志；
- 对地图质量，要结合数值统计与人工目视。

---

## 10. contract test 的价值

contract test 非常适合检查“工程约束”，尤其适合 ROS launch 工程。

它不替代 live experiment，但能防止低级回归。

本项目中 contract test 的价值包括：

- 新增 launch 后，检查是否误引用旧地图；
- 检查在线 SLAM 模式是否误启动 AMCL / `map_server`；
- 检查静态地图回放模式是否误启动 `slam_toolbox`；
- 检查原 `tugbot_nav.launch.py` 是否被污染；
- 检查 `tugbot_explore.launch.py` 是否仍保留探索参数；
- 检查 Phase 8 是否使用 Phase 6 cleanup map，而不是 Phase 7A budget map；
- 检查新增参数是否被 launch 暴露。

适合 contract test 检查的内容：

| 检查对象 | 示例 |
| --- | --- |
| 地图路径 | 不引用 `map_1725111373.yaml` 或不误用 `phase7_budget`。 |
| launch include | SLAM+Nav2 使用 `navigation_launch.py`，静态回放使用 `bringup_launch.py`。 |
| 参数文件 | 在线 SLAM Nav2 参数不包含 `amcl:`、`map_server:`、`yaml_filename`。 |
| 节点模式 | 探索模式包含 `frontier_explorer`，回放模式不包含它。 |
| 新参数 | `save_map`、`enable_cleanup_mode`、relaxed search 参数等被正确传入。 |

经验：

```text
contract test 最适合防止“模式边界被无意破坏”。
```

---

## 11. 推荐默认成果与负面实验要分开

本项目最终明确区分：

| 类型 | 文件 | 定位 |
| --- | --- | --- |
| 推荐成果 | `tugbot_nav_world_slam_phase6_cleanup.yaml/.pgm` | 当前最佳自建地图，用于静态导航回放。 |
| 负面实验 | `tugbot_nav_world_slam_phase7_budget.yaml/.pgm` | 参数强化复跑结果，证明过度 cleanup 会降低地图质量。 |

负面实验不是没有价值。

Phase 7A 的价值在于：

- 证明“更多 goal”不等于“更好地图”；
- 证明“更激进 cleanup”可能引入重影和错层；
- 帮助确定 Phase 6 是更合理的默认推荐成果；
- 避免后续 README 或用户误把所有产物都当成默认成果。

建议：

```text
README 和默认 launch 应只推荐经过最终取舍的成果。
负面实验应保留在报告中，而不是进入默认路径。
```

---

## 12. 对后续项目的复用模板

以后做类似 ROS 2 + Gazebo + Nav2 + SLAM 项目，可以按以下模板推进。

| Step | 动作 | 目的 |
| --- | --- | --- |
| Step 1 | 保留原成功工程作为 baseline | 先拥有可回退的真值基线。 |
| Step 2 | 复制新工作空间 | 避免污染旧工程。 |
| Step 3 | 先验证 baseline | 确认新 workspace 可构建、可启动原链路。 |
| Step 4 | SLAM-only | 单独验证 `/scan`、`/odom`、`/tf`、`/map`。 |
| Step 5 | SLAM + Nav2 手动 goal | 验证 Nav2 能基于 live `/map` 导航。 |
| Step 6 | frontier 探索 | 跑通自主目标生成与执行闭环。 |
| Step 7 | 地图保存 | 将在线 `/map` 落盘为 `.yaml + .pgm`。 |
| Step 8 | 修正完成条件 | 避免 valid candidate 暂时为空导致提前结束。 |
| Step 9 | 补扫 residual unknown | 有限 cleanup，提高地图实用性。 |
| Step 10 | 保存地图回放导航 | 用自建 map 跑 `map_server + AMCL + Nav2`。 |
| Step 11 | README 与经验文档 | 固化对外使用方法和工程经验。 |

推荐节奏：

```text
每一步只改一个层次；
每一步有可执行命令；
每一步有验收证据；
每一步结束后写报告并停止。
```

---

## 13. 常见风险与规避方法

| 风险 | 表现 | 规避方法 |
| --- | --- | --- |
| AMCL 和 SLAM 同时抢 `map -> odom` | TF 混乱、定位跳变、Nav2 行为不稳定 | 在线探索阶段只启动 `slam_toolbox`；静态回放阶段只启动 `map_server + AMCL`。 |
| 误加载旧地图 | 以为在探索未知环境，其实 `/map` 来自旧静态地图 | 检查 `ros2 topic info -v /map`，确认 publisher 是 `/slam_toolbox`。 |
| frontier 过早结束 | 地图只覆盖一部分，日志显示 raw frontier 仍多 | 区分 `frontier_clusters` 和 `valid_candidates`，禁止仅凭 `valid_candidates=0` 结束。 |
| cleanup 过度导致重影 | 墙线重复、局部错层、扫描未对齐 | 限制 cleanup budget，优先几何一致性，不盲目追求覆盖扩大。 |
| RViz/OpenGL 干扰自动验证 | GUI warning、资源占用、自动化不稳定 | 自动验证使用 `headless:=true use_rviz:=false`，人工验收再打开 RViz。 |
| 目标点在 unknown 或障碍附近 | Nav2 planner failure、controller recovery、timeout | 候选点必须在 known free，检查 free neighbors、障碍距离和 blacklist。 |
| Nav2 planner 可达性不足 | `ComputePathToPose` 或导航 action 失败 | 先用 lightweight precheck，后续可加入批量 `ComputePathToPose`。 |
| 保存地图后未检查 yaml/pgm | 以为保存成功，实际文件缺失或为空 | 检查 map_saver 返回码、`.yaml/.pgm` 是否存在、大小是否非零、yaml 是否引用 pgm。 |
| 只看 unknown 数量不看几何质量 | free 增多但地图重影、错层 | 结合 PGM 统计、RViz/PGM 目视、静态导航回放结果综合判断。 |
| 修改默认入口污染历史基线 | 后续无法比较新旧系统 | 新模式使用新增 launch，保留原 launch 不变。 |
| 长时间实验残留进程污染结论 | 重复 publisher/subscriber、旧 Gazebo/ROS graph 影响新实验 | 阶段验收后清理进程；不要把污染状态下的 graph 当作权威证据。 |

---

## 14. 当前仍可优化的方向

以下只是后续建议，本经验文档不执行这些任务。

1. `ComputePathToPose` 批量预检查

   在发送 `NavigateToPose` 前，对多个候选 goal 批量做 planner reachability precheck，减少不可达目标进入执行队列。

2. 地图质量自动评分

   除了 free/unknown/occupied 统计，还可尝试评估：

   - 墙线连续性；
   - 重影程度；
   - occupied 边界清晰度；
   - 与 Gazebo world 或人工标注轮廓的相似度。

3. 回环检测参数调优

   对长时间探索中的 SLAM 累积误差、局部错层和扫描未对齐进行更细粒度调参。

4. 更精细 coverage planner

   让 cleanup 更关注“可达且有价值”的 residual unknown，而不是追求全局 unknown 数量最小。

5. 保存地图后自动退出 launch

   当前地图保存后仍可能需要外部停止 launch。后续可让 explorer 保存成功后触发更明确的 shutdown/exit 策略。

6. 多地图、多环境泛化测试

   在不同 Gazebo world、不同障碍布局和不同初始位姿下复验探索建图与静态回放闭环。

---

## 15. 最终一句话总结

本项目成功的关键，不是一次性写出复杂算法，而是守住已验证基线、阶段化扩展、用证据验收每一步，并在地图覆盖率与几何一致性之间做出工程取舍。
