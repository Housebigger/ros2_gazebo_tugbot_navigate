# Phase 15 Phase 14 Map Manual Acceptance Archive Report

工作区：`/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514`

报告文件：`doc/doc_report/phase15_phase14_map_manual_acceptance_archive_report.md`

结论：`PASS`

> 本阶段只执行 Phase 14 优质地图人工验收归档、README 推荐地图更新与报告生成。  
> 本阶段没有修改源码，没有运行 Gazebo/RViz，没有重新生成地图，没有覆盖 Phase 6 cleanup map，也没有删除 Phase 14 原始 perimeter_test 地图。

---

## 1. 人工验收结论

Phase 14 人工验收通过。

人工验收观察：使用 no-spin + 低角速度 + `perimeter_then_frontier` 策略后，本次生成地图质量非常高：

- 墙线清晰；
- 外轮廓完整；
- 内墙稳定；
- 没有明显重影、错层、多层黑边；
- box pillar 能被稳定测绘；
- camera image 正常显示；
- Gazebo 与 RViz 地图对应关系良好。

验收解释：

- Phase 13 观察到的原地大角度 spin 引发地图重影风险，在 Phase 14 中通过 no-spin、低角速度与 perimeter-first 多观察点补扫策略得到明显改善。
- Phase 14 地图已达到推荐候选地图质量。

---

## 2. 原地图文件

人工验收通过的原始 perimeter test 地图文件为：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_perimeter_test.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_perimeter_test.pgm
```

文件检查结果：

| 文件 | 存在 | 大小 |
| --- | --- | ---: |
| `src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_perimeter_test.yaml` | yes | 158 bytes |
| `src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_perimeter_test.pgm` | yes | 49155 bytes |

边界：原始 `perimeter_test` 地图文件已保留，没有删除或覆盖。

---

## 3. 新归档地图文件

Phase 15 已将人工验收通过的 Phase 14 地图复制归档为新的推荐候选地图：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.pgm
```

文件检查结果：

| 文件 | 存在 | 大小 |
| --- | --- | ---: |
| `src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml` | yes | 170 bytes |
| `src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.pgm` | yes | 49155 bytes |

说明：

- 新 `.pgm` 与原 `perimeter_test.pgm` 同大小，来源为复制归档。
- 新 `.yaml` 大小略有变化，是因为 `image` 字段改为新的 `.pgm` 文件名。

---

## 4. YAML image 字段修正情况

原始 YAML image 字段：

```yaml
image: tugbot_nav_world_slam_perimeter_test.pgm
```

新归档 YAML image 字段已修正为：

```yaml
image: tugbot_nav_world_slam_phase14_perimeter_no_spin.pgm
```

新 YAML 当前内容：

```yaml
image: tugbot_nav_world_slam_phase14_perimeter_no_spin.pgm
mode: trinary
resolution: 0.050
origin: [-8.689, -5.462, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

结论：新归档 YAML 已正确指向新的 Phase 14 `.pgm` 文件名，不再指向 `perimeter_test.pgm`。

---

## 5. README 更新摘要

`README.md` 已更新以下内容：

- 当前推荐地图从 Phase 6 cleanup map 更新为 Phase 14 perimeter no-spin map。
- Phase 6 cleanup map 改为上一版稳定地图保留。
- 明确 Phase 14 人工验收已通过。
- 明确 no-spin + 低角速度 + `perimeter_then_frontier` 改善了地图重影问题。
- 明确 Phase 14 地图质量观察：墙线清晰、外轮廓完整、内墙稳定、无明显重影/错层/多层黑边、box pillar 稳定、camera image 正常。
- 保留 Phase 7A 为负面实验记录，并继续说明 Phase 7A 不推荐作为默认成果。
- 验收状态表增加 Phase 14 与 Phase 15 记录。
- `doc/doc_report/` 说明从 Phase 0~9 更新为 Phase 0~15。

---

## 6. 当前推荐地图

当前推荐地图为 Phase 14 perimeter no-spin map：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.pgm
```

推荐理由：

- Phase 14 人工验收通过。
- no-spin + 低角速度 + `perimeter_then_frontier` 明显改善了此前 spin 相关地图重影风险。
- 地图视觉质量高，墙线、外轮廓、内墙、box pillar 与 camera 显示均满足人工验收预期。

---

## 7. 保留的历史地图

### 7.1 Phase 6 cleanup map：上一版稳定地图

Phase 6 cleanup map 保留为上一版稳定地图：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
```

文件检查结果：

| 文件 | 存在 | 大小 |
| --- | --- | ---: |
| `src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml` | yes | 158 bytes |
| `src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm` | yes | 54755 bytes |

处理结果：未覆盖，未删除。

### 7.2 Phase 7A budget map：负面实验记录

Phase 7A 地图继续保留为负面实验记录，不作为推荐地图：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.pgm
```

文件检查结果：

| 文件 | 存在 | 大小 |
| --- | --- | ---: |
| `src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.yaml` | yes | 157 bytes |
| `src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.pgm` | yes | 75570 bytes |

保留原因：Phase 7A 覆盖更激进，但人工观察到重影、墙线重复、局部错层和扫描未对齐。该阶段用于证明过度补扫可能降低 SLAM 一致性，因此只作为负面实验记录。

---

## 8. Phase 15 执行边界

本阶段遵守以下边界：

- 未修改源码。
- 未运行 Gazebo/RViz。
- 未重新生成地图。
- 未覆盖 Phase 6 cleanup map。
- 未删除 `perimeter_test` 原始文件。
- 只新增 Phase 14 归档地图文件，更新 README，并生成本报告。

新增文件：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.pgm
doc/doc_report/phase15_phase14_map_manual_acceptance_archive_report.md
```

修改文件：

```text
README.md
```

---

## 9. Phase 15 验收结论

结论：`PASS`

Phase 15 已完成：

- 检查当前 `perimeter_test` 地图文件存在且非空；
- 复制归档为 Phase 14 perimeter no-spin 命名；
- 修正新 YAML 的 `image` 字段，使其指向新的 `.pgm` 文件名；
- 保留 Phase 6 cleanup map 作为上一版稳定地图；
- 保留 Phase 7A 为负面实验记录；
- 更新 README，将当前推荐地图更新为 Phase 14 perimeter no-spin map；
- 生成本 Phase 15 报告。

最终状态：

- `PASS`
- 停止于归档和文档更新。
- 等待人工确认。
