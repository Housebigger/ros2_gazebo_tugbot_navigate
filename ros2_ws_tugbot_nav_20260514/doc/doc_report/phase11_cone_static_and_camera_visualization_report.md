# Phase 11 Gazebo 锥桶静态化与 Tugbot Camera 可视化增强报告

## 1. 人工复检提出的问题

本轮 Phase 11 处理两个人工复检改进项：

1. Gazebo 世界地图中的两个锥形雪糕筒会被 Tugbot 撞动，进而影响地图、避障和人工观察。
2. Tugbot 需要增加一个 camera，并在 RViz 的 Image Display 中实时显示 camera 视野画面。

边界遵守情况：

- 未修改 0513 工程。
- 未修改 Phase 6 推荐地图文件。
- 未删除 Phase 7A 地图。
- 未改变 Phase 0~10 的验收结论。
- 未运行长时间探索实验。
- camera 仅作为可视化增强，不接入 Nav2 costmap，不替代 `/scan`。
- 原有 5 个 launch 入口保持可用。

## 2. 静态检查结果

检查范围包括：

- `src/tugbot_gazebo/worlds/tugbot_nav_world.sdf`
- `src/tugbot_gazebo/launch/tugbot_gazebo.launch.py`
- `src/tugbot_description/models/tugbot/model.sdf`
- `src/tugbot_bringup/launch/*.launch.py`
- `src/tugbot_bringup/rviz/tugbot_nav.rviz`
- `src/tugbot_bringup/test/test_contract.py`
- `README.md`

发现情况：

- `tugbot_nav_world.sdf` 中存在两个 cone 模型：
  - `cone`
  - `cone_0`
- 两个 cone 原本均为 `<static>false</static>`。
- Tugbot 模型原本已有前/后 camera-like sensor，但 topic 为 `camera_front_color` / `camera_back_color`，未统一桥接到要求的 ROS topic `/camera/image_raw` / `/camera/camera_info`。
- `tugbot_bridge.yaml` 原本只桥接 `/clock`、`/cmd_vel`、`/odom`、`/scan`、`/tf`。
- RViz 原本已有 Image Display，但 topic 为 `/camera`，不是本轮要求的 `/camera/image_raw`。

## 3. 修改内容

### 3.1 固定 Gazebo 世界中的两个锥桶

修改文件：

- `src/tugbot_gazebo/worlds/tugbot_nav_world.sdf`

固定方式：

- 将 `cone` 与 `cone_0` 的 `<static>false</static>` 改为 `<static>true</static>`。
- 未删除锥桶。
- 未修改锥桶位置。
- 未扩大修改到其它动态模型。

锥桶位置：

```text
cone:
  pose: 3.805579848063759 -2.5960888656381456 0.49999999990741445 ...

cone_0:
  pose: 6.0753818100601649 2.0667701473945264 0.49999999990741445 ...
```

### 3.2 Tugbot 增加前向 camera 可视化

修改文件：

- `src/tugbot_description/models/tugbot/model.sdf`

实现方式：

- 用一个明确的前向 `camera_link` 替代原先未统一桥接的前/后 camera sensor。
- 新增固定关节 `camera_link_joint`：
  - parent: `base_link`
  - child: `camera_link`
- 新增 camera sensor：
  - sensor name: `color`
  - type: `camera`
  - topic: `/camera/image_raw`
  - frame: `camera_optical_frame`
  - frame 配置方式：使用 SDF 标准 `<camera><optical_frame_id>camera_optical_frame</optical_frame_id></camera>`，避免非标准 `<sensor><gz_frame_id>...` warning
  - update_rate: `15`
  - width: `640`
  - height: `480`
  - horizontal_fov: `1.047`
  - near: `0.05`
  - far: `20.0`

camera 仅用于可视化；未修改 `/odom`、`/tf`、`/cmd_vel`、Nav2 costmap 或导航参数。为清理 SDF schema warning，本轮同时移除了 `scan_omni` sensor 下非标准 `<frame_id>scan_omni</frame_id>`；`/scan` topic、bridge、static TF 与 Nav2 `/scan` 订阅仍已 live 验证正常。

### 3.3 Bridge 增加 camera topic

修改文件：

- `src/tugbot_gazebo/config/tugbot_bridge.yaml`

新增桥接：

```yaml
- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "/camera/image_raw"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: "GZ_TO_ROS"

- ros_topic_name: "/camera/camera_info"
  gz_topic_name: "/camera/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: "GZ_TO_ROS"
```

### 3.4 TF 增加 camera frame

修改文件：

- `src/tugbot_gazebo/launch/tugbot_gazebo.launch.py`

新增 static TF：

- `camera_link_static_tf`
  - `base_link -> camera_link`
  - translation: `0.30 0.0 0.55`
  - rotation: `0 0 0`
- `camera_optical_frame_static_tf`
  - `camera_link -> camera_optical_frame`
  - optical rotation: roll `-1.57079632679`，yaw `-1.57079632679`

原 `scan_omni_static_tf` 保持不变。

### 3.5 RViz Image Display 修正

修改文件：

- `src/tugbot_bringup/rviz/tugbot_nav.rviz`

修改内容：

- Image Display topic 从 `/camera` 改为 `/camera/image_raw`。
- Fixed Frame 仍保持原配置 `map`。

### 3.6 README 更新

修改文件：

- `README.md`

新增说明：

- 功能特性中增加 camera 可视化与锥桶静态化说明。
- 新增 `Camera 可视化` 小节，说明：
  - `/camera/image_raw`
  - `/camera/camera_info`
  - RViz Image Display
- 已知限制中明确：
  - camera 仅用于可视化增强。
  - camera 不参与当前 Nav2 costmap 或避障决策。
  - Nav2 当前仍主要依赖 `/scan` 进行避障。

### 3.7 Contract test 加强

修改文件：

- `src/tugbot_bringup/test/test_contract.py`

新增契约：

1. 检查 `cone` 与 `cone_0` 存在且 pose 未变。
2. 检查两个 cone 均为 `<static>true</static>`。
3. 检查 cone geometry 仍为 cone。
4. 检查 Tugbot 模型含 `camera_link` 与 camera sensor。
5. 检查 camera topic 为 `/camera/image_raw`。
6. 检查 camera frame 为 `camera_optical_frame`。
7. 检查 camera frame 通过标准 `<optical_frame_id>` 配置为 `camera_optical_frame`，且不含非标准 `<gz_frame_id>`。
8. 检查 camera 分辨率、FOV、near/far、update_rate。
9. 检查 `camera_link_joint` 固定连接 `base_link -> camera_link`。
10. 检查 Gazebo launch 启动 camera static TF。
11. 检查 bridge 配置含 `/camera/image_raw` 和 `/camera/camera_info`。
12. 检查 RViz Image Display 指向 `/camera/image_raw`。
13. 检查 README 含 camera 可视化与不参与 Nav2 避障说明。

RED 阶段结果：

```text
1 failed, 9 passed
失败项：test_phase11_cone_obstacles_are_static
原因：cone / cone_0 原本为 static=false
```

最终结果：

```text
14 passed in 0.08s
```

补充修正：人工收到后台 watch 后，发现首轮 live 日志有 SDF schema warning：

- `<sensor><gz_frame_id>camera_optical_frame</gz_frame_id>` 不是 SDF 标准 camera sensor 子元素。
- `scan_omni` 下 `<sensor><frame_id>scan_omni</frame_id>` 也会触发同类 schema warning。

处理方式：

- camera frame 改为标准 `<camera><optical_frame_id>camera_optical_frame</optical_frame_id></camera>`。
- 移除 `scan_omni` sensor 下非标准 `<frame_id>`，保持已有 `scan_omni_static_tf` 与 `/scan` bridge 不变。
- contract test 加强为禁止 camera 非标准 `<gz_frame_id>`，并禁止重新引入 `scan_omni` sensor-level `<frame_id>`。

补充验证结果：

```text
pytest: 14 passed in 0.08s
colcon build: Summary: 5 packages finished [1.74s]
py_compile: passed
short live: /camera/image_raw、/camera/camera_info、/scan、map_server、amcl 正常
launch log: 不再出现 XML Element[gz_frame_id] / XML Element[frame_id] SDF schema warning
```

## 4. 验证结果

### 4.1 Build

命令：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

结果：

```text
Summary: 5 packages finished [1.45s]
```

### 4.2 Python launch 编译检查

命令：

```bash
python3 -m py_compile src/tugbot_bringup/launch/*.launch.py src/tugbot_gazebo/launch/*.launch.py
```

结果：通过，无输出错误。

### 4.3 Contract test

命令：

```bash
python3 -m pytest -q src/tugbot_bringup/test/test_contract.py
```

结果：

```text
14 passed in 0.08s
```

### 4.4 colcon test

命令：

```bash
colcon test --packages-select tugbot_bringup --event-handlers console_direct+
colcon test-result --verbose
```

结果：

```text
Summary: 1 package finished [0.15s]
Summary: 0 tests, 0 errors, 0 failures, 0 skipped
```

说明：当前包的 colcon 集成测试未收集到 pytest；因此 authoritative contract 仍以直接 `python3 -m pytest` 为准。

## 5. 短时间 live 验证

### 5.1 Headless bringup 验证

命令：

```bash
ros2 launch tugbot_bringup tugbot_nav_phase6_map.launch.py headless:=true use_rviz:=false
```

检查结果：

- `/map_server` 存在。
- `/amcl` 存在。
- `/ros_gz_bridge` 存在。
- `/camera_link_static_tf` 存在。
- `/camera_optical_frame_static_tf` 存在。
- `/slam_toolbox` 不存在。
- `/frontier_explorer` 不存在。
- `/camera/image_raw` 存在，publisher 为 `/ros_gz_bridge`。
- `/camera/camera_info` 存在，publisher 为 `/ros_gz_bridge`。
- `/camera/image_raw` 能 echo 到 header，frame_id 为 `camera_optical_frame`。
- `/camera/camera_info` 能 echo 到数据，分辨率为 `640x480`，frame_id 为 `camera_optical_frame`。
- `base_link -> camera_link` TF 可查到，translation 为 `[0.300, 0.000, 0.550]`。
- `camera_link -> camera_optical_frame` TF 可查到。
- `/map` publisher 为 `/map_server`。
- `/map_server yaml_filename` 指向 Phase 6 cleanup map。
- `/map_server` lifecycle 为 `active [3]`。
- `/amcl` lifecycle 为 `active [3]`。
- localization lifecycle active service 返回 success=True。
- navigation lifecycle active service 返回 success=True。

关键输出摘要：

```text
/camera/image_raw:
  Type: sensor_msgs/msg/Image
  Publisher count: 1
  Node name: ros_gz_bridge

/camera/camera_info:
  Type: sensor_msgs/msg/CameraInfo
  Publisher count: 1
  Node name: ros_gz_bridge

camera_info:
  frame_id: camera_optical_frame
  height: 480
  width: 640

/map:
  Publisher count: 1
  Node name: map_server

/map_server yaml_filename:
  .../maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml

/map_server lifecycle: active [3]
/amcl lifecycle: active [3]
```

### 5.2 可视化入口短时验证

命令：

```bash
ros2 launch tugbot_bringup tugbot_nav_phase6_map.launch.py headless:=false use_rviz:=true
```

检查结果：

- `/rviz2` 节点存在。
- `/camera/image_raw` 存在。
- `/camera/camera_info` 存在。
- `/camera/image_raw` publisher 为 `/ros_gz_bridge`。
- `/camera/image_raw` subscription count 为 `1`，subscriber 为 `/rviz2`。
- `/camera/camera_info` 可 echo，frame_id 为 `camera_optical_frame`。

关键输出摘要：

```text
VISUAL_NODES:
/amcl
/camera_link_static_tf
/camera_optical_frame_static_tf
/lifecycle_manager_localization
/lifecycle_manager_navigation
/map_server
/ros_gz_bridge
/rviz2

/camera/image_raw:
  Publisher count: 1
  Node name: ros_gz_bridge
  Subscription count: 1
  Node name: rviz2

/camera/camera_info header:
  frame_id: camera_optical_frame
```

说明：RViz 在日志中出现过 OpenGL shader/link warning 与 LaserScan Message Filter drop warning，未影响 ROS graph、camera topic、RViz 订阅或 Nav2 lifecycle。该类 warning 属于图形/TF 时序提示，需人工视觉最终确认画面质量。

### 5.3 SDF schema warning 复检

首轮 headless live 日志中出现过：

```text
XML Element[gz_frame_id], child of element[sensor], not defined in SDF
XML Element[frame_id], child of element[sensor], not defined in SDF
```

复检修正后重新 build 并短时 launch。结果：

- `/camera/camera_info` 仍发布，header.frame_id 为 `camera_optical_frame`。
- `/camera/image_raw` 仍由 `/ros_gz_bridge` 发布。
- `/scan` 仍由 `/ros_gz_bridge` 发布，并有 AMCL / costmap / collision_monitor 订阅。
- `/map_server` 与 `/amcl` 仍为 `active [3]`。
- `/slam_toolbox` 与 `/frontier_explorer` 仍未启动。
- 重新检查 launch log 与 ROS 日志，未再发现 `XML Element[gz_frame_id]` 或 `XML Element[frame_id]` schema warning。

## 6. 进程清理

三次短时间 launch 验证完成后均已停止后台进程。

清理后检查：

```text
RESIDUAL_NODES_FINAL_PRE: 空
MATCHING_PROCESSES_FINAL_PRE: 空
```

说明：第一次 headless launch kill 后曾短暂残留一个 `/lifecycle_manager_navigation` 幽灵节点/进程，已通过 kill/kill -9 和 `ros2 daemon` refresh 清理。最终确认无目标 ROS 节点、Gazebo、RViz、Nav2、bridge 进程残留。

## 7. 对 Phase 6 / Phase 10 的影响检查

- `tugbot_nav_phase6_map.launch.py` 未修改。
- Phase 6 cleanup map 文件未修改。
- Phase 7A 地图未删除、未改为默认推荐。
- Phase 10 中静态 map replay 链路仍保持：
  - `/map` 由 `/map_server` 发布。
  - `/map_server yaml_filename` 指向 Phase 6 cleanup map。
  - `/slam_toolbox` 未启动。
  - `/frontier_explorer` 未启动。
- Camera 未接入 Nav2 costmap，未改变 `/scan` obstacle source；移除的是非标准 SDF `<frame_id>` 标签，`/scan` topic 与 TF 链路 live 复检正常。

## 8. 修改文件清单

本轮修改文件：

```text
README.md
src/tugbot_gazebo/worlds/tugbot_nav_world.sdf
src/tugbot_description/models/tugbot/model.sdf
src/tugbot_gazebo/config/tugbot_bridge.yaml
src/tugbot_gazebo/launch/tugbot_gazebo.launch.py
src/tugbot_bringup/rviz/tugbot_nav.rviz
src/tugbot_bringup/test/test_contract.py
doc/doc_report/phase11_cone_static_and_camera_visualization_report.md
```

未修改：

```text
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase6_cleanup.pgm
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.yaml
src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase7_budget.pgm
src/tugbot_navigation/config/nav2_params.yaml
src/tugbot_bringup/launch/tugbot_nav_phase6_map.launch.py
```

## 9. 当前遗留问题

1. RViz Image Display 的画面内容仍建议人工目视确认，尤其是视角是否符合人工观察习惯。
2. Camera 当前仅用于可视化，不参与导航避障；如果未来要基于视觉导航或视觉避障，需要另开阶段设计，不应混入 Phase 11。
3. RViz 可能出现 OpenGL shader warning 或 LaserScan Message Filter drop warning；本轮未发现其影响 camera topic、RViz 订阅或 Nav2 lifecycle。

## 10. Phase 11 验收结论

结论：`PASS_FOR_MANUAL_ACCEPTANCE`

Phase 11 已完成：

- 两个 Gazebo 锥桶已 static 固定。
- Tugbot 已新增前向 camera 可视化链路。
- `/camera/image_raw` 与 `/camera/camera_info` 已由 `/ros_gz_bridge` 发布。
- RViz Image Display 已订阅 `/camera/image_raw`。
- SDF schema warning 已收口：camera 使用标准 `<optical_frame_id>`，并移除 scan sensor 非标准 `<frame_id>`；复检日志未再出现 `XML Element[gz_frame_id]` / `XML Element[frame_id]` warning。
- Phase 6 静态地图回放链路仍保持正确。
- 未启动 `slam_toolbox` / `frontier_explorer`。
- build、py_compile、pytest、短时间 live 验证均通过。
- 验证后已清理进程。

等待人工验收。
