# go2_maze_bringup

[工作区](../../README.md) · [配置细节](config/README.md) · [工具](../../tools/README.md)

统一启动 Go2 的原生 Gazebo 物理、ROS bridge、robot_state_publisher、安装 TF、扫描投影、步态、在线求解、独立验收与可选 RViz。当前聚焦链不启动 Nav2 或 slam_toolbox。

## 入口与参数

从已构建的工作区根目录：

```bash
source setup.bash
ros2 launch go2_maze_bringup maze.launch.py gui:=true rviz:=true
```

日常演示优先使用 `./run_go2_maze.sh`，自动验收使用 `./tools/run_checked_maze.sh`；它们还选择本机图形环境并创建新的记录目录。

| 参数 | launch 默认值 | 作用 |
| --- | --- | --- |
| gui | false | Gazebo GUI；演示脚本设为 true |
| rviz | false | RViz；演示脚本设为 true |
| observer_cameras | true | 启用二/三号真实观察相机及 RViz 三窗格；false 恢复原地图布局 |
| autonomous | true | 启用在线导航、建图和独立验收；false 仅站立 |
| world | 安装后的 go2_maze.sdf | 当前迷宫；平地测试需同时设 autonomous=false |
| run_dir | /tmp/go2-maze-run | 裸 launch 默认目录；正式实验应换新目录或用包装脚本 |

默认导航模式明确设为 `online_slam`。已知入口、已知外围边界和在线发现墙体参与定位，完整内部墙图只在验收/绘图使用。直接启动旧 `flood_fill_solver` 默认可能进入完整墙图 `scan_match` 模式，不能当成当前在线模式。

## 时钟、坐标与显示

`/clock` 驱动控制、定位和验收。`map→odom` 由在线求解器发布，`odom→base_link` 来自原生 Gazebo 里程计，body→各腿来自真实 `/joint_states`。安装 TF 与 sensor pose 保持一致。

当前速度流为求解器 `/cmd_vel_nav` 在 launch 中重映射至 `/cmd_vel`，直接进入 Go2 步态；同一实例只能有一个正常运行的速度控制源。仿真实例的 ROS domain / Gazebo partition 由 `setup.bash` 和 GO2_MAZE 环境变量统一设置。

RViz 使用 map 总览，另存 Go2 close view；Gazebo / RViz 的 Qt 字体缩放固定以适应本机显示。GPU 雷达即使没有 GUI 也需渲染环境，headless 不等于禁止所有图形依赖。

默认 RViz 图像窗格：一号头部前视在下方，二号固定世界方向的斜上方跟随在右上，三号固定迷宫俯视在右下。标题栏可拖动浮动，调整尺寸；关闭了某个图像窗格时，重新启用对应 Image display 或重载 `three_cameras.rviz`。[跟随距离、视场与 TF](../go2_observer_cameras/README.md)。`observer_cameras:=false` 时不生成观察模型、不启动额外桥接和观察 TF，一号原有传感器仍保留。

背部绿色方向箭头在 Gazebo 传感器与 RViz RobotModel 中同时显示。完整开启三相机加箭头的一轮通关记录见 [本轮验证报告](../../../docs/validation/2026-11-07-go2-three-cameras/README.md)。
