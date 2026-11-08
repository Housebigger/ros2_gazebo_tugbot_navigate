# go2_observer_cameras

[工作区](../../README.md) · [启动参数](../go2_maze_bringup/README.md) · [桥接与 RViz](../go2_maze_bringup/config/README.md)

二号和三号均为 Gazebo 原生 camera sensor；一号复用 Go2 已有头部相机。默认与 `maze.launch.py` 一同生成，真实图像通过 ros_gz_bridge 进入 RViz Image dock。

| 相机 | 位置 / 方向 | HFOV | 图像 |
| --- | --- | --- | --- |
| 1 头部 | base_link 前方 0.35 m、上方 0.05 m | 保留原模型内参 | 720×540，20 Hz |
| 2 跟随 | Go2 位置加世界坐标偏移 (-1.2,-0.9,+3.5) m，瞄准 base_link 下方 0.1 m | 0.72 rad | 1280×720，20 Hz |
| 3 全景 | 世界 (0,0,24) m，垂直向下，画面上方 +Y、右方 +X | 1.0 rad | 1024×1024，10 Hz |

帧率按仿真时间配置，实际接收受实时因子、渲染和订阅负载影响。额外相机由分辨率/HFOV 自动生成居中 CameraInfo，不沿用头部相机的 fx/fy/cx/cy。三号视场覆盖迷宫所有墙体角点（实际墙体边界约 ±10.114 m）；狗在全景中较小，姿态细节使用二号。

## 跟随实现与隔离

`go2_chase_camera` 是 Gazebo system 插件，只挂载在独立的静态 camera_chase 模型。PreUpdate 读取 go2/base_link 世界位置，按固定世界坐标偏移平移相机，保持视线方向和地平线，不跟随 body yaw/roll/pitch；仅向观察模型写 SetWorldPoseCmd。PostUpdate 读取实际 ECS 相机位姿，再发布 optical TF。按用户选择，模型配置默认 `follow_yaw=false`。改为 true 可恢复随 Go2 航向旋转的备选模式；此时实时检查器也需加 `--follow-yaw`。

两份观察 SDF 无碰撞、无关节，也不连接 Go2。不会增加机器人质量或向 Go2 写位姿、速度、力矩。静态观察模型不会进入 SceneBroadcaster 的动态 top-level 模型真值流，保留现有独立验收约定。相机只观察，导航不订阅它们，完整内部墙图仍不输入导航器。

`ros_gz_sim/create` 的 EntityFactory 默认位姿会覆盖 SDF model pose；bringup 显式从 SDF 提取六维位姿作为生成参数，避免俯视相机落在世界原点。修改 SDF 相机高度/方向时须一起更新其固定 optical TF 及覆盖测试。

当前工程使用世界原点的 `odom`，故观察相机以 odom 为 parent。后续换里程计原点时必须适配世界到 odom 变换。一号保留原有 sensor header 与安装 TF，RViz Image 不依赖 optical 投影显示；新两路使用标准 optical frame。

## 构建与检查

依赖 ROS 2 Lyrical 的 ament_cmake、gz_sim_vendor、gz_plugin_vendor（Gazebo Jetty）。从工作区根目录执行：

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select go2_observer_cameras go2_maze_bringup
ctest --test-dir build/go2_observer_cameras --output-on-failure
./tools/test_go2_core.sh
./run_go2_maze.sh
```

新建完整工作区时先按顶层 README 构建全部六个包。关闭观察相机使用 `observer_cameras:=false`；仅站立使用 `autonomous:=false`。

只读实时检查在另一个同 domain/partition 的终端运行 `python3 tools/check_go2_cameras.py /tmp/new-camera-check --seconds 90 --require-motion`。它保存实际图像和 JSON；不发布运动命令，不证明整轮通关。证据见 [三相机联调记录](../../../docs/validation/2026-11-07-go2-three-cameras/README.md)。

Go2 背部纯 visual 的绿色黑边箭头由 go2_description / tugbot_description 共享，使固定世界方向下的转弯和全景位置更容易辨认。原生相机包不负责改变机器人外观或控制。[本轮经验](../../../docs/experience/2026-11-07-go2-cameras-and-marker.md) 区分初始随航向与最终固定方向数据，并记录带标记后的完整通关。
