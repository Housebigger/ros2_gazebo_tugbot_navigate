# Unitree Go2：最新版迷宫自主通关

本工作区基于仓库 `2381e500e369b7a33d6ca9968e3b7ce7d45f762f` 的 `ros2_ws_tugbot_nav_20260728`，保留最新 2 倍尺寸迷宫和在线 flood-fill 导航，替换为宇树官方 Go2 模型。历史工作区的运行代码保留。模型来源、固定提交和 BSD 许可证见 `src/go2_description/SOURCE.md`。

[通关经验](../docs/experience/2026-11-07-go2-maze-success.md) · [三相机与箭头经验](../docs/experience/2026-11-07-go2-cameras-and-marker.md) · [包索引](src/README.md) · [启动参数](src/go2_maze_bringup/README.md) · [工具](tools/README.md)

## 构建与启动

本机验证环境为 Ubuntu 26.04、ROS 2 Lyrical、Gazebo Jetty；使用已安装的 `ros_gz`、Gazebo vendor 插件和 NVIDIA 显卡。当前聚焦启动不启动 Nav2 或 slam_toolbox。请在新终端中只加载此工作区，避免同名旧包覆盖。

```bash
cd ros2_ws_go2_maze_20261107
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select \
  go2_description go2_observer_cameras tugbot_description tugbot_gazebo tugbot_maze go2_maze_bringup
./run_go2_maze.sh
```

默认同时打开 Gazebo、RViz，并自动从入口开始导航。Ctrl+C 关闭本次 launch 及其子进程，不会全局杀死其他 ROS/Gazebo 会话。使用本机桌面显示时不应设置 `QT_QPA_PLATFORM=offscreen`，GPU 传感器需要可用的 OpenGL 渲染环境。

```bash
# 无界面自主运行，仍运行真实物理与渲染雷达/相机
./run_go2_maze.sh gui:=false rviz:=false
# 仅站立，不启动导航、建图或验收器
./run_go2_maze.sh autonomous:=false
# 关闭两路观察相机，使用原地图 RViz 布局
./run_go2_maze.sh observer_cameras:=false
# 平地运动检查
source setup.bash
ros2 launch go2_maze_bringup maze.launch.py autonomous:=false \
  world:="$PWD/install/tugbot_gazebo/share/tugbot_gazebo/worlds/go2_test.sdf"
```

`setup.bash` 默认使用 ROS domain 55、Gazebo partition `go2_maze`。另开独立实例前设置 `GO2_MAZE_ROS_DOMAIN_ID` 和 `GO2_MAZE_GZ_PARTITION`。同一实例的调试终端应加载同一设置。

需要自动等待独立验收并在结束后关闭本次仿真时，运行 `./tools/run_checked_maze.sh`。其运行目录必须为空，防止复用旧成功记录；最多运行 1800 仿真秒或 3600 墙钟秒，完整轨迹重放通过后才返回成功。

## 联调链路

- Go2 官方惯量、碰撞体、关节限位与 DAE 材质；12 个 Gazebo 原生关节力矩 PID，运动由足地接触产生。
- `/cmd_vel` → 连续对角步态、IK、姿态修正 → 12 个关节目标；0.6 仿真秒无新指令时收步站立。
- `/joint_states` → robot_state_publisher；`/odom` → `odom→base_link`；在线定位发布 `map→odom`。
- 中置 16 线三维雷达 `/lidar/points` → 900 束 `/scan`；前置相机 `/camera/front/image`。
- 原有 `online_slam` flood-fill 使用已知边界、已知入口和在线探测的内部墙体；内部完整墙图不输入导航器。
- 保留原项目的 Gazebo 世界坐标里程计设计。这是仿真导航验证，尚不等于接入实机腿式里程计的无真值 SLAM。

Go2 的速度上限为 0.25 m/s、0.5 rad/s，巡航指令 0.22 m/s。实际速度受步态接触和转弯影响；导航超时已按实测调整。

RViz 显示 Go2 实时关节姿态、在线积累的三维迷宫、实时扫描和独立真实轨迹。轨迹/完整墙体验收不会发布控制命令。

Go2 背部带黑边的亮绿色箭头沿机体 +X 指向机头，帮助在灰色迷宫中辨认位置与朝向。标记在 Gazebo 相机和 RViz RobotModel 中一致显示，仅新增视觉元素。

## 三路实时相机

默认启用 `observer_cameras:=true`，Gazebo 和 RViz 同时运行时无需另开图像查看器：

| RViz 窗格 | 视角 | ROS 图像 topic | 原始分辨率 / 仿真帧率 |
| --- | --- | --- | --- |
| Camera 1 - Head Forward | Go2 头部前向传感器 | /camera/front/image | 720×540 / 20 Hz |
| Camera 2 - Stable Chase | 只随位置平移，保持世界方向和地平线的斜上方近景 | /camera/chase/image | 1280×720 / 20 Hz |
| Camera 3 - Maze Overview | 世界中心 (0,0,24 m) 垂直俯视，画面上方为 +Y | /camera/overview/image | 1024×1024 / 10 Hz |

一号在地图下方，二、三号在右侧上下排列；可以拖动标题栏浮动、调整尺寸。三路均为真实 Gazebo camera 图像，使用同一仿真时钟，按各自帧率异步刷新。头部相机随身体姿态运动，二号采用用户选择的固定世界方向：只随位置平移，Go2 转弯时镜头不绕狗旋转，也不随身体横滚/俯仰倾斜。三号覆盖全部迷宫，Go2 在大全景中相应较小，姿态细节看二号。

新增观察模型无碰撞、无关节，不连接 Go2，不写 Go2 位姿、速度或力矩。原步态和导航配置保持原值。额外图像按订阅懒桥接，但启用传感器仍有渲染开销；完全关闭请用 `observer_cameras:=false`。[相机包说明](src/go2_observer_cameras/README.md)。

在同一实例的另一个终端执行只读检查：

```bash
source setup.bash
python3 tools/check_go2_cameras.py /tmp/my-new-camera-check --seconds 90 --require-motion
ctest --test-dir build/go2_observer_cameras --output-on-failure
```

检查器不发布运动指令；`--require-motion` 需要正在自主行走并转弯。仅站立检查时去掉该参数。输出目录必须为空，保存三路起止 PNG 与时间、内参、实际跟随误差 JSON。[实际联调记录](../docs/validation/2026-11-07-go2-three-cameras/README.md)。

## 独立验收与记录

每次运行输出到 `log/runs/<时间>/`：

- `ground_truth.csv`：原生 Gazebo 动态模型位姿，按已知入口平移到 map 坐标。
- `status.json`：真实出口距离、路径长度、倾角、碰墙/越界计数及最终 `success`。
- 导航器生成的岔路决策与在线墙图记录。

验收器检查高频原生位姿并在相邻姿态间以不超过 1 cm / 0.02 rad 细分，使用保守 Go2 足迹与完整墙几何做碰撞检查。它独立于 ICP 修正后的导航位姿；这属于几何碰撞验收，不是接触力传感器报告。

到达离散出口格后，Go2 会使用在线导航位姿与新鲜雷达，以不超过 0.12 m/s 的速度继续进入半径 0.85 m 的内部目标区并站立 2 秒，再报告 EXIT_REACHED。该收尾不读取验收真值。

只有导航器报告出口、真实位姿距出口 `(21.072562,18.083566)` 不超过 1.2 m、线速度小于 0.03 m/s、角速度小于 0.08 rad/s 持续 2 秒，且无碰墙、无非法越界、无跌倒，最大总倾角不超过足迹几何抽样的 0.1 rad 范围，才记为成功。单独出现 `EXIT_REACHED` 日志不能代替此结果。

## 检查

```bash
./tools/test_go2_core.sh
source setup.bash
python3 tools/check_go2_motion.py  # 需先单独启动平地仿真
```

上述检查覆盖核心导航、定位与 Go2 关节限位。本机三相机阶段 Go2 核心回归为 218 项通过，原生相机几何 CTest 1/1 通过。更广的历史测试实测为 564 项通过、7 项失败：6 项旧 wall-follow 算法用例和 1 项要求缺失的 maze_20260522.jpg 的旧资产用例；原工作区 README 已记录此类历史失败。依赖未安装 nav2_msgs 的旧 reactive_pilot 测试未运行。不能声称整个历史测试集全绿。不要使用旧 `run_sim.sh` 的全局进程清理逻辑启动本工作区。

可用 `python3 tools/replay_go2_audit.py <运行目录> --require-success` 对完整真实轨迹重新验收。保守足迹为 x=[-0.42,0.37]、|y|≤0.25 m，覆盖 Go2 后膝及后小腿；不能沿用轮式底盘边界。

## 模型重生成

工作区包含可直接运行的 SDF；需要从同一官方 Xacro 重生成时，先构建并加载本工作区：

```bash
source setup.bash
xacro install/go2_description/share/go2_description/urdf/go2.urdf.xacro > /tmp/go2-native.urdf
gz sdf -p /tmp/go2-native.urdf > /tmp/go2-native.sdf
python3 tools/generate_go2_model.py /tmp/go2-native.sdf
OPENBLAS_NUM_THREADS=1 python3 tools/check_go2_envelope.py "$PWD"
```

几何抽样覆盖 81000 个步态配置、25 个机体姿态和全部 27 个官方碰撞体，足迹另外预留超过 2 cm 的跟踪余量。这是有限域几何检查，不覆盖跌倒或任意外部关节指令。

`tools/plot_go2_run.py <运行目录>` 可绘制完整真实轨迹与倾角；绘图需要可选的 python3-matplotlib，导航运行不依赖它。

RViz 的 Views 面板已预设 `Maze overview` 与跟随 base_link 的 `Go2 close view`，可切换地图总览和关节姿态近景。

本机完整通关记录见 [2026-11-07 验收报告](../docs/validation/2026-11-07-go2-maze/README.md)。

## 前阶段通关基线

修正后从入口完整自主运行一轮，通过出口范围和持续停稳验收：939.334 s、119.122 m，出口距离 0.829765 m，在线验收零非法越界，55,271 条原生位姿完整重放零几何碰撞。当前仍使用 Gazebo 世界坐标里程计，未做多种子批量统计。

经验重点是模型/运动学/反馈一起适配，先实体标定再调导航时限，足迹覆盖后膝和小腿，以及用真实数据发现“出口格到达但实际停车仍在范围外”的情况。详见经验总结和原始验收报告。历史 Nav2 包的 README 已标明其适用范围。


## 本轮观察功能结论

三路真实图像在 RViz 独立窗格同时显示，默认二号 follow_yaw=false、世界坐标偏移 (-1.2,-0.9,+3.5) m；三号固定世界中心上空 24 m。背部亮绿黑边箭头随身体朝向机头。观察模型与标记不改变 Go2 物理参数，图像不输入导航。

最终设置完整通关并封存成功快照：918.932 s、117.936 m、出口距离 0.709254 m、线速度 0 m/s，54,050 条原生位姿严格重放通过，零几何碰撞与非法越界；最大总倾角 4.195°。阶段相机检查、完整数据及发布回归见 [本轮报告](../docs/validation/2026-11-07-go2-three-cameras/README.md)。
