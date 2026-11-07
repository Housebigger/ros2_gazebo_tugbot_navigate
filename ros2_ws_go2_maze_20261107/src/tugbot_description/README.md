# tugbot_description：当前 Go2 原生仿真资源

[包索引](../README.md) · [启动](../go2_maze_bringup/README.md) · [官方视觉模型](../go2_description/README.md)

当前默认模型是 [models/go2/model.sdf](models/go2/model.sdf)，名称 `go2`。包名保留原项目接口；目录内的 Tugbot、ANYmal C、MR-Buggy3 是继承资源，当前 Go2 launch 不加载这些历史底盘。

## 模型与控制

- 官方模型保留惯量、关节原点/轴、碰撞体、限位和 DAE 材质，固定 link 经 SDF 转换合并后有 13 个物理 link、12 个关节。
- 每个关节使用原生 `JointPositionController` 的力矩 PID：P=320、D=4、I=0，力矩限制为原模型的 23.7 / 45.43 Nm。
- 命令 `/model/go2/joint/<LF_HAA 等>/cmd_pos`；原生关节反馈 `/model/go2/joint_state`，ROS 桥接到 `/joint_states`。
- `OdometryPublisher` 发布三维 `/odom` 与 `odom→base_link`。当前保留世界坐标里程计约定。
- 模型不通过基座外力或平移插件运动。Go2 步态由 `locomotion_controller` 计算目标、由足地接触推进身体。

## 感知安装

| 传感器 | base_link 下位置 (m) | 实际 frame |
| --- | --- | --- |
| 中置 16 线雷达 | (0,0,0.25) | go2/base_link/lidar_3d |
| 前相机 | (0.35,0,0.05) | go2/base_link/camera_front |

命令初始站立目标不等于 Gazebo 直接重置关节角。当前 world 用 z=0.46 m 的零配置落地收腿，避免以直腿零角在低高度生成而穿地。先完成站立，再开始自主运动。

[来源与适配](models/go2/SOURCE.md) · [许可证](models/go2/LICENSE)。重生成方法与有限步态足迹检查见 [工作区说明](../../README.md)，生成器在 `tools/generate_go2_model.py`。
