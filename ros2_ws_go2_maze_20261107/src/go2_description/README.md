# go2_description

[包索引](../README.md) · [原生 Gazebo 模型](../tugbot_description/README.md)

官方 Unitree Go2 的 Xacro、七个 Collada mesh 与 BSD-3-Clause 许可。固定来源提交为 `41dec58c7a3ccb3050243ef16ac2ca74b2b0f9ca`；原文件校验值、改名范围和来源见 [SOURCE.md](SOURCE.md)，许可见 [LICENSE](LICENSE)。

- `urdf/go2.urdf.xacro`：robot_state_publisher 使用的视觉/运动学描述，mesh 路径通过 ament 解析。
- `dae/`：官方材质与形状。
- 原始关节名为 `fl/fr/rl/rr_hip/thigh/knee`；当前 launch 将它们同步映射为 `LF/RF/LH/RH_HAA/HFE/KFE`，与 Gazebo 原生关节反馈保持一致。

换名时同步 frame 引用，不能只换显示名称。惯量、轴和限位保持官方定义。当前 Go2 FK/IK 以该 URDF 为依据，并由独立矩阵链检查对照。

在工作区构建加载后查看描述：

```bash
source setup.bash
xacro install/go2_description/share/go2_description/urdf/go2.urdf.xacro
```

这份 Xacro 本身不承载本轮原生仿真插件；控制/传感器插件在 `tugbot_description/models/go2/model.sdf`，统一启动在 `go2_maze_bringup`。
