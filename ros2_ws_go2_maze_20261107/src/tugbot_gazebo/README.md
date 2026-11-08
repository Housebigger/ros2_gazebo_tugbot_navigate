# tugbot_gazebo：Go2 验证场景

[包索引](../README.md) · [启动参数](../go2_maze_bringup/README.md)

| 世界 | 用途 |
| --- | --- |
| [go2_maze.sdf](worlds/go2_maze.sdf) | 当前默认：原 20260528 clean_scaled2x 迷宫几何，替换为 Go2 |
| [go2_test.sdf](worlds/go2_test.sdf) | 站立、直行、转向与失联停车的平地阶梯 |
| 其余 worlds | 从历史工作区继承，不是当前 Go2 默认入口 |

迷宫保持 2 m 栅格、约 1.76 m 净走廊、0.24 m 墙厚和 1.2 m 墙高，仿真步长 1 ms。Go2 在 world (-11.011281,-9.025070,0.46) 生成，已知入口平移将其对应到 map (0,0)。出口验收中心为 map (21.072562,18.083566)，半径 1.2 m。

世界名称保留 `tugbot_maze_world_20260528_clean_scaled2x`，因此原生动态位姿 topic 中也保留此名称。修改 world 名称时必须同步独立验收 bridge。原生模型与 sensor 资源路径由当前 launch 设置。

构建并加载工作区后，平地仅站立启动：

```bash
ros2 launch go2_maze_bringup maze.launch.py autonomous:=false \
  world:="$PWD/install/tugbot_gazebo/share/tugbot_gazebo/worlds/go2_test.sdf"
```

`$PWD` 应为工作区根目录。不要用历史 Tugbot/ANYmal 的 bringup 代替当前入口；它们的底盘控制和传感器契约不同。

观察相机由 go2_maze_bringup 按 observer_cameras 开关独立生成；world 几何不因观察功能改变。三号相机对应世界 (0,0,24) m，以北向上的图像覆盖全部迷宫墙角；两路观察模型静态、无碰撞，不进入 Go2 动态顶层真值流。
