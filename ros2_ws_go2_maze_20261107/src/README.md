# Go2 工作区包索引

[工作区运行说明](../README.md) · [成功经验](../../docs/experience/2026-11-07-go2-maze-success.md)

当前聚焦启动只需构建以下五个包：

| 包 | 职责 |
| --- | --- |
| [go2_description](go2_description/README.md) | 官方 Go2 Xacro、DAE、许可与来源 |
| [go2_maze_bringup](go2_maze_bringup/README.md) | 当前 Gazebo / ROS / RViz / 自主导航启动 |
| [tugbot_description](tugbot_description/README.md) | 原生 Go2 Gazebo SDF、关节插件与 sensor |
| [tugbot_gazebo](tugbot_gazebo/README.md) | 当前迷宫、Go2 平地验证 world |
| [tugbot_maze](tugbot_maze/README.md) | 连续步态、在线求解、出口收尾与独立验收 |

`tugbot_*` 名称沿用原项目包接口，当前启动的机器人是 Go2。保留的 [tugbot_navigation](tugbot_navigation/README.md)、[tugbot_bringup](tugbot_bringup/README.md)、`tugbot_exploration` 属于原 Nav2 / SLAM 入口资源，不在 Go2 聚焦启动链中。不要把它们与当前启动叠加，造成重复 TF 或多个速度发布者。
