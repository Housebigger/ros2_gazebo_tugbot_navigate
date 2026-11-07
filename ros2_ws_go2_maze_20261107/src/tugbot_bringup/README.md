# tugbot_bringup：保留的历史启动入口

[当前包索引](../README.md) · [Go2 启动入口](../go2_maze_bringup/README.md)

此目录继承原 Tugbot / Nav2 / SLAM 阶段 launch，用于保留项目演进上下文。当前 Go2 统一入口为 `go2_maze_bringup/maze.launch.py`，工作区的两个运行脚本都调用它。

不要同时运行历史 bringup 与当前 Go2 launch；同名 `/cmd_vel`、`/tf`、`/scan` 或多套定位器会破坏联调。历史全局 pkill 与清理共享内存的脚本也不用于本轮。
