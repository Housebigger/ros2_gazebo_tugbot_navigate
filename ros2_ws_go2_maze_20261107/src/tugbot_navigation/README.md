# tugbot_navigation：历史 Nav2 资源

[当前包索引](../README.md) · [Go2 启动入口](../go2_maze_bringup/README.md)

此包保留原 Tugbot 的 Nav2 参数、地图与导航资源，不参与当前 Go2 聚焦启动。原参数包括 base_link / odom / map、`/odom`、`/scan` 和轮式 `robot_radius=0.35`，该半径不能替代 Go2 后膝与动态步态足迹。

当前 Go2 使用在线 flood-fill、原生里程计和连续步态，不启动此包的 Nav2 或 slam_toolbox。需要继续做旧 Nav2 实验时，参照 [20260728 历史工作区](../../../ros2_ws_tugbot_nav_20260728/README.md) 与对应地图/参数；迁移到 Go2 前重新验证足迹、速度、TF 与控制接口。
