# Bridge 与 RViz 配置

[包说明](../README.md)

[bridge.yaml](bridge.yaml) 定义当前 Go2 所用数据流：

| ROS topic | Gazebo 来源 / 方向 | 用途 |
| --- | --- | --- |
| /clock | /clock → ROS | 仿真时钟 |
| /odom、/tf | 原生 OdometryPublisher → ROS | 三维位姿及 odom→base_link |
| /joint_states | /model/go2/joint_state → ROS | 12 关节真实反馈 |
| /lidar/points | /lidar/points/points → ROS | 16×1800 点云；注意实际 gz 后缀 |
| /camera/front/image、/camera/front/camera_info | 同名 → ROS | 前相机与内参 |
| /gt/dynamic_pose | /world/tugbot_maze_world_20260528_clean_scaled2x/dynamic_pose/info → ROS | 仅独立验收 |
| /model/go2/joint/<关节>/cmd_pos | ROS → gz Double | 12 个关节目标 |

关节名使用 LF/RF/LH/RH 与 HAA/HFE/KFE，不在 ROS topic 中使用数字 `/0` 关节索引段。原生插件使用显式的对应命令 topic。

点云投影为 `/scan`，frame 保留 `go2/base_link/lidar_3d`；安装 TF 平移为 (0,0,0.25)。相机 frame 为 `go2/base_link/camera_front`，平移为 (0.35,0,0.05)。投影、在线定位和格边感知采用居中扫描约定。

[maze.rviz](maze.rviz) 使用固定 frame `map`，显示官方 RobotModel、实时扫描、`/maze/cloud_map_3d` 与 `/maze/go2_actual_path`。后两者分别是在线累积点云与独立验收记录的真实轨迹；轨迹不作为导航控制输入。Views 中保存 Maze overview 和跟随 base_link 的 Go2 close view。

显示红色状态时先核对 `/clock`、sensor header、TF 时间戳、domain/partition 和 robot_description，而不是把 Fixed Frame 随意改成 world 以掩盖缺失链路。
