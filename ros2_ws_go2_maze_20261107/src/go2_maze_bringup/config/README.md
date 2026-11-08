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

## 三相机扩展

`observer_cameras:=true` 默认选择 [three_cameras.rviz](three_cameras.rviz)：保留上述地图、关节和轨迹显示，额外开启三个独立 Image dock。配置保存默认布局，重载时右侧两个图像窗格保持可见。Image 使用 Best Effort、深度 1，避免积压旧帧。

[observer_bridge.yaml](observer_bridge.yaml) 将 `/camera/chase/image`、`/camera/overview/image` 及各自 `camera_info` 从 Gazebo 桥接到 ROS；图像/内参均懒订阅。原生跟随插件发布的 `/camera/chase/tf` 转到 `/tf`，frame 为 `odom→camera_chase_optical`；固定俯视 TF 为 `odom→camera_overview_optical`，位置 (0,0,24)、四元数 xyzw=(1,0,0,0)。额外相机图像与 CameraInfo 均直接标记 optical frame：+Z 前、+X 右、+Y 下。

二号使用 ECS 中实际生效的相机位姿发布 TF，不把期望目标位姿当成测量值。观察 TF 当前依赖本工程 `odom` 与 Gazebo 世界原点重合的设计；未来更改为入口原点或实机里程计时，要一起修改世界到 odom 的变换。

实时检查器同时核对图像和跟随 TF 的完整观测窗口与末尾时效；不能仅凭历史图像/TF 帧数判断直播正常。最终默认二号固定世界方向，背部箭头仍随 base_link 转向。
