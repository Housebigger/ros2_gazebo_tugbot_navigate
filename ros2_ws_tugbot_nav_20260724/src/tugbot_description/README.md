# tugbot_description

Tugbot 模型资源包。模型来源为仓库根目录的 `tugbot_without_plugins`，本包内副本已做最小 ROS 2 / Gazebo 导航适配：

- DiffDrive 使用已确认 joint：`wheel_left_joint` / `wheel_right_joint`；
- ROS-facing command topic：`/cmd_vel`；
- odom topic：`/odom`；
- odom TF：`odom -> base_link`；
- 2D Nav2 激光使用已确认 link/sensor：`scan_omni`，topic：`/scan`。
