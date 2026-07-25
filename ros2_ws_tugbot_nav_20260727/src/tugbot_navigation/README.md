# tugbot_navigation

Nav2 参数、地图与导航资源包。`config/nav2_params.yaml` 来源于 `project_reference/ros2_ws8/src/nav_car/param/param_nav2.yaml`，只做 Tugbot 最小适配：

- `base_frame_id = base_link`
- `odom_frame_id = odom`
- `global_frame_id = map`
- `odom_topic = /odom`
- scan topic = `/scan`
- 初始 `robot_radius = 0.35`
