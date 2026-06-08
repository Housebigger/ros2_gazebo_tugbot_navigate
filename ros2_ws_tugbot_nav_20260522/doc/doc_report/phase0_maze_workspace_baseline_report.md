# Phase 0: 20260522 Maze Workspace Baseline and Asset Inventory

## Conclusion

`PASS_FOR_FRAMEWORK_BASELINE`

本阶段已创建 `ros2_ws_tugbot_nav_20260522` 基础工程框架。该工作空间从 `ros2_ws_tugbot_nav_20260514` 复制有用源码演进而来，并排除了 `build/`、`install/`、`log/` 等生成产物。

## Source Workspace

```text
ros2_ws_tugbot_nav_20260514
```

继承内容：

- `tugbot_description`
- `tugbot_gazebo`
- `tugbot_navigation`
- `tugbot_bringup`
- `tugbot_exploration`
- 0514 的 Nav2 / SLAM / frontier / perimeter / no-spin 参数与报告资料

## Target Workspace

```text
ros2_ws_tugbot_nav_20260522
```

0522 的目标是：

```text
Tugbot 从迷宫入口出发，在未知迷宫中自主探索，并最终从出口驶出。
```

## Maze Image Inventory

用户提供的迷宫参考图已归档：

```text
src/tugbot_maze/assets/maze_20260522.jpg
```

原始来源：

```text
tmp_resources/maze_20260522.jpg
```

已检查属性：

```text
format: JPEG
mode: RGB
dimensions: 1000 x 1000
```

该图片是 3D / 浮雕风格迷宫示意图，包含阴影、渐变和水印，不是干净黑白 occupancy map。因此当前不直接自动二值化为最终 Gazebo world。

## Maze Package

新增包：

```text
src/tugbot_maze
```

当前包含：

```text
package.xml
setup.py
setup.cfg
resource/tugbot_maze
tugbot_maze/__init__.py
tugbot_maze/maze_goal_monitor.py
tugbot_maze/maze_image_to_world.py
tugbot_maze/maze_explorer.py
tugbot_maze/maze_utils.py
config/maze_instance.yaml
assets/maze_20260522.jpg
test/test_maze_contract.py
```

## Maze World

新增第一版手写简化迷宫 world：

```text
src/tugbot_gazebo/worlds/tugbot_maze_world.sdf
```

说明：

- 该 world 是第一版工程 scaffold。
- 墙体为人工手写正交 box。
- Tugbot 初始 pose 在左下入口附近。
- 出口区域在右上附近，并添加了绿色 visual marker。
- 后续可用清理后的 occupancy extraction 替换该 scaffold。

## Launch Entrypoints

新增 maze launch：

```text
src/tugbot_bringup/launch/tugbot_maze_slam.launch.py
src/tugbot_bringup/launch/tugbot_maze_slam_nav.launch.py
src/tugbot_bringup/launch/tugbot_maze_explore.launch.py
```

当前自主探索入口仍使用 0514 继承的 `frontier_explorer`，并额外启动 `maze_goal_monitor`。

## Runtime Policy

`maze_instance.yaml` 明确记录：

```text
image_allowed_for_world_generation: true
image_allowed_for_exit_truth: true
image_allowed_for_runtime_path_planning: false
```

即：迷宫图片可以作为 world 生成和出口真值参考，但不能作为运行时全局路径答案。

## Success Token

出口检测节点在到达出口区域时输出：

```text
MAZE_EXIT_REACHED
```

## Next Verification

下一步应执行：

```bash
cd ros2_ws_tugbot_nav_20260522
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

然后运行 headless smoke：

```bash
ros2 launch tugbot_bringup tugbot_maze_slam.launch.py headless:=true use_rviz:=false
```
