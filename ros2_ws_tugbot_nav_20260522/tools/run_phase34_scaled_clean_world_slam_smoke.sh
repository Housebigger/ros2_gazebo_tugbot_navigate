#!/usr/bin/env bash
set -eo pipefail

RUN_ID="phase34_scaled_clean_world_slam_smoke"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ARTIFACT_DIR="log/${RUN_ID}"
LAUNCH_LOG="${ARTIFACT_DIR}/${RUN_ID}_launch.log"
RVIZ_LOG="${ARTIFACT_DIR}/${RUN_ID}_rviz.log"
MAP_PREFIX="${ARTIFACT_DIR}/${RUN_ID}_map_snapshot"
WORLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
SCAFFOLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world.sdf"
UNSCALED="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
RVIZ_CONFIG="$ROOT/install/tugbot_bringup/share/tugbot_bringup/rviz/tugbot_nav.rviz"
DURATION_SEC="${PHASE34_DURATION_SEC:-45}"
mkdir -p "$ARTIFACT_DIR"

if [[ ! -f "$WORLD" ]]; then
  echo "missing visually accepted scaled world: $WORLD" >&2
  exit 2
fi
if [[ "$WORLD" == "$SCAFFOLD" || "$WORLD" == "$UNSCALED" ]]; then
  echo "refusing to launch scaffold or unscaled clean world for Phase34" >&2
  exit 3
fi
if [[ ! -f "$SLAM_PARAMS" ]]; then
  echo "missing slam params: $SLAM_PARAMS" >&2
  exit 4
fi

{
  echo "# Phase34 Scaled Clean World SLAM / Map Smoke"
  echo "World: $WORLD"
  echo "SLAM params: $SLAM_PARAMS"
  echo "Duration: ${DURATION_SEC}s"
  echo "Guardrails: no autonomous navigation, no maze_explorer, no Nav2 tuning, no world promotion."
  echo "Manual checklist:"
  echo "- RViz map 是否与 Gazebo 墙体方向一致；"
  echo "- 墙体是否大量缺失、重影、错位；"
  echo "- 起点附近 TF/RobotModel 是否正确；"
  echo "- LaserScan 是否能扫到墙；"
  echo "- 不基于本阶段声称导航成功。"
} > "$ARTIFACT_DIR/${RUN_ID}_manual_checklist.txt"

{
  echo "=== timestamp ==="
  date -Is
  echo "=== DISPLAY ==="
  printf '%s\n' "${DISPLAY:-}"
  echo "=== hashes before launch ==="
  sha256sum "$SCAFFOLD" "$UNSCALED" "$WORLD" "$SLAM_PARAMS"
  echo "=== launch contract ==="
  echo "ros2 launch tugbot_bringup tugbot_maze_slam.launch.py world_sdf:=$WORLD headless:=false use_rviz:=true use_sim_time:=true autostart:=true"
  echo "Explicitly not launching maze_explorer/tugbot_maze_explore/Nav2 navigation_launch."
} > "$ARTIFACT_DIR/${RUN_ID}_preflight.txt"

set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u

ros2 launch tugbot_bringup tugbot_maze_slam.launch.py \
  world_sdf:="$WORLD" \
  slam_params_file:="$SLAM_PARAMS" \
  headless:=false \
  use_rviz:=true \
  use_sim_time:=true \
  autostart:=true \
  > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$ARTIFACT_DIR/${RUN_ID}_launch.pid"

sleep 18

# Snapshot live graph/data early, while the visual run remains active.
{
  echo "=== timestamp ==="
  date -Is
  echo "=== ROS nodes ==="
  ros2 node list || true
  echo "=== ROS topics ==="
  ros2 topic list || true
  echo "=== forbidden ROS/Nav2/explorer nodes grep ==="
  ros2 node list | grep -E 'maze_explorer|frontier_explorer|controller_server|planner_server|bt_navigator|map_server|amcl|behavior_server|waypoint_follower|velocity_smoother|smoother_server' || true
} > "$ARTIFACT_DIR/${RUN_ID}_ros_graph_initial.txt"

{
  echo "=== gz topics ==="
  gz topic -l || true
  echo "=== gz services ==="
  gz service -l || true
} > "$ARTIFACT_DIR/${RUN_ID}_gz_graph_initial.txt"

# Record one-shot topic data. Keep files bounded.
(timeout 10 ros2 topic echo --once /map || true) > "$ARTIFACT_DIR/${RUN_ID}_map_once.txt" 2>&1
(timeout 10 ros2 topic echo --once /map_metadata || true) > "$ARTIFACT_DIR/${RUN_ID}_map_metadata_once.txt" 2>&1
(timeout 10 ros2 topic echo --once /scan || true) > "$ARTIFACT_DIR/${RUN_ID}_scan_once.txt" 2>&1
(timeout 10 ros2 topic echo --once /odom || true) > "$ARTIFACT_DIR/${RUN_ID}_odom_once.txt" 2>&1
(timeout 10 ros2 topic echo --once /tf || true) > "$ARTIFACT_DIR/${RUN_ID}_tf_once.txt" 2>&1
(timeout 10 ros2 topic hz /scan || true) > "$ARTIFACT_DIR/${RUN_ID}_scan_hz.txt" 2>&1
(timeout 10 ros2 topic hz /map || true) > "$ARTIFACT_DIR/${RUN_ID}_map_hz.txt" 2>&1
(timeout 10 ros2 lifecycle get /slam_toolbox || true) > "$ARTIFACT_DIR/${RUN_ID}_slam_lifecycle.txt" 2>&1

# Save map snapshot if map_saver_cli is available and /map has appeared.
if command -v ros2 >/dev/null 2>&1; then
  timeout 20 ros2 run nav2_map_server map_saver_cli -f "$MAP_PREFIX" > "$ARTIFACT_DIR/${RUN_ID}_map_saver.log" 2>&1 || true
fi

# Capture display after RViz has had time to render /map.
if command -v xwd >/dev/null 2>&1; then
  timeout 10 xwd -root -silent -out "$ARTIFACT_DIR/${RUN_ID}_display_root.xwd" || true
fi

sleep "$DURATION_SEC"

pgrep -af "ros2 launch|gz sim|rviz2|slam_toolbox|maze_explorer|frontier_explorer|controller_server|planner_server|bt_navigator|ros_gz_bridge|parameter_bridge|static_transform_publisher" > "$ARTIFACT_DIR/${RUN_ID}_processes.txt" || true

cat <<EOF
Phase34 SLAM/map visual smoke started and bounded data captured.
launch_pid=$LAUNCH_PID
artifact_dir=$ARTIFACT_DIR
world=$WORLD
Do not send navigation goals. Observe Gazebo/RViz map and report human acceptance.
EOF
