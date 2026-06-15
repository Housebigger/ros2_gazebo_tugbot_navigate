#!/usr/bin/env bash
set -eo pipefail

RUN_ID="phase35_scaled_clean_world_nav2_manual_goal_smoke"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ARTIFACT_DIR="log/${RUN_ID}"
LAUNCH_LOG="${ARTIFACT_DIR}/${RUN_ID}_launch.log"
WORLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
NAV2_PARAMS="$ROOT/src/tugbot_navigation/config/nav2_slam_params.yaml"
RVIZ_CONFIG="$ROOT/install/tugbot_bringup/share/tugbot_bringup/rviz/tugbot_nav.rviz"
SCAFFOLD_ARCHIVE="$ROOT/archive/deprecated_maze_worlds/worlds/manual_simplified_first_pass_scaffold/tugbot_maze_world.sdf"
UNSCALED_ARCHIVE="$ROOT/archive/deprecated_maze_worlds/worlds/unscaled_clean_candidate/tugbot_maze_world_20260528_clean.sdf"
SNAPSHOT_DELAY_SEC="${PHASE35_SNAPSHOT_DELAY_SEC:-35}"
mkdir -p "$ARTIFACT_DIR"

if [[ ! -f "$WORLD" ]]; then
  echo "missing active scaled clean world: $WORLD" >&2
  exit 2
fi
if [[ "$WORLD" != *"tugbot_maze_world_20260528_clean_scaled2x.sdf" ]]; then
  echo "refusing to launch a non-active Phase35 world: $WORLD" >&2
  exit 3
fi
if [[ ! -f "$SLAM_PARAMS" ]]; then
  echo "missing slam params: $SLAM_PARAMS" >&2
  exit 4
fi
if [[ ! -f "$NAV2_PARAMS" ]]; then
  echo "missing Nav2 params: $NAV2_PARAMS" >&2
  exit 5
fi

{
  echo "# Phase35 Scaled Clean World Nav2 Manual Goal Smoke"
  echo "World: $WORLD"
  echo "SLAM params: $SLAM_PARAMS"
  echo "Nav2 params: $NAV2_PARAMS"
  echo "RViz config: $RVIZ_CONFIG"
  echo "Guardrails: no maze_explorer, no autonomous exploration, no Nav2/MPPI/controller tuning, no fallback/terminal acceptance continuation, no old world/map."
  echo "Manual goal checklist for human RViz observation:"
  echo "1. Confirm RViz fixed frame/map, RobotModel, LaserScan, global costmap, local costmap are visible."
  echo "2. Send only a few short Nav2 Goal poses from RViz inside nearby corridors."
  echo "3. Observe whether a global plan appears and whether the local costmap/footprint avoid walls."
  echo "4. Observe whether Tugbot can pass representative corridor turns/openings."
  echo "5. Do not treat this as autonomous exploration or exit-success validation."
} > "$ARTIFACT_DIR/${RUN_ID}_manual_checklist.txt"

{
  echo "=== timestamp ==="
  date -Is
  echo "=== DISPLAY ==="
  printf '%s\n' "${DISPLAY:-}"
  echo "=== explicit launch contract ==="
  echo "ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py world_sdf:=$WORLD slam_params_file:=$SLAM_PARAMS params_file:=$NAV2_PARAMS headless:=false use_rviz:=true use_sim_time:=true autostart:=true"
  echo "Explicitly not launching tugbot_maze_explore.launch.py or maze_explorer."
  echo "=== hashes before launch ==="
  sha256sum "$WORLD" "$SLAM_PARAMS" "$NAV2_PARAMS"
  if [[ -f "$SCAFFOLD_ARCHIVE" ]]; then sha256sum "$SCAFFOLD_ARCHIVE"; fi
  if [[ -f "$UNSCALED_ARCHIVE" ]]; then sha256sum "$UNSCALED_ARCHIVE"; fi
  echo "=== nav config diff guard ==="
  git diff -- src/tugbot_navigation/config || true
  echo "=== current maze launch defaults grep ==="
  grep -R "tugbot_maze_world" -n src/tugbot_bringup/launch/tugbot_maze_slam.launch.py src/tugbot_bringup/launch/tugbot_maze_slam_nav.launch.py src/tugbot_bringup/launch/tugbot_maze_explore.launch.py || true
} > "$ARTIFACT_DIR/${RUN_ID}_preflight.txt"

set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u

ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py \
  world_sdf:="$WORLD" \
  slam_params_file:="$SLAM_PARAMS" \
  params_file:="$NAV2_PARAMS" \
  headless:=false \
  use_rviz:=true \
  use_sim_time:=true \
  autostart:=true \
  > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$ARTIFACT_DIR/${RUN_ID}_launch.pid"

sleep "$SNAPSHOT_DELAY_SEC"

{
  echo "=== timestamp ==="
  date -Is
  echo "=== ROS nodes ==="
  ros2 node list || true
  echo "=== ROS topics ==="
  ros2 topic list || true
  echo "=== forbidden explorer nodes grep ==="
  ros2 node list | grep -E 'maze_explorer|frontier_explorer' || true
  echo "=== expected Nav2 nodes grep ==="
  ros2 node list | grep -E 'controller_server|planner_server|bt_navigator|behavior_server|waypoint_follower|velocity_smoother|smoother_server|slam_toolbox|rviz|ros_gz_bridge' || true
} > "$ARTIFACT_DIR/${RUN_ID}_ros_graph_initial.txt"

{
  echo "=== gz topics ==="
  gz topic -l || true
  echo "=== gz services ==="
  gz service -l || true
} > "$ARTIFACT_DIR/${RUN_ID}_gz_graph_initial.txt"

(timeout 10 ros2 topic echo --once /map || true) > "$ARTIFACT_DIR/${RUN_ID}_map_once.txt" 2>&1
(timeout 10 ros2 topic echo --once /map_metadata || true) > "$ARTIFACT_DIR/${RUN_ID}_map_metadata_once.txt" 2>&1
(timeout 10 ros2 topic echo --once /scan || true) > "$ARTIFACT_DIR/${RUN_ID}_scan_once.txt" 2>&1
(timeout 10 ros2 topic echo --once /odom || true) > "$ARTIFACT_DIR/${RUN_ID}_odom_once.txt" 2>&1
(timeout 10 ros2 topic echo --once /tf || true) > "$ARTIFACT_DIR/${RUN_ID}_tf_once.txt" 2>&1
(timeout 10 ros2 topic echo --once /local_costmap/costmap || true) > "$ARTIFACT_DIR/${RUN_ID}_local_costmap_once.txt" 2>&1
(timeout 10 ros2 topic echo --once /global_costmap/costmap || true) > "$ARTIFACT_DIR/${RUN_ID}_global_costmap_once.txt" 2>&1
(timeout 10 ros2 topic hz /scan || true) > "$ARTIFACT_DIR/${RUN_ID}_scan_hz.txt" 2>&1
(timeout 10 ros2 topic hz /map || true) > "$ARTIFACT_DIR/${RUN_ID}_map_hz.txt" 2>&1
(timeout 10 ros2 lifecycle get /slam_toolbox || true) > "$ARTIFACT_DIR/${RUN_ID}_slam_lifecycle.txt" 2>&1
for node in /controller_server /planner_server /bt_navigator /behavior_server /waypoint_follower /velocity_smoother /smoother_server; do
  safe="${node#/}"
  (timeout 8 ros2 lifecycle get "$node" || true) > "$ARTIFACT_DIR/${RUN_ID}_${safe}_lifecycle.txt" 2>&1
done

if command -v xwd >/dev/null 2>&1; then
  timeout 10 xwd -root -silent -out "$ARTIFACT_DIR/${RUN_ID}_display_root_initial.xwd" || true
fi

pgrep -af "ros2 launch|gz sim|rviz2|slam_toolbox|maze_explorer|frontier_explorer|controller_server|planner_server|bt_navigator|behavior_server|waypoint_follower|velocity_smoother|smoother_server|ros_gz_bridge|parameter_bridge|static_transform_publisher" > "$ARTIFACT_DIR/${RUN_ID}_processes_initial.txt" || true

cat <<EOF
Phase35 SLAM+Nav2+RViz manual-goal smoke is running.
launch_pid=$LAUNCH_PID
artifact_dir=$ARTIFACT_DIR
world=$WORLD
Guardrail reminder: use RViz manual Nav2 Goal only; do not launch maze_explorer/autonomous exploration; do not tune Nav2/MPPI/controller params.
EOF
