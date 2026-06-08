#!/usr/bin/env bash
set -eo pipefail

RUN_ID="phase32_clean_world_visual_acceptance"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ARTIFACT_DIR="log/${RUN_ID}"
LAUNCH_LOG="${ARTIFACT_DIR}/${RUN_ID}_launch.log"
ROS_NODES_LOG="${ARTIFACT_DIR}/${RUN_ID}_ros_nodes.txt"
ROS_TOPICS_LOG="${ARTIFACT_DIR}/${RUN_ID}_ros_topics.txt"
GZ_TOPICS_LOG="${ARTIFACT_DIR}/${RUN_ID}_gz_topics.txt"
GZ_SERVICES_LOG="${ARTIFACT_DIR}/${RUN_ID}_gz_services.txt"
PROCESS_LOG="${ARTIFACT_DIR}/${RUN_ID}_processes.txt"
CHECKLIST="${ARTIFACT_DIR}/${RUN_ID}_manual_checklist.txt"
WORLD_SDF="${ROOT}/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf"
BASELINE_WORLD="${ROOT}/src/tugbot_gazebo/worlds/tugbot_maze_world.sdf"
PHASE29_WORLD="${ROOT}/src/tugbot_gazebo/worlds/tugbot_maze_world_image_faithful.sdf"
RVIZ_CONFIG="${ROOT}/install/tugbot_bringup/share/tugbot_bringup/rviz/tugbot_nav.rviz"

mkdir -p "$ARTIFACT_DIR"

if [[ ! -f "$WORLD_SDF" ]]; then
  echo "clean candidate world missing: $WORLD_SDF" >&2
  exit 1
fi
if [[ "$WORLD_SDF" == "$BASELINE_WORLD" ]]; then
  echo "refusing to use baseline scaffold as candidate world" >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f install/setup.bash ]]; then
  source install/setup.bash
fi
set -u

cat > "$CHECKLIST" <<'TXT'
Phase32 clean 2D candidate world visual acceptance checklist

Observe Gazebo/RViz only. Do not send navigation goals.

Check:
- 迷宫整体是否贴合 maze_20260528.png；
- 是否有明显漏墙、错墙、重叠墙；
- 入口/出口是否符合人的预期；
- Tugbot 是否能放进入口；
- 通道宽度是否足够 Tugbot 通过；
- 墙高、墙厚、尺度是否合理；
- 是否需要修改 maze_wall_segments_20260528.yaml；
- 是否可进入下一阶段 clean world map/SLAM smoke。

Candidate clean world:
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf

Entrance candidate:
(-6.0, -1.0, yaw=0)

Exit marker:
(0.0, 6.0), radius=0.6

Guardrails:
- no maze_explorer
- no SLAM
- no Nav2 autonomous exploration
- no candidate promotion
- no scaffold overwrite
TXT

{
  echo "Phase32 visual-only clean candidate world run"
  date -Is
  echo "ROOT=$ROOT"
  echo "WORLD_SDF=$WORLD_SDF"
  echo "BASELINE_WORLD=$BASELINE_WORLD"
  echo "PHASE29_WORLD=$PHASE29_WORLD"
  echo "DISPLAY=${DISPLAY:-}"
  sha256sum "$BASELINE_WORLD" "$WORLD_SDF" "$PHASE29_WORLD"
  echo "Command: ros2 launch tugbot_gazebo tugbot_gazebo.launch.py world_sdf:=$WORLD_SDF headless:=false use_sim_time:=true"
} > "$LAUNCH_LOG"

# Gazebo-only launch: starts visible Gazebo, ros_gz_bridge, and static camera/scan TFs.
# It does not start Nav2, slam_toolbox, maze_explorer, or any autonomous node.
ros2 launch tugbot_gazebo tugbot_gazebo.launch.py \
  world_sdf:="$WORLD_SDF" \
  headless:=false \
  use_sim_time:=true >> "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "${ARTIFACT_DIR}/${RUN_ID}_launch.pid"

sleep 8
if command -v rviz2 >/dev/null 2>&1 && [[ -f "$RVIZ_CONFIG" ]]; then
  rviz2 -d "$RVIZ_CONFIG" >> "${ARTIFACT_DIR}/${RUN_ID}_rviz.log" 2>&1 &
  echo $! > "${ARTIFACT_DIR}/${RUN_ID}_rviz.pid"
fi

sleep 4
ros2 node list > "$ROS_NODES_LOG" 2>&1 || true
ros2 topic list > "$ROS_TOPICS_LOG" 2>&1 || true
gz topic -l > "$GZ_TOPICS_LOG" 2>&1 || true
gz service -l > "$GZ_SERVICES_LOG" 2>&1 || true
pgrep -af 'ros2 launch tugbot_gazebo|gz sim|rviz2|ros_gz_bridge|maze_explorer|controller_server|planner_server|bt_navigator|slam_toolbox|nav2|maze_goal_monitor|static_transform_publisher' > "$PROCESS_LOG" || true
{
  echo
  echo "Post-start snapshots"
  echo "ROS nodes:"; cat "$ROS_NODES_LOG"
  echo "ROS topics:"; cat "$ROS_TOPICS_LOG"
  echo "GZ topics:"; cat "$GZ_TOPICS_LOG"
  echo "GZ services:"; cat "$GZ_SERVICES_LOG"
  echo "Processes:"; cat "$PROCESS_LOG"
} >> "$LAUNCH_LOG"

echo "Phase32 clean world visual-only run started"
echo "artifact_dir=$ARTIFACT_DIR"
echo "launch_pid=$LAUNCH_PID"
echo "launch_log=$LAUNCH_LOG"
echo "manual_checklist=$CHECKLIST"
