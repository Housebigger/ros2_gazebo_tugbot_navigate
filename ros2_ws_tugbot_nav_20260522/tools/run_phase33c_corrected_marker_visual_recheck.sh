#!/usr/bin/env bash
set -eo pipefail

RUN_ID="phase33c_corrected_marker_visual_recheck"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ARTIFACT_DIR="log/${RUN_ID}"
LAUNCH_LOG="${ARTIFACT_DIR}/${RUN_ID}_launch.log"
RVIZ_LOG="${ARTIFACT_DIR}/${RUN_ID}_rviz.log"
mkdir -p "$ARTIFACT_DIR"

WORLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
SCAFFOLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world.sdf"
UNSCALED="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf"
META="$ROOT/src/tugbot_maze/config/maze_20260528_scaled_instance.yaml"
RVIZ_CONFIG="$ROOT/install/tugbot_bringup/share/tugbot_bringup/rviz/tugbot_nav.rviz"

if [[ ! -f "$WORLD" ]]; then
  echo "missing corrected scaled world: $WORLD" >&2
  exit 2
fi
if [[ "$WORLD" == "$SCAFFOLD" || "$WORLD" == "$UNSCALED" ]]; then
  echo "refusing to launch scaffold or unscaled clean world for Phase33C" >&2
  exit 3
fi

{
  echo "# Phase33C corrected marker visual-only checklist"
  echo "World: $WORLD"
  echo "Metadata: $META"
  echo "Guardrails: no autonomous navigation, no SLAM/Nav2 exploration, no maze_explorer, no promotion."
  echo "Expected Tugbot initial pose: (-11.011281337, -9.025069638, yaw=0)"
  echo "Expected entrance arrow: (-10.661281337, -9.025069638), direction +X into maze, visual-only/no collision"
  echo "Expected exit finish band: (10.061281337, 9.058495822), visual-only/no collision"
  echo "Expected exit marker: (10.061281337, 9.058495822), visual-only/no collision"
  echo "Expected wall_count: 53"
  echo "Manual observation checklist:"
  echo "- 入口是否正确；"
  echo "- 出口是否正确；"
  echo "- 右上 opening 是否确实是希望的终点；"
  echo "- 箭头和终点带是否贴地、无碰撞、可见；"
  echo "- Tugbot 是否摆在入口外侧且朝向入口；"
  echo "- 是否可进入下一步 SLAM/map smoke。"
  echo "Important: RViz is for TF/RobotModel/topic viewing only; do not send navigation goals."
} > "$ARTIFACT_DIR/${RUN_ID}_manual_checklist.txt"

{
  echo "=== timestamp ==="
  date -Is
  echo "=== DISPLAY ==="
  printf '%s\n' "${DISPLAY:-}"
  echo "=== hashes before launch ==="
  sha256sum "$SCAFFOLD" "$UNSCALED" "$WORLD"
  echo "=== corrected SDF parse ==="
  python3 - <<'PY'
import xml.etree.ElementTree as ET
from pathlib import Path
p=Path('src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf')
root=ET.parse(p).getroot(); world=root.find('.//world')
models={m.get('name'):m for m in world.findall('model')}
print('world_name', world.get('name'))
print('wall_count', sum(1 for n in models if n and n.startswith('maze_wall')))
for name in ['maze_entrance_arrow_visual','maze_exit_finish_band_visual','maze_exit_marker']:
    m=models.get(name)
    print(name, 'present', m is not None, 'pose', (m.findtext('pose') if m is not None else None), 'collisions', (len(m.findall('.//collision')) if m is not None else None))
for inc in world.findall('include'):
    if (inc.findtext('uri') or '').strip() == 'model://tugbot':
        print('tugbot_pose', inc.findtext('pose'))
PY
} > "$ARTIFACT_DIR/${RUN_ID}_preflight.txt"

set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u

ros2 launch tugbot_gazebo tugbot_gazebo.launch.py \
  world_sdf:="$WORLD" \
  headless:=false \
  use_sim_time:=true \
  > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$ARTIFACT_DIR/${RUN_ID}_launch.pid"

sleep 12

if [[ -f "$RVIZ_CONFIG" ]]; then
  rviz2 -d "$RVIZ_CONFIG" > "$RVIZ_LOG" 2>&1 &
  RVIZ_PID=$!
  echo "$RVIZ_PID" > "$ARTIFACT_DIR/${RUN_ID}_rviz.pid"
fi

sleep 6

{
  echo "=== timestamp ==="
  date -Is
  echo "=== ROS nodes ==="
  ros2 node list || true
  echo "=== ROS topics ==="
  ros2 topic list || true
  echo "=== forbidden ROS nodes grep ==="
  ros2 node list | grep -E 'maze_explorer|controller_server|planner_server|bt_navigator|slam_toolbox|map_server|amcl|behavior_server|waypoint_follower|velocity_smoother|smoother_server' || true
} > "$ARTIFACT_DIR/${RUN_ID}_ros_graph.txt"

{
  echo "=== gz topics ==="
  gz topic -l || true
  echo "=== gz services ==="
  gz service -l || true
} > "$ARTIFACT_DIR/${RUN_ID}_gz_graph.txt"

pgrep -af "ros2 launch|gz sim|rviz2|maze_explorer|controller_server|planner_server|bt_navigator|slam_toolbox|ros_gz_bridge|parameter_bridge|static_transform_publisher" > "$ARTIFACT_DIR/${RUN_ID}_processes.txt" || true

if command -v xwd >/dev/null 2>&1; then
  timeout 10 xwd -root -silent -out "$ARTIFACT_DIR/${RUN_ID}_display_root.xwd" || true
fi

cat <<EOF
Phase33C visual-only run started.
launch_pid=$LAUNCH_PID
artifact_dir=$ARTIFACT_DIR
world=$WORLD
Do not send navigation goals. Observe Gazebo/RViz and report human acceptance.
EOF
