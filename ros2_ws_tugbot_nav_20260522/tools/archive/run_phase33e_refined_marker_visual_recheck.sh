#!/usr/bin/env bash
set -eo pipefail

RUN_ID="phase33e_refined_marker_visual_recheck"
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
  echo "missing refined scaled world: $WORLD" >&2
  exit 2
fi
if [[ "$WORLD" == "$SCAFFOLD" || "$WORLD" == "$UNSCALED" ]]; then
  echo "refusing to launch scaffold or unscaled clean world for Phase33E" >&2
  exit 3
fi

{
  echo "# Phase33E refined marker visual-only checklist"
  echo "World: $WORLD"
  echo "Metadata: $META"
  echo "Guardrails: no autonomous navigation, no SLAM/Nav2 exploration, no maze_explorer, no promotion."
  echo "Expected wall_count: 53"
  echo "Expected entrance marker: maze_entrance_arrow_visual style=green_isosceles_triangle_arrow / phase33d_style=triangle_arrow / +X into maze / no collision"
  echo "Expected exit finish band: maze_exit_finish_band_visual phase33d_style=rotated_checker_finish_band / yaw≈1.570796 / horizontal checker band / no collision"
  echo "Expected exit marker: maze_exit_marker phase33d_style=hidden_semantic_marker / semantic-only hidden / no visible green disk / no collision"
  echo "Manual observation checklist:"
  echo "- 入口三角箭头是否清楚，不再像正方形；"
  echo "- 入口三角箭头是否指向迷宫内部 +X；"
  echo "- 出口黑白终点带是否横向铺满出口 opening；"
  echo "- 绿色圆形 exit marker 是否已隐藏且不遮挡终点带；"
  echo "- 墙体布局、尺度、入口/出口位置是否保持不变；"
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
  echo "=== refined SDF parse ==="
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
    print(name, 'present', m is not None, 'style', (m.get('phase33d_style') if m is not None else None), 'pose', (m.findtext('pose') if m is not None else None), 'collisions', (len(m.findall('.//collision')) if m is not None else None), 'visuals', (len(m.findall('./link/visual')) if m is not None else None))
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
Phase33E visual-only run started.
launch_pid=$LAUNCH_PID
artifact_dir=$ARTIFACT_DIR
world=$WORLD
Do not send navigation goals. Observe Gazebo/RViz and report human acceptance.
EOF
