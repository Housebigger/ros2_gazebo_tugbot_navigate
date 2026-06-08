#!/usr/bin/env bash
set -euo pipefail

RUN_ID="phase46_nav2_lifecycle_activation_root_cause"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ARTIFACT_DIR="log/${RUN_ID}"
mkdir -p "$ARTIFACT_DIR"

LAUNCH_LOG="$ARTIFACT_DIR/launch.log"
NODES_TXT="$ARTIFACT_DIR/nodes.txt"
TOPICS_TXT="$ARTIFACT_DIR/topics.txt"
ACTIONS_TXT="$ARTIFACT_DIR/actions.txt"
LIFECYCLE_TXT="$ARTIFACT_DIR/lifecycle_states_before_after.txt"
MANAGER_PARAMS_TXT="$ARTIFACT_DIR/manager_params.txt"
MANAGED_NODES_TXT="$ARTIFACT_DIR/managed_nodes.txt"
TRANSITION_TXT="$ARTIFACT_DIR/transition_service_results.txt"
DIAGNOSTICS_JSON="$ARTIFACT_DIR/diagnostics.json"
CLEANUP_TXT="$ARTIFACT_DIR/cleanup_processes_after.txt"
STATIC_JSON="$ARTIFACT_DIR/nav2_launch_static_analysis.json"
NAV2_DIFF_TXT="$ARTIFACT_DIR/nav2_config_diff.txt"
ACTION_INFO_TXT="$ARTIFACT_DIR/navigate_to_pose_action_info.txt"
GOAL_POSE_INFO_TXT="$ARTIFACT_DIR/goal_pose_topic_info.txt"
PROCESS_TREE_TXT="$ARTIFACT_DIR/nav2_process_tree.txt"
PRECHECK_TXT="$ARTIFACT_DIR/precheck.txt"

PHASE46_RUNTIME_SEC="${PHASE46_RUNTIME_SEC:-5}"
PHASE46_STARTUP_WAIT_SEC="${PHASE46_STARTUP_WAIT_SEC:-45}"
PHASE46_POST_CAPTURE_WAIT_SEC="${PHASE46_POST_CAPTURE_WAIT_SEC:-10}"
PHASE46_HEADLESS="${PHASE46_HEADLESS:-true}"
PHASE46_USE_RVIZ="${PHASE46_USE_RVIZ:-false}"

WORLD_SDF="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
NAV2_PARAMS="$ROOT/src/tugbot_navigation/config/nav2_slam_params.yaml"

ROS_PATTERN='ros2 launch|gz sim|rviz2|slam_toolbox|maze_explorer|maze_goal_monitor|frontier_explorer|controller_server|planner_server|bt_navigator|behavior_server|waypoint_follower|velocity_smoother|smoother_server|route_server|collision_monitor|docking_server|lifecycle_manager|ros_gz_bridge|parameter_bridge|static_transform_publisher|analyze_phase46'

source_ros() {
  set +u
  source /opt/ros/jazzy/setup.bash
  if [ -f install/setup.bash ]; then
    source install/setup.bash
  fi
  set -u
}

cleanup() {
  set +e
  if [ -n "${LAUNCH_PID:-}" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -TERM "$LAUNCH_PID" 2>/dev/null || true
  fi
  pkill -TERM -f 'ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py' 2>/dev/null || true
  pkill -TERM -f 'ruby /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz sim' 2>/dev/null || true
  pkill -TERM -f '/opt/ros/jazzy/lib/ros_gz_bridge/bridge_node' 2>/dev/null || true
  pkill -TERM -f '/opt/ros/jazzy/lib/slam_toolbox/async_slam_toolbox_node' 2>/dev/null || true
  pkill -TERM -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null || true
  pkill -TERM -f '/opt/ros/jazzy/lib/rviz2/rviz2' 2>/dev/null || true
  pkill -TERM -f '/opt/ros/jazzy/lib/tf2_ros/static_transform_publisher' 2>/dev/null || true
  pkill -TERM -f '^gz sim -s -r .*tugbot_maze_world_20260528_clean_scaled2x.sdf' 2>/dev/null || true
  sleep 4
  pkill -KILL -f 'ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py' 2>/dev/null || true
  pkill -KILL -f 'ruby /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz sim' 2>/dev/null || true
  pkill -KILL -f '/opt/ros/jazzy/lib/ros_gz_bridge/bridge_node' 2>/dev/null || true
  pkill -KILL -f '/opt/ros/jazzy/lib/slam_toolbox/async_slam_toolbox_node' 2>/dev/null || true
  pkill -KILL -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null || true
  pkill -KILL -f '/opt/ros/jazzy/lib/rviz2/rviz2' 2>/dev/null || true
  pkill -KILL -f '/opt/ros/jazzy/lib/tf2_ros/static_transform_publisher' 2>/dev/null || true
  pkill -KILL -f '^gz sim -s -r .*tugbot_maze_world_20260528_clean_scaled2x.sdf' 2>/dev/null || true
  sleep 2
  (pgrep -af "$ROS_PATTERN" || true) | grep -v 'hermes-snap' | grep -v 'pgrep -af' > "$CLEANUP_TXT" || true
}
trap cleanup EXIT

source_ros

: > "$PRECHECK_TXT"
{
  echo "run_id=$RUN_ID"
  echo "world_sdf=$WORLD_SDF"
  echo "nav2_params=$NAV2_PARAMS"
  echo "slam_params=$SLAM_PARAMS"
  echo "guardrail_no_maze_explorer=true"
  echo "guardrail_no_autonomous_success_claim=true"
} > "$PRECHECK_TXT"

python3 tools/analyze_phase46_nav2_lifecycle_activation_root_cause.py \
  --root "$ROOT" \
  --artifact-dir "$ARTIFACT_DIR" \
  --static-output "$STATIC_JSON" \
  --output "$DIAGNOSTICS_JSON" >/dev/null || true

git diff -- src/tugbot_navigation/config > "$NAV2_DIFF_TXT" || true

ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py \
  world_sdf:="$WORLD_SDF" \
  slam_params_file:="$SLAM_PARAMS" \
  params_file:="$NAV2_PARAMS" \
  headless:="$PHASE46_HEADLESS" \
  use_rviz:="$PHASE46_USE_RVIZ" \
  use_sim_time:=true \
  autostart:=true \
  > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

sleep "$PHASE46_STARTUP_WAIT_SEC"

capture_graph() {
  local suffix="$1"
  echo "## ${suffix}" >> "$LIFECYCLE_TXT"
  for node in /lifecycle_manager_navigation /controller_server /planner_server /bt_navigator /behavior_server /waypoint_follower /velocity_smoother /smoother_server /route_server /collision_monitor /docking_server /slam_toolbox; do
    state="$(timeout 5 ros2 lifecycle get "$node" 2>&1 || true)"
    state="$(printf '%s' "$state" | tr '\n' ' ' | sed 's/[[:space:]]\+/ /g; s/^ //; s/ $//')"
    if printf '%s' "$state" | grep -q 'Node not found'; then
      echo "${node}: missing" >> "$LIFECYCLE_TXT"
    elif [ -z "$state" ]; then
      echo "${node}: unknown" >> "$LIFECYCLE_TXT"
    else
      echo "${node}: ${state}" >> "$LIFECYCLE_TXT"
    fi
  done
}

: > "$LIFECYCLE_TXT"
capture_graph "before_optional_wait"
sleep "$PHASE46_POST_CAPTURE_WAIT_SEC"
capture_graph "after_optional_wait"

timeout 10 ros2 node list > "$NODES_TXT" 2>&1 || true
timeout 10 ros2 topic list > "$TOPICS_TXT" 2>&1 || true
timeout 10 ros2 action list > "$ACTIONS_TXT" 2>&1 || true
timeout 10 ros2 action info /navigate_to_pose > "$ACTION_INFO_TXT" 2>&1 || true
timeout 10 ros2 topic info /goal_pose > "$GOAL_POSE_INFO_TXT" 2>&1 || true

{
  echo "# lifecycle_manager_navigation params"
  timeout 10 ros2 param list /lifecycle_manager_navigation 2>&1 || true
  echo
  echo "# autostart"
  timeout 10 ros2 param get /lifecycle_manager_navigation autostart 2>&1 || true
  echo
  echo "# node_names"
  timeout 10 ros2 param get /lifecycle_manager_navigation node_names 2>&1 || true
  echo
  echo "# use_sim_time"
  timeout 10 ros2 param get /lifecycle_manager_navigation use_sim_time 2>&1 || true
} > "$MANAGER_PARAMS_TXT"

{
  timeout 10 ros2 param get /lifecycle_manager_navigation node_names 2>&1 || true
} > "$MANAGED_NODES_TXT"

{
  echo "# lifecycle service list"
  timeout 10 ros2 service list | grep -E 'change_state|get_state|lifecycle_manager' || true
  echo
  echo "# manager node info"
  timeout 10 ros2 node info /lifecycle_manager_navigation 2>&1 || true
} > "$TRANSITION_TXT"

{
  echo "## process tree"
  pgrep -af "$ROS_PATTERN" || true
} > "$PROCESS_TREE_TXT"

python3 tools/analyze_phase46_nav2_lifecycle_activation_root_cause.py \
  --root "$ROOT" \
  --artifact-dir "$ARTIFACT_DIR" \
  --static-output "$STATIC_JSON" \
  --output "$DIAGNOSTICS_JSON"
