#!/usr/bin/env bash
set -euo pipefail

RUN_ID="phase45_nav2_bringup_runtime_recovery_diagnostics"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ARTIFACT_DIR="log/${RUN_ID}"
mkdir -p "$ARTIFACT_DIR/params_snapshot"

WORLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
NAV2_PARAMS="$ROOT/src/tugbot_navigation/config/nav2_slam_params.yaml"
LAUNCH_LOG="$ARTIFACT_DIR/launch.log"
NODES_TXT="$ARTIFACT_DIR/nodes.txt"
TOPICS_TXT="$ARTIFACT_DIR/topics.txt"
ACTIONS_TXT="$ARTIFACT_DIR/actions.txt"
LIFECYCLE_TXT="$ARTIFACT_DIR/lifecycle_states.txt"
ACTION_INFO_TXT="$ARTIFACT_DIR/navigate_to_pose_action_info.txt"
GOAL_POSE_INFO_TXT="$ARTIFACT_DIR/goal_pose_topic_info.txt"
PROCESS_TREE_TXT="$ARTIFACT_DIR/nav2_process_tree.txt"
STATIC_JSON="$ARTIFACT_DIR/nav2_launch_static_analysis.json"
SUMMARY_JSON="$ARTIFACT_DIR/phase45_nav2_bringup_diagnostics.json"
NAV2_DIFF_TXT="$ARTIFACT_DIR/nav2_config_diff.txt"
CLEANUP_TXT="$ARTIFACT_DIR/cleanup_processes_after.txt"
PRECHECK_TXT="$ARTIFACT_DIR/precheck.txt"
PARAM_DIR="$ARTIFACT_DIR/params_snapshot"

RUNTIME_SEC="${PHASE45_RUNTIME_SEC:-70}"
STARTUP_WAIT_SEC="${PHASE45_STARTUP_WAIT_SEC:-28}"
POST_CAPTURE_WAIT_SEC="${PHASE45_POST_CAPTURE_WAIT_SEC:-8}"
HEADLESS="${PHASE45_HEADLESS:-true}"
USE_RVIZ="${PHASE45_USE_RVIZ:-false}"

ROS_PATTERN='[r]os2 launch|[g]z sim|[r]viz2|[s]lam_toolbox|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]os_gz_bridge|[p]arameter_bridge|[s]tatic_transform_publisher|record_phase45|analyze_phase45'
FORBIDDEN_PATTERN='[m]aze_explorer|[m]aze_goal_monitor|[f]rontier_explorer'

cleanup() {
  set +e
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    sleep 4
    kill -TERM "$LAUNCH_PID" 2>/dev/null || true
    sleep 2
    kill -KILL "$LAUNCH_PID" 2>/dev/null || true
  fi
  # Target only this launch mode and its common children. This is cleanup, not test logic.
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

{
  echo "run_id=$RUN_ID"
  echo "world_sdf=$WORLD"
  echo "slam_params_file=$SLAM_PARAMS"
  echo "params_file=$NAV2_PARAMS"
  echo "guardrails=no Nav2 parameter semantic edits; no maze_explorer; no maze_goal_monitor; no frontier_explorer; no fallback; no autonomous success claim"
  echo "phase44_preserved_conclusion=MANUAL_NAV2_BASELINE_FAIL"
} > "$PRECHECK_TXT"

git diff -- src/tugbot_navigation/config > "$NAV2_DIFF_TXT"
python3 tools/analyze_phase45_nav2_bringup_runtime_recovery.py \
  --root "$ROOT" \
  --artifact-dir "$ARTIFACT_DIR" \
  --static-output "$STATIC_JSON" \
  --static-only >/dev/null

# ROS setup files can reference unset variables; temporarily relax nounset while sourcing.
set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "$ROOT/install/setup.bash" ]]; then
  source "$ROOT/install/setup.bash"
fi
set -u

# Ensure forbidden explorer nodes are absent before the Nav2-only diagnostics launch.
pkill -f "$FORBIDDEN_PATTERN" 2>/dev/null || true

ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py \
  world_sdf:="$WORLD" \
  slam_params_file:="$SLAM_PARAMS" \
  params_file:="$NAV2_PARAMS" \
  headless:="$HEADLESS" \
  use_rviz:="$USE_RVIZ" \
  use_sim_time:=true \
  autostart:=true \
  > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

sleep "$STARTUP_WAIT_SEC"

ros2 node list > "$NODES_TXT" 2>&1 || true
ros2 topic list --include-hidden-topics > "$TOPICS_TXT" 2>&1 || true
ros2 action list > "$ACTIONS_TXT" 2>&1 || true
ros2 action info /navigate_to_pose > "$ACTION_INFO_TXT" 2>&1 || true
ros2 topic info /goal_pose > "$GOAL_POSE_INFO_TXT" 2>&1 || true
{
  for node in /slam_toolbox /lifecycle_manager_navigation /controller_server /planner_server /bt_navigator /behavior_server /waypoint_follower /velocity_smoother /smoother_server /route_server /collision_monitor /docking_server; do
    echo "## $node"
    ros2 lifecycle get "$node" || true
  done
} > "$LIFECYCLE_TXT" 2>&1

{
  echo "## process tree"
  pgrep -af '[r]os2 launch|[g]z sim|[r]viz2|[s]lam_toolbox|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]oute_server|[c]ollision_monitor|[d]ocking_server|[r]os_gz_bridge|[p]arameter_bridge|[s]tatic_transform_publisher' || true
} > "$PROCESS_TREE_TXT"

for node in /controller_server /planner_server /bt_navigator /behavior_server /waypoint_follower /velocity_smoother /smoother_server /slam_toolbox; do
  safe_name="${node#/}"
  ros2 param dump "$node" > "$PARAM_DIR/${safe_name}.yaml" 2>&1 || true
done

sleep "$POST_CAPTURE_WAIT_SEC"
python3 tools/analyze_phase45_nav2_bringup_runtime_recovery.py \
  --root "$ROOT" \
  --artifact-dir "$ARTIFACT_DIR" \
  --static-output "$STATIC_JSON" \
  --output "$SUMMARY_JSON"

sleep "$RUNTIME_SEC"
