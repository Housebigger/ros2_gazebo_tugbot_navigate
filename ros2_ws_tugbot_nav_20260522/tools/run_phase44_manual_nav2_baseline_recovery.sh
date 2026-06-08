#!/usr/bin/env bash
set -euo pipefail

RUN_ID="phase44_manual_nav2_baseline_recovery"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ARTIFACT_DIR="log/${RUN_ID}"
mkdir -p "$ARTIFACT_DIR"

WORLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
NAV2_PARAMS="$ROOT/src/tugbot_navigation/config/nav2_slam_params.yaml"
LAUNCH_LOG="$ARTIFACT_DIR/launch.log"
SUMMARY_JSON="$ARTIFACT_DIR/${RUN_ID}.json"
SAMPLE_JSON="$ARTIFACT_DIR/map_scan_tf_odom_sample.json"
NODES_TXT="$ARTIFACT_DIR/nodes.txt"
TOPICS_TXT="$ARTIFACT_DIR/topics.txt"
ACTIONS_TXT="$ARTIFACT_DIR/actions.txt"
ACTION_INFO_TXT="$ARTIFACT_DIR/navigate_to_pose_action_info.txt"
LIFECYCLE_TXT="$ARTIFACT_DIR/lifecycle_states.txt"
PRECHECK_TXT="$ARTIFACT_DIR/precheck.txt"
NAV2_DIFF_TXT="$ARTIFACT_DIR/nav2_config_diff.txt"
CLEANUP_TXT="$ARTIFACT_DIR/cleanup_processes_after.txt"
SCREENSHOT_NOTES="$ARTIFACT_DIR/screenshot_locations_and_manual_steps.txt"

RUNTIME_SEC="${PHASE44_RUNTIME_SEC:-90}"
RECORDER_SEC="${PHASE44_RECORDER_SEC:-35}"
KEEP_ALIVE_FOR_HUMAN="${PHASE44_KEEP_ALIVE_FOR_HUMAN:-0}"

ROS_PATTERN='[r]os2 launch|[g]z sim|[r]viz2|[s]lam_toolbox|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]os_gz_bridge|[p]arameter_bridge|[s]tatic_transform_publisher|record_phase44_manual_nav2_baseline_evidence.py'
FORBIDDEN_PATTERN='[m]aze_explorer|[m]aze_goal_monitor|[f]rontier_explorer'

cleanup() {
  set +e
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    sleep 5
    kill -TERM "$LAUNCH_PID" 2>/dev/null || true
    sleep 3
    kill -KILL "$LAUNCH_PID" 2>/dev/null || true
  fi
  pkill -f 'record_phase44_manual_nav2_baseline_evidence.py' 2>/dev/null || true
  pkill -f 'ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py' 2>/dev/null || true
  sleep 2
  pgrep -af "$ROS_PATTERN" > "$CLEANUP_TXT" 2>/dev/null || true
}
trap cleanup EXIT

{
  echo "run_id=$RUN_ID"
  echo "world_sdf=$WORLD"
  echo "slam_params_file=$SLAM_PARAMS"
  echo "params_file=$NAV2_PARAMS"
  echo "guardrails=no maze_explorer; no maze_goal_monitor; no frontier_explorer; no Nav2/MPPI/controller parameter edits; no autonomous exploration success claim"
  echo "manual_steps=RViz 2D Pose Estimate near entrance, then Nav2 Goal near exit_map=(21.072562,18.083566), observe plan/costmap/motion/result"
} > "$PRECHECK_TXT"

git diff -- src/tugbot_navigation/config > "$NAV2_DIFF_TXT"

cat > "$SCREENSHOT_NOTES" <<'NOTES'
Phase44 RViz/Gazebo screenshot notes:
- Save RViz screenshots under log/phase44_manual_nav2_baseline_recovery/rviz_2d_pose_estimate.png and rviz_nav2_goal_to_exit.png if manually captured.
- Save Gazebo screenshot under log/phase44_manual_nav2_baseline_recovery/gazebo_manual_nav2_goal_motion.png if manually captured.
- Manual acceptance requires human observation of global plan, local costmap, robot motion, and arrival/failure result.
- This wrapper records readiness evidence only; it does not send a Nav2 goal.
NOTES

# ROS setup files can reference unset variables; temporarily relax nounset while sourcing.
set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "$ROOT/install/setup.bash" ]]; then
  source "$ROOT/install/setup.bash"
fi
set -u

# Ensure stale graph/processes from previous runs do not pollute the manual baseline check.
pkill -f "$FORBIDDEN_PATTERN" 2>/dev/null || true

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

# Allow Gazebo, bridge, SLAM, Nav2 and RViz startup before evidence capture.
sleep 25

ros2 node list > "$NODES_TXT" 2>&1 || true
ros2 topic list --include-hidden-topics > "$TOPICS_TXT" 2>&1 || true
ros2 action list > "$ACTIONS_TXT" 2>&1 || true
ros2 action info /navigate_to_pose > "$ACTION_INFO_TXT" 2>&1 || true
{
  for node in /slam_toolbox /controller_server /planner_server /bt_navigator /behavior_server /waypoint_follower /velocity_smoother /smoother_server; do
    echo "## $node"
    ros2 lifecycle get "$node" || true
  done
} > "$LIFECYCLE_TXT" 2>&1

python3 tools/record_phase44_manual_nav2_baseline_evidence.py \
  --duration "$RECORDER_SEC" \
  --output "$SAMPLE_JSON" || true

python3 - <<'PY' "$ARTIFACT_DIR" "$SAMPLE_JSON" "$NODES_TXT" "$ACTION_INFO_TXT" "$SUMMARY_JSON" "$NAV2_DIFF_TXT"
import json, pathlib, sys
artifact_dir = pathlib.Path(sys.argv[1])
sample_path = pathlib.Path(sys.argv[2])
nodes_path = pathlib.Path(sys.argv[3])
action_path = pathlib.Path(sys.argv[4])
summary_path = pathlib.Path(sys.argv[5])
diff_path = pathlib.Path(sys.argv[6])
sample = json.loads(sample_path.read_text()) if sample_path.exists() and sample_path.stat().st_size else {}
nodes = nodes_path.read_text(errors='replace').splitlines() if nodes_path.exists() else []
action_info = action_path.read_text(errors='replace') if action_path.exists() else ''
forbidden = [n for n in nodes if any(tok in n for tok in ('maze_explorer','maze_goal_monitor','frontier_explorer'))]
required_nodes = ['/slam_toolbox','/controller_server','/planner_server','/bt_navigator','/behavior_server','/waypoint_follower','/velocity_smoother','/smoother_server','/rviz2']
required_present = {n: n in nodes for n in required_nodes}
readiness_ok = (
    not forbidden
    and sample.get('map',{}).get('available')
    and sample.get('scan',{}).get('available')
    and sample.get('odom',{}).get('available')
    and all((sample.get('tf',{}).get(k) or {}).get('available') for k in ('map->base_link','odom->base_link','map->odom'))
    and sample.get('navigate_to_pose_action_available')
)
summary = {
    'run_id': 'phase44_manual_nav2_baseline_recovery',
    'classification': 'MANUAL_NAV2_INCONCLUSIVE_NEEDS_HUMAN_RVIZ_CHECK',
    'classification_reason': 'Gazebo+SLAM+Nav2+RViz readiness evidence collected without explorer; final OK/FAIL requires human RViz 2D Pose Estimate and Nav2 Goal to exit.',
    'manual_nav2_baseline_readiness_ok': bool(readiness_ok),
    'manual_goal_sent_by_wrapper': False,
    'autonomous_exploration_success_claimed': False,
    'active_world': 'tugbot_maze_world_20260528_clean_scaled2x.sdf',
    'truth_frame': 'map',
    'entrance_map': [0.0, 0.0, 0.0],
    'exit_map': [21.072562, 18.083566],
    'exit_radius': 1.2,
    'required_nodes_present': required_present,
    'forbidden_explorer_nodes_seen': forbidden,
    'navigate_to_pose_action_info_has_server': ('Action clients:' in action_info or 'Action servers:' in action_info or '/navigate_to_pose' in action_info),
    'sample': sample,
    'nav2_config_diff_empty': diff_path.exists() and diff_path.read_text() == '',
    'artifacts': sorted(p.name for p in artifact_dir.iterdir()),
}
summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n')
print(json.dumps({'classification': summary['classification'], 'readiness_ok': readiness_ok, 'forbidden': forbidden}, sort_keys=True))
PY

# For default CI/agent mode, this is a bounded readiness run and cleanup follows.
# Set PHASE44_KEEP_ALIVE_FOR_HUMAN=1 only when a human wants to perform the RViz goal now.
if [[ "$KEEP_ALIVE_FOR_HUMAN" == "1" ]]; then
  echo "Phase44 manual baseline is live for human RViz acceptance. PID=$LAUNCH_PID"
  echo "After manual check, terminate PID $LAUNCH_PID or let this script receive Ctrl-C for cleanup."
  wait "$LAUNCH_PID" || true
else
  sleep "$RUNTIME_SEC"
fi
