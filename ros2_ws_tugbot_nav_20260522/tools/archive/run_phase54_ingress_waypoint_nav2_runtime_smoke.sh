#!/usr/bin/env bash
set -euo pipefail

RUN_ID="phase54_ingress_waypoint_nav2_runtime_smoke"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ARTIFACT_DIR="log/${RUN_ID}"
WORLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
MAZE_CONFIG="$ROOT/src/tugbot_maze/config/maze_20260528_scaled_instance.yaml"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
NAV2_PARAMS="$ROOT/src/tugbot_navigation/config/nav2_slam_params.yaml"
INGRESS_X="1.0"
INGRESS_Y="0.0"
INGRESS_YAW="0.0"
ACCEPTANCE_RADIUS_M="${PHASE54_ACCEPTANCE_RADIUS_M:-0.35}"
RUN_TIMEOUT_SEC="${PHASE54_RUN_TIMEOUT_SEC:-240}"
RECORD_TIMEOUT_SEC="${PHASE54_RECORD_TIMEOUT_SEC:-150}"
GOAL_TIMEOUT_SEC="${PHASE54_GOAL_TIMEOUT_SEC:-90}"
READINESS_TIMEOUT_SEC="${PHASE54_READINESS_TIMEOUT_SEC:-120}"
USE_RVIZ="${PHASE54_USE_RVIZ:-false}"

if (( RUN_TIMEOUT_SEC < 120 || RUN_TIMEOUT_SEC > 360 )); then
  echo "Phase54 RUN_TIMEOUT_SEC must be bounded in [120, 360], got $RUN_TIMEOUT_SEC" >&2
  exit 2
fi
if (( RECORD_TIMEOUT_SEC < 60 || RECORD_TIMEOUT_SEC > 240 )); then
  echo "Phase54 RECORD_TIMEOUT_SEC must be bounded in [60, 240], got $RECORD_TIMEOUT_SEC" >&2
  exit 2
fi

LAUNCH_LOG="$ARTIFACT_DIR/${RUN_ID}_launch.log"
RUNTIME_EVIDENCE="$ARTIFACT_DIR/${RUN_ID}_runtime_evidence.json"
ACTION_RESULT="$ARTIFACT_DIR/${RUN_ID}_navigate_to_pose_action_result.json"
ACTION_INFO="$ARTIFACT_DIR/${RUN_ID}_navigate_to_pose_action_info.txt"
GOAL_POSE_INFO="$ARTIFACT_DIR/${RUN_ID}_goal_pose_topic_info.txt"
LIFECYCLE_READINESS="$ARTIFACT_DIR/${RUN_ID}_lifecycle_readiness.txt"
SUMMARY_JSON="$ARTIFACT_DIR/${RUN_ID}.json"
NAV2_CONFIG_DIFF="$ARTIFACT_DIR/${RUN_ID}_nav2_config_diff.txt"
CLEANUP_AFTER="$ARTIFACT_DIR/${RUN_ID}_cleanup_processes_after.txt"
PIDS=()
LAUNCH_PID=""
RECORDER_PID=""

cleanup() {
  set +e
  trap - EXIT
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  sleep 3
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
  pkill -TERM -f '[a]nalyze_phase54_ingress_waypoint_nav2_runtime_smoke.py --record-runtime' || true
  pkill -TERM -f '[a]nalyze_phase54_ingress_waypoint_nav2_runtime_smoke.py --send-goal' || true
  pkill -TERM -f '[r]os2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py' || true
  pkill -TERM -f '[r]os_gz_bridge|[p]arameter_bridge|[s]tatic_transform_publisher' || true
  pkill -TERM -f '[s]lam_toolbox|[a]sync_slam_toolbox_node|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[s]moother_server|[w]aypoint_follower|[v]elocity_smoother|[l]ifecycle_manager_navigation|[r]oute_server|[c]ollision_monitor|[d]ocking_server' || true
  pkill -TERM -f '[g]z sim|[r]uby .*gz sim' || true
  sleep 3
  pkill -KILL -f '[r]os_gz_bridge|[p]arameter_bridge|[s]tatic_transform_publisher|[s]lam_toolbox|[a]sync_slam_toolbox_node|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[g]z sim|[r]uby .*gz sim' || true
  if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
  fi
  mkdir -p "$ARTIFACT_DIR"
  (pgrep -af '[r]os2 launch|[g]z sim|[r]viz2|[s]lam_toolbox|[m]aze_explorer|[m]aze_goal_monitor|[f]rontier_explorer|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]oute_server|[c]ollision_monitor|[d]ocking_server|[r]os_gz_bridge|[p]arameter_bridge|[s]tatic_transform_publisher|[a]nalyze_phase54_ingress_waypoint' || true) \
    | grep -v 'hermes-snap' \
    | grep -v 'pgrep -af' \
    > "$CLEANUP_AFTER" || true
}
trap cleanup EXIT

mkdir -p "$ARTIFACT_DIR"
rm -f \
  "$LAUNCH_LOG" "$RUNTIME_EVIDENCE" "$ACTION_RESULT" "$ACTION_INFO" "$GOAL_POSE_INFO" \
  "$LIFECYCLE_READINESS" "$SUMMARY_JSON" "$NAV2_CONFIG_DIFF" "$CLEANUP_AFTER" \
  "$ARTIFACT_DIR/${RUN_ID}_preflight.txt" "$ARTIFACT_DIR/${RUN_ID}_ros_graph_snapshot.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_ros_graph_final.txt" "$ARTIFACT_DIR/${RUN_ID}_goal_stdout.json" \
  "$ARTIFACT_DIR/${RUN_ID}_goal_stderr.log" "$ARTIFACT_DIR/${RUN_ID}_runtime_recorder_stdout.json" \
  "$ARTIFACT_DIR/${RUN_ID}_runtime_recorder_stderr.log"

if [[ ! -f "$WORLD" ]]; then echo "missing active scaled2x world: $WORLD" >&2; exit 3; fi
if [[ ! -f "$MAZE_CONFIG" ]]; then echo "missing active maze metadata: $MAZE_CONFIG" >&2; exit 4; fi
if [[ ! -f "$SLAM_PARAMS" || ! -f "$NAV2_PARAMS" ]]; then echo "missing SLAM or Nav2 params" >&2; exit 5; fi
if [[ "$WORLD" != *"tugbot_maze_world_20260528_clean_scaled2x.sdf" ]]; then echo "refusing non-active world: $WORLD" >&2; exit 6; fi
if [[ "$MAZE_CONFIG" != *"maze_20260528_scaled_instance.yaml" ]]; then echo "refusing non-active maze metadata: $MAZE_CONFIG" >&2; exit 7; fi

{
  echo "# Phase54 Ingress Waypoint Nav2 Runtime Smoke"
  echo "timestamp=$(date -Is)"
  echo "run_id=$RUN_ID"
  echo "world=$WORLD"
  echo "maze_config=$MAZE_CONFIG"
  echo "slam_params=$SLAM_PARAMS"
  echo "nav2_params=$NAV2_PARAMS"
  echo "ingress_waypoint_map=x=${INGRESS_X},y=${INGRESS_Y},yaw=${INGRESS_YAW}"
  echo "acceptance_radius_m=$ACCEPTANCE_RADIUS_M"
  echo "guardrails=SLAM+Nav2 only; no maze_explorer; no maze_goal_monitor; no frontier_explorer; no Nav2/MPPI/controller tuning; no clearance_radius_m tuning; no map threshold tuning; no autonomous exploration success claim."
  echo "nav2_config_diff_begin"
  git diff -- src/tugbot_navigation/config | tee "$NAV2_CONFIG_DIFF" || true
  echo "nav2_config_diff_end"
} > "$ARTIFACT_DIR/${RUN_ID}_preflight.txt"

set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u
ros2 daemon stop >/dev/null 2>&1 || true

set -o pipefail
(
  timeout --preserve-status "$RUN_TIMEOUT_SEC" ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py \
    world_sdf:="$WORLD" \
    slam_params_file:="$SLAM_PARAMS" \
    params_file:="$NAV2_PARAMS" \
    headless:=true \
    use_rviz:="${USE_RVIZ}" \
    use_sim_time:=true \
    autostart:=true \
    2>&1 | tee "$LAUNCH_LOG"
) &
LAUNCH_PID="$!"
PIDS+=("$LAUNCH_PID")
echo "$LAUNCH_PID" > "$ARTIFACT_DIR/${RUN_ID}_launch.pid"

ready=0
start_sec=$(date +%s)
: > "$LIFECYCLE_READINESS"
while (( $(date +%s) - start_sec < READINESS_TIMEOUT_SEC )); do
  ctrl_tmp="$ARTIFACT_DIR/${RUN_ID}_controller_lifecycle.tmp"
  planner_tmp="$ARTIFACT_DIR/${RUN_ID}_planner_lifecycle.tmp"
  bt_tmp="$ARTIFACT_DIR/${RUN_ID}_bt_lifecycle.tmp"
  (timeout 5 ros2 lifecycle get /controller_server || true) > "$ctrl_tmp" 2>&1
  (timeout 5 ros2 lifecycle get /planner_server || true) > "$planner_tmp" 2>&1
  (timeout 5 ros2 lifecycle get /bt_navigator || true) > "$bt_tmp" 2>&1
  (timeout 5 ros2 action info /navigate_to_pose || true) > "$ACTION_INFO" 2>&1
  (timeout 5 ros2 topic info /goal_pose || true) > "$GOAL_POSE_INFO" 2>&1
  {
    echo "# gate snapshot $(date -Is)"
    echo "/controller_server"
    cat "$ctrl_tmp"
    echo "/planner_server"
    cat "$planner_tmp"
    echo "/bt_navigator"
    cat "$bt_tmp"
    echo "/navigate_to_pose"
    cat "$ACTION_INFO"
    echo "/goal_pose"
    cat "$GOAL_POSE_INFO"
  } >> "$LIFECYCLE_READINESS" 2>&1
  if grep -q 'active \[3\]' "$ctrl_tmp" \
    && grep -q 'active \[3\]' "$planner_tmp" \
    && grep -q 'active \[3\]' "$bt_tmp" \
    && grep -q 'Action servers: 1' "$ACTION_INFO" \
    && grep -q 'Subscription count: 1' "$GOAL_POSE_INFO"; then
    ready=1
    break
  fi
  sleep 5
done
rm -f "$ARTIFACT_DIR/${RUN_ID}"_*_lifecycle.tmp

echo "readiness_ready=$ready" >> "$LIFECYCLE_READINESS"

{
  echo "=== timestamp ==="
  date -Is
  echo "=== ROS nodes ==="
  ros2 node list || true
  echo "=== ROS topics ==="
  ros2 topic list || true
  echo "=== ROS actions ==="
  ros2 action list || true
  echo "=== forbidden explorer grep ==="
  ros2 node list | grep -E 'maze_explorer|maze_goal_monitor|frontier_explorer' || true
} > "$ARTIFACT_DIR/${RUN_ID}_ros_graph_snapshot.txt" 2>&1

python3 tools/analyze_phase54_ingress_waypoint_nav2_runtime_smoke.py --record-runtime "$RECORD_TIMEOUT_SEC" \
  --snapshot-period-sec 1.0 \
  --output "$RUNTIME_EVIDENCE" \
  > "$ARTIFACT_DIR/${RUN_ID}_runtime_recorder_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_runtime_recorder_stderr.log" &
RECORDER_PID="$!"
PIDS+=("$RECORDER_PID")

sleep 3
if [[ "$ready" == "1" ]]; then
  python3 tools/analyze_phase54_ingress_waypoint_nav2_runtime_smoke.py --send-goal \
    --goal-timeout-sec "$GOAL_TIMEOUT_SEC" \
    --output "$ACTION_RESULT" \
    > "$ARTIFACT_DIR/${RUN_ID}_goal_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_goal_stderr.log" || true
else
  cat > "$ACTION_RESULT" <<JSON
{"phase":"Phase54 Ingress Waypoint Nav2 Runtime Smoke","run_id":"$RUN_ID","goal_sent":false,"success":false,"result_received":false,"reason":"readiness_gates_not_ready"}
JSON
fi

sleep 5
if kill -0 "$RECORDER_PID" 2>/dev/null; then
  kill -TERM "$RECORDER_PID" 2>/dev/null || true
fi
wait "$RECORDER_PID" 2>/dev/null || true

{
  echo "=== timestamp ==="
  date -Is
  echo "=== ROS nodes ==="
  ros2 node list || true
  echo "=== ROS topics ==="
  ros2 topic list || true
} > "$ARTIFACT_DIR/${RUN_ID}_ros_graph_final.txt" 2>&1

cleanup
trap - EXIT

python3 tools/analyze_phase54_ingress_waypoint_nav2_runtime_smoke.py --analyze \
  --artifact-dir "$ARTIFACT_DIR" \
  --output "$SUMMARY_JSON" \
  --runtime-evidence "$RUNTIME_EVIDENCE" \
  --action-result "$ACTION_RESULT" \
  --lifecycle-readiness "$LIFECYCLE_READINESS" \
  --action-info "$ACTION_INFO" \
  --goal-pose-info "$GOAL_POSE_INFO" \
  --ros-graph-snapshot "$ARTIFACT_DIR/${RUN_ID}_ros_graph_snapshot.txt" \
  --ros-graph-final "$ARTIFACT_DIR/${RUN_ID}_ros_graph_final.txt" \
  --launch-log "$LAUNCH_LOG" \
  --cleanup-processes-after "$CLEANUP_AFTER" \
  > "$ARTIFACT_DIR/${RUN_ID}_analyzer_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_analyzer_stderr.log" || true

cat "$SUMMARY_JSON"
