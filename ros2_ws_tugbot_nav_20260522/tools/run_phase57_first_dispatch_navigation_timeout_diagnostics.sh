#!/usr/bin/env bash
set -euo pipefail

RUN_ID="phase57_first_dispatch_navigation_timeout_diagnostics"
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
ACCEPTANCE_RADIUS_M="${PHASE57_ACCEPTANCE_RADIUS_M:-0.35}"
RUN_TIMEOUT_SEC="${PHASE57_RUN_TIMEOUT_SEC:-300}"
GOAL_TIMEOUT_SEC="${PHASE57_GOAL_TIMEOUT_SEC:-100}"
READINESS_TIMEOUT_SEC="${PHASE57_READINESS_TIMEOUT_SEC:-170}"
EXPLORER_OBSERVE_SEC="${PHASE57_EXPLORER_OBSERVE_SEC:-120}"
EXECUTION_RECORD_TIMEOUT_SEC="${PHASE57_EXECUTION_RECORD_TIMEOUT_SEC:-140}"
USE_RVIZ="${PHASE57_USE_RVIZ:-false}"
MAX_GOALS="1"

if (( RUN_TIMEOUT_SEC < 150 || RUN_TIMEOUT_SEC > 360 )); then
  echo "Phase57 RUN_TIMEOUT_SEC must be bounded in [150, 360], got $RUN_TIMEOUT_SEC" >&2
  exit 2
fi
if (( EXPLORER_OBSERVE_SEC < 45 || EXPLORER_OBSERVE_SEC > 160 )); then
  echo "Phase57 EXPLORER_OBSERVE_SEC must be bounded in [45, 160], got $EXPLORER_OBSERVE_SEC" >&2
  exit 2
fi
if (( EXECUTION_RECORD_TIMEOUT_SEC < 60 || EXECUTION_RECORD_TIMEOUT_SEC > 180 )); then
  echo "Phase57 EXECUTION_RECORD_TIMEOUT_SEC must be bounded in [60, 180], got $EXECUTION_RECORD_TIMEOUT_SEC" >&2
  exit 2
fi

LAUNCH_LOG="$ARTIFACT_DIR/${RUN_ID}_launch.log"
ACTION_RESULT="$ARTIFACT_DIR/${RUN_ID}_ingress_navigate_to_pose_action_result.json"
ACTION_INFO="$ARTIFACT_DIR/${RUN_ID}_navigate_to_pose_action_info.txt"
GOAL_POSE_INFO="$ARTIFACT_DIR/${RUN_ID}_goal_pose_topic_info.txt"
LIFECYCLE_READINESS="$ARTIFACT_DIR/${RUN_ID}_lifecycle_readiness.txt"
EXPLORER_STATE="$ARTIFACT_DIR/${RUN_ID}_explorer_state.jsonl"
GOAL_EVENTS="$ARTIFACT_DIR/${RUN_ID}_goal_events.jsonl"
SUMMARY_JSON="$ARTIFACT_DIR/${RUN_ID}.json"
NAV2_CONFIG_DIFF="$ARTIFACT_DIR/${RUN_ID}_nav2_config_diff.txt"
CLEANUP_AFTER="$ARTIFACT_DIR/${RUN_ID}_cleanup_processes_after.txt"
PREFLIGHT="$ARTIFACT_DIR/${RUN_ID}_preflight.txt"
FIRST_DISPATCH_EXECUTION_JSONL="$ARTIFACT_DIR/first_dispatch_execution.jsonl"
CONTROLLER_DYNAMICS_JSONL="$ARTIFACT_DIR/controller_dynamics.jsonl"
NAV2_FEEDBACK_JSONL="$ARTIFACT_DIR/nav2_feedback.jsonl"
LOCAL_COSTMAP_SAMPLES_JSONL="$ARTIFACT_DIR/local_costmap_samples.jsonl"
GLOBAL_PLAN_SAMPLES_JSONL="$ARTIFACT_DIR/global_plan_samples.jsonl"
COLLISION_MONITOR_JSONL="$ARTIFACT_DIR/collision_monitor_state.jsonl"
FIRST_DISPATCH_EXECUTION_SUMMARY="$ARTIFACT_DIR/first_dispatch_execution_summary.json"
PIDS=()
LAUNCH_PID=""
EXECUTION_RECORDER_PID=""
STATE_PID=""
GOAL_EVENTS_PID=""
EXPLORER_PID=""

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
  pkill -TERM -f '[a]nalyze_phase57_first_dispatch_navigation_timeout_diagnostics.py --record-execution' || true
  pkill -TERM -f '[a]nalyze_phase57_first_dispatch_navigation_timeout_diagnostics.py --send-ingress-goal' || true
  pkill -TERM -f '[r]ecord_explorer_state_series.py.*phase57_first_dispatch_navigation_timeout_diagnostics' || true
  pkill -TERM -f '[r]os2 run tugbot_maze maze_explorer' || true
  pkill -TERM -f '[m]aze_explorer' || true
  pkill -TERM -f '[m]aze_goal_monitor|[f]rontier_explorer' || true
  pkill -TERM -f '[r]os2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py' || true
  pkill -TERM -f '[r]os_gz_bridge|[p]arameter_bridge|[s]tatic_transform_publisher' || true
  pkill -TERM -f '[s]lam_toolbox|[a]sync_slam_toolbox_node|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[s]moother_server|[w]aypoint_follower|[v]elocity_smoother|[l]ifecycle_manager_navigation|[r]oute_server|[c]ollision_monitor|[d]ocking_server' || true
  pkill -TERM -f '[g]z sim|[r]uby .*gz sim' || true
  sleep 3
  pkill -KILL -f '[r]os_gz_bridge|[p]arameter_bridge|[s]tatic_transform_publisher|[m]aze_explorer|[m]aze_goal_monitor|[f]rontier_explorer|[s]lam_toolbox|[a]sync_slam_toolbox_node|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[g]z sim|[r]uby .*gz sim' || true
  if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
  fi
  mkdir -p "$ARTIFACT_DIR"
  (pgrep -af '[r]os2 launch|[r]os2 run tugbot_maze maze_explorer|[g]z sim|[r]viz2|[s]lam_toolbox|[m]aze_explorer|[m]aze_goal_monitor|[f]rontier_explorer|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]oute_server|[c]ollision_monitor|[d]ocking_server|[r]os_gz_bridge|[p]arameter_bridge|[s]tatic_transform_publisher|[a]nalyze_phase57_first_dispatch_navigation_timeout_diagnostics' || true) \
    | grep -v 'hermes-snap' \
    | grep -v 'pgrep -af' \
    > "$CLEANUP_AFTER" || true
}
trap cleanup EXIT

mkdir -p "$ARTIFACT_DIR"
rm -f \
  "$LAUNCH_LOG" "$ACTION_RESULT" "$ACTION_INFO" "$GOAL_POSE_INFO" "$LIFECYCLE_READINESS" \
  "$EXPLORER_STATE" "$GOAL_EVENTS" "$SUMMARY_JSON" "$NAV2_CONFIG_DIFF" "$CLEANUP_AFTER" "$PREFLIGHT" \
  "$FIRST_DISPATCH_EXECUTION_JSONL" "$CONTROLLER_DYNAMICS_JSONL" "$NAV2_FEEDBACK_JSONL" "$COLLISION_MONITOR_JSONL" \
  "$LOCAL_COSTMAP_SAMPLES_JSONL" "$GLOBAL_PLAN_SAMPLES_JSONL" \
  "$FIRST_DISPATCH_EXECUTION_SUMMARY" \
  "$ARTIFACT_DIR/${RUN_ID}_ros_graph_snapshot.txt" "$ARTIFACT_DIR/${RUN_ID}_ros_graph_after_explorer.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_ros_graph_final.txt" "$ARTIFACT_DIR/${RUN_ID}_goal_stdout.json" \
  "$ARTIFACT_DIR/${RUN_ID}_goal_stderr.log" "$ARTIFACT_DIR/${RUN_ID}_execution_recorder_stdout.json" \
  "$ARTIFACT_DIR/${RUN_ID}_execution_recorder_stderr.log" "$ARTIFACT_DIR/${RUN_ID}_explorer_stdout.log" \
  "$ARTIFACT_DIR/${RUN_ID}_explorer_stderr.log" "$ARTIFACT_DIR/${RUN_ID}_explorer.pid"

if [[ ! -f "$WORLD" ]]; then echo "missing active scaled2x world: $WORLD" >&2; exit 3; fi
if [[ ! -f "$MAZE_CONFIG" ]]; then echo "missing active maze metadata: $MAZE_CONFIG" >&2; exit 4; fi
if [[ ! -f "$SLAM_PARAMS" || ! -f "$NAV2_PARAMS" ]]; then echo "missing SLAM or Nav2 params" >&2; exit 5; fi
if [[ "$WORLD" != *"tugbot_maze_world_20260528_clean_scaled2x.sdf" ]]; then echo "refusing non-active world: $WORLD" >&2; exit 6; fi
if [[ "$MAZE_CONFIG" != *"maze_20260528_scaled_instance.yaml" ]]; then echo "refusing non-active maze metadata: $MAZE_CONFIG" >&2; exit 7; fi

{
  echo "# Phase57 First Dispatch Navigation Timeout Diagnostics"
  echo "timestamp=$(date -Is)"
  echo "run_id=$RUN_ID"
  echo "world=$WORLD"
  echo "maze_config=$MAZE_CONFIG"
  echo "slam_params=$SLAM_PARAMS"
  echo "nav2_params=$NAV2_PARAMS"
  echo "ingress_waypoint_map=x=${INGRESS_X},y=${INGRESS_Y},yaw=${INGRESS_YAW}"
  echo "acceptance_radius_m=$ACCEPTANCE_RADIUS_M"
  echo "max_goals=1"
  echo "near_exit_fallback_enabled=false"
  echo "startup_warmup_no_dispatch=false"
  echo "guardrail_violation_class=GUARDRAIL_VIOLATION_STRATEGY_CHANGED"
  echo "guardrails=bounded first-dispatch execution diagnostics only; no Nav2/MPPI/controller parameter edits; no clearance_radius_m tuning; no map threshold tuning; no maze_explorer strategy change; no fallback/terminal acceptance; no old scaffold world/map; no autonomous exploration success claim; first dispatch is not exit success."
  echo "artifact_contract=first_dispatch_execution.jsonl controller_dynamics.jsonl nav2_feedback.jsonl collision_monitor_state.jsonl first_dispatch_execution_summary.json explorer_state.jsonl goal_events.jsonl controller_server/bt_navigator logs summary/analysis JSON cleanup_processes_after.txt"
  echo "artifact_contract_extended=local_costmap_samples.jsonl global_plan_samples.jsonl"
  echo "maze_explorer_command=ros2 run tugbot_maze maze_explorer --ros-args -p max_goals:=1 -p near_exit_fallback_enabled:=false -p startup_warmup_no_dispatch:=false"
  echo "nav2_config_diff_begin"
  git diff -- src/tugbot_navigation/config | tee "$NAV2_CONFIG_DIFF" || true
  echo "nav2_config_diff_end"
} > "$PREFLIGHT"

set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u
ros2 daemon stop >/dev/null 2>&1 || true

launch_phase57() {
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
}

wait_for_readiness() {
  local ready=0
  local start_sec
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
  if (( ready == 1 )); then
    return 0
  fi
  return 1
}

start_first_dispatch_execution_recorder() {
  python3 tools/analyze_phase57_first_dispatch_navigation_timeout_diagnostics.py --record-execution \
    --timeout-sec "$EXECUTION_RECORD_TIMEOUT_SEC" \
    --periodic-snapshot-sec 1.0 \
    --output "$ARTIFACT_DIR/${RUN_ID}_record_execution_placeholder.json" \
    --execution-output "$FIRST_DISPATCH_EXECUTION_JSONL" \
    --controller-dynamics-output "$CONTROLLER_DYNAMICS_JSONL" \
    --nav2-feedback-output "$NAV2_FEEDBACK_JSONL" \
    --local-costmap-samples-output "$LOCAL_COSTMAP_SAMPLES_JSONL" \
    --global-plan-samples-output "$GLOBAL_PLAN_SAMPLES_JSONL" \
    --collision-monitor-output "$COLLISION_MONITOR_JSONL" \
    > "$ARTIFACT_DIR/${RUN_ID}_execution_recorder_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_execution_recorder_stderr.log" &
  EXECUTION_RECORDER_PID="$!"
  PIDS+=("$EXECUTION_RECORDER_PID")
}

start_state_recorders_after_ingress() {
  python3 tools/record_explorer_state_series.py --topic /maze/goal_events \
    --output "$GOAL_EVENTS" \
    --max-samples 60 \
    --min-samples 0 \
    --timeout-sec "$(( EXPLORER_OBSERVE_SEC + 30 ))" \
    > "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stderr.log" &
  GOAL_EVENTS_PID="$!"
  PIDS+=("$GOAL_EVENTS_PID")

  python3 tools/record_explorer_state_series.py --topic /maze/explorer_state \
    --output "$EXPLORER_STATE" \
    --max-samples 220 \
    --min-samples 1 \
    --timeout-sec "$(( EXPLORER_OBSERVE_SEC + 30 ))" \
    --stop-on-terminal \
    --terminal-linger-sec 5.0 \
    > "$ARTIFACT_DIR/${RUN_ID}_explorer_state_recorder_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_explorer_state_recorder_stderr.log" &
  STATE_PID="$!"
  PIDS+=("$STATE_PID")
}

start_maze_explorer_after_ingress() {
  ros2 run tugbot_maze maze_explorer --ros-args \
    -p use_sim_time:=true \
    -p map_topic:=/map \
    -p base_frame:=base_link \
    -p map_frame:=map \
    -p action_name:=/navigate_to_pose \
    -p state_topic:=/maze/explorer_state \
    -p goal_events_topic:=/maze/goal_events \
    -p scan_topic:=/scan \
    -p goal_pose_topic:=/goal_pose \
    -p entrance_x:=0.0 \
    -p entrance_y:=0.0 \
    -p entrance_yaw:=0.0 \
    -p exit_x:=21.072562 \
    -p exit_y:=18.083566 \
    -p exit_radius:=1.2 \
    -p max_goals:=1 \
    -p near_exit_fallback_enabled:=false \
    -p startup_warmup_no_dispatch:=false \
    > "$ARTIFACT_DIR/${RUN_ID}_explorer_stdout.log" 2> "$ARTIFACT_DIR/${RUN_ID}_explorer_stderr.log" &
  EXPLORER_PID="$!"
  PIDS+=("$EXPLORER_PID")
  echo "$EXPLORER_PID" > "$ARTIFACT_DIR/${RUN_ID}_explorer.pid"
}

set -o pipefail
launch_phase57
if ! wait_for_readiness; then
  cat > "$ACTION_RESULT" <<JSON
{"phase":"Phase57 First Dispatch Navigation Timeout Diagnostics","run_id":"$RUN_ID","goal_sent":false,"success":false,"result_received":false,"reason":"readiness_gates_not_ready"}
JSON
else
  sleep 3
  {
    echo "=== timestamp ==="
    date -Is
    echo "=== ROS nodes ==="
    ros2 node list || true
    echo "=== ROS topics ==="
    ros2 topic list || true
    echo "=== ROS actions ==="
    ros2 action list || true
  } > "$ARTIFACT_DIR/${RUN_ID}_ros_graph_snapshot.txt" 2>&1

  python3 tools/analyze_phase57_first_dispatch_navigation_timeout_diagnostics.py --send-ingress-goal \
    --goal-timeout-sec "$GOAL_TIMEOUT_SEC" \
    --output "$ACTION_RESULT" \
    > "$ARTIFACT_DIR/${RUN_ID}_goal_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_goal_stderr.log" || true

  if python3 - <<PY
import json
p='$ACTION_RESULT'
data=json.load(open(p))
raise SystemExit(0 if data.get('success') else 1)
PY
  then
    start_first_dispatch_execution_recorder
    start_state_recorders_after_ingress
    sleep 2
    start_maze_explorer_after_ingress
    sleep "$EXPLORER_OBSERVE_SEC"
  fi
fi

{
  echo "=== timestamp ==="
  date -Is
  echo "=== ROS nodes ==="
  ros2 node list || true
  echo "=== ROS topics ==="
  ros2 topic list || true
  echo "=== expected post-ingress explorer grep ==="
  ros2 node list | grep -E 'maze_explorer|maze_goal_monitor|frontier_explorer|controller_server|planner_server|bt_navigator|slam_toolbox|collision_monitor|velocity_smoother' || true
} > "$ARTIFACT_DIR/${RUN_ID}_ros_graph_after_explorer.txt" 2>&1

for pid in "$STATE_PID" "$GOAL_EVENTS_PID" "$EXPLORER_PID" "$EXECUTION_RECORDER_PID"; do
  if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -TERM "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
done

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

python3 tools/analyze_phase57_first_dispatch_navigation_timeout_diagnostics.py --analyze \
  --artifact-dir "$ARTIFACT_DIR" \
  --output "$SUMMARY_JSON" \
  --action-result "$ACTION_RESULT" \
  --lifecycle-readiness "$LIFECYCLE_READINESS" \
  --action-info "$ACTION_INFO" \
  --goal-pose-info "$GOAL_POSE_INFO" \
  --explorer-state "$EXPLORER_STATE" \
  --goal-events "$GOAL_EVENTS" \
  --first-dispatch-execution "$FIRST_DISPATCH_EXECUTION_JSONL" \
  --controller-dynamics "$CONTROLLER_DYNAMICS_JSONL" \
  --nav2-feedback "$NAV2_FEEDBACK_JSONL" \
  --collision-monitor "$COLLISION_MONITOR_JSONL" \
  --launch-log "$LAUNCH_LOG" \
  --preflight "$PREFLIGHT" \
  --cleanup-processes-after "$CLEANUP_AFTER" \
  --nav2-config-diff "$NAV2_CONFIG_DIFF" \
  --first-dispatch-execution-summary "$FIRST_DISPATCH_EXECUTION_SUMMARY" \
  > "$ARTIFACT_DIR/${RUN_ID}_analyzer_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_analyzer_stderr.log" || true

cat "$SUMMARY_JSON"
