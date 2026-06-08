#!/usr/bin/env bash
set -euo pipefail

RUN_ID="phase55_ingress_then_maze_explorer_gated_startup_smoke"
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
ACCEPTANCE_RADIUS_M="${PHASE55_ACCEPTANCE_RADIUS_M:-0.35}"
RUN_TIMEOUT_SEC="${PHASE55_RUN_TIMEOUT_SEC:-300}"
RECORD_TIMEOUT_SEC="${PHASE55_RECORD_TIMEOUT_SEC:-220}"
GOAL_TIMEOUT_SEC="${PHASE55_GOAL_TIMEOUT_SEC:-100}"
READINESS_TIMEOUT_SEC="${PHASE55_READINESS_TIMEOUT_SEC:-170}"
EXPLORER_OBSERVE_SEC="${PHASE55_EXPLORER_OBSERVE_SEC:-90}"
USE_RVIZ="${PHASE55_USE_RVIZ:-false}"
MAX_GOALS="1"

if (( RUN_TIMEOUT_SEC < 150 || RUN_TIMEOUT_SEC > 360 )); then
  echo "Phase55 RUN_TIMEOUT_SEC must be bounded in [150, 360], got $RUN_TIMEOUT_SEC" >&2
  exit 2
fi
if (( RECORD_TIMEOUT_SEC < 90 || RECORD_TIMEOUT_SEC > 300 )); then
  echo "Phase55 RECORD_TIMEOUT_SEC must be bounded in [90, 300], got $RECORD_TIMEOUT_SEC" >&2
  exit 2
fi
if (( EXPLORER_OBSERVE_SEC < 30 || EXPLORER_OBSERVE_SEC > 140 )); then
  echo "Phase55 EXPLORER_OBSERVE_SEC must be bounded in [30, 140], got $EXPLORER_OBSERVE_SEC" >&2
  exit 2
fi

LAUNCH_LOG="$ARTIFACT_DIR/${RUN_ID}_launch.log"
RUNTIME_EVIDENCE="$ARTIFACT_DIR/${RUN_ID}_runtime_evidence.json"
ACTION_RESULT="$ARTIFACT_DIR/${RUN_ID}_ingress_navigate_to_pose_action_result.json"
ACTION_INFO="$ARTIFACT_DIR/${RUN_ID}_navigate_to_pose_action_info.txt"
GOAL_POSE_INFO="$ARTIFACT_DIR/${RUN_ID}_goal_pose_topic_info.txt"
LIFECYCLE_READINESS="$ARTIFACT_DIR/${RUN_ID}_lifecycle_readiness.txt"
EXPLORER_STATE="$ARTIFACT_DIR/phase55_ingress_then_maze_explorer_gated_startup_smoke_explorer_state.jsonl"
GOAL_EVENTS="$ARTIFACT_DIR/phase55_ingress_then_maze_explorer_gated_startup_smoke_goal_events.jsonl"
SUMMARY_JSON="$ARTIFACT_DIR/${RUN_ID}.json"
NAV2_CONFIG_DIFF="$ARTIFACT_DIR/${RUN_ID}_nav2_config_diff.txt"
CLEANUP_AFTER="$ARTIFACT_DIR/${RUN_ID}_cleanup_processes_after.txt"
PREFLIGHT="$ARTIFACT_DIR/${RUN_ID}_preflight.txt"
PIDS=()
LAUNCH_PID=""
RECORDER_PID=""
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
  pkill -TERM -f '[a]nalyze_phase55_ingress_then_maze_explorer_gated_startup_smoke.py --record-runtime' || true
  pkill -TERM -f '[a]nalyze_phase55_ingress_then_maze_explorer_gated_startup_smoke.py --send-goal' || true
  pkill -TERM -f '[r]ecord_explorer_state_series.py.*phase55_ingress_then_maze_explorer_gated_startup_smoke' || true
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
  (pgrep -af '[r]os2 launch|[r]os2 run tugbot_maze maze_explorer|[g]z sim|[r]viz2|[s]lam_toolbox|[m]aze_explorer|[m]aze_goal_monitor|[f]rontier_explorer|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]oute_server|[c]ollision_monitor|[d]ocking_server|[r]os_gz_bridge|[p]arameter_bridge|[s]tatic_transform_publisher|[a]nalyze_phase55_ingress_then_maze_explorer' || true) \
    | grep -v 'hermes-snap' \
    | grep -v 'pgrep -af' \
    > "$CLEANUP_AFTER" || true
}
trap cleanup EXIT

mkdir -p "$ARTIFACT_DIR"
rm -f \
  "$LAUNCH_LOG" "$RUNTIME_EVIDENCE" "$ACTION_RESULT" "$ACTION_INFO" "$GOAL_POSE_INFO" \
  "$LIFECYCLE_READINESS" "$EXPLORER_STATE" "$GOAL_EVENTS" "$SUMMARY_JSON" "$NAV2_CONFIG_DIFF" "$CLEANUP_AFTER" "$PREFLIGHT" \
  "$ARTIFACT_DIR/${RUN_ID}_ros_graph_snapshot.txt" "$ARTIFACT_DIR/${RUN_ID}_ros_graph_after_explorer.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_ros_graph_final.txt" "$ARTIFACT_DIR/${RUN_ID}_goal_stdout.json" \
  "$ARTIFACT_DIR/${RUN_ID}_goal_stderr.log" "$ARTIFACT_DIR/${RUN_ID}_runtime_recorder_stdout.json" \
  "$ARTIFACT_DIR/${RUN_ID}_runtime_recorder_stderr.log" "$ARTIFACT_DIR/${RUN_ID}_explorer_stdout.log" \
  "$ARTIFACT_DIR/${RUN_ID}_explorer_stderr.log" "$ARTIFACT_DIR/${RUN_ID}_explorer.pid"

if [[ ! -f "$WORLD" ]]; then echo "missing active scaled2x world: $WORLD" >&2; exit 3; fi
if [[ ! -f "$MAZE_CONFIG" ]]; then echo "missing active maze metadata: $MAZE_CONFIG" >&2; exit 4; fi
if [[ ! -f "$SLAM_PARAMS" || ! -f "$NAV2_PARAMS" ]]; then echo "missing SLAM or Nav2 params" >&2; exit 5; fi
if [[ "$WORLD" != *"tugbot_maze_world_20260528_clean_scaled2x.sdf" ]]; then echo "refusing non-active world: $WORLD" >&2; exit 6; fi
if [[ "$MAZE_CONFIG" != *"maze_20260528_scaled_instance.yaml" ]]; then echo "refusing non-active maze metadata: $MAZE_CONFIG" >&2; exit 7; fi

{
  echo "# Phase55 Ingress-then-maze_explorer Gated Startup Smoke"
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
  echo "guardrail_violation_class=GUARDRAIL_VIOLATION_UNBOUNDED_OR_SUCCESS_CLAIM"
  echo "guardrails=bounded startup smoke only; no clearance_radius_m tuning; no map threshold tuning; no Nav2/MPPI/controller parameter edits; no fallback/terminal acceptance; no old scaffold world/map; no autonomous exploration success claim; first dispatch is not exit success."
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

launch_phase55() {
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

start_recorders() {
  python3 tools/analyze_phase55_ingress_then_maze_explorer_gated_startup_smoke.py --record-runtime "$RECORD_TIMEOUT_SEC" \
    --snapshot-period-sec 1.0 \
    --output "$RUNTIME_EVIDENCE" \
    > "$ARTIFACT_DIR/${RUN_ID}_runtime_recorder_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_runtime_recorder_stderr.log" &
  RECORDER_PID="$!"
  PIDS+=("$RECORDER_PID")
}

start_state_recorders_after_ingress() {
  python3 tools/record_explorer_state_series.py --topic /maze/goal_events \
    --output "$GOAL_EVENTS" \
    --max-samples 40 \
    --min-samples 0 \
    --timeout-sec "$(( EXPLORER_OBSERVE_SEC + 30 ))" \
    > "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stderr.log" &
  GOAL_EVENTS_PID="$!"
  PIDS+=("$GOAL_EVENTS_PID")

  python3 tools/record_explorer_state_series.py --topic /maze/explorer_state \
    --output "$EXPLORER_STATE" \
    --max-samples 180 \
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
launch_phase55
if ! wait_for_readiness; then
  cat > "$ACTION_RESULT" <<JSON
{"phase":"Phase55 Ingress-then-maze_explorer Gated Startup Smoke","run_id":"$RUN_ID","goal_sent":false,"success":false,"result_received":false,"reason":"readiness_gates_not_ready"}
JSON
else
  start_recorders
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

  python3 tools/analyze_phase55_ingress_then_maze_explorer_gated_startup_smoke.py --send-goal \
    --goal-timeout-sec "$GOAL_TIMEOUT_SEC" \
    --output "$ACTION_RESULT" \
    > "$ARTIFACT_DIR/${RUN_ID}_goal_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_goal_stderr.log" || true

  if python3 - <<PY
import json, math
p='$ACTION_RESULT'
data=json.load(open(p))
raise SystemExit(0 if data.get('success') else 1)
PY
  then
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
  ros2 node list | grep -E 'maze_explorer|maze_goal_monitor|frontier_explorer|controller_server|planner_server|bt_navigator|slam_toolbox' || true
} > "$ARTIFACT_DIR/${RUN_ID}_ros_graph_after_explorer.txt" 2>&1

for pid in "$STATE_PID" "$GOAL_EVENTS_PID" "$EXPLORER_PID"; do
  if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -TERM "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
done
if [[ -n "${RECORDER_PID:-}" ]] && kill -0 "$RECORDER_PID" 2>/dev/null; then
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

python3 tools/analyze_phase55_ingress_then_maze_explorer_gated_startup_smoke.py --analyze \
  --artifact-dir "$ARTIFACT_DIR" \
  --output "$SUMMARY_JSON" \
  --runtime-evidence "$RUNTIME_EVIDENCE" \
  --action-result "$ACTION_RESULT" \
  --lifecycle-readiness "$LIFECYCLE_READINESS" \
  --action-info "$ACTION_INFO" \
  --goal-pose-info "$GOAL_POSE_INFO" \
  --explorer-state "$EXPLORER_STATE" \
  --goal-events "$GOAL_EVENTS" \
  --ros-graph-snapshot "$ARTIFACT_DIR/${RUN_ID}_ros_graph_snapshot.txt" \
  --ros-graph-final "$ARTIFACT_DIR/${RUN_ID}_ros_graph_final.txt" \
  --launch-log "$LAUNCH_LOG" \
  --preflight "$PREFLIGHT" \
  --cleanup-processes-after "$CLEANUP_AFTER" \
  > "$ARTIFACT_DIR/${RUN_ID}_analyzer_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_analyzer_stderr.log" || true

cat "$SUMMARY_JSON"
