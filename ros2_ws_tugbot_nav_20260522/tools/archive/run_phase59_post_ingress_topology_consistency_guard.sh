#!/usr/bin/env bash
set -euo pipefail

RUN_ID="phase59_post_ingress_topology_consistency_guard"
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
REPLAY_COUNT="${PHASE59_REPLAY_COUNT:-2}"
RUN_TIMEOUT_SEC="${PHASE59_RUN_TIMEOUT_SEC:-300}"
GOAL_TIMEOUT_SEC="${PHASE59_GOAL_TIMEOUT_SEC:-100}"
READINESS_TIMEOUT_SEC="${PHASE59_READINESS_TIMEOUT_SEC:-170}"
EXPLORER_OBSERVE_SEC="${PHASE59_EXPLORER_OBSERVE_SEC:-95}"
STABILITY_RECORD_TIMEOUT_SEC="${PHASE59_STABILITY_RECORD_TIMEOUT_SEC:-120}"
TOPOLOGY_CONSISTENCY_REQUIRED_NO_CANDIDATE_FRAMES="${PHASE59_TOPOLOGY_CONSISTENCY_REQUIRED_NO_CANDIDATE_FRAMES:-2}"
TOPOLOGY_CONSISTENCY_WINDOW_SEC="${PHASE59_TOPOLOGY_CONSISTENCY_WINDOW_SEC:-4.0}"
USE_RVIZ="${PHASE59_USE_RVIZ:-false}"
MAX_GOALS="1"

if (( REPLAY_COUNT < 1 || REPLAY_COUNT > 3 )); then
  echo "Phase59 REPLAY_COUNT must be bounded in [1, 3], got $REPLAY_COUNT" >&2
  exit 2
fi
if (( RUN_TIMEOUT_SEC < 150 || RUN_TIMEOUT_SEC > 360 )); then
  echo "Phase59 RUN_TIMEOUT_SEC must be bounded in [150, 360], got $RUN_TIMEOUT_SEC" >&2
  exit 2
fi
if (( EXPLORER_OBSERVE_SEC < 30 || EXPLORER_OBSERVE_SEC > 140 )); then
  echo "Phase59 EXPLORER_OBSERVE_SEC must be bounded in [30, 140], got $EXPLORER_OBSERVE_SEC" >&2
  exit 2
fi
if (( STABILITY_RECORD_TIMEOUT_SEC < 45 || STABILITY_RECORD_TIMEOUT_SEC > 160 )); then
  echo "Phase59 STABILITY_RECORD_TIMEOUT_SEC must be bounded in [45, 160], got $STABILITY_RECORD_TIMEOUT_SEC" >&2
  exit 2
fi

SUMMARY_JSON="$ARTIFACT_DIR/${RUN_ID}.json"
NAV2_CONFIG_DIFF="$ARTIFACT_DIR/${RUN_ID}_nav2_config_diff.txt"
CLEANUP_AFTER="$ARTIFACT_DIR/${RUN_ID}_cleanup_processes_after.txt"
PREFLIGHT="$ARTIFACT_DIR/${RUN_ID}_preflight.txt"
PIDS=()
LAUNCH_PID=""
STABILITY_RECORDER_PID=""
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
  pkill -TERM -f '[a]nalyze_phase59_post_ingress_topology_consistency_guard.py --record-stability' || true
  pkill -TERM -f '[a]nalyze_phase59_post_ingress_topology_consistency_guard.py --send-ingress-goal' || true
  pkill -TERM -f '[r]ecord_explorer_state_series.py.*phase59_post_ingress_topology_consistency_guard' || true
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
  (pgrep -af '[r]os2 launch|[r]os2 run tugbot_maze maze_explorer|[g]z sim|[r]viz2|[s]lam_toolbox|[m]aze_explorer|[m]aze_goal_monitor|[f]rontier_explorer|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]oute_server|[c]ollision_monitor|[d]ocking_server|[r]os_gz_bridge|[p]arameter_bridge|[s]tatic_transform_publisher|[a]nalyze_phase59_post_ingress_topology_consistency_guard' || true) \
    | grep -v 'hermes-snap' \
    | grep -v 'pgrep -af' \
    > "$CLEANUP_AFTER" || true
}
trap cleanup EXIT

mkdir -p "$ARTIFACT_DIR"
rm -rf "$ARTIFACT_DIR"/replay_*
rm -f "$SUMMARY_JSON" "$NAV2_CONFIG_DIFF" "$CLEANUP_AFTER" "$PREFLIGHT" \
  "$ARTIFACT_DIR/${RUN_ID}_analyzer_stdout.json" "$ARTIFACT_DIR/${RUN_ID}_analyzer_stderr.log"

if [[ ! -f "$WORLD" ]]; then echo "missing active scaled2x world: $WORLD" >&2; exit 3; fi
if [[ ! -f "$MAZE_CONFIG" ]]; then echo "missing active maze metadata: $MAZE_CONFIG" >&2; exit 4; fi
if [[ ! -f "$SLAM_PARAMS" || ! -f "$NAV2_PARAMS" ]]; then echo "missing SLAM or Nav2 params" >&2; exit 5; fi
if [[ "$WORLD" != *"tugbot_maze_world_20260528_clean_scaled2x.sdf" ]]; then echo "refusing non-active world: $WORLD" >&2; exit 6; fi
if [[ "$MAZE_CONFIG" != *"maze_20260528_scaled_instance.yaml" ]]; then echo "refusing non-active maze metadata: $MAZE_CONFIG" >&2; exit 7; fi

{
  echo "# Phase59 Post-Ingress Multi-frame Topology Consistency Guard"
  echo "timestamp=$(date -Is)"
  echo "run_id=$RUN_ID"
  echo "world=$WORLD"
  echo "maze_config=$MAZE_CONFIG"
  echo "slam_params=$SLAM_PARAMS"
  echo "nav2_params=$NAV2_PARAMS"
  echo "ingress_waypoint_map=x=${INGRESS_X},y=${INGRESS_Y},yaw=${INGRESS_YAW}"
  echo "replay_count=$REPLAY_COUNT"
  echo "max_goals=1"
  echo "near_exit_fallback_enabled=false"
  echo "startup_warmup_no_dispatch=false"
  echo "topology_consistency_enabled=true"
  echo "topology_consistency_required_no_candidate_frames=$TOPOLOGY_CONSISTENCY_REQUIRED_NO_CANDIDATE_FRAMES"
  echo "topology_consistency_window_sec=$TOPOLOGY_CONSISTENCY_WINDOW_SEC"
  echo "phase58_baseline=CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE"
  echo "guardrail_violation_class=GUARDRAIL_VIOLATION_STRATEGY_CHANGED"
  echo "guardrails=bounded replay only; no Nav2/MPPI/controller parameter edits; no clearance_radius_m tuning; no map sufficiency threshold tuning; no branch selection strategy change; no fallback/terminal acceptance; no old scaffold world/map; no autonomous exploration success claim; first dispatch is not exit success; does not attribute first-dispatch timeout."
  echo "maze_explorer_command=ros2 run tugbot_maze maze_explorer --ros-args -p max_goals:=1 -p near_exit_fallback_enabled:=false -p startup_warmup_no_dispatch:=false -p topology_consistency_enabled:=true -p topology_consistency_required_no_candidate_frames:=$TOPOLOGY_CONSISTENCY_REQUIRED_NO_CANDIDATE_FRAMES -p topology_consistency_window_sec:=$TOPOLOGY_CONSISTENCY_WINDOW_SEC"
  echo "nav2_config_diff_begin"
  git diff -- src/tugbot_navigation/config | tee "$NAV2_CONFIG_DIFF" || true
  echo "nav2_config_diff_end"
} > "$PREFLIGHT"

set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u
ros2 daemon stop >/dev/null 2>&1 || true

launch_replay() {
  local replay_dir="$1"
  (
    timeout --preserve-status "$RUN_TIMEOUT_SEC" ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py \
      world_sdf:="$WORLD" \
      slam_params_file:="$SLAM_PARAMS" \
      params_file:="$NAV2_PARAMS" \
      headless:=true \
      use_rviz:="${USE_RVIZ}" \
      use_sim_time:=true \
      autostart:=true \
      2>&1 | tee "$replay_dir/${RUN_ID}_${replay_dir##*/}_launch.log"
  ) &
  LAUNCH_PID="$!"
  PIDS+=("$LAUNCH_PID")
  echo "$LAUNCH_PID" > "$replay_dir/${RUN_ID}_${replay_dir##*/}_launch.pid"
}

wait_for_readiness() {
  local replay_dir="$1"
  local replay_id="${replay_dir##*/}"
  local action_info="$replay_dir/${RUN_ID}_${replay_id}_navigate_to_pose_action_info.txt"
  local goal_pose_info="$replay_dir/${RUN_ID}_${replay_id}_goal_pose_topic_info.txt"
  local lifecycle_readiness="$replay_dir/${RUN_ID}_${replay_id}_lifecycle_readiness.txt"
  local ready=0
  local start_sec
  start_sec=$(date +%s)
  : > "$lifecycle_readiness"
  while (( $(date +%s) - start_sec < READINESS_TIMEOUT_SEC )); do
    ctrl_tmp="$replay_dir/${RUN_ID}_${replay_id}_controller_lifecycle.tmp"
    planner_tmp="$replay_dir/${RUN_ID}_${replay_id}_planner_lifecycle.tmp"
    bt_tmp="$replay_dir/${RUN_ID}_${replay_id}_bt_lifecycle.tmp"
    (timeout 5 ros2 lifecycle get /controller_server || true) > "$ctrl_tmp" 2>&1
    (timeout 5 ros2 lifecycle get /planner_server || true) > "$planner_tmp" 2>&1
    (timeout 5 ros2 lifecycle get /bt_navigator || true) > "$bt_tmp" 2>&1
    (timeout 5 ros2 action info /navigate_to_pose || true) > "$action_info" 2>&1
    (timeout 5 ros2 topic info /goal_pose || true) > "$goal_pose_info" 2>&1
    {
      echo "# gate snapshot $(date -Is)"
      echo "/controller_server"; cat "$ctrl_tmp"
      echo "/planner_server"; cat "$planner_tmp"
      echo "/bt_navigator"; cat "$bt_tmp"
      echo "/navigate_to_pose"; cat "$action_info"
      echo "/goal_pose"; cat "$goal_pose_info"
    } >> "$lifecycle_readiness" 2>&1
    if grep -q 'active \[3\]' "$ctrl_tmp" \
      && grep -q 'active \[3\]' "$planner_tmp" \
      && grep -q 'active \[3\]' "$bt_tmp" \
      && grep -q 'Action servers: 1' "$action_info" \
      && grep -q 'Subscription count: 1' "$goal_pose_info"; then
      ready=1
      break
    fi
    sleep 5
  done
  rm -f "$replay_dir/${RUN_ID}_${replay_id}"_*_lifecycle.tmp
  echo "readiness_ready=$ready" >> "$lifecycle_readiness"
  if (( ready == 1 )); then return 0; fi
  return 1
}

start_stability_recorder() {
  local replay_dir="$1"
  local replay_id="${replay_dir##*/}"
  python3 tools/analyze_phase59_post_ingress_topology_consistency_guard.py --record-stability \
    --timeout-sec "$STABILITY_RECORD_TIMEOUT_SEC" \
    --periodic-snapshot-sec 1.0 \
    --output "$replay_dir/${RUN_ID}_${replay_id}_stability_evidence.jsonl" \
    > "$replay_dir/${RUN_ID}_${replay_id}_stability_recorder_stdout.json" 2> "$replay_dir/${RUN_ID}_${replay_id}_stability_recorder_stderr.log" &
  STABILITY_RECORDER_PID="$!"
  PIDS+=("$STABILITY_RECORDER_PID")
}

start_state_recorders_after_ingress() {
  local replay_dir="$1"
  local replay_id="${replay_dir##*/}"
  python3 tools/record_explorer_state_series.py --topic /maze/goal_events \
    --output "$replay_dir/${RUN_ID}_${replay_id}_goal_events.jsonl" \
    --max-samples 80 \
    --min-samples 0 \
    --timeout-sec "$(( EXPLORER_OBSERVE_SEC + 30 ))" \
    > "$replay_dir/${RUN_ID}_${replay_id}_goal_events_recorder_stdout.json" 2> "$replay_dir/${RUN_ID}_${replay_id}_goal_events_recorder_stderr.log" &
  GOAL_EVENTS_PID="$!"
  PIDS+=("$GOAL_EVENTS_PID")

  python3 tools/record_explorer_state_series.py --topic /maze/explorer_state \
    --output "$replay_dir/${RUN_ID}_${replay_id}_explorer_state.jsonl" \
    --max-samples 220 \
    --min-samples 1 \
    --timeout-sec "$(( EXPLORER_OBSERVE_SEC + 30 ))" \
    --stop-on-terminal \
    --terminal-linger-sec 5.0 \
    > "$replay_dir/${RUN_ID}_${replay_id}_explorer_state_recorder_stdout.json" 2> "$replay_dir/${RUN_ID}_${replay_id}_explorer_state_recorder_stderr.log" &
  STATE_PID="$!"
  PIDS+=("$STATE_PID")
}

start_maze_explorer_after_ingress() {
  local replay_dir="$1"
  local replay_id="${replay_dir##*/}"
  date +%s.%N > "$replay_dir/${RUN_ID}_${replay_id}_explorer_start_wall_time.txt"
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
    -p topology_consistency_enabled:=true \
    -p topology_consistency_required_no_candidate_frames:="$TOPOLOGY_CONSISTENCY_REQUIRED_NO_CANDIDATE_FRAMES" \
    -p topology_consistency_window_sec:="$TOPOLOGY_CONSISTENCY_WINDOW_SEC" \
    > "$replay_dir/${RUN_ID}_${replay_id}_explorer_stdout.log" 2> "$replay_dir/${RUN_ID}_${replay_id}_explorer_stderr.log" &
  EXPLORER_PID="$!"
  PIDS+=("$EXPLORER_PID")
  echo "$EXPLORER_PID" > "$replay_dir/${RUN_ID}_${replay_id}_explorer.pid"
}

for replay_index in $(seq 1 "$REPLAY_COUNT"); do
  replay_id="replay_$(printf '%02d' "$replay_index")"
  replay_dir="$ARTIFACT_DIR/$replay_id"
  mkdir -p "$replay_dir"
  PIDS=()
  LAUNCH_PID=""; STABILITY_RECORDER_PID=""; STATE_PID=""; GOAL_EVENTS_PID=""; EXPLORER_PID=""
  echo "phase59 replay $replay_index/$REPLAY_COUNT starting" >&2
  launch_replay "$replay_dir"
  if ! wait_for_readiness "$replay_dir"; then
    cat > "$replay_dir/${RUN_ID}_${replay_id}_ingress_navigate_to_pose_action_result.json" <<JSON
{"phase":"Phase59 Post-Ingress Multi-frame Topology Consistency Guard","run_id":"$RUN_ID","replay_id":"$replay_id","goal_sent":false,"success":false,"result_received":false,"reason":"readiness_gates_not_ready"}
JSON
  else
    sleep 3
    {
      echo "=== timestamp ==="; date -Is
      echo "=== ROS nodes ==="; ros2 node list || true
      echo "=== ROS topics ==="; ros2 topic list || true
      echo "=== ROS actions ==="; ros2 action list || true
    } > "$replay_dir/${RUN_ID}_${replay_id}_ros_graph_snapshot.txt" 2>&1

    python3 tools/analyze_phase59_post_ingress_topology_consistency_guard.py --send-ingress-goal \
      --goal-timeout-sec "$GOAL_TIMEOUT_SEC" \
      --output "$replay_dir/${RUN_ID}_${replay_id}_ingress_navigate_to_pose_action_result.json" \
      > "$replay_dir/${RUN_ID}_${replay_id}_goal_stdout.json" 2> "$replay_dir/${RUN_ID}_${replay_id}_goal_stderr.log" || true

    if python3 - <<PY
import json
p='$replay_dir/${RUN_ID}_${replay_id}_ingress_navigate_to_pose_action_result.json'
data=json.load(open(p))
raise SystemExit(0 if data.get('success') else 1)
PY
    then
      start_stability_recorder "$replay_dir"
      start_state_recorders_after_ingress "$replay_dir"
      sleep 2
      start_maze_explorer_after_ingress "$replay_dir"
      sleep "$EXPLORER_OBSERVE_SEC"
    fi
  fi

  {
    echo "=== timestamp ==="; date -Is
    echo "=== ROS nodes ==="; ros2 node list || true
    echo "=== ROS topics ==="; ros2 topic list || true
    echo "=== expected post-ingress explorer grep ==="
    ros2 node list | grep -E 'maze_explorer|maze_goal_monitor|frontier_explorer|controller_server|planner_server|bt_navigator|slam_toolbox|collision_monitor|velocity_smoother' || true
  } > "$replay_dir/${RUN_ID}_${replay_id}_ros_graph_after_explorer.txt" 2>&1

  for pid in "$STATE_PID" "$GOAL_EVENTS_PID" "$EXPLORER_PID" "$STABILITY_RECORDER_PID" "$LAUNCH_PID"; do
    if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
  done
  cleanup
  trap cleanup EXIT
  sleep 5
  echo "phase59 replay $replay_index/$REPLAY_COUNT completed" >&2
done

cleanup
trap - EXIT

python3 tools/analyze_phase59_post_ingress_topology_consistency_guard.py --analyze \
  --artifact-dir "$ARTIFACT_DIR" \
  --output "$SUMMARY_JSON" \
  > "$ARTIFACT_DIR/${RUN_ID}_analyzer_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_analyzer_stderr.log" || true

cat "$SUMMARY_JSON"
