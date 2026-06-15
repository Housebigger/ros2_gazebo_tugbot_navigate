#!/usr/bin/env bash
set -eo pipefail

# Phase65 Ingress Goal Depth Adjustment / Inner Entrance Staging Point
# Guardrails: bounded runtime only; max_goals=1; no Nav2/MPPI/controller parameter edits;
# no clearance_radius_m tuning; no map sufficiency threshold tuning; no branch selection scoring change;
# no target projection integration; no fallback/terminal acceptance change;
# no autonomous exploration success claim; no exit success claim.

RUN_ID="phase65_ingress_goal_depth_adjustment_inner_staging"
PHASE="Phase65 Ingress Goal Depth Adjustment / Inner Entrance Staging Point"
# Analyzer function contract: send_inner_ingress_goal is exposed via --send-inner-ingress-goal.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

REPLAY_COUNT="${PHASE65_REPLAY_COUNT:-1}"
if (( REPLAY_COUNT < 1 || REPLAY_COUNT > 3 )); then
  echo "PHASE65_REPLAY_COUNT must be 1..3 for bounded runtime only" >&2
  exit 2
fi

ACTIVE_INGRESS_X="1.0"
ACTIVE_INGRESS_Y="0.0"
ACTIVE_INGRESS_YAW="0.0"
INNER_INGRESS_X="2.0"
INNER_INGRESS_Y="0.0"
INNER_INGRESS_YAW="0.0"
MAX_GOALS="1"
WORLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
MAZE_CONFIG="$ROOT/src/tugbot_maze/config/maze_20260528_scaled_instance.yaml"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
NAV2_PARAMS="$ROOT/src/tugbot_navigation/config/nav2_slam_params.yaml"
ARTIFACT_DIR="$ROOT/log/${RUN_ID}"
SUMMARY_JSON="$ARTIFACT_DIR/${RUN_ID}.json"
NAV2_CONFIG_DIFF="$ARTIFACT_DIR/${RUN_ID}_nav2_config_diff.txt"
CLEANUP_AFTER="$ARTIFACT_DIR/${RUN_ID}_cleanup_processes_after.txt"
PREFLIGHT="$ARTIFACT_DIR/${RUN_ID}_preflight.txt"
RUN_TIMEOUT_SEC="${PHASE65_RUN_TIMEOUT_SEC:-180}"
READINESS_TIMEOUT_SEC="${PHASE65_READINESS_TIMEOUT_SEC:-70}"
POST_READINESS_SETTLE_SEC="${PHASE65_POST_READINESS_SETTLE_SEC:-10}"
GOAL_TIMEOUT_SEC="${PHASE65_GOAL_TIMEOUT_SEC:-90}"
EXPLORER_OBSERVE_SEC="${PHASE65_EXPLORER_OBSERVE_SEC:-35}"
EXECUTION_RECORD_TIMEOUT_SEC="${PHASE65_EXECUTION_RECORD_TIMEOUT_SEC:-45}"
TOPOLOGY_CONSISTENCY_REQUIRED_NO_CANDIDATE_FRAMES="${PHASE65_TOPOLOGY_CONSISTENCY_REQUIRED_NO_CANDIDATE_FRAMES:-2}"
TOPOLOGY_CONSISTENCY_WINDOW_SEC="${PHASE65_TOPOLOGY_CONSISTENCY_WINDOW_SEC:-4.0}"
PIDS=()
LAUNCH_PID=""
STATE_PID=""
GOAL_EVENTS_PID=""
EXPLORER_PID=""
EXECUTION_RECORDER_PID=""

cleanup() {
  set +e
  trap - EXIT
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  sleep 2
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
  pkill -TERM -f '[a]nalyze_phase65_ingress_goal_depth_adjustment_inner_staging.py --send-inner-ingress-goal' || true
  pkill -TERM -f '[r]ecord_explorer_state_series.py.*phase65_ingress_goal_depth_adjustment_inner_staging' || true
  pkill -TERM -f '[a]nalyze_phase62_first_dispatch_local_cost_traversability_diagnostics.py --record-execution' || true
  pkill -TERM -f '[r]os2 run tugbot_maze maze_explorer' || true
  pkill -TERM -f '[m]aze_explorer' || true
  pkill -TERM -f '[m]aze_goal_monitor|[f]rontier_explorer' || true
  pkill -TERM -f '[r]os2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py' || true
  pkill -TERM -f '[g]z sim|[r]os_gz_bridge|[p]arameter_bridge|[s]lam_toolbox|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]oute_server|[c]ollision_monitor|[d]ocking_server' || true
  sleep 2
  pkill -KILL -f '[g]z sim|[r]os_gz_bridge|[p]arameter_bridge|[s]lam_toolbox|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]oute_server|[c]ollision_monitor|[d]ocking_server|[m]aze_explorer' || true
  if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
  fi
  mkdir -p "$ARTIFACT_DIR"
  (pgrep -af '[r]os2 launch|[r]os2 run tugbot_maze maze_explorer|[g]z sim|[r]viz2|[s]lam_toolbox|[m]aze_explorer|[m]aze_goal_monitor|[f]rontier_explorer|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]oute_server|[c]ollision_monitor|[d]ocking_server|[r]os_gz_bridge|[p]arameter_bridge|[s]tatic_transform_publisher|[a]nalyze_phase65_ingress_goal_depth_adjustment_inner_staging|[a]nalyze_phase62_first_dispatch_local_cost_traversability_diagnostics' || true) \
    | grep -v 'hermes-snap' \
    | grep -v 'pgrep -af' \
    > "$CLEANUP_AFTER" || true
}
trap cleanup EXIT

mkdir -p "$ARTIFACT_DIR"
rm -rf "$ARTIFACT_DIR"/replay_*

# Source ROS without set -u; ROS setup files may read unset AMENT_TRACE_SETUP_FILES.
source /opt/ros/jazzy/setup.bash
if [[ -f "$ROOT/install/setup.bash" ]]; then
  source "$ROOT/install/setup.bash"
fi

{
  echo "run_id=$RUN_ID"
  echo "active_world=$WORLD"
  echo "maze_config=$MAZE_CONFIG"
  echo "slam_params=$SLAM_PARAMS"
  echo "nav2_params=$NAV2_PARAMS"
  echo "old_ingress_waypoint_map=x=${ACTIVE_INGRESS_X},y=${ACTIVE_INGRESS_Y},yaw=${ACTIVE_INGRESS_YAW}"
  echo "inner_ingress_waypoint_map=x=${INNER_INGRESS_X},y=${INNER_INGRESS_Y},yaw=${INNER_INGRESS_YAW}"
  echo "ingress_waypoint_map=x=${INNER_INGRESS_X},y=${INNER_INGRESS_Y},yaw=${INNER_INGRESS_YAW}"
  echo "inner_ingress_depth_adjustment_m=1.0"
  echo "replay_count=$REPLAY_COUNT"
  echo "max_goals=1"
  echo "MAX_GOALS=$MAX_GOALS"
  echo "near_exit_fallback_enabled=false"
  echo "startup_warmup_no_dispatch=false"
  echo "phase61_baseline=SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED"
  echo "phase62_baseline=CORRIDOR_TOO_NARROW;LOCAL_COSTMAP_INFLATION_DOMINANT"
  echo "phase64_baseline=GEOMETRY_FEASIBILITY_BLOCKED"
  echo "phase65_hypothesis=ENTRY_STAGING_TOO_SHALLOW"
  echo "guardrail_violation_class=GUARDRAIL_VIOLATION_STRATEGY_CHANGED"
  echo "guardrails=bounded runtime only; max_goals=1; no Nav2/MPPI/controller parameter edits; no clearance_radius_m tuning; no map sufficiency threshold tuning; no branch selection scoring change; no target projection integration; no fallback/terminal acceptance change; no autonomous exploration success claim; no exit success claim."
  echo "maze_explorer_command=ros2 run tugbot_maze maze_explorer --ros-args -p max_goals:=1 -p near_exit_fallback_enabled:=false -p startup_warmup_no_dispatch:=false -p topology_consistency_enabled:=true -p post_ingress_single_open_exception_enabled:=true"
  echo "nav2_config_diff_begin"
  git diff -- src/tugbot_navigation/config | tee "$NAV2_CONFIG_DIFF" || true
  echo "nav2_config_diff_end"
} > "$PREFLIGHT"

wait_for_readiness() {
  local replay_dir="$1"
  local start
  start=$(date +%s)
  while (( $(date +%s) - start < READINESS_TIMEOUT_SEC )); do
    if ros2 action list 2>/dev/null | grep -q '^/navigate_to_pose$' \
      && ros2 node list 2>/dev/null | grep -q '/bt_navigator' \
      && ros2 node list 2>/dev/null | grep -q '/controller_server' \
      && ros2 topic list 2>/dev/null | grep -q '^/map$' \
      && ros2 topic list 2>/dev/null | grep -q '^/local_costmap/costmap$'; then
      {
        echo "ready_at=$(date -Is)"
        ros2 node list || true
        ros2 action list || true
        ros2 topic list || true
      } > "$replay_dir/${RUN_ID}_${replay_dir##*/}_readiness_ready_snapshot.txt" 2>&1
      return 0
    fi
    sleep 2
  done
  {
    echo "timeout_at=$(date -Is)"
    ros2 node list || true
    ros2 action list || true
    ros2 topic list || true
  } > "$replay_dir/${RUN_ID}_${replay_dir##*/}_readiness_timeout_snapshot.txt" 2>&1
  return 1
}

launch_replay() {
  local replay_dir="$1"
  (
    timeout --preserve-status "$RUN_TIMEOUT_SEC" ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py \
      world_sdf:="$WORLD" \
      slam_params_file:="$SLAM_PARAMS" \
      params_file:="$NAV2_PARAMS" \
      use_rviz:=false
  ) > "$replay_dir/${RUN_ID}_${replay_dir##*/}_launch.log" 2>&1 &
  LAUNCH_PID="$!"
  PIDS+=("$LAUNCH_PID")
  echo "$LAUNCH_PID" > "$replay_dir/${RUN_ID}_${replay_dir##*/}_launch.pid"
}

start_state_recorders_after_ingress() {
  local replay_dir="$1"
  local replay_id="${replay_dir##*/}"
  python3 tools/record_explorer_state_series.py --topic /maze/goal_events \
    --output "$replay_dir/${RUN_ID}_${replay_id}_goal_events.jsonl" \
    --max-samples 160 \
    --min-samples 0 \
    --timeout-sec "$EXPLORER_OBSERVE_SEC" \
    --stop-on-terminal > "$replay_dir/${RUN_ID}_${replay_id}_goal_events_recorder_stdout.json" 2> "$replay_dir/${RUN_ID}_${replay_id}_goal_events_recorder_stderr.log" &
  GOAL_EVENTS_PID="$!"
  PIDS+=("$GOAL_EVENTS_PID")

  python3 tools/record_explorer_state_series.py --topic /maze/explorer_state \
    --output "$replay_dir/${RUN_ID}_${replay_id}_explorer_state.jsonl" \
    --max-samples 260 \
    --min-samples 1 \
    --timeout-sec "$EXPLORER_OBSERVE_SEC" \
    --stop-on-terminal > "$replay_dir/${RUN_ID}_${replay_id}_state_recorder_stdout.json" 2> "$replay_dir/${RUN_ID}_${replay_id}_state_recorder_stderr.log" &
  STATE_PID="$!"
  PIDS+=("$STATE_PID")
}

start_first_dispatch_execution_recorder() {
  local replay_dir="$1"
  python3 tools/analyze_phase62_first_dispatch_local_cost_traversability_diagnostics.py --record-execution \
    --timeout-sec "$EXECUTION_RECORD_TIMEOUT_SEC" \
    --output-jsonl "$replay_dir/first_dispatch_execution.jsonl" \
    --summary-output "$replay_dir/first_dispatch_execution_summary.json" \
    > "$replay_dir/${RUN_ID}_${replay_dir##*/}_execution_recorder_stdout.json" 2> "$replay_dir/${RUN_ID}_${replay_dir##*/}_execution_recorder_stderr.log" &
  EXECUTION_RECORDER_PID="$!"
  PIDS+=("$EXECUTION_RECORDER_PID")
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
    -p post_ingress_single_open_exception_enabled:=true \
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
  LAUNCH_PID=""; STATE_PID=""; GOAL_EVENTS_PID=""; EXPLORER_PID=""; EXECUTION_RECORDER_PID=""
  echo "phase65 replay $replay_index/$REPLAY_COUNT starting" >&2
  launch_replay "$replay_dir"
  if ! wait_for_readiness "$replay_dir"; then
    cat > "$replay_dir/${RUN_ID}_${replay_id}_inner_ingress_navigate_to_pose_action_result.json" <<JSON
{"phase":"$PHASE","run_id":"$RUN_ID","replay_id":"$replay_id","goal_sent":false,"success":false,"result_received":false,"reason":"readiness_gates_not_ready"}
JSON
  else
    sleep "$POST_READINESS_SETTLE_SEC"
    {
      echo "=== timestamp ==="; date -Is
      echo "=== ROS nodes ==="; ros2 node list || true
      echo "=== ROS topics ==="; ros2 topic list || true
      echo "=== ROS actions ==="; ros2 action list || true
    } > "$replay_dir/${RUN_ID}_${replay_id}_ros_graph_snapshot.txt" 2>&1

    python3 tools/analyze_phase65_ingress_goal_depth_adjustment_inner_staging.py --send-inner-ingress-goal \
      --goal-timeout-sec "$GOAL_TIMEOUT_SEC" \
      --output "$replay_dir/${RUN_ID}_${replay_id}_inner_ingress_navigate_to_pose_action_result.json" \
      > "$replay_dir/${RUN_ID}_${replay_id}_goal_stdout.json" 2> "$replay_dir/${RUN_ID}_${replay_id}_goal_stderr.log" || true

    if python3 - <<PY
import json
p='$replay_dir/${RUN_ID}_${replay_id}_inner_ingress_navigate_to_pose_action_result.json'
data=json.load(open(p))
raise SystemExit(0 if data.get('success') else 1)
PY
    then
      start_state_recorders_after_ingress "$replay_dir"
      start_first_dispatch_execution_recorder "$replay_dir"
      sleep 2
      start_maze_explorer_after_ingress "$replay_dir"
      sleep "$EXPLORER_OBSERVE_SEC"
    fi
  fi

  {
    echo "=== timestamp ==="; date -Is
    echo "=== ROS nodes ==="; ros2 node list || true
    echo "=== ROS topics ==="; ros2 topic list || true
    echo "=== expected post-inner-ingress explorer grep ==="
    ros2 node list | grep -E 'maze_explorer|maze_goal_monitor|frontier_explorer|controller_server|planner_server|bt_navigator|slam_toolbox|collision_monitor|velocity_smoother' || true
  } > "$replay_dir/${RUN_ID}_${replay_id}_ros_graph_after_explorer.txt" 2>&1

  for pid in "$STATE_PID" "$GOAL_EVENTS_PID" "$EXPLORER_PID" "$EXECUTION_RECORDER_PID" "$LAUNCH_PID"; do
    if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
  done
  cleanup
  trap cleanup EXIT
  sleep 5
  echo "phase65 replay $replay_index/$REPLAY_COUNT completed" >&2
done

cleanup
trap - EXIT

python3 tools/analyze_phase65_ingress_goal_depth_adjustment_inner_staging.py \
  --artifact-dir "$ARTIFACT_DIR" \
  --output "$SUMMARY_JSON" \
  > "$ARTIFACT_DIR/${RUN_ID}_analyzer_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_analyzer_stderr.log" || true

cat "$SUMMARY_JSON"
