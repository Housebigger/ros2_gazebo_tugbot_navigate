#!/usr/bin/env bash
set -euo pipefail

RUN_ID="phase52_startup_map_boundary_warmup"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ARTIFACT_DIR="log/${RUN_ID}"
WORLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
MAZE_CONFIG="$ROOT/src/tugbot_maze/config/maze_20260528_scaled_instance.yaml"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
NAV2_PARAMS="$ROOT/src/tugbot_navigation/config/nav2_slam_params.yaml"
RUN_TIMEOUT_SEC="${PHASE52_RUN_TIMEOUT_SEC:-360}"
STATE_RECORDER_TIMEOUT_SEC="${PHASE52_STATE_RECORDER_TIMEOUT_SEC:-390}"
RUNTIME_RECORDER_TIMEOUT_SEC="${PHASE52_RUNTIME_RECORDER_TIMEOUT_SEC:-330}"
SNAPSHOT_DELAY_SEC="${PHASE52_SNAPSHOT_DELAY_SEC:-75}"
USE_RVIZ="${PHASE52_USE_RVIZ:-false}"
MAX_GOALS="0"

if (( RUN_TIMEOUT_SEC < 180 || RUN_TIMEOUT_SEC > 420 )); then
  echo "Phase52 RUN_TIMEOUT_SEC must be bounded in [180, 420], got $RUN_TIMEOUT_SEC" >&2
  exit 2
fi
if (( RUNTIME_RECORDER_TIMEOUT_SEC < 150 || RUNTIME_RECORDER_TIMEOUT_SEC > 390 )); then
  echo "Phase52 RUNTIME_RECORDER_TIMEOUT_SEC must be bounded in [150, 390], got $RUNTIME_RECORDER_TIMEOUT_SEC" >&2
  exit 2
fi

LAUNCH_LOG="$ARTIFACT_DIR/${RUN_ID}_launch.log"
GOAL_EVENTS="$ARTIFACT_DIR/${RUN_ID}_goal_events.jsonl"
EXPLORER_STATE="$ARTIFACT_DIR/${RUN_ID}_explorer_state.jsonl"
RUNTIME_EVIDENCE="$ARTIFACT_DIR/${RUN_ID}_runtime_evidence.json"
SUMMARY_JSON="$ARTIFACT_DIR/${RUN_ID}.json"
ACTION_INFO="$ARTIFACT_DIR/${RUN_ID}_navigate_to_pose_action_info.txt"
GOAL_POSE_INFO="$ARTIFACT_DIR/${RUN_ID}_goal_pose_topic_info.txt"
LIFECYCLE_STATES="$ARTIFACT_DIR/${RUN_ID}_lifecycle_states.txt"
NAV2_CONFIG_DIFF="$ARTIFACT_DIR/${RUN_ID}_nav2_config_diff.txt"
CLEANUP_AFTER="$ARTIFACT_DIR/${RUN_ID}_cleanup_processes_after.txt"
PIDS=()
LAUNCH_PID=""

cleanup() {
  set +e
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  sleep 2
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
  pkill -TERM -f 'ros2 launch tugbot_bringup tugbot_maze_explore.launch.py' || true
  pkill -TERM -f 'tools/record_explorer_state_series.py.*phase52_startup_map_boundary_warmup' || true
  pkill -TERM -f 'tools/analyze_phase52_startup_map_boundary_warmup.py --record-runtime' || true
  pkill -TERM -f 'ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|maze_explorer|frontier_explorer' || true
  pkill -TERM -f 'slam_toolbox|async_slam_toolbox_node|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower|velocity_smoother|lifecycle_manager_navigation|route_server|collision_monitor|docking_server' || true
  pkill -TERM -f 'gz sim|ruby .*gz sim' || true
  sleep 2
  pkill -KILL -f 'ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|maze_explorer|frontier_explorer|slam_toolbox|async_slam_toolbox_node|controller_server|planner_server|bt_navigator|gz sim|ruby .*gz sim' || true
  if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
  fi
  mkdir -p "$ARTIFACT_DIR"
  pgrep -af 'ros2 launch|gz sim|rviz2|slam_toolbox|maze_explorer|frontier_explorer|controller_server|planner_server|bt_navigator|behavior_server|waypoint_follower|velocity_smoother|smoother_server|route_server|collision_monitor|docking_server|ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|analyze_phase52_startup_map_boundary_warmup.py|record_explorer_state_series.py' > "$CLEANUP_AFTER" || true
}
trap cleanup EXIT

mkdir -p "$ARTIFACT_DIR"
rm -f \
  "$LAUNCH_LOG" "$GOAL_EVENTS" "$EXPLORER_STATE" "$RUNTIME_EVIDENCE" "$SUMMARY_JSON" \
  "$ACTION_INFO" "$GOAL_POSE_INFO" "$LIFECYCLE_STATES" "$NAV2_CONFIG_DIFF" "$CLEANUP_AFTER" \
  "$ARTIFACT_DIR/${RUN_ID}_preflight.txt" "$ARTIFACT_DIR/${RUN_ID}_ros_graph_snapshot.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_gz_graph_snapshot.txt" "$ARTIFACT_DIR/${RUN_ID}_ros_graph_final.txt"

if [[ ! -f "$WORLD" ]]; then echo "missing active scaled2x world: $WORLD" >&2; exit 3; fi
if [[ ! -f "$MAZE_CONFIG" ]]; then echo "missing active maze metadata: $MAZE_CONFIG" >&2; exit 4; fi
if [[ ! -f "$SLAM_PARAMS" || ! -f "$NAV2_PARAMS" ]]; then echo "missing SLAM or Nav2 params" >&2; exit 5; fi
if [[ "$WORLD" != *"tugbot_maze_world_20260528_clean_scaled2x.sdf" ]]; then echo "refusing non-active world: $WORLD" >&2; exit 6; fi
if [[ "$MAZE_CONFIG" != *"maze_20260528_scaled_instance.yaml" ]]; then echo "refusing non-active maze metadata: $MAZE_CONFIG" >&2; exit 7; fi

{
  echo "# Phase52 Startup Map Boundary Warmup Bounded Runtime Validation"
  echo "timestamp=$(date -Is)"
  echo "world=$WORLD"
  echo "maze_config=$MAZE_CONFIG"
  echo "slam_params=$SLAM_PARAMS"
  echo "nav2_params=$NAV2_PARAMS"
  echo "run_timeout_sec=$RUN_TIMEOUT_SEC"
  echo "runtime_recorder_timeout_sec=$RUNTIME_RECORDER_TIMEOUT_SEC"
  echo "max_goals=$MAX_GOALS"
  echo "startup_warmup_no_dispatch=true"
  echo "active_truth_frame=map active_truth=entrance_x=0.0 entrance_y=0.0 entrance_yaw=0.0 exit_x=21.072562 exit_y=18.083566 exit_radius=1.2"
  echo "guardrails=bounded startup warmup only; no clearance_radius_m tuning; no map threshold tuning; no Nav2/MPPI/controller parameter edits; no maze_explorer exploration strategy edits; no fallback/terminal acceptance; no autonomous success claim; dispatch forbidden."
  echo "nav2_config_diff_begin"
  git diff -- src/tugbot_navigation/config | tee "$NAV2_CONFIG_DIFF" || true
  echo "nav2_config_diff_end"
} > "$ARTIFACT_DIR/${RUN_ID}_preflight.txt"

set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u
ros2 daemon stop >/dev/null 2>&1 || true

python3 tools/record_explorer_state_series.py --topic /maze/goal_events \
  --output "$GOAL_EVENTS" \
  --max-samples 20 \
  --min-samples 0 \
  --timeout-sec "$STATE_RECORDER_TIMEOUT_SEC" \
  > "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stderr.log" &
PIDS+=("$!")
GOAL_EVENTS_PID="$!"

python3 tools/record_explorer_state_series.py --topic /maze/explorer_state \
  --output "$EXPLORER_STATE" \
  --max-samples 420 \
  --min-samples 3 \
  --timeout-sec "$STATE_RECORDER_TIMEOUT_SEC" \
  > "$ARTIFACT_DIR/${RUN_ID}_explorer_state_recorder_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_explorer_state_recorder_stderr.log" &
PIDS+=("$!")
STATE_PID="$!"

python3 tools/analyze_phase52_startup_map_boundary_warmup.py --record-runtime "$RUNTIME_RECORDER_TIMEOUT_SEC" \
  --snapshot-period-sec 10 \
  --output "$RUNTIME_EVIDENCE" \
  > "$ARTIFACT_DIR/${RUN_ID}_runtime_recorder_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_runtime_recorder_stderr.log" &
PIDS+=("$!")
RUNTIME_RECORDER_PID="$!"

set -o pipefail
(
  timeout --preserve-status "$RUN_TIMEOUT_SEC" ros2 launch tugbot_bringup tugbot_maze_explore.launch.py \
    world_sdf:="$WORLD" \
    maze_config:="$MAZE_CONFIG" \
    slam_params_file:="$SLAM_PARAMS" \
    params_file:="$NAV2_PARAMS" \
    headless:=true \
    use_rviz:="${USE_RVIZ}" \
    use_sim_time:=true \
    autostart:=true \
    explorer_type:=maze_dfs \
    max_goals:="0" \
    startup_warmup_no_dispatch:=true \
    goal_timeout_sec:=45.0 \
    near_exit_goal_timeout_sec:=55.0 \
    near_exit_timeout_extension_radius_m:=1.0 \
    near_exit_fallback_enabled:=false \
    clearance_radius_m:=0.38 \
    dispatch_readiness_near_robot_radius_m:=1.0 \
    dispatch_readiness_min_map_known_ratio:=0.70 \
    dispatch_readiness_min_map_free_ratio:=0.50 \
    dispatch_readiness_min_local_costmap_known_ratio:=0.95 \
    dispatch_readiness_min_local_costmap_free_ratio:=0.50 \
    dispatch_readiness_min_scan_finite_count:=120 \
    entrance_x:="0.0" \
    entrance_y:="0.0" \
    entrance_yaw:="0.0" \
    exit_x:="21.072562" \
    exit_y:="18.083566" \
    exit_radius:="1.2" \
    2>&1 | tee "$LAUNCH_LOG"
) &
PIDS+=("$!")
LAUNCH_PID="$!"
echo "$LAUNCH_PID" > "$ARTIFACT_DIR/${RUN_ID}_launch.pid"

sleep "$SNAPSHOT_DELAY_SEC"
{
  echo "=== timestamp ==="
  date -Is
  echo "=== ROS nodes ==="
  ros2 node list || true
  echo "=== ROS topics ==="
  ros2 topic list || true
  echo "=== ROS actions ==="
  ros2 action list || true
  echo "=== expected explorer/nav2 nodes grep ==="
  ros2 node list | grep -E 'maze_explorer|maze_goal_monitor|controller_server|planner_server|bt_navigator|slam_toolbox|ros_gz_bridge' || true
} > "$ARTIFACT_DIR/${RUN_ID}_ros_graph_snapshot.txt" 2>&1

{
  echo "=== gz topics ==="
  gz topic -l || true
  echo "=== gz services ==="
  gz service -l || true
} > "$ARTIFACT_DIR/${RUN_ID}_gz_graph_snapshot.txt" 2>&1

(timeout 10 ros2 action info /navigate_to_pose || true) > "$ACTION_INFO" 2>&1
(timeout 10 ros2 topic info /goal_pose || true) > "$GOAL_POSE_INFO" 2>&1
{
  echo "/controller_server"
  timeout 10 ros2 lifecycle get /controller_server || true
  echo "/planner_server"
  timeout 10 ros2 lifecycle get /planner_server || true
  echo "/bt_navigator"
  timeout 10 ros2 lifecycle get /bt_navigator || true
} > "$LIFECYCLE_STATES" 2>&1

wait "$LAUNCH_PID" || true
wait "$RUNTIME_RECORDER_PID" 2>/dev/null || true
sleep 3
for pid in "$GOAL_EVENTS_PID" "$STATE_PID"; do
  if kill -0 "$pid" 2>/dev/null; then
    kill -TERM "$pid" 2>/dev/null || true
  fi
done
wait "$GOAL_EVENTS_PID" 2>/dev/null || true
wait "$STATE_PID" 2>/dev/null || true

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

python3 tools/analyze_phase52_startup_map_boundary_warmup.py --analyze \
  --artifact-dir "$ARTIFACT_DIR" \
  --output "$SUMMARY_JSON" \
  --explorer-state "$EXPLORER_STATE" \
  --goal-events "$GOAL_EVENTS" \
  --runtime-evidence "$RUNTIME_EVIDENCE" \
  --action-info "$ACTION_INFO" \
  --goal-pose-info "$GOAL_POSE_INFO" \
  --lifecycle-states "$LIFECYCLE_STATES" \
  --cleanup-processes-after "$CLEANUP_AFTER" \
  > "$ARTIFACT_DIR/${RUN_ID}_analyzer_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_analyzer_stderr.log" || true

cat "$SUMMARY_JSON"
