#!/usr/bin/env bash
set -euo pipefail

RUN_ID="phase43_initial_topology_sampling_diagnostics"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ARTIFACT_DIR="log/${RUN_ID}"
WORLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
MAZE_CONFIG="$ROOT/src/tugbot_maze/config/maze_20260528_scaled_instance.yaml"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
NAV2_PARAMS="$ROOT/src/tugbot_navigation/config/nav2_slam_params.yaml"
RUN_TIMEOUT_SEC="${PHASE43_RUN_TIMEOUT_SEC:-240}"
MAX_GOALS="${PHASE43_MAX_GOALS:-4}"
GOAL_TIMEOUT_SEC="${PHASE43_GOAL_TIMEOUT_SEC:-45.0}"
SNAPSHOT_DELAY_SEC="${PHASE43_SNAPSHOT_DELAY_SEC:-45}"
STATE_RECORDER_TIMEOUT_SEC="${PHASE43_STATE_RECORDER_TIMEOUT_SEC:-260}"
RUNTIME_RECORDER_TIMEOUT_SEC="${PHASE43_RUNTIME_RECORDER_TIMEOUT_SEC:-120}"
USE_RVIZ="${PHASE43_USE_RVIZ:-false}"

if (( RUN_TIMEOUT_SEC < 120 || RUN_TIMEOUT_SEC > 300 )); then
  echo "Phase43 RUN_TIMEOUT_SEC must be bounded in [120, 300], got $RUN_TIMEOUT_SEC" >&2
  exit 2
fi

LAUNCH_LOG="$ARTIFACT_DIR/${RUN_ID}_launch.log"
GOAL_EVENTS="$ARTIFACT_DIR/${RUN_ID}_goal_events.jsonl"
EXPLORER_STATE="$ARTIFACT_DIR/${RUN_ID}_explorer_state.jsonl"
FULL_DATA_JSON="$ARTIFACT_DIR/phase43_initial_topology_full_data.json"
SUMMARY_JSON="$ARTIFACT_DIR/phase43_initial_topology_sampling_diagnostics.json"
MAP_SCAN_ODOM_TF_COSTMAP_SAMPLES="$ARTIFACT_DIR/map_scan_odom_tf_costmap_samples.json"
FIRST_TOPOLOGY_SNAPSHOT="$ARTIFACT_DIR/first_topology_sampling_snapshot.json"
ACTION_INFO="$ARTIFACT_DIR/${RUN_ID}_navigate_to_pose_action_info.txt"
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
  pkill -TERM -f 'tools/record_explorer_state_series.py.*phase43_initial_topology_sampling_diagnostics' || true
  pkill -TERM -f 'tools/analyze_phase43_initial_topology_sampling_diagnostics.py --record-runtime' || true
  pkill -TERM -f 'ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|maze_explorer|frontier_explorer' || true
  pkill -TERM -f 'slam_toolbox|async_slam_toolbox_node|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower|velocity_smoother|lifecycle_manager_navigation' || true
  pkill -TERM -f 'gz sim|ruby .*gz sim' || true
  sleep 2
  pkill -KILL -f 'ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|maze_explorer|frontier_explorer|slam_toolbox|async_slam_toolbox_node|controller_server|planner_server|bt_navigator|gz sim|ruby .*gz sim' || true
  if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
  fi
  mkdir -p "$ARTIFACT_DIR"
  pgrep -af 'ros2 launch|gz sim|rviz2|slam_toolbox|maze_explorer|frontier_explorer|controller_server|planner_server|bt_navigator|behavior_server|waypoint_follower|velocity_smoother|smoother_server|ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|analyze_phase43_initial_topology_sampling_diagnostics.py|record_explorer_state_series.py' > "$CLEANUP_AFTER" || true
}
trap cleanup EXIT

mkdir -p "$ARTIFACT_DIR"
rm -f \
  "$LAUNCH_LOG" \
  "$GOAL_EVENTS" \
  "$EXPLORER_STATE" \
  "$FULL_DATA_JSON" \
  "$SUMMARY_JSON" \
  "$MAP_SCAN_ODOM_TF_COSTMAP_SAMPLES" \
  "$FIRST_TOPOLOGY_SNAPSHOT" \
  "$ACTION_INFO" \
  "$NAV2_CONFIG_DIFF" \
  "$CLEANUP_AFTER" \
  "$ARTIFACT_DIR/${RUN_ID}_preflight.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_ros_graph_initial.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_ros_graph_final.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_gz_graph_initial.txt"

if [[ ! -f "$WORLD" ]]; then echo "missing active scaled2x world: $WORLD" >&2; exit 3; fi
if [[ ! -f "$MAZE_CONFIG" ]]; then echo "missing active maze metadata: $MAZE_CONFIG" >&2; exit 4; fi
if [[ "$WORLD" != *"tugbot_maze_world_20260528_clean_scaled2x.sdf" ]]; then echo "refusing non-active world: $WORLD" >&2; exit 5; fi
if [[ "$MAZE_CONFIG" != *"maze_20260528_scaled_instance.yaml" ]]; then echo "refusing non-active maze metadata: $MAZE_CONFIG" >&2; exit 6; fi

{
  echo "# Phase43 Initial Topology Sampling Diagnostics"
  echo "timestamp=$(date -Is)"
  echo "world=$WORLD"
  echo "maze_config=$MAZE_CONFIG"
  echo "slam_params=$SLAM_PARAMS"
  echo "nav2_params=$NAV2_PARAMS"
  echo "run_timeout_sec=$RUN_TIMEOUT_SEC"
  echo "max_goals=$MAX_GOALS"
  echo "goal_timeout_sec=$GOAL_TIMEOUT_SEC"
  echo "active_truth_frame=map active_truth=entrance_x=0.0 entrance_y=0.0 entrance_yaw=0.0 exit_x=21.072562 exit_y=18.083566 exit_radius=1.2"
  echo "guardrails=diagnostics only; no Nav2/MPPI/controller parameter edits; no maze_explorer strategy edits; no fallback/terminal acceptance continuation; no long run; no autonomous success claim."
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
  --max-samples 80 \
  --min-samples 1 \
  --timeout-sec "$STATE_RECORDER_TIMEOUT_SEC" \
  > "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stderr.log" &
PIDS+=("$!")
GOAL_EVENTS_PID="$!"

python3 tools/record_explorer_state_series.py --topic /maze/explorer_state \
  --output "$EXPLORER_STATE" \
  --max-samples 260 \
  --min-samples 3 \
  --timeout-sec "$STATE_RECORDER_TIMEOUT_SEC" \
  --stop-on-terminal \
  --terminal-linger-sec 5.0 \
  > "$ARTIFACT_DIR/${RUN_ID}_explorer_state_recorder_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_explorer_state_recorder_stderr.log" &
PIDS+=("$!")
STATE_PID="$!"

python3 tools/analyze_phase43_initial_topology_sampling_diagnostics.py --record-runtime "$RUNTIME_RECORDER_TIMEOUT_SEC" \
  --snapshot-period-sec 5 \
  --output "$MAP_SCAN_ODOM_TF_COSTMAP_SAMPLES" \
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
    max_goals:="$MAX_GOALS" \
    goal_timeout_sec:="$GOAL_TIMEOUT_SEC" \
    near_exit_goal_timeout_sec:=55.0 \
    near_exit_timeout_extension_radius_m:=1.0 \
    near_exit_fallback_enabled:=false \
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
} > "$ARTIFACT_DIR/${RUN_ID}_ros_graph_initial.txt" 2>&1

{
  echo "=== gz topics ==="
  gz topic -l || true
  echo "=== gz services ==="
  gz service -l || true
} > "$ARTIFACT_DIR/${RUN_ID}_gz_graph_initial.txt" 2>&1

(timeout 10 ros2 action info /navigate_to_pose || true) > "$ACTION_INFO" 2>&1

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

# Alias for contract/readability: runtime recorder stores full map/scan/odom/TF/costmap evidence.
cp "$MAP_SCAN_ODOM_TF_COSTMAP_SAMPLES" "$FULL_DATA_JSON" 2>/dev/null || true

# Produce final analysis only after bounded runtime cleanup, so cleanup_empty is
# based on the actual post-run process state rather than the pre-trap placeholder.
cleanup
trap - EXIT

python3 tools/analyze_phase43_initial_topology_sampling_diagnostics.py --analyze \
  --artifact-dir "$ARTIFACT_DIR" \
  --output "$SUMMARY_JSON" \
  --explorer-state "$EXPLORER_STATE" \
  --goal-events "$GOAL_EVENTS" \
  --runtime-evidence "$MAP_SCAN_ODOM_TF_COSTMAP_SAMPLES" \
  --action-info "$ACTION_INFO" \
  --cleanup-processes-after "$CLEANUP_AFTER" \
  > "$ARTIFACT_DIR/${RUN_ID}_analyzer_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_analyzer_stderr.log" || true

cat "$SUMMARY_JSON"
