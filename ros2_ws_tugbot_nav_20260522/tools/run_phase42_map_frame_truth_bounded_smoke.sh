#!/usr/bin/env bash
set -euo pipefail

RUN_ID="phase42_map_frame_truth_bounded_smoke"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ARTIFACT_DIR="log/${RUN_ID}"
WORLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
MAZE_CONFIG="$ROOT/src/tugbot_maze/config/maze_20260528_scaled_instance.yaml"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
NAV2_PARAMS="$ROOT/src/tugbot_navigation/config/nav2_slam_params.yaml"
RUN_TIMEOUT_SEC="${PHASE42_RUN_TIMEOUT_SEC:-900}"
MAX_GOALS="${PHASE42_MAX_GOALS:-4}"
GOAL_TIMEOUT_SEC="${PHASE42_GOAL_TIMEOUT_SEC:-45.0}"
SNAPSHOT_DELAY_SEC="${PHASE42_SNAPSHOT_DELAY_SEC:-45}"
STATE_RECORDER_TIMEOUT_SEC="${PHASE42_STATE_RECORDER_TIMEOUT_SEC:-930}"
USE_RVIZ="${PHASE42_USE_RVIZ:-false}"

LAUNCH_LOG="$ARTIFACT_DIR/${RUN_ID}_launch.log"
GOAL_EVENTS="$ARTIFACT_DIR/${RUN_ID}_goal_events.jsonl"
EXPLORER_STATE="$ARTIFACT_DIR/${RUN_ID}_explorer_state.jsonl"
SUMMARY_JSON="$ARTIFACT_DIR/${RUN_ID}_summary.json"

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
  pkill -TERM -f 'tools/record_explorer_state_series.py.*phase42_map_frame_truth_bounded_smoke' || true
  pkill -TERM -f 'ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|maze_explorer|frontier_explorer' || true
  pkill -TERM -f 'slam_toolbox|async_slam_toolbox_node|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower|velocity_smoother|lifecycle_manager_navigation' || true
  pkill -TERM -f 'gz sim|ruby .*gz sim' || true
  sleep 2
  pkill -KILL -f 'ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|maze_explorer|frontier_explorer|slam_toolbox|async_slam_toolbox_node|controller_server|planner_server|bt_navigator|gz sim|ruby .*gz sim' || true

  if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
  fi

  mkdir -p "$ARTIFACT_DIR"
  pgrep -af 'ros2 launch|gz sim|rviz2|slam_toolbox|maze_explorer|frontier_explorer|controller_server|planner_server|bt_navigator|behavior_server|waypoint_follower|velocity_smoother|smoother_server|ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor' \
    > "$ARTIFACT_DIR/${RUN_ID}_cleanup_processes_after.txt" || true
}
trap cleanup EXIT

mkdir -p "$ARTIFACT_DIR"
rm -f \
  "$LAUNCH_LOG" \
  "$GOAL_EVENTS" \
  "$EXPLORER_STATE" \
  "$SUMMARY_JSON" \
  "$ARTIFACT_DIR/${RUN_ID}_ros_graph_initial.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_ros_graph_final.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_gz_graph_initial.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_map_once.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_scan_once.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_odom_once.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_tf_once.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_local_costmap_once.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_global_costmap_once.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_navigate_to_pose_action_info.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_preflight.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_cleanup_processes_after.txt"

if [[ ! -f "$WORLD" ]]; then
  echo "missing active scaled clean world: $WORLD" >&2
  exit 2
fi
if [[ ! -f "$MAZE_CONFIG" ]]; then
  echo "missing active maze metadata: $MAZE_CONFIG" >&2
  exit 3
fi
if [[ ! -f "$SLAM_PARAMS" || ! -f "$NAV2_PARAMS" ]]; then
  echo "missing SLAM or Nav2 params" >&2
  exit 4
fi
if [[ "$WORLD" != *"tugbot_maze_world_20260528_clean_scaled2x.sdf" ]]; then
  echo "refusing non-active world: $WORLD" >&2
  exit 5
fi
if [[ "$MAZE_CONFIG" != *"maze_20260528_scaled_instance.yaml" ]]; then
  echo "refusing non-active maze metadata: $MAZE_CONFIG" >&2
  exit 6
fi

{
  echo "# Phase42 Map-frame Truth Bounded Autonomous Smoke"
  echo "timestamp=$(date -Is)"
  echo "world=$WORLD"
  echo "maze_config=$MAZE_CONFIG"
  echo "slam_params=$SLAM_PARAMS"
  echo "nav2_params=$NAV2_PARAMS"
  echo "run_timeout_sec=$RUN_TIMEOUT_SEC"
  echo "max_goals=$MAX_GOALS"
  echo "goal_timeout_sec=$GOAL_TIMEOUT_SEC"
  echo "active_truth_frame=map active_truth=entrance_x=0.0 entrance_y=0.0 entrance_yaw=0.0 exit_x=21.072562 exit_y=18.083566 exit_radius=1.2"
  echo "guardrails=no Nav2/MPPI/controller parameter edits; no strategy edits; no fallback/terminal acceptance continuation; bounded smoke only; no complete autonomous success claim unless evidence reaches exit."
  echo "nav2_config_diff_begin"
  git diff -- src/tugbot_navigation/config || true
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
  --max-samples 240 \
  --min-samples 3 \
  --timeout-sec "$STATE_RECORDER_TIMEOUT_SEC" \
  --stop-on-terminal \
  --terminal-linger-sec 5.0 \
  > "$ARTIFACT_DIR/${RUN_ID}_explorer_state_recorder_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_explorer_state_recorder_stderr.log" &
PIDS+=("$!")
STATE_PID="$!"

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

(timeout 10 ros2 action info /navigate_to_pose || true) > "$ARTIFACT_DIR/${RUN_ID}_navigate_to_pose_action_info.txt" 2>&1
(timeout 10 ros2 topic echo --once /map || true) > "$ARTIFACT_DIR/${RUN_ID}_map_once.txt" 2>&1
(timeout 10 ros2 topic echo --once /scan || true) > "$ARTIFACT_DIR/${RUN_ID}_scan_once.txt" 2>&1
(timeout 10 ros2 topic echo --once /odom || true) > "$ARTIFACT_DIR/${RUN_ID}_odom_once.txt" 2>&1
(timeout 10 ros2 topic echo --once /tf || true) > "$ARTIFACT_DIR/${RUN_ID}_tf_once.txt" 2>&1
(timeout 10 ros2 topic echo --once /local_costmap/costmap || true) > "$ARTIFACT_DIR/${RUN_ID}_local_costmap_once.txt" 2>&1
(timeout 10 ros2 topic echo --once /global_costmap/costmap || true) > "$ARTIFACT_DIR/${RUN_ID}_global_costmap_once.txt" 2>&1

wait "$LAUNCH_PID" || true
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

python3 - <<PY
import json
from pathlib import Path
run_id = '$RUN_ID'
artifact_dir = Path('$ARTIFACT_DIR')
goal_events = Path('$GOAL_EVENTS')
explorer_state = Path('$EXPLORER_STATE')
launch_log = Path('$LAUNCH_LOG')
summary = {
    'run_id': run_id,
    'artifact_dir': str(artifact_dir),
    'active_world': '$WORLD',
    'active_metadata': '$MAZE_CONFIG',
    'bounded': {'timeout_sec': float('$RUN_TIMEOUT_SEC'), 'max_goals': int('$MAX_GOALS')},
    'active_truth': {
        'entrance_x': 0.0,
        'entrance_y': 0.0,
        'entrance_yaw': 0.0,
        'exit_x': 21.072562,
        'exit_y': 18.083566,
        'exit_radius': 1.2,
    },
    'goal_events_samples': 0,
    'explorer_state_samples': 0,
    'dispatch_events': 0,
    'outcome_events': 0,
    'exit_reached_events': 0,
    'final_state': {},
    'legacy_truth_seen_in_launch_log': False,
    'complete_autonomous_success_claimed': False,
    'active_truth_frame': 'map',
    'phase41_conclusion_preserved': 'MAP_FRAME_TRUTH_ALIGNED_READY_FOR_PHASE37_RERUN',
}
for path, key in [(goal_events, 'goal_events_samples'), (explorer_state, 'explorer_state_samples')]:
    if path.exists() and path.stat().st_size:
        rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        summary[key] = len(rows)
        if path == goal_events:
            payloads = [row.get('state', row) for row in rows]
            summary['dispatch_events'] = sum(1 for p in payloads if p.get('event') == 'dispatch')
            summary['outcome_events'] = sum(1 for p in payloads if p.get('event') == 'outcome')
            summary['exit_reached_events'] = sum(1 for p in payloads if p.get('event') == 'exit_reached')
            summary['last_goal_event'] = payloads[-1] if payloads else {}
        else:
            summary['final_state'] = rows[-1].get('state', rows[-1]) if rows else {}
log_text = launch_log.read_text(errors='replace') if launch_log.exists() else ''
summary['legacy_truth_seen_in_launch_log'] = any(token in log_text for token in ['maze_instance.yaml', 'entrance_x:=-4.0', 'entrance_y:=-3.0', 'exit_x:=4.0', 'exit_y:=3.0', 'exit_radius:=0.6'])
summary['maze_explorer_started_evidence'] = 'maze_explorer' in log_text or summary['explorer_state_samples'] > 0
summary['nav2_action_info_file'] = str(artifact_dir / f'{run_id}_navigate_to_pose_action_info.txt')
Path('$SUMMARY_JSON').write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n')
print(json.dumps(summary, indent=2, sort_keys=True))
PY

python3 tools/analyze_phase42_map_frame_truth_bounded_smoke.py --artifact-dir "$ARTIFACT_DIR" --run-id "$RUN_ID" --output "$ARTIFACT_DIR/${RUN_ID}_acceptance_analysis.json" > "$ARTIFACT_DIR/${RUN_ID}_acceptance_analysis_stdout.json"

cleanup
trap - EXIT

echo "Phase42 map-frame truth bounded smoke complete. Artifacts: $ARTIFACT_DIR"
