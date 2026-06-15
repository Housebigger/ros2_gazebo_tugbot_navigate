#!/usr/bin/env bash
set -euo pipefail

RUN_ID="phase40_start_pose_entrance_frame_reconciliation"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ARTIFACT_DIR="log/${RUN_ID}"
WORLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
MAZE_CONFIG="$ROOT/src/tugbot_maze/config/maze_20260528_scaled_instance.yaml"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
NAV2_PARAMS="$ROOT/src/tugbot_navigation/config/nav2_slam_params.yaml"
RUN_TIMEOUT_SEC="${PHASE40_RUN_TIMEOUT_SEC:-75}"
RECORDER_DURATION_SEC="${PHASE40_RECORDER_DURATION_SEC:-45}"
USE_RVIZ="${PHASE40_USE_RVIZ:-false}"

LAUNCH_LOG="$ARTIFACT_DIR/${RUN_ID}_launch.log"
RECORDER_STDOUT="$ARTIFACT_DIR/${RUN_ID}_recorder_stdout.json"
RECORDER_STDERR="$ARTIFACT_DIR/${RUN_ID}_recorder_stderr.log"
SUMMARY_JSON="$ARTIFACT_DIR/phase40_start_pose_entrance_frame_reconciliation.json"
PIDS=()
LAUNCH_PID=""
RECORDER_PID=""

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
  pkill -TERM -f 'tools/record_phase40_start_pose_alignment.py' || true
  pkill -TERM -f 'ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py' || true
  pkill -TERM -f 'ros_gz_bridge|parameter_bridge|static_transform_publisher' || true
  pkill -TERM -f 'slam_toolbox|async_slam_toolbox_node|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower|velocity_smoother|lifecycle_manager_navigation' || true
  pkill -TERM -f 'gz sim|ruby .*gz sim|rviz2' || true
  sleep 2
  pkill -KILL -f 'tools/record_phase40_start_pose_alignment.py|ros_gz_bridge|parameter_bridge|static_transform_publisher|slam_toolbox|async_slam_toolbox_node|controller_server|planner_server|bt_navigator|gz sim|ruby .*gz sim|rviz2' || true
  if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
  fi
  mkdir -p "$ARTIFACT_DIR"
  pgrep -af 'ros2 launch|gz sim|rviz2|slam_toolbox|frontier_explorer|controller_server|planner_server|bt_navigator|behavior_server|waypoint_follower|velocity_smoother|smoother_server|ros_gz_bridge|parameter_bridge|static_transform_publisher|record_phase40_start_pose_alignment' \
    > "$ARTIFACT_DIR/${RUN_ID}_cleanup_processes_after.txt" || true
}
trap cleanup EXIT

mkdir -p "$ARTIFACT_DIR"
rm -f \
  "$LAUNCH_LOG" \
  "$RECORDER_STDOUT" \
  "$RECORDER_STDERR" \
  "$SUMMARY_JSON" \
  "$ARTIFACT_DIR/phase40_start_pose_full_data.json" \
  "$ARTIFACT_DIR/start_pose_entrance_alignment_overlay.png" \
  "$ARTIFACT_DIR/${RUN_ID}_preflight.txt" \
  "$ARTIFACT_DIR/${RUN_ID}_launch.pid" \
  "$ARTIFACT_DIR/${RUN_ID}_recorder.pid" \
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
  echo "missing SLAM or baseline Nav2 params" >&2
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
if (( RUN_TIMEOUT_SEC < 45 || RUN_TIMEOUT_SEC > 90 )); then
  echo "PHASE40_RUN_TIMEOUT_SEC must remain bounded in [45,90], got $RUN_TIMEOUT_SEC" >&2
  exit 7
fi
if (( RECORDER_DURATION_SEC < 20 || RECORDER_DURATION_SEC > 60 )); then
  echo "PHASE40_RECORDER_DURATION_SEC must remain bounded in [20,60], got $RECORDER_DURATION_SEC" >&2
  exit 8
fi

{
  echo "# Phase40 Start Pose / Entrance Frame Reconciliation"
  echo "timestamp=$(date -Is)"
  echo "world=$WORLD"
  echo "maze_config=$MAZE_CONFIG"
  echo "slam_params=$SLAM_PARAMS"
  echo "nav2_params=$NAV2_PARAMS"
  echo "run_timeout_sec=$RUN_TIMEOUT_SEC"
  echo "recorder_duration_sec=$RECORDER_DURATION_SEC"
  echo "snapshots=15,30,45"
  echo "active_truth=entrance_x=-11.011281 entrance_y=-9.025070 entrance_yaw=0.0 exit_x=10.061281 exit_y=9.058496 exit_radius=1.2"
  echo "preserved_phase37_conclusion=BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH"
  echo "preserved_phase39_conclusion=FRAME_ALIGNMENT_ISSUE_CONFIRMED"
  echo "guardrails=pose-only bounded startup check; no maze_explorer; no goal dispatch; no autonomous success claim; no Nav2/MPPI/controller parameter edits; no maze_explorer strategy edits; no fallback/terminal acceptance continuation; no old scaffold world/map; no long run."
  echo "nav2_and_maze_strategy_diff_begin"
  git diff -- src/tugbot_navigation/config/nav2*.yaml src/tugbot_maze/tugbot_maze/maze_explorer.py || true
  echo "nav2_and_maze_strategy_diff_end"
} > "$ARTIFACT_DIR/${RUN_ID}_preflight.txt"

set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u
ros2 daemon stop >/dev/null 2>&1 || true

python3 tools/record_phase40_start_pose_alignment.py \
  --output-dir "$ARTIFACT_DIR" \
  --duration "$RECORDER_DURATION_SEC" \
  --snapshots 15,30,45 \
  --metadata "$MAZE_CONFIG" \
  --world "$WORLD" \
  --slam-params "$SLAM_PARAMS" \
  > "$RECORDER_STDOUT" 2> "$RECORDER_STDERR" &
RECORDER_PID="$!"
PIDS+=("$RECORDER_PID")
echo "$RECORDER_PID" > "$ARTIFACT_DIR/${RUN_ID}_recorder.pid"

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

wait "$RECORDER_PID" || true
if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
  kill -TERM "$LAUNCH_PID" 2>/dev/null || true
fi
wait "$LAUNCH_PID" 2>/dev/null || true

if [[ ! -s "$SUMMARY_JSON" ]]; then
  echo "Phase40 summary missing: $SUMMARY_JSON" >&2
  exit 9
fi

python3 - <<PY
import json
from pathlib import Path
summary = json.loads(Path('$SUMMARY_JSON').read_text())
print(json.dumps({
    'run_id': '$RUN_ID',
    'artifact_dir': '$ARTIFACT_DIR',
    'conclusion': summary.get('conclusion'),
    'snapshot_count': summary.get('snapshot_count'),
    'autonomous_success_claimed': summary.get('autonomous_success_claimed'),
    'robot_to_active_entrance_distance_m': (summary.get('frame_alignment') or {}).get('robot_to_active_entrance_distance_m'),
}, indent=2, sort_keys=True))
PY
