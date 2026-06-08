#!/usr/bin/env bash
set -euo pipefail

RUN_ID="${1:-phase27_alt_r1_smoke}"
if [[ ! "$RUN_ID" =~ ^(phase27_alt_r1_smoke[0-9]*|phase27_alt_r4_enabled_run[0-9]+)$ ]]; then
  echo "usage: $0 phase27_alt_r1_smoke[N] | phase27_alt_r4_enabled_run[N]" >&2
  exit 2
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

LOG_DIR="log"
mkdir -p "$LOG_DIR"

LAUNCH_LOG="$LOG_DIR/${RUN_ID}_launch.log"
GOAL_EVENTS="$LOG_DIR/${RUN_ID}_goal_events.jsonl"
EXPLORER_STATE="$LOG_DIR/${RUN_ID}_explorer_state.jsonl"
ANALYSIS_JSON="$LOG_DIR/${RUN_ID}_phase27_alt_fallback_analysis.json"
MAX_GOALS="${PHASE27_ALT_R1_MAX_GOALS:-12}"
RUN_TIMEOUT_SEC="${PHASE27_ALT_R1_TIMEOUT_SEC:-420}"
GOAL_EVENTS_MAX_SAMPLES="${PHASE27_ALT_R1_GOAL_EVENTS_MAX_SAMPLES:-220}"
STATE_MAX_SAMPLES="${PHASE27_ALT_R1_STATE_MAX_SAMPLES:-460}"

PIDS=()

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
  pkill -TERM -f "tools/record_explorer_state_series.py.*${RUN_ID}" || true
  pkill -TERM -f 'ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|maze_explorer' || true
  pkill -TERM -f 'slam_toolbox|async_slam_toolbox_node|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|route_server|waypoint_follower|velocity_smoother|collision_monitor|opennav_docking|lifecycle_manager_navigation' || true
  pkill -TERM -f 'gz sim|ruby .*gz sim' || true
  sleep 2
  pkill -KILL -f 'ros_gz_bridge|parameter_bridge|static_transform_publisher|maze_goal_monitor|maze_explorer|slam_toolbox|async_slam_toolbox_node|controller_server|planner_server|bt_navigator|gz sim|ruby .*gz sim' || true
  if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

set +u
. /opt/ros/jazzy/setup.sh
. install/setup.sh
set -u

rm -f "$LAUNCH_LOG" "$GOAL_EVENTS" "$EXPLORER_STATE" "$ANALYSIS_JSON"
ros2 daemon stop >/dev/null 2>&1 || true

python3 tools/record_explorer_state_series.py --topic /maze/goal_events \
  --output "$GOAL_EVENTS" \
  --max-samples "$GOAL_EVENTS_MAX_SAMPLES" \
  --min-samples 1 \
  --timeout-sec "$RUN_TIMEOUT_SEC" &
PIDS+=("$!")
GOAL_EVENTS_PID="$!"

python3 tools/record_explorer_state_series.py --output "$EXPLORER_STATE" \
  --max-samples "$STATE_MAX_SAMPLES" \
  --min-samples 12 \
  --timeout-sec "$RUN_TIMEOUT_SEC" \
  --stop-on-terminal \
  --terminal-linger-sec 6.0 &
PIDS+=("$!")
STATE_PID="$!"

set -o pipefail
ros2 launch tugbot_bringup tugbot_maze_explore.launch.py \
  headless:=true \
  use_rviz:=false \
  explorer_type:=maze_dfs \
  max_goals:="$MAX_GOALS" \
  goal_timeout_sec:=35.0 \
  near_exit_goal_timeout_sec:=55.0 \
  near_exit_timeout_extension_radius_m:=1.0 \
  goal_settle_sec:=1.5 \
  branch_goal_step_m:=0.9 \
  open_direction_lookahead_m:=1.8 \
  lateral_centering_search_m:=1.0 \
  clearance_radius_m:=0.34 \
  min_open_distance_m:=0.5 \
  near_exit_fallback_enabled:=true \
  2>&1 | tee "$LAUNCH_LOG" &
PIDS+=("$!")
LAUNCH_PID="$!"

wait "$STATE_PID" || true

for pid in "$LAUNCH_PID" "$GOAL_EVENTS_PID"; do
  if kill -0 "$pid" 2>/dev/null; then
    kill -TERM "$pid" 2>/dev/null || true
  fi
done
sleep 2

python3 tools/analyze_phase27_alt_fallback_runtime.py \
  --goal-events "$GOAL_EVENTS" \
  --explorer-state "$EXPLORER_STATE" \
  --output-json "$ANALYSIS_JSON"

python3 - <<PY
import json
from pathlib import Path
analysis = json.loads(Path('$ANALYSIS_JSON').read_text()) if Path('$ANALYSIS_JSON').exists() else {}
print(json.dumps({
    'run_id': '$RUN_ID',
    'goal_events': '$GOAL_EVENTS',
    'explorer_state': '$EXPLORER_STATE',
    'analysis_json': '$ANALYSIS_JSON',
    'final_state': analysis.get('final_state'),
    'fallback': analysis.get('fallback'),
    'topology_non_pollution': analysis.get('topology_non_pollution'),
    'conclusion': analysis.get('conclusion'),
}, indent=2, sort_keys=True))
PY

echo "[phase27-alt-r1] complete $RUN_ID"
