#!/usr/bin/env bash
set -eo pipefail

RUN_ID="phase27_alt_r6_manual_observation"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
mkdir -p log

LAUNCH_LOG="log/${RUN_ID}_launch.log"
STATE_LOG="log/${RUN_ID}_explorer_state.jsonl"
EVENT_LOG="log/${RUN_ID}_goal_events.jsonl"
CMD_LOG="log/${RUN_ID}_cmd_vel_nav_echo.log"
TOPIC_SNAPSHOT="log/${RUN_ID}_topics.txt"

: > "$LAUNCH_LOG"
: > "$CMD_LOG"

set +u
source /opt/ros/jazzy/setup.sh
source install/setup.bash
set -u

cleanup() {
  set +e
  for pid in ${STATE_REC_PID:-} ${EVENT_REC_PID:-} ${CMD_ECHO_PID:-}; do
    if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  wait ${STATE_REC_PID:-} ${EVENT_REC_PID:-} ${CMD_ECHO_PID:-} 2>/dev/null || true
}
trap cleanup EXIT

python3 tools/record_explorer_state_series.py \
  --topic /maze/explorer_state \
  --output "$STATE_LOG" \
  --max-samples 200000 \
  --timeout-sec 86400 \
  --min-samples 1 > "log/${RUN_ID}_explorer_state_recorder.log" 2>&1 &
STATE_REC_PID=$!

python3 tools/record_explorer_state_series.py \
  --topic /maze/goal_events \
  --output "$EVENT_LOG" \
  --max-samples 200000 \
  --timeout-sec 86400 \
  --min-samples 1 > "log/${RUN_ID}_goal_events_recorder.log" 2>&1 &
EVENT_REC_PID=$!

# Lightweight readable cmd_vel trace for manual correlation. It is auxiliary;
# the requested durable logs are goal_events, explorer_state, and launch log.
ros2 topic echo /cmd_vel_nav > "$CMD_LOG" 2>&1 &
CMD_ECHO_PID=$!

{
  echo "RUN_ID=$RUN_ID"
  echo "Started at $(date -Is)"
  echo "DISPLAY=${DISPLAY:-}"
  echo "Logs: $EVENT_LOG $STATE_LOG $LAUNCH_LOG"
  echo "Observation topics of interest:"
  echo "  /maze/goal_events"
  echo "  /maze/explorer_state"
  echo "  /cmd_vel_nav"
  echo "  /local_costmap/costmap"
  echo "  /plan"
  echo "  /tf /tf_static base_link"
  echo "  /map"
  echo "Launch command:"
  echo "  ros2 launch tugbot_bringup tugbot_maze_explore.launch.py headless:=false use_rviz:=true near_exit_fallback_enabled:=true max_goals:=30"
} | tee -a "$LAUNCH_LOG"

# Snapshot topics shortly after startup without controlling shutdown.
(
  sleep 35
  ros2 topic list > "$TOPIC_SNAPSHOT" 2>&1 || true
) &

exec ros2 launch tugbot_bringup tugbot_maze_explore.launch.py \
  headless:=false \
  use_rviz:=true \
  near_exit_fallback_enabled:=true \
  max_goals:=30 \
  2>&1 | tee -a "$LAUNCH_LOG"
