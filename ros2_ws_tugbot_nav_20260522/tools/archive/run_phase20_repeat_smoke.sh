#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 RUN_ID" >&2
  exit 2
fi

RUN_ID="$1"
PREFIX="log/${RUN_ID}"

. /opt/ros/jazzy/setup.sh
. install/setup.sh

rm -f "${PREFIX}_launch.log" \
      "${PREFIX}_goal_events.jsonl" \
      "${PREFIX}_explorer_state.jsonl" \
      "${PREFIX}_goal_nav2_analysis.json" \
      "${PREFIX}_geometry_nav2_summary.json" \
      "${PREFIX}_goal_event_cost_summary.json"

echo "[phase20] starting recorders for ${RUN_ID}"
python3 tools/record_explorer_state_series.py \
  --topic /maze/goal_events \
  --output "${PREFIX}_goal_events.jsonl" \
  --max-samples 220 \
  --min-samples 1 \
  --timeout-sec 520 &
GOAL_EVENTS_PID=$!

python3 tools/record_explorer_state_series.py \
  --output "${PREFIX}_explorer_state.jsonl" \
  --max-samples 900 \
  --min-samples 12 \
  --timeout-sec 540 \
  --stop-on-terminal \
  --terminal-linger-sec 6.0 &
STATE_PID=$!

sleep 2

echo "[phase20] launching smoke for ${RUN_ID}"
(
  set -o pipefail
  ros2 launch tugbot_bringup tugbot_maze_explore.launch.py \
    headless:=true \
    use_rviz:=false \
    explorer_type:=maze_dfs \
    max_goals:=12 \
    goal_timeout_sec:=35.0 \
    near_exit_goal_timeout_sec:=55.0 \
    near_exit_timeout_extension_radius_m:=1.0 \
    goal_settle_sec:=1.5 \
    branch_goal_step_m:=0.9 \
    open_direction_lookahead_m:=1.8 \
    lateral_centering_search_m:=1.0 \
    clearance_radius_m:=0.34 \
    min_open_distance_m:=0.5 \
    2>&1 | tee "${PREFIX}_launch.log"
) &
LAUNCH_PID=$!

cleanup() {
  echo "[phase20] cleanup ${RUN_ID}"
  kill "${LAUNCH_PID}" 2>/dev/null || true
  kill "${GOAL_EVENTS_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
  wait "${GOAL_EVENTS_PID}" 2>/dev/null || true
  ros2 daemon stop >/dev/null 2>&1 || true
}
trap cleanup EXIT

STATE_STATUS=0
wait "${STATE_PID}" || STATE_STATUS=$?
echo "[phase20] state recorder exited status=${STATE_STATUS} for ${RUN_ID}"
cleanup
trap - EXIT

python3 tools/analyze_goal_events_with_nav2_log.py \
  --goal-events "${PREFIX}_goal_events.jsonl" \
  --log "${PREFIX}_launch.log" \
  --output-json "${PREFIX}_goal_nav2_analysis.json"

python3 tools/summarize_goal_geometry_nav2.py \
  --goal-events "${PREFIX}_goal_events.jsonl" \
  --nav2-analysis "${PREFIX}_goal_nav2_analysis.json" \
  --output-json "${PREFIX}_geometry_nav2_summary.json"

python3 tools/summarize_goal_event_local_costs.py \
  --goal-events "${PREFIX}_goal_events.jsonl" \
  --output-json "${PREFIX}_goal_event_cost_summary.json"

python3 - <<PY
import json, pathlib
prefix = pathlib.Path('${PREFIX}')
state_path = pathlib.Path(str(prefix) + '_explorer_state.jsonl')
last = {}
if state_path.exists() and state_path.stat().st_size:
    lines = state_path.read_text().splitlines()
    if lines:
        row = json.loads(lines[-1])
        last = row.get('state', row)
nav = json.loads(pathlib.Path(str(prefix) + '_goal_nav2_analysis.json').read_text())
print(json.dumps({
    'run_id': '${RUN_ID}',
    'final_mode': last.get('mode'),
    'exit_distance_m': last.get('exit_distance_m'),
    'goal_count': last.get('goal_count'),
    'goal_success_count': last.get('goal_success_count'),
    'goal_failure_count': last.get('goal_failure_count'),
    'timeout_cancel_count': last.get('timeout_cancel_count'),
    'blocked_branch_count': last.get('blocked_branch_count'),
    'blacklisted_goal_count': last.get('blacklisted_goal_count'),
    'nav2_summary': nav.get('summary'),
}, indent=2, sort_keys=True))
PY

echo "[phase20] complete ${RUN_ID}"
