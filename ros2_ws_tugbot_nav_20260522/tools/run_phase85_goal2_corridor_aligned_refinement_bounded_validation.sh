#!/usr/bin/env bash
set -eo pipefail

# Phase85 bounded visible Goal2 reproduction validation.
# Goal: validate whether Phase84 corridor-aligned refinement is present in the
# Goal2 dispatch context and whether the refined Goal2 improves execution.
# This is validation only; it does not edit strategy/config/Nav2 parameters.

RUN_ID="phase85_goal2_corridor_aligned_refinement_bounded_validation"
PHASE="Phase85 corridor-aligned refinement bounded Goal2 reproduction validation"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

MAX_GOALS="${PHASE85_MAX_GOALS:-2}"
if (( MAX_GOALS != 2 )); then
  echo "PHASE85_MAX_GOALS must remain 2: Goal1 ingress-branch success, then bounded Goal2 validation" >&2
  exit 2
fi

WORLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
NAV2_PARAMS="$ROOT/src/tugbot_navigation/config/nav2_slam_params.yaml"
ARTIFACT_DIR="$ROOT/log/${RUN_ID}"
MIN_SUMMARY="$ARTIFACT_DIR/${RUN_ID}_minimal_field_summary.md"
ANALYSIS_JSON="$ARTIFACT_DIR/${RUN_ID}_analysis.json"
RAW_CAPTURE_JSON="$ARTIFACT_DIR/${RUN_ID}_raw_capture.json"
READY_MARKER="$ARTIFACT_DIR/${RUN_ID}_SCENE_HELD_WAITING_FOR_USER_SCREENSHOT.txt"
PREFLIGHT="$ARTIFACT_DIR/${RUN_ID}_preflight.txt"
NAV2_CONFIG_DIFF="$ARTIFACT_DIR/${RUN_ID}_nav2_config_diff.txt"
STRATEGY_DIFF="$ARTIFACT_DIR/${RUN_ID}_strategy_config_diff.txt"
ROS_GRAPH_READY="$ARTIFACT_DIR/${RUN_ID}_ros_graph_ready.txt"
ROS_GRAPH_HELD="$ARTIFACT_DIR/${RUN_ID}_ros_graph_held.txt"

INNER_INGRESS_GOAL_TIMEOUT_SEC="${PHASE85_INNER_INGRESS_GOAL_TIMEOUT_SEC:-90}"
GOAL_TIMEOUT_SEC="${PHASE85_GOAL_TIMEOUT_SEC:-45.0}"
READINESS_TIMEOUT_SEC="${PHASE85_READINESS_TIMEOUT_SEC:-100}"
POST_READINESS_SETTLE_SEC="${PHASE85_POST_READINESS_SETTLE_SEC:-8}"
TRIGGER_TIMEOUT_SEC="${PHASE85_TRIGGER_TIMEOUT_SEC:-95}"
RUNTIME_RECORD_TIMEOUT_SEC="${PHASE85_RUNTIME_RECORD_TIMEOUT_SEC:-120}"
RAW_CAPTURE_DURATION_SEC="${PHASE85_RAW_CAPTURE_DURATION_SEC:-5}"
HOLD_AFTER_TRIGGER_SEC="${PHASE85_HOLD_AFTER_TRIGGER_SEC:-0}"
CLEANUP_ON_EXIT="${PHASE85_CLEANUP_ON_EXIT:-0}"
SKIP_BUILD="${PHASE85_SKIP_BUILD:-0}"

LAUNCH_PID=""
EXPLORER_PID=""
GOAL_EVENTS_PID=""
STATE_PID=""
RUNTIME_RECORDER_PID=""
PIDS=()

mkdir -p "$ARTIFACT_DIR"

cleanup() {
  set +e
  trap - EXIT
  if [[ "$CLEANUP_ON_EXIT" != "1" ]]; then
    return 0
  fi
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
  pkill -TERM -f '[r]os2 run tugbot_maze maze_explorer.*goal_events_topic:=/maze/goal_events' || true
  pkill -TERM -f '[i]nstall/tugbot_maze/lib/tugbot_maze/maze_explorer' || true
  pkill -TERM -f '[r]ecord_explorer_state_series.py.*phase85_goal2_corridor_aligned_refinement_bounded_validation' || true
  pkill -TERM -f '[a]nalyze_phase74_directional_local_costmap_readiness_gate_validation.py --record-runtime' || true
  pkill -TERM -f '[g]z sim|[r]viz2|[r]os_gz_bridge|[p]arameter_bridge|[s]lam_toolbox|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]oute_server|[c]ollision_monitor|[d]ocking_server' || true
  sleep 2
  pkill -KILL -f '[g]z sim|[r]viz2|[r]os_gz_bridge|[p]arameter_bridge|[s]lam_toolbox|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]oute_server|[c]ollision_monitor|[d]ocking_server' || true
}
trap cleanup EXIT

source /opt/ros/jazzy/setup.bash

if [[ "$SKIP_BUILD" != "1" ]]; then
  colcon build --packages-select tugbot_maze --symlink-install \
    > "$ARTIFACT_DIR/${RUN_ID}_colcon_build_tugbot_maze_stdout.log" \
    2> "$ARTIFACT_DIR/${RUN_ID}_colcon_build_tugbot_maze_stderr.log"
fi

if [[ -f "$ROOT/install/setup.bash" ]]; then
  source "$ROOT/install/setup.bash"
fi

wait_for_readiness() {
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
      } > "$ROS_GRAPH_READY" 2>&1
      return 0
    fi
    sleep 2
  done
  {
    echo "readiness_timeout_at=$(date -Is)"
    ros2 node list || true
    ros2 action list || true
    ros2 topic list || true
  } > "$ARTIFACT_DIR/${RUN_ID}_readiness_timeout_snapshot.txt" 2>&1
  return 1
}

start_recorders() {
  python3 tools/record_explorer_state_series.py --topic /maze/goal_events \
    --output "$ARTIFACT_DIR/${RUN_ID}_goal_events.jsonl" \
    --max-samples 100 \
    --min-samples 0 \
    --timeout-sec "$RUNTIME_RECORD_TIMEOUT_SEC" \
    --stop-on-terminal \
    > "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stdout.json" \
    2> "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stderr.log" &
  GOAL_EVENTS_PID="$!"
  PIDS+=("$GOAL_EVENTS_PID")

  python3 tools/record_explorer_state_series.py --topic /maze/explorer_state \
    --output "$ARTIFACT_DIR/${RUN_ID}_explorer_state.jsonl" \
    --max-samples 280 \
    --min-samples 0 \
    --timeout-sec "$RUNTIME_RECORD_TIMEOUT_SEC" \
    > "$ARTIFACT_DIR/${RUN_ID}_state_recorder_stdout.json" \
    2> "$ARTIFACT_DIR/${RUN_ID}_state_recorder_stderr.log" &
  STATE_PID="$!"
  PIDS+=("$STATE_PID")

  python3 tools/analyze_phase74_directional_local_costmap_readiness_gate_validation.py --record-runtime \
    --timeout-sec "$RUNTIME_RECORD_TIMEOUT_SEC" \
    --output "$ARTIFACT_DIR/${RUN_ID}_runtime_recorder_summary.json" \
    --timeline-output "$ARTIFACT_DIR/${RUN_ID}_runtime_timeline.jsonl" \
    --controller-dynamics-output "$ARTIFACT_DIR/${RUN_ID}_controller_dynamics.jsonl" \
    --nav2-feedback-output "$ARTIFACT_DIR/${RUN_ID}_nav2_feedback.jsonl" \
    --local-costmap-samples-output "$ARTIFACT_DIR/${RUN_ID}_local_costmap_samples.jsonl" \
    --global-plan-samples-output "$ARTIFACT_DIR/${RUN_ID}_global_plan_samples.jsonl" \
    --collision-monitor-output "$ARTIFACT_DIR/${RUN_ID}_collision_monitor_state.jsonl" \
    > "$ARTIFACT_DIR/${RUN_ID}_runtime_recorder_stdout.json" \
    2> "$ARTIFACT_DIR/${RUN_ID}_runtime_recorder_stderr.log" &
  RUNTIME_RECORDER_PID="$!"
  PIDS+=("$RUNTIME_RECORDER_PID")
}

start_explorer() {
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
    -p max_goals:="$MAX_GOALS" \
    -p goal_timeout_sec:="$GOAL_TIMEOUT_SEC" \
    -p near_exit_fallback_enabled:=false \
    -p startup_warmup_no_dispatch:=false \
    -p topology_consistency_enabled:=true \
    -p topology_consistency_required_no_candidate_frames:=2 \
    -p topology_consistency_window_sec:=4.0 \
    -p post_ingress_single_open_exception_enabled:=true \
    -p centerline_target_refinement_enabled:=true \
    -p centerline_target_refinement_gate_mode:=balance_first \
    -p directional_local_costmap_readiness_override_enabled:=true \
    -p centerline_target_refinement_min_clearance_floor_m:=0.45 \
    -p centerline_target_refinement_forward_progress_tolerance_m:=0.05 \
    > "$ARTIFACT_DIR/${RUN_ID}_maze_explorer_stdout.log" \
    2> "$ARTIFACT_DIR/${RUN_ID}_maze_explorer_stderr.log" &
  EXPLORER_PID="$!"
  PIDS+=("$EXPLORER_PID")
  echo "$EXPLORER_PID" > "$ARTIFACT_DIR/${RUN_ID}_maze_explorer.pid"
}

wait_for_goal2_terminal_or_context() {
  local start events_file trigger_file
  start=$(date +%s)
  events_file="$ARTIFACT_DIR/${RUN_ID}_goal_events.jsonl"
  trigger_file="$ARTIFACT_DIR/${RUN_ID}_trigger_detected.json"
  while (( $(date +%s) - start < TRIGGER_TIMEOUT_SEC )); do
    if [[ -s "$events_file" ]] && python3 - "$events_file" "$trigger_file" <<'PY'
import json, sys
from pathlib import Path
rows=[]
for line in Path(sys.argv[1]).read_text(encoding='utf-8', errors='replace').splitlines():
    if not line.strip():
        continue
    try:
        raw=json.loads(line)
    except Exception:
        continue
    state=raw.get('state') if isinstance(raw.get('state'), dict) else raw
    if isinstance(state, dict):
        state=dict(state)
        state['_recorder_elapsed_sec']=raw.get('elapsed_sec')
        rows.append(state)
g2=[r for r in rows if r.get('goal_sequence') == 2]
dispatch=next((r for r in g2 if r.get('event') == 'dispatch'), None)
outcome=next((r for r in g2 if r.get('event') in {'success','failure','timeout','timeout_cancel_result'} or r.get('result_reason') in {'goal_timeout','goal_canceled_after_timeout','goal_reached'}), None)
if dispatch and outcome:
    out={
        'elapsed_sec': outcome.get('_recorder_elapsed_sec'),
        'event': outcome.get('event'),
        'result_reason': outcome.get('result_reason'),
        'goal_sequence': 2,
        'target': outcome.get('target') or dispatch.get('target'),
        'refinement_applied': dispatch.get('refinement_applied'),
        'original_target': dispatch.get('original_target'),
        'centerline_projected_target': dispatch.get('centerline_projected_target'),
        'corridor_heading_yaw': dispatch.get('corridor_heading_yaw'),
    }
    Path(sys.argv[2]).write_text(json.dumps(out, indent=2, sort_keys=True)+'\n', encoding='utf-8')
    raise SystemExit(0)
raise SystemExit(1)
PY
    then
      return 0
    fi
    sleep 2
  done
  return 1
}

{
  echo "run_id=$RUN_ID"
  echo "phase=$PHASE"
  echo "visible mode: headless=false use_rviz=true"
  echo "max_goals=$MAX_GOALS"
  echo "goal_timeout_sec=$GOAL_TIMEOUT_SEC"
  echo "guardrails=No navigation strategy changed; No Nav2/MPPI/controller tuning; no inflation/robot_radius/clearance_radius_m/map threshold tuning; no branch scoring/centerline/directional readiness/fallback/terminal acceptance edits; No autonomous exploration success claimed; No exit success claimed; Phase86 not entered."
  echo "nav2_config_diff_begin"
  git diff -- src/tugbot_navigation/config | tee "$NAV2_CONFIG_DIFF" || true
  echo "nav2_config_diff_end"
  echo "strategy_config_diff_begin"
  git diff -- src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_maze/tugbot_maze/maze_perception.py src/tugbot_bringup/launch src/tugbot_navigation/config | tee "$STRATEGY_DIFF" || true
  echo "strategy_config_diff_end"
} > "$PREFLIGHT"

ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py \
  world_sdf:="$WORLD" \
  slam_params_file:="$SLAM_PARAMS" \
  params_file:="$NAV2_PARAMS" \
  headless:=false \
  use_rviz:=true \
  use_sim_time:=true \
  autostart:=true \
  > "$ARTIFACT_DIR/${RUN_ID}_visible_launch.log" 2>&1 &
LAUNCH_PID="$!"
PIDS+=("$LAUNCH_PID")
echo "$LAUNCH_PID" > "$ARTIFACT_DIR/${RUN_ID}_visible_launch.pid"

if ! wait_for_readiness; then
  echo "REPRODUCTION_START_BLOCKED_READINESS_TIMEOUT" > "$ARTIFACT_DIR/${RUN_ID}_blocked.txt"
  exit 3
fi

sleep "$POST_READINESS_SETTLE_SEC"

python3 tools/analyze_phase74_directional_local_costmap_readiness_gate_validation.py --send-inner-ingress-goal \
  --goal-timeout-sec "$INNER_INGRESS_GOAL_TIMEOUT_SEC" \
  --output "$ARTIFACT_DIR/${RUN_ID}_inner_ingress_navigate_to_pose_action_result.json" \
  > "$ARTIFACT_DIR/${RUN_ID}_inner_ingress_goal_stdout.json" \
  2> "$ARTIFACT_DIR/${RUN_ID}_inner_ingress_goal_stderr.log" || true

if ! python3 - "$ARTIFACT_DIR/${RUN_ID}_inner_ingress_navigate_to_pose_action_result.json" <<'PY'
import json, sys
from pathlib import Path
p=Path(sys.argv[1])
data=json.loads(p.read_text()) if p.exists() and p.stat().st_size else {}
raise SystemExit(0 if data.get('success') else 1)
PY
then
  echo "REPRODUCTION_START_BLOCKED_INNER_INGRESS_FAILED" > "$ARTIFACT_DIR/${RUN_ID}_blocked.txt"
  exit 4
fi

start_recorders
sleep 2
start_explorer

if wait_for_goal2_terminal_or_context; then
  if [[ -n "${EXPLORER_PID:-}" ]] && kill -0 "$EXPLORER_PID" 2>/dev/null; then
    kill -TERM "$EXPLORER_PID" 2>/dev/null || true
    wait "$EXPLORER_PID" 2>/dev/null || true
  fi
  pkill -TERM -f '[i]nstall/tugbot_maze/lib/tugbot_maze/maze_explorer' || true
  pkill -TERM -f '[r]os2 run tugbot_maze maze_explorer' || true
else
  echo "BOUNDED_WINDOW_ELAPSED_WITHOUT_GOAL2_TERMINAL_OUTCOME" > "$ARTIFACT_DIR/${RUN_ID}_bounded_window_elapsed.txt"
fi

python3 tools/record_phase85_goal2_corridor_aligned_refinement_evidence.py \
  --duration-sec "$RAW_CAPTURE_DURATION_SEC" \
  --output-json "$RAW_CAPTURE_JSON" \
  > "$ARTIFACT_DIR/${RUN_ID}_raw_capture_stdout.json" \
  2> "$ARTIFACT_DIR/${RUN_ID}_raw_capture_stderr.log" || true

python3 tools/analyze_phase85_goal2_corridor_aligned_refinement_validation.py \
  --artifact-dir "$ARTIFACT_DIR" \
  --output-json "$ANALYSIS_JSON" \
  --minimal-summary "$MIN_SUMMARY"

{
  echo "SCENE_HELD_WAITING_FOR_USER_SCREENSHOT"
  echo "timestamp=$(date -Is)"
  echo "artifact_dir=$ARTIFACT_DIR"
  echo "analysis_json=$ANALYSIS_JSON"
  echo "minimal_field_summary=$MIN_SUMMARY"
  echo "instruction=If classification is timeout/insufficient and scene is visible, take 1-4 screenshots plus a brief judgment; no long report required."
} > "$READY_MARKER"

{
  echo "held_at=$(date -Is)"
  ros2 node list || true
  ros2 action list || true
  ros2 topic list || true
} > "$ROS_GRAPH_HELD" 2>&1

if (( HOLD_AFTER_TRIGGER_SEC > 0 )); then
  sleep "$HOLD_AFTER_TRIGGER_SEC"
  exit 0
fi

while true; do
  sleep 30
done
