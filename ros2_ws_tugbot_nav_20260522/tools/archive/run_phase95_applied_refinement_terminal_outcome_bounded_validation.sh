#!/usr/bin/env bash
set -eo pipefail

# Phase95 bounded visible validation: run only until the Goal2-equivalent
# applied Phase88 refinement reaches a terminal outcome (success / timeout /
# cancel / failure) or a finite terminal-window cap expires. Validation only.
# No maze_explorer strategy edits, no Phase88/92 logic edits, no branch scoring,
# exploration order, centerline gate, directional readiness, fallback/terminal
# acceptance edits, and no Nav2/MPPI/controller/inflation/robot_radius/
# clearance_radius_m/map-threshold tuning.

RUN_ID="phase95_applied_refinement_terminal_outcome_bounded_validation"
PHASE="Phase95 Applied refinement terminal outcome bounded validation"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

MAX_GOALS="${PHASE95_MAX_GOALS:-3}"
if (( MAX_GOALS < 2 || MAX_GOALS > 3 )); then
  echo "PHASE95_MAX_GOALS must be 2 or 3: bounded Goal2-equivalent validation only" >&2
  exit 2
fi

WORLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
NAV2_PARAMS="$ROOT/src/tugbot_navigation/config/nav2_slam_params.yaml"
ARTIFACT_DIR="$ROOT/log/${RUN_ID}"
MIN_SUMMARY="$ARTIFACT_DIR/${RUN_ID}_minimal_field_summary.md"
ANALYSIS_JSON="$ARTIFACT_DIR/${RUN_ID}_analysis.json"
RAW_CAPTURE_JSON="$ARTIFACT_DIR/${RUN_ID}_raw_capture.json"
PREFLIGHT="$ARTIFACT_DIR/${RUN_ID}_preflight.txt"
ROS_GRAPH_READY="$ARTIFACT_DIR/${RUN_ID}_ros_graph_ready.txt"
ROS_GRAPH_HELD="$ARTIFACT_DIR/${RUN_ID}_ros_graph_held.txt"
TRIGGER_DETECTED="$ARTIFACT_DIR/${RUN_ID}_terminal_detected.json"
READY_MARKER="$ARTIFACT_DIR/${RUN_ID}_SCENE_HELD_WAITING_FOR_USER_SCREENSHOT.txt"

INNER_INGRESS_GOAL_TIMEOUT_SEC="${PHASE95_INNER_INGRESS_GOAL_TIMEOUT_SEC:-90}"
GOAL_TIMEOUT_SEC="${PHASE95_GOAL_TIMEOUT_SEC:-180.0}"
READINESS_TIMEOUT_SEC="${PHASE95_READINESS_TIMEOUT_SEC:-120}"
POST_READINESS_SETTLE_SEC="${PHASE95_POST_READINESS_SETTLE_SEC:-8}"
TERMINAL_WINDOW_SEC="${PHASE95_TERMINAL_WINDOW_SEC:-260}"
RUNTIME_RECORD_TIMEOUT_SEC="${PHASE95_RUNTIME_RECORD_TIMEOUT_SEC:-320}"
RAW_CAPTURE_DURATION_SEC="${PHASE95_RAW_CAPTURE_DURATION_SEC:-5}"
HOLD_AFTER_TRIGGER_SEC="${PHASE95_HOLD_AFTER_TRIGGER_SEC:-0}"
CLEANUP_ON_EXIT="${PHASE95_CLEANUP_ON_EXIT:-0}"
SKIP_BUILD="${PHASE95_SKIP_BUILD:-0}"

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
    --max-samples 240 \
    --min-samples 0 \
    --timeout-sec "$RUNTIME_RECORD_TIMEOUT_SEC" \
    --stop-on-terminal \
    > "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stdout.json" \
    2> "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stderr.log" &
  GOAL_EVENTS_PID="$!"
  PIDS+=("$GOAL_EVENTS_PID")

  python3 tools/record_explorer_state_series.py --topic /maze/explorer_state \
    --output "$ARTIFACT_DIR/${RUN_ID}_explorer_state.jsonl" \
    --max-samples 480 \
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

wait_for_goal2_terminal_outcome() {
  local start events_file trigger_file
  start=$(date +%s)
  events_file="$ARTIFACT_DIR/${RUN_ID}_goal_events.jsonl"
  trigger_file="$TRIGGER_DETECTED"
  while (( $(date +%s) - start < TERMINAL_WINDOW_SEC )); do
    if [[ -s "$events_file" ]] && python3 - "$events_file" "$trigger_file" <<'PY'
import json, math, sys
from pathlib import Path
old=[2.057855221699651, 1.0261005743935105]
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
        row=dict(state)
        row['_recorder_elapsed_sec']=raw.get('elapsed_sec')
        rows.append(row)
def seq(row):
    try: return int(row.get('goal_sequence'))
    except Exception: return None
def xy(v):
    if isinstance(v, dict) and 'x' in v and 'y' in v:
        return [v.get('x'), v.get('y')]
    if isinstance(v, (list, tuple)) and len(v) >= 2:
        return [v[0], v[1]]
    return None
def dist(v):
    p=xy(v)
    try: return math.hypot(float(p[0])-old[0], float(p[1])-old[1])
    except Exception: return 999.0
def ref(row):
    r=row.get('centerline_target_refinement')
    return r if isinstance(r, dict) else row
def matches_goal2(row):
    r=ref(row)
    values=[row.get('target'), row.get('original_target'), row.get('refined_target'), row.get('selected_candidate_target'), r.get('target'), r.get('original_target'), r.get('refined_target'), r.get('selected_candidate_target')]
    return any(dist(v) <= 0.45 for v in values)
def applied(row):
    r=ref(row)
    return bool(r.get('refinement_applied')) and r.get('hard_safety_pass_candidate_count') is not None and r.get('selected_candidate_target') is not None

dispatches=[r for r in rows if r.get('event')=='dispatch' and matches_goal2(r) and applied(r)]
if not dispatches:
    raise SystemExit(1)
dispatch=dispatches[0]
validation_seq=seq(dispatch)
related=[r for r in rows if validation_seq is None or seq(r)==validation_seq]
terminal_events={'success','succeeded','timeout','failure','failed','cancel','canceled','timeout_cancel_result'}
terminal_reasons={'goal_timeout','goal_canceled_after_timeout','goal_canceled','canceled','goal_reached','succeeded','nav2_failed'}
terminal=next((r for r in related if r.get('event') in terminal_events or r.get('result_reason') in terminal_reasons), None)
if terminal is None:
    raise SystemExit(1)
out={
    'elapsed_sec': terminal.get('_recorder_elapsed_sec'),
    'goal_sequence': validation_seq,
    'dispatch_target': dispatch.get('target'),
    'selected_candidate_target': ref(dispatch).get('selected_candidate_target'),
    'selected_candidate_yaw': ref(dispatch).get('selected_candidate_yaw'),
    'hard_safety_pass_candidate_count': ref(dispatch).get('hard_safety_pass_candidate_count'),
    'terminal_event': terminal.get('event'),
    'terminal_reason': terminal.get('result_reason'),
    'terminal_pose': terminal.get('robot_pose'),
}
Path(sys.argv[2]).write_text(json.dumps(out, indent=2, sort_keys=True)+'\n', encoding='utf-8')
raise SystemExit(0)
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
  echo "visible mode: headless:=false use_rviz:=true"
  echo "max_goals=$MAX_GOALS"
  echo "goal_timeout_sec=$GOAL_TIMEOUT_SEC"
  echo "PHASE95_TERMINAL_WINDOW_SEC=$TERMINAL_WINDOW_SEC"
  echo "validation_scope=Goal2-equivalent terminal outcome only; no algorithm/config tuning"
  echo "guardrails=No maze_explorer strategy changed; No Phase88/92 logic changed; No branch scoring changed; No exploration order changed; No centerline gate changed; No directional readiness changed; No fallback/terminal acceptance changed; No Nav2/MPPI/controller tuning; no inflation/robot_radius/clearance_radius_m/map threshold tuning; No autonomous exploration success claimed; No exit success claimed; Phase96 not entered."
  echo "nav2_config_diff_begin"
  git diff -- src/tugbot_navigation/config || true
  echo "nav2_config_diff_end"
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
  echo "PHASE95_REPRODUCTION_START_BLOCKED_READINESS_TIMEOUT" > "$ARTIFACT_DIR/${RUN_ID}_blocked.txt"
  python3 tools/analyze_phase95_applied_refinement_terminal_outcome_bounded_validation.py --artifact-dir "$ARTIFACT_DIR" --output-json "$ANALYSIS_JSON" --minimal-summary "$MIN_SUMMARY" || true
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
  echo "PHASE95_REPRODUCTION_START_BLOCKED_INNER_INGRESS_FAILED" > "$ARTIFACT_DIR/${RUN_ID}_blocked.txt"
  python3 tools/analyze_phase95_applied_refinement_terminal_outcome_bounded_validation.py --artifact-dir "$ARTIFACT_DIR" --output-json "$ANALYSIS_JSON" --minimal-summary "$MIN_SUMMARY" || true
  exit 4
fi

start_recorders
sleep 2
start_explorer

if wait_for_goal2_terminal_outcome; then
  if [[ -n "${EXPLORER_PID:-}" ]] && kill -0 "$EXPLORER_PID" 2>/dev/null; then
    kill -TERM "$EXPLORER_PID" 2>/dev/null || true
    wait "$EXPLORER_PID" 2>/dev/null || true
  fi
  pkill -TERM -f '[i]nstall/tugbot_maze/lib/tugbot_maze/maze_explorer' || true
  pkill -TERM -f '[r]os2 run tugbot_maze maze_explorer' || true
else
  echo "PHASE95_BOUNDED_TERMINAL_WINDOW_ELAPSED_WITHOUT_TERMINAL_OUTCOME" > "$ARTIFACT_DIR/${RUN_ID}_bounded_terminal_window_elapsed.txt"
  if [[ -n "${EXPLORER_PID:-}" ]] && kill -0 "$EXPLORER_PID" 2>/dev/null; then
    kill -TERM "$EXPLORER_PID" 2>/dev/null || true
    wait "$EXPLORER_PID" 2>/dev/null || true
  fi
  pkill -TERM -f '[i]nstall/tugbot_maze/lib/tugbot_maze/maze_explorer' || true
  pkill -TERM -f '[r]os2 run tugbot_maze maze_explorer' || true
fi

python3 tools/analyze_phase95_applied_refinement_terminal_outcome_bounded_validation.py \
  --artifact-dir "$ARTIFACT_DIR" \
  --output-json "$ANALYSIS_JSON" \
  --minimal-summary "$MIN_SUMMARY" || true

python3 tools/record_phase95_terminal_evidence.py \
  --duration-sec "$RAW_CAPTURE_DURATION_SEC" \
  --source-analysis-json "$ANALYSIS_JSON" \
  --output-json "$RAW_CAPTURE_JSON" \
  > "$ARTIFACT_DIR/${RUN_ID}_raw_capture_stdout.json" \
  2> "$ARTIFACT_DIR/${RUN_ID}_raw_capture_stderr.log" || true

python3 tools/analyze_phase95_applied_refinement_terminal_outcome_bounded_validation.py \
  --artifact-dir "$ARTIFACT_DIR" \
  --output-json "$ANALYSIS_JSON" \
  --minimal-summary "$MIN_SUMMARY"

{
  echo "SCENE_HELD_WAITING_FOR_USER_SCREENSHOT"
  echo "timestamp=$(date -Is)"
  echo "artifact_dir=$ARTIFACT_DIR"
  echo "analysis_json=$ANALYSIS_JSON"
  echo "minimal_field_summary=$MIN_SUMMARY"
  echo "instruction=If terminal outcome is timeout/local-cost/recovery dominant, capture Gazebo wide view plus RViz local costmap/footprint/front wedge and goal/tolerance screenshots. No long report required."
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
