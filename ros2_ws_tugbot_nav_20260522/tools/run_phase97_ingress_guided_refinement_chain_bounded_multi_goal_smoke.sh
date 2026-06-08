#!/usr/bin/env bash
set -eo pipefail

# Phase97: Ingress-guided refinement chain bounded multi-goal smoke.
# Validation-only: visible Gazebo/RViz/SLAM/Nav2, explicit inner-ingress Nav2
# goal before maze_explorer, then max_goals:=2~3 bounded smoke until the
# observed goal set reaches terminal outcome or a bounded timeout/failure.
# No maze_explorer strategy, Phase88/92 refinement logic, branch scoring,
# exploration order, centerline gate, directional readiness, fallback/terminal
# acceptance, Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map
# threshold tuning.

RUN_ID="phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke"
PHASE="Phase97 Ingress-guided refinement chain bounded multi-goal smoke"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

MAX_GOALS="${PHASE97_MAX_GOALS:-3}"
if (( MAX_GOALS < 2 || MAX_GOALS > 3 )); then
  echo "PHASE97_MAX_GOALS must be 2 or 3: max_goals:=2~3 bounded smoke only" >&2
  exit 2
fi

WORLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
NAV2_PARAMS="$ROOT/src/tugbot_navigation/config/nav2_slam_params.yaml"
ARTIFACT_DIR="$ROOT/log/${RUN_ID}"
MIN_SUMMARY="$ARTIFACT_DIR/${RUN_ID}_minimal_field_summary.md"
ANALYSIS_JSON="$ARTIFACT_DIR/${RUN_ID}_analysis.json"
RAW_CAPTURE_JSON="$ARTIFACT_DIR/${RUN_ID}_raw_capture.json"
INGRESS_RESULT="$ARTIFACT_DIR/${RUN_ID}_ingress_result.json"
TRIGGER_DETECTED="$ARTIFACT_DIR/${RUN_ID}_trigger_detected.json"

PHASE97_INGRESS_X="${PHASE97_INGRESS_X:-2.0}"
PHASE97_INGRESS_Y="${PHASE97_INGRESS_Y:-0.0}"
PHASE97_INGRESS_YAW="${PHASE97_INGRESS_YAW:-0.0}"
INGRESS_TIMEOUT_SEC="${PHASE97_INGRESS_TIMEOUT_SEC:-90}"
GOAL_TIMEOUT_SEC="${PHASE97_GOAL_TIMEOUT_SEC:-180.0}"
READINESS_TIMEOUT_SEC="${PHASE97_READINESS_TIMEOUT_SEC:-140}"
POST_READINESS_SETTLE_SEC="${PHASE97_POST_READINESS_SETTLE_SEC:-5}"
SMOKE_WINDOW_SEC="${PHASE97_SMOKE_WINDOW_SEC:-420}"
RUNTIME_RECORD_TIMEOUT_SEC="${PHASE97_RUNTIME_RECORD_TIMEOUT_SEC:-480}"
RAW_CAPTURE_DURATION_SEC="${PHASE97_RAW_CAPTURE_DURATION_SEC:-5}"
CLEANUP_ON_EXIT="${PHASE97_CLEANUP_ON_EXIT:-0}"
REUSE_VISIBLE_STACK="${PHASE97_REUSE_VISIBLE_STACK:-0}"
SKIP_BUILD="${PHASE97_SKIP_BUILD:-0}"

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
    echo "cleanup_on_exit=false; preserving visible scene if stack is still running" > "$ARTIFACT_DIR/${RUN_ID}_cleanup_on_exit_decision.txt"
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
      && ros2 topic list 2>/dev/null | grep -q '^/scan$' \
      && ros2 topic list 2>/dev/null | grep -q '^/local_costmap/costmap$' \
      && timeout 8 bash -c 'ros2 run tf2_ros tf2_echo map base_link 2>&1 | tee /tmp/phase97_tf_ready.txt | grep -m 1 "Translation:" >/dev/null'; then
      {
        echo "ready_at=$(date -Is)"
        echo "TF ready: map->base_link"
        echo "/navigate_to_pose ready"
        echo "/map ready"
        echo "/scan ready"
        echo "/local_costmap/costmap ready"
        cat /tmp/phase97_tf_ready.txt || true
        ros2 node list || true
        ros2 action list || true
        ros2 topic list || true
      } > "$ARTIFACT_DIR/${RUN_ID}_ros_graph_ready.txt" 2>&1
      return 0
    fi
    sleep 2
  done
  {
    echo "readiness_timeout_at=$(date -Is)"
    echo "TF ready check failed or required ROS interfaces missing"
    ros2 node list || true
    ros2 action list || true
    ros2 topic list || true
  } > "$ARTIFACT_DIR/${RUN_ID}_readiness_timeout_snapshot.txt" 2>&1
  return 1
}

start_visible_stack_if_needed() {
  if [[ "$REUSE_VISIBLE_STACK" == "1" ]]; then
    echo "reuse_visible_stack=true" > "$ARTIFACT_DIR/${RUN_ID}_visible_stack_mode.txt"
    return 0
  fi
  ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py \
    world_sdf:="$WORLD" \
    slam_params_file:="$SLAM_PARAMS" \
    params_file:="$NAV2_PARAMS" \
    headless:=false \
    use_rviz:=true \
    use_sim_time:=true \
    autostart:=true \
    > "$ARTIFACT_DIR/${RUN_ID}_visible_launch.log" \
    2>&1 &
  LAUNCH_PID="$!"; PIDS+=("$LAUNCH_PID")
  echo "$LAUNCH_PID" > "$ARTIFACT_DIR/${RUN_ID}_visible_launch.pid"
}

start_recorders() {
  python3 tools/record_explorer_state_series.py --topic /maze/goal_events \
    --output "$ARTIFACT_DIR/${RUN_ID}_goal_events.jsonl" \
    --max-samples 600 \
    --min-samples 0 \
    --timeout-sec "$RUNTIME_RECORD_TIMEOUT_SEC" \
    --stop-on-terminal \
    > "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stdout.json" \
    2> "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stderr.log" &
  GOAL_EVENTS_PID="$!"; PIDS+=("$GOAL_EVENTS_PID")

  python3 tools/record_explorer_state_series.py --topic /maze/explorer_state \
    --output "$ARTIFACT_DIR/${RUN_ID}_explorer_state.jsonl" \
    --max-samples 1000 \
    --min-samples 0 \
    --timeout-sec "$RUNTIME_RECORD_TIMEOUT_SEC" \
    > "$ARTIFACT_DIR/${RUN_ID}_state_recorder_stdout.json" \
    2> "$ARTIFACT_DIR/${RUN_ID}_state_recorder_stderr.log" &
  STATE_PID="$!"; PIDS+=("$STATE_PID")

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
  RUNTIME_RECORDER_PID="$!"; PIDS+=("$RUNTIME_RECORDER_PID")
}

send_ingress_goal() {
  # Explicit goal equivalent for auditability:
  # ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: ${PHASE97_INGRESS_X}, y: ${PHASE97_INGRESS_Y}, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
  # The existing Phase65 action client is used for robust terminal JSON and feedback capture.
  echo "explicit inner-ingress Nav2 goal before maze_explorer" > "$ARTIFACT_DIR/${RUN_ID}_ingress_goal_intent.txt"
  python3 tools/analyze_phase65_ingress_goal_depth_adjustment_inner_staging.py --send-inner-ingress-goal \
    --goal-timeout-sec "$INGRESS_TIMEOUT_SEC" \
    --output "$INGRESS_RESULT" \
    > "$ARTIFACT_DIR/${RUN_ID}_ingress_goal_stdout.json" \
    2> "$ARTIFACT_DIR/${RUN_ID}_ingress_goal_stderr.log" || true
}

wait_for_ingress_terminal() {
  python3 - "$INGRESS_RESULT" "$PHASE97_INGRESS_X" "$PHASE97_INGRESS_Y" "$PHASE97_INGRESS_YAW" <<'PY'
import json, sys
from pathlib import Path
path = Path(sys.argv[1])
if not path.exists() or not path.stat().st_size:
    fallback = {'success': False, 'status': 'missing', 'target': [float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])], 'reason': 'ingress_result_missing'}
    path.write_text(json.dumps(fallback, indent=2, sort_keys=True) + '\n')
    raise SystemExit(1)
data = json.loads(path.read_text())
data.setdefault('target', [float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])])
status = str(data.get('status') or data.get('status_text') or '').lower()
if data.get('success') is True or status in {'succeeded', 'status_succeeded'} or status.endswith('succeeded'):
    data['status'] = 'succeeded'
    data['success'] = True
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n')
    raise SystemExit(0)
data['status'] = data.get('status') or 'failed'
data['success'] = False
path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n')
raise SystemExit(1)
PY
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
  EXPLORER_PID="$!"; PIDS+=("$EXPLORER_PID")
  echo "$EXPLORER_PID" > "$ARTIFACT_DIR/${RUN_ID}_maze_explorer.pid"
}

wait_for_terminal_set_or_failure() {
  local start events_file trigger_file
  start=$(date +%s)
  events_file="$ARTIFACT_DIR/${RUN_ID}_goal_events.jsonl"
  trigger_file="$TRIGGER_DETECTED"
  while (( $(date +%s) - start < SMOKE_WINDOW_SEC )); do
    if [[ -s "$events_file" ]] && python3 - "$events_file" "$trigger_file" "$MAX_GOALS" <<'PY'
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
    state=raw.get('state') if isinstance(raw, dict) and isinstance(raw.get('state'), dict) else raw
    if isinstance(state, dict):
        rows.append(state)
max_goals=int(sys.argv[3])
dispatch=[r for r in rows if str(r.get('event')).lower() == 'dispatch']
terminal_events={'success','succeeded','timeout','failure','failed','cancel','canceled','cancelled'}
terminals=[r for r in rows if str(r.get('event')).lower() in terminal_events]
terminal_seqs=sorted({int(r.get('goal_sequence')) for r in terminals if r.get('goal_sequence') is not None})
dispatch_seqs=sorted({int(r.get('goal_sequence')) for r in dispatch if r.get('goal_sequence') is not None})
bad=[r for r in terminals if str(r.get('event')).lower() in {'timeout','failure','failed','cancel','canceled','cancelled'}]
staging=[r for r in dispatch if bool(r.get('staging_applied'))]
trigger=None
if bad:
    trigger={'trigger':'goal_terminal_failure_observed','dispatch_sequences':dispatch_seqs,'terminal_sequences':terminal_seqs,'bad_terminal_event':bad[-1].get('event')}
elif staging:
    trigger={'trigger':'staging_applied_observed','dispatch_sequences':dispatch_seqs,'terminal_sequences':terminal_seqs}
elif len(terminal_seqs) >= min(2, max_goals):
    trigger={'trigger':'bounded_terminal_goal_set_observed','dispatch_sequences':dispatch_seqs,'terminal_sequences':terminal_seqs}
elif len(dispatch_seqs) >= max_goals and len(terminal_seqs) >= max_goals:
    trigger={'trigger':'max_goals_terminal_set_observed','dispatch_sequences':dispatch_seqs,'terminal_sequences':terminal_seqs}
if trigger:
    Path(sys.argv[2]).write_text(json.dumps(trigger, indent=2, sort_keys=True)+'\n')
    raise SystemExit(0)
raise SystemExit(1)
PY
    then
      return 0
    fi
    if [[ -n "$EXPLORER_PID" ]] && ! kill -0 "$EXPLORER_PID" 2>/dev/null; then
      echo '{"trigger":"maze_explorer_exited_before_terminal_set"}' > "$trigger_file"
      return 0
    fi
    sleep 2
  done
  echo '{"trigger":"bounded_smoke_window_expired_before_terminal_set"}' > "$trigger_file"
  return 0
}

run_analyzer() {
  python3 tools/analyze_phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke.py \
    --artifact-dir "$ARTIFACT_DIR" \
    --run-id "$RUN_ID" \
    --max-goals "$MAX_GOALS" \
    --output-json "$ANALYSIS_JSON" \
    --minimal-summary-output "$MIN_SUMMARY"
}

{
  echo "phase=$PHASE"
  echo "run_id=$RUN_ID"
  echo "world=$WORLD"
  echo "reuse_visible_stack=$REUSE_VISIBLE_STACK"
  echo "inner_ingress_goal=(${PHASE97_INGRESS_X},${PHASE97_INGRESS_Y},${PHASE97_INGRESS_YAW})"
  echo "max_goals=$MAX_GOALS"
  echo "max_goals:=2~3"
  echo "goal_timeout_sec=$GOAL_TIMEOUT_SEC"
  echo "smoke_window_sec=$SMOKE_WINDOW_SEC"
  echo "readiness=/navigate_to_pose,/map,/scan,/local_costmap/costmap,TF ready"
  echo "explicit inner-ingress Nav2 goal before maze_explorer"
  echo "wait_for_terminal_set_or_failure"
  echo "No Phase88/92 refinement logic changed"
  echo "No branch scoring/exploration order/centerline gate/directional readiness/fallback/terminal acceptance changed"
  echo "No Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map threshold tuning"
  echo "No autonomous exploration success claimed"
  echo "No exit success claimed"
  echo "Phase98 not entered"
  git diff -- src/tugbot_navigation/config || true
} > "$ARTIFACT_DIR/${RUN_ID}_preflight.txt" 2>&1

start_visible_stack_if_needed
wait_for_readiness
sleep "$POST_READINESS_SETTLE_SEC"
start_recorders
sleep 2
send_ingress_goal
if wait_for_ingress_terminal; then
  start_explorer
  wait_for_terminal_set_or_failure
else
  echo '{"trigger":"ingress_failed_explorer_not_started"}' > "$TRIGGER_DETECTED"
fi

run_analyzer > "$ARTIFACT_DIR/${RUN_ID}_analyzer_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_analyzer_stderr.log" || true

python3 tools/record_phase97_smoke_evidence.py \
  --duration-sec "$RAW_CAPTURE_DURATION_SEC" \
  --analysis-json "$ANALYSIS_JSON" \
  --output "$RAW_CAPTURE_JSON" \
  > "$ARTIFACT_DIR/${RUN_ID}_raw_capture_stdout.json" \
  2> "$ARTIFACT_DIR/${RUN_ID}_raw_capture_stderr.log" || true

run_analyzer

{
  echo "held_or_cleanup_at=$(date -Is)"
  echo "trigger=$(cat "$TRIGGER_DETECTED" 2>/dev/null || true)"
  ros2 node list || true
  ros2 topic list || true
} > "$ARTIFACT_DIR/${RUN_ID}_ros_graph_final.txt" 2>&1
