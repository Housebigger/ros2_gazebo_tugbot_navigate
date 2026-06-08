#!/usr/bin/env bash
set -eo pipefail

# Phase106: Phase105-preflighted carry-over bounded Goal1 staging validation.
# Validation-only. Visible Gazebo/RViz/SLAM/Nav2, fail-closed Phase105
# ingress preflight, unchanged explicit inner-ingress goal only after preflight
# pass, then bounded max_goals=1 Goal1 carry-over/staging observation.
# No maze_explorer strategy, Phase88/92/101/105 logic, branch scoring,
# exploration order, centerline gate, directional readiness, fallback/terminal
# acceptance, Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map threshold tuning.

RUN_ID="phase106_preflighted_carry_over_bounded_goal1_staging_validation"
PHASE="Phase106 Phase105-preflighted carry-over bounded Goal1 staging validation"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

MAX_GOALS="${PHASE106_MAX_GOALS:-1}"
if (( MAX_GOALS != 1 )); then
  echo "PHASE106_MAX_GOALS must be 1: bounded Goal1 validation only" >&2
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
INGRESS_PREFLIGHT_JSON="$ARTIFACT_DIR/${RUN_ID}_ingress_preflight.json"
TRIGGER_DETECTED="$ARTIFACT_DIR/${RUN_ID}_trigger_detected.json"
INITIAL_CLEANUP_JSON="$ARTIFACT_DIR/${RUN_ID}_initial_cleanup_summary.json"
PROCESS_SCAN_AFTER_INITIAL="$ARTIFACT_DIR/${RUN_ID}_process_scan_after_initial_cleanup.txt"

PHASE106_INGRESS_X="${PHASE106_INGRESS_X:-2.0}"
PHASE106_INGRESS_Y="${PHASE106_INGRESS_Y:-0.0}"
PHASE106_INGRESS_YAW="${PHASE106_INGRESS_YAW:-0.0}"
INGRESS_TIMEOUT_SEC="${PHASE106_INGRESS_TIMEOUT_SEC:-90}"
GOAL_TIMEOUT_SEC="${PHASE106_GOAL_TIMEOUT_SEC:-190.0}"
READINESS_TIMEOUT_SEC="${PHASE106_READINESS_TIMEOUT_SEC:-150}"
POST_READINESS_SETTLE_SEC="${PHASE106_POST_READINESS_SETTLE_SEC:-5}"
VALIDATION_WINDOW_SEC="${PHASE106_VALIDATION_WINDOW_SEC:-260}"
RUNTIME_RECORD_TIMEOUT_SEC="${PHASE106_RUNTIME_RECORD_TIMEOUT_SEC:-320}"
RAW_CAPTURE_DURATION_SEC="${PHASE106_RAW_CAPTURE_DURATION_SEC:-5}"
INGRESS_PREFLIGHT_TIMEOUT_SEC="${PHASE106_INGRESS_PREFLIGHT_TIMEOUT_SEC:-20}"
INGRESS_PREFLIGHT_TF_STABILITY_WINDOW_SEC="${PHASE106_INGRESS_PREFLIGHT_TF_STABILITY_WINDOW_SEC:-2.0}"
INGRESS_PREFLIGHT_SAMPLE_PERIOD_SEC="${PHASE106_INGRESS_PREFLIGHT_SAMPLE_PERIOD_SEC:-0.5}"
INGRESS_PREFLIGHT_TF_MAX_AGE_SEC="${PHASE106_INGRESS_PREFLIGHT_TF_MAX_AGE_SEC:-1.5}"
INGRESS_PREFLIGHT_SCAN_MAX_AGE_SEC="${PHASE106_INGRESS_PREFLIGHT_SCAN_MAX_AGE_SEC:-1.5}"
CLEANUP_ON_EXIT="${PHASE106_CLEANUP_ON_EXIT:-0}"
SKIP_BUILD="${PHASE106_SKIP_BUILD:-0}"

LAUNCH_PID=""
EXPLORER_PID=""
GOAL_EVENTS_PID=""
STATE_PID=""
RUNTIME_RECORDER_PID=""
PIDS=()
mkdir -p "$ARTIFACT_DIR"

scan_project_processes() {
  pgrep -af 'ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py|ros2 run tugbot_maze maze_explorer|install/tugbot_maze/lib/tugbot_maze/maze_explorer|record_explorer_state_series.py|record_phase(102|106)_|analyze_phase74_directional_local_costmap_readiness_gate_validation.py --record-runtime|analyze_phase74_directional_local_costmap_readiness_gate_validation.py --send-inner-ingress-goal|phase105_inner_ingress_tf_controller_preflight.py|gz sim|ruby .*gz sim|ros_gz_bridge|parameter_bridge|static_transform_publisher|slam_toolbox|async_slam_toolbox_node|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower|velocity_smoother|lifecycle_manager_navigation|route_server|collision_monitor|rviz2' || true
}

initial_cleanup() {
  local before after
  before="$(scan_project_processes)"
  printf '%s\n' "$before" > "$ARTIFACT_DIR/${RUN_ID}_process_scan_before_initial_cleanup.txt"
  pkill -TERM -f '[r]os2 run tugbot_maze maze_explorer' || true
  pkill -TERM -f '[i]nstall/tugbot_maze/lib/tugbot_maze/maze_explorer' || true
  pkill -TERM -f '[r]ecord_explorer_state_series.py' || true
  pkill -TERM -f '[r]ecord_phase(102|106)_' || true
  pkill -TERM -f '[a]nalyze_phase74_directional_local_costmap_readiness_gate_validation.py --record-runtime' || true
  pkill -TERM -f '[a]nalyze_phase74_directional_local_costmap_readiness_gate_validation.py --send-inner-ingress-goal' || true
  pkill -TERM -f '[p]hase105_inner_ingress_tf_controller_preflight.py' || true
  pkill -TERM -f '[r]os2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py' || true
  pkill -TERM -f '[r]os_gz_bridge|[p]arameter_bridge|[s]tatic_transform_publisher' || true
  pkill -TERM -f '[s]lam_toolbox|[a]sync_slam_toolbox_node|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[s]moother_server|[w]aypoint_follower|[v]elocity_smoother|[l]ifecycle_manager_navigation|[r]oute_server|[c]ollision_monitor|[d]ocking_server' || true
  pkill -TERM -f '[g]z sim|[r]uby .*gz sim|[r]viz2' || true
  sleep 3
  pkill -KILL -f '[r]os_gz_bridge|[p]arameter_bridge|[s]tatic_transform_publisher|[m]aze_explorer|[s]lam_toolbox|[a]sync_slam_toolbox_node|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[g]z sim|[r]uby .*gz sim|[r]viz2' || true
  sleep 1
  after="$(scan_project_processes)"
  printf '%s\n' "$after" > "$PROCESS_SCAN_AFTER_INITIAL"
  python3 - "$INITIAL_CLEANUP_JSON" "$before" "$after" <<'PY'
import json, sys
from pathlib import Path
before = [line for line in sys.argv[2].splitlines() if line.strip()]
after = [line for line in sys.argv[3].splitlines() if line.strip()]
Path(sys.argv[1]).write_text(json.dumps({
    'scope': 'project-scoped Gazebo/RViz/SLAM/Nav2/maze_explorer/recorder/ros2 launch processes only',
    'before_count': len(before),
    'after_count': len(after),
    'before': before,
    'after': after,
    'unrelated_processes_targeted': False,
}, indent=2, sort_keys=True) + '\n')
PY
}

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
initial_cleanup
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
      && timeout 8 bash -c 'ros2 run tf2_ros tf2_echo map base_link 2>&1 | tee /tmp/phase106_tf_ready.txt | grep -m 1 "Translation:" >/dev/null'; then
      {
        echo "ready_at=$(date -Is)"
        echo "TF ready: map->base_link"
        echo "/navigate_to_pose ready"
        echo "/map ready"
        echo "/scan ready"
        echo "/local_costmap/costmap ready"
        cat /tmp/phase106_tf_ready.txt || true
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

start_visible_stack() {
  echo "visible_stack_fresh_after_initial_cleanup=true" > "$ARTIFACT_DIR/${RUN_ID}_visible_stack_mode.txt"
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
    --max-samples 400 --min-samples 0 --timeout-sec "$RUNTIME_RECORD_TIMEOUT_SEC" --stop-on-terminal \
    > "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stdout.json" \
    2> "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stderr.log" &
  GOAL_EVENTS_PID="$!"; PIDS+=("$GOAL_EVENTS_PID")

  python3 tools/record_explorer_state_series.py --topic /maze/explorer_state \
    --output "$ARTIFACT_DIR/${RUN_ID}_explorer_state.jsonl" \
    --max-samples 800 --min-samples 0 --timeout-sec "$RUNTIME_RECORD_TIMEOUT_SEC" \
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

run_ingress_preflight() {
  python3 tools/phase105_inner_ingress_tf_controller_preflight.py \
    --timeout-sec "$INGRESS_PREFLIGHT_TIMEOUT_SEC" \
    --tf-stability-window-sec "$INGRESS_PREFLIGHT_TF_STABILITY_WINDOW_SEC" \
    --sample-period-sec "$INGRESS_PREFLIGHT_SAMPLE_PERIOD_SEC" \
    --tf-max-age-sec "$INGRESS_PREFLIGHT_TF_MAX_AGE_SEC" \
    --scan-max-age-sec "$INGRESS_PREFLIGHT_SCAN_MAX_AGE_SEC" \
    --output "$INGRESS_PREFLIGHT_JSON" \
    > "$ARTIFACT_DIR/${RUN_ID}_ingress_preflight_stdout.json" \
    2> "$ARTIFACT_DIR/${RUN_ID}_ingress_preflight_stderr.log"
}

mark_ingress_preflight_wrapper_state() {
  local ingress_goal_sent="$1"
  local maze_explorer_started="$2"
  python3 tools/phase105_inner_ingress_tf_controller_preflight.py \
    --mark-wrapper-state \
    --output "$INGRESS_PREFLIGHT_JSON" \
    --ingress-goal-sent "$ingress_goal_sent" \
    --maze-explorer-started "$maze_explorer_started" \
    > "$ARTIFACT_DIR/${RUN_ID}_ingress_preflight_mark_stdout.json" \
    2> "$ARTIFACT_DIR/${RUN_ID}_ingress_preflight_mark_stderr.log" || true
}

write_preflight_reject_ingress_result() {
  python3 - "$INGRESS_PREFLIGHT_JSON" "$INGRESS_RESULT" "$TRIGGER_DETECTED" <<'PY'
import json, sys
from pathlib import Path
preflight_path = Path(sys.argv[1])
ingress_path = Path(sys.argv[2])
trigger_path = Path(sys.argv[3])
try:
    data = json.loads(preflight_path.read_text(encoding='utf-8', errors='replace'))
except Exception:
    data = {}
preflight = data.get('ingress_preflight') if isinstance(data.get('ingress_preflight'), dict) else {}
reason = preflight.get('ingress_preflight_reject_reason') or 'ingress_preflight_timeout'
result = {
    'success': False,
    'status': 'preflight_rejected',
    'ingress_goal_sent': False,
    'maze_explorer_started': False,
    'ingress_preflight_reject_reason': reason,
    'ingress_preflight': preflight,
    'target': [2.0, 0.0, 0.0],
}
trigger = {
    'trigger': 'ingress_preflight_rejected_explorer_not_started',
    'ingress_preflight_reject_reason': reason,
}
ingress_path.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
trigger_path.write_text(json.dumps(trigger, indent=2, sort_keys=True) + '\n')
PY
}

send_ingress_goal() {
  echo "explicit unchanged inner-ingress Nav2 goal after Phase105 preflight pass: frame_id=map x=2.0 y=0.0 yaw=0.0" > "$ARTIFACT_DIR/${RUN_ID}_ingress_goal_intent.txt"
  python3 tools/analyze_phase74_directional_local_costmap_readiness_gate_validation.py --send-inner-ingress-goal \
    --goal-timeout-sec "$INGRESS_TIMEOUT_SEC" \
    --output "$INGRESS_RESULT" \
    > "$ARTIFACT_DIR/${RUN_ID}_ingress_goal_stdout.json" \
    2> "$ARTIFACT_DIR/${RUN_ID}_ingress_goal_stderr.log" || true
}

wait_for_ingress_terminal() {
  python3 - "$INGRESS_RESULT" "$PHASE106_INGRESS_X" "$PHASE106_INGRESS_Y" "$PHASE106_INGRESS_YAW" <<'PY'
import json, sys
from pathlib import Path
path = Path(sys.argv[1])
if not path.exists() or not path.stat().st_size:
    fallback = {'success': False, 'status': 'missing', 'ingress_goal_sent': True, 'target': [float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])], 'reason': 'ingress_result_missing'}
    path.write_text(json.dumps(fallback, indent=2, sort_keys=True) + '\n')
    raise SystemExit(1)
data = json.loads(path.read_text())
data.setdefault('target', [float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])])
data['ingress_goal_sent'] = True
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

wait_for_goal1_carry_over_or_terminal() {
  local start events_file trigger_file
  start=$(date +%s)
  events_file="$ARTIFACT_DIR/${RUN_ID}_goal_events.jsonl"
  trigger_file="$TRIGGER_DETECTED"
  while (( $(date +%s) - start < VALIDATION_WINDOW_SEC )); do
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
    state=raw.get('state') if isinstance(raw, dict) and isinstance(raw.get('state'), dict) else raw
    if isinstance(state, dict):
        rows.append(state)
goal1=[r for r in rows if int(r.get('goal_sequence') or -1) == 1]
dispatch=[r for r in goal1 if str(r.get('event')).lower() == 'dispatch']
terminal_events={'success','succeeded','timeout','failure','failed','cancel','canceled','cancelled'}
terminals=[r for r in goal1 if str(r.get('event')).lower() in terminal_events]
trigger=None
if dispatch:
    d=dispatch[-1]
    if 'corridor_evidence_carry_over' in d or 'carry_over_applied' in d or d.get('staging_reject_reason') is not None or d.get('staging_applied') is not None:
        trigger={'trigger':'staging_or_carry_over_context_observed','goal_sequence':1,'event':d.get('event'),'carry_over_applied':d.get('carry_over_applied'),'staging_applied':d.get('staging_applied'),'staging_reject_reason':d.get('staging_reject_reason')}
if terminals and trigger is None:
    trigger={'trigger':'goal1_terminal_outcome_observed','goal_sequence':1,'terminal_event':terminals[-1].get('event')}
if trigger:
    Path(sys.argv[2]).write_text(json.dumps(trigger, indent=2, sort_keys=True)+'\n')
    raise SystemExit(0)
raise SystemExit(1)
PY
    then
      return 0
    fi
    if [[ -n "$EXPLORER_PID" ]] && ! kill -0 "$EXPLORER_PID" 2>/dev/null; then
      echo '{"trigger":"maze_explorer_exited_before_goal1_context"}' > "$trigger_file"
      return 0
    fi
    sleep 2
  done
  echo '{"trigger":"bounded_goal1_validation_window_expired"}' > "$trigger_file"
  return 0
}

run_analyzer() {
  python3 tools/analyze_phase106_preflighted_carry_over_bounded_goal1_staging_validation.py \
    --artifact-dir "$ARTIFACT_DIR" \
    --run-id "$RUN_ID" \
    --output-json "$ANALYSIS_JSON" \
    --minimal-summary-output "$MIN_SUMMARY"
}

{
  echo "phase=$PHASE"
  echo "run_id=$RUN_ID"
  echo "world=$WORLD"
  echo "initial_cleanup=project-scoped before visible stack"
  echo "inner_ingress_goal=(frame_id=map,x=${PHASE106_INGRESS_X},y=${PHASE106_INGRESS_Y},yaw=${PHASE106_INGRESS_YAW})"
  echo "max_goals=$MAX_GOALS"
  echo "goal_timeout_sec=$GOAL_TIMEOUT_SEC"
  echo "validation_window_sec=$VALIDATION_WINDOW_SEC"
  echo "ingress_preflight=Phase105 fail-closed TF/controller preflight before sender"
  echo "ingress_preflight_timeout_sec=$INGRESS_PREFLIGHT_TIMEOUT_SEC"
  echo "ingress_preflight_reject_contract=ingress_goal_sent=false,maze_explorer_started=false"
  echo "readiness=/navigate_to_pose,/map,/scan,/local_costmap/costmap,TF ready"
  echo "No Phase88/92/101/105 logic changed"
  echo "No branch scoring/exploration order/centerline gate/directional readiness/fallback/terminal acceptance changed"
  echo "No Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map threshold tuning"
  echo "No autonomous exploration success claimed"
  echo "No exit success claimed"
  echo "Phase107 not entered"
  git diff -- src/tugbot_navigation/config || true
} > "$ARTIFACT_DIR/${RUN_ID}_preflight.txt" 2>&1

start_visible_stack
wait_for_readiness
sleep "$POST_READINESS_SETTLE_SEC"
start_recorders
sleep 2
if run_ingress_preflight; then
  mark_ingress_preflight_wrapper_state true false
  send_ingress_goal
  if wait_for_ingress_terminal; then
    start_explorer
    mark_ingress_preflight_wrapper_state true true
    wait_for_goal1_carry_over_or_terminal
  else
    echo '{"trigger":"ingress_failed_explorer_not_started"}' > "$TRIGGER_DETECTED"
  fi
else
  mark_ingress_preflight_wrapper_state false false
  write_preflight_reject_ingress_result
fi

run_analyzer > "$ARTIFACT_DIR/${RUN_ID}_analyzer_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_analyzer_stderr.log" || true

python3 tools/record_phase106_preflighted_carry_over_validation_evidence.py \
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
