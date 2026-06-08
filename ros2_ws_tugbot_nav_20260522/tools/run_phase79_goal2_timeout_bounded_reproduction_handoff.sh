#!/usr/bin/env bash
set -eo pipefail

# Phase79 Goal2 timeout bounded reproduction handoff run
# Goal: run to Goal2 problem point, stop continued exploration after the known
# timeout/local-cost/recovery-loop trigger, and hold scene for screenshot.
# This is a visible Phase78 handoff run, not an algorithm repair run.

RUN_ID="phase79_goal2_timeout_bounded_reproduction_handoff"
PHASE="Phase79 Goal2 timeout bounded reproduction handoff run"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

MAX_GOALS="${PHASE79_MAX_GOALS:-2}"
if (( MAX_GOALS != 2 )); then
  echo "PHASE79_MAX_GOALS must remain 2: Goal1 ingress-branch success, then Goal2 timeout reproduction" >&2
  exit 2
fi

WORLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
NAV2_PARAMS="$ROOT/src/tugbot_navigation/config/nav2_slam_params.yaml"
PHASE75_JSON="$ROOT/log/phase75_goal2_timeout_after_directional_redispatch_diagnosis/phase75_goal2_timeout_after_directional_redispatch_diagnosis.json"
PHASE77_EVENT_JSON="$ROOT/log/phase77_goal2_visual_root_cause_replay/phase77_goal2_timeout_visual_root_cause_event.json"
PHASE74_RUN_ID="phase74_directional_local_costmap_readiness_gate_validation"
PHASE74_REPLAY_DIR="$ROOT/log/${PHASE74_RUN_ID}/replay_02"
ARTIFACT_DIR="$ROOT/log/${RUN_ID}"
MIN_SUMMARY="$ARTIFACT_DIR/${RUN_ID}_minimal_field_summary.md"
READY_MARKER="$ARTIFACT_DIR/${RUN_ID}_SCENE_HELD_WAITING_FOR_USER_SCREENSHOT.txt"
PREFLIGHT="$ARTIFACT_DIR/${RUN_ID}_preflight.txt"
NAV2_CONFIG_DIFF="$ARTIFACT_DIR/${RUN_ID}_nav2_config_diff.txt"
STRATEGY_DIFF="$ARTIFACT_DIR/${RUN_ID}_strategy_config_diff.txt"
ROS_GRAPH_READY="$ARTIFACT_DIR/${RUN_ID}_ros_graph_ready.txt"
ROS_GRAPH_HELD="$ARTIFACT_DIR/${RUN_ID}_ros_graph_held.txt"

INNER_INGRESS_GOAL_TIMEOUT_SEC="${PHASE79_INNER_INGRESS_GOAL_TIMEOUT_SEC:-90}"
GOAL_TIMEOUT_SEC="${PHASE79_GOAL_TIMEOUT_SEC:-45.0}"
READINESS_TIMEOUT_SEC="${PHASE79_READINESS_TIMEOUT_SEC:-100}"
POST_READINESS_SETTLE_SEC="${PHASE79_POST_READINESS_SETTLE_SEC:-8}"
TRIGGER_TIMEOUT_SEC="${PHASE79_TRIGGER_TIMEOUT_SEC:-95}"
RUNTIME_RECORD_TIMEOUT_SEC="${PHASE79_RUNTIME_RECORD_TIMEOUT_SEC:-120}"
HOLD_AFTER_TRIGGER_SEC="${PHASE79_HOLD_AFTER_TRIGGER_SEC:-0}"
CLEANUP_ON_EXIT="${PHASE79_CLEANUP_ON_EXIT:-0}"

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
  pkill -TERM -f '[a]nalyze_phase74_directional_local_costmap_readiness_gate_validation.py --record-runtime' || true
  pkill -TERM -f '[r]ecord_explorer_state_series.py.*phase79_goal2_timeout_bounded_reproduction_handoff' || true
  pkill -TERM -f '[g]z sim|[r]viz2|[r]os_gz_bridge|[p]arameter_bridge|[s]lam_toolbox|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]oute_server|[c]ollision_monitor|[d]ocking_server' || true
  sleep 2
  pkill -KILL -f '[g]z sim|[r]viz2|[r]os_gz_bridge|[p]arameter_bridge|[s]lam_toolbox|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]oute_server|[c]ollision_monitor|[d]ocking_server' || true
  ros2 daemon stop >/dev/null 2>&1 || true
}
trap cleanup EXIT

source /opt/ros/jazzy/setup.bash
if [[ -f "$ROOT/install/setup.bash" ]]; then
  source "$ROOT/install/setup.bash"
fi

write_minimal_summary() {
  local status="$1"
  local trigger="$2"
  python3 - "$MIN_SUMMARY" "$status" "$trigger" "$ARTIFACT_DIR" "$PHASE75_JSON" "$PHASE77_EVENT_JSON" <<'PY'
import json, sys
from pathlib import Path
out = Path(sys.argv[1])
status = sys.argv[2]
trigger = sys.argv[3]
artifact_dir = Path(sys.argv[4])
phase75_path = Path(sys.argv[5])
phase77_path = Path(sys.argv[6])
phase75 = json.loads(phase75_path.read_text()) if phase75_path.exists() else {}
phase77 = json.loads(phase77_path.read_text()) if phase77_path.exists() else {}
robot_pose = phase77.get('terminal_pose') or phase77.get('final_pose') or phase75.get('final_pose')
target = phase77.get('dispatch_target') or phase77.get('target') or phase75.get('goal2_actual_target') or phase75.get('goal2_refined_target')
recoveries = phase77.get('number_of_recoveries') or phase75.get('nav2_recovery_summary', {}).get('number_of_recoveries_max')
xy_error = phase77.get('final_xy_error_m') or phase75.get('nav2_result_summary', {}).get('final_xy_error_m')
key = f"Goal2 timeout chain: target={target}, final/terminal pose={robot_pose}, recoveries={recoveries}, final_xy_error_m={xy_error}."
lines = [
    '# Phase79 minimal field summary',
    '',
    f'Status: {status}',
    'Trigger: goal_timeout / local_cost_risk / recovery loop / near-goal outside tolerance',
    f'Observed trigger: {trigger}',
    f'Run/artifacts: {artifact_dir}',
    f'Robot pose: {robot_pose}',
    f'Target: {target}',
    f'Key evidence: {key}',
    '',
    'Screenshot suggestions (1-4 images only):',
    '- Gazebo wide: robot stopped at the Goal2 problem area and nearby corridor/walls.',
    '- RViz local costmap / footprint / front wedge: show local-cost risk around the robot nose and footprint.',
    '- Goal tolerance: show target marker/tolerance circle vs final robot pose.',
    '- Recovery loop: if visible, show Nav2 recovery/local oscillation context.',
    '',
    'No lengthy human observation report is required; send screenshots plus a brief judgment to ChatGPT discussion, then Hermes will execute only a confirmed plan.',
]
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text('\n'.join(lines) + '\n', encoding='utf-8')
PY
}

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
    --max-samples 80 \
    --min-samples 0 \
    --timeout-sec "$RUNTIME_RECORD_TIMEOUT_SEC" \
    --stop-on-terminal \
    > "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stdout.json" \
    2> "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stderr.log" &
  GOAL_EVENTS_PID="$!"
  PIDS+=("$GOAL_EVENTS_PID")

  python3 tools/record_explorer_state_series.py --topic /maze/explorer_state \
    --output "$ARTIFACT_DIR/${RUN_ID}_explorer_state.jsonl" \
    --max-samples 260 \
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

wait_for_goal2_trigger() {
  local start events_file trigger_file
  start=$(date +%s)
  events_file="$ARTIFACT_DIR/${RUN_ID}_goal_events.jsonl"
  trigger_file="$ARTIFACT_DIR/${RUN_ID}_trigger_detected.json"
  while (( $(date +%s) - start < TRIGGER_TIMEOUT_SEC )); do
    if [[ -s "$events_file" ]] && python3 - "$events_file" "$trigger_file" <<'PY'
import json, sys
from pathlib import Path
events = Path(sys.argv[1])
out = Path(sys.argv[2])
for line in events.read_text(encoding='utf-8', errors='replace').splitlines():
    if not line.strip():
        continue
    try:
        row = json.loads(line)
    except Exception:
        continue
    state = row.get('state') if isinstance(row.get('state'), dict) else row
    if not isinstance(state, dict):
        continue
    if state.get('goal_sequence') == 2 and (
        state.get('event') in {'timeout', 'timeout_cancel_result'}
        or state.get('result_reason') in {'goal_timeout', 'goal_canceled_after_timeout'}
        or state.get('timeout_front_wedge_cost_max') is not None
    ):
        out.write_text(json.dumps({'elapsed_sec': row.get('elapsed_sec'), 'event': state.get('event'), 'result_reason': state.get('result_reason'), 'goal_sequence': state.get('goal_sequence'), 'target': state.get('target'), 'timeout_front_wedge_cost_max': state.get('timeout_front_wedge_cost_max')}, sort_keys=True, indent=2) + '\n', encoding='utf-8')
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
  echo "workflow=automatic bounded reproduction -> user screenshot -> ChatGPT discussion -> Hermes executes confirmed plan"
  echo "source_chain=Phase75/74 Goal2 timeout chain"
  echo "phase74_run_id=$PHASE74_RUN_ID"
  echo "phase74_replay_dir=$PHASE74_REPLAY_DIR"
  echo "phase75_json=$PHASE75_JSON"
  echo "phase77_goal2_timeout_visual_root_cause_event.json=$PHASE77_EVENT_JSON"
  echo "visible mode: headless=false use_rviz=true"
  echo "max_goals=$MAX_GOALS"
  echo "goal_timeout_sec=$GOAL_TIMEOUT_SEC"
  echo "guardrails=No navigation strategy changed; No Nav2/MPPI/controller tuning; no inflation/robot_radius/clearance_radius_m/map threshold tuning; no branch scoring/centerline/directional readiness/fallback/terminal acceptance edits; No autonomous exploration success claimed; No exit success claimed; algorithm repair phase not entered."
  echo "nav2_config_diff_begin"
  git diff -- src/tugbot_navigation/config | tee "$NAV2_CONFIG_DIFF" || true
  echo "nav2_config_diff_end"
  echo "strategy_config_diff_begin"
  git diff -- src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_bringup/launch src/tugbot_navigation/config | tee "$STRATEGY_DIFF" || true
  echo "strategy_config_diff_end"
} > "$PREFLIGHT"

write_minimal_summary "STARTING_VISIBLE_REPRODUCTION" "pending_goal2_timeout"

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
  write_minimal_summary "REPRODUCTION_START_BLOCKED_READINESS_TIMEOUT" "readiness_timeout"
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
p = Path(sys.argv[1])
data = json.loads(p.read_text()) if p.exists() else {}
raise SystemExit(0 if data.get('success') else 1)
PY
then
  write_minimal_summary "REPRODUCTION_START_BLOCKED_INNER_INGRESS_FAILED" "inner_ingress_failed"
  echo "REPRODUCTION_START_BLOCKED_INNER_INGRESS_FAILED" > "$ARTIFACT_DIR/${RUN_ID}_blocked.txt"
  exit 4
fi

start_recorders
sleep 2
start_explorer

if wait_for_goal2_trigger; then
  if [[ -n "${EXPLORER_PID:-}" ]] && kill -0 "$EXPLORER_PID" 2>/dev/null; then
    kill -TERM "$EXPLORER_PID" 2>/dev/null || true
    wait "$EXPLORER_PID" 2>/dev/null || true
  fi
  # The ros2 CLI wrapper can exit before the installed Python executable does.
  # Enforce the Phase79 handoff rule: stop continued exploration while keeping
  # Gazebo/RViz/SLAM/Nav2 alive for screenshots.
  pkill -TERM -f '[i]nstall/tugbot_maze/lib/tugbot_maze/maze_explorer' || true
  pkill -TERM -f '[r]os2 run tugbot_maze maze_explorer' || true
  sleep 1
  pkill -KILL -f '[i]nstall/tugbot_maze/lib/tugbot_maze/maze_explorer' || true
  pkill -KILL -f '[r]os2 run tugbot_maze maze_explorer' || true
  TRIGGER_TEXT="goal_timeout/local_cost_risk/recovery loop/near-goal outside tolerance"
  write_minimal_summary "SCENE_HELD_WAITING_FOR_USER_SCREENSHOT" "$TRIGGER_TEXT"
  {
    echo "SCENE_HELD_WAITING_FOR_USER_SCREENSHOT"
    echo "timestamp=$(date -Is)"
    echo "artifact_dir=$ARTIFACT_DIR"
    echo "minimal_field_summary=$MIN_SUMMARY"
    echo "trigger_file=$ARTIFACT_DIR/${RUN_ID}_trigger_detected.json"
    echo "instruction=Take 1-4 screenshots, add a brief judgment, discuss with ChatGPT, then return the confirmed plan to Hermes."
  } > "$READY_MARKER"
else
  write_minimal_summary "SCENE_HELD_WAITING_FOR_USER_SCREENSHOT_TRIGGER_NOT_CONFIRMED_WITHIN_BOUND" "bounded_window_elapsed"
  {
    echo "SCENE_HELD_WAITING_FOR_USER_SCREENSHOT_TRIGGER_NOT_CONFIRMED_WITHIN_BOUND"
    echo "timestamp=$(date -Is)"
    echo "artifact_dir=$ARTIFACT_DIR"
    echo "minimal_field_summary=$MIN_SUMMARY"
    echo "note=Goal2 trigger was not confirmed within bounded window; scene still held for visual inspection."
  } > "$READY_MARKER"
fi

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

# Default for Phase79 handoff: keep Gazebo/RViz/SLAM/Nav2 alive for user screenshots.
# Use PHASE79_CLEANUP_ON_EXIT=1 or stop this shell process to clean up when done.
while true; do
  sleep 30
done
