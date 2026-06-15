#!/usr/bin/env bash
set -eo pipefail

# Phase96 bounded visible smoke: validate the Phase88/92 refinement chain across
# only 2..3 consecutive explore goals. Validation only: no maze_explorer strategy
# edits, no Phase88/92 logic edits, no branch scoring, exploration order,
# centerline gate, directional readiness, fallback/terminal acceptance edits,
# and no Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map
# threshold tuning.

RUN_ID="phase96_refinement_chain_bounded_multi_goal_smoke"
PHASE="Phase96 Phase88/92 refinement chain bounded multi-goal smoke"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

MAX_GOALS="${PHASE96_MAX_GOALS:-3}"
if (( MAX_GOALS < 2 || MAX_GOALS > 3 )); then
  echo "PHASE96_MAX_GOALS must be 2 or 3: max_goals:=2~3 bounded smoke only" >&2
  exit 2
fi

WORLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
NAV2_PARAMS="$ROOT/src/tugbot_navigation/config/nav2_slam_params.yaml"
ARTIFACT_DIR="$ROOT/log/${RUN_ID}"
MIN_SUMMARY="$ARTIFACT_DIR/${RUN_ID}_minimal_field_summary.md"
ANALYSIS_JSON="$ARTIFACT_DIR/${RUN_ID}_analysis.json"
RAW_CAPTURE_JSON="$ARTIFACT_DIR/${RUN_ID}_raw_capture.json"
TRIGGER_DETECTED="$ARTIFACT_DIR/${RUN_ID}_terminal_detected.json"
READY_MARKER="$ARTIFACT_DIR/${RUN_ID}_SCENE_HELD_WAITING_FOR_USER_SCREENSHOT.txt"

GOAL_TIMEOUT_SEC="${PHASE96_GOAL_TIMEOUT_SEC:-180.0}"
READINESS_TIMEOUT_SEC="${PHASE96_READINESS_TIMEOUT_SEC:-120}"
POST_READINESS_SETTLE_SEC="${PHASE96_POST_READINESS_SETTLE_SEC:-8}"
SMOKE_WINDOW_SEC="${PHASE96_SMOKE_WINDOW_SEC:-360}"
RUNTIME_RECORD_TIMEOUT_SEC="${PHASE96_RUNTIME_RECORD_TIMEOUT_SEC:-420}"
RAW_CAPTURE_DURATION_SEC="${PHASE96_RAW_CAPTURE_DURATION_SEC:-5}"
HOLD_AFTER_TRIGGER_SEC="${PHASE96_HOLD_AFTER_TRIGGER_SEC:-0}"
CLEANUP_ON_EXIT="${PHASE96_CLEANUP_ON_EXIT:-0}"
SKIP_BUILD="${PHASE96_SKIP_BUILD:-0}"

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
      } > "$ARTIFACT_DIR/${RUN_ID}_ros_graph_ready.txt" 2>&1
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
    --max-samples 300 \
    --min-samples 0 \
    --timeout-sec "$RUNTIME_RECORD_TIMEOUT_SEC" \
    --stop-on-terminal \
    > "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stdout.json" \
    2> "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stderr.log" &
  GOAL_EVENTS_PID="$!"; PIDS+=("$GOAL_EVENTS_PID")

  python3 tools/record_explorer_state_series.py --topic /maze/explorer_state \
    --output "$ARTIFACT_DIR/${RUN_ID}_explorer_state.jsonl" \
    --max-samples 720 \
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

wait_for_smoke_terminal_set() {
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
    state=raw.get('state') if isinstance(raw.get('state'), dict) else raw
    if isinstance(state, dict):
        rows.append(state)
max_goals=int(sys.argv[3])
by_seq={}
for row in rows:
    try:
        seq=int(row.get('goal_sequence'))
    except Exception:
        continue
    by_seq.setdefault(seq, []).append(row)
terminal_events={'success','succeeded','timeout','failure','failed','cancel','canceled','cancelled'}
terminals={seq: [r for r in seq_rows if str(r.get('event')).lower() in terminal_events] for seq, seq_rows in by_seq.items()}
trigger=None
for seq, seq_rows in sorted(by_seq.items()):
    for row in seq_rows:
        event=str(row.get('event')).lower()
        if event in {'timeout','failure','failed','cancel','canceled','cancelled'}:
            trigger={'trigger':'failure_or_timeout','goal_sequence':seq,'event':event,'observed_sequences':sorted(by_seq),'terminal_sequences':sorted(k for k,v in terminals.items() if v)}
            break
    if trigger:
        break
if trigger is None and len([k for k,v in terminals.items() if v]) >= max_goals:
    trigger={'trigger':'max_goals_terminal_set','observed_sequences':sorted(by_seq),'terminal_sequences':sorted(k for k,v in terminals.items() if v)}
if trigger:
    Path(sys.argv[2]).write_text(json.dumps(trigger, indent=2, sort_keys=True)+'\n')
    sys.exit(0)
sys.exit(1)
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
  echo '{"trigger":"bounded_smoke_window_expired"}' > "$trigger_file"
  return 0
}

{
  echo "phase=$PHASE"
  echo "run_id=$RUN_ID"
  echo "world=$WORLD"
  echo "max_goals=$MAX_GOALS"
  echo "goal_timeout_sec=$GOAL_TIMEOUT_SEC"
  echo "smoke_window_sec=$SMOKE_WINDOW_SEC"
  echo "validation_only=true"
  echo "max_goals:=2~3"
} > "$ARTIFACT_DIR/${RUN_ID}_preflight.txt"

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

wait_for_readiness
sleep "$POST_READINESS_SETTLE_SEC"
start_recorders
sleep 2
start_explorer
wait_for_smoke_terminal_set

python3 tools/analyze_phase96_refinement_chain_bounded_multi_goal_smoke.py \
  --artifact-dir "$ARTIFACT_DIR" \
  --run-id "$RUN_ID" \
  --max-goals "$MAX_GOALS" \
  --output-json "$ANALYSIS_JSON" \
  --minimal-summary-output "$MIN_SUMMARY" \
  > "$ARTIFACT_DIR/${RUN_ID}_analyzer_stdout.json" \
  2> "$ARTIFACT_DIR/${RUN_ID}_analyzer_stderr.log" || true

python3 tools/record_phase96_smoke_evidence.py \
  --duration-sec "$RAW_CAPTURE_DURATION_SEC" \
  --analysis-json "$ANALYSIS_JSON" \
  --output "$RAW_CAPTURE_JSON" \
  > "$ARTIFACT_DIR/${RUN_ID}_raw_capture_stdout.json" \
  2> "$ARTIFACT_DIR/${RUN_ID}_raw_capture_stderr.log" || true

python3 tools/analyze_phase96_refinement_chain_bounded_multi_goal_smoke.py \
  --artifact-dir "$ARTIFACT_DIR" \
  --run-id "$RUN_ID" \
  --max-goals "$MAX_GOALS" \
  --output-json "$ANALYSIS_JSON" \
  --minimal-summary-output "$MIN_SUMMARY"

CLASSIFICATION="$(python3 - "$ANALYSIS_JSON" <<'PY'
import json, sys
try:
    print(json.load(open(sys.argv[1])).get('classification','UNKNOWN'))
except Exception:
    print('UNKNOWN')
PY
)"

{
  echo "held_at=$(date -Is)"
  echo "classification=$CLASSIFICATION"
  echo "trigger=$(cat "$TRIGGER_DETECTED" 2>/dev/null || true)"
  ros2 node list || true
  ros2 topic list || true
} > "$ARTIFACT_DIR/${RUN_ID}_ros_graph_held.txt" 2>&1

echo "SCENE HELD for Phase96 classification: $CLASSIFICATION" > "$READY_MARKER"

if [[ "$CLASSIFICATION" == "REFINEMENT_CHAIN_BOUNDED_SMOKE_PASS" ]]; then
  CLEANUP_ON_EXIT="1"
  cleanup
fi

if [[ "$HOLD_AFTER_TRIGGER_SEC" != "0" ]]; then
  sleep "$HOLD_AFTER_TRIGGER_SEC"
fi
