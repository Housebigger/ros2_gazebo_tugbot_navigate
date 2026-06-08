#!/usr/bin/env bash
set -eo pipefail

# Phase67 Goal 1 Timeout Visual Replay / Terminal Pose Diagnosis
# Visual/manual diagnosis only. Guardrails: bounded Goal 1 replay; max_goals=1;
# no Nav2/MPPI/controller parameter edits; no inflation/robot_radius/clearance_radius_m/map threshold tuning;
# no branch scoring change; no target projection integration; no fallback/terminal acceptance change;
# no autonomous exploration success claim; no exit success claim.
#
# Default behavior leaves Gazebo/RViz/SLAM/Nav2/overlay and the bounded replay run
# available for human observation. Set PHASE67_CLEANUP_ON_EXIT=1 for CI-style cleanup.

RUN_ID="phase67_goal1_timeout_visual_replay"
PHASE="Phase67 Goal 1 Timeout Visual Replay / Terminal Pose Diagnosis"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PHASE67_MAX_GOALS="${PHASE67_MAX_GOALS:-1}"
if (( PHASE67_MAX_GOALS != 1 )); then
  echo "PHASE67_MAX_GOALS must remain 1 for Goal 1 timeout visual replay" >&2
  exit 2
fi

INNER_INGRESS_X="2.0"
INNER_INGRESS_Y="0.0"
INNER_INGRESS_YAW="0.0"
WORLD="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
NAV2_PARAMS="$ROOT/src/tugbot_navigation/config/nav2_slam_params.yaml"
PHASE66_ARTIFACT="$ROOT/log/phase66_bounded_autonomous_rerun_from_inner_ingress/phase66_bounded_autonomous_rerun_from_inner_ingress.json"
ARTIFACT_DIR="$ROOT/log/${RUN_ID}"
SUMMARY_JSON="$ARTIFACT_DIR/${RUN_ID}.json"
NAV2_CONFIG_DIFF="$ARTIFACT_DIR/${RUN_ID}_nav2_config_diff.txt"
PREFLIGHT="$ARTIFACT_DIR/${RUN_ID}_preflight.txt"
READY_MARKER="$ARTIFACT_DIR/${RUN_ID}_ready_for_human_observation.txt"
CLEANUP_ON_EXIT="${PHASE67_CLEANUP_ON_EXIT:-0}"
READINESS_TIMEOUT_SEC="${PHASE67_READINESS_TIMEOUT_SEC:-90}"
POST_READINESS_SETTLE_SEC="${PHASE67_POST_READINESS_SETTLE_SEC:-8}"
GOAL_TIMEOUT_SEC="${PHASE67_GOAL_TIMEOUT_SEC:-110}"
EXPLORER_OBSERVE_SEC="${PHASE67_EXPLORER_OBSERVE_SEC:-65}"
RUNTIME_RECORD_TIMEOUT_SEC="${PHASE67_RUNTIME_RECORD_TIMEOUT_SEC:-85}"
VISUAL_SETTLE_SEC="${PHASE67_VISUAL_SETTLE_SEC:-5}"
PIDS=()
LAUNCH_PID=""
STATE_PID=""
GOAL_EVENTS_PID=""
RUNTIME_RECORDER_PID=""
EXPLORER_PID=""

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
  pkill -TERM -f '[a]nalyze_phase66_bounded_autonomous_rerun_from_inner_ingress.py --send-inner-ingress-goal' || true
  pkill -TERM -f '[a]nalyze_phase66_bounded_autonomous_rerun_from_inner_ingress.py --record-runtime' || true
  pkill -TERM -f '[r]ecord_explorer_state_series.py.*phase67_goal1_timeout_visual_replay' || true
  pkill -TERM -f '[r]os2 run tugbot_maze maze_explorer' || true
  pkill -TERM -f '[p]hase67_goal1_timeout_visual_overlay|[m]aze_explorer|[g]z sim|[r]viz2|[r]os_gz_bridge|[p]arameter_bridge|[s]lam_toolbox|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]oute_server|[c]ollision_monitor|[d]ocking_server' || true
  sleep 2
  pkill -KILL -f '[p]hase67_goal1_timeout_visual_overlay|[m]aze_explorer|[g]z sim|[r]viz2|[r]os_gz_bridge|[p]arameter_bridge|[s]lam_toolbox|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]oute_server|[c]ollision_monitor|[d]ocking_server' || true
  ros2 daemon stop >/dev/null 2>&1 || true
}
trap cleanup EXIT

# Source ROS without set -u; ROS setup files may read unset AMENT_TRACE_SETUP_FILES.
source /opt/ros/jazzy/setup.bash
if [[ -f "$ROOT/install/setup.bash" ]]; then
  source "$ROOT/install/setup.bash"
fi

python3 tools/analyze_phase67_goal1_timeout_visual_replay.py \
  --phase66-artifact "$PHASE66_ARTIFACT" \
  --output "$SUMMARY_JSON" \
  > "$ARTIFACT_DIR/${RUN_ID}_payload_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_payload_stderr.log"

{
  echo "run_id=$RUN_ID"
  echo "phase=$PHASE"
  echo "active_world=$WORLD"
  echo "phase67_payload=$SUMMARY_JSON"
  echo "inner_ingress_waypoint_map=x=${INNER_INGRESS_X},y=${INNER_INGRESS_Y},yaw=${INNER_INGRESS_YAW}"
  echo "PHASE67_MAX_GOALS=$PHASE67_MAX_GOALS"
  echo "max_goals=$PHASE67_MAX_GOALS"
  echo "marker_topic=/phase67/goal1_timeout_visual_markers"
  echo "guardrails=visual/manual diagnosis only; bounded Goal 1 replay; max_goals=1; no Nav2/MPPI/controller parameter edits; no inflation/robot_radius/clearance_radius_m/map threshold tuning; no branch scoring change; no target projection integration; no fallback/terminal acceptance change; no autonomous exploration success claim; no exit success claim."
  echo "nav2_config_diff_begin"
  git diff -- src/tugbot_navigation/config | tee "$NAV2_CONFIG_DIFF" || true
  echo "nav2_config_diff_end"
} > "$PREFLIGHT"

wait_for_readiness() {
  local start
  start=$(date +%s)
  while (( $(date +%s) - start < READINESS_TIMEOUT_SEC )); do
    if ros2 action list 2>/dev/null | grep -q '^/navigate_to_pose$' \
      && ros2 node list 2>/dev/null | grep -q '/bt_navigator' \
      && ros2 node list 2>/dev/null | grep -q '/controller_server' \
      && ros2 topic list 2>/dev/null | grep -q '^/map$' \
      && ros2 topic list 2>/dev/null | grep -q '^/local_costmap/costmap$' \
      && ros2 topic list 2>/dev/null | grep -q '^/phase67/goal1_timeout_visual_markers$'; then
      {
        echo "ready_at=$(date -Is)"
        ros2 node list || true
        ros2 action list || true
        ros2 topic list || true
      } > "$ARTIFACT_DIR/${RUN_ID}_readiness_ready_snapshot.txt" 2>&1
      return 0
    fi
    sleep 2
  done
  {
    echo "timeout_at=$(date -Is)"
    ros2 node list || true
    ros2 action list || true
    ros2 topic list || true
  } > "$ARTIFACT_DIR/${RUN_ID}_readiness_timeout_snapshot.txt" 2>&1
  return 1
}

ros2 launch tugbot_bringup phase67_goal1_timeout_visual_replay.launch.py \
  world_sdf:="$WORLD" \
  slam_params_file:="$SLAM_PARAMS" \
  params_file:="$NAV2_PARAMS" \
  phase67_payload:="$SUMMARY_JSON" \
  headless:=false \
  use_rviz:=true \
  > "$ARTIFACT_DIR/${RUN_ID}_visible_launch.log" 2>&1 &
LAUNCH_PID="$!"
PIDS+=("$LAUNCH_PID")
echo "$LAUNCH_PID" > "$ARTIFACT_DIR/${RUN_ID}_visible_launch.pid"

wait_for_readiness
sleep "$POST_READINESS_SETTLE_SEC"

python3 tools/analyze_phase66_bounded_autonomous_rerun_from_inner_ingress.py --send-inner-ingress-goal \
  --goal-timeout-sec "$GOAL_TIMEOUT_SEC" \
  --output "$ARTIFACT_DIR/${RUN_ID}_inner_ingress_navigate_to_pose_action_result.json" \
  > "$ARTIFACT_DIR/${RUN_ID}_inner_ingress_goal_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_inner_ingress_goal_stderr.log" || true

python3 tools/record_explorer_state_series.py --topic /maze/goal_events \
  --output "$ARTIFACT_DIR/${RUN_ID}_goal_events.jsonl" \
  --max-samples 80 \
  --min-samples 0 \
  --timeout-sec "$EXPLORER_OBSERVE_SEC" \
  --stop-on-terminal > "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_goal_events_recorder_stderr.log" &
GOAL_EVENTS_PID="$!"
PIDS+=("$GOAL_EVENTS_PID")

python3 tools/record_explorer_state_series.py --topic /maze/explorer_state \
  --output "$ARTIFACT_DIR/${RUN_ID}_explorer_state.jsonl" \
  --max-samples 260 \
  --min-samples 1 \
  --timeout-sec "$EXPLORER_OBSERVE_SEC" \
  --stop-on-terminal > "$ARTIFACT_DIR/${RUN_ID}_state_recorder_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_state_recorder_stderr.log" &
STATE_PID="$!"
PIDS+=("$STATE_PID")

python3 tools/analyze_phase66_bounded_autonomous_rerun_from_inner_ingress.py --record-runtime \
  --timeout-sec "$RUNTIME_RECORD_TIMEOUT_SEC" \
  --output "$ARTIFACT_DIR/${RUN_ID}_runtime_recorder_summary.json" \
  --timeline-output "$ARTIFACT_DIR/${RUN_ID}_runtime_timeline.jsonl" \
  --controller-dynamics-output "$ARTIFACT_DIR/${RUN_ID}_controller_dynamics.jsonl" \
  --nav2-feedback-output "$ARTIFACT_DIR/${RUN_ID}_nav2_feedback.jsonl" \
  --local-costmap-samples-output "$ARTIFACT_DIR/${RUN_ID}_local_costmap_samples.jsonl" \
  --global-plan-samples-output "$ARTIFACT_DIR/${RUN_ID}_global_plan_samples.jsonl" \
  --collision-monitor-output "$ARTIFACT_DIR/${RUN_ID}_collision_monitor_state.jsonl" \
  > "$ARTIFACT_DIR/${RUN_ID}_runtime_recorder_stdout.json" 2> "$ARTIFACT_DIR/${RUN_ID}_runtime_recorder_stderr.log" &
RUNTIME_RECORDER_PID="$!"
PIDS+=("$RUNTIME_RECORDER_PID")

sleep 2
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
  -p max_goals:="$PHASE67_MAX_GOALS" \
  -p near_exit_fallback_enabled:=false \
  -p startup_warmup_no_dispatch:=false \
  -p topology_consistency_enabled:=true \
  -p topology_consistency_required_no_candidate_frames:=2 \
  -p topology_consistency_window_sec:=4.0 \
  -p post_ingress_single_open_exception_enabled:=true \
  > "$ARTIFACT_DIR/${RUN_ID}_maze_explorer_stdout.log" 2> "$ARTIFACT_DIR/${RUN_ID}_maze_explorer_stderr.log" &
EXPLORER_PID="$!"
PIDS+=("$EXPLORER_PID")
echo "$EXPLORER_PID" > "$ARTIFACT_DIR/${RUN_ID}_maze_explorer.pid"

# Bounded observation window; the visible launch remains alive afterward unless cleanup is requested.
sleep "$EXPLORER_OBSERVE_SEC"
{
  echo "PHASE67_READY_FOR_HUMAN_OBSERVATION"
  echo "timestamp=$(date -Is)"
  echo "marker_topic=/phase67/goal1_timeout_visual_markers"
  echo "artifact_dir=$ARTIFACT_DIR"
  echo "rviz_fixed_frame=map"
  echo "suggested_screenshot_moments=inner ingress, Goal 1 dispatch, turning near target, timeout terminal pose, local costmap + footprint + front wedge overlay"
  ros2 node list || true
  ros2 topic list || true
} > "$READY_MARKER" 2>&1

cat "$READY_MARKER"

if [[ "$CLEANUP_ON_EXIT" == "1" ]]; then
  cleanup
fi
