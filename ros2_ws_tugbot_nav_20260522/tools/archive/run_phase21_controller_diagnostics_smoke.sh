#!/usr/bin/env bash
set -euo pipefail

RUN_ID="${1:-phase21_run1}"
RUN_ID_PATTERN='(phase(21|22|23|24|24c|25a|25b|25d|25e)_run[0-9]+|candidate_baseline_run[0-9]+|phase26a_fingerprint_smoke|phase26a_candidate_fingerprint_smoke|phase26b_baseline_run[0-9]+|phase26b_candidate_run[0-9]+|phase26e_branch_diagnostics_smoke|phase26g_baseline_run[0-9]+|phase26g_candidate_run[0-9]+|phase26l_baseline_run[0-9]+|phase26l_candidate_run[0-9]+|phase26p_baseline_diag_run[0-9]+|phase26p_candidate_diag_run[0-9]+|phase26r_baseline_summary_run[0-9]+|phase26r_candidate_summary_run[0-9]+|phase26v_baseline_geometry_run[0-9]+|phase26v_candidate_geometry_run[0-9]+)'
if [[ ! "$RUN_ID" =~ ^${RUN_ID_PATTERN}$ ]]; then
  echo "usage: $0 phase21_runN|phase22_runN|phase23_runN|phase24_runN|phase24c_runN|phase25a_runN|phase25b_runN|phase25d_runN|phase25e_runN|candidate_baseline_runN|phase26a_fingerprint_smoke|phase26a_candidate_fingerprint_smoke|phase26b_baseline_runN|phase26b_candidate_runN|phase26e_branch_diagnostics_smoke|phase26g_baseline_runN|phase26g_candidate_runN|phase26l_baseline_runN|phase26l_candidate_runN|phase26p_baseline_diag_runN|phase26p_candidate_diag_runN|phase26r_baseline_summary_runN|phase26r_candidate_summary_runN" >&2
  exit 2
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

LOG_DIR="log"
mkdir -p "$LOG_DIR"

LAUNCH_LOG="$LOG_DIR/${RUN_ID}_launch.log"
GOAL_EVENTS="$LOG_DIR/${RUN_ID}_goal_events.jsonl"
EXPLORER_STATE="$LOG_DIR/${RUN_ID}_explorer_state.jsonl"
CONTROLLER_DYNAMICS="$LOG_DIR/${RUN_ID}_controller_dynamics.jsonl"
NAV2_ANALYSIS="$LOG_DIR/${RUN_ID}_goal_nav2_analysis.json"
GEOMETRY_SUMMARY="$LOG_DIR/${RUN_ID}_geometry_nav2_summary.json"
LOCAL_COST_SUMMARY="$LOG_DIR/${RUN_ID}_goal_event_cost_summary.json"
CONTROLLER_SUMMARY="$LOG_DIR/${RUN_ID}_goal_controller_dynamics.json"
POST_RECOVERY_SNAPSHOTS="$LOG_DIR/${RUN_ID}_post_recovery_snapshots.jsonl"
POST_RECOVERY_SUMMARY="$LOG_DIR/${RUN_ID}_post_recovery_snapshots_summary.json"
PARAMS_FINGERPRINT="$LOG_DIR/${RUN_ID}_params_fingerprint.json"
RUNTIME_PARAMS_DIR="$LOG_DIR/${RUN_ID}_runtime_params"
MAX_GOALS="${PHASE_RUN_MAX_GOALS:-12}"
GOAL_EVENTS_MAX_SAMPLES="${PHASE_RUN_GOAL_EVENTS_MAX_SAMPLES:-180}"
STATE_MAX_SAMPLES="${PHASE_RUN_STATE_MAX_SAMPLES:-420}"
RUN_TIMEOUT_SEC="${PHASE_RUN_TIMEOUT_SEC:-400}"
SNAPSHOT_TIMEOUT_SEC="${PHASE_RUN_SNAPSHOT_TIMEOUT_SEC:-420}"
FAILURE_WINDOWS="$LOG_DIR/${RUN_ID}_failure_windows.json"
TIMEOUT_SUBTYPES="$LOG_DIR/${RUN_ID}_timeout_subtypes.json"
POST_RECOVERY_ENRICHED="$LOG_DIR/${RUN_ID}_post_recovery_enriched.json"
PHASE26P_MPPI_EVIDENCE="$LOG_DIR/${RUN_ID}_mppi_evidence_summary.jsonl"
PHASE26P_ANALYSIS="$LOG_DIR/${RUN_ID}_phase26p_mppi_evidence_analysis.json"
PHASE26Q_CAPABILITY_AUDIT="$LOG_DIR/${RUN_ID}_phase26q_mppi_debug_capabilities.json"
PHASE26R_COVERAGE="$LOG_DIR/${RUN_ID}_phase26r_summary_evidence_coverage.json"
PHASE26V_JOIN="$LOG_DIR/${RUN_ID}_phase26v_sampled_path_local_cost_join.json"

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

  # Robust cleanup for orphaned children observed after Phase 20. Keep this
  # analysis-only wrapper isolated by matching the project launch/runtime names.
  pkill -TERM -f 'ros2 launch tugbot_bringup tugbot_maze_explore.launch.py' || true
  pkill -TERM -f 'tee log/(phase(21|22|23|24|24c|25a|25b|25d|25e|26b_baseline|26b_candidate|26g_baseline|26g_candidate|26l_baseline|26l_candidate)_run[0-9]+|candidate_baseline_run[0-9]+|phase26e_branch_diagnostics_smoke)_launch.log' || true
  pkill -TERM -f 'tools/record_explorer_state_series.py.*(phase(21|22|23|24|24c|25a|25b|25d|25e|26b_baseline|26b_candidate|26g_baseline|26g_candidate|26l_baseline|26l_candidate)_run[0-9]+|candidate_baseline_run[0-9]+|phase26e_branch_diagnostics_smoke)' || true
  pkill -TERM -f 'tools/record_controller_dynamics.py.*(phase(21|22|23|24|24c|25a|25b|25d|25e|26b_baseline|26b_candidate|26g_baseline|26g_candidate|26l_baseline|26l_candidate)_run[0-9]+|candidate_baseline_run[0-9]+|phase26e_branch_diagnostics_smoke)' || true
  pkill -TERM -f 'tools/record_post_recovery_snapshots.py.*(phase(21|22|23|24|24c|25a|25b|25d|25e|26b_baseline|26b_candidate|26g_baseline|26g_candidate|26l_baseline|26l_candidate)_run[0-9]+|candidate_baseline_run[0-9]+|phase26e_branch_diagnostics_smoke)' || true
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

rm -f \
  "$LAUNCH_LOG" \
  "$GOAL_EVENTS" \
  "$EXPLORER_STATE" \
  "$CONTROLLER_DYNAMICS" \
  "$NAV2_ANALYSIS" \
  "$GEOMETRY_SUMMARY" \
  "$LOCAL_COST_SUMMARY" \
  "$CONTROLLER_SUMMARY" \
  "$POST_RECOVERY_SNAPSHOTS" \
  "$POST_RECOVERY_SUMMARY" \
  "$PARAMS_FINGERPRINT" \
  "$FAILURE_WINDOWS" \
  "$TIMEOUT_SUBTYPES" \
  "$POST_RECOVERY_ENRICHED" \
  "$PHASE26P_MPPI_EVIDENCE" \
  "$PHASE26P_ANALYSIS" \
  "$PHASE26Q_CAPABILITY_AUDIT" \
  "$PHASE26R_COVERAGE" \
  "$PHASE26V_JOIN"
rm -rf "$RUNTIME_PARAMS_DIR"

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

python3 tools/record_controller_dynamics.py \
  --odom-topic /odom \
  --cmd-topic cmd_vel_nav \
  --output "$CONTROLLER_DYNAMICS" \
  --max-samples 40000 \
  --timeout-sec "$RUN_TIMEOUT_SEC" &
PIDS+=("$!")
CONTROLLER_PID="$!"

python3 tools/record_post_recovery_snapshots.py \
  --local-costmap-topic /local_costmap/costmap \
  --path-topic /plan \
  --odom-topic /odom \
  --cmd-topic cmd_vel_nav \
  --goal-events-topic /maze/goal_events \
  --output "$POST_RECOVERY_SNAPSHOTS" \
  --max-high-cost-points 40 \
  --timeout-sec "$SNAPSHOT_TIMEOUT_SEC" &
PIDS+=("$!")
POST_RECOVERY_PID="$!"

python3 tools/record_phase26p_mppi_evidence.py \
  --output "$PHASE26P_MPPI_EVIDENCE" \
  --topic-regex 'critics_stats|optimal_trajectory|transformed_global_plan|trajectories|trajectory' \
  --raw-topics-regex '^$' \
  --sample-points 30 \
  --max-samples 50000 \
  --timeout-sec "$SNAPSHOT_TIMEOUT_SEC" &
PIDS+=("$!")
PHASE26P_MPPI_PID="$!"

PHASE25A_PROFILE=false
if [[ "$RUN_ID" =~ ^phase25a_run[0-9]+$ ]]; then
  PHASE25A_PROFILE=true
fi
PHASE25B_PROFILE=false
if [[ "$RUN_ID" =~ ^phase25b_run[0-9]+$ ]]; then
  PHASE25B_PROFILE=true
fi
PHASE25D_PROFILE=false
if [[ "$RUN_ID" =~ ^phase25d_run[0-9]+$ ]]; then
  PHASE25D_PROFILE=true
fi
PHASE25E_PROFILE=false
if [[ "$RUN_ID" =~ ^phase25e_run[0-9]+$ ]]; then
  PHASE25E_PROFILE=true
fi
CANDIDATE_COSTCRITIC_275_PROFILE=false
if [[ "$RUN_ID" =~ ^candidate_baseline_run[0-9]+$ ]]; then
  CANDIDATE_COSTCRITIC_275_PROFILE=true
fi
PHASE26A_CANDIDATE_FINGERPRINT_PROFILE=false
if [[ "$RUN_ID" == "phase26a_candidate_fingerprint_smoke" ]]; then
  PHASE26A_CANDIDATE_FINGERPRINT_PROFILE=true
  CANDIDATE_COSTCRITIC_275_PROFILE=true
fi
PHASE26B_CANDIDATE_PROFILE=false
if [[ "$RUN_ID" =~ ^phase26b_candidate_run[0-9]+$ ]]; then
  PHASE26B_CANDIDATE_PROFILE=true
  CANDIDATE_COSTCRITIC_275_PROFILE=true
fi
PHASE26G_CANDIDATE_PROFILE=false
if [[ "$RUN_ID" =~ ^phase26g_candidate_run[0-9]+$ ]]; then
  PHASE26G_CANDIDATE_PROFILE=true
  CANDIDATE_COSTCRITIC_275_PROFILE=true
fi
PHASE26L_BASELINE_PROFILE=false
if [[ "$RUN_ID" =~ ^phase26l_baseline_run[0-9]+$ ]]; then
  PHASE26L_BASELINE_PROFILE=true
fi
PHASE26L_CANDIDATE_PROFILE=false
if [[ "$RUN_ID" =~ ^phase26l_candidate_run[0-9]+$ ]]; then
  PHASE26L_CANDIDATE_PROFILE=true
  CANDIDATE_COSTCRITIC_275_PROFILE=true
fi
PHASE26P_BASELINE_DIAG_PROFILE=false
if [[ "$RUN_ID" =~ ^phase26p_baseline_diag_run[0-9]+$ ]]; then
  PHASE26P_BASELINE_DIAG_PROFILE=true
fi
PHASE26P_CANDIDATE_DIAG_PROFILE=false
if [[ "$RUN_ID" =~ ^phase26p_candidate_diag_run[0-9]+$ ]]; then
  PHASE26P_CANDIDATE_DIAG_PROFILE=true
fi
PHASE26R_BASELINE_SUMMARY_PROFILE=false
# phase26r_summary_smoke: diagnostics-only alias using the Phase26P MPPI diagnostics profile plus compact evidence coverage checks.
if [[ "$RUN_ID" =~ ^phase26r_baseline_summary_run[0-9]+$ ]]; then
  PHASE26R_BASELINE_SUMMARY_PROFILE=true
  PHASE26P_BASELINE_DIAG_PROFILE=true
fi
PHASE26R_CANDIDATE_SUMMARY_PROFILE=false
if [[ "$RUN_ID" =~ ^phase26r_candidate_summary_run[0-9]+$ ]]; then
  PHASE26R_CANDIDATE_SUMMARY_PROFILE=true
  PHASE26P_CANDIDATE_DIAG_PROFILE=true
fi

PHASE26V_BASELINE_GEOMETRY_PROFILE=false
if [[ "$RUN_ID" =~ ^phase26v_baseline_geometry_run[0-9]+$ ]]; then
  PHASE26V_BASELINE_GEOMETRY_PROFILE=true
  PHASE26R_BASELINE_SUMMARY_PROFILE=true
  PHASE26P_BASELINE_DIAG_PROFILE=true
fi
PHASE26V_CANDIDATE_GEOMETRY_PROFILE=false
if [[ "$RUN_ID" =~ ^phase26v_candidate_geometry_run[0-9]+$ ]]; then
  PHASE26V_CANDIDATE_GEOMETRY_PROFILE=true
  PHASE26R_CANDIDATE_SUMMARY_PROFILE=true
  PHASE26P_CANDIDATE_DIAG_PROFILE=true
fi

NAV2_PARAMS_FILE="src/tugbot_navigation/config/nav2_slam_params.yaml"
SELECTED_PROFILE="canonical_baseline"
RUNTIME_EXPECTED_COST_WEIGHT="3.81"
if [[ "$PHASE25A_PROFILE" == true ]]; then
  NAV2_PARAMS_FILE="src/tugbot_navigation/config/nav2_slam_phase25a_local_cost_relief_params.yaml"
  SELECTED_PROFILE="phase25a_local_cost_relief"
elif [[ "$PHASE25B_PROFILE" == true ]]; then
  NAV2_PARAMS_FILE="src/tugbot_navigation/config/nav2_slam_phase25b_costcritic_relief_params.yaml"
  SELECTED_PROFILE="phase25b_costcritic_relief"
  RUNTIME_EXPECTED_COST_WEIGHT="2.5"
elif [[ "$PHASE25D_PROFILE" == true ]]; then
  NAV2_PARAMS_FILE="src/tugbot_navigation/config/nav2_slam_phase25d_costcritic_mid_params.yaml"
  SELECTED_PROFILE="phase25d_costcritic_mid"
  RUNTIME_EXPECTED_COST_WEIGHT="3.0"
elif [[ "$PHASE25E_PROFILE" == true ]]; then
  NAV2_PARAMS_FILE="src/tugbot_navigation/config/nav2_slam_phase25e_costcritic_compromise_params.yaml"
  SELECTED_PROFILE="phase25e_costcritic_compromise"
  RUNTIME_EXPECTED_COST_WEIGHT="2.75"
elif [[ "$CANDIDATE_COSTCRITIC_275_PROFILE" == true ]]; then
  NAV2_PARAMS_FILE="src/tugbot_navigation/config/nav2_slam_candidate_costcritic_275_params.yaml"
  SELECTED_PROFILE="candidate_costcritic_275"
  RUNTIME_EXPECTED_COST_WEIGHT="2.75"
fi
if [[ "$PHASE26P_BASELINE_DIAG_PROFILE" == true ]]; then
  NAV2_PARAMS_FILE="src/tugbot_navigation/config/nav2_slam_phase26p_mppi_diagnostics_params.yaml"
  SELECTED_PROFILE="phase26p_mppi_diagnostics_baseline"
  RUNTIME_EXPECTED_COST_WEIGHT="3.81"
elif [[ "$PHASE26P_CANDIDATE_DIAG_PROFILE" == true ]]; then
  NAV2_PARAMS_FILE="src/tugbot_navigation/config/nav2_slam_phase26p_candidate_mppi_diagnostics_params.yaml"
  SELECTED_PROFILE="phase26p_mppi_diagnostics_candidate_275"
  RUNTIME_EXPECTED_COST_WEIGHT="2.75"
fi
python3 tools/fingerprint_nav2_params.py \
  --params-file "$NAV2_PARAMS_FILE" \
  --baseline-params-file src/tugbot_navigation/config/nav2_slam_params.yaml \
  --run-id "$RUN_ID" \
  --selected-profile "$SELECTED_PROFILE" \
  --output-json "$PARAMS_FINGERPRINT"

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
  phase25a_local_cost_relief_profile:="$PHASE25A_PROFILE" \
  phase25b_costcritic_relief_profile:="$PHASE25B_PROFILE" \
  phase25d_costcritic_mid_profile:="$PHASE25D_PROFILE" \
  phase25e_costcritic_compromise_profile:="$PHASE25E_PROFILE" \
  candidate_costcritic_275_profile:="$CANDIDATE_COSTCRITIC_275_PROFILE" \
  phase26p_mppi_diagnostics_profile:="$PHASE26P_BASELINE_DIAG_PROFILE" \
  phase26p_candidate_mppi_diagnostics_profile:="$PHASE26P_CANDIDATE_DIAG_PROFILE" \
  2>&1 | tee "$LAUNCH_LOG" &
PIDS+=("$!")
LAUNCH_PID="$!"

(
  set +e
  mkdir -p "$RUNTIME_PARAMS_DIR"
  for _ in $(seq 1 120); do
    python3 tools/dump_controller_runtime_params.py \
      --node /controller_server \
      --output "$RUNTIME_PARAMS_DIR/controller_server.yaml" \
      --summary-json "$RUNTIME_PARAMS_DIR/controller_server_summary.json" \
      --expected-cost-weight "$RUNTIME_EXPECTED_COST_WEIGHT" \
      --timeout-sec 120 \
      --interval-sec 1.0 && exit 0
    sleep 1
  done
  exit 0
) &
PIDS+=("$!")
RUNTIME_DUMP_PID="$!"

# The state recorder is the terminal sentinel. If it exits because max samples
# rather than terminal, downstream summaries still expose the final mode.
wait "$STATE_PID" || true

# Stop launch and recorders after terminal state or state-recorder timeout.
for pid in "$LAUNCH_PID" "$GOAL_EVENTS_PID" "$CONTROLLER_PID" "$POST_RECOVERY_PID" "$PHASE26P_MPPI_PID" "$RUNTIME_DUMP_PID"; do
  if kill -0 "$pid" 2>/dev/null; then
    kill -TERM "$pid" 2>/dev/null || true
  fi
done
sleep 2

python3 tools/analyze_goal_events_with_nav2_log.py \
  --goal-events "$GOAL_EVENTS" \
  --log "$LAUNCH_LOG" \
  --output-json "$NAV2_ANALYSIS"

python3 tools/summarize_goal_geometry_nav2.py \
  --goal-events "$GOAL_EVENTS" \
  --nav2-analysis "$NAV2_ANALYSIS" \
  --output-json "$GEOMETRY_SUMMARY"

python3 tools/summarize_goal_event_local_costs.py \
  --goal-events "$GOAL_EVENTS" \
  --output-json "$LOCAL_COST_SUMMARY"

python3 tools/analyze_goal_controller_dynamics.py \
  --goal-events "$GOAL_EVENTS" \
  --controller-dynamics "$CONTROLLER_DYNAMICS" \
  --nav2-analysis "$NAV2_ANALYSIS" \
  --output-json "$CONTROLLER_SUMMARY"

python3 tools/summarize_post_recovery_snapshots.py \
  --snapshots "$POST_RECOVERY_SNAPSHOTS" \
  --output-json "$POST_RECOVERY_SUMMARY" || true

python3 tools/analyze_failure_windows.py \
  --controller-analysis "$CONTROLLER_SUMMARY" \
  --controller-dynamics "$CONTROLLER_DYNAMICS" \
  --local-cost-summary "$LOCAL_COST_SUMMARY" \
  --nav2-analysis "$NAV2_ANALYSIS" \
  --output-json "$FAILURE_WINDOWS"

python3 tools/analyze_timeout_subtypes.py \
  --failure-windows "$FAILURE_WINDOWS" \
  --output-json "$TIMEOUT_SUBTYPES"

python3 tools/enrich_post_recovery_snapshots.py \
  --snapshots "$POST_RECOVERY_SNAPSHOTS" \
  --nav2-analysis "$NAV2_ANALYSIS" \
  --output-json "$POST_RECOVERY_ENRICHED" || true

if [[ "$PHASE26P_BASELINE_DIAG_PROFILE" == true || "$PHASE26P_CANDIDATE_DIAG_PROFILE" == true ]]; then
  python3 tools/audit_phase26q_mppi_debug_capabilities.py \
    --ros-prefix /opt/ros/jazzy \
    --output-json "$PHASE26Q_CAPABILITY_AUDIT" || true
  python3 tools/analyze_phase26n_goal_timeline.py \
    --log-dir "$LOG_DIR" \
    --case "${RUN_ID}:2" \
    --output-json "$LOG_DIR/${RUN_ID}_phase26p_single_goal_timeline.json" || true
  python3 tools/analyze_phase26p_mppi_evidence.py \
    --timeline-json "$LOG_DIR/${RUN_ID}_phase26p_single_goal_timeline.json" \
    --mppi-evidence "$PHASE26P_MPPI_EVIDENCE" \
    --output-json "$PHASE26P_ANALYSIS" || true
  if [[ "$PHASE26R_BASELINE_SUMMARY_PROFILE" == true || "$PHASE26R_CANDIDATE_SUMMARY_PROFILE" == true ]]; then
    python3 tools/check_phase26r_summary_evidence_coverage.py \
      --timeline-json "$LOG_DIR/${RUN_ID}_phase26p_single_goal_timeline.json" \
      --mppi-evidence "$PHASE26P_MPPI_EVIDENCE" \
      --analysis-json "$PHASE26P_ANALYSIS" \
      --output-json "$PHASE26R_COVERAGE" || true
    python3 tools/analyze_phase26v_sampled_path_local_cost_join.py \
      --log-dir "$LOG_DIR" \
      --run-ids "$RUN_ID" \
      --output-json "$PHASE26V_JOIN" || true
  fi
fi

python3 - <<PY
import json
from pathlib import Path
state_path = Path('$EXPLORER_STATE')
nav_path = Path('$NAV2_ANALYSIS')
controller_path = Path('$CONTROLLER_SUMMARY')
last = {}
if state_path.exists() and state_path.stat().st_size:
    for line in state_path.read_text().splitlines():
        if line.strip():
            row = json.loads(line)
            last = row.get('state', row)
nav = json.loads(nav_path.read_text()).get('summary', {}) if nav_path.exists() else {}
controller = json.loads(controller_path.read_text()).get('summary', {}) if controller_path.exists() else {}
print(json.dumps({
    'run_id': '$RUN_ID',
    'final_mode': last.get('mode'),
    'goal_count': last.get('goal_count'),
    'goal_success_count': last.get('goal_success_count'),
    'goal_failure_count': last.get('goal_failure_count'),
    'timeout_cancel_count': last.get('timeout_cancel_count'),
    'blocked_branch_count': last.get('blocked_branch_count'),
    'blacklisted_goal_count': last.get('blacklisted_goal_count'),
    'exit_distance_m': last.get('exit_distance_m'),
    'nav2_summary': nav,
    'controller_summary': controller,
}, indent=2, sort_keys=True))
PY

echo "[phase21] complete $RUN_ID"
