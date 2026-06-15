#!/usr/bin/env bash
set -eo pipefail

# Phase81 Goal2 forward-open machine evidence capture.
# Evidence-only: reuse the held Phase79 scene if available, otherwise the caller
# may run Phase79 first.  This script records raw scan/local-costmap/footprint/
# TF/odom and runs the Phase81 analyzer.  It does not start maze_explorer and
# does not tune Nav2/controller/geometry parameters.

RUN_ID="phase81_goal2_forward_open_machine_evidence_capture"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ARTIFACT_DIR="$ROOT/log/$RUN_ID"
CAPTURE_JSON="$ARTIFACT_DIR/${RUN_ID}_raw_capture.json"
ANALYSIS_JSON="$ARTIFACT_DIR/${RUN_ID}_analysis.json"
MIN_SUMMARY="$ARTIFACT_DIR/${RUN_ID}_minimal_field_summary.md"
PREFLIGHT="$ARTIFACT_DIR/${RUN_ID}_preflight.txt"
GRAPH="$ARTIFACT_DIR/${RUN_ID}_ros_graph_snapshot.txt"
DURATION_SEC="${PHASE81_CAPTURE_DURATION_SEC:-6}"

mkdir -p "$ARTIFACT_DIR"
source /opt/ros/jazzy/setup.bash
if [[ -f "$ROOT/install/setup.bash" ]]; then
  source "$ROOT/install/setup.bash"
fi

{
  echo "run_id=$RUN_ID"
  echo "phase=Phase81 Goal2 forward-open machine evidence capture"
  echo "mode=evidence-only raw topic capture"
  echo "duration_sec=$DURATION_SEC"
  echo "guardrails=No maze_explorer strategy changed; No branch scoring changed; No centerline gate changed; No directional readiness override changed; No fallback/terminal acceptance changed; No Nav2/MPPI/controller tuning; no inflation/robot_radius/clearance_radius_m/map threshold tuning; No autonomous exploration success claimed; No exit success claimed; Phase82 not entered."
  echo "strategy_config_diff_begin"
  git diff -- src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_bringup/launch src/tugbot_navigation/config | tee "$ARTIFACT_DIR/${RUN_ID}_strategy_config_diff.txt" || true
  echo "strategy_config_diff_end"
  echo "nav2_config_diff_begin"
  git diff -- src/tugbot_navigation/config | tee "$ARTIFACT_DIR/${RUN_ID}_nav2_config_diff.txt" || true
  echo "nav2_config_diff_end"
} > "$PREFLIGHT"

{
  echo "captured_at=$(date -Is)"
  ros2 node list || true
  ros2 topic list || true
} > "$GRAPH" 2>&1

python3 tools/record_phase81_goal2_forward_open_machine_evidence.py \
  --duration-sec "$DURATION_SEC" \
  --output-json "$CAPTURE_JSON"

PYTHONDONTWRITEBYTECODE=1 python3 tools/analyze_phase81_goal2_forward_open_machine_evidence.py \
  --capture-json "$CAPTURE_JSON" \
  --output-json "$ANALYSIS_JSON" \
  --minimal-summary "$MIN_SUMMARY"

# ANALYSIS_JSON and MIN_SUMMARY already use the stable filenames expected by tests/reports.
echo "Phase81 capture complete: $ANALYSIS_JSON"
