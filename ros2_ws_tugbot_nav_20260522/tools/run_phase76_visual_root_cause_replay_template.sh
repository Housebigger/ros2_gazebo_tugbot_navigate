#!/usr/bin/env bash
set -eo pipefail

# Phase76 Obstacle-Triggered Visual Root-Cause Workflow runbook template.
# RViz/Gazebo visual replay packaging only. This script does not tune Nav2/MPPI/controller,
# does not change navigation strategy, and does not claim autonomous exploration success or exit success.

PHASE76_VISUAL_REPLAY_ONLY=1
RUN_ID="${PHASE76_RUN_ID:-phase76_visual_root_cause_case}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EVENT_JSON="${PHASE76_EVENT_JSON:-}"
ARTIFACT_DIR="${PHASE76_ARTIFACT_DIR:-$ROOT/log/phase76_obstacle_triggered_visual_root_cause_workflow/$RUN_ID}"
PHASE76_MAX_OBSERVE_SEC="${PHASE76_MAX_OBSERVE_SEC:-90}"
MARKER_TOPIC="${PHASE76_MARKER_TOPIC:-/phase76/visual_root_cause_markers}"

cd "$ROOT"
mkdir -p "$ARTIFACT_DIR"

if [[ -z "$EVENT_JSON" ]]; then
  cat >&2 <<'USAGE'
Missing PHASE76_EVENT_JSON.

Usage:
  PHASE76_EVENT_JSON=log/<run>/<trigger_event>.json \
  PHASE76_RUN_ID=<case_id> \
  tools/run_phase76_visual_root_cause_replay_template.sh

This is a bounded RViz/Gazebo visual root-cause workflow template only.
USAGE
  exit 2
fi

if [[ ! -f "$EVENT_JSON" ]]; then
  echo "PHASE76_EVENT_JSON does not exist: $EVENT_JSON" >&2
  exit 2
fi

python3 tools/phase76_visual_root_cause_replay_template.py \
  --event-json "$EVENT_JSON" \
  --artifact-dir "$ARTIFACT_DIR" \
  --run-id "$RUN_ID" \
  | tee "$ARTIFACT_DIR/${RUN_ID}_template_stdout.json"

cat > "$ARTIFACT_DIR/${RUN_ID}_ready_for_human_observation.txt" <<EOF
PHASE76_READY_FOR_HUMAN_OBSERVATION
run_id=$RUN_ID
artifact_dir=$ARTIFACT_DIR
marker_topic=$MARKER_TOPIC
max_observe_sec=$PHASE76_MAX_OBSERVE_SEC
rviz_gazebo=RViz/Gazebo visual root-cause observation only
plan=$ARTIFACT_DIR/${RUN_ID}_visual_root_cause_plan.json
marker_specs=$ARTIFACT_DIR/${RUN_ID}_marker_specs.json
screenshot_checklist=$ARTIFACT_DIR/${RUN_ID}_screenshot_checklist.md
human_observation_report_template=$ARTIFACT_DIR/${RUN_ID}_human_observation_report_template.md
replay_command_template=ros2 launch tugbot_bringup phase76_visual_root_cause_overlay.launch.py phase76_plan:=$ARTIFACT_DIR/${RUN_ID}_visual_root_cause_plan.json headless:=false use_rviz:=true marker_topic:=$MARKER_TOPIC
suggested_screenshot_moments=dispatch, near_goal, recovery, timeout, final_pose
scope=visual workflow only; no navigation strategy changed; no autonomous exploration success claim; no exit success claim
EOF

cat "$ARTIFACT_DIR/${RUN_ID}_ready_for_human_observation.txt"
