#!/usr/bin/env bash
set -euo pipefail

RUN_ID="phase47_manual_nav2_goal_smoke_after_lifecycle_active"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ARTIFACT_DIR="log/${RUN_ID}"
mkdir -p "$ARTIFACT_DIR"

WORLD_SDF="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
SLAM_PARAMS="$ROOT/src/tugbot_navigation/config/slam_toolbox_params.yaml"
NAV2_PARAMS="$ROOT/src/tugbot_navigation/config/nav2_slam_params.yaml"
READY_TIMEOUT_SEC="${PHASE47_READY_TIMEOUT_SEC:-150}"
KEEP_ALIVE_FOR_HUMAN="${PHASE47_KEEP_ALIVE_FOR_HUMAN:-1}"
POLL_SEC="${PHASE47_POLL_SEC:-5}"

LAUNCH_PID=""

setup_ros() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  if [[ -f "$ROOT/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$ROOT/install/setup.bash"
  fi
  set -u
}

cleanup() {
  local status=$?
  if [[ "$KEEP_ALIVE_FOR_HUMAN" == "1" && "$status" == "0" ]]; then
    return 0
  fi
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -TERM "$LAUNCH_PID" 2>/dev/null || true
    sleep 3
    kill -KILL "$LAUNCH_PID" 2>/dev/null || true
  fi
  (pgrep -af '[r]os2 launch|[g]z sim|[r]viz2|[s]lam_toolbox|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]oute_server|[c]ollision_monitor|[d]ocking_server|[o]pennav_docking|[l]ifecycle_manager|[r]os_gz_bridge|[p]arameter_bridge|[s]tatic_transform_publisher|[m]aze_explorer|[m]aze_goal_monitor|[f]rontier_explorer' || true) \
    | grep -v 'hermes-snap' \
    | grep -v 'pgrep -af' \
    > "$ARTIFACT_DIR/cleanup_processes_after.txt" || true
}
trap cleanup EXIT INT TERM

setup_ros

if [[ ! -f "$WORLD_SDF" ]]; then
  echo "Missing active scaled2x world: $WORLD_SDF" >&2
  exit 2
fi

: > "$ARTIFACT_DIR/launch.log"
: > "$ARTIFACT_DIR/lifecycle_readiness.txt"
: > "$ARTIFACT_DIR/cleanup_processes_after.txt"
cat > "$ARTIFACT_DIR/manual_goal_observation_notes.md" <<'NOTES'
# Phase47 manual RViz Nav2 Goal observation notes

Status: RUN_ACTIVE_PENDING_HUMAN_MANUAL_GOAL_ACCEPTANCE

Human checklist after the readiness marker is printed:
1. In RViz, confirm fixed frame/map, RobotModel, LaserScan, local/global costmaps, and footprint are plausible.
2. Send one nearby in-corridor Nav2 Goal from the entrance area.
3. Observe whether /plan appears and whether the robot moves/responds.
4. Optionally send one short corridor/turn goal if the first goal is plausible.
5. Report observation back for Phase47 acceptance/failure classification.

Guardrail: this phase does not run maze_explorer and does not claim autonomous exploration success.
NOTES

ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py \
  world_sdf:="$WORLD_SDF" \
  slam_params_file:="$SLAM_PARAMS" \
  params_file:="$NAV2_PARAMS" \
  headless:=false \
  use_rviz:=true \
  use_sim_time:=true \
  autostart:=true \
  > "$ARTIFACT_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$ARTIFACT_DIR/launch.pid"

is_lifecycle_active() {
  local node="$1"
  ros2 lifecycle get "$node" 2>/dev/null | tee -a "$ARTIFACT_DIR/lifecycle_readiness.txt" | grep -q 'active \[3\]'
}

capture_graph() {
  ros2 node list > "$ARTIFACT_DIR/nodes.txt" 2>&1 || true
  ros2 topic list --include-hidden-topics > "$ARTIFACT_DIR/topics.txt" 2>&1 || true
  ros2 action list > "$ARTIFACT_DIR/actions.txt" 2>&1 || true
  ros2 action info /navigate_to_pose > "$ARTIFACT_DIR/navigate_to_pose_action_info.txt" 2>&1 || true
  ros2 topic info /goal_pose > "$ARTIFACT_DIR/goal_pose_topic_info.txt" 2>&1 || true
  {
    echo "# lifecycle readiness $(date -Is)"
    ros2 lifecycle get /controller_server 2>&1 || true
    ros2 lifecycle get /planner_server 2>&1 || true
    ros2 lifecycle get /bt_navigator 2>&1 || true
  } >> "$ARTIFACT_DIR/lifecycle_readiness.txt"
}

readiness_json() {
  local classification="$1"
  local reason="$2"
  python3 - "$classification" "$reason" "$ARTIFACT_DIR" "$LAUNCH_PID" <<'PY'
import json, pathlib, sys, time
classification, reason, artifact_dir, launch_pid = sys.argv[1:5]
art = pathlib.Path(artifact_dir)
summary = {
    "run_id": "phase47_manual_nav2_goal_smoke_after_lifecycle_active",
    "classification": classification,
    "reason": reason,
    "status": "RUN_ACTIVE_PENDING_HUMAN_MANUAL_GOAL_ACCEPTANCE" if classification == "READY_FOR_MANUAL_NAV2_GOAL" else classification,
    "launch_pid": int(launch_pid),
    "guardrails": {
        "no_maze_explorer": True,
        "no_nav2_param_tuning": True,
        "manual_rviz_goal_only": True,
        "no_autonomous_success_claim": True,
    },
    "required_gates": [
        "/controller_server active [3]",
        "/planner_server active [3]",
        "/bt_navigator active [3]",
        "/navigate_to_pose Action servers: 1",
        "/goal_pose Subscription count: 1",
    ],
    "artifacts": sorted(p.name for p in art.iterdir()),
    "timestamp_epoch": time.time(),
}
(art / "phase47_manual_nav2_goal_smoke_readiness.json").write_text(json.dumps(summary, indent=2, sort_keys=True))
print(json.dumps(summary, ensure_ascii=False))
PY
}

start_epoch=$(date +%s)
ready="0"
while true; do
  capture_graph
  : > "$ARTIFACT_DIR/lifecycle_readiness.txt.tmp"
  controller_ok="0"; planner_ok="0"; bt_ok="0"; action_ok="0"; goal_pose_ok="0"

  if ros2 lifecycle get /controller_server 2>&1 | tee -a "$ARTIFACT_DIR/lifecycle_readiness.txt.tmp" | grep -q 'active \[3\]'; then controller_ok="1"; fi
  if ros2 lifecycle get /planner_server 2>&1 | tee -a "$ARTIFACT_DIR/lifecycle_readiness.txt.tmp" | grep -q 'active \[3\]'; then planner_ok="1"; fi
  if ros2 lifecycle get /bt_navigator 2>&1 | tee -a "$ARTIFACT_DIR/lifecycle_readiness.txt.tmp" | grep -q 'active \[3\]'; then bt_ok="1"; fi
  if grep -q 'Action servers: 1' "$ARTIFACT_DIR/navigate_to_pose_action_info.txt"; then action_ok="1"; fi
  if grep -q 'Subscription count: 1' "$ARTIFACT_DIR/goal_pose_topic_info.txt"; then goal_pose_ok="1"; fi

  {
    echo "# gate snapshot $(date -Is)"
    cat "$ARTIFACT_DIR/lifecycle_readiness.txt.tmp"
    echo "controller_ok=$controller_ok"
    echo "planner_ok=$planner_ok"
    echo "bt_navigator_ok=$bt_ok"
    echo "navigate_to_pose_action_servers_1=$action_ok"
    echo "goal_pose_subscription_count_1=$goal_pose_ok"
  } >> "$ARTIFACT_DIR/lifecycle_readiness.txt"

  if [[ "$controller_ok" == "1" && "$planner_ok" == "1" && "$bt_ok" == "1" && "$action_ok" == "1" && "$goal_pose_ok" == "1" ]]; then
    ready="1"
    break
  fi

  now=$(date +%s)
  if (( now - start_epoch >= READY_TIMEOUT_SEC )); then
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    break
  fi
  sleep "$POLL_SEC"
done

if [[ "$ready" != "1" ]]; then
  readiness_json "READINESS_TIMEOUT" "required lifecycle/action/topic gates were not met before timeout"
  exit 1
fi

readiness_json "READY_FOR_MANUAL_NAV2_GOAL" "all required lifecycle/action/topic gates passed before manual RViz Nav2 Goal"
echo "PHASE47_READY_FOR_MANUAL_NAV2_GOAL"
echo "RUN_ACTIVE_PENDING_HUMAN_MANUAL_GOAL_ACCEPTANCE"
echo "请在 RViz 中发送短距离 Nav2 Goal；完成后把人工观察结果告诉我。"

if [[ "$KEEP_ALIVE_FOR_HUMAN" == "1" ]]; then
  wait "$LAUNCH_PID"
else
  exit 0
fi
