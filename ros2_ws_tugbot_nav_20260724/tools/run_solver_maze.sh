#!/usr/bin/env bash
# Trémaux autonomous solver maze run (explorer_type:=tremaux, guided_corridor_mode:=false):
# the robot must discover the maze via SLAM + the Trémaux brain and find the exit
# on its own (reactive pilot for locomotion). Same process/SHM hygiene as the
# 20260522 run_dfs_maze.sh.
#
# Usage: tools/run_solver_maze.sh [MAX_SECONDS] [HEADLESS] [USE_RVIZ] [MAX_GOALS]
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

MAX_SECONDS="${1:-900}"
HEADLESS="${2:-true}"
USE_RVIZ="${3:-false}"
MAX_GOALS="${4:-400}"
STAMP="$(date +%Y%m%d_%H%M%S)"
ART="log/solver_run_${STAMP}"
mkdir -p "$ART"

kill_all_sim() {
    for pat in tugbot_maze_explore "ros2 launch" "gz sim" "ruby.*gz" parameter_bridge \
               bridge_node slam_toolbox controller_server planner_server bt_navigator \
               behavior_server smoother_server route_server waypoint_follower \
               velocity_smoother collision_monitor lifecycle_manager map_server amcl \
               maze_explorer maze_solver frontier_explorer maze_goal_monitor robot_state_publisher \
               scan_slice_projector \
               static_transform_publisher component_container rviz; do
        pkill -9 -f "$pat" 2>/dev/null
    done
}

kill_all_sim
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
export ROS_DOMAIN_ID=$(( ($(date +%s) % 90) + 1 ))
echo "[SOLVER] artifact dir: $ART max=${MAX_SECONDS}s headless=${HEADLESS} rviz=${USE_RVIZ} max_goals=${MAX_GOALS} DOMAIN=$ROS_DOMAIN_ID"

ros2 launch tugbot_bringup tugbot_maze_explore.launch.py \
    headless:="${HEADLESS}" \
    use_rviz:="${USE_RVIZ}" \
    explorer_type:=tremaux \
    guided_corridor_mode:=false \
    entry_direct_enabled:=true \
    entry_direct_distance_m:=2.0 \
    max_goals:="${MAX_GOALS}" \
    > "$ART/launch.log" 2>&1 &
LAUNCH_PID=$!
echo "[SOLVER] launch PID=$LAUNCH_PID DOMAIN=$ROS_DOMAIN_ID" | tee "$ART/run_meta.txt"

END=$(( $(date +%s) + MAX_SECONDS ))
RESULT="TIMEOUT"
while [ "$(date +%s)" -lt "$END" ]; do
    sleep 10
    if grep -qa "EXIT_REACHED" "$ART/launch.log" 2>/dev/null; then RESULT="EXIT_REACHED"; break; fi
    if grep -qa "FAILED_EXHAUSTED" "$ART/launch.log" 2>/dev/null; then RESULT="EXHAUSTED"; break; fi
    if grep -qa "open_and_lock_file failed" "$ART/launch.log" 2>/dev/null; then RESULT="DDS_SHM_FAIL"; break; fi
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then RESULT="LAUNCH_DIED"; break; fi
done

echo "[SOLVER] result=$RESULT" | tee -a "$ART/run_meta.txt"
echo "$RESULT" > "$ART/result.txt"
# Progress summary: goals dispatched (kind), reactive/back-out/nudge events, closest approach.
grep -aoE "explore goal #[0-9]+ kind=[a-z_]+" "$ART/launch.log" | tail -25 > "$ART/goals.txt" 2>/dev/null
grep -aE "EXIT_REACHED|MAZE_EXIT_REACHED|FAILED_EXHAUSTED|REACTIVE DRIVE|BACK_OUT|nudging forward|false DEAD_END|reroute" "$ART/launch.log" | tail -50 > "$ART/solver_tail.txt" 2>/dev/null

kill -INT "$LAUNCH_PID" 2>/dev/null
sleep 5
kill_all_sim
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
echo "[SOLVER] done result=$RESULT artifact=$ART"
echo "ARTIFACT_DIR=$ART"
