#!/usr/bin/env bash
# Flood-fill autonomous maze run (explorer_type:=flood_fill):
# the robot drives a flood-fill solver (no Nav2 in the loop)
# and must reach the exit on its own. Same process/SHM hygiene as run_solver_maze.sh.
#
# Usage: tools/run_flood_fill_maze.sh [MAX_SECONDS] [HEADLESS] [USE_RVIZ] [POSE_SOURCE] [SENSE_DEBUG]
#   POSE_SOURCE: online_slam (DEFAULT; self-built-map ICP) | scan_match (fed map; A/B upper bound - KNOWN BROKEN on anymal_c, see spec addendum)
#                (self-built map: perimeter + committed/local-sensed walls, no prior interior map)
#                | odom_locked | slam
#   SENSE_DEBUG: false (default) | true  (log per-cell sensed walls + min LIDAR ranges)
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

MAX_SECONDS="${1:-1500}"
HEADLESS="${2:-true}"
USE_RVIZ="${3:-false}"
POSE_SOURCE="${4:-online_slam}"
SENSE_DEBUG="${5:-false}"
STAMP="$(date +%Y%m%d_%H%M%S)"
ART="log/flood_fill_run_${STAMP}"
mkdir -p "$ART"

kill_all_sim() {
    for pat in tugbot_maze_explore "ros2 launch" "gz sim" "ruby.*gz" parameter_bridge \
               bridge_node slam_toolbox controller_server planner_server bt_navigator \
               behavior_server smoother_server route_server waypoint_follower \
               velocity_smoother collision_monitor lifecycle_manager map_server amcl \
               maze_explorer maze_solver wall_follow_solver frontier_explorer maze_goal_monitor \
               flood_fill_solver gait_animator \
               robot_state_publisher static_transform_publisher component_container rviz; do
        pkill -9 -f "$pat" 2>/dev/null
    done
}

kill_all_sim
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
export ROS_DOMAIN_ID=$(( ($(date +%s) % 90) + 1 ))
echo "[FLOODFILL] artifact dir: $ART max=${MAX_SECONDS}s headless=${HEADLESS} rviz=${USE_RVIZ} pose_source=${POSE_SOURCE} sense_debug=${SENSE_DEBUG} DOMAIN=$ROS_DOMAIN_ID"

ros2 launch tugbot_bringup tugbot_maze_explore.launch.py \
    headless:="${HEADLESS}" use_rviz:="${USE_RVIZ}" \
    explorer_type:=flood_fill entry_direct_distance_m:=2.0 \
    pose_source:="${POSE_SOURCE}" sense_debug:="${SENSE_DEBUG}" \
    junction_log_dir:="$ART" \
    > "$ART/launch.log" 2>&1 &
LAUNCH_PID=$!
echo "[FLOODFILL] launch PID=$LAUNCH_PID DOMAIN=$ROS_DOMAIN_ID" | tee "$ART/run_meta.txt"

END=$(( $(date +%s) + MAX_SECONDS ))
RESULT="TIMEOUT"
while [ "$(date +%s)" -lt "$END" ]; do
    sleep 10
    if grep -qa "EXIT_REACHED" "$ART/launch.log" 2>/dev/null; then RESULT="EXIT_REACHED"; break; fi
    if grep -qa "open_and_lock_file failed" "$ART/launch.log" 2>/dev/null; then RESULT="DDS_SHM_FAIL"; break; fi
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then RESULT="LAUNCH_DIED"; break; fi
done

echo "[FLOODFILL] result=$RESULT" | tee -a "$ART/run_meta.txt"
echo "$RESULT" > "$ART/result.txt"
grep -aE "EXIT_REACHED|HOP_BACKUP|JUNCTION|DIAG|SENSE|flood_fill_solver" "$ART/launch.log" | tail -80 > "$ART/flood_fill_tail.txt" 2>/dev/null

kill -INT "$LAUNCH_PID" 2>/dev/null
sleep 5
kill_all_sim
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
echo "[FLOODFILL] done result=$RESULT artifact=$ART"
echo "ARTIFACT_DIR=$ART"
