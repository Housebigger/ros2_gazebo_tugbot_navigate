#!/usr/bin/env bash
# Guided Corridor Navigation (GCN) maze run: follow the pre-computed BFS
# corridor route to the exit. Headless, logs to a timestamped artifact dir.
#
# Usage: tools/run_gcn_maze.sh [MAX_SECONDS] [HEADLESS true|false]
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

MAX_SECONDS="${1:-600}"
HEADLESS="${2:-true}"
USE_RVIZ="${3:-false}"
STAMP="$(date +%Y%m%d_%H%M%S)"
ART="log/gcn_run_${STAMP}"
mkdir -p "$ART"

kill_all_sim() {
    # Force-kill every node this stack spawns. Use -9 and broad patterns so
    # nothing lingers holding Fast-DDS shared-memory port locks (which poison
    # the next run's DDS transport -> Nav2 never activates).
    for pat in tugbot_maze_explore "ros2 launch" "gz sim" "ruby.*gz" parameter_bridge \
               bridge_node slam_toolbox controller_server planner_server bt_navigator \
               behavior_server smoother_server route_server waypoint_follower \
               velocity_smoother collision_monitor lifecycle_manager map_server amcl \
               maze_explorer frontier_explorer maze_goal_monitor robot_state_publisher \
               static_transform_publisher component_container rviz; do
        pkill -9 -f "$pat" 2>/dev/null
    done
}

# ── Pre-run hygiene: kill any stragglers + purge stale Fast-DDS SHM ──
kill_all_sim
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
# Unique domain per run so the SHM port (derived from domain id) cannot
# collide with a stale lock left by a crashed prior process.
export ROS_DOMAIN_ID=$(( ($(date +%s) % 90) + 1 ))
echo "[GCN] artifact dir: $ART  max=${MAX_SECONDS}s headless=${HEADLESS} ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

ros2 launch tugbot_bringup tugbot_maze_explore.launch.py \
    headless:="${HEADLESS}" \
    use_rviz:="${USE_RVIZ}" \
    explorer_type:=maze_dfs \
    guided_corridor_mode:=true \
    entry_direct_enabled:=true \
    entry_direct_distance_m:=1.5 \
    max_goals:=400 \
    > "$ART/launch.log" 2>&1 &
LAUNCH_PID=$!
echo "[GCN] launch PID=$LAUNCH_PID ROS_DOMAIN_ID=$ROS_DOMAIN_ID" | tee "$ART/run_meta.txt"
echo "$LAUNCH_PID" > "$ART/launch.pid"

# Monitor loop: tail GCN progress + terminal state from the launch log.
END=$(( $(date +%s) + MAX_SECONDS ))
RESULT="TIMEOUT"
while [ "$(date +%s)" -lt "$END" ]; do
    sleep 10
    if grep -qa "EXIT_REACHED" "$ART/launch.log" 2>/dev/null; then
        RESULT="EXIT_REACHED"; break
    fi
    if grep -qa "FAILED_EXHAUSTED\|marking exhausted\|cannot proceed" "$ART/launch.log" 2>/dev/null; then
        RESULT="EXHAUSTED"; break
    fi
    # Detect DDS-transport poisoning early so we don't waste the whole budget.
    if grep -qa "open_and_lock_file failed" "$ART/launch.log" 2>/dev/null; then
        RESULT="DDS_SHM_FAIL"; break
    fi
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        RESULT="LAUNCH_DIED"; break
    fi
done

echo "[GCN] result=$RESULT" | tee -a "$ART/run_meta.txt"
grep -a "GCN:" "$ART/launch.log" | tail -40 > "$ART/gcn_tail.txt" 2>/dev/null
echo "$RESULT" > "$ART/result.txt"

# ── Shutdown + post-run hygiene ──
kill -INT "$LAUNCH_PID" 2>/dev/null
sleep 5
kill_all_sim
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
echo "[GCN] done. artifact dir: $ART"
echo "ARTIFACT_DIR=$ART"
