#!/usr/bin/env bash
# tugbot-return smoke (Task 5): model/plugin semantics in the REAL maze world,
# headless, no solver in the loop. Checks, in code order:
#   1. maze-world-loaded: the maze world's gz topic namespace is present (the
#      gazebo-only launch defaults to tugbot_empty_world.sdf, which would
#      silently green-light checks 2-5 while starving the gt bridge)
#   2. no-model-/scan-topic: gz has NO /scan topic (stock scan_omni really
#      gone -- the projection chain must be the only /scan source once the
#      full stack runs)
#   3. cloud-split-topic: /lidar/points/points is present on the gz graph
#   4. diffdrive-odom-redirect: /diffdrive/odom_unused exists on the gz graph
#      (odom redirect took)
#   5. live-lidar-frame: /lidar/points delivers a PointCloud2 whose frame_id
#      is the tugbot/base_link/lidar_3d scoped frame (records the LIVE string)
#   6. world-anchored-odom: drive an arc, /odom displacement == gt displacement
#      (tools/tugbot_odom_anchor_check.py; gt readout is EVAL-ONLY)
# Same PRIME/env discipline as the run scripts.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WS_ROOT}"

export DISPLAY="${DISPLAY:-:1}"
export __NV_PRIME_RENDER_OFFLOAD="${__NV_PRIME_RENDER_OFFLOAD:-1}"
export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

kill_sim() {
    # scan_slice_projector is a launch-file Python node that does NOT reliably
    # exit on the parent `ros2 launch` SIGINT -- without it here, every smoke
    # run leaks one more zombie that competes for CPU with the next run's
    # Gazebo/SceneBroadcaster startup, causing snowballing gt-bridge latency.
    for pat in "ros2 launch" "gz sim" "ruby.*gz" parameter_bridge robot_state_publisher \
               static_transform_publisher scan_slice_projector tugbot_odom_anchor_check; do
        pkill -9 -f "$pat" 2>/dev/null
    done
}
kill_sim; sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
export ROS_DOMAIN_ID=$(( ($(date +%s) % 90) + 1 ))

LOGDIR="$(mktemp -d /tmp/tugbot_smoke.XXXXXX)"
echo "[SMOKE] logdir=$LOGDIR DOMAIN=$ROS_DOMAIN_ID"

# The gazebo-only launch DEFAULTS to tugbot_empty_world.sdf (pre-dog relic
# that happens to include model://tugbot) — the full explore stack passes the
# maze world explicitly. Smoke on the wrong world silently green-lights the
# model checks while the gt bridge (pinned to the maze world name) starves.
MAZE_WORLD="${WS_ROOT}/install/tugbot_gazebo/share/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
ros2 launch tugbot_gazebo tugbot_gazebo.launch.py headless:=true \
    world_sdf:="${MAZE_WORLD}" \
    > "$LOGDIR/launch.log" 2>&1 &
LAUNCH_PID=$!
trap 'kill -INT "$LAUNCH_PID" 2>/dev/null; sleep 1; kill_sim' EXIT

for i in $(seq 1 30); do
    kill -0 "$LAUNCH_PID" 2>/dev/null || { echo "[SMOKE] launch died early"; break; }
    gz topic -l 2>/dev/null | grep -q "/world/tugbot_maze_world_20260528_clean_scaled2x/" && break
    sleep 2
done

PASS=0; FAIL=0
check() {  # check <name> <0-for-pass>
    if [ "$2" -eq 0 ]; then echo "[SMOKE] PASS $1"; PASS=$((PASS+1));
    else echo "[SMOKE] FAIL $1"; FAIL=$((FAIL+1)); fi
}

gz topic -l > "$LOGDIR/gz_topics.txt" 2>&1
grep -q "/world/tugbot_maze_world_20260528_clean_scaled2x/" "$LOGDIR/gz_topics.txt"; check "maze-world-loaded" $?
! grep -qx "/scan" "$LOGDIR/gz_topics.txt"; check "no-model-/scan-topic" $?
grep -q "/lidar/points/points" "$LOGDIR/gz_topics.txt"; check "cloud-split-topic" $?
grep -q "/diffdrive/odom_unused" "$LOGDIR/gz_topics.txt"; check "diffdrive-odom-redirect" $?

timeout 15 ros2 topic echo --once /lidar/points --field header.frame_id \
    > "$LOGDIR/cloud_frame.txt" 2>&1
grep -q "tugbot/base_link/lidar_3d" "$LOGDIR/cloud_frame.txt"; check "live-lidar-frame" $?
echo "[SMOKE] live cloud frame: $(tr -d '\n' < "$LOGDIR/cloud_frame.txt")"

timeout 60 python3 "${SCRIPT_DIR}/tugbot_odom_anchor_check.py" \
    | tee "$LOGDIR/anchor.txt"
grep -q "ANCHOR_CHECK PASS" "$LOGDIR/anchor.txt"; check "world-anchored-odom" $?

kill -INT "$LAUNCH_PID" 2>/dev/null; sleep 3; kill_sim
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
echo "[SMOKE] result: PASS=$PASS FAIL=$FAIL (logs: $LOGDIR)"
[ "$FAIL" -eq 0 ]
