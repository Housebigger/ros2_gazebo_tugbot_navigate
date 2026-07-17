#!/usr/bin/env bash
# Front-camera live gate: boot the maze sim headless (gz + bridge + TFs via
# tugbot_gazebo.launch.py - no solver), then assert /camera/front/image
# publishes real frames. The dog just stands (JointPositionController holds
# STAND_POSE), which is all an observation camera needs.
#
# NOTE: tugbot_gazebo.launch.py's own world_sdf default is
# tugbot_empty_world.sdf (ground plane only -- no anymal_c robot; the robot
# is only <include>d inside the maze world SDF). So world_sdf is pinned to
# the installed maze world below -- without it there is no robot, hence no
# camera_front sensor, hence no /camera/front/image at all.
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"; cd "$WS"
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=$((RANDOM % 100))
echo "DOMAIN=$ROS_DOMAIN_ID"
MAZE_WORLD="$WS/install/tugbot_gazebo/share/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"

LAUNCH_PID=""

kill_all_sim() {
  # Subset of run_flood_fill_maze.sh's kill_all_sim relevant to this launch:
  # the launch tree here spawns the gz server (a ruby wrapper), the ros_gz
  # bridge_node, and TWO static_transform_publisher nodes (scan_tf +
  # camera_tf) -- all must die, or an interrupted run leaves orphans.
  for pat in "ros2 launch" "gz sim" "ruby.*gz" parameter_bridge bridge_node \
             ros_gz_bridge static_transform_publisher; do
    pkill -9 -f "$pat" 2>/dev/null
  done
}

cleanup() {
  if [[ -n "$LAUNCH_PID" ]]; then
    kill -INT "$LAUNCH_PID" 2>/dev/null
    sleep 2
  fi
  kill_all_sim
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

kill_all_sim; sleep 1
# Stale FastRTPS shared-memory segments after a hard kill wedge DDS discovery
# (DDS_SHM_FAIL is a named failure mode in this repo; same cleanup as
# verify_legged_walk.sh).
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

ros2 launch tugbot_gazebo tugbot_gazebo.launch.py headless:=true world_sdf:="$MAZE_WORLD" > /tmp/front_camera_launch.log 2>&1 &
LAUNCH_PID=$!
# Interruptible boot wait (gz boot + sensors + bridge up). A plain foreground
# 'sleep 18' would defer a trapped INT/TERM until it finishes -- and for a
# SIGINT sent to this PID alone, bash discards the pending trap entirely when
# the sleep then exits normally (cooperative-exit behavior; verified live).
# Backgrounding the sleep and wait-ing on it lets the trap fire immediately.
sleep 18 &
wait $!
python3 tools/front_camera_check.py
RC=$?
# Teardown is owned by the EXIT trap; a plain exit preserves the check's RC
# (the trap body never calls exit, so $? survives it).
exit $RC
