#!/usr/bin/env bash
# 3D-lidar live gate: boot the maze sim headless (gz + bridge + TFs +
# scan_slice_projector via tugbot_gazebo.launch.py - no solver), then assert
# /lidar/points streams 3D clouds and the projected /scan reproduces the
# legacy field contract exactly. The dog just stands (JointPositionController
# holds STAND_POSE), which is all an observation lidar needs.
#
# NOTE: tugbot_gazebo.launch.py's own world_sdf default is
# tugbot_empty_world.sdf (ground plane only -- no anymal_c robot; the robot
# is only <include>d inside the maze world SDF). So world_sdf is pinned to
# the installed maze world below -- without it there is no robot, hence no
# lidar_3d sensor, hence no /lidar/points at all.
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"; cd "$WS"

# On this machine EVERY camera-equipped sim run (headless included) must render
# on the RTX 4070: the AMD 610M iGPU copes with gpu_lidar alone, but
# lidar+camera overloads it stochastically -> ICP nan streaks -> oracle false
# collisions / livelock (A/B 2026-07-18: plain headless oracle 4.2% then a
# TIMEOUT livelock; with PRIME 2/2 EXIT_REACHED oracle 0.000%). Pure EGL vendor
# pinning does NOT move gz off the iGPU; the GLX PRIME path needs a DISPLAY.
export DISPLAY="${DISPLAY:-:1}"
export __NV_PRIME_RENDER_OFFLOAD="${__NV_PRIME_RENDER_OFFLOAD:-1}"
export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}"

source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=$((RANDOM % 100))
echo "DOMAIN=$ROS_DOMAIN_ID"
MAZE_WORLD="$WS/install/tugbot_gazebo/share/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"

LAUNCH_PID=""

kill_all_sim() {
  # Subset of run_flood_fill_maze.sh's kill_all_sim relevant to this launch:
  # the launch tree here spawns the gz server (a ruby wrapper), the ros_gz
  # bridge_node, TWO static_transform_publisher nodes (scan_tf + camera_tf),
  # and scan_slice_projector -- all must die, or an interrupted run leaves
  # orphans (observed live: ros2 launch's SIGINT cascade does not reliably
  # reap scan_slice_projector, an rclpy node, within the 2s teardown window).
  for pat in "ros2 launch" "gz sim" "ruby.*gz" parameter_bridge bridge_node \
             ros_gz_bridge static_transform_publisher scan_slice_projector; do
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

ros2 launch tugbot_gazebo tugbot_gazebo.launch.py headless:=true world_sdf:="$MAZE_WORLD" > /tmp/lidar3d_launch.log 2>&1 &
LAUNCH_PID=$!
# Interruptible boot wait (gz boot + sensors + bridge up). A plain foreground
# 'sleep 18' would defer a trapped INT/TERM until it finishes -- and for a
# SIGINT sent to this PID alone, bash discards the pending trap entirely when
# the sleep then exits normally (cooperative-exit behavior; verified live).
# Backgrounding the sleep and wait-ing on it lets the trap fire immediately.
sleep 18 &
wait $!
python3 tools/lidar3d_check.py
RC=$?
# Teardown is owned by the EXIT trap; a plain exit preserves the check's RC
# (the trap body never calls exit, so $? survives it).
exit $RC
