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
pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f ros_gz_bridge 2>/dev/null; sleep 1
ros2 launch tugbot_gazebo tugbot_gazebo.launch.py headless:=true world_sdf:="$MAZE_WORLD" > /tmp/front_camera_launch.log 2>&1 &
LAUNCH_PID=$!
sleep 18   # gz boot + sensors + bridge up
python3 tools/front_camera_check.py
RC=$?
kill -INT $LAUNCH_PID 2>/dev/null; sleep 2
pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f ros_gz_bridge 2>/dev/null
exit $RC
