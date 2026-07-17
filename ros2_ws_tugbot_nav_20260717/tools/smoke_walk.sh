#!/usr/bin/env bash
# Closed-loop smoke: gz server + bridge + locomotion_controller; drive 0.3 m/s
# for 10 sim-s; assert forward displacement > 1.5 m and no fall.
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"; cd "$WS"
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export GZ_SIM_RESOURCE_PATH="${WS}/install/tugbot_description/share/tugbot_description/models:${GZ_SIM_RESOURCE_PATH:-}"
export ROS_DOMAIN_ID=$(( ($(date +%s) % 90) + 1 ))
pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null
pkill -9 -f locomotion_controller 2>/dev/null; sleep 1
gz sim -s -r src/tugbot_gazebo/worlds/anymal_test_world.sdf > /tmp/smoke_walk_gz.log 2>&1 &
sleep 6
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=$WS/install/tugbot_gazebo/share/tugbot_gazebo/config/tugbot_bridge.yaml \
  > /tmp/smoke_walk_bridge.log 2>&1 &
ros2 run tugbot_maze locomotion_controller --ros-args -p use_sim_time:=true \
  > /tmp/smoke_walk_loco.log 2>&1 &
sleep 6    # INIT settle
python3 - <<'EOF'
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rclpy.init()
n = Node('smoke_walk_check')
poses = []
n.create_subscription(Odometry, '/odom', lambda m: poses.append(
    (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)), 10)
pub = n.create_publisher(Twist, '/cmd_vel', 10)
t0 = time.time()
tw = Twist(); tw.linear.x = 0.3
while time.time() - t0 < 25 and rclpy.ok():   # ~10+ sim-s even at RTF~0.5
    pub.publish(tw)
    rclpy.spin_once(n, timeout_sec=0.05)
pub.publish(Twist())
assert poses, 'no odom received'
dx = poses[-1][0] - poses[0][0]
zs = [p[2] for p in poses[len(poses)//2:]]
print(f'dx={dx:.2f} z_range=({min(zs):.2f},{max(zs):.2f}) n={len(poses)}')
assert dx > 1.5, f'walked only {dx:.2f} m'
assert min(zs) > 0.30, 'body collapsed'
print('WALK SMOKE PASS')
EOF
RC=$?
pkill -9 -f "gz sim"; pkill -9 -f parameter_bridge; pkill -9 -f locomotion_controller
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
exit $RC
