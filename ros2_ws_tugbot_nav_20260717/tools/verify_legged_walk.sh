#!/usr/bin/env bash
# Verification ladder rungs 1-4 (spec section 8). Boots gz server + bridge +
# locomotion_controller once, then runs stand -> step -> walk -> turn,
# aborting on the first failure. Maze (rung 5) is run via run_flood_fill_maze.sh.
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"; cd "$WS"
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export GZ_SIM_RESOURCE_PATH="${WS}/install/tugbot_description/share/tugbot_description/models:${GZ_SIM_RESOURCE_PATH:-}"
export ROS_DOMAIN_ID=$(( ($(date +%s) % 90) + 1 ))

cleanup() {
  pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null
  pkill -9 -f locomotion_controller 2>/dev/null
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM
cleanup; sleep 1

LOG="$(mktemp -d /tmp/legged_ladder.XXXXXX)"
echo "ladder logs: $LOG"
gz sim -s -r src/tugbot_gazebo/worlds/anymal_test_world.sdf > "$LOG/gz.log" 2>&1 &
sleep 6
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:="$WS/install/tugbot_gazebo/share/tugbot_gazebo/config/tugbot_bridge.yaml" \
  > "$LOG/bridge.log" 2>&1 &
ros2 run tugbot_maze locomotion_controller --ros-args -p use_sim_time:=true \
  > "$LOG/loco.log" 2>&1 &
sleep 8   # INIT settle done

for rung in stand step walk turn; do
  echo "=== RUNG $rung ==="
  if ! python3 tools/legged_walk_check.py --rung "$rung"; then
    echo "LADDER FAIL at rung $rung (logs: $LOG)"; exit 1
  fi
done
echo "LADDER PASS (all 4 rungs)"
