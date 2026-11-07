# Source this file from a fresh Bash terminal.
_GO2_MAZE_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/lyrical/setup.bash || return
source "$_GO2_MAZE_WS/install/local_setup.bash" || return
export ROS_DOMAIN_ID="${GO2_MAZE_ROS_DOMAIN_ID:-55}"
export GZ_PARTITION="${GO2_MAZE_GZ_PARTITION:-go2_maze}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_HOME="$_GO2_MAZE_WS/log/ros"
export ROS_LOG_DIR="$ROS_HOME/log"
unset _GO2_MAZE_WS
