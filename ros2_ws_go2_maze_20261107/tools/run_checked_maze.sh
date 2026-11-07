#!/usr/bin/env bash
# Run a bounded, independently checked experiment; never reuse evidence directories.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source setup.bash
if command -v nvidia-smi >/dev/null && nvidia-smi -L >/dev/null 2>&1; then
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
fi
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
mkdir -p log/runs
if [ "$#" -gt 0 ]; then
  run_dir="$(realpath -m "$1")"
  shift
  if [ -d "$run_dir" ] && [ -n "$(ls -A "$run_dir")" ]; then
    echo "Evidence directory must be empty: $run_dir" >&2
    exit 2
  fi
  mkdir -p "$run_dir"
else
  run_dir="$(mktemp -d "$PWD/log/runs/checked-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
fi
export ROS_HOME="$run_dir/ros-home" ROS_LOG_DIR="$run_dir/ros"
setsid ros2 launch go2_maze_bringup maze.launch.py gui:=false rviz:=false \
  "run_dir:=$run_dir" "$@" > "$run_dir/launch.log" 2>&1 &
launch_pid=$!
cleanup() {
  # ROS/Gazebo shutdown can emit another interrupt while children exit.
  # Finish the owned-process cleanup before allowing another cancellation.
  trap '' INT TERM
  kill -INT -- "-$launch_pid" 2>/dev/null || true
  for _ in {1..15}; do
    kill -0 -- "-$launch_pid" 2>/dev/null || break
    sleep 0.4
  done
  kill -TERM -- "-$launch_pid" 2>/dev/null || true
  sleep 0.3
  kill -KILL -- "-$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
python3 -u tools/watch_maze_run.py "$run_dir" "$launch_pid" | tee "$run_dir/watch.log"
cleanup
trap - EXIT
trap 'exit 130' INT TERM
python3 tools/replay_go2_audit.py "$run_dir" --require-success
