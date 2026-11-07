#!/usr/bin/env bash
# Start one simulation. Ctrl+C shuts down this launch and its child processes.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source setup.bash
if command -v nvidia-smi >/dev/null && nvidia-smi -L >/dev/null 2>&1; then
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
fi
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
mkdir -p "$PWD/log/runs"
run_dir="$(mktemp -d "$PWD/log/runs/$(date +%Y%m%d-%H%M%S)-XXXXXX")"
exec ros2 launch go2_maze_bringup maze.launch.py gui:=true rviz:=true "run_dir:=$run_dir" "$@"
