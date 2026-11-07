#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source setup.bash
maze_test_dir=src/tugbot_maze/test
tests=(
  "$maze_test_dir"/test_go2_*.py
  "$maze_test_dir"/test_legged_*.py
  "$maze_test_dir/test_flood_fill_brain.py"
  "$maze_test_dir/test_cell_walls.py"
  "$maze_test_dir/test_scan_match_localizer.py"
  "$maze_test_dir/test_online_scan_match_localizer.py"
  "$maze_test_dir/test_pose_tracking.py"
  "$maze_test_dir/test_footprint.py"
  "$maze_test_dir/test_maze_sim.py"
  "$maze_test_dir/test_slice_to_scan.py"
  "$maze_test_dir/test_cloud_map_3d.py"
  "$maze_test_dir/test_map_memory.py"
  "$maze_test_dir/test_maze_motion.py"
)
exec python3 -m pytest -q "${tests[@]}" "$@"
