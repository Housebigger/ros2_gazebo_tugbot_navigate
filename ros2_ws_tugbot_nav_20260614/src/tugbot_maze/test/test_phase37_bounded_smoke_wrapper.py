from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase37_scaled_clean_world_maze_explorer_bounded_smoke.sh'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
ACTIVE_METADATA = 'maze_20260528_scaled_instance.yaml'


def test_phase37_wrapper_exists_and_uses_bounded_timeout_contract():
    assert WRAPPER.exists(), 'Phase37 bounded smoke wrapper must exist'
    source = WRAPPER.read_text(encoding='utf-8')

    assert 'set -euo pipefail' in source
    assert 'RUN_TIMEOUT_SEC="${PHASE37_RUN_TIMEOUT_SEC:-900}"' in source
    assert 'MAX_GOALS="${PHASE37_MAX_GOALS:-4}"' in source
    assert 'timeout --preserve-status "$RUN_TIMEOUT_SEC"' in source
    assert 'ros2 launch tugbot_bringup tugbot_maze_explore.launch.py' in source
    assert 'max_goals:="$MAX_GOALS"' in source
    assert 'headless:=true' in source
    assert 'use_rviz:="${USE_RVIZ}"' in source


def test_phase37_wrapper_explicitly_uses_active_scaled_world_and_metadata_truth():
    assert WRAPPER.exists(), 'Phase37 bounded smoke wrapper must exist'
    source = WRAPPER.read_text(encoding='utf-8')

    assert ACTIVE_WORLD in source
    assert ACTIVE_METADATA in source
    for token in [
        'entrance_x:="0.0"',
        'entrance_y:="0.0"',
        'entrance_yaw:="0.0"',
        'exit_x:="21.072562"',
        'exit_y:="18.083566"',
        'exit_radius:="1.2"',
    ]:
        assert token in source
    forbidden_launch_args = [
        'maze_config:="$ROOT/src/tugbot_maze/config/maze_instance.yaml"',
        'world_sdf:="$ROOT/src/tugbot_gazebo/worlds/tugbot_maze_world.sdf"',
        'entrance_x:="-4.0"',
        'entrance_y:="-3.0"',
        'exit_x:="4.0"',
        'exit_y:="3.0"',
        'exit_radius:="0.6"',
    ]
    for legacy_token in forbidden_launch_args:
        assert legacy_token not in source


def test_phase37_wrapper_records_required_smoke_artifacts_and_cleanup():
    assert WRAPPER.exists(), 'Phase37 bounded smoke wrapper must exist'
    source = WRAPPER.read_text(encoding='utf-8')

    for artifact in [
        'goal_events.jsonl',
        'explorer_state.jsonl',
        'launch.log',
        'ros_graph_initial.txt',
        'ros_graph_final.txt',
        'map_once.txt',
        'scan_once.txt',
        'odom_once.txt',
        'tf_once.txt',
        'local_costmap_once.txt',
        'global_costmap_once.txt',
        'navigate_to_pose_action_info.txt',
        'summary.json',
        'cleanup_processes_after.txt',
    ]:
        assert artifact in source

    assert 'record_explorer_state_series.py --topic /maze/goal_events' in source
    assert 'record_explorer_state_series.py --topic /maze/explorer_state' in source
    assert 'ros2 action info /navigate_to_pose' in source
    assert 'ros2 topic echo --once /local_costmap/costmap' in source
    assert 'ros2 topic echo --once /global_costmap/costmap' in source
    assert 'trap cleanup EXIT' in source
    for cleanup_token in [
        'maze_explorer',
        'maze_goal_monitor',
        'ros_gz_bridge',
        'static_transform_publisher',
        'slam_toolbox',
        'controller_server',
        'planner_server',
        'bt_navigator',
        'gz sim',
    ]:
        assert cleanup_token in source
