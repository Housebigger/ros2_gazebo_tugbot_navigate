from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MAZE_PKG = ROOT / 'src' / 'tugbot_maze'


def test_tugbot_maze_package_skeleton_exists():
    assert (MAZE_PKG / 'package.xml').exists()
    assert (MAZE_PKG / 'setup.py').exists()
    assert (MAZE_PKG / 'setup.cfg').exists()
    assert (MAZE_PKG / 'resource' / 'tugbot_maze').exists()
    assert (MAZE_PKG / 'tugbot_maze' / '__init__.py').exists()


def test_maze_asset_and_config_are_present():
    assert (MAZE_PKG / 'assets' / 'maze_20260522.jpg').exists()
    config = (MAZE_PKG / 'config' / 'maze_instance.yaml').read_text()
    for required in [
        'maze_20260522.jpg',
        'manual_simplified_first_pass',
        'image_allowed_for_runtime_path_planning: false',
        'MAZE_EXIT_REACHED',
        'entrance:',
        'exit:',
        'min_corridor_width_policy: at_least_twice_tugbot_turning_outer_diameter',
        'min_corridor_width_m: 1.6',
    ]:
        assert required in config


def test_maze_console_scripts_and_monitor_contract():
    setup_text = (MAZE_PKG / 'setup.py').read_text()
    for required in [
        'maze_goal_monitor = tugbot_maze.maze_goal_monitor:main',
        'maze_image_to_world = tugbot_maze.maze_image_to_world:main',
        'maze_explorer = tugbot_maze.maze_explorer:main',
        'config/*.yaml',
        'assets/*',
    ]:
        assert required in setup_text

    monitor = (MAZE_PKG / 'tugbot_maze' / 'maze_goal_monitor.py').read_text()
    for required in [
        'MazeGoalMonitor',
        'map_frame',
        'base_frame',
        'exit_x',
        'exit_y',
        'exit_radius',
        '/maze/exit_reached',
        'MAZE_EXIT_REACHED',
        'tf2_ros.Buffer',
        'Bool(data=reached)',
    ]:
        assert required in monitor


def test_maze_explorer_is_real_ros2_dfs_node():
    explorer = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()
    assert 'placeholder extension point' not in explorer
    for required in [
        'class MazeExplorer(Node)',
        "super().__init__('maze_explorer')",
        "declare_parameter('map_topic'",
        "declare_parameter('base_frame'",
        "declare_parameter('map_frame'",
        "declare_parameter('action_name'",
        "declare_parameter('entrance_x'",
        "declare_parameter('entrance_y'",
        "declare_parameter('exit_x'",
        "declare_parameter('exit_y'",
        "declare_parameter('exit_radius'",
        'OccupancyGrid',
        'create_subscription',
        'tf2_ros.Buffer',
        'TransformListener',
        'ActionClient',
        'NavigateToPose',
        '/maze/explorer_state',
        'String',
        'MazeTopology',
        'classify_local_topology',
        'make_centered_branch_goal',
        'blacklist',
        'record_branch_failure',
        'record_backtrack_failure',
        'last_failure_reason',
        'blocked_branch_count',
        'blacklisted_goal_count',
        'last_local_topology_kind',
        'last_open_direction_count',
        'active_goal_target',
        'goal_sequence_id',
        'GOAL_PREEMPTED',
        'stale_result_count',
        'preempted_goal_count',
        'goal_settle_sec',
        'SETTLING',
        'GOAL_CANCELED_AFTER_EXIT',
        'GOAL_CANCELED_AFTER_TIMEOUT',
        'terminal_cancel_count',
        'timeout_cancel_count',
        'canceled_after_exit_count',
        'canceled_after_timeout_count',
        'last_terminal_reason',
        'MAZE_EXPLORER_EXIT_REACHED',
        'MAZE_EXPLORER_EXHAUSTED',
    ]:
        assert required in explorer


def test_tugbot_maze_package_declares_maze_explorer_runtime_dependencies():
    package_xml = (MAZE_PKG / 'package.xml').read_text()
    for required in [
        '<exec_depend>nav_msgs</exec_depend>',
        '<exec_depend>nav2_msgs</exec_depend>',
        '<exec_depend>action_msgs</exec_depend>',
    ]:
        assert required in package_xml
