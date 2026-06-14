from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
NAV_CONFIG = ROOT / 'src' / 'tugbot_navigation' / 'config'
BRINGUP_LAUNCH = ROOT / 'src' / 'tugbot_bringup' / 'launch'


def test_phase16_nav2_tolerant_profile_exists_and_only_changes_progress_controller_tolerance():
    base = (NAV_CONFIG / 'nav2_slam_params.yaml').read_text()
    phase16_path = NAV_CONFIG / 'nav2_slam_phase16_progress_params.yaml'
    phase16 = phase16_path.read_text()

    assert 'Phase 16' in phase16
    assert 'required_movement_radius: 0.25' in phase16
    assert 'movement_time_allowance: 20.0' in phase16
    assert 'failure_tolerance: 1.0' in phase16

    for unchanged in [
        'branch_goal_step_m',
        'clearance_radius_m',
        'open_direction_lookahead_m',
        'lateral_centering_search_m',
    ]:
        assert unchanged not in phase16

    for nav2_unchanged in [
        'controller_frequency: 20.0',
        'controller_plugins: ["FollowPath"]',
        'plugin: "nav2_mppi_controller::MPPIController"',
        'vx_max: 0.5',
        'vx_min: -0.35',
        'wz_max: 0.5',
        'inflation_radius: 0.70',
        'robot_radius: 0.35',
    ]:
        assert nav2_unchanged in base
        assert nav2_unchanged in phase16


def test_phase16_maze_launch_exposes_progress_profile_argument_without_changing_dfs_defaults():
    launch = (BRINGUP_LAUNCH / 'tugbot_maze_explore.launch.py').read_text()

    assert "nav2_slam_phase16_progress_params.yaml" in launch
    assert "DeclareLaunchArgument('phase16_nav2_progress_profile'" in launch
    assert 'nav2_params_file = PythonExpression' in launch
    assert "'params_file': nav2_params_file" in launch
    assert "'nav2_slam_phase16_progress_params.yaml'" in launch

    # Branch geometry defaults must remain unchanged in the Phase 16 experiment.
    assert "DeclareLaunchArgument('branch_goal_step_m', default_value='1.1'" in launch
    assert "DeclareLaunchArgument('clearance_radius_m', default_value='0.38'" in launch
    assert "DeclareLaunchArgument('open_direction_lookahead_m', default_value='1.6'" in launch
    assert "DeclareLaunchArgument('lateral_centering_search_m', default_value='0.9'" in launch
