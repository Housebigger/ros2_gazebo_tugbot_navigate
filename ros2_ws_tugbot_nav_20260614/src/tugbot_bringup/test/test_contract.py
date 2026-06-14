import ast
import re
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def _launch_default(launch_text: str, argument_name: str) -> str:
    pattern = (
        r"DeclareLaunchArgument\(\s*"
        rf"['\"]{re.escape(argument_name)}['\"]\s*,\s*"
        r"default_value\s*=\s*['\"]([^'\"]+)['\"]"
    )
    match = re.search(pattern, launch_text)
    assert match, f"Launch argument {argument_name!r} with literal default_value not found"
    return match.group(1)


def _declared_parameter_default(source_text: str, parameter_name: str):
    pattern = (
        r"declare_parameter\(\s*"
        rf"['\"]{re.escape(parameter_name)}['\"]\s*,\s*"
        r"([^\)]+?)\s*\)"
    )
    match = re.search(pattern, source_text)
    assert match, f"declare_parameter default for {parameter_name!r} not found"
    return ast.literal_eval(match.group(1))


def _yaml_scalar(text: str, key: str) -> float:
    match = re.search(rf"^\s*{re.escape(key)}:\s*([-+]?[0-9]+(?:\.[0-9]+)?)\s*$", text, re.MULTILINE)
    assert match, f"YAML scalar {key!r} not found"
    return float(match.group(1))


def _yaml_list(text: str, key: str):
    match = re.search(rf"^\s*{re.escape(key)}:\s*\[([^\]]+)\]\s*$", text, re.MULTILINE)
    assert match, f"YAML list {key!r} not found"
    return [float(item.strip()) for item in match.group(1).split(',')]

def test_expected_packages_exist():
    for pkg in ['tugbot_description', 'tugbot_gazebo', 'tugbot_navigation', 'tugbot_bringup']:
        assert (ROOT / 'src' / pkg / 'package.xml').exists()


def test_frontier_exploration_contract():
    exploration_pkg = ROOT / 'src' / 'tugbot_exploration'
    assert (exploration_pkg / 'package.xml').exists()
    assert (exploration_pkg / 'setup.py').exists()
    assert (exploration_pkg / 'resource' / 'tugbot_exploration').exists()
    assert (exploration_pkg / 'tugbot_exploration' / '__init__.py').exists()

    explorer = (exploration_pkg / 'tugbot_exploration' / 'frontier_explorer.py').read_text()
    for required in [
        'rclpy',
        'OccupancyGrid',
        'NavigateToPose',
        'ActionClient',
        'tf2_ros',
        "value == -1",
        "value == 0",
        "value > 50",
        'min_obstacle_distance_m',
        'min_goal_distance_m',
        'max_goal_distance_m',
        'blacklist_radius_m',
        'finish_no_frontier_cycles',
        'max_goals',
        'save_map',
        'map_save_path',
        'map_saver_cli',
        'subprocess.run',
        '.yaml',
        '.pgm',
        'frontier_explorer = tugbot_exploration.frontier_explorer:main',
    ]:
        assert required in explorer or required in (exploration_pkg / 'setup.py').read_text()

    explore_launch = (ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_explore.launch.py').read_text()
    assert 'tugbot_slam_nav.launch.py' in explore_launch
    assert 'frontier_explorer' in explore_launch
    assert 'tugbot_exploration' in explore_launch
    assert 'max_goals' in explore_launch
    assert "default_value='20'" in explore_launch or 'default_value="20"' in explore_launch
    assert 'min_obstacle_distance_m' in explore_launch
    assert 'max_goal_distance_m' in explore_launch
    assert 'finish_no_frontier_cycles' in explore_launch
    assert 'save_map' in explore_launch
    assert 'map_save_path' in explore_launch
    assert 'maps/explored' in explore_launch
    assert 'tugbot_nav_world_slam' in explore_launch
    assert 'map_1725111373.yaml' not in explore_launch
    assert 'bringup_launch.py' not in explore_launch
    assert "'map':" not in explore_launch


def test_phase10_explore_defaults_are_phase6_stable_not_phase7a_aggressive():
    explore_launch = (ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_explore.launch.py').read_text()

    expected_phase6_defaults = {
        'max_goals': '20',
        'max_cleanup_goals': '5',
        'cleanup_search_radius_max_m': '2.0',
        'cleanup_min_obstacle_distance_m': '0.35',
        'target_unknown_ratio': '0.03',
    }
    for argument_name, expected_default in expected_phase6_defaults.items():
        assert _launch_default(explore_launch, argument_name) == expected_default

    forbidden_phase7a_defaults = {
        'max_goals': '25',
        'max_cleanup_goals': '12',
        'cleanup_search_radius_max_m': '3.0',
        'cleanup_min_obstacle_distance_m': '0.30',
        'target_unknown_ratio': '0.05',
    }
    for argument_name, forbidden_default in forbidden_phase7a_defaults.items():
        assert _launch_default(explore_launch, argument_name) != forbidden_default


def test_phase13_perimeter_first_strategy_contract():
    explorer = (ROOT / 'src' / 'tugbot_exploration' / 'tugbot_exploration' / 'frontier_explorer.py').read_text()
    explore_launch = (ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_explore.launch.py').read_text()

    assert _launch_default(explore_launch, 'exploration_strategy') == 'frontier'
    assert _launch_default(explore_launch, 'exploration_strategy') != 'perimeter_then_frontier'

    perimeter_defaults = {
        'perimeter_enable_initial_spin': 'false',
        'perimeter_spin_angle': '1.57',
        'perimeter_wall_min_cluster_size': '80',
        'perimeter_wall_min_length_m': '1.5',
        'perimeter_wall_aspect_ratio_min': '4.0',
        'perimeter_wall_offset_m': '0.75',
        'perimeter_waypoint_spacing_m': '1.0',
        'perimeter_max_waypoints': '20',
        'perimeter_direction': 'ccw',
        'perimeter_switch_to_frontier_after_done': 'true',
        'perimeter_goal_timeout_sec': '60.0',
    }
    for argument_name, expected_default in perimeter_defaults.items():
        assert _launch_default(explore_launch, argument_name) == expected_default
        assert argument_name in explorer
        assert argument_name in explore_launch

    for required in [
        "exploration_strategy', 'frontier'",
        'perimeter_then_frontier',
        'perimeter wall cluster detection',
        'detected wall clusters',
        'generated perimeter waypoints',
        'accepted perimeter waypoints',
        'rejected perimeter waypoints',
        'Executing perimeter waypoint',
        'Perimeter phase complete; switching to frontier',
        'perimeter waypoint failed',
        'perimeter initial spin enabled/disabled',
        'initial spin disabled; skipping',
        'spin skipped because disabled',
        'wall cluster detection result',
        'fallback to frontier if insufficient map',
        '_run_perimeter_then_frontier',
        '_detect_perimeter_wall_clusters',
        '_generate_perimeter_waypoints',
        '_sort_perimeter_waypoints',
    ]:
        assert required in explorer

    assert "ParameterValue(exploration_strategy" not in explore_launch
    assert "'exploration_strategy': exploration_strategy" in explore_launch
    assert 'camera' not in explorer.lower()
    assert 'box_pillar' not in explorer


def test_phase5_frontier_completion_recovery_contract():
    explorer = (ROOT / 'src' / 'tugbot_exploration' / 'tugbot_exploration' / 'frontier_explorer.py').read_text()
    explore_launch = (ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_explore.launch.py').read_text()

    for required in [
        'min_goals_before_finish',
        'min_frontier_clusters_before_finish',
        'recovery_scan_attempts',
        'relaxed_retry_attempts',
        'map_stable_cycles_before_finish',
        'map_change_free_threshold',
        'map_change_unknown_threshold',
        'relaxed_min_obstacle_distance_m',
        'relaxed_max_goal_distance_m',
        'relaxed_min_cluster_size',
        'enable_recovery_scan',
        'recovery_spin_angle',
        'recovery_wait_after_scan_sec',
    ]:
        assert required in explorer
        assert required in explore_launch

    for required in [
        'raw frontier cluster count',
        'valid candidate count',
        'finish_no_frontier_cycles',
        'recovery_no_candidate_cycles',
        'recovery_mode',
        'search_mode=strict',
        'search_mode=relaxed',
        'completed_goals',
        'map_stable_cycles',
        'relaxed search',
        'recovery scan',
    ]:
        assert required in explorer

    assert 'valid_candidates == 0' in explorer
    assert 'frontier_clusters <= min_frontier_clusters_before_finish' in explorer
    assert 'raw_cluster_count > self.min_frontier_clusters_before_finish' in explorer
    assert 'No valid frontier candidate (%d/%d cycles)' not in explorer
    assert 'Exploration complete: no valid frontier remained' not in explorer

    assert "default_value='8'" in explore_launch or 'default_value="8"' in explore_launch
    assert "default_value='5'" in explore_launch or 'default_value="5"' in explore_launch
    assert "default_value='3'" in explore_launch or 'default_value="3"' in explore_launch
    assert "default_value='50'" in explore_launch or 'default_value="50"' in explore_launch
    assert "default_value='0.35'" in explore_launch or 'default_value="0.35"' in explore_launch
    assert "default_value='8.0'" in explore_launch or 'default_value="8.0"' in explore_launch
    assert "default_value='true'" in explore_launch or 'default_value="true"' in explore_launch


def test_phase6_coverage_cleanup_contract():
    explorer = (ROOT / 'src' / 'tugbot_exploration' / 'tugbot_exploration' / 'frontier_explorer.py').read_text()
    explore_launch = (ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_explore.launch.py').read_text()

    for required in [
        'enable_cleanup_mode',
        'cleanup_unknown_cluster_min_size',
        'cleanup_max_unknown_clusters',
        'cleanup_search_radius_min_m',
        'cleanup_search_radius_max_m',
        'cleanup_min_obstacle_distance_m',
        'cleanup_goal_timeout_sec',
        'cleanup_spin_after_goal',
        'cleanup_spin_angle',
        'cleanup_wait_after_spin_sec',
        'target_unknown_ratio',
        'max_cleanup_goals',
    ]:
        assert required in explorer
        assert required in explore_launch

    for required in [
        'unknown_total_count',
        'unknown_ratio',
        'unknown_cluster_count',
        'cleanup_candidate_count',
        'cleanup mode',
        'Selected cleanup goal',
        'cleanup spin',
        'cleanup spin disabled; skipping',
        'cleanup_goals_completed',
        'cleanup_spins_completed',
        'residual unknown cluster',
    ]:
        assert required in explorer

    assert 'value == -1' in explorer
    assert '_find_unknown_clusters' in explorer
    assert '_find_cleanup_candidates' in explorer
    assert '@dataclass\nclass UnknownCluster' in explorer
    assert '@dataclass\nclass CleanupCandidate' in explorer
    assert 'yaw = math.atan2' in explorer
    assert 'cleanup_candidate_count > 0' in explorer
    assert 'Cleanup candidates remain; postponing final map save' in explorer
    assert 'reached max_goals=%d' not in explorer

    for default in [
        "default_value='true'",
        "default_value='30'",
        "default_value='5'",
        "default_value='0.5'",
        "default_value='2.0'",
        "default_value='0.35'",
        "default_value='60.0'",
        "default_value='1.57'",
        "default_value='0.03'",
    ]:
        assert default in explore_launch or default.replace("'", '"') in explore_launch



def test_phase14_no_spin_mapping_stability_contract():
    explorer = (ROOT / 'src' / 'tugbot_exploration' / 'tugbot_exploration' / 'frontier_explorer.py').read_text()
    explore_launch = (ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_explore.launch.py').read_text()
    nav2_slam = (ROOT / 'src' / 'tugbot_navigation' / 'config' / 'nav2_slam_params.yaml').read_text()
    nav_no_spin_bt = (ROOT / 'src' / 'tugbot_navigation' / 'behavior_trees' / 'navigate_to_pose_w_replanning_no_spin.xml').read_text()
    through_no_spin_bt = (ROOT / 'src' / 'tugbot_navigation' / 'behavior_trees' / 'navigate_through_poses_w_replanning_no_spin.xml').read_text()
    nav_cmake = (ROOT / 'src' / 'tugbot_navigation' / 'CMakeLists.txt').read_text()

    assert _launch_default(explore_launch, 'perimeter_enable_initial_spin') == 'false'
    assert _declared_parameter_default(explorer, 'perimeter_enable_initial_spin') is False
    assert _launch_default(explore_launch, 'cleanup_spin_after_goal') == 'false'
    assert _declared_parameter_default(explorer, 'cleanup_spin_after_goal') is False
    assert _launch_default(explore_launch, 'enable_recovery_scan') == 'false'
    assert _declared_parameter_default(explorer, 'enable_recovery_scan') is False

    assert float(_launch_default(explore_launch, 'recovery_spin_angle')) <= 1.57
    assert _declared_parameter_default(explorer, 'recovery_spin_angle') <= 1.57
    assert float(_launch_default(explore_launch, 'cleanup_spin_angle')) <= 1.57
    assert _declared_parameter_default(explorer, 'cleanup_spin_angle') <= 1.57

    for required_log in [
        'enable_recovery_scan=%s',
        'recovery_spin_angle=%.2f',
        'cleanup_spin_after_goal=%s',
        'cleanup_spin_angle=%.2f',
        'perimeter_enable_initial_spin=%s',
        'perimeter_spin_angle=%.2f',
        'max_vel_theta limited by Nav2 params_file',
        'initial spin disabled; skipping',
        'cleanup spin disabled; skipping',
        'recovery scan disabled; not spinning',
        'perimeter initial spin enabled/disabled',
        'spin skipped because disabled',
        'wall cluster detection result',
        'fallback to frontier if insufficient map',
    ]:
        assert required_log in explorer

    assert 'This file is intentionally separate from nav2_params.yaml' in nav2_slam
    assert 'default_nav_to_pose_bt_xml' in nav2_slam
    assert 'navigate_to_pose_w_replanning_no_spin.xml' in nav2_slam
    assert 'default_nav_through_poses_bt_xml' in nav2_slam
    assert 'navigate_through_poses_w_replanning_no_spin.xml' in nav2_slam
    assert 'behavior_trees' in nav_cmake
    for bt_text in (nav_no_spin_bt, through_no_spin_bt):
        assert '<Spin ' not in bt_text
        assert 'RecoveryActionsNoSpin' in bt_text
        assert 'ClearEntireCostmap' in bt_text
        assert '<Wait wait_duration="5.0"/>' in bt_text
        assert '<BackUp ' in bt_text
    assert _yaml_scalar(nav2_slam, 'wz_max') <= 0.5
    assert _yaml_scalar(nav2_slam, 'az_max') <= 0.8
    assert _yaml_scalar(nav2_slam, 'max_rotational_vel') <= 0.5
    assert _yaml_scalar(nav2_slam, 'rotational_acc_lim') <= 0.8
    assert _yaml_list(nav2_slam, 'max_velocity')[2] <= 0.5
    assert abs(_yaml_list(nav2_slam, 'min_velocity')[2]) <= 0.5
    assert _yaml_list(nav2_slam, 'max_accel')[2] <= 0.8
    assert abs(_yaml_list(nav2_slam, 'max_decel')[2]) <= 0.8

def test_phase4_readme_contract():
    readme = (ROOT / 'README.md').read_text()
    for required in [
        'ros2_ws_tugbot_nav_20260514',
        '继承 0513',
        'SLAM 在线建图',
        'SLAM + Nav2 手动导航',
        'frontier 自主探索建图',
        '地图保存',
        'tugbot_nav.launch.py',
        'tugbot_slam.launch.py',
        'tugbot_slam_nav.launch.py',
        'tugbot_explore.launch.py',
        'save_map:=true',
        'map_save_path:=/absolute/path/to/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_manual',
        '探索阶段 `/map` 由 `slam_toolbox` 发布',
        '探索阶段不启动 AMCL / 静态 `map_server`',
        '.yaml + .pgm',
    ]:
        assert required in readme

def test_phase8_phase6_static_map_nav_launch_contract():
    phase6_map = ROOT / 'src' / 'tugbot_navigation' / 'maps' / 'explored' / 'tugbot_nav_world_slam_phase6_cleanup.yaml'
    phase6_pgm = ROOT / 'src' / 'tugbot_navigation' / 'maps' / 'explored' / 'tugbot_nav_world_slam_phase6_cleanup.pgm'
    assert phase6_map.exists()
    assert phase6_pgm.exists()
    assert 'image: tugbot_nav_world_slam_phase6_cleanup.pgm' in phase6_map.read_text()

    launch_path = ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_nav_phase6_map.launch.py'
    assert launch_path.exists()
    launch = launch_path.read_text()

    for required in [
        'tugbot_nav_world.sdf',
        'tugbot_nav_world_slam_phase6_cleanup.yaml',
        'nav2_phase6_map_params.yaml',
        'tugbot_gazebo.launch.py',
        'ros_gz_bridge',
        'scan_omni_static_tf',
        'bringup_launch.py',
        'nav2_bringup',
        "'map': map_yaml",
        "'params_file': params_file",
        "'use_sim_time': use_sim_time",
        "'autostart': autostart",
        "'use_composition': 'False'",
        "'slam': 'False'",
        "'use_localization': 'True'",
        'rviz2',
        'IfCondition',
        'headless',
        'use_rviz',
        'use_sim_time',
        'autostart',
    ]:
        assert required in launch

    assert 'map_1725111373.yaml' not in launch
    assert 'tugbot_nav_world_slam_phase7_budget' not in launch
    assert 'slam_toolbox' not in launch
    assert 'frontier_explorer' not in launch
    assert 'tugbot_explore.launch.py' not in launch
    assert 'tugbot_slam_nav.launch.py' not in launch
    assert 'tugbot_slam.launch.py' not in launch
    assert 'online_async_launch.py' not in launch
    for forbidden_explore_param in [
        'save_map',
        'map_save_path',
        'enable_cleanup_mode',
        'max_cleanup_goals',
        'target_unknown_ratio',
        'cleanup_search_radius',
        'cleanup_min_obstacle_distance',
        'cleanup_unknown_cluster',
    ]:
        assert forbidden_explore_param not in launch

    phase6_params_path = ROOT / 'src' / 'tugbot_navigation' / 'config' / 'nav2_phase6_map_params.yaml'
    assert phase6_params_path.exists()
    phase6_params = phase6_params_path.read_text()
    assert 'robot_radius: 0.35' in phase6_params
    assert 'topic: /scan' in phase6_params
    assert 'collision_monitor:' in phase6_params
    assert 'inflation_radius: 0.50' in phase6_params

    nav_launch = (ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_nav.launch.py').read_text()
    slam_launch = (ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_slam.launch.py').read_text()
    slam_nav_launch = (ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_slam_nav.launch.py').read_text()
    explore_launch = (ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_explore.launch.py').read_text()
    assert 'tugbot_nav_world_slam_phase6_cleanup.yaml' not in nav_launch
    assert 'tugbot_nav_world_slam_phase6_cleanup.yaml' not in slam_launch
    assert 'tugbot_nav_world_slam_phase6_cleanup.yaml' not in slam_nav_launch
    assert 'tugbot_nav_world_slam_phase6_cleanup.yaml' not in explore_launch


def test_phase16_phase14_static_map_nav_launch_contract():
    phase14_map = ROOT / 'src' / 'tugbot_navigation' / 'maps' / 'explored' / 'tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml'
    phase14_pgm = ROOT / 'src' / 'tugbot_navigation' / 'maps' / 'explored' / 'tugbot_nav_world_slam_phase14_perimeter_no_spin.pgm'
    assert phase14_map.exists()
    assert phase14_pgm.exists()
    assert 'image: tugbot_nav_world_slam_phase14_perimeter_no_spin.pgm' in phase14_map.read_text()

    launch_path = ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_nav_phase14_map.launch.py'
    assert launch_path.exists()
    launch = launch_path.read_text()

    for required in [
        'tugbot_nav_world.sdf',
        'tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml',
        'nav2_phase14_map_params.yaml',
        'tugbot_gazebo.launch.py',
        'ros_gz_bridge',
        'scan_omni_static_tf',
        'bringup_launch.py',
        'nav2_bringup',
        "'map': map_yaml",
        "'params_file': params_file",
        "'use_sim_time': use_sim_time",
        "'autostart': autostart",
        "'use_composition': 'False'",
        "'slam': 'False'",
        "'use_localization': 'True'",
        'rviz2',
        'IfCondition',
        'headless',
        'use_rviz',
        'use_sim_time',
        'autostart',
        'map',
        'params_file',
    ]:
        assert required in launch

    for forbidden in [
        'map_1725111373.yaml',
        'tugbot_nav_world_slam_phase6_cleanup.yaml',
        'tugbot_nav_world_slam_phase7_budget.yaml',
        'slam_toolbox',
        'async_slam_toolbox_node',
        'frontier_explorer',
        'tugbot_explore.launch.py',
        'tugbot_slam.launch.py',
        'tugbot_slam_nav.launch.py',
        'save_map',
        'map_save_path',
        'cleanup',
    ]:
        assert forbidden not in launch

    phase14_params_path = ROOT / 'src' / 'tugbot_navigation' / 'config' / 'nav2_phase14_map_params.yaml'
    phase6_params_path = ROOT / 'src' / 'tugbot_navigation' / 'config' / 'nav2_phase6_map_params.yaml'
    assert phase14_params_path.exists()
    assert phase6_params_path.exists()
    phase14_params = phase14_params_path.read_text()
    phase6_params = phase6_params_path.read_text()
    assert 'robot_radius: 0.35' in phase14_params
    assert 'topic: /scan' in phase14_params
    assert 'collision_monitor:' in phase14_params
    assert 'inflation_radius: 0.50' in phase14_params
    assert 'tugbot_nav_world_slam_phase6_cleanup' not in phase14_params
    assert 'tugbot_nav_world_slam_phase14_perimeter_no_spin' not in phase14_params
    assert phase14_params != ''
    assert phase6_params != ''


def test_phase16_readme_quickstart_and_phase14_replay_contract():
    readme = (ROOT / 'README.md').read_text()

    assert 'Phase 14 perimeter no-spin map 是当前推荐成果地图' in readme
    assert '不建议普通用户直接覆盖' in readme
    assert '若要重新探索，请保存为新文件名' in readme

    assert '人工可视化探索' in readme
    manual_idx = readme.index('人工可视化探索')
    manual_section = readme[manual_idx: manual_idx + 1200]
    assert 'tugbot_explore.launch.py' in manual_section
    assert 'headless:=false' in manual_section
    assert 'use_rviz:=true' in manual_section
    assert 'max_goals:=20' in manual_section
    assert 'enable_cleanup_mode:=true' in manual_section
    assert 'save_map:=true' in manual_section
    assert 'map_save_path:=/absolute/path/to/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_manual' in manual_section

    assert '自动/headless 验证' in readme
    headless_idx = readme.index('自动/headless 验证')
    headless_section = readme[headless_idx: headless_idx + 1200]
    assert 'tugbot_explore.launch.py' in headless_section
    assert 'headless:=true' in headless_section
    assert 'use_rviz:=false' in headless_section
    assert 'max_goals:=20' in headless_section
    assert 'enable_cleanup_mode:=true' in headless_section
    assert 'save_map:=true' in headless_section
    assert 'map_save_path:=/absolute/path/to/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_headless_test' in headless_section

    assert 'Phase 7A 激进实验参数' in readme
    assert '不推荐默认使用/负面实验记录' in readme
    phase7a_idx = readme.index('Phase 7A 激进实验参数')
    phase7a_section = readme[phase7a_idx: phase7a_idx + 1000]
    for phase7a_token in [
        'max_goals:=25',
        'max_cleanup_goals:=12',
        'cleanup_search_radius_max_m:=3.0',
        'cleanup_min_obstacle_distance_m:=0.30',
        'target_unknown_ratio:=0.05',
    ]:
        assert phase7a_token in phase7a_section

    quickstart_idx = readme.index('## 快速开始')
    phase14_replay_idx = readme.index('使用当前推荐稳定地图做静态导航回放')
    phase6_replay_idx = readme.index('使用 Phase 6 上一版稳定地图做静态导航回放')
    phase7a_idx = readme.index('Phase 7A 激进实验参数')
    assert phase14_replay_idx < phase6_replay_idx < phase7a_idx

    phase14_replay_section = readme[phase14_replay_idx: phase6_replay_idx]
    for required in [
        'ros2 launch tugbot_bringup tugbot_nav_phase14_map.launch.py',
        'headless:=false',
        'use_rviz:=true',
        'src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml',
        'map_server + AMCL + Nav2',
        '不启动 slam_toolbox',
        '不启动 frontier_explorer',
        '不进行重新建图',
        '2D Pose Estimate',
        '2D Goal Pose',
        'NavigateToPose',
        '推荐稳定地图回放导航',
        '不是未知地图探索',
    ]:
        assert required in phase14_replay_section

    phase6_replay_section = readme[phase6_replay_idx: phase7a_idx]
    for required in [
        'ros2 launch tugbot_bringup tugbot_nav_phase6_map.launch.py',
        'Phase 6 cleanup map',
        '上一版稳定地图',
    ]:
        assert required in phase6_replay_section

    overview_idx = readme.index('## 启动入口总览')
    quickstart_section = readme[quickstart_idx: phase7a_idx]
    overview_section = readme[overview_idx: quickstart_idx]
    for required in [
        '`tugbot_nav_phase14_map.launch.py`',
        '当前推荐静态回放入口',
        'Phase 14 推荐地图静态导航回放',
        'Phase 14 perimeter no-spin map',
        'AMCL',
        '加载当前推荐地图，适合验证最终推荐地图的静态导航能力',
        '新增 `tugbot_nav_phase14_map.launch.py` 用于加载当前推荐 Phase 14 map',
    ]:
        assert required in overview_section
    assert '当前已有静态回放入口 `tugbot_nav_phase6_map.launch.py` 仍加载 Phase 6 cleanup map' in overview_section

    for required in [
        'ros2 launch tugbot_bringup tugbot_nav_phase14_map.launch.py',
        '当前推荐静态回放入口',
        'src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml',
        '不启动 slam_toolbox',
        '不启动 frontier_explorer',
    ]:
        assert required in readme

    recommended_quickstart = readme[quickstart_idx: phase7a_idx]
    for phase7a_token in [
        'max_goals:=25',
        'max_cleanup_goals:=12',
        'cleanup_search_radius_max_m:=3.0',
        'cleanup_min_obstacle_distance_m:=0.30',
        'target_unknown_ratio:=0.05',
    ]:
        assert phase7a_token not in recommended_quickstart

    for line in readme.splitlines():
        if 'map_save_path:=' in line:
            assert 'tugbot_nav_world_slam_phase6_cleanup' not in line


def test_tugbot_model_contract():
    text = (ROOT / 'src' / 'tugbot_description' / 'models' / 'tugbot' / 'model.sdf').read_text()
    assert '<left_joint>wheel_left_joint</left_joint>' in text
    assert '<right_joint>wheel_right_joint</right_joint>' in text
    assert '<topic>/cmd_vel</topic>' in text
    assert '<odom_topic>/odom</odom_topic>' in text
    assert '<tf_topic>/tf</tf_topic>' in text
    assert '<child_frame_id>base_link</child_frame_id>' in text
    assert '<sensor name="scan_omni" type="gpu_lidar">' in text
    assert '<topic>/scan</topic>' in text
    assert '<frame_id>scan_omni</frame_id>' not in text
    assert '<pose>0 0 0.2 0 0 0</pose>' not in text
    launch = (ROOT / 'src' / 'tugbot_gazebo' / 'launch' / 'tugbot_gazebo.launch.py').read_text()
    assert 'static_transform_publisher' in launch
    assert "'--frame-id', 'base_link'" in launch
    assert "'--child-frame-id', 'tugbot/scan_omni/scan_omni'" in launch


def test_phase12_box_pillar_obstacles_replace_cones_contract():
    world_path = ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_nav_world.sdf'
    world_text = world_path.read_text()
    world = ET.fromstring(world_text)
    models = {model.get('name'): model for model in world.findall('.//world/model')}

    # Phase 12: cone-shaped navigation obstacles are intentionally removed.
    assert 'cone' not in models
    assert 'cone_0' not in models
    assert '<model name=\'cone\'>' not in world_text
    assert '<model name=\'cone_0\'>' not in world_text
    assert world.find('.//world/model/link/collision/geometry/cone') is None

    expected_pillars = {
        'box_pillar_0': '3.805579848063759 -2.5960888656381456 0.6',
        'box_pillar_1': '6.0753818100601649 2.0667701473945264 0.6',
    }
    assert set(expected_pillars).issubset(models)

    for name, expected_pose_prefix in expected_pillars.items():
        model = models[name]
        assert (model.findtext('pose') or '').startswith(expected_pose_prefix)
        assert (model.findtext('static') or '').strip().lower() == 'true'
        assert (model.findtext('self_collide') or '').strip().lower() == 'false'

        link = model.find("link[@name='box_pillar_link']")
        assert link is not None
        collision_box = link.find("collision[@name='box_pillar_collision']/geometry/box/size")
        visual_box = link.find("visual[@name='box_pillar_visual']/geometry/box/size")
        assert collision_box is not None
        assert visual_box is not None
        assert (collision_box.text or '').strip() == '0.50 0.50 1.20'
        assert (visual_box.text or '').strip() == '0.50 0.50 1.20'
        assert link.find("collision[@name='box_pillar_collision']/geometry/cone") is None
        assert link.find("visual[@name='box_pillar_visual']/geometry/cone") is None


def test_phase11_camera_visualization_contract():
    model_text = (ROOT / 'src' / 'tugbot_description' / 'models' / 'tugbot' / 'model.sdf').read_text()
    model = ET.fromstring(model_text)

    camera_link = model.find(".//model/link[@name='camera_link']")
    assert camera_link is not None
    camera_sensor = camera_link.find("sensor[@name='color']")
    assert camera_sensor is not None
    assert camera_sensor.get('type') == 'camera'
    assert (camera_sensor.findtext('topic') or '').strip() == '/camera/image_raw'
    assert camera_sensor.find('gz_frame_id') is None
    assert (camera_sensor.findtext('.//optical_frame_id') or '').strip() == 'camera_optical_frame'
    assert (camera_sensor.findtext('update_rate') or '').strip() in {'15', '15.0', '30', '30.0'}
    assert (camera_sensor.findtext('.//width') or '').strip() == '640'
    assert (camera_sensor.findtext('.//height') or '').strip() == '480'
    assert (camera_sensor.findtext('.//horizontal_fov') or '').strip().startswith('1.047')
    assert (camera_sensor.findtext('.//near') or '').strip() == '0.05'
    assert (camera_sensor.findtext('.//far') or '').strip() == '20.0'

    camera_joint = model.find(".//model/joint[@name='camera_link_joint']")
    assert camera_joint is not None
    assert camera_joint.get('type') == 'fixed'
    assert (camera_joint.findtext('parent') or '').strip() == 'base_link'
    assert (camera_joint.findtext('child') or '').strip() == 'camera_link'

    gazebo_launch = (ROOT / 'src' / 'tugbot_gazebo' / 'launch' / 'tugbot_gazebo.launch.py').read_text()
    assert 'camera_link_static_tf' in gazebo_launch
    assert "'--frame-id', 'base_link'" in gazebo_launch
    assert "'--child-frame-id', 'camera_link'" in gazebo_launch
    assert 'camera_optical_frame_static_tf' in gazebo_launch
    assert "'--child-frame-id', 'camera_optical_frame'" in gazebo_launch

    bridge = (ROOT / 'src' / 'tugbot_gazebo' / 'config' / 'tugbot_bridge.yaml').read_text()
    for required in [
        'ros_topic_name: "/camera/image_raw"',
        'gz_topic_name: "/camera/image_raw"',
        'sensor_msgs/msg/Image',
        'gz.msgs.Image',
        'ros_topic_name: "/camera/camera_info"',
        'sensor_msgs/msg/CameraInfo',
        'gz.msgs.CameraInfo',
    ]:
        assert required in bridge

    rviz = (ROOT / 'src' / 'tugbot_bringup' / 'rviz' / 'tugbot_nav.rviz').read_text()
    assert 'rviz_default_plugins/Image' in rviz
    assert 'Value: /camera/image_raw' in rviz

    readme = (ROOT / 'README.md').read_text()
    for required in [
        'Camera 可视化',
        '/camera/image_raw',
        '/camera/camera_info',
        'Image Display',
        'camera 仅用于可视化，不参与当前 Nav2 避障',
        'Nav2 当前仍主要依赖 `/scan` 进行避障',
    ]:
        assert required in readme


def test_bridge_and_nav_contract():
    bridge = (ROOT / 'src' / 'tugbot_gazebo' / 'config' / 'tugbot_bridge.yaml').read_text()
    for topic in ['/clock', '/cmd_vel', '/odom', '/scan', '/tf']:
        assert topic in bridge
    params = (ROOT / 'src' / 'tugbot_navigation' / 'config' / 'nav2_params.yaml').read_text()
    assert 'base_frame_id: "base_link"' in params
    assert 'odom_frame_id: "odom"' in params
    assert 'global_frame_id: "map"' in params
    assert 'odom_topic: /odom' in params
    assert 'topic: /scan' in params
    assert 'robot_radius: 0.35' in params


def test_slam_online_mapping_contract():
    slam_params = (ROOT / 'src' / 'tugbot_navigation' / 'config' / 'slam_toolbox_params.yaml').read_text()
    for required in [
        'use_sim_time: true',
        'mode: mapping',
        'map_frame: map',
        'odom_frame: odom',
        'base_frame: base_link',
        'scan_topic: /scan',
    ]:
        assert required in slam_params

    slam_launch = (ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_slam.launch.py').read_text()
    assert 'tugbot_gazebo.launch.py' in slam_launch
    assert 'online_async_launch.py' in slam_launch
    assert 'slam_toolbox_params.yaml' in slam_launch
    assert 'bringup_launch.py' not in slam_launch
    assert 'map_1725111373.yaml' not in slam_launch
    assert 'nav2_bringup' not in slam_launch


def test_slam_nav_manual_goal_contract():
    slam_nav_launch = (ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_slam_nav.launch.py').read_text()
    assert 'tugbot_gazebo.launch.py' in slam_nav_launch
    assert 'online_async_launch.py' in slam_nav_launch
    assert 'navigation_launch.py' in slam_nav_launch
    assert 'nav2_slam_params.yaml' in slam_nav_launch
    assert 'slam_toolbox_params.yaml' in slam_nav_launch
    assert 'bringup_launch.py' not in slam_nav_launch
    assert 'map_1725111373.yaml' not in slam_nav_launch
    assert "'map':" not in slam_nav_launch

    nav2_slam_params = (ROOT / 'src' / 'tugbot_navigation' / 'config' / 'nav2_slam_params.yaml').read_text()
    assert 'amcl:' not in nav2_slam_params
    assert 'map_server:' not in nav2_slam_params
    assert 'yaml_filename' not in nav2_slam_params
    assert 'bt_navigator:' in nav2_slam_params
    assert 'planner_server:' in nav2_slam_params
    assert 'controller_server:' in nav2_slam_params
    assert 'global_frame: map' in nav2_slam_params
    assert 'robot_base_frame: base_link' in nav2_slam_params
    assert 'odom_topic: /odom' in nav2_slam_params
    assert 'topic: /scan' in nav2_slam_params
    assert 'static_layer:' in nav2_slam_params
    assert 'map_subscribe_transient_local: True' in nav2_slam_params
