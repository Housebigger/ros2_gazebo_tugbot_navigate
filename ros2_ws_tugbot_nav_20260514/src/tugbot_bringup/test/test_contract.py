from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

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
        "default_value='6.28'",
        "default_value='0.03'",
    ]:
        assert default in explore_launch or default.replace("'", '"') in explore_launch


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
        'map_save_path:=/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam',
        '/map 由 slam_toolbox 发布',
        'AMCL/map_server 不启动',
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
        'nav2_params.yaml',
        'tugbot_gazebo.launch.py',
        'ros_gz_bridge',
        'scan_omni_static_tf',
        'bringup_launch.py',
        'nav2_bringup',
        "'map': map_yaml",
        "'params_file': params_file",
        "'use_sim_time': use_sim_time",
        "'autostart': autostart",
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
    assert 'online_async_launch.py' not in launch

    nav_launch = (ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_nav.launch.py').read_text()
    slam_launch = (ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_slam.launch.py').read_text()
    slam_nav_launch = (ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_slam_nav.launch.py').read_text()
    explore_launch = (ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_explore.launch.py').read_text()
    assert 'tugbot_nav_world_slam_phase6_cleanup.yaml' not in nav_launch
    assert 'tugbot_nav_world_slam_phase6_cleanup.yaml' not in slam_launch
    assert 'tugbot_nav_world_slam_phase6_cleanup.yaml' not in slam_nav_launch
    assert 'tugbot_nav_world_slam_phase6_cleanup.yaml' not in explore_launch


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
    assert '<frame_id>scan_omni</frame_id>' in text
    assert '<pose>0 0 0.2 0 0 0</pose>' not in text
    launch = (ROOT / 'src' / 'tugbot_gazebo' / 'launch' / 'tugbot_gazebo.launch.py').read_text()
    assert 'static_transform_publisher' in launch
    assert "'--frame-id', 'base_link'" in launch
    assert "'--child-frame-id', 'tugbot/scan_omni/scan_omni'" in launch

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
