import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('tugbot_bringup')

    world_sdf = LaunchConfiguration('world_sdf')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    map_topic = LaunchConfiguration('map_topic')
    base_frame = LaunchConfiguration('base_frame')
    map_frame = LaunchConfiguration('map_frame')
    action_name = LaunchConfiguration('action_name')
    spin_action_name = LaunchConfiguration('spin_action_name')
    min_cluster_size = LaunchConfiguration('min_cluster_size')
    min_obstacle_distance_m = LaunchConfiguration('min_obstacle_distance_m')
    min_goal_distance_m = LaunchConfiguration('min_goal_distance_m')
    max_goal_distance_m = LaunchConfiguration('max_goal_distance_m')
    relaxed_min_obstacle_distance_m = LaunchConfiguration('relaxed_min_obstacle_distance_m')
    relaxed_max_goal_distance_m = LaunchConfiguration('relaxed_max_goal_distance_m')
    relaxed_min_cluster_size = LaunchConfiguration('relaxed_min_cluster_size')
    max_candidates_per_cluster = LaunchConfiguration('max_candidates_per_cluster')
    goal_timeout_sec = LaunchConfiguration('goal_timeout_sec')
    blacklist_radius_m = LaunchConfiguration('blacklist_radius_m')
    exploration_rate_hz = LaunchConfiguration('exploration_rate_hz')
    finish_no_frontier_cycles = LaunchConfiguration('finish_no_frontier_cycles')
    max_goals = LaunchConfiguration('max_goals')
    min_goals_before_finish = LaunchConfiguration('min_goals_before_finish')
    min_frontier_clusters_before_finish = LaunchConfiguration('min_frontier_clusters_before_finish')
    recovery_scan_attempts = LaunchConfiguration('recovery_scan_attempts')
    relaxed_retry_attempts = LaunchConfiguration('relaxed_retry_attempts')
    map_stable_cycles_before_finish = LaunchConfiguration('map_stable_cycles_before_finish')
    map_change_free_threshold = LaunchConfiguration('map_change_free_threshold')
    map_change_unknown_threshold = LaunchConfiguration('map_change_unknown_threshold')
    enable_recovery_scan = LaunchConfiguration('enable_recovery_scan')
    recovery_spin_angle = LaunchConfiguration('recovery_spin_angle')
    recovery_wait_after_scan_sec = LaunchConfiguration('recovery_wait_after_scan_sec')
    enable_cleanup_mode = LaunchConfiguration('enable_cleanup_mode')
    cleanup_unknown_cluster_min_size = LaunchConfiguration('cleanup_unknown_cluster_min_size')
    cleanup_max_unknown_clusters = LaunchConfiguration('cleanup_max_unknown_clusters')
    cleanup_search_radius_min_m = LaunchConfiguration('cleanup_search_radius_min_m')
    cleanup_search_radius_max_m = LaunchConfiguration('cleanup_search_radius_max_m')
    cleanup_min_obstacle_distance_m = LaunchConfiguration('cleanup_min_obstacle_distance_m')
    cleanup_goal_timeout_sec = LaunchConfiguration('cleanup_goal_timeout_sec')
    cleanup_spin_after_goal = LaunchConfiguration('cleanup_spin_after_goal')
    cleanup_spin_angle = LaunchConfiguration('cleanup_spin_angle')
    cleanup_wait_after_spin_sec = LaunchConfiguration('cleanup_wait_after_spin_sec')
    target_unknown_ratio = LaunchConfiguration('target_unknown_ratio')
    max_cleanup_goals = LaunchConfiguration('max_cleanup_goals')
    save_map = LaunchConfiguration('save_map')
    map_save_path = LaunchConfiguration('map_save_path')
    map_save_timeout_sec = LaunchConfiguration('map_save_timeout_sec')

    slam_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'tugbot_slam_nav.launch.py')),
        launch_arguments={
            'world_sdf': world_sdf,
            'slam_params_file': slam_params_file,
            'params_file': nav2_params_file,
            'rviz_config': rviz_config,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'headless': headless,
            'use_rviz': use_rviz,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'log_level': log_level,
        }.items(),
    )

    frontier_explorer = Node(
        package='tugbot_exploration',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'map_topic': map_topic,
            'base_frame': base_frame,
            'map_frame': map_frame,
            'action_name': action_name,
            'spin_action_name': spin_action_name,
            'min_cluster_size': ParameterValue(min_cluster_size, value_type=int),
            'min_obstacle_distance_m': ParameterValue(min_obstacle_distance_m, value_type=float),
            'min_goal_distance_m': ParameterValue(min_goal_distance_m, value_type=float),
            'max_goal_distance_m': ParameterValue(max_goal_distance_m, value_type=float),
            'relaxed_min_obstacle_distance_m': ParameterValue(relaxed_min_obstacle_distance_m, value_type=float),
            'relaxed_max_goal_distance_m': ParameterValue(relaxed_max_goal_distance_m, value_type=float),
            'relaxed_min_cluster_size': ParameterValue(relaxed_min_cluster_size, value_type=int),
            'max_candidates_per_cluster': ParameterValue(max_candidates_per_cluster, value_type=int),
            'goal_timeout_sec': ParameterValue(goal_timeout_sec, value_type=float),
            'blacklist_radius_m': ParameterValue(blacklist_radius_m, value_type=float),
            'exploration_rate_hz': ParameterValue(exploration_rate_hz, value_type=float),
            'finish_no_frontier_cycles': ParameterValue(finish_no_frontier_cycles, value_type=int),
            'max_goals': ParameterValue(max_goals, value_type=int),
            'min_goals_before_finish': ParameterValue(min_goals_before_finish, value_type=int),
            'min_frontier_clusters_before_finish': ParameterValue(min_frontier_clusters_before_finish, value_type=int),
            'recovery_scan_attempts': ParameterValue(recovery_scan_attempts, value_type=int),
            'relaxed_retry_attempts': ParameterValue(relaxed_retry_attempts, value_type=int),
            'map_stable_cycles_before_finish': ParameterValue(map_stable_cycles_before_finish, value_type=int),
            'map_change_free_threshold': ParameterValue(map_change_free_threshold, value_type=int),
            'map_change_unknown_threshold': ParameterValue(map_change_unknown_threshold, value_type=int),
            'enable_recovery_scan': ParameterValue(enable_recovery_scan, value_type=bool),
            'recovery_spin_angle': ParameterValue(recovery_spin_angle, value_type=float),
            'recovery_wait_after_scan_sec': ParameterValue(recovery_wait_after_scan_sec, value_type=float),
            'enable_cleanup_mode': ParameterValue(enable_cleanup_mode, value_type=bool),
            'cleanup_unknown_cluster_min_size': ParameterValue(cleanup_unknown_cluster_min_size, value_type=int),
            'cleanup_max_unknown_clusters': ParameterValue(cleanup_max_unknown_clusters, value_type=int),
            'cleanup_search_radius_min_m': ParameterValue(cleanup_search_radius_min_m, value_type=float),
            'cleanup_search_radius_max_m': ParameterValue(cleanup_search_radius_max_m, value_type=float),
            'cleanup_min_obstacle_distance_m': ParameterValue(cleanup_min_obstacle_distance_m, value_type=float),
            'cleanup_goal_timeout_sec': ParameterValue(cleanup_goal_timeout_sec, value_type=float),
            'cleanup_spin_after_goal': ParameterValue(cleanup_spin_after_goal, value_type=bool),
            'cleanup_spin_angle': ParameterValue(cleanup_spin_angle, value_type=float),
            'cleanup_wait_after_spin_sec': ParameterValue(cleanup_wait_after_spin_sec, value_type=float),
            'target_unknown_ratio': ParameterValue(target_unknown_ratio, value_type=float),
            'max_cleanup_goals': ParameterValue(max_cleanup_goals, value_type=int),
            'save_map': ParameterValue(save_map, value_type=bool),
            'map_save_path': map_save_path,
            'map_save_timeout_sec': ParameterValue(map_save_timeout_sec, value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_sdf', default_value=os.path.join(get_package_share_directory('tugbot_gazebo'), 'worlds', 'tugbot_nav_world.sdf'), description='Gazebo world SDF.'),
        DeclareLaunchArgument('slam_params_file', default_value=os.path.join(get_package_share_directory('tugbot_navigation'), 'config', 'slam_toolbox_params.yaml'), description='slam_toolbox params YAML.'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(get_package_share_directory('tugbot_navigation'), 'config', 'nav2_slam_params.yaml'), description='Nav2 params YAML for live SLAM map navigation.'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(bringup_share, 'rviz', 'tugbot_nav.rviz'), description='RViz config.'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time.'),
        DeclareLaunchArgument('autostart', default_value='true', description='Autostart slam_toolbox and Nav2 lifecycle nodes.'),
        DeclareLaunchArgument('headless', default_value='false', description='Open Gazebo GUI by default.'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument('use_composition', default_value='False', description='Use Nav2 composed bringup.'),
        DeclareLaunchArgument('use_respawn', default_value='False', description='Respawn Nav2 nodes if they crash.'),
        DeclareLaunchArgument('log_level', default_value='info', description='Nav2 log level.'),
        DeclareLaunchArgument('map_topic', default_value='/map', description='OccupancyGrid topic from slam_toolbox.'),
        DeclareLaunchArgument('base_frame', default_value='base_link', description='Robot base TF frame.'),
        DeclareLaunchArgument('map_frame', default_value='map', description='Map TF frame.'),
        DeclareLaunchArgument('action_name', default_value='/navigate_to_pose', description='Nav2 NavigateToPose action name.'),
        DeclareLaunchArgument('spin_action_name', default_value='/spin', description='Nav2 Spin action name used for recovery scan.'),
        DeclareLaunchArgument('min_cluster_size', default_value='5', description='Minimum frontier cluster size in cells.'),
        DeclareLaunchArgument('min_obstacle_distance_m', default_value='0.45', description='Minimum clearance from occupied cells.'),
        DeclareLaunchArgument('min_goal_distance_m', default_value='0.5', description='Minimum goal distance from robot.'),
        DeclareLaunchArgument('max_goal_distance_m', default_value='4.0', description='Maximum goal distance from robot.'),
        DeclareLaunchArgument('relaxed_min_obstacle_distance_m', default_value='0.35', description='Relaxed search minimum clearance from occupied cells.'),
        DeclareLaunchArgument('relaxed_max_goal_distance_m', default_value='8.0', description='Relaxed search maximum goal distance from robot.'),
        DeclareLaunchArgument('relaxed_min_cluster_size', default_value='3', description='Relaxed search minimum frontier cluster size in cells.'),
        DeclareLaunchArgument('max_candidates_per_cluster', default_value='3', description='Maximum safe free-cell candidate goals generated per frontier cluster.'),
        DeclareLaunchArgument('goal_timeout_sec', default_value='60.0', description='Goal timeout in seconds.'),
        DeclareLaunchArgument('blacklist_radius_m', default_value='0.5', description='Radius around failed goals to avoid.'),
        DeclareLaunchArgument('exploration_rate_hz', default_value='0.5', description='Explorer decision rate.'),
        DeclareLaunchArgument('finish_no_frontier_cycles', default_value='5', description='No-frontier cycles before declaring complete.'),
        DeclareLaunchArgument('max_goals', default_value='20', description='Maximum successful goals before stopping.'),
        DeclareLaunchArgument('min_goals_before_finish', default_value='8', description='Minimum completed goals before strict completion may finish, unless raw frontiers are few and map is stable.'),
        DeclareLaunchArgument('min_frontier_clusters_before_finish', default_value='5', description='Raw frontier cluster threshold for near-complete map state.'),
        DeclareLaunchArgument('recovery_scan_attempts', default_value='3', description='Recovery scan attempts before allowing many-frontier no-candidate completion.'),
        DeclareLaunchArgument('relaxed_retry_attempts', default_value='3', description='Relaxed search no-candidate cycles before allowing completion.'),
        DeclareLaunchArgument('map_stable_cycles_before_finish', default_value='3', description='Map stable cycles required before completion.'),
        DeclareLaunchArgument('map_change_free_threshold', default_value='50', description='Maximum free-cell count delta considered stable.'),
        DeclareLaunchArgument('map_change_unknown_threshold', default_value='50', description='Maximum unknown-cell count delta considered stable.'),
        DeclareLaunchArgument('enable_recovery_scan', default_value='true', description='Enable Nav2 Spin recovery scan when raw frontiers remain but no candidate is valid.'),
        DeclareLaunchArgument('recovery_spin_angle', default_value='6.28', description='Recovery spin angle in radians.'),
        DeclareLaunchArgument('recovery_wait_after_scan_sec', default_value='2.0', description='Wait after recovery scan for map updates.'),
        DeclareLaunchArgument('enable_cleanup_mode', default_value='true', description='Enable residual unknown coverage cleanup mode.'),
        DeclareLaunchArgument('cleanup_unknown_cluster_min_size', default_value='30', description='Minimum residual unknown cluster size in cells.'),
        DeclareLaunchArgument('cleanup_max_unknown_clusters', default_value='5', description='Maximum residual unknown clusters considered per cycle.'),
        DeclareLaunchArgument('cleanup_search_radius_min_m', default_value='0.5', description='Minimum observation-goal search radius around unknown cluster center.'),
        DeclareLaunchArgument('cleanup_search_radius_max_m', default_value='2.0', description='Maximum observation-goal search radius around unknown cluster center.'),
        DeclareLaunchArgument('cleanup_min_obstacle_distance_m', default_value='0.35', description='Minimum cleanup goal clearance from occupied cells.'),
        DeclareLaunchArgument('cleanup_goal_timeout_sec', default_value='60.0', description='Cleanup NavigateToPose timeout in seconds.'),
        DeclareLaunchArgument('cleanup_spin_after_goal', default_value='true', description='Run Nav2 Spin after a successful cleanup goal.'),
        DeclareLaunchArgument('cleanup_spin_angle', default_value='6.28', description='Cleanup spin angle in radians.'),
        DeclareLaunchArgument('cleanup_wait_after_spin_sec', default_value='2.0', description='Wait after cleanup spin for map updates.'),
        DeclareLaunchArgument('target_unknown_ratio', default_value='0.03', description='Unknown-cell ratio target before cleanup can stop.'),
        DeclareLaunchArgument('max_cleanup_goals', default_value='5', description='Maximum cleanup observation goals.'),
        DeclareLaunchArgument('save_map', default_value='false', description='Save the live SLAM map after exploration finishes.'),
        DeclareLaunchArgument('map_save_path', default_value='/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_nav_20260514/src/tugbot_navigation/maps/explored/tugbot_nav_world_slam', description='Map save output prefix without .yaml/.pgm suffix.'),
        DeclareLaunchArgument('map_save_timeout_sec', default_value='30.0', description='Timeout for nav2_map_server map_saver_cli.'),
        slam_nav_launch,
        TimerAction(period=13.0, actions=[frontier_explorer]),
    ])
