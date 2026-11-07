import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory('tugbot_gazebo')
    navigation_share = get_package_share_directory('tugbot_navigation')
    bringup_share = get_package_share_directory('tugbot_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    # Phase35-pre: default to the human-accepted scaled clean world for the current maze workflow.
    # This launch still starts Nav2, so use only in later navigation phases that explicitly allow it.
    default_world = os.path.join(gazebo_share, 'worlds', 'tugbot_maze_world_20260528_clean_scaled2x.sdf')
    default_slam_params = os.path.join(navigation_share, 'config', 'slam_toolbox_params.yaml')
    default_nav2_params = os.path.join(navigation_share, 'config', 'nav2_slam_params.yaml')
    default_rviz = os.path.join(bringup_share, 'rviz', 'tugbot_nav.rviz')

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

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'tugbot_gazebo.launch.py')),
        launch_arguments={
            'world_sdf': world_sdf,
            'use_sim_time': use_sim_time,
            'headless': headless,
        }.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_lifecycle_manager': 'false',
        }.items(),
    )

    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'log_level': log_level,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_sdf', default_value=default_world, description='Gazebo maze world SDF.'),
        DeclareLaunchArgument('slam_params_file', default_value=default_slam_params, description='slam_toolbox params YAML.'),
        DeclareLaunchArgument('params_file', default_value=default_nav2_params, description='Nav2 params YAML for live SLAM maze navigation.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz, description='RViz config.'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time.'),
        DeclareLaunchArgument('autostart', default_value='true', description='Autostart slam_toolbox and Nav2 lifecycle nodes.'),
        DeclareLaunchArgument('headless', default_value='true', description='Run Gazebo server only by default.'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Start RViz.'),
        DeclareLaunchArgument('use_composition', default_value='False', description='Use Nav2 composed bringup.'),
        DeclareLaunchArgument('use_respawn', default_value='False', description='Respawn Nav2 nodes if they crash.'),
        DeclareLaunchArgument('log_level', default_value='info', description='Nav2 log level.'),
        gazebo_launch,
        TimerAction(period=3.0, actions=[slam_launch]),
        TimerAction(period=7.0, actions=[nav2_navigation_launch]),
        TimerAction(period=10.0, actions=[rviz]),
    ])
