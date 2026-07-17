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
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    default_world = os.path.join(gazebo_share, 'worlds', 'tugbot_nav_world.sdf')
    default_slam_params = os.path.join(navigation_share, 'config', 'slam_toolbox_params.yaml')
    default_rviz = os.path.join(bringup_share, 'rviz', 'tugbot_nav.rviz')

    world_sdf = LaunchConfiguration('world_sdf')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'tugbot_gazebo.launch.py')
        ),
        launch_arguments={
            'world_sdf': world_sdf,
            'use_sim_time': use_sim_time,
            'headless': headless,
        }.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_lifecycle_manager': 'false',
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
        DeclareLaunchArgument('world_sdf', default_value=default_world, description='Gazebo world SDF.'),
        DeclareLaunchArgument('slam_params_file', default_value=default_slam_params, description='slam_toolbox params YAML.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz, description='RViz config.'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time.'),
        DeclareLaunchArgument('autostart', default_value='true', description='Autostart slam_toolbox lifecycle.'),
        DeclareLaunchArgument('headless', default_value='false', description='Open Gazebo GUI by default.'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz.'),
        gazebo_launch,
        TimerAction(period=3.0, actions=[slam_launch]),
        TimerAction(period=6.0, actions=[rviz]),
    ])
