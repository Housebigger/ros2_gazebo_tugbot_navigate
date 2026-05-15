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

    default_world = os.path.join(gazebo_share, 'worlds', 'tugbot_nav_world.sdf')
    default_phase14_map = os.path.join(
        navigation_share,
        'maps',
        'explored',
        'tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml',
    )
    default_params = os.path.join(navigation_share, 'config', 'nav2_phase14_map_params.yaml')
    default_rviz = os.path.join(bringup_share, 'rviz', 'tugbot_nav.rviz')

    world_sdf = LaunchConfiguration('world_sdf')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')

    # Includes tugbot_gazebo.launch.py, which starts Gazebo, ros_gz_bridge, and scan_omni_static_tf.
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

    # Static-map replay navigation for Phase 16 intentionally uses Nav2 bringup_launch.py.
    # This starts map_server + AMCL + the Nav2 navigation stack from the Phase 14 recommended map.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
            'slam': 'False',
            'use_localization': 'True',
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
        DeclareLaunchArgument('world_sdf', default_value=default_world, description='Gazebo world SDF for Phase 14 recommended static-map replay.'),
        DeclareLaunchArgument('map', default_value=default_phase14_map, description='Phase 14 recommended static map YAML.'),
        DeclareLaunchArgument('params_file', default_value=default_params, description='Phase 14 static replay Nav2 params YAML, copied from the Phase 6 static replay safety profile for clearer open-source semantics.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz, description='RViz config.'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use Gazebo simulation clock.'),
        DeclareLaunchArgument('autostart', default_value='true', description='Autostart Nav2 and localization lifecycle nodes.'),
        DeclareLaunchArgument('headless', default_value='true', description='Run Gazebo server only by default.'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Start RViz for manual 2D Pose Estimate and goal inspection.'),
        gazebo_launch,
        TimerAction(period=5.0, actions=[nav2_launch]),
        TimerAction(period=8.0, actions=[rviz]),
    ])
