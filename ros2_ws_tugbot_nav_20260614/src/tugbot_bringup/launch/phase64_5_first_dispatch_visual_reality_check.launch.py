from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Phase64.5 phase64_5_first_dispatch_visual_reality_check: visual/manual overlay only.
    # It launches the active scaled2x SLAM+Nav2 visible baseline and a MarkerArray
    # publisher. It avoids the autonomous explorer, sends no goals, and keeps
    # projection and dispatch behavior unchanged.
    world_sdf = LaunchConfiguration('world_sdf')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    phase64_artifact = LaunchConfiguration('phase64_artifact')
    marker_topic = LaunchConfiguration('marker_topic')

    bringup_share = FindPackageShare('tugbot_bringup')
    maze_share = FindPackageShare('tugbot_maze')
    nav_share = FindPackageShare('tugbot_navigation')
    description_share = FindPackageShare('tugbot_description')
    existing_gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    model_resource_path = PathJoinSubstitution([description_share, 'models'])
    combined_gz_resource_path = [
        model_resource_path,
        os.pathsep,
        existing_gz_resource_path,
    ] if existing_gz_resource_path else model_resource_path

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', combined_gz_resource_path),
        DeclareLaunchArgument(
            'world_sdf',
            default_value=PathJoinSubstitution([
                FindPackageShare('tugbot_gazebo'),
                'worlds',
                'tugbot_maze_world_20260528_clean_scaled2x.sdf',
            ]),
            description='Active scaled2x maze world for Phase64.5 visual check.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([nav_share, 'config', 'nav2_slam_params.yaml']),
            description='Active Nav2 params; this launch only reads/uses existing config.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([nav_share, 'config', 'slam_toolbox_params.yaml']),
            description='Existing slam_toolbox params for visible manual comparison.',
        ),
        DeclareLaunchArgument(
            'phase64_artifact',
            default_value=PathJoinSubstitution([
                maze_share,
                '..',
                '..',
                '..',
                '..',
                'log',
                'phase64_corridor_width_robot_footprint_feasibility_decision',
                'phase64_corridor_width_robot_footprint_feasibility_decision.json',
            ]),
            description='Phase64 static geometry artifact used by overlay marker node.',
        ),
        DeclareLaunchArgument(
            'marker_topic',
            default_value='/phase64_5/first_dispatch_visual_markers',
            description='MarkerArray topic for first dispatch target / radius / inflation / corridor helpers.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup_share,
                'launch',
                'tugbot_maze_slam_nav.launch.py',
            ])),
            launch_arguments={
                'world_sdf': world_sdf,
                'params_file': params_file,
                'slam_params_file': slam_params_file,
                'headless': 'false',
                'use_rviz': 'true',
                'use_sim_time': 'true',
                'autostart': 'true',
            }.items(),
        ),
        Node(
            package='tugbot_maze',
            executable='phase64_5_first_dispatch_visual_overlay',
            name='phase64_5_first_dispatch_visual_overlay',
            output='screen',
            parameters=[{
                'phase64_artifact': phase64_artifact,
                'marker_topic': marker_topic,
                'publish_period_sec': 1.0,
                'use_sim_time': True,
            }],
        ),
    ])
