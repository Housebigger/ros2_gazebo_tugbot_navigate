from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Phase67 phase67_goal1_timeout_visual_replay: visible Goal 1 timeout replay
    # / terminal-pose overlay only.  It uses the active scaled2x SLAM+Nav2
    # visible baseline and publishes MarkerArray diagnostics.  Runtime replay is
    # bounded by tools/run_phase67_goal1_timeout_visual_replay.sh with max_goals=1.
    # This launch itself sends no goals and does not change navigation strategy.
    world_sdf = LaunchConfiguration('world_sdf')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    phase67_payload = LaunchConfiguration('phase67_payload')
    marker_topic = LaunchConfiguration('marker_topic')

    bringup_share = FindPackageShare('tugbot_bringup')
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
            description='Active clean scaled2x maze world for Phase67 visible replay.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([nav_share, 'config', 'nav2_slam_params.yaml']),
            description='Existing active Nav2 params; Phase67 does not edit them.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([nav_share, 'config', 'slam_toolbox_params.yaml']),
            description='Existing slam_toolbox params for visible replay.',
        ),
        DeclareLaunchArgument(
            'phase67_payload',
            default_value=PathJoinSubstitution([
                FindPackageShare('tugbot_maze'), '..', '..', '..', '..',
                'log', 'phase67_goal1_timeout_visual_replay',
                'phase67_goal1_timeout_visual_replay.json',
            ]),
            description='Phase67 payload generated from accepted Phase66 Goal 1 timeout artifact.',
        ),
        DeclareLaunchArgument(
            'marker_topic',
            default_value='/phase67/goal1_timeout_visual_markers',
            description='MarkerArray topic for Goal 1 target/trajectory/terminal pose/footprint/front wedge/risk markers.',
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
            executable='phase67_goal1_timeout_visual_overlay',
            name='phase67_goal1_timeout_visual_overlay',
            output='screen',
            parameters=[{
                'phase67_payload': phase67_payload,
                'marker_topic': marker_topic,
                'publish_period_sec': 1.0,
                'use_sim_time': True,
            }],
        ),
    ])
