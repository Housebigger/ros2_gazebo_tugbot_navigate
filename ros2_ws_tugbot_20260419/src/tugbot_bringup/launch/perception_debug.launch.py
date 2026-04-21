import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('tugbot_bringup')
    gazebo_share = get_package_share_directory('tugbot_gazebo')
    perception_share = get_package_share_directory('tugbot_perception')
    rviz_config = os.path.join(bringup_share, 'config', 'perception_debug.rviz')

    default_world = os.path.join(gazebo_share, 'worlds', 'tugbot_lane_world_debug.sdf')
    world_sdf = LaunchConfiguration('world_sdf')
    world_sdf_arg = DeclareLaunchArgument(
        'world_sdf',
        default_value=default_world,
        description='Absolute path to the Gazebo world SDF to launch.',
    )

    sim_minimal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'sim_minimal.launch.py')),
        launch_arguments=[('world_sdf', world_sdf)],
    )

    lane_detector_node = Node(
        package='tugbot_perception',
        executable='lane_detector_node',
        parameters=[os.path.join(perception_share, 'config', 'perception.yaml')],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        world_sdf_arg,
        sim_minimal,
        lane_detector_node,
        rviz_node,
    ])
