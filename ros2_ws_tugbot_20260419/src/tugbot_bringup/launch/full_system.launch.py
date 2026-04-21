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
    control_share = get_package_share_directory('tugbot_control')

    default_world = os.path.join(gazebo_share, 'worlds', 'tugbot_lane_world.sdf')
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

    lane_controller_node = Node(
        package='tugbot_control',
        executable='lane_controller_node',
        parameters=[os.path.join(control_share, 'config', 'control.yaml')],
        output='screen',
    )

    return LaunchDescription([
        world_sdf_arg,
        sim_minimal,
        lane_detector_node,
        lane_controller_node,
    ])
