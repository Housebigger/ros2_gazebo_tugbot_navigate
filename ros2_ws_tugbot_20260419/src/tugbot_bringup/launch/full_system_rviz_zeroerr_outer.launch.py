import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('tugbot_bringup')
    gazebo_share = get_package_share_directory('tugbot_gazebo')
    rviz_config = os.path.join(bringup_share, 'config', 'perception_debug.rviz')
    zeroerr_outer_world = os.path.join(gazebo_share, 'worlds', 'tugbot_lane_world_zeroerr_outer.sdf')

    full_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'full_system.launch.py')),
        launch_arguments=[('world_sdf', zeroerr_outer_world)],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        full_system,
        rviz_node,
    ])
