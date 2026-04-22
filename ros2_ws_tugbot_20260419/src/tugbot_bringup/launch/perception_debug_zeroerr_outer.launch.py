import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_share = get_package_share_directory('tugbot_bringup')
    gazebo_share = get_package_share_directory('tugbot_gazebo')
    zeroerr_outer_world = os.path.join(gazebo_share, 'worlds', 'tugbot_lane_world_zeroerr_outer.sdf')

    perception_debug = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'perception_debug.launch.py')),
        launch_arguments=[('world_sdf', zeroerr_outer_world)],
    )

    return LaunchDescription([
        perception_debug,
    ])
