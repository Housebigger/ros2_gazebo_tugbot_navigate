import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('tugbot_description')
    gazebo_share = get_package_share_directory('tugbot_gazebo')

    model_path = os.path.join(description_share, 'models')
    default_world = os.path.join(gazebo_share, 'worlds', 'tugbot_lane_world.sdf')
    bridge_config = os.path.join(gazebo_share, 'config', 'bridge.yaml')

    world_sdf = LaunchConfiguration('world_sdf')
    world_sdf_arg = DeclareLaunchArgument(
        'world_sdf',
        default_value=default_world,
        description='Absolute path to the Gazebo world SDF to launch.',
    )

    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    combined_resource_path = model_path if not existing_resource_path else model_path + os.pathsep + existing_resource_path

    gazebo_resource_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=combined_resource_path,
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[('gz_args', [world_sdf, ' -r -v4'])],
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    return LaunchDescription([
        world_sdf_arg,
        gazebo_resource_env,
        gazebo_node,
        bridge_node,
        image_bridge_node,
    ])
