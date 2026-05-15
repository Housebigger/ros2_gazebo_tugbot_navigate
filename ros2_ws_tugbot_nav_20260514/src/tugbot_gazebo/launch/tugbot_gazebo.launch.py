import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('tugbot_description')
    gazebo_share = get_package_share_directory('tugbot_gazebo')

    default_world = os.path.join(gazebo_share, 'worlds', 'tugbot_empty_world.sdf')
    bridge_config = os.path.join(gazebo_share, 'config', 'tugbot_bridge.yaml')
    model_resource_path = os.path.join(description_share, 'models')

    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    combined_resource_path = model_resource_path if not existing_resource_path else model_resource_path + os.pathsep + existing_resource_path

    world_sdf = LaunchConfiguration('world_sdf')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    gui = PythonExpression(["'", headless, "'.lower() not in ['true', '1', 'yes']"])

    world_arg = DeclareLaunchArgument(
        'world_sdf',
        default_value=default_world,
        description='Absolute path to Gazebo world SDF.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock.',
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run Gazebo server only by default. Set false to open GUI.',
    )

    gazebo_resource_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=combined_resource_path,
    )

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[('gz_args', ['-s -r ', world_sdf])],
        condition=IfCondition(headless),
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[('gz_args', ['-r ', world_sdf])],
        condition=IfCondition(gui),
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_bridge'), 'launch', 'ros_gz_bridge.launch.py')
        ),
        launch_arguments={'bridge_name': 'ros_gz_bridge', 'config_file': bridge_config}.items(),
    )

    scan_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='scan_omni_static_tf',
        arguments=[
            '--x', '-0.1855', '--y', '0.0', '--z', '0.5318',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'tugbot/scan_omni/scan_omni',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    camera_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_static_tf',
        arguments=[
            '--x', '0.30', '--y', '0.0', '--z', '0.55',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_frame_static_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '-1.57079632679', '--pitch', '0.0', '--yaw', '-1.57079632679',
            '--frame-id', 'camera_link', '--child-frame-id', 'camera_optical_frame',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        headless_arg,
        gazebo_resource_env,
        gazebo_headless,
        gazebo_gui,
        bridge,
        scan_tf,
        camera_link_tf,
        camera_optical_tf,
    ])
