"""Go2 leg physics, perception and the online maze solver."""
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def camera_spawn_arguments(model_path):
    # create supplies an identity EntityFactory pose unless we pass these flags;
    # its default would discard the SDF model's overhead position/orientation.
    model = ET.parse(model_path).getroot().find('model')
    pose = model.findtext('pose', '0 0 0 0 0 0').split()
    args = ['-name', model.get('name'), '-file', str(model_path)]
    for flag, value in zip(('-x', '-y', '-z', '-R', '-P', '-Y'), pose):
        args.extend([flag, value])
    return args


def generate_launch_description():
    share = Path(get_package_share_directory('go2_maze_bringup'))
    description = Path(get_package_share_directory('tugbot_description'))
    gazebo = Path(get_package_share_directory('tugbot_gazebo'))
    go2 = Path(get_package_share_directory('go2_description'))
    sim = Path(get_package_share_directory('ros_gz_sim'))
    observers = Path(get_package_share_directory('go2_observer_cameras'))
    observer_lib = Path(get_package_prefix('go2_observer_cameras')) / 'lib'
    robot = ET.fromstring(xacro.process_file(str(go2 / 'urdf/go2.urdf.xacro')).toxml())
    names = {}
    for old, new in (('fl', 'LF'), ('fr', 'RF'), ('rl', 'LH'), ('rr', 'RH')):
        for source, target in (('hip', 'HAA'), ('thigh', 'HFE'), ('knee', 'KFE')):
            names[f'{old}_{source}'] = f'{new}_{target}'
    for joint in robot.findall('joint'):
        if joint.get('name') in names:
            joint.set('name', names[joint.get('name')])
    robot_description = ET.tostring(robot, encoding='unicode')
    use_sim_time = {'use_sim_time': True}
    rviz_environment = {
        'QT_QPA_PLATFORM': 'xcb', 'QT_SCREEN_SCALE_FACTORS': '1', 'QT_SCALE_FACTOR': '1',
        'QT_ENABLE_HIGHDPI_SCALING': '0', 'QT_FONT_DPI': '96',
        '__NV_PRIME_RENDER_OFFLOAD': '0', '__GLX_VENDOR_LIBRARY_NAME': 'mesa',
    }
    solver = Node(
        package='tugbot_maze', executable='go2_maze_solver', output='screen',
        condition=IfCondition(LaunchConfiguration('autonomous')),
        parameters=[use_sim_time, {
            'pose_source': 'online_slam',
            'entrance_x': 11.011281, 'entrance_y': 9.025070, 'entrance_yaw': 0.0,
            'cruise_v': 0.22, 'hop_timeout_s': 40.0,
            'startup_delay_sec': 3.0, 'entry_direct_distance_m': 2.0,
            'exit_x': 21.072562, 'exit_y': 18.083566, 'exit_radius': 1.2,
            'junction_log_dir': LaunchConfiguration('run_dir'),
        }],
        remappings=[('/cmd_vel_nav', '/cmd_vel')],
    )
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('observer_cameras', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('autonomous', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('world', default_value=str(gazebo / 'worlds/go2_maze.sdf')),
        DeclareLaunchArgument('run_dir', default_value='/tmp/go2-maze-run'),
        SetEnvironmentVariable('QT_SCALE_FACTOR', '1'),
        SetEnvironmentVariable('QT_SCREEN_SCALE_FACTORS', '1'),
        SetEnvironmentVariable('QT_FONT_DPI', '96'),
        SetEnvironmentVariable('QT_ENABLE_HIGHDPI_SCALING', '0'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join([
            str(description / 'models'), str(observers / 'models'), os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ])),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', os.pathsep.join([
            str(observer_lib), os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
        ])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim / 'launch/gz_sim.launch.py')),
            launch_arguments={
                'gz_args': ['-r -v 3 ', IfElseSubstitution(LaunchConfiguration('gui'), '', '-s '),
                            LaunchConfiguration('world')],
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': str(share / 'config/bridge.yaml')}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[use_sim_time, {'robot_description': robot_description}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='go2_lidar_tf', arguments=['--x', '0', '--y', '0', '--z', '0.25',
                 '--frame-id', 'base_link', '--child-frame-id', 'go2/base_link/lidar_3d'],
             parameters=[use_sim_time]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='go2_camera_tf', arguments=['--x', '0.35', '--y', '0', '--z', '0.05',
                 '--frame-id', 'base_link', '--child-frame-id', 'go2/base_link/camera_front'],
             parameters=[use_sim_time]),
        Node(package='tugbot_maze', executable='scan_slice_projector', output='screen',
             parameters=[use_sim_time]),
        Node(package='tugbot_maze', executable='locomotion_controller', output='screen',
             parameters=[use_sim_time, {'model_name': 'go2', 'fall_z_m': 0.22}]),
        TimerAction(period=5.0, actions=[solver]),
        Node(package='tugbot_maze', executable='go2_maze_audit', output='screen',
             condition=IfCondition(LaunchConfiguration('autonomous')),
             parameters=[use_sim_time, {'run_dir': LaunchConfiguration('run_dir')}]),
        Node(package='tugbot_maze', executable='cloud_map_accumulator', output='screen',
             condition=IfCondition(LaunchConfiguration('autonomous')),
             parameters=[use_sim_time]),
        Node(package='tugbot_maze', executable='maze_goal_monitor', output='screen',
             condition=IfCondition(LaunchConfiguration('autonomous')),
             parameters=[use_sim_time, {'exit_x': 21.072562, 'exit_y': 18.083566, 'exit_radius': 1.2}]),
        GroupAction(condition=IfCondition(LaunchConfiguration('observer_cameras')), actions=[
            Node(package='ros_gz_sim', executable='create', name='chase_camera_spawner',
                 output='screen', arguments=camera_spawn_arguments(
                     observers / 'models/camera_chase/model.sdf')),
            Node(package='ros_gz_sim', executable='create', name='overview_camera_spawner',
                 output='screen', arguments=camera_spawn_arguments(
                     observers / 'models/camera_overview/model.sdf')),
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name='observer_camera_bridge', output='screen',
                 parameters=[{'config_file': str(share / 'config/observer_bridge.yaml')}]),
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name='overview_camera_tf', parameters=[use_sim_time],
                 arguments=['--x', '0', '--y', '0', '--z', '24',
                            '--qx', '1', '--qy', '0', '--qz', '0', '--qw', '0',
                            '--frame-id', 'odom', '--child-frame-id', 'camera_overview_optical']),
        ]),
        Node(package='rviz2', executable='rviz2', name='go2_maze_rviz', output='log',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', IfElseSubstitution(LaunchConfiguration('observer_cameras'),
                 str(share / 'config/three_cameras.rviz'), str(share / 'config/maze.rviz'))],
             parameters=[use_sim_time], additional_env=rviz_environment),
    ])
