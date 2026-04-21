import xml.etree.ElementTree as ET
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]


def _xyz(text: str):
    return [float(value) for value in text.split()[:3]]


def test_minimal_model_contract_is_clean_and_camera_only():
    model_path = WORKSPACE_ROOT / 'src' / 'tugbot_description' / 'models' / 'tugbot' / 'model.sdf'
    assert model_path.exists(), 'model.sdf should exist'
    content = model_path.read_text(encoding='utf-8')

    assert '<model name="tugbot">' in content
    assert '<topic>/cmd_vel</topic>' in content
    assert '<topic>/camera/image_raw</topic>' in content
    assert '<camera_info_topic>/camera/camera_info</camera_info_topic>' in content
    assert 'gz::sim::systems::DiffDrive' in content
    assert 'gz::sim::systems::Sensors' in content
    assert '<pose relative_to="base_link">0.28 0 0.44 0 1.05 0</pose>' in content

    forbidden = [
        'camera_back',
        'scan_front',
        'scan_back',
        'scan_omni',
        'imu_link',
        'sensor_contact',
    ]
    for token in forbidden:
        assert token not in content, f'minimal description should not keep legacy sensor structure: {token}'


def _pose_values(text: str):
    return [float(value) for value in text.split()]



def test_formal_world_contract_uses_packaged_model_and_closed_loop_blue_lane_without_debug_target():
    world_path = WORKSPACE_ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_lane_world.sdf'
    assert world_path.exists(), 'tugbot_lane_world.sdf should exist'
    content = world_path.read_text(encoding='utf-8')
    root = ET.fromstring(content)

    assert "<world name='tugbot_lane_world'>" in content or '<world name="tugbot_lane_world">' in content
    assert 'model://tugbot' in content
    assert '<ambient>0.0 0.2 0.98 1</ambient>' in content
    assert 'camera_diagnostic_target' not in content

    track_models = [
        model for model in root.findall('./world/model')
        if (model.get('name') or '').startswith('visual_track_')
    ]
    assert len(track_models) >= 10

    poses = [_pose_values(model.findtext('pose', default='')) for model in track_models]
    sizes = [_pose_values(model.find('./link/visual/geometry/box/size').text) for model in track_models]

    widths = [size[1] for size in sizes]
    heights = [size[2] for size in sizes]
    xs = [pose[0] for pose in poses]
    ys = [pose[1] for pose in poses]
    yaws = [pose[5] for pose in poses]

    assert max(widths) <= 0.22
    assert max(heights) <= 0.02
    assert max(xs) - min(xs) >= 4.5
    assert max(ys) - min(ys) >= 2.4
    assert any(pose[0] < 1.0 and abs(pose[1]) < 0.20 for pose in poses)
    assert any(abs(yaw) < 0.10 for yaw in yaws)
    assert any(abs(abs(yaw) - 3.14159) < 0.25 for yaw in yaws)



def test_debug_world_contract_contains_near_camera_diagnostic_target():
    world_path = WORKSPACE_ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_lane_world_debug.sdf'
    assert world_path.exists(), 'tugbot_lane_world_debug.sdf should exist'
    content = world_path.read_text(encoding='utf-8')

    assert 'camera_diagnostic_target' in content
    assert '<pose>0.55 0.00 0.45 0 0 0</pose>' in content
    assert '<size>0.12 0.12 0.60</size>' in content
    assert '<ambient>0.0 0.2 0.98 1</ambient>' in content



def test_bridge_contract_contains_control_state_and_camera_info_topics():
    bridge_path = WORKSPACE_ROOT / 'src' / 'tugbot_gazebo' / 'config' / 'bridge.yaml'
    assert bridge_path.exists(), 'bridge.yaml should exist'
    content = bridge_path.read_text(encoding='utf-8')

    assert '/cmd_vel' in content
    assert '/odom' in content
    assert '/camera/camera_info' in content
    assert 'ROS_TO_GZ' in content
    assert 'GZ_TO_ROS' in content


def test_launch_layering_contracts_are_respected():
    bringup_root = WORKSPACE_ROOT / 'src' / 'tugbot_bringup'
    sim_minimal = (bringup_root / 'launch' / 'sim_minimal.launch.py').read_text(encoding='utf-8')
    full_system = (bringup_root / 'launch' / 'full_system.launch.py').read_text(encoding='utf-8')
    perception_debug = (bringup_root / 'launch' / 'perception_debug.launch.py').read_text(encoding='utf-8')

    assert 'ros_gz_sim' in sim_minimal
    assert 'ros_gz_bridge' in sim_minimal
    assert 'ros_gz_image' in sim_minimal
    assert 'DeclareLaunchArgument' in sim_minimal
    assert 'LaunchConfiguration' in sim_minimal
    assert 'world_sdf' in sim_minimal
    assert 'tugbot_lane_world.sdf' in sim_minimal
    assert 'lane_detector_node' not in sim_minimal
    assert 'lane_controller_node' not in sim_minimal

    assert 'sim_minimal.launch.py' in full_system
    assert 'lane_detector_node' in full_system
    assert 'lane_controller_node' in full_system
    assert 'world_sdf' in full_system
    assert 'tugbot_lane_world.sdf' in full_system

    assert 'sim_minimal.launch.py' in perception_debug
    assert 'lane_detector_node' in perception_debug
    assert 'lane_controller_node' not in perception_debug
    assert "package='rviz2'" in perception_debug
    assert 'world_sdf' in perception_debug
    assert 'tugbot_lane_world_debug.sdf' in perception_debug


def test_parameter_files_expose_perception_and_control_knobs():
    perception_cfg = (WORKSPACE_ROOT / 'src' / 'tugbot_perception' / 'config' / 'perception.yaml').read_text(encoding='utf-8')
    control_cfg = (WORKSPACE_ROOT / 'src' / 'tugbot_control' / 'config' / 'control.yaml').read_text(encoding='utf-8')

    for token in ['blue_threshold', 'blue_margin', 'crop_top_ratio', 'debug_image_enabled']:
        assert token in perception_cfg

    assert 'blue_threshold: 80' in perception_cfg
    assert 'blue_margin: 20' in perception_cfg
    assert 'crop_top_ratio: 0.45' in perception_cfg
    assert 'detection_hold_frames: 3' in perception_cfg

    for token in ['kp', 'ki', 'kd', 'base_linear_speed', 'min_linear_speed', 'max_angular_speed']:
        assert token in control_cfg
