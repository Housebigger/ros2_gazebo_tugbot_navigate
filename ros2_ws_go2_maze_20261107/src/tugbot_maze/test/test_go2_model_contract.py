"""Prevent regressions in the converted Go2's native physics and frame wiring."""
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from tugbot_maze.legged.kinematics import JOINTS

WORKSPACE = Path(__file__).resolve().parents[3]
MODEL = WORKSPACE / 'src/tugbot_description/models/go2/model.sdf'


def test_native_joint_controllers_keep_official_effort_limits():
    model = ET.parse(MODEL).getroot().find('model')
    joints = {joint.get('name'): joint for joint in model.findall('joint')}
    assert set(joints) == set(JOINTS)
    controllers = [p for p in model.findall('plugin')
                   if p.get('filename') == 'gz-sim-joint-position-controller-system']
    assert len(controllers) == 12
    assert not any('force' in p.get('filename', '').lower() for p in model.findall('plugin'))
    for controller in controllers:
        name = controller.findtext('joint_name')
        effort = float(joints[name].findtext('axis/limit/effort'))
        assert effort == (45.43 if name.endswith('KFE') else 23.7)
        assert float(controller.findtext('cmd_max')) == effort
        assert float(controller.findtext('cmd_min')) == -effort
        assert controller.findtext('use_velocity_commands', 'false') == 'false'


def test_renamed_joint_frames_and_mesh_resources_resolve():
    model = ET.parse(MODEL).getroot().find('model')
    frames = {node.get('name') for node in model if node.tag in ('link', 'joint', 'frame')}
    frames.add('__model__')
    for node in model.iter():
        for attribute in ('relative_to', 'attached_to', 'expressed_in'):
            assert node.get(attribute) is None or node.get(attribute) in frames
    for uri in model.findall('.//mesh/uri'):
        assert uri.text.startswith('model://go2/dae/')
        assert (MODEL.parent / 'dae' / Path(uri.text).name).is_file()


def test_sensor_and_command_bridge_contract():
    model = ET.parse(MODEL).getroot().find('model')
    lidar = model.find(".//sensor[@name='lidar_3d']")
    assert lidar.findtext('pose') == '0 0 0.25 0 0 0'
    assert lidar.findtext('gz_frame_id') == 'go2/base_link/lidar_3d'
    bridge = yaml.safe_load((WORKSPACE / 'src/go2_maze_bringup/config/bridge.yaml').read_text())
    topics = {entry['ros_topic_name']: entry for entry in bridge}
    assert topics['/lidar/points']['gz_topic_name'] == '/lidar/points/points'
    assert topics['/joint_states']['gz_topic_name'] == '/model/go2/joint_state'
    for name in JOINTS:
        topic = f'/model/go2/joint/{name}/cmd_pos'
        assert topics[topic]['gz_topic_name'] == topic
        assert topics[topic]['direction'] == 'ROS_TO_GZ'


def test_footprint_contains_rear_shank_not_just_body_and_feet():
    # Worst official collider projections found over the current gait domain,
    # stabilizer ±0.06 rad and independent body roll/pitch ±0.1 rad.
    # Previous rear=-0.32 missed the rear knee/shank despite enclosing all feet.
    from tugbot_maze.footprint import inside_footprint
    assert inside_footprint(-0.395255, 0.0, margin=-0.01)
    assert inside_footprint(0.344527, 0.0, margin=-0.01)
    assert inside_footprint(0.0, 0.218049, margin=-0.01)
