"""Contract: the mr_buggy3 kinematic chassis. Recipe ported from the archived
anymal kinematic phase (20260716): gravity off, ZERO collision bodies (collision
truth lives in the offline footprint oracle), VelocityControl on /cmd_vel,
world-anchored OdometryPublisher with the base_link TF-label trick; 16-beam
lidar centered (SCAN_OFFSET_X=0) with specs field-identical to the dog's."""
import re
from pathlib import Path


def _ws_src():
    return Path(__file__).resolve().parents[2]


def _buggy_sdf():
    return (_ws_src() / 'tugbot_description' / 'models' / 'mr_buggy3' / 'model.sdf').read_text()


def _dog_sdf():
    return (_ws_src() / 'tugbot_description' / 'models' / 'anymal_c' / 'model.sdf').read_text()


def _block(text, pattern):
    m = re.search(pattern, text, re.S)
    assert m is not None, f'pattern not found: {pattern}'
    return m.group(1)


def test_model_named_mr_buggy3():
    sdf = _buggy_sdf()
    assert '<model name="mr_buggy3">' in sdf     # lowercase, no dash, no leading digit (ROS token rules)
    assert '<static>false</static>' in sdf       # VelocityControl needs a dynamic model


def test_zero_collision_bodies():
    assert '<collision' not in _buggy_sdf()      # kinematic recipe: oracle owns collision truth


def test_gravity_off_on_every_link():
    sdf = _buggy_sdf()
    links = re.findall(r'<link name="[^"]+">', sdf)
    assert len(links) == 1                       # single-link kinematic chassis
    assert sdf.count('<gravity>false</gravity>') == 1


def test_velocity_control_plugin():
    sdf = _buggy_sdf()
    assert 'gz-sim-velocity-control-system' in sdf
    p = _block(sdf, r'name="gz::sim::systems::VelocityControl">(.*?)</plugin>')
    assert '<topic>/cmd_vel</topic>' in p


def test_odometry_publisher_contract():
    p = _block(_buggy_sdf(), r'name="gz::sim::systems::OdometryPublisher">(.*?)</plugin>')
    assert '<odom_frame>odom</odom_frame>' in p
    assert '<robot_base_frame>base_link</robot_base_frame>' in p
    assert '<odom_topic>/odom</odom_topic>' in p
    assert '<tf_topic>/tf</tf_topic>' in p
    assert '<odom_publish_frequency>30</odom_publish_frequency>' in p
    assert '<dimensions>3</dimensions>' in p
    assert 'xyz_offset' not in p                 # archived trap: body-frame post-multiply teleports


def test_no_ackermann_no_ignition_naming():
    sdf = _buggy_sdf()
    assert 'ckermann' not in sdf                 # Ackermann drive removed (kinematic route)
    assert 'ignition' not in sdf                 # Fortress-era plugin names won't load on Harmonic


def test_lidar_centered_specs_match_dog():
    b = _block(_buggy_sdf(), r'<sensor name="lidar_3d" type="gpu_lidar">(.*?)</sensor>')
    d = _block(_dog_sdf(), r'<sensor name="lidar_3d" type="gpu_lidar">(.*?)</sensor>')
    pose = _block(b, r'<pose>([^<]+)</pose>').split()
    assert float(pose[0]) == 0.0 and float(pose[1]) == 0.0   # centered: SCAN_OFFSET_X=0 contract
    assert float(pose[2]) == 0.30                            # mast height (dog was 0.35)
    assert '<topic>/lidar/points</topic>' in b
    bb = re.sub(r'\s+', '', _block(b, r'<lidar>(.*?)</lidar>'))
    dd = re.sub(r'\s+', '', _block(d, r'<lidar>(.*?)</lidar>'))
    assert bb == dd                                          # scan/range/noise field-identical to the dog


def test_camera_front_forward_and_spec():
    b = _block(_buggy_sdf(), r'<sensor name="camera_front" type="camera">(.*?)</sensor>')
    pose = _block(b, r'<pose>([^<]+)</pose>').split()
    assert float(pose[0]) > 0.0 and abs(float(pose[5])) < 0.3    # faces +x travel direction
    assert '<topic>/camera/front/image</topic>' in b
    assert '<width>720</width>' in b and '<height>540</height>' in b


def test_world_spawns_buggy_not_dog():
    world = (_ws_src() / 'tugbot_gazebo' / 'worlds'
             / 'tugbot_maze_world_20260528_clean_scaled2x.sdf').read_text()
    assert 'model://mr_buggy3' in world
    assert 'model://anymal_c' not in world       # dog assets stay in repo but are not spawned
    m = re.search(r'<uri>model://mr_buggy3</uri>\s*<name>mr_buggy3</name>\s*<pose>([^<]+)</pose>', world)
    assert m is not None
    x, y, z = (float(v) for v in m.group(1).split()[:3])
    assert (x, y) == (-11.011, -9.025)           # entrance spawn, same xy as the dog
    assert abs(z - 0.04) < 1e-9                  # wheel radius 0.0365 + clearance (original model's own z)


def test_bridge_has_no_dog_joint_topics():
    bridge = (_ws_src() / 'tugbot_gazebo' / 'config' / 'tugbot_bridge.yaml').read_text()
    assert 'anymal_c/joint' not in bridge        # 12 legged cmd_pos entries removed
    assert '/cmd_vel' in bridge                  # VelocityControl feed stays
    assert '/lidar/points/points' in bridge      # cloud split-topic entry stays


def test_static_tfs_point_at_buggy_frames():
    launch = (_ws_src() / 'tugbot_gazebo' / 'launch' / 'tugbot_gazebo.launch.py').read_text()
    assert 'mr_buggy3/base/lidar_3d' in launch and 'anymal_c/base/lidar_3d' not in launch
    assert 'mr_buggy3/base/camera_front' in launch and 'anymal_c/base/camera_front' not in launch
