"""Contract: the tugbot-return phase — the ORIGINAL Tugbot (real DiffDrive
physics, full collision set incl. rear gripper) carrying the modern perception
stack. Pins: stock 2D sensors REMOVED (scan_omni published directly on /scan
and would collide with the projection chain — the only legitimate /scan
source), lidar_3d centered (SCAN_OFFSET_X=0 contract) with the dog-identical
<lidar> block, camera_front forward, world-anchored OdometryPublisher owning
/odom + /tf while DiffDrive's wheel-integrated odom (drifts under slip — would
break both aliasing gates' drift-free premise) is redirected to inert topics."""
import re
from pathlib import Path


def _ws_src():
    return Path(__file__).resolve().parents[2]


def _tugbot_sdf():
    return (_ws_src() / 'tugbot_description' / 'models' / 'tugbot' / 'model.sdf').read_text()


def _dog_sdf():
    return (_ws_src() / 'tugbot_description' / 'models' / 'anymal_c' / 'model.sdf').read_text()


def _block(text, pattern):
    m = re.search(pattern, text, re.S)
    assert m is not None, f'pattern not found: {pattern}'
    return m.group(1)


def test_model_is_dynamic_tugbot():
    sdf = _tugbot_sdf()
    assert '<model name="tugbot">' in sdf
    assert '<static>' not in sdf or '<static>false</static>' in sdf
    assert '<gravity>false</gravity>' not in sdf     # real physics everywhere


def test_stock_2d_sensors_removed():
    sdf = _tugbot_sdf()
    # scan_omni published DIRECTLY on /scan -> would collide with the
    # slice-projector-regenerated /scan. Removed wholesale, not muted.
    assert 'scan_omni' not in sdf
    assert 'scan_front' not in sdf
    assert 'scan_back' not in sdf
    assert 'sensor_contact' not in sdf
    assert '<sensor name="color"' not in sdf         # stock camera gone
    assert '/camera/image_raw' not in sdf
    # NOTHING in the model may publish on /scan
    assert '<topic>/scan</topic>' not in sdf


def test_diff_drive_kept_with_odom_silenced():
    p = _block(_tugbot_sdf(), r'name="gz::sim::systems::DiffDrive">(.*?)</plugin>')
    assert '<left_joint>wheel_left_joint</left_joint>' in p
    assert '<right_joint>wheel_right_joint</right_joint>' in p
    assert '<wheel_separation>0.514</wheel_separation>' in p
    assert '<wheel_radius>0.195</wheel_radius>' in p
    assert '<topic>/cmd_vel</topic>' in p
    # Wheel-integrated odom DRIFTS under slip; both aliasing gates snap to
    # /odom on the drift-free premise. DiffDrive keeps only its drive role.
    assert '<odom_topic>/diffdrive/odom_unused</odom_topic>' in p
    assert '<tf_topic>/diffdrive/tf_unused</tf_topic>' in p
    assert '<odom_topic>/odom</odom_topic>' not in p
    assert '<tf_topic>/tf</tf_topic>' not in p


def test_world_anchored_odometry_publisher():
    p = _block(_tugbot_sdf(), r'name="gz::sim::systems::OdometryPublisher">(.*?)</plugin>')
    assert '<odom_frame>odom</odom_frame>' in p
    assert '<robot_base_frame>base_link</robot_base_frame>' in p
    assert '<odom_topic>/odom</odom_topic>' in p
    assert '<tf_topic>/tf</tf_topic>' in p
    assert '<odom_publish_frequency>30</odom_publish_frequency>' in p
    assert '<dimensions>3</dimensions>' in p
    assert 'xyz_offset' not in p                     # archived trap: body-frame post-multiply teleports


def test_lidar_3d_centered_specs_match_dog():
    b = _block(_tugbot_sdf(), r'<sensor name="lidar_3d" type="gpu_lidar">(.*?)</sensor>')
    d = _block(_dog_sdf(), r'<sensor name="lidar_3d" type="gpu_lidar">(.*?)</sensor>')
    pose = _block(b, r'<pose>([^<]+)</pose>').split()
    assert float(pose[0]) == 0.0 and float(pose[1]) == 0.0   # SCAN_OFFSET_X=0 contract
    assert float(pose[2]) == 0.55                            # above the stock pole top (0.5318)
    assert '<topic>/lidar/points</topic>' in b
    bb = re.sub(r'\s+', '', _block(b, r'<lidar>(.*?)</lidar>'))
    dd = re.sub(r'\s+', '', _block(d, r'<lidar>(.*?)</lidar>'))
    assert bb == dd                                          # field-identical to the dog


def test_camera_front_forward_and_spec():
    b = _block(_tugbot_sdf(), r'<sensor name="camera_front" type="camera">(.*?)</sensor>')
    pose = _block(b, r'<pose>([^<]+)</pose>').split()
    assert float(pose[0]) > 0.0 and abs(float(pose[5])) < 0.3    # faces +x travel direction
    assert '<topic>/camera/front/image</topic>' in b
    assert '<width>720</width>' in b and '<height>540</height>' in b


def test_gripper_and_collisions_survive():
    sdf = _tugbot_sdf()
    assert '<link name="gripper">' in sdf            # honest rear overhang stays
    assert '<link name="gripper_hand">' in sdf
    assert sdf.count('<collision') >= 11             # base+gripper+hand set (incl. side pads)+4 wheels


def test_world_spawns_tugbot():
    world = (_ws_src() / 'tugbot_gazebo' / 'worlds'
             / 'tugbot_maze_world_20260528_clean_scaled2x.sdf').read_text()
    assert 'model://tugbot' in world
    assert 'model://mr_buggy3' not in world          # buggy assets stay in repo, not spawned
    m = re.search(r'<uri>model://tugbot</uri>\s*<name>tugbot</name>\s*<pose>([^<]+)</pose>', world)
    assert m is not None
    x, y, z = (float(v) for v in m.group(1).split()[:3])
    assert (x, y) == (-11.011, -9.025)               # entrance spawn, historic original pose
    assert z == 0.0


def test_static_tfs_point_at_tugbot_frames():
    launch = (_ws_src() / 'tugbot_gazebo' / 'launch' / 'tugbot_gazebo.launch.py').read_text()
    assert 'tugbot/base_link/lidar_3d' in launch and 'mr_buggy3/base/lidar_3d' not in launch
    assert 'tugbot/base_link/camera_front' in launch and 'mr_buggy3/base/camera_front' not in launch
    # The TRANSLATIONS must match the SDF mounts (lidar z=0.55, camera 0.20/0.40):
    # a renamed frame with a stale buggy offset would project the cloud ~0.25 m low
    # while every name-only assertion stays green (Task 1 quality-review finding).
    lidar = _block(launch, r"name='lidar_3d_static_tf'.*?arguments=\[(.*?)\]")
    assert "'--z', '0.55'" in lidar
    cam = _block(launch, r"name='camera_front_static_tf'.*?arguments=\[(.*?)\]")
    assert "'--x', '0.20'" in cam and "'--z', '0.40'" in cam


def test_bridge_unchanged_platform_agnostic():
    bridge = (_ws_src() / 'tugbot_gazebo' / 'config' / 'tugbot_bridge.yaml').read_text()
    assert '/cmd_vel' in bridge
    assert '/lidar/points/points' in bridge          # cloud split-topic trap entry stays
    assert '/camera/front/image' in bridge
    assert 'diffdrive' not in bridge                 # inert topics are NOT bridged
