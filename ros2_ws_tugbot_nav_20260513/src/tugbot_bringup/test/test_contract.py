from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

def test_expected_packages_exist():
    for pkg in ['tugbot_description', 'tugbot_gazebo', 'tugbot_navigation', 'tugbot_bringup']:
        assert (ROOT / 'src' / pkg / 'package.xml').exists()

def test_tugbot_model_contract():
    text = (ROOT / 'src' / 'tugbot_description' / 'models' / 'tugbot' / 'model.sdf').read_text()
    assert '<left_joint>wheel_left_joint</left_joint>' in text
    assert '<right_joint>wheel_right_joint</right_joint>' in text
    assert '<topic>/cmd_vel</topic>' in text
    assert '<odom_topic>/odom</odom_topic>' in text
    assert '<tf_topic>/tf</tf_topic>' in text
    assert '<child_frame_id>base_link</child_frame_id>' in text
    assert '<sensor name="scan_omni" type="gpu_lidar">' in text
    assert '<topic>/scan</topic>' in text
    assert '<frame_id>scan_omni</frame_id>' in text
    assert '<pose>0 0 0.2 0 0 0</pose>' not in text
    launch = (ROOT / 'src' / 'tugbot_gazebo' / 'launch' / 'tugbot_gazebo.launch.py').read_text()
    assert 'static_transform_publisher' in launch
    assert "'--frame-id', 'base_link', '--child-frame-id', 'scan_omni'" in launch

def test_bridge_and_nav_contract():
    bridge = (ROOT / 'src' / 'tugbot_gazebo' / 'config' / 'tugbot_bridge.yaml').read_text()
    for topic in ['/clock', '/cmd_vel', '/odom', '/scan', '/tf']:
        assert topic in bridge
    params = (ROOT / 'src' / 'tugbot_navigation' / 'config' / 'nav2_params.yaml').read_text()
    assert 'base_frame_id: "base_link"' in params
    assert 'odom_frame_id: "odom"' in params
    assert 'global_frame_id: "map"' in params
    assert 'odom_topic: /odom' in params
    assert 'topic: /scan' in params
    assert 'robot_radius: 0.35' in params
