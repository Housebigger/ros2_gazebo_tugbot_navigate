"""Cross-check real Gazebo camera resources, ROS wiring and RViz image panes."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np
import yaml
import ast
import importlib.util
import pytest

WS = Path(__file__).resolve().parents[3]
OBSERVER = WS / 'src/go2_observer_cameras/models'
BRINGUP = WS / 'src/go2_maze_bringup'


def model(name):
    return ET.parse(OBSERVER / name / 'model.sdf').getroot().find('model')


def test_observer_rigs_are_static_and_cannot_collide_with_the_go2():
    for name in ('camera_chase', 'camera_overview'):
        rig = model(name)
        assert rig.get('name') == name
        assert rig.findtext('static') == 'true'
        assert not rig.findall('.//collision')
        assert not rig.findall('joint')
    plugin = model('camera_chase').find('plugin')
    assert plugin.findtext('target_model') == 'go2'
    assert plugin.findtext('target_link') == 'base_link'


def test_observer_image_and_intrinsic_topics_are_distinct():
    expected = {'camera_chase': (1280, 720, '/camera/chase'),
                'camera_overview': (1024, 1024, '/camera/overview')}
    for name, (width, height, topic) in expected.items():
        sensor = model(name).find('.//sensor')
        assert sensor.findtext('topic') == topic+'/image'
        assert sensor.findtext('frame_id') == name+'_optical'
        assert sensor.findtext('camera/camera_info_topic') == topic+'/camera_info'
        assert int(sensor.findtext('camera/image/width')) == width
        assert int(sensor.findtext('camera/image/height')) == height


def test_overview_camera_is_north_up_and_covers_every_wall_corner():
    rig = model('camera_overview')
    x, y, z, roll, pitch, yaw = map(float, rig.findtext('pose').split())
    assert abs(x) < 1e-8 and abs(y) < 1e-8
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    assert np.allclose(rotation[:, 0], [0, 0, -1])
    assert np.allclose(-rotation[:, 1], [1, 0, 0])
    assert np.allclose(rotation[:, 2], [0, 1, 0])
    cam = rig.find('.//camera')
    horizontal = math.tan(float(cam.findtext('horizontal_fov'))/2)
    vertical = horizontal*int(cam.findtext('image/height'))/int(cam.findtext('image/width'))
    world = ET.parse(WS / 'src/tugbot_gazebo/worlds/go2_maze.sdf').getroot().find('world')
    for wall in world.findall('model'):
        if not wall.get('name').startswith('maze_wall'):
            continue
        wx, wy, wz, *_ = map(float, wall.findtext('pose').split())
        sx, sy, sz = map(float, wall.findtext('.//collision/geometry/box/size').split())
        for a in (-1, 1):
            for b in (-1, 1):
                for c in (-1, 1):
                    ray = rotation.T @ (np.array([wx+a*sx/2, wy+b*sy/2, wz+c*sz/2])-np.array([x, y, z]))
                    assert ray[0] > 0
                    assert abs(ray[1])/ray[0] < horizontal
                    assert abs(ray[2])/ray[0] < vertical


def test_extra_camera_bridge_wires_images_info_and_actual_optical_tf():
    config = BRINGUP / 'config/observer_bridge.yaml'
    assert config.is_file(), 'Observer camera streams have not been bridged yet'
    entries = yaml.safe_load(config.read_text())
    pairs = {(e['ros_topic_name'], e['gz_topic_name']): e for e in entries}
    for prefix in ('/camera/chase', '/camera/overview'):
        for suffix, typename in (('/image', 'sensor_msgs/msg/Image'), ('/camera_info', 'sensor_msgs/msg/CameraInfo')):
            entry = pairs[(prefix+suffix, prefix+suffix)]
            assert entry['ros_type_name'] == typename
            assert entry['direction'] == 'GZ_TO_ROS'
            assert entry['lazy'] is True
    assert pairs[('/tf', '/camera/chase/tf')]['ros_type_name'] == 'tf2_msgs/msg/TFMessage'


def test_rviz_has_three_enabled_independent_image_panes():
    config = BRINGUP / 'config/three_cameras.rviz'
    assert config.is_file(), 'Three simultaneous RViz Image panes are not configured yet'
    config = yaml.safe_load(config.read_text())
    images = [d for d in config['Visualization Manager']['Displays'] if d['Class'] == 'rviz_default_plugins/Image']
    assert len(images) == 3
    assert {d['Topic']['Value'] for d in images} == {'/camera/front/image', '/camera/chase/image', '/camera/overview/image'}
    assert len({d['Name'] for d in images}) == 3
    assert all(d['Enabled'] and d['Value'] for d in images)


def test_lens_intrinsics_cannot_override_the_requested_zoom_or_image_center():
    for name in ('camera_chase', 'camera_overview'):
        camera = model(name).find('.//camera')
        intrinsic = camera.find('lens/intrinsics')
        if intrinsic is None:
            continue  # Gazebo derives a centered projection from HFOV/resolution.
        width = int(camera.findtext('image/width'))
        height = int(camera.findtext('image/height'))
        focal = width/(2*math.tan(float(camera.findtext('horizontal_fov'))/2))
        assert abs(float(intrinsic.findtext('fx'))-focal) < 1.0
        assert abs(float(intrinsic.findtext('fy'))-focal) < 1.0
        assert abs(float(intrinsic.findtext('cx'))-width/2) <= 1.0
        assert abs(float(intrinsic.findtext('cy'))-height/2) <= 1.0


def test_spawn_preserves_overview_pose_instead_of_create_defaults():
    # ros_gz_sim/create always overrides SDF model pose, even with no pose flags.
    tree = ast.parse((BRINGUP / 'launch/maze.launch.py').read_text())
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'camera_spawn_arguments')
    isolated = ast.Module(body=[function], type_ignores=[])
    namespace = {'ET': ET, 'Path': Path}
    exec(compile(ast.fix_missing_locations(isolated), '<spawn-contract>', 'exec'), namespace)
    args = namespace['camera_spawn_arguments'](OBSERVER / 'camera_overview/model.sdf')
    flags = dict(zip(args[::2], args[1::2]))
    assert [float(flags[f]) for f in ('-x', '-y', '-z', '-R', '-P', '-Y')] == list(map(float, model('camera_overview').findtext('pose').split()))
    assert flags['-name'] == 'camera_overview'


def test_camera_verifier_rejects_a_stream_that_froze_while_the_robot_kept_moving():
    spec = importlib.util.spec_from_file_location('check_go2_cameras', WS / 'tools/check_go2_cameras.py')
    checker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checker)
    # A previously healthy stream is stale even if its internal timestamps look good.
    with pytest.raises(AssertionError, match='stale'):
        checker.check_image_times(np.arange(0, 4, .1), 90., 0.)
    checker.check_image_times(np.arange(0, 90.01, .1), 90., 0.)
    with pytest.raises(AssertionError, match='window'):
        checker.check_image_times(np.arange(86, 90.01, .1), 90., 0.)


def test_image_change_measurement_does_not_mistake_camera_noise_for_motion():
    spec = importlib.util.spec_from_file_location('camera_checker', WS / 'tools/check_go2_cameras.py')
    checker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checker)
    rng = np.random.default_rng(72)
    frames = [np.clip(128+rng.normal(0, 255*.007, (540, 720, 3)), 0, 255).astype(np.uint8) for _ in range(5)]
    reference = checker.scene_thumbnail(frames[0])
    assert max(np.mean(np.abs(checker.scene_thumbnail(frame)-reference)) for frame in frames[1:]) < .3
    changed = frames[0].copy()
    changed[100:400, 150:500] = 64
    assert np.mean(np.abs(checker.scene_thumbnail(changed)-reference)) > 2
