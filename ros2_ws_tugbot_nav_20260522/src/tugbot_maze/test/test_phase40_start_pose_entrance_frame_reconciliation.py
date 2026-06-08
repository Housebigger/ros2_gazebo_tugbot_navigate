import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
ACTIVE_WORLD = ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
METADATA = ROOT / 'src' / 'tugbot_maze' / 'config' / 'maze_20260528_scaled_instance.yaml'
SLAM_PARAMS = ROOT / 'src' / 'tugbot_navigation' / 'config' / 'slam_toolbox_params.yaml'
MAZE_SLAM = ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_maze_slam.launch.py'
MAZE_SLAM_NAV = ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_maze_slam_nav.launch.py'
MAZE_EXPLORE = ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_maze_explore.launch.py'
RECORDER = ROOT / 'tools' / 'record_phase40_start_pose_alignment.py'
WRAPPER = ROOT / 'tools' / 'run_phase40_start_pose_entrance_frame_reconciliation_diagnostics.sh'

ALLOWED = {
    'START_POSE_ALIGNED_READY_FOR_TOPOLOGY_RERUN',
    'SPAWN_WIRING_FIXED_PENDING_RUNTIME_CHECK',
    'METADATA_WORLD_FRAME_CONVENTION_CONFLICT',
    'INSUFFICIENT_EVIDENCE',
}


def _floats(text: str):
    return [float(v) for v in text.split()]


def _metadata():
    return yaml.safe_load(METADATA.read_text(encoding='utf-8'))


def _world_root():
    tree = ET.parse(ACTIVE_WORLD)
    world = tree.getroot().find('.//world')
    assert world is not None
    return world


def _tugbot_include_pose():
    world = _world_root()
    for include in world.findall('include'):
        if (include.findtext('uri') or '').strip() == 'model://tugbot':
            assert (include.findtext('name') or '').strip() == 'tugbot'
            return _floats(include.findtext('pose') or '')
    raise AssertionError('active scaled2x SDF must include model://tugbot')


def _wall_signature():
    out = {}
    for model in _world_root().findall('model'):
        name = model.get('name') or ''
        if not name.startswith('maze_wall'):
            continue
        out[name] = {
            'pose': _floats(model.findtext('pose') or ''),
            'size': _floats(model.findtext('./link/collision/geometry/box/size') or ''),
        }
    return out


def test_phase40_active_sdf_tugbot_spawn_pose_equals_active_metadata_entrance():
    meta = _metadata()
    entrance = meta['entrance']
    pose = _tugbot_include_pose()
    assert math.isclose(pose[0], float(entrance['x_m']), abs_tol=0.005)
    assert math.isclose(pose[1], float(entrance['y_m']), abs_tol=0.005)
    assert math.isclose(pose[5], float(entrance['yaw_rad']), abs_tol=0.005)
    assert math.hypot(pose[0], pose[1]) > 2.0, 'do not regress active Tugbot spawn back to the world origin'


def test_phase40_slam_map_start_pose_probe_was_rejected_by_phase41_convention_fix():
    params = yaml.safe_load(SLAM_PARAMS.read_text(encoding='utf-8'))
    slam_ros_params = params['slam_toolbox']['ros__parameters']
    assert 'map_start_pose' not in slam_ros_params, 'Phase41 resolved the conflict by converting runtime truth to map frame, not by forcing SLAM to world coordinates'
    assert slam_ros_params['map_frame'] == 'map'
    assert slam_ros_params['odom_frame'] == 'odom'
    assert slam_ros_params['base_frame'] == 'base_link'


def test_phase40_launches_do_not_override_tugbot_spawn_back_to_origin():
    for path in [MAZE_SLAM, MAZE_SLAM_NAV, MAZE_EXPLORE]:
        source = path.read_text(encoding='utf-8')
        assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in source
        assert 'world_sdf' in source
        forbidden = [
            'ros_gz_sim create',
            'spawn_entity.py',
            'spawn_x',
            'spawn_y',
            'spawn_z',
            'x:=0',
            'y:=0',
            '--x 0',
            '--y 0',
        ]
        for token in forbidden:
            assert token not in source, f'{path.name} must not override active SDF Tugbot pose with {token}'


def test_phase40_maze_explore_uses_phase41_map_frame_truth_while_sdf_spawn_matches_world_entrance():
    meta = _metadata()
    pose = _tugbot_include_pose()
    source = MAZE_EXPLORE.read_text(encoding='utf-8')
    world_entrance = meta['world_frame_truth']['entrance']
    map_entrance = meta['map_frame_truth']['entrance']
    map_exit = meta['map_frame_truth']['exit']
    assert "DeclareLaunchArgument('entrance_x', default_value='0.0'" in source
    assert "DeclareLaunchArgument('entrance_y', default_value='0.0'" in source
    assert f"DeclareLaunchArgument('exit_x', default_value='{float(map_exit['x_m']):.6f}'" in source
    assert f"DeclareLaunchArgument('exit_y', default_value='{float(map_exit['y_m']):.6f}'" in source
    assert math.isclose(pose[0], float(world_entrance['x_m']), abs_tol=0.005)
    assert math.isclose(pose[1], float(world_entrance['y_m']), abs_tol=0.005)
    assert math.isclose(float(map_entrance['x_m']), 0.0, abs_tol=1e-9)
    assert math.isclose(float(map_entrance['y_m']), 0.0, abs_tol=1e-9)


def test_phase40_wall_geometry_and_marker_xyz_unchanged_contract():
    walls = _wall_signature()
    assert len(walls) == 53
    assert walls['maze_wall_outer_001_outer_001']['pose'] == [-0.034, 9.994, 0.6, 0.0, 0.0, 0.0]
    assert walls['maze_wall_outer_001_outer_001']['size'] == [20.056, 0.24, 1.2]
    world = _world_root()
    models = {m.get('name'): m for m in world.findall('model')}
    assert _floats(models['maze_entrance_arrow_visual'].findtext('pose') or '')[:3] == [-10.661, -9.025, 0.005]
    assert _floats(models['maze_exit_finish_band_visual'].findtext('pose') or '')[:3] == [10.061, 9.058, 0.005]
    assert _floats(models['maze_exit_marker'].findtext('pose') or '')[:3] == [10.061, 9.058, 0.01]
    for name in ['maze_entrance_arrow_visual', 'maze_exit_finish_band_visual', 'maze_exit_marker']:
        assert models[name].find('.//collision') is None


def test_phase40_tools_are_bounded_pose_only_and_preserve_phase37_39_semantics():
    assert RECORDER.exists(), 'Phase40 pose alignment recorder must exist'
    assert WRAPPER.exists(), 'Phase40 bounded diagnostics wrapper must exist'
    recorder = RECORDER.read_text(encoding='utf-8')
    wrapper = WRAPPER.read_text(encoding='utf-8')
    for label in ALLOWED:
        assert label in recorder
    for token in ['map->base_link', 'odom->base_link', 'map->odom', '/odom', 'robot_to_active_entrance_distance_m']:
        assert token in recorder
    assert 'phase40_start_pose_entrance_frame_reconciliation' in wrapper
    assert 'tugbot_maze_slam_nav.launch.py' in wrapper
    assert 'record_phase40_start_pose_alignment.py' in wrapper
    assert 'ros2 launch tugbot_bringup tugbot_maze_explore.launch.py' not in wrapper
    assert 'max_goals' not in wrapper
    assert 'ros2 action send_goal' not in wrapper
    assert 'NavigateToPose' not in wrapper
    assert 'BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH' in wrapper
    assert 'FRAME_ALIGNMENT_ISSUE_CONFIRMED' in wrapper
    assert 'cleanup_processes_after.txt' in wrapper
    assert 'no goal dispatch' in wrapper
    assert 'no maze_explorer' in wrapper
    assert 'no maze_explorer strategy edits' in wrapper


def test_phase40_nav2_and_maze_explorer_strategy_diff_empty():
    result = subprocess.run(
        ['git', 'diff', '--', 'src/tugbot_navigation/config/nav2*.yaml', 'src/tugbot_maze/tugbot_maze/maze_explorer.py'],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )
    assert result.stdout == ''
