import ast
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
METADATA = ROOT / 'src' / 'tugbot_maze' / 'config' / 'maze_20260528_scaled_instance.yaml'
ACTIVE_WORLD = ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
MAZE_EXPLORE = ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_maze_explore.launch.py'
SLAM_PARAMS = ROOT / 'src' / 'tugbot_navigation' / 'config' / 'slam_toolbox_params.yaml'
RECORDER = ROOT / 'tools' / 'record_phase41_world_to_ros_pose_frame_convention.py'
WRAPPER = ROOT / 'tools' / 'run_phase41_world_to_ros_pose_frame_convention_diagnostics.sh'

WORLD_ENTRANCE = (-11.011281, -9.025070, 0.0)
WORLD_EXIT = (10.061281, 9.058496, 1.2)
MAP_ENTRANCE = (0.0, 0.0, 0.0)
MAP_EXIT = (21.072562, 18.083566, 1.2)
ALLOWED_CONCLUSIONS = {
    'MAP_FRAME_TRUTH_ALIGNED_READY_FOR_PHASE37_RERUN',
    'EXPLICIT_WORLD_TO_MAP_TRANSFORM_REQUIRED',
    'FRAME_CONVENTION_UNRESOLVED',
}


def _metadata():
    return yaml.safe_load(METADATA.read_text(encoding='utf-8'))


def _floats(text: str):
    return [float(v) for v in text.split()]


def _world_root():
    tree = ET.parse(ACTIVE_WORLD)
    world = tree.getroot().find('.//world')
    assert world is not None
    return world


def _tugbot_pose():
    for include in _world_root().findall('include'):
        if (include.findtext('uri') or '').strip() == 'model://tugbot':
            return _floats(include.findtext('pose') or '')
    raise AssertionError('missing active Tugbot include')


def _launch_defaults(path: Path):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    defaults = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != 'DeclareLaunchArgument':
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        key = str(node.args[0].value)
        for keyword in node.keywords:
            if keyword.arg == 'default_value' and isinstance(keyword.value, ast.Constant):
                defaults[key] = str(keyword.value.value)
    return defaults


def test_phase41_metadata_separates_world_frame_and_map_frame_truth():
    meta = _metadata()
    assert meta['coordinate_convention']['phase41_selected_convention'] == 'A_map_frame_follows_slam_startup_pose'
    assert meta['coordinate_convention']['runtime_truth_frame'] == 'map'
    assert meta['coordinate_convention']['gazebo_marker_frame'] == 'world'
    assert meta['world_frame_truth']['entrance']['x_m'] == WORLD_ENTRANCE[0]
    assert meta['world_frame_truth']['entrance']['y_m'] == WORLD_ENTRANCE[1]
    assert meta['world_frame_truth']['exit']['x_m'] == WORLD_EXIT[0]
    assert meta['world_frame_truth']['exit']['y_m'] == WORLD_EXIT[1]
    assert meta['map_frame_truth']['entrance']['x_m'] == MAP_ENTRANCE[0]
    assert meta['map_frame_truth']['entrance']['y_m'] == MAP_ENTRANCE[1]
    assert meta['map_frame_truth']['entrance']['yaw_rad'] == MAP_ENTRANCE[2]
    assert math.isclose(meta['map_frame_truth']['exit']['x_m'], MAP_EXIT[0], abs_tol=1e-6)
    assert math.isclose(meta['map_frame_truth']['exit']['y_m'], MAP_EXIT[1], abs_tol=1e-6)
    assert meta['map_frame_truth']['exit']['radius_m'] == MAP_EXIT[2]


def test_phase41_legacy_top_level_truth_keeps_world_frame_for_marker_compatibility():
    meta = _metadata()
    assert meta['entrance']['x_m'] == meta['world_frame_truth']['entrance']['x_m']
    assert meta['entrance']['y_m'] == meta['world_frame_truth']['entrance']['y_m']
    assert meta['exit']['x_m'] == meta['world_frame_truth']['exit']['x_m']
    assert meta['exit']['y_m'] == meta['world_frame_truth']['exit']['y_m']
    assert meta['markers']['entrance_arrow']['x_m'] == -10.661281
    assert meta['markers']['exit_finish_band']['x_m'] == 10.061281
    assert meta['markers']['exit_marker']['x_m'] == 10.061281


def test_phase41_map_frame_truth_is_world_truth_minus_entrance_offset():
    meta = _metadata()
    world_entrance = meta['world_frame_truth']['entrance']
    world_exit = meta['world_frame_truth']['exit']
    map_entrance = meta['map_frame_truth']['entrance']
    map_exit = meta['map_frame_truth']['exit']
    assert math.isclose(map_entrance['x_m'], 0.0, abs_tol=1e-9)
    assert math.isclose(map_entrance['y_m'], 0.0, abs_tol=1e-9)
    assert math.isclose(map_exit['x_m'], world_exit['x_m'] - world_entrance['x_m'], abs_tol=1e-6)
    assert math.isclose(map_exit['y_m'], world_exit['y_m'] - world_entrance['y_m'], abs_tol=1e-6)
    assert math.isclose(map_exit['radius_m'], world_exit['radius_m'], abs_tol=1e-9)


def test_phase41_explore_launch_defaults_pass_map_frame_truth_to_explorer_and_monitor():
    defaults = _launch_defaults(MAZE_EXPLORE)
    assert defaults['entrance_x'] == '0.0'
    assert defaults['entrance_y'] == '0.0'
    assert defaults['entrance_yaw'] == '0.0'
    assert defaults['exit_x'] == '21.072562'
    assert defaults['exit_y'] == '18.083566'
    assert defaults['exit_radius'] == '1.2'
    source = MAZE_EXPLORE.read_text(encoding='utf-8')
    assert 'map_frame_truth' in source
    assert 'world_frame_truth' in source
    assert "'entrance_x': ParameterValue(LaunchConfiguration('entrance_x'), value_type=float)" in source
    assert "'exit_x': ParameterValue(LaunchConfiguration('exit_x'), value_type=float)" in source


def test_phase41_sdf_spawn_and_markers_stay_in_world_coordinates():
    pose = _tugbot_pose()
    assert math.isclose(pose[0], WORLD_ENTRANCE[0], abs_tol=0.005)
    assert math.isclose(pose[1], WORLD_ENTRANCE[1], abs_tol=0.005)
    world = _world_root()
    models = {m.get('name'): m for m in world.findall('model')}
    assert _floats(models['maze_entrance_arrow_visual'].findtext('pose') or '')[:3] == [-10.661, -9.025, 0.005]
    assert _floats(models['maze_exit_finish_band_visual'].findtext('pose') or '')[:3] == [10.061, 9.058, 0.005]
    assert _floats(models['maze_exit_marker'].findtext('pose') or '')[:3] == [10.061, 9.058, 0.01]


def test_phase41_slam_online_mapping_params_do_not_try_to_force_world_coordinates():
    params = yaml.safe_load(SLAM_PARAMS.read_text(encoding='utf-8'))['slam_toolbox']['ros__parameters']
    assert 'map_start_pose' not in params
    assert params['map_frame'] == 'map'
    assert params['odom_frame'] == 'odom'
    assert params['base_frame'] == 'base_link'


def test_phase41_tools_are_bounded_pose_only_and_classify_selected_convention():
    assert RECORDER.exists()
    assert WRAPPER.exists()
    recorder = RECORDER.read_text(encoding='utf-8')
    wrapper = WRAPPER.read_text(encoding='utf-8')
    for label in ALLOWED_CONCLUSIONS:
        assert label in recorder
    for token in ['map_frame_truth', 'world_frame_truth', 'exit_map_distance_m', 'map->base_link', 'odom->base_link', 'map->odom']:
        assert token in recorder
    assert 'phase41_world_to_ros_pose_frame_convention' in wrapper
    assert 'tugbot_maze_slam_nav.launch.py' in wrapper
    assert 'tugbot_maze_explore.launch.py' not in wrapper
    assert 'max_goals' not in wrapper
    assert 'ros2 action send_goal' not in wrapper
    assert 'NavigateToPose' not in wrapper
    assert 'no maze_explorer' in wrapper
    assert 'no goal dispatch' in wrapper
    assert 'METADATA_WORLD_FRAME_CONVENTION_CONFLICT' in wrapper


def test_phase41_nav2_and_maze_explorer_strategy_diff_empty():
    result = subprocess.run(
        ['git', 'diff', '--', 'src/tugbot_navigation/config/nav2*.yaml', 'src/tugbot_maze/tugbot_maze/maze_explorer.py'],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )
    assert result.stdout == ''
