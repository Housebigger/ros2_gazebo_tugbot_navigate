from pathlib import Path
import math
import struct
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
BRINGUP = ROOT / 'src' / 'tugbot_bringup'
GAZEBO = ROOT / 'src' / 'tugbot_gazebo'
DESCRIPTION = ROOT / 'src' / 'tugbot_description'
MAZE = ROOT / 'src' / 'tugbot_maze'


def _maze_wall_rectangles():
    world = ET.parse(GAZEBO / 'worlds' / 'tugbot_maze_world.sdf').getroot()
    rectangles = []
    for model in world.findall('.//world/model'):
        name = model.get('name') or ''
        if not name.startswith('maze_wall'):
            continue
        pose = (model.findtext('pose') or '0 0 0 0 0 0').split()
        x = float(pose[0])
        y = float(pose[1])
        size_text = model.findtext('.//collision/geometry/box/size')
        assert size_text is not None, name
        sx, sy, _ = [float(part) for part in size_text.split()]
        rectangles.append((name, x - sx / 2.0, x + sx / 2.0, y - sy / 2.0, y + sy / 2.0))
    return rectangles


def _rectangle_gap(a, b):
    _, ax_min, ax_max, ay_min, ay_max = a
    _, bx_min, bx_max, by_min, by_max = b
    dx = max(bx_min - ax_max, ax_min - bx_max, 0.0)
    dy = max(by_min - ay_max, ay_min - by_max, 0.0)
    return math.hypot(dx, dy)


def _binary_stl_xy_turning_diameter(path):
    data = path.read_bytes()
    triangle_count = struct.unpack('<I', data[80:84])[0]
    assert 84 + triangle_count * 50 == len(data)
    max_radius = 0.0
    offset = 84
    for _ in range(triangle_count):
        values = struct.unpack('<12fH', data[offset:offset + 50])
        vertices = [values[3:6], values[6:9], values[9:12]]
        for x, y, _z in vertices:
            max_radius = max(max_radius, math.hypot(x, y))
        offset += 50
    return max_radius * 2.0


def test_maze_launch_files_exist_and_use_maze_world():
    for launch_name in [
        'tugbot_maze_slam.launch.py',
        'tugbot_maze_slam_nav.launch.py',
        'tugbot_maze_explore.launch.py',
    ]:
        path = BRINGUP / 'launch' / launch_name
        assert path.exists(), launch_name
        text = path.read_text()
        assert 'tugbot_maze_world.sdf' in text
        assert 'tugbot_nav_world.sdf' not in text


def test_maze_explore_defaults_to_dfs_explorer_and_keeps_frontier_fallback():
    text = (BRINGUP / 'launch' / 'tugbot_maze_explore.launch.py').read_text()
    for required in [
        "DeclareLaunchArgument('explorer_type', default_value='maze_dfs'",
        "PythonExpression([\"'\", explorer_type, \"' == 'maze_dfs'\"])",
        "PythonExpression([\"'\", explorer_type, \"' == 'frontier'\"])",
        "package='tugbot_maze'",
        "executable='maze_explorer'",
        "name='maze_explorer'",
        "package='tugbot_exploration'",
        "executable='frontier_explorer'",
        "executable='maze_goal_monitor'",
        '/maze/explorer_state',
        '/maze/exit_reached',
        'lateral_centering_search_m',
        'allow_reverse_branch_goals',
        'goal_settle_sec',
        'near_exit_goal_timeout_sec',
        'near_exit_timeout_extension_radius_m',
        'reverse_branch_angle_threshold_deg',
        'blacklist_radius_m',
        'max_failures_per_branch',
        'max_backtrack_failures_per_node',
    ]:
        assert required in text
    assert "perimeter_enable_initial_spin': False" in text
    assert "cleanup_spin_after_goal': False" in text
    assert "enable_recovery_scan': False" in text


def test_maze_world_scaffold_contract():
    world = (GAZEBO / 'worlds' / 'tugbot_maze_world.sdf').read_text()
    for required in [
        '<world name="tugbot_maze_world">',
        'model://tugbot',
        '<name>tugbot</name>',
        '<pose>-4.0 -3.0 0 0 0 0</pose>',
        'maze_exit_marker',
        'maze_wall_bottom',
    ]:
        assert required in world


def test_bringup_declares_tugbot_maze_dependency():
    package_xml = (BRINGUP / 'package.xml').read_text()
    assert '<exec_depend>tugbot_maze</exec_depend>' in package_xml


def test_maze_corridors_are_at_least_twice_tugbot_turning_diameter():
    turning_diameter = _binary_stl_xy_turning_diameter(
        DESCRIPTION / 'models' / 'tugbot' / 'meshes' / 'base' / 'tugbot_simp.stl'
    )
    required_width = 2.0 * turning_diameter

    metadata = (MAZE / 'config' / 'maze_instance.yaml').read_text()
    assert 'min_corridor_width_policy: at_least_twice_tugbot_turning_outer_diameter' in metadata
    assert 'min_corridor_width_m: 1.6' in metadata
    assert 'measured_min_wall_gap_m: 1.76' in metadata
    assert required_width < 1.6

    rectangles = _maze_wall_rectangles()
    assert len(rectangles) >= 8
    positive_gaps = []
    for index, first in enumerate(rectangles):
        for second in rectangles[index + 1:]:
            gap = _rectangle_gap(first, second)
            if gap > 1e-6:
                positive_gaps.append((gap, first[0], second[0]))

    assert positive_gaps
    min_gap, first_name, second_name = min(positive_gaps)
    assert min_gap >= 1.6, (min_gap, first_name, second_name, required_width)
    assert min_gap >= required_width, (min_gap, first_name, second_name, required_width)
