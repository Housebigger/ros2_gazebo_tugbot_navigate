from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MAZE_PKG = ROOT / 'src' / 'tugbot_maze'


def test_phase17_goal_events_include_dispatch_geometry_quality_fields():
    source = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()
    required = [
        'target_cell_occupancy',
        'target_clearance_m',
        'line_of_sight_occupied_count',
        'line_of_sight_unknown_count',
        'line_of_sight_min_clearance_m',
        'path_corridor_min_clearance_m',
        'target_near_wall',
        'target_crosses_wall_corner',
        'target_crosses_narrow_passage',
        'target_yaw_corridor_conflict',
        '_compute_goal_geometry_diagnostics',
    ]
    for field in required:
        assert field in source


def test_phase17_grid_view_can_measure_clearance_and_line_samples():
    source = (MAZE_PKG / 'tugbot_maze' / 'grid_utils.py').read_text()
    required = [
        'nearest_obstacle_distance',
        'line_sample_values',
        'line_min_clearance',
    ]
    for field in required:
        assert field in source


def test_phase17_diagnostics_are_added_to_dispatch_context_before_publish():
    source = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()
    context_pos = source.index('self.active_goal_event_context = {')
    diagnostics_pos = source.index('self.active_goal_event_context.update(self._compute_goal_geometry_diagnostics')
    publish_pos = source.index("self._publish_goal_event('dispatch'")
    assert context_pos < diagnostics_pos < publish_pos
    assert 'self.active_goal_event_context.update(' in source


def test_phase17_goal_event_payload_publishes_geometry_context_fields():
    source = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()
    publish_body = source[source.index('payload = {'):source.index('self.goal_events_pub.publish')]
    required = [
        "'target_cell_occupancy': context.get('target_cell_occupancy')",
        "'target_clearance_m': context.get('target_clearance_m')",
        "'line_of_sight_occupied_count': context.get('line_of_sight_occupied_count')",
        "'line_of_sight_unknown_count': context.get('line_of_sight_unknown_count')",
        "'line_of_sight_min_clearance_m': context.get('line_of_sight_min_clearance_m')",
        "'path_corridor_min_clearance_m': context.get('path_corridor_min_clearance_m')",
        "'target_near_wall': context.get('target_near_wall')",
        "'target_crosses_wall_corner': context.get('target_crosses_wall_corner')",
        "'target_crosses_narrow_passage': context.get('target_crosses_narrow_passage')",
        "'target_yaw_corridor_conflict': context.get('target_yaw_corridor_conflict')",
    ]
    for snippet in required:
        assert snippet in publish_body
