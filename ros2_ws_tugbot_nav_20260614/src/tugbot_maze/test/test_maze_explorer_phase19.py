from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'


LOCAL_COST_FIELDS = [
    'local_costmap_topic',
    'local_costmap_msg',
    'local_costmap_view',
    'local_costmap_sub',
    '_local_costmap_callback',
    '_compute_local_cost_diagnostics',
    'dispatch_target_local_cost',
    'dispatch_target_local_cost_max_radius',
    'dispatch_path_local_cost_max',
    'dispatch_path_local_cost_mean',
    'timeout_robot_local_cost_max',
    'timeout_robot_obstacle_cluster_count',
    'footprint_corridor_inflation_squeezed',
]


def test_phase19_maze_explorer_subscribes_to_local_costmap_topic_without_changing_branch_selection():
    source = MAZE_EXPLORER.read_text()
    assert "declare_parameter('local_costmap_topic', '/local_costmap/costmap')" in source
    assert 'self.local_costmap_sub = self.create_subscription(OccupancyGrid, self.local_costmap_topic, self._local_costmap_callback' in source
    assert '/local_costmap/costmap_raw' not in source


def test_phase19_goal_events_publish_local_cost_fields_from_context():
    source = MAZE_EXPLORER.read_text()
    for field in LOCAL_COST_FIELDS:
        assert field in source
    assert "active_goal_event_context.update(self._compute_local_cost_diagnostics(dispatch_pose, target_xy))" in source
    publish_pos = source.index('def _publish_goal_event')
    payload = source[publish_pos:source.index('self.goal_events_pub.publish', publish_pos)]
    for field in [
        'dispatch_target_local_cost',
        'dispatch_target_local_cost_max_radius',
        'dispatch_path_local_cost_max',
        'dispatch_path_local_cost_mean',
        'timeout_robot_local_cost_max',
        'timeout_robot_obstacle_cluster_count',
        'footprint_corridor_inflation_squeezed',
    ]:
        assert f"'{field}': context.get('{field}')" in payload


def test_phase19_does_not_mutate_branch_selection_from_local_cost_diagnostics():
    source = MAZE_EXPLORER.read_text()
    diagnostics_pos = source.index('def _compute_local_cost_diagnostics')
    diagnostics_body = source[diagnostics_pos:source.index('def _goal_elapsed_sec', diagnostics_pos)]
    forbidden = ['mark_branch_state', 'record_branch_failure', 'blacklist', 'active_branch =', 'branch.state']
    for token in forbidden:
        assert token not in diagnostics_body
