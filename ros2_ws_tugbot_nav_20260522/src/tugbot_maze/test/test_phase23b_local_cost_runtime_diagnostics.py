from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
LOCAL_COST_SUMMARY = ROOT / 'tools' / 'summarize_goal_event_local_costs.py'
FAILURE_WINDOWS = ROOT / 'tools' / 'analyze_failure_windows.py'

PHASE23B_FIELDS = [
    'timeout_footprint_cost_max',
    'timeout_footprint_cost_mean',
    'timeout_footprint_cost_p95',
    'timeout_footprint_inflated_cell_count',
    'timeout_footprint_lethal_cell_count',
    'timeout_front_wedge_cost_max',
    'timeout_front_wedge_cost_mean',
    'timeout_front_wedge_clearance_m',
    'timeout_left_side_cost_max',
    'timeout_left_side_clearance_m',
    'timeout_right_side_cost_max',
    'timeout_right_side_clearance_m',
    'timeout_path_ahead_0_5m_cost_max',
    'timeout_path_ahead_0_5m_cost_mean',
    'timeout_path_ahead_1_0m_cost_max',
    'timeout_path_ahead_1_0m_cost_mean',
]


def test_phase23b_goal_events_publish_footprint_front_side_and_path_ahead_local_cost_fields():
    source = MAZE_EXPLORER.read_text(encoding='utf-8')
    for token in [
        '_local_cost_values_in_oriented_box',
        '_local_cost_values_in_wedge',
        '_local_cost_path_ahead_values',
    ]:
        assert token in source

    publish_pos = source.index('def _publish_goal_event')
    payload = source[publish_pos:source.index('self.goal_events_pub.publish', publish_pos)]
    for field in PHASE23B_FIELDS:
        assert field in payload, field
        assert f"context.get('{field}')" in payload, field

    timeout_pos = source.index('def _update_active_timeout_local_cost_diagnostics')
    timeout_body = source[timeout_pos:source.index('def _local_cost_values_in_radius', timeout_pos)]
    for field in PHASE23B_FIELDS:
        assert field in timeout_body, field

    # Diagnostics must remain observational only.
    forbidden = ['mark_branch_state', 'record_branch_failure', 'blacklist', 'branch.state']
    for token in forbidden:
        assert token not in timeout_body


def test_phase23b_summary_and_failure_window_tools_carry_new_local_cost_fields():
    summary_source = LOCAL_COST_SUMMARY.read_text(encoding='utf-8')
    failure_source = FAILURE_WINDOWS.read_text(encoding='utf-8')
    for field in PHASE23B_FIELDS:
        assert field in summary_source, field
        assert field in failure_source, field

    assert 'front_side_clearance' in failure_source
    assert 'path_ahead_cost' in failure_source
    assert 'footprint_cost_stats' in failure_source
