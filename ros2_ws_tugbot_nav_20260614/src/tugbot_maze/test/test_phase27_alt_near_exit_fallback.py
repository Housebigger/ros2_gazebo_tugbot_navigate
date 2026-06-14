from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
MAZE_LAUNCH = ROOT / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_maze_explore.launch.py'
NAV2_CONFIG_DIR = ROOT / 'src' / 'tugbot_navigation' / 'config'
PROPOSAL = ROOT / 'doc' / 'doc_proposal' / 'phase27_alt_near_exit_fallback_design.md'


def _source() -> str:
    return MAZE_EXPLORER.read_text()


def _launch() -> str:
    return MAZE_LAUNCH.read_text()


def test_phase27_alt_design_proposal_exists_and_preserves_guardrails():
    text = PROPOSAL.read_text()
    required = [
        'near_exit_fallback_enabled',
        'default `false`',
        'not an explanation of the Phase26 MPPI selected-control near-zero root cause',
        'Do not modify Nav2/MPPI parameters',
        'Do not change the normal DFS/topology branch-selection strategy',
        'goal_kind=\'near_exit_micro_goal\'',
    ]
    for token in required:
        assert token in text


def test_phase27_alt_maze_explorer_declares_disabled_bounded_fallback_parameters():
    source = _source()
    expected_declares = [
        "declare_parameter('near_exit_fallback_enabled', False)",
        "declare_parameter('near_exit_fallback_trigger_radius_m', 0.9)",
        "declare_parameter('near_exit_terminal_acceptance_radius_m', 0.6)",
        "declare_parameter('near_exit_micro_goal_min_step_m', 0.20)",
        "declare_parameter('near_exit_micro_goal_max_step_m', 0.35)",
        "declare_parameter('near_exit_fallback_max_attempts', 1)",
        "declare_parameter('near_exit_fallback_require_clean_topology', True)",
        "declare_parameter('near_exit_fallback_robot_to_path_max_m', 0.15)",
    ]
    for snippet in expected_declares:
        assert snippet in source
    assert 'self.near_exit_fallback_attempts = 0' in source


def test_phase27_alt_launch_exposes_default_disabled_switch_and_thresholds():
    launch = _launch()
    required_args = [
        "DeclareLaunchArgument('near_exit_fallback_enabled', default_value='false'",
        "DeclareLaunchArgument('near_exit_fallback_trigger_radius_m', default_value='0.9'",
        "DeclareLaunchArgument('near_exit_terminal_acceptance_radius_m', default_value='0.6'",
        "DeclareLaunchArgument('near_exit_micro_goal_min_step_m', default_value='0.20'",
        "DeclareLaunchArgument('near_exit_micro_goal_max_step_m', default_value='0.35'",
        "DeclareLaunchArgument('near_exit_fallback_max_attempts', default_value='1'",
    ]
    for snippet in required_args:
        assert snippet in launch
    required_params = [
        "'near_exit_fallback_enabled': ParameterValue(LaunchConfiguration('near_exit_fallback_enabled'), value_type=bool)",
        "'near_exit_fallback_trigger_radius_m': ParameterValue(LaunchConfiguration('near_exit_fallback_trigger_radius_m'), value_type=float)",
        "'near_exit_terminal_acceptance_radius_m': ParameterValue(LaunchConfiguration('near_exit_terminal_acceptance_radius_m'), value_type=float)",
    ]
    for snippet in required_params:
        assert snippet in launch


def test_phase27_alt_goal_event_schema_contains_fallback_diagnostics_fields():
    source = _source()
    publish_start = source.index('def _publish_goal_event')
    publish_body = source[publish_start:source.index('self.goal_events_pub.publish', publish_start)]
    required_fields = [
        'near_exit_fallback_triggered',
        'fallback_reason',
        'robot_exit_dist',
        'cmd_near_zero_duration',
        'last_nav2_result',
        'robot_to_path_distance',
        "'action'",
    ]
    for field in required_fields:
        assert field in publish_body
    assert "self._publish_goal_event('near_exit_fallback'" in source


def test_phase27_alt_decision_helper_encodes_disabled_boundaries_and_clean_topology_gate():
    source = _source()
    decision_start = source.index('def _near_exit_fallback_decision')
    decision_body = source[decision_start:source.index('def _maybe_apply_near_exit_fallback', decision_start)]
    required_logic = [
        "'action': 'no_action'",
        "'action': 'terminal_acceptance'",
        "'action': 'micro_goal'",
        'near_exit_fallback_enabled',
        'robot_exit_dist > self.near_exit_fallback_trigger_radius_m',
        'robot_exit_dist <= self.near_exit_terminal_acceptance_radius_m',
        'self.topology.blocked_branch_count() != 0',
        'len(self.topology.blacklist) != 0',
        'robot_to_path_distance > self.near_exit_fallback_robot_to_path_max_m',
    ]
    for snippet in required_logic:
        assert snippet in decision_body


def test_phase27_alt_decision_helper_terminal_acceptance_contract_includes_recent_failure_clean_topology_and_action_reason():
    source = _source()
    decision_start = source.index('def _near_exit_fallback_decision')
    decision_body = source[decision_start:source.index('def _maybe_apply_near_exit_fallback', decision_start)]
    required_logic = [
        'recent_problem_reasons = {GOAL_TIMEOUT, BLOCKED_NAV2, GOAL_CANCELED_AFTER_TIMEOUT, GOAL_REJECTED}',
        'has_recent_failure = self.last_failure_reason in recent_problem_reasons',
        'if not (has_recent_failure or has_cmd_stall):',
        'robot_exit_dist <= self.near_exit_terminal_acceptance_radius_m',
        "'fallback_reason': 'terminal_acceptance_radius'",
        "'action': 'terminal_acceptance'",
        "'near_exit_fallback_triggered': True",
    ]
    for snippet in required_logic:
        assert snippet in decision_body
    assert 'GOAL_CANCELED_AFTER_TIMEOUT,' in source


def test_phase27_alt_micro_goal_is_bounded_feasible_and_does_not_pollute_topology():
    source = _source()
    assert "goal_kind='near_exit_micro_goal'" in source
    assert 'def _compute_near_exit_micro_goal' in source
    micro_start = source.index('def _compute_near_exit_micro_goal')
    micro_body = source[micro_start:source.index('def _near_exit_micro_goal_feasible', micro_start)]
    for snippet in [
        'near_exit_micro_goal_min_step_m',
        'near_exit_micro_goal_max_step_m',
        'near_exit_terminal_acceptance_radius_m',
        'math.hypot(self.exit_x - robot_pose[0], self.exit_y - robot_pose[1])',
    ]:
        assert snippet in micro_body
    feasible_start = source.index('def _near_exit_micro_goal_feasible')
    feasible_body = source[feasible_start:source.index('def _publish_near_exit_fallback_event', feasible_start)]
    for snippet in [
        "geometry.get('line_of_sight_occupied_count')",
        "geometry.get('target_clearance_m')",
        "local_cost.get('dispatch_target_local_cost')",
        'self.local_cost_obstacle_threshold',
    ]:
        assert snippet in feasible_body
    failure_start = source.index('def _handle_goal_failure')
    failure_body = source[failure_start:source.index('def _is_stale_goal_result', failure_start)]
    assert "elif self.active_goal_kind == 'near_exit_micro_goal':" in failure_body
    micro_failure_block = failure_body[
        failure_body.index("elif self.active_goal_kind == 'near_exit_micro_goal':"):
        failure_body.index("elif self.active_goal_kind == 'explore'", failure_body.index("elif self.active_goal_kind == 'near_exit_micro_goal':"))
    ]
    forbidden = ['record_branch_failure', 'mark_branch_state', 'blacklist']
    for token in forbidden:
        assert token not in micro_failure_block


def test_phase27_alt_no_nav2_mppi_config_semantics_changed_by_phase():
    for path in NAV2_CONFIG_DIR.glob('*.yaml'):
        text = path.read_text()
        assert 'near_exit_fallback' not in text
        assert 'terminal_acceptance' not in text
        assert 'micro_goal' not in text
