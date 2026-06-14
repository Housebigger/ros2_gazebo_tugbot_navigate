from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MAZE_PKG = ROOT / 'src' / 'tugbot_maze'
BRINGUP = ROOT / 'src' / 'tugbot_bringup'


def test_maze_topology_declares_goal_preempted_reason():
    topology = (MAZE_PKG / 'tugbot_maze' / 'maze_topology.py').read_text()
    assert "GOAL_PREEMPTED = 'goal_preempted'" in topology


def test_maze_explorer_has_goal_sequence_and_stale_result_guards():
    explorer = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()
    for required in [
        'self.goal_sequence_id = 0',
        'self.active_goal_sequence_id',
        'self.last_completed_goal_sequence_id',
        'self.stale_result_count',
        'self.preempted_goal_count',
        'goal_sequence_id = self.goal_sequence_id',
        'lambda future, sequence_id=goal_sequence_id:',
        '_goal_response_callback(self, future, sequence_id:',
        '_goal_result_callback(self, future, sequence_id:',
        '_is_stale_goal_result(sequence_id)',
        '_mark_goal_preempted(sequence_id, status)',
        'GOAL_PREEMPTED',
    ]:
        assert required in explorer


def test_maze_explorer_has_settle_cooldown_before_next_analysis():
    explorer = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()
    for required in [
        "declare_parameter('goal_settle_sec'",
        'self.goal_settle_until',
        'self.last_goal_completion_time',
        '_start_goal_settle_cooldown()',
        '_goal_settle_active()',
        "mode == 'SETTLING'",
        "SETTLING = 'SETTLING'",
    ]:
        assert required in explorer


def test_maze_explorer_reports_phase8_diagnostics_in_state_json():
    explorer = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()
    for required in [
        "'goal_sequence_id'",
        "'active_goal_sequence_id'",
        "'last_completed_goal_sequence_id'",
        "'stale_result_count'",
        "'preempted_goal_count'",
        "'goal_settle_active'",
        "'goal_settle_remaining_sec'",
    ]:
        assert required in explorer


def test_maze_explore_defaults_reverse_branch_goals_to_true_and_declares_settle():
    launch = (BRINGUP / 'launch' / 'tugbot_maze_explore.launch.py').read_text()
    for required in [
        "DeclareLaunchArgument('allow_reverse_branch_goals', default_value='true'",
        "DeclareLaunchArgument('goal_settle_sec', default_value='1.5'",
        "'goal_settle_sec': ParameterValue(LaunchConfiguration('goal_settle_sec'), value_type=float)",
    ]:
        assert required in launch
