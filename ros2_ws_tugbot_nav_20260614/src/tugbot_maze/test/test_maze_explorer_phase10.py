from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MAZE_PKG = ROOT / 'src' / 'tugbot_maze'


def test_maze_topology_declares_terminal_cancel_reason():
    topology = (MAZE_PKG / 'tugbot_maze' / 'maze_topology.py').read_text()
    assert "GOAL_CANCELED_AFTER_EXIT = 'goal_canceled_after_exit'" in topology


def test_maze_explorer_cancels_active_goal_when_exit_is_reached():
    explorer = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()
    for required in [
        '_enter_terminal_state(',
        "terminal_reason='exit_reached'",
        '_cancel_active_goal_for_terminal_state',
        'cancel_goal_async()',
        'self.terminal_cancel_count += 1',
        'self.terminal_cancel_goal_sequence_id',
        'self.last_terminal_reason',
        'self.terminal_state_entered',
    ]:
        assert required in explorer


def test_maze_explorer_does_not_count_terminal_cancel_as_blocked_nav2():
    explorer = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()
    for required in [
        '_is_terminal_canceled_goal_result(sequence_id)',
        '_mark_goal_canceled_after_exit(sequence_id, status)',
        'GOAL_CANCELED_AFTER_EXIT',
        'self.canceled_after_exit_count += 1',
        'return',
    ]:
        assert required in explorer
    terminal_cancel_block = explorer[explorer.index('def _mark_goal_canceled_after_exit'):explorer.index('def _start_goal_settle_cooldown')]
    assert 'self.goal_failure_count += 1' not in terminal_cancel_block
    assert 'self.nav2_failure_count += 1' not in terminal_cancel_block
    assert 'record_branch_failure' not in terminal_cancel_block


def test_maze_explorer_reports_phase10_terminal_diagnostics():
    explorer = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()
    for required in [
        "'terminal_cancel_count'",
        "'canceled_after_exit_count'",
        "'terminal_cancel_goal_sequence_id'",
        "'last_terminal_reason'",
        "'terminal_state_entered'",
    ]:
        assert required in explorer
