from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MAZE_PKG = ROOT / 'src' / 'tugbot_maze'
TOOLS = ROOT / 'tools'


def test_phase11_topology_declares_timeout_cancel_reason():
    topology = (MAZE_PKG / 'tugbot_maze' / 'maze_topology.py').read_text()
    assert "GOAL_CANCELED_AFTER_TIMEOUT = 'goal_canceled_after_timeout'" in topology
    assert "GOAL_CANCELED_AFTER_EXIT = 'goal_canceled_after_exit'" in topology


def test_phase11_timeout_failure_cancels_nav2_goal_before_clearing_handle():
    explorer = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()
    for required in [
        '_cancel_goal_after_timeout()',
        'self.timeout_cancel_count += 1',
        'self.timeout_cancel_goal_sequence_id',
        'GOAL_CANCELED_AFTER_TIMEOUT',
        '_mark_goal_canceled_after_timeout(sequence_id, status)',
    ]:
        assert required in explorer
    timeout_call = "if reason == GOAL_TIMEOUT:\n            self._cancel_goal_after_timeout()"
    assert timeout_call in explorer
    failure_block = explorer[explorer.index('def _handle_goal_failure'):explorer.index('def _is_stale_goal_result')]
    assert failure_block.index('_cancel_goal_after_timeout()') < failure_block.index('self.goal_handle = None')


def test_phase11_terminal_cancel_uses_goal_handle_not_goal_active_only():
    explorer = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()
    block = explorer[explorer.index('def _cancel_active_goal_for_terminal_state'):explorer.index('def _mark_goal_preempted')]
    assert 'if self.goal_handle is None:' in block
    assert 'if not self.goal_active or self.goal_handle is None' not in block
    assert 'cancel_goal_async()' in block


def test_phase11_canceled_results_are_classified_by_cancel_reason():
    explorer = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()
    for required in [
        '_is_timeout_canceled_goal_result(sequence_id)',
        '_is_terminal_canceled_goal_result(sequence_id)',
        '_mark_goal_canceled_after_timeout(sequence_id, status)',
        '_mark_goal_canceled_after_exit(sequence_id, status)',
        'self.canceled_after_timeout_count += 1',
        'self.canceled_after_exit_count += 1',
        "self.last_failure_reason = GOAL_CANCELED_AFTER_TIMEOUT",
        "self.last_failure_reason = GOAL_CANCELED_AFTER_EXIT",
    ]:
        assert required in explorer
    timeout_block = explorer[explorer.index('def _mark_goal_canceled_after_timeout'):explorer.index('def _start_goal_settle_cooldown')]
    assert 'self.goal_failure_count += 1' not in timeout_block
    assert 'self.nav2_failure_count += 1' not in timeout_block
    assert 'record_branch_failure' not in timeout_block


def test_phase11_state_json_reports_timeout_cancel_diagnostics():
    explorer = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()
    for required in [
        "'timeout_cancel_count'",
        "'canceled_after_timeout_count'",
        "'timeout_cancel_goal_sequence_id'",
    ]:
        assert required in explorer


def test_phase11_recorder_can_linger_after_terminal_state():
    recorder = (TOOLS / 'record_explorer_state_series.py').read_text()
    for required in [
        '--terminal-linger-sec',
        'self.terminal_first_seen_at',
        'self.args.terminal_linger_sec',
        'terminal_linger_sec',
        "last_mode in TERMINAL_MODES",
    ]:
        assert required in recorder
    assert 'self.done = True' in recorder
