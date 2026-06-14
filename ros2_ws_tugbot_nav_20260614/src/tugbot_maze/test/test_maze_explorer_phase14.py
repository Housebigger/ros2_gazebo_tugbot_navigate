from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MAZE_PKG = ROOT / 'src' / 'tugbot_maze'
BRINGUP_PKG = ROOT / 'src' / 'tugbot_bringup'
TOOLS = ROOT / 'tools'


def test_phase14_maze_explorer_publishes_goal_events_topic():
    source = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()

    assert "goal_events_topic" in source
    assert "'/maze/goal_events'" in source
    assert "self.goal_events_pub" in source
    assert "_publish_goal_event" in source


def test_phase14_goal_event_schema_contains_root_cause_fields():
    source = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()
    required = [
        "goal_sequence",
        "event",
        "dispatch_pose",
        "target",
        "target_exit_dist",
        "robot_exit_dist_at_dispatch",
        "local_topology",
        "branch_angle",
        "goal_kind",
        "effective_timeout_sec",
        "result_status",
        "result_reason",
        "elapsed_sec",
        "near_exit",
        "branch_failure_state",
        "caused_branch_failure",
        "caused_blacklist",
    ]
    for field in required:
        assert field in source


def test_phase14_goal_lifecycle_emits_dispatch_and_terminal_events():
    source = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()
    required = [
        "self._publish_goal_event('dispatch'",
        "self._publish_goal_event('success'",
        "self._publish_goal_event('timeout'",
        "self._publish_goal_event('timeout_cancel_result'",
        "self._publish_goal_event('terminal_cancel'",
        "self._publish_goal_event('terminal_cancel_result'",
        "self._publish_goal_event('stale_result'",
    ]
    for snippet in required:
        assert snippet in source


def test_phase14_bringup_exposes_goal_events_topic():
    launch = (BRINGUP_PKG / 'launch' / 'tugbot_maze_explore.launch.py').read_text()

    assert "goal_events_topic" in launch
    assert "/maze/goal_events" in launch


def test_phase14_generic_recorder_supports_goal_events_name():
    recorder = (TOOLS / 'record_explorer_state_series.py').read_text()

    assert "--topic" in recorder
    assert "/maze/goal_events" in recorder
    assert "last_event" in recorder
