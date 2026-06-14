from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MAZE_PKG = ROOT / 'src' / 'tugbot_maze'
BRINGUP_PKG = ROOT / 'src' / 'tugbot_bringup'


def test_phase13_maze_explorer_declares_near_exit_timeout_extension_parameters():
    source = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()

    assert "near_exit_timeout_extension_radius_m" in source
    assert "near_exit_goal_timeout_sec" in source
    assert "_effective_goal_timeout_sec" in source
    assert "_active_goal_near_exit" in source


def test_phase13_timeout_check_uses_effective_timeout_not_raw_timeout():
    source = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()

    assert "self._effective_goal_timeout_sec()" in source
    assert "elapsed.nanoseconds / 1e9 > self.goal_timeout_sec" not in source


def test_phase13_state_publishes_effective_timeout_diagnostics():
    source = (MAZE_PKG / 'tugbot_maze' / 'maze_explorer.py').read_text()

    required = [
        "effective_goal_timeout_sec",
        "active_goal_near_exit",
        "near_exit_goal_timeout_sec",
        "near_exit_timeout_extension_radius_m",
    ]
    for field in required:
        assert field in source


def test_phase13_bringup_exposes_near_exit_timeout_launch_args():
    launch = (BRINGUP_PKG / 'launch' / 'tugbot_maze_explore.launch.py').read_text()

    assert "near_exit_timeout_extension_radius_m" in launch
    assert "near_exit_goal_timeout_sec" in launch
    assert "Near-exit" in launch or "near exit" in launch.lower()
