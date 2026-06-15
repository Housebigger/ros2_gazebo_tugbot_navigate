from tugbot_maze.wall_follow_control import exit_reached, entering_done, StallWatchdog


def test_exit_reached_within_radius():
    assert exit_reached((21.0, 18.0, 0.0), (21.07, 18.08), 1.2) is True


def test_exit_not_reached_outside_radius():
    assert exit_reached((10.0, 5.0, 0.0), (21.07, 18.08), 1.2) is False


def test_entering_done_after_distance():
    assert entering_done((0.0, 0.0), (2.1, 0.0), 2.0) is True
    assert entering_done((0.0, 0.0), (1.5, 0.0), 2.0) is False


def test_stall_watchdog_first_sample_not_stalled():
    wd = StallWatchdog(stall_s=4.0, progress_eps_m=0.2)
    assert wd.update(0.0, 0.0, 0.0) is False


def test_stall_watchdog_triggers_without_progress():
    wd = StallWatchdog(stall_s=4.0, progress_eps_m=0.2)
    wd.update(0.0, 0.0, 0.0)            # anchor
    assert wd.update(2.0, 0.05, 0.0) is False     # < eps, < stall_s
    assert wd.update(4.5, 0.05, 0.0) is True      # < eps moved, >= stall_s elapsed


def test_stall_watchdog_resets_on_progress():
    wd = StallWatchdog(stall_s=4.0, progress_eps_m=0.2)
    wd.update(0.0, 0.0, 0.0)
    assert wd.update(5.0, 1.0, 0.0) is False       # moved >= eps -> re-anchor, not stalled
    assert wd.update(6.0, 1.0, 0.0) is False       # only 1 s since new anchor


def test_stall_watchdog_explicit_reset():
    wd = StallWatchdog(stall_s=4.0, progress_eps_m=0.2)
    wd.update(0.0, 0.0, 0.0)
    wd.update(3.0, 0.0, 0.0)
    wd.reset(10.0, 0.0, 0.0)
    assert wd.update(13.9, 0.0, 0.0) is False      # < stall_s since reset
    assert wd.update(14.1, 0.0, 0.0) is True
