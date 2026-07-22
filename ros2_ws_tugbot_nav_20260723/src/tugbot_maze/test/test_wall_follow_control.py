import math
import pytest
from tugbot_maze.wall_follow_control import (
    exit_reached, entering_done, StallWatchdog,
    entrance_seal_segment, fuse_virtual_segment)


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


def test_entrance_seal_segment_left_is_vertical():
    # west-wall opening (vertical wall) -> seal spans y at constant x
    seg = entrance_seal_segment((0.95, 0.0), 2.072423, 'left')
    assert seg == pytest.approx((0.95, -1.0362115, 0.95, 1.0362115))


def test_entrance_seal_segment_top_is_horizontal():
    # top/bottom opening (horizontal wall) -> seal spans x at constant y
    seg = entrance_seal_segment((4.0, 9.0), 2.0, 'top')
    assert seg == pytest.approx((3.0, 9.0, 5.0, 9.0))


def test_entrance_seal_segment_bad_side_raises():
    with pytest.raises(ValueError):
        entrance_seal_segment((0.0, 0.0), 1.0, 'sideways')


def test_fuse_virtual_segment_beam_hits_seal():
    # single beam pointing west (yaw=pi, beam angle 0) -> hits the seal at x=0.95
    seg = (0.95, -1.0, 0.95, 1.0)
    fused = fuse_virtual_segment([12.0], 0.0, 0.0, (2.0, 0.0, math.pi), seg,
                                 wall_half_thickness_m=0.12, max_range=12.0)
    assert fused[0] == pytest.approx(0.93, abs=1e-6)   # 1.05 - 0.12


def test_fuse_virtual_segment_beam_pointing_away_unchanged():
    # beam pointing east (yaw=0) -> seal is behind the ray -> range unchanged
    seg = (0.95, -1.0, 0.95, 1.0)
    fused = fuse_virtual_segment([5.0], 0.0, 0.0, (2.0, 0.0, 0.0), seg)
    assert fused[0] == pytest.approx(5.0)


def test_fuse_virtual_segment_keeps_nearer_real_obstacle():
    # a real return at 0.5 m is nearer than the seal (0.93) -> keep 0.5
    seg = (0.95, -1.0, 0.95, 1.0)
    fused = fuse_virtual_segment([0.5], 0.0, 0.0, (2.0, 0.0, math.pi), seg)
    assert fused[0] == pytest.approx(0.5)


def test_fuse_virtual_segment_nonfinite_real_uses_seal():
    seg = (0.95, -1.0, 0.95, 1.0)
    fused = fuse_virtual_segment([float('inf')], 0.0, 0.0, (2.0, 0.0, math.pi), seg)
    assert fused[0] == pytest.approx(0.93, abs=1e-6)


def test_fuse_virtual_segment_parallel_beam_no_crash():
    # beam pointing +y (up); the seal segment is also vertical -> denom ~= 0 ->
    # guarded (|denom| <= 1e-12), so no crash and the range is unchanged.
    seg = (0.95, -1.0, 0.95, 1.0)
    fused = fuse_virtual_segment([5.0], math.pi / 2, 0.0, (2.0, 0.0, 0.0), seg)
    assert fused[0] == pytest.approx(5.0)
