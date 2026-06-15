import math
import pytest
from tugbot_maze.wall_follower import Sectors, Command, State, sectorize


# ---- sectorize -----------------------------------------------------------

def _ranges_at(angle_to_range, n=360, max_range=12.0):
    """Build a ranges list of length n over [-pi, pi); angle_to_range(ang)->meters."""
    amin = -math.pi
    ainc = 2 * math.pi / n
    return [angle_to_range(amin + i * ainc) for i in range(n)], amin, ainc


def test_sectorize_picks_min_in_each_window_right_hand():
    # Wall close on the RIGHT (-pi/2) at 0.5 m, open elsewhere at 5 m.
    def a2r(ang):
        return 0.5 if abs(math.atan2(math.sin(ang + math.pi / 2),
                                     math.cos(ang + math.pi / 2))) <= math.radians(10) else 5.0
    ranges, amin, ainc = _ranges_at(a2r)
    s = sectorize(ranges, amin, ainc, 'right')
    assert s.side == pytest.approx(0.5, abs=1e-6)
    assert s.front == pytest.approx(5.0, abs=1e-6)


def test_sectorize_front_window_sees_obstacle_ahead():
    def a2r(ang):
        return 0.8 if abs(ang) <= math.radians(10) else 5.0
    ranges, amin, ainc = _ranges_at(a2r)
    s = sectorize(ranges, amin, ainc, 'right')
    assert s.front == pytest.approx(0.8, abs=1e-6)


def test_sectorize_left_hand_reads_left_side():
    # Wall close on the LEFT (+pi/2); right side open.
    def a2r(ang):
        return 0.6 if abs(math.atan2(math.sin(ang - math.pi / 2),
                                     math.cos(ang - math.pi / 2))) <= math.radians(10) else 5.0
    ranges, amin, ainc = _ranges_at(a2r)
    s = sectorize(ranges, amin, ainc, 'left')
    assert s.side == pytest.approx(0.6, abs=1e-6)


def test_sectorize_sanitizes_inf_nan_and_nonpositive():
    ranges = [float('inf'), float('nan'), 0.0, -1.0]  # all invalid
    amin, ainc = -math.pi, math.pi / 2
    s = sectorize(ranges, amin, ainc, 'right', max_range=9.0)
    assert s.front == 9.0 and s.side == 9.0 and s.front_side == 9.0


def test_sectorize_clamps_to_max_range():
    ranges, amin, ainc = _ranges_at(lambda ang: 99.0)
    s = sectorize(ranges, amin, ainc, 'right', max_range=12.0)
    assert s.front == 12.0
