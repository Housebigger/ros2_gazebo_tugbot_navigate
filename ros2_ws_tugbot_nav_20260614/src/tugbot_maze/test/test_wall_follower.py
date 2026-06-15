import math
import pytest
from tugbot_maze.wall_follower import Sectors, Command, State, sectorize, WallFollower


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
    assert s.front == 12.0 and s.side == 12.0 and s.front_side == 12.0


def test_sectorize_invalid_follow_side_raises():
    with pytest.raises(ValueError):
        sectorize([1.0], -math.pi, math.pi, 'center')


def test_sectorize_front_side_window_right_hand():
    # Wall close at -45 deg (right-forward diagonal) at 1.0 m.
    def a2r(ang):
        return 1.0 if abs(math.atan2(math.sin(ang + math.pi / 4),
                                     math.cos(ang + math.pi / 4))) <= math.radians(10) else 5.0
    ranges, amin, ainc = _ranges_at(a2r)
    s = sectorize(ranges, amin, ainc, 'right')
    assert s.front_side == pytest.approx(1.0, abs=1e-6)
    assert s.front == pytest.approx(5.0, abs=1e-6)


# ---- WallFollower policy --------------------------------------------------

def _open():  # everything far: open corridor, no wall engaged
    return Sectors(front=5.0, side=5.0, front_side=5.0)


def test_find_wall_creeps_forward_when_all_open():
    wf = WallFollower(follow_side='right')
    cmd = wf.update(_open())
    assert cmd.v > 0 and cmd.w == 0.0
    assert wf.state == State.FIND_WALL


def test_front_blocked_turns_away_in_place_right_hand():
    wf = WallFollower(follow_side='right')
    cmd = wf.update(Sectors(front=0.5, side=0.6, front_side=0.6))
    assert cmd.v == pytest.approx(0.0)
    assert cmd.w > 0          # right-hand turns LEFT (+w) away from the right wall
    assert wf.state == State.TURN_AWAY


def test_front_blocked_turns_away_mirrored_left_hand():
    wf = WallFollower(follow_side='left')
    cmd = wf.update(Sectors(front=0.5, side=0.6, front_side=0.6))
    assert cmd.w < 0          # left-hand turns RIGHT (-w)


def test_wall_lost_corners_toward_wall_right_hand():
    wf = WallFollower(follow_side='right')
    wf.state = State.FOLLOW                       # already engaged, then the wall ends
    cmd = wf.update(Sectors(front=5.0, side=1.5, front_side=5.0))
    assert cmd.v > 0          # keep moving while rounding
    assert cmd.w < 0          # right-hand arcs RIGHT (-w) toward the lost wall
    assert wf.state == State.CORNER


def test_follow_steers_toward_wall_when_too_far_right_hand():
    wf = WallFollower(follow_side='right')
    wf.state = State.FOLLOW
    # side (0.9) > target (0.6): too far from the right wall -> steer right (-w)
    cmd = wf.update(Sectors(front=5.0, side=0.9, front_side=5.0))
    assert cmd.w < 0
    assert wf.state == State.FOLLOW


def test_follow_steers_away_from_wall_when_too_close_right_hand():
    wf = WallFollower(follow_side='right')
    wf.state = State.FOLLOW
    # side (0.3) < target (0.6): too close -> steer left (+w) away from wall
    cmd = wf.update(Sectors(front=5.0, side=0.3, front_side=5.0))
    assert cmd.w > 0


def test_follow_w_is_clamped_to_w_max():
    wf = WallFollower(follow_side='right', w_max=0.8)
    wf.state = State.FOLLOW
    cmd = wf.update(Sectors(front=5.0, side=11.0, front_side=5.0))  # huge error
    assert abs(cmd.w) <= 0.8 + 1e-9


def test_corner_hysteresis_holds_until_wall_regained():
    wf = WallFollower(follow_side='right', min_state_ticks=3)
    wf.state = State.FOLLOW                                         # already engaged
    wf.update(Sectors(front=5.0, side=1.5, front_side=5.0))         # enter CORNER
    assert wf.state == State.CORNER
    # side between target (0.6) and wall_lost (1.2): stay cornering (hysteresis)
    wf.update(Sectors(front=5.0, side=0.9, front_side=5.0))
    assert wf.state == State.CORNER
    # wall regained (side <= target) but dwell not yet elapsed: still CORNER
    wf.update(Sectors(front=5.0, side=0.5, front_side=5.0))
    assert wf.state == State.CORNER
    # dwell elapsed AND wall regained: back to FOLLOW
    wf.update(Sectors(front=5.0, side=0.5, front_side=5.0))
    assert wf.state == State.FOLLOW


def test_invalid_follow_side_raises():
    with pytest.raises(ValueError):
        WallFollower(follow_side='up')
