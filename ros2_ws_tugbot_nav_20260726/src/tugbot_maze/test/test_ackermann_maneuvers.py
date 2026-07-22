"""N-point-turn planning for the Ackermann chassis: pure math, no ROS."""
import math

from tugbot_maze.ackermann_maneuvers import (
    NPointTurnRunner, plan_n_point_turn, simulate_segments)


def _final_yaw(segs, yaw0=0.0):
    yaw, _ = simulate_segments(segs, yaw0)
    return yaw


def test_plan_empty_when_aligned():
    assert plan_n_point_turn(1.2, 1.2) == []


def test_plan_converges_90_in_two_segments():
    segs = plan_n_point_turn(0.0, math.pi / 2.0)
    assert len(segs) == 2
    assert abs(_final_yaw(segs) - math.pi / 2.0) < 1e-9


def test_plan_converges_180_in_three_segments():
    segs = plan_n_point_turn(0.0, math.pi)
    assert len(segs) == 3
    assert abs(abs(_final_yaw(segs)) - math.pi) < 1e-9


def test_plan_takes_shortest_direction():
    # 3.0 -> -3.0 rad: shortest is +0.283 (through pi), not -6.0.
    segs = plan_n_point_turn(3.0, -3.0)
    total = sum(v * L * k for v, k, L in segs)
    assert 0 < total < 0.3


def test_alternating_drive_direction():
    segs = plan_n_point_turn(0.0, math.pi)
    assert [s[0] for s in segs] == [1, -1, 1]


def test_curvature_at_limit_every_segment():
    for v, k, L in plan_n_point_turn(0.0, math.pi / 2.0, max_curvature=2.4):
        assert abs(k) == 2.4


def test_excursion_bounded():
    for target in (math.pi / 2.0, math.pi, -math.pi / 2.0):
        segs = plan_n_point_turn(0.0, target, excursion_limit=0.5)
        _, exc = simulate_segments(segs, 0.0)
        assert exc <= 0.5 + 1e-9


def test_runner_never_emits_in_place_rotation():
    r = NPointTurnRunner(0.0, math.pi / 2.0, t_now=0.0)
    t, cmds = 0.0, []
    while True:
        v, w, done = r.command(t)
        cmds.append((v, w))
        if done:
            break
        t += 0.1
        assert t < 60.0, 'runner never finished'
    assert any(abs(w) > 0.02 for _, w in cmds)              # it does turn
    for v, w in cmds:
        if abs(w) > 0.02:
            assert abs(v) >= 0.01                            # ... but never in place


def test_runner_pauses_between_segments():
    r = NPointTurnRunner(0.0, math.pi, t_now=0.0, v_mag=0.15, pause_s=0.6)
    t, saw_pause_after_motion = 0.0, False
    moving_prev = False
    while True:
        v, w, done = r.command(t)
        if done:
            break
        if moving_prev and v == 0.0 and w == 0.0:
            saw_pause_after_motion = True                    # steering-rack settle gap
        moving_prev = abs(v) > 0.0
        t += 0.05
    assert saw_pause_after_motion
