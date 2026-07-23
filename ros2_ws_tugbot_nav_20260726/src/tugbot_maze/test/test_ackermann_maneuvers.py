"""N-point-turn planning for the Ackermann chassis: pure math, no ROS."""
import math

from tugbot_maze.ackermann_maneuvers import (
    NPointTurnRunner, plan_n_point_turn, simulate_segments, clamp_to_ackermann,
    plan_back_and_arc)


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


def test_clamp_passes_normal_driving_untouched():
    assert clamp_to_ackermann(0.4, 0.5) == (0.4, 0.5)      # cruise cap 0.96 > 0.5
    assert clamp_to_ackermann(0.3, 0.0) == (0.3, 0.0)      # straight
    assert clamp_to_ackermann(-0.3, -0.4) == (-0.3, -0.4)  # reverse arc within cap


def test_clamp_projects_pivot_to_slow_reverse_arc():
    # Pure pivot (v exactly 0): retreat while steering -- the stop-and-reorient
    # regime suppresses front_block, so forward creep would close on the obstacle.
    v, w = clamp_to_ackermann(0.0, -0.5)
    assert v == -0.08 and abs(w) <= abs(v) * 2.4 + 1e-12 and w < 0
    v2, w2 = clamp_to_ackermann(0.02, 0.5)                  # small FORWARD intent keeps its sign
    assert v2 == 0.08 and w2 == min(0.5, v2 * 2.4)


def test_simulate_handles_straight_segments():
    yaw, exc = simulate_segments([(-1, 0.0, 0.6)], 0.0)
    assert yaw == 0.0 and abs(exc - 0.6) < 1e-9


def test_back_and_arc_geometry_left():
    segs = plan_back_and_arc(+1, 0.6)
    assert segs[0] == (-1, 0.0, 0.6)                      # straight reverse, no steering
    yaw, _ = simulate_segments(segs, 0.0)
    assert abs(yaw - math.pi / 2.0) < 1e-9                # ends heading +90
    # endpoint: back 0.6 along -x, then quarter-arc -> net displacement (0, +0.6)
    # relative to the start (the junction center): verify via fine integration
    x = y = 0.0; hdg = 0.0
    for v_sign, k, L in segs:
        # n=1000 leaves a forward-Euler truncation error (~4.7e-4, O(1/n)) that
        # cannot clear the 1e-6 tolerance below; 1e6 steps pushes it to ~4.7e-7.
        n = 1_000_000
        for _ in range(n):
            s = v_sign * L / n
            x += s * math.cos(hdg); y += s * math.sin(hdg)
            hdg += s * k
    assert abs(x - 0.0) < 1e-6 and abs(y - 0.6) < 1e-6    # on the new centerline, R past center


def test_back_and_arc_curvature_within_limit():
    for d in (0.45, 0.5, 0.65):
        for v_sign, k, L in plan_back_and_arc(-1, d):
            assert abs(k) <= 2.4 + 1e-9


def test_back_and_arc_rejects_insufficient_room():
    import pytest
    with pytest.raises(ValueError):
        plan_back_and_arc(+1, 0.44)


def test_runner_accepts_injected_segments():
    r = NPointTurnRunner(0.0, math.pi / 2.0, t_now=0.0, segments=plan_back_and_arc(+1, 0.6))
    t, saw_reverse, saw_forward_arc = 0.0, False, False
    while True:
        v, w, done = r.command(t)
        if done: break
        if v < 0 and abs(w) < 1e-9: saw_reverse = True
        if v > 0 and abs(w) > 0.02: saw_forward_arc = True
        t += 0.1
        assert t < 60.0
    assert saw_reverse and saw_forward_arc
