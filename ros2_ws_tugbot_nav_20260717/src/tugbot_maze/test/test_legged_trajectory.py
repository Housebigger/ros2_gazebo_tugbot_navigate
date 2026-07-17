import math

import pytest

from tugbot_maze.legged.params import (
    LeggedParams, NEUTRAL_FOOT, STAND_POSE, foot_envelope,
)
from tugbot_maze.legged.kinematics import LEGS, leg_ik
from tugbot_maze.legged.trajectory import foot_displacement

P = LeggedParams()


def test_stand_pose_is_ik_of_neutral_foot():
    for leg in LEGS:
        seed = (STAND_POSE[f'{leg}_HAA'], STAND_POSE[f'{leg}_HFE'], STAND_POSE[f'{leg}_KFE'])
        q, err = leg_ik(leg, NEUTRAL_FOOT[leg], seed=seed)
        assert err < 1e-6
        assert q == pytest.approx(list(seed), abs=1e-3)


def test_zero_cmd_steps_in_place():
    """Zero command: never any xy stride; the swing z-lift still applies so
    that force_trot steps in place (ladder rung 2). The FSM's STAND mode is
    what holds the feet fully still — not the trajectory."""
    for leg in LEGS:
        for ph in (0.0, 1.0, 2.0, 4.0):
            d = foot_displacement(leg, ph, 0.0, 0.0, P)
            assert d[0] == 0.0 and d[1] == 0.0
            assert 0.0 <= d[2] <= P.swing_lift + 1e-12


def test_stance_swing_continuity():
    """Displacement is continuous at the stance->swing and swing->stance boundaries."""
    for ph_lo, ph_hi in ((P.duty * 2 * math.pi - 1e-6, P.duty * 2 * math.pi + 1e-6),
                         (2 * math.pi - 1e-6, 1e-6)):
        a = foot_displacement('LF', ph_lo, 0.3, 0.2, P)
        b = foot_displacement('LF', ph_hi, 0.3, 0.2, P)
        assert a == pytest.approx(b, abs=1e-3)


def test_swing_lifts_stance_stays_down():
    mid_stance = foot_displacement('LF', P.duty * math.pi, 0.3, 0.0, P)
    mid_swing = foot_displacement('LF', (1 + P.duty) * math.pi, 0.3, 0.0, P)
    assert mid_stance[2] == 0.0
    assert mid_swing[2] == pytest.approx(P.swing_lift, abs=1e-9)


def test_stance_moves_backward_under_forward_cmd():
    early = foot_displacement('LF', 0.1, 0.4, 0.0, P)
    late = foot_displacement('LF', P.duty * 2 * math.pi - 0.1, 0.4, 0.0, P)
    assert early[0] > late[0]          # foot drifts backward relative to body
    assert early[1] == late[1] == 0.0  # no lateral component for pure vx


def test_yaw_cmd_gives_tangential_strides():
    dlf = foot_displacement('LF', 0.0, 0.0, 0.5, P)
    drh = foot_displacement('RH', 0.0, 0.0, 0.5, P)
    # LF at (+0.34,+0.3012): tangent (-wz*ny, wz*nx) points back-left; RH mirrored
    assert dlf[0] < 0 and dlf[1] > 0
    assert drh[0] > 0 and drh[1] < 0


def test_envelope_bound_values():
    ex, ey = foot_envelope()
    assert ex == pytest.approx(0.439, abs=5e-3)
    assert ey == pytest.approx(0.353, abs=5e-3)


def test_all_targets_reachable_across_cycle():
    """Every commanded foot target over a full cycle at max cmd is IK-reachable."""
    for leg in LEGS:
        seed = (STAND_POSE[f'{leg}_HAA'], STAND_POSE[f'{leg}_HFE'], STAND_POSE[f'{leg}_KFE'])
        for i in range(24):
            ph = 2 * math.pi * i / 24
            d = foot_displacement(leg, ph, P.vx_max, P.wz_max, P)
            n = NEUTRAL_FOOT[leg]
            _q, err = leg_ik(leg, (n[0] + d[0], n[1] + d[1], n[2] + d[2]), seed=seed)
            assert err < 1e-4, (leg, ph)
