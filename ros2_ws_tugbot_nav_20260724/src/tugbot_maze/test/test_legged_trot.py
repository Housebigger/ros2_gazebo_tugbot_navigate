import math

import pytest

from tugbot_maze.legged.params import LeggedParams, STAND_POSE
from tugbot_maze.legged.stabilizer import height_offsets
from tugbot_maze.legged.trot import PAIR_PHASE, LocomotionFSM

P = LeggedParams()


def test_stabilizer_signs():
    """+roll = left side up = left legs too long -> raise left foot targets (dz>0)."""
    dz = height_offsets(0.1, 0.0, P)
    assert dz['LF'] > 0 and dz['LH'] > 0 and dz['RF'] < 0 and dz['RH'] < 0
    # +pitch = nose down = front legs too short -> extend front (dz<0)
    dz = height_offsets(0.0, 0.1, P)
    assert dz['LF'] < 0 and dz['RF'] < 0 and dz['LH'] > 0 and dz['RH'] > 0


def test_stabilizer_clamped():
    dz = height_offsets(1.0, -1.0, P)
    assert all(abs(v) <= P.dz_max + 1e-12 for v in dz.values())


def test_pair_phase_diagonal():
    assert PAIR_PHASE['LF'] == PAIR_PHASE['RH'] == 0.0
    assert PAIR_PHASE['RF'] == PAIR_PHASE['LH'] == pytest.approx(math.pi)


def _run(fsm, seconds, vx, wz, dt=0.01, roll=0.0, pitch=0.0, force_trot=False):
    out = None
    for _ in range(int(seconds / dt)):
        out = fsm.step(dt, vx, wz, roll, pitch, force_trot=force_trot)
    return out


def test_init_holds_stand_pose_then_stand():
    fsm = LocomotionFSM()
    out = fsm.step(0.01, 0.4, 0.0, 0.0, 0.0)
    assert fsm.mode == fsm.INIT
    assert out == pytest.approx(STAND_POSE, abs=1e-9)
    _run(fsm, P.init_s + 0.1, 0.0, 0.0)
    assert fsm.mode == fsm.STAND


def test_stand_trot_hysteresis():
    fsm = LocomotionFSM()
    _run(fsm, P.init_s + 0.1, 0.0, 0.0)
    _run(fsm, 1.0, 0.3, 0.0)
    assert fsm.mode == fsm.TROT
    # window = slew-down time (0.3/ax_max) + quiet dwell + margin: the dwell
    # clock only starts once the slewed cmd decays below the half-threshold
    _run(fsm, 0.3 / P.ax_max + P.stand_dwell_s + 0.3, 0.0, 0.0)
    assert fsm.mode == fsm.STAND


def test_force_trot_diagnostic():
    fsm = LocomotionFSM()
    _run(fsm, P.init_s + 0.1, 0.0, 0.0)
    _run(fsm, 0.2, 0.0, 0.0, force_trot=True)
    assert fsm.mode == fsm.TROT


def test_trot_outputs_all_joints_within_limits():
    from tugbot_maze.legged.kinematics import JOINTS, HAA_LIMITS
    fsm = LocomotionFSM()
    _run(fsm, P.init_s + 0.1, 0.0, 0.0)
    for _ in range(200):
        out = fsm.step(0.01, P.vx_max, P.wz_max, 0.02, -0.02)
        assert sorted(out) == sorted(JOINTS)
        for leg, (lo, hi) in HAA_LIMITS.items():
            assert lo - 1e-9 <= out[f'{leg}_HAA'] <= hi + 1e-9
        assert all(math.isfinite(v) for v in out.values())


def test_slew_limits_cmd():
    fsm = LocomotionFSM()
    _run(fsm, P.init_s + 0.1, 0.0, 0.0)
    fsm.step(0.01, 10.0, 10.0, 0.0, 0.0)   # absurd cmd
    assert abs(fsm.vx) <= P.ax_max * 0.011
    assert abs(fsm.wz) <= P.aw_max * 0.011
    for _ in range(500):
        fsm.step(0.01, 10.0, 10.0, 0.0, 0.0)
    assert abs(fsm.vx) <= P.vx_max + 1e-9 and abs(fsm.wz) <= P.wz_max + 1e-9


def test_stand_applies_attitude_feedback():
    fsm = LocomotionFSM()
    _run(fsm, P.init_s + 0.1, 0.0, 0.0)
    flat = dict(fsm.step(0.01, 0.0, 0.0, 0.0, 0.0))
    rolled = dict(fsm.step(0.01, 0.0, 0.0, 0.2, 0.0))
    assert rolled != pytest.approx(flat)   # feedback changes the pose


def test_transition_output_continuity():
    """C1 regression: joint targets must stay gait-sized across STAND->TROT
    and TROT->STAND transitions. Entry resets phase to a grounded boundary;
    exit waits for one — without these, a stale mid-swing phase snapped a
    lifted foot by up to ~0.35 rad in one tick (6-7x normal gait motion)."""
    fsm = LocomotionFSM()
    _run(fsm, P.init_s + 0.1, 0.0, 0.0)
    prev = fsm.step(0.01, 0.0, 0.0, 0.0, 0.0)
    max_delta = 0.0
    modes = set()
    for push in (0.6, 0.85, 1.1, 1.35, 1.6):   # varied durations scan phase offsets
        for _ in range(int(push / 0.01)):
            out = fsm.step(0.01, 0.35, 0.0, 0.0, 0.0)
            max_delta = max(max_delta, max(abs(out[j] - prev[j]) for j in out))
            prev = out
            modes.add(fsm.mode)
        for _ in range(int(2.0 / 0.01)):
            out = fsm.step(0.01, 0.0, 0.0, 0.0, 0.0)
            max_delta = max(max_delta, max(abs(out[j] - prev[j]) for j in out))
            prev = out
            modes.add(fsm.mode)
        assert fsm.mode == fsm.STAND   # the boundary gate must not deadlock the exit
    assert modes >= {LocomotionFSM.STAND, LocomotionFSM.TROT}
    assert max_delta < 0.12, max_delta


def test_init_ignores_attitude_and_force_trot():
    """INIT holds the raw stand pose: no attitude feedback (model may be
    mid-drop) and force_trot is deliberately ignored until settle ends."""
    fsm = LocomotionFSM()
    out = fsm.step(0.01, 0.0, 0.0, 0.5, -0.5, force_trot=True)
    assert fsm.mode == fsm.INIT
    assert out == pytest.approx(STAND_POSE, abs=1e-9)
