"""The Go2 gait must match its real model and remain within its joint limits."""
import math

from tugbot_maze.legged.kinematics import LEGS, leg_fk, leg_ik
from tugbot_maze.legged.params import LeggedParams, NEUTRAL_FOOT, STAND_POSE
from tugbot_maze.legged.trot import LocomotionFSM


def test_go2_stance_matches_official_joint_origins():
    expected = {'LF': (0.1934, 0.142, -0.32), 'RF': (0.1934, -0.142, -0.32),
                'LH': (-0.1934, 0.142, -0.32), 'RH': (-0.1934, -0.142, -0.32)}
    for leg in LEGS:
        assert math.dist(NEUTRAL_FOOT[leg], expected[leg]) < 1e-8
        angles = [STAND_POSE[f'{leg}_{axis}'] for axis in ('HAA', 'HFE', 'KFE')]
        assert math.dist(leg_fk(leg, angles), expected[leg]) < 1e-8
        recovered, residual = leg_ik(leg, expected[leg], seed=angles)
        assert residual < 1e-8
        assert math.dist(angles, recovered) < 1e-8


def test_go2_command_domain_respects_official_joint_limits():
    params = LeggedParams()
    for vx in (-params.vx_max, 0.0, params.vx_max):
        for wz in (-params.wz_max, 0.0, params.wz_max):
            fsm = LocomotionFSM(params)
            for _ in range(600):
                angles = fsm.step(0.01, vx, wz, 0.03, -0.03)
                for leg in LEGS:
                    assert -1.0472 <= angles[f'{leg}_HAA'] <= 1.0472
                    low = -1.5708 if leg in ('LF', 'RF') else -0.5236
                    high = 3.4907 if leg in ('LF', 'RF') else 4.5379
                    assert low <= angles[f'{leg}_HFE'] <= high
                    assert -2.7227 <= angles[f'{leg}_KFE'] <= -0.83776


def test_go2_returns_to_stand_after_commands_stop():
    fsm = LocomotionFSM()
    for _ in range(450):
        fsm.step(0.01, 0.2, 0.2, 0.0, 0.0)
    assert fsm.mode == fsm.TROT
    for _ in range(250):
        targets = fsm.step(0.01, 0.0, 0.0, 0.0, 0.0)
    assert fsm.mode == fsm.STAND
    assert max(abs(targets[j]-q) for j,q in STAND_POSE.items()) < 1e-8
