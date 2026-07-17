"""Tests for the ANYmal C leg FK/IK. The FK chain anchors are locked to the
SDF by test_chain_matches_sdf, so any drift between code constants and the
model is caught here, not in Gazebo."""
import math
import re
from pathlib import Path

import pytest

from tugbot_maze.legged.kinematics import (
    LEGS, JOINTS, HAA_LIMITS, FOOT_BALL_R, _CHAIN, leg_fk, leg_ik,
)


def _model_sdf_text():
    # workspace layout: <ws>/src/tugbot_maze/test/  ->  <ws>/src/tugbot_description/...
    ws_src = Path(__file__).resolve().parents[2]
    return (ws_src / 'tugbot_description' / 'models' / 'anymal_c' / 'model.sdf').read_text()


def test_chain_matches_sdf():
    """Chain anchor constants must equal the model.sdf zero-config link poses."""
    sdf = _model_sdf_text()
    for leg in LEGS:
        hip, thigh, shank, _axes, _foot = _CHAIN[leg]
        for link, expect in ((f'{leg}_HIP', hip), (f'{leg}_THIGH', thigh), (f'{leg}_SHANK', shank)):
            m = re.search(rf'<link name="{link}">\s*<pose[^>]*>([^<]+)</pose>', sdf)
            got = [float(v) for v in m.group(1).split()]
            assert got == pytest.approx(list(expect), abs=1e-5), link


def test_fk_x_stance_matches_measured():
    """X-stance FK verified against the numeric derivation of 2026-07-17."""
    xst = {'LF': (0, 0.4, -0.8), 'RF': (0, 0.4, -0.8),
           'LH': (0, -0.4, 0.8), 'RH': (0, -0.4, 0.8)}
    expect = {'LF': (0.4527, 0.3012, -0.5188), 'RF': (0.4527, -0.3012, -0.5188),
              'LH': (-0.4527, 0.3012, -0.5188), 'RH': (-0.4527, -0.3012, -0.5188)}
    for leg in LEGS:
        assert leg_fk(leg, xst[leg]) == pytest.approx(expect[leg], abs=2e-4)


def test_ik_fk_roundtrip():
    for leg in LEGS:
        for q in [(0.1, 0.5, -0.9), (-0.2, 0.7, -1.2), (0.0, 0.3, -0.6)]:
            # mirror the sagittal signs for hind legs (their HFE/KFE bend the other way)
            qq = q if leg in ('LF', 'RF') else (q[0], -q[1], -q[2])
            target = leg_fk(leg, qq)
            sol, err = leg_ik(leg, target, seed=qq)
            assert err < 1e-6
            assert leg_fk(leg, sol) == pytest.approx(target, abs=1e-5)


def test_ik_warm_start_converges_fast():
    near = (0.36, 0.3012, -0.48)   # a step-sized hop from the stand pose
    _sol, err = leg_ik('LF', near, seed=(0.0, 0.705, -0.9608))
    assert err < 1e-6


def test_ik_unreachable_reports_error():
    _sol, err = leg_ik('LF', (1.5, 0.3, -0.5), seed=(0.0, 0.705, -0.9608))
    assert err > 0.5   # far outside the ~0.6 m leg reach: residual reported, no exception


def test_mirror_symmetry():
    p = leg_fk('LF', (0.1, 0.6, -1.0))
    pr = leg_fk('RF', (-0.1, 0.6, -1.0))
    assert pr == pytest.approx((p[0], -p[1], p[2]), abs=1e-9)


def test_joints_order_and_limits():
    assert JOINTS[:3] == ['LF_HAA', 'LF_HFE', 'LF_KFE'] and len(JOINTS) == 12
    assert HAA_LIMITS['LF'] == (-0.72, 0.49) and HAA_LIMITS['RF'] == (-0.49, 0.72)
