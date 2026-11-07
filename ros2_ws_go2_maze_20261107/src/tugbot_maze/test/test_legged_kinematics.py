"""Go2 analytic FK/IK checked independently against the official URDF chain."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from tugbot_maze.legged.kinematics import LEGS, JOINTS, HAA_LIMITS, FOOT_BALL_R, leg_fk, leg_ik
from tugbot_maze.legged.params import NEUTRAL_FOOT

URDF = ET.parse(Path(__file__).resolve().parents[2] /
                'go2_description/urdf/go2.urdf.xacro').getroot()
PREFIX = dict(LF='fl', RF='fr', LH='rl', RH='rr')


def _rotation(axis, angle):
    axis = np.array(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew @ skew)


def _official_chain(leg, angles):
    prefix = PREFIX[leg]
    result = np.eye(4)
    for suffix, angle in zip(('hip', 'thigh', 'knee', 'foot_fixed'), (*angles, 0.0)):
        joint = URDF.find(f"joint[@name='{prefix}_{suffix}']")
        origin = joint.find('origin')
        r, p, y = map(float, origin.get('rpy', '0 0 0').split())
        transform = np.eye(4)
        transform[:3, :3] = (_rotation((0, 0, 1), y) @ _rotation((0, 1, 0), p)
                             @ _rotation((1, 0, 0), r))
        transform[:3, 3] = list(map(float, origin.get('xyz').split()))
        result = result @ transform
        if joint.get('type') == 'revolute':
            rotation = np.eye(4)
            rotation[:3, :3] = _rotation(list(map(float, joint.find('axis').get('xyz').split())), angle)
            result = result @ rotation
    return result[:3, 3]


def test_fk_matches_official_urdf_at_multiple_joint_configurations():
    for leg in LEGS:
        for angles in ((0.1, 0.5, -0.9), (-0.2, 0.7, -1.2), (0.0, 0.9, -1.7)):
            assert leg_fk(leg, angles) == pytest.approx(_official_chain(leg, angles), abs=1e-9)


def test_ik_recovers_official_urdf_foot_positions():
    for leg in LEGS:
        for angles in ((0.1, 0.5, -0.9), (-0.2, 0.7, -1.2), (0.0, 0.9, -1.7)):
            target = _official_chain(leg, angles)
            solution, residual = leg_ik(leg, target)
            assert residual < 1e-8
            assert solution == pytest.approx(angles, abs=1e-8)


def test_ik_step_targets_near_neutral_are_reachable():
    for leg in LEGS:
        x, y, z = NEUTRAL_FOOT[leg]
        _, residual = leg_ik(leg, (x+0.04, y+0.01, z+0.02))
        assert residual < 1e-8


def test_ik_unreachable_reports_residual():
    for leg in LEGS:
        _, residual = leg_ik(leg, (1.5, 0.3, -0.5))
        assert residual > 0.5


def test_mirror_symmetry():
    position = leg_fk('LF', (0.1, 0.6, -1.0))
    mirrored = leg_fk('RF', (-0.1, 0.6, -1.0))
    assert mirrored == pytest.approx((position[0], -position[1], position[2]), abs=1e-9)


def test_joint_order_limits_and_foot_radius_match_official_urdf():
    assert JOINTS[:3] == ['LF_HAA', 'LF_HFE', 'LF_KFE'] and len(JOINTS) == 12
    for leg, prefix in PREFIX.items():
        limit = URDF.find(f"joint[@name='{prefix}_hip']/limit")
        assert HAA_LIMITS[leg] == (float(limit.get('lower')), float(limit.get('upper')))
        radius = URDF.find(f"link[@name='{prefix}_foot']/collision/geometry/sphere").get('radius')
        assert FOOT_BALL_R == float(radius)
