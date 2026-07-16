"""Tests for the pure open-loop trot gait module (visual animation only)."""
import math

from tugbot_maze.gait import (
    JOINTS, STAND_POSE, HAA_LIMITS, trot_pose, amplitude_scale, stride_frequency,
)


def test_all_12_joints_present():
    assert sorted(JOINTS) == sorted(
        f'{leg}_{j}' for leg in ('LF', 'RF', 'LH', 'RH') for j in ('HAA', 'HFE', 'KFE'))
    assert sorted(STAND_POSE) == sorted(JOINTS)
    pose = trot_pose(1.234, 0.3, 0.1)
    assert sorted(pose) == sorted(JOINTS)


def test_stand_at_zero_speed():
    pose = trot_pose(2.0, 0.0, 0.0)
    for j in JOINTS:
        assert abs(pose[j] - STAND_POSE[j]) < 1e-9
    assert stride_frequency(0.0, 0.0) == 0.0


def test_periodic_two_pi():
    a = trot_pose(0.7, 0.3, 0.0)
    b = trot_pose(0.7 + 2 * math.pi, 0.3, 0.0)
    for j in JOINTS:
        assert abs(a[j] - b[j]) < 1e-9


def _lift(pose, leg):
    """Knee-flex deviation from stand = swing-lift indicator (>=0 by design)."""
    kfe = f'{leg}_KFE'
    return abs(pose[kfe] - STAND_POSE[kfe])


def test_diagonal_pairs_in_phase_and_antiphase():
    # At phase pi/2 pair A (LF+RH) is mid-swing, pair B (RF+LH) grounded.
    pose = trot_pose(math.pi / 2, 0.4, 0.0)
    assert _lift(pose, 'LF') > 1e-3 and _lift(pose, 'RH') > 1e-3
    assert _lift(pose, 'RF') < 1e-9 and _lift(pose, 'LH') < 1e-9
    assert abs(_lift(pose, 'LF') - _lift(pose, 'RH')) < 1e-9
    # Half a cycle later the roles swap.
    pose2 = trot_pose(3 * math.pi / 2, 0.4, 0.0)
    assert _lift(pose2, 'RF') > 1e-3 and _lift(pose2, 'LH') > 1e-3
    assert _lift(pose2, 'LF') < 1e-9 and _lift(pose2, 'RH') < 1e-9


def test_amplitude_scales_with_speed_and_saturates():
    slow = trot_pose(math.pi / 2, 0.1, 0.0)
    fast = trot_pose(math.pi / 2, 0.4, 0.0)
    faster = trot_pose(math.pi / 2, 2.0, 0.0)
    assert _lift(slow, 'LF') < _lift(fast, 'LF')
    assert abs(_lift(fast, 'LF') - _lift(faster, 'LF')) < 1e-9  # saturated at V_REF
    assert amplitude_scale(0.0, 0.0) == 0.0
    assert amplitude_scale(10.0, 0.0) == 1.0


def test_rotation_only_also_animates():
    pose = trot_pose(math.pi / 2, 0.0, 0.8)
    assert _lift(pose, 'LF') > 1e-3
    assert stride_frequency(0.0, 0.8) > 0.0


def test_haa_within_hardware_limits():
    for phase in (0.0, 1.0, 2.0, 3.0, 4.5, 6.0):
        pose = trot_pose(phase, 5.0, 5.0)
        for j, (lo, hi) in HAA_LIMITS.items():
            assert lo - 1e-9 <= pose[j] <= hi + 1e-9
