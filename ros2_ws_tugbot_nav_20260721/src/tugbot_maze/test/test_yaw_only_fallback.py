"""1-DOF yaw-only fallback: when full ICP is gated (junction cells have
almost no walls -> deterministic dead reckoning), wall DIRECTION is still
observable at range - even from the perimeter alone. The fallback corrects
yaw only; x,y must be bit-identical to the prior on EVERY path (a 1-DOF
fit structurally cannot reproduce the 2026-06 two-axis position corruption)."""
import math

import pytest

from tugbot_maze.maze_sim import MazeSim
from tugbot_maze.online_scan_match_localizer import (
    YAW_MIN_IMPROVE, YAW_MIN_INLIERS, YAW_STEP_CLAMP, YAW_WINDOW_RAD,
    OnlineScanMatchLocalizer, yaw_only_correct,
)

# 20x20 m perimeter rectangle (matches the maze scale); interior EMPTY so
# OnlineScanMatchLocalizer's sparse-interior gate always fires -> fallback path.
PERIM = [(0.0, 0.0, 20.0, 0.0), (20.0, 0.0, 20.0, 20.0),
         (20.0, 20.0, 0.0, 20.0), (0.0, 20.0, 0.0, 0.0)]
POSE_TRUE = (7.0, 9.0, 0.9)     # off-center, arbitrary heading


def _scan_at(pose):
    sim = MazeSim(PERIM, (pose[0], pose[1]), pose[2])
    return sim.scan(n_beams=360, fov_rad=2 * math.pi)


def _localizer():
    return OnlineScanMatchLocalizer(PERIM)


def _icp_of(loc):
    return loc._icp     # the underlying ScanMatchLocalizer holding the reference


@pytest.mark.parametrize('bias', [0.05, 0.15])
def test_recovers_in_window_bias_and_freezes_xy(bias):
    ranges, amin, ainc = _scan_at(POSE_TRUE)
    prior = (POSE_TRUE[0], POSE_TRUE[1], POSE_TRUE[2] + bias)
    loc = _localizer()
    est, info = yaw_only_correct(prior, ranges, amin, ainc, _icp_of(loc))
    assert info['rejected'] is False and 'yaw_only' in info['fell_back']
    assert est[0] == prior[0] and est[1] == prior[1]          # x,y bit-identical
    resid = math.atan2(math.sin(est[2] - POSE_TRUE[2]), math.cos(est[2] - POSE_TRUE[2]))
    assert abs(resid) < 0.02, f'bias {bias} -> residual {resid}'


def test_overwindow_bias_steps_clamped_toward_truth():
    ranges, amin, ainc = _scan_at(POSE_TRUE)
    prior = (POSE_TRUE[0], POSE_TRUE[1], POSE_TRUE[2] + 0.3)   # beyond the 0.2 window
    loc = _localizer()
    est, info = yaw_only_correct(prior, ranges, amin, ainc, _icp_of(loc))
    assert info['rejected'] is False
    step = est[2] - prior[2]
    assert step == pytest.approx(-YAW_STEP_CLAMP, abs=1e-6)    # full clamp, correct sign
    assert est[0] == prior[0] and est[1] == prior[1]


def test_declines_when_scan_unrelated_to_reference():
    # Beams all shorter than any wall distance (robot "sees" phantom close
    # clutter the reference cannot explain) -> too few inliers -> decline.
    prior = (10.0, 10.0, 0.0)
    ranges = [0.6] * 360
    amin, ainc = -math.pi, 2 * math.pi / 360
    loc = _localizer()
    est, info = yaw_only_correct(prior, ranges, amin, ainc, _icp_of(loc))
    assert info['rejected'] is True and 'yaw_only_declined' in info['reason']
    assert est == prior


def test_declines_when_no_improvement_available():
    # Prior yaw already exact: best-case rms improvement ~0 < 20% -> decline
    # (guards against noise-driven idle corrections).
    ranges, amin, ainc = _scan_at(POSE_TRUE)
    loc = _localizer()
    est, info = yaw_only_correct(POSE_TRUE, ranges, amin, ainc, _icp_of(loc))
    assert info['rejected'] is True and 'yaw_only_declined' in info['reason']
    assert est == POSE_TRUE


def test_repeated_ticks_converge_without_oscillation():
    ranges, amin, ainc = _scan_at(POSE_TRUE)
    pose = (POSE_TRUE[0], POSE_TRUE[1], POSE_TRUE[2] + 0.28)
    loc = _localizer()
    resids = []
    for _ in range(6):
        est, info = yaw_only_correct(pose, ranges, amin, ainc, _icp_of(loc))
        pose = est
        resids.append(abs(math.atan2(math.sin(pose[2] - POSE_TRUE[2]),
                                     math.cos(pose[2] - POSE_TRUE[2]))))
    assert resids[-1] < 0.02
    assert all(resids[i + 1] <= resids[i] + 1e-9 for i in range(len(resids) - 1)), resids


def test_constants_exported_with_spec_values():
    assert YAW_WINDOW_RAD == pytest.approx(0.2)
    assert YAW_MIN_INLIERS == 60
    assert YAW_MIN_IMPROVE == pytest.approx(0.20)
    assert YAW_STEP_CLAMP == pytest.approx(0.1)
