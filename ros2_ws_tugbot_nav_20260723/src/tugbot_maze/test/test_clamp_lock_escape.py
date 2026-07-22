"""Clamp-lock escape + yaw-only rms ceiling (2026-07-19 spec amendment,
section "## 修案").

Forensic background (R3): the inner ScanMatchLocalizer.correct() clamp
("reject-if-exceeds") discarded a CONVERGED fit (n=421, eigmin=22) wholesale
because the correction magnitude exceeded trans_clamp_m/yaw_clamp_rad -- 575
consecutive rejections, nothing downstream could spend the recoverable
position error ("clamp-lock"). This module pins:

  1. scan_match_localizer.ScanMatchLocalizer.correct()'s clamp-reject path
     now carries a pure diagnostic 'conv_pose' (the discarded converged pose).
  2. OnlineScanMatchLocalizer tracks a _clamp_streak of consecutive HEALTHY
     clamp-type inner-ICP rejections; at CLAMP_LOCK_STREAK=3 it escapes with
     a bounded step toward the discarded converged pose, in PRIORITY over the
     yaw-only fallback.
  3. yaw_only_correct now additionally requires best_rms <= YAW_RMS_CEILING.
"""
import math

import pytest

from tugbot_maze.maze_sim import MazeSim
from tugbot_maze.scan_match_localizer import ScanMatchLocalizer
from tugbot_maze.online_scan_match_localizer import (
    CLAMP_ESCAPE_TRANS, CLAMP_LOCK_MIN_INLIERS, CLAMP_LOCK_STREAK,
    YAW_RMS_CEILING, YAW_STEP_CLAMP,
    OnlineScanMatchLocalizer, yaw_only_correct,
)

# Same fixture as test_yaw_only_fallback.py: 20x20 perimeter, interior empty
# unless a gate-pass wall is explicitly added.
PERIM = [(0.0, 0.0, 20.0, 0.0), (20.0, 0.0, 20.0, 20.0),
         (20.0, 20.0, 0.0, 20.0), (0.0, 20.0, 0.0, 0.0)]

# An interior wall ~0.7-0.8 m from the (3,3) prior below: close enough to pass
# gate 2 (endpoints within local_radius=1.0m) yet INERT for the scan (no beam
# endpoint lands near it, every point associates to the much nearer
# perimeter) -- routes correct() past gates 1/2 into the inner-ICP path
# without perturbing the ICP fit itself. Copied from test_yaw_only_fallback.py.
_GATE_PASS_WALL = [(2.6, 2.3, 3.4, 2.3)]


def _scan_at(pose):
    sim = MazeSim(PERIM, (pose[0], pose[1]), pose[2])
    return sim.scan(n_beams=360, fov_rad=2 * math.pi)


def _localizer():
    return OnlineScanMatchLocalizer(PERIM)


# --- 1. conv_pose diagnostic on a clamp-reject (ScanMatchLocalizer, direct) -


def test_conv_pose_present_on_clamp_reject():
    # Bias 0.19 rad sits inside the yaw-only fallback's 0.2 window but beyond
    # the full ICP's yaw_clamp_rad (~0.1745): the inner ICP converges and then
    # clamp-rejects (proven passing behavior, see
    # test_correct_inner_icp_clamp_reject_recovered_by_yaw_fallback in
    # test_yaw_only_fallback.py). Drive the SAME inner ICP directly.
    pose_true = (3.0, 3.0, 0.9)
    ranges, amin, ainc = _scan_at(pose_true)
    prior = (pose_true[0], pose_true[1], pose_true[2] + 0.19)
    online = _localizer()
    masked = online._mask_far_beams(prior, ranges, amin, ainc)
    icp = ScanMatchLocalizer(PERIM + _GATE_PASS_WALL)   # same segs the wrapper builds
    est, info = icp.correct(prior, masked, amin, ainc)
    assert info['rejected'] is True
    assert est == prior                                  # unchanged (pure dead reckoning)
    assert 'conv_pose' in info
    cx, cy, cyaw = info['conv_pose']
    dyaw = math.atan2(math.sin(cyaw - prior[2]), math.cos(cyaw - prior[2]))
    # it was clamp-rejected BECAUSE the converged correction exceeds the clamp
    assert abs(dyaw) > icp.yaw_clamp_rad


# --- 2. streak semantics: escape only on the 3rd consecutive clamp-type ----


def _fake_clamp_info(conv_pose=(3.3, 3.3, 1.2), n_inliers=CLAMP_LOCK_MIN_INLIERS + 50):
    return {'rejected': True, 'under_inliers': False, 'n_inliers': n_inliers,
            'residual_rms': 0.05, 'fell_back': set(), 'conv_pose': conv_pose}


def _fake_under_inliers_info():
    return {'rejected': True, 'under_inliers': True, 'n_inliers': 0,
            'residual_rms': float('nan'), 'fell_back': set()}


def _declining_yaw_only(*a, **k):
    return (a[0], {'rejected': True, 'reason': 'yaw_only_declined',
                   'residual_rms': 1.0, 'n_inliers': 0})


def _prep_gate_pass_localizer(monkeypatch):
    """Build a localizer whose interior is already committed (no mid-loop
    rebuild) and whose module-level yaw_only_correct always declines, so the
    streak sequence below is governed purely by the injected inner-ICP kind."""
    loc = _localizer()
    loc._rebuild(_GATE_PASS_WALL)
    loc._sig = frozenset(tuple(s) for s in _GATE_PASS_WALL)
    monkeypatch.setattr(
        'tugbot_maze.online_scan_match_localizer.yaw_only_correct',
        _declining_yaw_only)
    return loc


def test_streak_escape_only_on_third_consecutive_clamp_type(monkeypatch):
    loc = _prep_gate_pass_localizer(monkeypatch)
    prior = (3.0, 3.0, 0.9)
    ranges, amin, ainc = _scan_at(prior)

    sequence = ['clamp', 'clamp', 'under_inliers', 'clamp', 'clamp', 'clamp']
    results = []
    for kind in sequence:
        fake = _fake_clamp_info() if kind == 'clamp' else _fake_under_inliers_info()
        monkeypatch.setattr(loc._icp, 'correct',
                            lambda *a, f=fake, **k: (prior, dict(f)))
        est, info = loc.correct(prior, ranges, amin, ainc, _GATE_PASS_WALL)
        results.append(info)

    for info in results[:-1]:
        assert info.get('reason') != 'clamp_lock_escape'
        assert info.get('fell_back') != {'clamp_escape'}
    last = results[-1]
    assert last['rejected'] is False
    assert last['reason'] == 'clamp_lock_escape'
    assert last['fell_back'] == {'clamp_escape'}


def test_streak_resets_after_escape_needs_three_more():
    # After an escape the streak clears -- a 4th immediate clamp-type
    # rejection must NOT escape again (needs 3 fresh consecutive ones).
    import tugbot_maze.online_scan_match_localizer as osml
    loc = _localizer()
    loc._rebuild(_GATE_PASS_WALL)
    loc._sig = frozenset(tuple(s) for s in _GATE_PASS_WALL)
    prior = (3.0, 3.0, 0.9)
    ranges, amin, ainc = _scan_at(prior)
    orig_yaw_only = osml.yaw_only_correct
    osml.yaw_only_correct = _declining_yaw_only
    try:
        loc._icp.correct = lambda *a, **k: (prior, _fake_clamp_info())
        for _ in range(3):
            est, info = loc.correct(prior, ranges, amin, ainc, _GATE_PASS_WALL)
        assert info['reason'] == 'clamp_lock_escape'
        assert loc._clamp_streak == 0
        est, info = loc.correct(prior, ranges, amin, ainc, _GATE_PASS_WALL)
        assert info.get('reason') != 'clamp_lock_escape'
        assert loc._clamp_streak == 1
    finally:
        osml.yaw_only_correct = orig_yaw_only


# --- 3. escape step bounds: scaled translation, clamped yaw, full-delta ----


def test_escape_translation_scaled_and_direction_preserved():
    loc = _localizer()
    prior = (0.0, 0.0, 0.0)
    info = {'conv_pose': (10.0, 0.0, 0.0), 'residual_rms': 0.05, 'n_inliers': 150}
    pose, out = loc._clamp_lock_escape(prior, info)
    dx, dy = pose[0] - prior[0], pose[1] - prior[1]
    assert math.hypot(dx, dy) == pytest.approx(CLAMP_ESCAPE_TRANS, abs=1e-9)
    assert dx > 0 and dy == pytest.approx(0.0, abs=1e-9)   # direction preserved
    assert out['conv_dist'] == pytest.approx(10.0)
    # pure-translation conv delta: yaw must be genuinely untouched
    assert out['yaw_step'] == 0.0
    assert pose[2] == prior[2]


def test_escape_translation_diagonal_direction_preserved():
    loc = _localizer()
    prior = (0.0, 0.0, 0.0)
    info = {'conv_pose': (3.0, 4.0, 0.0), 'residual_rms': 0.05, 'n_inliers': 150}
    pose, out = loc._clamp_lock_escape(prior, info)
    dx, dy = pose[0] - prior[0], pose[1] - prior[1]
    assert math.hypot(dx, dy) == pytest.approx(CLAMP_ESCAPE_TRANS, abs=1e-9)
    # original delta is (3,4) -> unit direction (0.6, 0.8); must be preserved
    assert dx / CLAMP_ESCAPE_TRANS == pytest.approx(0.6, abs=1e-6)
    assert dy / CLAMP_ESCAPE_TRANS == pytest.approx(0.8, abs=1e-6)


def test_escape_yaw_step_clamped():
    loc = _localizer()
    prior = (0.0, 0.0, 0.0)
    info = {'conv_pose': (0.0, 0.0, 1.0), 'residual_rms': 0.05, 'n_inliers': 150}
    pose, out = loc._clamp_lock_escape(prior, info)
    assert abs(pose[2] - prior[2]) == pytest.approx(YAW_STEP_CLAMP, abs=1e-9)
    assert out['yaw_step'] == pytest.approx(YAW_STEP_CLAMP)


def test_escape_full_delta_applies_when_within_bounds():
    loc = _localizer()
    prior = (1.0, 1.0, 0.2)
    conv = (1.05, 1.02, 0.24)     # well within CLAMP_ESCAPE_TRANS and YAW_STEP_CLAMP
    info = {'conv_pose': conv, 'residual_rms': 0.05, 'n_inliers': 150}
    pose, out = loc._clamp_lock_escape(prior, info)
    assert pose[0] == pytest.approx(conv[0], abs=1e-9)
    assert pose[1] == pytest.approx(conv[1], abs=1e-9)
    assert pose[2] == pytest.approx(conv[2], abs=1e-9)


# --- gates 1/2 never touch the streak (no inner ICP ran) -------------------


@pytest.mark.parametrize('gate,interior', [
    ('sparse_interior_gate', []),                            # gate 1: empty interior
    ('no_local_interior_walls', [(15.0, 15.0, 17.0, 15.0)]),  # gate 2: all walls distant
])
def test_gates_never_touch_clamp_streak(gate, interior):
    # On a gate-1/2 tick no inner ICP ran, so clamp-type vs not is
    # undetermined -- the streak must be neither incremented nor reset,
    # regardless of the gate's own yaw-only accept/decline outcome.
    prior = (3.0, 3.0, 0.9)
    ranges, amin, ainc = _scan_at(prior)
    loc = _localizer()
    loc._clamp_streak = 2                       # mid-streak when the gate fires
    est, info = loc.correct(prior, ranges, amin, ainc, interior)
    assert (info.get('gate_reason') == gate     # yaw-only accepted, or
            or gate in info.get('reason', ''))  # declined -- gate fired either way
    assert loc._clamp_streak == 2               # untouched


# --- 4. priority: escape beats yaw-only even when yaw-only would accept ----


def test_escape_has_priority_and_yaw_only_never_consulted(monkeypatch):
    loc = _localizer()
    loc._rebuild(_GATE_PASS_WALL)
    loc._sig = frozenset(tuple(s) for s in _GATE_PASS_WALL)
    prior = (3.0, 3.0, 0.9)
    ranges, amin, ainc = _scan_at(prior)

    def _explode(*a, **k):
        raise AssertionError('yaw_only_correct must not be called on the '
                             'escaping tick (escape has priority)')

    monkeypatch.setattr('tugbot_maze.online_scan_match_localizer.yaw_only_correct',
                        _explode)
    monkeypatch.setattr(loc._icp, 'correct',
                        lambda *a, **k: (prior, _fake_clamp_info()))
    loc._clamp_streak = CLAMP_LOCK_STREAK - 1   # one more clamp-type reject triggers escape
    est, info = loc.correct(prior, ranges, amin, ainc, _GATE_PASS_WALL)
    assert info['rejected'] is False
    assert info['reason'] == 'clamp_lock_escape'


# --- 5. rms ceiling ----------------------------------------------------


def test_yaw_only_declines_above_rms_ceiling_kwarg_boundary():
    # A geometric scenario proven to ACCEPT at the default ceiling
    # (test_recovers_in_window_bias_and_freezes_xy[bias=0.05] in
    # test_yaw_only_fallback.py): forcing rms_ceiling far below its achieved
    # rms must flip the same fit to a decline. Honest attempts at a
    # rms>0.15-through-noisy-geometry case (extra spurious short returns,
    # larger bias) kept landing well under the ceiling (healthy fits here
    # cluster ~0.02-0.06 given wall_half_thickness=0.12) or losing enough
    # inliers/improvement to decline for OTHER reasons first, so this is a
    # deliberate kwarg-boundary probe of the new ceiling parameter itself
    # (noted per the task's documented fallback).
    pose_true = (3.0, 3.0, 0.9)
    ranges, amin, ainc = _scan_at(pose_true)
    prior = (pose_true[0], pose_true[1], pose_true[2] + 0.05)
    loc = _localizer()
    icp = loc._icp
    est_ok, info_ok = yaw_only_correct(prior, ranges, amin, ainc, icp)
    assert info_ok['rejected'] is False
    assert info_ok['residual_rms'] <= YAW_RMS_CEILING
    est_bad, info_bad = yaw_only_correct(prior, ranges, amin, ainc, icp,
                                         rms_ceiling=1e-6)
    assert info_bad['rejected'] is True
    assert 'yaw_only_declined' in info_bad['reason']
    assert est_bad == prior


def test_yaw_rms_ceiling_constant_value():
    assert YAW_RMS_CEILING == pytest.approx(0.15)


def test_clamp_lock_constants_exported_with_spec_values():
    assert CLAMP_LOCK_STREAK == 3
    assert CLAMP_LOCK_MIN_INLIERS == 100
    assert CLAMP_ESCAPE_TRANS == pytest.approx(0.15)


# --- 6. existing bias-0.19 scenario unaffected at streak 1, 2 --------------


def test_bias019_still_accepts_via_yaw_only_at_low_streak():
    # Real (unmocked) ICP + yaw-only: the SAME scenario as
    # test_correct_inner_icp_clamp_reject_recovered_by_yaw_fallback, called
    # twice on a fresh localizer. Each call clamp-rejects (streak +1) but is
    # immediately rescued by the yaw-only fallback, which is an ACCEPTED
    # correction and so resets the streak back to 0 -- confirms no existing
    # scenario silently starts escaping instead of yaw-only-ing.
    pose_true = (3.0, 3.0, 0.9)
    ranges, amin, ainc = _scan_at(pose_true)
    prior = (pose_true[0], pose_true[1], pose_true[2] + 0.19)
    loc = _localizer()
    for _ in range(2):
        est, info = loc.correct(prior, ranges, amin, ainc, _GATE_PASS_WALL)
        assert info['rejected'] is False
        assert info.get('fell_back') == {'yaw_only'}
        assert info.get('gate_reason') == 'icp_rejected'
        assert loc._clamp_streak == 0        # accepted correction resets it
