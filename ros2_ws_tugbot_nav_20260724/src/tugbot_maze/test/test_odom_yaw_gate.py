"""Odom-prior yaw-consistency gate: reject an ICP accept whose yaw disagrees with the
drift-free odom yaw by more than the bound, keeping the odom-propagated prior. Calibrated
bound 0.5 rad (clean-run |ydis| max 0.297; alias pin ~1.5). localization-root-cause Task 5."""
import math
from tugbot_maze.pose_tracking import apply_odom_yaw_gate


def test_healthy_correction_passes():
    # est yaw agrees with odom yaw within the bound -> accept est unchanged.
    est = (2.0, 1.0, 0.30); odom_map = (2.1, 1.1, 0.20); prior = (2.05, 1.05, 0.19)
    out, gated = apply_odom_yaw_gate(est, odom_map, prior, 0.5)
    assert gated is False and out == est


def test_clean_ceiling_still_passes():
    # 0.29 rad disagreement (the observed clean-run max) must NOT be gated at bound 0.5.
    est = (2.0, 1.0, 0.29); odom_map = (2.0, 1.0, 0.00); prior = (1.9, 1.0, 0.00)
    out, gated = apply_odom_yaw_gate(est, odom_map, prior, 0.5)
    assert gated is False and out == est


def test_alias_accept_snaps_yaw_to_odom():
    # est yaw ~pi/2 off odom (the alias) -> gate, recover: prior position + odom yaw.
    est = (5.0, 3.0, 1.57); odom_map = (2.0, 1.0, 0.02); prior = (2.0, 1.0, 0.01)
    out, gated = apply_odom_yaw_gate(est, odom_map, prior, 0.5)
    assert gated is True and out == (2.0, 1.0, 0.02)


def test_negative_alias_is_rejected():
    est = (0.0, 0.0, -1.57); odom_map = (0.0, 0.0, 0.02); prior = (0.0, 0.0, 0.01)
    out, gated = apply_odom_yaw_gate(est, odom_map, prior, 0.5)
    assert gated is True and out == (0.0, 0.0, 0.02)


def test_gate_snaps_yaw_not_prior_yaw():
    # prior yaw deliberately != odom_map yaw -> recovered yaw comes from odom_map, not prior.
    est = (0.0, 0.0, 1.57); odom_map = (0.0, 0.0, 0.10); prior = (3.0, 4.0, 0.40)
    out, gated = apply_odom_yaw_gate(est, odom_map, prior, 0.5)
    assert gated is True and out == (3.0, 4.0, 0.10)


def test_wrap_around_pi():
    # est yaw 3.0, odom yaw -3.0 -> raw diff 6.0 but wrapped is -0.283 (< 0.5) -> pass.
    est = (0.0, 0.0, 3.0); odom_map = (0.0, 0.0, -3.0); prior = (0.0, 0.0, -3.0)
    out, gated = apply_odom_yaw_gate(est, odom_map, prior, 0.5)
    assert gated is False and out == est


def test_bound_is_exclusive_boundary():
    # exactly at the bound is NOT gated (strict > ); just past is gated.
    est = (0.0, 0.0, 0.50); odom_map = (0.0, 0.0, 0.0); prior = (0.0, 0.0, 0.0)
    assert apply_odom_yaw_gate(est, odom_map, prior, 0.5)[1] is False
    est2 = (0.0, 0.0, 0.5001)
    assert apply_odom_yaw_gate(est2, odom_map, prior, 0.5)[1] is True
