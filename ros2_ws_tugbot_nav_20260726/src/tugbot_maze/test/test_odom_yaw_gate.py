"""Odom-prior yaw-consistency gate: reject an ICP accept whose yaw disagrees with the
drift-free odom yaw by more than the bound, keeping the odom-propagated prior. Calibrated
bound 0.5 rad (clean-run |ydis| max 0.297; alias pin ~1.5). localization-root-cause Task 5."""
import math
from tugbot_maze.pose_tracking import apply_odom_yaw_gate, apply_odom_pos_gate


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


def test_pos_gate_passes_clean_correction():
    # Clean-mode agreement (measured p99 0.182): a small ICP correction passes through.
    est, gated = apply_odom_pos_gate((10.1, 12.05, 0.4), (10.0, 12.0, 0.39), 1.0)
    assert est == (10.1, 12.05, 0.4) and gated is False


def test_pos_gate_recovers_one_cell_alias():
    # The 20260723 aliased mode: believed pose one cell (~2.0m) from the drift-free odom
    # position with healthy yaw -> position snaps to odom, the accepted yaw is KEPT.
    est, gated = apply_odom_pos_gate((16.0, 14.0, 0.02), (14.0, 14.0, 0.01), 1.0)
    assert est == (14.0, 14.0, 0.02) and gated is True


def test_pos_gate_boundary_exact_bound_not_gated():
    est, gated = apply_odom_pos_gate((15.0, 14.0, 0.0), (14.0, 14.0, 0.0), 1.0)
    assert gated is False                        # strict >: exactly 1.0m passes


def test_pos_gate_diagonal_distance():
    # Euclidean, not per-axis: (0.8, 0.8) -> 1.13m > 1.0 -> gated.
    est, gated = apply_odom_pos_gate((14.8, 14.8, 1.5), (14.0, 14.0, 1.5), 1.0)
    assert est == (14.0, 14.0, 1.5) and gated is True
