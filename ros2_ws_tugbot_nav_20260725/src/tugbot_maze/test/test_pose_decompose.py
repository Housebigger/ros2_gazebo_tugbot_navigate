"""Offline pose-decomposition for the localization-root-cause measurement (Task A).

POSEDIAG log lines carry three raw world/frame poses per tick (gt, odom, solver). This tool
reconciles frames and decomposes the solver-vs-ground-truth error into along-track / lateral /
yaw so the along-track-lag verdict can be confirmed or refuted. Pure Python, no ROS."""
import math
from tugbot_maze.pose_decompose import (align_gt_to_map, decompose_error,
                                        offset_consistency, parse_posediag_line)


def test_parse_posediag_line():
    line = ("[1784500000.500000000] [flood_fill_solver]: POSEDIAG "
            "gt=(1.000, 2.000, 0.500) odom=(1.100, 2.050, 0.480) solver=(3.000, 2.100, 0.520)")
    r = parse_posediag_line(line)
    assert r is not None
    assert r["t"] == 1784500000.5
    assert r["gt"] == (1.0, 2.0, 0.5)
    assert r["odom"] == (1.1, 2.05, 0.48)
    assert r["solver"] == (3.0, 2.1, 0.52)


def test_decompose_pure_alongtrack_error():
    gt = (5.0, 1.0, 0.0)
    solver = (3.0, 1.0, 0.0)
    d = decompose_error(gt, solver)
    assert d["along"] == __import__("pytest").approx(2.0, abs=1e-9)
    assert d["lateral"] == __import__("pytest").approx(0.0, abs=1e-9)
    assert d["yaw"] == __import__("pytest").approx(0.0, abs=1e-9)


def test_decompose_alongtrack_respects_heading():
    gt = (1.0, 5.0, math.pi / 2)
    solver = (1.0, 3.0, math.pi / 2)
    d = decompose_error(gt, solver)
    assert d["along"] == __import__("pytest").approx(2.0, abs=1e-9)
    assert d["lateral"] == __import__("pytest").approx(0.0, abs=1e-9)


def test_decompose_lateral_error():
    gt = (3.0, 1.5, 0.0)
    solver = (3.0, 1.0, 0.0)
    d = decompose_error(gt, solver)
    assert d["along"] == __import__("pytest").approx(0.0, abs=1e-9)
    assert d["lateral"] == __import__("pytest").approx(0.5, abs=1e-9)


def test_malformed_and_nonposediag_return_none():
    assert parse_posediag_line("garbage") is None
    assert parse_posediag_line("[1784500000.5] POSEDIAG gt=(1.0, 2.0, 0.5)") is None


def test_decompose_yaw_error_wraps():
    gt = (0.0, 0.0, 3.0)
    solver = (0.0, 0.0, -3.0)
    d = decompose_error(gt, solver)
    assert d["yaw"] == __import__("pytest").approx(6.0 - 2 * math.pi, abs=1e-9)


def test_align_gt_to_map_removes_frame_offset_leaving_true_error():
    """gt lives in WORLD = MAP - (11, 9); odom is the true MAP pose (world-anchored); solver has a
    known +2 m along-track error. After align_gt_to_map, gt-solver along must be +2 -- NOT +2+11 --
    proving the fixed frame offset is reconciled away, not folded into the error."""
    pytest = __import__("pytest")
    # MAP-frame poses (heading 0): row a at x=5, row b at x=7. solver lags gt_map by 2 m along.
    row_a = {"t": 0.0, "gt": (5.0 - 11.0, 3.0 - 9.0, 0.0),
             "odom": (5.0, 3.0, 0.0), "solver": (3.0, 3.0, 0.0)}
    row_b = {"t": 1.0, "gt": (7.0 - 11.0, 3.0 - 9.0, 0.0),
             "odom": (7.0, 3.0, 0.0), "solver": (5.0, 3.0, 0.0)}
    rows = [row_a, row_b]

    # The offset the reconciler derives is odom - gt = (+11, +9, 0), constant across both rows.
    oc = offset_consistency(rows)
    assert oc["off_first"] == pytest.approx((11.0, 9.0, 0.0), abs=1e-9)
    assert oc["dpos"] == pytest.approx(0.0, abs=1e-9)
    assert oc["dyaw"] == pytest.approx(0.0, abs=1e-9)

    aligned = align_gt_to_map(rows)
    # gt is now in the map frame: (5,3) and (7,3), coinciding with odom (odom does not drift here).
    assert aligned[0]["gt"] == pytest.approx((5.0, 3.0, 0.0), abs=1e-9)
    assert aligned[1]["gt"] == pytest.approx((7.0, 3.0, 0.0), abs=1e-9)
    # odom / solver pass through untouched.
    assert aligned[0]["odom"] == pytest.approx((5.0, 3.0, 0.0), abs=1e-9)
    assert aligned[0]["solver"] == pytest.approx((3.0, 3.0, 0.0), abs=1e-9)

    # The real belief error is the known +2 m along-track lag, with the ~11 m frame offset gone.
    d = decompose_error(aligned[0]["gt"], aligned[0]["solver"])
    assert d["along"] == pytest.approx(2.0, abs=1e-9)
    assert d["lateral"] == pytest.approx(0.0, abs=1e-9)
    assert d["yaw"] == pytest.approx(0.0, abs=1e-9)
    # gt-odom collapses to ~0 (odom is world-anchored ground truth up to the frame offset).
    d_od = decompose_error(aligned[0]["gt"], aligned[0]["odom"])
    assert d_od["along"] == pytest.approx(0.0, abs=1e-9)
    assert d_od["lateral"] == pytest.approx(0.0, abs=1e-9)


def test_align_gt_to_map_flags_odom_drift():
    """If odom is NOT world-anchored (drifts), the first-vs-last offset disagrees -- surfaced as a
    finding via offset_consistency, not silently absorbed."""
    pytest = __import__("pytest")
    row_a = {"t": 0.0, "gt": (0.0, 0.0, 0.0), "odom": (11.0, 9.0, 0.0), "solver": (11.0, 9.0, 0.0)}
    # last row: odom slid +0.5 m in x relative to gt -> offset no longer constant.
    row_b = {"t": 9.0, "gt": (2.0, 0.0, 0.0), "odom": (13.5, 9.0, 0.0), "solver": (13.5, 9.0, 0.0)}
    oc = offset_consistency([row_a, row_b])
    assert oc["dpos"] == pytest.approx(0.5, abs=1e-9)


def test_align_gt_to_map_empty():
    assert align_gt_to_map([]) == []
