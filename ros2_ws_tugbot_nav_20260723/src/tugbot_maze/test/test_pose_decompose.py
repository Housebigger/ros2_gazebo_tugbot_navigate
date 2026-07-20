"""Offline pose-decomposition for the localization-root-cause measurement (Task A).

POSEDIAG log lines carry three raw world/frame poses per tick (gt, odom, solver). This tool
reconciles frames and decomposes the solver-vs-ground-truth error into along-track / lateral /
yaw so the along-track-lag verdict can be confirmed or refuted. Pure Python, no ROS."""
import math
from tugbot_maze.pose_decompose import decompose_error, parse_posediag_line


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
