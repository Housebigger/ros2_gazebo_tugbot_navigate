import math
from tugbot_maze.wall_localize import cell_center_offset, heading_snap, HALF_CORRIDOR_M

H = HALF_CORRIDOR_M  # 0.88


def _scan(dN, dE, dS, dW, n=360):
    """360-beam scan (angle_min=-pi, inc=2pi/n); one beam set per map cardinal, rest far."""
    amin, ainc = -math.pi, 2 * math.pi / n
    ranges = [12.0] * n
    for ang, d in [(0.0, dE), (math.pi / 2, dN), (math.pi, dW), (-math.pi / 2, dS)]:
        ranges[int(round((ang - amin) / ainc)) % n] = d
    return ranges, amin, ainc


def test_centered_gives_zero_offset():
    ox, oy = cell_center_offset(*_scan(H, H, H, H), yaw=0.0)
    assert math.isclose(ox, 0.0, abs_tol=1e-6)
    assert math.isclose(oy, 0.0, abs_tol=1e-6)


def test_offset_east_detected_from_both_walls():
    ox, oy = cell_center_offset(*_scan(H, H - 0.2, H, H + 0.2), yaw=0.0)
    assert math.isclose(ox, 0.2, abs_tol=1e-6)
    assert math.isclose(oy, 0.0, abs_tol=1e-6)


def test_offset_from_single_east_wall():
    ox, oy = cell_center_offset(*_scan(12.0, H - 0.2, 12.0, 12.0), yaw=0.0)
    assert math.isclose(ox, 0.2, abs_tol=1e-6)
    assert oy is None


def test_open_axis_is_none():
    ox, oy = cell_center_offset(*_scan(12.0, H, 12.0, H), yaw=0.0)
    assert math.isclose(ox, 0.0, abs_tol=1e-6)
    assert oy is None


def test_offset_respects_yaw():
    ox, oy = cell_center_offset(*_scan(H, H - 0.2, H, H + 0.2), yaw=math.pi / 2)
    assert math.isclose(ox, 0.2, abs_tol=1e-6)


def test_heading_snap():
    s, d = heading_snap(0.1)
    assert math.isclose(s, 0.0, abs_tol=1e-9) and math.isclose(d, -0.1, abs_tol=1e-9)
    s, d = heading_snap(1.5)
    assert math.isclose(s, math.pi / 2, abs_tol=1e-9) and math.isclose(d, math.pi / 2 - 1.5, abs_tol=1e-9)
    s, d = heading_snap(-3.10)
    assert math.isclose(abs(s), math.pi, abs_tol=1e-9)
