import math
from tugbot_maze.wall_localize import cell_center_offset, heading_snap, HALF_CORRIDOR_M

H = HALF_CORRIDOR_M  # 0.88


def _scan(dN, dE, dS, dW, n=360, yaw=0.0, half_window_deg=30):
    """A ROBOT-frame scan (angle_min=-pi, inc=2pi/n) for a robot facing `yaw`. Each MAP
    cardinal's wall is modeled as a flat perpendicular wall at distance d: every beam
    within half_window_deg of that cardinal reads the slant range d/cos(off), so the
    perpendicular-projection estimator recovers exactly d. d>=12 (max_range) means no
    wall (open) -- those beams stay at max_range. The window (>=22 deg) fully fills the
    sensor's +/-22 deg window, matching a real cell-edge wall (which subtends ~+/-45 deg)."""
    amin, ainc = -math.pi, 2 * math.pi / n
    ranges = [12.0] * n
    hw = math.radians(half_window_deg)
    for map_ang, d in [(0.0, dE), (math.pi / 2, dN), (math.pi, dW), (-math.pi / 2, dS)]:
        if d >= 12.0:
            continue
        ta = math.atan2(math.sin(map_ang - yaw), math.cos(map_ang - yaw))   # scan-frame cardinal
        for i in range(n):
            off = math.atan2(math.sin((amin + i * ainc) - ta), math.cos((amin + i * ainc) - ta))
            if abs(off) <= hw:
                ranges[i] = min(ranges[i], d / math.cos(off))               # slant range to flat wall
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
    ox, oy = cell_center_offset(*_scan(H, H - 0.2, H, H + 0.2, yaw=math.pi / 2), yaw=math.pi / 2)
    assert math.isclose(ox, 0.2, abs_tol=1e-6)


def test_heading_snap():
    s, d = heading_snap(0.1)
    assert math.isclose(s, 0.0, abs_tol=1e-9) and math.isclose(d, -0.1, abs_tol=1e-9)
    s, d = heading_snap(1.5)
    assert math.isclose(s, math.pi / 2, abs_tol=1e-9) and math.isclose(d, math.pi / 2 - 1.5, abs_tol=1e-9)
    s, d = heading_snap(-3.10)
    assert math.isclose(abs(s), math.pi, abs_tol=1e-9)
