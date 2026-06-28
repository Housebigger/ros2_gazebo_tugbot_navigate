import math
from tugbot_maze.maze_sim import MazeSim, load_segments
from tugbot_maze.wall_localize import cell_center_offset, heading_snap, HALF_CORRIDOR_M, perimeter_offset

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


def _scan_at(x, y, yaw=0.0):
    sim = MazeSim(load_segments(), (x, y), yaw)
    ranges, amin, ainc = sim.scan(n_beams=360, fov_rad=2 * math.pi)
    return ranges, amin, ainc, yaw


def test_perimeter_offset_recovers_true_x_when_off_center_in_boundary_cell():
    # Cell (10,5) center=(20,10); robot 0.6 m off-center toward interior (true x=19.4).
    r, amin, ainc, yaw = _scan_at(19.4, 10.0)
    out = perimeter_offset(r, amin, ainc, yaw, (10, 5))
    assert 0 in out
    true_x, implied = out[0]
    assert abs(true_x - 19.4) < 0.10            # absolute true x, NOT cell center (20)
    assert implied == 10


def test_perimeter_offset_flags_cell_desync():
    # Robot physically in (9,4) at (18.6,8) but caller believes (10,4); E wall ~2.29 m (< 2.5).
    # Uses cy=4 (y=8) — no interior E wall blocks the perimeter at this row.
    r, amin, ainc, yaw = _scan_at(18.6, 8.0)
    out = perimeter_offset(r, amin, ainc, yaw, (10, 4))
    assert 0 in out
    true_x, implied = out[0]
    assert abs(true_x - 18.6) < 0.12 and implied == 9


def test_perimeter_offset_abstains_when_far_from_perimeter():
    # ~2.89 m from the E wall (> PERIM_MAX_M 2.5) -> abstain.
    # Uses cy=4 (y=8) — no interior E wall blocks the perimeter at this row.
    r, amin, ainc, yaw = _scan_at(18.0, 8.0)
    assert perimeter_offset(r, amin, ainc, yaw, (10, 4)) == {}


def test_perimeter_offset_empty_for_interior_cell():
    r, amin, ainc, yaw = _scan_at(10.0, 10.0)
    assert perimeter_offset(r, amin, ainc, yaw, (5, 5)) == {}


def test_perimeter_offset_north_axis_in_top_row():
    # Cell (5,9) center=(10,18); N wall surface ~18.90; robot true y=18.5 -> perp_N ~0.40.
    r, amin, ainc, yaw = _scan_at(10.0, 18.5)
    out = perimeter_offset(r, amin, ainc, yaw, (5, 9))
    assert 1 in out
    true_y, implied = out[1]
    assert abs(true_y - 18.5) < 0.12 and implied == 9
