import math
from tugbot_maze.footprint import (beam_endpoint, front_gap, rear_gap, inside_footprint,
                                    FOOT_X_FRONT, FOOT_X_REAR, FOOT_HALF_W, SCAN_OFFSET_X)


def _scan(n, idx_r):  # idx_r: {beam_index: range}; others = math.inf (filtered by module)
    ranges = [math.inf] * n
    for i, r in idx_r.items():
        ranges[i] = r
    return (ranges, -math.pi, 2 * math.pi / n)


def test_beam_endpoint_applies_offset():
    ex, ey = beam_endpoint(1.0, 0.0)               # forward beam, range 1.0
    assert abs(ex - (SCAN_OFFSET_X + 1.0)) < 1e-9 and abs(ey) < 1e-9


def test_inside_footprint():
    assert inside_footprint(0.1, 0.0) is True        # mid-body
    assert inside_footprint(-0.1, 0.0) is True        # rear region (symmetric stance, no gripper any more)
    # exactly on the rear face (FOOT_X_REAR = -0.15): inside_footprint's rectangle test uses <= on
    # both bounds, so a boundary point is inclusive (True) -- assert that documented behavior
    # explicitly.
    assert inside_footprint(FOOT_X_REAR, 0.0) is True
    assert inside_footprint(0.5, 0.0) is False       # beyond front (front face 0.15)
    assert inside_footprint(0.0, 0.4) is False       # beyond side (half-width 0.13)
    assert inside_footprint(0.4, 0.0, margin=0.3) is True   # inflated: front+margin = 0.15+0.3 = 0.45 >= 0.4


def test_front_gap_detects_wall_ahead():
    n = 16                                           # beam 8 = bearing 0 (forward)
    # a return at range R forward -> endpoint ex = SCAN_OFFSET_X + R; gap = ex - FOOT_X_FRONT
    R = FOOT_X_FRONT + 0.30 - SCAN_OFFSET_X          # choose so gap == 0.30
    g = front_gap(*_scan(n, {8: R}))
    assert abs(g - 0.30) < 1e-6


def test_front_gap_ignores_pure_side():
    n = 16                                           # beam 12 = bearing +pi/2 (left side)
    assert front_gap(*_scan(n, {12: 0.3})) == math.inf   # side return not ahead of front face


def test_rear_gap_detects_wall_behind():
    n = 16                                           # beam 0 = bearing -pi (rear)
    R = abs(SCAN_OFFSET_X - FOOT_X_REAR) + 0.20      # gap == 0.20 behind rear face
    g = rear_gap(*_scan(n, {0: R}))
    assert abs(g - 0.20) < 1e-6


def test_footprint_covers_buggy_envelope():
    """The footprint rectangle must dominate the mr_buggy3 wheel-inclusive
    envelope (wheel centers x +0.112/-0.1135, radius 0.0365; outer wheel faces
    |y| = 0.10+0.015+0.015). The old legged gait-envelope tie-in (dog envelope
    0.439 x 0.353) retired with the platform swap -- legged/params.py still
    carries it for the archived dog."""
    assert FOOT_X_FRONT >= 0.112 + 0.0365
    assert -FOOT_X_REAR >= 0.1135 + 0.0365
    assert FOOT_HALF_W >= 0.10 + 0.015 + 0.015
