import math
import pytest
from tugbot_maze.maze_sim import px_to_map, load_segments, MazeSim


def test_px_to_map_entrance_and_exit_corners():
    # Left outer wall x=30 px maps to ~map x 1.0; near the bottom (entrance y).
    x_m, _ = px_to_map(30, 314)
    assert x_m == pytest.approx(1.02, abs=0.05)
    # Right outer wall x=329 px, exit gap near y=44 px -> map ~ (21.0, 18.08)
    ex, ey = px_to_map(329, 44)
    assert ex == pytest.approx(21.0, abs=0.1)
    assert ey == pytest.approx(18.08, abs=0.1)


def test_load_segments_returns_real_maze():
    segs = load_segments()
    assert len(segs) == 53           # 53 wall segments in the 20260528 yaml
    for x0, y0, x1, y1 in segs:
        assert all(math.isfinite(v) for v in (x0, y0, x1, y1))


def test_scan_hits_a_wall_in_front():
    # One vertical wall 2 m ahead (east) of a robot at origin facing +x.
    seg = [(2.0, -1.0, 2.0, 1.0)]
    sim = MazeSim(seg, start_xy=(0.0, 0.0), start_yaw=0.0,
                  wall_half_thickness_m=0.0)
    ranges, amin, ainc = sim.scan(n_beams=72)
    # beam nearest to forward (angle 0) should read ~2.0 m
    fwd_i = min(range(len(ranges)), key=lambda i: abs(amin + i * ainc))
    assert ranges[fwd_i] == pytest.approx(2.0, abs=0.1)


def test_scan_returns_max_range_when_no_wall():
    sim = MazeSim([], start_xy=(0.0, 0.0), start_yaw=0.0, max_range_m=12.0)
    ranges, _, _ = sim.scan(n_beams=36)
    assert all(r == 12.0 for r in ranges)


def test_scan_subtracts_wall_half_thickness():
    seg = [(2.0, -1.0, 2.0, 1.0)]
    sim = MazeSim(seg, start_xy=(0.0, 0.0), start_yaw=0.0,
                  wall_half_thickness_m=0.12)
    ranges, amin, ainc = sim.scan(n_beams=72)
    fwd_i = min(range(len(ranges)), key=lambda i: abs(amin + i * ainc))
    assert ranges[fwd_i] == pytest.approx(1.88, abs=0.1)   # 2.0 - 0.12
