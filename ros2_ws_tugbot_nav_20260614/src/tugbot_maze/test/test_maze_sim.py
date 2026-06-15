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
        # map-frame plausibility: the maze spans ~0..22 m (catches a flipped transform)
        assert -1.0 <= x0 <= 23.0 and -2.0 <= y0 <= 23.0
        assert -1.0 <= x1 <= 23.0 and -2.0 <= y1 <= 23.0


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


def test_step_moves_forward_in_open_space():
    sim = MazeSim([], start_xy=(0.0, 0.0), start_yaw=0.0)
    sim.step(1.0, 0.0, 0.5)
    x, y, _ = sim.pose
    assert x == pytest.approx(0.5, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)


def test_step_rotation_normalizes_yaw():
    sim = MazeSim([], start_xy=(0.0, 0.0), start_yaw=3.0)
    sim.step(0.0, 1.0, 1.0)          # yaw -> 4.0 rad, normalized into (-pi, pi]
    _, _, yaw = sim.pose
    assert yaw == pytest.approx(math.atan2(math.sin(4.0), math.cos(4.0)), abs=1e-9)


def test_collides_true_near_wall_segment():
    seg = [(1.0, -1.0, 1.0, 1.0)]    # vertical wall at x=1
    sim = MazeSim(seg, start_xy=(0.0, 0.0), start_yaw=0.0,
                  robot_radius_m=0.35, wall_half_thickness_m=0.12)
    assert sim.collides(0.6, 0.0) is True       # 0.4 m to wall < 0.47 margin
    assert sim.collides(0.4, 0.0) is False       # 0.6 m to wall > 0.47 margin


def test_step_blocked_by_wall_keeps_position_but_allows_rotation():
    seg = [(0.5, -1.0, 0.5, 1.0)]    # wall just ahead at x=0.5
    sim = MazeSim(seg, start_xy=(0.0, 0.0), start_yaw=0.0,
                  robot_radius_m=0.35, wall_half_thickness_m=0.12)
    sim.step(1.0, 1.0, 0.5)          # try to drive into the wall while turning
    x, y, yaw = sim.pose
    assert x == pytest.approx(0.0, abs=1e-9)     # translation rejected
    assert yaw != 0.0                            # rotation still applied
