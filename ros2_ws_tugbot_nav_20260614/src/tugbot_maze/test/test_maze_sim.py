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


def test_ground_truth_edge_open_matches_perfect_maze_tree():
    # The 20260528 maze is a perfect maze: 100 cells, 99 undirected open edges.
    from tugbot_maze.maze_sim import MazeSim, load_segments, ground_truth_edge_open
    from tugbot_maze.flood_fill_brain import in_grid, DIRS, cell_center
    sim = MazeSim(load_segments(), (2.0, 0.0), 0.0)
    undirected = set()
    for cx in range(1, 11):
        for cy in range(0, 10):
            for d, (dx, dy) in DIRS.items():
                nb = (cx + dx, cy + dy)
                if in_grid(nb) and ground_truth_edge_open(sim, (cx, cy), nb):
                    undirected.add(frozenset({(cx, cy), nb}))
    assert len(undirected) == 99


def test_inertia_step_rate_limits_and_clamps():
    from tugbot_maze.maze_sim import MazeSim, load_segments
    sim = MazeSim(load_segments(), (10.0, 10.0), 0.0, inertia=True)
    # command beyond the envelope; one 0.1 s step can change v by <= 0.5*0.1 = 0.05
    sim.step(1.0, 1.0, 0.1)
    assert sim.v_cur == pytest.approx(0.05, abs=1e-9)     # accel-limited toward clamp(1.0)->0.5
    assert sim.w_cur == pytest.approx(0.08, abs=1e-9)     # ang accel 0.8 * 0.1
    # eventually saturates at the clamp, not the commanded 1.0
    for _ in range(200):
        sim.step(1.0, 1.0, 0.1)
    assert sim.v_cur == pytest.approx(0.5, abs=1e-6)
    assert sim.w_cur == pytest.approx(0.5, abs=1e-6)


def test_odom_drift_accumulates_with_distance():
    from tugbot_maze.maze_sim import MazeSim
    sim = MazeSim([], (0.0, 0.0), 0.0, odom_drift_per_m=0.1)   # 0.1 m drift per m driven
    for _ in range(20):
        sim.step(0.5, 0.0, 0.1)                                 # drive forward ~1 m
    tx, ty, _ = sim.pose
    ox, oy, _ = sim.reported_pose
    assert abs(tx - 1.0) < 0.05                 # true pose moved ~1 m along +x
    assert abs(ox - tx) < 1e-6                  # no drift ALONG travel (x)
    assert abs(oy - ty) > 0.05                  # drift is LATERAL (perpendicular, in y)
    assert abs((oy - ty) - 0.1 * tx) < 0.02     # ~0.1 m lateral drift per m driven


def test_reported_pose_equals_true_with_zero_drift():
    from tugbot_maze.maze_sim import MazeSim
    sim = MazeSim([], (0.0, 0.0), 0.0)          # default: no drift
    for _ in range(10):
        sim.step(0.5, 0.0, 0.1)
    assert sim.reported_pose == sim.pose
