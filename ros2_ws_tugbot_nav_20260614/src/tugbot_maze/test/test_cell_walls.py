import math
from tugbot_maze.cell_walls import sense_cell_walls


def _scan(near_scan_angles, n=360, far=5.0, near=0.9, window_deg=12):
    """LaserScan ranges over [-pi, pi); set beams within window_deg of each given
    SCAN-frame angle to `near`, the rest to `far`."""
    amin = -math.pi
    ainc = 2 * math.pi / n
    ranges = [far] * n
    for i in range(n):
        ang = math.atan2(math.sin(amin + i * ainc), math.cos(amin + i * ainc))
        for a in near_scan_angles:
            if abs(math.atan2(math.sin(ang - a), math.cos(ang - a))) <= math.radians(window_deg):
                ranges[i] = near
    return ranges, amin, ainc


def test_all_far_is_all_open():
    ranges, amin, ainc = _scan([])
    assert sense_cell_walls(ranges, amin, ainc, 0.0) == {'N': False, 'S': False, 'E': False, 'W': False}


def test_near_wall_ahead_is_east_at_yaw0():
    ranges, amin, ainc = _scan([0.0])          # wall straight ahead (scan angle 0)
    assert sense_cell_walls(ranges, amin, ainc, 0.0) == {'E': True, 'N': False, 'S': False, 'W': False}


def test_direction_is_map_frame_not_robot_frame():
    # Wall to the robot's RIGHT (scan -pi/2). At yaw=+pi/2 the robot faces map-N, so its
    # right is map-E -> only E should read wall. Confirms the yaw->map-frame transform.
    ranges, amin, ainc = _scan([-math.pi / 2])
    w = sense_cell_walls(ranges, amin, ainc, math.pi / 2)
    assert w['E'] is True and w['N'] is False and w['S'] is False and w['W'] is False


def test_matches_ground_truth_on_real_maze_at_all_cells():
    # The decisive check: LIDAR sensing (via the maze raycaster) agrees with ground-truth
    # traversability at every interior cell edge -> sensing is faithful before Gazebo.
    from tugbot_maze.maze_sim import MazeSim, load_segments, ground_truth_edge_open
    from tugbot_maze.flood_fill_brain import in_grid, DIRS
    segs = load_segments()
    mismatches = 0
    for cx in range(1, 11):
        for cy in range(0, 10):
            sim = MazeSim(segs, (2 * cx, 2 * cy), 0.0)
            ranges, amin, ainc = sim.scan(n_beams=360)
            sensed = sense_cell_walls(ranges, amin, ainc, 0.0)
            for d, (dx, dy) in DIRS.items():
                nb = (cx + dx, cy + dy)
                if not in_grid(nb):
                    continue
                if sensed[d] != (not ground_truth_edge_open(sim, (cx, cy), nb)):
                    mismatches += 1
    assert mismatches == 0, f"{mismatches} LIDAR-vs-ground-truth sensing mismatches"
