import math
from tugbot_maze.cell_walls import sense_cell_walls


def _scan(near_scan_angles, n=360, far=5.0, near=0.9, window_deg=25):
    """LaserScan ranges over [-pi, pi); set beams within window_deg of each given
    SCAN-frame angle to `near`, the rest to `far`. window_deg >= the sensor's 22 deg
    window so a modeled wall fully fills it (a real cell-edge wall subtends ~45 deg)."""
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


def test_oblique_graze_on_open_edge_is_not_a_wall():
    """The (1,4) trap in miniature: the S edge is OPEN (far corridor wall ~2.4 m
    perpendicular) but a single oblique beam grazes a corner at 1.07 m inside the S
    window. A min-over-window estimator reads S as a WALL (1.07 < 1.3); the
    perpendicular-projection median rejects the lone graze and reads S OPEN."""
    n = 360
    amin, ainc = -math.pi, 2 * math.pi / n
    ranges = [12.0] * n
    ta = -math.pi / 2                                  # S cardinal, scan frame, yaw=0
    for i in range(n):
        off = math.atan2(math.sin((amin + i * ainc) - ta), math.cos((amin + i * ainc) - ta))
        if abs(off) <= math.radians(22):
            ranges[i] = 2.4 / math.cos(off)            # open: far wall ~2.4 m perpendicular
    graze = ta + math.radians(18)                      # one oblique beam grazing a corner
    ranges[int(round((graze - amin) / ainc)) % n] = 1.07
    sensed = sense_cell_walls(ranges, amin, ainc, 0.0)
    assert sensed['S'] is False, "lone oblique graze must not be read as a wall"


def test_no_false_walls_when_off_centre_on_real_maze():
    """The (1,4) failure class: an OPEN edge misread as WALL because an oblique corner
    graze fooled the old min-over-window estimator when the robot was off-centre. With
    perpendicular-projection median sensing, no OPEN edge is read as a wall at lateral
    offsets up to 0.45 m in every direction, at every interior cell. (Missing a present
    far wall at extreme offset is tolerated -- the hop watchdog self-corrects -- but a
    FALSE wall cuts a real route and can trap the robot, as it did at (1,4).)"""
    from tugbot_maze.maze_sim import MazeSim, load_segments, ground_truth_edge_open
    from tugbot_maze.flood_fill_brain import in_grid, DIRS
    segs = load_segments()
    false_walls = []
    for cx in range(1, 11):
        for cy in range(0, 10):
            for ox, oy in [(0.45, 0.0), (-0.45, 0.0), (0.0, 0.45), (0.0, -0.45)]:
                sim = MazeSim(segs, (2 * cx + ox, 2 * cy + oy), 0.0)
                ranges, amin, ainc = sim.scan(n_beams=360)
                sensed = sense_cell_walls(ranges, amin, ainc, 0.0)
                for d, (dx, dy) in DIRS.items():
                    nb = (cx + dx, cy + dy)
                    if in_grid(nb) and sensed[d] and ground_truth_edge_open(sim, (cx, cy), nb):
                        false_walls.append(((cx, cy), d, (ox, oy)))
    assert not false_walls, f"{len(false_walls)} OPEN edges misread as walls off-centre: {false_walls[:8]}"


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
