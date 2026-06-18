import math
from tugbot_maze.maze_motion import MazeMotion
from tugbot_maze.maze_sim import MazeSim, load_segments
from tugbot_maze.flood_fill_brain import ENTRANCE_CELL, EXIT_CELL, cell_center


def _scan_at(sim):
    return sim.scan(n_beams=360, fov_rad=2 * math.pi)


def test_starts_in_center_at_entrance():
    m = MazeMotion()
    assert m.cell == ENTRANCE_CELL and m.phase == 'center'


def test_center_senses_then_turns_toward_a_chosen_neighbor():
    # At the entrance cell centre, MazeMotion should sense, pick next_cell, and enter 'turn'.
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0)
    m = MazeMotion()
    t = 0.0
    for _ in range(60):                       # a few ticks to settle + sense + plan
        v, w, done = m.step(sim.pose, _scan_at(sim), t)
        t += 0.1
        if m.phase in ('turn', 'drive'):
            break
    assert m.phase in ('turn', 'drive')
    assert m.hop_target is not None and m.hop_dir is not None


def test_turn_rotates_then_settles_to_drive():
    # Force a turn target 90 deg from current heading; stepping with a rotating yaw must
    # eventually settle into 'drive'.
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0)
    m = MazeMotion()
    m.phase = 'turn'
    m.hop_dir = (0, 1)
    m.target_cardinal = math.pi / 2
    m.hop_start = cell_center(ENTRANCE_CELL)
    yaw = 0.0
    t = 0.0
    for _ in range(200):
        v, w, _ = m.step((0.0, 0.0, yaw), _scan_at(sim), t)
        assert abs(v) < 1e-9                  # no translation during a turn
        yaw = math.atan2(math.sin(yaw + w * 0.1), math.cos(yaw + w * 0.1))
        t += 0.1
        if m.phase == 'drive':
            break
    assert m.phase == 'drive'
    assert abs(math.atan2(math.sin(math.pi / 2 - yaw), math.cos(math.pi / 2 - yaw))) <= 0.11


def test_done_when_cell_is_exit():
    m = MazeMotion()
    m.cell = EXIT_CELL
    sim = MazeSim(load_segments(), cell_center(EXIT_CELL), 0.0)
    v, w, done = m.step(sim.pose, _scan_at(sim), 0.0)
    assert done is True and (v, w) == (0.0, 0.0)
