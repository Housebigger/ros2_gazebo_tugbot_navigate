from tugbot_maze.flood_fill_brain import (
    FloodFillBrain, CELL_SIZE_M, ENTRANCE_CELL, EXIT_CELL, DIRS, OPP,
    cell_center, pose_to_cell)


def test_cell_center_and_pose_to_cell_roundtrip():
    assert cell_center((1, 0)) == (2.0, 0.0)
    assert cell_center((10, 9)) == (20.0, 18.0)
    assert pose_to_cell(2.1, -0.2) == (1, 0)
    assert pose_to_cell(19.6, 18.3) == (10, 9)


def test_flood_open_maze_distances_to_exit():
    # No walls known -> every edge optimistically open -> Manhattan distance to exit.
    b = FloodFillBrain()
    dist = b.flood()
    assert dist[EXIT_CELL] == 0
    assert dist[(9, 9)] == 1
    assert dist[(10, 8)] == 1
    assert dist[ENTRANCE_CELL] == abs(10 - 1) + abs(9 - 0)   # 18


def test_next_cell_heads_toward_exit_when_open():
    b = FloodFillBrain()
    # from entrance (1,0), open maze -> step that reduces Manhattan distance (N or E)
    nxt = b.next_cell((1, 0))
    assert nxt in {(1, 1), (2, 0)}


def test_mark_wall_is_symmetric_and_blocks():
    b = FloodFillBrain()
    b.mark((1, 0), 'N', is_wall=True)
    assert b.is_wall((1, 0), 'N') is True
    assert b.is_wall((1, 1), 'S') is True          # symmetric
    # with N walled at (1,0), next_cell must not choose (1,1)
    assert b.next_cell((1, 0)) != (1, 1)


def test_reflood_reroutes_around_new_wall():
    b = FloodFillBrain()
    # Wall off the whole exit column except via the top: force a detour by walling
    # the direct E step from (1,0); next_cell should then prefer N.
    b.mark((1, 0), 'E', is_wall=True)
    assert b.next_cell((1, 0)) == (1, 1)


def test_backtracks_when_only_open_edge_increases_distance():
    # At (5,5) wall off N, E, S -> only W is open. next_cell must still return the
    # one open neighbor (4,5) even though it steps AWAY from the exit (backtracking),
    # rather than getting stuck.
    b = FloodFillBrain()
    for d in ('N', 'E', 'S'):
        b.mark((5, 5), d, is_wall=True)
    assert b.next_cell((5, 5)) == (4, 5)


def test_is_done_only_at_exit():
    b = FloodFillBrain()
    assert b.is_done(EXIT_CELL) is True
    assert b.is_done(ENTRANCE_CELL) is False


def test_fully_enclosed_cell_returns_none():
    # all four edges walled -> no passable neighbor -> next_cell yields None (not stuck-crash)
    b = FloodFillBrain()
    for d in DIRS:
        b.mark((5, 5), d, is_wall=True)
    assert b.next_cell((5, 5)) is None


def test_does_not_reverse_to_came_from_unless_dead_end():
    # On an open cell with forward options, next_cell must NOT return the cell we just
    # left -- this commits the robot to the path and makes A->B->A ping-ponging impossible.
    b = FloodFillBrain()
    assert b.next_cell((5, 5), came_from=(5, 4)) != (5, 4)


def test_reverses_to_came_from_only_at_dead_end():
    # If the only passable neighbor IS the came-from cell, the robot must backtrack to it.
    b = FloodFillBrain()
    for d in ('N', 'E', 'W'):
        b.mark((5, 5), d, is_wall=True)
    assert b.next_cell((5, 5), came_from=(5, 4)) == (5, 4)
