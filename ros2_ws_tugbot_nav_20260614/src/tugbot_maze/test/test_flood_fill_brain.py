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


def test_prefers_unmarked_edge_over_traversed():
    # At (1,0), N->(1,1) and E->(2,0) tie on flood distance. Mark the edge to (1,1) twice
    # (used up) -> next_cell must take the unmarked edge E->(2,0). This is how Trémaux
    # breaks cycles: a revisited edge is avoided in favor of an unexplored one.
    b = FloodFillBrain()
    b.mark_traversal((1, 0), (1, 1))
    b.mark_traversal((1, 0), (1, 1))
    assert b.next_cell((1, 0)) == (2, 0)


def test_takes_the_only_unused_edge():
    # Mark 3 of (5,5)'s 4 edges twice (used up); only W->(4,5) is still usable -> take it.
    b = FloodFillBrain()
    for nb in [(5, 6), (6, 5), (5, 4)]:
        b.mark_traversal((5, 5), nb)
        b.mark_traversal((5, 5), nb)
    assert b.next_cell((5, 5)) == (4, 5)


def test_falls_back_when_all_edges_used_up():
    # All 4 edges twice-marked -> no unused edge -> still returns a passable neighbor
    # (degenerate safety: never returns None when a passable neighbor exists).
    b = FloodFillBrain()
    for nb in [(5, 6), (5, 4), (6, 5), (4, 5)]:
        b.mark_traversal((5, 5), nb)
        b.mark_traversal((5, 5), nb)
    assert b.next_cell((5, 5)) is not None


def test_exit_greedy_prefers_progress_over_unexplored_detour():
    # Run-5 efficiency fix. At (5,5) with N walled: E->(6,5) descends the flood
    # gradient (dist 8, closer to the exit) but is already traversed once; S->(5,4)
    # and W->(4,5) are UNEXPLORED yet lead AWAY (dist 10). next_cell must take the
    # exit-ward E, not wander into a farther unexplored cell. Edge-count stays a
    # tiebreak (the <=2 cycle cap is intact) but no longer overrides flood progress.
    b = FloodFillBrain()
    b.mark((5, 5), 'N', is_wall=True)
    b.mark_traversal((5, 5), (6, 5))      # E traversed once; still legal (count 1 < 2)
    assert b.next_cell((5, 5)) == (6, 5)
