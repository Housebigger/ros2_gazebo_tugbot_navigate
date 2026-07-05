import math
from tugbot_maze.flood_fill_brain import FloodFillBrain
from tugbot_maze.online_scan_match_localizer import confirmed_wall_segments


def _canon(seg):
    a, b = (round(seg[0], 3), round(seg[1], 3)), (round(seg[2], 3), round(seg[3], 3))
    return (a, b) if a <= b else (b, a)


def test_committed_wall_becomes_grid_snapped_segment():
    brain = FloodFillBrain()
    brain.mark((3, 4), 'N', True)                       # a wall on the N edge of cell (3,4)
    segs = confirmed_wall_segments(brain, {(3, 4)})
    # cell (3,4) centre = (6,8); N edge y=9, x in [5,7] -> (5,9,7,9)
    assert [_canon(s) for s in segs] == [((5.0, 9.0), (7.0, 9.0))]


def test_uncommitted_and_non_wall_edges_excluded():
    brain = FloodFillBrain()
    brain.mark((3, 4), 'N', True)                       # wall, but its cell is NOT committed
    assert confirmed_wall_segments(brain, set()) == []
    brain.mark((3, 4), 'E', False)                      # OPEN edge in a committed cell
    segs = confirmed_wall_segments(brain, {(3, 4)})
    assert len(segs) == 1                               # only the N wall, not the E-open edge


def test_symmetric_edge_deduplicated():
    brain = FloodFillBrain()
    brain.mark((3, 4), 'N', True)                       # also marks (3,5) S == the same wall
    segs = confirmed_wall_segments(brain, {(3, 4), (3, 5)})
    assert len(segs) == 1                               # one physical wall, not two
