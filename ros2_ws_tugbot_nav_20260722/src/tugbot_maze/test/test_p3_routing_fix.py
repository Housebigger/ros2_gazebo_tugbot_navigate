"""P3 routing fix: single-edge exit-directed UNSTICK + zero-growth escape escalation.

Offline TDD for docs/superpowers/specs/2026-07-19-p3-routing-fix-design.md. The whole-tier
mass re-open and the degenerate 90s tier-2 reverse cadence are the two mechanisms behind
the P3 exploration-graph-exhaustion TIMEOUTs (runs 20260719_175751 / 20260719_201541)."""
import math
from tugbot_maze.maze_motion import MazeMotion
from tugbot_maze.maze_sim import MazeSim, load_segments
from tugbot_maze.flood_fill_brain import (
    ENTRANCE_CELL, EXIT_CELL, DIRS, OPP, in_grid, cell_center)


def _pocket_walls():
    # isolate the pocket {(5,5),(5,6)}: open only the (5,5)<->(5,6) edge, wall everything else.
    return [((5, 5), 'E'), ((5, 5), 'W'), ((5, 5), 'S'),
            ((5, 6), 'N'), ((5, 6), 'E'), ((5, 6), 'W')]


def _seal(m, walls, loco=True):
    for (c, d) in walls:
        m.brain.mark(c, d, is_wall=True)
    if loco:
        m.locomotion_walls.update(walls)


# ---- Task 3: UNSTICK single-edge, exit-nearest ----

def test_unstick_reopens_exactly_one_edge_exit_nearest():
    # OLD code re-opened the whole tier at once (the n=16 committed mass-rollback of the P3
    # runs). NEW contract: ONE edge per invocation, the dexit-minimal one.
    m = MazeMotion(); m.cell = (5, 5)
    _seal(m, _pocket_walls())
    m._unstick(0.0)
    assert m.phase == 'center'
    assert len(m.reopened) == 2                        # ONE undirected edge = both directed reps
    assert ((5, 6), 'E') in m.reopened and ((6, 6), 'W') in m.reopened   # dexit argmin (5.00)
    assert m.brain._state((5, 6), 'E') != 'wall'
    assert m.brain._state((5, 5), 'E') == 'wall'       # the rest of the tier stays sealed


def test_unstick_second_call_picks_next_best_edge():
    # With the best edge already spent (reopened bound), the NEXT dexit-min edge is chosen.
    m = MazeMotion(); m.cell = (5, 5)
    _seal(m, _pocket_walls())
    m.reopened.update({((5, 6), 'E'), ((6, 6), 'W')})  # best edge already consumed
    m._unstick(0.0)
    assert ((5, 6), 'N') in m.reopened                 # next best: nb (5,7), d=5.39
    assert m.brain._state((5, 6), 'N') != 'wall'
    assert m.brain._state((5, 6), 'E') == 'wall'       # consumed edge NOT re-touched


def test_unstick_tier_order_beats_dexit():
    # Trust tiers still come first: a loco-tier edge is picked even when a committed-tier
    # edge has the globally minimal dexit.
    m = MazeMotion(); m.cell = (5, 5)
    _seal(m, _pocket_walls(), loco=False)
    m.locomotion_walls.update([((5, 5), 'E'), ((5, 5), 'W'), ((5, 5), 'S')])  # loco = (5,5) edges
    m.committed.add((5, 6))                            # (5,6) edges (incl. global dexit-min) tier-3
    m._unstick(0.0)
    assert ((5, 5), 'E') in m.reopened                 # best WITHIN the loco tier (d=5.66)
    assert ((5, 6), 'E') not in m.reopened             # global min (d=5.00) outranked by tier


def test_unstick_incident_tiebreak_then_deterministic():
    # Three cut edges tie at dexit=5.00; two are incident to self.cell; 'E' < 'N' settles it.
    m = MazeMotion(); m.cell = (6, 5)
    walls = [((5, 5), 'W'), ((5, 5), 'S'),
             ((6, 5), 'N'), ((6, 5), 'E'), ((6, 5), 'S'),
             ((5, 6), 'N'), ((5, 6), 'E'), ((5, 6), 'W')]   # R = {(5,5),(6,5),(5,6)}
    _seal(m, walls)
    m._unstick(0.0)
    assert ((6, 5), 'E') in m.reopened                 # incident tie-winner, deterministic 'E'
    assert ((5, 6), 'E') not in m.reopened             # equal dexit but NOT incident


def test_unstick_exhaustion_still_terminates_stuck():
    # The reopened bound is untouched: all cut edges consumed -> terminal 'stuck'.
    m = MazeMotion(); m.cell = (5, 5)
    base = _pocket_walls()
    _seal(m, base)
    m.reopened.update(base)                            # every cut edge already re-opened once
    m._unstick(0.0)
    assert m.phase == 'stuck'


def test_unstick_diag_single_edge_format():
    m = MazeMotion(); m.cell = (5, 5)
    _seal(m, _pocket_walls())
    m._unstick(0.0)
    ev = m.events[-1]
    assert ev.startswith('UNSTICK reopen')
    assert ' edge=' in ev and ' tier=loco' in ev and ' dexit=5.00' in ev
