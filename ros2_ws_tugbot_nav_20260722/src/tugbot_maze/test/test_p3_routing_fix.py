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


# ---- Task 4: ESCAPE zero-growth escalation + adaptive watchdog window ----

def test_escape_first_fire_growth_positive_keeps_ladder():
    # _esc_visited_n starts at 0, so the FIRST escape always sees growth>0 -> the Tier-1
    # reverse ladder is behavior-identical to the old code.
    m = MazeMotion(); m.cell = (5, 5); m.prev_cell = (5, 4)
    m.visited.update({(5, 4), (5, 5)})
    m._escape((10.0, 10.0, 0.0), 100.0)
    assert m._escape_backout is True and m.phase == 'backout'
    assert m.backout_target == (5, 4)
    ev = [e for e in m.events if e.startswith('ESCAPE')][-1]
    assert ' growth=' in ev and ' win=' in ev


def test_escape_zero_growth_diverts_to_unstick_and_fast_window():
    m = MazeMotion(); m.cell = (5, 5); m.prev_cell = (5, 6)   # (5,5)<->(5,6) is the OPEN edge
    m.visited.update({(5, 5), (5, 6)})
    _seal(m, _pocket_walls())
    m._escape((10.0, 10.0, 0.0), 100.0)                # growth>0 -> normal reverse
    assert m.phase == 'backout'
    m.phase = 'center'; m._escape_backout = False
    m._escape((10.0, 10.0, 0.0), 200.0)                # visited unchanged -> growth==0
    assert m._no_progress_win == m.no_progress_fast_s  # fast cadence armed
    assert m.phase == 'center'                         # diverted to _unstick: single-edge reopen
    assert len(m.reopened) == 2
    assert m._escape_backout is False                  # NO degenerate reverse
    esc = [e for e in m.events if e.startswith('ESCAPE')][-1]
    assert 'growth=0' in esc


def test_escape_growth_restores_calm_window():
    m = MazeMotion(); m.cell = (5, 5); m.prev_cell = (5, 6)
    m.visited.update({(5, 5), (5, 6)})
    _seal(m, _pocket_walls())
    m._escape((10.0, 10.0, 0.0), 100.0)
    m.phase = 'center'; m._escape_backout = False
    m._escape((10.0, 10.0, 0.0), 200.0)                # zero growth -> fast window
    assert m._no_progress_win == m.no_progress_fast_s
    m.cell = (6, 6)                                    # NEW GROUND
    m._track_cell(300.0)
    assert m._no_progress_win == m.no_progress_s       # calm window restored
    assert m.escape_tier == 0                          # existing growth-clears-escalation kept


def test_watchdog_fires_on_adaptive_window():
    # The step() gate reads _no_progress_win, not no_progress_s.
    m = MazeMotion()
    fired = []
    m._escape = lambda pose, t: (fired.append(t), (0.0, 0.0, False))[1]
    m._no_progress_win = 5.0
    m.explore_t = 0.0
    m.recent = [(5.5, ENTRANCE_CELL)]                  # confined footprint
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0)
    scan = sim.scan(n_beams=360, fov_rad=2 * math.pi)
    m.step(sim.pose, scan, 6.0)                        # 6.0 > 5.0 fast window
    assert fired == [6.0]
    m2 = MazeMotion()
    m2._escape = lambda pose, t: (fired.append(-t), (0.0, 0.0, False))[1]
    m2.explore_t = 0.0
    m2.recent = [(5.5, ENTRANCE_CELL)]
    m2.step(sim.pose, scan, 6.0)                       # 6.0 < default 90.0 -> silent
    assert fired == [6.0]


def test_p3_constants_pinned():
    m = MazeMotion()
    assert m.no_progress_s == 90.0
    assert m.no_progress_fast_s == 30.0
    assert m._no_progress_win == 90.0
    assert m._esc_visited_n == 0
