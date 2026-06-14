import math
from collections import namedtuple

from tugbot_maze.tremaux_solver import (
    TremauxSolver, EXPLORE, REROUTE, BACK_OUT, OUT_WALL, OUT_WEDGED,
)

Dir = namedtuple('Dir', 'angle_rad target_xy distance_m')
Local = namedtuple('Local', 'kind open_directions')


def _local(kind, dirs):
    return Local(kind=kind, open_directions=[Dir(a, t, d) for (a, t, d) in dirs])


def test_escapes_se_pocket_to_northward_passage():
    solver = TremauxSolver(exit_xy=(21.0, 18.0))
    j1 = _local('junction', [
        (0.0, (11.5, 9.0), 1.5),            # east -> pocket
        (math.pi / 2, (10.0, 10.5), 1.5),   # north -> toward exit
        (math.pi, (8.5, 9.0), 1.5),         # west -> entry
    ])
    a1 = solver.update((10.0, 9.0), 0.0, j1)
    assert a1.kind == EXPLORE
    j2 = _local('junction', [
        (0.0, (16.5, 9.0), 1.5),            # east -> deeper into pocket
        (math.pi, (13.5, 9.0), 1.5),        # west -> back to J1
    ])
    solver.update((15.0, 9.0), 0.0, j2)
    a2 = solver.update((15.0, 9.0), 0.0, j2)
    assert a2.kind in (EXPLORE, REROUTE)
    dead = solver.update((19.0, 9.0), 0.0,
                         _local('dead_end', [(math.pi, (17.5, 9.0), 1.5)]))
    assert dead.kind == BACK_OUT
    nxt = solver.update((15.0, 9.0), 0.0, j2)
    assert nxt.kind in (EXPLORE, REROUTE)
    at_j1 = solver.update((10.0, 9.0), 0.0, j1)
    assert at_j1.kind == EXPLORE
    assert at_j1.target_xy[1] > 9.5   # heading NORTH (toward exit), not EAST


def test_pocket_never_blacklists_the_corridor():
    solver = TremauxSolver(exit_xy=(21.0, 18.0))
    j = _local('junction', [
        (0.0, (1.5, 0.0), 1.5),
        (math.pi / 2, (0.0, 1.5), 1.5),
        (math.pi, (-1.5, 0.0), 1.5),
    ])
    solver.update((0.0, 0.0), 0.0, j)
    solver.report_outcome(OUT_WEDGED)
    node = solver.topology.nodes[solver.active_start_node_id or 1]
    assert all(b.state != 'blacklisted' and b.state != 'blocked' for b in node.branches)
