from tugbot_maze.maze_topology import MazeTopology, IN_PROGRESS


def test_topoedge_has_visit_count_default_zero():
    topo = MazeTopology()
    a = topo.find_or_create_node(0.0, 0.0)
    b = topo.find_or_create_node(2.0, 0.0)
    edge = topo.connect_nodes(a.node_id, b.node_id, state=IN_PROGRESS)
    assert edge.visit_count == 0
    edge.visit_count = 2
    assert topo.edges[edge.edge_id].visit_count == 2


import math
from collections import namedtuple
from tugbot_maze.tremaux_solver import TremauxSolver, EXPLORE, REROUTE, BACK_OUT, DONE

Dir = namedtuple('Dir', 'angle_rad target_xy distance_m')
Local = namedtuple('Local', 'kind open_directions')


def _local(kind, dirs):
    return Local(kind=kind, open_directions=[Dir(a, t, d) for (a, t, d) in dirs])


def test_first_update_at_junction_returns_explore():
    solver = TremauxSolver(exit_xy=(20.0, 18.0))
    local = _local('junction', [
        (0.0, (1.5, 0.0), 1.5),
        (math.pi / 2, (0.0, 1.5), 1.5),
        (-math.pi / 2, (0.0, -1.5), 1.5),
    ])
    action = solver.update((0.0, 0.0), 0.0, local)
    assert action.kind == EXPLORE
    assert action.target_xy is not None
