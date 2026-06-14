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


def test_explored_branch_not_rechosen_after_traversal():
    solver = TremauxSolver(exit_xy=(20.0, 18.0))
    j = _local('junction', [
        (0.0, (1.5, 0.0), 1.5),
        (math.pi / 2, (0.0, 1.5), 1.5),
        (-math.pi / 2, (0.0, -1.5), 1.5),
    ])
    first = solver.update((0.0, 0.0), 0.0, j)
    solver.update(first.target_xy, first.yaw, _local('corridor', [
        (first.yaw, (first.target_xy[0] + 1.0, first.target_xy[1]), 1.0),
        (first.yaw + math.pi, (0.0, 0.0), 1.0),
    ]))
    second = solver.update((0.0, 0.0), 0.0, j)
    assert second.kind == EXPLORE
    assert second.target_xy != first.target_xy


def test_dead_end_returns_back_out_toward_previous_node():
    solver = TremauxSolver(exit_xy=(20.0, 18.0))
    j = _local('junction', [
        (0.0, (1.5, 0.0), 1.5),
        (math.pi / 2, (0.0, 1.5), 1.5),
        (-math.pi / 2, (0.0, -1.5), 1.5),
    ])
    a = solver.update((0.0, 0.0), 0.0, j)
    back = solver.update(a.target_xy, a.yaw,
                         _local('dead_end', [(a.yaw + math.pi, (0.0, 0.0), 1.0)]))
    assert back.kind == BACK_OUT
    assert math.hypot(back.target_xy[0], back.target_xy[1]) < 0.8


def test_done_when_single_corridor_dead_ends_both_ways():
    solver = TremauxSolver(exit_xy=(20.0, 18.0))
    solver.update((0.0, 0.0), 0.0, _local('dead_end', [(0.0, (1.0, 0.0), 1.0)]))
    final = solver.update((0.0, 0.0), 0.0, _local('dead_end', [(0.0, (1.0, 0.0), 1.0)]))
    assert final.kind in (DONE, BACK_OUT)


def test_report_outcome_wall_marks_branch_dead_end():
    from tugbot_maze.tremaux_solver import OUT_WALL
    solver = TremauxSolver(exit_xy=(20.0, 18.0))
    j = _local('junction', [
        (0.0, (1.5, 0.0), 1.5),
        (math.pi / 2, (0.0, 1.5), 1.5),
        (-math.pi / 2, (0.0, -1.5), 1.5),
    ])
    a = solver.update((0.0, 0.0), 0.0, j)
    solver.report_outcome(OUT_WALL)
    assert solver.active_branch is None
    nxt = solver.update((0.0, 0.0), 0.0, j)
    assert nxt.kind == EXPLORE and nxt.target_xy != a.target_xy
