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


def test_done_when_all_branches_exhausted():
    from tugbot_maze.tremaux_solver import OUT_WALL, DONE
    solver = TremauxSolver(exit_xy=(20.0, 18.0))
    j = _local('junction', [
        (0.0, (1.5, 0.0), 1.5),
        (math.pi, (-1.5, 0.0), 1.5),
    ])
    solver.update((0.0, 0.0), 0.0, j)
    solver.report_outcome(OUT_WALL)
    solver.update((0.0, 0.0), 0.0, j)
    solver.report_outcome(OUT_WALL)
    final = solver.update((0.0, 0.0), 0.0, j)
    assert final.kind == DONE


def test_wedged_gives_up_after_max_soft_failures_without_blacklist():
    from tugbot_maze.tremaux_solver import OUT_WEDGED, DONE
    solver = TremauxSolver(exit_xy=(20.0, 18.0), max_soft_failures=3)
    j = _local('junction', [(0.0, (1.5, 0.0), 1.5)])
    for _ in range(3):
        act = solver.update((0.0, 0.0), 0.0, j)
        assert act.kind == EXPLORE
        solver.report_outcome(OUT_WEDGED)
    node = solver.topology.nodes[solver.active_start_node_id]
    b = node.branches[0]
    assert b.state == 'explored'
    assert b.state not in ('blocked', 'blacklisted')
    final = solver.update((0.0, 0.0), 0.0, j)
    assert final.kind == DONE


def test_active_branch_tracks_driven_branch_after_repeat_update():
    """A repeat update() at the same pose must re-issue the committed branch,
    not re-decide. The branch actually driven gets the dead-end mark; the other
    stays untried. (Regression for the active_branch desync.)"""
    from tugbot_maze.tremaux_solver import BACK_OUT
    solver = TremauxSolver(exit_xy=(20.0, 18.0))
    j = _local('junction', [
        (0.0, (1.5, 0.0), 1.5),
        (math.pi, (-1.5, 0.0), 1.5),
    ])
    a1 = solver.update((0.0, 0.0), 0.0, j)
    a2 = solver.update((0.0, 0.0), 0.0, j)
    assert a2.kind == EXPLORE
    assert a2.target_xy == a1.target_xy
    back = solver.update(a1.target_xy, a1.yaw,
                         _local('dead_end', [(a1.yaw + math.pi, (0.0, 0.0), 1.0)]))
    assert back.kind == BACK_OUT
    node = solver.topology.nodes[1]
    driven = [b for b in node.branches
              if abs(b.target_xy[0] - a1.target_xy[0]) < 0.5
              and abs(b.target_xy[1] - a1.target_xy[1]) < 0.5]
    other = [b for b in node.branches if b not in driven]
    assert driven and driven[0].state == 'dead_end'
    assert other and other[0].state == 'untried'


def test_report_outcome_success_marks_branch_explored_and_advances():
    from tugbot_maze.tremaux_solver import OUT_SUCCESS
    solver = TremauxSolver(exit_xy=(20.0, 18.0))
    j = _local('junction', [
        (0.0, (1.5, 0.0), 1.5),
        (math.pi / 2, (0.0, 1.5), 1.5),
        (-math.pi / 2, (0.0, -1.5), 1.5),
    ])
    a = solver.update((0.0, 0.0), 0.0, j)
    assert a.kind == EXPLORE
    solver.report_outcome(OUT_SUCCESS)
    assert solver.active_branch is None
    # Re-deciding at the same node (robot didn't register a new node) must NOT
    # re-commit the same branch — the explored branch is skipped for a new one.
    nxt = solver.update((0.0, 0.0), 0.0, j)
    assert nxt.kind == EXPLORE
    assert nxt.target_xy != a.target_xy
